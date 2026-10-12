#!/usr/bin/env python3

"""실행 궤적(R1)·연산 이벤트(R2) 채널의 적재 — `row_values` 변환과 registry 바인딩.

DB 없이 도는 시험이다(다른 writer 시험과 같은 방식). 지키는 것은 넷이다.

1. **`stamp` 의 뜻이 다르다** — 궤적의 stamp 는 발행 시각이 아니라 **t₀**(궤적의 0초)
   이고, 이벤트의 stamp 는 **레이블 구간의 끝이 아니다**. 변환이 그 값을 그대로 옮겨야
   export 가 `t₀ + t` 로 경유점 시각을 만들 수 있다.
2. **빈 속도·가속도를 0 으로 채우지 않는다** — 0 은 "그 점에서 멈춰 있으라"는 뜻이라
   3차 Hermite 보간이 실제와 다른 곡선을 만든다.
3. **orientation 은 ROS 규약 xyzw** — 바꾸면 4개 float 에 unit norm 이라 어떤 검사도
   통과하면서 '그럴듯하게 틀린 자세'가 된다.
4. **pose 컬럼은 `target_kind == 'pose'` 일 때만 채운다** — 관절 목표에 기본 pose(전부
   0, w=0)가 들어가면 원점을 겨냥한 것처럼 보인다.
"""

from __future__ import annotations

from types import SimpleNamespace

from rdfp.dataset.db.registry import MESSAGE_TYPE_REGISTRY
from rdfp.dataset.db.schema_check import REQUIRED_COLUMNS
from rdfp.dataset.db.writers.motion_event import MotionEventWriter
from rdfp.dataset.db.writers.motion_trajectory import MotionTrajectoryWriter


def _dur(sec: int = 0, nanosec: int = 0) -> SimpleNamespace:
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def _point(t_sec: float, positions, velocities=(), accelerations=()) -> SimpleNamespace:
    ns = int(round(t_sec * 1_000_000_000))
    return SimpleNamespace(positions=list(positions), velocities=list(velocities),
                           accelerations=list(accelerations),
                           time_from_start=_dur(ns // 1_000_000_000, ns % 1_000_000_000))


def _trajectory_msg(points=None, joint_names=('j1', 'j2')) -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(stamp=_dur(100, 250_000_000)),
        operation_id='op-7', velocity_scaling=0.4, time_scaling=2.0,
        trajectory=SimpleNamespace(
            joint_names=list(joint_names),
            points=points if points is not None else [
                _point(0.0, [0.0, 0.0]),
                _point(1.5, [0.1, 0.2], velocities=[0.05, -0.05]),
            ]))


def _event_msg(**kw) -> SimpleNamespace:
    base = dict(
        header=SimpleNamespace(stamp=_dur(103, 750_000_000)),
        operation_id='op-7', operation='cartesian', result='aborted',
        reason='CONTROL_FAILED', truncated_at=_dur(2, 125_000_000),
        target_kind='pose', target_joint_names=[], target_joint_values=[],
        target_frame='panda_link8',
        target_pose=SimpleNamespace(
            position=SimpleNamespace(x=0.4, y=0.0, z=0.55),
            orientation=SimpleNamespace(x=1.0, y=0.0, z=0.0, w=0.0)))
    base.update(kw)
    return SimpleNamespace(**base)


def _row(writer_cls, msg, episode_id=7):
    w = writer_cls(conn=None, topic_id=3)
    return dict(zip(writer_cls.columns, w.row_values(episode_id, msg)))


# ── 실행 궤적 ───────────────────────────────────────────────────────────────

def test_trajectory_row_carries_t0_and_scalings() -> None:
    """`stamp` 는 **t₀** 다 — 틀리면 레이블 전체가 통째로 밀린다."""
    row = _row(MotionTrajectoryWriter, _trajectory_msg())

    assert (row['stamp_sec'], row['stamp_nanosec']) == (100, 250_000_000)
    assert row['operation_id'] == 'op-7'
    assert (row['velocity_scaling'], row['time_scaling']) == (0.4, 2.0)
    assert row['joint_names'] == ['j1', 'j2']


def test_trajectory_points_keep_elapsed_seconds() -> None:
    """`t` 는 궤적 시작으로부터의 경과 초다 (`time_from_start`)."""
    points = _row(MotionTrajectoryWriter, _trajectory_msg())['points'].obj

    assert [p['t'] for p in points] == [0.0, 1.5]
    assert points[1]['positions'] == [0.1, 0.2]


def test_trajectory_keeps_empty_velocities_empty() -> None:
    """0 으로 채우면 '그 점에서 멈춰 있으라'가 되어 보간이 달라진다."""
    points = _row(MotionTrajectoryWriter, _trajectory_msg())['points'].obj

    assert points[0]['velocities'] == []
    assert points[0]['accelerations'] == []


def test_trajectory_accepts_an_empty_trajectory() -> None:
    """빈 궤적도 적재를 막지 않는다 — `duration_sec` 는 DB 가 NULL 로 만든다."""
    row = _row(MotionTrajectoryWriter, _trajectory_msg(points=[], joint_names=()))

    assert row['points'].obj == []
    assert row['joint_names'] == []


# ── 연산 이벤트 ─────────────────────────────────────────────────────────────

def test_event_row_carries_result_and_truncation() -> None:
    row = _row(MotionEventWriter, _event_msg())

    assert (row['operation'], row['result']) == ('cartesian', 'aborted')
    assert row['reason'] == 'CONTROL_FAILED'
    assert row['truncated_at_sec'] == 2.125
    assert row['target_frame'] == 'panda_link8'


def test_event_pose_stays_xyzw() -> None:
    """순서를 바꾸면 unit norm 이라 어떤 검사도 통과하면서 조용히 틀린다."""
    row = _row(MotionEventWriter, _event_msg())

    assert row['target_position'] == [0.4, 0.0, 0.55]
    assert row['target_orientation'] == [1.0, 0.0, 0.0, 0.0]


def test_event_leaves_pose_null_for_joint_targets() -> None:
    """관절 목표에 기본 pose 가 들어가면 **원점을 겨냥한 것처럼** 보인다."""
    row = _row(MotionEventWriter, _event_msg(
        target_kind='joints', target_joint_names=['j1', 'j2'],
        target_joint_values=[0.3, 0.4]))

    assert row['target_position'] is None
    assert row['target_orientation'] is None
    assert row['target_joint_names'] == ['j1', 'j2']
    assert row['target_joint_values'] == [0.3, 0.4]


def test_event_keeps_the_target_on_failure() -> None:
    """`failed` 여도 채운다 — 무엇을 하려다 실패했는지가 남아야 한다."""
    row = _row(MotionEventWriter, _event_msg(
        result='failed', truncated_at=_dur(0, 0), target_kind='joints',
        target_joint_names=['j1'], target_joint_values=[0.5]))

    assert row['result'] == 'failed'
    assert row['truncated_at_sec'] == 0.0
    assert row['target_joint_values'] == [0.5]


# ── 배선 ────────────────────────────────────────────────────────────────────

def test_registry_binds_both_channels() -> None:
    for typ, table in (('rdfp_msgs/msg/MotionTrajectory', 'motion_trajectories'),
                       ('rdfp_msgs/msg/MotionEvent', 'motion_events')):
        assert MESSAGE_TYPE_REGISTRY[typ].table == table


def test_schema_check_knows_both_tables() -> None:
    """`operation_id` 가 빠지면 두 채널을 이을 수 없다 — 필수 컬럼에 있어야 한다."""
    for table in ('motion_trajectories', 'motion_events'):
        assert 'operation_id' in REQUIRED_COLUMNS[table]
    assert 'duration_sec' in REQUIRED_COLUMNS['motion_trajectories']
    assert 'truncated_at_sec' in REQUIRED_COLUMNS['motion_events']
