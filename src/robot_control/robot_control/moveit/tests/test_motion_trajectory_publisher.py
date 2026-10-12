#!/usr/bin/env python3

"""실행 궤적 채널(R1) 발행부 — `motion/trajectory`.

무엇을 지키는지:

1. **메시지 계약** — `time_scaling` 이 `time_from_start` 에 반영돼 나간다. 스트리머는
   원래 궤적을 안 고치고 발행 일정에만 곱하므로, 발행자가 맞춰 주지 않으면 소비자가
   `t₀ + time_from_start` 로 계산한 시각이 틀린다.
2. **호출자의 궤적을 건드리지 않는다** — ROS 2 파이썬 메시지는 참조 대입이라,
   실어 보낼 궤적의 header 를 지우다가 **컨트롤러로 보내는 바로 그 궤적**까지 지울 수
   있다. 그러면 팔이 이상하게 움직이는데 원인이 발행부에 있다.
3. **발행 실패가 연산을 막지 않는다** (설계 §6) — 기록용이므로 예외를 삼키고 경고만
   남긴다.
4. **채널 이름이 규약을 지킨다** — 루트 상대, 앞에 `/` 없음.
"""

from __future__ import annotations

import pytest

pytest.importorskip('rclpy', reason='requires ROS 2 runtime')

from builtin_interfaces.msg import Duration                      # noqa: E402
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: E402

from robot_control.moveit import motion_trajectory_publisher as m  # noqa: E402


def _traj(*, secs=(0.0, 1.0, 2.0), vel=0.4, acc=0.2) -> JointTrajectory:
    jt = JointTrajectory()
    jt.joint_names = ['a', 'b']
    jt.header.frame_id = 'panda_link0'      # 비워지는지 확인하려고 일부러 채운다
    for s in secs:
        p = JointTrajectoryPoint()
        p.positions = [s, -s]
        p.velocities = [vel, vel]
        p.accelerations = [acc, acc]
        p.time_from_start = Duration(sec=int(s), nanosec=int((s % 1) * 1e9))
        jt.points.append(p)
    return jt


def _secs(jt: JointTrajectory) -> list[float]:
    return [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in jt.points]


class _FakeLogger:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def warning(self, msg: str) -> None:
        self.warnings.append(msg)


class _FakeClock:
    @staticmethod
    def now():
        class _T:
            @staticmethod
            def to_msg():
                from builtin_interfaces.msg import Time
                return Time(sec=7, nanosec=8)
        return _T()


class _FakeNode:
    """`create_publisher` 만 가로채는 최소 노드."""

    def __init__(self, publisher) -> None:
        self._publisher = publisher
        self._logger = _FakeLogger()

    def create_publisher(self, msg_type, topic, qos):
        self.topic = topic
        self.qos = qos
        return self._publisher

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return _FakeClock()


class _Recorder:
    def __init__(self) -> None:
        self.sent = []

    def publish(self, msg) -> None:
        self.sent.append(msg)


class _Exploding:
    def publish(self, msg) -> None:
        raise RuntimeError('DDS is down')


# ── 1. 메시지 계약: time_scaling 이 궤적에 반영된다 ────────────────────────────

def test_time_scaling_is_baked_into_time_from_start():
    out = m.apply_time_scaling(_traj(), 2.0)
    assert _secs(out) == pytest.approx([0.0, 2.0, 4.0])


def test_time_scaling_adjusts_velocity_and_acceleration():
    out = m.apply_time_scaling(_traj(vel=0.4, acc=0.2), 2.0)
    # 두 배 느리게 재생하면 속도는 1/2, 가속도는 1/4 이다.
    assert out.points[0].velocities == pytest.approx([0.2, 0.2])
    assert out.points[0].accelerations == pytest.approx([0.05, 0.05])


def test_time_scaling_one_is_a_noop():
    jt = _traj()
    assert m.apply_time_scaling(jt, 1.0) is jt


def test_time_scaling_does_not_mutate_the_input():
    jt = _traj()
    m.apply_time_scaling(jt, 3.0)
    assert _secs(jt) == pytest.approx([0.0, 1.0, 2.0])


def test_published_trajectory_carries_scaled_times():
    rec = _Recorder()
    pub = m.MotionTrajectoryPublisher(_FakeNode(rec))
    pub.publish(_traj(), time_scaling=2.0)
    assert _secs(rec.sent[0].trajectory) == pytest.approx([0.0, 2.0, 4.0])


# ── 2. 호출자의 궤적을 건드리지 않는다 ────────────────────────────────────────

def test_publish_does_not_clear_the_callers_header():
    """**컨트롤러로 보내는 그 궤적**의 header 를 지우면 안 된다."""
    rec = _Recorder()
    pub = m.MotionTrajectoryPublisher(_FakeNode(rec))
    jt = _traj()
    pub.publish(jt)
    assert jt.header.frame_id == 'panda_link0'


def test_published_inner_header_is_empty():
    """시각은 바깥 header 하나에만 둔다 — 두 곳이면 어느 쪽이 t₀ 인지 모호해진다."""
    rec = _Recorder()
    pub = m.MotionTrajectoryPublisher(_FakeNode(rec))
    pub.publish(_traj())
    assert rec.sent[0].trajectory.header.frame_id == ''
    assert rec.sent[0].trajectory.header.stamp.sec == 0


def test_publish_does_not_mutate_caller_when_time_scaled():
    rec = _Recorder()
    pub = m.MotionTrajectoryPublisher(_FakeNode(rec))
    jt = _traj()
    pub.publish(jt, time_scaling=5.0)
    assert _secs(jt) == pytest.approx([0.0, 1.0, 2.0])


# ── 3. 발행 실패가 연산을 막지 않는다 ─────────────────────────────────────────

def test_publish_failure_is_swallowed_and_warned():
    node = _FakeNode(_Exploding())
    pub = m.MotionTrajectoryPublisher(node)
    assert pub.publish(_traj()) is None
    assert node.get_logger().warnings, '실패를 조용히 넘기면 안 된다'
    assert 'motion/trajectory' in node.get_logger().warnings[0]


# ── 4. 채널 이름과 식별자 ─────────────────────────────────────────────────────

def test_topic_is_root_relative():
    """앞에 `/` 를 붙이면 로봇별 네임스페이스가 안 붙는다 (규약 §2)."""
    assert not m.MOTION_TRAJECTORY_TOPIC.startswith('/')
    assert m.MOTION_TRAJECTORY_TOPIC == 'motion/trajectory'


def test_operation_id_is_unique_per_publish():
    rec = _Recorder()
    pub = m.MotionTrajectoryPublisher(_FakeNode(rec))
    first, second = pub.publish(_traj()), pub.publish(_traj())
    assert first and second and first != second


def test_given_operation_id_is_kept():
    """R2 가 같은 값으로 짝지으려면 호출자가 준 식별자가 그대로 실려야 한다."""
    rec = _Recorder()
    pub = m.MotionTrajectoryPublisher(_FakeNode(rec))
    pub.publish(_traj(), operation_id='op-42')
    assert rec.sent[0].operation_id == 'op-42'


def test_scalings_are_recorded():
    rec = _Recorder()
    pub = m.MotionTrajectoryPublisher(_FakeNode(rec))
    pub.publish(_traj(), velocity_scaling=0.1, time_scaling=2.0)
    assert rec.sent[0].velocity_scaling == pytest.approx(0.1)
    assert rec.sent[0].time_scaling == pytest.approx(2.0)


def test_stamp_defaults_to_node_clock():
    """t₀ 를 안 주면 노드 시계를 쓴다 — **비워 두면 적재가 버린다.**"""
    rec = _Recorder()
    pub = m.MotionTrajectoryPublisher(_FakeNode(rec))
    pub.publish(_traj())
    assert (rec.sent[0].header.stamp.sec, rec.sent[0].header.stamp.nanosec) == (7, 8)
