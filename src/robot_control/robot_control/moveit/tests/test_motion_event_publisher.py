#!/usr/bin/env python3

"""연산 이벤트 채널(R2) 발행부 — `motion/event`.

무엇을 지키는지:

1. **`truncated_at` 규칙** — 정상 종료는 궤적 길이, 중단·실패는 0, 값을 주면 궤적
   길이 안으로 자른다. 이 규칙이 한 곳만 틀려도 **에러 없이 레이블만 어긋난다** —
   중단된 연산 뒤의 목표가 팔이 가지 않은 자리를 가리킨다.
2. **`target_kind` 를 발행부가 정한다** — 호출자가 종류와 값을 따로 주면 어긋날 수
   있고, 그 어긋남은 "어느 필드를 봐야 하는지 모른다" 로 나타난다.
3. **고정 어휘를 지킨다** — 오타는 에러가 아니라 "레이블이 이상하다"로만 드러난다.
4. **발행 실패가 연산을 막지 않는다** (설계 §6) — 예외를 삼키고 경고만 남긴다.
5. **채널 이름이 규약을 지킨다** — 루트 상대, 앞에 `/` 없음.
"""

from __future__ import annotations

import pytest

pytest.importorskip('rclpy', reason='requires ROS 2 runtime')

from builtin_interfaces.msg import Duration                             # noqa: E402
from geometry_msgs.msg import Pose                                      # noqa: E402
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint   # noqa: E402

from robot_control.moveit import motion_event_publisher as m            # noqa: E402
from robot_control.moveit.motion_trajectory_publisher import trajectory_duration_sec  # noqa: E402


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


@pytest.fixture
def rec():
    return _Recorder()


@pytest.fixture
def pub(rec):
    return m.MotionEventPublisher(_FakeNode(rec))


def _ctx(**kw) -> m.MotionContext:
    base = dict(operation_id='op-1', stamp=None, operation=m.OPERATION_JOINTS,
                duration_sec=2.5)
    base.update(kw)
    return m.MotionContext(**base)


def _secs(d: Duration) -> float:
    return d.sec + d.nanosec * 1e-9


# ── 1. truncated_at 규칙 ────────────────────────────────────────────────────

def test_succeeded_without_explicit_value_uses_trajectory_duration(pub, rec):
    """끝까지 갔으면 중단 시각은 궤적 길이다 — 소비자가 끝점을 얻는다."""
    pub.publish_context(_ctx(duration_sec=2.5), result=m.RESULT_SUCCEEDED)

    assert _secs(rec.sent[0].truncated_at) == pytest.approx(2.5)


@pytest.mark.parametrize('result', [m.RESULT_ABORTED, m.RESULT_FAILED])
def test_no_value_means_nothing_ran(pub, rec, result):
    """goal 거부·계획 실패는 한 점도 실행되지 않았다 — 0 이다."""
    pub.publish_context(_ctx(duration_sec=2.5), result=result)

    assert _secs(rec.sent[0].truncated_at) == 0.0


def test_explicit_value_is_kept(pub, rec):
    pub.publish_context(_ctx(duration_sec=2.5), result=m.RESULT_ABORTED,
                        truncated_at_sec=1.25)

    assert _secs(rec.sent[0].truncated_at) == pytest.approx(1.25)


def test_value_beyond_the_trajectory_is_clamped(pub, rec):
    """**JTC 에서 실제로 넘친다** — goal tolerance 확인 때문에 결과가 궤적 끝보다 늦다.

    안 자르면 `L2(truncated_at)` 가 궤적 밖을 가리켜 소비자가 외삽한다.
    """
    pub.publish_context(_ctx(duration_sec=2.5), result=m.RESULT_SUCCEEDED,
                        truncated_at_sec=2.9)

    assert _secs(rec.sent[0].truncated_at) == pytest.approx(2.5)


def test_negative_value_is_floored_at_zero(pub, rec):
    """중단 시각이 궤적의 0초보다 앞설 수는 없다."""
    pub.publish_context(_ctx(), result=m.RESULT_ABORTED, truncated_at_sec=-0.3)

    assert _secs(rec.sent[0].truncated_at) == 0.0


def test_duration_sec_matches_the_trajectory_helper():
    """R1 이 재는 길이와 같은 축이어야 자르기가 맞는다."""
    jt = JointTrajectory()
    for s in (0.0, 0.5, 1.75):
        p = JointTrajectoryPoint()
        p.time_from_start = Duration(sec=int(s), nanosec=int(round((s % 1) * 1e9)))
        jt.points.append(p)

    assert trajectory_duration_sec(jt) == pytest.approx(1.75)


def test_empty_trajectory_has_zero_duration():
    assert trajectory_duration_sec(JointTrajectory()) == 0.0


# ── 2. target_kind 는 발행부가 정한다 ────────────────────────────────────────

def test_pose_target_sets_kind_and_frame(pub, rec):
    pose = Pose()
    pose.position.x = 0.3
    pub.publish_context(_ctx(operation=m.OPERATION_CARTESIAN, target_pose=pose,
                             target_frame='panda_link8'),
                        result=m.RESULT_SUCCEEDED)
    sent = rec.sent[0]

    assert sent.target_kind == m.TARGET_KIND_POSE
    assert sent.target_pose.position.x == pytest.approx(0.3)
    assert sent.target_frame == 'panda_link8'


def test_joint_target_sets_kind_and_values(pub, rec):
    pub.publish_context(_ctx(target_joint_names=('j1', 'j2'),
                             target_joint_values=(0.1, -0.2)),
                        result=m.RESULT_SUCCEEDED)
    sent = rec.sent[0]

    assert sent.target_kind == m.TARGET_KIND_JOINTS
    assert list(sent.target_joint_names) == ['j1', 'j2']
    assert list(sent.target_joint_values) == pytest.approx([0.1, -0.2])


def test_no_target_leaves_kind_empty(pub, rec):
    pub.publish_context(_ctx(), result=m.RESULT_FAILED)

    assert rec.sent[0].target_kind == ''


def test_target_is_kept_on_failure(pub, rec):
    """`failed` 여도 채운다 — 무엇을 하려다 실패했는지가 남아야 한다."""
    pub.publish_context(_ctx(target_joint_names=('j1',), target_joint_values=(0.5,)),
                        result=m.RESULT_FAILED)

    assert list(rec.sent[0].target_joint_values) == pytest.approx([0.5])


# ── 3. 고정 어휘 ────────────────────────────────────────────────────────────

def test_unknown_result_is_refused_with_a_warning(pub, rec):
    """오타는 에러가 아니라 '레이블이 이상하다'로만 드러난다 — 내보내지 않는다."""
    ok = pub.publish(operation_id='op-1', operation=m.OPERATION_JOINTS, result='done')

    assert ok is False
    assert rec.sent == []
    assert "result must be one of" in pub._node.get_logger().warnings[0]


def test_unknown_operation_is_refused_with_a_warning(pub, rec):
    ok = pub.publish(operation_id='op-1', operation='move_linear',
                     result=m.RESULT_SUCCEEDED)

    assert ok is False
    assert rec.sent == []
    assert "operation must be one of" in pub._node.get_logger().warnings[0]


def test_operation_vocabulary_is_movement_shaped_not_rest_names():
    """트윈 REST 이름이 아니라 이동 방식이다 — 스크립트 경로도 같은 값을 낸다."""
    assert m.OPERATIONS == {'named_target', 'joints', 'cartesian'}


def test_result_vocabulary_is_three():
    assert m.RESULTS == {'succeeded', 'aborted', 'failed'}


# ── 4. 발행 실패가 연산을 막지 않는다 ────────────────────────────────────────

def test_publish_failure_is_swallowed_and_warned():
    node = _FakeNode(_Exploding())
    pub = m.MotionEventPublisher(node)

    ok = pub.publish_context(_ctx(), result=m.RESULT_SUCCEEDED)

    assert ok is False
    assert 'DDS is down' in node.get_logger().warnings[0]


# ── 5. 채널 규약 · 나머지 계약 ──────────────────────────────────────────────

def test_topic_is_root_relative(rec):
    node = _FakeNode(rec)
    m.MotionEventPublisher(node)

    assert node.topic == 'motion/event'
    assert not node.topic.startswith('/')


def test_stamp_defaults_to_node_clock(pub, rec):
    """`header.stamp` 를 비우면 적재가 에피소드 창 밖으로 버린다."""
    pub.publish_context(_ctx(), result=m.RESULT_SUCCEEDED)

    assert (rec.sent[0].header.stamp.sec, rec.sent[0].header.stamp.nanosec) == (7, 8)


def test_operation_id_is_carried_through(pub, rec):
    """두 채널을 잇는 유일한 끈이다 — 어긋나면 레이블이 이상해진다."""
    pub.publish_context(_ctx(operation_id='abc-123'), result=m.RESULT_SUCCEEDED)

    assert rec.sent[0].operation_id == 'abc-123'


def test_reason_is_carried_but_optional(pub, rec):
    pub.publish_context(_ctx(), result=m.RESULT_ABORTED, reason='CONTROL_FAILED')

    assert rec.sent[0].reason == 'CONTROL_FAILED'
