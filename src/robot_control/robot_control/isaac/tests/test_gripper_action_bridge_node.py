"""isaac/gripper_action_bridge_node 의 액션 실행 판정 테스트.

노드를 spin 하지 않고 `_on_joint_state` / `_current_width` / `_publish_target` /
`_execute` 만 직접 부른다. `_execute` 는 벽시계로 도는 루프라, 테스트에서는
`timeout_sec` 과 `publish_period` 를 아주 짧게 두고 finger 상태를 대역이 밀어 넣는다.

**stalled 판정이 이 노드의 핵심이다.** 물체를 쥐어 목표에 못 미치는 상태가 정상이며,
그것을 실패로 보고하면 파지 성공이 액션 실패로 뒤집힌다.
"""

from __future__ import annotations

import threading

import pytest

pytest.importorskip('control_msgs', reason='requires ROS 2 runtime (control_msgs)')

from control_msgs.action import GripperCommand as GripperCommandAction   # noqa: E402
from sensor_msgs.msg import JointState                                   # noqa: E402

import robot_control.isaac.gripper_action_bridge_node as bridge_module   # noqa: E402
from robot_control.isaac.gripper_action_bridge_node import GripperActionBridge  # noqa: E402

FINGERS = ['panda_finger_joint1', 'panda_finger_joint2']


@pytest.fixture(autouse=True)
def _rclpy_running(monkeypatch):
    """`_execute` 의 루프 조건인 `rclpy.ok()` 를 참으로 만든다.

    `rclpy.init()` 을 부르지 않은 프로세스에서는 `ok()` 가 False 라 **루프가 한 번도
    돌지 않는다.** 그러면 어떤 목표를 줘도 `reached_goal=False` 가 나와 테스트가
    통과하는 것처럼 보이지만 아무것도 검증하지 못한다. DDS 를 띄우지 않으려고
    init 대신 이 지점만 대체한다.
    """
    monkeypatch.setattr(bridge_module.rclpy, 'ok', lambda *a, **k: True)


class _StubClock:
    nanoseconds = 0

    def now(self):
        return self

    def to_msg(self):
        from builtin_interfaces.msg import Time
        return Time()


class _StubLogger:
    def __init__(self) -> None:
        self.infos: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(message)


class _StubPublisher:
    def __init__(self) -> None:
        self.published: list = []

    def publish(self, msg) -> None:
        self.published.append(msg)


class _StubGoalHandle:
    def __init__(self, position: float) -> None:
        self.request = GripperCommandAction.Goal()
        self.request.command.position = float(position)
        self.succeeded = False

    def succeed(self) -> None:
        self.succeeded = True


class _StubNode:
    """Node 초기화를 피하고 대상 메서드만 빌려 쓴다."""

    def __init__(self, *, timeout_sec: float = 0.2, tolerance: float = 0.002) -> None:
        self._clock = _StubClock()
        self._logger = _StubLogger()
        self._publisher = _StubPublisher()
        self._lock = threading.Lock()
        self._positions: dict[str, float] = {}
        self._finger_joints = list(FINGERS)
        self._tolerance = tolerance
        self._timeout_sec = timeout_sec
        self._publish_period = 0.01

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._logger

    _on_joint_state = GripperActionBridge._on_joint_state
    _current_width = GripperActionBridge._current_width
    _publish_target = GripperActionBridge._publish_target
    _execute = GripperActionBridge._execute


def _joint_state(width: float, names=FINGERS) -> JointState:
    msg = JointState()
    msg.name = list(names)
    msg.position = [width] * len(names)
    return msg


# ---------- 상태 수집 ----------

def test_joint_state_without_names_is_ignored():
    """이름 없는 JointState 를 zip 하면 **엉뚱한 관절에 값이 들어간다.**"""
    node = _StubNode()
    msg = JointState()
    msg.position = [0.01, 0.01]
    node._on_joint_state(msg)
    assert node._positions == {}
    assert node._current_width() is None


def test_width_reads_the_first_finger_joint():
    node = _StubNode()
    node._on_joint_state(_joint_state(0.031))
    assert node._current_width() == pytest.approx(0.031)


def test_unrelated_joints_do_not_disturb_width():
    node = _StubNode()
    node._on_joint_state(_joint_state(0.7, names=['panda_joint1', 'panda_joint2']))
    assert node._current_width() is None


# ---------- 명령 발행 ----------

def test_target_is_published_to_both_fingers():
    node = _StubNode()
    node._publish_target(0.04)
    msg = node._publisher.published[-1]
    assert list(msg.name) == FINGERS
    assert list(msg.position) == [0.04, 0.04]


# ---------- 액션 결과 ----------

def test_reached_goal_when_within_tolerance():
    node = _StubNode()
    node._on_joint_state(_joint_state(0.0405))      # 목표 0.04, 오차 0.0005
    handle = _StubGoalHandle(0.04)
    result = node._execute(handle)

    assert result.reached_goal is True
    assert result.stalled is False
    assert handle.succeeded is True
    assert result.position == pytest.approx(0.0405)


def test_blocked_by_object_is_stalled_not_failed():
    """물체를 문 상태 — 목표 0.0 인데 0.0225 에서 멈춘다. **성공으로 끝나야 한다.**"""
    node = _StubNode()
    node._on_joint_state(_joint_state(0.0225))
    handle = _StubGoalHandle(0.0)
    result = node._execute(handle)

    assert result.reached_goal is False
    assert result.stalled is True
    # 액션 자체는 성공으로 끝난다 — 파지 성공이 실패로 뒤집히면 안 된다.
    assert handle.succeeded is True
    assert result.position == pytest.approx(0.0225)


def test_target_is_republished_after_the_loop():
    """마지막에 한 번 더 내야 목표가 유지된다 — 안 그러면 문 물체를 놓는다."""
    node = _StubNode()
    node._on_joint_state(_joint_state(0.0225))
    node._execute(_StubGoalHandle(0.0))

    assert node._publisher.published, '명령이 한 번도 나가지 않았다'
    assert list(node._publisher.published[-1].position) == [0.0, 0.0]


def test_result_position_falls_back_to_target_without_joint_state():
    """finger 상태를 한 번도 못 받으면 목표값을 그대로 돌려준다 (예외가 아니라)."""
    node = _StubNode()
    result = node._execute(_StubGoalHandle(0.04))
    assert result.position == pytest.approx(0.04)
    assert result.stalled is True


def test_late_joint_state_is_picked_up_mid_loop():
    """루프 도중 상태가 도착하면 그 시점에 도달로 판정한다."""
    node = _StubNode(timeout_sec=2.0)

    def _deliver():
        import time
        time.sleep(0.05)
        node._on_joint_state(_joint_state(0.04))

    thread = threading.Thread(target=_deliver)
    thread.start()
    result = node._execute(_StubGoalHandle(0.04))
    thread.join()

    assert result.reached_goal is True and result.stalled is False
