"""isaac/servo_command_bridge_node 의 Float64MultiArray → JointState 변환 테스트.

**이 다리가 없으면 Isaac 을 손으로 몰 수 없다.** teleop 두 경로가 모두 servo 로
수렴하는데, servo 는 `JointTrajectory` 나 `Float64MultiArray` 만 내고 Isaac 은
`JointState` 만 받는다 — 토픽 remap 으로는 못 잇는다.

가장 위험한 실패는 **길이 불일치를 잘라서 쓰는 것**이다. 순서가 밀린 채 팔이 움직이며,
크래시가 아니라 '그럴듯하게 틀린 자세'로 나타나 원인을 찾기 어렵다.
"""

from __future__ import annotations

import pytest

pytest.importorskip('sensor_msgs', reason='requires ROS 2 runtime (sensor_msgs)')

from sensor_msgs.msg import JointState                    # noqa: E402
from std_msgs.msg import Float64MultiArray                # noqa: E402

from robot_control.isaac.servo_command_bridge_node import ServoCommandBridge  # noqa: E402

JOINTS = [f'panda_joint{i}' for i in range(1, 8)]


class _StubClock:
    def __init__(self, seconds: float = 100.0) -> None:
        self.nanoseconds = int(seconds * 1e9)

    def now(self):
        return self

    def to_msg(self):
        from builtin_interfaces.msg import Time
        stamp = Time()
        stamp.sec = int(self.nanoseconds // 1_000_000_000)
        stamp.nanosec = int(self.nanoseconds % 1_000_000_000)
        return stamp


class _StubLogger:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def warning(self, message: str) -> None:
        self.warnings.append(message)


class _StubPublisher:
    def __init__(self) -> None:
        self.published: list[JointState] = []

    def publish(self, msg) -> None:
        self.published.append(msg)


class _StubNode:
    """Node 초기화를 피하고 대상 메서드만 빌려 쓴다."""

    def __init__(self, joints=None, seconds: float = 100.0) -> None:
        self._clock = _StubClock(seconds)
        self._logger = _StubLogger()
        self._pub = _StubPublisher()
        self._joint_names = list(JOINTS if joints is None else joints)
        self._warn_interval = 5.0
        self._last_warn = float('-inf')

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._logger

    _warn_throttled = ServoCommandBridge._warn_throttled
    _on_command = ServoCommandBridge._on_command


def _cmd(*values: float) -> Float64MultiArray:
    msg = Float64MultiArray()
    msg.data = [float(v) for v in values]
    return msg


# ---------- 변환 ----------

def test_array_order_becomes_joint_names_in_order():
    """배열 순서가 곧 관절 순서다 — 재배열하면 조용히 틀린다."""
    node = _StubNode()
    node._on_command(_cmd(0.1, -0.2, 0.3, -2.4, 0.5, 1.6, 0.7))

    out = node._pub.published[-1]
    assert list(out.name) == JOINTS
    assert [round(v, 3) for v in out.position] == [0.1, -0.2, 0.3, -2.4, 0.5, 1.6, 0.7]


def test_stamp_is_filled():
    """stamp 를 비우면 적재 시 epoch 0 으로 들어가 어느 에피소드에도 속하지 못한다."""
    node = _StubNode()
    node._on_command(_cmd(*[0.0] * 7))
    stamp = node._pub.published[-1].header.stamp
    assert (stamp.sec, stamp.nanosec) != (0, 0)


def test_velocity_and_effort_stay_empty():
    """servo 는 위치만 낸다 — 빈 필드를 지어내지 않는다."""
    node = _StubNode()
    node._on_command(_cmd(*[0.0] * 7))
    out = node._pub.published[-1]
    assert list(out.velocity) == [] and list(out.effort) == []


# ---------- 길이 불일치 ----------

@pytest.mark.parametrize('count', [0, 1, 6, 8, 9])
def test_length_mismatch_is_dropped_not_truncated(count):
    """잘라 쓰면 순서가 밀린 채 팔이 움직인다. 버리고 알린다."""
    node = _StubNode()
    node._on_command(_cmd(*[0.1] * count))
    assert node._pub.published == []
    assert node._logger.warnings, '조용히 버리면 원인을 찾을 수 없다'


def test_mismatch_warning_is_throttled_but_first_always_logged():
    """**sim time 0 근처에서도 첫 경고는 나와야 한다.** Isaac 은 Stop 마다 0 으로 돌아간다."""
    node = _StubNode(seconds=0.0)
    node._on_command(_cmd(0.1))
    assert len(node._logger.warnings) == 1

    node._on_command(_cmd(0.1))          # 같은 시각 — 억제된다
    assert len(node._logger.warnings) == 1

    node._clock.nanoseconds = int(6 * 1e9)
    node._on_command(_cmd(0.1))          # 간격을 넘겼다
    assert len(node._logger.warnings) == 2


def test_recovers_after_a_bad_message():
    """한 번 어긋났다고 이후 정상 메시지까지 막으면 안 된다."""
    node = _StubNode()
    node._on_command(_cmd(0.1))
    node._on_command(_cmd(*[0.0] * 7))
    assert len(node._pub.published) == 1


# ---------- 필수 파라미터 ----------

def test_joint_names_is_required():
    """조회할 컨트롤러가 없으므로 이름은 반드시 설정돼야 한다.

    비워 두면 이름 없는 JointState 가 나가고, Isaac 은 그것을 조용히 무시하거나
    엉뚱한 관절을 움직인다 — 기동 단계에서 막는 편이 낫다.
    """
    import inspect
    source = inspect.getsource(ServoCommandBridge.__init__)
    assert "'joint_names' is required" in source
    assert 'Parameter.Type.STRING_ARRAY' in source, (
        '빈 리스트를 기본값으로 선언하면 rclpy 가 BYTE_ARRAY 로 추론해 '
        'STRING_ARRAY set 이 조용히 실패한다')


# ---------- launch 쪽 servo 파라미터 오버라이드 ----------

def _isaac_launch():
    """`panda_isaac.launch.py` 를 모듈로 읽어 온다 (launch 실행은 하지 않는다)."""
    import importlib.util
    import pathlib

    pytest.importorskip('launch_ros', reason='requires ROS 2 launch')
    # robot_control/robot_control/isaac/tests/<this> -> src/robot_control/launch/
    path = (pathlib.Path(__file__).resolve().parents[3]
            / 'launch' / 'panda_isaac.launch.py')
    if not path.is_file():
        pytest.skip('launch 파일이 없는 트리 (installed)')
    spec = importlib.util.spec_from_file_location('panda_isaac_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_servo_must_not_publish_velocities():
    """**선택이 아니라 필수다.** 켜 두면 servo 의 파라미터 검증이 실패해 노드가
    아예 기동하지 못한다 — 증상은 '팔이 안 움직인다' 뿐이라 원인이 안 보인다.
    """
    launch = _isaac_launch()
    params = launch.override_servo_params_for_isaac(
        {'moveit_servo': {'publish_joint_velocities': True}})['moveit_servo']

    assert params['publish_joint_velocities'] is False
    assert params['publish_joint_positions'] is True
    assert params['command_out_type'] == 'std_msgs/Float64MultiArray'


def test_servo_output_topic_is_not_a_controller_name():
    """컨트롤러가 없는 스택에서 `/panda_arm_controller/...` 를 쓰면 오해를 부른다."""
    launch = _isaac_launch()
    topic = launch.override_servo_params_for_isaac(
        {'moveit_servo': {}})['moveit_servo']['command_out_topic']

    assert topic == launch.SERVO_COMMAND_TOPIC
    assert 'controller' not in topic


def test_bridge_joint_names_match_the_arm_group():
    """배열 순서가 곧 관절 순서다 — launch 가 넘기는 이름이 팔 7관절이어야 한다."""
    launch = _isaac_launch()
    assert launch.ARM_JOINT_NAMES == JOINTS


# ---------- 유한하지 않은 값 ----------

@pytest.mark.parametrize('bad', [float('nan'), float('inf'), float('-inf')])
def test_non_finite_values_are_dropped(bad):
    """servo 는 내부 상태가 망가지면 **일곱 값을 전부 NaN 으로** 내보내면서
    status 는 `NO_WARNING` 을 유지한다 (실측). 그대로 전달하면 시뮬레이터가 조용히
    무시해 "명령은 나가는데 팔이 안 움직인다"만 남는다.
    """
    node = _StubNode()
    node._on_command(_cmd(*[bad] * 7))
    assert node._pub.published == []
    assert node._logger.warnings


def test_single_non_finite_value_poisons_the_whole_command():
    """한 관절만 NaN 이어도 나머지를 보내면 안 된다 — 팔이 부분적으로 움직인다."""
    node = _StubNode()
    node._on_command(_cmd(0.1, 0.2, float('nan'), 0.4, 0.5, 0.6, 0.7))
    assert node._pub.published == []


def test_finite_commands_still_pass_after_a_nan():
    """servo 가 회복하면 다시 흘려야 한다."""
    node = _StubNode()
    node._on_command(_cmd(*[float('nan')] * 7))
    node._on_command(_cmd(*[0.1] * 7))
    assert len(node._pub.published) == 1
