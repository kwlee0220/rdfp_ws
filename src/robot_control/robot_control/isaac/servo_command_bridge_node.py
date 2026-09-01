"""servo 출력(``std_msgs/Float64MultiArray``)을 Isaac 의 관절 명령으로 바꾼다.

    /servo_node/command (Float64MultiArray)
        -> [본 노드] sensor_msgs/JointState
        -> /isaac/arm_command

**두 텔레오퍼레이션 경로가 모두 servo 로 수렴하므로 이 다리가 없으면 Isaac 을 손으로
몰 수 없다.**

    teleop_keyboard  -> delta_twist_cmds ─┐
    teleop_retarget  -> ee_twist_node ────┴─> servo -> (여기)

Isaac 의 ROS2 브리지는 ``ROS2SubscribeJointState`` 만 제공하고 ``Float64MultiArray``
구독자가 없다. servo 쪽도 ``command_out_type`` 이 ``trajectory_msgs/JointTrajectory``
아니면 ``std_msgs/Float64MultiArray`` 둘뿐이라 **양쪽 어느 쪽도 상대 타입을 낼 수
없다.** 토픽 remap 으로는 못 잇는다 — 타입이 다르다.

``Float64MultiArray`` 를 고른 이유는 JGPC mock 과 같다. ``JointTrajectory`` 를 쓰면
컨트롤러가 없는 스택에서 ``/panda_arm_controller/joint_trajectory`` 라는 이름을 쓰게
되어 오해를 부른다.

joint 이름
----------

``Float64MultiArray`` 에는 이름이 없고 **배열 순서가 곧 관절 순서**다. Isaac 에는
조회할 컨트롤러가 없으므로 ``joint_names`` 파라미터가 **필수**다. 비워 두면 기동에
실패한다 — 이름 없는 ``JointState`` 를 흘리면 Isaac 이 조용히 무시하거나 엉뚱한
관절을 움직인다.

길이가 다른 배열은 **버린다.** 잘라서 쓰면 순서가 밀린 채 팔이 움직이는데, 그것이
크래시가 아니라 '그럴듯하게 틀린 자세'로 나타나 원인을 찾기 어렵다.

NaN/inf 도 **버린다.** servo 는 내부 상태가 망가지면 일곱 값을 전부 NaN 으로 내보내면서
status 는 ``NO_WARNING`` 으로 유지한다(실측). 그대로 전달하면 시뮬레이터가 조용히
무시해서, 증상이 "명령은 나가는데 팔이 안 움직인다"로만 남는다.

파라미터
--------

=================== ===================== ==========================================
이름                 기본                  설명
=================== ===================== ==========================================
joint_names         (미설정 — **필수**)    배열 순서에 대응하는 관절 이름
warn_interval_sec   5.0                   길이 불일치 경고 간격(초)
=================== ===================== ==========================================
"""

from __future__ import annotations

from typing import Optional

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from robot_control.ros2_utils import get_parameter, parse_float

_DEFAULT_INPUT_TOPIC = 'commands'
_DEFAULT_OUTPUT_TOPIC = 'arm_command'
_DEFAULT_QUEUE_SIZE = 10


class ServoCommandBridge(Node):
    """servo 의 Float64MultiArray 명령을 JointState 로 중계한다."""

    def __init__(self) -> None:
        super().__init__('servo_command_bridge')

        # 빈 리스트를 기본값으로 선언하면 rclpy 가 BYTE_ARRAY 로 추론해
        # STRING_ARRAY set 이 조용히 실패한다. 타입만 선언한다.
        self.declare_parameter('joint_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('warn_interval_sec', 5.0)

        empty = Parameter('joint_names', Parameter.Type.STRING_ARRAY, [])
        self._joint_names: list[str] = list(
            self.get_parameter_or('joint_names', empty).value or [])
        if not self._joint_names:
            raise ValueError(
                "'joint_names' is required: Float64MultiArray carries no names and "
                'there is no controller to query in a topic-linked simulator'
            )
        self._warn_interval = get_parameter(self, 'warn_interval_sec', parse_float)
        self._last_warn = float('-inf')

        self._pub = self.create_publisher(JointState, _DEFAULT_OUTPUT_TOPIC,
                                          _DEFAULT_QUEUE_SIZE)
        self._sub = self.create_subscription(Float64MultiArray, _DEFAULT_INPUT_TOPIC,
                                             self._on_command, _DEFAULT_QUEUE_SIZE)
        self.get_logger().info(
            f'ServoCommandBridge started: {_DEFAULT_INPUT_TOPIC} -> '
            f'{_DEFAULT_OUTPUT_TOPIC} (joints={self._joint_names})')

    def _warn_throttled(self, message: str) -> None:
        """같은 경고를 초당 수십 번 쏟지 않게 간격을 둔다.

        기본값이 `-inf` 여야 **첫 번째는 반드시 남는다.** 0.0 으로 두면 sim time 이
        막 0 에서 시작한 직후 구간이 통째로 삼켜지는데, Isaac 은 Stop 마다 시계가
        0 으로 되돌아가므로 늘 그 구간을 지난다.
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_warn < self._warn_interval:
            return
        self._last_warn = now
        self.get_logger().warning(message)

    def _on_command(self, msg: Float64MultiArray) -> None:
        positions = list(msg.data)
        if len(positions) != len(self._joint_names):
            # 잘라 쓰면 순서가 밀린 채 팔이 움직인다 — 크래시가 아니라 '그럴듯하게
            # 틀린 자세'가 되므로 버리고 알린다.
            self._warn_throttled(
                f'dropping command: got {len(positions)} value(s) but '
                f'{len(self._joint_names)} joint name(s) are configured'
            )
            return

        # **NaN/inf 를 흘려보내지 않는다.** servo 는 내부 상태가 망가지면 일곱 값을
        # 전부 NaN 으로 내보내면서도 status 는 `NO_WARNING` 으로 유지한다(실측).
        # 그것을 그대로 전달하면 시뮬레이터는 조용히 무시하고, 화면에는 "명령은
        # 나가는데 팔이 안 움직인다"만 남아 원인을 servo 밖에서 찾게 된다.
        if not all(math.isfinite(v) for v in positions):
            self._warn_throttled(
                'dropping command: non-finite value(s) from servo '
                f'({positions[:3]}...). servo internal state is likely broken; '
                'restart servo_node'
            )
            return

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(self._joint_names)
        out.position = [float(v) for v in positions]
        self._pub.publish(out)


def main(args=None) -> int:
    rclpy.init(args=args)
    node: Optional[ServoCommandBridge] = None
    try:
        node = ServoCommandBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
