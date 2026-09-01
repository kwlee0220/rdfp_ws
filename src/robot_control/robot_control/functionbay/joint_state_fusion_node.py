"""펑션베이 관절 보고 → `/joint_states` 변환 노드.

펑션베이는 `sensor_msgs/JointState` 를 발행하지만 **`name` 을 채우지 않는다** —
배열 순서가 곧 계약이다. 이 노드가 순서에 이름을 부여해 ROS 관례에 맞는
`/joint_states` 로 바꾼다.

왜 필요한가
-----------
`robot_state_publisher` 는 URDF 의 non-fixed joint 를 **이름으로** 찾는다. 이름이
없으면 TF 트리가 만들어지지 않아 RViz 표시 · MoveIt 충돌검사 · `ee_pose_node`
가 전부 동작하지 않는다.

또 펑션베이는 팔 7축만 보고한다. Panda URDF 에는 `panda_finger_joint1` 도
non-fixed 이므로 이것까지 채워야 TF 가 손끝까지 이어진다
(`panda_finger_joint2` 는 URDF 에서 `<mimic>` 이라 rsp 가 파생한다).

그리퍼는 아직 연동하지 않는다 (실물이 Robotiq 2F-85 라 Panda Hand 를 전제한
URDF/SRDF 와 기구학이 다르다). 그때까지는 `extra_joint_positions` 의 고정값으로
TF 만 성립시킨다.

파라미터
--------
======================== ============================ ==================================
이름                      기본값                        설명
======================== ============================ ==================================
input_topic              /output/panda_joint          시뮬레이터 관절 보고
output_topic             /joint_states                변환 결과
joint_names              panda_joint1..7              **입력 배열 순서와 일치해야 한다**
extra_joint_names        [panda_finger_joint1]        입력에 없지만 URDF 가 요구하는 관절
extra_joint_positions    [0.04]                       위 관절에 채울 고정값
stamp_source             auto                         auto | incoming | now
======================== ============================ ==================================

``stamp_source=auto`` 는 입력 stamp 가 0 이면 노드 시계를, 아니면 입력값을 쓴다.
펑션베이는 시뮬 시간을 쓰지 않으므로 벽시계 기준이다.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import JointState

from robot_control.ros2_utils import get_parameter, log_periodic, parse_str

_DEFAULT_INPUT_TOPIC = '/output/panda_joint'
_DEFAULT_OUTPUT_TOPIC = '/joint_states'
_DEFAULT_ARM_JOINT_NAMES = [f'panda_joint{i}' for i in range(1, 8)]
_DEFAULT_EXTRA_JOINT_NAMES = ['panda_finger_joint1']
_DEFAULT_EXTRA_JOINT_POSITIONS = [0.04]
_DEFAULT_QUEUE_SIZE = 10

_STAMP_AUTO = 'auto'
_STAMP_INCOMING = 'incoming'
_STAMP_NOW = 'now'
_VALID_STAMP_SOURCES = (_STAMP_AUTO, _STAMP_INCOMING, _STAMP_NOW)

# 반복 경고 사이 최소 간격(초).
_WARN_INTERVAL_SEC = 5.0


class JointStateFusionNode(Node):
    """이름 없는 관절 보고에 이름을 부여해 `/joint_states` 로 재발행한다."""

    def __init__(self) -> None:
        super().__init__('joint_state_fusion')

        self.declare_parameter('input_topic', _DEFAULT_INPUT_TOPIC)
        self.declare_parameter('output_topic', _DEFAULT_OUTPUT_TOPIC)
        self.declare_parameter('stamp_source', _STAMP_AUTO)
        # STRING_ARRAY / DOUBLE_ARRAY 는 타입만 선언한다. 빈 리스트를 기본값으로
        # 주면 rclpy 가 BYTE_ARRAY 로 추론해 이후 설정이 조용히 실패한다.
        self.declare_parameter('joint_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('extra_joint_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('extra_joint_positions', Parameter.Type.DOUBLE_ARRAY)

        input_topic = get_parameter(self, 'input_topic', parse_str)
        output_topic = get_parameter(self, 'output_topic', parse_str)
        self._stamp_source = get_parameter(self, 'stamp_source', parse_str)
        if self._stamp_source not in _VALID_STAMP_SOURCES:
            raise ValueError(
                f"'stamp_source' must be one of {list(_VALID_STAMP_SOURCES)}, "
                f'got {self._stamp_source!r}')

        self._joint_names = self._string_array('joint_names', _DEFAULT_ARM_JOINT_NAMES)
        self._extra_names = self._string_array('extra_joint_names', _DEFAULT_EXTRA_JOINT_NAMES)
        self._extra_positions = self._double_array('extra_joint_positions',
                                                   _DEFAULT_EXTRA_JOINT_POSITIONS)
        if len(self._extra_names) != len(self._extra_positions):
            raise ValueError(
                f"'extra_joint_names' ({len(self._extra_names)}) and "
                f"'extra_joint_positions' ({len(self._extra_positions)}) must have "
                f'the same length')

        self._publisher = self.create_publisher(JointState, output_topic, _DEFAULT_QUEUE_SIZE)
        self._subscription = self.create_subscription(
            JointState, input_topic, self._on_joint_state, _DEFAULT_QUEUE_SIZE)

        # 입력이 이름을 채워 주는지는 첫 메시지에서만 로깅한다.
        self._logged_name_source = False
        # log_periodic 은 마지막 기록 시각을 호출자가 들고 있는 방식이다.
        self._last_name_warn = 0.0
        self._last_pos_warn = 0.0

        self.get_logger().info(
            f'JointStateFusionNode started: {input_topic} -> {output_topic}')
        # 순서가 계약이므로 반드시 남긴다. 틀리면 에러 없이 엉뚱한 관절이 움직인다.
        self.get_logger().info(f'  expected input order: {self._joint_names}')
        self.get_logger().info(
            f'  appended joints: {list(zip(self._extra_names, self._extra_positions))}')

    def _string_array(self, name: str, default: list[str]) -> list[str]:
        """STRING_ARRAY 파라미터를 읽는다. 미설정이면 기본값."""
        empty = Parameter(name, Parameter.Type.STRING_ARRAY, [])
        value = list(self.get_parameter_or(name, empty).value or [])
        return value or list(default)

    def _double_array(self, name: str, default: list[float]) -> list[float]:
        """DOUBLE_ARRAY 파라미터를 읽는다. 미설정이면 기본값."""
        empty = Parameter(name, Parameter.Type.DOUBLE_ARRAY, [])
        value = list(self.get_parameter_or(name, empty).value or [])
        return [float(v) for v in value] if value else list(default)

    def _resolve_names(self, msg: JointState) -> Optional[list[str]]:
        """입력에 쓸 관절 이름을 정한다. 길이가 맞지 않으면 ``None``."""
        if msg.name:
            # 시뮬레이터가 이름을 채우기 시작하면 그것을 신뢰한다 (순서 계약 탈피).
            if not self._logged_name_source:
                self.get_logger().info('input provides joint names; using them as-is')
                self._logged_name_source = True
            if len(msg.name) != len(msg.position):
                self._last_name_warn = log_periodic(
                    self.get_logger().error,
                    f'name/position length mismatch: {len(msg.name)} vs '
                    f'{len(msg.position)}; dropping',
                    self._last_name_warn, _WARN_INTERVAL_SEC)
                return None
            return list(msg.name)

        if not self._logged_name_source:
            self.get_logger().info(
                'input has no joint names; assigning by array order (see joint_names)')
            self._logged_name_source = True
        if len(msg.position) != len(self._joint_names):
            self._last_pos_warn = log_periodic(
                self.get_logger().error,
                f'expected {len(self._joint_names)} positions, got {len(msg.position)}; '
                f'dropping (check joint_names order)',
                self._last_pos_warn, _WARN_INTERVAL_SEC)
            return None
        return list(self._joint_names)

    def _on_joint_state(self, msg: JointState) -> None:
        """입력 보고에 이름을 붙이고 고정 관절을 덧붙여 재발행한다."""
        names = self._resolve_names(msg)
        if names is None:
            return

        out = JointState()
        out.header.stamp = self._stamp(msg)
        out.header.frame_id = msg.header.frame_id
        out.name = names + list(self._extra_names)
        out.position = list(msg.position) + list(self._extra_positions)
        # 속도/토크는 길이가 맞을 때만 옮긴다. 덧붙인 관절 몫은 0 으로 채운다.
        if len(msg.velocity) == len(names):
            out.velocity = list(msg.velocity) + [0.0] * len(self._extra_names)
        if len(msg.effort) == len(names):
            out.effort = list(msg.effort) + [0.0] * len(self._extra_names)
        self._publisher.publish(out)

    def _stamp(self, msg: JointState):
        """`stamp_source` 정책에 따라 헤더 시각을 정한다."""
        if self._stamp_source == _STAMP_NOW:
            return self.get_clock().now().to_msg()
        incoming = msg.header.stamp
        if self._stamp_source == _STAMP_INCOMING:
            return incoming
        # auto: 입력 stamp 가 0 이면 노드 시계를 쓴다.
        if incoming.sec == 0 and incoming.nanosec == 0:
            return self.get_clock().now().to_msg()
        return incoming


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = JointStateFusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:      # noqa: BLE001 - 기동 실패를 명확히 보고한다
        print(f'Fatal error: {exc}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
