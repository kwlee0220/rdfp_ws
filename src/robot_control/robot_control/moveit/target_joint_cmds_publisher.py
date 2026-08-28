"""arm 컨트롤러로 나간 관절 명령을 ``sensor_msgs/JointState`` 로 통일 발행한다.

JTC 스택과 JGPC 스택은 arm 명령의 메시지 타입이 서로 다르다. 본 노드는 두 형식을
모두 받아 **동일한 ``sensor_msgs/JointState``** 로 변환해 하나의 토픽
(``target_joint_cmds``) 으로 내보낸다. 그 결과 데이터셋에서는 스택 종류와 무관하게
같은 스키마(``joint_states`` 테이블)로 명령값이 적재되고, 관측값
(``/joint_states``) 과 대칭적으로 다룰 수 있다.

입력 형식은 ``source`` 파라미터로 고른다.

``joint_trajectory``
    ``trajectory_msgs/JointTrajectory`` (JTC 스택). 한 메시지에 여러 point 가 담기지만
    **마지막 point 만** 사용한다 — :mod:`target_joint_states_publisher` 와 같은 규약이다.
    joint 이름은 메시지의 ``joint_names`` 를 그대로 쓴다.

``float64_multi_array``
    ``std_msgs/Float64MultiArray`` (JGPC 스택). 이름도 header 도 없는 순수 배열이므로,
    joint 이름은 ``joint_names`` 파라미터 또는 컨트롤러의 ``joints`` 파라미터 조회로
    확정한다.

header.stamp 는 두 경우 모두 **수신 시각** 으로 채운다. JGPC 의
``Float64MultiArray`` 에는 stamp 가 없어 달리 방법이 없고, JTC 쪽도 같은 규약을 써야
두 스택의 데이터가 동일한 의미를 갖기 때문이다. 원본 발행 시각이 아니므로 DDS 전달
지연이 포함된다.

토픽 이름은 상대 이름으로 선언하므로 launch 에서 remap 한다.

- 입력: ``joint_trajectory`` 또는 ``commands``
- 출력: ``target_joint_cmds``

파라미터
--------

======================  ====================  ==================================
이름                    기본값                설명
======================  ====================  ==================================
source                  joint_trajectory      입력 형식
joint_names             (미설정)              JointState.name 에 채울 이름. 미설정
                                              시 float64_multi_array 에 한해
                                              컨트롤러에서 조회한다
controller_node_name    /panda_arm_controller  joints 파라미터를 조회할 컨트롤러
joint_names_timeout     10.0                  조회 대기 한도(초)
======================  ====================  ==================================
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

from robot_control.ros2_utils import get_parameter, log_periodic, parse_float, parse_str


_DEFAULT_OUTPUT_TOPIC = 'target_joint_cmds'
_DEFAULT_JOINT_TRAJECTORY_TOPIC = 'joint_trajectory'
_DEFAULT_COMMANDS_TOPIC = 'commands'
_DEFAULT_JOINT_STATE_TOPIC = 'arm_command'
_DEFAULT_QUEUE_SIZE = 10

_SOURCE_JOINT_TRAJECTORY = 'joint_trajectory'
_SOURCE_FLOAT64_MULTI_ARRAY = 'float64_multi_array'
# 토픽 연동형 시뮬레이터(예: 펑션베이)는 명령 자체가 이미 JointState 다.
_SOURCE_JOINT_STATE = 'joint_state'
_VALID_SOURCES = (_SOURCE_JOINT_TRAJECTORY, _SOURCE_FLOAT64_MULTI_ARRAY, _SOURCE_JOINT_STATE)

# 반복 경고 사이 최소 간격(초).
_WARN_INTERVAL_SEC = 5.0


class TargetJointCmdsPublisher(Node):
    """arm 관절 명령을 ``sensor_msgs/JointState`` 로 통일하여 재발행한다."""

    def __init__(self) -> None:
        super().__init__('target_joint_cmds_publisher')

        self.declare_parameter('source', _SOURCE_JOINT_TRAJECTORY)
        # 빈 리스트를 default 로 주면 rclpy 가 타입을 BYTE_ARRAY 로 잘못 추론하고
        # (`all(isinstance(v, bytes) for v in [])` 가 vacuous true), 이후 launch 가
        # STRING_ARRAY 로 set 할 때 조용히 실패한다. 타입만 선언하고 미설정 상태를
        # `get_parameter_or` 로 처리한다.
        self.declare_parameter('joint_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('controller_node_name', '/panda_arm_controller')
        self.declare_parameter('joint_names_timeout', 10.0)

        self._source = get_parameter(self, 'source', parse_str)
        if self._source not in _VALID_SOURCES:
            self.get_logger().error(f'Invalid source: {self._source!r}')
            raise ValueError(f"'source' must be one of {_VALID_SOURCES}, got {self._source!r}")

        self._controller_node_name = get_parameter(self, 'controller_node_name', parse_str)
        self._joint_names_timeout = get_parameter(self, 'joint_names_timeout', parse_float)
        if self._joint_names_timeout <= 0.0:
            raise ValueError(
                f"'joint_names_timeout' must be > 0, got {self._joint_names_timeout}")

        empty = Parameter('joint_names', Parameter.Type.STRING_ARRAY, [])
        self._joint_names: list[str] = list(
            self.get_parameter_or('joint_names', empty).value or [])

        # log_periodic 은 최초 호출 시 0.0 을 받으면 즉시 기록한다.
        self._last_warn_ts: float = 0.0
        self._pub = self.create_publisher(JointState, _DEFAULT_OUTPUT_TOPIC, _DEFAULT_QUEUE_SIZE)

        if self._source == _SOURCE_JOINT_TRAJECTORY:
            input_topic = _DEFAULT_JOINT_TRAJECTORY_TOPIC
            self._sub = self.create_subscription(
                JointTrajectory, input_topic, self._on_joint_trajectory, _DEFAULT_QUEUE_SIZE)
        elif self._source == _SOURCE_JOINT_STATE:
            # 이미 JointState 이므로 재stamp 만 해서 흘린다. 명령 시각을 관측
            # 시각과 같은 기준(수신 시점)으로 맞추기 위해 header 는 새로 찍는다.
            input_topic = _DEFAULT_JOINT_STATE_TOPIC
            self._sub = self.create_subscription(
                JointState, input_topic, self._on_joint_state, _DEFAULT_QUEUE_SIZE)
        else:
            input_topic = _DEFAULT_COMMANDS_TOPIC
            self._sub = self.create_subscription(
                Float64MultiArray, input_topic, self._on_float64_multi_array,
                _DEFAULT_QUEUE_SIZE)
            # Float64MultiArray 에는 이름이 없으므로 컨트롤러에서 조회한다. 컨트롤러가
            # 아직 안 떴을 수 있으니 blocking 하지 않고 one-shot 타이머로 비동기 요청한다.
            if not self._joint_names:
                self._resolve_timer = self.create_timer(0.0, self._request_joint_names)

        self.get_logger().info(
            f'TargetJointCmdsPublisher started: source={self._source!r} '
            f'{input_topic} -> {_DEFAULT_OUTPUT_TOPIC} '
            f'(queue={_DEFAULT_QUEUE_SIZE}, joint_names={self._joint_names or "(auto)"})'
        )

    # ------------------------------------------------------------------
    # joint 이름 조회 (float64_multi_array 전용)
    # ------------------------------------------------------------------

    def _request_joint_names(self) -> None:
        """컨트롤러의 ``joints`` 파라미터를 비동기로 조회한다."""
        self._resolve_timer.cancel()

        service_name = f'{self._controller_node_name}/get_parameters'
        client = self.create_client(GetParameters, service_name)
        if not client.wait_for_service(timeout_sec=self._joint_names_timeout):
            self.get_logger().error(
                f'Timed out waiting for {service_name}; JointState.name will be empty. '
                f"Set the 'joint_names' parameter explicitly to avoid this."
            )
            self.destroy_client(client)
            return

        request = GetParameters.Request()
        request.names = ['joints']
        future = client.call_async(request)
        future.add_done_callback(lambda fut: self._on_joint_names(fut, client))

    def _on_joint_names(self, future, client) -> None:
        """``joints`` 파라미터 응답을 반영한다."""
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - 서비스 실패를 로그로만 남긴다
            self.get_logger().error(f'Failed to read joints parameter: {exc}')
            return
        finally:
            self.destroy_client(client)

        if not response or not response.values:
            self.get_logger().error(
                f'{self._controller_node_name} returned no joints parameter; '
                f'JointState.name will be empty'
            )
            return

        self._joint_names = list(response.values[0].string_array_value)
        self.get_logger().info(
            f'Resolved joint_names from {self._controller_node_name}: {self._joint_names}')

    # ------------------------------------------------------------------
    # 입력 콜백
    # ------------------------------------------------------------------

    def _on_joint_trajectory(self, msg: JointTrajectory) -> None:
        """JointTrajectory 의 마지막 point 를 JointState 로 변환해 발행한다."""
        if not msg.points:
            return

        point = msg.points[-1]
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        # joint_names 파라미터가 명시되었으면 우선하고, 없으면 메시지 값을 쓴다.
        out.name = list(self._joint_names or msg.joint_names)
        out.position = [float(x) for x in point.positions]
        out.velocity = [float(x) for x in point.velocities]
        out.effort = [float(x) for x in point.effort]
        self._publish(out)

    def _on_joint_state(self, msg: JointState) -> None:
        """이미 JointState 인 명령을 재stamp 해서 그대로 발행한다.

        토픽 연동형 시뮬레이터는 명령 형식이 곧 `sensor_msgs/JointState` 다. 그래도
        이 노드를 거치는 이유는 **action 채널 토픽 이름을 백엔드 무관하게
        `/target_joint_cmds` 로 고정**하기 위해서다 — 그래야 mock / JGPC /
        시뮬레이터 데이터셋을 같은 스키마로 섞을 수 있다.
        """
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        # 입력에 이름이 없으면(순서 규약 백엔드) joint_names 파라미터로 채운다.
        out.name = list(msg.name or self._joint_names)
        out.position = [float(x) for x in msg.position]
        out.velocity = [float(x) for x in msg.velocity]
        out.effort = [float(x) for x in msg.effort]
        self._publish(out)

    def _on_float64_multi_array(self, msg: Float64MultiArray) -> None:
        """Float64MultiArray 를 JointState 로 변환해 발행한다.

        배열에는 이름도 시각도 없으므로 이름은 조회/파라미터 값으로, stamp 는 수신
        시각으로 채운다. velocity / effort 는 원본에 없으므로 빈 배열이다.
        """
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(self._joint_names)
        out.position = [float(x) for x in msg.data]
        self._publish(out)

    def _publish(self, msg: JointState) -> None:
        """이름/위치 길이를 검증한 뒤 발행한다."""
        if msg.name and len(msg.name) != len(msg.position):
            self._warn_throttled(
                f'joint_names length {len(msg.name)} != position length '
                f'{len(msg.position)}; publishing anyway'
            )
        elif not msg.name:
            self._warn_throttled(
                'joint_names is not resolved yet; publishing JointState with empty name')
        self._pub.publish(msg)

    def _warn_throttled(self, message: str) -> None:
        """동일 경고가 로그를 채우지 않도록 간격을 둔다."""
        self._last_warn_ts = log_periodic(
            self.get_logger().warning, message, self._last_warn_ts, _WARN_INTERVAL_SEC)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TargetJointCmdsPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
