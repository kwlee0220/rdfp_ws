"""재생된 관절 명령(``sensor_msgs/JointState``)을 arm 컨트롤러로 흘려보낸다.

:mod:`target_joint_cmds_publisher` 가 녹화 시점에 만든 ``target_joint_cmds``
스트림을 재생할 때, 이를 ``trajectory_msgs/JointTrajectory`` (point 1개) 로 감싸
``panda_arm_controller`` 가 소비할 수 있게 변환한다. 관절 **절대 위치** 를 그대로
명령하므로 위치 폐루프이며 드리프트가 없다.

    /target_joint_cmds (JointState)
        -> [본 노드] JointTrajectory(points=[1개])
        -> /panda_arm_controller/joint_trajectory

joint 이름 처리
---------------

2026-09-18 부터 DB 의 ``joint_states`` 테이블이 관절 이름을 저장하고 재생 reader 가
``JointState.name`` 으로 되살리므로, DB 재생으로 들어온 메시지에도 대개 ``name`` 이 있다.
비어 오는 것은 그 전에 적재한 에피소드와, 발행자(``target_joint_cmds_publisher``)가
컨트롤러 ``joints`` 조회 시간 초과로 이름을 못 채운 행뿐이다. 그런 경우
``joint_names`` 파라미터 값을 사용한다. 우선순위는 다음과 같다.

1. 메시지의 ``name`` (라이브 중계, 09-18 이후 적재분의 DB 재생)
2. ``joint_names`` 파라미터 (이름이 빈 행 — 배열이 그 순서라고 가정한다)

둘 다 비면 컨트롤러가 거부할 수 있는 빈 ``joint_names`` 로 발행하고 경고한다.

파라미터
--------

=================  ========  ================================================
이름               기본값    설명
=================  ========  ================================================
joint_names        (미설정)  메시지에 name 이 없을 때 사용할 이름 목록
time_from_start    0.0       생성할 point 의 time_from_start(초).
                             0 이면 컨트롤러가 즉시 반영한다
=================  ========  ================================================
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robot_control.ros2_utils import get_parameter, log_periodic, parse_float


_DEFAULT_INPUT_TOPIC = 'target_joint_cmds'
_DEFAULT_OUTPUT_TOPIC = 'joint_trajectory'
_DEFAULT_QUEUE_SIZE = 10

# 반복 경고 사이 최소 간격(초).
_WARN_INTERVAL_SEC = 5.0


class TargetJointCmdsExecutor(Node):
    """``JointState`` 명령을 길이 1 짜리 ``JointTrajectory`` 로 감싸 재발행한다."""

    def __init__(self) -> None:
        super().__init__('target_joint_cmds_executor')

        # 빈 리스트를 default 로 주면 rclpy 가 타입을 BYTE_ARRAY 로 잘못 추론하고
        # 이후 launch 의 STRING_ARRAY set 이 조용히 실패한다. 타입만 선언한다.
        self.declare_parameter('joint_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('time_from_start', 0.0)

        empty = Parameter('joint_names', Parameter.Type.STRING_ARRAY, [])
        self._joint_names: list[str] = list(
            self.get_parameter_or('joint_names', empty).value or [])

        time_from_start = get_parameter(self, 'time_from_start', parse_float)
        if time_from_start < 0.0:
            raise ValueError(f"'time_from_start' must be >= 0, got {time_from_start}")
        self._time_from_start = Duration()
        self._time_from_start.sec = int(time_from_start)
        self._time_from_start.nanosec = int(round((time_from_start % 1.0) * 1e9))

        self._last_warn_ts: float = 0.0

        self._pub = self.create_publisher(JointTrajectory, _DEFAULT_OUTPUT_TOPIC,
                                          _DEFAULT_QUEUE_SIZE)
        self._sub = self.create_subscription(JointState, _DEFAULT_INPUT_TOPIC, self._on_msg,
                                             _DEFAULT_QUEUE_SIZE)

        self.get_logger().info(
            f'TargetJointCmdsExecutor started: {_DEFAULT_INPUT_TOPIC} -> '
            f'{_DEFAULT_OUTPUT_TOPIC} (queue={_DEFAULT_QUEUE_SIZE}, '
            f'joint_names={self._joint_names or "(from message)"}, '
            f'time_from_start={time_from_start}s)'
        )

    def _on_msg(self, msg: JointState) -> None:
        """JointState 를 point 1개짜리 JointTrajectory 로 변환해 발행한다."""
        if not msg.position:
            return

        # 메시지의 이름을 우선하고, 비어 있으면 파라미터로 폴백한다.
        joint_names = list(msg.name) if msg.name else list(self._joint_names)
        if not joint_names:
            self._warn_throttled(
                'JointState has no name and the joint_names parameter is unset; publishing '
                'JointTrajectory with empty joint_names — the controller may reject it'
            )
        elif len(joint_names) != len(msg.position):
            self._warn_throttled(
                f'joint_names length {len(joint_names)} != position length '
                f'{len(msg.position)}; publishing anyway'
            )

        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in msg.position]
        # velocity / effort 는 길이가 맞을 때만 싣는다. JTC 는 길이가 어긋나면
        # 궤적 전체를 거부한다.
        if len(msg.velocity) == len(msg.position):
            point.velocities = [float(x) for x in msg.velocity]
        if len(msg.effort) == len(msg.position):
            point.effort = [float(x) for x in msg.effort]
        point.time_from_start = self._time_from_start

        out = JointTrajectory()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.joint_names = joint_names
        out.points = [point]
        self._pub.publish(out)

    def _warn_throttled(self, message: str) -> None:
        """동일 경고가 로그를 채우지 않도록 간격을 둔다."""
        self._last_warn_ts = log_periodic(
            self.get_logger().warning, message, self._last_warn_ts, _WARN_INTERVAL_SEC)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TargetJointCmdsExecutor()
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
