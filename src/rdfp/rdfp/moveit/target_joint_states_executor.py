"""TargetJointStates 를 길이 1 짜리 JointTrajectory 로 래핑하여 재발행하는 relay 노드.

:mod:`target_joint_states_publisher` 의 **반대 동작** 을 수행한다.
학습/재생 시나리오에서, 기록돼 있던 단일 `TargetJointStates` 샘플을 다시
trajectory_msgs/JointTrajectory 를 요구하는 FollowJointTrajectoryAction 경로로
흘려보내기 위한 어댑터이다.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from rdfp_msgs.msg import TargetJointStates
from trajectory_msgs.msg import JointTrajectory


_DEFAULT_INPUT_TOPIC = 'target_joint_states'
_DEFAULT_OUTPUT_TOPIC = 'joint_trajectory'
_DEFAULT_QUEUE_SIZE = 10


class TargetJointStatesExecutor(Node):
    """`TargetJointStates` 메시지를 받아 `JointTrajectory` (points=[point]) 로 재발행한다.

    - 입력: TargetJointStates (로봇 학습 데이터의 action 토픽)
    - 출력: JointTrajectory (예: MoveIt! 의 FollowJointTrajectoryAction 입력)

    출력 `JointTrajectory` 의 `points` 는 입력 메시지의 단일 point 를 원소로
    갖는 길이 1 리스트로 채워진다. `header.stamp` 는 현재 시각으로, `frame_id`
    는 입력 메시지의 값을 복사한다.

    `TargetJointStates` 스키마에는 `joint_names` 필드가 없으므로 ROS 2 파라미터
    ``joint_names`` 로 주입받는다. 파라미터가 비어 있으면 빈 리스트 그대로
    발행하되 최초 1 회 경고를 남긴다 (FollowJointTrajectoryAction 등 일부
    소비자는 빈 joint_names 를 거부한다).
    """

    def __init__(self) -> None:
        super().__init__('target_joint_states_executor')

        # 출력 JointTrajectory 의 joint_names 를 채우기 위한 파라미터.
        # 빈 리스트 (`[]`) 를 default 로 전달하면 rclpy 가 타입을 BYTE_ARRAY 로
        # 잘못 추론한다 (`all(isinstance(v, bytes) for v in [])` 가 vacuous true 라
        # 가장 먼저 매칭됨). 그래서 이후 launch 가 STRING_ARRAY 로 set 할 때
        # InvalidParameterType 예외로 노드가 즉사한다. `Parameter.Type.STRING_ARRAY`
        # 를 default 로 넘겨 "기본값 없이 타입만" 선언한다 — override 가 반드시
        # 주어져야 동작한다 (launch 또는 `ros2 run ... -p joint_names:=[...]`).
        self.declare_parameter('joint_names', Parameter.Type.STRING_ARRAY)
        self._joint_names: list[str] = list(
            self.get_parameter('joint_names').get_parameter_value().string_array_value
        )
        self._warned_empty_joint_names = False

        self._pub = self.create_publisher(JointTrajectory, _DEFAULT_OUTPUT_TOPIC,
                                          _DEFAULT_QUEUE_SIZE)
        self._sub = self.create_subscription(TargetJointStates, _DEFAULT_INPUT_TOPIC,
                                             self._on_msg, _DEFAULT_QUEUE_SIZE)

        self.get_logger().info(
            f"TargetJointStatesExecutor started: {_DEFAULT_INPUT_TOPIC} -> "
            f"{_DEFAULT_OUTPUT_TOPIC} (queue={_DEFAULT_QUEUE_SIZE}, "
            f"joint_names={self._joint_names or '[]'})"
        )

    def _on_msg(self, msg: TargetJointStates) -> None:
        if not self._joint_names and not self._warned_empty_joint_names:
            self.get_logger().warning(
                "joint_names parameter is empty; publishing JointTrajectory with empty "
                "joint_names — downstream controllers may reject. Set the 'joint_names' "
                "parameter to the robot's joint list."
            )
            self._warned_empty_joint_names = True

        out = JointTrajectory()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.joint_names = list(self._joint_names)
        out.points = [msg.point]
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TargetJointStatesExecutor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
