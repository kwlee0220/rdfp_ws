"""JointTrajectory 의 header.stamp 를 현재 시각으로 갱신하여 재발행하는 relay 노드."""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from rdfp_msgs.msg import TargetJointStates
from trajectory_msgs.msg import JointTrajectory

_DEFAULT_INPUT_TOPIC = 'joint_trajectory'
_DEFAULT_OUTPUT_TOPIC = 'target_joint_states'
_DEFAULT_QUEUE_SIZE = 10


class TargetJointStatePublisher(Node):
    """입력 토픽의 JointTrajectory 메시지를 받아 header.stamp 를 현재 시각으로 채워 재발행한다.

    `JointTrajectory` 메시지의 point 들 중 마지막 point 만을 사용하여
    `TargetJointStates` 메시지를 구성한다.
    또한 헤더 부분에는 현재 시각이 `stamp` 로, 입력 메시지의 `frame_id` 가 그대로 복사된다.
    생성된 메시지는 로봇 학습 데이터에서 action 부분을 구성하는 데 사용된다.
    
     - 입력: JointTrajectory (예: MoveIt! 의 FollowJointTrajectoryAction 의 goal 토픽)
     - 출력: TargetJointStates (로봇 학습 데이터의 action 토픽)
    """

    def __init__(self) -> None:
        super().__init__('target_joint_states_publisher')

        self._pub = self.create_publisher(TargetJointStates, _DEFAULT_OUTPUT_TOPIC,
                                          _DEFAULT_QUEUE_SIZE)
        self._sub = self.create_subscription(JointTrajectory, _DEFAULT_INPUT_TOPIC, self._on_msg,
                                             _DEFAULT_QUEUE_SIZE)

        self.get_logger().info(
            f"TargetJointStatePublisher started: {_DEFAULT_INPUT_TOPIC} -> {_DEFAULT_OUTPUT_TOPIC} "
            f"(queue={_DEFAULT_QUEUE_SIZE})"
        )

    def _on_msg(self, msg: JointTrajectory) -> None:
        # 마지막 point 만 뽑아 현재 시각을 header.stamp 로 갖는 TargetJointStates 로 구성한다.
        if not msg.points:
            return
        
        if len(msg.points) > 1:
            self.get_logger().warning(f"Received JointTrajectory with {len(msg.points)} points; "
                                      f"the intermediate points will be ignored.")

        stamped = TargetJointStates()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = msg.header.frame_id
        stamped.point = msg.points[-1]  # type: ignore[index]
        self._pub.publish(stamped)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TargetJointStatePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
