"""``control_msgs/GripperCommand`` 액션 → 관절 위치 토픽 브리지.

Isaac 백엔드에는 ros2_control 이 없어 `panda_hand_controller`(GripperActionController)
가 제공하던 **액션 서버가 존재하지 않는다.** 그래서 `GripperNode` 가 액션을 부르는
순간 응답 없이 멈춘다.

이 노드가 그 자리를 대신한다 — 같은 이름의 액션 서버를 열고, 받은 목표를
`sensor_msgs/JointState` 로 시뮬레이터에 발행한다.

    /gripper_cmds  (rdfp_msgs/GripperCommand)
        └ GripperNode
            └ /panda_hand_controller/gripper_cmd  (control_msgs/GripperCommand 액션)
                └ **이 노드**
                    └ /isaac/gripper_command  (sensor_msgs/JointState)

**액션 이름을 그대로 쓰는 것이 핵심이다.** 상위 경로(teleop, robot twin, 데이터셋
재생)는 백엔드가 mock 인지 Isaac 인지 몰라도 된다.

목표 도달 판정은 `/joint_states` 의 finger 관절을 보고 한다. `position_tolerance`
안에 들어오면 성공, `timeout_sec` 을 넘기면 **실패가 아니라 stalled** 로 보고한다 —
물체를 쥐어 더 닫히지 않는 상태가 정상 동작이기 때문이다.
"""

from __future__ import annotations

from typing import Optional

import threading
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import GripperCommand as GripperCommandAction
from sensor_msgs.msg import JointState

from robot_control.ros2_utils import get_parameter, parse_stripped_str

_DEFAULT_ACTION_NAME = '/panda_hand_controller/gripper_cmd'
_DEFAULT_COMMAND_TOPIC = '/isaac/gripper_command'
_DEFAULT_JOINT_STATE_TOPIC = '/joint_states'
# Panda Hand 는 손가락 두 개가 같은 폭으로 움직인다. URDF 에서는 mimic 이지만
# 시뮬레이터에는 실제 관절이 둘이므로 **둘 다** 명령한다.
_DEFAULT_FINGER_JOINTS = ['panda_finger_joint1', 'panda_finger_joint2']


class GripperActionBridge(Node):
    """액션 목표를 관절 위치 명령으로 바꿔 발행하는 브리지."""

    def __init__(self) -> None:
        super().__init__('gripper_action_bridge')

        self.declare_parameter('action_name', _DEFAULT_ACTION_NAME)
        self.declare_parameter('command_topic', _DEFAULT_COMMAND_TOPIC)
        self.declare_parameter('joint_state_topic', _DEFAULT_JOINT_STATE_TOPIC)
        self.declare_parameter('finger_joint_names', _DEFAULT_FINGER_JOINTS)
        self.declare_parameter('position_tolerance', 0.002)
        self.declare_parameter('timeout_sec', 3.0)
        self.declare_parameter('publish_rate', 50.0)

        action_name = get_parameter(self, 'action_name', parse_stripped_str)
        command_topic = get_parameter(self, 'command_topic', parse_stripped_str)
        joint_state_topic = get_parameter(self, 'joint_state_topic', parse_stripped_str)
        self._finger_joints = list(
            self.get_parameter('finger_joint_names').get_parameter_value().string_array_value
        ) or list(_DEFAULT_FINGER_JOINTS)
        self._tolerance = float(self.get_parameter('position_tolerance').value)
        self._timeout_sec = float(self.get_parameter('timeout_sec').value)
        self._publish_period = 1.0 / max(float(self.get_parameter('publish_rate').value), 1.0)

        self._lock = threading.Lock()
        self._positions: dict[str, float] = {}

        self._publisher = self.create_publisher(JointState, command_topic, 10)
        self.create_subscription(JointState, joint_state_topic, self._on_joint_state, 50)
        self._server = ActionServer(self, GripperCommandAction, action_name, self._execute)

        self.get_logger().info(
            f'GripperActionBridge started: {action_name} -> {command_topic}')
        self.get_logger().info(f'  finger joints: {self._finger_joints}')

    def _on_joint_state(self, msg: JointState) -> None:
        if not msg.name:
            return
        with self._lock:
            self._positions.update(zip(msg.name, msg.position))

    def _current_width(self) -> Optional[float]:
        """첫 번째 finger 관절의 현재 값. 아직 못 받았으면 ``None``."""
        with self._lock:
            return self._positions.get(self._finger_joints[0])

    def _publish_target(self, position: float) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._finger_joints)
        msg.position = [position] * len(self._finger_joints)
        self._publisher.publish(msg)

    def _execute(self, goal_handle) -> GripperCommandAction.Result:
        """목표 폭에 도달할 때까지 명령을 밀고 결과를 만든다."""
        target = float(goal_handle.request.command.position)
        self.get_logger().info(f'gripper goal: position={target:.4f}')

        deadline = time.time() + self._timeout_sec
        reached = False
        while rclpy.ok() and time.time() < deadline:
            self._publish_target(target)
            current = self._current_width()
            if current is not None and abs(current - target) <= self._tolerance:
                reached = True
                break
            time.sleep(self._publish_period)

        # 목표를 유지시키기 위해 마지막 명령을 한 번 더 낸다.
        self._publish_target(target)

        current = self._current_width()
        result = GripperCommandAction.Result()
        result.position = float(current) if current is not None else target
        result.effort = 0.0
        result.reached_goal = reached
        # 시간 안에 도달하지 못한 것을 실패로 보지 않는다 — 물체를 쥐어 더 닫히지
        # 않는 상태가 정상이며, 상위는 stalled 로 그것을 구분한다.
        result.stalled = not reached
        goal_handle.succeed()
        self.get_logger().info(
            f'gripper result: position={result.position:.4f} reached={reached}')
        return result


def main(args=None) -> int:
    rclpy.init(args=args)
    node = GripperActionBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
