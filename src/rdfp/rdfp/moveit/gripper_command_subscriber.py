"""`rdfp_msgs/GripperCommand` 구독 → gripper action 실행 노드.

데이터셋 재생이나 외부 퍼블리셔가 발행하는 `GripperCommand` 메시지를 받아
`command` 필드(``"open"`` / ``"close"``) 에 해당하는 gripper action goal 을
`/panda_hand_controller/gripper_cmd` 로 전송한다.

:class:`rdfp.moveit.gripper_control_node.GripperControlNode` 와 달리 Trigger
서비스 인터페이스는 제공하지 않으며, 구독 토픽 단방향으로만 동작한다.
"""

from __future__ import annotations

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node

from control_msgs.action import GripperCommand as GripperCommandAction
from rdfp_msgs.msg import GripperCommand


_GRIPPER_ACTION_NAME = '/panda_hand_controller/gripper_cmd'
_GRIPPER_CMD_TOPIC = '~/gripper_cmds'

# Panda 손 관례: open = 0.04 m, close = 0.0 m.
_GRIPPER_OPEN_POSITION = 0.04
_GRIPPER_CLOSE_POSITION = 0.0
# 0 이면 드라이버 기본 효과치를 사용한다. 필요 시 파라미터화 가능.
_GRIPPER_MAX_EFFORT = 0.0


class GripperCommandSubscriber(Node):
    """GripperCommand 를 구독해 gripper action 을 실행하는 단일 책임 노드."""

    def __init__(self) -> None:
        super().__init__('gripper_command_subscriber')

        # 액션 서버가 당장 없을 수 있으므로 send 시점에 ready 여부만 확인한다.
        self._action = ActionClient(self, GripperCommandAction, _GRIPPER_ACTION_NAME)
        self._sub = self.create_subscription(GripperCommand, _GRIPPER_CMD_TOPIC,
                                             self._on_cmd, 10)

        self.get_logger().info(
            "GripperCommandSubscriber started "
            f"(topic: {_GRIPPER_CMD_TOPIC}, action: {_GRIPPER_ACTION_NAME})"
        )

    # ── Subscription callback ───────────────────────────────────

    def _on_cmd(self, msg: GripperCommand) -> None:
        """``command`` 필드를 해석해 gripper action goal 로 변환·전송한다."""
        command = msg.command.strip().lower()
        if command == 'open':
            position = _GRIPPER_OPEN_POSITION
        elif command == 'close':
            position = _GRIPPER_CLOSE_POSITION
        else:
            self.get_logger().warning(
                f"[gripper] unknown command {msg.command!r}; expected 'open' or 'close'"
            )
            return

        if not self._action.server_is_ready():
            self.get_logger().warning(
                f"[gripper] {command}: action server '{_GRIPPER_ACTION_NAME}' not ready"
            )
            return

        goal = GripperCommandAction.Goal()
        goal.command.position = position
        goal.command.max_effort = _GRIPPER_MAX_EFFORT
        self._action.send_goal_async(goal)
        self.get_logger().info(f"[gripper] {command} (position={position:.3f} m)")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperCommandSubscriber()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


__all__ = ['GripperCommandSubscriber', 'main']
