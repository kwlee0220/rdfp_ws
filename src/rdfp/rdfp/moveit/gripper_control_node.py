"""Gripper 제어 전담 ROS 2 노드.

`~/open_gripper` / `~/close_gripper` (Trigger) 서비스 호출을 받아 다음 세 경로로
명령을 내보낸다.

1. `/teleop/gripper_cmds` 토픽에 `rdfp_msgs/GripperCommand` 로 의도(``"open"`` /
   ``"close"``) 를 기록한다.
2. `/panda_hand_controller/gripper_cmd` 액션에 실제 goal 을 전송한다.
3. 액션 feedback 을 `_on_feedback` 에서 `/teleop/gripper_states` 로 재발행한다.

TeleopKeyboard 등 클라이언트는 본 노드의 두 서비스를 호출하여 gripper 를
제어한다.
"""

from __future__ import annotations

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node

from control_msgs.action import GripperCommand as GripperCommandAction
from rdfp_msgs.msg import GripperCommand, GripperState
from std_srvs.srv import Trigger


_GRIPPER_ACTION_NAME = '/panda_hand_controller/gripper_cmd'
_GRIPPER_CMD_TOPIC = '~/gripper_cmds'
_GRIPPER_STATE_TOPIC = '~/gripper_states'

# Panda 손 관례: open = 0.04 m, close = 0.0 m.
_GRIPPER_OPEN_POSITION = 0.04
_GRIPPER_CLOSE_POSITION = 0.0
# 0 이면 드라이버 기본 효과치를 사용한다. 필요 시 파라미터화 가능.
_GRIPPER_MAX_EFFORT = 0.0


class GripperControlNode(Node):
    """Gripper 를 open/close 하는 단일 책임 노드."""

    def __init__(self) -> None:
        super().__init__('gripper_control')

        # --- Publishers ---
        self._cmd_pub = self.create_publisher(GripperCommand, _GRIPPER_CMD_TOPIC, 10)
        self._state_pub = self.create_publisher(GripperState, _GRIPPER_STATE_TOPIC, 10)

        # --- Action client ---
        # 서버가 당장 없을 수 있으므로 send 시점에 ready 여부만 확인한다.
        self._action = ActionClient(self, GripperCommandAction, _GRIPPER_ACTION_NAME)

        # --- Services ---
        self._open_srv = self.create_service(Trigger, '~/open_gripper', self._handle_open)
        self._close_srv = self.create_service(Trigger, '~/close_gripper', self._handle_close)

        self.get_logger().info(
            "GripperControlNode started "
            f"(services: ~/open_gripper, ~/close_gripper; action: {_GRIPPER_ACTION_NAME})"
        )

    # ── Service callbacks ───────────────────────────────────────

    def _handle_open(self, request: Trigger.Request,
                     response: Trigger.Response) -> Trigger.Response:
        return self._dispatch('open', _GRIPPER_OPEN_POSITION, response)

    def _handle_close(self, request: Trigger.Request,
                      response: Trigger.Response) -> Trigger.Response:
        return self._dispatch('close', _GRIPPER_CLOSE_POSITION, response)

    # ── Core dispatch ───────────────────────────────────────────

    def _dispatch(self, command: str, position: float,
                  response: Trigger.Response) -> Trigger.Response:
        """의도 토픽 발행 + 액션 goal 전송. 액션 서버 미준비는 message 로 알린다."""
        now = self.get_clock().now().to_msg()

        cmd_msg = GripperCommand()
        cmd_msg.header.stamp = now
        cmd_msg.command = command
        self._cmd_pub.publish(cmd_msg)

        if not self._action.server_is_ready():
            msg = f"gripper action server '{_GRIPPER_ACTION_NAME}' not ready"
            self.get_logger().warning(f"[gripper] {command}: {msg}")
            response.success = False
            response.message = msg
            return response

        goal = GripperCommandAction.Goal()
        goal.command.position = position
        goal.command.max_effort = _GRIPPER_MAX_EFFORT
        self._action.send_goal_async(goal, feedback_callback=self._on_feedback)
        self.get_logger().info(f"[gripper] {command} (position={position:.3f} m)")
        response.success = True
        response.message = f"{command} dispatched"
        return response

    # ── Action feedback ─────────────────────────────────────────

    def _on_feedback(self, feedback_msg) -> None:
        """액션 feedback 을 `/teleop/gripper_states` 로 재발행한다."""
        fb = feedback_msg.feedback
        state = GripperState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.position = float(getattr(fb, 'position', 0.0))
        state.effort = float(getattr(fb, 'effort', 0.0))
        state.stalled = bool(getattr(fb, 'stalled', False))
        state.reached_goal = bool(getattr(fb, 'reached_goal', False))
        self._state_pub.publish(state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperControlNode()
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


__all__ = ['GripperControlNode', 'main']
