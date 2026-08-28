"""`rdfp_msgs/GripperCommand` 구독 → gripper 액션 실행 노드.

`~/gripper_cmds` 로 들어온 명령(`position` / `max_effort`)을
`/panda_hand_controller/gripper_cmd` 액션 goal 로 전달하고, 액션 result 를
`~/gripper_action_states` 로 재발행한다. 기본 노드명 기준 토픽은
`/gripper_control/gripper_cmds` · `/gripper_control/gripper_action_states` 다.

**액션을 부르는 주체는 이 노드 하나다.** 호출자(키보드 teleop, robot twin, 데이터셋
재생)는 모두 명령 토픽에 publish 만 한다. 이유는 둘이다.

1. **기록 가능성** — 액션 goal 전송은 서비스라 rosbag2(Humble)가 기록하지 못한다.
   명령을 토픽으로 한 번 흘려야 학습 데이터의 action 채널이 남는다.
2. **경로 일관성** — 액션 호출자가 여럿이면 result(`~/gripper_action_states`)를 누가 발행할지
   갈리고, 생성기에 따라 결과 채널이 있다가 없어진다.

`GripperActionController` 는 feedback 을 발행하지 않으므로 `~/gripper_action_states` 는
**명령당 1건(result 기반)** 이다. 즉 연속 상태가 아니라 이벤트다 — 현재 그리퍼 폭은
`/joint_states` 의 finger joint 에서 읽는다.

성공/취소/abort 여부는 `~/gripper_action_states` 의 ``status`` 필드로 판정한다
(`GripperActionState.STATUS_*`, `action_msgs/msg/GoalStatus` 와 동일한 값).

자세한 내용은 `docs/moveit/gripper_action_server_notes.md` 를 참고한다.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand as GripperCommandAction
from rdfp_msgs.msg import GripperActionState, GripperCommand


_GRIPPER_ACTION_NAME = '/panda_hand_controller/gripper_cmd'
_GRIPPER_CMD_TOPIC = '~/gripper_cmds'
_GRIPPER_ACTION_STATE_TOPIC = '~/gripper_action_states'

# 로그 표시용 GoalStatus 이름 매핑.
_GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


class GripperControlNode(Node):
    """명령 토픽을 gripper 액션으로 중계하는 단일 책임 노드."""

    def __init__(self) -> None:
        super().__init__('gripper_control')

        # --- Publisher ---
        self._state_pub = self.create_publisher(
            GripperActionState, _GRIPPER_ACTION_STATE_TOPIC, 10)

        # --- Action client ---
        # 서버가 당장 없을 수 있으므로 send 시점에 ready 여부만 확인한다.
        self._action = ActionClient(self, GripperCommandAction, _GRIPPER_ACTION_NAME)

        # --- Subscription ---
        self._cmd_sub = self.create_subscription(GripperCommand, _GRIPPER_CMD_TOPIC,
                                                 self._on_cmd, 10)

        self.get_logger().info(
            'GripperControlNode started '
            f'(subscribe: {_GRIPPER_CMD_TOPIC}, action: {_GRIPPER_ACTION_NAME})'
        )

    # ── Subscription callback ───────────────────────────────────

    def _on_cmd(self, msg: GripperCommand) -> None:
        """명령을 액션 goal 로 변환해 전송한다.

        ``label`` 은 로그와 데이터셋 가독성을 위한 값이며 **제어에 쓰지 않는다.**
        위치·힘 검증은 하지 않는다 — 관절 한계 판정은 컨트롤러의 몫이고, 트윈처럼
        상위 계층이 이미 검증한 값을 다시 막으면 실패 지점만 흐려진다.
        """
        label = _describe(msg)

        if not self._action.server_is_ready():
            self.get_logger().warning(
                f"[gripper] {label}: action server '{_GRIPPER_ACTION_NAME}' not ready"
            )
            return

        goal = GripperCommandAction.Goal()
        goal.command.position = float(msg.position)
        goal.command.max_effort = float(msg.max_effort)
        send_future = self._action.send_goal_async(goal, feedback_callback=self._on_feedback)
        # goal 수락 → 결과 수신 순으로 콜백을 연결한다. 구독 콜백은 여기서 즉시
        # 반환하므로 executor 를 막지 않는다.
        send_future.add_done_callback(lambda fut: self._on_goal_response(fut, label))

        self.get_logger().info(f'[gripper] {label} dispatched')

    # ── Action feedback ─────────────────────────────────────────

    def _on_feedback(self, feedback_msg) -> None:
        """액션 feedback 을 `~/gripper_action_states` 로 재발행한다.

        현재 `GripperActionController` 가 feedback 을 발행하지 않으므로 본
        콜백은 호출되지 않는다. feedback 을 지원하는 컨트롤러로 교체하면
        수정 없이 동작하도록 `getattr` 기본값으로 방어해 둔다.
        """
        # feedback 은 goal 이 실행 중일 때만 오므로 EXECUTING 으로 기록한다.
        self._publish_state(feedback_msg.feedback, GoalStatus.STATUS_EXECUTING)

    # ── Action result ───────────────────────────────────────────

    def _on_goal_response(self, future: Future, label: str) -> None:
        """goal 수락 여부를 확인하고 결과 대기 단계로 진입한다."""
        try:
            goal_handle = future.result()
        except Exception as exc:   # noqa: BLE001
            self.get_logger().error(f'[gripper] {label}: goal send failed: {exc}')
            return

        if goal_handle is None:
            self.get_logger().error(f'[gripper] {label}: no goal handle returned')
            return
        if not goal_handle.accepted:
            self.get_logger().warning(f'[gripper] {label}: goal rejected by action server')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_result(fut, label))

    def _on_result(self, future: Future, label: str) -> None:
        """액션 결과를 `~/gripper_action_states` 로 발행한다.

        `GripperCommand` 의 Result 는 Feedback 과 필드 구성이 동일하므로 같은
        `GripperActionState` 메시지로 매핑되며, Result 래퍼에만 있는 ``status``
        (SUCCEEDED / CANCELED / ABORTED) 는 `GripperActionState.status` 에 담는다.
        """
        try:
            response = future.result()
        except Exception as exc:   # noqa: BLE001
            self.get_logger().error(f'[gripper] {label}: result unavailable: {exc}')
            return

        if response is None:
            self.get_logger().error(f'[gripper] {label}: empty result response')
            return

        status = response.status
        status_name = _GOAL_STATUS_NAMES.get(status, str(status))
        result = response.result
        self._publish_state(result, status)

        message = (
            f'[gripper] {label} result: status={status_name} '
            f'position={result.position:.4f} effort={result.effort:.2f} '
            f'stalled={result.stalled} reached_goal={result.reached_goal}'
        )
        # rclpy 로거는 호출 지점(call site) 단위로 severity 를 캐시하며, 같은
        # 지점에서 severity 가 바뀌면 ValueError 를 던진다. 따라서 분기별로
        # 호출 지점을 분리해야 한다 (삼항 연산자로 묶으면 노드가 죽는다).
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(message)
        else:
            self.get_logger().warning(message)

    # ── 공통 ────────────────────────────────────────────────────

    def _publish_state(self, source, status: int) -> None:
        """Feedback 또는 Result 를 `GripperActionState` 로 변환해 발행한다.

        두 메시지는 필드 구성이 같으므로 동일한 변환을 사용한다. 다만 goal
        상태는 메시지에 들어 있지 않으므로 호출자가 `status` 로 넘긴다
        (`action_msgs/msg/GoalStatus` 의 STATUS_* 값). 필드가 없는 구현을
        만나도 죽지 않도록 `getattr` 기본값으로 방어한다.
        """
        state = GripperActionState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.position = float(getattr(source, 'position', 0.0))
        state.effort = float(getattr(source, 'effort', 0.0))
        state.stalled = bool(getattr(source, 'stalled', False))
        state.reached_goal = bool(getattr(source, 'reached_goal', False))
        state.status = int(status)
        self._state_pub.publish(state)


def _describe(msg: GripperCommand, label: Optional[str] = None) -> str:
    """로그용 명령 설명. ``label`` 이 있으면 앞에 붙인다."""
    name = (label if label is not None else msg.label).strip()
    body = f'position={msg.position:.4f} max_effort={msg.max_effort:.1f}'
    return f'{name} ({body})' if name else body


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
