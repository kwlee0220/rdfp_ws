"""`control_msgs/GripperCommand` **액션**으로 실행하는 `GripperNode` 구현.

    /gripper_cmds (rdfp_msgs/GripperCommand)   심볼 'open'/'close'/'grasp'
        -> [본 노드] targets 로 풀어 액션 goal 전송
        -> /panda_hand_controller/gripper_cmd (control_msgs/GripperCommand action)

    /joint_states + 액션 result
        -> [본 노드]
        -> /gripper_states (rdfp_msgs/GripperState)  주기 발행

**백엔드가 아니라 실행 수단으로 이름 붙였다.** 액션 서버만 있으면 되므로 mock 과
Isaac 이 같은 노드를 쓴다 — Isaac 은 ros2_control 이 없지만 `isaac_gripper_bridge` 가
같은 이름의 액션 서버를 연다. 액션 서버가 없는 펑션베이는 별도 구현이 필요하다.
(예전 이름 `MockGripperNode` 는 Isaac 이 쓰는 순간 거짓이 됐다.)

**명령을 받는 노드가 상태도 낸다.** `GripperState.goal` 때문이다 — 명령을 아는 쪽이
상태를 내면 목표가 자연히 손에 있고 경합도 없다. 나누면 상태 발행자가 명령 토픽을 따로
구독해야 하고, 명령 직후·발행 직전 구간에서 옛 목표가 실리는 창이 생긴다.

계약 전문은 `docs/moveit/GripperNode_Design.md` §3 에 있다. 이 노드가 지키는 것:

  * 모르는 심볼은 **거부한다** — 조용히 무시하면 팔은 움직이는데 그리퍼만 안 움직여
    원인이 보이지 않는다.
  * `width` 는 **개구 폭(m)** 이다. 구할 수 없으면 **NaN** (0 이 아니다 — 0 은 "닫혀
    있다"는 거짓말이 된다).
  * `at_goal` 은 **의도의 달성 여부**이지 위치 도달이 아니다. 판정식은 §2.3.
  * 명령이 없어도 주기 발행한다 — 연속 상태 채널이다.

`stalled` 은 아직 판정하지 않는다
--------------------------------

항상 ``False`` 이고, 따라서 §2.3 의 판정식에 의해 **`grasp` 의 `at_goal` 도 항상
``False``** 다. 이유는 백엔드마다 다르다.

  * **mock 은 원리적으로 불가능하다.** ``panda_hand.ros2_control.xacro`` 가 손가락에
    선언하는 state interface 가 ``position``/``velocity`` 뿐이라 ``/joint_states`` 에
    effort 가 실리지 않는다. 게다가 planning scene 물체는 물리를 갖지 않아 파지에
    실패해도 pose 가 그대로다 — **애초에 성패를 관측할 수 없는 백엔드**다.
  * **Isaac 은 가능한데 미구현이다.** effort 를 실을 수 있으므로 임계값만 정하면 된다.

계약대로 ``False`` 를 돌려준다 — **"물지 않았다"가 아니라 "모른다"는 뜻이다.** 여기서
``True`` 를 내면 "성공했다"는 거짓말이 데이터에 쌓인다.

구현할 때는 **파라미터 하나(임계 effort)를 더하는 쪽이 맞다.** 별도 서브클래스로
가르면 액션 경로가 같은 코드가 두 벌이 된다 — 다른 것은 임계값뿐이다.

파라미터
--------

=========================== ========================= ===============================
이름                         기본                      설명
=========================== ========================= ===============================
targets.<goal>              open/close/grasp 아래 참조  심볼 → [position(m), max_effort(N)]
finger_joint                panda_finger_joint1        폭을 계산할 손가락 관절
width_scale                 2.0                        관절값 → 개구 폭 배수 (대칭 평행 조)
width_tolerance             0.005                      `at_goal` 판정의 폭 허용오차 (m)
publish_rate                10.0                       `gripper_states` 발행 Hz
=========================== ========================= ===============================

``targets`` 의 숫자가 이 노드에 있는 이유는 **그리퍼에 종속**이기 때문이다. 명령에는
의도만 실려 오고, 그 로봇의 수치는 기구를 아는 여기에 남는다.
"""

from __future__ import annotations

from typing import Optional

import math

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand as GripperCommandAction
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState

from rdfp_msgs.msg import GripperCommand, GripperState

_GRIPPER_ACTION_NAME = '/panda_hand_controller/gripper_cmd'
# **루트 상대 경로다 (`~/` 가 아니다).** 노드 이름이 바뀌어도 토픽이 이동하지 않게
# 하기 위해서다 — 예전 `gripper_control` / `gripper_state_publisher` 두 노드가
# 각자의 네임스페이스를 쓰는 바람에 명령은 `/gripper_control/gripper_cmds`,
# 상태는 `/gripper_states` 로 비대칭이었다. 계약은 노드 이름과 무관해야 한다.
_CMD_TOPIC = 'gripper_cmds'
_STATE_TOPIC = 'gripper_states'
_JOINT_STATES_TOPIC = '/joint_states'

# 심볼 → (position[m], max_effort[N]).
#
# **`position` 은 액션 goal 이 받는 관절값이며 개구 폭이 아니다.** 폭과 견주려면
# `width_scale` 을 곱해야 한다 (`_target_width`) — 두 단위를 그대로 비교하면 `open`
# 의 `at_goal` 이 영영 서지 않는다. 명령 경로는 관절값을, 관측 경로는 폭을 쓴다.
#
#   open   최대 개구
#   close  빈손으로 닫는다 — 힘을 주지 않으므로 파지 용도가 아니다
#   grasp  close 와 자세는 같고 **힘이 다르다**
_DEFAULT_TARGETS = {
    'open': (0.035, 10.0),
    'close': (0.0, 10.0),
    'grasp': (0.0, 30.0),
}


class GripperActionNode(Node):
    """`GripperNode` 계약의 액션 기반 구현."""

    def __init__(self) -> None:
        super().__init__('gripper')

        self._targets: dict[str, tuple[float, float]] = {}
        for goal, default in _DEFAULT_TARGETS.items():
            self.declare_parameter(f'targets.{goal}', list(default))
            values = list(self.get_parameter(f'targets.{goal}').value or default)
            if len(values) != 2:
                raise ValueError(
                    f"'targets.{goal}' must be [position, max_effort], got {values}")
            self._targets[goal] = (float(values[0]), float(values[1]))

        self._finger_joint = str(
            self.declare_parameter('finger_joint', 'panda_finger_joint1').value)
        self._width_scale = float(self.declare_parameter('width_scale', 2.0).value)
        self._width_tolerance = float(self.declare_parameter('width_tolerance', 0.005).value)
        publish_rate = float(self.declare_parameter('publish_rate', 10.0).value)
        if publish_rate <= 0.0:
            raise ValueError(f"'publish_rate' must be > 0, got {publish_rate}")

        # 상태. `goal` 은 명령 수신 시점에 갱신되며, 명령 이전에는 ''.
        self._goal: str = ''
        self._width: float = math.nan

        self._state_pub = self.create_publisher(GripperState, _STATE_TOPIC, 10)
        self._action = ActionClient(self, GripperCommandAction, _GRIPPER_ACTION_NAME)
        self.create_subscription(GripperCommand, _CMD_TOPIC, self._on_cmd, 10)
        self.create_subscription(JointState, _JOINT_STATES_TOPIC, self._on_joint_state, 10)
        self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            f'GripperActionNode started: {_CMD_TOPIC} -> {_GRIPPER_ACTION_NAME}, '
            f'{_STATE_TOPIC} @ {publish_rate} Hz')
        self.get_logger().info(
            f'  width: {self._finger_joint} x{self._width_scale} '
            f'(tolerance {self._width_tolerance} m)')
        self.get_logger().info(
            f'  targets: { {k: list(v) for k, v in self._targets.items()} }')
        self.get_logger().warning(
            '  stalled is not judged yet — always False, therefore '
            "'grasp' never reports at_goal=True. On mock this is unfixable "
            '(no effort state interface); on Isaac it needs a measured threshold.')

    # ── 명령 ────────────────────────────────────────────────────

    def _on_cmd(self, msg: GripperCommand) -> None:
        """심볼을 액션 goal 로 풀어 전송한다.

        **모르는 심볼은 거부한다.** 조용히 무시하면 팔은 움직이는데 그리퍼만 안
        움직이는 상태가 되어 원인이 보이지 않는다.

        `goal` 은 **거부하지 않은 경우에만** 갱신한다 — 실행되지 않은 명령을 상태에
        실으면 `at_goal` 이 있지도 않은 목표를 판정하게 된다.
        """
        goal = str(msg.goal or '').strip()
        target = self._targets.get(goal)
        if target is None:
            self.get_logger().error(
                f'[gripper] unknown goal {goal!r}; known: {sorted(self._targets)}')
            return

        if not self._action.server_is_ready():
            self.get_logger().warning(
                f"[gripper] {goal}: action server '{_GRIPPER_ACTION_NAME}' not ready")
            return

        position, max_effort = target
        action_goal = GripperCommandAction.Goal()
        action_goal.command.position = position
        action_goal.command.max_effort = max_effort
        send_future = self._action.send_goal_async(action_goal)
        send_future.add_done_callback(lambda fut: self._on_goal_response(fut, goal))

        self._goal = goal
        self.get_logger().info(
            f'[gripper] {goal} dispatched (position={position:.4f} '
            f'max_effort={max_effort:.1f})')

    def _on_goal_response(self, future: Future, goal: str) -> None:
        """goal 수락 여부를 확인하고 결과 대기 단계로 진입한다."""
        try:
            handle = future.result()
        except Exception as exc:   # noqa: BLE001
            self.get_logger().error(f'[gripper] {goal}: goal send failed: {exc}')
            return
        if handle is None or not handle.accepted:
            self.get_logger().warning(f'[gripper] {goal}: goal rejected by action server')
            return
        handle.get_result_async().add_done_callback(lambda fut: self._on_result(fut, goal))

    def _on_result(self, future: Future, goal: str) -> None:
        """결과를 로그로만 남긴다.

        **상태는 액션 결과가 아니라 `/joint_states` 에서 만든다.** 결과는 명령당 1건
        이라 "지금 어떤 상태인가"에 답할 수 없고, `at_goal` 은 매 주기 재평가여야
        하기 때문이다 (설계 §2.2).
        """
        try:
            response = future.result()
        except Exception as exc:   # noqa: BLE001
            self.get_logger().error(f'[gripper] {goal}: result failed: {exc}')
            return
        status = getattr(response, 'status', GoalStatus.STATUS_UNKNOWN)
        # CANCELED 는 후속 명령에 의한 선점이라 실패가 아니다.
        level = (self.get_logger().info if status == GoalStatus.STATUS_SUCCEEDED
                 else self.get_logger().warning)
        level(f'[gripper] {goal}: action finished (status={status})')

    # ── 관측 ────────────────────────────────────────────────────

    def _on_joint_state(self, msg: JointState) -> None:
        """손가락 관절값을 개구 폭으로 바꿔 보관한다.

        **없으면 NaN 으로 둔다.** 0 을 넣으면 "닫혀 있다"는 거짓말이 된다. 펑션베이가
        `/joint_states` 에 고정값을 주입하는 것과 같은 종류의 오염을 만들지 않는다.
        """
        try:
            idx = list(msg.name).index(self._finger_joint)
        except ValueError:
            return
        if idx >= len(msg.position):
            return
        self._width = float(msg.position[idx]) * self._width_scale

    def _target_width(self, goal: str) -> float:
        """심볼의 목표를 **개구 폭**으로 돌려준다. 모르는 심볼이면 NaN.

        `targets` 가 갖는 것은 액션 goal 에 실을 **관절값**이라 `width` 와 직접 비교
        하면 안 된다 — Panda 는 폭이 관절값의 2배라 `open` 이 0.035 vs 0.070 으로
        어긋나 판정이 영영 서지 않는다.
        """
        target = self._targets.get(goal)
        return math.nan if target is None else target[0] * self._width_scale

    def _at_goal(self) -> bool:
        """§2.3 판정식. **의도의 달성 여부**이지 위치 도달이 아니다.

        `grasp` 는 `stalled` 에 딸리는데 그것을 아직 판정하지 않아 항상 ``False`` 다.
        위치 기준이었다면 목표 폭에 도달해 ``True`` 가 됐을 텐데, 파지를 관측하지
        못하는 스택에서 그것은 거짓말이다.
        """
        if not self._goal or math.isnan(self._width):
            return False
        if self._goal == 'grasp':
            return self._stalled()
        target_width = self._target_width(self._goal)
        if math.isnan(target_width):
            return False
        return abs(self._width - target_width) <= self._width_tolerance and not self._stalled()

    def _stalled(self) -> bool:
        """아직 판정하지 않는다 — 계약대로 ``False`` 를 돌려준다.

        **"물지 않았다"는 뜻이 아니라 "모른다"는 뜻이다.** mock 에서는 원리적으로
        불가능하고(effort state interface 자체가 없다), Isaac 에서는 임계값 실측이
        선행 작업이다. 모듈 docstring 참조.
        """
        return False

    def _on_timer(self) -> None:
        """명령이 없어도 주기 발행한다 — 연속 상태 채널이다."""
        msg = GripperState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.goal = self._goal
        msg.width = self._width
        msg.stalled = self._stalled()
        msg.at_goal = self._at_goal()
        self._state_pub.publish(msg)


def main(args: Optional[list] = None) -> int:
    rclpy.init(args=args)
    node = GripperActionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
