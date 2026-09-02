"""관절 공간으로 직접 지령하는 `GripperNode` 구현 — 액션 서버가 없는 스택용.

    gripper_cmds (rdfp_msgs/GripperCommand)   심볼 'open'/'close'/'grasp'
        -> [본 노드] targets 의 스칼라 s 를 axis_signs 로 펼쳐 6축 목표각 생성
        -> joint_command (std_msgs/Float64MultiArray)

    joint_report (sensor_msgs/JointState, position/velocity/effort)
        -> [본 노드] 잔차 + 토크로 판정
        -> gripper_states (rdfp_msgs/GripperState)  주기 발행

`GripperActionNode` 와 **계약은 같고 실행 수단이 다르다.** 저쪽은
`control_msgs/GripperCommand` 액션 서버를 부르고, 이쪽은 관절 목표각을 토픽으로
직접 쓴다. 펑션베이처럼 ros2_control 이 없어 액션 서버가 존재하지 않는 스택이
대상이다. **토픽은 전부 상대 경로**이므로 백엔드 결합은 launch 의 remap 이 만든다.

왜 스칼라 하나로 지령하나
------------------------

Robotiq 2F-85 는 6축이 링키지로 묶여 있어 **한 축만 바꾸면 어긋난 자세**가 된다.
시뮬레이터가 mimic 을 강제하지 않으므로(실측) 6축을 일관되게 채워 보내는 것이 이
노드의 책임이다. 어긋난 지령은 에러 없이 TF·파지 판정·데이터셋에 스며든다.

``s · axis_signs`` 로 채우면 전 구간에서 추종 오차 0.0000 이다 — 한때 "인덱스 1·4 는
25% 오차" 로 기록됐던 것은 축 특성이 아니라 한 축만 바꾼 모순된 지령에 링키지가
저항한 결과였다.

`at_goal` 을 관절 잔차로 판정한다
--------------------------------

계약이 요구하는 것은 "목표 **자세** 도달"이고 무엇으로 재는지는 구현이 정한다
(`GripperNode_Design.md` §2.3). 이 노드는 **관절 잔차**를 쓴다 — 지령과 보고가 같은
관절 공간이라 실측 잔차가 0.0000 이기 때문이다. 개구 폭으로 환산하면 2F-85 링키지
근사 오차만 더해진다.

`width` 는 NaN 이다
------------------

개구 폭은 6축 각도에서 바로 나오지 않는다. 2F-85 링키지 기하가 필요하고 **선형이
아니며** 제원이 아직 없다. 근사값을 실으면 "폭을 안다"는 신호가 데이터셋에 남아,
나중에 제원을 받아 고칠 때 과거 에피소드의 의미가 조용히 바뀐다. 계약이 NaN 을
"모른다"로 정의해 두었으므로 그대로 둔다.

**`at_goal` 은 영향받지 않는다** — 잔차로 판정하기 때문이다. 잃는 것은 `grasp` 성공과
헛닫힘을 폭으로 교차 검증하는 것뿐이며, 판정 자체는 `stalled` 이 가른다.

파라미터
--------

=========================== ========================= ===============================
이름                         기본                      설명
=========================== ========================= ===============================
axis_signs                  [1,1,-1,-1,-1,1]           스칼라 목표를 축별로 펼치는 부호
targets.<goal>              open/close/grasp 아래 참조  심볼 → 구동 스칼라 s [rad]
position_tolerance          0.005                      자세 도달 판정의 잔차 한계 [rad]
stall_effort                1.0                        `stalled` 판정 토크 임계 [N·m]
stall_velocity              0.001                      `stalled` 판정 속도 상한 [rad/s]
publish_rate                10.0                       `gripper_states` 발행 Hz
=========================== ========================= ===============================

임계값 근거 (펑션베이 실측, `docs/simulation/functionbay_backend_design.md` §6.1):
빈손 폐쇄 토크 0.004 N·m / 자유 이동 중 최대 0.845 / **파지 17.19**. `1.0` 은 양쪽에서
17배 이상 여유가 있고 네 상황에서 오분류 0건이었다. 계단 응답 99% 도달이 1051 ms 이므로
소비자 쪽 타임아웃은 1.5 초면 충분하다.
"""

from __future__ import annotations

from typing import Optional

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from rdfp_msgs.msg import GripperCommand, GripperState

# 계약 채널 — 루트 상대 (`~/` 가 아니다). 노드 이름이 바뀌어도 이동하지 않는다.
_CMD_TOPIC = 'gripper_cmds'
_STATE_TOPIC = 'gripper_states'
# 백엔드 채널 — launch 의 remap 이 시뮬레이터 토픽으로 연결한다.
_JOINT_COMMAND_TOPIC = 'joint_command'
_JOINT_REPORT_TOPIC = 'joint_report'

# 2F-85 의 축별 부호. 인덱스 2·3·4 는 음수 영역이 정상 범위다.
_DEFAULT_AXIS_SIGNS = [1.0, 1.0, -1.0, -1.0, -1.0, 1.0]

# 심볼 → 구동 스칼라 s [rad]. `finger_joint` 실측 범위가 [0, 0.725] 이다.
#
#   open   완전 개방
#   close  빈손으로 닫는다 — 힘을 주지 않으므로 파지 용도가 아니다
#   grasp  close 와 목표 자세가 같고 **물체에 막혀 멈추는 것이 성공**이다
_DEFAULT_TARGETS = {'open': 0.0, 'close': 0.725, 'grasp': 0.725}


class GripperJointNode(Node):
    """`GripperNode` 계약의 관절 지령 기반 구현."""

    def __init__(self) -> None:
        super().__init__('gripper')

        signs = list(self.declare_parameter('axis_signs', _DEFAULT_AXIS_SIGNS).value or [])
        if not signs:
            raise ValueError("'axis_signs' must not be empty")
        self._axis_signs = [float(v) for v in signs]

        self._targets: dict[str, float] = {}
        for goal, default in _DEFAULT_TARGETS.items():
            value = float(self.declare_parameter(f'targets.{goal}', default).value)
            if not math.isfinite(value):
                raise ValueError(f"'targets.{goal}' must be finite, got {value}")
            self._targets[goal] = value

        self._tolerance = float(self.declare_parameter('position_tolerance', 0.005).value)
        self._stall_effort = float(self.declare_parameter('stall_effort', 1.0).value)
        self._stall_velocity = float(self.declare_parameter('stall_velocity', 0.001).value)
        publish_rate = float(self.declare_parameter('publish_rate', 10.0).value)
        if publish_rate <= 0.0:
            raise ValueError(f"'publish_rate' must be > 0, got {publish_rate}")

        # 상태. `_target` 은 마지막으로 **보낸** 목표 배열이며, 명령 이전에는 None.
        self._goal: str = ''
        self._target: Optional[list[float]] = None
        self._positions: list[float] = []
        self._velocities: list[float] = []
        self._efforts: list[float] = []

        self._state_pub = self.create_publisher(GripperState, _STATE_TOPIC, 10)
        self._joint_pub = self.create_publisher(Float64MultiArray, _JOINT_COMMAND_TOPIC, 10)
        self.create_subscription(GripperCommand, _CMD_TOPIC, self._on_cmd, 10)
        self.create_subscription(JointState, _JOINT_REPORT_TOPIC, self._on_report, 10)
        self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            f'GripperJointNode started: {_CMD_TOPIC} -> {_JOINT_COMMAND_TOPIC}, '
            f'{_JOINT_REPORT_TOPIC} -> {_STATE_TOPIC} @ {publish_rate} Hz')
        self.get_logger().info(
            f'  axes: {len(self._axis_signs)}, signs: {self._axis_signs}')
        self.get_logger().info(
            f'  targets: {self._targets}, tolerance: {self._tolerance} rad')
        self.get_logger().info(
            f'  stalled: |effort| > {self._stall_effort} N·m '
            f'AND |velocity| < {self._stall_velocity} rad/s')
        self.get_logger().warning(
            '  width is always NaN — the 2F-85 gap needs linkage geometry we do not '
            'have. at_goal is unaffected (judged on joint residual).')

    # ── 명령 ────────────────────────────────────────────────────

    def _command_array(self, goal: str) -> Optional[list[float]]:
        """심볼을 축별 목표각으로 펼친다. 모르는 심볼이면 ``None``.

        **6축을 한꺼번에 채운다.** 한 축만 보내면 나머지는 그대로 남아 링키지가 어긋난
        자세가 되고, 시뮬레이터는 그것을 거부하지 않는다.
        """
        scalar = self._targets.get(goal)
        if scalar is None:
            return None
        return [scalar * sign for sign in self._axis_signs]

    def _on_cmd(self, msg: GripperCommand) -> None:
        """심볼을 관절 목표각으로 풀어 발행한다.

        **모르는 심볼은 거부한다.** 조용히 무시하면 팔은 움직이는데 그리퍼만 안
        움직이는 상태가 되어 원인이 보이지 않는다. 거부한 명령은 `goal` 에 싣지
        않는다 — 실행되지 않은 목표를 상태에 실으면 `at_goal` 이 있지도 않은 목표를
        판정하게 된다.

        구독자가 없어도 **보낸다.** 액션 구현이 서버 미준비 시 포기하는 것과 다른데,
        토픽 발행은 실패하는 연산이 아니고 시뮬레이터가 곧 붙을 수 있기 때문이다.
        명령이 실제로 유실됐다면 잔차가 줄지 않아 `at_goal` 이 false 로 남는다 —
        거짓말이 아니라 정직한 결과다.
        """
        goal = str(msg.goal or '').strip()
        target = self._command_array(goal)
        if target is None:
            self.get_logger().error(
                f'[gripper] unknown goal {goal!r}; known: {sorted(self._targets)}')
            return

        if self._joint_pub.get_subscription_count() == 0:
            self.get_logger().warning(
                f"[gripper] {goal}: nobody subscribes '{_JOINT_COMMAND_TOPIC}' — "
                'publishing anyway; the simulator may not be connected')

        command = Float64MultiArray()
        command.data = target
        self._joint_pub.publish(command)

        self._goal = goal
        self._target = target
        self.get_logger().info(
            f'[gripper] {goal} sent (s={self._targets[goal]:.4f}, '
            f'{len(target)} axes)')

    # ── 관측 ────────────────────────────────────────────────────

    def _on_report(self, msg: JointState) -> None:
        """관절 보고를 보관한다.

        **길이가 축 수와 다르면 버린다.** `Float64MultiArray` 지령이 인덱스 계약이므로
        보고도 같은 순서여야 잔차가 의미를 갖는다 (슬롯 k → 출력 k 는 실측으로
        확인됐다). 길이가 어긋난 채 잘라 쓰면 엉뚱한 축을 비교하게 된다.
        """
        expected = len(self._axis_signs)
        if len(msg.position) != expected:
            self.get_logger().error(
                f"[gripper] '{_JOINT_REPORT_TOPIC}' has {len(msg.position)} positions, "
                f'expected {expected} — ignoring', throttle_duration_sec=5.0)
            return
        self._positions = list(msg.position)
        self._velocities = list(msg.velocity)
        self._efforts = list(msg.effort)

    def _stalled(self) -> bool:
        """힘을 내는데 안 움직인다.

        **신호가 둘이라 서로 검증된다.** 토크만 보면 가속 구간(최대 0.845 N·m)을
        파지로 오독할 수 있고, 속도만 보면 도달해 멈춘 것과 막혀 멈춘 것이 같아진다.
        """
        if not self._efforts or not self._velocities:
            return False
        return (max(abs(v) for v in self._efforts) > self._stall_effort
                and max(abs(v) for v in self._velocities) < self._stall_velocity)

    def _residual(self) -> float:
        """지령 대비 최대 관절 잔차. 지령이나 보고가 없으면 NaN."""
        if self._target is None or len(self._positions) != len(self._target):
            return math.nan
        return max(abs(p - t) for p, t in zip(self._positions, self._target))

    def _at_goal(self) -> bool:
        """§2.3 판정식. **의도의 달성 여부**이지 위치 도달이 아니다.

        `grasp` 만 자세 조건이 없다 — 목표 자세까지 닫히면 오히려 헛닫힘이라
        `stalled` 하나가 판정 전부다.
        """
        if not self._goal:
            return False
        if self._goal == 'grasp':
            return self._stalled()
        residual = self._residual()
        if math.isnan(residual):
            return False
        return residual <= self._tolerance and not self._stalled()

    def _on_timer(self) -> None:
        """명령이 없어도 주기 발행한다 — 연속 상태 채널이다."""
        msg = GripperState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.goal = self._goal
        # 개구 폭을 구할 수단이 없다. 0 을 넣으면 "닫혀 있다"는 거짓말이 된다.
        msg.width = math.nan
        msg.stalled = self._stalled()
        msg.at_goal = self._at_goal()
        self._state_pub.publish(msg)


def main(args: Optional[list] = None) -> int:
    rclpy.init(args=args)
    node = GripperJointNode()
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
