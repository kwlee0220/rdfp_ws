"""Robotiq 2F 계열용 `GripperNode` 구현 — 관절 목표각을 토픽으로 직접 쓴다.

    gripper_cmds (rdfp_msgs/GripperCommand)   심볼 'open'/'close'/'grasp'
        -> [본 노드] targets 의 스칼라 s 를 axis_signs 로 펼쳐 6축 목표각 생성
        -> joint_command (std_msgs/Float64MultiArray)

    joint_report (sensor_msgs/JointState, position/velocity/effort)
        -> [본 노드] 잔차 + 토크로 판정
        -> gripper_states (rdfp_msgs/GripperState)  주기 발행

`GripperActionNode` 와 **계약은 같고 실행 수단이 다르다.** 저쪽은
`control_msgs/GripperCommand` 액션 서버를 부르므로 그 뒤의 기구를 알 필요가 없다.
여기는 그런 인터페이스가 없어 **기구를 직접 구동**하며, 그래서 이름도 인터페이스가
아니라 기구 계열을 가리킨다. 대상은 펑션베이처럼 ros2_control 이 없어 액션 서버가
존재하지 않는 스택이다. **토픽은 전부 상대 경로**이므로 백엔드 결합은 launch 의
remap 이 만든다.

**계열 안에서는 파라미터로 흡수된다.** 기본값은 2F-85 실측치인데, 2F-140 은 링키지
구조(6축·부호 벡터)가 같고 스트로크만 다르므로 ``targets`` 만 바꾸면 된다.

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
(`docs/gripper/GripperNode_Design.md` §2.3). 이 노드는 **관절 잔차**를 쓴다 — 지령과 보고가 같은
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
import time

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


class Robotiq2FGripperNode(Node):
    """`GripperNode` 계약의 관절 지령 기반 구현."""

    def __init__(self) -> None:
        super().__init__('gripper')

        # **명령 규약은 씬마다 다르다** — `gripper_profile_file` 이 주어지면 축 수·부호·
        # 단위·packing 을 그 파일이 정한다 (`robot_control.gripper.profile`). 안 주면
        # 옛 동작(6축·라디안·위치 나열)을 그대로 쓴다 — mock·Isaac 은 이 경로다.
        profile_file = str(self.declare_parameter('gripper_profile_file', '').value or '')
        self._profile = None
        if profile_file:
            from robot_control.gripper.profile import load_gripper_profile_file
            self._profile = load_gripper_profile_file(profile_file)

        if self._profile is not None:
            self._axis_signs = list(self._profile.axis_signs)
        else:
            signs = list(self.declare_parameter('axis_signs', _DEFAULT_AXIS_SIGNS).value or [])
            if not signs:
                raise ValueError("'axis_signs' must not be empty")
            self._axis_signs = [float(v) for v in signs]

        self._targets: dict[str, float] = {}
        for goal, default in _DEFAULT_TARGETS.items():
            if self._profile is not None and goal in self._profile.targets:
                default = self._profile.targets[goal]
            value = float(self.declare_parameter(f'targets.{goal}', default).value)
            if not math.isfinite(value):
                raise ValueError(f"'targets.{goal}' must be finite, got {value}")
            self._targets[goal] = value

        def _profiled(name: str, default: float) -> float:
            if self._profile is not None:
                default = float(getattr(self._profile, name))
            return float(self.declare_parameter(name, default).value)

        self._tolerance = _profiled('position_tolerance', 0.005)
        self._stall_effort = _profiled('stall_effort', 1.0)
        self._stall_velocity = _profiled('stall_velocity', 0.001)
        # **개폐를 몇 초에 걸쳐 나눠 보낼 것인가.** 0(기본)이면 예전처럼 한 번에 던진다.
        self._ramp_sec = _profiled('command_ramp_sec', 0.0)
        self._ramp_rate = float(self.declare_parameter('command_ramp_rate', 50.0).value)
        if self._ramp_rate <= 0.0:
            raise ValueError(f"'command_ramp_rate' must be > 0, got {self._ramp_rate}")
        publish_rate = float(self.declare_parameter('publish_rate', 10.0).value)
        if publish_rate <= 0.0:
            raise ValueError(f"'publish_rate' must be > 0, got {publish_rate}")

        # 상태. `_target` 은 마지막으로 **보낸** 목표 배열이며, 명령 이전에는 None.
        self._goal: str = ''
        # 진행 중인 램프 `(시작 스칼라, 목표 스칼라, 시작 시각)`. 없으면 `None`.
        self._ramp: Optional[tuple] = None
        # **관절 목표각 [rad]** 이다 — 실제로 발행하는 배열과 다를 수 있다(단위·packing).
        # 잔차는 보고와 같은 단위여야 하므로 라디안 쪽을 들고 있는다.
        self._target: Optional[list[float]] = None
        # 보고 축 수가 규약과 어긋나면 선다. **명령을 거부하는 근거다** — 규약이 씬과
        # 안 맞으면 명령이 통째로 잘못 해석되고, 증상은 "거의 안 움직인다" 뿐이다.
        self._convention_mismatch = False
        self._positions: list[float] = []
        self._velocities: list[float] = []
        self._efforts: list[float] = []

        self._state_pub = self.create_publisher(GripperState, _STATE_TOPIC, 10)
        self._joint_pub = self.create_publisher(Float64MultiArray, _JOINT_COMMAND_TOPIC, 10)
        self.create_subscription(GripperCommand, _CMD_TOPIC, self._on_cmd, 10)
        self.create_subscription(JointState, _JOINT_REPORT_TOPIC, self._on_report, 10)
        self.create_timer(1.0 / publish_rate, self._on_timer)
        # 램프를 안 쓰면 타이머도 만들지 않는다 — 쓰지 않는 스택의 동작을 그대로 둔다.
        if self._ramp_sec > 0.0:
            self.create_timer(1.0 / self._ramp_rate, self._on_ramp)

        self.get_logger().info(
            f'Robotiq2FGripperNode started: {_CMD_TOPIC} -> {_JOINT_COMMAND_TOPIC}, '
            f'{_JOINT_REPORT_TOPIC} -> {_STATE_TOPIC} @ {publish_rate} Hz')
        self.get_logger().info(
            f'  axes: {len(self._axis_signs)}, signs: {self._axis_signs}')
        if self._profile is not None:
            self.get_logger().info(
                f'  convention: solver={self._profile.solver} scene={self._profile.scene}, '
                f'units={self._profile.command_units}, '
                f'packing={self._profile.command_packing} ({self._profile.path})')
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

        if self._convention_mismatch:
            self.get_logger().error(
                f'[gripper] {goal}: refusing — the joint report does not match the '
                f'declared convention (see the earlier error). Fix the gripper profile '
                f'before commanding; sending anyway would be silently misinterpreted.')
            return

        if self._joint_pub.get_subscription_count() == 0:
            self.get_logger().warning(
                f"[gripper] {goal}: nobody subscribes '{_JOINT_COMMAND_TOPIC}' — "
                'publishing anyway; the simulator may not be connected')

        scalar = self._targets[goal]
        # **목표는 즉시 기록한다** — 램프 중에도 `at_goal` 은 최종 목표로 판정해야 한다.
        self._goal = goal
        self._target = target

        start = self._current_scalar()
        if self._ramp_sec > 0.0 and math.isfinite(start):
            # 지령을 나눠 보낸다. 시뮬레이터의 그리퍼 서보는 계단 지령을 자기 최대
            # 속도로 쫓으므로, 천천히 물리려면 **지령 자체**를 천천히 옮겨야 한다.
            self._ramp = (start, scalar, time.monotonic())
            self._publish_scalar(start)
            self.get_logger().info(
                f'[gripper] {goal} ramping s {start:.4f} -> {scalar:.4f} '
                f'over {self._ramp_sec:.1f}s ({len(target)} axes)')
            return

        self._ramp = None
        self._publish_scalar(scalar)
        self.get_logger().info(
            f'[gripper] {goal} sent (s={scalar:.4f}, {len(target)} axes)')

    def _publish_scalar(self, scalar: float) -> None:
        """구동 스칼라 하나를 규약대로 배열로 만들어 발행한다.

        **발행 배열 ≠ 목표각**일 수 있다 — 규약이 단위 환산과 packing 을 얹는다.
        비유한값은 규약(`GripperProfile.command`)이 거부한다. ⚠️ NaN 이 나가면
        그리퍼만이 아니라 **전 관절이 발산하고 안 돌아온다.**
        """
        command = Float64MultiArray()
        if self._profile is not None:
            command.data = self._profile.command(scalar)
        else:
            command.data = [scalar * sign for sign in self._axis_signs]
        self._joint_pub.publish(command)

    def _current_scalar(self) -> float:
        """보고에서 되짚은 지금의 구동 스칼라. 못 구하면 `NaN`.

        램프의 **출발점**이다. 여기서 시작하지 않으면 첫 발행이 그대로 점프가 되어
        천천히 보내는 의미가 없다.
        """
        if self._profile is not None:
            return self._profile.scalar_from_report(self._positions)
        if len(self._positions) != len(self._axis_signs) or not self._positions:
            return math.nan
        return (sum(p * s for p, s in zip(self._positions, self._axis_signs))
                / len(self._axis_signs))

    def _on_ramp(self) -> None:
        """진행 중인 램프를 한 스텝 내보낸다. 끝나면 **정확히 목표값**으로 마친다."""
        if self._ramp is None:
            return
        start, target, began = self._ramp
        ratio = (time.monotonic() - began) / self._ramp_sec
        if ratio >= 1.0:
            self._ramp = None
            self._publish_scalar(target)
            return
        self._publish_scalar(start + (target - start) * ratio)

    # ── 관측 ────────────────────────────────────────────────────

    def _on_report(self, msg: JointState) -> None:
        """관절 보고를 보관한다.

        **길이가 축 수와 다르면 버린다.** `Float64MultiArray` 지령이 인덱스 계약이므로
        보고도 같은 순서여야 잔차가 의미를 갖는다 (슬롯 k → 출력 k 는 실측으로
        확인됐다). 길이가 어긋난 채 잘라 쓰면 엉뚱한 축을 비교하게 된다.
        """
        expected = len(self._axis_signs)
        if len(msg.position) != expected:
            if not self._convention_mismatch:
                self._convention_mismatch = True
                self.get_logger().error(
                    f"[gripper] '{_JOINT_REPORT_TOPIC}' has {len(msg.position)} positions "
                    f'but the convention declares {expected} — REFUSING all commands. '
                    f'The gripper profile does not match the running scene; point '
                    f"'gripper_profile_file' at the right file.")
            return
        if self._convention_mismatch:
            self._convention_mismatch = False
            self.get_logger().info(
                f'[gripper] report is back to {expected} axes — accepting commands again')
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
    node = Robotiq2FGripperNode()
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
