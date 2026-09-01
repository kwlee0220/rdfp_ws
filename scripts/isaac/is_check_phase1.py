#!/usr/bin/env python3
"""Isaac 백엔드 **Phase 1 수용 기준**(팔 관절 명령)을 검사한다.

``docs/simulation/isaac_backend_skeleton.md`` §2 Phase 1 의 항목을 잰다.

    0. 재생 상태    — Isaac 이 Play 중인가 (아니면 뒤 검사가 전부 엉뚱하게 실패한다)
    1. 명령 경로    — /isaac/arm_command 를 Isaac 이 구독하는가
    2. 계단 응답    — dead time 과 시정수 τ (중력 중립축 joint1)
    3. 절대 정착 오차 @ ready    — 명령한 목표 대비 남는 오차
    4. 절대 정착 오차 @ extended — 같은 값의 **자세 의존성**

**모든 측정은 named target 으로 이동한 뒤 그 목표값을 기준으로 한다.**
"현재 자세를 그대로 명령하고 오차를 재는" 방식은 쓰지 않는다 — 이미 처진 자리를
목표로 삼는 것이라 중력 처짐이 **보이지 않고**, 반복하면 처짐이 누적된다
(펑션베이 문서 §3.1 · §10.2 에서 실제로 당한 함정이다).

계단 응답도 자세를 고정한 뒤 잰다. joint1 은 중력 중립축이라 처짐은 섞이지 않지만
**관성은 팔을 펼수록 커지므로**, 자세를 고정하지 않으면 τ 가 재현되지 않는다
(같은 스택에서 205 ms 와 766 ms 가 나왔다).

    ./is_check_phase1.py
    ./is_check_phase1.py --steps 4

종료 코드: 전부 통과하면 0, 하나라도 실패하면 1.
"""
from __future__ import annotations

from typing import Optional

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

ARM_JOINT_NAMES = [f'panda_joint{i}' for i in range(1, 8)]
ARM_COMMAND_TOPIC = '/isaac/arm_command'
JOINT_STATE_TOPIC = '/joint_states'

# 계단 응답을 잴 관절. 회전축이 수직이라 중력 토크가 0 이다.
STEP_JOINT = 'panda_joint1'
STEP_SIZE_RAD = 0.04
STEP_SETTLE_SEC = 2.5
COMMAND_RATE_HZ = 50.0

# 측정 기준 자세. 두 자세를 재야 오차의 **자세 의존성**이 드러난다.
ANCHOR_POSE = 'ready'
SETTLE_WAIT_SEC = 5.0

# 두 번째 자세는 named target 을 쓰지 않고 **관절값을 직접 준다.**
#
# `extended` 를 쓰면 안 되는 이유가 둘이다.
#   1. 팔이 수직으로 곧게 서는 자세라 **중력 토크가 거의 0** 이다 — 재려는 값이
#      나오지 않는다.
#   2. moveit_resources 의 URDF 는 panda_joint4 상한이 +0.0873 인데 **Isaac USD 는
#      실제 Franka 스펙인 -0.0698** 이다. `extended` 는 j4=0 을 요구하므로 Isaac 이
#      한계에서 막고, 그 -0.0698 이 "정착 오차"로 잘못 잡힌다. 강성을 625배 올려도
#      값이 소수점 4자리까지 그대로였던 것이 물리적 정지의 증거다.
#
# 아래는 `ready` 에서 팔꿈치(j4)만 펴 **모멘트 암을 늘린** 자세다. 두 한계 집합
# 모두에 여유 있게 들어간다.
STRETCH_POSE = {
    'panda_joint1': 0.0,
    'panda_joint2': -0.785,
    'panda_joint3': 0.0,
    'panda_joint4': -1.2,
    'panda_joint5': 0.0,
    'panda_joint6': 1.571,
    'panda_joint7': 0.785,
}
SECOND_POSE = 'stretch'

# 현재 drive stiffness. 오차로부터 중력 토크를 역산해 보고하는 데만 쓴다.
DRIVE_STIFFNESS = 10000.0

# 수용 기준.
MAX_SETTLE_ERROR_RAD = 0.005
MAX_TAU_SEC = 0.050
MAX_OVERSHOOT = 0.05        # 계단 크기 대비 5%

# DDS 디스커버리 대기 한도(초). Windows(Isaac) ↔ WSL2 경계는 매칭이 늦다.
DISCOVERY_TIMEOUT_SEC = 15.0


class Phase1Checker(Node):
    """관절 상태를 기록하고 명령을 발행하는 검사 노드."""

    def __init__(self) -> None:
        super().__init__('is_check_phase1',
                         parameter_overrides=[Parameter('use_sim_time', value=False)])
        self.samples: list[tuple[float, dict[str, float]]] = []
        self.clock_samples: list[float] = []
        self.create_subscription(JointState, JOINT_STATE_TOPIC, self._on_state, 50)
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self.publisher = self.create_publisher(JointState, ARM_COMMAND_TOPIC, 10)

    def _on_clock(self, msg: Clock) -> None:
        self.clock_samples.append(msg.clock.sec + msg.clock.nanosec * 1e-9)

    def _on_state(self, msg: JointState) -> None:
        if msg.name:
            self.samples.append((time.time(), dict(zip(msg.name, msg.position))))

    def spin(self, seconds: float) -> None:
        deadline = time.time() + seconds
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def latest(self) -> dict[str, float]:
        if not self.samples:
            raise RuntimeError(f'no JointState received on {JOINT_STATE_TOPIC}')
        return dict(self.samples[-1][1])

    def hold(self, targets: dict[str, float], seconds: float) -> None:
        """목표를 COMMAND_RATE_HZ 로 계속 밀면서 상태를 기록한다."""
        msg = JointState()
        msg.name = list(targets.keys())
        msg.position = [float(v) for v in targets.values()]
        period = 1.0 / COMMAND_RATE_HZ
        deadline = time.time() + seconds
        next_publish = time.time()
        while rclpy.ok() and time.time() < deadline:
            now = time.time()
            if now >= next_publish:
                self.publisher.publish(msg)
                next_publish = now + period
            rclpy.spin_once(self, timeout_sec=0.005)


def _fmt(ok: bool) -> str:
    return '통과' if ok else '실패'


def _row(values: dict[str, float]) -> str:
    return '  '.join(f'j{i + 1}:{values.get(j, math.nan):+.4f}'
                     for i, j in enumerate(ARM_JOINT_NAMES))


def analyze_step(samples: list[tuple[float, float]], t_cmd: float, start: float,
                 target: float) -> tuple[Optional[float], Optional[float], float]:
    """계단 응답에서 dead time · 시정수 τ · 오버슈트를 뽑는다.

    dead time 은 **어떤 반응이든 처음 나타나기까지**, τ 는 그 뒤 목표의 63.2% 에
    이르기까지다. 둘을 나누는 이유는 처방이 갈리기 때문이다 — dead time 은
    되먹임으로 보상할 수 없고, τ 는 강성으로 줄인다.

    오버슈트는 **강성을 올린 뒤 반드시 봐야 하는 값**이다. K 만 올리고 감쇠 B 를
    함께 올리지 않으면 감쇠비가 떨어져 진동한다 — τ 는 좋아지는데 진동하는 상태를
    "개선"으로 오독하지 않기 위해 같이 잰다.

    Returns:
        ``(dead_time, tau, overshoot_ratio)``. 오버슈트는 계단 크기 대비 비율.
    """
    delta = target - start
    if abs(delta) < 1e-9:
        return None, None, 0.0
    move_threshold = abs(delta) * 0.02
    tau_level = start + delta * 0.632

    dead_time = None
    tau = None
    overshoot = 0.0
    for t, value in samples:
        if t < t_cmd:
            continue
        if dead_time is None and abs(value - start) > move_threshold:
            dead_time = t - t_cmd
        if dead_time is not None and tau is None:
            reached = value >= tau_level if delta > 0 else value <= tau_level
            if reached:
                tau = (t - t_cmd) - dead_time
        beyond = (value - target) if delta > 0 else (target - value)
        overshoot = max(overshoot, beyond / abs(delta))
    return dead_time, tau, overshoot


def check_playback(node: Phase1Checker) -> bool:
    """기준 0 — Isaac 이 재생 중인가.

    그래프를 다시 만들면 Play 가 풀린다. 정지 상태에서는 노드가 tick 하지 않아
    구독조차 생성되지 않으므로, 뒤 검사가 전부 "명령 경로 없음"으로 보인다.
    """
    deadline = time.time() + DISCOVERY_TIMEOUT_SEC
    advancing = False
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.clock_samples) >= 2 and node.clock_samples[-1] > node.clock_samples[0]:
            advancing = True
            break
    waited = DISCOVERY_TIMEOUT_SEC - max(deadline - time.time(), 0.0)
    print(f'  0. 재생 상태          : {_fmt(advancing)} — /clock 샘플 '
          f'{len(node.clock_samples)}개, 대기 {waited:.1f}s'
          + (f', sim {node.clock_samples[-1]:.2f}s' if node.clock_samples else ''))
    if not advancing:
        if node.clock_samples:
            print('       → /clock 이 오지만 시간이 멈춰 있다. **Play(▶) 를 누른다.**')
        else:
            print('       → /clock 이 전혀 오지 않는다. Play 상태와 도메인(31)을 확인한다')
    return advancing


def check_command_path(node: Phase1Checker) -> bool:
    """기준 1 — Isaac 이 명령 토픽을 구독하고 있는가."""
    deadline = time.time() + DISCOVERY_TIMEOUT_SEC
    count = 0
    while rclpy.ok() and time.time() < deadline:
        count = node.count_subscribers(ARM_COMMAND_TOPIC)
        if count:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    ok = count > 0
    print(f'  1. 명령 경로          : {_fmt(ok)} — {ARM_COMMAND_TOPIC} 구독자 {count}개')
    if not ok:
        print('       → Isaac 그래프의 PHASE 가 1 인지 확인한다')
    return ok


def goto_named(client, node: Phase1Checker, name: str) -> dict[str, float]:
    """named target 으로 이동하고 **그 목표 관절값**을 돌려준다.

    현재 자세가 아니라 이 목표값이 이후 모든 오차의 기준이 된다.
    """
    trajectory = client.plan_named_target(name)
    joint_trajectory = trajectory.joint_trajectory
    goal = dict(zip(joint_trajectory.joint_names, joint_trajectory.points[-1].positions))
    client.stream_trajectory(trajectory, publish_rate=COMMAND_RATE_HZ)
    # 스트리밍이 끝나면 마지막 명령이 유지된다. 정착을 기다린다.
    node.hold({j: goal[j] for j in ARM_JOINT_NAMES if j in goal}, SETTLE_WAIT_SEC)
    return goal


def measure_settle(node: Phase1Checker, goal: dict[str, float]) -> dict[str, float]:
    """목표 대비 관절별 오차를 잰다 (절대 기준)."""
    actual = node.latest()
    return {j: actual.get(j, math.nan) - goal[j] for j in ARM_JOINT_NAMES if j in goal}


def measure_step(node: Phase1Checker, base: dict[str, float],
                 steps: int) -> tuple[list[float], list[float], list[float]]:
    """기준 자세에서 계단 응답을 반복해 dead time 과 τ 를 모은다."""
    targets = {j: base[j] for j in ARM_JOINT_NAMES if j in base}
    dead_times: list[float] = []
    taus: list[float] = []
    overshoots: list[float] = []
    for i in range(steps):
        direction = 1.0 if i % 2 == 0 else -1.0
        start = targets[STEP_JOINT]
        target = start + direction * STEP_SIZE_RAD
        targets[STEP_JOINT] = target

        mark = len(node.samples)
        t_cmd = time.time()
        node.hold(targets, STEP_SETTLE_SEC)
        window = [(t, s[STEP_JOINT]) for t, s in node.samples[mark:] if STEP_JOINT in s]

        dead_time, tau, overshoot = analyze_step(window, t_cmd, start, target)
        if dead_time is not None:
            dead_times.append(dead_time)
        if tau is not None:
            taus.append(tau)
        overshoots.append(overshoot)
    return dead_times, taus, overshoots


def run_measurements(node: Phase1Checker, steps: int) -> list[bool]:
    """기준 2~4 를 잰다. 실패해도 남은 항목을 계속 잰다."""
    from robot_control.moveit.move_group_factory import create_move_group_client

    planner = rclpy.create_node('is_check_phase1_planner',
                                parameter_overrides=[Parameter('use_sim_time', value=True)])
    try:
        client = create_move_group_client(
            planner, mode='jgpc', arm_command_topic=ARM_COMMAND_TOPIC,
            arm_command_joint_names=ARM_JOINT_NAMES, arm_command_format='joint_state')

        anchor_goal = goto_named(client, node, ANCHOR_POSE)
        anchor_error = measure_settle(node, anchor_goal)

        dead_times, taus, overshoots = measure_step(node, anchor_goal, steps)

        # 계단 시험이 joint1 을 흔들어 놓았으므로 기준 자세로 되돌린 뒤 다음 자세로 간다.
        goto_named(client, node, ANCHOR_POSE)
        client.move_to_joints_streamed(STRETCH_POSE, publish_rate=COMMAND_RATE_HZ)
        node.hold(STRETCH_POSE, SETTLE_WAIT_SEC)
        second_error = measure_settle(node, STRETCH_POSE)
    finally:
        planner.destroy_node()

    # --- 기준 2 ---
    if taus:
        dt_avg = sum(dead_times) / len(dead_times) if dead_times else math.nan
        tau_avg = sum(taus) / len(taus)
        overshoot_max = max(overshoots) if overshoots else 0.0
        step_ok = tau_avg <= MAX_TAU_SEC and overshoot_max <= MAX_OVERSHOOT
        print(f'  2. 계단 응답 @{ANCHOR_POSE}  : {_fmt(step_ok)} — dead time {dt_avg * 1000:.1f} ms, '
              f'τ {tau_avg * 1000:.1f} ms, 오버슈트 {overshoot_max * 100:.1f}% '
              f'(기준 τ ≤ {MAX_TAU_SEC * 1000:.0f} ms, 오버슈트 ≤ {MAX_OVERSHOOT * 100:.0f}%, '
              f'{len(taus)}회)')
        if tau_avg > MAX_TAU_SEC:
            print('       → τ 초과. articulation drive 의 stiffness 를 올린다 (damping 도 함께)')
        if overshoot_max > MAX_OVERSHOOT:
            print('       → 오버슈트 초과. damping 이 부족하다 (K 만 올린 결과일 수 있다)')
    else:
        step_ok = False
        print(f'  2. 계단 응답 @{ANCHOR_POSE}  : {_fmt(False)} — 반응을 관측하지 못했다')

    # --- 기준 3·4 ---
    results = [step_ok]
    for label, errors in ((ANCHOR_POSE, anchor_error), (SECOND_POSE, second_error)):
        worst_joint = max(errors, key=lambda j: abs(errors[j]))
        worst = abs(errors[worst_joint])
        ok = worst <= MAX_SETTLE_ERROR_RAD
        index = 3 if label == ANCHOR_POSE else 4
        print(f'  {index}. 정착 오차 @{label}  : {_fmt(ok)} — 최악 {worst_joint} '
              f'{errors[worst_joint]:+.4f} rad (기준 ≤ {MAX_SETTLE_ERROR_RAD})')
        print(f'       {_row(errors)}')
        results.append(ok)

    # 자세 의존성은 통과/실패와 별개로 항상 보고한다 — 중력 처짐의 지문이다.
    anchor_worst = max(abs(v) for v in anchor_error.values())
    second_worst = max(abs(v) for v in second_error.values())
    print(f'     자세 의존성        : {ANCHOR_POSE} {anchor_worst:.4f} → '
          f'{SECOND_POSE} {second_worst:.4f} rad')
    if second_worst > anchor_worst * 2 and second_worst > MAX_SETTLE_ERROR_RAD:
        print('       → 자세에 따라 커진다 = 중력 토크 / 강성. drive stiffness 를 올린다')
    # 오차에서 중력 토크를 역산한다. 한계 클램프(값이 강성과 무관하게 고정)와
    # 실제 처짐(강성에 반비례)을 가르는 데 쓴다.
    print(f'     추정 중력 토크     : {second_worst * DRIVE_STIFFNESS:.1f} Nm '
          f'(오차 x stiffness {DRIVE_STIFFNESS:.0f})')
    return results


def main() -> int:
    parser = argparse.ArgumentParser(description='Isaac Phase 1 수용 기준 검사')
    parser.add_argument('--steps', type=int, default=6, help='계단 응답 반복 횟수')
    args = parser.parse_args()

    rclpy.init()
    node = Phase1Checker()
    print('Isaac Phase 1 수용 기준 검사')

    results = [check_playback(node)]
    if results[0]:
        results.append(check_command_path(node))
    if all(results):
        try:
            results.extend(run_measurements(node, args.steps))
        except Exception as exc:
            print(f'  측정 중단: {type(exc).__name__}: {exc}')
            results.append(False)
    node.destroy_node()

    passed = sum(1 for r in results if r)
    print(f'\n{passed}/{len(results)} 통과')
    rclpy.shutdown()
    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
