#!/usr/bin/env python3
"""수직 이동 중 손끝이 **옆으로 새는 양**을 잰다 (체크리스트 B-17).

    ./fb_curl.py                  # 기본 지점에서 +45 mm 상승
    ./fb_curl.py 0.045 --down     # 하강으로
    ./fb_curl.py --at 0.45,0.15,0.12
    ./fb_curl.py --no-ready       # ready 복귀를 건너뛴다

인자: [이동량(m), 기본 0.045] [--down] [--at x,y,z] [--no-ready] [--keep-grasp]
      [--vs <배율>, 기본 0.12]

**무엇을 보는가.** 순수 수직 이동인데도 팔이 중력으로 처지면서 손끝이 **로봇 쪽으로
말렸다가 풀린다.** peg 은 base 기준 +x 쪽이므로 **`dx < 0` 이 안쪽으로 당겨짐**이다.
사람 눈에는 "올라가기 전에 안으로 한 번 말았다가 편다"로 보인다.

**왜 별도 항목인가.** B-3·B-4·B-5 는 **정지 상태**의 관절 오차를, B-8 은 lag 과 평균
오차를, B-9 는 상승·하강 비대칭을 본다. **이동 중 손끝이 옆으로 얼마나 새는지**를 보는
항목이 없었다 — 그리고 그것이 파지 정확도를 직접 망친다(집을 때 기울어지고 구멍에
어긋나게 놓인다).

**보상이 조용히 꺼진 것도 여기서 걸린다.** `motion.gravity_compensation` 이 빠지거나
`Kp` 가 바뀌거나 URDF 경로가 깨지면 로더가 `None` 을 돌려주고 **아무 오류 없이** 보상만
사라진다. 그때 말림이 수 mm 로 돌아온다. 반대로 부호가 뒤집히거나 벤더가 A-1 을
반영했는데 우리 보상을 안 끄면 **오차가 반대로 커진다**(이중 보정).

배경·수치: `docs/simulation/functionbay_gravity_compensation.md`
"""
from __future__ import annotations

import sys
import threading
import time

import numpy as np
import rclpy
import fb_grasp as FG
import fb_gripper as G
import fb_ready as R
from robot_control.moveit.move_group_factory import create_move_group_client

BACKEND = 'functionbay'
DEFAULT_LIFT = 0.045
DEFAULT_VS = 0.12
DEFAULT_AT = (0.50, 0.00, 0.120)
SAMPLE_HZ = 100.0
SETTLE_SEC = 4.0
# 판정 기준. 보상 OFF 실측이 말림 −7.14 / 도달 오차 12.25 mm 였고, ON 은 0.01 이었다.
# 0.5 mm 는 그 사이에 넉넉히 들어가면서 측정 잡음(0.05 mm)보다 한 자릿수 크다.
CURL_LIMIT_MM = 0.5
REACH_LIMIT_MM = 1.0


def take_flag(name: str) -> bool:
    if name in sys.argv:
        sys.argv.remove(name)
        return True
    return False


def take_option(name: str, default):
    if name not in sys.argv:
        return default
    index = sys.argv.index(name)
    value = sys.argv[index + 1]
    del sys.argv[index:index + 2]
    return value


def sample_during(node, probe, move) -> list:
    """이동하는 동안 TCP 를 `SAMPLE_HZ` 로 기록한다. 이동 후 `SETTLE_SEC` 더 본다.

    **정착까지 봐야 한다** — 스트리밍은 open loop 라 반환이 도달을 뜻하지 않는다.
    """
    rows: list = []
    done = [False]
    began = time.time()

    def spin() -> None:
        nxt = time.time()
        while not done[0]:
            rclpy.spin_once(node, timeout_sec=0.002)
            if time.time() >= nxt:
                try:
                    tcp = probe.tcp()
                    rows.append((time.time() - began, tcp[0], tcp[1], tcp[2]))
                except Exception:
                    pass
                nxt += 1.0 / SAMPLE_HZ

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    move()
    deadline = time.time() + SETTLE_SEC
    while time.time() < deadline:
        time.sleep(0.02)
    done[0] = True
    thread.join(timeout=2.0)
    return rows


def report(rows: list, goal, lift_mm: float) -> tuple:
    """말림 최대·정착·도달 오차를 내고 `(말림, 도달오차)` 를 돌려준다."""
    t = np.array([r[0] for r in rows])
    x = np.array([r[1] for r in rows])
    z = np.array([r[3] for r in rows])
    dx = (x - x[0]) * 1000.0
    dz = (z - z[0]) * 1000.0

    print(f'\n{"t(s)":>6} {"dz(mm)":>9} {"dx(mm)":>9}')
    for mark in (0.0, 0.4, 0.8, 1.2, 1.6, 2.4, 3.5, 5.0, 7.0, 9.0):
        if mark > t[-1]:
            break
        index = int(np.argmin(np.abs(t - mark)))
        print(f'{t[index]:>6.2f} {dz[index]:>9.2f} {dx[index]:>9.3f}')

    early = t <= 2.0
    curl = float(dx[early].min()) if early.any() else float(dx.min())
    settle = float(dx[-1])
    reach = (np.array([x[-1], rows[-1][2], z[-1]]) - goal) * 1000.0
    print(f'\n  말림 최대 {curl:+.3f} mm → 정착 {settle:+.3f} mm '
          f'(풀림 {settle - curl:+.3f})')
    print(f'  수직 달성 {dz[-1]:+.2f} / 지령 {lift_mm:+.1f} mm')
    print(f'  도달 오차 dx={reach[0]:+.2f} dy={reach[1]:+.2f} dz={reach[2]:+.2f}  '
          f'|오차| {float(np.linalg.norm(reach)):.2f} mm')
    return curl, float(np.linalg.norm(reach))


def verdict(curl: float, reach: float) -> None:
    """판정과 **다음에 볼 곳**을 함께 낸다."""
    print()
    if abs(curl) <= CURL_LIMIT_MM and reach <= REACH_LIMIT_MM:
        print(f'  ✅ 통과 — 말림 {abs(curl):.2f} ≤ {CURL_LIMIT_MM} mm, '
              f'도달 오차 {reach:.2f} ≤ {REACH_LIMIT_MM} mm')
        return
    print(f'  ⚠️ 기준 초과 — 말림 {abs(curl):.2f} mm (한계 {CURL_LIMIT_MM}), '
          f'도달 오차 {reach:.2f} mm (한계 {REACH_LIMIT_MM})')
    print('     중력 보상을 먼저 본다:')
    print("       python3 -c \"from robot_control.gravity import describe; "
          "print(describe('functionbay'))\"")
    print('     ① 출력이 "중력 보상 없음" → 프로파일에서 키가 빠졌다')
    print('     ② Kp 가 2000 이 아니다 → 씬 XML 의 <Gain Kp=...> 가 바뀌었다')
    print('     ③ 오차가 **반대 부호로** 크다 → 이중 보정 '
          '(벤더가 A-1 을 반영했으면 yaml 블록을 지운다)')


def main() -> None:
    skip_ready = take_flag('--no-ready')
    keep_grasp = take_flag('--keep-grasp')
    downward = take_flag('--down')
    velocity_scaling = float(take_option('--vs', DEFAULT_VS))
    at_text = take_option('--at', None)
    lift = float(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_LIFT
    start_xyz = np.array([float(v) for v in at_text.split(',')] if at_text else DEFAULT_AT)

    rclpy.init()
    node = rclpy.create_node('fb_curl')
    probe = FG.Approach(node)
    probe.spin(3.0)
    gripper_state, gripper_pub = G.attach(node)
    G.wait_for_state(node, gripper_state)

    client = create_move_group_client(node, backend=BACKEND)
    client.wait_until_ready(timeout_sec=30.0)
    probe.spin(2.0)

    if not skip_ready:
        current, publisher = R.attach(node)
        R.goto_ready(node, current, publisher, ramp_sec=4.0, verbose=False,
                     open_gripper=not keep_grasp)
        probe.spin(2.0)

    # 출발점으로 이동한 뒤 정착시킨다 — 여기서부터의 변화량을 본다.
    client.follow_trajectory_streamed([probe.tip_pose_for(start_xyz.tolist())],
                                      publish_rate=50.0, velocity_scaling=0.2)
    probe.spin(3.5)
    start = probe.tcp().copy()
    print(f'출발점 지령 {start_xyz.round(4)} → 실제 {start.round(4)}  '
          f'(오차 {np.linalg.norm(start - start_xyz) * 1000:.2f} mm)')

    signed = -lift if downward else lift
    goal = start.copy()
    goal[2] += signed
    print(f'{"하강" if downward else "상승"} {abs(signed) * 1000:.0f} mm, '
          f'velocity_scaling {velocity_scaling}')

    rows = sample_during(node, probe,
                         lambda: client.follow_trajectory_streamed(
                             [probe.tip_pose_for(goal.tolist())], publish_rate=50.0,
                             velocity_scaling=velocity_scaling))
    if len(rows) < 10:
        raise SystemExit(f'표본이 {len(rows)}개뿐이다 — TF 가 오고 있는지 본다')

    curl, reach = report(rows, goal, signed * 1000.0)
    verdict(curl, reach)
    print('\n기준선 (보상 OFF, 2026-09-11): 말림 -7.14 mm · 도달 오차 12.25 mm')

    if not skip_ready:
        current, publisher = R.attach(node)
        R.goto_ready(node, current, publisher, ramp_sec=4.0, verbose=False,
                     open_gripper=not keep_grasp)
    client.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
