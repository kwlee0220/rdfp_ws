#!/usr/bin/env python3
"""고정물에 꽂힌 peg 을 쥐고 **바로 위로 들어 올려 유지**되는지 본다 (체크리스트 B-16).

    ./fb_grasp_hold.py              # +78 mm 들어 30초 유지
    ./fb_grasp_hold.py 0.05 15      # 들어 올릴 높이(m), 유지 시간(초)

인자: [높이(m), 기본 0.078] [유지(초), 기본 30]

**고정물 바로 위에서만 든다.** `r2` 씬에는 `peg ↔ 탁자` 접촉이 없어 고정물 밖에서 놓치면
peg 이 탁자를 통과해 사라지고 복구는 시뮬레이터 재시작뿐이다. 수직으로만 들었다 내리므로
유지 중에 놓쳐도 peg 은 접촉이 정의된 고정물 위로 떨어진다.

판정은 둘이다 — **effort 가 남아 있는가**(`Kp × 잔차` 라 빈손이면 0.01 근처로 떨어진다),
**peg 이 tcp 를 따라 올라왔는가**(`peg − tcp` 간격이 파지 직후와 같은가).

내려놓을 때는 `fb_move_peg` 와 같은 앉음 확인을 거친다 — **안 앉았으면 쥔 채로 둔다.**
"""
from __future__ import annotations

import sys
import time

import rclpy
import fb_gripper as G
import fb_grasp as FG
import fb_ready as R
from fb_move_peg import Mover, RIDE_LIMIT, SEAT_TOL, tilt_deg, UPRIGHT_LIMIT_DEG

DEFAULT_LIFT = 0.078
DEFAULT_HOLD_SEC = 30.0
# 빈손 기준선이 0.01 N·m 이고 r2 파지가 2.3 N·m 근처다. 그 사이에 둔다.
LOST_EFFORT = 0.5
# 파지 직후보다 이만큼 간격이 벌어지면 peg 이 그리퍼 안에서 미끄러진 것이다.
SLIP_LIMIT = 0.005


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith('-')]
    lift = float(args[0]) if len(args) > 0 else DEFAULT_LIFT
    hold_sec = float(args[1]) if len(args) > 1 else DEFAULT_HOLD_SEC

    rclpy.init()
    node = rclpy.create_node('fb_grasp_hold')
    mover = Mover(node)

    obj = mover.object()
    if tilt_deg(obj) > UPRIGHT_LIMIT_DEG:
        raise SystemExit(f'peg 이 {tilt_deg(obj):.1f}° 기울어져 있다 — 서 있는 물체만 다룬다')
    here = mover.position()
    _, source, _ = mover.fixtures.floor_at(here[0], here[1])
    if source is None:
        raise SystemExit('peg 이 고정물 안에 있지 않다 — 탁자 위에서는 들지 않는다 (놓치면 사라진다)')
    fixture = mover.fixtures[source]
    print(f'peg ({here[0]:+.4f}, {here[1]:+.4f}, {here[2]:+.4f}) @ {source}  '
          f'들기 +{lift * 1000:.0f} mm · 유지 {hold_sec:.0f} s')

    cur, pub = R.attach(node)
    R.goto_ready(node, cur, pub, ramp_sec=4.0, verbose=False)
    mover.approach.spin(2.0)
    empty = G.max_effort(mover.gripper_state)
    mover.show('시작 (빈손)')

    print('\n=== 1. 집기 ===')
    gap0 = mover.grasp(obj)
    grasp_tcp = mover.approach.tcp().copy()
    eff0 = G.max_effort(mover.gripper_state)
    mover.show('파지')

    print(f'\n=== 2. 들기 +{lift * 1000:.0f} mm ===')
    peg_before = mover.position()
    up = grasp_tcp.copy()
    up[2] += lift
    mover.move(up.tolist(), velocity_scaling=0.12, settle=3.0)
    rise = mover.position()[2] - peg_before[2]
    tcp_rise = mover.approach.tcp()[2] - grasp_tcp[2]
    mover.show('들기 완료')
    print(f'  tcp +{tcp_rise * 1000:.1f} mm · peg +{rise * 1000:.1f} mm')

    print(f'\n=== 3. 유지 {hold_sec:.0f} s ===')
    lost_at = None
    worst_slip = 0.0
    began = time.time()
    while time.time() - began < hold_sec:
        mover.approach.spin(1.0)
        t = time.time() - began
        slip = (mover.position()[2] - mover.approach.tcp()[2]) - gap0
        eff = G.max_effort(mover.gripper_state)
        worst_slip = max(worst_slip, abs(slip))
        if int(t) % 5 == 0 or eff < LOST_EFFORT or abs(slip) > SLIP_LIMIT:
            print(f'  t={t:5.1f}s  |eff|={eff:.3f}  간격 변화 {slip * 1000:+.2f} mm')
        if lost_at is None and (eff < LOST_EFFORT or abs(slip) > SLIP_LIMIT):
            lost_at = t
            print(f'  ⚠ t={t:.1f}s 에 파지 상실')
            break
    eff_end = G.max_effort(mover.gripper_state)
    mover.show('유지 끝')

    print('\n=== 4. 내려놓기 ===')
    if lost_at is None:
        mover.move(grasp_tcp.tolist(), velocity_scaling=0.08, settle=4.0)
        p = mover.position()
        ride = (p[2] - mover.approach.tcp()[2]) - gap0
        entry = fixture.absolute_z('entry_z')
        seated = abs(p[2] - entry) < SEAT_TOL and ride <= RIDE_LIMIT
        print(f'  peg z={p[2]:+.4f} (목표 {entry:.4f} ± {SEAT_TOL * 1000:.0f} mm), '
              f'밀림 {ride * 1000:+.2f} mm → {"앉음 — 개방한다" if seated else "**미착석 — 쥔 채로 둔다**"}')
        if not seated:
            node.destroy_node()
            rclpy.shutdown()
            return 1
    G.set_span(node, mover.gripper_state, mover.gripper_pub, G.S_OPEN,
               ramp_sec=2.0, hold_sec=2.5, verbose=False)
    retreat = mover.approach.tcp().copy()
    retreat[2] = FG.APPROACH_Z
    mover.move(retreat.tolist(), settle=2.0)
    R.goto_ready(node, cur, pub, ramp_sec=4.0, verbose=False)
    mover.approach.spin(2.0)
    mover.show('복귀')

    print('\n' + '=' * 78)
    print(f'요약  빈손 |eff| {empty:.3f} → 파지 {eff0:.3f} → 유지 끝 {eff_end:.3f} '
          f'({100.0 * eff_end / eff0 if eff0 else 0.0:.0f}%)')
    print(f'      peg 실제 상승 +{rise * 1000:.1f} mm (tcp +{tcp_rise * 1000:.1f}) · '
          f'유지 중 최대 간격 변화 {worst_slip * 1000:.2f} mm')
    print(f'      판정: {"유지" if lost_at is None else f"상실 (t={lost_at:.1f}s)"}')
    print('=' * 78)
    mover.client.close()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if lost_at is None else 2


if __name__ == '__main__':
    sys.exit(main())
