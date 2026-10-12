#!/usr/bin/env python3
"""그리퍼 **계단 응답**을 잰다 — dead time · τ · 99% 도달 (체크리스트 B-13).

    ./fb_gripper_step.py            # 닫힘/열림 번갈아 10회
    ./fb_gripper_step.py 6          # 횟수

`fb_gripper.py` 는 2초 **램프**로 움직이므로 계단 응답을 못 잰다. 여기서는 목표 구동값
`s` 를 한 번에 바꿔 보내고 `fb_step.py` 와 **같은 분석**(10%·63.2% 교차점으로 1차 지연을
풀고, 수신 공백 0.3 s 가 낀 계단은 버린다)을 쓴다 — 팔과 그리퍼 수치를 나란히 비교하려고.

**빈손에서만 잰다.** 물체를 쥔 채 계단을 주면 접촉축이 막혀 응답이 아니라 접촉을 잰다.
시작 시 `|effort|` 가 접촉 임계를 넘으면 거부한다.

⚠️ 측정은 **수신 시각** 기준이다. 시뮬레이터가 보고를 묶어 보내면(2026-09-17 r2: 2~3건씩
약 48 ms 마다) dead time 은 그 간격만큼 뭉개진다 — τ 와 99% 는 덜 민감하다.
"""
from __future__ import annotations

from typing import Optional

import statistics
import sys
import time

import rclpy
import fb_gripper as G
from fb_step import STALL_GAP_SEC, _analyze

HOLD_SEC = 2.0
SETTLE_FRAC = 0.01


def _settle_time(samples: list[tuple[float, float]], cmd_time: float, start: float) -> Optional[float]:
    """최종값의 1% 대역에 들어와 **다시 나가지 않는** 첫 시각."""
    final = samples[-1][1]
    band = abs(final - start) * SETTLE_FRAC
    for i, (t, _) in enumerate(samples):
        if all(abs(v - final) <= band for _, v in samples[i:]):
            return t - cmd_time
    return None


def main() -> int:
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 10

    rclpy.init()
    node = rclpy.create_node('fb_gripper_step')
    state, pub = G.attach(node)
    G.wait_for_state(node, state)
    profile = G.profile_for(state)
    if G.max_effort(state) > G.CONTACT_EFFORT_NM:
        raise SystemExit(f'|effort| {G.max_effort(state):.3f} N·m — 무언가를 쥐고 있다. 빈손에서 잰다')

    print(f'그리퍼 계단 응답 — {profile["label"]}, {count}회, 유지 {HOLD_SEC}s')
    results: list[dict[str, float]] = []
    discarded = 0
    for k in range(count):
        target_s = G.S_CLOSE if k % 2 == 0 else G.S_OPEN
        start_s = G.current_span(state, profile)
        msg = G.make_command([target_s * sign for sign in profile['signs']], profile)
        samples: list[tuple[float, float]] = []
        cmd_time = time.time()
        pub.publish(msg)
        nxt = cmd_time
        while rclpy.ok() and time.time() - cmd_time < HOLD_SEC:
            if time.time() >= nxt:
                pub.publish(msg)
                nxt += G.COMMAND_PERIOD_SEC
            rclpy.spin_once(node, timeout_sec=0.002)
            samples.append((time.time(), G.current_span(state, profile)))
        fit = _analyze(samples, cmd_time, start_s)
        label = 'close' if target_s == G.S_CLOSE else 'open '
        if fit is None:
            discarded += 1
            print(f'  {k + 1:2d}  {label}  버림 (정지 구간 또는 응답 없음)')
            continue
        fit['settle_sec'] = _settle_time(samples, cmd_time, start_s) or float('nan')
        results.append(fit)
        print(f'  {k + 1:2d}  {label}  dead {fit["dead_sec"] * 1000:6.1f} ms  '
              f'tau {fit["tau_sec"] * 1000:6.1f} ms  99% {fit["settle_sec"] * 1000:6.1f} ms  '
              f'도달 {fit["reached"]:+.4f}')

    G.set_span(node, state, pub, G.S_OPEN, ramp_sec=1.0, hold_sec=1.0, verbose=False)
    print()
    if not results:
        print(f'유효한 계단이 없다 (정지 간격 {STALL_GAP_SEC}s 초과가 전부 끼었다)')
        return 1
    for key, name in (('dead_sec', 'dead time'), ('tau_sec', '시정수 tau'), ('settle_sec', '99% 도달')):
        vals = [r[key] * 1000 for r in results if r[key] == r[key]]
        print(f'{name:10s} 중앙 {statistics.median(vals):6.1f} ms  범위 {min(vals):.1f} ~ {max(vals):.1f}')
    print(f'유효 {len(results)}회 / 버림 {discarded}회')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
