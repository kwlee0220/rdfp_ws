#!/usr/bin/env python3
"""관절을 한계 쪽으로 천천히 밀어 **실효 가동범위**를 잰다 (체크리스트 B-10 · B-11).

시뮬레이터의 실제 정지점은 URDF 한계와 다르고 **자세에 따라 변한다.** 기준선:

    joint7 상한  **1.6387 rad**  (URDF 2.9671)   ← B-10
    joint4 하한  **-2.7599 rad** (URDF -3.1416)  ← B-10
    joint4 상한  **0.0698 rad**  (URDF  0.0873)  ← B-11 (한계를 강제하는가)

**계단으로 던지지 않고 천천히 램프한다.** 시뮬레이터는 내부 보간을 하지 않아 큰
도약을 그대로 실행하므로, 한계 밖을 한 번에 지시하면 어디서 멈췄는지가 아니라
"튕겨 나온 결과"를 재게 된다. 여기서는 지령을 일정 속도로 올리며 **실제가 따라오기를
멈추는 지점**을 찾는다.

**정지(A-17)를 한계 도달로 오독하지 않는다.** 시뮬레이터가 34초마다 2초 멈추는데,
그동안에도 "실제값이 안 변한다". 둘의 차이는 **메시지가 오느냐**다 — 한계에 걸린
것이면 보고는 정상 주기로 계속 오면서 값만 고정되고, 정지면 보고 자체가 끊긴다.
그래서 판정에 쓰는 표본은 **수신 간격이 정상인 것만** 센다.

안전을 위해 지령은 URDF 한계에서 `OVERSHOOT_RAD` 이상 넘어가지 않는다.

    ./fb_limit.py                  # 기준선 3항목 (joint7+, joint4-, joint4+)
    ./fb_limit.py 7 +              # 관절 번호와 방향 지정

인자: [관절 번호 1~7] [+|-]  (둘 다 생략하면 기준선 3항목) [--no-ready] [--ramp <초>]
"""
from __future__ import annotations

from typing import Optional

import sys
import time

import rclpy
from sensor_msgs.msg import JointState

import fb_ready

ARM = fb_ready.ARM
# 기준선이 있는 조합. (관절 번호, 부호, 기준선 정지점, 체크리스트 항목)
BASELINE_CASES = [(7, +1.0, 1.6387, 'B-10'), (4, -1.0, -2.7599, 'B-10'),
                  (4, +1.0, 0.0698, 'B-11')]
# 지령을 올리는 속도. 느릴수록 정지점이 또렷하지만 오래 걸린다.
RAMP_RATE_RAD_S = 0.15
# URDF 한계를 이만큼까지만 넘어간다. 안전 상한이다.
OVERSHOOT_RAD = 0.25
# 이 시간 동안 실제가 이 폭보다 덜 움직이면 멈춘 것으로 본다.
STUCK_WINDOW_SEC = 1.0
STUCK_BAND_RAD = 0.002
# 지령과 실제가 이만큼 벌어져야 '못 따라온다' 고 판정한다.
MIN_LAG_RAD = 0.05
# 이 값을 넘는 수신 공백은 시뮬레이터 정지다 — 판정 표본에서 뺀다.
STALL_GAP_SEC = 0.3
MAX_RUN_SEC = 40.0


def _probe(node, cur, pub, idx: int, sign: float, base: list[float]) -> Optional[dict]:
    """한 방향으로 램프하며 실제가 멈추는 지점을 찾는다."""
    joint = ARM[idx]
    lo, hi = fb_ready.LIMITS[joint]
    ceiling = (hi + OVERSHOOT_RAD) if sign > 0 else (lo - OVERSHOOT_RAD)
    print(f'  {joint} {"+" if sign > 0 else "-"} 방향  '
          f'시작 {base[idx]:+.4f} → 지령 상한 {ceiling:+.4f} (URDF {lo:+.4f}~{hi:+.4f})')

    msg = JointState()
    msg.name = list(ARM)
    target = list(base)
    started = time.time()
    nxt = started
    # (수신시각, 값) — 수신 간격이 정상인 구간만 정지 판정에 쓴다.
    trail: list[tuple[float, float]] = []
    last_seen = None
    result = None
    while rclpy.ok() and time.time() - started < MAX_RUN_SEC:
        elapsed = time.time() - started
        want = base[idx] + sign * RAMP_RATE_RAD_S * elapsed
        want = min(want, ceiling) if sign > 0 else max(want, ceiling)
        target[idx] = want
        if time.time() >= nxt:
            msg.position = list(target)
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            nxt += fb_ready.COMMAND_PERIOD_SEC
        rclpy.spin_once(node, timeout_sec=0.005)

        now, actual = time.time(), float(cur[joint])
        if last_seen is not None and now - last_seen > STALL_GAP_SEC:
            trail.clear()                  # 정지 구간을 지나왔다 — 표본을 버린다
        last_seen = now
        trail.append((now, actual))
        trail = [s for s in trail if now - s[0] <= STUCK_WINDOW_SEC]

        if len(trail) < 15 or trail[-1][0] - trail[0][0] < STUCK_WINDOW_SEC * 0.8:
            continue
        span = max(v for _, v in trail) - min(v for _, v in trail)
        lag = abs(want - actual)
        if span < STUCK_BAND_RAD and lag > MIN_LAG_RAD:
            result = {'stop': actual, 'commanded': want, 'lag': lag, 'elapsed': elapsed}
            break
        if abs(want - ceiling) < 1e-9 and lag < MIN_LAG_RAD:
            result = {'stop': actual, 'commanded': want, 'lag': lag, 'elapsed': elapsed,
                      'note': '지령 상한까지 따라왔다 — 실효 한계가 더 바깥이다'}
            break
    return result


def main() -> None:
    skip_ready = fb_ready.take_flag('--no-ready')
    keep_grasp = fb_ready.take_flag('--keep-grasp')
    ramp_sec = fb_ready.DEFAULT_RAMP_SEC
    if '--ramp' in sys.argv:
        i = sys.argv.index('--ramp')
        ramp_sec = float(sys.argv[i + 1])
        del sys.argv[i:i + 2]
    if len(sys.argv) > 2:
        cases = [(int(sys.argv[1]), 1.0 if sys.argv[2] == '+' else -1.0, None, '수동')]
    else:
        cases = BASELINE_CASES

    rclpy.init()
    node = rclpy.create_node('fb_limit')
    cur, pub = fb_ready.attach(node)
    if skip_ready:
        fb_ready.wait_for_state(node, cur)
        print('[ready] 건너뜀 (--no-ready)')
    else:
        fb_ready.goto_ready(node, cur, pub, ramp_sec=ramp_sec, open_gripper=not keep_grasp)

    rows = []
    for joint_no, sign, expected, item in cases:
        # 매번 ready 에서 출발한다 — 실효 한계가 자세 의존이라 출발점이 같아야 비교된다.
        fb_ready.goto_ready(node, cur, pub, ramp_sec=3.0, verbose=False,
                            open_gripper=not keep_grasp)
        base = fb_ready.wait_for_state(node, cur)
        print(f'\n[{item}] panda_joint{joint_no}')
        found = _probe(node, cur, pub, joint_no - 1, sign, base)
        if found is None:
            print(f'  판정 불가 — {MAX_RUN_SEC:.0f}초 안에 멈추지 않았다')
            rows.append((item, joint_no, sign, expected, None))
            continue
        note = found.get('note', '')
        print(f'  정지점 {found["stop"]:+.4f} rad  (지령 {found["commanded"]:+.4f}, '
              f'벌어짐 {found["lag"]:.4f}, {found["elapsed"]:.1f}s) {note}')
        if expected is not None:
            print(f'  기준선 {expected:+.4f} rad  → 차 {found["stop"] - expected:+.4f} rad')
        rows.append((item, joint_no, sign, expected, found['stop']))

    fb_ready.goto_ready(node, cur, pub, ramp_sec=4.0, open_gripper=not keep_grasp)

    print('\n요약')
    print(f'{"항목":>6} {"관절":>14} {"방향":>4} {"기준선":>10} {"이번":>10} {"차":>9}')
    for item, joint_no, sign, expected, stop in rows:
        s = f'{stop:+.4f}' if stop is not None else '  판정불가'
        e = f'{expected:+.4f}' if expected is not None else '        -'
        known = stop is not None and expected is not None
        d = f'{stop - expected:+.4f}' if known else '        -'
        print(f'{item:>6} {"panda_joint%d" % joint_no:>14} {"+" if sign > 0 else "-":>4} '
              f'{e:>10} {s:>10} {d:>9}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
