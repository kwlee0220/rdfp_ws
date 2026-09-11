#!/usr/bin/env python3
"""계단 응답으로 **dead time 과 시정수 τ** 를 분해한다 (체크리스트 B-6 · B-7).

한 관절에 ``±delta`` 계단을 번갈아 주고 응답이 언제 움직이기 시작해(dead time)
얼마나 빨리 따라붙는지(τ) 잰다. 기준선은 dead time **43 ~ 58 ms**, τ **252 ~ 259 ms**
(2026-08-31 3차, 서울대 Crisp 솔버). τ 는 카메라 주기 선택의 근거이기도 하다.

**기본 관절이 joint7 인 이유** — 회전축이 수직이라 중력 토크가 0 이다. 중력 처짐이
섞이면 "늦게 도달한 것" 과 "덜 도달한 것" 이 구별되지 않는다.

**1차 지연 모델의 두 점으로 푼다.** 응답이 ``final * (1 - exp(-(t-dead)/tau))`` 라면
10% 와 63.2% 도달 시각이 각각 ``dead + 0.105*tau`` 와 ``dead + tau`` 이므로

    tau  = (t63 - t10) / 0.895
    dead = t10 - 0.105 * tau

교차 시각은 표본 사이를 선형 보간해 구한다. **보고 주기가 30 Hz(33 ms)라 표본 하나가
dead time 기준선(43~58 ms)과 비슷한 크기다.** 그래서 계단을 여러 번 주고 중앙값을
쓴다 — 계단마다 표본 격자와의 위상이 달라 평균적으로 분해능이 올라간다. 그래도
dead time 은 τ 만큼 신뢰하지 못하며, 정밀히 재려면 시뮬레이터 보고 주기를 올려야 한다.

**시뮬레이터 정지(A-17)가 낀 계단은 버린다.** 34초마다 2초씩 멈추므로 12회를 돌리면
한두 번은 반드시 걸리는데, 그 구간은 응답이 아니라 정지를 재게 된다.

    ./fb_step.py                    # joint7, ±0.05 rad, 12회
    ./fb_step.py 0.05 12 7          # delta, 횟수, 관절 번호(1~7)
    ./fb_step.py 0.05 12 7 --no-ready

인자: [delta(rad), 기본 0.05] [횟수, 기본 12] [관절 번호, 기본 7] [--no-ready] [--ramp <초>]
"""
from __future__ import annotations

from typing import Optional

import statistics
import sys
import time

import rclpy
from sensor_msgs.msg import JointState

import fb_ready

ARM = fb_ready.ARM
# 계단 하나를 유지하는 시간. τ 가 약 0.26 초이므로 5τ 이상 준다.
HOLD_SEC = 1.5
# 이 값을 넘는 수신 공백이 계단 구간에 있으면 시뮬레이터 정지로 보고 버린다.
STALL_GAP_SEC = 0.3
# 1차 지연 모델의 교차 비율. 10% 와 63.2% 두 점으로 dead time 과 τ 를 가른다.
LOW_FRAC, TAU_FRAC = 0.10, 0.632


def _cross_time(samples: list[tuple[float, float]], start: float, final: float,
                frac: float) -> Optional[float]:
    """응답이 목표 변화량의 `frac` 을 처음 넘는 시각을 선형 보간으로 구한다."""
    threshold = start + (final - start) * frac
    rising = final > start
    for i in range(1, len(samples)):
        (t0, v0), (t1, v1) = samples[i - 1], samples[i]
        if (rising and v1 >= threshold) or (not rising and v1 <= threshold):
            if v1 == v0:
                return t1
            return t0 + (t1 - t0) * (threshold - v0) / (v1 - v0)
    return None


def _analyze(samples: list[tuple[float, float]], cmd_time: float,
             start: float) -> Optional[dict[str, float]]:
    """계단 하나에서 dead time 과 τ 를 뽑는다. 정지가 끼었으면 버린다."""
    if len(samples) < 5:
        return None
    gaps = [samples[i + 1][0] - samples[i][0] for i in range(len(samples) - 1)]
    if max(gaps) > STALL_GAP_SEC:
        return None                       # 시뮬레이터 정지 구간 — 응답이 아니다
    final = samples[-1][1]
    if abs(final - start) < 1e-4:
        return None
    t10 = _cross_time(samples, start, final, LOW_FRAC)
    t63 = _cross_time(samples, start, final, TAU_FRAC)
    if t10 is None or t63 is None or t63 <= t10:
        return None
    # 1차 지연에서 t10 = dead + 0.105*tau, t63 = dead + tau 이므로 두 식을 푼다.
    tau = (t63 - t10) / 0.895
    dead = (t10 - cmd_time) - 0.105 * tau
    return {'dead_sec': dead, 'tau_sec': tau, 'reached': final - start}


def main() -> None:
    skip_ready = fb_ready.take_flag('--no-ready')
    keep_grasp = fb_ready.take_flag('--keep-grasp')
    ramp_sec = fb_ready.DEFAULT_RAMP_SEC
    if '--ramp' in sys.argv:
        i = sys.argv.index('--ramp')
        ramp_sec = float(sys.argv[i + 1])
        del sys.argv[i:i + 2]
    delta = float(sys.argv[1]) if len(sys.argv) > 1 else 0.05
    count = int(sys.argv[2]) if len(sys.argv) > 2 else 12
    joint_no = int(sys.argv[3]) if len(sys.argv) > 3 else 7
    idx = joint_no - 1

    rclpy.init()
    node = rclpy.create_node('fb_step')
    cur, pub = fb_ready.attach(node)
    if skip_ready:
        fb_ready.wait_for_state(node, cur)
        print('[ready] 건너뜀 (--no-ready)')
    else:
        fb_ready.goto_ready(node, cur, pub, ramp_sec=ramp_sec, open_gripper=not keep_grasp)

    base = fb_ready.wait_for_state(node, cur)
    joint = ARM[idx]
    print(f'\n계단 응답 — {joint}, ±{delta} rad, {count}회, 유지 {HOLD_SEC}s')
    print(f'기준 자세값 {base[idx]:+.4f} rad')

    msg = JointState()
    msg.name = list(ARM)
    results: list[dict[str, float]] = []
    discarded = 0
    for k in range(count):
        # 번갈아 밀고 당긴다. 한 방향만 주면 한계에 걸려도 알아채지 못한다.
        sign = 1.0 if k % 2 == 0 else -1.0
        target = list(base)
        target[idx] = base[idx] + sign * delta
        start_value = float(cur[joint])
        msg.position = target
        samples: list[tuple[float, float]] = []
        cmd_time = time.time()
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        nxt = cmd_time
        while rclpy.ok() and time.time() - cmd_time < HOLD_SEC:
            if time.time() >= nxt:
                msg.header.stamp = node.get_clock().now().to_msg()
                pub.publish(msg)
                nxt += fb_ready.COMMAND_PERIOD_SEC
            rclpy.spin_once(node, timeout_sec=0.002)
            samples.append((time.time(), float(cur[joint])))
        fit = _analyze(samples, cmd_time, start_value)
        if fit is None:
            discarded += 1
            print(f'  {k + 1:2d}  {"+" if sign > 0 else "-"}  버림 (정지 구간 또는 응답 없음)')
            continue
        results.append(fit)
        print(f'  {k + 1:2d}  {"+" if sign > 0 else "-"}  dead {fit["dead_sec"] * 1000:6.1f} ms  '
              f'tau {fit["tau_sec"] * 1000:6.1f} ms  도달 {fit["reached"]:+.4f} rad')

    # 원위치 — 진단이 자세를 바꾸면 다음 측정의 기준이 흔들린다.
    msg.position = list(base)
    end = time.time() + 2.0
    nxt = time.time()
    while rclpy.ok() and time.time() < end:
        if time.time() >= nxt:
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            nxt += fb_ready.COMMAND_PERIOD_SEC
        rclpy.spin_once(node, timeout_sec=0.002)

    print()
    if not results:
        print('유효한 계단이 없다 — 정지 구간에 전부 걸렸거나 관절이 한계에 있다')
    else:
        dead = [r['dead_sec'] * 1000 for r in results]
        tau = [r['tau_sec'] * 1000 for r in results]
        print(f'유효 {len(results)}회 / 버림 {discarded}회')
        print(f'dead time  중앙 {statistics.median(dead):6.1f} ms  '
              f'범위 {min(dead):.1f} ~ {max(dead):.1f}   기준선 43 ~ 58 ms')
        print(f'시정수 tau 중앙 {statistics.median(tau):6.1f} ms  '
              f'범위 {min(tau):.1f} ~ {max(tau):.1f}   기준선 252 ~ 259 ms')
        print('* 보고 주기 30 Hz = 표본 간격 33 ms — dead time 은 이보다 잘 못 가른다.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
