#!/usr/bin/env python3
"""고정 목표를 계속 명령하면서 실제값이 수렴하는지 추적해 **정착 오차**를 잰다.

펑션베이 시뮬레이터는 중력 보상이 없는 유한 강성 위치 제어라, 목표를 정확히
유지하지 못하고 중력 토크에 비례하는 정상상태 오차가 남는다. 이 스크립트는
그 오차가 수렴하는지(정상상태) 발산하는지(불안정) 가른다.

현재 자세를 그대로 목표로 주므로, **팔이 처진 만큼 더 처진다.** 그래서 측정 전에
반드시 `ready` 로 옮긴다 — 그러지 않으면 회차마다 시작 자세가 달라 기준선과 비교할
수 없고, 반복하면 처짐이 누적되어 자기충돌 자세까지 간다. 이동은 `fb_ready` 가 맡는다.

    ./fb_hold.py 15             # ready 로 옮긴 뒤 15초간 유지하며 추적
    ./fb_hold.py 15 --no-ready  # 현재 자세에서 그대로 (자세별 반복 측정용)

인자: [유지 시간(초), 기본 15] [--no-ready] [--ramp <초>, 기본 5]

**`--no-ready` 는 B-4(자세별 처짐)처럼 의도적으로 다른 자세에서 재는 경우에만 쓴다.**
"""
from __future__ import annotations

import sys
import time

import fb_ready
import rclpy
from sensor_msgs.msg import JointState

ARM = fb_ready.ARM
REPORT_TOPIC = fb_ready.REPORT_TOPIC
COMMAND_TOPIC = fb_ready.COMMAND_TOPIC
RATE_HZ = fb_ready.RATE_HZ
COMMAND_PERIOD_SEC = fb_ready.COMMAND_PERIOD_SEC
SAMPLE_INTERVAL_SEC = 1.5


def main():
    skip_ready = fb_ready.take_flag('--no-ready')
    keep_grasp = fb_ready.take_flag('--keep-grasp')
    ramp_sec = fb_ready.DEFAULT_RAMP_SEC
    if '--ramp' in sys.argv:
        i = sys.argv.index('--ramp')
        ramp_sec = float(sys.argv[i + 1])
        del sys.argv[i:i + 2]
    hold_sec = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0

    rclpy.init()
    node = rclpy.create_node('fb_hold')
    cur, pub = fb_ready.attach(node)
    if skip_ready:
        fb_ready.wait_for_state(node, cur)
        print('[ready] 건너뜀 (--no-ready) — 현재 자세에서 측정한다')
    else:
        fb_ready.goto_ready(node, cur, pub, ramp_sec=ramp_sec, open_gripper=not keep_grasp)

    target = [float(cur[j]) for j in ARM]
    print('고정 목표 (현재 자세):', [round(v, 4) for v in target])
    print(f'{"t(s)":>5} {"j2":>9} {"j4":>9} {"j6":>9}   {"j2 오차":>9}')

    msg = JointState()
    msg.name = list(ARM)
    msg.position = target

    started = time.time()
    next_sample = 0.0
    next_publish = started
    while rclpy.ok() and time.time() - started < hold_sec:
        now = time.time()
        if now >= next_publish:
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            next_publish += COMMAND_PERIOD_SEC
        rclpy.spin_once(node, timeout_sec=0.002)
        elapsed = time.time() - started
        if elapsed >= next_sample:
            act = [float(cur[j]) for j in ARM]
            print(f'{elapsed:5.1f} {act[1]:9.4f} {act[3]:9.4f} {act[5]:9.4f} '
                  f'  {act[1] - target[1]:+9.4f}')
            next_sample += SAMPLE_INTERVAL_SEC

    final = [float(cur[j]) for j in ARM]
    print('관절별 정착 오차 (실제 - 목표):')
    for i, joint in enumerate(ARM):
        print(f'  {joint}: {final[i] - target[i]:+.4f} rad')

    node.destroy_node()
    rclpy.shutdown()


main()
