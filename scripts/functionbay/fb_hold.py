#!/usr/bin/env python3
"""고정 목표를 계속 명령하면서 실제값이 수렴하는지 추적해 **정착 오차**를 잰다.

펑션베이 시뮬레이터는 중력 보상이 없는 유한 강성 위치 제어라, 목표를 정확히
유지하지 못하고 중력 토크에 비례하는 정상상태 오차가 남는다. 이 스크립트는
그 오차가 수렴하는지(정상상태) 발산하는지(불안정) 가른다.

현재 자세를 그대로 목표로 주므로, **팔이 처진 만큼 더 처진다.** 반복 실행하면
누적되어 자기충돌 자세까지 갈 수 있다 — `fb_recover.py` 로 되돌린다.

    ./fb_hold.py 15        # 15초간 유지하며 1.5초 간격으로 추적

인자: [유지 시간(초), 기본 15]
"""
from __future__ import annotations

import sys
import time

import rclpy
from sensor_msgs.msg import JointState

ARM = [f'panda_joint{i}' for i in range(1, 8)]
REPORT_TOPIC = '/output/panda_joint'
COMMAND_TOPIC = '/input/panda_joint'
RATE_HZ = 50.0
# 시뮬레이터 수신 주기(50 Hz) 에 맞춘다 (문서 §10.3).
COMMAND_PERIOD_SEC = 1.0 / RATE_HZ
SAMPLE_INTERVAL_SEC = 1.5


def main():
    hold_sec = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0

    rclpy.init()
    node = rclpy.create_node('fb_hold')
    cur: dict[str, float] = {}
    node.create_subscription(JointState, REPORT_TOPIC,
                             lambda m: cur.update(zip(m.name or ARM, m.position)), 10)
    pub = node.create_publisher(JointState, COMMAND_TOPIC, 10)

    deadline = time.time() + 5.0
    while rclpy.ok() and time.time() < deadline and not cur:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not cur:
        raise SystemExit(f'no JointState received on {REPORT_TOPIC}')

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
