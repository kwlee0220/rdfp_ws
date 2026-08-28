#!/usr/bin/env python3
"""이상 자세에 빠진 팔을 `ready` 로 되돌린다 — **MoveIt 을 쓰지 않는다.**

시뮬레이터는 자기충돌을 막지 않으므로, 중력 처짐이 누적되면 Panda 모델에서
자기충돌인 자세에 도달할 수 있다. 그 상태에서는 MoveIt 이 시작 상태를 무효로
판정해 계획 자체가 실패하므로(`Motion planning start tree could not be
initialized!`) `MoveGroupClient` 로는 빠져나올 수 없다.

그래서 원시 `JointState` 를 직접 민다. 시뮬레이터가 내부 보간을 하지 않으므로
큰 도약을 한 번에 주지 않고 50 Hz 로 선형 보간한다.

    ./fb_recover.py 5.0        # 5초에 걸쳐 ready 로 램프한 뒤 정착

인자: [램프 시간(초), 기본 5]
"""
from __future__ import annotations

import sys
import time

import rclpy
from sensor_msgs.msg import JointState

ARM = [f'panda_joint{i}' for i in range(1, 8)]
# SRDF 의 panda_arm `ready` group_state 와 같은 값이다.
READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
REPORT_TOPIC = '/output/panda_joint'
COMMAND_TOPIC = '/input/panda_joint'
RATE_HZ = 50.0
# 시뮬레이터 수신 주기(50 Hz) 에 맞춘다 (문서 §10.3).
COMMAND_PERIOD_SEC = 1.0 / RATE_HZ
SETTLE_SEC = 4.0


def main():
    ramp_sec = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0

    rclpy.init()
    node = rclpy.create_node('fb_recover')
    cur: dict[str, float] = {}
    node.create_subscription(JointState, REPORT_TOPIC,
                             lambda m: cur.update(zip(m.name or ARM, m.position)), 10)
    pub = node.create_publisher(JointState, COMMAND_TOPIC, 10)

    deadline = time.time() + 5.0
    while rclpy.ok() and time.time() < deadline and not cur:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not cur:
        raise SystemExit(f'no JointState received on {REPORT_TOPIC}')

    start = [float(cur[j]) for j in ARM]
    print('start :', [round(v, 4) for v in start])
    print('target:', [round(v, 4) for v in READY])

    msg = JointState()
    msg.name = list(ARM)

    started = time.time()
    published = 0
    next_publish = started
    while rclpy.ok():
        elapsed = time.time() - started
        if elapsed > ramp_sec:
            break
        if time.time() >= next_publish:
            alpha = min(elapsed / ramp_sec, 1.0)
            msg.position = [start[i] + (READY[i] - start[i]) * alpha for i in range(7)]
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            published += 1
            next_publish += COMMAND_PERIOD_SEC
        rclpy.spin_once(node, timeout_sec=0.002)

    # 목표를 계속 유지해 정착시킨다 — 놓으면 중력으로 다시 처진다.
    msg.position = list(READY)
    deadline = time.time() + SETTLE_SEC
    next_publish = time.time()
    while rclpy.ok() and time.time() < deadline:
        if time.time() >= next_publish:
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            published += 1
            next_publish += COMMAND_PERIOD_SEC
        rclpy.spin_once(node, timeout_sec=0.002)

    final = [float(cur[j]) for j in ARM]
    print(f'published {published} msg(s) at ~{RATE_HZ:.0f} Hz')
    print('final :', [round(v, 4) for v in final])
    print('관절별 정착 오차 (실제 - 목표):')
    for i, joint in enumerate(ARM):
        print(f'  {joint}: {final[i] - READY[i]:+.4f} rad')
    print(f'최대 |오차| = {max(abs(final[i] - READY[i]) for i in range(7)):.4f} rad')

    node.destroy_node()
    rclpy.shutdown()


main()
