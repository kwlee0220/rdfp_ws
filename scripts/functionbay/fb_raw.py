#!/usr/bin/env python3
"""시뮬레이터가 관절 명령에 반응하는지 **최소 구성**으로 확인한다.

MoveIt 도 MoveGroupClient 도 쓰지 않고 손으로 만든 ``sensor_msgs/JointState`` 를
``/input/panda_joint`` 로 직접 민다. 반응이 없으면 원인이 우리 스택 바깥
(시뮬레이터 쪽)에 있다는 뜻이므로, 문제를 가를 때 가장 먼저 돌린다.

시뮬레이터가 `name` 을 요구하는지 무시하는지도 함께 가른다.

    ./fb_raw.py named  0.30 4.0    # name 채움
    ./fb_raw.py noname 0.30 4.0    # name 비움 (배열 순서 규약만)

인자: <variant> <joint7 에 더할 값(rad)> <발행 지속시간(초)>
"""
from __future__ import annotations

import sys
import time

import rclpy
from sensor_msgs.msg import JointState

ARM = [f'panda_joint{i}' for i in range(1, 8)]
REPORT_TOPIC = '/output/panda_joint'
COMMAND_TOPIC = '/input/panda_joint'
# 시뮬레이터 수신 주기(50 Hz) 에 맞춘다.
COMMAND_PERIOD_SEC = 0.02


def read_current(node, timeout_sec=5.0):
    """`/output/panda_joint` 첫 메시지에서 현재 관절값을 읽는다."""
    got: dict[str, float] = {}
    sub = node.create_subscription(
        JointState, REPORT_TOPIC,
        lambda m: got.update(zip(m.name or ARM, m.position)), 10)
    deadline = time.time() + timeout_sec
    while rclpy.ok() and time.time() < deadline and not got:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if not got:
        raise RuntimeError(f'no JointState received on {REPORT_TOPIC}')
    return [float(got[j]) for j in ARM]


def main():
    variant, delta, seconds = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    if variant not in ('named', 'noname'):
        raise SystemExit("variant must be 'named' or 'noname'")

    rclpy.init()
    node = rclpy.create_node('fb_raw')
    pub = node.create_publisher(JointState, COMMAND_TOPIC, 10)

    start = read_current(node)
    target = list(start)
    target[6] = start[6] + delta
    print('start :', [round(v, 4) for v in start], flush=True)
    print('target:', [round(v, 4) for v in target], flush=True)

    msg = JointState()
    # name 을 비우는 쪽이 시뮬레이터의 원래 규약(배열 순서)이다. 양쪽을 모두
    # 시도해 시뮬레이터가 어느 형식을 받는지 가른다.
    msg.name = list(ARM) if variant == 'named' else []
    msg.position = target

    # 발행 주기를 명시적으로 고정한다. `spin_once` 의 timeout 을 주기로 삼으면
    # 콜백이 일찍 반환될 때 루프가 훨씬 빨리 돌아 의도치 않게 수백 Hz 가 된다.
    deadline = time.time() + seconds
    next_publish = time.time()
    published = 0
    while rclpy.ok() and time.time() < deadline:
        now = time.time()
        if now >= next_publish:
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            published += 1
            next_publish += COMMAND_PERIOD_SEC
        rclpy.spin_once(node, timeout_sec=0.002)
    print(f'published {published} msg(s) as variant={variant}', flush=True)

    # 명령이 끊긴 뒤 정착할 시간을 준다.
    deadline = time.time() + 3.0
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    final = read_current(node)
    print('final :', [round(v, 4) for v in final], flush=True)
    print(f'joint7 moved by {final[6] - start[6]:+.5f} rad (requested {delta:+.5f})', flush=True)

    node.destroy_node()
    rclpy.shutdown()


main()
