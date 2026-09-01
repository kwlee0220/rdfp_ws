#!/usr/bin/env python3
"""시뮬레이터가 관절 명령에 반응하는지 **최소 구성**으로 확인한다.

MoveIt 도 MoveGroupClient 도 쓰지 않고 손으로 만든 ``sensor_msgs/JointState`` 를
``/input/panda_joint`` 로 직접 민다. 반응이 없으면 원인이 우리 스택 바깥
(시뮬레이터 쪽)에 있다는 뜻이므로, 문제를 가를 때 가장 먼저 돌린다.

**관절 하나만 보고 판정하지 않는다.** 예전 판( joint7 만 항상 + 방향으로 밀었다)은
그 관절이 한계 근처에 있으면 "시뮬레이터 전체 무반응" 으로 오진했다. 실제로
시뮬레이터의 joint7 은 URDF 한계(±2.9671)보다 훨씬 안쪽인 **약 +1.64 rad** 에서
막히며, 그 상태에서도 나머지 관절은 정상 동작한다.

그래서 이 스크립트는 **관절마다 여유가 있는 방향을 골라** 하나씩 시험하고, 매번
원위치로 되돌린다.

    ./fb_raw.py                 # 7관절 전부, 기본 delta 0.20
    ./fb_raw.py 0.30            # delta 지정
    ./fb_raw.py 0.20 named      # name 필드를 채워서 (기본은 비움)

인자: [delta(rad), 기본 0.20] [named|noname, 기본 noname]
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
DRIVE_SEC = 3.0
SETTLE_SEC = 2.0
# 반응으로 인정할 최소 이동 비율.
RESPOND_RATIO = 0.5

# moveit_resources_panda_description 의 URDF 한계.
LIMITS = {
    'panda_joint1': (-2.9671, 2.9671), 'panda_joint2': (-1.8326, 1.8326),
    'panda_joint3': (-2.9671, 2.9671), 'panda_joint4': (-3.1416, 0.0873),
    'panda_joint5': (-2.9671, 2.9671), 'panda_joint6': (-0.0873, 3.8223),
    'panda_joint7': (-2.9671, 2.9671),
}


def read_current(node, cur, timeout_sec=5.0):
    deadline = time.time() + timeout_sec
    while rclpy.ok() and time.time() < deadline and not cur:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not cur:
        raise SystemExit(f'no JointState received on {REPORT_TOPIC}')
    return [float(cur[j]) for j in ARM]


def drive(node, pub, msg, positions, seconds):
    """고정 목표를 50 Hz 로 지정 시간 동안 민다."""
    msg.position = list(positions)
    started = time.time()
    next_publish = started
    while rclpy.ok() and time.time() - started < seconds:
        if time.time() >= next_publish:
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            next_publish += COMMAND_PERIOD_SEC
        rclpy.spin_once(node, timeout_sec=0.002)


def main():
    delta = float(sys.argv[1]) if len(sys.argv) > 1 else 0.20
    use_name = len(sys.argv) > 2 and sys.argv[2] == 'named'

    rclpy.init()
    node = rclpy.create_node('fb_raw')
    cur: dict[str, float] = {}
    node.create_subscription(JointState, REPORT_TOPIC,
                             lambda m: cur.update(zip(m.name or ARM, m.position)), 50)
    pub = node.create_publisher(JointState, COMMAND_TOPIC, 10)
    read_current(node, cur)

    msg = JointState()
    msg.name = list(ARM) if use_name else []

    print(f'delta={delta} rad, name={"채움" if use_name else "비움"}')
    print(f'{"관절":>14} {"방향":>5} {"이전":>9} {"목표":>9} {"이후":>9} {"이동률":>7}  판정')
    responded = 0
    for idx, joint in enumerate(ARM):
        base = read_current(node, cur)
        lo, hi = LIMITS[joint]
        # 한계까지 여유가 큰 쪽으로 민다.
        sign = 1.0 if (hi - base[idx]) >= (base[idx] - lo) else -1.0
        target = list(base)
        target[idx] = base[idx] + sign * delta

        drive(node, pub, msg, target, DRIVE_SEC)
        settle_end = time.time() + SETTLE_SEC
        while rclpy.ok() and time.time() < settle_end:
            rclpy.spin_once(node, timeout_sec=0.05)
        after = float(cur[joint])
        ratio = (after - base[idx]) / (sign * delta)
        ok = ratio >= RESPOND_RATIO
        responded += ok
        print(f'{joint:>14} {"+" if sign > 0 else "-":>5} {base[idx]:9.4f} '
              f'{target[idx]:9.4f} {after:9.4f} {ratio*100:6.0f}%  '
              f'{"반응" if ok else "**무반응**"}')

        drive(node, pub, msg, base, DRIVE_SEC)   # 원위치

    print(f'\n반응한 관절 {responded}/7 — ', end='')
    if responded == 7:
        print('시뮬레이터 정상')
    elif responded == 0:
        print('**시뮬레이터가 명령을 받지 않는다**')
    else:
        print('**일부 관절만 반응** — 해당 관절의 한계·간섭을 먼저 의심한다')

    node.destroy_node()
    rclpy.shutdown()


main()
