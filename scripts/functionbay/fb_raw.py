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

**측정 전에 `ready` 로 옮긴다.** 관절마다 여유가 있는 방향을 고르는 로직이 시작
자세에 의존하고(한계까지의 여유로 부호를 정한다), 작업면에 물체가 있으면 임의 자세에서
0.2 rad 씩 흔드는 것이 위험하기 때문이다. 이동은 `fb_ready` 가 맡는다.

    ./fb_raw.py                    # ready 로 옮긴 뒤 7관절 전부, 기본 delta 0.20
    ./fb_raw.py 0.30               # delta 지정
    ./fb_raw.py 0.20 named         # name 필드를 채워서 (기본은 비움)
    ./fb_raw.py 0.20 noname --no-ready   # 현재 자세에서 그대로

인자: [delta(rad), 기본 0.20] [named|noname, 기본 noname] [--no-ready] [--ramp <초>]
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
COMMAND_PERIOD_SEC = fb_ready.COMMAND_PERIOD_SEC
DRIVE_SEC = 3.0
SETTLE_SEC = 2.0
# 반응으로 인정할 최소 이동 비율.
RESPOND_RATIO = 0.5

# 한계표는 fb_ready 가 갖는다 — fb_cmdpath 와 같은 표를 봐야 판정이 어긋나지 않는다.
LIMITS = fb_ready.LIMITS


def read_current(node, cur, timeout_sec=5.0):
    return fb_ready.wait_for_state(node, cur, timeout_sec)


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
    skip_ready = fb_ready.take_flag('--no-ready')
    keep_grasp = fb_ready.take_flag('--keep-grasp')
    ramp_sec = fb_ready.DEFAULT_RAMP_SEC
    if '--ramp' in sys.argv:
        i = sys.argv.index('--ramp')
        ramp_sec = float(sys.argv[i + 1])
        del sys.argv[i:i + 2]
    delta = float(sys.argv[1]) if len(sys.argv) > 1 else 0.20
    use_name = len(sys.argv) > 2 and sys.argv[2] == 'named'

    rclpy.init()
    node = rclpy.create_node('fb_raw')
    cur, pub = fb_ready.attach(node)
    if skip_ready:
        read_current(node, cur)
        print('[ready] 건너뜀 (--no-ready) — 현재 자세에서 측정한다')
    else:
        fb_ready.goto_ready(node, cur, pub, ramp_sec=ramp_sec, open_gripper=not keep_grasp)

    msg = JointState()
    msg.name = list(ARM) if use_name else []

    print(f'delta={delta} rad, name={"채움" if use_name else "비움"}')
    print(f'{"관절":>14} {"방향":>5} {"이전":>9} {"목표":>9} {"이후":>9} {"이동률":>7}  판정')
    responded = 0
    for idx, joint in enumerate(ARM):
        base = read_current(node, cur)
        # 한계까지 여유가 큰 쪽으로 민다.
        sign = fb_ready.free_direction(joint, base[idx])
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
