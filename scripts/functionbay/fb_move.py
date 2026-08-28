#!/usr/bin/env python3
"""`MoveGroupJgpcClient` 로 계획하고 `/input/panda_joint` 로 스트리밍한다.

펑션베이 스택의 정규 명령 경로를 그대로 태우는 스크립트다. 측정은
`fb_probe.py` 가 별도 프로세스에서 맡는다.

**단일 스레드 + 동기 메서드만 쓴다.** 별도 executor 스레드를 두면 동일 Node 를
두 곳에서 spin 하게 되어 `wait set index ... out of bounds` 로 깨진다
(MoveGroupClient_UserGuide §14.2).

    ./fb_move.py named   ready         50.0 /tmp/start.json   # 시작 자세 저장 후 이동
    ./fb_move.py restore /tmp/start.json none                 # 저장한 자세로 복귀

`publish_rate` 에 ``none`` 을 주면 궤적 원래 point 시각을 따른다 — TOTG 리샘플
때문에 ~10 Hz 가 되며, 보간하지 않는 시뮬레이터에서는 계단처럼 움직인다.
"""
from __future__ import annotations

import json
import sys
import time

import rclpy
from sensor_msgs.msg import JointState

from robot_control.moveit.move_group_factory import create_move_group_client

ARM = [f'panda_joint{i}' for i in range(1, 8)]
REPORT_TOPIC = '/output/panda_joint'
COMMAND_TOPIC = '/input/panda_joint'


def read_current(node, timeout_sec=10.0):
    """`/output/panda_joint` 에서 현재 관절값을 `{이름: 값}` 으로 읽는다."""
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
    return {k: float(v) for k, v in got.items() if k in ARM}


def main():
    mode, arg, rate_arg = sys.argv[1], sys.argv[2], sys.argv[3]
    publish_rate = None if rate_arg == 'none' else float(rate_arg)

    rclpy.init()
    node = rclpy.create_node('fb_move')
    start = read_current(node)
    print('current:', {k: round(v, 4) for k, v in sorted(start.items())}, flush=True)

    # mode='jgpc' 를 명시한다. auto 는 /panda_arm_controller/commands 를 보는데
    # 이 스택에는 없어 JTC 로 오판하고, JTC 실행 액션 서버는 존재하지 않는다.
    client = create_move_group_client(
        node, mode='jgpc',
        arm_command_topic=COMMAND_TOPIC,
        arm_command_joint_names=ARM,
        arm_command_format='joint_state')
    client.wait_until_ready(timeout_sec=30.0)
    print('client ready:', type(client).__name__, flush=True)

    started = time.time()
    if mode == 'named':
        with open(sys.argv[4], 'w') as f:
            json.dump(start, f)
        published = client.move_to_named_target_streamed(arg, publish_rate=publish_rate)
    else:
        with open(arg) as f:
            target = json.load(f)
        published = client.move_to_joints_streamed(target, publish_rate=publish_rate)
    elapsed = time.time() - started

    print(json.dumps({
        'published_points': published,
        'elapsed_sec': round(elapsed, 3),
        'publish_rate_arg': publish_rate,
        'end_pose': {k: round(v, 4) for k, v in sorted(read_current(node).items())},
    }, indent=2), flush=True)

    client.close()
    node.destroy_node()
    rclpy.shutdown()


main()
