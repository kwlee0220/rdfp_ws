#!/usr/bin/env python3
"""Isaac ↔ 스택 사이에 **무엇이 오고 있는지** 한 번에 본다.

`ros2 topic list` / `ros2 topic hz` 는 daemon 이 꼬이거나 토픽이 없을 때 끝나지
않는 경우가 있다. 이 스크립트는 CLI 를 쓰지 않고 rclpy 로 직접 재므로 **반드시
정해진 시간 안에 끝난다.**

    ./is_topics.py            # 기본 6초 관측
    ./is_topics.py 10

무엇을 가르는가 — 넷 중 하나로 원인이 좁혀진다.

    자체시험 실패      **WSL 안의 DDS 자체가 안 된다** (wsl --shutdown)
    아무것도 없음      Isaac 이 안 보인다 (브리지 또는 Windows<->WSL2 네트워크)
    /clock 만 옴       PublishJointState 노드만 고장 (targetPrim 재바인딩)
    전부 오는데 실패   스택 쪽 문제 (QoS·타이밍)

**자체시험**은 이 프로세스 안에서 노드 둘을 만들어 서로 통신시킨다. rclpy 는
같은 프로세스라도 기본적으로 DDS 를 거치므로, 이것이 실패하면 Isaac 과 무관하게
WSL 쪽 DDS 가 깨진 것이다 — 그때는 Windows 를 아무리 봐도 소용없다.
"""
from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

# 타입을 **미리 못박는다.** 그래프에서 타입을 조회해 구독하는 방식은 여기서
# 쓸 수 없다 — Isaac 의 발행 노드는 구독자가 생기기 전에는 publisher 를 만들지
# 않는 경우가 있어, "안 보이니 구독 안 함 → 구독이 없으니 발행 안 함" 교착이
# 생긴다. 실제로 그 교착 때문에 /clock 이 0건으로 보였다.
# **Isaac 의 배관**을 보는 것이므로 `/isaac_joint_states` 를 본다. `/joint_states` 는
# `joint_state_broadcaster` 가 소유하므로, 그것을 보면 스택이 안 떠 있을 때 멀쩡한
# 배관을 고장으로 오진한다.
JOINT_STATE_TOPIC = '/isaac_joint_states'

WATCH = {
    '/clock': 'rosgraph_msgs/msg/Clock',
    JOINT_STATE_TOPIC: 'sensor_msgs/msg/JointState',
    '/tf': 'tf2_msgs/msg/TFMessage',
    '/scene/objects': 'rdfp_msgs/msg/SceneObjects',
}


def selftest(timeout_sec: float = 4.0) -> bool:
    """WSL 안에서 노드끼리 통신이 되는지 본다 (Isaac 과 무관한 하한 검사)."""
    from std_msgs.msg import String

    pub_node = Node('is_topics_selftest_pub',
                    parameter_overrides=[Parameter('use_sim_time', value=False)])
    sub_node = Node('is_topics_selftest_sub',
                    parameter_overrides=[Parameter('use_sim_time', value=False)])
    received = {'n': 0}
    sub_node.create_subscription(String, '/is_topics_selftest',
                                 lambda _m: received.__setitem__('n', received['n'] + 1), 10)
    publisher = pub_node.create_publisher(String, '/is_topics_selftest', 10)

    msg = String()
    msg.data = 'ping'
    started = time.time()
    while time.time() - started < timeout_sec and received['n'] == 0:
        publisher.publish(msg)
        rclpy.spin_once(sub_node, timeout_sec=0.05)

    ok = received['n'] > 0
    pub_node.destroy_node()
    sub_node.destroy_node()
    return ok


def main() -> int:
    observe_sec = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0

    rclpy.init()
    node = Node('is_topics', parameter_overrides=[Parameter('use_sim_time', value=False)])

    self_ok = selftest()
    print(f'자체시험(WSL 내부 DDS): {"통과" if self_ok else "실패"}')
    if not self_ok:
        print('  → WSL 안에서 노드끼리도 통신이 안 된다. Isaac 과 무관한 문제다.')
        print('     `wsl --shutdown` 후 재접속한다.')
    print()

    counts: dict[str, int] = {name: 0 for name in WATCH}
    from rosidl_runtime_py.utilities import get_message

    subs = []
    for name, type_name in WATCH.items():
        try:
            msg_type = get_message(type_name)
        except Exception as exc:
            print(f'  {name}: 타입 {type_name} 을 불러오지 못했다 ({type(exc).__name__})')
            continue
        subs.append(node.create_subscription(
            msg_type, name, (lambda n: (lambda _m: counts.__setitem__(n, counts[n] + 1)))(name),
            10))

    started = time.time()
    while rclpy.ok() and time.time() - started < observe_sec:
        rclpy.spin_once(node, timeout_sec=0.05)
    elapsed = time.time() - started

    visible = sorted(name for name, _ in node.get_topic_names_and_types())
    print(f'관측 {elapsed:.1f}초')
    print(f'보이는 토픽 ({len(visible)}개): {" ".join(visible)}')
    print()
    for name in WATCH:
        hz = counts[name] / elapsed if elapsed > 0 else 0.0
        mark = '있음' if counts[name] else '없음'
        print(f'  {name:<18} {mark}  {counts[name]:5d}건  {hz:6.1f} Hz')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
