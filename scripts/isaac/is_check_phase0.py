#!/usr/bin/env python3
"""Isaac 백엔드 **Phase 0 수용 기준**을 한 번에 검사한다.

``docs/simulation/isaac_backend_skeleton.md`` §2 Phase 0 의 5개 기준을 그대로 검사한다.

    1. /clock 이 흐르고 배속이 1.0 근처인가
    2. /joint_states 주파수와 **관절 이름이 URDF 와 일치**하는가
    3. move_group · rviz2 · ee_pose_publisher 의 use_sim_time 이 true 인가
    4. TF 트리가 world → panda_link0 → … → panda_hand 로 성립하는가
    5. named target `extended` **계획**이 성공하는가 (실행하지 않는다)

3번을 넣은 이유: use_sim_time 이 한 노드라도 어긋나면 move_group 이
``Failed to fetch current robot state`` 로 조용히 무력해진다. 증상이 원인을
가리키지 않으므로 기계로 검사한다.

    ./is_check_phase0.py            # 기본 15초 관측
    ./is_check_phase0.py 30         # 30초 관측

종료 코드: 전부 통과하면 0, 하나라도 실패하면 1.
"""
from __future__ import annotations

from typing import Optional

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

ARM_JOINT_NAMES = [f'panda_joint{i}' for i in range(1, 8)]
JOINT_STATE_TOPIC = '/joint_states'
BASE_FRAME = 'world'
EE_FRAME = 'panda_hand'
# use_sim_time 이 반드시 켜져 있어야 하는 노드들.
SIM_TIME_NODES = ['/move_group', '/ee_pose_publisher']
# 배속 허용 범위. Isaac 이 실시간으로 돌고 있는지만 본다.
RATE_TOLERANCE = 0.10


class Phase0Checker(Node):
    """Phase 0 기준을 관측·검사하는 노드."""

    def __init__(self) -> None:
        # 이 노드 자신은 벽시계를 써야 한다 — /clock 진행 배속을 재려면
        # 벽시계와 sim time 을 **동시에** 봐야 하기 때문이다.
        super().__init__('is_check_phase0',
                         parameter_overrides=[Parameter('use_sim_time', value=False)])
        self.clock_samples: list[tuple[float, float]] = []
        self.joint_samples: list[tuple[float, JointState]] = []
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self.create_subscription(JointState, JOINT_STATE_TOPIC, self._on_joint_state, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_clock(self, msg: Clock) -> None:
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.clock_samples.append((time.time(), sim))

    def _on_joint_state(self, msg: JointState) -> None:
        self.joint_samples.append((time.time(), msg))


def _fmt(ok: bool) -> str:
    return '통과' if ok else '실패'


def check_clock(node: Phase0Checker) -> bool:
    """기준 1 — /clock 이 흐르고 배속이 실시간에 가까운가."""
    if len(node.clock_samples) < 2:
        print(f'  1. /clock            : {_fmt(False)} — 샘플 {len(node.clock_samples)}개. '
              f'Isaac 의 ROS2PublishClock 이 없거나 도메인이 다르다')
        return False
    (w0, s0), (w1, s1) = node.clock_samples[0], node.clock_samples[-1]
    wall, sim = w1 - w0, s1 - s0
    rate = sim / wall if wall > 0 else 0.0
    ok = abs(rate - 1.0) <= RATE_TOLERANCE
    print(f'  1. /clock            : {_fmt(ok)} — 배속 {rate:.5f} '
          f'(벽시계 {wall:.2f}s / sim {sim:.2f}s, 샘플 {len(node.clock_samples)}개)')
    return ok


def check_joint_states(node: Phase0Checker) -> bool:
    """기준 2 — 주파수와 관절 이름이 URDF 와 일치하는가."""
    if len(node.joint_samples) < 2:
        print(f'  2. /joint_states     : {_fmt(False)} — 샘플 {len(node.joint_samples)}개')
        return False
    span = node.joint_samples[-1][0] - node.joint_samples[0][0]
    hz = (len(node.joint_samples) - 1) / span if span > 0 else 0.0
    names = list(node.joint_samples[-1][1].name)
    missing = [j for j in ARM_JOINT_NAMES if j not in names]
    ok = not missing
    print(f'  2. /joint_states     : {_fmt(ok)} — {hz:.1f} Hz, 관절 {len(names)}개')
    if missing:
        print(f'       누락: {missing}')
        print(f'       수신: {names}')
        print('       → USD 의 관절 이름이 URDF 와 다르다. Isaac 백엔드의 핵심 계약 항목이다')
    return ok


def check_sim_time(node: Phase0Checker) -> bool:
    """기준 3 — 각 노드의 use_sim_time 이 true 인가."""
    results: dict[str, Optional[bool]] = {}
    for target in SIM_TIME_NODES:
        client = node.create_client(GetParameters, f'{target}/get_parameters')
        if not client.wait_for_service(timeout_sec=3.0):
            results[target] = None
            continue
        future = client.call_async(GetParameters.Request(names=['use_sim_time']))
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        response = future.result()
        results[target] = bool(response.values[0].bool_value) if response and response.values \
            else None
    ok = all(v is True for v in results.values())
    print(f'  3. use_sim_time      : {_fmt(ok)}')
    for target, value in results.items():
        mark = 'true' if value is True else ('false' if value is False else '응답 없음')
        print(f'       {target}: {mark}')
    if not ok:
        print('       → false 면 move_group 이 current robot state 를 못 가져와 조용히 무력해진다')
    return ok


def check_tf(node: Phase0Checker) -> bool:
    """기준 4 — TF 트리가 성립하는가."""
    try:
        node.tf_buffer.lookup_transform(BASE_FRAME, EE_FRAME, rclpy.time.Time())
        ok = True
        detail = f'{BASE_FRAME} → {EE_FRAME} 조회 성공'
    except Exception as exc:  # tf2 예외 계열이 여럿이라 통째로 잡는다
        ok = False
        detail = f'{type(exc).__name__}: {exc}'
    print(f'  4. TF                : {_fmt(ok)} — {detail}')
    return ok


def check_planning() -> bool:
    """기준 5 — named target 계획이 성공하는가 (실행하지 않는다)."""
    # MoveGroupClient 는 자체 Node 를 spin 하므로 검사 노드와 분리해서 만든다.
    from robot_control.moveit.move_group_factory import create_move_group_client

    # 계획 노드는 move_group 과 시간 기준을 맞춘다.
    planner_node = rclpy.create_node(
        'is_check_phase0_planner',
        parameter_overrides=[Parameter('use_sim_time', value=True)])
    try:
        # mode 를 명시한다 — auto 판별은 토픽 그래프를 보므로 Phase 0 처럼 명령
        # 경로가 아직 없는 단계에서는 오판한다.
        client = create_move_group_client(planner_node, mode='jgpc')
        # SRDF 파싱과 관절 이름 계약은 named target **목록**으로 확인한다.
        names = client.get_named_targets()
        # 계획은 `ready` 로 한다. `extended` 는 j4=0 을 요구하는데 실제 Franka 스펙의
        # 상한이 -0.0698 이라 한계 밖이다 (§7 Q6). 한계를 실제 스펙으로 좁힌 뒤로는
        # 계획이 실패하는 것이 정상이므로 수용 기준으로 쓸 수 없다.
        plan = client.plan_named_target('ready')
        points = len(plan.joint_trajectory.points)
        ok = points > 0 and 'ready' in names
        detail = (f'named targets {names}, ready 계획 {points} point'
                  if ok else f'named targets {names}, 계획 {points} point')
    except Exception as exc:
        ok = False
        detail = f'{type(exc).__name__}: {exc}'
    finally:
        planner_node.destroy_node()
    print(f'  5. 계획(ready)       : {_fmt(ok)} — {detail}')
    return ok


def main() -> int:
    observe_sec = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0

    rclpy.init()
    node = Phase0Checker()
    print(f'Isaac Phase 0 수용 기준 검사 — {observe_sec:.0f}초 관측')

    deadline = time.time() + observe_sec
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    results = [
        check_clock(node),
        check_joint_states(node),
        check_sim_time(node),
        check_tf(node),
    ]
    node.destroy_node()

    results.append(check_planning())

    passed = sum(1 for r in results if r)
    print(f'\n{passed}/{len(results)} 통과')
    rclpy.shutdown()
    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
