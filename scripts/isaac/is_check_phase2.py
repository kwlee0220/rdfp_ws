#!/usr/bin/env python3
"""Isaac 백엔드 **Phase 2 수용 기준**(그리퍼)을 검사한다.

    0. 재생 상태     — Isaac 이 Play 중인가
    1. 기준 자세     — 팔을 `ready` 로 보낸다 (손가락이 테이블에 박히지 않게)
    2. finger 상태   — /joint_states 에 finger 관절 2개가 오는가
    3. 액션 서버     — /panda_hand_controller/gripper_cmd 가 살아 있는가
    4. 열기/닫기 왕복 — 액션으로 open → close → open, 폭이 실제로 바뀌는가
    5. mimic 일치    — 두 finger 가 같은 값을 유지하는가

**액션 이름을 그대로 쓴다는 점이 이 단계의 핵심이다.** mock 백엔드에서는
ros2_control 의 `GripperActionController` 가, Isaac 에서는
`gripper_action_bridge` 가 같은 이름의 서버를 연다. 상위 경로(teleop·twin·재생)는
백엔드를 몰라도 된다 — 그 사실을 여기서 검증한다.

    ./is_check_phase2.py

종료 코드: 전부 통과하면 0, 하나라도 실패하면 1.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from control_msgs.action import GripperCommand as GripperCommandAction
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

ARM_JOINT_NAMES = [f'panda_joint{i}' for i in range(1, 8)]
ARM_COMMAND_TOPIC = '/isaac/arm_command'
# 그리퍼를 시험하기 전에 팔을 여기로 보낸다. scene 에 테이블이 생긴 뒤로 **자세에 따라
# 손가락 끝이 상판 안에 박혀** 그리퍼가 물리적으로 막힌다 — 실제로 그 상태를
# "그리퍼 고장"으로 오독했다(이동 0.0018 m). `ready` 는 손이 상판 위로 뜬다.
ANCHOR_POSE = 'ready'

FINGER_JOINTS = ['panda_finger_joint1', 'panda_finger_joint2']
JOINT_STATE_TOPIC = '/joint_states'
GRIPPER_ACTION = '/panda_hand_controller/gripper_cmd'

# Panda Hand 의 손가락 하나가 움직이는 범위(m). 열림 0.04 / 닫힘 0.0.
OPEN_WIDTH = 0.04
CLOSED_WIDTH = 0.0
# 명령 후 폭이 이만큼은 바뀌어야 "실제로 움직였다"고 본다.
MIN_TRAVEL = 0.01
POSITION_TOLERANCE = 0.005
MIMIC_TOLERANCE = 0.002
DISCOVERY_TIMEOUT_SEC = 15.0
ACTION_TIMEOUT_SEC = 10.0


class Phase2Checker(Node):
    """finger 상태를 기록하고 그리퍼 액션을 부르는 검사 노드."""

    def __init__(self) -> None:
        super().__init__('is_check_phase2',
                         parameter_overrides=[Parameter('use_sim_time', value=False)])
        self.positions: dict[str, float] = {}
        self.clock_samples: list[float] = []
        self.create_subscription(JointState, JOINT_STATE_TOPIC, self._on_state, 50)
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self.action = ActionClient(self, GripperCommandAction, GRIPPER_ACTION)

    def _on_clock(self, msg: Clock) -> None:
        self.clock_samples.append(msg.clock.sec + msg.clock.nanosec * 1e-9)

    def _on_state(self, msg: JointState) -> None:
        if msg.name:
            self.positions.update(zip(msg.name, msg.position))

    def spin(self, seconds: float) -> None:
        deadline = time.time() + seconds
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def width(self) -> float:
        return self.positions.get(FINGER_JOINTS[0], float('nan'))

    def send_gripper(self, position: float) -> bool:
        """그리퍼 액션 목표를 보내고 결과를 기다린다."""
        goal = GripperCommandAction.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 20.0
        send_future = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=ACTION_TIMEOUT_SEC)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=ACTION_TIMEOUT_SEC)
        return result_future.result() is not None


def _fmt(ok: bool) -> str:
    return '통과' if ok else '실패'


def check_playback(node: Phase2Checker) -> bool:
    deadline = time.time() + DISCOVERY_TIMEOUT_SEC
    advancing = False
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.clock_samples) >= 2 and node.clock_samples[-1] > node.clock_samples[0]:
            advancing = True
            break
    print(f'  0. 재생 상태          : {_fmt(advancing)} — /clock 샘플 {len(node.clock_samples)}개')
    if not advancing:
        print('       → Isaac 이 정지 상태다. **Play(▶) 를 누른다.**')
    return advancing


def goto_anchor() -> bool:
    """팔을 기준 자세로 보낸다. 그리퍼가 scene 과 간섭하지 않는 자리를 만든다."""
    from rclpy.parameter import Parameter as RclParameter

    from robot_control.moveit.move_group_factory import create_move_group_client

    planner = rclpy.create_node(
        'is_check_phase2_planner',
        parameter_overrides=[RclParameter('use_sim_time', value=True)])
    try:
        client = create_move_group_client(
            planner, mode='jgpc', arm_command_topic=ARM_COMMAND_TOPIC,
            arm_command_joint_names=ARM_JOINT_NAMES, arm_command_format='joint_state')
        client.move_to_named_target_streamed(ANCHOR_POSE, publish_rate=50.0)
        ok, detail = True, f'{ANCHOR_POSE} 로 이동'
    except Exception as exc:
        ok, detail = False, f'{type(exc).__name__}: {exc}'
    finally:
        planner.destroy_node()
    print(f'  1. 기준 자세          : {_fmt(ok)} — {detail}')
    if not ok:
        print('       → 팔을 못 옮기면 손가락이 테이블에 박힌 채로 시험하게 된다')
    return ok


def check_finger_state(node: Phase2Checker) -> bool:
    node.spin(2.0)
    missing = [j for j in FINGER_JOINTS if j not in node.positions]
    ok = not missing
    values = '  '.join(f'{j.split("_")[-1]}:{node.positions.get(j, float("nan")):+.4f}'
                       for j in FINGER_JOINTS)
    print(f'  2. finger 상태        : {_fmt(ok)} — {values}')
    if missing:
        print(f'       누락: {missing}')
    return ok


def check_action_server(node: Phase2Checker) -> bool:
    ok = node.action.wait_for_server(timeout_sec=DISCOVERY_TIMEOUT_SEC)
    print(f'  3. 액션 서버          : {_fmt(ok)} — {GRIPPER_ACTION}')
    if not ok:
        print('       → enable_gripper:=true 로 스택을 띄웠는지 확인한다')
    return ok


def check_open_close(node: Phase2Checker) -> tuple[bool, bool]:
    """기준 3·4 — 열기/닫기 왕복과 두 finger 의 일치."""
    travels: list[float] = []
    mimic_gaps: list[float] = []

    for label, target in (('닫기', CLOSED_WIDTH), ('열기', OPEN_WIDTH),
                          ('닫기', CLOSED_WIDTH), ('열기', OPEN_WIDTH)):
        before = node.width()
        if not node.send_gripper(target):
            print(f'  4. 열기/닫기          : {_fmt(False)} — {label} 목표가 거부됐다')
            return False, False
        node.spin(1.0)
        after = node.width()
        travels.append(abs(after - before))
        mimic_gaps.append(abs(node.positions.get(FINGER_JOINTS[0], 0.0)
                              - node.positions.get(FINGER_JOINTS[1], 0.0)))

    final = node.width()
    moved = max(travels) >= MIN_TRAVEL
    at_open = abs(final - OPEN_WIDTH) <= POSITION_TOLERANCE
    travel_ok = moved and at_open
    print(f'  4. 열기/닫기 왕복     : {_fmt(travel_ok)} — 최대 이동 {max(travels):.4f} m, '
          f'최종 폭 {final:.4f} m (목표 {OPEN_WIDTH})')
    if not moved:
        print('       → 명령은 받았는데 움직이지 않았다. Isaac 그래프의 PHASE 와,')
        print('          손가락이 테이블 같은 물체에 박혀 있지 않은지 확인한다')
    elif not at_open:
        print('       → 움직이지만 목표에 정착하지 않는다. finger drive 게인을 확인한다')

    worst_gap = max(mimic_gaps)
    mimic_ok = worst_gap <= MIMIC_TOLERANCE
    print(f'  5. 두 finger 일치     : {_fmt(mimic_ok)} — 최대 차이 {worst_gap:.4f} m '
          f'(기준 ≤ {MIMIC_TOLERANCE})')
    return travel_ok, mimic_ok


def main() -> int:
    rclpy.init()
    node = Phase2Checker()
    print('Isaac Phase 2 수용 기준 검사')

    results = [check_playback(node)]
    if results[0]:
        results.append(goto_anchor())
        results.append(check_finger_state(node))
        results.append(check_action_server(node))
    if all(results):
        travel_ok, mimic_ok = check_open_close(node)
        results.extend([travel_ok, mimic_ok])
    node.destroy_node()

    passed = sum(1 for r in results if r)
    print(f'\n{passed}/{len(results)} 통과')
    rclpy.shutdown()
    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
