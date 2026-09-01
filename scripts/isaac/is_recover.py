#!/usr/bin/env python3
"""팔을 **계획 없이** 기준 자세로 되돌린다.

시작 자세가 planning scene 과 충돌하면 MoveIt 은 계획 자체를 거부한다
(`INVALID_MOTION_PLAN`, -2). 그러면 MoveIt 을 거치는 모든 경로가 막힌다 — 트윈의
팔 오퍼레이션, replay GUI 의 '위치 초기화', 텔레오퍼레이션 복귀가 전부 여기 해당한다.
**빠져나오려면 계획을 거치지 않는 경로가 필요하다.**

그리퍼를 먼저 열고(조 사이에 물체가 낀 채로 팔을 올리면 물체를 끌고 간다), 현재
자세에서 목표까지 선형 보간해 관절 명령을 직접 흘린다. 개루프이므로 도달을 확인하고
`/check_state_validity` 로 충돌이 실제로 풀렸는지 본다.

**servo 가 떠 있으면 먼저 멈춘다.** servo 가 충돌 정지 상태일 때 명령 채널에 직접
쓰면 내부 상태가 망가져 **이후 출력이 전부 NaN** 이 된다 — 그러면서 status 는
`NO_WARNING` 을 유지해서 지표만 보면 정상으로 보인다(실측으로 재현했다).

`stop_servo` 로 멈추고 `start_servo` 로 되살린다. **`pause`/`unpause` 쌍은 쓰지
않는다** — `unpause_servo` 가 성공을 반환하면서도 발행을 되살리지 못했다.

    ./is_recover.py                 # ready 로
    ./is_recover.py --target extended
    ./is_recover.py --no-gripper    # 그리퍼는 건드리지 않는다

종료 코드: 충돌이 풀리면 0, 아니면 1.
"""
from __future__ import annotations

from typing import Optional

import argparse
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from control_msgs.action import GripperCommand as GripperCommandAction
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState

ARM_JOINT_NAMES = [f'panda_joint{i}' for i in range(1, 8)]
ARM_COMMAND_TOPIC = '/isaac/arm_command'
GRIPPER_ACTION = '/panda_hand_controller/gripper_cmd'
OPEN_WIDTH = 0.04
RAMP_SEC = 3.0
RAMP_RATE = 50.0
TOLERANCE = 0.05
DISCOVERY_TIMEOUT_SEC = 15.0


class Recoverer(Node):
    def __init__(self) -> None:
        super().__init__('is_recover',
                         parameter_overrides=[Parameter('use_sim_time', value=False)])
        self.positions: dict[str, float] = {}
        self.create_subscription(JointState, '/joint_states', self._on_state, 50)
        self._arm = self.create_publisher(JointState, ARM_COMMAND_TOPIC, 10)
        self._gripper = ActionClient(self, GripperCommandAction, GRIPPER_ACTION)
        self._validity = self.create_client(GetStateValidity, '/check_state_validity')

    def _on_state(self, msg: JointState) -> None:
        if msg.name:
            self.positions.update(zip(msg.name, msg.position))

    def spin(self, seconds: float) -> None:
        deadline = time.time() + seconds
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def open_gripper(self) -> bool:
        if not self._gripper.wait_for_server(timeout_sec=DISCOVERY_TIMEOUT_SEC):
            return False
        goal = GripperCommandAction.Goal()
        goal.command.position = OPEN_WIDTH
        goal.command.max_effort = 50.0
        send = self._gripper.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=15.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=15.0)
        return True

    def stream_to(self, targets: dict) -> None:
        """MoveIt 을 거치지 않고 관절 명령을 직접 보간해 흘린다."""
        names = list(targets)
        start = {j: self.positions.get(j, 0.0) for j in names}
        steps = max(1, int(RAMP_SEC * RAMP_RATE))
        period = 1.0 / RAMP_RATE
        for i in range(steps + 1):
            ratio = i / steps
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = names
            msg.position = [start[j] + (targets[j] - start[j]) * ratio for j in names]
            self._arm.publish(msg)
            deadline = time.time() + period
            while rclpy.ok() and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.005)

    def state_is_valid(self) -> tuple[Optional[bool], list]:
        if not self._validity.wait_for_service(timeout_sec=5.0):
            return None, []
        request = GetStateValidity.Request()
        request.group_name = 'panda_arm'
        state = RobotState()
        state.joint_state.name = list(self.positions)
        state.joint_state.position = [self.positions[j] for j in self.positions]
        state.is_diff = False
        request.robot_state = state
        future = self._validity.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None:
            return None, []
        return response.valid, [f'{c.contact_body_1}<->{c.contact_body_2}'
                                for c in response.contacts]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', default='ready', help='SRDF 의 group_state 이름')
    parser.add_argument('--no-gripper', action='store_true', help='그리퍼를 건드리지 않는다')
    args = parser.parse_args()

    from robot_control.moveit.move_group_factory import create_move_group_client
    from robot_control.moveit.servo_client import ServoClient

    rclpy.init()
    node = Recoverer()
    planner = rclpy.create_node('is_recover_planner',
                                parameter_overrides=[Parameter('use_sim_time', value=True)])
    try:
        node.spin(2.0)
        before, contacts = node.state_is_valid()
        print(f'현재 자세 충돌없음: {before}')
        if contacts:
            print(f'  접촉: {", ".join(contacts[:4])}')

        client = create_move_group_client(
            planner, mode='jgpc', arm_command_topic=ARM_COMMAND_TOPIC,
            arm_command_joint_names=ARM_JOINT_NAMES, arm_command_format='joint_state')
        # SRDF 를 출처로 삼는다 — 값을 여기 적으면 출처가 둘이 된다.
        targets = client._get_named_state_joint_values(args.target,
                                                       timeout=DISCOVERY_TIMEOUT_SEC)

        if not args.no_gripper:
            # 먼저 연다. 조 사이에 물체가 낀 채로 팔을 올리면 물체를 끌고 간다.
            print(f'그리퍼 열기: {"성공" if node.open_gripper() else "실패"}')
            node.spin(1.0)

        # **servo 를 먼저 멈춘다.** 활성인 채로 명령 채널에 직접 쓰면, servo 가
        # 충돌 정지 상태였을 때 이후 출력이 전부 NaN 이 된다 — 그러면서 status 는
        # `NO_WARNING` 을 유지해 지표만으로는 정상으로 보인다(실측 재현).
        #
        # **`pause`/`unpause` 가 아니라 `stop`/`start` 를 쓴다.** `unpause_servo` 는
        # 성공을 반환하면서도 발행을 되살리지 못했다 — 그 쌍을 쓰면 복구할 때마다
        # servo 가 조용히 죽는다. `start_servo` 만이 다시 내보내게 한다(실측).
        servo = ServoClient.create(planner, '/servo_node')
        stopped = False
        if servo.wait_for_services_ready(timeout_sec=2.0):
            ok, message = servo.stop()
            stopped = ok
            print(f'servo 정지: {"성공" if ok else f"실패 ({message})"}')
        else:
            print('servo 없음 — 그대로 진행한다')

        print(f'{args.target} 로 복귀 (개루프, {RAMP_SEC}초)...')
        node.stream_to(targets)
        node.spin(1.0)

        if stopped:
            ok, message = servo.start()
            print(f'servo 재개: {"성공" if ok else f"실패 ({message})"}')

        worst = max((abs(node.positions.get(j, 0.0) - v) for j, v in targets.items()),
                    default=9.9)
        after, contacts = node.state_is_valid()
        print(f'관절 오차 {worst:.4f} rad / 충돌없음: {after}')
        if contacts:
            print(f'  남은 접촉: {", ".join(contacts[:4])}')
        ok = worst <= TOLERANCE and after is not False
    finally:
        planner.destroy_node()
        node.destroy_node()
        rclpy.shutdown()

    print('복구 완료' if ok else '복구 실패')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
