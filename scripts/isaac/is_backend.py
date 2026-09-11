#!/usr/bin/env python3
"""검사·복구 스크립트가 공유하는 Isaac 백엔드 계약.

**연동 방식은 하나다 (2026-09-05).** `controller_manager` 가 ROS 쪽에서 돌고
(`topic_based_ros2_control/TopicBasedSystem`), Isaac 은 토픽으로만 말한다.
launch 는 `panda_isaac.launch.py`, 시뮬레이터는 `run_isaac_sim.sh` 다.

ros2_control 없이 토픽만으로 잇던 옛 경로(bridge)는 **삭제했다.** 고유하게 덮는
것이 없었기 때문이다 — `MoveGroupJgpcClient`·`TrajectoryStreamer` 는
`panda_jgpc_mock` 이, `arm_command_format='joint_state'` 와 `servo_command_bridge` 는
펑션베이가 덮는다. 반면 방식이 둘이면 **launch 와 어긋났을 때 에러 없이 관절 상태가
오지 않는** 함정이 생기고, 팔 실행이 개루프라 "성공을 반환했는데 도달하지 않음"이
원리적으로 가능했다. 경위: `docs/simulation/isaac_backend_skeleton.md` §0 D1″.
"""
from __future__ import annotations

from typing import Callable, Optional

import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINT_NAMES = [f'panda_joint{i}' for i in range(1, 8)]

# `joint_state_broadcaster` 가 소유한다. Isaac 자신은 `/isaac_joint_states` 로 비켜서
# 있다 — 같은 토픽에 둘이 발행하면 `TopicBasedSystem` 이 자기 출력을 되읽는다.
JOINT_STATE_TOPIC = '/joint_states'

# 팔에 명령을 **직접** 넣는 채널 (JTC).
ARM_COMMAND_TOPIC = '/panda_arm_controller/joint_trajectory'

# '계단 입력'을 흉내낼 최소 도달 시간. JTC 는 0 을 거부한다.
#
# **50 ms 였을 때 `phase1` 의 τ 기준(≤ 50 ms)과 값이 같아 측정이 무의미했다** — 잰 값
# 52.6 ms 는 사실상 이 상수 + 2.6 ms 였다. `controller_manager` 의 update 주기(기본
# 100 Hz = 10 ms)가 어차피 하한이므로 그 수준으로 낮췄다(실측 τ 35.8 ms).
MIN_REACH_SEC = 0.01


def mode_banner() -> str:
    """검사 출력 머리에 붙일 한 줄."""
    return f'팔 명령 {ARM_COMMAND_TOPIC}, MoveGroup jtc'


def create_arm_client(node: Node):
    """MoveGroup 클라이언트를 만든다.

    `mode='auto'` 로 두지 않는다 — 자동 판별은 토픽 그래프 조회라 DDS 디스커버리가
    앉기 전에 부르면 오판한다. 어느 스택인지 이미 알고 있으므로 못박는다.
    """
    from robot_control.moveit.move_group_factory import create_move_group_client
    return create_move_group_client(node, mode='jtc')


def move_to_named(client, name: str, **kwargs):
    """named target 이동 (동기)."""
    return client.move_to_named_target(name, **kwargs)


def move_to_joints(client, joint_values: dict, **kwargs):
    """관절 목표 이동 (동기)."""
    return client.move_to_joints(joint_values, **kwargs)


def follow_waypoints(client, waypoints: list, **kwargs):
    """cartesian 경유점 추종 (동기)."""
    return client.follow_trajectory(waypoints, **kwargs)


def execute_plan(client, trajectory, **kwargs):
    """이미 세운 계획을 실행한다."""
    return client.execute_trajectory(trajectory, **kwargs)


class ArmCommander:
    """MoveIt 을 거치지 않고 관절 목표를 직접 밀어 넣는다.

    계획이 거부되는 상황(시작 자세가 충돌)에서 빠져나오거나, 계단 응답처럼 계획으로는
    만들 수 없는 입력을 넣을 때 쓴다. `JointTrajectory` 를 한 번 넣으면 JTC 가 보간하고
    마지막 점을 유지하므로 반복 발행이 필요 없다.
    """

    def __init__(self, node: Node, *, current: Optional[Callable[[], dict]] = None) -> None:
        self._node = node
        self._current = current
        self._pub = node.create_publisher(JointTrajectory, ARM_COMMAND_TOPIC, 10)

    def _spin(self, seconds: float) -> None:
        deadline = time.time() + seconds
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.005)

    def drive(self, targets: dict[str, float], *, reach_sec: float = 0.0,
              hold_sec: float = 0.0) -> None:
        """`reach_sec` 동안 `targets` 로 이동한 뒤 `hold_sec` 동안 유지한다.

        `reach_sec=0` 은 계단 입력이다. JTC 가 0 을 거부하므로 `MIN_REACH_SEC` 을 쓰며,
        **그만큼 계단 응답에는 컨트롤러 보간이 섞인다.**
        """
        names = list(targets)
        reach = max(reach_sec, MIN_REACH_SEC)
        point = JointTrajectoryPoint()
        point.positions = [float(targets[j]) for j in names]
        point.velocities = [0.0] * len(names)
        point.time_from_start.sec = int(reach)
        point.time_from_start.nanosec = int((reach % 1.0) * 1e9)
        msg = JointTrajectory()
        msg.joint_names = names
        msg.points = [point]
        self._pub.publish(msg)
        self._spin(reach + hold_sec)
