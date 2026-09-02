"""`GripperNode` 설정.

설계: `docs/moveit/GripperNode_Design.md`

**구현이 둘이고 갈리는 축은 실행 수단이다** — 백엔드가 아니다.

  `create_gripper_node()`        액션 서버가 있는 스택 (mock · Gazebo · Isaac)
  `create_gripper_joint_node()`  관절 목표각을 토픽으로 쓰는 스택 (펑션베이)

**노드가 하나다.** 예전에는 명령을 받는 `gripper_control_node` 와 상태를 내는
`gripper_state_publisher` 가 나뉘어 있었는데, `GripperState.goal`(마지막 명령)을 실으려면
상태 발행자가 명령을 알아야 해서 합쳤다. 나누면 상태 발행자가 명령 토픽을 따로 구독해야
하고, 명령 직후·발행 직전 구간에서 옛 목표가 실리는 창이 생긴다.
"""

from __future__ import annotations

from typing import Any, Optional

from launch.condition import Condition
from launch_ros.actions import Node

from robot_control.launch_helpers.common import extra_parameter_list


def create_gripper_node(extra_parameters: Optional[dict] = None) -> Node:
    """`GripperActionNode` 를 생성한다.

    `control_msgs/GripperCommand` 액션 서버가 있는 스택에서 쓴다 — mock 은
    `panda_hand_controller` 가, Isaac 은 `isaac_gripper_bridge` 가 제공한다.
    **백엔드가 아니라 실행 수단으로 갈린다.**

    **펑션베이에는 쓸 수 없다.** 액션 서버가 없고 명령 채널이 토픽이라 토픽 기반
    구현이 따로 필요하다 (미구현).
    """
    return Node(
        package="robot_control",
        executable="gripper_action_node",
        name="gripper",
        output="screen",
        emulate_tty=True,
        parameters=extra_parameter_list(extra_parameters),
    )


def create_gripper_joint_node(command_topic: Any, report_topic: Any,
                              extra_parameters: Optional[dict] = None,
                              condition: Optional[Condition] = None) -> Node:
    """`GripperJointNode` 를 생성한다.

    `control_msgs/GripperCommand` 액션 서버가 **없는** 스택에서 쓴다. 노드 자체는
    백엔드 중립이다 — 토픽이 상대 경로라 remap 이 결합을 만든다.

    Args:
        command_topic: 시뮬레이터가 관절 목표각을 받는 토픽 (`Float64MultiArray`).
        report_topic: 시뮬레이터의 관절 보고 토픽 (`JointState`, position/velocity/effort).
        extra_parameters: `axis_signs` · `targets.<goal>` · 임계값 등 재정의.
        condition: 기동 조건 (예: `IfCondition(LaunchConfiguration('enable_gripper'))`).
    """
    return Node(
        package="robot_control",
        executable="gripper_joint_node",
        name="gripper",
        output="screen",
        emulate_tty=True,
        parameters=extra_parameter_list(extra_parameters),
        remappings=[("joint_command", command_topic), ("joint_report", report_topic)],
        condition=condition,
    )
