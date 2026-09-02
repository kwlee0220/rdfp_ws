"""`GripperNode` 설정.

설계: `docs/moveit/GripperNode_Design.md`

**노드가 하나다.** 예전에는 명령을 받는 `gripper_control_node` 와 상태를 내는
`gripper_state_publisher` 가 나뉘어 있었는데, `GripperState.goal`(마지막 명령)을 실으려면
상태 발행자가 명령을 알아야 해서 합쳤다. 나누면 상태 발행자가 명령 토픽을 따로 구독해야
하고, 명령 직후·발행 직전 구간에서 옛 목표가 실리는 창이 생긴다.
"""

from __future__ import annotations

from typing import Optional

from launch_ros.actions import Node

from robot_control.launch_helpers.common import extra_parameter_list


def create_gripper_node(extra_parameters: Optional[dict] = None) -> Node:
    """`GripperNode` 를 생성한다 (액션 기반 구현).

    `control_msgs/GripperCommand` 액션 서버가 있는 스택에서 쓴다 — mock 은
    `panda_hand_controller` 가, Isaac 은 `isaac_gripper_bridge` 가 제공한다.

    **펑션베이에는 쓸 수 없다.** 액션 서버가 없고 명령 채널이 토픽이라
    `FunctionBayGripperNode` 가 따로 필요하다 (미구현).
    """
    return Node(
        package="robot_control",
        executable="mock_gripper_node",
        name="gripper",
        output="screen",
        emulate_tty=True,
        parameters=extra_parameter_list(extra_parameters),
    )
