from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


# ros2_control_node 는 기본 노드 이름 `controller_manager` 로 뜨며, 본 launch
# 스택에서는 단일 CM 만 사용하므로 spawner 들도 이 상수를 그대로 참조한다.
CONTROLLER_MANAGER_NAME = "controller_manager"


def create_ros2_control_node(moveit_config, moveit_configs_package_name: str) -> Node:
    """ros2_control 노드를 생성한다."""
    ros2_controllers_file = os.path.join(
        get_package_share_directory(moveit_configs_package_name),
        "config",
        "ros2_controllers.yaml",
    )
    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_file,
        ],
    )


def create_controller_spawner(controller_name: str) -> Node:
    """지정한 컨트롤러 spawner 노드를 생성한다."""
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            controller_name,
            "--controller-manager",
            CONTROLLER_MANAGER_NAME,
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )


def create_joint_state_broadcaster_spawner() -> Node:
    """joint_state_broadcaster spawner 노드를 생성한다."""
    return create_controller_spawner("joint_state_broadcaster")


def create_panda_arm_controller_spawner() -> Node:
    """panda_arm_controller spawner 노드를 생성한다."""
    return create_controller_spawner("panda_arm_controller")


def create_panda_hand_controller_spawner() -> Node:
    """panda_hand_controller spawner 노드를 생성한다."""
    return create_controller_spawner("panda_hand_controller")
