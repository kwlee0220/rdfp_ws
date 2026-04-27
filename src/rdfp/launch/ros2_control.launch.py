"""Panda ros2_control 중심 bringup 만 기동하는 launch 파일.

본 launch 는 TF, robot_state_publisher, ros2_control, controller spawner 까지의
제어 런타임만 기동한다.
"""

from __future__ import annotations

import os
import sys

from launch import LaunchDescription

# ROS2 launch 러너는 본 파일을 단일 스크립트로 로드하므로, 같은 디렉터리의
# sibling 모듈을 패키지 import 로 가져올 수 없다. 이 파일의 디렉터리를
# sys.path 의 맨 앞에 추가하여 top-level 모듈처럼 import 한다 (sibling 우선 보장; append 로 바꾸지 말 것).
sys.path.insert(0, os.path.dirname(__file__))

from controller_launch_helper import (
    create_joint_state_broadcaster_spawner,
    create_panda_arm_controller_spawner,
    create_panda_hand_controller_spawner,
    create_ros2_control_node,
)
from controller_startup_launch_helper import create_controller_startup_handlers
from launch_helper import (
    MOVEIT_CONFIGS_PACKAGE_NAME,
    build_moveit_config,
    create_robot_state_publisher,
    create_static_tf_node,
    declare_ros2_control_hardware_type_argument,
)


def generate_launch_description() -> LaunchDescription:
    moveit_config = build_moveit_config()

    static_tf = create_static_tf_node()
    robot_state_publisher = create_robot_state_publisher(moveit_config)
    ros2_control_node = create_ros2_control_node(
        moveit_config,
        MOVEIT_CONFIGS_PACKAGE_NAME,
    )

    joint_state_broadcaster_spawner = create_joint_state_broadcaster_spawner()
    panda_arm_controller_spawner = create_panda_arm_controller_spawner()
    panda_hand_controller_spawner = create_panda_hand_controller_spawner()

    controller_startup_handlers = create_controller_startup_handlers(
        ros2_control_node,
        joint_state_broadcaster_spawner,
        panda_arm_controller_spawner,
        panda_hand_controller_spawner,
        [],
    )

    return LaunchDescription(
        [
            declare_ros2_control_hardware_type_argument(),
            static_tf,
            robot_state_publisher,
            ros2_control_node,
            *controller_startup_handlers,
        ]
    )
