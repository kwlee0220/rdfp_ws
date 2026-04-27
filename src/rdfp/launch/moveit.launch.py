"""MoveIt 기반 manipulation 계층만 기동하는 launch 파일.

본 launch 는 move_group, servo, EE pose publisher 를 기동한다.
ros2_control 계층이 별도로 올라와 있다는 가정하에 사용하는 것을 권장한다.
"""

from __future__ import annotations

import os
import sys

from launch import LaunchDescription

# ROS2 launch 러너는 본 파일을 단일 스크립트로 로드하므로, 같은 디렉터리의
# sibling 모듈을 패키지 import 로 가져올 수 없다. 이 파일의 디렉터리를
# sys.path 의 맨 앞에 추가하여 top-level 모듈처럼 import 한다 (sibling 우선 보장; append 로 바꾸지 말 것).
sys.path.insert(0, os.path.dirname(__file__))

from ee_pose_launch_helper import create_ee_pose_node, declare_ee_pose_arguments
from launch_helper import (
    build_moveit_config,
    build_servo_params,
    create_move_group_node,
    create_servo_node,
    declare_log_level_argument,
    declare_ros2_control_hardware_type_argument,
)


def generate_launch_description() -> LaunchDescription:
    moveit_config = build_moveit_config()
    servo_params = build_servo_params()

    move_group_node = create_move_group_node(moveit_config)
    servo_node = create_servo_node(moveit_config, servo_params)
    ee_pose_node = create_ee_pose_node()

    return LaunchDescription(
        [
            declare_ros2_control_hardware_type_argument(),
            declare_log_level_argument(),
            *declare_ee_pose_arguments(),
            move_group_node,
            servo_node,
            ee_pose_node,
        ]
    )
