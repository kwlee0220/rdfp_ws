"""Panda + MoveIt2 풀 스택을 최종 orchestration 하는 launch 파일.

기능은 논리적으로 다음 단위로 분리되어 있다.

- ``ros2_control``:
    - /controller_manager
    - /joint_state_broadcaster
    - /panda_arm_controller
    - /panda_hand_controller
    - /static_transform_publisher
    - /robot_state_publisher
- ``moveit``:
    - /move_group
    - /moveit_servo
    - /ee_pose_publisher
- ``rviz2``: /rviz2
- ``camera``: /camera
- ``gripper``: /gripper_control

본 파일은 위 단위와 같은 책임 경계를 유지하면서, 기존과 동일한 순차 기동
정책을 event handler 로 orchestration 한다.
"""

from __future__ import annotations

import os
import sys

# ROS2 launch 러너는 본 파일을 단일 스크립트로 로드하므로, 같은 디렉터리의
# sibling 모듈을 패키지 import 로 가져올 수 없다. 이 파일의 디렉터리를
# sys.path 의 맨 앞에 추가하여 top-level 모듈처럼 import 한다 (sibling 우선 보장; append 로 바꾸지 말 것).
sys.path.insert(0, os.path.dirname(__file__))

from launch import LaunchDescription

from camera_launch_helper import declare_camera_arguments, create_camera_node
from controller_launch_helper import (
    create_joint_state_broadcaster_spawner,
    create_panda_arm_controller_spawner,
    create_panda_hand_controller_spawner,
    create_ros2_control_node,
)
from controller_startup_launch_helper import create_controller_startup_handlers
from ee_pose_launch_helper import create_ee_pose_node, declare_ee_pose_arguments
from gripper_launch_helper import create_gripper_control_node
from launch_helper import (
    MOVEIT_CONFIGS_PACKAGE_NAME,
    build_moveit_config,
    build_servo_params,
    create_move_group_node,
    create_robot_state_publisher,
    create_rviz_node,
    create_servo_node,
    create_static_tf_node,
    declare_log_level_argument,
    declare_ros2_control_hardware_type_argument,
)


def generate_launch_description() -> LaunchDescription:
    moveit_config = build_moveit_config()
    servo_params = build_servo_params()

    static_tf = create_static_tf_node()
    robot_state_publisher = create_robot_state_publisher(moveit_config)
    ros2_control_node = create_ros2_control_node(
        moveit_config,
        MOVEIT_CONFIGS_PACKAGE_NAME,
    )
    joint_state_broadcaster_spawner = create_joint_state_broadcaster_spawner()
    panda_arm_controller_spawner = create_panda_arm_controller_spawner()
    panda_hand_controller_spawner = create_panda_hand_controller_spawner()

    move_group_node = create_move_group_node(moveit_config)
    servo_node = create_servo_node(moveit_config, servo_params)
    rviz_node = create_rviz_node(moveit_config)
    camera_node = create_camera_node()
    ee_pose_node = create_ee_pose_node()
    gripper_control_node = create_gripper_control_node()

    controller_startup_handlers = create_controller_startup_handlers(
        ros2_control_node,
        joint_state_broadcaster_spawner,
        panda_arm_controller_spawner,
        panda_hand_controller_spawner,
        [move_group_node, servo_node, rviz_node, camera_node, ee_pose_node,
         gripper_control_node],
    )

    return LaunchDescription(
        [
            declare_ros2_control_hardware_type_argument(),
            declare_log_level_argument(),
            *declare_ee_pose_arguments(),
            *declare_camera_arguments(),
            static_tf,
            robot_state_publisher,
            ros2_control_node,
            *controller_startup_handlers,
        ]
    )
