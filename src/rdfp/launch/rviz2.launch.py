"""RViz2 시각화만 기동하는 launch 파일."""

from __future__ import annotations

import os
import sys

from launch import LaunchDescription

# ROS2 launch 러너는 본 파일을 단일 스크립트로 로드하므로, 같은 디렉터리의
# sibling 모듈을 패키지 import 로 가져올 수 없다. 이 파일의 디렉터리를
# sys.path 의 맨 앞에 추가하여 top-level 모듈처럼 import 한다 (sibling 우선 보장; append 로 바꾸지 말 것).
sys.path.insert(0, os.path.dirname(__file__))

from launch_helper import (
    build_moveit_config,
    create_rviz_node,
    declare_ros2_control_hardware_type_argument,
)


def generate_launch_description() -> LaunchDescription:
    moveit_config = build_moveit_config()
    rviz_node = create_rviz_node(moveit_config)

    return LaunchDescription(
        [
            declare_ros2_control_hardware_type_argument(),
            rviz_node,
        ]
    )
