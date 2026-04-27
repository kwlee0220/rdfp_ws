"""EE pose publisher 노드 및 파라미터 설정."""

from __future__ import annotations

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def declare_ee_pose_arguments() -> list[DeclareLaunchArgument]:
    """EE pose publisher 관련 launch argument들을 선언한다."""
    return [
        DeclareLaunchArgument(
            "base_frame",
            default_value="panda_link0",
            description="Base frame for EE pose TF lookup",
        ),
        DeclareLaunchArgument(
            "ee_frame",
            default_value="panda_hand",
            description="End-effector frame for EE pose TF lookup",
        ),
        DeclareLaunchArgument(
            "publish_rate",
            default_value="50.0",
            description="EE pose publish rate in Hz",
        ),
    ]


def create_ee_pose_node() -> Node:
    """EE pose publisher 노드를 생성한다."""
    return Node(
        package="rdfp",
        executable="ee_pose_node",
        name="ee_pose_publisher",
        output="screen",
        parameters=[
            {
                "base_frame": LaunchConfiguration("base_frame"),
                "ee_frame": LaunchConfiguration("ee_frame"),
                "publish_rate": LaunchConfiguration("publish_rate"),
            }
        ],
    )
