from __future__ import annotations

import os
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

MOVEIT_CONFIGS_PACKAGE_NAME = "moveit_resources_panda_moveit_config"
MOVEIT_SERVO_PACKAGE_NAME = "moveit_servo"


def declare_ros2_control_hardware_type_argument() -> DeclareLaunchArgument:
    """ros2_control hardware 타입 launch argument를 선언한다."""
    return DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description=(
            "ROS 2 control hardware interface type "
            "(e.g. mock_components for fake hardware)"
        ),
    )


def declare_log_level_argument() -> DeclareLaunchArgument:
    """MoveIt 관련 노드의 로그 레벨 launch argument를 선언한다."""
    return DeclareLaunchArgument(
        "log_level",
        default_value="info",
        choices=["debug", "info", "warn", "error", "fatal"],
        description="MoveIt2 주요 노드(move_group / servo) 의 로그 레벨",
    )


def build_moveit_config():
    """Panda MoveIt 설정 객체를 생성한다."""
    return (
        MoveItConfigsBuilder("panda", package_name=MOVEIT_CONFIGS_PACKAGE_NAME)
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"]
        )
        .to_moveit_configs()
    )


def build_servo_params() -> dict[str, Any]:
    """MoveIt Servo 파라미터를 로드한다."""
    return (
        ParameterBuilder(MOVEIT_SERVO_PACKAGE_NAME)
        .yaml(
            parameter_namespace="moveit_servo",
            file_path="config/panda_simulated_config.yaml",
        )
        .to_dict()
    )


def create_static_tf_node() -> Node:
    """world -> panda_link0 static TF 노드를 생성한다."""
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
    )


def create_robot_state_publisher(moveit_config) -> Node:
    """robot_state_publisher 노드를 생성한다."""
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )


def create_move_group_node(moveit_config) -> Node:
    """move_group 노드를 생성한다."""
    return Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
    )


def create_servo_node(moveit_config, servo_params: dict[str, Any]) -> Node:
    """MoveIt Servo 노드를 생성한다."""
    return Node(
        package=MOVEIT_SERVO_PACKAGE_NAME,
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
    )


def create_rviz_node(moveit_config) -> Node:
    """RViz2 노드를 생성한다."""
    rviz_config_file = os.path.join(
        get_package_share_directory("rdfp"),
        "config",
        "panda.rviz",
    )
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )
