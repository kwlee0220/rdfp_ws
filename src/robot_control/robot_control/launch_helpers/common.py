from __future__ import annotations

import os
from typing import Any, Optional

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

MOVEIT_CONFIGS_PACKAGE_NAME = "moveit_resources_panda_moveit_config"
MOVEIT_SERVO_PACKAGE_NAME = "moveit_servo"

# description 을 소유한 패키지. robot_description(URDF) 만 이 포크에서 오고,
# SRDF/kinematics/joint_limits/planning_pipelines 는 계속 moveit_resources 것을 쓴다.
DESCRIPTION_PACKAGE_NAME = "robot_control"


def description_path(*parts: str) -> str:
    """`robot_control` share 의 description 경로를 구성한다."""
    return os.path.join(
        get_package_share_directory(DESCRIPTION_PACKAGE_NAME), "description", *parts)


def declare_ros2_control_hardware_type_argument(
        default_value: str = "mock_components") -> DeclareLaunchArgument:
    """ros2_control hardware 타입 launch argument를 선언한다.

    백엔드 launch 가 자기 기본값을 준다 (Isaac plugin 은 `isaac`). 값은
    `description/panda.ros2_control.xacro` 의 분기 이름과 일치해야 한다.
    """
    return DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value=default_value,
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


def build_moveit_config(joint_limits_file: Optional[str] = None,
                        description_mappings: Optional[dict[str, Any]] = None):
    """Panda MoveIt 설정 객체를 생성한다.

    **URDF 는 `robot_control/description/` 의 포크를 쓴다.** moveit_resources 원본과
    링크·관절·한계·충돌형상이 완전히 같고(구조 비교로 확인), 더해지는 것은 inertial
    12개와 `gazebo`/`isaac` 하드웨어 분기뿐이다. 원본을 쓰면 백엔드별 분기를 넣을
    자리가 없어 — 외부 패키지를 고칠 수는 없으므로 — 포크가 단일 출처가 된다.

    Args:
        joint_limits_file: 관절 한계 YAML 절대경로 (Isaac 은 실제 Franka 스펙을 쓴다).
        description_mappings: xacro 에 추가로 넘길 인자. Isaac plugin 이
            `isaac_arm_command_topic` 등 토픽 이름을 여기로 주입한다.
    """
    return (
        MoveItConfigsBuilder("panda", package_name=MOVEIT_CONFIGS_PACKAGE_NAME)
        .robot_description(
            file_path=description_path("panda.urdf.xacro"),
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                ),
                **(description_mappings or {}),
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        # 절대 경로를 주면 pathlib 이 그것을 그대로 쓴다 — 백엔드마다 다른 한계
        # 파일을 넣을 수 있는 자리다 (Isaac 은 실제 Franka 스펙을 쓴다).
        .joint_limits(file_path=joint_limits_file or "config/joint_limits.yaml")
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


def extra_parameter_list(extra_parameters: Optional[dict]) -> list:
    """``parameters=[...]`` 뒤에 이어 붙일 추가 파라미터를 리스트로 만든다.

    백엔드마다 노드에 얹어야 하는 파라미터가 다르다(예: Isaac 백엔드의
    ``use_sim_time``). 헬퍼 시그니처를 백엔드별로 늘리지 않기 위해 dict 하나를
    받아 그대로 뒤에 붙인다. ``None`` 이면 아무것도 붙이지 않으므로 기존 호출부는
    동작이 바뀌지 않는다.
    """
    return [extra_parameters] if extra_parameters else []


def create_static_tf_node(extra_parameters: Optional[dict] = None) -> Node:
    """world -> panda_link0 static TF 노드를 생성한다."""
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
        parameters=extra_parameter_list(extra_parameters),
    )


def create_robot_state_publisher(moveit_config, extra_parameters: Optional[dict] = None) -> Node:
    """robot_state_publisher 노드를 생성한다."""
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, *extra_parameter_list(extra_parameters)],
    )


def create_move_group_node(moveit_config, extra_parameters: Optional[dict] = None) -> Node:
    """move_group 노드를 생성한다."""
    return Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), *extra_parameter_list(extra_parameters)],
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
    )


def create_servo_node(moveit_config, servo_params: dict[str, Any], condition=None,
                      extra_parameters: Optional[dict] = None) -> Node:
    """MoveIt Servo 노드를 생성한다.

    condition 을 주면 해당 조건이 참일 때만 기동한다 (예: enable_servo 토글).
    extra_parameters 는 `use_sim_time` 처럼 백엔드가 덧붙이는 값이다.
    """
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
            *extra_parameter_list(extra_parameters),
        ],
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        condition=condition,
    )


def create_rviz_node(moveit_config, condition=None,
                     extra_parameters: Optional[dict] = None) -> Node:
    """RViz2 노드를 생성한다.

    condition 을 주면 해당 조건이 참일 때만 기동한다 (예: enable_rviz 토글).
    """
    rviz_config_file = os.path.join(
        get_package_share_directory("robot_control"),
        "config",
        "panda.rviz",
    )
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=condition,
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            *extra_parameter_list(extra_parameters),
        ],
    )
