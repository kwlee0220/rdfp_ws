"""Gazebo (Fortress / gz-sim6) 백엔드 전용 launch helper.

mock 백엔드(:mod:`launch_helper`, :mod:`controller_launch_helper`)와의 차이점만
여기에 모은다.

- robot_description: rdfp 의 description(inertial 보강 + ign_ros2_control 분기)을
  ``ros2_control_hardware_type:=gazebo`` 로 빌드한다.
- controller_manager 는 별도 ros2_control_node 가 아니라 Gazebo 프로세스 내부의
  ign_ros2_control 시스템 플러그인이 호스팅한다. 따라서 본 모듈은
  ``ros2_control_node`` 를 생성하지 않고, **Gazebo 기동 + 로봇 스폰** 후
  controller spawner 를 순차 실행하는 핸들러를 제공한다.
- ROS<->gz 브리지(clock / camera)를 ros_gz_bridge 로 구성한다.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from .controller_startup import _chain_or_shutdown, _chain_or_skip
from .common import MOVEIT_CONFIGS_PACKAGE_NAME, build_servo_params, description_path

# gazebo 백엔드에서 사용하는 ros2_control hardware_type 값.
GAZEBO_HARDWARE_TYPE = "gazebo"

# Fortress 는 gz-sim6 이다. ros_gz_sim/gz_sim.launch.py 의 gz_version 인자.
GZ_SIM_VERSION = "6"


def gazebo_ros2_controllers_file() -> str:
    """ign_ros2_control 플러그인에 넘길 ros2_controllers.yaml 절대경로.

    rdfp 의 Gazebo 전용 컨트롤러 설정을 사용한다(panda_arm_controller 에
    allow_nonzero_velocity_at_trajectory_end: false 추가 — servo 입력이 끊기면
    로봇이 즉시 정지하도록 한다).
    """
    return os.path.join(
        get_package_share_directory("robot_control"),
        "config",
        "gazebo_ros2_controllers.yaml",
    )


def build_gazebo_servo_params():
    """Gazebo 백엔드용 MoveIt Servo 파라미터.

    mock 설정(panda_simulated_config.yaml)을 로드하되 두 값을 덮어쓴다.

    - ``use_gazebo: true`` — servo 가 redundant point 를 채운 trajectory 를
      발행해 Gazebo JTC 의 타이밍 문제를 회피한다.
    - ``publish_joint_velocities: false`` — **드리프트 해결의 핵심**. true 면
      servo 가 모든 trajectory point(끝점 포함)에 jogging 속도를 넣는데, position
      인터페이스 JTC 는 끝점 속도가 0 이 아니면 (allow_nonzero=false 시) 거부하고,
      (true 시) 그 속도로 계속 진행(드리프트)한다. 위치만 보내면 끝점 속도가
      없어 거부도 드리프트도 사라지고, 입력이 끊기면 마지막 위치에서 정지한다.
    """
    params = build_servo_params()
    params["moveit_servo"]["use_gazebo"] = True
    params["moveit_servo"]["publish_joint_velocities"] = False
    return params


def build_gazebo_moveit_config():
    """gazebo 하드웨어로 빌드한 Panda MoveIt 설정 객체를 생성한다.

    robot_description 만 rdfp 의 description(inertial + ign_ros2_control 분기)로
    교체하고, SRDF/kinematics/joint_limits/planning_pipelines 는 mock 과 동일한
    moveit_resources 패키지 자산을 그대로 사용한다(`panda_*` 이름 계약 공유).
    """
    return (
        MoveItConfigsBuilder("panda", package_name=MOVEIT_CONFIGS_PACKAGE_NAME)
        .robot_description(
            file_path=description_path("panda.urdf.xacro"),
            mappings={
                "ros2_control_hardware_type": GAZEBO_HARDWARE_TYPE,
                "ros2_controllers_file": gazebo_ros2_controllers_file(),
                "initial_positions_file": description_path("initial_positions.yaml"),
                "simulate_camera": LaunchConfiguration("simulate_camera"),
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"])
        .to_moveit_configs()
    )


def gz_resource_path_actions() -> list[SetEnvironmentVariable]:
    """Gazebo 가 ``model://`` 메시를 해석하도록 리소스 경로를 주입한다.

    ign 의 URDF 파서는 ``package://moveit_resources_panda_description/...`` 를
    ``model://moveit_resources_panda_description/...`` 로 변환한다. gz 는 이를
    리소스 경로 하위에서 ``moveit_resources_panda_description/...`` 로 찾으므로,
    해당 패키지 share 의 **부모 디렉터리**(= ament share 루트)를 경로에 추가한다.

    Fortress 는 ``IGN_GAZEBO_RESOURCE_PATH`` 를, 신버전 호환을 위해
    ``GZ_SIM_RESOURCE_PATH`` 도 함께 설정한다. 기존 값은 보존하고 append 한다.
    """
    mesh_root = os.path.dirname(
        get_package_share_directory("moveit_resources_panda_description")
    )
    actions = []
    for var in ("IGN_GAZEBO_RESOURCE_PATH", "GZ_SIM_RESOURCE_PATH"):
        actions.append(
            SetEnvironmentVariable(
                name=var,
                value=[EnvironmentVariable(var, default_value=""), os.pathsep, mesh_root],
            )
        )
    return actions


def create_gz_sim(world: LaunchConfiguration) -> IncludeLaunchDescription:
    """Gazebo(gz-sim) 서버+클라이언트를 기동한다.

    ``gz_args`` 의 ``-r`` 은 즉시 시뮬레이션을 시작(run)한다는 뜻이다. world 는
    ``empty.sdf`` 등 SDF 월드 파일/이름을 받는다.
    """
    gz_sim_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            "gz_args": ["-r -v3 ", world],
            "gz_version": GZ_SIM_VERSION,
        }.items(),
    )


def create_spawn_entity_node() -> Node:
    """robot_description 토픽의 모델을 Gazebo 월드에 스폰한다."""
    return Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "panda",
            "-allow_renaming", "true",
        ],
    )


def create_gz_clock_bridge_node() -> Node:
    """Gazebo /clock 을 ROS 로 브리지한다(use_sim_time 의 클럭 소스)."""
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
    )


def create_gz_camera_bridge_node(image_topic: LaunchConfiguration,
                                 info_topic: LaunchConfiguration,
                                 condition=None) -> Node:
    """Gazebo 카메라 센서 토픽(image/camera_info)을 ROS 로 브리지한다.

    description 의 카메라 센서는 ``camera/image_raw`` 로 발행하고, gz 는
    같은 베이스에 ``camera/camera_info`` 를 함께 낸다. rdfp 카메라 토픽 계약
    (기본 ``/camera/image_raw`` / ``/camera/camera_info``)에 맞춰 remap 한다.
    """
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_camera_bridge",
        output="screen",
        condition=condition,
        arguments=[
            "/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ],
        remappings=[
            ("/camera/image_raw", image_topic),
            ("/camera/camera_info", info_topic),
        ],
    )


def create_gazebo_controller_startup_handlers(spawn_entity_node,
                                              joint_state_broadcaster_spawner,
                                              panda_arm_controller_spawner,
                                              panda_hand_controller_spawner,
                                              post_hand_actions) -> list:
    """Gazebo 스폰 완료를 기점으로 controller 들을 순차 기동하는 핸들러.

    mock 의 :func:`create_controller_startup_handlers` 와 체인 구조는 같으나,
    최초 트리거가 ``ros2_control_node`` 시작이 아니라 ``create``(스폰) 노드의
    종료다 — ign_ros2_control 플러그인이 controller_manager 를 띄운 직후가 되어야
    spawner 가 controller 를 붙일 수 있기 때문이다.
    """
    delay_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=_chain_or_shutdown(joint_state_broadcaster_spawner, "spawn_entity"),
        )
    )
    delay_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=_chain_or_shutdown(panda_arm_controller_spawner,
                                       "joint_state_broadcaster_spawner"),
        )
    )
    delay_hand_after_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=panda_arm_controller_spawner,
            on_exit=_chain_or_shutdown(panda_hand_controller_spawner,
                                       "panda_arm_controller_spawner"),
        )
    )
    delay_post_after_hand = RegisterEventHandler(
        OnProcessExit(
            target_action=panda_hand_controller_spawner,
            on_exit=_chain_or_skip(post_hand_actions, "panda_hand_controller_spawner"),
        )
    )
    return [
        delay_jsb_after_spawn,
        delay_arm_after_jsb,
        delay_hand_after_arm,
        delay_post_after_hand,
    ]
