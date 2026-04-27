"""데이터셋 재생(replay) 전용 Panda + MoveIt2 런치.

:mod:`rdfp_panda_mock.launch` 와 유사한 스택을 띄우되, **재생 시 데이터셋에서
이미 제공되는 정보** 를 생성하는 노드들은 제외한다.

재생 도구(예: ``ros2 bag play`` / 커스텀 dataset player) 가 다음 토픽들을
퍼블리시한다는 전제 하에 구성된다.

- ``/joint_states`` — :mod:`rdfp_panda_mock.launch` 의 ``joint_state_broadcaster``
  를 통해 실시간 관측값을 사용한다 (대체하지 않는다).
- ``/ee_pose`` — 데이터셋에서 재생되므로 ``ee_pose_publisher`` 를 띄우지 않는다.
- 이미지 (``/camera/...``) — 데이터셋에서 재생되므로 ``camera`` 노드를 띄우지
  않는다. 재생된 이미지 토픽을 구독하는 ``rdfp_image_viewer_node`` 는 유지한다.
- ``/gripper_control/gripper_cmds`` — 데이터셋 재생 쪽에서 발행한다. 본 런치의
  ``gripper_command_subscriber`` 가 이를 받아 gripper action 으로 변환한다.
- 세션 / 녹화 / target_joint_states 재발행은 **수행하지 않는다**
  (``session_control`` / ``image_recorder`` / ``target_joint_states_publisher``
  모두 제외).
- 대신 재생 도구가 보낸 ``/target_joint_states`` 를 panda_arm_controller 로
  흘려보내기 위해 ``target_joint_states_executor`` 를 띄운다 (JointTrajectory
  길이 1 로 래핑 → ``/panda_arm_controller/joint_trajectory`` 에 발행).

기동되는 노드 단위:

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
- ``rviz2``: /rviz2
- ``rdfp 애플리케이션``:
    - /rdfp_image_viewer_node
    - /gripper_command_subscriber
    - /target_joint_states_executor

기동 정책은 :mod:`panda_mock.launch` 와 동일하다: ``panda_hand_controller``
spawner 가 종료되면 상위 노드들을 일괄 spawn 한다.

설정 파일
---------

:mod:`rdfp_panda_mock.launch` 와 동일한 YAML 구조를 사용한다.

- 기본 경로: ``<rdfp share>/config/rdfp_panda_mock.yaml``
- ``config_file:=<path>`` launch argument 로 다른 YAML 을 지정할 수 있다.
  ``$HOME`` / ``~`` 같은 경로 확장은 쉘에 맡긴다
  (예: ``config_file:=$HOME/my.yaml`` — 쉘이 먼저 확장한 절대경로가 전달된다).
- CLI 에서 ``arg:=value`` 로 개별 argument 를 덮어쓰는 것은 그대로 동작한다.

YAML 파일의 구조는 ``config/rdfp_panda_mock.yaml`` 을 참고한다. 재생 모드에서
기동하지 않는 노드(camera / ee_pose / image_recorder / target_joint_states 등)
와 관련된 argument 는 현재 소비자가 없으므로 값이 무시된다.
"""

from __future__ import annotations

from typing import Any

import os
import sys

import yaml

# ROS2 launch 러너는 본 파일을 단일 스크립트로 로드하므로, 같은 디렉터리의
# sibling 모듈을 패키지 import 로 가져올 수 없다. 이 파일의 디렉터리를
# sys.path 의 맨 앞에 추가하여 top-level 모듈처럼 import 한다 (sibling 우선 보장; append 로 바꾸지 말 것).
sys.path.insert(0, os.path.dirname(__file__))

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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
    build_servo_params,
    create_move_group_node,
    create_robot_state_publisher,
    create_rviz_node,
    create_servo_node,
    create_static_tf_node,
)

# YAML 설정 파일의 기본 경로. setup.py 가 ``config/*`` 를
# ``share/rdfp/config/`` 로 설치하므로 package share 에서 읽는다.
DEFAULT_CONFIG_RELPATH = os.path.join("config", "rdfp_panda_mock.yaml")


def _default_config_path() -> str:
    """패키지 share 경로의 기본 YAML 위치를 반환한다."""
    return os.path.join(get_package_share_directory("rdfp"), DEFAULT_CONFIG_RELPATH)


def _load_config(config_path: str) -> dict[str, Any]:
    """YAML 설정 파일을 로드하여 dict 로 반환한다."""
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _as_launch_str(value: Any) -> str:
    """Python 값을 DeclareLaunchArgument 의 default_value 로 쓰이는 문자열로 변환한다.

    - bool 은 ROS launch 관례에 맞춰 소문자 ``"true"`` / ``"false"`` 로 변환한다.
    - 그 외는 ``str()`` 으로 변환한다.
    """
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _declare_arguments(config: dict[str, Any]) -> list[DeclareLaunchArgument]:
    """YAML 설정값을 default 로 사용하는 DeclareLaunchArgument 목록을 생성한다.

    helper 모듈의 ``declare_*_arguments()`` 는 기본값을 하드코딩하므로, YAML
    기반 기본값을 적용하려면 본 launch 파일에서 직접 선언해야 한다. helper 의
    ``create_*_node()`` 는 ``LaunchConfiguration(<name>)`` 으로 값을 참조하므로
    여기서 선언한 argument 이름만 일치시키면 그대로 동작한다.
    """
    rc = config["ros2_control"]
    ee = config["ee_pose"]
    cam = config["camera"]
    iv = config["image_viewer"]
    ir = config["image_recorder"]
    tjs = config["target_joint_states"]

    return [
        # --- ros2_control ---
        DeclareLaunchArgument(
            "ros2_control_hardware_type",
            default_value=_as_launch_str(rc["hardware_type"]),
            description=(
                "ROS 2 control hardware interface type "
                "(e.g. mock_components for fake hardware)"
            ),
        ),
        # --- log_level ---
        DeclareLaunchArgument(
            "log_level",
            default_value=_as_launch_str(config["log_level"]),
            choices=["debug", "info", "warn", "error", "fatal"],
            description="MoveIt2 주요 노드(move_group / servo) 및 session_control 의 로그 레벨",
        ),
        # --- ee_pose ---
        DeclareLaunchArgument(
            "base_frame",
            default_value=_as_launch_str(ee["base_frame"]),
            description="Base frame for EE pose TF lookup",
        ),
        DeclareLaunchArgument(
            "ee_frame",
            default_value=_as_launch_str(ee["ee_frame"]),
            description="End-effector frame for EE pose TF lookup",
        ),
        DeclareLaunchArgument(
            "publish_rate",
            default_value=_as_launch_str(ee["publish_rate"]),
            description="EE pose publish rate in Hz",
        ),
        # --- camera ---
        DeclareLaunchArgument(
            "enable_camera_node",
            default_value=_as_launch_str(cam["enabled"]),
            description="Whether to start rdfp camera_node",
        ),
        DeclareLaunchArgument(
            "camera_id",
            default_value=_as_launch_str(cam["id"]),
            description="Camera device index or URI/path for camera_node",
        ),
        DeclareLaunchArgument(
            "camera_image_topic",
            default_value=_as_launch_str(cam["image_topic"]),
            description="Remap target for base image topic ('image')",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value=_as_launch_str(cam["info_topic"]),
            description="Remap target for base camera_info topic ('camera_info')",
        ),
        DeclareLaunchArgument(
            "camera_status_topic",
            default_value=_as_launch_str(cam["status_topic"]),
            description="Remap target for camera status topic ('image/status')",
        ),
        DeclareLaunchArgument(
            "camera_fps",
            default_value=_as_launch_str(cam["fps"]),
            description="Target FPS for camera_node",
        ),
        DeclareLaunchArgument(
            "camera_resolution",
            default_value=_as_launch_str(cam["resolution"]),
            description="Target resolution for camera_node (e.g. 640x480)",
        ),
        DeclareLaunchArgument(
            "camera_frame_id",
            default_value=_as_launch_str(cam["frame_id"]),
            description="frame_id for published Image/CameraInfo",
        ),
        DeclareLaunchArgument(
            "camera_compress_image",
            default_value=_as_launch_str(cam["compress_image"]),
            description="Publish JPEG compressed image when true",
        ),
        # --- image_viewer ---
        DeclareLaunchArgument(
            "enable_image_viewer_node",
            default_value=_as_launch_str(iv["enabled"]),
            description="Whether to start rdfp_image_viewer_node",
        ),
        # --- image_recorder ---
        DeclareLaunchArgument(
            "enable_image_recorder_node",
            default_value=_as_launch_str(ir["enabled"]),
            description="Whether to start image_recorder_node",
        ),
        DeclareLaunchArgument(
            "image_recorder_fps",
            default_value=_as_launch_str(ir["fps"]),
            description=(
                "Target FPS for image_recorder_node "
                "(should match camera_node fps to avoid frame drops)"
            ),
        ),
        DeclareLaunchArgument(
            "image_recorder_output_dir",
            default_value=_as_launch_str(ir["output_dir"]),
            description="Output directory for image_recorder_node MP4 files",
        ),
        DeclareLaunchArgument(
            "image_recorder_auto_start",
            default_value=_as_launch_str(ir["auto_start"]),
            description="If true, image_recorder_node starts recording immediately at launch",
        ),
        # --- target_joint_states ---
        DeclareLaunchArgument(
            "target_joint_states_input_topic",
            default_value=_as_launch_str(tjs["input_topic"]),
            description=(
                "Remap target for target_joint_states_publisher input topic "
                "('joint_trajectory'); typically /servo_node/joint_trajectory"
            ),
        ),
    ]


def _build_actions(context: LaunchContext) -> list:
    """`config_file` 이 resolve 된 뒤 YAML 을 로드해 나머지 argument/노드를 구성한다.

    `OpaqueFunction` 이 실행되는 시점에는 `LaunchConfiguration` 이 이미
    resolve 가능하므로, top-level CLI 의 ``config_file:=...`` 든
    ``IncludeLaunchDescription(..., launch_arguments=...)`` 든 동일하게 반영된다.
    """
    config_path = LaunchConfiguration("config_file").perform(context)
    config = _load_config(config_path)

    moveit_config = build_moveit_config()
    servo_params = build_servo_params()

    # --- ros2_control 스택 ---
    static_tf = create_static_tf_node()
    robot_state_publisher = create_robot_state_publisher(moveit_config)
    ros2_control_node = create_ros2_control_node(moveit_config, MOVEIT_CONFIGS_PACKAGE_NAME)
    joint_state_broadcaster_spawner = create_joint_state_broadcaster_spawner()
    panda_arm_controller_spawner = create_panda_arm_controller_spawner()
    panda_hand_controller_spawner = create_panda_hand_controller_spawner()

    # --- MoveIt / RViz / 주변 노드 ---
    # replay 모드에서는 /camera, /ee_pose_publisher, /gripper_control 은 기동하지
    # 않는다 (프레임/포즈/그리퍼 명령이 모두 데이터셋 재생에서 공급됨).
    move_group_node = create_move_group_node(moveit_config)
    servo_node = create_servo_node(moveit_config, servo_params)
    rviz_node = create_rviz_node(moveit_config)

    # --- rdfp 애플리케이션: RdfpImageViewerNode ---
    # 재생 중인 이미지 토픽을 구독하여 수신 프레임 좌상단에 상태 오버레이를
    # 덧입혀 표시한다. /session_control 이 없으므로 오버레이의 세션 상태 필드는
    # 기본값으로 유지된다.
    rdfp_image_viewer_node = Node(
        package="rdfp",
        executable="rdfp_image_viewer_node",
        name="rdfp_image_viewer_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_image_viewer_node")),
        remappings=[
            ("image", LaunchConfiguration("camera_image_topic")),
        ],
    )

    # --- rdfp 애플리케이션: GripperCommandSubscriber ---
    # 데이터셋 재생 도구가 발행하는 `rdfp_msgs/GripperCommand` 를 받아 gripper
    # action 에 그대로 전달한다. 녹화된 토픽 이름과 맞추기 위해 `~/gripper_cmds`
    # 를 `/gripper_control/gripper_cmds` 로 remap 한다 (replay 모드에서는 본
    # 토픽에 퍼블리셔가 `GripperControlNode` 가 아니라 재생 도구라는 점만 다름).
    gripper_command_subscriber_node = Node(
        package="rdfp",
        executable="gripper_command_subscriber",
        name="gripper_command_subscriber",
        output="screen",
        emulate_tty=True,
        remappings=[
            ("~/gripper_cmds", "/gripper_control/gripper_cmds"),
        ],
    )

    # --- rdfp 애플리케이션: TargetJointStatesExecutor ---
    # 재생 도구가 발행하는 `/target_joint_states` (rdfp_msgs/TargetJointStates) 를
    # 받아 길이 1 짜리 `trajectory_msgs/JointTrajectory` 로 래핑하여
    # `/panda_arm_controller/joint_trajectory` 로 흘려보낸다 — panda_arm_controller
    # 가 이 토픽을 consume 하여 실제 관절 궤적을 실행한다.
    target_joint_states_executor_node = Node(
        package="rdfp",
        executable="target_joint_states_executor",
        name="target_joint_states_executor",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "joint_names": [
                "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                "panda_joint5", "panda_joint6", "panda_joint7",
            ],
        }],
        remappings=[
            ("target_joint_states", "/target_joint_states"),
            ("joint_trajectory", "/panda_arm_controller/joint_trajectory"),
        ],
    )

    # panda_hand_controller 기동 완료 후 MoveIt/주변 노드와 rdfp 애플리케이션
    # 노드를 일괄 spawn 한다.
    controller_startup_handlers = create_controller_startup_handlers(
        ros2_control_node,
        joint_state_broadcaster_spawner,
        panda_arm_controller_spawner,
        panda_hand_controller_spawner,
        [
            move_group_node, servo_node, rviz_node,
            rdfp_image_viewer_node, gripper_command_subscriber_node,
            target_joint_states_executor_node,
        ],
    )

    return [
        # --- YAML 기본값을 가진 argument 들 (config_file resolve 후 결정) ---
        *_declare_arguments(config),
        # --- 즉시 기동 노드 ---
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        # --- 순차 기동 핸들러 (controllers -> moveit + rdfp app nodes) ---
        *controller_startup_handlers,
    ]


def generate_launch_description() -> LaunchDescription:
    # `config_file` 만 declaration 시점에 노출하고, 그 값에 의존하는 YAML 로딩과
    # 나머지 argument / 노드 생성은 `OpaqueFunction` 안에서 수행한다. 이렇게 해야
    # top-level CLI 뿐 아니라 `IncludeLaunchDescription(..., launch_arguments=...)`
    # 경유 호출에서도 동일하게 `config_file` override 가 반영된다.
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=_default_config_path(),
        description=(
            "Path to the rdfp_panda_mock YAML configuration file. "
            "기본값은 <rdfp share>/config/rdfp_panda_mock.yaml 이며, "
            "CLI 또는 IncludeLaunchDescription launch_arguments 로 "
            "'config_file:=<path>' 를 주면 덮어쓸 수 있다 "
            "($HOME 등 쉘 확장은 쉘에 맡긴다)."
        ),
    )
    return LaunchDescription([
        config_file_arg,
        OpaqueFunction(function=_build_actions),
    ])
