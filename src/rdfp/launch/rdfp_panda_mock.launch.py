"""Panda + MoveIt2 풀 스택과 rdfp 애플리케이션 노드를 함께 기동하는 launch.

:mod:`panda_mock.launch` 의 ``ros2_control`` / ``moveit`` / ``rviz2`` /
``camera`` / ``ee_pose`` / ``gripper`` 스택에 더해, :mod:`rdfp.launch`
에서 기동하던 rdfp 애플리케이션 노드들을 하나의 launch 로 통합한다.

기능은 다음 단위로 분리되어 있다.

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
- ``scene``: /mock_scene_state (``enable_scene_node:=false`` 로 비활성화 가능)
- ``rdfp 애플리케이션``:
    - /session_control
    - /rdfp_image_viewer_node
    - /image_recorder
    - /target_joint_cmds_publisher

기본 기동 정책은 ``panda_mock`` 과 동일하다: ``panda_hand_controller`` spawner
가 종료되면 상위 노드들을 일괄 spawn 한다.

설정 파일
---------

각 argument 의 **기본값** 은 YAML 설정 파일에서 로드된다.

- 기본 경로: ``<rdfp share>/config/panda_robot.yaml``
- ``config_file:=<path>`` launch argument 로 다른 YAML 을 지정할 수 있다.
  ``$HOME`` / ``~`` 같은 경로 확장은 쉘에 맡긴다
  (예: ``config_file:=$HOME/my.yaml`` — 쉘이 먼저 확장한 절대경로가 전달된다).
- CLI 에서 ``arg:=value`` 로 개별 argument 를 덮어쓰는 것은 그대로 동작한다.

YAML 파일의 구조는 ``config/panda_robot.yaml`` 을 참고한다.
"""

from __future__ import annotations

from typing import Any

import os

import yaml


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

from robot_control.launch_helpers.camera import create_camera_node
from robot_control.launch_helpers.controller import (
    create_joint_state_broadcaster_spawner,
    create_panda_arm_controller_spawner,
    create_panda_hand_controller_spawner,
    create_ros2_control_node,
)
from robot_control.launch_helpers.controller_startup import create_controller_startup_handlers
from robot_control.launch_helpers.ee_pose import create_ee_pose_node
from robot_control.launch_helpers.gripper import create_gripper_control_node
from robot_control.launch_helpers.image_pipeline import (
    declare_config_file_argument as declare_image_pipeline_config_file_argument,
    declare_image_pipeline_arguments,
    load_config as load_image_pipeline_config,
)
from robot_control.launch_helpers.common import (
    MOVEIT_CONFIGS_PACKAGE_NAME,
    build_moveit_config,
    build_servo_params,
    create_move_group_node,
    create_robot_state_publisher,
    create_rviz_node,
    create_servo_node,
    create_static_tf_node,
)
from robot_control.launch_helpers.scene import create_mock_scene_node, declare_scene_arguments

# YAML 설정 파일의 기본 경로. setup.py 가 ``config/*`` 를
# ``share/rdfp/config/`` 로 설치하므로 package share 에서 읽는다.
DEFAULT_CONFIG_RELPATH = os.path.join("config", "panda_robot.yaml")


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
    # 구 키(`target_joint_states`)를 쓰는 외부 YAML(도커 RDFP_CONFIG_DIR 마운트 등)
    # 과의 호환을 위해 새 키를 우선하되 없으면 구 키로 폴백한다.
    tjc = config.get("target_joint_cmds") or config["target_joint_states"]

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
        # --- target_joint_cmds ---
        DeclareLaunchArgument(
            "target_joint_cmds_input_topic",
            default_value=_as_launch_str(tjc["input_topic"]),
            description=(
                "Remap target for target_joint_cmds_publisher input topic "
                "('joint_trajectory'); typically /panda_arm_controller/joint_trajectory"
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

    # 이미지 파이프라인(camera / image_viewer / image_recorder) 설정은 별도 YAML 에서
    # 온다 — 카메라를 띄우는 launch 가 넷이라 기본값을 한 곳으로 모았다.
    image_config_path = LaunchConfiguration("image_pipeline_config_file").perform(context)
    image_config = load_image_pipeline_config(image_config_path)

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
    move_group_node = create_move_group_node(moveit_config)
    servo_node = create_servo_node(moveit_config, servo_params)
    rviz_node = create_rviz_node(moveit_config)
    camera_node = create_camera_node()
    ee_pose_node = create_ee_pose_node()
    gripper_control_node = create_gripper_control_node()
    # 씬 노드는 move_group 의 planning scene 에 의존하지만 생성자에서 서비스를
    # 기다리지 않으므로 같은 그룹에서 동시에 spawn 해도 안전하다.
    scene_node = create_mock_scene_node()

    # --- rdfp 애플리케이션: SessionControlNode ---
    # 로그 레벨은 노드 스코프로만 적용하여 전역 기본 레벨을 건드리지 않는다.
    session_control_node = Node(
        package="rdfp",
        executable="session_control_node",
        name="session_control",
        output="screen",
        emulate_tty=True,
        ros_arguments=[
            "--log-level",
            [TextSubstitution(text="session_control:="), LaunchConfiguration("log_level")],
        ],
    )

    # --- rdfp 애플리케이션: RdfpImageViewerNode ---
    # session 토픽을 구독하여 수신 프레임 좌상단에 상태 오버레이를 덧입혀 표시한다.
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

    # --- rdfp 애플리케이션: ImageRecorderNode ---
    # service 기반, 세션 토픽과 독립적으로 동작한다. fps / resolution 은 카메라
    # 설정을 그대로 사용해 파라미터 불일치로 인한 프레임 drop 을 방지한다.
    image_recorder_node = Node(
        package="rdfp",
        executable="image_recorder_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_image_recorder_node")),
        parameters=[{
            "output_dir": LaunchConfiguration("image_recorder_output_dir"),
            "fps": LaunchConfiguration("image_recorder_fps"),
            "resolution": LaunchConfiguration("camera_resolution"),
            "auto_start": LaunchConfiguration("image_recorder_auto_start"),
        }],
        remappings=[
            ("image", LaunchConfiguration("camera_image_topic")),
        ],
    )

    # --- rdfp 애플리케이션: TargetJointCmdsPublisher ---
    # servo_node 가 발행하는 JointTrajectory 의 마지막 point 를 뽑아 현재 시각을
    # header.stamp 로 채운 `sensor_msgs/JointState` 로 변환해 `target_joint_cmds`
    # 토픽에 재발행한다 — 학습 데이터의 action(명령값) 채널이다.
    #
    # JGPC 스택(rdfp_panda_jgpc_mock)은 같은 노드를 source=float64_multi_array 로
    # 띄워 `/panda_arm_controller/commands` 를 동일한 JointState 로 변환한다.
    # 두 스택이 같은 토픽 이름과 같은 메시지 타입을 쓰므로, 데이터셋에서는
    # `joint_states` 테이블에 topic_id 로만 구분되어 함께 적재된다 — 관측값
    # (`/joint_states`) 과 대칭적으로 다룰 수 있다.
    #
    # 이전의 `target_joint_states_publisher`(rdfp_msgs/TargetJointStates) 는 본
    # 런치에서 제거되었다. 노드 구현 자체는 남아 있으므로 필요하면 되살릴 수 있다.
    target_joint_cmds_publisher = Node(
        package="robot_control",
        executable="target_joint_cmds_publisher",
        name="target_joint_cmds_publisher",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "source": "joint_trajectory",
        }],
        remappings=[
            ("joint_trajectory", LaunchConfiguration("target_joint_cmds_input_topic")),
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
            move_group_node, servo_node, rviz_node, camera_node, ee_pose_node,
            gripper_control_node, scene_node,
            session_control_node, rdfp_image_viewer_node, image_recorder_node,
            target_joint_cmds_publisher,
        ],
    )

    return [
        # --- YAML 기본값을 가진 argument 들 (config_file resolve 후 결정) ---
        *_declare_arguments(config),
        *declare_image_pipeline_arguments(image_config),
        # 씬 노드는 YAML 블록 없이 helper 의 하드코딩 기본값을 쓴다 — 노브가
        # `enable_scene_node` / `scene_publish_rate` 둘뿐이고 스택마다 달라질
        # 값이 아니다. YAML 로 옮기면 기존 외부 설정 파일이 KeyError 로 깨진다.
        *declare_scene_arguments(),
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
            "기본값은 <rdfp share>/config/panda_robot.yaml 이며, "
            "CLI 또는 IncludeLaunchDescription launch_arguments 로 "
            "'config_file:=<path>' 를 주면 덮어쓸 수 있다 "
            "($HOME 등 쉘 확장은 쉘에 맡긴다)."
        ),
    )
    return LaunchDescription([
        config_file_arg,
        declare_image_pipeline_config_file_argument('image_pipeline_config_file'),
        OpaqueFunction(function=_build_actions),
    ])
