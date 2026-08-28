"""학습 데이터 수집 노드만 별도로 기동하는 launch.

이미 떠 있는 **로봇 제어 스택 위에** 수집 계층을 얹는다. 다음 두 조합이 같은
노드 그래프를 만든다.

    ros2 launch rdfp rdfp_panda_mock.launch.py

    ros2 launch robot_control panda_mock.launch.py     # 터미널 1
    ros2 launch rdfp rdfp_collect.launch.py           # 터미널 2

기동 순서
---------
수집 노드 넷은 구독자·상태머신·서비스 서버라 **발행자보다 먼저 떠도 무방하다.**
그래서 `panda_mock` 의 controller 순차 기동 체인에 끼어들 필요가 없고, 별도
launch 로 분리할 수 있다.

유일한 예외가 `target_joint_cmds_publisher` 의 관절 이름 조회
(`<arm controller>/get_parameters`)인데, `__init__` 이 아니라 타이머에서
`joint_names_timeout`(기본 10초) 안에 비동기로 수행하며 실패해도 `JointState.name`
이 비는 것으로 그친다. **제어 스택을 먼저 띄우면 이 조회가 오히려 더 안전하다** —
묶음 launch 는 hand 컨트롤러 spawner 종료 직후 동시에 뜨므로 여유가 더 적다.

백엔드별 인자
-------------
arm 명령 채널이 백엔드마다 다르므로 `arm_cmd_source` 로 고른다.

===================== ======================== ==================================
백엔드                 arm_cmd_source           입력 토픽 (자동)
===================== ======================== ==================================
panda_mock            joint_trajectory         /panda_arm_controller/joint_trajectory
panda_jgpc_mock       float64_multi_array      /panda_arm_controller/commands
panda_gazebo          joint_trajectory         /panda_arm_controller/joint_trajectory
===================== ======================== ==================================

Gazebo 백엔드는 `use_sim_time:=true` 를 함께 준다 — 백엔드가 /clock 을 쓰므로
수집 노드만 벽시계를 쓰면 타임스탬프가 어긋난다.

주의 — 설정값 정합
------------------
`camera_image_topic` / `camera_resolution` / `camera_fps` 는 제어 스택과 **공유**
한다. 기본값이 같은 `image_pipeline.yaml` 에서 오므로 손대지 않으면 자동으로
일치하지만, **한쪽에만 override 하면 조용히 어긋난다** (레코더가 프레임을 버리고
auto-stop 한다). 바꿀 때는 양쪽에 같은 `config_file:=` 을 준다.
"""

from __future__ import annotations

from typing import Any

import os
import sys

# ROS2 launch 러너는 본 파일을 단일 스크립트로 로드한다.
sys.path.insert(0, os.path.dirname(__file__))

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter

from robot_control.launch_helpers.image_pipeline import (
    declare_config_file_argument,
    declare_image_pipeline_arguments,
    load_config,
)

# arm 명령 채널. `target_joint_cmds_publisher` 의 `source` 파라미터 값과
# 그때 노드가 구독하는 토픽 이름(remap 대상), 기본 입력 토픽의 대응표다.
ARM_CMD_SOURCES: dict[str, tuple[str, str]] = {
    # source              : (노드 내부 토픽 이름, 기본 입력 토픽)
    "joint_trajectory": ("joint_trajectory", "/panda_arm_controller/joint_trajectory"),
    "float64_multi_array": ("commands", "/panda_arm_controller/commands"),
    # 토픽 연동형 시뮬레이터(펑션베이). 명령이 이미 JointState 다.
    "joint_state": ("arm_command", "/input/panda_joint"),
}

DEFAULT_ARM_CONTROLLER_NODE_NAME = "/panda_arm_controller"


def _declare_arguments() -> list[DeclareLaunchArgument]:
    """이미지 파이프라인 밖의 인자를 선언한다."""
    return [
        DeclareLaunchArgument(
            "log_level", default_value="info",
            choices=["debug", "info", "warn", "error", "fatal"],
            description="session_control 의 로그 레벨",
        ),
        DeclareLaunchArgument(
            "arm_cmd_source", default_value="joint_trajectory",
            choices=sorted(ARM_CMD_SOURCES) + ["none"],
            description=(
                "arm 명령 채널. JTC=joint_trajectory / JGPC=float64_multi_array / "
                "토픽 연동 시뮬레이터=joint_state. none 이면 "
                "target_joint_cmds_publisher 를 띄우지 않는다"
            ),
        ),
        DeclareLaunchArgument(
            "target_joint_cmds_input_topic", default_value="",
            description=(
                "target_joint_cmds_publisher 의 입력 토픽. 비우면 arm_cmd_source 에 "
                "맞는 기본값을 쓴다"
            ),
        ),
        DeclareLaunchArgument(
            "arm_controller_node_name", default_value=DEFAULT_ARM_CONTROLLER_NODE_NAME,
            description="관절 이름을 조회할 arm 컨트롤러 노드 이름",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            choices=["true", "false"],
            description="Gazebo 등 시뮬레이션 백엔드와 함께 쓸 때 true",
        ),
    ]


def _build_actions(context: LaunchContext) -> list[Any]:
    """`config_file` resolve 이후 인자와 노드를 구성한다."""
    image_config = load_config(LaunchConfiguration("config_file").perform(context))

    arm_cmd_source = LaunchConfiguration("arm_cmd_source").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"

    # --- session_control ---
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

    # --- 세션 상태 오버레이 뷰어 ---
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

    # --- 서비스 기반 녹화 ---
    # fps / resolution 은 카메라 설정을 그대로 써야 프레임이 버려지지 않는다.
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

    actions: list[Any] = [session_control_node, rdfp_image_viewer_node, image_recorder_node]

    # --- action(명령값) 채널 ---
    if arm_cmd_source != "none":
        internal_topic, default_input = ARM_CMD_SOURCES[arm_cmd_source]
        input_topic = (LaunchConfiguration("target_joint_cmds_input_topic").perform(context)
                       or default_input)
        actions.append(Node(
            package="robot_control",
            executable="target_joint_cmds_publisher",
            name="target_joint_cmds_publisher",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "source": arm_cmd_source,
                "controller_node_name": LaunchConfiguration("arm_controller_node_name"),
            }],
            remappings=[(internal_topic, input_topic)],
        ))

    # 시뮬레이션 백엔드와 클럭을 맞춘다. 노드 생성보다 먼저 와야 적용된다.
    if use_sim_time:
        actions.insert(0, SetParameter(name="use_sim_time", value=True))

    # YAML 에 의존하는 인자만 여기서 선언한다. 나머지는 generate_launch_description
    # 에서 미리 선언해야 한다 — OpaqueFunction 안에서 perform() 하려면 그 시점에
    # 이미 선언되어 있어야 하기 때문이다.
    return [*declare_image_pipeline_arguments(image_config), *actions]


def generate_launch_description() -> LaunchDescription:
    # YAML 과 무관한 인자는 여기서 선언한다. `_build_actions` 가 이들을
    # `perform()` 하므로 OpaqueFunction 보다 앞서야 하고, 덕분에 `--show-args`
    # 에도 그대로 드러난다.
    return LaunchDescription([
        declare_config_file_argument("config_file"),
        *_declare_arguments(),
        OpaqueFunction(function=_build_actions),
    ])
