"""펑션베이 백엔드 + rdfp 수집 노드까지 함께 기동하는 풀 스택 launch.

:mod:`panda_functionbay.launch` (시뮬레이터 연동 + MoveIt) 를 그대로 include 하고 그
위에 수집 계층을 얹는다. ``panda_mock`` ↔ ``rdfp_panda_mock`` 관계의 펑션베이 판이다.

**시뮬레이터(Unity + ros_tcp_endpoint)가 먼저 떠 있어야 한다.** 백엔드의
``readiness_gate`` 가 ``/output/panda_joint`` 첫 메시지를 기다리며, 오지 않으면
launch 전체가 실패 종료한다.

카메라 설정의 출처
------------------
**Isaac 과 달리 공유 정의 파일이 없다.** Isaac 은 ``isaac_scene.json`` 의 ``camera``
블록을 시뮬레이터 쪽 ``setup_graph.py`` 와 나눠 쓰지만, 펑션베이에는 대응하는 파일이
없다 — 카메라 설정이 Unity 씬 안에 있고 우리가 읽을 수 없다. 그래서 값을 여기에
상수로 둔다.

⚠️ **레코더는 CFR 이라 선언한 fps 가 곧 영상의 시간축이 된다.** 소스 주기와 다르면
재생 속도가 어긋나며, 파이프라인에 데시메이션 로직이 없어 자동으로 맞춰지지 않는다.

    2026-09-01 실측: 시뮬레이터 카메라 = 9.606 Hz (관절 보고 주기와 무관)
    목표          : 5 Hz

즉 **지금 이 launch 를 그대로 쓰면 영상이 약 1.9배 빠르게 재생된다.** 5 Hz 가 되려면
둘 중 하나가 선행되어야 한다.

  1. 시뮬레이터 카메라를 5 Hz 로 설정 (벤더 요청 — 설정 가능 여부 미확인)
  2. ``/camera_image`` 를 5 Hz 로 솎는 데시메이션 노드

목표값을 기본으로 둔 이유는, 소스가 맞춰지는 순간 바로 정합하고 그 전까지는 위
경고가 이 파일에 남아 있게 하기 위해서다. 소스를 바꾸지 않은 채 쓰려면
``image_recorder_fps:=10`` 으로 실측값에 맞춘다.

arm 명령 채널
-------------
펑션베이는 ``/input/panda_joint`` 로 ``sensor_msgs/JointState`` 를 받는다. 그래서
``target_joint_cmds_publisher`` 의 ``source`` 는 ``joint_state`` 이고, 이 경로는
**컨트롤러의 ``joints`` 파라미터를 조회하지 않는다** — 메시지에 이름이 이미 들어 있다.
ros2_control 컨트롤러가 없는 이 스택에서 중요한 성질이다.

사용 예
-------
    ros2 launch rdfp rdfp_panda_functionbay.launch.py
    ros2 launch rdfp rdfp_panda_functionbay.launch.py image_recorder_fps:=10
    ros2 launch rdfp rdfp_panda_functionbay.launch.py image_recorder_auto_start:=true
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

# 시뮬레이터가 발행하는 이미지 토픽. Unity 쪽 설정과 같아야 한다.
_CAMERA_IMAGE_TOPIC = "/camera_image"
# 목표 주기. 위 docstring 의 경고를 읽는다 — 실측 소스는 아직 9.606 Hz 다.
_CAMERA_FPS = "5"
# 시뮬레이터 카메라 해상도 (2026-09-01 실측).
_CAMERA_RESOLUTION = "640x480"
# 펑션베이가 JointState 명령을 받는 토픽. panda_functionbay.launch 의 값과 같아야 한다.
_ARM_COMMAND_TOPIC = "/input/panda_joint"


def _included_backend() -> IncludeLaunchDescription:
    backend_launch = os.path.join(
        get_package_share_directory("robot_control"), "launch", "panda_functionbay.launch.py")
    # 백엔드와 공유하는 argument 만 전달한다. 나머지는 백엔드 기본값을 쓴다.
    forwarded = {
        "camera_image_topic": LaunchConfiguration("camera_image_topic"),
        "log_level": LaunchConfiguration("log_level"),
    }
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(backend_launch), launch_arguments=forwarded.items())


def _declare_arguments() -> list[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            "log_level", default_value="info",
            choices=["debug", "info", "warn", "error", "fatal"],
            description="MoveIt·수집 노드 로그 레벨"),
        # --- 카메라 (시뮬레이터가 발행한다) ---
        DeclareLaunchArgument(
            "camera_image_topic", default_value=_CAMERA_IMAGE_TOPIC,
            description="시뮬레이터가 발행하는 이미지 토픽"),
        DeclareLaunchArgument(
            "camera_resolution", default_value=_CAMERA_RESOLUTION,
            description="레코더 해상도. 시뮬레이터 렌더 해상도와 같아야 한다"),
        DeclareLaunchArgument(
            "image_recorder_fps", default_value=_CAMERA_FPS,
            description=("레코더 FPS. **CFR 이라 이 값이 곧 영상의 시간축이다** — "
                         "소스 주기와 다르면 재생 속도가 어긋난다 (docstring 참고)")),
        # --- 수집 노드 ---
        DeclareLaunchArgument(
            "enable_image_viewer_node", default_value="true",
            choices=["true", "false"], description="세션 상태를 겹쳐 보여주는 뷰어"),
        DeclareLaunchArgument(
            "enable_image_recorder_node", default_value="true",
            choices=["true", "false"], description="MP4 레코더"),
        DeclareLaunchArgument(
            "image_recorder_output_dir", default_value="/tmp/rdfp_recordings",
            description="레코더 출력 디렉터리"),
        DeclareLaunchArgument(
            "image_recorder_auto_start", default_value="false",
            choices=["true", "false"], description="기동과 동시에 녹화를 시작한다"),
        DeclareLaunchArgument(
            "target_joint_cmds_input_topic", default_value=_ARM_COMMAND_TOPIC,
            description="arm 명령 토픽. /target_joint_cmds 로 변환할 원본이다"),
    ]


def generate_launch_description() -> LaunchDescription:
    session_control_node = Node(
        package="rdfp", executable="session_control_node", name="session_control",
        output="screen", emulate_tty=True,
        ros_arguments=[
            "--log-level",
            [TextSubstitution(text="session_control:="), LaunchConfiguration("log_level")],
        ],
    )

    # arm 명령 → /target_joint_cmds (데이터셋의 action 채널).
    # source=joint_state 는 메시지의 이름을 그대로 쓰므로 컨트롤러 조회가 없다.
    target_joint_cmds_publisher = Node(
        package="robot_control", executable="target_joint_cmds_publisher",
        name="target_joint_cmds_publisher", output="screen", emulate_tty=True,
        parameters=[{"source": "joint_state"}],
        remappings=[("arm_command", LaunchConfiguration("target_joint_cmds_input_topic"))],
    )

    rdfp_image_viewer_node = Node(
        package="rdfp", executable="rdfp_image_viewer_node", name="rdfp_image_viewer_node",
        output="screen", emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_image_viewer_node")),
        remappings=[("image", LaunchConfiguration("camera_image_topic"))],
    )

    image_recorder_node = Node(
        package="rdfp", executable="image_recorder_node",
        output="screen", emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_image_recorder_node")),
        parameters=[{
            "output_dir": LaunchConfiguration("image_recorder_output_dir"),
            "fps": LaunchConfiguration("image_recorder_fps"),
            "resolution": LaunchConfiguration("camera_resolution"),
            "auto_start": LaunchConfiguration("image_recorder_auto_start"),
        }],
        remappings=[("image", LaunchConfiguration("camera_image_topic"))],
    )

    return LaunchDescription([
        # **선언이 백엔드 include 보다 먼저 와야 한다** — 뒤에 두면 전달할
        # LaunchConfiguration 을 아직 모르는 상태에서 참조해 죽는다.
        *_declare_arguments(),
        _included_backend(),
        session_control_node,
        target_joint_cmds_publisher,
        rdfp_image_viewer_node,
        image_recorder_node,
    ])
