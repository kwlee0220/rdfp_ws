"""Gazebo 백엔드 + rdfp 애플리케이션 노드까지 함께 기동하는 풀 스택 launch.

:mod:`panda_gazebo.launch` (Gazebo + MoveIt 백엔드)를 그대로 include 하고, 그
위에 :mod:`rdfp_panda_mock.launch` 가 띄우던 rdfp 애플리케이션 노드를 얹는다.
즉 ``panda_mock`` ↔ ``rdfp_panda_mock`` 관계의 Gazebo 판이다.

추가되는 rdfp 애플리케이션 노드
-------------------------------
- ``session_control`` : teleop / recorder 가 의존하는 세션 상태머신
  (``start_session`` / ``stop_session`` 서비스 제공). **teleop_keyboard 는 이
  노드가 없으면 기동 시 예외로 죽는다.**
- ``target_joint_cmds_publisher`` : servo 출력(JointTrajectory)의 마지막 point 를
  ``sensor_msgs/JointState`` 로 변환해 ``target_joint_cmds`` 로 재발행
  (dataset 의 action 채널).
- ``rdfp_image_viewer_node`` / ``image_recorder`` : 카메라 토픽 의존. 기본
  비활성. ``simulate_camera:=true`` 로 gz 카메라를 켠 뒤 enable 플래그로 활성화.

모든 노드에 ``use_sim_time:=true`` 를 전파하므로, teleop 등 외부 노드도
``--ros-args -p use_sim_time:=true`` 로 맞추는 것을 권장한다.

사용 예
-------
    ros2 launch rdfp rdfp_panda_gazebo.launch.py
    ros2 launch rdfp rdfp_panda_gazebo.launch.py enable_rviz:=true
    ros2 launch rdfp rdfp_panda_gazebo.launch.py simulate_camera:=true \\
        enable_image_viewer_node:=true
"""

from __future__ import annotations

import os
import sys

# ROS2 launch 러너는 본 파일을 단일 스크립트로 로드하므로, 같은 디렉터리의
# sibling 모듈을 패키지 import 로 가져올 수 없다. 이 파일의 디렉터리를
# sys.path 의 맨 앞에 추가하여 top-level 모듈처럼 import 한다 (sibling 우선 보장; append 로 바꾸지 말 것).
sys.path.insert(0, os.path.dirname(__file__))

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter


def _included_backend() -> IncludeLaunchDescription:
    """panda_gazebo.launch (Gazebo + MoveIt 백엔드)를 include 한다."""
    panda_gazebo_launch = os.path.join(
        get_package_share_directory("rdfp"), "launch", "panda_gazebo.launch.py"
    )
    # 백엔드와 공유하는 argument 만 전달한다. 나머지는 백엔드 기본값을 사용한다.
    forwarded = {
        "world": LaunchConfiguration("world"),
        "simulate_camera": LaunchConfiguration("simulate_camera"),
        "enable_rviz": LaunchConfiguration("enable_rviz"),
        "log_level": LaunchConfiguration("log_level"),
        "camera_image_topic": LaunchConfiguration("camera_image_topic"),
    }
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(panda_gazebo_launch),
        launch_arguments=forwarded.items(),
    )


def _declare_arguments() -> list[DeclareLaunchArgument]:
    """rdfp_panda_gazebo 전용/공유 launch argument 를 선언한다."""
    return [
        # --- 백엔드로 전달되는 공유 argument ---
        DeclareLaunchArgument(
            "world", default_value="empty.sdf",
            description="Gazebo world file/name",
        ),
        DeclareLaunchArgument(
            "simulate_camera", default_value="false",
            description="Attach a gz camera sensor and bridge it to ROS",
        ),
        DeclareLaunchArgument(
            "enable_rviz", default_value="false",
            description="Also launch RViz2",
        ),
        DeclareLaunchArgument(
            "log_level", default_value="info",
            choices=["debug", "info", "warn", "error", "fatal"],
            description="Log level for MoveIt / session_control nodes",
        ),
        DeclareLaunchArgument(
            "camera_image_topic", default_value="/camera/image_raw",
            description="Camera image topic (gz bridge target + app image input)",
        ),
        # --- rdfp 애플리케이션 노드 argument ---
        DeclareLaunchArgument(
            "enable_image_viewer_node", default_value="false",
            description="Start rdfp_image_viewer_node (needs a camera topic)",
        ),
        DeclareLaunchArgument(
            "enable_image_recorder_node", default_value="false",
            description="Start image_recorder_node (needs a camera topic)",
        ),
        DeclareLaunchArgument(
            "image_recorder_fps", default_value="10",
            description="Target FPS for image_recorder_node",
        ),
        DeclareLaunchArgument(
            "image_recorder_output_dir", default_value="/tmp/recordings",
            description="Output directory for image_recorder_node MP4 files",
        ),
        DeclareLaunchArgument(
            "image_recorder_auto_start", default_value="false",
            description="Start recording immediately at launch",
        ),
        DeclareLaunchArgument(
            "camera_resolution", default_value="640x480",
            description="Resolution for image_recorder_node",
        ),
        DeclareLaunchArgument(
            "target_joint_cmds_input_topic",
            default_value="/panda_arm_controller/joint_trajectory",
            description=(
                "Input JointTrajectory topic for target_joint_cmds_publisher "
                "(servo output)"
            ),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    # --- rdfp 애플리케이션: 세션 상태머신 (teleop / recorder 의존) ---
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

    # --- rdfp 애플리케이션: arm 명령 → JointState 재발행 (학습 데이터 action) ---
    target_joint_cmds_publisher = Node(
        package="rdfp",
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

    # --- rdfp 애플리케이션: 카메라 의존 노드 (기본 비활성) ---
    rdfp_image_viewer_node = Node(
        package="rdfp",
        executable="rdfp_image_viewer_node",
        name="rdfp_image_viewer_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_image_viewer_node")),
        remappings=[("image", LaunchConfiguration("camera_image_topic"))],
    )
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
        remappings=[("image", LaunchConfiguration("camera_image_topic"))],
    )

    return LaunchDescription([
        # 백엔드(panda_gazebo)도 use_sim_time 을 설정하지만, parent 스코프에서
        # 추가하는 앱 노드에도 전파되도록 여기서도 선언한다.
        SetParameter(name="use_sim_time", value=True),
        *_declare_arguments(),
        _included_backend(),
        session_control_node,
        target_joint_cmds_publisher,
        rdfp_image_viewer_node,
        image_recorder_node,
    ])
