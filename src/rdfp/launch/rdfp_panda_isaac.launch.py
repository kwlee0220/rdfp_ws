"""Isaac Sim 백엔드 + rdfp 수집 노드까지 함께 기동하는 풀 스택 launch.

:mod:`panda_isaac.launch` (Isaac 연동 + MoveIt) 를 그대로 include 하고 그 위에
수집 계층을 얹는다. ``panda_mock`` ↔ ``rdfp_panda_mock`` 관계의 Isaac 판이다.

카메라 설정의 출처
------------------
**``config/isaac_scene.json`` 하나다.** 다른 백엔드는 ``image_pipeline.yaml`` 을
읽지만 Isaac 은 그럴 수 없다 — 이미지를 카메라 노드가 아니라 **시뮬레이터가 직접**
발행하므로, 해상도·주파수·토픽 이름을 정하는 쪽이 Isaac 의 렌더 프로덕트다. 그 값이
``isaac_scene.json`` 의 ``camera`` 블록에 있고 ``setup_graph.py`` 가 같은 파일을
읽는다. 여기서 그것을 그대로 가져와 뷰어·레코더의 기본값으로 쓴다.

그래서 **해상도를 바꾸려면 JSON 한 곳만 고치면 된다.** 고친 뒤에는

  1. ``colcon build --packages-select robot_control`` (share 사본 갱신)
  2. Isaac 에서 ``setup_graph.py`` 재실행

두 가지가 모두 필요하다 — 빌드를 빠뜨리면 Isaac 만 새 해상도로 바뀌고 레코더는
옛 값으로 프레임을 버린다(§7 단일 출처).

arm 명령 채널
-------------
Isaac 은 ``/isaac/arm_command`` 로 ``sensor_msgs/JointState`` 를 받는다. 그래서
``target_joint_cmds_publisher`` 의 ``source`` 는 ``joint_state`` 이고, 이 경로는
**컨트롤러의 ``joints`` 파라미터를 조회하지 않는다** — 메시지에 이름이 이미
들어 있기 때문이다. ros2_control 컨트롤러가 없는 Isaac 스택에서 중요한 성질이다.

사용 예
-------
    ros2 launch rdfp rdfp_panda_isaac.launch.py
    ros2 launch rdfp rdfp_panda_isaac.launch.py enable_rviz:=true
    ros2 launch rdfp rdfp_panda_isaac.launch.py image_recorder_auto_start:=true
"""

from __future__ import annotations

import json
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter

_SCENE_CONFIG_RELPATH = os.path.join("config", "isaac_scene.json")
# Isaac 이 JointState 명령을 받는 토픽. panda_isaac.launch 의 값과 같아야 한다.
_ARM_COMMAND_TOPIC = "/isaac/arm_command"


def _camera_defaults() -> dict:
    """`isaac_scene.json` 의 camera 블록에서 수집 노드 기본값을 뽑는다.

    JSON 이 없거나 camera 블록이 비면 **조용히 기본값으로 넘어가지 않고** 예외를
    낸다 — 그 상태로 뜨면 레코더가 엉뚱한 해상도로 프레임을 버리는데, 증상이
    '녹화가 안 된다'로만 보여 원인을 찾기 어렵다.
    """
    path = os.path.join(get_package_share_directory("robot_control"), _SCENE_CONFIG_RELPATH)
    with open(path, encoding="utf-8") as f:
        camera = (json.load(f) or {}).get("camera")
    if not camera:
        raise RuntimeError(f"no 'camera' block in {path}")

    width, height = camera["resolution"]
    return {
        "image_topic": camera["image_topic"],
        "resolution": f"{int(width)}x{int(height)}",
        "fps": str(int(camera["fps"])),
    }


def _included_backend() -> IncludeLaunchDescription:
    """panda_isaac.launch (Isaac 연동 + MoveIt) 를 include 한다."""
    backend_launch = os.path.join(
        get_package_share_directory("robot_control"), "launch", "panda_isaac.launch.py")
    # 백엔드와 공유하는 argument 만 전달한다. 나머지는 백엔드 기본값을 쓴다.
    forwarded = {
        "enable_rviz": LaunchConfiguration("enable_rviz"),
        "enable_gripper": LaunchConfiguration("enable_gripper"),
        "enable_scene": LaunchConfiguration("enable_scene"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(backend_launch), launch_arguments=forwarded.items())


def _declare_arguments(camera: dict) -> list[DeclareLaunchArgument]:
    return [
        # --- 백엔드로 전달되는 공유 argument ---
        DeclareLaunchArgument(
            "enable_rviz", default_value="true",
            choices=["true", "false"], description="RViz2 도 함께 띄운다"),
        DeclareLaunchArgument(
            "enable_gripper", default_value="true",
            choices=["true", "false"],
            description="그리퍼 액션 브리지. 수집에는 파지 사건이 필요하므로 기본 on"),
        DeclareLaunchArgument(
            "enable_scene", default_value="true",
            choices=["true", "false"],
            description="scene 물체 상태(/scene/objects). 데이터셋 채널이므로 기본 on"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            choices=["true", "false"],
            description="Isaac 의 /clock 을 쓴다. false 로 두면 타임스탬프가 어긋난다"),
        # --- 수집 계층 ---
        DeclareLaunchArgument(
            "log_level", default_value="info",
            choices=["debug", "info", "warn", "error", "fatal"],
            description="session_control 의 로그 레벨"),
        DeclareLaunchArgument(
            "camera_image_topic", default_value=camera["image_topic"],
            description="이미지 토픽. 기본값은 isaac_scene.json 의 camera.image_topic"),
        DeclareLaunchArgument(
            "camera_resolution", default_value=camera["resolution"],
            description="레코더 해상도. 기본값은 isaac_scene.json 의 camera.resolution"),
        DeclareLaunchArgument(
            "image_recorder_fps", default_value=camera["fps"],
            description="레코더 FPS. 기본값은 isaac_scene.json 의 camera.fps"),
        DeclareLaunchArgument(
            "enable_image_viewer_node", default_value="false",
            choices=["true", "false"],
            description="세션 상태 오버레이 뷰어. 창을 띄우므로 기본 off"),
        DeclareLaunchArgument(
            "enable_image_recorder_node", default_value="true",
            choices=["true", "false"], description="서비스 기반 MP4 레코더"),
        DeclareLaunchArgument(
            "image_recorder_output_dir", default_value="/tmp/recordings",
            description="레코더 MP4 출력 디렉터리"),
        DeclareLaunchArgument(
            "image_recorder_auto_start", default_value="false",
            choices=["true", "false"], description="기동 즉시 녹화를 시작한다"),
        DeclareLaunchArgument(
            "target_joint_cmds_input_topic", default_value=_ARM_COMMAND_TOPIC,
            description="target_joint_cmds_publisher 의 입력 토픽 (Isaac arm 명령)"),
    ]


def generate_launch_description() -> LaunchDescription:
    camera = _camera_defaults()

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

    # fps / resolution 은 Isaac 의 렌더 설정과 같아야 한다 — 어긋나면 프레임을 버린다.
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
        # **선언이 SetParameter 보다 먼저 와야 한다.** LaunchDescription 의 항목은
        # 순서대로 방문되므로, 앞에 두면 `use_sim_time` 을 아직 모르는 상태에서
        # 참조해 `launch configuration ... does not exist` 로 죽는다.
        *_declare_arguments(camera),
        # 백엔드도 use_sim_time 을 설정하지만, 이 스코프에서 추가하는 수집 노드에도
        # 전파되도록 여기서 한 번 더 선언한다.
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
        _included_backend(),
        session_control_node,
        target_joint_cmds_publisher,
        rdfp_image_viewer_node,
        image_recorder_node,
    ])
