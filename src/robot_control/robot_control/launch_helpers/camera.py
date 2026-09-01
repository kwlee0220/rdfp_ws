"""카메라 노드 및 파라미터 설정.

argument 기본값은 :mod:`image_pipeline_launch_helper` 를 통해
``config/image_pipeline.yaml`` 에서 온다. 예전에는 이 파일이 기본값을
하드코딩했는데, 같은 카메라를 쓰는 launch 들끼리 값이 갈라지는 문제가 있었다
(앱 계열 ``640x480`` / ``id: 4`` vs panda 계열 YAML ``1280x720`` / mp4 경로).
"""

from __future__ import annotations

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from .image_pipeline import (
    declare_camera_arguments as _declare_camera_arguments_from,
    load_config,
)


def declare_camera_arguments() -> list[DeclareLaunchArgument]:
    """카메라 관련 launch argument들을 선언한다 (기본 YAML 의 값을 기본값으로 사용).

    설정 파일을 바꿔 끼워야 하면 이 함수 대신
    ``image_pipeline_launch_helper.declare_camera_arguments(load_config(path))`` 를
    쓴다 — 그러려면 ``config_file`` 이 resolve 된 뒤여야 하므로 launch 쪽에서
    ``OpaqueFunction`` 이 필요하다.
    """
    return _declare_camera_arguments_from(load_config())


def declare_simulator_camera_arguments() -> list[DeclareLaunchArgument]:
    """시뮬레이터가 이미지를 발행하는 스택용 argument.

    **OpenCV 카메라 설정 인자를 선언하지 않는다** — `camera_id` `camera_fps`
    `camera_info_topic` 이 빠진다. 그 값들은 우리가 정하는 것이 아니라 시뮬레이터가
    주는 대로 쓰는 것이라, 인자로 열어 두면 실제와 다른 값을 넣을 수 있고 그것이
    조용히 틀린 데이터가 된다 — 특히 레코더는 CFR 이라 fps 가 어긋나면 영상의
    시간축이 통째로 밀린다.

    `enable_camera_node` 는 남긴다. USB 카메라를 함께 붙이는 경우가 있어서인데,
    그때 쓰는 설정은 `config/image_pipeline.yaml` 에서 온다
    (`create_camera_node(load_config())`).

    `camera_info` 는 OpenCV 경로에서만 쓴다 — 시뮬레이터는 제공하지 않는다.
    """
    cam = load_config()["camera"]
    return [
        DeclareLaunchArgument(
            "enable_camera_node", default_value="false",
            choices=["true", "false"],
            description=("OpenCV 카메라 노드 기동 여부. 시뮬레이터가 이미지를 주므로 "
                         "기본 off 다. 켤 때의 설정은 image_pipeline.yaml 에서 온다"),
        ),
        DeclareLaunchArgument(
            "camera_image_topic", default_value=str(cam["image_topic"]),
            description="이미지 토픽. 시뮬레이터 발행 토픽으로 바꿔 준다",
        ),
    ]


def create_camera_node(config: dict | None = None) -> Node:
    """카메라 노드를 생성한다.

    Args:
        config: 주면 OpenCV 전용 설정(`id`/`fps`/`camera_info` 등)을 이 dict 에서
            읽는다. 시뮬레이터 스택처럼 해당 argument 를 선언하지 않는 launch 에서
            쓴다. ``None`` 이면 기존대로 LaunchConfiguration 에서 읽는다.
    """
    if config is not None:
        cam = config["camera"]
        return Node(
            package="robot_control",
            executable="camera_node",
            name="camera_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_camera_node")),
            parameters=[{
                "camera_id": str(cam["id"]),
                "fps": int(cam["fps"]),
                "resolution": str(cam["resolution"]),
                "frame_id": str(cam["frame_id"]),
                "compress_image": bool(cam["compress_image"]),
            }],
            remappings=[
                ("~/image_raw", LaunchConfiguration("camera_image_topic")),
                ("~/camera_info", str(cam["info_topic"])),
                ("~/camera_status", str(cam["status_topic"])),
            ],
        )
    return Node(
        package="robot_control",
        executable="camera_node",
        name="camera_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_camera_node")),
        parameters=[
            {
                "camera_id": LaunchConfiguration("camera_id"),
                "fps": LaunchConfiguration("camera_fps"),
                "resolution": LaunchConfiguration("camera_resolution"),
                "frame_id": LaunchConfiguration("camera_frame_id"),
                "compress_image": LaunchConfiguration("camera_compress_image"),
            }
        ],
        remappings=[
            ("~/image_raw", LaunchConfiguration("camera_image_topic")),
            ("~/camera_info", LaunchConfiguration("camera_info_topic")),
            ("~/camera_status", LaunchConfiguration("camera_status_topic")),
        ],
    )
