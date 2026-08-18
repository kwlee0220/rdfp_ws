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

from image_pipeline_launch_helper import (
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


def create_camera_node() -> Node:
    """카메라 노드를 생성한다."""
    return Node(
        package="rdfp",
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
