"""이미지 파이프라인(camera / image_viewer / image_recorder) 설정 helper.

``config/image_pipeline.yaml`` 을 읽어 launch argument 선언을 만든다. 카메라를
띄우는 launch 가 넷이라 기본값을 한 곳에 모으기 위한 모듈이다.

소비자
------

=============================  ==========================================
launch                         설정 파일 지정 방법
=============================  ==========================================
rdfp_panda_mock                ``image_pipeline_config_file:=<path>``
rdfp_panda_jgpc_mock           ``image_pipeline_config_file:=<path>``
rdfp                           ``config_file:=<path>``
rdfp_advanced                  ``config_file:=<path>``
panda_mock / panda_jgpc_mock   (지정 불가 — 기본 YAML 을 기본값으로만 사용)
=============================  ==========================================

``camera_launch_helper.declare_camera_arguments()`` 는 본 모듈의
:func:`declare_camera_arguments` 를 기본 YAML 로 호출하는 얇은 래퍼다. 그래서
helper 를 쓰는 launch 는 코드 변경 없이 YAML 기본값을 따라간다.
"""

from __future__ import annotations

from typing import Any, Optional

import os

import yaml

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument

# YAML 설정 파일의 기본 경로. setup.py 가 ``config/*`` 를 ``share/rdfp/config/``
# 로 설치하므로 package share 에서 읽는다.
DEFAULT_CONFIG_RELPATH = os.path.join("config", "image_pipeline.yaml")


def default_config_path() -> str:
    """패키지 share 경로의 기본 YAML 위치를 반환한다."""
    return os.path.join(get_package_share_directory("robot_control"), DEFAULT_CONFIG_RELPATH)


def load_config(config_path: Optional[str] = None) -> dict[str, Any]:
    """이미지 파이프라인 YAML 을 로드한다. 경로를 비우면 기본 경로를 쓴다."""
    path = config_path or default_config_path()
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def as_launch_str(value: Any) -> str:
    """Python 값을 DeclareLaunchArgument 의 default_value 문자열로 변환한다.

    bool 은 ROS launch 관례에 맞춰 소문자 ``"true"`` / ``"false"`` 로 변환한다.
    """
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def declare_config_file_argument(name: str, extra: str = "") -> DeclareLaunchArgument:
    """이미지 파이프라인 YAML 경로 argument 를 선언한다.

    argument 이름이 launch 마다 다르다 — 로봇 YAML 을 따로 받는 panda 계열은
    ``image_pipeline_config_file``, 이미지 설정만 쓰는 앱 계열은 ``config_file``.
    """
    return DeclareLaunchArgument(
        name,
        default_value=default_config_path(),
        description=(
            "Path to the image pipeline YAML (camera / image_viewer / image_recorder). "
            f"기본값은 <rdfp share>/{DEFAULT_CONFIG_RELPATH} 다. "
            "$HOME 등 쉘 확장은 쉘에 맡긴다. " + extra
        ).strip(),
    )


def declare_camera_arguments(config: dict[str, Any]) -> list[DeclareLaunchArgument]:
    """``camera`` 블록에서 카메라 argument 9개를 선언한다."""
    cam = config["camera"]
    return [
        DeclareLaunchArgument(
            "enable_camera_node",
            default_value=as_launch_str(cam["enabled"]),
            description="Whether to start the camera node",
        ),
        DeclareLaunchArgument(
            "camera_id",
            default_value=as_launch_str(cam["id"]),
            description="Camera device index or URI/path for camera_node",
        ),
        DeclareLaunchArgument(
            "camera_image_topic",
            default_value=as_launch_str(cam["image_topic"]),
            description="Remap target for base image topic ('image')",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value=as_launch_str(cam["info_topic"]),
            description="Remap target for base camera_info topic ('camera_info')",
        ),
        DeclareLaunchArgument(
            "camera_status_topic",
            default_value=as_launch_str(cam["status_topic"]),
            description="Remap target for camera status topic ('image/status')",
        ),
        DeclareLaunchArgument(
            "camera_fps",
            default_value=as_launch_str(cam["fps"]),
            description="Target FPS for camera_node",
        ),
        DeclareLaunchArgument(
            "camera_resolution",
            default_value=as_launch_str(cam["resolution"]),
            description=(
                "Target resolution for camera_node (e.g. 640x480). "
                "image_recorder 가 같은 값을 사용한다"
            ),
        ),
        DeclareLaunchArgument(
            "camera_frame_id",
            default_value=as_launch_str(cam["frame_id"]),
            description="frame_id for published Image/CameraInfo",
        ),
        DeclareLaunchArgument(
            "camera_compress_image",
            default_value=as_launch_str(cam["compress_image"]),
            description="Publish JPEG compressed image when true",
        ),
    ]


def declare_image_viewer_arguments(config: dict[str, Any]) -> list[DeclareLaunchArgument]:
    """``image_viewer`` 블록에서 뷰어 argument 를 선언한다."""
    return [
        DeclareLaunchArgument(
            "enable_image_viewer_node",
            default_value=as_launch_str(config["image_viewer"]["enabled"]),
            description=(
                "Whether to start the image viewer node. "
                "헤드리스 환경(DISPLAY 미설정)에서는 false 로 끈다"
            ),
        ),
    ]


def declare_image_recorder_arguments(
    config: dict[str, Any], include_auto_start: bool = True
) -> list[DeclareLaunchArgument]:
    """``image_recorder`` 블록에서 레코더 argument 를 선언한다.

    ``include_auto_start`` 가 False 면 ``image_recorder_auto_start`` 를 빼고
    선언한다. ``rdfp_advanced`` 의 ``RdfpImageRecorderNode`` 는 ``/session`` 상태로
    녹화를 시작하므로 그 파라미터를 받지 않는다 — 선언해 봐야 소비자가 없다.
    """
    ir = config["image_recorder"]
    args = [
        DeclareLaunchArgument(
            "enable_image_recorder_node",
            default_value=as_launch_str(ir["enabled"]),
            description="Whether to start the image recorder node",
        ),
        DeclareLaunchArgument(
            "image_recorder_fps",
            default_value=as_launch_str(ir["fps"]),
            description=(
                "Target FPS for the image recorder "
                "(should match camera_fps to avoid frame drops)"
            ),
        ),
        DeclareLaunchArgument(
            "image_recorder_output_dir",
            default_value=as_launch_str(ir["output_dir"]),
            description="Output directory for recorded MP4 files",
        ),
    ]
    if include_auto_start:
        args.append(DeclareLaunchArgument(
            "image_recorder_auto_start",
            default_value=as_launch_str(ir["auto_start"]),
            description="If true, the recorder starts recording immediately at launch",
        ))
    return args


def declare_image_pipeline_arguments(
    config: dict[str, Any], include_auto_start: bool = True
) -> list[DeclareLaunchArgument]:
    """camera + image_viewer + image_recorder argument 를 한 번에 선언한다."""
    return [
        *declare_camera_arguments(config),
        *declare_image_viewer_arguments(config),
        *declare_image_recorder_arguments(config, include_auto_start=include_auto_start),
    ]
