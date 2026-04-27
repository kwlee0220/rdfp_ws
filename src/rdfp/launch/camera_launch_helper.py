"""카메라 노드 및 파라미터 설정."""

from __future__ import annotations

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def declare_camera_arguments() -> list[DeclareLaunchArgument]:
    """카메라 관련 launch argument들을 선언한다."""
    return [
        DeclareLaunchArgument(
            "enable_camera_node",
            default_value="true",
            description="Whether to start rdfp camera_node",
        ),
        DeclareLaunchArgument(
            "camera_id",
            default_value="4",
            description="Camera device index or URI/path for camera_node",
        ),
        DeclareLaunchArgument(
            "camera_image_topic",
            default_value="/camera/image_raw",
            description="Remap target for base image topic ('image')",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/camera_info",
            description="Remap target for base camera_info topic ('camera_info')",
        ),
        DeclareLaunchArgument(
            "camera_status_topic",
            default_value="/camera/image_raw/status",
            description="Remap target for camera status topic ('image/status')",
        ),
        DeclareLaunchArgument(
            "camera_fps",
            default_value="30",
            description="Target FPS for camera_node",
        ),
        DeclareLaunchArgument(
            "camera_resolution",
            default_value="640x480",
            description="Target resolution for camera_node (e.g. 640x480)",
        ),
        DeclareLaunchArgument(
            "camera_frame_id",
            default_value="camera_link",
            description="frame_id for published Image/CameraInfo",
        ),
        DeclareLaunchArgument(
            "camera_compress_image",
            default_value="false",
            description="Publish JPEG compressed image when true",
        ),
    ]


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
