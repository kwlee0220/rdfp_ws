"""Gripper control 노드 설정."""

from __future__ import annotations

from launch_ros.actions import Node


def create_gripper_control_node() -> Node:
    """Gripper control 노드를 생성한다."""
    return Node(
        package="rdfp",
        executable="gripper_control_node",
        name="gripper_control",
        output="screen",
    )
