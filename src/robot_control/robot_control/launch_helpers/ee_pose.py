"""EE pose publisher 노드 및 파라미터 설정."""

from __future__ import annotations

from typing import Optional

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_control.launch_helpers.common import extra_parameter_list


def declare_ee_pose_arguments(ee_frame: str = "panda_hand") -> list[DeclareLaunchArgument]:
    """EE pose publisher 관련 launch argument들을 선언한다.

    Args:
        ee_frame: `ee_frame` 인자의 기본값. **description 이 싣는 그리퍼와 맞아야 한다** —
            Panda Hand 를 싣는 스택은 `panda_hand`, 실물이 Robotiq 인 펑션베이는
            `grasp_center`(정적 TF, `create_grasp_center_tf_node()`)를 쓴다.
    """
    return [
        DeclareLaunchArgument(
            "base_frame",
            default_value="panda_link0",
            description="Base frame for EE pose TF lookup",
        ),
        DeclareLaunchArgument(
            "ee_frame",
            default_value=ee_frame,
            description="End-effector frame for EE pose TF lookup",
        ),
        DeclareLaunchArgument(
            "publish_rate",
            default_value="50.0",
            description="EE pose publish rate in Hz",
        ),
    ]


def create_ee_pose_node(extra_parameters: Optional[dict] = None) -> Node:
    """EE pose publisher 노드를 생성한다."""
    return Node(
        package="robot_control",
        executable="ee_pose_node",
        name="ee_pose_publisher",
        output="screen",
        parameters=[
            {
                "base_frame": LaunchConfiguration("base_frame"),
                "ee_frame": LaunchConfiguration("ee_frame"),
                "publish_rate": LaunchConfiguration("publish_rate"),
            },
            *extra_parameter_list(extra_parameters),
        ],
    )


# panda_link7 기준 Robotiq 2F-85 파지 중심. URDF 실측으로 얻었다 —
# `robotiq2panda`(xyz 0 0 0.130, rpy 0 0 π/2) 다음에 손끝 안쪽면 중점
# (robotiq_85_base_link 기준 z +0.12627, grasp 자세 s=0.725).
_GRASP_CENTER_XYZ = ("0", "0.0003", "0.25627")
_GRASP_CENTER_YAW = "1.5708"


def create_grasp_center_tf_node() -> Node:
    """`panda_link7` → `grasp_center` 정적 TF.

    **실물이 Robotiq 인데 description 은 Panda Hand 인 스택을 위한 임시 보정이다.**
    `/ee_pose` 의 기본 프레임 `panda_hand` 는 `panda_link7` 에서 z +0.107 인데 실제
    파지점은 +0.256 이라 **149 mm 와 90° 가 어긋난다.** 이 프레임을 `ee_frame` 으로
    쓰면 `/ee_pose` 가 실제 파지점을 가리킨다.

    **TF·충돌 형상은 여전히 Panda Hand 다.** 근본 해결은 description 교체이며
    (`panda_ftsensor_robotiq` 패키지), 필요한 정보는
    `docs/simulation/functionbay_backend_design.md` §6.2 에 있다.
    """
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="grasp_center_tf",
        output="log",
        arguments=[
            "--x", _GRASP_CENTER_XYZ[0],
            "--y", _GRASP_CENTER_XYZ[1],
            "--z", _GRASP_CENTER_XYZ[2],
            "--yaw", _GRASP_CENTER_YAW,
            "--frame-id", "panda_link7",
            "--child-frame-id", "grasp_center",
        ],
    )
