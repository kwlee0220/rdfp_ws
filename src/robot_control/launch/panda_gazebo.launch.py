"""Panda + MoveIt2 풀 스택을 **Gazebo (Fortress)** 백엔드로 기동하는 launch.

:mod:`panda_mock.launch` 의 Gazebo 대응본이다. mock 백엔드와의 차이는 로봇
백엔드(Layer B)뿐이며, MoveIt / RViz / ee_pose / gripper 등 상위 노드(Layer A)는
동일하게 재사용한다.

기능 단위
---------
- ``gazebo``:
    - gz-sim 서버/클라이언트 (ign_ros2_control 플러그인이 controller_manager 호스팅)
    - robot_state_publisher (rdfp description, inertial 보강)
    - /clock 브리지 (use_sim_time 클럭 소스)
    - 로봇 스폰(create) → joint_state_broadcaster → panda_arm_controller →
      panda_hand_controller 순차 기동
- ``moveit``: /move_group, /moveit_servo, /ee_pose_publisher
- ``rviz2``: /rviz2
- ``gripper``: /gripper_control
- ``camera`` (옵션, ``simulate_camera:=true``): gz 카메라 센서 + ros_gz 브리지

mock 과의 핵심 차이
-------------------
- 별도 ``ros2_control_node`` 를 띄우지 않는다 (ign_ros2_control 플러그인이 대체).
- ``static_transform_publisher`` 대신 URDF 의 ``world`` 고정 조인트로 베이스를
  앵커링한다.
- 모든 노드에 ``use_sim_time:=true`` 를 전파한다 (Gazebo 클럭 사용).

사용 예
-------
    ros2 launch rdfp panda_gazebo.launch.py
    ros2 launch rdfp panda_gazebo.launch.py world:=empty.sdf
    ros2 launch rdfp panda_gazebo.launch.py simulate_camera:=true
"""

from __future__ import annotations

import os


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter

from robot_control.launch_helpers.controller import (
    create_joint_state_broadcaster_spawner,
    create_panda_arm_controller_spawner,
    create_panda_hand_controller_spawner,
)
from robot_control.launch_helpers.ee_pose import create_ee_pose_node, declare_ee_pose_arguments
from robot_control.launch_helpers.gazebo import (
    build_gazebo_moveit_config,
    build_gazebo_servo_params,
    create_gazebo_controller_startup_handlers,
    create_gz_camera_bridge_node,
    create_gz_clock_bridge_node,
    create_gz_sim,
    create_spawn_entity_node,
    gz_resource_path_actions,
)
from robot_control.launch_helpers.gripper import create_gripper_node
from robot_control.launch_helpers.common import (
    create_move_group_node,
    create_robot_state_publisher,
    create_rviz_node,
    create_servo_node,
    declare_log_level_argument,
)


def _declare_arguments() -> list[DeclareLaunchArgument]:
    """Gazebo 백엔드 전용 launch argument 를 선언한다."""
    return [
        DeclareLaunchArgument(
            "world",
            default_value="empty.sdf",
            description="Gazebo world file/name passed to gz_args (e.g. empty.sdf)",
        ),
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="false",
            description="If true, also launch RViz2 (MoveIt MotionPlanning UI)",
        ),
        DeclareLaunchArgument(
            "simulate_camera",
            default_value="false",
            description="If true, attach a gz camera sensor to panda_hand and bridge it to ROS",
        ),
        DeclareLaunchArgument(
            "camera_image_topic",
            default_value="/camera/image_raw",
            description="ROS topic for the bridged camera image",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/camera_info",
            description="ROS topic for the bridged camera_info",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    moveit_config = build_gazebo_moveit_config()
    servo_params = build_gazebo_servo_params()

    world = LaunchConfiguration("world")

    # --- gazebo 백엔드 ---
    robot_state_publisher = create_robot_state_publisher(moveit_config)
    gz_sim = create_gz_sim(world)
    clock_bridge = create_gz_clock_bridge_node()
    spawn_entity = create_spawn_entity_node()

    joint_state_broadcaster_spawner = create_joint_state_broadcaster_spawner()
    panda_arm_controller_spawner = create_panda_arm_controller_spawner()
    panda_hand_controller_spawner = create_panda_hand_controller_spawner()

    # --- MoveIt / RViz / 주변 노드 (mock 과 동일 helper 재사용) ---
    move_group_node = create_move_group_node(moveit_config)
    servo_node = create_servo_node(moveit_config, servo_params)
    # RViz 는 enable_rviz 가 true 일 때만 기동한다 (기본 false → Gazebo 단독 실행).
    rviz_node = create_rviz_node(
        moveit_config,
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
    )
    ee_pose_node = create_ee_pose_node()
    gripper_node = create_gripper_node()

    # --- 옵션: gz 카메라 센서 → ROS 브리지 (simulate_camera 일 때만) ---
    camera_bridge = create_gz_camera_bridge_node(
        LaunchConfiguration("camera_image_topic"),
        LaunchConfiguration("camera_info_topic"),
        condition=IfCondition(LaunchConfiguration("simulate_camera")),
    )

    # 스폰 완료 후 controller -> moveit/주변 노드 순차 기동.
    controller_startup_handlers = create_gazebo_controller_startup_handlers(
        spawn_entity,
        joint_state_broadcaster_spawner,
        panda_arm_controller_spawner,
        panda_hand_controller_spawner,
        [move_group_node, servo_node, rviz_node, ee_pose_node,
         gripper_node, camera_bridge],
    )

    return LaunchDescription(
        [
            # gz 가 model:// 메시를 찾도록 리소스 경로를 먼저 설정한다
            # (gz_sim 기동 전에 와야 서버/GUI 프로세스에 반영된다).
            *gz_resource_path_actions(),
            # Gazebo 클럭을 쓰므로 모든 노드에 use_sim_time 을 전파한다.
            SetParameter(name="use_sim_time", value=True),
            declare_log_level_argument(),
            *declare_ee_pose_arguments(),
            *_declare_arguments(),
            robot_state_publisher,
            gz_sim,
            clock_bridge,
            spawn_entity,
            *controller_startup_handlers,
        ]
    )
