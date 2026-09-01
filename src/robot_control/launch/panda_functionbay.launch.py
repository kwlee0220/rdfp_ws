"""Panda + MoveIt2 스택을 **펑션베이 시뮬레이터** 백엔드로 기동하는 launch.

:mod:`panda_mock.launch` 에서 **ros2_control 계층을 통째로 들어낸** 형태다.
펑션베이는 ros2_control 하드웨어 플러그인을 제공하지 않고 ROS 2 토픽으로만
연동하므로 `controller_manager` / `joint_state_broadcaster` / spawner 가 없다.
그 자리를 `robot_control.functionbay` 의 브리지 노드가 대신한다.

기능 단위
---------
- ``functionbay 브리지``:
    - /joint_state_fusion  — /output/panda_joint → /joint_states (이름 부여)
    - /readiness_gate      — 첫 관절 보고를 기동 완료 신호로 삼고 종료
- ``moveit``: /move_group, /moveit_servo, /ee_pose_publisher
- ``rviz2``: /rviz2
- ``camera``: /camera (또는 시뮬레이터 카메라를 remap)

mock 과의 차이
--------------
- ``ros2_control_node`` / controller spawner 3종이 **없다.**
- ``/joint_states`` 를 `joint_state_broadcaster` 가 아니라 fusion 노드가 낸다.
- 기동 순서 신호가 spawner 의 ``OnProcessExit`` 가 아니라
  `readiness_gate` 의 종료다 (같은 event handler 로 엮인다).
- ``gripper_control_node`` 를 띄우지 않는다 — 실물이 Robotiq 2F-85 라 Panda Hand
  를 전제한 URDF/SRDF 와 기구학이 다르다. 팔 연동을 먼저 완성하고 그리퍼는 별도
  진행한다. 그때까지 ``panda_finger_joint1`` 은 고정값으로 채워 TF 만 성립시킨다.

명령 경로
---------
펑션베이는 **내부 보간을 하지 않는다** — 주어진 관절 위치로 바로 간다. MoveIt 의
Cartesian 궤적은 TOTG 로 ~10 Hz 로 리샘플되므로 그대로 흘리면 계단처럼 움직인다.
`MoveGroupJgpcClient` 를 ``arm_command_format='joint_state'`` 로 만들고
``publish_rate=50.0`` 으로 스트리밍한다.

    client = create_move_group_client(
        node, mode='jgpc',
        arm_command_topic='/input/panda_joint',
        arm_command_joint_names=[f'panda_joint{i}' for i in range(1, 8)],
        arm_command_format='joint_state')
    client.follow_trajectory(waypoints, publish_rate=50.0)

설계 배경: ``docs/simulation/multi_simulator_backend_design.md`` §6.3 (B).
"""

from __future__ import annotations

from typing import Any, List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from robot_control.launch_helpers.camera import create_camera_node, declare_camera_arguments
from robot_control.launch_helpers.controller_startup import _chain_or_shutdown
from robot_control.launch_helpers.common import (
    build_moveit_config,
    build_servo_params,
    create_move_group_node,
    create_robot_state_publisher,
    create_rviz_node,
    create_servo_node,
    create_static_tf_node,
    declare_log_level_argument,
    declare_ros2_control_hardware_type_argument,
)
from robot_control.launch_helpers.ee_pose import create_ee_pose_node, declare_ee_pose_arguments

# 펑션베이가 제공하는 토픽. 시뮬레이터 쪽 고정값이라 상수로 둔다.
FB_JOINT_REPORT_TOPIC = '/output/panda_joint'
FB_JOINT_COMMAND_TOPIC = '/input/panda_joint'

ARM_JOINT_NAMES = [f'panda_joint{i}' for i in range(1, 8)]
# 그리퍼 미연동 구간 동안 TF 를 성립시키기 위한 고정값 (열림, m).
FINGER_JOINT_NAME = 'panda_finger_joint1'
FINGER_FIXED_POSITION = 0.04


def declare_functionbay_arguments() -> list[DeclareLaunchArgument]:
    """펑션베이 백엔드 전용 argument."""
    return [
        DeclareLaunchArgument(
            "fb_joint_report_topic", default_value=FB_JOINT_REPORT_TOPIC,
            description="시뮬레이터의 관절 보고 토픽 (sensor_msgs/JointState)",
        ),
        DeclareLaunchArgument(
            "fb_finger_position", default_value=str(FINGER_FIXED_POSITION),
            description=(
                "그리퍼 미연동 구간 동안 panda_finger_joint1 에 채울 고정값(m). "
                "TF 성립용이며 실제 그리퍼 상태가 아니다"
            ),
        ),
        DeclareLaunchArgument(
            "fb_ready_timeout", default_value="60.0",
            description="시뮬레이터 첫 보고 대기 한도(초). 초과하면 launch 가 실패한다",
        ),
    ]


def create_joint_state_fusion_node() -> Node:
    """시뮬레이터 관절 보고를 `/joint_states` 로 변환하는 노드."""
    return Node(
        package="robot_control",
        executable="fb_joint_state_fusion",
        name="joint_state_fusion",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "input_topic": LaunchConfiguration("fb_joint_report_topic"),
            "output_topic": "/joint_states",
            "joint_names": ARM_JOINT_NAMES,
            "extra_joint_names": [FINGER_JOINT_NAME],
            # 스칼라 argument 를 리스트 리터럴로 감싸 DOUBLE_ARRAY 로 만든다.
            # `[LaunchConfiguration(...)]` 만 쓰면 launch_ros 가 원소 하나짜리
            # 리스트를 스칼라 DOUBLE 로 평탄화해 노드가 타입 오류로 죽는다.
            "extra_joint_positions": ParameterValue(
                ["[", LaunchConfiguration("fb_finger_position"), "]"], value_type=List[float]),
        }],
    )


def create_readiness_gate_node() -> Node:
    """첫 관절 보고를 기다렸다 종료하는 게이트 (spawner 대체)."""
    return Node(
        package="robot_control",
        executable="fb_readiness_gate",
        name="readiness_gate",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "topic": LaunchConfiguration("fb_joint_report_topic"),
            "timeout_sec": LaunchConfiguration("fb_ready_timeout"),
        }],
    )


def generate_launch_description() -> LaunchDescription:
    moveit_config = build_moveit_config()
    servo_params = build_servo_params()

    # --- 즉시 기동: TF 소스와 브리지 ---
    static_tf = create_static_tf_node()
    robot_state_publisher = create_robot_state_publisher(moveit_config)
    joint_state_fusion = create_joint_state_fusion_node()
    readiness_gate = create_readiness_gate_node()

    # --- 시뮬레이터 준비 후 기동 ---
    post_ready_nodes: list[Any] = [
        create_move_group_node(moveit_config),
        create_servo_node(moveit_config, servo_params),
        create_rviz_node(moveit_config),
        create_camera_node(),
        create_ee_pose_node(),
    ]

    # spawner 가 없으므로 게이트 노드의 종료를 기동 신호로 쓴다. mock/Gazebo 가
    # `panda_hand_controller` spawner 의 OnProcessExit 를 쓰는 것과 같은 구조다.
    #
    # **종료 코드를 반드시 본다.** `OnProcessExit` 는 실패 종료에도 발동하므로,
    # 그냥 엮으면 시뮬레이터가 없어 타임아웃된 뒤에도 move_group 이 올라온다.
    # spawner 체인이 쓰는 `_chain_or_shutdown` 을 그대로 재사용해, 실패 시 런치
    # 전체를 종료시킨다.
    startup_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=readiness_gate,
            on_exit=_chain_or_shutdown(post_ready_nodes, "readiness_gate"),
        )
    )

    return LaunchDescription([
        declare_ros2_control_hardware_type_argument(),
        declare_log_level_argument(),
        *declare_ee_pose_arguments(),
        *declare_camera_arguments(),
        *declare_functionbay_arguments(),
        static_tf,
        robot_state_publisher,
        joint_state_fusion,
        readiness_gate,
        startup_handler,
    ])
