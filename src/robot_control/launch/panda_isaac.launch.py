"""Panda + MoveIt2 스택을 **Isaac Sim** 백엔드로 기동하는 launch (Phase 0 골격).

:mod:`panda_functionbay.launch` 와 같은 구조다 — Isaac 도 ros2_control 하드웨어
플러그인을 쓰지 않고 ROS 2 토픽으로만 연동하므로 `controller_manager` /
`joint_state_broadcaster` / spawner 가 없다. 설계 근거는
``docs/simulation/multi_simulator_backend_design.md`` §6.3 (B안) 이고, 단계 계획은
``docs/simulation/isaac_backend_skeleton.md`` 에 있다.

펑션베이와 갈리는 지점
----------------------
- **`use_sim_time` 을 켠다.** Isaac 은 ``/clock`` 을 발행한다. 펑션베이는 발행하지
  않아 벽시계를 썼고 그래서 시계 오프셋 문제를 안고 있었다.
- 브리지 노드를 새로 만들지 않고 `robot_control.functionbay` 의 두 노드를 **토픽
  파라미터만 바꿔** 재사용한다. 구현이 백엔드 중립이기 때문이다.

단계 게이팅
-----------
한 번에 전부 붙이지 않는다. 아래 인자는 **전부 기본 false** 이며, 해당 단계 작업을
시작할 때 하나씩 연다. 켜지 않은 단계의 노드는 아예 뜨지 않는다.

    enable_gripper   Phase 2 — 그리퍼
    enable_scene     Phase 3 — scene 객체

Phase 0 수용 기준은 ``scripts/isaac/is_check_phase0.py`` 가 검사한다.

기동 순서
---------
spawner 가 없으므로 `readiness_gate` 의 **정상 종료**를 기동 신호로 쓴다. 실패
종료(시뮬레이터 미기동)면 launch 전체가 내려간다 — `move_group` 만 올라와 원인을
감추는 상황을 막는다.

    ros2 launch robot_control panda_isaac.launch.py
    ros2 launch robot_control panda_isaac.launch.py enable_rviz:=false   # VRAM 절약
"""

from __future__ import annotations

import os

from typing import Any

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

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
from robot_control.launch_helpers.controller_startup import _chain_or_shutdown
from robot_control.launch_helpers.ee_pose import create_ee_pose_node, declare_ee_pose_arguments

# Isaac 이 관절 상태를 직접 발행하는 토픽.
#
# 펑션베이와 달리 **fusion 노드를 두지 않는다.** Phase 0 실측에서 Isaac 이
# `name` 을 채우고 팔 7관절 + finger 2관절을 모두 보내는 것이 확인됐다
# (docs/simulation/isaac_backend_skeleton.md §7 Q1). 중계할 것이 없을 뿐 아니라,
# `fb_joint_state_fusion` 은 `extra_joint_*` 주입을 끌 수 없어(빈 리스트를 주면
# 기본값으로 되돌아간다) `panda_finger_joint1` 이 **중복 발행된다.**
JOINT_STATE_TOPIC = '/joint_states'

# 관절 한계 파일. 기본 moveit_resources 의 URDF 한계는 실제 Franka 보다 전 관절이
# 4° 헐겁고(panda_joint4 상한은 9°), **Isaac 은 실제 스펙을 강제한다.** 그대로 두면
# MoveIt 이 계획한 자세를 시뮬레이터가 한계에서 막고, 그것이 추종 오차처럼 보인다
# (실제로 그렇게 오독했다 — §7 Q6). 계획 단계에서 막는 편이 낫다.
JOINT_LIMITS_RELPATH = os.path.join('config', 'panda_real_joint_limits.yaml')
GRIPPER_COMMAND_TOPIC = '/isaac/gripper_command'

# Isaac 이 관절 명령을 받는 토픽. servo 경로도 최종적으로 여기로 모인다.
ARM_COMMAND_TOPIC = '/isaac/arm_command'
# servo 출력을 받아 위 토픽으로 옮기는 중간 토픽. 컨트롤러가 없는 스택이므로
# `/panda_arm_controller/...` 라는 이름을 쓰지 않는다 — 없는 것을 있는 것처럼 보이게
# 하면 진단이 어려워진다.
SERVO_COMMAND_TOPIC = '/isaac/servo_command'
# Float64MultiArray 는 이름이 없고 **배열 순서가 곧 관절 순서**다. 조회할 컨트롤러가
# 없으므로 여기서 못박는다.
ARM_JOINT_NAMES = [f'panda_joint{i}' for i in range(1, 8)]


def declare_isaac_arguments() -> list[DeclareLaunchArgument]:
    """Isaac 백엔드 전용 argument."""
    return [
        DeclareLaunchArgument(
            "joint_state_topic", default_value=JOINT_STATE_TOPIC,
            description="Isaac 이 발행하는 관절 상태 토픽 (sensor_msgs/JointState)",
        ),
        DeclareLaunchArgument(
            "enable_servo", default_value="false",
            choices=["true", "false"],
            description=(
                "servo(twist) 경로. teleop_keyboard 와 teleop_retarget 이 모두 "
                "여기로 수렴하므로, 손으로 몰려면 켠다"
            ),
        ),
        DeclareLaunchArgument(
            "isaac_ready_timeout", default_value="60.0",
            description="Isaac 첫 관절 보고 대기 한도(초). 초과하면 launch 가 실패한다",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description=(
                "Isaac 의 /clock 을 시간 원본으로 쓴다. false 로 두면 move_group 이 "
                "current robot state 를 가져오지 못해 조용히 무력해진다"
            ),
        ),
        DeclareLaunchArgument(
            "enable_rviz", default_value="true",
            description="RViz2 기동 여부. VRAM 이 빠듯한 호스트에서는 false 를 권한다",
        ),
        DeclareLaunchArgument(
            "enable_gripper", default_value="false",
            description=(
                "Phase 2 (그리퍼) 연동. gripper_action_bridge 와 GripperActionNode 를 "
                "띄운다. 상태(finger 관절)는 이 값과 무관하게 항상 들어온다"
            ),
        ),
        DeclareLaunchArgument(
            "gripper_command_topic", default_value=GRIPPER_COMMAND_TOPIC,
            description="Isaac 이 구독하는 그리퍼 명령 토픽 (sensor_msgs/JointState)",
        ),
        DeclareLaunchArgument(
            "enable_scene", default_value="false",
            description=(
                "Phase 3 (scene 객체) 연동. Isaac 이 TF 로 내보낸 물체 pose 를 "
                "isaac_scene_state_node 가 /scene/objects 로 바꾼다"
            ),
        ),
        DeclareLaunchArgument(
            "scene_publish_rate", default_value="2.0",
            description="/scene/objects 발행 Hz",
        ),
    ]


# 아직 구현이 없는 단계 인자. 켜면 **조용히 무시되지 않고 실패한다** — 인자가 있는데
# 아무 일도 안 일어나는 것이 가장 찾기 어려운 종류의 버그다.
#
# Phase 0~4 가 모두 구현되어 현재는 비어 있다. 새 단계를 계획할 때 다시 채운다.
_UNIMPLEMENTED_PHASES: dict[str, str] = {}


def _reject_unimplemented_phases(context) -> list:
    """미구현 단계 인자가 켜져 있으면 launch 를 중단한다."""
    for name, phase in _UNIMPLEMENTED_PHASES.items():
        if context.perform_substitution(LaunchConfiguration(name)).lower() == "true":
            raise RuntimeError(
                f"{name}:=true is not implemented yet ({phase}); "
                f"see docs/simulation/isaac_backend_skeleton.md"
            )
    return []


def create_readiness_gate_node() -> Node:
    """첫 관절 보고를 기다렸다 종료하는 게이트 (spawner 대체)."""
    return Node(
        package="robot_control",
        executable="fb_readiness_gate",
        name="readiness_gate",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "topic": LaunchConfiguration("joint_state_topic"),
            "timeout_sec": LaunchConfiguration("isaac_ready_timeout"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )


def create_gripper_nodes() -> list[Node]:
    """Phase 2 — 그리퍼 명령 경로.

    Isaac 에는 ros2_control 이 없어 `panda_hand_controller` 액션 서버가 없다.
    `gripper_action_bridge` 가 같은 이름의 액션을 열고 관절 위치 토픽으로 바꾼다.
    상위(teleop·twin·재생)는 백엔드를 몰라도 되게 하려는 것이다.
    """
    condition = IfCondition(LaunchConfiguration("enable_gripper"))
    sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}
    return [
        Node(
            package="robot_control", executable="isaac_gripper_bridge",
            name="gripper_action_bridge", output="screen", emulate_tty=True,
            condition=condition,
            parameters=[{
                "command_topic": LaunchConfiguration("gripper_command_topic"),
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                **sim_time,
            }],
        ),
        # `GripperActionNode` — mock 과 **같은 노드**다. 이름이 백엔드가 아니라 실행
        # 수단(액션 클라이언트)을 가리키고, 위 브리지가 액션 서버를 제공하므로 명령
        # 경로가 mock 과 같기 때문이다.
        # **`stall_effort` 는 실측값이다 (2026-09-02).** 빈손으로 닫은 채 팔을 흔들어도
        # 손가락 |effort| 가 0.13 N·m 를 넘지 않았고(과도 최대 0.50), 블록을 물면
        # 22.4 N·m 로 유지된다. 1.0 은 거짓 양성 최악값의 2배, 파지의 1/22 지점이다.
        #
        # **속도 조건은 켜지 않는다.** Isaac 은 PhysX 접촉에서 손가락이 계속 떨려
        # (파지 유지 중 |vel| 최대 0.26 rad/s) 속도 게이트를 걸면 파지를 놓친다 —
        # 펑션베이(링키지가 멎는다)와 다른 점이다.
        Node(
            package="robot_control", executable="gripper_action_node",
            name="gripper", output="screen", emulate_tty=True,
            condition=condition,
            parameters=[sim_time, {"stall_effort": 1.0, "stall_velocity": 0.0}],
        ),
    ]


def override_servo_params_for_isaac(servo_params: dict[str, Any]) -> dict[str, Any]:
    """servo 출력을 Isaac 용 Float64MultiArray 명령 토픽으로 돌린다.

    ``publish_joint_velocities`` 를 끄는 것은 **선택이 아니라 필수**다. servo 의
    파라미터 검증은 ``command_out_type`` 이 Float64MultiArray 인데 positions 와
    velocities 를 모두 발행하도록 설정되어 있으면 실패를 반환하고, 그 경우 servo
    노드가 아예 기동하지 못한다 (JGPC mock 과 같은 제약이다).

    ``JointTrajectory`` 를 쓰지 않는 이유는 컨트롤러가 없는 스택에서
    ``/panda_arm_controller/joint_trajectory`` 라는 이름을 쓰게 되어 **없는 것을
    있는 것처럼** 보이게 만들기 때문이다.
    """
    params = servo_params["moveit_servo"]
    params["command_out_type"] = "std_msgs/Float64MultiArray"
    params["command_out_topic"] = SERVO_COMMAND_TOPIC
    params["publish_joint_positions"] = True
    params["publish_joint_velocities"] = False
    params["publish_joint_accelerations"] = False
    return servo_params


def create_servo_nodes(moveit_config) -> list[Node]:
    """servo(twist) 경로 — servo_node + auto_start + 명령 다리.

    **teleop 두 경로가 모두 여기로 수렴한다.**

        teleop_keyboard  -> delta_twist_cmds ─┐
        teleop_retarget  -> ee_twist_node ────┴─> servo -> bridge -> Isaac

    servo 는 `trajectory_msgs/JointTrajectory` 나 `std_msgs/Float64MultiArray` 만
    낼 수 있고 Isaac 은 `sensor_msgs/JointState` 만 받는다. **토픽 remap 으로는 못
    잇는다 — 타입이 다르다.** 그래서 다리를 하나 둔다.

    `publish_joint_velocities: false` 는 선택이 아니라 필수다. `command_out_type` 이
    Float64MultiArray 인데 positions 와 velocities 를 모두 발행하도록 두면 servo 의
    파라미터 검증이 실패해 **노드가 아예 기동하지 못한다.**
    """
    condition = IfCondition(LaunchConfiguration("enable_servo"))
    sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}
    servo_params = override_servo_params_for_isaac(build_servo_params())

    return [
        create_servo_node(moveit_config, servo_params, condition=condition,
                          extra_parameters=sim_time),
        # **servo 는 start_servo 를 부르기 전까지 입력을 조용히 무시한다.**
        # 에러도 경고도 없이 그냥 안 움직인다.
        Node(
            package="robot_control", executable="servo_auto_start_node",
            name="servo_auto_start", output="screen", emulate_tty=True,
            parameters=[sim_time],
            condition=condition,
        ),
        Node(
            package="robot_control", executable="isaac_servo_bridge",
            name="servo_command_bridge", output="screen", emulate_tty=True,
            parameters=[{"joint_names": ARM_JOINT_NAMES}, sim_time],
            remappings=[("commands", SERVO_COMMAND_TOPIC),
                        ("arm_command", ARM_COMMAND_TOPIC)],
            condition=condition,
        ),
    ]


def create_scene_node() -> Node:
    """Phase 3 — Isaac 물체 TF → `/scene/objects`.

    물체의 이름·종류·크기는 TF 에 없으므로 시뮬레이터 쪽 생성 스크립트와 **같은
    JSON**(`config/isaac_scene.json`)을 읽는다.
    """
    return Node(
        package="robot_control", executable="isaac_scene_state_node",
        name="isaac_scene_state", output="screen", emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_scene")),
        parameters=[{
            "publish_rate": LaunchConfiguration("scene_publish_rate"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )


def generate_launch_description() -> LaunchDescription:
    joint_limits_file = os.path.join(
        get_package_share_directory('robot_control'), JOINT_LIMITS_RELPATH)
    moveit_config = build_moveit_config(joint_limits_file=joint_limits_file)
    # 모든 노드에 같은 dict 를 넘긴다. SetParameter 를 쓰지 않는 이유는 게이트
    # 통과 후 event handler 로 뜨는 노드까지 확실히 덮기 위해서다.
    sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    # --- 즉시 기동: TF 소스와 브리지 ---
    static_tf = create_static_tf_node(extra_parameters=sim_time)
    robot_state_publisher = create_robot_state_publisher(moveit_config, extra_parameters=sim_time)
    readiness_gate = create_readiness_gate_node()

    # --- Isaac 준비 후 기동 ---
    post_ready_nodes: list[Any] = [
        create_move_group_node(moveit_config, extra_parameters=sim_time),
        create_rviz_node(moveit_config, condition=IfCondition(LaunchConfiguration("enable_rviz")),
                         extra_parameters=sim_time),
        create_ee_pose_node(extra_parameters=sim_time),
        *create_gripper_nodes(),
        *create_servo_nodes(moveit_config),
        create_scene_node(),
    ]

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
        *declare_isaac_arguments(),
        OpaqueFunction(function=_reject_unimplemented_phases),
        static_tf,
        robot_state_publisher,
        readiness_gate,
        startup_handler,
    ])
