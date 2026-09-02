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
- ``camera``: 시뮬레이터가 ``/camera_image`` 로 발행한다. OpenCV 카메라 노드는
  기본 off 이며(`enable_camera_node`), 켤 때의 설정은 image_pipeline.yaml 에서 온다

mock 과의 차이
--------------
- ``ros2_control_node`` / controller spawner 3종이 **없다.**
- ``/joint_states`` 를 `joint_state_broadcaster` 가 아니라 fusion 노드가 낸다.
- 기동 순서 신호가 spawner 의 ``OnProcessExit`` 가 아니라
  `readiness_gate` 의 종료다 (같은 event handler 로 엮인다).
- ``GripperNode`` 를 띄우지 않는다 — 실물이 Robotiq 2F-85 라 Panda Hand
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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from robot_control.launch_helpers.camera import (
    create_camera_node,
    declare_simulator_camera_arguments,
)
from robot_control.launch_helpers.image_pipeline import load_config
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
from robot_control.launch_helpers.gripper import create_robotiq_2f_gripper_node

# 펑션베이가 제공하는 토픽. 시뮬레이터 쪽 고정값이라 상수로 둔다.
FB_JOINT_REPORT_TOPIC = '/output/panda_joint'
FB_JOINT_COMMAND_TOPIC = '/input/panda_joint'

ARM_JOINT_NAMES = [f'panda_joint{i}' for i in range(1, 8)]

# servo 출력을 받는 중간 토픽. 브리지가 이것을 JointState 로 바꿔 시뮬레이터로 넘긴다.
SERVO_COMMAND_TOPIC = '/servo_node/commands'
# 시뮬레이터가 관절 명령을 받는 토픽.
ARM_COMMAND_TOPIC = '/input/panda_joint'
# 시뮬레이터의 그리퍼 채널 (2F-85, 6축). 실측 확정 — 설계서 §6.1.
FB_GRIPPER_COMMAND_TOPIC = '/input/gripper_joint'
FB_GRIPPER_REPORT_TOPIC = '/output/gripper_joint'

# `/joint_states` 의 손가락에 채우는 **고정값** (열림, m).
#
# ⚠️ **그리퍼 상태가 아니다 — TF 성립용이다.** URDF 는 Panda Hand 인데 시뮬레이터는
# 2F-85 라 관절이 대응하지 않는다. 그리퍼 노드가 떠 있어도 이 값은 여전히 고정이며,
# 실제 그리퍼 상태는 `/gripper_states` 에서 읽는다. 여기서 읽으면 "항상 열려 있다"는
# 거짓말을 받는다 (docs/topic_naming_contract.md §2.2).
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
        DeclareLaunchArgument(
            "enable_gripper", default_value="true",
            description="그리퍼 노드(Robotiq2FGripperNode) 기동 여부",
        ),
        DeclareLaunchArgument(
            "fb_gripper_command_topic", default_value=FB_GRIPPER_COMMAND_TOPIC,
            description="시뮬레이터의 그리퍼 목표각 토픽 (std_msgs/Float64MultiArray, 6축)",
        ),
        DeclareLaunchArgument(
            "fb_gripper_report_topic", default_value=FB_GRIPPER_REPORT_TOPIC,
            description="시뮬레이터의 그리퍼 보고 토픽 (sensor_msgs/JointState, 6축 q/v/f)",
        ),
    ]


def override_servo_params_for_functionbay(servo_params: dict) -> dict:
    """servo 출력을 Float64MultiArray 로 돌린다 — 브리지가 받을 수 있는 형식이다.

    기본값(``trajectory_msgs/JointTrajectory`` → ``/panda_arm_controller/joint_trajectory``)
    을 그대로 두면 **아무도 받지 않는다.** 이 스택에는 ros2_control 컨트롤러가 없어서
    그 토픽의 구독자가 0 이고, 그 결과 텔레오퍼레이션 모션 키 전체가 조용히 무동작이
    된다 (2026-09-01 실측: j/q/' 세 방향 모두 관절 이동 0.00000 rad).

    ``publish_joint_velocities`` 를 끄는 것은 **선택이 아니라 필수**다. servo 의 파라미터
    검증은 ``command_out_type`` 이 Float64MultiArray 인데 positions 와 velocities 를 모두
    발행하도록 설정되어 있으면 실패를 반환하고, 그러면 servo 노드가 아예 기동하지
    못한다 (JGPC mock · Isaac 과 같은 제약이다).

    ``JointTrajectory`` 를 쓰지 않는 이유는 컨트롤러가 없는 스택에서
    ``/panda_arm_controller/joint_trajectory`` 라는 이름을 쓰게 되어 **없는 것을 있는
    것처럼** 보이게 만들기 때문이다.
    """
    params = servo_params["moveit_servo"]
    params["command_out_type"] = "std_msgs/Float64MultiArray"
    params["command_out_topic"] = SERVO_COMMAND_TOPIC
    params["publish_joint_positions"] = True
    params["publish_joint_velocities"] = False
    params["publish_joint_accelerations"] = False
    # 실효 속도 보정용. **런타임 파라미터로는 바꿀 수 없다** — moveit_servo 는 기동
    # 시점에만 읽는다(실측: `ros2 param set` 이 성공해도 동작이 그대로였다).
    scale = LaunchConfiguration("servo_linear_scale")
    params["scale"] = dict(params.get("scale") or {})
    params["scale"]["linear"] = scale
    return servo_params


def create_servo_bridge_node() -> Node:
    """servo 출력(Float64MultiArray) → 시뮬레이터 관절 명령(JointState).

    노드 자체는 백엔드 중립이다 — 토픽이 상대 경로라 remap 으로 결정된다. Isaac 이
    쓰는 것과 **같은 노드**이며 출력만 `/input/panda_joint` 로 돌린다.

    `joint_names` 는 **필수**다. Float64MultiArray 에는 이름이 없고 배열 순서가 곧
    관절 순서인데, 이 스택에는 조회할 컨트롤러가 없다.
    """
    return Node(
        package="robot_control", executable="servo_command_bridge",
        name="servo_command_bridge", output="screen", emulate_tty=True,
        parameters=[{"joint_names": ARM_JOINT_NAMES}],
        remappings=[("commands", SERVO_COMMAND_TOPIC),
                    ("arm_command", ARM_COMMAND_TOPIC)],
    )


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


def create_gripper_node_for_functionbay() -> Node:
    """그리퍼 — **액션이 아니라 관절 지령** 구현을 쓴다.

    이 스택에는 ros2_control 이 없어 `panda_hand_controller` 액션 서버가 존재하지
    않는다. mock·Isaac 이 쓰는 `GripperActionNode` 는 그래서 쓸 수 없고, 6축 목표각을
    토픽으로 직접 쓰는 `Robotiq2FGripperNode` 가 대신한다.

    파라미터 기본값이 이미 2F-85 실측치라 여기서 재정의할 것이 없다 (부호 벡터,
    `close` 0.725 rad, 파지 임계 1.0 N·m — 설계서 §6.1).
    """
    return create_robotiq_2f_gripper_node(
        command_topic=LaunchConfiguration("fb_gripper_command_topic"),
        report_topic=LaunchConfiguration("fb_gripper_report_topic"),
        condition=IfCondition(LaunchConfiguration("enable_gripper")),
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
    servo_params = override_servo_params_for_functionbay(build_servo_params())

    # --- 즉시 기동: TF 소스와 브리지 ---
    static_tf = create_static_tf_node()
    robot_state_publisher = create_robot_state_publisher(moveit_config)
    joint_state_fusion = create_joint_state_fusion_node()
    readiness_gate = create_readiness_gate_node()

    # --- 시뮬레이터 준비 후 기동 ---
    post_ready_nodes: list[Any] = [
        create_move_group_node(moveit_config),
        create_servo_node(moveit_config, servo_params),
        create_servo_bridge_node(),
        create_rviz_node(moveit_config),
        create_camera_node(load_config()),
        create_ee_pose_node(),
        # 그리퍼도 시뮬레이터 준비 후에 띄운다 — 관절 보고를 받아야 판정이 선다.
        create_gripper_node_for_functionbay(),
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
        DeclareLaunchArgument(
            "servo_linear_scale", default_value="0.8",
            description=("servo 의 unitless twist → m/s 환산 계수. **MoveIt 원본값 0.4 가 "
                         "아니라 0.8 이 기본이다** — 실측으로 고른 값이다(§5.5). 런타임 "
                         "param set 으로는 바뀌지 않는다")),
        *declare_ee_pose_arguments(),
        *declare_simulator_camera_arguments(),
        *declare_functionbay_arguments(),
        static_tf,
        robot_state_publisher,
        joint_state_fusion,
        readiness_gate,
        startup_handler,
    ])
