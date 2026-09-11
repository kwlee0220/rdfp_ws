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
- ``rviz2``: /rviz2 (``enable_rviz:=true`` 로 켠다, **기본 off**)
- ``image_viewer``: ``/image_viewer`` (``enable_image_viewer:=true`` 로 켠다,
  **기본 off**). 시뮬레이터 이미지를 화면에 띄운다 — 헤드리스 환경에서는 켜지 않는다.
- ``camera_republish``: ``enable_camera`` 가 켜는 ``image_transport/republish`` 다.
  시뮬레이터의 압축 이미지를 ``camera_image_topic`` (raw) 으로 되살린다.
- ``scene``: 시뮬레이터가 ``/tf`` 로 내보내는 조작 대상 body pose 를 읽어
  ``/scene/objects`` 로 낸다 (``enable_scene:=false`` 로 끈다). **``/scene/reset`` 은
  제공되지 않는다** — 런타임 배치 변경 수단이 없다.
- ``camera``: 시뮬레이터가 ``/camera_image/compressed`` (``sensor_msgs/CompressedImage``,
  jpeg) 로 발행한다 — **2026-09-08 새 빌드에서 ``/camera_image``(``Image``, bgr8) 에서
  바뀌었다** (체크리스트 §A-6 상세).

  **raw 발행자는 ``enable_camera`` 하나가 켜고, 무엇이 뜨는지는 백엔드 프로파일의
  ``camera.source`` 가 정한다** — 이 백엔드는 ``compressed`` 라
  ``image_transport/republish`` 가 뜬다. ``enable_camera`` 의 기본값은 **이 launch 의
  raw 소비자**(여기서는 뷰어뿐)에서 파생되므로, 뷰어를 켜면 디코더가 함께 뜬다.
  수집 계층의 세션 구동 레코더(``rdfp_image_recorder``)는 ``input_format='mjpeg'`` 로
  압축을 그대로 받으므로 raw 를 요구하지 않는다.
  설계: ``docs/camera/compressed_image_pipeline_design.md``

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
``publish_rate=50.0`` 으로 스트리밍한다. 명령 채널은 백엔드 프로파일이 채운다.

    client = create_move_group_client(node, backend='functionbay')
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
    create_raw_image_source_node,
    declare_camera_republish_arguments,
    declare_simulator_camera_arguments,
    warn_if_raw_consumer_has_no_source,
)
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
from robot_control.launch_helpers.ee_pose import (
    create_ee_pose_node, create_grasp_center_tf_node, declare_ee_pose_arguments)
from robot_control.launch_helpers.gripper import create_robotiq_2f_gripper_node
from robot_control.backends import get_backend
from robot_control.gripper.profile import gripper_profile_path
from robot_control.launch_helpers.scene import (
    create_functionbay_scene_node,
    declare_scene_arguments)

# **백엔드별 값은 `config/backends/functionbay.yaml` 에 있다.** 여기서 다시 적으면 두
# 곳이 갈리고, 그 어긋남은 에러가 아니라 "명령이 안 먹는다" 로 나타난다.
#
# 아래는 **launch 인자의 기본값·remap 대상**으로만 쓴다 — 덮어쓰기는 그대로다.
_BACKEND = get_backend('functionbay')
# 그리퍼 명령 규약 파일 (절대 경로). 프로파일의 `gripper.profile_file` 이 가리킨다.
_GRIPPER_PROFILE_FILE = gripper_profile_path('functionbay')

# 펑션베이가 제공하는 토픽. 시뮬레이터 쪽 고정값이다.
FB_JOINT_REPORT_TOPIC = _BACKEND.value('simulator', 'joint_report_topic')
FB_JOINT_COMMAND_TOPIC = _BACKEND.value('simulator', 'joint_command_topic')

# **`arm_command` 블록에서 온다** — 이 백엔드는 ros2_control 이 없어 MoveGroup
# 클라이언트가 이 토픽으로 직접 스트리밍한다 (JTC 백엔드와 다른 점이다).
ARM_JOINT_NAMES = _BACKEND.arm_command_joint_names
ARM_COMMAND_TOPIC = _BACKEND.arm_command_topic

# servo 출력을 받는 중간 토픽. 브리지가 이것을 JointState 로 바꿔 시뮬레이터로 넘긴다.
#
# **servo 의 `command_out_topic` 을 그대로 읽는다** — 브리지의 입력은 정의상 servo 의
# 출력이라 키가 하나여야 한다. 따로 적었다가 갈리면 servo 는 발행하고 브리지는 못 받아
# 모션 전체가 조용히 무동작이 된다.
SERVO_COMMAND_TOPIC = _BACKEND.value('servo', 'command_out_topic')

# 시뮬레이터의 그리퍼 채널 (2F-85, 6축). 실측 확정 — 설계서 §6.1.
FB_GRIPPER_COMMAND_TOPIC = _BACKEND.value('simulator', 'gripper_command_topic')
FB_GRIPPER_REPORT_TOPIC = _BACKEND.value('simulator', 'gripper_report_topic')

# 카메라 토픽 둘. **`image_pipeline.yaml` 에서 오면 안 된다** — 그쪽은 OpenCV 카메라의
# 값이라 시뮬레이터 스택에서는 프로파일이 선언한 이름과 갈린다 (실제로 갈려 있었다:
# 프로파일 `/camera_image` 대 실효 `/camera/image_raw`). 수집 계열
# `rdfp_panda_functionbay` 도 같은 프로파일을 읽으므로 두 계층이 같은 이름을 쓴다.
CAMERA_IMAGE_TOPIC = _BACKEND.value('camera', 'image_topic')
CAMERA_COMPRESSED_TOPIC = _BACKEND.value('camera', 'compressed_topic')

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
    # 형식·토픽과 그에 딸린 `publish_joint_*` 규칙은 백엔드가 갖는다
    # (`config/backends/functionbay.yaml` + `robot_control.backends`).
    servo_params = _BACKEND.apply_servo_parameters(servo_params)
    params = servo_params["moveit_servo"]
    # 실효 속도 보정용. **런타임 파라미터로는 바꿀 수 없다** — moveit_servo 는 기동
    # 시점에만 읽는다(실측: `ros2 param set` 이 성공해도 동작이 그대로였다).
    # 프로파일 값(0.8)이 인자 기본값이므로, 여기서는 인자를 다시 얹어 덮어쓰기를 살린다.
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


def create_image_viewer_node_for_functionbay() -> Node:
    """시뮬레이터 이미지를 화면에 띄우는 뷰어 노드. **기본 off** 다.

    **`robot_control` 의 `image_viewer_node` 를 쓴다** — `rdfp` 의
    `rdfp_image_viewer_node` 는 `/session` 상태를 오버레이하는 수집 계층 노드이고,
    제어 계층 launch 가 그것을 띄우면 `rdfp` 없이는 스택이 기동하지 못한다.

    인자 이름은 수집 계층과 같은 `enable_image_viewer` 다 — 같은 것을 켜고 끄는
    스위치가 계층마다 다른 이름이면 launch 를 갈아탈 때마다 안 먹는 인자를 넘기게 된다.
    **기본값만 다르다**: 수집 계층은 `image_pipeline.yaml` 의 `image_viewer.enabled` 가
    정하지만, 여기서는 시뮬레이터 스택이라 **항상 off 로 시작**한다 —
    시뮬레이터 스택이라 화면을 겹쳐 띄우지 않는 것이 기본이라는 뜻이다.

    **압축 이미지는 이 노드가 풀지 않는다.** 뷰어는 `sensor_msgs/Image` 만 구독하고
    새 빌드의 시뮬레이터는 `CompressedImage` 로 발행하므로, `enable_camera` 가 켜는
    `camera_republish` (`image_transport/republish`) 가 raw 로 되살려 준다 —
    `enable_camera` 의 기본값이 이 인자에서 파생되므로 뷰어를 켜면 함께 뜬다. 그 노드가
    없으면 **타입이 달라 ROS 2 가 연결만 안 하고 오류도 내지 않아** 증상이 "빈 창"뿐이다
    — 2026-09-08 까지 실제로 그 상태였다.

    **뷰어가 직접 디코드하지 않는 이유**는 성능이 아니다 (`cv2.imdecode` 는 0.75 ms/frame
    이고 GIL 도 놓는다). 소비자가 백엔드를 몰라야 하기 때문이다 — mock 은 장치, Isaac 은
    raw, 펑션베이는 압축이라, 뷰어에 분기를 넣으면 **제어 계층 노드가 백엔드를 알게 된다.**
    설계: `docs/camera/compressed_image_pipeline_design.md` §2.
    """
    return Node(
        package="robot_control",
        executable="image_viewer_node",
        name="image_viewer",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_image_viewer")),
        remappings=[("image", LaunchConfiguration("camera_image_topic"))],
    )


def create_gripper_node_for_functionbay() -> Node:
    """그리퍼 — **액션이 아니라 관절 지령** 구현을 쓴다.

    이 스택에는 ros2_control 이 없어 `panda_hand_controller` 액션 서버가 존재하지
    않는다. mock·Isaac 이 쓰는 `GripperActionNode` 는 그래서 쓸 수 없고, 6축 목표각을
    토픽으로 직접 쓰는 `Robotiq2FGripperNode` 가 대신한다.

    **명령 규약은 `gripper.profile_file` 이 정한다** — `/input/gripper_joint` 는 이름
    없는 배열이라 순서·단위·packing 이 계약인데 씬마다 다르다 (t1 은 6축·라디안·위치
    나열, r2 는 2축·**도**·관절당 p/v/f). 노드가 그 파일을 읽고 보고 축 수와 대조해
    어긋나면 명령을 거부한다 — 안 그러면 증상이 "그리퍼가 1.7% 만 움직인다" 뿐이다.
    """
    return create_robotiq_2f_gripper_node(
        command_topic=LaunchConfiguration("fb_gripper_command_topic"),
        report_topic=LaunchConfiguration("fb_gripper_report_topic"),
        extra_parameters={"gripper_profile_file": _GRIPPER_PROFILE_FILE},
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
        create_rviz_node(moveit_config,
                         condition=IfCondition(LaunchConfiguration("enable_rviz"))),
        # raw 이미지 발행자. 이 백엔드는 `camera.source: compressed` 라
        # `image_transport/republish` 가 뜬다 — 소비자는 어느 쪽이 떴는지 모른다.
        # 뷰어보다 먼저 띄운다 (늦게 붙어도 되지만 로그에서 짝이 붙어 보인다).
        create_raw_image_source_node(_BACKEND),
        create_image_viewer_node_for_functionbay(),
        create_grasp_center_tf_node(),
        create_ee_pose_node(),
        # 그리퍼도 시뮬레이터 준비 후에 띄운다 — 관절 보고를 받아야 판정이 선다.
        create_gripper_node_for_functionbay(),
        # scene 은 시뮬레이터가 내보내는 물체 TF 를 읽는다 (벤더 B-12, 2026-09-08).
        # `world -> panda_link0` 정적 TF 가 먼저 있어야 조회가 성립하므로 준비 후에 띄운다.
        create_functionbay_scene_node(_BACKEND.scene_file()),
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
            "servo_linear_scale",
            default_value=str(_BACKEND.value("servo", "linear_scale")),
            description=("servo 의 unitless twist → m/s 환산 계수. **MoveIt 원본값 0.4 가 "
                         "아니라 0.8 이 기본이다** — 실측으로 고른 값이다(§5.5). 런타임 "
                         "param set 으로는 바뀌지 않는다")),
        # EE 프레임 이름은 **프로파일이 갖는다** — 트윈의 `move_linear` 프레임
        # 변환도 같은 값을 읽으므로, 여기 손으로 적으면 한쪽만 뒤처진다.
        *declare_ee_pose_arguments(ee_frame=_BACKEND.value("frames", "ee")),
        DeclareLaunchArgument(
            "enable_rviz", default_value="false",
            choices=["true", "false"],
            description=("RViz2 기동 여부. **기본 off** — 시뮬레이터가 이미 자기 화면을 "
                         "그리므로 화면이 겹치고 자원만 더 쓴다")),
        DeclareLaunchArgument(
            "enable_image_viewer", default_value="false",
            choices=["true", "false"],
            description=("이미지 뷰어 노드 기동 여부. **기본 off** — 헤드리스 환경이 흔하다. "
                         "켜면 압축 이미지를 raw 로 되살리는 camera_republish 도 함께 뜬다")),
        # **raw 소비자는 뷰어뿐이다** — 레코더는 수집 계열에 있고 압축을 직접 받는다.
        # `enable_camera` 기본값이 여기서 파생되므로 뷰어를 켜면 디코더가 함께 뜬다.
        *declare_simulator_camera_arguments(CAMERA_IMAGE_TOPIC,
                                            ("enable_image_viewer", "true")),
        *declare_camera_republish_arguments(CAMERA_COMPRESSED_TOPIC),
        *declare_scene_arguments(),
        *declare_functionbay_arguments(),
        # raw 소비자를 켠 채 `enable_camera:=false` 로 덮어썼으면 알린다 — 막지는 않는다.
        warn_if_raw_consumer_has_no_source(("enable_image_viewer", "true")),
        static_tf,
        robot_state_publisher,
        joint_state_fusion,
        readiness_gate,
        startup_handler,
    ])
