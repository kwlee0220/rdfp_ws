"""Panda + MoveIt2 스택을 **Isaac Sim** 백엔드로 기동하는 launch.

**Isaac 백엔드는 이것 하나다.** `controller_manager` 가 이쪽(ROS)에서 돌고, 하드웨어는
`topic_based_ros2_control/TopicBasedSystem` 이 토픽으로 Isaac 과 말한다. 그 결과
**mock 과 같은 그림**이 된다 — JTC · GripperActionController · `joint_state_broadcaster` ·
spawner 기동 신호.

ros2_control 없이 토픽만으로 잇던 옛 경로(`panda_isaac_bridge`)는 **삭제했다** — 고유하게
덮는 것이 없었고(펑션베이가 같은 코드를 태운다), 방식이 둘이면 launch 와 어긋났을 때
에러 없이 관절 상태가 오지 않는 함정이 생겼다. 경위는 설계 문서 §0 D1″.

이 방식이 얻는 것
-----------------
- `execute_trajectory()` 가 돌아온다. 완료/중단이 액션으로 판정되므로 개루프 스트리밍의
  "성공을 반환했지만 도달하지 않았다"가 사라진다.
- 호출부가 백엔드를 몰라도 된다 — `create_move_group_client(mode='auto')` 가 JTC 를
  고르므로 트윈 설정의 `arm_command_*` 재정의 3종과 `teleop_keyboard` 의 4파라미터
  지정이 필요 없다.
- servo 가 기본 경로(`JointTrajectory` → `/panda_arm_controller/joint_trajectory`)로
  돌아온다. 브리지 노드도, `Float64MultiArray` 제약도 없다.

토픽
----
Isaac 은 관절 상태를 **`/isaac_joint_states`** 로 낸다 — `/joint_states` 는
`joint_state_broadcaster` 가 가져가기 때문이다. 같은 토픽에 둘이 발행하면
`TopicBasedSystem` 이 자기 출력을 되읽는 고리가 생기고, 소비자는 주기가 다른 발행자
둘을 보게 된다. 시뮬레이터를 `./scripts/run_isaac_sim.sh` 로 띄우면 그 이름이 나온다.

명령 토픽은 bridge 와 같다(`/isaac/arm_command`, `/isaac/gripper_command`). xacro 의
`<ros2_control>` 이 팔·손으로 나뉘어 있어 시스템마다 다른 토픽을 줄 수 있고, Isaac
그래프의 `ArticulationController` 도 이미 둘로 갈려 있기 때문이다.

기동 순서
---------
`readiness_gate` 를 **`ros2_control_node` 앞에** 둔다. spawner 만으로는 "Isaac 이 Play
중"을 보증하지 못하기 때문이다 — `controller_manager` 는 `use_sim_time` 이 켜진 채
`/clock` 이 없으면 시각 0 에 멈춘 채 조용히 아무것도 안 한다.

    readiness_gate → ros2_control_node → joint_state_broadcaster
                   → panda_arm_controller → panda_hand_controller → 나머지

실측 (2026-09-05)
-----------------
- `use_sim_time` 하의 `controller_manager` update 루프 — 컨트롤러 셋이 active 로 돌고,
  타임라인 Stop/Play 는 `resetOnStop` 이 꺼져 있어 **자동 복구**된다. 다만 Isaac
  **프로세스**를 다시 띄우면 시계가 0 으로 돌아가 스택을 다시 띄워야 한다.
- `TopicBasedSystem` 이 `JointState.effort` 를 넘긴다 — 파지 시 22.4 N·m 로
  `stall_effort` 판정이 서고 `grasp` 의 `at_goal` 이 통과한다.
- 정착 오차 0.3 mrad · 카테시안 0.2 mm · 트윈 REST 로 집기 전 과정 · 수집 전 과정 통과.

대가도 실측됐다 — 펑션베이식 토픽 브리지 대비 `/clock` 배속 −5%, load 3배,
`phase1` dead time +100 ms. 상세는 설계 문서 §2 Phase 11.

    ros2 launch robot_control panda_isaac.launch.py
    ros2 launch robot_control panda_isaac.launch.py enable_rviz:=true
"""

from __future__ import annotations

from typing import Any

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from robot_control.launch_helpers.common import (
    MOVEIT_CONFIGS_PACKAGE_NAME,
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
from robot_control.launch_helpers.controller import (
    create_joint_state_broadcaster_spawner,
    create_panda_arm_controller_spawner,
    create_panda_hand_controller_spawner,
    create_ros2_control_node,
)
from robot_control.launch_helpers.controller_startup import (
    _chain_or_shutdown,
    create_controller_startup_handlers,
)
from robot_control.launch_helpers.ee_pose import create_ee_pose_node, declare_ee_pose_arguments
from robot_control.launch_helpers.gripper import create_gripper_node
from robot_control.backends import get_backend
from robot_control.backends.isaac import COMMANDED_JOINT_STATE_TOPIC

# **백엔드별 값은 `config/backends/isaac.yaml` 에 있다.** 여기서 다시 적으면 두 곳이
# 갈리고, 그 어긋남은 에러가 아니라 "명령이 안 먹는다" 로 나타난다.
#
# 아래 값들은 **launch 인자의 기본값**으로만 쓴다 — 덮어쓰기는 그대로 살아 있다.
_BACKEND = get_backend('isaac')

# `description/panda.ros2_control.xacro` 의 분기 이름.
HARDWARE_TYPE = _BACKEND.hardware_type

# Isaac 이 관절 상태를 내는 토픽. **`/joint_states` 가 아니다** — 그것은
# `joint_state_broadcaster` 가 소유한다 (모듈 docstring 「토픽」 참조).
JOINT_STATE_TOPIC = _BACKEND.value('ros2_control', 'joint_states_topic')

# Isaac 이 관절 명령을 받는 토픽. **`arm_command` 블록이 아니다** — 그쪽은 MoveGroup
# 클라이언트의 채널이고, 이것은 TopicBasedSystem 이 시뮬레이터와 주고받는 채널이다.
ARM_COMMAND_TOPIC = _BACKEND.value('ros2_control', 'joint_commands_topic')
GRIPPER_COMMAND_TOPIC = _BACKEND.value('ros2_control', 'gripper_commands_topic')

# 관절 한계 파일. 기본 moveit_resources 의 URDF 한계는 실제 Franka 보다 전 관절이
# 4° 헐겁고(panda_joint4 상한은 9°), **Isaac 은 실제 스펙을 강제한다.** 그대로 두면
# MoveIt 이 계획한 자세를 시뮬레이터가 한계에서 막고, 그것이 추종 오차처럼 보인다.
JOINT_LIMITS_RELPATH = os.path.join('config', 'panda_real_joint_limits.yaml')


def declare_isaac_arguments() -> list[DeclareLaunchArgument]:
    """Isaac 백엔드 전용 argument."""
    return [
        DeclareLaunchArgument(
            'joint_state_topic', default_value=JOINT_STATE_TOPIC,
            description=(
                'Isaac 이 발행하는 관절 상태 토픽. TopicBasedSystem 이 읽고 '
                'readiness_gate 가 기다린다. `/joint_states` 를 주면 '
                'joint_state_broadcaster 와 충돌한다'
            ),
        ),
        DeclareLaunchArgument(
            'arm_command_topic', default_value=ARM_COMMAND_TOPIC,
            description='TopicBasedSystem(팔) 이 관절 명령을 내보낼 토픽',
        ),
        DeclareLaunchArgument(
            'gripper_command_topic', default_value=GRIPPER_COMMAND_TOPIC,
            description='TopicBasedSystem(손) 이 관절 명령을 내보낼 토픽',
        ),
        DeclareLaunchArgument(
            'isaac_ready_timeout', default_value='60.0',
            description='Isaac 첫 관절 보고 대기 한도(초). 초과하면 launch 가 실패한다',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value=str(_BACKEND.use_sim_time).lower(),
            description=(
                'Isaac 의 /clock 을 시간 원본으로 쓴다. controller_manager 에도 '
                '얹히므로 /clock 이 없으면 update 루프가 시각 0 에 멈춘다 — '
                'readiness_gate 가 그 앞을 막는다'
            ),
        ),
        DeclareLaunchArgument(
            'enable_servo', default_value='true', choices=['true', 'false'],
            description=(
                'servo(twist) 경로. teleop_keyboard 와 teleop_retarget 이 모두 '
                '여기로 수렴하므로 **기본으로 켠다** — 끄면 teleop 이 조용히 죽는다'
            ),
        ),
        DeclareLaunchArgument(
            'servo_linear_scale',
            default_value=str(_BACKEND.value('servo', 'linear_scale')),
            description=(
                'servo 의 unitless twist 환산 계수. **m/s 가 아니다** — 실측 EE 속도는 '
                '입력 1.0 에서 130 mm/s, 0.5 에서 63, 0.25 에서 28 (2026-09-07, 래칫 수정 후; '
                '수정 전에는 되읽기가 이동량을 먹어 47.5/24.5/11.5 였다). 비례는 유지된다. '
                '속도를 바꾸려면 teleop 의 linear_step 을 만진다. **런타임 param set 으로는 안 바뀐다**'
            ),
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=_BACKEND.controllers_file(),
            description=(
                'ros2_control 컨트롤러 설정 yaml. 기본은 이 패키지의 복사본으로 JTC 에 '
                '`open_loop_control: true` 가 들어 있다 (servo 래칫의 두 번째 되읽기 지점). '
                '원래 동작을 재현하려면 moveit_resources_panda_moveit_config 의 '
                'ros2_controllers.yaml 을 주고 servo_joint_source:=measured 를 함께 준다'
            ),
        ),
        DeclareLaunchArgument(
            'servo_joint_source',
            default_value=_BACKEND.value('servo', 'joint_source'),
            choices=['commanded', 'measured'],
            description=(
                'servo 가 매 주기 IK 증분을 더할 **기준 관절 상태**. measured 는 servo 원래 '
                '동작(/joint_states)이고, commanded 는 컨트롤러가 추종 중인 명령 위치'
                '(commanded_joint_state_node 가 /joint_states_commanded 로 낸다). '
                '**기본은 commanded** — measured 는 부하로 처진 만큼을 다음 명령에 적분하는 '
                '래칫이 있다: 100 g 을 쥔 채 +z 1.5 초에 joint5 가 −0.22 rad 돌아 손끝이 옆으로 '
                '30 mm 샜다 (빈손 3 mm). 원인·실측: docs/teleop/servo_vs_planned_motion.md §3'
            ),
        ),
        DeclareLaunchArgument(
            'enable_gripper', default_value='true', choices=['true', 'false'],
            description=(
                'GripperActionNode 기동 여부. 액션 서버는 panda_hand_controller 가 '
                '제공한다 — bridge 와 달리 다리 노드가 없다'
            ),
        ),
        DeclareLaunchArgument(
            'enable_rviz', default_value='false', choices=['true', 'false'],
            description=(
                'RViz2 기동 여부. **기본 off** — Isaac 자체가 뷰포트를 그리므로 '
                '화면이 겹치고 VRAM 만 더 쓴다'
            ),
        ),
        DeclareLaunchArgument(
            'enable_image_viewer', default_value='true', choices=['true', 'false'],
            description=(
                'Isaac 카메라 이미지를 창으로 띄운다. ⚠️ **GUI 가 필요하다** — '
                '화면 없는 곳에서는 false 로 끈다'
            ),
        ),
        DeclareLaunchArgument(
            'camera_image_topic',
            default_value=_BACKEND.value('camera', 'image_topic'),
            description='뷰어가 구독할 이미지 토픽. Isaac 그래프가 내는 이름이다',
        ),
        DeclareLaunchArgument(
            'enable_scene', default_value='true', choices=['true', 'false'],
            description=(
                'Isaac 이 TF 로 내보낸 물체 pose 를 /scene/objects 로 바꾼다. '
                '**기본으로 켠다** — 데이터셋 채널이고 /scene/reset 도 이 노드가 연다'
            ),
        ),
        DeclareLaunchArgument(
            'scene_publish_rate', default_value='2.0',
            description='/scene/objects 발행 Hz',
        ),
    ]


def create_readiness_gate_node() -> Node:
    """Isaac 의 첫 관절 보고를 기다렸다 종료하는 게이트.

    **`ros2_control_node` 보다 앞이다.** spawner 는 `controller_manager` 만 뜨면
    성공하므로 "Isaac 이 Play 중"을 보증하지 못한다. 게이트 없이 CM 을 먼저 띄우면
    `/clock` 이 없어 update 루프가 시각 0 에 멈추고, 증상은 '컨트롤러는 active 인데
    아무것도 안 움직인다'가 된다.

    한도는 `time.monotonic()` 으로 잰다(노드 구현). ROS 클럭으로 재면 `use_sim_time`
    이 켜진 채 sim time 이 이미 커진 시뮬레이터에 붙을 때 **첫 `/clock` 이 오는 순간
    곧바로 타임아웃**한다 — 실측으로 60초 한도가 113 ms 만에 터진 적이 있다.
    """
    return Node(
        package='robot_control',
        executable='fb_readiness_gate',
        name='readiness_gate',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'topic': LaunchConfiguration('joint_state_topic'),
            'timeout_sec': LaunchConfiguration('isaac_ready_timeout'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )


def _servo_params_with_scale() -> dict:
    """servo 파라미터에 `servo_linear_scale` 을 얹는다.

    **런타임 파라미터로는 바꿀 수 없다** — `moveit_servo` 는 기동 시점에만 읽는다
    (`ros2 param set` 이 성공을 반환해도 동작은 그대로다). 그래서 launch 인자로 뺀다.

    `command_in_type: unitless` 라 이 값은 **m/s 가 아니다.** Isaac 실측, `ready` 에서
    +z 1.5 초, scale 0.4:

    ==================================  =========  =========  =========  ==========
    스택                                 입력 1.0   입력 0.5   입력 0.25  비례성
    ==================================  =========  =========  =========  ==========
    **2026-09-07 래칫 수정 후 (현재)**    130 mm/s   63 mm/s    28 mm/s    유지
    2026-09-06 수정 전 (되읽기 있음)      47.5       24.5       11.5       유지
    2026-09-06 수정 전, scale 0.8         56.8       39.6       22.9       무너짐
    ==================================  =========  =========  =========  ==========

    **같은 scale 에서 속도가 2.7배 올랐다.** 수정 전에는 servo 와 JTC 가 매 주기 측정
    상태로 되돌아가며 이동량의 상당 부분을 먹고 있었다(`servo_joint_source` 인자 설명).
    수정 후 값이 servo 가 실제로 내는 양이다. **0.4 는 그대로 둔다** — 비례가 유지되고,
    체감 속도는 teleop 의 `linear_step`/`angular_step`(기본 0.25 — 이 2.7배를 되맞춘 값)
    쪽에서 잡는다.

    펑션베이는 0.8 을 쓰지만 그쪽은 0.4 에서 상하 비대칭이 6.7배라 사정이 달랐다 —
    백엔드마다 따로 재야 한다.
    """
    params = build_servo_params()
    servo = dict(params.get('moveit_servo') or {})
    servo['scale'] = dict(servo.get('scale') or {})
    servo['scale']['linear'] = LaunchConfiguration('servo_linear_scale')
    # 기준 상태 토픽. `servo_joint_source` 참고 — commanded 면 commanded_joint_state_node 의
    # 출력을, measured 면 servo 원래대로 /joint_states 를 본다.
    servo['joint_topic'] = PythonExpression([
        "'", COMMANDED_JOINT_STATE_TOPIC, "' if '", LaunchConfiguration('servo_joint_source'),
        "' == 'commanded' else '/joint_states'"])
    return {'moveit_servo': servo}


def create_servo_nodes(moveit_config, sim_time: dict) -> list[Node]:
    """servo(twist) 경로 — servo_node + auto_start.

    **bridge 와 갈리는 지점이다.** 거기서는 servo 출력을 `Float64MultiArray` 로 바꾸고
    다리 노드로 `sensor_msgs/JointState` 를 만들어야 했다. 여기서는 JTC 가 있으므로
    servo 의 기본 출력(`JointTrajectory` → `/panda_arm_controller/joint_trajectory`)이
    그대로 쓰인다 — 파라미터 재정의도, 다리도 없다.

    `servo_auto_start_node` 는 남는다. **servo 는 `start_servo` 를 부르기 전까지 입력을
    조용히 무시한다** — 에러도 경고도 없이 그냥 안 움직인다.
    """
    condition = IfCondition(LaunchConfiguration('enable_servo'))
    commanded = IfCondition(PythonExpression([
        "'", LaunchConfiguration('enable_servo'), "' == 'true' and '",
        LaunchConfiguration('servo_joint_source'), "' == 'commanded'"]))
    return [
        # servo 의 기준 상태를 측정값이 아니라 컨트롤러의 명령 위치로 바꾼다 — 부하 처짐이
        # servo 명령에 적분되는 래칫을 끊는다 (`servo_joint_source` 인자 설명 참고).
        # `controller_state` 만 remap 한다; `joint_states`(손가락 통과용)와 출력은 루트 상대다.
        Node(
            package='robot_control', executable='commanded_joint_state_node',
            name='commanded_joint_state_publisher', output='screen', emulate_tty=True,
            parameters=[sim_time],
            remappings=[('controller_state', '/panda_arm_controller/controller_state')],
            condition=commanded,
        ),
        create_servo_node(moveit_config, _servo_params_with_scale(), condition=condition,
                          extra_parameters=sim_time),
        Node(
            package='robot_control', executable='servo_auto_start_node',
            name='servo_auto_start', output='screen', emulate_tty=True,
            parameters=[sim_time],
            condition=condition,
        ),
    ]


def create_gripper_nodes(sim_time: dict) -> list[Node]:
    """그리퍼 — `GripperActionNode` 하나뿐이다.

    `panda_hand_controller`(GripperActionController) 가 액션 서버이므로 mock 과 같은
    구성이 된다 — 다리 노드가 없다.

    **`stall_effort` 는 실측값이다 (2026-09-02).** 빈손으로 닫은 채 팔을 흔들어도
    손가락 |effort| 가 0.13 N·m 를 넘지 않았고(과도 최대 0.50), 블록을 물면 22.4 N·m
    로 유지된다. **속도 조건은 켜지 않는다** — Isaac 은 PhysX 접촉에서 손가락이 계속
    떨려(파지 유지 중 |vel| 최대 0.26 rad/s) 속도 게이트를 걸면 파지를 놓친다.

    이 판정은 effort state interface 를 전제하며, `TopicBasedSystem` 이 그것을 넘기는
    것을 실측으로 확인했다 (파지 22.4 N·m).
    """
    return [
        create_gripper_node(
            extra_parameters={**sim_time, 'stall_effort': 1.0, 'stall_velocity': 0.0},
            condition=IfCondition(LaunchConfiguration('enable_gripper')),
        ),
    ]


def create_image_viewer_node(sim_time: dict) -> Node:
    """Isaac 카메라 이미지를 창으로 띄운다.

    **카메라 노드는 없다** — Isaac 이 OmniGraph 로 이미지를 직접 발행한다.
    """
    return Node(
        package='robot_control', executable='image_viewer_node',
        name='image_viewer', output='screen', emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_image_viewer')),
        parameters=[sim_time],
        remappings=[('image', LaunchConfiguration('camera_image_topic'))],
    )


def create_scene_node(sim_time: dict) -> Node:
    """Isaac 물체 TF → `/scene/objects`.

    물체의 이름·종류·크기는 TF 에 없으므로 시뮬레이터 쪽 생성 스크립트와 **같은
    JSON**(`config/isaac_scene.json`)을 읽는다.
    """
    return Node(
        package='robot_control', executable='isaac_scene_state_node',
        name='isaac_scene_state', output='screen', emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_scene')),
        # `config_file` 은 **프로파일의 `scene.file`** 에서 온다. 안 넘기면 노드가 자기
        # 기본 경로로 떨어져 프로파일에 적힌 값이 아무 데도 안 닿는다 — "고쳤는데 안
        # 바뀐다" 가 된다.
        parameters=[sim_time, {'publish_rate': LaunchConfiguration('scene_publish_rate'),
                               'config_file': _BACKEND.scene_file() or ''}],
    )


def generate_launch_description() -> LaunchDescription:
    joint_limits_file = os.path.join(
        get_package_share_directory('robot_control'), JOINT_LIMITS_RELPATH)
    moveit_config = build_moveit_config(
        joint_limits_file=joint_limits_file,
        description_mappings={
            'isaac_arm_command_topic': LaunchConfiguration('arm_command_topic'),
            'isaac_gripper_command_topic': LaunchConfiguration('gripper_command_topic'),
            'isaac_joint_states_topic': LaunchConfiguration('joint_state_topic'),
        },
    )
    # 모든 노드에 같은 dict 를 넘긴다. SetParameter 를 쓰지 않는 이유는 게이트 통과 후
    # event handler 로 뜨는 노드까지 확실히 덮기 위해서다.
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    # --- 즉시 기동: TF 소스와 게이트 ---
    static_tf = create_static_tf_node(extra_parameters=sim_time)
    robot_state_publisher = create_robot_state_publisher(moveit_config, extra_parameters=sim_time)
    readiness_gate = create_readiness_gate_node()

    # --- 게이트 통과 후: ros2_control 과 컨트롤러 사슬 ---
    # 컨트롤러 설정은 **이 패키지의 복사본**을 쓴다 — moveit_resources 것에 JTC 의
    # `open_loop_control: true` 를 더한 것으로, servo 래칫의 두 번째 되읽기 지점을 끊는다.
    # 인라인 파라미터로는 컨트롤러에 닿지 않아 파일이어야 한다 (yaml 머리말 참고).
    ros2_control_node = create_ros2_control_node(
        moveit_config, MOVEIT_CONFIGS_PACKAGE_NAME,
        controllers_file=LaunchConfiguration('controllers_file'), extra_parameters=sim_time)
    joint_state_broadcaster_spawner = create_joint_state_broadcaster_spawner()
    panda_arm_controller_spawner = create_panda_arm_controller_spawner()
    panda_hand_controller_spawner = create_panda_hand_controller_spawner()

    post_controller_nodes: list[Any] = [
        create_move_group_node(moveit_config, extra_parameters=sim_time),
        create_rviz_node(moveit_config, condition=IfCondition(LaunchConfiguration('enable_rviz')),
                         extra_parameters=sim_time),
        create_ee_pose_node(extra_parameters=sim_time),
        *create_servo_nodes(moveit_config, sim_time),
        *create_gripper_nodes(sim_time),
        create_scene_node(sim_time),
        create_image_viewer_node(sim_time),
    ]

    # 게이트가 실패 종료하면 launch 전체를 내린다 — move_group 만 올라와 원인을
    # 감추는 상황을 막는다 (bridge 와 같은 정책).
    gate_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=readiness_gate,
            on_exit=_chain_or_shutdown([ros2_control_node], 'readiness_gate'),
        )
    )

    controller_startup_handlers = create_controller_startup_handlers(
        ros2_control_node,
        joint_state_broadcaster_spawner,
        panda_arm_controller_spawner,
        panda_hand_controller_spawner,
        post_controller_nodes,
    )

    return LaunchDescription([
        declare_ros2_control_hardware_type_argument(default_value=HARDWARE_TYPE),
        declare_log_level_argument(),
        *declare_ee_pose_arguments(),
        *declare_isaac_arguments(),
        static_tf,
        robot_state_publisher,
        readiness_gate,
        gate_handler,
        *controller_startup_handlers,
    ])
