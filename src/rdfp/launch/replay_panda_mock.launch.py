"""데이터셋 재생(replay) 전용 Panda + MoveIt2 런치.

:mod:`rdfp_panda_mock.launch` 와 유사한 스택을 띄우되, **재생 시 데이터셋에서
이미 제공되는 정보** 를 생성하는 노드들은 제외한다.

재생 도구(예: ``ros2 bag play`` / 커스텀 dataset player) 가 다음 토픽들을
퍼블리시한다는 전제 하에 구성된다.

- ``/joint_states`` — :mod:`rdfp_panda_mock.launch` 의 ``joint_state_broadcaster``
  를 통해 실시간 관측값을 사용한다 (대체하지 않는다).
- ``/ee_pose`` — 데이터셋에서 재생되므로 ``ee_pose_publisher`` 를 띄우지 않는다.
- 이미지 (``/camera/...``) — 데이터셋에서 재생되므로 ``camera`` 노드를 띄우지
  않는다. 재생된 이미지 토픽을 구독하는 ``rdfp_image_viewer_node`` 는 유지한다.
- ``/gripper_cmds`` — 데이터셋 재생 쪽에서 발행한다. 본 런치의
  ``GripperNode`` 가 이를 받아 gripper action 으로 중계한다.
- ``/target_joint_cmds`` — 데이터셋에서 재생되므로
  ``target_joint_cmds_publisher`` 를 띄우지 않는다.
- 세션 / 녹화는 **수행하지 않는다** (``session_control`` / ``image_recorder``
  제외).

arm 재생 경로 (``replay_arm_path``)
-----------------------------------

arm 을 구동하는 노드 조합을 ``replay_arm_path`` argument 로 고른다. 두 경로가
동시에 ``/panda_arm_controller/joint_trajectory`` 를 쓰면 명령이 충돌하므로
**배타적으로만 기동한다**.

``ee_twist`` (기본값)
    ``/ee_pose`` (PoseStamped) → ``ee_twist_publisher`` (유한 차분) →
    ``/servo_node/delta_twist_cmds`` → ``servo_node`` →
    ``/panda_arm_controller/joint_trajectory``. servo 는 ``start_servo`` 호출
    전까지 입력을 무시하므로 ``servo_auto_start_node`` 를 함께 띄운다.

    **개루프** 경로다. 적분 드리프트가 누적되고, 시작 pose 가 녹화 때와 다르면
    전체가 오프셋되며, EE twist 는 6-DOF 라 7-DOF 여유자유도가 재현되지 않는다.
    servo 의 ``command_in_type: unitless`` 를 보정하기 위해 게인 기본값이
    ``linear 2.5`` / ``angular 1.25`` 로 설정되어 있다. 근거와 대안 비교는
    ``docs/replay/replay_approaches.md`` 참고.

``target_joint_cmds``
    ``/target_joint_cmds`` (sensor_msgs/JointState) → ``target_joint_cmds_executor``
    → ``/panda_arm_controller/joint_trajectory``. 관절 절대 위치를 그대로
    재생하므로 **위치 폐루프** 이며 여유자유도(팔꿈치 형상)까지 원본과 일치한다.
    재현 충실도는 이쪽이 가장 높다.

    DB 의 ``joint_states`` 테이블은 joint 이름을 저장하지 않으므로 재생된
    JointState 의 ``name`` 은 비어 있다. executor 에 ``joint_names`` 파라미터로
    panda_joint1~7 을 넘겨 폴백하게 한다.

``none``
    arm 구동 노드를 띄우지 않는다. ``MoveGroupClient.follow_trajectory`` 처럼
    외부 클라이언트로 직접 구동할 때 사용한다.

기동되는 노드 단위:

- ``ros2_control``:
    - /controller_manager
    - /joint_state_broadcaster
    - /panda_arm_controller
    - /panda_hand_controller
    - /static_transform_publisher
    - /robot_state_publisher
- ``moveit``:
    - /move_group
    - /moveit_servo
- ``rviz2``: /rviz2
- ``rdfp 애플리케이션``:
    - /rdfp_image_viewer_node
    - /gripper_control
    - /ee_twist_publisher, /servo_auto_start (``replay_arm_path=ee_twist``, 기본)
    - /target_joint_cmds_executor (``replay_arm_path=target_joint_cmds``)

기동 정책은 :mod:`panda_mock.launch` 와 동일하다: ``panda_hand_controller``
spawner 가 종료되면 상위 노드들을 일괄 spawn 한다.

설정 파일
---------

**전용** YAML 을 쓴다 — ``rdfp_panda_mock.launch`` 와 공유하지 않는다.

- 기본 경로: ``<rdfp share>/config/replay_panda_mock.yaml``
- ``config_file:=<path>`` launch argument 로 다른 YAML 을 지정할 수 있다.
  ``$HOME`` / ``~`` 같은 경로 확장은 쉘에 맡긴다
  (예: ``config_file:=$HOME/my.yaml`` — 쉘이 먼저 확장한 절대경로가 전달된다).
- CLI 에서 ``arg:=value`` 로 개별 argument 를 덮어쓰는 것은 그대로 동작한다.

과거에는 ``panda_robot.yaml`` 을 공유했는데, 본 런치가 기동하지 않는 노드
(camera / ee_pose_publisher / image_recorder / session_control) 의 키가 절반을
넘어 **고쳐도 아무 일이 일어나지 않으면서 경고도 없었다.** 전용 파일에는 실제로
소비자가 있는 키만 두어, 없는 블록이 곧 "그 노드를 안 띄운다"는 뜻이 되도록 했다.

YAML 키 ↔ argument 대응표는 ``src/rdfp/launch/README.md`` 의 "Launch 인자" 절에
있다. ``config_file`` 이 ``OpaqueFunction`` 안에서 resolve 된 **뒤에야** 나머지
argument 가 선언되므로 ``--show-args`` 는 ``config_file`` 하나만 출력한다.
"""

from __future__ import annotations

from typing import Any

import os

import yaml


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_control.launch_helpers.gripper import create_gripper_node
from robot_control.launch_helpers.controller import (
    create_joint_state_broadcaster_spawner,
    create_panda_arm_controller_spawner,
    create_panda_hand_controller_spawner,
    create_ros2_control_node,
)
from robot_control.launch_helpers.controller_startup import create_controller_startup_handlers
from robot_control.launch_helpers.common import (
    MOVEIT_CONFIGS_PACKAGE_NAME,
    build_moveit_config,
    build_servo_params,
    create_move_group_node,
    create_robot_state_publisher,
    create_rviz_node,
    create_servo_node,
    create_static_tf_node,
)

# YAML 설정 파일의 기본 경로. setup.py 가 ``config/*`` 를
# ``share/rdfp/config/`` 로 설치하므로 package share 에서 읽는다.
#
# panda_robot.yaml 과 **별도 파일**이다. 본 런치는 camera / ee_pose_publisher /
# session_control / image_recorder / target_joint_cmds_publisher 를 기동하지 않으므로
# 공유 파일을 쓰면 값이 조용히 무시되는 키가 절반을 넘었다.
DEFAULT_CONFIG_RELPATH = os.path.join("config", "replay_panda_mock.yaml")

# arm 재생 경로 선택지. `replay_arm_path` launch argument 의 허용 값이다.
_ARM_PATH_TARGET_JOINT_CMDS = "target_joint_cmds"
_ARM_PATH_EE_TWIST = "ee_twist"
_ARM_PATH_NONE = "none"
_ARM_PATHS = (_ARM_PATH_TARGET_JOINT_CMDS, _ARM_PATH_EE_TWIST, _ARM_PATH_NONE)

# JointState 명령에 name 이 비어 있을 때 사용할 joint 이름. DB 의 joint_states
# 테이블은 이름을 저장하지 않으므로 DB 재생 경로에서는 항상 이 값이 쓰인다.
_PANDA_ARM_JOINT_NAMES = [
    "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
    "panda_joint5", "panda_joint6", "panda_joint7",
]


def _default_config_path() -> str:
    """패키지 share 경로의 기본 YAML 위치를 반환한다."""
    return os.path.join(get_package_share_directory("rdfp"), DEFAULT_CONFIG_RELPATH)


def _load_config(config_path: str) -> dict[str, Any]:
    """YAML 설정 파일을 로드하여 dict 로 반환한다."""
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _as_launch_str(value: Any) -> str:
    """Python 값을 DeclareLaunchArgument 의 default_value 로 쓰이는 문자열로 변환한다.

    - bool 은 ROS launch 관례에 맞춰 소문자 ``"true"`` / ``"false"`` 로 변환한다.
    - 그 외는 ``str()`` 으로 변환한다.
    """
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _declare_arguments(config: dict[str, Any]) -> list[DeclareLaunchArgument]:
    """YAML 설정값을 default 로 사용하는 DeclareLaunchArgument 목록을 생성한다.

    helper 모듈의 ``declare_*_arguments()`` 는 기본값을 하드코딩하므로, YAML
    기반 기본값을 적용하려면 본 launch 파일에서 직접 선언해야 한다. helper 의
    ``create_*_node()`` 는 ``LaunchConfiguration(<name>)`` 으로 값을 참조하므로
    여기서 선언한 argument 이름만 일치시키면 그대로 동작한다.

    **실제로 소비자가 있는 argument 만 선언한다.** 과거에는
    ``rdfp_panda_mock.launch`` 에서 통째로 복사한 camera / image_recorder /
    publish_rate argument 13개가 함께 선언되어 있었는데, 본 런치가 그 노드들을
    기동하지 않으므로 값이 조용히 무시되었다 (경고도 없었다).
    """
    rc = config["ros2_control"]
    ee = config["ee_pose"]
    iv = config["image_viewer"]
    rp = config["replay"]
    et = rp["ee_twist"]

    return [
        # --- ros2_control ---
        DeclareLaunchArgument(
            "ros2_control_hardware_type",
            default_value=_as_launch_str(rc["hardware_type"]),
            description=(
                "ROS 2 control hardware interface type "
                "(e.g. mock_components for fake hardware)"
            ),
        ),
        # --- log_level ---
        DeclareLaunchArgument(
            "log_level",
            default_value=_as_launch_str(config["log_level"]),
            choices=["debug", "info", "warn", "error", "fatal"],
            description="MoveIt2 주요 노드(move_group / servo) 및 session_control 의 로그 레벨",
        ),
        # --- ee_pose ---
        DeclareLaunchArgument(
            "base_frame",
            default_value=_as_launch_str(ee["base_frame"]),
            description="Base frame for EE pose TF lookup",
        ),
        DeclareLaunchArgument(
            "ee_frame",
            default_value=_as_launch_str(ee["ee_frame"]),
            description="End-effector frame for EE pose TF lookup",
        ),
        # --- image_viewer ---
        # camera_node 는 기동하지 않는다. 재생 도구가 발행하는 이미지 토픽을
        # 뷰어가 구독할 뿐이므로 image_viewer 블록에 토픽을 둔다. argument 이름은
        # 기존 사용처 호환을 위해 `camera_image_topic` 을 유지한다.
        DeclareLaunchArgument(
            "enable_image_viewer",
            default_value=_as_launch_str(iv["enabled"]),
            description="Whether to start rdfp_image_viewer_node",
        ),
        DeclareLaunchArgument(
            "camera_image_topic",
            default_value=_as_launch_str(iv["image_topic"]),
            description="Replayed image topic the viewer subscribes to ('image' remap target)",
        ),
        # --- arm 재생 경로 ---
        DeclareLaunchArgument(
            "replay_arm_path",
            default_value=_as_launch_str(rp["arm_path"]),
            choices=list(_ARM_PATHS),
            description=(
                "Which node chain drives the arm during replay. "
                "'target_joint_cmds': /target_joint_cmds -> executor -> JTC "
                "(위치 폐루프, 여유자유도까지 재현). "
                "'ee_twist': /ee_pose -> ee_twist_publisher -> servo -> JTC "
                "(개루프, 드리프트 있음). "
                "'none': arm 구동 노드를 띄우지 않는다 "
                "(MoveGroupClient.follow_trajectory 등 외부 클라이언트로 구동할 때)"
            ),
        ),
        DeclareLaunchArgument(
            "ee_twist_source_topic",
            default_value=_as_launch_str(et["source_topic"]),
            description=(
                "PoseStamped input topic differentiated into twist "
                "(replay_arm_path=ee_twist)"
            ),
        ),
        DeclareLaunchArgument(
            "ee_twist_output_topic",
            default_value=_as_launch_str(et["output_topic"]),
            description="TwistStamped output topic consumed by servo (replay_arm_path=ee_twist)",
        ),
        DeclareLaunchArgument(
            "ee_twist_linear_gain",
            default_value=_as_launch_str(et["linear_gain"]),
            description=(
                "Linear twist gain. servo 의 command_in_type=unitless 를 보정하는 "
                "1/scale.linear 값 (speed_units 운용 시 1.0)"
            ),
        ),
        DeclareLaunchArgument(
            "ee_twist_angular_gain",
            default_value=_as_launch_str(et["angular_gain"]),
            description=(
                "Angular twist gain. servo 의 command_in_type=unitless 를 보정하는 "
                "1/scale.rotational 값 (speed_units 운용 시 1.0)"
            ),
        ),
        DeclareLaunchArgument(
            "ee_twist_max_dt",
            default_value=_as_launch_str(et["max_dt"]),
            description=(
                "Maximum sample gap in seconds. 이 값을 넘는 공백 뒤에는 속도 스파이크를 "
                "피하기 위해 한 주기를 건너뛴다"
            ),
        ),
        DeclareLaunchArgument(
            "servo_start_timeout",
            default_value=_as_launch_str(rp["servo_start_timeout"]),
            description="Seconds to wait for /servo_node/start_servo (replay_arm_path=ee_twist)",
        ),
    ]


def _resolve_arm_path(context: LaunchContext, config: dict[str, Any]) -> str:
    """arm 재생 경로를 결정한다 (CLI 우선, 없으면 YAML).

    `_build_actions` 는 이 값으로 **기동할 노드 자체를 고르므로** 즉시 읽어야
    하는데, argument 선언은 그 함수가 반환한 뒤에야 실행된다. 따라서
    `LaunchConfiguration(...).perform(context)` 은 쓸 수 없다 (CLI 로 값을 주지
    않으면 "launch configuration does not exist" 로 실패한다). CLI `arg:=value`
    는 launch 실행 전에 이미 context 에 들어와 있으므로 그것만 조회한다.
    """
    return context.launch_configurations.get(
        "replay_arm_path", _as_launch_str(config["replay"]["arm_path"])
    )


def _build_actions(context: LaunchContext) -> list:
    """`config_file` 이 resolve 된 뒤 YAML 을 로드해 나머지 argument/노드를 구성한다.

    `OpaqueFunction` 이 실행되는 시점에는 `LaunchConfiguration` 이 이미
    resolve 가능하므로, top-level CLI 의 ``config_file:=...`` 든
    ``IncludeLaunchDescription(..., launch_arguments=...)`` 든 동일하게 반영된다.
    """
    config_path = LaunchConfiguration("config_file").perform(context)
    config = _load_config(config_path)

    moveit_config = build_moveit_config()
    servo_params = build_servo_params()

    # --- ros2_control 스택 ---
    static_tf = create_static_tf_node()
    robot_state_publisher = create_robot_state_publisher(moveit_config)
    ros2_control_node = create_ros2_control_node(moveit_config, MOVEIT_CONFIGS_PACKAGE_NAME)
    joint_state_broadcaster_spawner = create_joint_state_broadcaster_spawner()
    panda_arm_controller_spawner = create_panda_arm_controller_spawner()
    panda_hand_controller_spawner = create_panda_hand_controller_spawner()

    # --- MoveIt / RViz / 주변 노드 ---
    # replay 모드에서는 /camera, /ee_pose_publisher, /gripper_control 은 기동하지
    # 않는다 (프레임/포즈/그리퍼 명령이 모두 데이터셋 재생에서 공급됨).
    move_group_node = create_move_group_node(moveit_config)
    servo_node = create_servo_node(moveit_config, servo_params)
    rviz_node = create_rviz_node(moveit_config)

    # --- rdfp 애플리케이션: RdfpImageViewerNode ---
    # 재생 중인 이미지 토픽을 구독하여 수신 프레임 좌상단에 상태 오버레이를
    # 덧입혀 표시한다. /session_control 이 없으므로 오버레이의 세션 상태 필드는
    # 기본값으로 유지된다.
    rdfp_image_viewer_node = Node(
        package="rdfp",
        executable="rdfp_image_viewer_node",
        name="rdfp_image_viewer_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_image_viewer")),
        remappings=[
            ("image", LaunchConfiguration("camera_image_topic")),
        ],
    )

    # --- rdfp 애플리케이션: GripperNode ---
    # 데이터셋 재생 도구가 발행하는 `rdfp_msgs/GripperCommand` 를 받아 gripper
    # action 으로 중계한다. 수집 때와 **같은 노드**이며, 다른 점은 명령 토픽의
    # 퍼블리셔가 teleop/트윈이 아니라 재생 도구라는 것뿐이다.
    gripper_node = create_gripper_node()

    # --- arm 재생 경로 선택 ---
    # `replay_arm_path` 값에 따라 arm 을 구동하는 노드 조합이 달라진다. 두 경로가
    # 동시에 `/panda_arm_controller/joint_trajectory` 를 쓰면 명령이 충돌하므로
    # 배타적으로만 기동한다.
    arm_path = _resolve_arm_path(context, config)
    if arm_path not in _ARM_PATHS:
        raise RuntimeError(
            f"'replay_arm_path' must be one of {list(_ARM_PATHS)}, got {arm_path!r}"
        )
    arm_path_nodes: list[Node] = []

    # --- rdfp 애플리케이션: TargetJointCmdsExecutor (replay_arm_path=target_joint_cmds) ---
    # 재생 도구가 발행하는 `/target_joint_cmds` (sensor_msgs/JointState) 를 받아
    # 길이 1 짜리 `trajectory_msgs/JointTrajectory` 로 래핑하여
    # `/panda_arm_controller/joint_trajectory` 로 흘려보낸다 — panda_arm_controller
    # 가 이 토픽을 consume 하여 실제 관절 궤적을 실행한다.
    #
    # 관절 절대 위치를 그대로 재생하므로 위치 폐루프이며 여유자유도(팔꿈치 형상)까지
    # 원본과 일치한다. 재현 충실도가 가장 높은 경로다.
    #
    # joint_names 를 명시하는 이유: DB 의 joint_states 테이블은 position/velocity/
    # effort 만 저장하고 이름은 버린다. 따라서 DB 재생으로 들어온 JointState 는 name
    # 이 비어 있고, 이 파라미터가 폴백으로 쓰인다.
    if arm_path == _ARM_PATH_TARGET_JOINT_CMDS:
        arm_path_nodes.append(Node(
            package="robot_control",
            executable="target_joint_cmds_executor",
            name="target_joint_cmds_executor",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "joint_names": _PANDA_ARM_JOINT_NAMES,
            }],
            remappings=[
                ("target_joint_cmds", "/target_joint_cmds"),
                ("joint_trajectory", "/panda_arm_controller/joint_trajectory"),
            ],
        ))

    # --- rdfp 애플리케이션: EeTwistPublisher + ServoAutoStart (replay_arm_path=ee_twist) ---
    # 재생된 `/ee_pose` (geometry_msgs/PoseStamped) 를 유한 차분하여 twist 로 바꾸고
    # servo 의 Cartesian 입력 토픽으로 발행한다. servo 가 이를 관절 궤적으로 변환해
    # `/panda_arm_controller/joint_trajectory` 로 내보낸다.
    #
    #   /ee_pose -> ee_twist_publisher -> /servo_node/delta_twist_cmds
    #            -> servo_node -> /panda_arm_controller/joint_trajectory
    #
    # 주의: 속도 명령 기반이라 **개루프** 다. 적분 드리프트가 누적되고 시작 pose 가
    # 다르면 전체가 오프셋되며, EE twist 는 6-DOF 라 7-DOF 여유자유도가 재현되지
    # 않는다. 자세한 근거는 docs/replay/replay_approaches.md 참고.
    # 정확한 재현이 목적이면 target_joint_cmds 경로를 쓴다.
    #
    # servo 는 `start_servo` 서비스 호출 전까지 입력을 무시하므로, 호출 주체가 없는
    # replay 스택에서는 servo_auto_start_node 를 함께 기동해야 한다.
    if arm_path == _ARM_PATH_EE_TWIST:
        arm_path_nodes.append(Node(
            package="robot_control",
            executable="ee_twist_node",
            name="ee_twist_publisher",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "source": "ee_pose",
                "ee_pose_topic": LaunchConfiguration("ee_twist_source_topic"),
                "twist_topic": LaunchConfiguration("ee_twist_output_topic"),
                "base_frame": LaunchConfiguration("base_frame"),
                "ee_frame": LaunchConfiguration("ee_frame"),
                "linear_gain": LaunchConfiguration("ee_twist_linear_gain"),
                "angular_gain": LaunchConfiguration("ee_twist_angular_gain"),
                "max_dt": LaunchConfiguration("ee_twist_max_dt"),
            }],
        ))
        arm_path_nodes.append(Node(
            package="robot_control",
            executable="servo_auto_start_node",
            name="servo_auto_start",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "servo_node_name": "/servo_node",
                "service_timeout": LaunchConfiguration("servo_start_timeout"),
            }],
        ))

    # panda_hand_controller 기동 완료 후 MoveIt/주변 노드와 rdfp 애플리케이션
    # 노드를 일괄 spawn 한다.
    controller_startup_handlers = create_controller_startup_handlers(
        ros2_control_node,
        joint_state_broadcaster_spawner,
        panda_arm_controller_spawner,
        panda_hand_controller_spawner,
        [
            move_group_node, servo_node, rviz_node,
            rdfp_image_viewer_node, gripper_node,
            *arm_path_nodes,
        ],
    )

    return [
        # --- YAML 기본값을 가진 argument 들 (config_file resolve 후 결정) ---
        *_declare_arguments(config),
        # --- 즉시 기동 노드 ---
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        # --- 순차 기동 핸들러 (controllers -> moveit + rdfp app nodes) ---
        *controller_startup_handlers,
    ]


def generate_launch_description() -> LaunchDescription:
    # `config_file` 만 declaration 시점에 노출하고, 그 값에 의존하는 YAML 로딩과
    # 나머지 argument / 노드 생성은 `OpaqueFunction` 안에서 수행한다. 이렇게 해야
    # top-level CLI 뿐 아니라 `IncludeLaunchDescription(..., launch_arguments=...)`
    # 경유 호출에서도 동일하게 `config_file` override 가 반영된다.
    #
    # `replay_arm_path` 도 YAML(`replay.arm_path`) 에서 기본값을 가져오므로 여기가
    # 아니라 `_declare_arguments()` 에서 선언한다. `_build_actions` 는 그 값을 즉시
    # 읽어야 하는데 선언이 아직 실행되지 않은 시점이므로, `perform()` 대신
    # `_resolve_arm_path()` 가 context 를 직접 조회한다.
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=_default_config_path(),
        description=(
            "Path to the replay_panda_mock YAML configuration file. "
            "기본값은 <rdfp share>/config/replay_panda_mock.yaml 이며, "
            "CLI 또는 IncludeLaunchDescription launch_arguments 로 "
            "'config_file:=<path>' 를 주면 덮어쓸 수 있다 "
            "($HOME 등 쉘 확장은 쉘에 맡긴다)."
        ),
    )
    return LaunchDescription([
        config_file_arg,
        OpaqueFunction(function=_build_actions),
    ])
