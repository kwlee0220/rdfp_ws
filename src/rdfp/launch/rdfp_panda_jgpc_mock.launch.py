"""JointGroupPositionController 기반 Panda + MoveIt2 풀 스택 + rdfp 애플리케이션 launch.

:mod:`rdfp_panda_mock.launch` 와 동일한 구성을 띄우되, arm 컨트롤러를
``joint_trajectory_controller/JointTrajectoryController`` 대신
``position_controllers/JointGroupPositionController`` (JGPC) 로 교체한다.
:mod:`panda_jgpc_mock.launch` 에 rdfp 애플리케이션 노드를 더한 것과 같다.

관계
----

=========================  =========================  ====================
스택 구성                   기본 (JTC)                  JGPC
=========================  =========================  ====================
Panda + MoveIt2 만          panda_mock                 panda_jgpc_mock
위 + rdfp 애플리케이션       rdfp_panda_mock            **본 파일**
=========================  =========================  ====================

기동되는 노드 단위

- ``ros2_control``:
    - /controller_manager
    - /joint_state_broadcaster
    - /panda_arm_controller  (JointGroupPositionController)
    - /panda_hand_controller
    - /static_transform_publisher
    - /robot_state_publisher
- ``moveit``:
    - /move_group
    - /servo_node
    - /ee_pose_publisher
- ``rviz2``: /rviz2
- ``camera``: /camera_node
- ``gripper``: /gripper_control
- ``scene``: /mock_scene_state (``enable_scene:=false`` 로 비활성화 가능)
- ``rdfp 애플리케이션``:
    - /session_control
    - /rdfp_image_viewer_node
    - /image_recorder
    - /target_joint_cmds_publisher

JGPC 교체에 따른 차이
---------------------

1. **controller 설정 YAML**
   ``moveit_resources_panda_moveit_config`` 의 ``ros2_controllers.yaml`` 대신
   ``<rdfp share>/config/panda_jgpc_ros2_controllers.yaml`` 을 사용한다.
   controller *이름* 은 ``panda_arm_controller`` 로 동일하게 두어 spawner helper
   와 순차 기동 핸들러를 그대로 재사용한다.

2. **arm 명령 인터페이스**
   ``/panda_arm_controller/joint_trajectory`` (JointTrajectory) 가 사라지고
   ``/panda_arm_controller/commands`` (std_msgs/Float64MultiArray) 가 된다.
   메시지에 joint 이름이 없으므로 배열 순서는 controller YAML 의 ``joints``
   순서(panda_joint1..7) 를 따라야 한다.

3. **servo 파라미터 override**
   ``moveit_servo`` 의 출력 타입을 ``std_msgs/Float64MultiArray`` 로 바꾸고
   출력 토픽을 위 commands 토픽으로 돌린다. 이때 ``publish_joint_positions`` 와
   ``publish_joint_velocities`` 를 **동시에 true 로 두면 servo 가 파라미터 검증
   에서 실패해 기동하지 않으므로**, velocity 발행을 끈다.

4. **move_group 실행 경로 없음**
   ``moveit_simple_controller_manager`` 는 ``FollowJointTrajectory`` /
   ``GripperCommand`` 타입만 다룰 수 있는데 JGPC 는 어느 쪽도 제공하지 않는다.
   따라서 arm 에 대한 **plan & execute 는 동작하지 않는다** (planning / IK /
   planning scene / RViz 표시는 정상). arm 제어는 servo, commands 토픽 직접
   발행, 또는 ``create_move_group_client()`` 가 돌려주는 ``MoveGroupJgpcClient``
   로 수행한다 — ``docs/moveit/MoveGroupJgpcClient_UserGuide.md`` 참고.
   gripper 는 ``panda_hand_controller`` 가 그대로 GripperActionController 이므로
   기존과 동일하게 동작한다.

설정 파일
---------

:mod:`rdfp_panda_mock.launch` 와 **동일한 YAML** 을 사용한다.

- 기본 경로: ``<rdfp share>/config/panda_robot.yaml``
- ``config_file:=<path>`` launch argument 로 다른 YAML 을 지정할 수 있다.
  ``$HOME`` / ``~`` 같은 경로 확장은 쉘에 맡긴다.
- CLI 에서 ``arg:=value`` 로 개별 argument 를 덮어쓰는 것은 그대로 동작한다.
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
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

from robot_control.launch_helpers.camera import create_camera_node
from robot_control.launch_helpers.controller import (
    create_joint_state_broadcaster_spawner,
    create_panda_arm_controller_spawner,
    create_panda_hand_controller_spawner,
    create_ros2_control_node,
)
from robot_control.launch_helpers.controller_startup import create_controller_startup_handlers
from robot_control.launch_helpers.ee_pose import create_ee_pose_node
from robot_control.launch_helpers.gripper import create_gripper_node
from robot_control.launch_helpers.image_pipeline import (
    declare_config_file_argument as declare_image_pipeline_config_file_argument,
    declare_image_pipeline_arguments,
    load_config as load_image_pipeline_config,
)
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
from robot_control.launch_helpers.scene import create_mock_scene_node, declare_scene_arguments

# YAML 설정 파일의 기본 경로. rdfp_panda_mock 과 동일한 파일을 공유한다.
# setup.py 가 ``config/*`` 를 ``share/rdfp/config/`` 로 설치하므로 share 에서 읽는다.
DEFAULT_CONFIG_RELPATH = os.path.join("config", "panda_robot.yaml")

# JGPC 용 controller 설정 YAML 의 상대 경로.
CONTROLLERS_CONFIG_RELPATH = os.path.join("config", "panda_jgpc_ros2_controllers.yaml")

# JGPC 가 구독하는 명령 토픽. forward_command_controller 는 ``~/commands`` 를
# 사용하므로 controller 이름 기준으로 아래 경로가 된다.
ARM_COMMAND_TOPIC = "/panda_arm_controller/commands"

# target_joint_cmds_publisher 가 joint 이름(`joints` 파라미터)을 조회할 컨트롤러.
ARM_CONTROLLER_NODE_NAME = "/panda_arm_controller"


def _default_config_path() -> str:
    """패키지 share 경로의 기본 YAML 위치를 반환한다."""
    return os.path.join(get_package_share_directory("rdfp"), DEFAULT_CONFIG_RELPATH)


def _controllers_config_path() -> str:
    """패키지 share 경로의 JGPC controller 설정 YAML 위치를 반환한다."""
    return os.path.join(get_package_share_directory("robot_control"), CONTROLLERS_CONFIG_RELPATH)


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


def _override_servo_params_for_jgpc(servo_params: dict[str, Any]) -> dict[str, Any]:
    """servo 출력을 JGPC 의 Float64MultiArray 명령 토픽으로 돌린다.

    ``publish_joint_velocities`` 를 끄는 것은 선택이 아니라 필수다. servo 의
    파라미터 검증은 ``command_out_type`` 이 ``std_msgs/Float64MultiArray`` 인데
    positions 와 velocities 를 모두 발행하도록 설정되어 있으면 실패를 반환하고,
    그 경우 servo 노드가 기동하지 못한다.
    """
    params = servo_params["moveit_servo"]
    params["command_out_type"] = "std_msgs/Float64MultiArray"
    params["command_out_topic"] = ARM_COMMAND_TOPIC
    params["publish_joint_positions"] = True
    params["publish_joint_velocities"] = False
    params["publish_joint_accelerations"] = False
    return servo_params


def _declare_arguments(config: dict[str, Any]) -> list[DeclareLaunchArgument]:
    """YAML 설정값을 default 로 사용하는 DeclareLaunchArgument 목록을 생성한다.

    helper 모듈의 ``declare_*_arguments()`` 는 기본값을 하드코딩하므로, YAML
    기반 기본값을 적용하려면 본 launch 파일에서 직접 선언해야 한다. helper 의
    ``create_*_node()`` 는 ``LaunchConfiguration(<name>)`` 으로 값을 참조하므로
    여기서 선언한 argument 이름만 일치시키면 그대로 동작한다.
    """
    rc = config["ros2_control"]
    ee = config["ee_pose"]

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
        DeclareLaunchArgument(
            "publish_rate",
            default_value=_as_launch_str(ee["publish_rate"]),
            description="EE pose publish rate in Hz",
        ),
    ]


def _build_actions(context: LaunchContext) -> list:
    """`config_file` 이 resolve 된 뒤 YAML 을 로드해 나머지 argument/노드를 구성한다.

    `OpaqueFunction` 이 실행되는 시점에는 `LaunchConfiguration` 이 이미
    resolve 가능하므로, top-level CLI 의 ``config_file:=...`` 든
    ``IncludeLaunchDescription(..., launch_arguments=...)`` 든 동일하게 반영된다.
    """
    config_path = LaunchConfiguration("config_file").perform(context)
    config = _load_config(config_path)

    # 이미지 파이프라인(camera / image_viewer / image_recorder) 설정은 별도 YAML 에서
    # 온다 — 카메라를 띄우는 launch 가 넷이라 기본값을 한 곳으로 모았다.
    image_config_path = LaunchConfiguration("image_pipeline_config_file").perform(context)
    image_config = load_image_pipeline_config(image_config_path)

    moveit_config = build_moveit_config()
    servo_params = _override_servo_params_for_jgpc(build_servo_params())

    # --- ros2_control 스택 ---
    # 기본 moveit config 패키지의 ros2_controllers.yaml 대신 JGPC 설정을 넘긴다.
    static_tf = create_static_tf_node()
    robot_state_publisher = create_robot_state_publisher(moveit_config)
    ros2_control_node = create_ros2_control_node(moveit_config, MOVEIT_CONFIGS_PACKAGE_NAME,
                                                 controllers_file=_controllers_config_path())
    joint_state_broadcaster_spawner = create_joint_state_broadcaster_spawner()
    panda_arm_controller_spawner = create_panda_arm_controller_spawner()
    panda_hand_controller_spawner = create_panda_hand_controller_spawner()

    # --- MoveIt / RViz / 주변 노드 ---
    move_group_node = create_move_group_node(moveit_config)
    servo_node = create_servo_node(moveit_config, servo_params)
    rviz_node = create_rviz_node(moveit_config)
    camera_node = create_camera_node()
    ee_pose_node = create_ee_pose_node()
    gripper_node = create_gripper_node()
    # scene 노드는 move_group 의 planning scene 에 의존하지만 생성자에서 서비스를
    # 기다리지 않으므로 같은 그룹에서 동시에 spawn 해도 안전하다.
    scene_node = create_mock_scene_node()

    # --- rdfp 애플리케이션: SessionControlNode ---
    # 로그 레벨은 노드 스코프로만 적용하여 전역 기본 레벨을 건드리지 않는다.
    session_control_node = Node(
        package="rdfp",
        executable="session_control_node",
        name="session_control",
        output="screen",
        emulate_tty=True,
        ros_arguments=[
            "--log-level",
            [TextSubstitution(text="session_control:="), LaunchConfiguration("log_level")],
        ],
    )

    # --- rdfp 애플리케이션: RdfpImageViewerNode ---
    # session 토픽을 구독하여 수신 프레임 좌상단에 상태 오버레이를 덧입혀 표시한다.
    rdfp_image_viewer_node = Node(
        package="rdfp",
        executable="rdfp_image_viewer_node",
        name="rdfp_image_viewer_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_image_viewer_node")),
        remappings=[
            ("image", LaunchConfiguration("camera_image_topic")),
        ],
    )

    # --- rdfp 애플리케이션: ImageRecorderNode ---
    # service 기반, 세션 토픽과 독립적으로 동작한다. fps / resolution 은 카메라
    # 설정을 그대로 사용해 파라미터 불일치로 인한 프레임 drop 을 방지한다.
    image_recorder_node = Node(
        package="rdfp",
        executable="image_recorder_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_image_recorder_node")),
        parameters=[{
            "output_dir": LaunchConfiguration("image_recorder_output_dir"),
            "fps": LaunchConfiguration("image_recorder_fps"),
            "resolution": LaunchConfiguration("camera_resolution"),
            "auto_start": LaunchConfiguration("image_recorder_auto_start"),
        }],
        remappings=[
            ("image", LaunchConfiguration("camera_image_topic")),
        ],
    )

    # --- rdfp 애플리케이션: TargetJointCmdsPublisher ---
    # JGPC 스택의 arm 명령은 `/panda_arm_controller/commands`
    # (std_msgs/Float64MultiArray) 로 나간다. 이름도 header 도 없는 순수 배열이므로
    # 컨트롤러의 `joints` 파라미터를 조회해 이름을 붙이고, 수신 시각을 stamp 로
    # 채워 `sensor_msgs/JointState` 로 변환해 `target_joint_cmds` 에 재발행한다.
    #
    # JTC 스택(rdfp_panda_mock)은 같은 노드를 source=joint_trajectory 로 띄운다.
    # 두 스택이 같은 토픽 이름과 같은 메시지 타입을 쓰므로 데이터셋에서는
    # `joint_states` 테이블에 topic_id 로만 구분되어 함께 적재된다.
    #
    # rdfp_msgs/TargetJointStates 는 JGPC 스택에서 만들 수 없다 — 그 변환의 입력인
    # JointTrajectory 발행자가 이 스택에 존재하지 않기 때문이다.
    target_joint_cmds_publisher = Node(
        package="robot_control",
        executable="target_joint_cmds_publisher",
        name="target_joint_cmds_publisher",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "source": "float64_multi_array",
            "controller_node_name": ARM_CONTROLLER_NODE_NAME,
        }],
        remappings=[
            ("commands", ARM_COMMAND_TOPIC),
        ],
    )

    # panda_hand_controller 기동 완료 후 MoveIt/주변 노드와 rdfp 애플리케이션
    # 노드를 일괄 spawn 한다.
    controller_startup_handlers = create_controller_startup_handlers(
        ros2_control_node,
        joint_state_broadcaster_spawner,
        panda_arm_controller_spawner,
        panda_hand_controller_spawner,
        [
            move_group_node, servo_node, rviz_node, camera_node, ee_pose_node,
            gripper_node, scene_node,
            session_control_node, rdfp_image_viewer_node, image_recorder_node,
            target_joint_cmds_publisher,
        ],
    )

    return [
        # --- YAML 기본값을 가진 argument 들 (config_file resolve 후 결정) ---
        *_declare_arguments(config),
        *declare_image_pipeline_arguments(image_config),
        # scene 노드는 YAML 블록 없이 helper 의 하드코딩 기본값을 쓴다 — 이유는
        # rdfp_panda_mock.launch.py 의 같은 위치 주석 참고.
        *declare_scene_arguments(),
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
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=_default_config_path(),
        description=(
            "Path to the rdfp_panda_mock YAML configuration file. "
            "기본값은 <rdfp share>/config/panda_robot.yaml 이며 "
            "rdfp_panda_mock.launch.py 와 동일한 파일을 공유한다. "
            "CLI 또는 IncludeLaunchDescription launch_arguments 로 "
            "'config_file:=<path>' 를 주면 덮어쓸 수 있다 "
            "($HOME 등 쉘 확장은 쉘에 맡긴다)."
        ),
    )
    return LaunchDescription([
        config_file_arg,
        declare_image_pipeline_config_file_argument('image_pipeline_config_file'),
        OpaqueFunction(function=_build_actions),
    ])
