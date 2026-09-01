"""JointGroupPositionController 기반 Panda + MoveIt2 launch.

:mod:`panda_mock.launch` 와 동일한 스택을 띄우되, arm 컨트롤러를
``joint_trajectory_controller/JointTrajectoryController`` 대신
``position_controllers/JointGroupPositionController`` (JGPC) 로 교체한다.

JGPC 는 ``forward_command_controller`` 기반이라 보간 없이 마지막으로 받은 관절
위치를 그대로 hardware 에 write 한다. 궤적/시간 정보가 없으므로 servo 처럼
고빈도로 목표 위치를 스트리밍하는 용도에 맞다.

교체에 따른 차이
----------------

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
   planning scene / RViz 표시는 정상). arm 제어는 servo 또는
   ``/panda_arm_controller/commands`` 직접 발행으로 수행한다.
   gripper 는 ``panda_hand_controller`` 가 그대로 GripperActionController 이므로
   기존과 동일하게 동작한다.

기동되는 노드 단위는 :mod:`panda_mock.launch` 와 같다.

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
- ``scene``: /mock_scene_state (``enable_scene_node:=false`` 로 비활성화 가능)
"""

from __future__ import annotations

from typing import Any

import os


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from robot_control.launch_helpers.camera import create_camera_node, declare_camera_arguments
from robot_control.launch_helpers.controller import (
    create_joint_state_broadcaster_spawner,
    create_panda_arm_controller_spawner,
    create_panda_hand_controller_spawner,
    create_ros2_control_node,
)
from robot_control.launch_helpers.controller_startup import create_controller_startup_handlers
from robot_control.launch_helpers.ee_pose import create_ee_pose_node, declare_ee_pose_arguments
from robot_control.launch_helpers.gripper import create_gripper_control_node
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
from robot_control.launch_helpers.scene import create_mock_scene_node, declare_scene_arguments

# JGPC 용 controller 설정 YAML 의 상대 경로. setup.py 가 ``config/*`` 를
# ``share/rdfp/config/`` 로 설치하므로 package share 에서 읽는다.
CONTROLLERS_CONFIG_RELPATH = os.path.join("config", "panda_jgpc_ros2_controllers.yaml")

# JGPC 가 구독하는 명령 토픽. forward_command_controller 는 ``~/commands`` 를
# 사용하므로 controller 이름 기준으로 아래 경로가 된다.
ARM_COMMAND_TOPIC = "/panda_arm_controller/commands"


def _controllers_config_path() -> str:
    """패키지 share 경로의 JGPC controller 설정 YAML 위치를 반환한다."""
    return os.path.join(get_package_share_directory("robot_control"), CONTROLLERS_CONFIG_RELPATH)


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


def generate_launch_description() -> LaunchDescription:
    moveit_config = build_moveit_config()
    servo_params = _override_servo_params_for_jgpc(build_servo_params())

    static_tf = create_static_tf_node()
    robot_state_publisher = create_robot_state_publisher(moveit_config)
    # 기본 moveit config 패키지의 ros2_controllers.yaml 대신 JGPC 설정을 넘긴다.
    ros2_control_node = create_ros2_control_node(moveit_config, MOVEIT_CONFIGS_PACKAGE_NAME,
                                                 controllers_file=_controllers_config_path())
    joint_state_broadcaster_spawner = create_joint_state_broadcaster_spawner()
    panda_arm_controller_spawner = create_panda_arm_controller_spawner()
    panda_hand_controller_spawner = create_panda_hand_controller_spawner()

    move_group_node = create_move_group_node(moveit_config)
    servo_node = create_servo_node(moveit_config, servo_params)
    rviz_node = create_rviz_node(moveit_config)
    camera_node = create_camera_node()
    ee_pose_node = create_ee_pose_node()
    gripper_control_node = create_gripper_control_node()
    # scene 노드는 move_group 의 planning scene 에 의존하지만 생성자에서 서비스를
    # 기다리지 않으므로 같은 그룹에서 동시에 spawn 해도 안전하다.
    scene_node = create_mock_scene_node()

    controller_startup_handlers = create_controller_startup_handlers(
        ros2_control_node,
        joint_state_broadcaster_spawner,
        panda_arm_controller_spawner,
        panda_hand_controller_spawner,
        [move_group_node, servo_node, rviz_node, camera_node, ee_pose_node,
         gripper_control_node, scene_node],
    )

    return LaunchDescription(
        [
            declare_ros2_control_hardware_type_argument(),
            declare_log_level_argument(),
            *declare_ee_pose_arguments(),
            *declare_camera_arguments(),
            *declare_scene_arguments(),
            static_tf,
            robot_state_publisher,
            ros2_control_node,
            *controller_startup_handlers,
        ]
    )
