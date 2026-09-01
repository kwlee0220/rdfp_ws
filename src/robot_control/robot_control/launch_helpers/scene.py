"""scene 상태 노드(백엔드별 scene_state_node) 및 파라미터 설정.

scene 노드는 백엔드와 **1:1 로 묶인다** — mock 스택은 `mock_scene_state_node`,
Gazebo 스택은 `gazebo_scene_state_node`(미구현) 를 띄워야 한다. 어느 스택을
기동하느냐가 곧 어느 어댑터가 필요한지를 결정하므로, 그 짝을 사용자에게 맡기지
않고 launch 파일이 백엔드별 팩토리를 골라 쓴다. 팩토리는 백엔드마다 하나씩
추가하되 argument 이름(`enable_scene_node`) 은 공유한다 — 그래야 스택을 바꿔도
같은 인자로 켜고 끌 수 있다.

기동 순서
---------

scene 노드는 `move_group` 이 제공하는 `/monitored_planning_scene` 과
`/apply_planning_scene` 에 의존하지만, 노드 생성자가 `wait_for_service` 를
호출하지 않으므로 **`move_group` 과 같은 그룹에서 동시에 spawn 해도 안전하다**.
초기 몇 초 동안은 diff 를 받지 못해 빈 목록을 발행하다가 채워진다
(`servo_auto_start_node` 처럼 별도의 대기 노드가 필요한 경우가 아니다).

argument 전제
-------------

`base_frame` 은 이 helper 가 선언하지 않고 **이미 선언되어 있다고 가정한다**
(`ee_pose_launch_helper.declare_ee_pose_arguments()` 또는 launch 파일의 YAML
기반 선언). scene 노드가 발행하는 물체 pose 의 기준 프레임은 EE pose 의 기준
프레임과 같은 로봇 base 여야 하므로, 별도 인자로 열면 두 값이 어긋날 때 조용히
틀린 좌표가 나간다. `camera_launch_helper` 가 `image_pipeline_launch_helper`
가 선언한 argument 를 참조하는 것과 같은 구조다.
"""

from __future__ import annotations

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def declare_scene_arguments() -> list[DeclareLaunchArgument]:
    """scene 상태 노드 관련 launch argument 들을 선언한다.

    `publish_rate` 라는 이름은 쓰지 않는다 — `ee_pose_launch_helper` 가 이미 그
    이름을 50 Hz 기본값으로 선언하고 있어 충돌한다.
    """
    return [
        DeclareLaunchArgument(
            "enable_scene_node",
            default_value="true",
            description=(
                "Launch the backend scene state node "
                "(/scene/objects 발행, /scene/reset 서비스)"
            ),
        ),
        DeclareLaunchArgument(
            "scene_publish_rate",
            default_value="2.0",
            description="Scene object publish rate in Hz",
        ),
    ]


def create_mock_scene_node() -> Node:
    """mock 백엔드(MoveIt planning scene) 의 scene 상태 노드를 생성한다.

    Gazebo / Isaac 백엔드를 추가할 때는 이 함수를 고치지 말고
    `create_gazebo_scene_node()` 같은 형제 팩토리를 새로 만든다 — executable 만
    다르고 argument 계약은 같다.
    """
    return Node(
        package="robot_control",
        executable="mock_scene_state_node",
        name="mock_scene_state",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_scene_node")),
        parameters=[
            {
                "base_frame": LaunchConfiguration("base_frame"),
                "publish_rate": LaunchConfiguration("scene_publish_rate"),
            }
        ],
    )
