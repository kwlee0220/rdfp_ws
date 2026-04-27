"""SessionControlNode · ImageViewerNode · ImageRecorderNode 를 함께 기동하는 launch.

세션/에피소드 생명주기를 관리하는 SessionControlNode, 카메라 토픽을 화면에
표시하는 ImageViewerNode, 그리고 이미지를 MP4 로 녹화하는 ImageRecorderNode
를 하나의 launch 로 띄운다. 카메라 노드 자체는 본 launch 에서 기동하지
않으며 (별도 launch 또는 ``ros2 run`` 으로 기동), 공유되는 카메라 관련
argument 는 :mod:`camera_launch_helper` 에서 선언한 것을 재사용한다.

SessionControlNode
    - 노드 이름: ``session_control``
    - 제어/상태 조회 서비스는 ``~/`` prefix 로 선언되어 있어 노드
      네임스페이스가 자동으로 prepend 된다 (예:
      ``/session_control/start_session``).
    - 세션 상태 토픽은 전역 토픽 ``/session`` 으로 발행된다
      (다른 노드들과의 통합을 단순화하기 위함).
    - ``log_level:=<debug|info|warn|error|fatal>`` 로 본 노드에만 로그
      레벨을 적용할 수 있다 (전역 기본 레벨은 건드리지 않는다).

ImageViewerNode
    - 노드 이름: ``image_viewer_node``
    - ``image`` 를 ``camera_image_topic`` (기본 ``/camera/image_raw``) 으로
      remap 해 구독하므로 CameraNode 와 함께 쓰면 별도 설정 없이 미리보기가
      가능하다.
    - ``enable_image_viewer_node:=false`` 로 비활성화 가능.
    - 헤드리스 환경(DISPLAY 미설정 등)에서는 뷰어 기동이 실패하므로
      비활성화하여 사용한다.

ImageRecorderNode
    - 노드 이름: ``image_recorder`` (서비스 namespace: ``/image_recorder/...``)
    - ``image`` 를 ``camera_image_topic`` 으로 remap 해 구독한다.
    - 녹화 파라미터는 독립 argument 로 선언된다:
        * ``image_recorder_fps`` (기본 ``10``) — 카메라 FPS 와 불일치 시 프레임
          drop 이 생길 수 있으므로 ``camera_fps`` 에 맞추는 것을 권장.
        * ``resolution`` 은 ``camera_resolution`` 을 그대로 사용한다.
        * ``image_recorder_output_dir`` (기본 ``/tmp/recordings``) — MP4
          저장 경로.
        * ``image_recorder_auto_start:=true`` (기본값) 이면 기동 직후 즉시
          녹화를 시작한다.
    - **세션 토픽과 독립적으로 동작** 하며 ``~/start_session`` /
      ``~/stop_session`` 서비스로 녹화를 제어한다 (SessionControlNode 와
      연동되는 녹화가 필요하면 ``rdfp_image_recorder`` 사용을 검토).
    - ``enable_image_recorder_node:=false`` 로 비활성화 가능.
"""

import os
import sys

# ROS2 launch 러너는 본 파일을 단일 스크립트로 로드하므로 같은 디렉터리의
# sibling 모듈(`camera_launch_helper`) 을 패키지 import 로 가져올 수 없다. launch 디렉터리를
# sys.path 의 맨 앞에 추가하여 top-level 모듈처럼 import 한다 (sibling 우선 보장; append 로 바꾸지 말 것). setup.py 가 launch
# 디렉터리를 install/share/rdfp/launch 로 복사하므로 src / install 양쪽에서
# 동일하게 동작한다.
sys.path.insert(0, os.path.dirname(__file__))

from launch import LaunchDescription                                     # noqa: E402
from launch.actions import DeclareLaunchArgument                         # noqa: E402
from launch.conditions import IfCondition                                # noqa: E402
from launch.substitutions import LaunchConfiguration, TextSubstitution   # noqa: E402
from launch_ros.actions import Node                                      # noqa: E402

from camera_launch_helper import declare_camera_arguments                      # noqa: E402


def generate_launch_description() -> LaunchDescription:
    log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        choices=["debug", "info", "warn", "error", "fatal"],
        description="session_control_node 의 로그 레벨",
    )

    session_control_node = Node(
        package="rdfp",
        executable="session_control_node",
        name="session_control",
        output="screen",
        emulate_tty=True,
        # 노드 스코프 로그 레벨: "session_control:=<level>" 형태로 전달하여
        # 전역 기본 레벨을 건드리지 않고 본 노드에만 적용한다.
        ros_arguments=[
            "--log-level",
            [TextSubstitution(text="session_control:="), LaunchConfiguration("log_level")],
        ],
    )

    # 카메라 argument (토픽명, 해상도, FPS 등) 는 뷰어/레코더가 공유하므로 선언만
    # 재사용한다. 카메라 노드 자체는 본 launch 에서 기동하지 않는다.
    camera_arguments = declare_camera_arguments()

    # 이미지 뷰어 노드 (parent ImageViewerNode). 카메라가 발행하는 토픽을 그대로
    # 구독하도록 `image` -> camera_image_topic 으로 remap. 헤드리스 환경에서는
    # `enable_image_viewer_node:=false` 로 끄고 사용한다.
    enable_image_viewer_node = DeclareLaunchArgument(
        "enable_image_viewer_node",
        default_value="true",
        description="Whether to start image_viewer_node",
    )

    image_viewer_node = Node(
        package="rdfp",
        executable="image_viewer_node",
        name="image_viewer_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_image_viewer_node")),
        remappings=[
            ("image", LaunchConfiguration("camera_image_topic")),
        ],
    )

    # 이미지 녹화 노드 (service 기반, 세션 토픽과 독립). fps / resolution 은
    # 카메라 설정을 그대로 사용해 파라미터 불일치로 인한 프레임 drop 을 방지.
    enable_image_recorder_node = DeclareLaunchArgument(
        "enable_image_recorder_node",
        default_value="true",
        description="Whether to start image_recorder_node",
    )
    image_recorder_fps = DeclareLaunchArgument(
        "image_recorder_fps",
        default_value="10",
        description="Target FPS for image_recorder_node (should match camera_node fps to avoid frame drops)",
    )
    image_recorder_output_dir = DeclareLaunchArgument(
        "image_recorder_output_dir",
        default_value="/tmp/recordings",
        description="Output directory for image_recorder_node MP4 files",
    )
    image_recorder_auto_start = DeclareLaunchArgument(
        "image_recorder_auto_start",
        default_value="true",
        description="If true, image_recorder_node starts recording immediately at launch",
    )

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

    # servo_node 가 발행하는 JointTrajectory 의 마지막 point 를 뽑아 현재 시각을
    # header.stamp 로 채운 TargetJointStates 메시지로 변환해 `target_joint_states`
    # 토픽에 재발행한다. 입력 토픽은 remap 으로 /servo_node/joint_trajectory 에
    # 연결한다.
    target_joint_states_publisher = Node(
        package="rdfp",
        executable="target_joint_states_publisher",
        name="target_joint_states_publisher",
        output="screen",
        emulate_tty=True,
        remappings=[
            ("joint_trajectory", "/servo_node/joint_trajectory"),
        ],
    )

    return LaunchDescription(
        [
            # --- launch arguments ---
            log_level,
            *camera_arguments,
            enable_image_viewer_node,
            enable_image_recorder_node,
            image_recorder_output_dir,
            image_recorder_fps,
            image_recorder_auto_start,
            # --- 노드 ---
            session_control_node,
            image_viewer_node,
            image_recorder_node,
            target_joint_states_publisher,
        ]
    )
