"""session_control_node, camera_node, image_viewer_node 를 함께 기동하는 launch.

세션/에피소드 생명주기를 관리하는 SessionControlNode, 영상 입력을 담당하는
CameraNode, 그리고 발행된 이미지를 화면에 표시하는 ImageViewerNode 를 하나의
launch 로 띄운다.

SessionControlNode
    - 노드 이름: ``session_control``
    - 제어/상태 조회 서비스는 ``~/`` prefix 로 선언되어 있어 노드
      네임스페이스가 자동으로 prepend 된다 (예:
      ``/session_control/start_session``).
    - 세션 상태 토픽은 전역 토픽 ``/session`` 으로 발행된다
      (다른 노드들과의 통합을 단순화하기 위함).

RdfpCameraNode
    - 노드 이름: ``rdfp_camera_node``
    - ``CameraNode`` 와 달리 세션 토픽(`/session`) 의 ``IN_EPISODE`` 상태
      동안에만 이미지를 발행한다 (재연결 로직 내장).
    - 토픽은 모두 private (``~/image_raw``, ``~/camera_info``, ``~/camera_status``)
      으로 선언되어 resolve 시 ``/rdfp_camera_node/...`` 로 매핑된다. launch
      의 remap 으로 ``/camera/image_raw`` 등 외부 토픽명에 연결한다.
    - ``enable_camera_node:=false`` 로 비활성화 가능.
    - 세부 설정(카메라 ID, FPS, 해상도, frame_id 등)은
      :mod:`camera_launch_helper` 헬퍼에서 선언한 launch argument 를 재사용한다
      (``camera_compress_image`` 는 ``RdfpCameraNode`` 가 지원하지 않으므로
      지정해도 무시된다).

RdfpImageViewerNode
    - 노드 이름: ``rdfp_image_viewer_node``
    - ``ImageViewerNode`` 를 상속하여 세션 토픽(`/session`) 상태를 프레임
      좌상단에 오버레이 (예: ``pick_apple (Recording)``).
    - ``camera_image_topic`` (기본 ``/camera/image_raw``) 을 구독하므로 카메라
      노드와 함께 쓰면 별도 remap 없이 바로 미리보기가 가능하다. session 토픽은
      기본 ``/session`` 이라 별도 remap 불필요.
    - ``enable_image_viewer_node:=false`` 로 비활성화 가능.
    - 헤드리스 환경(DISPLAY 미설정 등)에서는 뷰어 기동이 실패하므로
      비활성화하여 사용한다.

RdfpImageRecorderNode
    - 노드 이름: ``rdfp_image_recorder_node``
    - ``camera_image_topic`` 을 구독하며, fps / resolution 은 카메라 argument 를
      그대로 사용한다.
    - **세션 토픽 기반 자동 제어**: ``/session`` 의 ``IN_EPISODE`` 진입 시
      자동 녹화 시작, ``IN_SESSION`` 복귀 시 자동 중지. 별도 서비스 호출 필요 없음.
    - 타임스탬프 기반 녹화 경계 판정 + ``pending_image_queue`` 로 start/stop
      경계 근처 프레임을 정확히 처리.
    - ``enable_image_recorder_node:=false`` 로 비활성화 가능.
    - ``image_recorder_output_dir`` 로 저장 경로 지정 (기본 ``/tmp/recordings``).
    - ``image_recorder_fps`` 는 카메라 fps 와 반드시 일치시킬 것 (CFR 기반).
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

from camera_launch_helper import declare_camera_arguments                # noqa: E402


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

    # 카메라 노드 (RdfpCameraNode): 세션 토픽(/session)의 IN_EPISODE 구간에만
    # 이미지를 발행. launch/camera_launch_helper.py 의 argument 선언은 그대로 재사용한다
    # (compress_image 는 RdfpCameraNode 가 지원하지 않아 사용하지 않음).
    # `enable_camera_node:=false` 로 비활성화 가능.
    camera_arguments = declare_camera_arguments()
    camera_node = Node(
        package="rdfp",
        executable="rdfp_camera_node",
        name="rdfp_camera_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_camera_node")),
        parameters=[{
            "camera_id": LaunchConfiguration("camera_id"),
            "fps": LaunchConfiguration("camera_fps"),
            "resolution": LaunchConfiguration("camera_resolution"),
            "frame_id": LaunchConfiguration("camera_frame_id"),
        }],
        remappings=[
            ("~/image_raw", LaunchConfiguration("camera_image_topic")),
            ("~/camera_info", LaunchConfiguration("camera_info_topic")),
            ("~/camera_status", LaunchConfiguration("camera_status_topic")),
        ],
    )

    # 이미지 뷰어 노드 (RdfpImageViewerNode). 세션 토픽 상태를 프레임 좌상단에
    # 오버레이한다. 카메라가 발행하는 토픽을 그대로 구독하도록 `image` ->
    # camera_image_topic 으로 remap. session 토픽은 기본 /session 이므로 remap
    # 불필요. 헤드리스 환경에서는 `enable_image_viewer_node:=false` 로 끄고 사용.
    enable_image_viewer_node = DeclareLaunchArgument(
        "enable_image_viewer_node",
        default_value="true",
        description="Whether to start rdfp_image_viewer_node",
    )

    image_viewer_node = Node(
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

    # 이미지 녹화 노드 (RdfpImageRecorderNode): 세션 토픽(/session) 기반 자동
    # 제어. fps / resolution 은 카메라 설정을 그대로 사용해 파라미터 불일치로
    # 인한 프레임 drop 을 방지. session 토픽은 기본 /session 이라 remap 불필요.
    enable_image_recorder_node = DeclareLaunchArgument(
        "enable_image_recorder_node",
        default_value="true",
        description="Whether to start rdfp_image_recorder_node",
    )
    image_recorder_fps = DeclareLaunchArgument(
        "image_recorder_fps",
        default_value="10",
        description="Target FPS for rdfp_image_recorder_node",
    )
    image_recorder_output_dir = DeclareLaunchArgument(
        "image_recorder_output_dir",
        default_value="/tmp/recordings",
        description="Output directory for rdfp_image_recorder_node MP4 files",
    )

    image_recorder_node = Node(
        package="rdfp",
        executable="rdfp_image_recorder",
        name="rdfp_image_recorder_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_image_recorder_node")),
        parameters=[{
            "output_dir": LaunchConfiguration("image_recorder_output_dir"),
            "fps": LaunchConfiguration("image_recorder_fps"),
            "resolution": LaunchConfiguration("camera_resolution"),
        }],
        remappings=[
            ("image", LaunchConfiguration("camera_image_topic")),
        ],
    )

    # servo_node 가 발행하는 JointTrajectory 의 header.stamp 를 현재 시각으로
    # 갱신하여 /relay/timed_joint_trajectory 로 재발행한다. 입력 토픽은 remap 으로
    # /servo_node/joint_trajectory 에 연결한다.
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
            # --- 노드 ---
            session_control_node,
            camera_node,
            image_viewer_node,
            image_recorder_node,
            target_joint_states_publisher,
        ]
    )
