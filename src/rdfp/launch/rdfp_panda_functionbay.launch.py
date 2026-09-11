"""펑션베이 백엔드 + rdfp 수집 노드까지 함께 기동하는 풀 스택 launch.

:mod:`panda_functionbay.launch` (시뮬레이터 연동 + MoveIt) 를 그대로 include 하고 그
위에 수집 계층을 얹는다. ``panda_mock`` ↔ ``rdfp_panda_mock`` 관계의 펑션베이 판이다.

**시뮬레이터(Unity + ros_tcp_endpoint)가 먼저 떠 있어야 한다.** 백엔드의
``readiness_gate`` 가 ``/output/panda_joint`` 첫 메시지를 기다리며, 오지 않으면
launch 전체가 실패 종료한다.

카메라 설정의 출처
------------------
**토픽 이름은 백엔드 프로파일에서 온다** (``config/backends/functionbay.yaml`` 의
``camera`` 블록). 제어 계열 ``panda_functionbay`` 도 같은 곳을 읽으므로 두 계층이 같은
이름을 쓴다 — 여기 되적었을 때는 갈릴 수 있었다.

**해상도·fps 만 여기 상수로 남는다.** Isaac 은 ``isaac_scene.json`` 의 ``camera`` 블록을
시뮬레이터 쪽 ``setup_graph.py`` 와 나눠 쓰지만 펑션베이에는 대응하는 파일이 없다 —
카메라 설정이 Unity 씬 안에 있어 우리가 읽을 수 없다. 읽을 수 없는 값을 프로파일에
적으면 실측과 갈려도 아무도 모르므로, 실측값임이 드러나게 여기 둔다.

⚠️ **레코더는 CFR 이라 선언한 fps 가 곧 영상의 시간축이 된다.** 소스 주기와 다르면
재생 속도가 어긋나며, 파이프라인에 데시메이션 로직이 없어 자동으로 맞춰지지 않는다.

    2026-09-01 실측: 시뮬레이터 카메라 = 9.606 Hz (관절 보고 주기와 무관)
    목표          : 5 Hz

즉 **지금 이 launch 를 그대로 쓰면 영상이 약 1.9배 빠르게 재생된다.** 5 Hz 가 되려면
둘 중 하나가 선행되어야 한다.

  1. 시뮬레이터 카메라를 5 Hz 로 설정 (벤더 요청 — 설정 가능 여부 미확인)
  2. ``/camera_image`` 를 5 Hz 로 솎는 데시메이션 노드

목표값을 기본으로 둔 이유는, 소스가 맞춰지는 순간 바로 정합하고 그 전까지는 위
경고가 이 파일에 남아 있게 하기 위해서다. 소스를 바꾸지 않은 채 쓰려면
``image_recorder_fps:=10`` 으로 실측값에 맞춘다.

arm 명령 채널
-------------
펑션베이는 ``/input/panda_joint`` 로 ``sensor_msgs/JointState`` 를 받는다. 그래서
``target_joint_cmds_publisher`` 의 ``source`` 는 ``joint_state`` 이고, 이 경로는
**컨트롤러의 ``joints`` 파라미터를 조회하지 않는다** — 메시지에 이름이 이미 들어 있다.
ros2_control 컨트롤러가 없는 이 스택에서 중요한 성질이다.

텔레오퍼레이션
--------------
`teleop_keyboard` 는 이 launch 가 띄우지 않는다(대화형이라 별도 터미널이 필요하다).
띄울 때는 **arm 명령 채널을 넘겨야 `/`(ready 이동)가 동작한다** — 기본 `auto` 판별은
`/panda_arm_controller/commands` 의 유무만 보므로 이 스택을 JTC 로 오판하고, 그러면
계획만 되고 실행이 일어나지 않는다.

    ros2 run rdfp teleop_keyboard --ros-args -p backend:=functionbay

사용 예
-------
    ros2 launch rdfp rdfp_panda_functionbay.launch.py
    ros2 launch rdfp rdfp_panda_functionbay.launch.py image_recorder_fps:=10
    # 녹화는 `/session` 이 IN_EPISODE 일 때 자동으로 돈다 — 시작/정지 인자가 없다.
    ros2 service call /session_control/start_session std_srvs/srv/Trigger
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PythonExpression, TextSubstitution)
from launch_ros.actions import Node

from robot_control.backends import get_backend
from robot_control.launch_helpers.camera import (
    create_raw_image_source_node,
    declare_camera_enable_argument,
    declare_camera_republish_arguments,
    warn_if_raw_consumer_has_no_source,
)

# **토픽 이름은 백엔드 프로파일이 갖는다** (`robot_control/config/backends/functionbay.yaml`).
# 여기 손으로 되적으면 제어 계열 `panda_functionbay` 와 갈리고, 그 어긋남은 에러가 아니라
# "영상이 안 담긴다" / "명령이 안 먹는다" 로 나타난다 — 되적힌 값 옆에 "같아야 한다" 는
# 주석을 다는 것이 곧 갈릴 수 있다는 뜻이었다.
_BACKEND = get_backend("functionbay")

# **뷰어만** 구독하는 raw 이미지 토픽 (레코더는 압축을 직접 받는다).
#
# 시뮬레이터는 2026-09-08 새 빌드부터 `/camera_image/compressed`
# (`sensor_msgs/CompressedImage`) 로 발행한다.
#
# **레코더는 그것을 그대로 받고**(`input_format: mjpeg`), **뷰어만** raw 를 요구하므로
# `camera_republish` 가 뷰어와 짝으로 뜬다. 되살리는 순간 압축으로 아낀 대역폭이
# 돌아오므로(0.27 → 9.2 MB/s, 36배) raw 를 항상 만들지는 않는다.
_CAMERA_IMAGE_TOPIC = _BACKEND.value("camera", "image_topic")
# 시뮬레이터가 발행하는 압축 이미지 토픽.
_CAMERA_COMPRESSED_TOPIC = _BACKEND.value("camera", "compressed_topic")
# 목표 주기. 위 docstring 의 경고를 읽는다 — 실측 소스는 아직 9.606 Hz 다.
#
# **해상도·fps 는 프로파일에 없다** — 시뮬레이터 카메라 설정이 Unity 씬 안에 있어
# 우리가 읽을 수 없고, 프로파일에 적으면 실측과 갈려도 아무도 모른다 (레코더는 CFR 이라
# 그 어긋남이 곧 영상 시간축의 오차가 된다). 그래서 여기 실측값으로 둔다.
_CAMERA_FPS = "5"
# 시뮬레이터 카메라 해상도 (2026-09-01 실측).
_CAMERA_RESOLUTION = "640x480"
# 펑션베이가 JointState 명령을 받는 토픽 — `arm_command` 블록에서 온다.
_ARM_COMMAND_TOPIC = _BACKEND.arm_command_topic


def _included_backend() -> IncludeLaunchDescription:
    """백엔드 launch 를 include 한다.

    ⚠️ **여기 넘기는 `launch_arguments` 는 부모 스코프로 샌다.**
    `IncludeLaunchDescription.visit()` 은 인자를 `SetLaunchConfiguration` 액션으로 돌려주고
    launch 서비스가 그것을 **같은 스코프에서** 실행하므로, include **뒤에** 오는 부모의
    노드들이 바뀐 값을 본다. 그래서 조건을 가진 부모 노드는 include **앞에** 둔다
    (`generate_launch_description` 의 순서 주석).

    **`GroupAction(scoped=True)` 로는 못 막는다.** 감싸면 include 가 끝날 때 스코프가
    pop 되는데, 백엔드는 노드를 `OnProcessExit` 로 **나중에** 띄우므로 그 시점에
    `servo_linear_scale` 같은 설정이 이미 사라져 launch 가 죽는다 (2026-09-09 실측).
    """
    backend_launch = os.path.join(
        get_package_share_directory("robot_control"), "launch", "panda_functionbay.launch.py")
    # 백엔드와 공유하는 argument 만 전달한다. 나머지는 백엔드 기본값을 쓴다.
    forwarded = {
        "camera_image_topic": LaunchConfiguration("camera_image_topic"),
        "log_level": LaunchConfiguration("log_level"),
        "enable_rviz": LaunchConfiguration("enable_rviz"),
        # **제어 계층 뷰어를 명시적으로 끈다.** include 된 launch 는 부모의 launch
        # configuration 을 그대로 물려받고 자기 `DeclareLaunchArgument` 의 기본값으로
        # 되돌리지 않는다 — 넘기지 않으면 `enable_image_viewer:=true` 가 백엔드까지
        # 새어 들어가 **같은 영상을 띄우는 창이 두 개** 뜬다. 이 계층의 뷰어는 세션
        # 상태를 오버레이하는 `rdfp_image_viewer_node` 다.
        "enable_image_viewer": TextSubstitution(text="false"),
        # **같은 이유로 백엔드의 raw 발행자도 끈다.** 물려받으면 `republish` 가 둘 뜨고,
        # 둘이 같은 토픽에 발행해 프레임이 섞인다 — 오류는 없고 증상만 이상해진다.
        # raw 는 이 계층이 만든다 (`create_raw_image_source_node` 아래).
        "enable_camera": TextSubstitution(text="false"),
    }
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(backend_launch), launch_arguments=forwarded.items())


def _declare_arguments() -> list[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            "log_level", default_value="info",
            choices=["debug", "info", "warn", "error", "fatal"],
            description="MoveIt·수집 노드 로그 레벨"),
        DeclareLaunchArgument(
            "enable_rviz", default_value="false",
            choices=["true", "false"],
            description=("RViz2 기동 여부. **기본 off** — 시뮬레이터가 이미 자기 화면을 "
                         "그리므로 화면이 겹치고 자원만 더 쓴다")),
        # --- 카메라 (시뮬레이터가 발행한다) ---
        DeclareLaunchArgument(
            "camera_image_topic", default_value=_CAMERA_IMAGE_TOPIC,
            description="뷰어·레코더가 구독하는 raw 이미지 토픽 (camera_republish 의 출력)"),
        *declare_camera_republish_arguments(_CAMERA_COMPRESSED_TOPIC),
        DeclareLaunchArgument(
            "camera_resolution", default_value=_CAMERA_RESOLUTION,
            description="레코더 해상도. 시뮬레이터 렌더 해상도와 같아야 한다"),
        DeclareLaunchArgument(
            "image_recorder_fps", default_value=_CAMERA_FPS,
            description=("레코더 FPS. **CFR 이라 이 값이 곧 영상의 시간축이다** — "
                         "소스 주기와 다르면 재생 속도가 어긋난다 (docstring 참고)")),
        # --- 수집 노드 ---
        DeclareLaunchArgument(
            "enable_image_viewer", default_value="true",
            choices=["true", "false"],
            description=("세션 상태를 겹쳐 보여주는 뷰어. 압축 이미지를 raw 로 되살리는 "
                         "camera_republish 도 이 인자로 함께 뜬다")),
        DeclareLaunchArgument(
            "enable_image_recorder_node", default_value="true",
            choices=["true", "false"], description="MP4 레코더"),
        DeclareLaunchArgument(
            "image_recorder_output_dir", default_value="/tmp/rdfp_recordings",
            description="레코더 출력 디렉터리"),
        DeclareLaunchArgument(
            "image_recorder_input_format", default_value="mjpeg",
            choices=["mjpeg", "rawvideo"],
            description=("레코더 입력 형식. **mjpeg 이 기본이다** — JPEG 을 디코드 없이 "
                         "ffmpeg 에 넘기므로 raw 토픽(9.2 MB/s) 자체가 안 생긴다. "
                         "rawvideo 로 바꾸면 raw 가 필요해져 enable_camera 가 함께 켜진다"),
        ),
        # **raw 소비자가 둘이다** — 뷰어와, rawvideo 로 돌리는 레코더. 둘 중 하나라도
        # 켜져 있으면 `enable_camera` 기본값이 true 가 된다. 예전에는 디코더가
        # `enable_image_viewer` 조건으로만 떠서, 헤드리스 수집에서 raw 발행자가 사라져
        # **레코더가 조용히 0 프레임을 담았다.**
        declare_camera_enable_argument(("enable_image_viewer", "true"),
                                       ("image_recorder_input_format", "rawvideo")),
        DeclareLaunchArgument(
            "target_joint_cmds_input_topic", default_value=_ARM_COMMAND_TOPIC,
            description="arm 명령 토픽. /target_joint_cmds 로 변환할 원본이다"),
    ]


def generate_launch_description() -> LaunchDescription:
    session_control_node = Node(
        package="rdfp", executable="session_control_node", name="session_control",
        output="screen", emulate_tty=True,
        ros_arguments=[
            "--log-level",
            [TextSubstitution(text="session_control:="), LaunchConfiguration("log_level")],
        ],
    )

    # arm 명령 → /target_joint_cmds (데이터셋의 action 채널).
    # source=joint_state 는 메시지의 이름을 그대로 쓰므로 컨트롤러 조회가 없다.
    target_joint_cmds_publisher = Node(
        package="robot_control", executable="target_joint_cmds_publisher",
        name="target_joint_cmds_publisher", output="screen", emulate_tty=True,
        parameters=[{"source": "joint_state"}],
        remappings=[("arm_command", LaunchConfiguration("target_joint_cmds_input_topic"))],
    )

    rdfp_image_viewer_node = Node(
        package="rdfp", executable="rdfp_image_viewer_node", name="rdfp_image_viewer_node",
        output="screen", emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_image_viewer")),
        remappings=[("image", LaunchConfiguration("camera_image_topic"))],
    )

    image_recorder_node = Node(
        # **세션 구동 레코더로 압축을 직접 받는다** (`rdfp_image_recorder`).
        #
        # 서비스 구동 `image_recorder_node` 는 `sensor_msgs/Image` 만 구독하고
        # `input_format` 파라미터가 없어, 시뮬레이터의 압축을 받으려면 `camera_republish`
        # 가 raw 로 되살려 줘야 했다. 그런데 그 노드는 **뷰어와 함께 뜨므로**
        # `enable_image_viewer:=false` 로 헤드리스 수집을 하면 발행자가 사라져
        # **레코더가 조용히 0 프레임을 담았다** — 오류도 경고도 없다.
        #
        # 압축을 그대로 받으면 그 의존이 없어지고, raw 로 되살릴 때 돌아오는 대역폭
        # 36배(0.27 → 9.2 MB/s)도 치르지 않는다. JPEG 바이트는 디코드 없이 ffmpeg 에
        # 넘어간다.
        package="rdfp", executable="rdfp_image_recorder",
        output="screen", emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_image_recorder_node")),
        parameters=[{
            "output_dir": LaunchConfiguration("image_recorder_output_dir"),
            "fps": LaunchConfiguration("image_recorder_fps"),
            "resolution": LaunchConfiguration("camera_resolution"),
            "input_format": LaunchConfiguration("image_recorder_input_format"),
        }],
        # **`input_format` 이 메시지 타입을 고르므로 토픽도 함께 갈려야 한다.**
        # mjpeg 이면 압축을, rawvideo 면 raw 를 구독한다 — 어긋나면 ROS 2 는 타입이
        # 달라 연결만 안 하고 오류를 내지 않아 증상이 "0 프레임" 뿐이다.
        remappings=[("image", PythonExpression(
            ["'", LaunchConfiguration("camera_compressed_topic"), "' if '",
             LaunchConfiguration("image_recorder_input_format"), "' == 'mjpeg' else '",
             LaunchConfiguration("camera_image_topic"), "'"]))],
    )

    return LaunchDescription([
        # **선언이 백엔드 include 보다 먼저 와야 한다** — 뒤에 두면 전달할
        # LaunchConfiguration 을 아직 모르는 상태에서 참조해 죽는다.
        *_declare_arguments(),
        # raw 소비자를 켠 채 `enable_camera:=false` 로 덮어썼으면 알린다.
        warn_if_raw_consumer_has_no_source(("enable_image_viewer", "true"),
                                           ("image_recorder_input_format", "rawvideo")),
        # ⚠️ **조건을 가진 이 계층의 노드는 include 보다 먼저 온다.**
        # include 에 넘긴 `enable_image_viewer:=false` · `enable_camera:=false` 가
        # 부모 스코프로 새기 때문이다(`_included_backend` 참고). 뒤에 두면 이 셋의
        # 조건이 `false` 로 평가돼 **뷰어와 raw 발행자가 조용히 안 뜬다** — 오류가 없어서
        # 2026-09-09 까지 드러나지 않았다. 노드는 전부 구독자라 순서가 앞이어도 안전하다.
        #
        # raw 이미지 발행자. 무엇이 뜨는지는 프로파일의 `camera.source` 가 정하고
        # (이 백엔드는 `compressed` → republish), 뜰지 말지는 `enable_camera` 가 정한다.
        create_raw_image_source_node(_BACKEND),
        rdfp_image_viewer_node,
        image_recorder_node,
        _included_backend(),
        session_control_node,
        target_joint_cmds_publisher,
    ])
