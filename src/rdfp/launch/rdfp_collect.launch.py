"""학습 데이터 수집 노드만 별도로 기동하는 launch.

이미 떠 있는 **로봇 제어 스택 위에** 수집 계층을 얹는다. 다음 두 조합이 같은
노드 그래프를 만든다.

    ros2 launch robot_control panda_mock.launch.py     # 터미널 1
    ros2 launch rdfp rdfp_collect.launch.py            # 터미널 2

    ros2 launch robot_control panda_mock.launch.py     # 터미널 1
    ros2 launch rdfp rdfp_collect.launch.py           # 터미널 2

기동 순서
---------
수집 노드 넷은 구독자·상태머신·서비스 서버라 **발행자보다 먼저 떠도 무방하다.**
그래서 `panda_mock` 의 controller 순차 기동 체인에 끼어들 필요가 없고, 별도
launch 로 분리할 수 있다.

유일한 예외가 `target_joint_cmds_publisher` 의 관절 이름 조회
(`<arm controller>/get_parameters`)인데, `__init__` 이 아니라 타이머에서
`joint_names_timeout`(기본 10초) 안에 비동기로 수행하며 실패해도 `JointState.name`
이 비는 것으로 그친다. **제어 스택을 먼저 띄우면 이 조회가 오히려 더 안전하다** —
묶음 launch 는 hand 컨트롤러 spawner 종료 직후 동시에 뜨므로 여유가 더 적다.

백엔드별 인자 — `backend` 하나로 끝난다
--------------------------------------
`backend:=<이름>` 을 주면 `config/backends/<이름>.yaml` 에서 기본값을 가져온다.
**개별 인자를 명시하면 그쪽이 이긴다** (인자 > 프로파일 > 코드 기본값).

    ros2 launch robot_control panda_isaac.launch.py      # 터미널 1
    ros2 launch rdfp rdfp_collect.launch.py backend:=isaac   # 터미널 2

프로파일에서 오는 값은 다섯이다 — arm 명령 채널(`arm_cmd_source` +
`target_joint_cmds_input_topic`), `use_sim_time`, `camera_image_topic`,
`camera_compressed_topic`, `image_recorder_input_format`.

============ ==================== ====================================== =========
backend      arm_cmd_source       입력 토픽                              use_sim_time
============ ==================== ====================================== =========
mock         joint_trajectory     /panda_arm_controller/joint_trajectory false
mock_jgpc    float64_multi_array  /panda_arm_controller/commands         false
isaac        joint_state          /isaac/arm_command                     **true**
functionbay  joint_state          /input/panda_joint                     false
============ ==================== ====================================== =========

⚠️ **arm 명령 채널이 프로파일의 어느 키에 있는지가 백엔드마다 다르다.**
`arm_command.topic` 은 *MoveGroup 클라이언트*의 채널이라 JTC 백엔드에는 아예 없고,
Isaac 의 실제 명령은 `ros2_control.joint_commands_topic` 으로 나간다 — 한쪽만 보면
Isaac 이 조용히 기본 토픽을 구독해 **0 건**을 받는다. `_backend_defaults` 가 둘을
함께 본다.

`backend` 를 안 주면 종래대로 `image_pipeline.yaml` 과 코드 기본값
(`joint_trajectory` · `use_sim_time:=false`)을 쓴다. Gazebo 는 프로파일이 없으므로
`use_sim_time:=true` 를 손으로 준다 — 백엔드가 /clock 을 쓰는데 수집 노드만 벽시계를
쓰면 타임스탬프가 어긋난다.

레코더 — 세션 구동 하나다
------------------------
`rdfp_image_recorder` 가 `/session` 이 IN_EPISODE 인 구간을 자동 녹화하고 에피소드
경계로 분할하며 `.jsonl` 사이드카 + `metadata.json` 을 함께 쓴다.
`enable_image_recorder` 로 켜고 끈다.

**서비스 구동 `image_recorder_node` 는 이 launch 가 띄우지 않는다 (2026-09-13).**
한때 `recorder:=service` 로 고를 수 있었는데, 그 인자는 삭제된 묶음 launch 와의
등가를 맞추려고 둔 것이었다. 묶음이 전부 사라진 지금 남은 용도는 세션 없이 계속
녹화하는 `auto_start` 경로뿐이고 — **그 노드의 start/stop 서비스를 부르는 production
코드도 없다** (호출자는 테스트뿐이다). 필요하면 직접 띄운다:

    ros2 run rdfp image_recorder_node --ros-args -p output_dir:=/tmp/rec ...

사용법은 ``docs/collect/image_recorder_node_guide.md`` 를 본다.

`image_recorder_input_format:=mjpeg` 이면 세션 레코더가 **압축 토픽을 직접** 구독해
ffmpeg 에 그대로 넘긴다. 펑션베이처럼 시뮬레이터가 압축만 주는 백엔드에서 raw 로
되살릴 필요가 없어진다 — 되살리면 압축으로 아낀 대역폭이 36배로 돌아온다.
⚠️ **형식과 토픽은 함께 간다** — 어긋나면 ROS 2 는 타입이 달라 연결만 안 하고 오류를
내지 않아 증상이 "0 프레임" 뿐이다.

적재에는 영향이 없다. 후처리 파이프라인은 이 MP4 를 읽지 않고 rosbag 의 이미지
토픽에서 `FrameRouter` 가 직접 만든다 — 두 MP4 는 독립 산출물이다.

주의 — 설정값 정합
------------------
`camera_image_topic` / `camera_resolution` / `camera_fps` 는 제어 스택과 **공유**
한다. 기본값이 같은 `image_pipeline.yaml` 에서 오므로 손대지 않으면 자동으로
일치하지만, **한쪽에만 override 하면 조용히 어긋난다** (레코더가 프레임을 버리고
auto-stop 한다). 바꿀 때는 양쪽에 같은 `config_file:=` 을 준다.
"""

from __future__ import annotations

from typing import Any

import json
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter

from robot_control.backend_profiles import backends
from robot_control.backends import get_backend
from robot_control.launch_helpers.image_pipeline import (
    declare_config_file_argument,
    declare_image_pipeline_arguments,
    load_config,
)

# arm 명령 채널. `target_joint_cmds_publisher` 의 `source` 파라미터 값과
# 그때 노드가 구독하는 토픽 이름(remap 대상), 기본 입력 토픽의 대응표다.
ARM_CMD_SOURCES: dict[str, tuple[str, str]] = {
    # source              : (노드 내부 토픽 이름, 기본 입력 토픽)
    "joint_trajectory": ("joint_trajectory", "/panda_arm_controller/joint_trajectory"),
    "float64_multi_array": ("commands", "/panda_arm_controller/commands"),
    # 토픽 연동형 시뮬레이터(펑션베이). 명령이 이미 JointState 다.
    "joint_state": ("arm_command", "/input/panda_joint"),
}

DEFAULT_ARM_CONTROLLER_NODE_NAME = "/panda_arm_controller"


def _backend_defaults(name: str) -> dict:
    """백엔드 프로파일에서 이 launch 의 기본값을 뽑는다.

    **arm 명령 채널이 프로파일의 어느 키에 있는지가 백엔드마다 다르다.**
    `arm_command.topic` 은 *MoveGroup 클라이언트*의 채널이라 JTC 백엔드에는 아예
    없고(`mock`·`mock_jgpc`·`isaac` 모두 `None`), Isaac 의 실제 명령은
    `ros2_control.joint_commands_topic` 으로 나간다. 그래서 둘을 함께 본다 — 한쪽만
    보면 Isaac 이 조용히 기본 토픽을 구독해 **0 건**을 받는다.

    프로파일이 채널을 하나도 선언하지 않는 백엔드(mock 계열)는 컨트롤러 토픽이
    답이고, 그것은 프로파일이 아니라 :data:`ARM_CMD_SOURCES` 가 갖는다.
    """
    backend = get_backend(name)
    ros2_control = backend.block("ros2_control") or {}

    # 이미 JointState 로 나가는 채널이 있으면 그것이 곧 입력이다.
    topic = backend.arm_command_topic if backend.arm_command_format == "joint_state" else None
    topic = topic or ros2_control.get("joint_commands_topic")
    if topic:
        source = "joint_state"
    elif backend.arm_command_mode == "jgpc":
        source = "float64_multi_array"
    else:
        source = "joint_trajectory"

    resolution, fps = _camera_geometry(backend)
    return {
        "arm_cmd_source": source,
        "target_joint_cmds_input_topic": topic or ARM_CMD_SOURCES[source][1],
        "use_sim_time": "true" if backend.use_sim_time else "false",
        "camera_image_topic": backend.value("camera", "image_topic"),
        "camera_compressed_topic": backend.value("camera", "compressed_topic"),
        "camera_resolution": resolution,
        "image_recorder_fps": fps,
        # ⚠️ **Isaac 은 `rgb8` 로 발행한다.** 레코더 기본값 `bgr8` 로 두면 모든 프레임이
        # 버려지고 mp4 가 0 프레임이 된다 — 경고만 쌓이고 오류는 안 난다(2026-09-19 실측).
        "image_recorder_pixel_format": backend.value("camera", "pixel_format"),
        # 시뮬레이터가 압축만 주면 세션 레코더가 그대로 받는다 — raw 로 되살리면
        # 압축으로 아낀 대역폭이 그대로 돌아온다.
        "image_recorder_input_format":
            "mjpeg" if backend.camera_source == "compressed" else "rawvideo",
    }


def _camera_geometry(backend) -> tuple:
    """`(resolution, fps)`. 모르면 `(None, None)` — `image_pipeline.yaml` 이 답한다.

    **레코더는 CFR 이라 이 값이 곧 영상의 시간축이다.** 어긋나면 오류가 아니라 재생
    속도가 밀리므로, 정본을 **한 곳**에만 두고 여기서 그것을 찾아온다.

    * 시뮬레이터가 씬 정의 파일로 카메라를 함께 선언하면(Isaac 의 `isaac_scene.json`)
      **그 파일이 정본이다** — 시뮬레이터 자신이 그것을 읽어 렌더링하므로, 프로파일에
      복사해 두면 한쪽만 고쳤을 때 조용히 어긋난다.
    * 그런 파일이 없으면 프로파일의 `camera.resolution`/`camera.fps` 가 정본이다
      (펑션베이).
    * 둘 다 없으면 `image_pipeline.yaml` 로 떨어진다 (mock 계열 — OpenCV 카메라라
      그 YAML 이 값을 *정하는* 쪽이다).
    """
    camera = _scene_camera(backend)
    if camera:
        size = camera.get("resolution")
        resolution = f"{size[0]}x{size[1]}" if isinstance(size, (list, tuple)) else size
        return resolution, camera.get("fps")
    return backend.value("camera", "resolution"), backend.value("camera", "fps")


def _scene_camera(backend) -> dict:
    """`scene.file` 이 선언한 카메라 블록. 없으면 빈 dict."""
    path = backend.share_path(backend.value("scene", "file"))
    if not path or not os.path.isfile(path):
        return {}
    with open(path, encoding="utf-8") as handle:
        return (json.load(handle) or {}).get("camera") or {}


def _declare_arguments() -> list[DeclareLaunchArgument]:
    """이미지 파이프라인 밖의 인자를 선언한다."""
    return [
        DeclareLaunchArgument(
            "log_level", default_value="info",
            choices=["debug", "info", "warn", "error", "fatal"],
            description="session_control 의 로그 레벨",
        ),
        DeclareLaunchArgument(
            "backend", default_value="",
            choices=[""] + list(backends()),
            description=(
                "제어 스택의 백엔드 이름. 주면 arm 명령 채널 · 카메라 토픽 · "
                "use_sim_time · 레코더 입력 형식의 기본값을 "
                "config/backends/<이름>.yaml 에서 가져온다. 개별 인자를 명시하면 "
                "그쪽이 이긴다. 비우면 종래대로 image_pipeline.yaml 과 코드 기본값을 쓴다"
            ),
        ),
        DeclareLaunchArgument(
            "arm_cmd_source", default_value="",
            choices=[""] + sorted(ARM_CMD_SOURCES) + ["none"],
            description=(
                "arm 명령 채널. JTC=joint_trajectory / JGPC=float64_multi_array / "
                "토픽 연동 시뮬레이터=joint_state. none 이면 "
                "target_joint_cmds_publisher 를 띄우지 않는다. "
                "비우면 backend 가 정하고, backend 도 없으면 joint_trajectory"
            ),
        ),
        DeclareLaunchArgument(
            "target_joint_cmds_input_topic", default_value="",
            description=(
                "target_joint_cmds_publisher 의 입력 토픽. 비우면 arm_cmd_source 에 "
                "맞는 기본값을 쓴다"
            ),
        ),
        DeclareLaunchArgument(
            "arm_controller_node_name", default_value=DEFAULT_ARM_CONTROLLER_NODE_NAME,
            description="관절 이름을 조회할 arm 컨트롤러 노드 이름",
        ),
        DeclareLaunchArgument(
            "image_recorder_input_format", default_value="",
            choices=["", "rawvideo", "mjpeg"],
            description=(
                "session 레코더가 받을 형식. mjpeg 이면 압축 토픽을 직접 구독해 "
                "ffmpeg 에 그대로 넘긴다 (service 레코더는 rawvideo 뿐이다)"
            ),
        ),
        DeclareLaunchArgument(
            "camera_compressed_topic", default_value="",
            description=("input_format:=mjpeg 일 때 구독할 압축 토픽. 비우면 backend 의 "
                         "camera.compressed_topic"),
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="",
            choices=["", "true", "false"],
            description=("시뮬레이션 시계(/clock)를 쓸 것인가. 비우면 backend 가 정하고, "
                         "backend 도 없으면 false"),
        ),
    ]


def _build_actions(context: LaunchContext) -> list[Any]:
    """`config_file` resolve 이후 인자와 노드를 구성한다.

    **인자가 프로파일을 이긴다.** 파생 대상 인자의 기본값이 빈 문자열이고, 비어 있을
    때만 프로파일 값을 쓴다 — 그래서 `backend:=isaac servo 실험용 토픽:=...` 같은
    부분 덮어쓰기가 그대로 된다.
    """
    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    defaults = _backend_defaults(arg("backend")) if arg("backend") else {}

    def pick(name: str, fallback: Any = "") -> Any:
        """인자 > 프로파일 > fallback."""
        return arg(name) or defaults.get(name) or fallback

    image_config = load_config(arg("config_file"))
    # `camera_image_topic` 은 이 함수 안에서 선언되므로 perform 할 수 없다 —
    # 헬퍼에 넘기는 설정 쪽을 갈아 기본값을 바꾼다.
    if defaults.get("camera_image_topic"):
        image_config["camera"]["image_topic"] = defaults["camera_image_topic"]
    # 해상도·fps 도 같은 방식이다. **레코더 fps 는 카메라 fps 와 같아야 한다** —
    # 두 키가 따로 있지만 값은 하나에서 온다.
    if defaults.get("camera_resolution"):
        image_config["camera"]["resolution"] = defaults["camera_resolution"]
    if defaults.get("image_recorder_fps"):
        image_config["camera"]["fps"] = defaults["image_recorder_fps"]
        image_config["image_recorder"]["fps"] = defaults["image_recorder_fps"]
    # ⚠️ **픽셀 형식도 백엔드가 정한다 — Isaac 은 `rgb8` 이다** (2026-09-19 실측).
    # `bgr8` 로 두면 레코더가 **모든 프레임을 버리고** mp4 가 0 프레임이 된다.
    # 경고만 쌓이고 오류는 안 나므로 적재까지 가서야 `image_frames: 0` 으로 드러난다.
    if defaults.get("image_recorder_pixel_format"):
        image_config["image_recorder"]["pixel_format"] = \
            defaults["image_recorder_pixel_format"]

    arm_cmd_source = pick("arm_cmd_source", "joint_trajectory")
    use_sim_time = pick("use_sim_time", "false").lower() == "true"
    input_format = pick("image_recorder_input_format", "rawvideo")
    compressed_topic = pick("camera_compressed_topic", "/camera_image/compressed")

    # --- session_control ---
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

    # --- 세션 상태 오버레이 뷰어 ---
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

    # --- 라이브 MP4 녹화 (세션 구동) ---
    # `/session` 이 IN_EPISODE 인 구간을 자동 녹화하고 에피소드 경계로 분할하며
    # `.jsonl` 사이드카 + `metadata.json` 을 함께 쓴다. 이 launch 는 `session_control`
    # 을 무조건 띄우므로 전제 조건이 이미 갖춰져 있다.
    #
    # **서비스 구동 `image_recorder_node` 는 여기서 띄우지 않는다** — 그 노드의
    # start/stop 서비스를 부르는 production 코드가 없고, 세션 없이 계속 녹화하는
    # `auto_start` 경로만 남아 있었다. 필요하면 `ros2 run rdfp image_recorder_node`
    # 로 띄운다 (docs/collect/image_recorder_node_guide.md).
    #
    # fps / resolution 은 카메라 설정을 그대로 써야 프레임이 버려지지 않는다 —
    # 레코더는 CFR 이라 fps 가 어긋나면 영상의 시간축이 통째로 밀린다.
    rdfp_image_recorder = Node(
        package="rdfp",
        executable="rdfp_image_recorder",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_image_recorder")),
        # **remap 이 아니라 파라미터다** — 이 노드는 `Image` 와 `CompressedImage` 를
        # 같은 이름으로 함께 구독하므로 구독 이름을 자기가 갖는다.
        parameters=[{
            "output_dir": LaunchConfiguration("image_recorder_output_dir"),
            "fps": LaunchConfiguration("image_recorder_fps"),
            "pixel_format": LaunchConfiguration("image_recorder_pixel_format"),
            "bitrate": LaunchConfiguration("image_recorder_bitrate"),
            "gop_size": LaunchConfiguration("image_recorder_gop_size"),
            "preset": LaunchConfiguration("image_recorder_preset"),
            "encoder_mode": LaunchConfiguration("image_recorder_encoder_mode"),
            "resolution": LaunchConfiguration("camera_resolution"),
            "input_format": input_format,
            "image_topic": (compressed_topic if input_format == "mjpeg"
                            else LaunchConfiguration("camera_image_topic")),
        }],
    )

    actions: list[Any] = [session_control_node, rdfp_image_viewer_node,
                          rdfp_image_recorder]

    # --- action(명령값) 채널 ---
    if arm_cmd_source != "none":
        internal_topic, default_input = ARM_CMD_SOURCES[arm_cmd_source]
        input_topic = pick("target_joint_cmds_input_topic", default_input)
        actions.append(Node(
            package="robot_control",
            executable="target_joint_cmds_publisher",
            name="target_joint_cmds_publisher",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "source": arm_cmd_source,
                "controller_node_name": LaunchConfiguration("arm_controller_node_name"),
            }],
            remappings=[(internal_topic, input_topic)],
        ))

    # 시뮬레이션 백엔드와 클럭을 맞춘다. 노드 생성보다 먼저 와야 적용된다.
    if use_sim_time:
        actions.insert(0, SetParameter(name="use_sim_time", value=True))

    # YAML 에 의존하는 인자만 여기서 선언한다. 나머지는 generate_launch_description
    # 에서 미리 선언해야 한다 — OpaqueFunction 안에서 perform() 하려면 그 시점에
    # 이미 선언되어 있어야 하기 때문이다.
    # `include_auto_start=False`: 세션 구동 레코더는 `/session` 으로 시작하므로 그
    # 파라미터를 받지 않는다 — 선언해 봐야 소비자가 없다.
    return [*declare_image_pipeline_arguments(image_config, include_auto_start=False),
            *actions]


def generate_launch_description() -> LaunchDescription:
    # YAML 과 무관한 인자는 여기서 선언한다. `_build_actions` 가 이들을
    # `perform()` 하므로 OpaqueFunction 보다 앞서야 하고, 덕분에 `--show-args`
    # 에도 그대로 드러난다.
    return LaunchDescription([
        declare_config_file_argument("config_file"),
        *_declare_arguments(),
        OpaqueFunction(function=_build_actions),
    ])
