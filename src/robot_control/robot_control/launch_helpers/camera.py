"""카메라 노드 및 파라미터 설정.

argument 기본값은 :mod:`image_pipeline_launch_helper` 를 통해
``config/image_pipeline.yaml`` 에서 온다. 예전에는 이 파일이 기본값을
하드코딩했는데, 같은 카메라를 쓰는 launch 들끼리 값이 갈라지는 문제가 있었다
(앱 계열 ``640x480`` / ``id: 4`` vs panda 계열 YAML ``1280x720`` / mp4 경로).
"""

from __future__ import annotations

from typing import Optional

from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from .image_pipeline import (
    declare_camera_arguments as _declare_camera_arguments_from,
    load_config,
)


def declare_camera_arguments() -> list[DeclareLaunchArgument]:
    """카메라 관련 launch argument들을 선언한다 (기본 YAML 의 값을 기본값으로 사용).

    설정 파일을 바꿔 끼워야 하면 이 함수 대신
    ``image_pipeline_launch_helper.declare_camera_arguments(load_config(path))`` 를
    쓴다 — 그러려면 ``config_file`` 이 resolve 된 뒤여야 하므로 launch 쪽에서
    ``OpaqueFunction`` 이 필요하다.
    """
    return _declare_camera_arguments_from(load_config())


def declare_camera_enable_argument(*raw_consumers) -> DeclareLaunchArgument:
    """`enable_camera` — **raw 이미지 발행자를 둘 것인가.**

    이름이 노드가 아니라 **역할**을 가리킨다. 같은 스위치가 백엔드에 따라 OpenCV
    `camera_node` 를 띄우기도, 압축을 푸는 `image_transport/republish` 를 띄우기도,
    아무것도 안 띄우기도 한다(`camera.source`) — 그래서 `enable_camera_node` 라는 이름을
    버렸다. 둘을 따로 두면 **같은 토픽에 발행자가 둘**이 되는 조합이 만들어진다.

    Args:
        *raw_consumers: 이 launch 의 raw 소비자. ``(argument 이름, 켜진 값)`` 쌍이며,
            **하나라도 그 값이면 기본값이 `true`** 가 된다. 아무것도 안 주면 `false`.

    raw 소비자에서 기본값을 파생시키는 이유는 **"뷰어를 켰는데 빈 창"** 을 없애기
    위해서다. 예전에는 디코더가 `enable_image_viewer` 조건으로 떠서, 헤드리스로 돌리려고
    뷰어를 끄면 raw 발행자가 사라져 **서비스 구동 레코더가 조용히 0 프레임을 담았다.**
    파생은 그 결합을 끊는다 — 디코더는 모든 raw 소비자를 보고 뜬다.

    `enable_camera:=false` 로 덮어써서 끌 수는 있다(의도적으로 끄는 경우가 있다).
    그 조합은 launch 가 경고로 알린다 — :func:`warn_if_raw_consumer_has_no_source`.
    """
    if not raw_consumers:
        default = "false"
    elif len(raw_consumers) == 1 and raw_consumers[0][1] == "true":
        # 흔한 경우는 치환 하나로 끝난다 — PythonExpression 을 끼우지 않는다.
        default = LaunchConfiguration(raw_consumers[0][0])
    else:
        default = PythonExpression(["'true' if ", *_any_matches(raw_consumers), " else 'false'"])
    return DeclareLaunchArgument(
        "enable_camera", default_value=default,
        description=("raw 이미지 발행자 기동 여부. 무엇이 뜨는지는 백엔드 프로파일의 "
                     "camera.source 가 정한다. 기본값은 이 launch 의 raw 소비자에서 "
                     "파생된다"),
    )


def _any_matches(pairs) -> list:
    """``"'a' == 'x' or 'b' == 'y'"`` 형태의 PythonExpression 조각."""
    parts: list = []
    for index, (name, expected) in enumerate(pairs):
        if index:
            parts.append(" or ")
        parts.extend(["'", LaunchConfiguration(name), f"' == '{expected}'"])
    return parts


def warn_if_raw_consumer_has_no_source(*raw_consumers) -> LogInfo:
    """raw 소비자를 켜 놓고 `enable_camera:=false` 로 덮어썼을 때 알린다.

    **막지는 않는다.** 그 조합은 의도일 수 있다(예: 다른 곳에서 raw 를 발행 중). 다만
    조용히 실패하면 증상이 "빈 창" / "0 프레임" 뿐이라 원인을 안 가리키므로, 로그에 한 줄을
    남긴다.
    """
    return LogInfo(
        condition=IfCondition(PythonExpression(
            ["'true' if (", *_any_matches(raw_consumers),
             ") and '", LaunchConfiguration("enable_camera"), "' != 'true' else 'false'"])),
        msg=("[camera] WARNING: a raw-image consumer is enabled but enable_camera is false; "
             "nothing will publish sensor_msgs/Image and the consumer stays silent"),
    )


def declare_simulator_camera_arguments(
    image_topic: Optional[str] = None,
    *raw_consumers,
) -> list[DeclareLaunchArgument]:
    """시뮬레이터가 이미지를 발행하는 스택용 argument.

    **OpenCV 카메라 설정 인자를 선언하지 않는다** — `camera_id` `camera_fps`
    `camera_info_topic` 이 빠진다. 그 값들은 우리가 정하는 것이 아니라 시뮬레이터가
    주는 대로 쓰는 것이라, 인자로 열어 두면 실제와 다른 값을 넣을 수 있고 그것이
    조용히 틀린 데이터가 된다 — 특히 레코더는 CFR 이라 fps 가 어긋나면 영상의
    시간축이 통째로 밀린다.

    `camera_info` 는 OpenCV 경로에서만 쓴다 — 시뮬레이터는 제공하지 않는다.

    Args:
        image_topic: raw 이미지 토픽의 기본값. **백엔드 프로파일의 `camera.image_topic`
            을 넘긴다** — 이 토픽 이름은 우리가 정하는 것이 아니라 시뮬레이터(또는 그것을
            되살리는 republish)가 정하는 값이라 백엔드마다 다르다. ``None`` 이면
            `image_pipeline.yaml` 로 떨어지는데, 그 값은 **OpenCV 카메라용**이라
            시뮬레이터 스택에서는 프로파일이 선언한 것과 다른 이름이 된다 (실제로 갈려
            있었다: 프로파일 `/camera_image` 대 실효 `/camera/image_raw`).
        *raw_consumers: :func:`declare_camera_enable_argument` 에 그대로 넘긴다.
    """
    cam = load_config()["camera"]
    return [
        declare_camera_enable_argument(*raw_consumers),
        DeclareLaunchArgument(
            "camera_image_topic", default_value=str(image_topic or cam["image_topic"]),
            description="이미지 토픽. 시뮬레이터 발행 토픽으로 바꿔 준다",
        ),
    ]


def create_raw_image_source_node(backend, config: Optional[dict] = None):
    """이 백엔드에서 **raw 이미지를 만드는 노드.** 없으면 ``None``.

    무엇이 뜨는지는 프로파일의 `camera.source` 가 정한다 — 소비자는 어느 쪽이 떴는지
    몰라도 `camera_image_topic` 만 구독하면 된다. 그것이 이 설계의 목적이다
    (`docs/camera/compressed_image_pipeline_design.md` §2).

    ==============  ==========================================
    `camera.source` 뜨는 노드
    ==============  ==========================================
    `device`        `camera_node` (OpenCV, 장치/파일)
    `compressed`    `image_transport/republish` (압축 → raw)
    `native`        없다 — 시뮬레이터가 raw 를 직접 낸다
    ==============  ==========================================

    Args:
        backend: :class:`robot_control.backends.Backend`.
        config: `device` 경로에서 OpenCV 설정을 읽을 dict. ``None`` 이면 launch 가
            선언한 카메라 argument 에서 읽는다 (:func:`create_camera_node`).

    Raises:
        ValueError: 모르는 `camera.source` 일 때. **조용히 아무것도 안 띄우지 않는다** —
            그러면 증상이 "이미지가 안 온다" 뿐이라 원인을 안 가리킨다.
    """
    source = backend.camera_source
    if source is None or source == "native":
        # `None` 은 카메라 블록이 없다는 뜻이고, `native` 는 시뮬레이터가 직접 낸다.
        return None
    if source == "compressed":
        return create_camera_republish_node()
    if source == "device":
        return create_camera_node(config)
    raise ValueError(
        f"{backend.name}: unknown camera.source {source!r}; "
        "expected one of 'device', 'compressed', 'native'")


def create_camera_node(config: dict | None = None) -> Node:
    """카메라 노드를 생성한다.

    Args:
        config: 주면 OpenCV 전용 설정(`id`/`fps`/`camera_info` 등)을 이 dict 에서
            읽는다. 시뮬레이터 스택처럼 해당 argument 를 선언하지 않는 launch 에서
            쓴다. ``None`` 이면 기존대로 LaunchConfiguration 에서 읽는다.
    """
    if config is not None:
        cam = config["camera"]
        return Node(
            package="robot_control",
            executable="camera_node",
            name="camera_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_camera")),
            parameters=[{
                "camera_id": str(cam["id"]),
                "fps": int(cam["fps"]),
                "resolution": str(cam["resolution"]),
                "frame_id": str(cam["frame_id"]),
                "compress_image": bool(cam["compress_image"]),
            }],
            remappings=[
                ("~/image_raw", LaunchConfiguration("camera_image_topic")),
                ("~/camera_info", str(cam["info_topic"])),
                ("~/camera_status", str(cam["status_topic"])),
            ],
        )
    return Node(
        package="robot_control",
        executable="camera_node",
        name="camera_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_camera")),
        parameters=[
            {
                "camera_id": LaunchConfiguration("camera_id"),
                "fps": LaunchConfiguration("camera_fps"),
                "resolution": LaunchConfiguration("camera_resolution"),
                "frame_id": LaunchConfiguration("camera_frame_id"),
                "compress_image": LaunchConfiguration("camera_compress_image"),
            }
        ],
        remappings=[
            ("~/image_raw", LaunchConfiguration("camera_image_topic")),
            ("~/camera_info", LaunchConfiguration("camera_info_topic")),
            ("~/camera_status", LaunchConfiguration("camera_status_topic")),
        ],
    )


def declare_camera_republish_arguments(
    default_compressed_topic: str,
) -> list[DeclareLaunchArgument]:
    """압축 이미지를 raw 로 되살리는 노드용 argument.

    기본값을 호출 쪽에서 받는다 — 압축 토픽 이름은 우리가 정하는 것이 아니라
    **시뮬레이터가 정하는 값**이라 백엔드마다 다르다.
    """
    return [
        DeclareLaunchArgument(
            "camera_compressed_topic", default_value=default_compressed_topic,
            description=("시뮬레이터가 발행하는 압축 이미지 토픽. "
                         "camera_image_topic 으로 되살릴 원본이다"),
        ),
    ]


def create_camera_republish_node() -> Node:
    """``CompressedImage`` → ``Image`` 변환 노드. **``enable_camera`` 가 켠다.**

    **파이썬 노드를 만들지 않았다.** 디코드는 어차피 네이티브(libjpeg-turbo)이고
    ``image_transport`` 의 ``republish`` 가 C++ 로 같은 일을 한다. 실측(2026-09-08):
    ``cv2.imdecode`` 는 640x480 / 26.8 KB 에서 **0.75 ms/frame** 이고 **GIL 도 놓는다**
    (디코드 스레드를 붙여도 파이썬 루프가 101% 유지) — 즉 파이썬이 느려서 피한 것이
    아니라, 직접 만들면 **유지보수할 코드만 늘기 때문**이다.

    **필요할 때만 뜨는 이유** — raw 로 되살리는 순간 압축으로 아낀 대역폭이 되살아난다
    (0.27 → 9.2 MB/s, 36배). 세션 구동 레코더(``rdfp_image_recorder``)는
    ``input_format='mjpeg'`` 로 압축을 그대로 받으므로, raw 소비자가 하나도 없으면 이
    노드도 뜨지 않고 압축만 흐른다.

    ⚠️ **한때 ``enable_image_viewer`` 조건으로 떴다.** 그래서 헤드리스로 돌리려고 뷰어를
    끄면 raw 발행자가 사라져 **서비스 구동 ``image_recorder_node`` 가 조용히 0 프레임을
    담았다**(오류도 경고도 없다 — 그 노드는 ``sensor_msgs/Image`` 만 구독한다).
    지금은 ``enable_camera`` 가 켜고, 그 기본값이 **모든** raw 소비자에서 파생되므로
    (:func:`declare_camera_enable_argument`) 소비자를 늘려도 이 결합이 되살아나지 않는다.

    뷰어 없이 raw 가 필요하면 손으로 띄운다::

        ros2 run image_transport republish compressed raw --ros-args \\
            -r in/compressed:=<압축 토픽> -r out:=<raw 토픽>

    **remap 대상은 ``in/compressed`` 다 — ``in`` 이 아니다.** image_transport 의 구독
    플러그인은 base 토픽(``in``)에 transport 접미사를 붙인 **완성된 이름**으로 구독하므로
    ``in`` 만 remap 하면 한 장도 받지 못한다 (오류도 없다).

    ⚠️ 이 노드와 OpenCV ``camera_node`` 는 **같은 토픽에 발행한다.** 그래서 스위치를
    ``enable_camera`` 하나로 합치고 어느 쪽이 뜰지는 프로파일의 ``camera.source`` 가
    정하게 했다 — 둘 다 뜨는 조합이 **만들어질 수 없다**
    (:func:`create_raw_image_source_node`). 장치 카메라를 시뮬레이터 스택에 함께 붙여야
    하면 ``camera_node`` 를 손으로 띄우고 토픽을 따로 준다.
    """
    return Node(
        package="image_transport",
        executable="republish",
        name="camera_republish",
        output="screen",
        emulate_tty=True,
        # 위치 인자가 in/out transport 다 — launch 가 뒤에 붙이는 `--ros-args` 보다 앞이어야 한다.
        arguments=["compressed", "raw"],
        condition=IfCondition(LaunchConfiguration("enable_camera")),
        remappings=[
            ("in/compressed", LaunchConfiguration("camera_compressed_topic")),
            ("out", LaunchConfiguration("camera_image_topic")),
        ],
    )
