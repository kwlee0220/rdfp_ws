#!/usr/bin/env python3

from __future__ import annotations

from typing import Final


# 입력 인코딩 → ffmpeg `-pix_fmt` 매핑
_ENCODING_MAP: Final[dict[str, str]] = {
    "bgr8": "bgr24",
    "rgb8": "rgb24",
    "bgra8": "bgra",
    "rgba8": "rgba",
    "mono8": "gray",
}

# 입력 인코딩 → 픽셀당 채널 수 매핑
_CHANNELS_MAP: Final[dict[str, int]] = {
    "bgr8": 3,
    "rgb8": 3,
    "bgra8": 4,
    "rgba8": 4,
    "mono8": 1,
}

# 외부에서 참조할 수 있도록 공개하는 지원 인코딩 집합
SUPPORTED_ENCODINGS: Final[frozenset[str]] = frozenset(_ENCODING_MAP.keys())


def channels_for(pixel_format: str) -> int:
    """입력 픽셀 포맷의 픽셀당 채널 수를 반환한다.

    Args:
        pixel_format: `bgr8` / `rgb8` / `bgra8` / `rgba8` / `mono8` 중 하나.

    Returns:
        픽셀당 채널 수 (3 또는 4 또는 1).

    Raises:
        ValueError: 지원하지 않는 픽셀 포맷이 전달된 경우.
    """
    try:
        return _CHANNELS_MAP[pixel_format]
    except KeyError as exc:
        raise ValueError(
            f"unsupported pixel_format: {pixel_format!r}; "
            f"expected one of {sorted(SUPPORTED_ENCODINGS)}"
        ) from exc


# 입력 형식 — stdin 으로 들어오는 것이 **raw 픽셀인가 JPEG 인가**를 가른다.
#
# ``mjpeg`` 는 시뮬레이터가 `sensor_msgs/CompressedImage` (JPEG) 로 보낼 때 쓴다.
# JPEG 바이트를 그대로 ffmpeg 에 넘기면 **우리가 디코드하지 않아도** ffmpeg 이
# 디코드와 H.264 인코딩을 함께 한다. 실측(2026-09-08, 640x480 60프레임):
# Python `cv2.imdecode` 를 거치면 0.24 s, 그대로 넘기면 0.15 s 이고 결과 파일도
# 92 KB 대 99 KB 로 오히려 작다 — JPEG(yuvj420p) → BGR → yuv420p 왕복이 한 단계
# 줄기 때문이다.
INPUT_RAWVIDEO: Final[str] = "rawvideo"
INPUT_MJPEG: Final[str] = "mjpeg"

SUPPORTED_INPUT_FORMATS: Final[frozenset[str]] = frozenset({INPUT_RAWVIDEO, INPUT_MJPEG})


def validate_input_format(input_format: str) -> str:
    """지원하는 입력 형식인지 확인하고 그대로 돌려준다."""
    if input_format not in SUPPORTED_INPUT_FORMATS:
        raise ValueError(
            f"unsupported input_format: {input_format!r}; "
            f"expected one of {sorted(SUPPORTED_INPUT_FORMATS)}")
    return input_format


# 지원 코덱 상수 (probe / 커맨드 빌더 공용)
CODEC_LIBX264: Final[str] = "libx264"
# 무손실 보관용 코덱 — `build_lossless_output_args` 만 쓴다. 하드웨어 probe 의
# 후보가 아니므로 `SUPPORTED_CODECS` 에 넣지 않는다.
CODEC_LIBX264RGB: Final[str] = "libx264rgb"
# 스트림 복사(`-c:v copy`)로 담긴 JPEG 의 코덱 이름. metadata 의 `codec` 에 남는다.
CODEC_MJPEG: Final[str] = "mjpeg"
CODEC_H264_NVENC: Final[str] = "h264_nvenc"
CODEC_H264_QSV: Final[str] = "h264_qsv"
CODEC_H264_VAAPI: Final[str] = "h264_vaapi"

SUPPORTED_CODECS: Final[frozenset[str]] = frozenset(
    {
        CODEC_LIBX264,
        CODEC_H264_NVENC,
        CODEC_H264_QSV,
        CODEC_H264_VAAPI,
    }
)


def to_ffmpeg_pixel_format(pixel_format: str) -> str:
    """요구사항 픽셀 포맷을 ffmpeg `-pix_fmt` 값으로 매핑한다.

    Args:
        pixel_format: `bgr8` / `rgb8` / `bgra8` / `rgba8` / `mono8` 중 하나.

    Returns:
        ffmpeg 가 이해하는 픽셀 포맷 문자열 (`bgr24`, `rgb24`, `bgra`, `rgba`, `gray`).

    Raises:
        ValueError: 지원하지 않는 픽셀 포맷이 전달된 경우.
    """
    try:
        return _ENCODING_MAP[pixel_format]
    except KeyError as exc:
        raise ValueError(
            f"unsupported pixel_format: {pixel_format!r}; "
            f"expected one of {sorted(SUPPORTED_ENCODINGS)}"
        ) from exc


def build_input_args(
    *,
    pixel_format: str,
    width: int,
    height: int,
    fps: int,
    input_format: str = INPUT_RAWVIDEO,
) -> list[str]:
    """ffmpeg 공통 입력부 인자를 생성한다.

    `-n` 은 출력 파일이 이미 존재할 경우 덮어쓰지 않도록 한다 (Python 레벨
    선검증과 함께 TOCTOU 경쟁 조건을 차단).

    **`input_format="mjpeg"` 는 `-pix_fmt` 와 `-s` 를 내지 않는다.** 픽셀 포맷과
    해상도가 JPEG 헤더 안에 있으므로 ffmpeg 이 스스로 읽는다. 여기서 `-s` 를 주면
    실제 프레임과 어긋날 때 조용히 깨진 영상이 나온다.

    Args:
        pixel_format: `bgr8` / `rgb8` / `bgra8` / `rgba8` / `mono8`.
            `input_format="mjpeg"` 에서는 쓰이지 않는다.
        width: 프레임 너비(픽셀), 양의 정수.
        height: 프레임 높이(픽셀), 양의 정수.
            `input_format="mjpeg"` 에서는 쓰이지 않는다.
        fps: Constant frame rate 값, 양의 정수.
        input_format: `rawvideo`(기본) 또는 `mjpeg`.

    Returns:
        ffmpeg 입력부 인자 리스트. `-i -` 로 stdin 파이프 입력을 지정한다.

    Raises:
        ValueError: 해상도/fps 가 양의 정수가 아니거나 pixel_format ·
            input_format 이 지원되지 않는 경우.
    """
    validate_input_format(input_format)
    if fps <= 0:
        raise ValueError(f"invalid fps: {fps}; must be positive")

    if input_format == INPUT_MJPEG:
        # 해상도는 JPEG 헤더에서 읽힌다 — 검증만 하고 인자로는 넘기지 않는다.
        # (호출자가 메타데이터·로그에 쓰는 값이라 유효성은 여전히 확인한다)
        if width <= 0 or height <= 0:
            raise ValueError(
                f"invalid resolution: width={width}, height={height}; must be positive")
        return [
            "-n",
            "-f", INPUT_MJPEG,
            "-framerate", str(fps),
            "-i", "-",
        ]

    if width <= 0 or height <= 0:
        raise ValueError(f"invalid resolution: width={width}, height={height}; must be positive")

    return [
        "-n",
        "-f", INPUT_RAWVIDEO,
        "-pix_fmt", to_ffmpeg_pixel_format(pixel_format),
        "-s", f"{width}x{height}",
        "-framerate", str(fps),
        "-i", "-",
    ]


def build_libx264_args(
    *,
    bitrate: str,
    gop_size: int,
    preset: str,
) -> list[str]:
    """CPU 인코더(libx264) 출력 옵션을 생성한다.

    `-pix_fmt yuv420p` 는 일반 플레이어 호환성을 위해 고정한다.
    """
    _validate_bitrate(bitrate)
    _validate_gop_size(gop_size)
    _validate_non_empty(preset, field="preset")
    return [
        "-c:v", CODEC_LIBX264,
        "-preset", preset,
        "-pix_fmt", "yuv420p",
        "-b:v", bitrate,
        "-g", str(gop_size),
    ]


def build_h264_nvenc_args(
    *,
    bitrate: str,
    gop_size: int,
) -> list[str]:
    """NVIDIA NVENC(h264_nvenc) 출력 옵션을 생성한다.

    preset 은 NVENC 고유 표기인 `p4` (균형) 를 기본으로 사용한다.
    """
    _validate_bitrate(bitrate)
    _validate_gop_size(gop_size)
    return [
        "-c:v", CODEC_H264_NVENC,
        "-preset", "p4",
        "-b:v", bitrate,
        "-g", str(gop_size),
    ]


def build_h264_qsv_args(
    *,
    bitrate: str,
    gop_size: int,
) -> list[str]:
    """Intel QuickSync(h264_qsv) 출력 옵션을 생성한다."""
    _validate_bitrate(bitrate)
    _validate_gop_size(gop_size)
    return [
        "-init_hw_device", "qsv",
        "-c:v", CODEC_H264_QSV,
        "-b:v", bitrate,
        "-g", str(gop_size),
    ]


def build_h264_vaapi_args(
    *,
    bitrate: str,
    vaapi_device: str,
) -> list[str]:
    """VAAPI(h264_vaapi) 출력 옵션을 생성한다.

    입력 raw 프레임을 GPU 로 업로드하기 위해 `format=nv12,hwupload` 필터를
    강제한다. VAAPI 는 GOP 크기를 내부적으로 추정하므로 `-g` 는 생략한다.
    """
    _validate_bitrate(bitrate)
    _validate_non_empty(vaapi_device, field="vaapi_device")
    return [
        "-vaapi_device", vaapi_device,
        "-vf", "format=nv12,hwupload",
        "-c:v", CODEC_H264_VAAPI,
        "-b:v", bitrate,
    ]


# 무손실 raw 녹화에서 ffmpeg 에 넘길 출력 픽셀 포맷. **입력과 같은 색 표현을 유지해
# 변환 자체를 없애는 것이 목적이다** — RGB 를 yuv420p 로 바꾸면 색 해상도가 1/4 로
# 줄고 반올림 오차가 생기는데, 그것은 녹화 뒤에 되돌릴 수 없다. 알파는 버린다(RGB 는
# 그대로다). 흑백은 `libx264rgb` 가 받지 않아 `libx264` 의 `gray` 로 담는다.
_LOSSLESS_RAW_TARGETS: Final[dict[str, tuple[str, str]]] = {
    "bgr8": (CODEC_LIBX264RGB, "bgr24"),
    "rgb8": (CODEC_LIBX264RGB, "rgb24"),
    "bgra8": (CODEC_LIBX264RGB, "bgr0"),
    "rgba8": (CODEC_LIBX264RGB, "bgr0"),
    "mono8": (CODEC_LIBX264, "gray"),
}


def lossless_codec(*, input_format: str, pixel_format: str) -> str:
    """무손실 녹화에서 실제로 쓰이는 코덱 이름을 돌려준다.

    `mjpeg` 입력은 JPEG 을 다시 인코딩하지 않고 그대로 담으므로 `mjpeg` 이고,
    raw 입력은 픽셀 포맷에 따라 `libx264rgb` 또는 `libx264`(흑백) 다.

    Raises:
        ValueError: 지원하지 않는 input_format · pixel_format 인 경우.
    """
    validate_input_format(input_format)
    if input_format == INPUT_MJPEG:
        return CODEC_MJPEG
    return _lossless_raw_target(pixel_format)[0]


def build_lossless_output_args(
    *,
    input_format: str,
    pixel_format: str,
    gop_size: int,
    preset: str,
) -> list[str]:
    """받은 이미지를 **추가 손실 없이** 담는 출력 옵션을 생성한다.

    * ``mjpeg`` — `-c:v copy`. 시뮬레이터가 보낸 JPEG 바이트가 컨테이너 패킷에
      그대로 들어간다(디코드도 인코드도 없다). GOP · preset 은 쓰이지 않는다.
    * ``rawvideo`` — x264 무손실(`-qp 0`). 컬러는 `libx264rgb` 로 RGB 를 그대로
      담아 색공간 변환이 없고, 흑백은 `libx264 -pix_fmt gray` 다. 이 출력은
      `High 4:4:4 Predictive` 프로파일이라 ffmpeg · PyAV · OpenCV 는 읽지만
      브라우저 · 하드웨어 디코더는 재생하지 못할 수 있다.

    Raises:
        ValueError: 지원하지 않는 input_format · pixel_format 이거나 GOP · preset
            이 유효하지 않은 경우.
    """
    validate_input_format(input_format)
    if input_format == INPUT_MJPEG:
        return ["-c:v", "copy"]
    codec, out_pix_fmt = _lossless_raw_target(pixel_format)
    _validate_gop_size(gop_size)
    _validate_non_empty(preset, field="preset")
    return [
        "-c:v", codec,
        "-preset", preset,
        "-qp", "0",
        "-pix_fmt", out_pix_fmt,
        "-g", str(gop_size),
    ]


def _lossless_raw_target(pixel_format: str) -> tuple[str, str]:
    """raw 픽셀 포맷 → (코덱, 출력 `-pix_fmt`)."""
    try:
        return _LOSSLESS_RAW_TARGETS[pixel_format]
    except KeyError as exc:
        raise ValueError(
            f"unsupported pixel_format for lossless recording: {pixel_format!r}; "
            f"expected one of {sorted(_LOSSLESS_RAW_TARGETS)}"
        ) from exc


def build_output_args(
    *,
    codec: str,
    bitrate: str,
    gop_size: int,
    preset: str = "medium",
    vaapi_device: str = "/dev/dri/renderD128",
) -> list[str]:
    """선택된 코덱에 해당하는 출력 옵션 리스트를 생성한다.

    Args:
        codec: `libx264` / `h264_nvenc` / `h264_qsv` / `h264_vaapi` 중 하나.
        bitrate: ffmpeg `-b:v` 값 (예: `"4M"`).
        gop_size: GOP 크기 (I-frame 간격). vaapi 에서는 무시된다.
        preset: libx264 preset (그 외 코덱에서는 무시).
        vaapi_device: VAAPI 디바이스 경로 (vaapi 외 코덱에서는 무시).

    Returns:
        해당 코덱에 대한 ffmpeg 출력 옵션 리스트.

    Raises:
        ValueError: 지원하지 않는 codec 이 전달된 경우.
    """
    if codec == CODEC_LIBX264:
        return build_libx264_args(bitrate=bitrate, gop_size=gop_size, preset=preset)
    if codec == CODEC_H264_NVENC:
        return build_h264_nvenc_args(bitrate=bitrate, gop_size=gop_size)
    if codec == CODEC_H264_QSV:
        return build_h264_qsv_args(bitrate=bitrate, gop_size=gop_size)
    if codec == CODEC_H264_VAAPI:
        return build_h264_vaapi_args(bitrate=bitrate, vaapi_device=vaapi_device)
    raise ValueError(
        f"unsupported codec: {codec!r}; expected one of {sorted(SUPPORTED_CODECS)}"
    )


def build_ffmpeg_command(
    *,
    ffmpeg_binary: str = "ffmpeg",
    pixel_format: str,
    width: int,
    height: int,
    fps: int,
    codec: str,
    bitrate: str,
    gop_size: int,
    output_path: str,
    preset: str = "medium",
    vaapi_device: str = "/dev/dri/renderD128",
    input_format: str = INPUT_RAWVIDEO,
    lossless: bool = False,
) -> list[str]:
    """완전한 ffmpeg 실행 커맨드 리스트를 생성한다.

    구성: `[binary] -hide_banner [입력부] [출력부] -movflags +faststart [경로]`

    Args:
        ffmpeg_binary: ffmpeg 실행 파일 경로/이름.
        pixel_format: 입력 픽셀 포맷 (`bgr8`/`rgb8`/`bgra8`/`rgba8`/`mono8`).
        width: 프레임 너비.
        height: 프레임 높이.
        fps: 입력 CFR.
        codec: 선택된 코덱.
        bitrate: `-b:v` 값.
        gop_size: GOP 크기.
        output_path: 출력 MP4 파일 경로.
        preset: libx264 preset (그 외 코덱에서는 무시).
        vaapi_device: VAAPI 디바이스 경로 (vaapi 외 코덱에서는 무시).
        input_format: `rawvideo`(기본) 또는 `mjpeg`.
        lossless: True 면 `codec` · `bitrate` 를 쓰지 않고
            `build_lossless_output_args` 로 받은 이미지를 추가 손실 없이 담는다.

    Returns:
        subprocess 에 바로 넘길 수 있는 인자 리스트.
    """
    _validate_non_empty(ffmpeg_binary, field="ffmpeg_binary")
    _validate_non_empty(output_path, field="output_path")

    cmd: list[str] = [ffmpeg_binary, "-hide_banner"]
    cmd.extend(
        build_input_args(
            pixel_format=pixel_format,
            width=width,
            height=height,
            fps=fps,
            input_format=input_format,
        )
    )
    if lossless:
        cmd.extend(build_lossless_output_args(
            input_format=input_format, pixel_format=pixel_format, gop_size=gop_size,
            preset=preset))
    else:
        cmd.extend(
            build_output_args(
                codec=codec,
                bitrate=bitrate,
                gop_size=gop_size,
                preset=preset,
                vaapi_device=vaapi_device,
            )
        )
    cmd.extend(["-movflags", "+faststart", output_path])
    return cmd


def _validate_bitrate(bitrate: str) -> None:
    """비트레이트 문자열이 비어있지 않은지만 검사한다.

    ffmpeg 가 다양한 포맷(`4M`, `4000k`, `4000000`)을 지원하므로 엄밀한
    파싱은 하지 않고 빈 문자열만 차단한다.
    """
    _validate_non_empty(bitrate, field="bitrate")


def _validate_gop_size(gop_size: int) -> None:
    """GOP 크기는 양의 정수여야 한다."""
    if not isinstance(gop_size, int) or gop_size <= 0:
        raise ValueError(f"invalid gop_size: {gop_size}; must be a positive int")


def _validate_non_empty(value: str, *, field: str) -> None:
    """필수 문자열 필드가 비어있지 않은지 검사한다."""
    if not isinstance(value, str) or not value:
        raise ValueError(f"invalid {field}: must be a non-empty string")
