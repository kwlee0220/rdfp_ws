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

# 지원 코덱 상수 (probe / 커맨드 빌더 공용)
CODEC_LIBX264: Final[str] = "libx264"
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
) -> list[str]:
    """ffmpeg 공통 입력부 인자를 생성한다.

    `-n` 은 출력 파일이 이미 존재할 경우 덮어쓰지 않도록 한다 (Python 레벨
    선검증과 함께 TOCTOU 경쟁 조건을 차단).

    Args:
        pixel_format: `bgr8` / `rgb8` / `bgra8` / `rgba8` / `mono8`.
        width: 프레임 너비(픽셀), 양의 정수.
        height: 프레임 높이(픽셀), 양의 정수.
        fps: Constant frame rate 값, 양의 정수.

    Returns:
        ffmpeg 입력부 인자 리스트. `-i -` 로 stdin 파이프 입력을 지정한다.

    Raises:
        ValueError: 해상도/fps 가 양의 정수가 아니거나 pixel_format 이 지원되지
            않는 경우.
    """
    if width <= 0 or height <= 0:
        raise ValueError(f"invalid resolution: width={width}, height={height}; must be positive")
    if fps <= 0:
        raise ValueError(f"invalid fps: {fps}; must be positive")

    return [
        "-n",
        "-f", "rawvideo",
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
        )
    )
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
