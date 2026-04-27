#!/usr/bin/env python3

from __future__ import annotations

from typing import Final

import logging
import subprocess

from .exceptions import EncoderUnavailableError
from .ffmpeg_command import (
    CODEC_H264_NVENC,
    CODEC_H264_QSV,
    CODEC_H264_VAAPI,
    CODEC_LIBX264,
)


# 하드웨어 인코더 기본 우선순위 (plan.md §6)
DEFAULT_HW_CODEC_PRIORITY: Final[tuple[str, ...]] = (
    CODEC_H264_NVENC,
    CODEC_H264_QSV,
    CODEC_H264_VAAPI,
)

# `encoder_mode` 파라미터 값
ENCODER_MODE_AUTO: Final[str] = "auto"
ENCODER_MODE_CPU: Final[str] = "cpu"
ENCODER_MODE_GPU: Final[str] = "gpu"

VALID_ENCODER_MODES: Final[frozenset[str]] = frozenset(
    {ENCODER_MODE_AUTO, ENCODER_MODE_CPU, ENCODER_MODE_GPU}
)

# probe subprocess 호출의 기본 타임아웃 (초)
PROBE_TIMEOUT_SEC: Final[float] = 10.0


def parse_build_encoders(output: str) -> set[str]:
    """`ffmpeg -encoders` 출력에서 **비디오** 인코더 이름 집합을 추출한다.

    ffmpeg 출력 형식 예시::

        Encoders:
         V..... = Video
         A..... = Audio
         ------
         V....D libx264              libx264 H.264 / AVC / ...
         V....D h264_nvenc           NVIDIA NVENC H.264 encoder
         A....D aac                  AAC (Advanced Audio Coding)

    "V" 로 시작하는 라인만 수집하며 "V..... = Video" 같은 범례 라인은 제외한다.
    """
    encoders: set[str] = set()
    for raw_line in output.splitlines():
        stripped = raw_line.strip()
        if not stripped or stripped[0] != "V":
            continue
        parts = stripped.split()
        if len(parts) < 2:
            continue
        flags = parts[0]
        codec_name = parts[1]
        # 범례 라인("V..... = Video") 제외
        if codec_name == "=" or not all(ch in ".VADSFXBD" for ch in flags):
            continue
        encoders.add(codec_name)
    return encoders


def list_build_encoders(ffmpeg_binary: str = "ffmpeg") -> set[str]:
    """`ffmpeg -hide_banner -encoders` 를 실행하여 빌드에 포함된 비디오 인코더 이름을 반환한다.

    ffmpeg 실행 자체가 실패하는 경우(바이너리 없음, 타임아웃 등)에는 빈
    집합을 반환한다. 호출자는 빈 집합을 "사용 가능한 인코더 없음" 으로
    취급할 수 있다.
    """
    try:
        result = subprocess.run(
            [ffmpeg_binary, "-hide_banner", "-encoders"],
            capture_output=True,
            text=True,
            check=False,
            timeout=PROBE_TIMEOUT_SEC,
        )
    except (FileNotFoundError, OSError, subprocess.TimeoutExpired):
        return set()
    if result.returncode != 0:
        return set()
    return parse_build_encoders(result.stdout)


def build_probe_command(
    *,
    ffmpeg_binary: str,
    codec: str,
    vaapi_device: str,
) -> list[str]:
    """특정 인코더의 런타임 작동 여부를 확인하기 위한 ffmpeg 커맨드를 생성한다.

    `lavfi color` 소스에서 1 프레임을 생성해 해당 인코더로 인코딩한 뒤
    `null` 머그로 출력한다. 성공 시 exit code 0, 드라이버/디바이스 부재 시
    non-zero.
    """
    base: list[str] = [
        ffmpeg_binary,
        "-hide_banner",
        "-loglevel", "error",
    ]
    lavfi_source = [
        "-f", "lavfi",
        "-i", "color=black:s=64x64:rate=30",
        "-frames:v", "1",
    ]
    null_output = ["-f", "null", "-"]

    if codec == CODEC_H264_VAAPI:
        return [
            *base,
            "-vaapi_device", vaapi_device,
            *lavfi_source,
            "-vf", "format=nv12,hwupload",
            "-c:v", CODEC_H264_VAAPI,
            *null_output,
        ]
    if codec == CODEC_H264_QSV:
        return [
            *base,
            "-init_hw_device", "qsv=hw",
            "-filter_hw_device", "hw",
            *lavfi_source,
            "-vf", "format=nv12,hwupload",
            "-c:v", CODEC_H264_QSV,
            *null_output,
        ]
    # libx264, h264_nvenc 등은 추가 hw 셋업 없이 직접 호출
    return [
        *base,
        *lavfi_source,
        "-c:v", codec,
        *null_output,
    ]


def probe_runtime_encoder(
    *,
    ffmpeg_binary: str,
    codec: str,
    vaapi_device: str = "/dev/dri/renderD128",
    logger: logging.Logger | None = None,
) -> bool:
    """1 프레임 더미 인코딩을 실행하여 인코더가 실제로 동작하는지 확인한다.

    Args:
        ffmpeg_binary: ffmpeg 실행 파일.
        codec: 테스트할 인코더 이름.
        vaapi_device: VAAPI 디바이스 경로 (vaapi 외 코덱에서는 무시).
        logger: 진단용 로거. 생략 시 모듈 로거 사용.

    Returns:
        `True` 이면 런타임 인코딩 성공. 드라이버/디바이스 부재, ffmpeg 부재,
        타임아웃 등의 경우 `False`.
    """
    log = logger or logging.getLogger(__name__)
    cmd = build_probe_command(
        ffmpeg_binary=ffmpeg_binary,
        codec=codec,
        vaapi_device=vaapi_device,
    )
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            check=False,
            timeout=PROBE_TIMEOUT_SEC,
        )
    except (FileNotFoundError, OSError, subprocess.TimeoutExpired) as exc:
        log.debug("probe: %s runtime check raised %s", codec, type(exc).__name__)
        return False

    if result.returncode == 0:
        return True
    log.debug(
        "probe: %s runtime check failed rc=%s stderr=%s",
        codec,
        result.returncode,
        (result.stderr or "").strip().splitlines()[-1:] or [""],
    )
    return False


def probe_available_hw_encoders(
    *,
    ffmpeg_binary: str = "ffmpeg",
    vaapi_device: str = "/dev/dri/renderD128",
    candidates: tuple[str, ...] = DEFAULT_HW_CODEC_PRIORITY,
    logger: logging.Logger | None = None,
) -> list[str]:
    """빌드 포함 여부와 런타임 테스트를 모두 통과한 HW 인코더를 우선순위 순으로 반환한다.

    Args:
        ffmpeg_binary: ffmpeg 실행 파일.
        vaapi_device: VAAPI 디바이스 경로.
        candidates: 시도할 HW 인코더 목록 (우선순위 순).
        logger: 진단용 로거.

    Returns:
        실제로 사용 가능한 HW 인코더 리스트. 사용 가능한 것이 없으면 빈 리스트.
    """
    log = logger or logging.getLogger(__name__)
    build_set = list_build_encoders(ffmpeg_binary)
    available: list[str] = []
    for codec in candidates:
        if codec not in build_set:
            log.debug("probe: %s not in ffmpeg build", codec)
            continue
        if probe_runtime_encoder(
            ffmpeg_binary=ffmpeg_binary,
            codec=codec,
            vaapi_device=vaapi_device,
            logger=log,
        ):
            log.info("probe: %s runtime test passed", codec)
            available.append(codec)
    return available


def select_encoder(
    *,
    encoder_mode: str,
    ffmpeg_binary: str = "ffmpeg",
    preferred_hw_codec: str | None = None,
    vaapi_device: str = "/dev/dri/renderD128",
    logger: logging.Logger | None = None,
) -> str:
    """`encoder_mode` 파라미터에 따라 사용할 코덱을 하나 결정한다.

    동작 규칙 (plan.md §6):

    - `cpu` → subprocess 호출 없이 즉시 `libx264` 반환
    - `auto` → HW 인코더 probe, 성공한 첫 코덱 반환. 모두 실패 시 `libx264`
      로 fallback 하며 경고 로그를 남긴다
    - `gpu` → HW 인코더 probe, 성공한 첫 코덱 반환. 모두 실패 시
      `EncoderUnavailableError` 발생

    `preferred_hw_codec` 이 지정되면 해당 코덱만 probe 대상이 된다.

    Args:
        encoder_mode: `auto` / `cpu` / `gpu` 중 하나.
        ffmpeg_binary: ffmpeg 실행 파일.
        preferred_hw_codec: 지정 시 해당 HW 코덱만 probe. `DEFAULT_HW_CODEC_PRIORITY`
            에 포함된 값이어야 한다.
        vaapi_device: VAAPI 디바이스 경로.
        logger: 진단용 로거.

    Returns:
        선택된 코덱 이름 (`libx264` / `h264_nvenc` / `h264_qsv` / `h264_vaapi`).

    Raises:
        ValueError: `encoder_mode` 또는 `preferred_hw_codec` 값이 유효하지 않은 경우.
        EncoderUnavailableError: `encoder_mode="gpu"` 이지만 사용 가능한 HW 인코더가
            하나도 없는 경우.
    """
    if encoder_mode not in VALID_ENCODER_MODES:
        raise ValueError(
            f"invalid encoder_mode: {encoder_mode!r}; "
            f"expected one of {sorted(VALID_ENCODER_MODES)}"
        )

    log = logger or logging.getLogger(__name__)

    if encoder_mode == ENCODER_MODE_CPU:
        log.info("selected encoder: %s (mode=cpu)", CODEC_LIBX264)
        return CODEC_LIBX264

    # auto / gpu 공통: 후보 목록 결정
    if preferred_hw_codec is not None:
        if preferred_hw_codec not in DEFAULT_HW_CODEC_PRIORITY:
            raise ValueError(
                f"invalid preferred_hw_codec: {preferred_hw_codec!r}; "
                f"expected one of {list(DEFAULT_HW_CODEC_PRIORITY)}"
            )
        candidates: tuple[str, ...] = (preferred_hw_codec,)
    else:
        candidates = DEFAULT_HW_CODEC_PRIORITY

    available = probe_available_hw_encoders(
        ffmpeg_binary=ffmpeg_binary,
        vaapi_device=vaapi_device,
        candidates=candidates,
        logger=log,
    )

    if available:
        codec = available[0]
        log.info("selected encoder: %s (mode=%s)", codec, encoder_mode)
        return codec

    if encoder_mode == ENCODER_MODE_GPU:
        raise EncoderUnavailableError(
            f"no working hardware encoder found; candidates={list(candidates)}"
        )

    # encoder_mode == "auto" + HW 실패: CPU fallback
    log.warning(
        "no hardware encoder available; falling back to %s (mode=auto)",
        CODEC_LIBX264,
    )
    return CODEC_LIBX264
