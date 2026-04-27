#!/usr/bin/env python3

from __future__ import annotations

import pytest

from rdfp.recorder.ffmpeg_command import (
    CODEC_H264_NVENC,
    CODEC_H264_QSV,
    CODEC_H264_VAAPI,
    CODEC_LIBX264,
    SUPPORTED_CODECS,
    SUPPORTED_ENCODINGS,
    build_ffmpeg_command,
    build_h264_nvenc_args,
    build_h264_qsv_args,
    build_h264_vaapi_args,
    build_input_args,
    build_libx264_args,
    build_output_args,
    to_ffmpeg_pixel_format,
)


# ---------- pixel_format 매핑 -------------------------------------------------


@pytest.mark.parametrize(
    ("pixel_format", "expected"),
    [
        ("bgr8", "bgr24"),
        ("rgb8", "rgb24"),
        ("bgra8", "bgra"),
        ("rgba8", "rgba"),
        ("mono8", "gray"),
    ],
)
def test_pixel_format_mapping(pixel_format: str, expected: str) -> None:
    assert to_ffmpeg_pixel_format(pixel_format) == expected


def test_pixel_format_invalid_raises() -> None:
    with pytest.raises(ValueError, match="unsupported pixel_format"):
        to_ffmpeg_pixel_format("yuv420p")


def test_supported_encodings_set() -> None:
    assert SUPPORTED_ENCODINGS == frozenset({"bgr8", "rgb8", "bgra8", "rgba8", "mono8"})


# ---------- 입력부 -----------------------------------------------------------


def test_build_input_args_basic() -> None:
    args = build_input_args(pixel_format="bgr8", width=640, height=480, fps=30)
    # 필수 토큰 존재 및 값 검증
    assert "-n" in args
    assert args[args.index("-f") + 1] == "rawvideo"
    assert args[args.index("-pix_fmt") + 1] == "bgr24"
    assert args[args.index("-s") + 1] == "640x480"
    assert args[args.index("-framerate") + 1] == "30"
    assert args[args.index("-i") + 1] == "-"


def test_build_input_args_mono() -> None:
    args = build_input_args(pixel_format="mono8", width=320, height=240, fps=60)
    assert args[args.index("-pix_fmt") + 1] == "gray"
    assert args[args.index("-s") + 1] == "320x240"
    assert args[args.index("-framerate") + 1] == "60"


@pytest.mark.parametrize(
    ("width", "height"),
    [(0, 480), (640, 0), (-1, 480), (640, -1)],
)
def test_build_input_args_rejects_invalid_resolution(width: int, height: int) -> None:
    with pytest.raises(ValueError, match="invalid resolution"):
        build_input_args(pixel_format="bgr8", width=width, height=height, fps=30)


@pytest.mark.parametrize("fps", [0, -1])
def test_build_input_args_rejects_invalid_fps(fps: int) -> None:
    with pytest.raises(ValueError, match="invalid fps"):
        build_input_args(pixel_format="bgr8", width=640, height=480, fps=fps)


# ---------- libx264 출력부 ---------------------------------------------------


def test_build_libx264_args() -> None:
    args = build_libx264_args(bitrate="4M", gop_size=60, preset="medium")
    assert args[args.index("-c:v") + 1] == "libx264"
    assert args[args.index("-preset") + 1] == "medium"
    assert args[args.index("-pix_fmt") + 1] == "yuv420p"
    assert args[args.index("-b:v") + 1] == "4M"
    assert args[args.index("-g") + 1] == "60"


def test_build_libx264_args_rejects_empty_bitrate() -> None:
    with pytest.raises(ValueError, match="invalid bitrate"):
        build_libx264_args(bitrate="", gop_size=60, preset="medium")


def test_build_libx264_args_rejects_non_positive_gop() -> None:
    with pytest.raises(ValueError, match="invalid gop_size"):
        build_libx264_args(bitrate="4M", gop_size=0, preset="medium")


def test_build_libx264_args_rejects_empty_preset() -> None:
    with pytest.raises(ValueError, match="invalid preset"):
        build_libx264_args(bitrate="4M", gop_size=60, preset="")


# ---------- h264_nvenc 출력부 -----------------------------------------------


def test_build_h264_nvenc_args() -> None:
    args = build_h264_nvenc_args(bitrate="8M", gop_size=120)
    assert args[args.index("-c:v") + 1] == "h264_nvenc"
    assert args[args.index("-preset") + 1] == "p4"
    assert args[args.index("-b:v") + 1] == "8M"
    assert args[args.index("-g") + 1] == "120"


# ---------- h264_qsv 출력부 -------------------------------------------------


def test_build_h264_qsv_args() -> None:
    args = build_h264_qsv_args(bitrate="6M", gop_size=90)
    assert args[args.index("-init_hw_device") + 1] == "qsv"
    assert args[args.index("-c:v") + 1] == "h264_qsv"
    assert args[args.index("-b:v") + 1] == "6M"
    assert args[args.index("-g") + 1] == "90"


# ---------- h264_vaapi 출력부 -----------------------------------------------


def test_build_h264_vaapi_args() -> None:
    args = build_h264_vaapi_args(bitrate="5M", vaapi_device="/dev/dri/renderD128")
    assert args[args.index("-vaapi_device") + 1] == "/dev/dri/renderD128"
    assert args[args.index("-vf") + 1] == "format=nv12,hwupload"
    assert args[args.index("-c:v") + 1] == "h264_vaapi"
    assert args[args.index("-b:v") + 1] == "5M"
    # VAAPI 는 GOP 크기를 지정하지 않는다
    assert "-g" not in args


def test_build_h264_vaapi_args_rejects_empty_device() -> None:
    with pytest.raises(ValueError, match="invalid vaapi_device"):
        build_h264_vaapi_args(bitrate="5M", vaapi_device="")


# ---------- 출력부 디스패처 --------------------------------------------------


@pytest.mark.parametrize(
    "codec",
    [CODEC_LIBX264, CODEC_H264_NVENC, CODEC_H264_QSV, CODEC_H264_VAAPI],
)
def test_build_output_args_dispatches_by_codec(codec: str) -> None:
    args = build_output_args(codec=codec, bitrate="4M", gop_size=60)
    # 모든 코덱에서 -c:v 다음에 codec 이름이 나와야 한다
    assert args[args.index("-c:v") + 1] == codec


def test_build_output_args_rejects_unsupported_codec() -> None:
    with pytest.raises(ValueError, match="unsupported codec"):
        build_output_args(codec="h265_nvenc", bitrate="4M", gop_size=60)


def test_supported_codecs_set() -> None:
    assert SUPPORTED_CODECS == frozenset(
        {CODEC_LIBX264, CODEC_H264_NVENC, CODEC_H264_QSV, CODEC_H264_VAAPI}
    )


# ---------- 전체 커맨드 빌더 -------------------------------------------------


def test_build_ffmpeg_command_libx264_full() -> None:
    cmd = build_ffmpeg_command(
        pixel_format="bgr8",
        width=1280,
        height=720,
        fps=30,
        codec=CODEC_LIBX264,
        bitrate="4M",
        gop_size=60,
        output_path="/tmp/out.mp4",
    )
    # 바이너리 + hide_banner 로 시작
    assert cmd[0] == "ffmpeg"
    assert cmd[1] == "-hide_banner"
    # 덮어쓰기 금지 플래그
    assert "-n" in cmd
    assert "-y" not in cmd
    # 입력부
    assert cmd[cmd.index("-pix_fmt") + 1] == "bgr24"
    assert cmd[cmd.index("-s") + 1] == "1280x720"
    assert cmd[cmd.index("-framerate") + 1] == "30"
    # 출력부 (libx264 의 -pix_fmt yuv420p 는 입력부 bgr24 이후에 위치)
    assert cmd[cmd.index("-c:v") + 1] == CODEC_LIBX264
    # 공통 말미
    assert cmd[-3:] == ["-movflags", "+faststart", "/tmp/out.mp4"]


def test_build_ffmpeg_command_passes_custom_binary() -> None:
    cmd = build_ffmpeg_command(
        ffmpeg_binary="/opt/bin/ffmpeg",
        pixel_format="rgb8",
        width=640,
        height=480,
        fps=25,
        codec=CODEC_LIBX264,
        bitrate="2M",
        gop_size=50,
        output_path="/var/tmp/out.mp4",
    )
    assert cmd[0] == "/opt/bin/ffmpeg"


def test_build_ffmpeg_command_nvenc_end_to_end() -> None:
    cmd = build_ffmpeg_command(
        pixel_format="bgr8",
        width=1920,
        height=1080,
        fps=30,
        codec=CODEC_H264_NVENC,
        bitrate="8M",
        gop_size=60,
        output_path="/tmp/nvenc.mp4",
    )
    assert cmd[-3:] == ["-movflags", "+faststart", "/tmp/nvenc.mp4"]
    assert cmd[cmd.index("-c:v") + 1] == CODEC_H264_NVENC


def test_build_ffmpeg_command_vaapi_uses_device_arg() -> None:
    cmd = build_ffmpeg_command(
        pixel_format="bgr8",
        width=640,
        height=480,
        fps=30,
        codec=CODEC_H264_VAAPI,
        bitrate="4M",
        gop_size=60,
        output_path="/tmp/vaapi.mp4",
        vaapi_device="/dev/dri/renderD129",
    )
    assert cmd[cmd.index("-vaapi_device") + 1] == "/dev/dri/renderD129"
    assert cmd[cmd.index("-vf") + 1] == "format=nv12,hwupload"


def test_build_ffmpeg_command_rejects_empty_output_path() -> None:
    with pytest.raises(ValueError, match="invalid output_path"):
        build_ffmpeg_command(
            pixel_format="bgr8",
            width=640,
            height=480,
            fps=30,
            codec=CODEC_LIBX264,
            bitrate="4M",
            gop_size=60,
            output_path="",
        )
