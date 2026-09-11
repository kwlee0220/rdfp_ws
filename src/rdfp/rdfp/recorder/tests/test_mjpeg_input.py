#!/usr/bin/env python3
"""`input_format="mjpeg"` — 압축 프레임을 그대로 ffmpeg 에 넘기는 경로 테스트.

시뮬레이터가 `sensor_msgs/CompressedImage`(JPEG) 로 보내기 시작해(2026-09-08 새 빌드)
추가한 경로다. **우리가 디코드하지 않는 것이 목적**이므로, 테스트도 "디코드 코드가
다시 끼어들지 못하게" 하는 데 초점을 둔다.
"""

from __future__ import annotations

import shutil
import subprocess

import pytest

from robot_control.types import InvalidFrameError
from rdfp.recorder.ffmpeg_command import (
    INPUT_MJPEG,
    INPUT_RAWVIDEO,
    SUPPORTED_INPUT_FORMATS,
    build_ffmpeg_command,
    build_input_args,
    validate_input_format,
)
from rdfp.recorder.ffmpeg_mp4_recorder import FFMpegMp4Recorder


def test_supported_input_formats():
    assert SUPPORTED_INPUT_FORMATS == {INPUT_RAWVIDEO, INPUT_MJPEG}
    assert validate_input_format(INPUT_MJPEG) == INPUT_MJPEG


def test_unknown_input_format_rejected():
    with pytest.raises(ValueError, match='unsupported input_format'):
        validate_input_format('h264')


def test_rawvideo_is_the_default():
    """기본값이 바뀌면 기존 스택 전부가 영향을 받는다 — 고정한다."""
    args = build_input_args(pixel_format='bgr8', width=640, height=480, fps=30)
    assert '-f' in args and args[args.index('-f') + 1] == INPUT_RAWVIDEO


def test_mjpeg_input_omits_pix_fmt_and_size():
    """**`-pix_fmt` 와 `-s` 를 내지 않는다.**

    JPEG 헤더가 픽셀 포맷과 해상도를 갖고 있으므로 ffmpeg 이 스스로 읽는다. 여기서
    `-s` 를 주면 실제 프레임과 어긋날 때 조용히 깨진 영상이 나온다.
    """
    args = build_input_args(pixel_format='bgr8', width=640, height=480, fps=10,
                            input_format=INPUT_MJPEG)
    assert args[args.index('-f') + 1] == INPUT_MJPEG
    assert '-pix_fmt' not in args
    assert '-s' not in args
    # CFR 유지를 위해 framerate 와 stdin 입력은 그대로 필요하다.
    assert args[args.index('-framerate') + 1] == '10'
    assert args[-2:] == ['-i', '-']


def test_mjpeg_input_still_validates_fps_and_resolution():
    """해상도는 인자로 안 나가지만 **유효성은 확인한다** — 메타데이터·로그에 쓰인다."""
    with pytest.raises(ValueError, match='invalid fps'):
        build_input_args(pixel_format='bgr8', width=640, height=480, fps=0,
                         input_format=INPUT_MJPEG)
    with pytest.raises(ValueError, match='invalid resolution'):
        build_input_args(pixel_format='bgr8', width=0, height=480, fps=10,
                         input_format=INPUT_MJPEG)


def test_full_command_carries_mjpeg_input_and_h264_output():
    """입력만 mjpeg 이고 **출력은 그대로 H.264** 다 — 데이터셋 쪽이 안 바뀌는 이유다."""
    cmd = build_ffmpeg_command(
        pixel_format='bgr8', width=640, height=480, fps=10, codec='libx264',
        bitrate='2M', gop_size=20, output_path='/tmp/out.mp4', input_format=INPUT_MJPEG)
    assert cmd[cmd.index('-f') + 1] == INPUT_MJPEG
    assert cmd[cmd.index('-c:v') + 1] == 'libx264'
    assert '-pix_fmt' in cmd, '출력 쪽 pix_fmt(yuv420p) 는 남아 있어야 한다'
    assert cmd[cmd.index('-pix_fmt') + 1] == 'yuv420p'


def _real_jpeg(width: int = 640, height: int = 480) -> bytes:
    """ffmpeg 이 실제로 디코드할 수 있는 JPEG 을 만든다."""
    import cv2
    import numpy as np
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:, : width // 2] = (0, 0, 200)
    ok, buf = cv2.imencode('.jpg', img)
    assert ok
    return buf.tobytes()


@pytest.fixture
def mjpeg_recorder(tmp_path):
    """RECORDING 상태의 mjpeg 레코더. 가드는 그 상태에서만 판정되므로 실제로 start 한다."""
    rec = FFMpegMp4Recorder(fps=10, resolution=(640, 480), input_format=INPUT_MJPEG,
                            encoder_mode='cpu', bitrate='1M')
    rec.start(str(tmp_path / 'out.mp4'))
    yield rec
    if rec.state == 'RECORDING':
        rec.stop()


@pytest.mark.skipif(shutil.which('ffmpeg') is None, reason='ffmpeg not installed')
def test_write_bytes_rejected_in_mjpeg_mode(mjpeg_recorder):
    """조용히 통과시키면 JPEG 이 raw 픽셀로 해석돼 깨진 영상이 되고, 원인을 못 찾는다."""
    with pytest.raises(InvalidFrameError, match='write_compressed'):
        mjpeg_recorder.write_bytes(b'\x00' * (640 * 480 * 3))


@pytest.mark.skipif(shutil.which('ffmpeg') is None, reason='ffmpeg not installed')
def test_write_ndarray_rejected_in_mjpeg_mode(mjpeg_recorder):
    import numpy as np
    with pytest.raises(InvalidFrameError, match='write_compressed'):
        mjpeg_recorder.write(np.zeros((480, 640, 3), dtype=np.uint8))


@pytest.mark.skipif(shutil.which('ffmpeg') is None, reason='ffmpeg not installed')
def test_empty_compressed_frame_rejected(mjpeg_recorder):
    with pytest.raises(InvalidFrameError, match='empty'):
        mjpeg_recorder.write_compressed(b'')


@pytest.mark.skipif(shutil.which('ffmpeg') is None, reason='ffmpeg not installed')
def test_write_compressed_rejected_on_raw_recorder(tmp_path):
    """기본(raw) 레코더에서 압축 프레임을 넘기면 거부한다 — 반대 방향 가드."""
    rec = FFMpegMp4Recorder(fps=10, resolution=(640, 480), encoder_mode='cpu')
    rec.start(str(tmp_path / 'raw.mp4'))
    try:
        with pytest.raises(InvalidFrameError, match="requires input_format='mjpeg'"):
            rec.write_compressed(_real_jpeg())
        # raw 경로는 그대로 동작해야 한다.
        rec.write_bytes(b'\x00' * (640 * 480 * 3))
    finally:
        rec.stop()


@pytest.mark.skipif(shutil.which('ffmpeg') is None or shutil.which('ffprobe') is None,
                    reason='ffmpeg/ffprobe not installed')
def test_compressed_frames_produce_h264_mp4(tmp_path):
    """**JPEG 을 그대로 넣어 H.264 MP4 가 나온다** — 이 경로의 존재 이유다.

    출력이 H.264 라야 데이터셋·replay 쪽(`Mp4ImageReplayer`)이 손대지 않고 동작한다.
    """
    out = tmp_path / 'mjpeg_in.mp4'
    rec = FFMpegMp4Recorder(fps=10, resolution=(640, 480), input_format=INPUT_MJPEG,
                            encoder_mode='cpu', bitrate='1M')
    rec.start(str(out))
    frame = _real_jpeg()
    for _ in range(10):
        rec.write_compressed(frame)
    rec.stop()

    assert out.exists() and out.stat().st_size > 0
    assert rec.frames_written == 10
    probe = subprocess.run(
        ['ffprobe', '-hide_banner', '-loglevel', 'error', '-select_streams', 'v',
         '-show_entries', 'stream=codec_name,width,height', '-of', 'csv=p=0', str(out)],
        capture_output=True, text=True, check=True).stdout.strip()
    assert probe.startswith('h264'), probe
    assert '640,480' in probe, probe
