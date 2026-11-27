#!/usr/bin/env python3
"""무손실 보관(`lossless=True`) 테스트.

받은 이미지를 녹화 단계에서 **더 잃지 않는다**는 것이 이 모드의 약속이다. 그래서
명령 인자만 보지 않고, 실제 ffmpeg 으로 녹화한 뒤 **되읽은 값이 넣은 값과 같은지**
를 확인한다 — raw 는 픽셀이 비트 단위로 같고, JPEG 은 컨테이너 패킷이 받은 바이트와
같다.
"""

from __future__ import annotations

import shutil

import numpy as np
import pytest

from rdfp.recorder.ffmpeg_command import (
    CODEC_LIBX264,
    CODEC_LIBX264RGB,
    CODEC_MJPEG,
    INPUT_MJPEG,
    INPUT_RAWVIDEO,
    build_ffmpeg_command,
    build_lossless_output_args,
    lossless_codec,
)
from rdfp.recorder.ffmpeg_mp4_recorder import FFMpegMp4Recorder

_NEEDS_FFMPEG = pytest.mark.skipif(shutil.which('ffmpeg') is None,
                                   reason='ffmpeg not installed')


# ---------- 명령 인자 -------------------------------------------------------

@pytest.mark.parametrize('pixel_format, codec, out_pix_fmt', [
    ('bgr8', CODEC_LIBX264RGB, 'bgr24'),
    ('rgb8', CODEC_LIBX264RGB, 'rgb24'),
    ('bgra8', CODEC_LIBX264RGB, 'bgr0'),
    ('rgba8', CODEC_LIBX264RGB, 'bgr0'),
    ('mono8', CODEC_LIBX264, 'gray'),
])
def test_raw_lossless_keeps_the_input_color_representation(pixel_format, codec, out_pix_fmt):
    """raw 는 `-qp 0` 이고 **yuv420p 로 바꾸지 않는다** — 바꾸면 색 해상도가 1/4 로 준다."""
    args = build_lossless_output_args(input_format=INPUT_RAWVIDEO, pixel_format=pixel_format,
                                      gop_size=20, preset='medium')
    assert args[args.index('-c:v') + 1] == codec
    assert args[args.index('-qp') + 1] == '0'
    assert args[args.index('-pix_fmt') + 1] == out_pix_fmt
    assert '-b:v' not in args
    assert lossless_codec(input_format=INPUT_RAWVIDEO, pixel_format=pixel_format) == codec


def test_mjpeg_lossless_is_a_stream_copy():
    """JPEG 은 다시 인코딩하지 않는다 — 디코드 · 인코드 옵션이 하나도 없어야 한다."""
    args = build_lossless_output_args(input_format=INPUT_MJPEG, pixel_format='bgr8',
                                      gop_size=20, preset='medium')
    assert args == ['-c:v', 'copy']
    assert lossless_codec(input_format=INPUT_MJPEG, pixel_format='bgr8') == CODEC_MJPEG


def test_unknown_pixel_format_rejected_for_lossless():
    with pytest.raises(ValueError, match='lossless'):
        build_lossless_output_args(input_format=INPUT_RAWVIDEO, pixel_format='yuv422',
                                   gop_size=20, preset='medium')


def test_full_command_ignores_codec_and_bitrate_when_lossless():
    """`codec` · `bitrate` 가 넘어와도 무손실 출력이 이긴다 — 손실 인코더가 섞이면 안 된다."""
    cmd = build_ffmpeg_command(
        pixel_format='rgb8', width=640, height=480, fps=10, codec='h264_nvenc',
        bitrate='4M', gop_size=20, output_path='/tmp/out.mp4', lossless=True)
    assert cmd[cmd.index('-c:v') + 1] == CODEC_LIBX264RGB
    assert '-b:v' not in cmd
    assert cmd[-1] == '/tmp/out.mp4'


def test_default_is_still_lossy():
    """기본값이 바뀌면 `lossless` 를 안 넘기는 호출자(서비스 구동 레코더 등)가 달라진다."""
    cmd = build_ffmpeg_command(
        pixel_format='bgr8', width=640, height=480, fps=10, codec='libx264',
        bitrate='4M', gop_size=20, output_path='/tmp/out.mp4')
    assert cmd[cmd.index('-c:v') + 1] == 'libx264'
    assert cmd[cmd.index('-b:v') + 1] == '4M'


# ---------- 레코더 ----------------------------------------------------------

def test_lossless_recorder_skips_the_hardware_probe(monkeypatch):
    """무손실은 코덱이 입력으로 정해진다 — 하드웨어 인코더를 고르면 약속이 깨진다."""
    import rdfp.recorder.ffmpeg_mp4_recorder as mod

    def _fail(**_kwargs):
        raise AssertionError('select_encoder must not be called for lossless recording')

    monkeypatch.setattr(mod, 'select_encoder', _fail)
    rec = FFMpegMp4Recorder(fps=10, resolution=(64, 48), pixel_format='rgb8',
                            encoder_mode='gpu', lossless=True)
    assert rec.lossless is True
    assert rec.selected_codec == CODEC_LIBX264RGB
    rec.shutdown()


def _decode_rgb(path) -> list[np.ndarray]:
    av = pytest.importorskip('av')
    with av.open(str(path)) as container:
        return [f.to_ndarray(format='rgb24') for f in container.decode(video=0)]


@_NEEDS_FFMPEG
def test_raw_rgb_round_trip_is_bit_exact(tmp_path):
    """**넣은 픽셀이 그대로 나온다** — 색이 많은 노이즈 영상이라 근사로는 통과하지 못한다."""
    rng = np.random.default_rng(0)
    frames = [rng.integers(0, 256, size=(48, 64, 3), dtype=np.uint8) for _ in range(12)]
    out = tmp_path / 'raw_lossless.mp4'
    rec = FFMpegMp4Recorder(fps=10, resolution=(64, 48), pixel_format='rgb8',
                            encoder_mode='cpu', lossless=True)
    rec.start(str(out))
    for frame in frames:
        rec.write(frame)
    rec.stop()
    rec.shutdown()

    decoded = _decode_rgb(out)
    assert len(decoded) == len(frames)
    for got, want in zip(decoded, frames):
        np.testing.assert_array_equal(got, want)


@_NEEDS_FFMPEG
def test_raw_bgr_round_trip_is_bit_exact(tmp_path):
    """`bgr8` 도 같다 — 채널 순서가 뒤집혀 담기면 여기서 드러난다."""
    rng = np.random.default_rng(1)
    frames = [rng.integers(0, 256, size=(48, 64, 3), dtype=np.uint8) for _ in range(5)]
    out = tmp_path / 'bgr_lossless.mp4'
    rec = FFMpegMp4Recorder(fps=10, resolution=(64, 48), pixel_format='bgr8',
                            encoder_mode='cpu', lossless=True)
    rec.start(str(out))
    for frame in frames:
        rec.write(frame)
    rec.stop()
    rec.shutdown()

    decoded = _decode_rgb(out)
    assert len(decoded) == len(frames)
    for got, want in zip(decoded, frames):
        np.testing.assert_array_equal(got[:, :, ::-1], want)


@_NEEDS_FFMPEG
def test_jpeg_stream_copy_keeps_the_received_bytes(tmp_path):
    """**컨테이너 패킷이 받은 JPEG 바이트와 같다** — 다시 인코딩하지 않았다는 증거다."""
    cv2 = pytest.importorskip('cv2')
    av = pytest.importorskip('av')
    rng = np.random.default_rng(2)
    jpegs = []
    for _ in range(8):
        img = rng.integers(0, 256, size=(48, 64, 3), dtype=np.uint8)
        ok, buf = cv2.imencode('.jpg', img)
        assert ok
        jpegs.append(buf.tobytes())

    out = tmp_path / 'jpeg_copy.mp4'
    rec = FFMpegMp4Recorder(fps=10, resolution=(64, 48), input_format=INPUT_MJPEG,
                            encoder_mode='cpu', lossless=True)
    assert rec.selected_codec == CODEC_MJPEG
    rec.start(str(out))
    for data in jpegs:
        rec.write_compressed(data)
    rec.stop()
    rec.shutdown()

    with av.open(str(out)) as container:
        assert container.streams.video[0].codec_context.name == 'mjpeg'
        packets = [bytes(p) for p in container.demux(video=0) if p.size]
    assert packets == jpegs
    # OpenCV(재생 · 외부 도구)로도 같은 수가 읽힌다.
    cap = cv2.VideoCapture(str(out))
    count = 0
    while cap.read()[0]:
        count += 1
    cap.release()
    assert count == len(jpegs)
