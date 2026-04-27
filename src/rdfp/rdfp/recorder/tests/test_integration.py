#!/usr/bin/env python3
"""실제 ffmpeg 바이너리를 사용하는 통합 테스트.

ffmpeg/ffprobe 가 시스템에 설치되어 있어야 한다. 없을 경우 테스트 전체가
자동으로 skip 된다.
"""

from __future__ import annotations

from typing import Any

import json
import os
import shutil
import subprocess

import numpy as np
import pytest

from rdfp.recorder.ffmpeg_command import CODEC_LIBX264
from rdfp.recorder.ffmpeg_mp4_recorder import FFMpegMp4Recorder
from rdfp.types import Resolution


# ffmpeg / ffprobe 미설치 시 전체 모듈 skip
pytestmark = pytest.mark.skipif(
    shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None,
    reason="ffmpeg/ffprobe not installed on this system",
)


# ---------- 헬퍼 -------------------------------------------------------------


def _gradient_frame(width: int, height: int, t: int) -> np.ndarray:
    """시간에 따라 변화하는 그라디언트 bgr8 프레임을 생성한다."""
    row = np.linspace(0, 255, width, dtype=np.uint8)
    grad = np.tile(row, (height, 1))
    frame = np.stack([grad, (grad + t) % 256, (grad * 2) % 256], axis=-1).astype(
        np.uint8
    )
    return frame


def _ffprobe_json(path: str) -> dict[str, Any]:
    result = subprocess.run(
        [
            "ffprobe",
            "-v", "error",
            "-print_format", "json",
            "-show_streams",
            "-show_format",
            path,
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    return json.loads(result.stdout)


def _get_video_stream(info: dict[str, Any]) -> dict[str, Any]:
    for s in info["streams"]:
        if s.get("codec_type") == "video":
            return s
    raise AssertionError("no video stream in output")


# ---------- 통합: 그라디언트 프레임 → mp4 생성 ------------------------------


def test_integration_gradient_frames_to_mp4(tmp_path: Any) -> None:
    """100 프레임 그라디언트를 libx264 로 인코딩하고 ffprobe 로 검증한다."""
    width, height, fps = 160, 120, 30
    n_frames = 100
    out_path = str(tmp_path / "gradient.mp4")

    rec = FFMpegMp4Recorder(
        fps=fps,
        resolution=Resolution(width, height),
        pixel_format="bgr8",
        encoder_mode="cpu",  # 결정적 결과 보장
        preset="ultrafast",  # 테스트 속도 우선
        bitrate="500k",
    )
    try:
        assert rec.selected_codec == CODEC_LIBX264

        rec.start(out_path)
        for i in range(n_frames):
            rec.write(_gradient_frame(width, height, i))
        returned = rec.stop(timeout=30.0)
    finally:
        rec.shutdown()

    # 상태 및 반환 경로 확인
    assert returned == out_path
    assert os.path.exists(out_path)
    assert os.path.getsize(out_path) > 0

    # ffprobe 로 실제 메타데이터 검증
    info = _ffprobe_json(out_path)
    vs = _get_video_stream(info)

    # 해상도 일치
    assert vs["width"] == width
    assert vs["height"] == height

    # 코덱 일치 (container 의 h264 는 libx264 로 인코딩됨)
    assert vs["codec_name"] == "h264"

    # 프레임 수 = 입력 프레임 수
    nb_frames = int(vs.get("nb_frames") or 0)
    if nb_frames:
        assert nb_frames == n_frames
    else:
        # nb_frames 가 없으면 duration 으로 확인
        duration = float(info["format"]["duration"])
        expected = n_frames / fps
        # ±1 프레임 여유
        assert abs(duration - expected) <= (1.0 / fps) + 0.05

    # 오디오 스트림이 없어야 한다
    audio_streams = [
        s for s in info["streams"] if s.get("codec_type") == "audio"
    ]
    assert not audio_streams


def test_integration_mp4_has_faststart_flag(tmp_path: Any) -> None:
    """-movflags +faststart 가 적용되어 moov atom 이 파일 앞쪽에 위치해야 한다."""
    rec = FFMpegMp4Recorder(
        fps=15, resolution=Resolution(64, 48), pixel_format="bgr8",
        encoder_mode="cpu", preset="ultrafast", bitrate="200k",
    )
    out = str(tmp_path / "fast.mp4")
    try:
        rec.start(out)
        for i in range(10):
            rec.write(_gradient_frame(64, 48, i))
        rec.stop(timeout=30.0)
    finally:
        rec.shutdown()

    # moov atom 의 위치를 확인 (faststart 가 적용되면 앞쪽에 위치)
    # 단순 검증: 파일 앞 1KB 내에 'moov' 가 존재하는지 확인
    with open(out, "rb") as f:
        head = f.read(4096)
    assert b"moov" in head, "moov atom should appear near the beginning with faststart"


def test_integration_mono8_frames(tmp_path: Any) -> None:
    """mono8 입력 포맷이 실제로 인코딩되는지 확인한다."""
    width, height, fps = 128, 96, 15
    out = str(tmp_path / "mono.mp4")

    rec = FFMpegMp4Recorder(
        fps=fps, resolution=Resolution(width, height), pixel_format="mono8",
        encoder_mode="cpu", preset="ultrafast", bitrate="200k",
    )
    try:
        rec.start(out)
        for i in range(30):
            frame = np.full((height, width), (i * 8) % 256, dtype=np.uint8)
            rec.write(frame)
        rec.stop(timeout=30.0)
    finally:
        rec.shutdown()

    info = _ffprobe_json(out)
    vs = _get_video_stream(info)
    assert vs["width"] == width
    assert vs["height"] == height
    assert vs["codec_name"] == "h264"


def test_integration_start_rejects_existing_file(tmp_path: Any) -> None:
    """실제 파일 시스템에서도 FileExistsError 가 발생해야 한다."""
    out = tmp_path / "exists.mp4"
    out.write_bytes(b"pre-existing content")

    rec = FFMpegMp4Recorder(
        fps=30, resolution=Resolution(64, 48), encoder_mode="cpu"
    )
    try:
        with pytest.raises(FileExistsError):
            rec.start(str(out))
    finally:
        rec.shutdown()

    # 기존 파일이 유지되어야 함 (덮어쓰기 방지)
    assert out.read_bytes() == b"pre-existing content"


def test_integration_lifecycle_restart(tmp_path: Any) -> None:
    """실제 ffmpeg 로 start → stop → start → stop 사이클을 수행한다."""
    rec = FFMpegMp4Recorder(
        fps=15, resolution=Resolution(64, 48), encoder_mode="cpu",
        preset="ultrafast", bitrate="200k",
    )
    try:
        for i in range(2):
            out = str(tmp_path / f"cycle{i}.mp4")
            rec.start(out)
            for j in range(15):
                rec.write(_gradient_frame(64, 48, i * 10 + j))
            rec.stop(timeout=30.0)
            assert rec.state == "IDLE"
            assert os.path.getsize(out) > 0
    finally:
        rec.shutdown()
