#!/usr/bin/env python3

from __future__ import annotations

from typing import Any

import io
import subprocess
import time

import numpy as np
import pytest

from rdfp.recorder import ffmpeg_mp4_recorder as recorder_mod
from rdfp.recorder.exceptions import (
    EncoderUnavailableError,
    RecorderStateError,
)
from rdfp.recorder.ffmpeg_command import CODEC_LIBX264
from rdfp.recorder.ffmpeg_mp4_recorder import FFMpegMp4Recorder
from rdfp.types import InvalidFrameError, Resolution


# ---------- Fake subprocess.Popen --------------------------------------------


class _FakeStdin:
    """subprocess.Popen.stdin 대체. 수집된 바이트를 검증에 사용한다."""

    def __init__(self) -> None:
        self.buffer: bytearray = bytearray()
        self.closed: bool = False
        self.simulate_broken_pipe: bool = False

    def write(self, data: bytes) -> int:
        if self.closed or self.simulate_broken_pipe:
            raise BrokenPipeError("fake broken pipe")
        self.buffer.extend(data)
        return len(data)

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True


class FakePopen:
    """ffmpeg subprocess 를 흉내내는 가짜 Popen.

    `wait()` 시 `output_path` 에 가짜 MP4 바이트를 기록하여 finalize 된
    파일이 존재하는 것처럼 보이게 한다.
    """

    # 테스트가 동작을 주입할 수 있도록 클래스 레벨 플래그 제공
    simulate_broken_pipe: bool = False
    wait_returncode: int = 0
    wait_raises_timeout: bool = False
    create_output_file: bool = True
    wait_timeouts_received: list[float | None] = []

    last_instance: "FakePopen | None" = None

    def __init__(self, cmd: list[str], **_kwargs: Any) -> None:
        self.cmd = cmd
        # 커맨드 마지막 인자가 output_path
        self._output_path = cmd[-1]
        self.stdin = _FakeStdin()
        self.stdin.simulate_broken_pipe = FakePopen.simulate_broken_pipe
        self.stdout = io.BytesIO(b"")
        self.stderr = io.BytesIO(b"")
        self.returncode: int | None = None
        FakePopen.last_instance = self

    def wait(self, timeout: float | None = None) -> int:
        FakePopen.wait_timeouts_received.append(timeout)
        if FakePopen.wait_raises_timeout:
            raise subprocess.TimeoutExpired(cmd=self.cmd, timeout=timeout or 0)
        if FakePopen.create_output_file and self.returncode != -9:
            with open(self._output_path, "wb") as f:
                f.write(b"fake mp4 data")
        self.returncode = FakePopen.wait_returncode
        return self.returncode

    def poll(self) -> int | None:
        return self.returncode

    def terminate(self) -> None:
        if self.returncode is None:
            self.returncode = 1

    def kill(self) -> None:
        self.returncode = -9


# ---------- 공통 fixture -----------------------------------------------------


@pytest.fixture(autouse=True)
def _reset_fake_popen() -> None:
    """각 테스트 시작 시 FakePopen 의 클래스 플래그를 리셋."""
    FakePopen.simulate_broken_pipe = False
    FakePopen.wait_returncode = 0
    FakePopen.wait_raises_timeout = False
    FakePopen.create_output_file = True
    FakePopen.last_instance = None
    FakePopen.wait_timeouts_received = []


@pytest.fixture
def mock_select_encoder(monkeypatch: pytest.MonkeyPatch) -> None:
    """생성자의 인코더 probe 가 subprocess 를 호출하지 않도록 대체한다."""
    monkeypatch.setattr(
        recorder_mod, "select_encoder", lambda **kwargs: CODEC_LIBX264
    )


@pytest.fixture
def patched_popen(
    monkeypatch: pytest.MonkeyPatch, mock_select_encoder: None
) -> None:
    """subprocess.Popen 을 FakePopen 으로 대체."""
    monkeypatch.setattr(recorder_mod.subprocess, "Popen", FakePopen)


def _make_bgr_frame(width: int, height: int) -> np.ndarray:
    return np.zeros((height, width, 3), dtype=np.uint8)


def _make_recorder(**overrides: Any) -> FFMpegMp4Recorder:
    defaults: dict[str, Any] = dict(
        fps=30,
        resolution=Resolution(64, 48),
        pixel_format="bgr8",
        encoder_mode="cpu",
    )
    defaults.update(overrides)
    return FFMpegMp4Recorder(**defaults)


def _wait_state(rec: FFMpegMp4Recorder, state: str, timeout: float = 1.0) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if rec.state == state:
            return
        time.sleep(0.005)
    raise AssertionError(f"timed out waiting for state {state}; current={rec.state}")


# ---------- 생성자 -----------------------------------------------------------


def test_constructor_runs_probe_once(monkeypatch: pytest.MonkeyPatch) -> None:
    call_count = {"n": 0}

    def _fake_select(**_kwargs: Any) -> str:
        call_count["n"] += 1
        return CODEC_LIBX264

    monkeypatch.setattr(recorder_mod, "select_encoder", _fake_select)
    rec = FFMpegMp4Recorder(fps=30, resolution=Resolution(64, 48))
    assert call_count["n"] == 1
    assert rec.selected_codec == CODEC_LIBX264
    assert rec.state == "IDLE"


def test_constructor_rejects_invalid_fps(mock_select_encoder: None) -> None:
    with pytest.raises(ValueError, match="invalid fps"):
        FFMpegMp4Recorder(fps=0, resolution=Resolution(64, 48))
    with pytest.raises(ValueError, match="invalid fps"):
        FFMpegMp4Recorder(fps=29.97, resolution=Resolution(64, 48))
    with pytest.raises(ValueError, match="invalid fps"):
        FFMpegMp4Recorder(fps="30", resolution=Resolution(64, 48))  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="invalid fps"):
        FFMpegMp4Recorder(fps=30.0, resolution=Resolution(64, 48))  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="invalid fps"):
        FFMpegMp4Recorder(fps=True, resolution=Resolution(64, 48))  # type: ignore[arg-type]


def test_constructor_rejects_invalid_resolution(mock_select_encoder: None) -> None:
    with pytest.raises(
        ValueError,
        match="resolution width and height must be positive integers",
    ):
        FFMpegMp4Recorder(fps=30, resolution=(0, 48))
    with pytest.raises(
        ValueError,
        match="resolution width and height must be positive integers",
    ):
        FFMpegMp4Recorder(fps=30, resolution=(64, -1))


def test_constructor_rejects_non_resolution_type(
    mock_select_encoder: None,
) -> None:
    with pytest.raises(ValueError, match="resolution must be a string like 1280x720"):
        FFMpegMp4Recorder(fps=30, resolution=object())  # type: ignore[arg-type]


def test_constructor_accepts_tuple_resolution(mock_select_encoder: None) -> None:
    rec = FFMpegMp4Recorder(fps=30, resolution=(64, 48))
    assert rec._resolution == Resolution(64, 48)


def test_constructor_accepts_string_resolution(mock_select_encoder: None) -> None:
    rec = FFMpegMp4Recorder(fps=30, resolution="64x48")
    assert rec._resolution == Resolution(64, 48)


def test_constructor_rejects_invalid_pixel_format(
    mock_select_encoder: None,
) -> None:
    with pytest.raises(ValueError, match="invalid pixel_format"):
        FFMpegMp4Recorder(
            fps=30, resolution=Resolution(64, 48), pixel_format="yuv420p"
        )


def test_constructor_rejects_invalid_queue_size(mock_select_encoder: None) -> None:
    with pytest.raises(ValueError, match="invalid queue_size"):
        FFMpegMp4Recorder(fps=30, resolution=Resolution(64, 48), queue_size=0)


def test_constructor_defaults_gop_size_to_double_fps(
    mock_select_encoder: None,
) -> None:
    rec = FFMpegMp4Recorder(fps=25, resolution=Resolution(64, 48))
    assert rec._gop_size == 50


def test_constructor_gpu_mode_raises_when_no_hw(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def _raise(**_kwargs: Any) -> str:
        raise EncoderUnavailableError("no hw encoder")

    monkeypatch.setattr(recorder_mod, "select_encoder", _raise)
    with pytest.raises(EncoderUnavailableError):
        FFMpegMp4Recorder(
            fps=30, resolution=Resolution(64, 48), encoder_mode="gpu"
        )


# ---------- start() ----------------------------------------------------------


def test_start_transitions_to_recording(
    patched_popen: None, tmp_path: Any
) -> None:
    rec = _make_recorder()
    out = str(tmp_path / "out.mp4")
    rec.start(out)
    try:
        assert rec.state == "RECORDING"
        assert FakePopen.last_instance is not None
        assert out in FakePopen.last_instance.cmd
    finally:
        rec.shutdown()


def test_start_rejects_existing_file(
    patched_popen: None, tmp_path: Any
) -> None:
    out = tmp_path / "exists.mp4"
    out.write_bytes(b"existing")
    rec = _make_recorder()
    try:
        with pytest.raises(FileExistsError):
            rec.start(str(out))
        assert rec.state == "IDLE"
    finally:
        rec.shutdown()


def test_start_rejects_empty_path(patched_popen: None) -> None:
    rec = _make_recorder()
    try:
        with pytest.raises(ValueError, match="output_path"):
            rec.start("")
    finally:
        rec.shutdown()


def test_start_rejects_when_already_recording(
    patched_popen: None, tmp_path: Any
) -> None:
    rec = _make_recorder()
    out = str(tmp_path / "out.mp4")
    rec.start(out)
    try:
        out2 = str(tmp_path / "out2.mp4")
        with pytest.raises(RecorderStateError):
            rec.start(out2)
    finally:
        rec.shutdown()


# ---------- write() ----------------------------------------------------------


def test_write_in_idle_raises(
    patched_popen: None, mock_select_encoder: None
) -> None:
    rec = _make_recorder()
    frame = _make_bgr_frame(64, 48)
    with pytest.raises(RecorderStateError):
        rec.write(frame)


def test_write_rejects_wrong_dtype(
    patched_popen: None, tmp_path: Any
) -> None:
    rec = _make_recorder()
    rec.start(str(tmp_path / "out.mp4"))
    try:
        bad = np.zeros((48, 64, 3), dtype=np.float32)
        with pytest.raises(InvalidFrameError, match="dtype"):
            rec.write(bad)
    finally:
        rec.shutdown()


def test_write_rejects_wrong_shape(
    patched_popen: None, tmp_path: Any
) -> None:
    rec = _make_recorder()
    rec.start(str(tmp_path / "out.mp4"))
    try:
        bad = np.zeros((100, 64, 3), dtype=np.uint8)  # 높이 mismatch
        with pytest.raises(InvalidFrameError, match="shape mismatch"):
            rec.write(bad)
    finally:
        rec.shutdown()


def test_write_accepts_mono8_2d_and_3d(
    patched_popen: None, tmp_path: Any
) -> None:
    rec = _make_recorder(pixel_format="mono8")
    rec.start(str(tmp_path / "mono.mp4"))
    try:
        rec.write(np.zeros((48, 64), dtype=np.uint8))
        rec.write(np.zeros((48, 64, 1), dtype=np.uint8))
        # 잠시 대기하여 writer 가 소비하도록 함
        time.sleep(0.05)
        assert rec.frames_written >= 1
    finally:
        rec.shutdown()


def test_write_non_contiguous_warned_and_converted(
    patched_popen: None, tmp_path: Any
) -> None:
    # 경고 캡처를 위해 커스텀 핸들러를 직접 주입한 로거 사용
    import logging as _logging

    log = _logging.getLogger("test_non_contiguous")
    log.setLevel(_logging.DEBUG)
    records: list[_logging.LogRecord] = []

    class _ListHandler(_logging.Handler):
        def emit(self, record: _logging.LogRecord) -> None:
            records.append(record)

    log.addHandler(_ListHandler())

    rec = _make_recorder(logger=log)
    rec.start(str(tmp_path / "out.mp4"))
    try:
        # 큰 프레임을 잘라 non-contiguous 를 만든다
        big = np.zeros((48, 128, 3), dtype=np.uint8)
        view = big[:, :64, :]  # non-contiguous slice
        assert not view.flags["C_CONTIGUOUS"]
        rec.write(view)
        rec.write(view)  # 경고는 1 회만
        warnings = [
            r for r in records
            if r.levelno >= _logging.WARNING and "non-contiguous" in r.getMessage()
        ]
        assert len(warnings) == 1
        # payload 가 stdin 에 contiguous 바이트로 전달되었는지 확인
        time.sleep(0.05)
        assert FakePopen.last_instance is not None
        assert len(FakePopen.last_instance.stdin.buffer) == 2 * 48 * 64 * 3
    finally:
        rec.shutdown()


def test_write_enqueues_frame_bytes(
    patched_popen: None, tmp_path: Any
) -> None:
    rec = _make_recorder()
    rec.start(str(tmp_path / "out.mp4"))
    try:
        frame = np.full((48, 64, 3), 255, dtype=np.uint8)
        rec.write(frame)
        time.sleep(0.05)
        assert FakePopen.last_instance is not None
        # writer thread 가 stdin 으로 전달한 바이트가 프레임 크기와 일치
        expected_size = 48 * 64 * 3
        assert len(FakePopen.last_instance.stdin.buffer) == expected_size
        assert rec.frames_written == 1
    finally:
        rec.shutdown()


# ---------- write_bytes (fast path) ------------------------------------------


def test_write_bytes_in_idle_raises(
    patched_popen: None, mock_select_encoder: None
) -> None:
    rec = _make_recorder()
    payload = b"\x00" * (48 * 64 * 3)
    with pytest.raises(RecorderStateError):
        rec.write_bytes(payload)


def test_write_bytes_enqueues_payload(
    patched_popen: None, tmp_path: Any
) -> None:
    rec = _make_recorder()
    rec.start(str(tmp_path / "out.mp4"))
    try:
        payload = bytes([255]) * (48 * 64 * 3)
        rec.write_bytes(payload)
        time.sleep(0.05)
        assert FakePopen.last_instance is not None
        assert len(FakePopen.last_instance.stdin.buffer) == 48 * 64 * 3
        assert FakePopen.last_instance.stdin.buffer == bytearray(payload)
        assert rec.frames_written == 1
    finally:
        rec.shutdown()


def test_write_bytes_rejects_wrong_size(
    patched_popen: None, tmp_path: Any
) -> None:
    rec = _make_recorder()
    rec.start(str(tmp_path / "out.mp4"))
    try:
        # 한 바이트 적게
        with pytest.raises(InvalidFrameError, match="size mismatch"):
            rec.write_bytes(b"\x00" * (48 * 64 * 3 - 1))
    finally:
        rec.shutdown()


def test_write_bytes_accepts_bytearray_and_memoryview(
    patched_popen: None, tmp_path: Any
) -> None:
    rec = _make_recorder()
    rec.start(str(tmp_path / "out.mp4"))
    try:
        payload = bytearray(48 * 64 * 3)
        rec.write_bytes(payload)
        rec.write_bytes(memoryview(payload))
        time.sleep(0.05)
        assert FakePopen.last_instance is not None
        assert len(FakePopen.last_instance.stdin.buffer) == 2 * 48 * 64 * 3
        assert rec.frames_written == 2
    finally:
        rec.shutdown()


def test_write_bytes_size_matches_channel_count(
    patched_popen: None, tmp_path: Any
) -> None:
    """encoding 별로 expected size 가 (h*w*channels) 로 계산되는지."""
    # bgra8 (4채널)
    rec = _make_recorder(pixel_format="bgra8")
    rec.start(str(tmp_path / "bgra.mp4"))
    try:
        rec.write_bytes(b"\x00" * (48 * 64 * 4))
        time.sleep(0.05)
        assert len(FakePopen.last_instance.stdin.buffer) == 48 * 64 * 4
    finally:
        rec.shutdown()
    # mono8 (1채널)
    rec = _make_recorder(pixel_format="mono8")
    rec.start(str(tmp_path / "mono.mp4"))
    try:
        rec.write_bytes(b"\x00" * (48 * 64))
        time.sleep(0.05)
        # FakePopen.last_instance 는 가장 최근 인스턴스로 갱신됨
        assert len(FakePopen.last_instance.stdin.buffer) == 48 * 64
    finally:
        rec.shutdown()


# ---------- drop_oldest 정책 -------------------------------------------------


def test_write_drops_oldest_when_queue_full(
    patched_popen: None, tmp_path: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    rec = _make_recorder(queue_size=2)
    # writer thread 가 큐를 소비하지 못하도록 stdin 을 블록시킬 수는 없으므로,
    # writer 스레드 시작을 지연시키는 방식 대신 start() 직후 즉시 대량 enqueue
    rec.start(str(tmp_path / "out.mp4"))
    try:
        # writer thread 가 소비하기 전에 큐를 가득 채우기 위해
        # 일부러 stdin 을 일시 차단
        assert FakePopen.last_instance is not None
        stdin = FakePopen.last_instance.stdin
        original_write = stdin.write

        pause = {"hold": True}

        def _slow_write(data: bytes) -> int:
            while pause["hold"]:
                time.sleep(0.001)
            return original_write(data)

        stdin.write = _slow_write  # type: ignore[assignment]

        frame = _make_bgr_frame(64, 48)
        # 충분히 많은 프레임을 밀어 넣음
        for i in range(20):
            rec.write(frame)

        pause["hold"] = False
        time.sleep(0.05)
        assert rec.frames_dropped > 0
    finally:
        rec.shutdown()


# ---------- stop() -----------------------------------------------------------


def test_stop_returns_path_and_transitions_to_idle(
    patched_popen: None, tmp_path: Any
) -> None:
    rec = _make_recorder()
    out = str(tmp_path / "out.mp4")
    rec.start(out)
    rec.write(_make_bgr_frame(64, 48))
    returned = rec.stop()
    assert returned == out
    assert rec.state == "IDLE"


def test_stop_requires_recording_state(
    patched_popen: None, tmp_path: Any
) -> None:
    rec = _make_recorder()
    with pytest.raises(RecorderStateError):
        rec.stop()


def test_stop_transitions_to_failed_on_nonzero_rc(
    patched_popen: None, tmp_path: Any
) -> None:
    FakePopen.wait_returncode = 1
    FakePopen.create_output_file = False
    rec = _make_recorder()
    rec.start(str(tmp_path / "out.mp4"))
    rec.stop()
    assert rec.state == "FAILED"


def test_stop_timeout_is_shared_budget(
    patched_popen: None, tmp_path: Any
) -> None:
    """stop(timeout) 은 writer join 과 proc.wait 가 공유하는 단일 상한이어야 한다.

    writer thread 가 예산의 일부를 소모한 후, proc.wait 에 전달되는 timeout 이
    전체 예산보다 작아야 한다 (이중 소모 방지).
    """
    rec = _make_recorder()
    rec.start(str(tmp_path / "out.mp4"))

    assert FakePopen.last_instance is not None
    stdin = FakePopen.last_instance.stdin
    original_write = stdin.write

    # writer thread 가 첫 프레임 처리 시 일정 시간을 소모하게 만든다
    def _slow_write(data: bytes) -> int:
        time.sleep(0.15)
        return original_write(data)

    stdin.write = _slow_write  # type: ignore[assignment]

    rec.write(_make_bgr_frame(64, 48))
    # writer 가 작업을 시작할 여유
    time.sleep(0.02)

    total_timeout = 0.5
    start = time.monotonic()
    rec.stop(timeout=total_timeout)
    elapsed = time.monotonic() - start

    # 전체 소요는 예산 + 강제 종료 여유 내에서 완료되어야 함
    assert elapsed < total_timeout + 0.2
    # proc.wait 에 전달된 timeout 은 남은 예산이므로 전체 예산보다 작다
    wait_timeouts = [t for t in FakePopen.wait_timeouts_received if t is not None]
    assert wait_timeouts, "proc.wait should have been called with a timeout"
    assert wait_timeouts[0] < total_timeout, (
        f"proc.wait timeout {wait_timeouts[0]} should be less than "
        f"total {total_timeout} (budget was already partially consumed)"
    )


def test_failed_state_can_restart(
    patched_popen: None, tmp_path: Any
) -> None:
    FakePopen.wait_returncode = 1
    FakePopen.create_output_file = False
    rec = _make_recorder()
    rec.start(str(tmp_path / "out1.mp4"))
    rec.stop()
    assert rec.state == "FAILED"

    # 재시작 성공
    FakePopen.wait_returncode = 0
    FakePopen.create_output_file = True
    rec.start(str(tmp_path / "out2.mp4"))
    assert rec.state == "RECORDING"
    rec.stop()
    assert rec.state == "IDLE"


# ---------- shutdown() -------------------------------------------------------


def test_shutdown_idempotent(
    patched_popen: None, tmp_path: Any
) -> None:
    rec = _make_recorder()
    rec.shutdown()
    assert rec.state == "SHUTDOWN"
    rec.shutdown()  # no-op, 예외 없음
    assert rec.state == "SHUTDOWN"


def test_shutdown_force_terminates_recording(
    patched_popen: None, tmp_path: Any
) -> None:
    rec = _make_recorder()
    rec.start(str(tmp_path / "out.mp4"))
    assert rec.state == "RECORDING"
    rec.shutdown()
    assert rec.state == "SHUTDOWN"


def test_operations_after_shutdown_raise(
    patched_popen: None, tmp_path: Any
) -> None:
    rec = _make_recorder()
    rec.shutdown()
    with pytest.raises(RecorderStateError):
        rec.start(str(tmp_path / "out.mp4"))
    with pytest.raises(RecorderStateError):
        rec.write(_make_bgr_frame(64, 48))
    with pytest.raises(RecorderStateError):
        rec.stop()


# ---------- Context manager --------------------------------------------------


def test_context_manager_calls_shutdown(
    patched_popen: None, tmp_path: Any
) -> None:
    with _make_recorder() as rec:
        rec.start(str(tmp_path / "out.mp4"))
        rec.write(_make_bgr_frame(64, 48))
        rec.stop()
    assert rec.state == "SHUTDOWN"


# ---------- BrokenPipe → FAILED ----------------------------------------------


def test_broken_pipe_transitions_to_failed(
    patched_popen: None, tmp_path: Any
) -> None:
    FakePopen.simulate_broken_pipe = True
    rec = _make_recorder()
    rec.start(str(tmp_path / "out.mp4"))
    try:
        rec.write(_make_bgr_frame(64, 48))
        # writer thread 가 BrokenPipe 를 만나 FAILED 로 전이할 때까지 대기
        _wait_state(rec, "FAILED", timeout=1.0)
    finally:
        rec.shutdown()


def test_stop_escalates_to_terminate_on_wait_timeout(
    patched_popen: None, tmp_path: Any
) -> None:
    """proc.wait 가 타임아웃되면 terminate → proc.wait → kill 로 에스컬레이션한다."""
    rec = _make_recorder()
    rec.start(str(tmp_path / "out.mp4"))

    fake_proc = FakePopen.last_instance
    assert fake_proc is not None

    # stop() 진입 시점부터 wait() 가 TimeoutExpired 를 발생시키도록 설정
    # FakePopen.wait_raises_timeout 은 모든 wait() 호출에 적용되므로, terminate
    # 호출 후에도 계속 타임아웃되어 kill() 까지 진입한다.
    FakePopen.wait_raises_timeout = True

    rec.stop(timeout=0.1)

    # kill 까지 도달했으므로 returncode 는 -9 (FakePopen.kill 이 설정)
    assert fake_proc.returncode == -9
    assert rec.state == "FAILED"


def test_stop_transitions_to_failed_when_output_missing(
    patched_popen: None, tmp_path: Any
) -> None:
    """ffmpeg 가 정상 종료(rc=0)했더라도 출력 파일이 없으면 FAILED 로 전이한다."""
    FakePopen.create_output_file = False  # finalize 해도 파일 생성 안 함
    rec = _make_recorder()
    rec.start(str(tmp_path / "out.mp4"))
    rec.stop()
    assert rec.state == "FAILED"


def test_stop_transitions_to_failed_when_output_empty(
    patched_popen: None, tmp_path: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    """ffmpeg 가 정상 종료했지만 출력 파일이 0 바이트이면 FAILED 로 전이한다."""
    out = tmp_path / "out.mp4"

    class _EmptyFakePopen(FakePopen):
        def wait(self, timeout: float | None = None) -> int:
            FakePopen.wait_timeouts_received.append(timeout)
            # 비어있는 파일 생성
            open(self._output_path, "wb").close()
            self.returncode = 0
            return 0

    monkeypatch.setattr(recorder_mod.subprocess, "Popen", _EmptyFakePopen)

    rec = _make_recorder()
    rec.start(str(out))
    rec.stop()
    assert rec.state == "FAILED"


def test_lifecycle_cycle(
    patched_popen: None, tmp_path: Any
) -> None:
    """start → stop → start → stop → shutdown 전체 사이클이 정상 동작한다."""
    rec = _make_recorder()

    # Cycle 1
    out1 = str(tmp_path / "out1.mp4")
    rec.start(out1)
    assert rec.state == "RECORDING"
    rec.write(_make_bgr_frame(64, 48))
    rec.write(_make_bgr_frame(64, 48))
    assert rec.stop() == out1
    assert rec.state == "IDLE"

    # Cycle 2 (재시작)
    out2 = str(tmp_path / "out2.mp4")
    rec.start(out2)
    assert rec.state == "RECORDING"
    rec.write(_make_bgr_frame(64, 48))
    assert rec.stop() == out2
    assert rec.state == "IDLE"

    # Shutdown
    rec.shutdown()
    assert rec.state == "SHUTDOWN"
    # 사이클 이후 자원 참조가 모두 해제되어야 한다
    assert rec._proc is None
    assert rec._queue is None
    assert rec._writer_thread is None
    assert rec._stderr_thread is None


def test_no_zombie_threads_after_shutdown(
    patched_popen: None, tmp_path: Any
) -> None:
    """shutdown() 이후 writer/stderr 스레드가 살아있지 않아야 한다."""
    import threading as _threading

    before_names = {t.name for t in _threading.enumerate()}

    rec = _make_recorder()
    rec.start(str(tmp_path / "out.mp4"))
    rec.write(_make_bgr_frame(64, 48))
    rec.shutdown()

    # 충분히 대기하여 daemon thread 가 완전히 종료되도록 함
    time.sleep(0.1)

    recorder_thread_names = {"FFMpegMp4Recorder-writer", "FFMpegMp4Recorder-stderr"}
    alive_thread_names = {t.name for t in _threading.enumerate() if t.is_alive()}
    leaked = recorder_thread_names & (alive_thread_names - before_names)
    assert not leaked, f"recorder threads still alive after shutdown: {leaked}"


def test_shutdown_cleans_resources_after_writer_induced_failed(
    patched_popen: None, tmp_path: Any
) -> None:
    """_writer_loop 의 BrokenPipe 로 FAILED 가 된 후 shutdown() 이 자원을 정리해야 한다.

    stop() 경유 없이 writer 스레드가 직접 FAILED 로 전이한 경우에도
    subprocess / 스레드 / 큐가 모두 해제되어야 한다 (좀비 프로세스 방지).
    """
    FakePopen.simulate_broken_pipe = True
    rec = _make_recorder()
    rec.start(str(tmp_path / "out.mp4"))
    rec.write(_make_bgr_frame(64, 48))
    _wait_state(rec, "FAILED", timeout=1.0)

    fake_proc = FakePopen.last_instance
    assert fake_proc is not None

    rec.shutdown()

    assert rec.state == "SHUTDOWN"
    # 런타임 자원 참조가 모두 해제되어야 한다
    assert rec._proc is None
    assert rec._queue is None
    assert rec._writer_thread is None
    assert rec._stderr_thread is None
    # subprocess 는 종료 상태이어야 한다 (terminate 또는 정상 종료)
    assert fake_proc.returncode is not None
