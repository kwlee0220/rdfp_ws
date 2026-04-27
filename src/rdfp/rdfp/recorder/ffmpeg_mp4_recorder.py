#!/usr/bin/env python3

from __future__ import annotations

from typing import Final, Optional

import logging
import os
import queue
import subprocess
import threading
import time
from array import array

import numpy as np

from ..types import Resolution
from .encoder_probe import select_encoder
from ..types import InvalidFrameError
from .ffmpeg_command import SUPPORTED_ENCODINGS, build_ffmpeg_command, channels_for
from .state import RecorderState, RecorderStateMachine


_FFMPEG_BINARY = "ffmpeg"
_VAAPI_DEVICE_DEFAULT = "/dev/dri/renderD128"  # 일반적으로 VAAPI 디바이스의 기본 경로

# 큐 종료 신호 (값이 아닌 유일한 센티넬 객체)
_QUEUE_SENTINEL: Final[object] = object()

_DEFAULT_QUEUE_SIZE: Final[int] = 120
_DEFAULT_STOP_TIMEOUT_SEC: Final[float] = 5.0
_STDERR_JOIN_TIMEOUT_SEC: Final[float] = 1.0
_FORCE_KILL_GRACE_SEC: Final[float] = 2.0

# 큐 가득참 시의 처리 정책
_OVERFLOW_WAIT: Final[str] = "wait"
_OVERFLOW_DROP_OLDEST: Final[str] = "drop_oldest"
_OVERFLOW_DROP_NEWEST: Final[str] = "drop_newest"
_VALID_OVERFLOW_POLICIES: Final[frozenset[str]] = frozenset(
    {_OVERFLOW_WAIT, _OVERFLOW_DROP_OLDEST, _OVERFLOW_DROP_NEWEST}
)


class FFMpegMp4Recorder:
    """ffmpeg subprocess 를 이용한 MP4 녹화 클래스.

    OpenCV 이미지(`numpy.ndarray`)를 `write()` 로 주입하면 별도 writer 스레드가
    ffmpeg 의 stdin 으로 raw 프레임을 전달하고, `stop()` 시 finalize 된 MP4
    파일이 생성된다.

    주요 특성:
        - CFR + passthrough (write 순서 그대로 ffmpeg 에 전달)
        - GPU 인코더 probe 는 생성자에서 1 회 수행
        - 상태 모델: IDLE / RECORDING / STOPPING / FAILED / SHUTDOWN
        - 오디오 미지원, ROS2 비의존

    상태 전이는 `RecorderStateMachine` 이 관리하며, `write()` 중 상태 경합을
    막기 위해 상태 검사 + 큐 enqueue 를 동일 락으로 감싼다.

    Example:
        기본 사용법 (CPU 인코더)::

            import numpy as np
            from rdfp.recorder import FFMpegMp4Recorder
            from rdfp.types import Resolution

            rec = FFMpegMp4Recorder(
                fps=30, resolution=Resolution(640, 480),
                pixel_format="bgr8", encoder_mode="cpu",
            )
            rec.start("/tmp/out.mp4")
            for _ in range(300):  # 10 초
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                rec.write(frame)
            rec.stop()
            rec.shutdown()

        컨텍스트 매니저로 자동 정리::

            with FFMpegMp4Recorder(fps=30, resolution=Resolution(640, 480)) as rec:
                rec.start("/tmp/out.mp4")
                rec.write(frame)
                rec.stop()
            # __exit__ 이 shutdown() 을 호출한다

    생명주기 규칙:
        - `fps` / 해상도 / `pixel_format` / 인코더 설정은 생성자에서 고정
        - `start()` → `write()*` → `stop()` 순으로 호출
        - `stop()` 이후 `IDLE` 로 복귀하므로 동일 인스턴스로 재녹화 가능
        - 치명 오류 시 `FAILED` 로 전이되며, `start()` 로 재시작할 수 있다
        - `shutdown()` 이후에는 `RecorderStateError` (종착 상태)
    """

    def __init__(self, *,
        fps: int,
        resolution: str | tuple[int, int] | Resolution,
        pixel_format: str = "bgr8",
        encoder_mode: str = "auto",
        preferred_hw_codec: Optional[str] = None,
        bitrate: str = "4M",
        gop_size: Optional[int] = None,
        preset: str = "medium",
        ffmpeg_binary: str = _FFMPEG_BINARY,
        vaapi_device: str = _VAAPI_DEVICE_DEFAULT,
        queue_size: int = _DEFAULT_QUEUE_SIZE,
        overflow_policy: str = _OVERFLOW_WAIT,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        """
        Args:
            fps: 입력 CFR (양의 정수).
            resolution: 프레임 해상도. `Resolution(width, height)` 또는
                `(width, height)` 튜플, `"WIDTHxHEIGHT"` 문자열을 지원한다.
                내부에서 `Resolution` 으로 변환한다.
            pixel_format: `bgr8` / `rgb8` / `bgra8` / `rgba8` / `mono8` 중 하나.
            encoder_mode: `auto` / `cpu` / `gpu`.
            preferred_hw_codec: 지정 시 해당 HW 인코더만 probe.
            bitrate: ffmpeg `-b:v` 값 (예: `"4M"`).
            gop_size: GOP 크기. `None` 이면 `fps * 2`.
            preset: libx264 preset.
            ffmpeg_binary: ffmpeg 실행 파일.
            vaapi_device: VAAPI 디바이스 경로.
            queue_size: 프레임 큐 최대 크기. 가득차면 `overflow_policy` 적용.
            overflow_policy: 큐 가득참 시의 처리 정책. 다음 중 하나.

                * ``"wait"`` (기본) — 큐에 공간이 생길 때까지 ``write()`` 가
                  블록한다. 프레임을 한 장도 누락하지 않으나, 호출자가
                  백프레셔를 받는다.
                * ``"drop_oldest"`` — 가장 오래된 프레임을 제거하고 새 프레임을
                  삽입한다.
                * ``"drop_newest"`` — 새 프레임을 무시하고 기존 큐를 유지한다.
            logger: 외부 주입 로거. 생략 시 클래스 기본 로거 사용.

        Raises:
            ValueError: 인자 값이 유효하지 않은 경우.
            EncoderUnavailableError: `encoder_mode="gpu"` 인데 사용 가능한 HW
                인코더가 없는 경우 (생성자에서 probe).
        """
        # 1. 인자 검증
        if not isinstance(fps, int) or isinstance(fps, bool) or fps <= 0:
            raise ValueError(f"invalid fps: {fps}; must be a positive int")
        parsed_resolution = Resolution.parse(resolution, "resolution")
        if pixel_format not in SUPPORTED_ENCODINGS:
            raise ValueError(
                f"invalid pixel_format: {pixel_format!r}; "
                f"expected one of {sorted(SUPPORTED_ENCODINGS)}"
            )
        if not isinstance(queue_size, int) or queue_size <= 0:
            raise ValueError(
                f"invalid queue_size: {queue_size}; must be a positive int"
            )
        if gop_size is not None and (not isinstance(gop_size, int) or gop_size <= 0):
            raise ValueError(
                f"invalid gop_size: {gop_size}; must be a positive int or None"
            )
        if overflow_policy not in _VALID_OVERFLOW_POLICIES:
            raise ValueError(
                f"invalid overflow_policy: {overflow_policy!r}; "
                f"expected one of {sorted(_VALID_OVERFLOW_POLICIES)}"
            )

        # 2. 로거 / 상태 머신 초기화
        self._logger: logging.Logger = logger or logging.getLogger(
            f"{__name__}.{self.__class__.__name__}"
        )
        self._state_machine: RecorderStateMachine = RecorderStateMachine(self._logger)

        # 3. 구성 저장 (start() 이후 변경 불가)
        self._fps: int = fps
        self._resolution: Resolution = parsed_resolution
        self._pixel_format: str = pixel_format
        self._bitrate: str = bitrate
        self._gop_size: int = gop_size if gop_size is not None else fps * 2
        self._preset: str = preset
        self._ffmpeg_binary: str = ffmpeg_binary
        self._vaapi_device: str = vaapi_device
        self._queue_size: int = queue_size
        self._overflow_policy: str = overflow_policy

        # 4. 인코더 probe (생성자 1 회)
        self._selected_codec: str = select_encoder(
            encoder_mode=encoder_mode,
            ffmpeg_binary=ffmpeg_binary,
            preferred_hw_codec=preferred_hw_codec,
            vaapi_device=vaapi_device,
            logger=self._logger,
        )

        # 5. 런타임 상태 (start() 에서 설정)
        self._output_path: Optional[str] = None
        self._proc: Optional[subprocess.Popen] = None
        self._queue: Optional[queue.Queue] = None
        self._writer_thread: Optional[threading.Thread] = None
        self._stderr_thread: Optional[threading.Thread] = None

        # 6. 카운터 및 경고 플래그
        self._frames_written: int = 0
        self._frames_dropped: int = 0
        self._non_contiguous_warned: bool = False

        self._logger.debug(
            "FFMpegMp4Recorder initialized: fps=%d size=%s pixel_format=%s codec=%s",
            fps, parsed_resolution, pixel_format, self._selected_codec)

    # ---------- Properties --------------------------------------------------

    @property
    def state(self) -> str:
        """현재 상태의 문자열 표현 (스냅샷)."""
        return str(self._state_machine.state)
    
    @property
    def pixel_format(self) -> str:
        """생성자에서 설정된 pixel_format."""
        return self._pixel_format

    @property
    def selected_codec(self) -> str:
        """생성자 probe 로 선택된 코덱 이름."""
        return self._selected_codec

    @property
    def frames_written(self) -> int:
        """writer thread 가 ffmpeg 에 성공적으로 전달한 프레임 수."""
        return self._frames_written

    @property
    def frames_dropped(self) -> int:
        """큐 가득참으로 drop 된 프레임 수."""
        return self._frames_dropped

    @property
    def logger(self) -> logging.Logger:
        """내부 로거."""
        return self._logger

    # ---------- start -------------------------------------------------------

    def start(self, output_path: str) -> None:
        """녹화를 시작한다.

        `IDLE` 또는 `FAILED` 상태에서만 호출할 수 있다. 지정한 경로에 파일이
        이미 존재하면 `FileExistsError` 로 즉시 실패한다 (ffmpeg `-n` 옵션과
        이중 방어).

        Args:
            output_path: 출력 MP4 파일 경로.

        Raises:
            ValueError: `output_path` 가 빈 문자열인 경우.
            RecorderStateError: 허용되지 않는 상태에서 호출된 경우.
            FileExistsError: 출력 파일이 이미 존재하는 경우.
        """
        if not isinstance(output_path, str) or not output_path:
            raise ValueError("output_path must be a non-empty string")

        with self._state_machine.lock:
            self._state_machine.require(
                RecorderState.IDLE, RecorderState.FAILED
            )

            if os.path.exists(output_path):
                raise FileExistsError(
                    f"output file already exists: {output_path}"
                )

            cmd = build_ffmpeg_command(
                ffmpeg_binary=self._ffmpeg_binary,
                pixel_format=self._pixel_format,
                width=self._resolution.width,
                height=self._resolution.height,
                fps=self._fps,
                codec=self._selected_codec,
                bitrate=self._bitrate,
                gop_size=self._gop_size,
                output_path=output_path,
                preset=self._preset,
                vaapi_device=self._vaapi_device,
            )

            self._proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )

            self._output_path = output_path
            self._queue = queue.Queue(maxsize=self._queue_size)
            self._frames_written = 0
            self._frames_dropped = 0
            self._non_contiguous_warned = False

            self._writer_thread = threading.Thread(
                target=self._writer_loop,
                name="FFMpegMp4Recorder-writer",
                daemon=True,
            )
            self._stderr_thread = threading.Thread(
                target=self._stderr_drainer_loop,
                name="FFMpegMp4Recorder-stderr",
                daemon=True,
            )
            self._writer_thread.start()
            self._stderr_thread.start()

            self._state_machine.transition(RecorderState.RECORDING)

            self._logger.info("recorder started: path=%s codec=%s fps=%d size=%dx%d",
                              output_path,
                              self._selected_codec,
                              self._fps,
                              self._resolution.width,
                              self._resolution.height)

    # ---------- write -------------------------------------------------------

    def write(self, image: np.ndarray) -> None:
        """프레임을 큐에 enqueue 한다.

        상태 검사(snapshot)로 빠르게 실패한 뒤, 프레임 검증을 수행하고,
        상태 머신 락 하에서 재검사 + enqueue 를 원자적으로 수행한다.

        Args:
            image: OpenCV 이미지 (`np.ndarray`, dtype=uint8).

        Raises:
            RecorderStateError: 현재 상태가 `RECORDING` 이 아닌 경우.
            InvalidFrameError: 프레임 dtype / shape 이 recorder 설정과
                불일치하는 경우.
        """
        self._state_machine.require(RecorderState.RECORDING)
        payload = self._prepare_frame_bytes(image)

        with self._state_machine.lock:
            self._state_machine.require(RecorderState.RECORDING)
            self._enqueue(payload)

    def write_bytes(self, data: bytes | memoryview | bytearray | array) -> None:
        """이미 직렬화된 raw 픽셀 바이트를 큐에 enqueue 한다 (fast path).

        ``write()`` 의 ndarray 변환을 건너뛴다. ROS ``sensor_msgs/Image.data`` 처럼
        이미 ffmpeg 가 원하는 layout (`(H, W, channels)` C-order, ``uint8``) 으로
        직렬화된 바이트를 가진 호출자가 cv_bridge / ndarray 라운드트립을 피하기
        위해 사용한다.

        검증은 **총 바이트 수**만 한다 (`height * width * channels`). 호출자가
        layout 정확성을 보장해야 한다 (잘못된 레이아웃은 깨진 영상으로 이어짐).

        Args:
            data: 프레임 바이트. ``bytes`` / ``bytearray`` / contiguous ``memoryview``
                / ``array.array('B', ...)`` (rclpy 의 ``Image.data`` 표준 타입) 중 하나.
                memoryview 가 비-contiguous 인 경우 자동으로 bytes 화한다.

        Raises:
            RecorderStateError: 현재 상태가 ``RECORDING`` 이 아닌 경우.
            InvalidFrameError: 바이트 수가 ``height * width * channels`` 와
                일치하지 않는 경우.
        """
        # 빠른 상태 체크 (snapshot)
        self._state_machine.require(RecorderState.RECORDING)

        h, w = self._resolution.height, self._resolution.width
        expected = h * w * channels_for(self._pixel_format)

        if isinstance(data, memoryview):
            if not data.contiguous:
                if not self._non_contiguous_warned:
                    self._logger.warning("received non-contiguous memoryview; copying to contiguous bytes")
                    self._non_contiguous_warned = True
                data = bytes(data)
        if len(data) != expected:
            raise InvalidFrameError(
                f"data size mismatch for pixel_format={self._pixel_format!r} "
                f"({h}x{w}): expected {expected} bytes, got {len(data)}"
            )

        # bytes 가 아니면 한 번 정규화 (subprocess.stdin.write 가 bytes-like 만 받음)
        payload = data if isinstance(data, bytes) else bytes(data)

        with self._state_machine.lock:
            self._state_machine.require(RecorderState.RECORDING)
            self._enqueue(payload)

    def _prepare_frame_bytes(self, image: np.ndarray) -> bytes:
        """프레임을 검증하고 raw bytes 로 직렬화한다."""
        if not isinstance(image, np.ndarray):
            raise InvalidFrameError(f"image must be np.ndarray, got {type(image).__name__}")
        
        if image.dtype != np.uint8:
            raise InvalidFrameError(f"image dtype must be uint8, got {image.dtype}")

        h, w = self._resolution.height, self._resolution.width
        if self._pixel_format == "mono8":
            # (H, W, 1) 은 (H, W) 로 축소
            if image.ndim == 3 and image.shape == (h, w, 1):
                image = image.reshape(h, w)
            if image.shape != (h, w):
                raise InvalidFrameError(
                    f"mono8 image shape mismatch: expected ({h}, {w}) or "
                    f"({h}, {w}, 1), got {image.shape}"
                )
        elif self._pixel_format in ("bgra8", "rgba8"):
            if image.shape != (h, w, 4):
                raise InvalidFrameError(
                    f"{self._pixel_format} image shape mismatch: "
                    f"expected ({h}, {w}, 4), got {image.shape}"
                )
        else:
            if image.shape != (h, w, 3):
                raise InvalidFrameError(
                    f"{self._pixel_format} image shape mismatch: "
                    f"expected ({h}, {w}, 3), got {image.shape}"
                )

        if not image.flags["C_CONTIGUOUS"]:
            if not self._non_contiguous_warned:
                self._logger.warning("received non-contiguous frame; copying to contiguous buffer")
                self._non_contiguous_warned = True
            image = np.ascontiguousarray(image)

        return image.tobytes()

    def _enqueue(self, payload: bytes) -> None:
        """`overflow_policy` 에 따라 프레임을 큐에 추가한다.

        * ``wait`` — 공간이 생길 때까지 블록한다.
        * ``drop_oldest`` — 가장 오래된 프레임을 제거하고 새 프레임을 삽입한다.
        * ``drop_newest`` — 새 프레임을 무시한다.

        drop 발생 시 `frames_dropped` 카운터를 증가시키고 일정 주기로
        WARNING 로그를 남긴다.
        """
        q = self._queue
        assert q is not None
        try:
            q.put_nowait(payload)
            return
        except queue.Full:
            pass

        if self._overflow_policy == _OVERFLOW_WAIT:
            # 공간이 생길 때까지 블록 (writer thread 가 소비하면 풀린다)
            q.put(payload)
            return

        if self._overflow_policy == _OVERFLOW_DROP_NEWEST:
            self._record_drop()
            return

        # drop_oldest: 가장 오래된 프레임 제거 후 새 프레임 삽입
        try:
            q.get_nowait()
        except queue.Empty:
            pass
        self._record_drop()

        try:
            q.put_nowait(payload)
        except queue.Full:
            # 이 시점까지 경합이 일어난 경우에만 도달. 삽입 자체도 포기
            self._frames_dropped += 1

    def _record_drop(self) -> None:
        """drop 카운터 증가 및 주기적 WARNING 로깅."""
        self._frames_dropped += 1
        if self._frames_dropped == 1 or self._frames_dropped % 100 == 0:
            self._logger.warning(
                "frame dropped: queue full (dropped=%d)", self._frames_dropped
            )

    # ---------- stop --------------------------------------------------------

    def stop(self, timeout: float = _DEFAULT_STOP_TIMEOUT_SEC) -> str:
        """녹화를 종료하고 최종 MP4 파일 경로를 반환한다.

        `RECORDING` 상태에서만 호출 가능. sentinel 을 큐에 넣어 writer 가
        남은 프레임을 모두 소비한 뒤 ffmpeg finalize 를 기다린다. `timeout` 은
        drain + finalize 전체에 대한 **단일 상한**으로, writer join 과
        `proc.wait()` 가 이 예산을 **공유**한다 (경과 시간만큼 차감). 예산이
        소진되면 terminate → kill 순서로 강제 종료하며, 이 정리 단계는
        `_FORCE_KILL_GRACE_SEC` 상수를 사용하여 예산과 별개로 bounded 된다.

        Args:
            timeout: drain + finalize 전체에 대한 상한 (초).

        Returns:
            녹화된 MP4 파일 경로.

        Raises:
            RecorderStateError: 현재 상태가 `RECORDING` 이 아닌 경우.
        """
        with self._state_machine.lock:
            self._state_machine.require(RecorderState.RECORDING)
            self._state_machine.transition(RecorderState.STOPPING)
            assert self._queue is not None
            self._queue.put(_QUEUE_SENTINEL)

        # drain + finalize 전체 예산을 단조 시계 기반 deadline 으로 추적
        deadline = time.monotonic() + max(0.0, timeout)

        def _remaining() -> float:
            return max(0.0, deadline - time.monotonic())

        # writer join 은 락 밖에서 (writer 가 FAILED 전이 시 동일 락이 필요하므로 데드락 방지)
        assert self._writer_thread is not None
        self._writer_thread.join(timeout=_remaining())

        proc = self._proc
        finalize_success = True

        if self._writer_thread.is_alive():
            self._logger.error("writer thread did not finish within timeout")
            finalize_success = False

        if proc is not None and proc.stdin is not None:
            try:
                proc.stdin.close()
            except Exception:
                pass

        if proc is not None:
            try:
                proc.wait(timeout=_remaining())
            except subprocess.TimeoutExpired:
                self._logger.error("ffmpeg did not exit within timeout; terminating")
                proc.terminate()
                try:
                    proc.wait(timeout=_FORCE_KILL_GRACE_SEC)
                except subprocess.TimeoutExpired:
                    self._logger.error("ffmpeg did not respond to SIGTERM; killing")
                    proc.kill()
                    try:
                        proc.wait(timeout=_FORCE_KILL_GRACE_SEC)
                    except subprocess.TimeoutExpired:
                        pass
                finalize_success = False

            if proc.returncode != 0:
                self._logger.error(
                    "ffmpeg exited abnormally: rc=%s", proc.returncode
                )
                finalize_success = False

        if self._stderr_thread is not None:
            self._stderr_thread.join(timeout=_STDERR_JOIN_TIMEOUT_SEC)

        output_path = self._output_path or ""
        if not output_path or not os.path.exists(output_path):
            self._logger.error("output file not found after stop: %s", output_path)
            finalize_success = False
        elif os.path.getsize(output_path) == 0:
            self._logger.error("output file is empty after stop: %s", output_path)
            finalize_success = False

        self._logger.info(
            "recorder stopped: path=%s frames_written=%d frames_dropped=%d success=%s",
            output_path,
            self._frames_written,
            self._frames_dropped,
            finalize_success,
        )

        with self._state_machine.lock:
            if self._state_machine.state is RecorderState.STOPPING:
                target = (
                    RecorderState.IDLE if finalize_success else RecorderState.FAILED
                )
                self._state_machine.transition(target)

        self._release_runtime_resources()
        return output_path

    # ---------- shutdown ----------------------------------------------------

    def shutdown(self) -> None:
        """자원을 반환하고 SHUTDOWN 상태로 전이한다 (idempotent).

        상태에 상관없이 런타임 자원(ffmpeg subprocess, writer/stderr 스레드,
        큐)이 남아있으면 강제 정리한다. 이는 `_writer_loop` 가 BrokenPipe 를
        감지하여 `stop()` 경유 없이 직접 `FAILED` 로 전이한 경우에도 좀비
        프로세스가 남지 않도록 보장한다.
        """
        with self._state_machine.lock:
            if self._state_machine.is_shutdown():
                return

        # 런타임 자원이 남아있으면 항상 정리. `_force_shutdown_resources()` 는
        # None 참조를 허용하므로 IDLE/FAILED(cleanup 완료) 상태에서도 no-op.
        self._force_shutdown_resources()

        with self._state_machine.lock:
            self._state_machine.transition(RecorderState.SHUTDOWN)
        self._logger.info("recorder shutdown complete")

    def _force_shutdown_resources(self) -> None:
        """활성 상태에서 호출될 때 스레드/프로세스를 강제 종료한다."""
        q = self._queue
        proc = self._proc
        writer = self._writer_thread
        stderr_t = self._stderr_thread

        # writer 가 대기 중일 수 있으므로 sentinel 을 삽입하여 깨운다
        if q is not None:
            try:
                q.put_nowait(_QUEUE_SENTINEL)
            except queue.Full:
                try:
                    q.get_nowait()
                except queue.Empty:
                    pass
                try:
                    q.put_nowait(_QUEUE_SENTINEL)
                except queue.Full:
                    pass

        if proc is not None:
            try:
                if proc.stdin is not None:
                    proc.stdin.close()
            except Exception:
                pass
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=_FORCE_KILL_GRACE_SEC)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    try:
                        proc.wait(timeout=_FORCE_KILL_GRACE_SEC)
                    except subprocess.TimeoutExpired:
                        pass

        if writer is not None:
            writer.join(timeout=_FORCE_KILL_GRACE_SEC)
        if stderr_t is not None:
            stderr_t.join(timeout=_STDERR_JOIN_TIMEOUT_SEC)

        self._release_runtime_resources()

    def _release_runtime_resources(self) -> None:
        """start() 에서 설정한 런타임 자원 참조를 해제한다."""
        self._proc = None
        self._queue = None
        self._writer_thread = None
        self._stderr_thread = None

    # ---------- Thread loops ------------------------------------------------

    def _writer_loop(self) -> None:
        """큐에서 프레임을 꺼내 ffmpeg stdin 으로 전달한다.

        `stdin.flush()` 는 매 프레임마다 호출하지 않는다. Python BufferedWriter
        의 버퍼(기본 8KB)는 대부분의 비디오 프레임보다 작아 `write()` 호출이
        즉시 OS pipe 로 전달되며, 남은 버퍼는 `stop()` / `shutdown()` 의
        `stdin.close()` 에서 flush 된다.
        """
        try:
            q = self._queue
            proc = self._proc
            assert q is not None and proc is not None and proc.stdin is not None
            stdin = proc.stdin
            while True:
                item = q.get()
                if item is _QUEUE_SENTINEL:
                    break
                try:
                    stdin.write(item)
                except (BrokenPipeError, OSError) as exc:
                    self._logger.error("ffmpeg stdin write failed: %s", exc)
                    self._state_machine.try_transition(
                        RecorderState.FAILED,
                        from_states=[
                            RecorderState.RECORDING,
                            RecorderState.STOPPING,
                        ],
                    )
                    return
                self._frames_written += 1
        except Exception as exc:
            self._logger.exception("writer thread error: %s", exc)
            self._state_machine.try_transition(
                RecorderState.FAILED,
                from_states=[RecorderState.RECORDING, RecorderState.STOPPING],
            )

    def _stderr_drainer_loop(self) -> None:
        """ffmpeg stderr 를 지속적으로 읽어 DEBUG 로그로 배출한다."""
        try:
            proc = self._proc
            assert proc is not None and proc.stderr is not None
            for raw in iter(proc.stderr.readline, b""):
                if not raw:
                    break
                line = raw.decode(errors="replace").rstrip()
                if line:
                    self._logger.debug("ffmpeg: %s", line)
        except Exception:
            # drainer 는 진단 목적이므로 실패해도 전체 녹화에 영향을 주지 않는다
            pass

    # ---------- Context manager --------------------------------------------

    def __enter__(self) -> "FFMpegMp4Recorder":
        return self

    def __exit__(self, *_: object) -> None:
        self.shutdown()

    def __del__(self) -> None:
        # 가비지 컬렉션 시 최후 안전장치로 shutdown 을 시도한다.
        try:
            self.shutdown()
        except Exception:
            pass
