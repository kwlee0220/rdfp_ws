#!/usr/bin/env python3

"""RdfpImageRecorderNode 모듈.

세션 제어 토픽(`rdfp_msgs/msg/SessionCommand`)의 상태 변경에 따라
`sensor_msgs/Image` 스트림을 MP4 파일로 녹화하는 ROS2 노드.

녹화 경계는 메시지 도착 순서가 아닌 **타임스탬프 기반**으로 판정하며,
`pending_image_queue`를 사용하여 도착 순서 차이를 보상한다.
녹화 엔진은 `rdfp.recorder.FFMpegMp4Recorder`를 재사용한다.

녹화 1회 수행 시 다음 3개 파일이 생성된다 (`<base>=output_dir`,
`<prefix>=session_prefix`, `<start_ts>=YYYYMMDD-HHMMSS.SSS`):

* ``<base>/<prefix>_<start_ts>.mp4`` — 영상 파일
* ``<base>/<prefix>_<start_ts>.jsonl`` — frame-level sidecar
  (각 줄: ``{"frame_index": N, "stamp": {"sec": ..., "nanosec": ...}}``)
* ``<base>/<prefix>_metadata.json`` — recording metadata
  (``resolution`` / ``encoding`` / ``frame_id`` / ``is_bigendian`` / ``nframe``
  / ``start_ts`` / ``end_ts``)

mp4 프레임과 sidecar 라인은 1:1 로 매치된다. 자세한 사용법·파일 포맷은
``docs/recorder/rdfp_image_recorder_node_guide.md`` 참조.
"""

from __future__ import annotations

from typing import Any, Optional, TextIO

import collections
import json
import os
import sys
from datetime import datetime

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image

from rdfp_msgs.msg import SessionCommand

from ..types import Fps, Resolution
from ..ros2_utils import SYSTEM_QOS, get_parameter, parse_int, parse_stripped_str, parse_str
from .exceptions import EncoderUnavailableError, RecorderStateError
from .ffmpeg_mp4_recorder import FFMpegMp4Recorder


# recorder.stop() 의 타임아웃 (초). stop()이 이 시간 내에 완료되지 않으면 예외가 발생한다.
_STOP_TIMEOUT_SEC: float = 5.0
# 녹화 FPS 기본값. 카메라 퍼블리셔와 반드시 일치시켜야 한다
# (CFR 기반이므로 불일치 시 재생 속도가 어긋나거나 프레임이 drop 될 수 있다).
_DEFAULT_FPS: int = 10


class RdfpImageRecorderNode(Node):
    """타임스탬프 기반 녹화 경계를 갖는 이미지 레코더 노드.

    세션 토픽의 state 필드(`IN_EPISODE` / `IN_SESSION`)에 따라 녹화 구간을
    판정하고, `pending_image_queue`를 통해 도착 순서 차이를 보상한다.
    파라미터는 노드 기동 시 1회 로드되어 이후 불변으로 취급된다.

    녹화 1회마다 MP4, sidecar(jsonl), recording metadata(json) 3개 파일을
    생성한다. sidecar 는 매 프레임의 `frame_index`/`stamp` 를 기록하고,
    metadata 는 세션 전체의 통계와 원본 `sensor_msgs/Image` 재조립에 필요한
    메타 필드(encoding / frame_id / is_bigendian)를 담는다. 파일명 규칙과
    스키마는 모듈 docstring 및 `rdfp_image_recorder_node_guide.md` 참조.
    """

    def __init__(self, **node_kwargs: Any) -> None:
        """노드를 초기화한다.

        Args:
            **node_kwargs: `rclpy.node.Node.__init__`에 전달되는 추가 키워드
                인자. 테스트에서 `parameter_overrides=[...]` 등을 주입하는
                용도로 사용한다.
        """
        super().__init__('rdfp_image_recorder_node', **node_kwargs)

        # 1. 파라미터 선언 및 로드
        self._declare_parameters()
        self._load_parameters()

        # 2. 출력 디렉터리 준비
        self._prepare_output_dir()

        # 3. 파라미터 요약 로그
        self.get_logger().info(
            f'RdfpImageRecorderNode parameters loaded: '
            f'output_dir={self._output_dir} '
            f'session_prefix={self._session_prefix} '
            f'fps={self._fps} '
            f'resolution={self._resolution.width}x{self._resolution.height} '
            f'pixel_format={self._pixel_format} '
            f'encoder_mode={self._encoder_mode} '
            f'queue_size={self._queue_size} '
            f'pending_queue_length={self._pending_queue_length}'
        )

        # 4. Recorder 생성 (인코더 probe 1회 수행)
        self._recorder = self._create_recorder()
        self.get_logger().info(f'RdfpImageRecorderNode recorder ready: selected_codec='
                               f'{self._recorder.selected_codec}')

        # 5. 녹화 상태 변수 초기화
        self._recording: bool = False
        self._start_ts: int = 0  # 나노초
        self._stop_ts: int = 0   # 나노초. 노드 시작 직후 0으로 간주 (SRS §6.3.2)

        # 5-1. Sidecar / metadata 상태 변수 초기화
        #      sidecar 는 녹화 세션마다 open/close 되고, metadata 는 stop 시 1회 생성된다.
        self._sidecar_file: Optional[TextIO] = None
        self._start_ts_str: str = ''     # _build_output_path() 에서 생성된 타임스탬프 문자열
        self._frame_index: int = 0       # 다음에 기록될 sidecar frame_index (0부터 시작)
        self._first_stamp_ns: Optional[int] = None  # 녹화 중 첫 프레임 stamp (나노초)
        self._last_stamp_ns: Optional[int] = None   # 녹화 중 마지막 프레임 stamp (나노초)
        # metadata 에 기록되는 Image 메시지 메타 필드 (첫 유효 이미지에서 캡처)
        #   encoding 은 파라미터로부터 결정되므로 별도 필드 불필요 (_pixel_format 재사용)
        self._frame_id: str = ''
        self._is_bigendian: bool = False

        # 6. Pending image queue 초기화
        #    각 항목: (stamp_ns: int, ndarray)
        self._pending_queue: collections.deque[tuple[int, np.ndarray]] = (
            collections.deque(maxlen=self._pending_queue_length)
        )

        # 7. 이미지 토픽 구독
        self._image_sub = self.create_subscription(
            Image, 'image', self._on_image, qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'subscribed to image topic '
            f'(effective name={self._image_sub.topic_name})'
        )

        # 8. 세션 제어 토픽 구독 (publisher와 동일한 QoS)
        self._session_sub = self.create_subscription(
            SessionCommand, 'session', self._on_session, SYSTEM_QOS,
        )
        self.get_logger().info(
            f'subscribed to session topic '
            f'(effective name={self._session_sub.topic_name})'
        )

    # ---------- Parameter handling ------------------------------------------

    def _declare_parameters(self) -> None:
        """ROS2 파라미터를 선언한다.

        필수 파라미터는 ``output_dir`` / ``resolution`` 두 개이며, ``fps`` 는
        :data:`_DEFAULT_FPS` (10 FPS) 를 기본값으로 가진다 — 입력 이미지
        스트림의 실제 frame rate 와 반드시 일치시켜야 한다. 그 외에 파일명
        prefix(``session_prefix``), recorder 핵심 설정(``pixel_format``,
        ``encoder_mode``, ``queue_size``), recorder 에 passthrough 되는
        설정(``bitrate``, ``gop_size``, ``preset``, ``preferred_hw_codec``,
        ``ffmpeg_binary``, ``vaapi_device``), 그리고 pending image queue 의
        최대 길이(``pending_queue_length``) 를 함께 선언한다.
        """
        # 출력 관련 — output_dir은 필수
        self.declare_parameter('output_dir', descriptor=ParameterDescriptor(dynamic_typing=True),)
        self.declare_parameter('session_prefix', 'session')

        # Recorder 핵심 설정 — resolution은 필수, fps는 기본값(10) 보유
        self.declare_parameter('fps', _DEFAULT_FPS)
        self.declare_parameter('resolution', descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('pixel_format', 'bgr8')
        self.declare_parameter('encoder_mode', 'auto')
        self.declare_parameter('queue_size', 120)

        # Recorder 선택 설정 (passthrough)
        self.declare_parameter('bitrate', '4M')
        self.declare_parameter('gop_size', 0)
        self.declare_parameter('preset', 'medium')
        self.declare_parameter('preferred_hw_codec', '')
        self.declare_parameter('ffmpeg_binary', 'ffmpeg')
        self.declare_parameter('vaapi_device', '/dev/dri/renderD128')

        # Pending queue 설정
        self.declare_parameter('pending_queue_length', 60)

    def _load_parameters(self) -> None:
        """파라미터 값을 인스턴스 속성으로 로드한다.

        ``fps`` 는 미지정 시 :data:`_DEFAULT_FPS` (10) 가 사용된다.
        ``output_dir`` / ``resolution`` 은 필수 파라미터로, 미지정 시 예외가
        발생한다.

        Raises:
            ValueError: 필수 파라미터가 누락되었거나 값이 유효하지 않은 경우
                (예: ``fps`` 가 양의 정수가 아님).
        """
        # 출력 — output_dir은 필수 파라미터
        self._output_dir = get_parameter(self, 'output_dir', parse_stripped_str)
        self._session_prefix = get_parameter(self, 'session_prefix', parse_str)

        # Recorder 핵심 설정
        self._fps = get_parameter(self, 'fps', Fps).to_int()
        self._resolution = get_parameter(self, 'resolution', Resolution.parse)
        self._pixel_format = get_parameter(self, 'pixel_format', parse_str)
        self._encoder_mode = get_parameter(self, 'encoder_mode', parse_str)
        self._queue_size = get_parameter(self, 'queue_size', parse_int)

        # Recorder 선택 설정
        self._bitrate = get_parameter(self, 'bitrate', parse_str)
        self._gop_size = get_parameter(self, 'gop_size', parse_int)
        self._preset = get_parameter(self, 'preset', parse_str)
        self._preferred_hw_codec = get_parameter(self, 'preferred_hw_codec', parse_str)
        self._ffmpeg_binary = get_parameter(self, 'ffmpeg_binary', parse_str)
        self._vaapi_device = get_parameter(self, 'vaapi_device', parse_str)

        # Pending queue 설정
        self._pending_queue_length = get_parameter(self, 'pending_queue_length', parse_int)

    # ---------- Output directory --------------------------------------------

    def _prepare_output_dir(self) -> None:
        """`output_dir`이 존재하지 않으면 생성한다.

        Raises:
            RuntimeError: 디렉터리 생성에 실패한 경우 (권한 부족 등).
        """
        try:
            os.makedirs(self._output_dir, exist_ok=True)
        except OSError as exc:
            raise RuntimeError(
                f'failed to create output_dir {self._output_dir!r}: {exc}'
            ) from exc

    # ---------- Recorder construction ---------------------------------------

    def _create_recorder(self) -> FFMpegMp4Recorder:
        """파라미터를 바탕으로 `FFMpegMp4Recorder` 인스턴스를 생성한다.

        Raises:
            RuntimeError: 생성자에서 `ValueError` 또는 `EncoderUnavailableError`
                가 발생한 경우. `main()`이 FATAL로 보고하고 종료한다.
        """
        gop = self._gop_size if self._gop_size > 0 else None
        hw_codec = self._preferred_hw_codec or None

        try:
            return FFMpegMp4Recorder(
                fps=self._fps,
                resolution=self._resolution,
                pixel_format=self._pixel_format,
                encoder_mode=self._encoder_mode,
                preferred_hw_codec=hw_codec,
                bitrate=self._bitrate,
                gop_size=gop,
                preset=self._preset,
                ffmpeg_binary=self._ffmpeg_binary,
                vaapi_device=self._vaapi_device,
                queue_size=self._queue_size,
            )
        except EncoderUnavailableError as exc:
            raise RuntimeError(f'GPU encoder unavailable: {exc}') from exc
        except ValueError as exc:
            raise RuntimeError(f'invalid recorder configuration: {exc}') from exc

    # ---------- Image callback ----------------------------------------------

    def _on_image(self, msg: Image) -> None:
        """이미지 토픽 콜백.

        수신한 이미지를 검증·변환하여 `pending_image_queue`에 삽입한다.
        큐가 가득 차면 가장 오래된 프레임(victim)이 밀려나며, 녹화/비녹화
        구간에 따라 recorder에 전달하거나 drop한다 (SRS §6.3).
        """
        # encoding / 해상도 검증
        if (
            msg.encoding != self._pixel_format
            or msg.width != self._resolution.width
            or msg.height != self._resolution.height
        ):
            self.get_logger().warning(
                f'image dropped: encoding={msg.encoding} '
                f'size={msg.width}x{msg.height} '
                f'(expected encoding={self._pixel_format} '
                f'size={self._resolution.width}x{self._resolution.height})',
                throttle_duration_sec=5.0,
            )
            return

        # numpy 변환
        try:
            frame = self._msg_to_ndarray(msg)
        except ValueError as exc:
            self.get_logger().warning(
                f'image dropped: conversion failed ({exc})',
                throttle_duration_sec=5.0,
            )
            return

        # metadata 에 기록할 Image 메타 필드 캡처 (카메라별로 상수이므로 매번 덮어써도 무해)
        self._frame_id = msg.header.frame_id
        self._is_bigendian = bool(msg.is_bigendian)

        # 타임스탬프를 나노초 정수로 변환
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        # stop 이후 도착한 뒤늦은 녹화 대상 프레임 감지 (SRS §6.5.4)
        if not self._recording and self._stop_ts > 0 and stamp_ns < self._stop_ts:
            self.get_logger().warning(
                'late frame after stop: frame_ts < stop_ts, dropping',
                throttle_duration_sec=5.0,
            )
            return

        # overflow 감지: 큐가 가득 찬 상태에서 삽입하면 victim 발생
        victim: Optional[tuple[int, np.ndarray]] = None
        if len(self._pending_queue) == self._pending_queue.maxlen:
            victim = self._pending_queue[0]

        self._pending_queue.append((stamp_ns, frame))

        if victim is not None:
            self._process_victim(victim)

    def _process_victim(self, victim: tuple[int, np.ndarray]) -> None:
        """overflow로 밀려난 프레임을 녹화/비녹화 구간에 따라 처리한다 (SRS §6.3).

        녹화 구간: frame_ts >= start_ts → recorder.write(), 아니면 drop
        비녹화 구간: frame_ts < stop_ts → recorder.write(), 아니면 drop
        """
        frame_ts, frame = victim

        if self._recording:
            # 녹화 구간 (SRS §6.3.1)
            if frame_ts >= self._start_ts:
                self._write_frame(frame_ts, frame)
            # frame_ts < start_ts인 victim은 묵시적 drop
        else:
            # 비녹화 구간 (SRS §6.3.2)
            if frame_ts < self._stop_ts:
                self._write_frame(frame_ts, frame)
            # frame_ts >= stop_ts인 victim은 묵시적 drop

    def _msg_to_ndarray(self, msg: Image) -> np.ndarray:
        """`sensor_msgs/Image`를 `numpy.ndarray`로 변환한다.

        `bgr8`/`rgb8`은 `(H, W, 3)`, `mono8`은 `(H, W)` shape을 반환한다.
        `msg.step`이 `W * channels`와 다르면 stride padding이 붙어있는
        상태이므로 행별로 유효 픽셀만 슬라이싱한 뒤 contiguous 버퍼로 복사한다.

        호출자는 encoding/해상도가 이미 파라미터와 일치함을 보장해야 한다.

        Raises:
            ValueError: `msg.data` 길이가 `step * height` 미만인 경우.
        """
        height = msg.height
        width = msg.width
        step = msg.step

        if self._pixel_format == 'mono8':
            channels = 1
            target_shape: tuple[int, ...] = (height, width)
        else:
            channels = 3
            target_shape = (height, width, 3)

        expected_row = width * channels
        buffer = np.frombuffer(msg.data, dtype=np.uint8)
        if buffer.size < step * height:
            raise ValueError(
                f'image buffer too small: got {buffer.size} bytes, '
                f'expected at least {step * height}'
            )

        if step == expected_row:
            return buffer[: step * height].reshape(target_shape)

        # stride padding 존재 — 행별로 유효 픽셀만 추출
        rows = buffer[: step * height].reshape(height, step)
        trimmed = rows[:, :expected_row]
        if channels == 3:
            trimmed = trimmed.reshape(height, width, 3)
        return np.ascontiguousarray(trimmed)

    @staticmethod
    def _stamp_to_ns(stamp) -> int:
        """ROS2 `builtin_interfaces/Time`을 나노초 정수로 변환한다."""
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    # ---------- Session callback --------------------------------------------

    def _on_session(self, msg: SessionCommand) -> None:
        """세션 토픽 콜백.

        `state == "IN_EPISODE"` → start 처리 (SRS §6.4)
        `state == "IN_SESSION"` → stop 처리 (SRS §6.5)
        그 외 상태(`IDLE` 등)는 무시한다.
        """
        if msg.state == 'IN_EPISODE':
            self._handle_start(msg)
        elif msg.state == 'IN_SESSION':
            self._handle_stop(msg)

    def _handle_start(self, msg: SessionCommand) -> None:
        """녹화 시작 처리 (SRS §6.4).

        1. start_ts 기록 (`msg.header.stamp` 를 나노초 정수로 변환)
        2. 큐에서 frame_ts < start_ts 인 프레임 drop
        3. `recorder.start(output_path)` 호출 → 성공 시 녹화 구간으로 전환
        4. 세션용 sidecar(jsonl) 파일 open 및 frame_index / stamp 추적 변수 초기화

        이미 녹화 중이면 경고 로그만 남기고 no-op 이다. recorder.start 가
        실패하면 녹화 구간으로 전환하지 않고 sidecar 도 열지 않는다.
        """
        if self._recording:
            self.get_logger().warning('start received but already recording, ignoring')
            return

        self._start_ts = self._stamp_to_ns(msg.header.stamp)

        # 큐에서 start_ts 이전 프레임 제거
        dropped = 0
        while self._pending_queue and self._pending_queue[0][0] < self._start_ts:
            self._pending_queue.popleft()
            dropped += 1
        if dropped > 0:
            self.get_logger().info(f'start: dropped {dropped} frames with frame_ts < start_ts')

        # 파일명 생성 및 recorder 시작
        output_path = self._build_output_path()
        try:
            self._recorder.start(output_path)
        except FileExistsError as exc:
            self.get_logger().error(f'start failed: output file exists ({exc})')
            return
        except RecorderStateError as exc:
            self.get_logger().error(f'start rejected by recorder: {exc}')
            return
        except Exception as exc:
            self.get_logger().error(f'start failed: {exc}')
            return

        self._recording = True

        # sidecar 파일 open 및 카운터 초기화 (recorder.start 성공 후)
        self._open_sidecar()

        self.get_logger().info(
            f'recording started: path={output_path} '
            f'codec={self._recorder.selected_codec}'
        )

    def _handle_stop(self, msg: SessionCommand) -> None:
        """녹화 종료 처리 (SRS §6.5).

        1. stop_ts 기록, 비녹화 구간 전환
        2. 큐에서 frame_ts < stop_ts 인 프레임을 recorder 에 전달
           (sidecar 에도 함께 기록되어 mp4 ↔ sidecar 1:1 매치 유지)
        3. `recorder.stop()` 호출
        4. sidecar 파일 close 및 recording metadata(json) 작성

        녹화 중이 아니면 no-op 이다. `recorder.stop()` 이 예외로 실패하면
        sidecar 는 close 하되 metadata 는 작성하지 않는다.
        """
        if not self._recording:
            return

        self._stop_ts = self._stamp_to_ns(msg.header.stamp)
        self._recording = False

        # 큐에서 stop_ts 이전 프레임을 recorder에 전달 (sidecar 도 함께 기록됨)
        flushed = 0
        while self._pending_queue and self._pending_queue[0][0] < self._stop_ts:
            stamp_ns, frame = self._pending_queue.popleft()
            if self._write_frame(stamp_ns, frame):
                flushed += 1

        # recorder 종료
        try:
            returned_path = self._recorder.stop(timeout=_STOP_TIMEOUT_SEC)
        except Exception as exc:
            self.get_logger().error(f'recorder.stop failed: {exc}')
            self._close_sidecar()
            return

        # sidecar close + metadata 작성
        self._close_sidecar()
        self._write_metadata()

        post_state = self._recorder.state
        if post_state == 'FAILED':
            self.get_logger().error(
                f'recording stopped with FAILED state: path={returned_path} '
                f'frames_written={self._recorder.frames_written}'
            )
        else:
            self.get_logger().info(
                f'recording stopped: path={returned_path} '
                f'frames_written={self._recorder.frames_written} '
                f'flushed_from_queue={flushed}'
            )

    def _build_output_path(self) -> str:
        """출력 MP4 파일 경로를 생성한다 (SRS §8).

        포맷: ``<output_dir>/<session_prefix>_YYYYMMDD-HHMMSS.SSS.mp4``

        생성된 타임스탬프 문자열(`YYYYMMDD-HHMMSS.SSS`)은 sidecar/metadata
        파일명 생성에 재사용되도록 ``self._start_ts_str`` 에 저장된다.
        """
        now = datetime.now()
        ms = now.microsecond // 1000
        ts = now.strftime('%Y%m%d-%H%M%S') + f'.{ms:03d}'
        self._start_ts_str = ts
        filename = f'{self._session_prefix}_{ts}.mp4'
        return os.path.join(self._output_dir, filename)

    def _build_sidecar_path(self) -> str:
        """sidecar 파일 경로를 생성한다.

        포맷: ``<output_dir>/<session_prefix>_<start_ts>.jsonl``
        """
        filename = f'{self._session_prefix}_{self._start_ts_str}.jsonl'
        return os.path.join(self._output_dir, filename)

    def _build_metadata_path(self) -> str:
        """recording metadata 파일 경로를 생성한다.

        포맷: ``<output_dir>/<session_prefix>_metadata.json`` (start_ts 미포함)
        """
        filename = f'{self._session_prefix}_metadata.json'
        return os.path.join(self._output_dir, filename)

    # ---------- Sidecar / metadata 쓰기 ------------------------------------

    def _open_sidecar(self) -> None:
        """현재 세션 용 sidecar(jsonl) 파일을 열고 카운터를 리셋한다.

        호출 시점: recorder.start() 성공 직후.
        실패하면 경고 로그만 남기고 `_sidecar_file` 을 None 상태로 둔다
        (녹화는 계속 진행되지만 sidecar/metadata 는 기록되지 않는다).
        """
        self._frame_index = 0
        self._first_stamp_ns = None
        self._last_stamp_ns = None

        sidecar_path = self._build_sidecar_path()
        try:
            self._sidecar_file = open(sidecar_path, 'w', encoding='utf-8')
            self.get_logger().info(f'sidecar opened: path={sidecar_path}')
        except OSError as exc:
            self._sidecar_file = None
            self.get_logger().error(f'sidecar open failed: path={sidecar_path} error={exc}')

    def _close_sidecar(self) -> None:
        """sidecar 파일을 flush 후 close 한다 (idempotent)."""
        if self._sidecar_file is None:
            return
        try:
            self._sidecar_file.flush()
            self._sidecar_file.close()
        except OSError as exc:
            self.get_logger().warning(f'sidecar close failed: {exc}')
        self._sidecar_file = None

    def _write_frame(self, stamp_ns: int, frame: np.ndarray) -> bool:
        """프레임을 recorder 에 기록하고 sidecar 에 한 줄 append 한다.

        recorder.write() 성공 시에만 sidecar 기록과 카운터 증가가 수행되어
        "mp4 프레임 ↔ sidecar 라인 1:1 매치" 불변식을 유지한다.

        Returns:
            recorder.write 가 성공했으면 True, 실패했으면 False.
        """
        try:
            self._recorder.write(frame)
        except Exception as exc:
            self.get_logger().warning(
                f'recorder.write failed: {exc}', throttle_duration_sec=5.0,
            )
            return False

        # recorder.write 성공 — sidecar 에 한 줄 기록
        if self._sidecar_file is not None:
            record = {
                'frame_index': self._frame_index,
                'stamp': {
                    'sec': stamp_ns // 1_000_000_000,
                    'nanosec': stamp_ns % 1_000_000_000,
                },
            }
            try:
                self._sidecar_file.write(json.dumps(record) + '\n')
                self._sidecar_file.flush()
            except OSError as exc:
                self.get_logger().warning(
                    f'sidecar write failed: {exc}', throttle_duration_sec=5.0,
                )

        # 프레임 통계 갱신 (sidecar 실패 여부와 무관하게 계속 추적)
        if self._first_stamp_ns is None:
            self._first_stamp_ns = stamp_ns
        self._last_stamp_ns = stamp_ns
        self._frame_index += 1
        return True

    def _write_metadata(self) -> None:
        """recording metadata JSON 파일을 작성한다.

        기록 정보:
          * resolution — 영상 해상도 (``WxH``)
          * encoding — pixel format (예: ``bgr8``) — 노드 파라미터에서 가져옴
          * frame_id — 첫 유효 이미지의 ``header.frame_id``
          * is_bigendian — 첫 유효 이미지의 ``is_bigendian`` (대부분 ``false``)
          * nframe — 총 프레임 수
          * start_ts / end_ts — 첫/마지막 프레임 stamp (sec/nanosec)

        프레임이 하나도 기록되지 않았으면 start_ts/end_ts 는 생략된다.
        """
        metadata_path = self._build_metadata_path()
        metadata: dict[str, Any] = {
            'resolution': f'{self._resolution.width}x{self._resolution.height}',
            'encoding': self._pixel_format,
            'frame_id': self._frame_id,
            'is_bigendian': self._is_bigendian,
            'nframe': self._frame_index,
        }
        if self._first_stamp_ns is not None:
            metadata['start_ts'] = {
                'sec': self._first_stamp_ns // 1_000_000_000,
                'nanosec': self._first_stamp_ns % 1_000_000_000,
            }
        if self._last_stamp_ns is not None:
            metadata['end_ts'] = {
                'sec': self._last_stamp_ns // 1_000_000_000,
                'nanosec': self._last_stamp_ns % 1_000_000_000,
            }

        try:
            with open(metadata_path, 'w', encoding='utf-8') as fp:
                json.dump(metadata, fp, indent=2)
                fp.write('\n')
            self.get_logger().info(
                f'metadata written: path={metadata_path} nframe={self._frame_index}'
            )
        except OSError as exc:
            self.get_logger().error(
                f'metadata write failed: path={metadata_path} error={exc}'
            )

    # ---------- Lifecycle ---------------------------------------------------

    def destroy_node(self) -> None:
        """노드 종료 시 큐를 flush 하고 recorder / sidecar 를 안전하게 정리한다 (SRS §6.6).

        SIGINT / SIGTERM / `rclpy.shutdown()` 경로에서도 호출되도록 `main()`의
        `finally` 블록에서 보장된다. 큐에 남은 녹화 대상 프레임을 §6.3 규칙에
        따라 처리한 뒤, 녹화 중이면 `recorder.stop()` 으로 finalize 하고
        sidecar close / recording metadata 기록을 수행한다. 마지막으로
        `recorder.shutdown()` 을 호출한다 (idempotent). sidecar 가 열린 채로
        남아 있으면 녹화 중이 아니더라도 안전하게 close 한다.
        """
        recorder = getattr(self, '_recorder', None)
        pending_queue = getattr(self, '_pending_queue', None)

        if recorder is not None and pending_queue is not None:
            # 큐에 남은 프레임을 §6.3 규칙에 따라 처리 (sidecar 도 함께 기록됨)
            flushed = 0
            while pending_queue:
                stamp_ns, frame = pending_queue.popleft()
                if self._recording and stamp_ns >= self._start_ts:
                    if self._write_frame(stamp_ns, frame):
                        flushed += 1
                elif not self._recording and self._stop_ts > 0 and stamp_ns < self._stop_ts:
                    if self._write_frame(stamp_ns, frame):
                        flushed += 1
            if flushed > 0:
                self.get_logger().info(
                    f'destroy_node: flushed {flushed} frames from pending queue'
                )

            # 녹화 중이면 stop으로 finalize
            if recorder.state == 'RECORDING':
                self.get_logger().info(
                    'destroy_node: stopping active session before shutdown'
                )
                try:
                    returned_path = recorder.stop(timeout=_STOP_TIMEOUT_SEC)
                    self.get_logger().info(
                        f'destroy_node: session stopped: path={returned_path} '
                        f'frames_written={recorder.frames_written}'
                    )
                    # sidecar close + metadata 기록
                    self._close_sidecar()
                    self._write_metadata()
                except Exception as exc:
                    self.get_logger().error(
                        f'destroy_node: recorder.stop failed: {exc}'
                    )
                    self._close_sidecar()
            else:
                # 녹화 중이 아니어도 sidecar 가 열린 상태일 수 있으니 안전하게 close
                self._close_sidecar()

            # recorder shutdown (idempotent)
            try:
                recorder.shutdown()
            except Exception as exc:
                self.get_logger().error(
                    f'destroy_node: recorder.shutdown failed: {exc}'
                )

        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    """콘솔 엔트리 포인트.

    `SingleThreadedExecutor`로 노드를 spin한다. SIGINT / Ctrl+C /
    `ExternalShutdownException` 경로에서도 `destroy_node()`와
    `rclpy.try_shutdown()`이 반드시 호출되도록 finally 블록으로 보호한다.
    """
    rclpy.init(args=args)

    from ..logging_bridge import configure_logging_bridge
    configure_logging_bridge(package_logger_name='rdfp')

    node: Optional[RdfpImageRecorderNode] = None
    try:
        node = RdfpImageRecorderNode()
    except Exception as exc:
        print(f'[FATAL] RdfpImageRecorder init failed: {exc}', file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(1)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
