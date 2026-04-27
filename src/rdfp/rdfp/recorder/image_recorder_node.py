#!/usr/bin/env python3

"""ImageRecorderNode 모듈.

ROS2 토픽의 `sensor_msgs/Image` 스트림을 MP4 파일로 녹화하는 ROS2 노드.
녹화 엔진은 `rdfp.recorder.FFMpegMp4Recorder` 를 재사용하며, 본 모듈은
ROS2 인터페이스 ↔ recorder 간 얇은 어댑터 역할을 수행한다.

구현 단계별 상세는 `docs/image_recorder_node_srs.md` 와
`docs/image_recorder_node_dev_plan.md` 를 참고한다.
"""

from __future__ import annotations

from typing import Any, Optional

import os
import sys
from datetime import datetime

import numpy as np

import rclpy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image

from rdfp_msgs.srv import StartSession, StopSession

from ..types import InvalidFrameError, Resolution, Fps
from ..ros2_utils import get_optional_parameter, get_parameter, parse_bool, parse_str, parse_int
from .exceptions import (
    EncoderUnavailableError,
    RecorderStateError,
)
from .ffmpeg_mp4_recorder import FFMpegMp4Recorder


_STOP_TIMEOUT_SEC: float = 5.0
# 연속 불일치 프레임이 이 수를 채우면 세션을 자동 종료한다.
_MAX_CONSECUTIVE_INVALID_FRAMES: int = 5
# 동일 사유의 드롭 ERROR 로그는 (첫 1회) + (이후 이 주기마다 1회) 만 남긴다.
_INVALID_LOG_STRIDE: int = 100

_DEFAULT_SESSION_PREFIX: str = "recording"
# 녹화 FPS 기본값. 카메라 퍼블리셔와 반드시 일치시켜야 한다
# (CFR 기반이므로 불일치 시 재생 속도가 어긋나거나 프레임이 drop 될 수 있다).
_DEFAULT_FPS: int = 10


class ImageRecorderNode(Node):
    """`sensor_msgs/Image` 를 MP4 로 녹화하는 ROS2 노드.

    파라미터는 노드 기동 시점에 1 회 로드되어 이후 불변으로 취급된다.
    녹화 엔진 생성 / 구독 / 서비스 / 콜백을 포함한 녹화 생명주기를
    단일 노드에서 관리한다.
    """

    def __init__(self, **node_kwargs: Any) -> None:
        """노드를 초기화한다.

        파라미터 선언/로드 → 출력 디렉터리 준비 → recorder 생성 → 토픽 구독 →
        서비스 등록 순으로 진행한다. ``auto_start`` 가 ``true`` 이면 마지막
        단계에서 ``_handle_start_session`` 을 직접 호출해 즉시 녹화를 시작한다
        (실패해도 노드는 계속 살아 있고 사용자가 수동으로 재시도할 수 있다).

        Args:
            **node_kwargs: `rclpy.node.Node.__init__` 에 전달되는 추가 키워드
                인자. 테스트에서 `parameter_overrides=[...]` 등을 주입하는
                용도로 사용한다.
        """
        super().__init__("image_recorder", **node_kwargs)

        # 1. 파라미터 선언 및 로드
        self._declare_parameters()
        self._load_parameters()

        # 2. 출력 디렉터리 준비
        self._prepare_output_dir()

        # 3. 세션 상태 변수 초기화
        self._current_mp4_path: Optional[str] = None
        self._consecutive_invalid: int = 0
        self._invalid_log_count: int = 0
        # resolution 미지정 시 deferred 모드용 상태
        # _effective_resolution 은 실제로 recorder 가 사용하는 해상도이며,
        # _resolution 이 지정된 경우 그 값, 아니면 첫 수신 이미지에서 추론된 값.
        self._effective_resolution: Optional[Resolution] = self._resolution
        self._pending_start: bool = False
        self._pending_mp4_path: Optional[str] = None

        # 4. 파라미터 요약 로그
        res_text = (f"{self._resolution.width}x{self._resolution.height}"
                    if self._resolution is not None else "auto (from first image)")
        self.get_logger().info(
            f"ImageRecorderNode parameters loaded: "
            f"output_dir={self._output_dir} "
            f"session_prefix={self._session_prefix} "
            f"fps={self._fps} "
            f"resolution={res_text} "
            f"pixel_format={self._pixel_format} "
            f"encoder_mode={self._encoder_mode} "
            f"queue_size={self._queue_size}"
        )

        # 5. Recorder 생성 — resolution 이 지정된 경우에만 즉시 생성한다.
        #    미지정 시 첫 수신 이미지에서 추론하여 _on_image 콜백에서 lazy 생성한다.
        self._recorder: Optional[FFMpegMp4Recorder] = None
        if self._effective_resolution is not None:
            self._recorder = self._create_recorder()
            self.get_logger().info(f"recorder ready: selected_codec={self._recorder.selected_codec}")
        else:
            self.get_logger().info(
                "recorder creation deferred: resolution will be inferred from first image"
            )

        # 6. 이미지 토픽 구독 (기본 이름 "image", remap 으로 override)
        #    사용 예: --ros-args -r image:=/camera/color/image_raw
        self._image_sub = self.create_subscription(Image, "image", self._on_image, qos_profile_sensor_data,)
        self.get_logger().info(f"subscribed to image topic (effective name={self._image_sub.topic_name})")

        # 7. 생명주기 서비스 생성 (노드 namespace 기준 → /image_recorder/...)
        self._start_srv = self.create_service(StartSession, "~/start_session", self._handle_start_session)
        self._stop_srv = self.create_service(StopSession, "~/stop_session", self._handle_stop_session)
        self.get_logger().info(f"services ready: {self._start_srv.srv_name}, {self._stop_srv.srv_name}")

        # 8. auto_start=true 이면 서비스 콜과 동일 경로로 세션을 즉시 시작한다.
        #    실패해도 노드는 살아있으며, 사용자가 수동으로 ~/start_session 을 호출해 재시도할 수 있다.
        if self._auto_start:
            self.get_logger().info("auto_start=true: starting session automatically")
            req = StartSession.Request()
            resp = StartSession.Response()
            self._handle_start_session(req, resp)
            if not resp.success:
                self.get_logger().error("auto_start failed; waiting for manual ~/start_session call")

    # ---------- Parameter handling ------------------------------------------

    def _declare_parameters(self) -> None:
        """ROS2 파라미터를 선언한다.

        모든 파라미터는 기본값을 가진다. ``fps`` 는 :data:`_DEFAULT_FPS`
        (10 FPS) 를 사용하며, 입력 이미지 스트림의 실제 frame rate 와
        반드시 일치시켜야 한다.
        """
        # 출력 관련
        self.declare_parameter("output_dir", "/tmp/recordings")
        self.declare_parameter("session_prefix", _DEFAULT_SESSION_PREFIX)

        # Recorder 핵심 설정
        self.declare_parameter("fps", _DEFAULT_FPS)
        self.declare_parameter("resolution", descriptor=ParameterDescriptor(dynamic_typing=True),)
        self.declare_parameter("pixel_format", "bgr8")
        self.declare_parameter("encoder_mode", "auto")
        self.declare_parameter("queue_size", 120)

        # 노드 기동 시 자동으로 녹화 세션을 시작할지 여부
        self.declare_parameter("auto_start", False)

    def _load_parameters(self) -> None:
        """파라미터 값을 인스턴스 속성으로 로드한다.

        ``fps`` 는 미지정 시 :data:`_DEFAULT_FPS` (10) 가 사용된다.
        ``resolution`` 은 optional 로, 미지정 시 첫 수신 이미지의 해상도를
        사용한다.

        Raises:
            ValueError: 파라미터 값이 유효하지 않은 경우 (예: `resolution` 형식
                오류, `fps` 가 양수가 아닌 경우, `auto_start` 가 bool 이 아닌 경우 등).
        """
        # 출력
        self._output_dir = get_parameter(self, "output_dir", parse_str)
        self._session_prefix = self._validate_session_prefix(
                                                get_parameter(self, "session_prefix", parse_str))

        # 핵심 설정
        self._fps = get_parameter(self, "fps", Fps).to_int()
        # resolution 은 optional — 미지정 시 첫 수신 이미지의 해상도를 사용한다.
        self._resolution = get_optional_parameter(self, "resolution", Resolution.parse)
        self._pixel_format = get_parameter(self, "pixel_format", parse_str)
        self._encoder_mode = get_parameter(self, "encoder_mode", parse_str)
        self._queue_size = get_parameter(self, "queue_size", parse_int)
        self._auto_start: bool = get_parameter(self, "auto_start", parse_bool)

    # ---------- Service handlers --------------------------------------------

    def _handle_start_session(self, request: StartSession.Request,
                              response: StartSession.Response,) -> StartSession.Response:
        """`~start_session` 서비스 핸들러.

        recorder 가 이미 RECORDING 이거나 deferred 시작이 진행 중이면 실패
        응답을 반환한다. 그 외의 경우 파일명을 생성하고:

        - recorder 가 이미 존재하면(`resolution` 지정된 경로) `recorder.start()`
          를 즉시 호출한다.
        - recorder 가 아직 없으면(`resolution` 미지정 → 첫 이미지로 추론) 경로만
          예약하고 pending 상태로 진입한다. 실제 `recorder.start()` 는 첫 유효
          이미지가 도착했을 때 `_on_image` 에서 호출된다.

        `FileExistsError` / `RecorderStateError` / 그 외 예외는 모두 흡수해
        실패 응답으로 변환한다 (콜백 밖으로 전파되지 않는다).
        """
        del request  # empty 요청이므로 사용하지 않음

        if self._pending_start:
            self.get_logger().warning("start_session rejected: pending start in progress "
                f"(planned={self._pending_mp4_path})")
            response.success = False
            response.mp4_path = ""
            return response

        if self._recorder is not None and self._recorder.state == "RECORDING":
            self.get_logger().warning("start_session rejected: already recording "
                f"(current={self._current_mp4_path})")
            response.success = False
            response.mp4_path = ""
            return response

        mp4_path = self._build_output_path()

        # recorder 가 아직 없으면 첫 이미지가 도착할 때까지 deferred 모드로 진입
        if self._recorder is None:
            self._pending_start = True
            self._pending_mp4_path = mp4_path
            self._consecutive_invalid = 0
            self._invalid_log_count = 0
            self.get_logger().info(
                f"start_session pending: waiting for first image to infer resolution "
                f"(planned path={mp4_path})"
            )
            response.success = True
            response.mp4_path = mp4_path
            return response

        try:
            self._recorder.start(mp4_path)
        except FileExistsError as exc:
            self.get_logger().error(f"start_session failed: {exc}")
            response.success = False
            response.mp4_path = ""
            return response
        except RecorderStateError as exc:
            self.get_logger().error(f"start_session rejected by recorder: {exc}")
            response.success = False
            response.mp4_path = ""
            return response
        except Exception as exc:
            self.get_logger().error(f"start_session failed: {exc}")
            response.success = False
            response.mp4_path = ""
            return response

        # 성공 — 세션 상태 리셋
        self._current_mp4_path = mp4_path
        self._consecutive_invalid = 0
        self._invalid_log_count = 0

        self.get_logger().info(f"session started: path={mp4_path} codec={self._recorder.selected_codec}")
        response.success = True
        response.mp4_path = mp4_path
        return response

    def _handle_stop_session(self, request: StopSession.Request,
                             response: StopSession.Response,) -> StopSession.Response:
        """`~stop_session` 서비스 핸들러.

        현재 recorder 상태가 `RECORDING` 이 아니면 실패 응답을 반환한다.
        그 외의 경우 `recorder.stop(timeout=5.0)` 을 호출하고 이후 상태를
        검사해 성공/실패를 판정한다. writer 스레드가 RECORDING → FAILED 로
        전이하는 경합 등으로 `stop()` 이 예외를 던지면 흡수하여 실패 응답으로
        변환한다 (콜백 밖으로 전파되지 않는다).
        """
        del request  # empty 요청이므로 사용하지 않음

        # deferred 시작이 pending 중이면 (첫 이미지 도착 전) 예약을 취소한다.
        if self._pending_start:
            self.get_logger().warning(
                f"stop_session: cancelling pending start (no frames received yet, "
                f"planned={self._pending_mp4_path})"
            )
            self._pending_start = False
            self._pending_mp4_path = None
            response.success = False
            response.mp4_path = ""
            return response

        if self._recorder is None or self._recorder.state != "RECORDING":
            state = self._recorder.state if self._recorder is not None else "NO_RECORDER"
            self.get_logger().warning(f"stop_session rejected: not recording (state={state})")
            response.success = False
            response.mp4_path = ""
            return response

        # 반환 경로는 stop() 이 내부 저장 값을 그대로 돌려준다.
        # writer 스레드가 RECORDING → FAILED 로 전이하는 경합 시 RecorderStateError 가 발생할 수 있다.
        # 콜백 밖으로 예외가 전파되면 서비스 응답이 비어버리므로 여기서 흡수한다.
        try:
            returned_path = self._recorder.stop(timeout=_STOP_TIMEOUT_SEC)
        except RecorderStateError as exc:
            self.get_logger().error(f"stop_session failed: state race ({exc})")
            self._current_mp4_path = None
            self._consecutive_invalid = 0
            self._invalid_log_count = 0
            response.success = False
            response.mp4_path = ""
            return response
        except Exception as exc:
            self.get_logger().error(f"stop_session failed: recorder.stop raised {exc}")
            self._current_mp4_path = None
            self._consecutive_invalid = 0
            self._invalid_log_count = 0
            response.success = False
            response.mp4_path = ""
            return response

        post_state = self._recorder.state
        frames_written = self._recorder.frames_written

        self._current_mp4_path = None
        self._consecutive_invalid = 0
        self._invalid_log_count = 0

        if post_state == "IDLE":
            self.get_logger().info(f"session stopped: path={returned_path} frames_written={frames_written}")
            response.success = True
            response.mp4_path = returned_path
            return response

        # FAILED 또는 예기치 않은 상태
        self.get_logger().error(
            f"session stop reported failure: "
            f"state={post_state} path={returned_path} "
            f"frames_written={frames_written}"
        )
        response.success = False
        response.mp4_path = ""
        return response

    def _build_output_path(self) -> str:
        """`output_dir` / `session_prefix` / 현재 wall clock 으로 MP4 경로 생성.

        포맷: ``<output_dir>/<session_prefix>_YYYYMMDD-HHMMSS.SSS.mp4``

        `SSS` 는 밀리초 3 자리 (`dt.microsecond // 1000`).
        """
        now = datetime.now()
        ms = now.microsecond // 1000
        ts = now.strftime("%Y%m%d-%H%M%S") + f".{ms:03d}"
        filename = f"{self._session_prefix}_{ts}.mp4"
        return os.path.join(self._output_dir, filename)

    @staticmethod
    def _validate_session_prefix(prefix: str) -> str:
        """`session_prefix` 가 경로 탈출이 불가능한 단순 파일명 조각인지 검증한다.

        `session_prefix` 는 `_build_output_path` 에서 파일명 조각으로 그대로 삽입된다.
        절대 경로(`/tmp/pwn`)나 하위 경로(`nested/foo`), `..` 같은 성분이 들어가면
        ``output_dir`` 를 우회하거나 존재하지 않는 부모 디렉터리로 녹화를 시도하게 된다.
        노드 기동 시점에 fail-fast 하기 위해 단순 검증을 수행한다.

        Raises:
            ValueError: 빈 문자열, `.`/`..`, `/`/`\\`/OS 경로 구분자가 포함된 경우.
        """
        if not prefix or prefix in (".", ".."):
            raise ValueError(f"session_prefix cannot be empty, '.', or '..': {prefix!r}")
        if "/" in prefix or "\\" in prefix or os.sep in prefix:
            raise ValueError(f"session_prefix must not contain path separators: {prefix!r}")
        return prefix

    # ---------- Image callback ----------------------------------------------

    def _on_image(self, msg: Image) -> None:
        """이미지 토픽 콜백.

        deferred 모드(`pending_start=True`) 라면 첫 유효 이미지로 recorder 를
        지연 생성/시작하고, 그 외에는 일반 처리 경로를 탄다. 녹화 중이 아니거나
        메시지가 recorder 설정과 불일치하면 drop 한다. 정상 프레임만
        `recorder.write()` 로 전달한다.
        """
        # 1. deferred 모드: 첫 유효 이미지로 recorder 를 lazy 생성/시작한다.
        if self._pending_start and self._recorder is None:
            if not self._initialize_recorder_from_first_image(msg):
                # 첫 이미지가 유효하지 않거나 lazy 생성에 실패 — pending 유지하고
                # 다음 프레임에서 재시도한다 (단, 생성 실패는 pending 을 클리어함).
                return
            # recorder 가 이제 시작됐고, 동일 메시지를 정상 경로로 처리하기 위해 fall-through.

        # 2. 비녹화 상태는 조용히 drop (로그 없음)
        if self._recorder is None or self._recorder.state != "RECORDING":
            return

        # 3. encoding / 해상도 검증 — _effective_resolution 기준
        assert self._effective_resolution is not None  # 위 체크에서 recorder 가 RECORDING 이면 보장됨
        expected_w = self._effective_resolution.width
        expected_h = self._effective_resolution.height
        if msg.encoding != self._pixel_format or msg.width != expected_w or msg.height != expected_h:
            self._handle_invalid_frame(msg)
            return

        # 3. numpy 변환 후 recorder 로 전달
        try:
            frame = self._msg_to_ndarray(msg)
        except ValueError as exc:
            # 데이터 버퍼 길이 불일치 등 변환 실패도 invalid 프레임으로 취급한다.
            self._consecutive_invalid += 1
            self._invalid_log_count += 1
            if (
                self._invalid_log_count == 1
                or self._invalid_log_count % _INVALID_LOG_STRIDE == 0
            ):
                self.get_logger().error(
                    f"invalid frame dropped: conversion failed ({exc}) "
                    f"(consecutive={self._consecutive_invalid} "
                    f"total_dropped={self._invalid_log_count})"
                )
            if self._consecutive_invalid >= _MAX_CONSECUTIVE_INVALID_FRAMES:
                self._auto_stop_session()
            return

        # 변환까지 성공한 프레임만 invalid 연속 카운터를 리셋한다.
        self._consecutive_invalid = 0

        try:
            self._recorder.write(frame)
        except InvalidFrameError as exc:
            # 경계 케이스 (shape/dtype 불일치) — recorder 레벨에서 추가 거부.
            self.get_logger().warning(f"recorder rejected frame: {exc}")
        except RecorderStateError:
            # stop_session 과의 경합. 정상 시나리오의 일부로 취급.
            self.get_logger().debug("write dropped: recorder not in RECORDING")

    def _initialize_recorder_from_first_image(self, msg: Image) -> bool:
        """deferred 모드에서 첫 수신 이미지로부터 recorder 를 lazy 생성하고 시작한다.

        ``resolution`` 파라미터가 미지정이면 첫 이미지의 해상도를 사용해
        ``_effective_resolution`` 을 확정하고 recorder 를 생성한 뒤,
        ``_pending_mp4_path`` 로 ``recorder.start()`` 를 호출한다.

        첫 이미지가 유효성 검사를 통과하지 못하면 (`width`/`height` <= 0,
        encoding 불일치) ``False`` 를 반환하고 pending 상태를 유지한다.
        recorder 생성/시작 자체가 예외를 던지면 pending 을 클리어해 사용자가
        새 ``start_session`` 으로 재시도할 수 있도록 한다.

        Returns:
            ``True``: recorder 가 정상적으로 시작되어 호출자가 본 메시지를 정상
                경로로 계속 처리할 수 있는 상태.
            ``False``: 본 메시지를 처리하지 말고 다음 콜백에서 재시도해야 함.
        """
        if msg.width <= 0 or msg.height <= 0:
            self._consecutive_invalid += 1
            self._invalid_log_count += 1
            if self._invalid_log_count == 1 or self._invalid_log_count % _INVALID_LOG_STRIDE == 0:
                self.get_logger().error(
                    f"first image has invalid resolution {msg.width}x{msg.height}; "
                    f"waiting for a valid frame "
                    f"(consecutive={self._consecutive_invalid})"
                )
            return False

        if msg.encoding != self._pixel_format:
            self._consecutive_invalid += 1
            self._invalid_log_count += 1
            if self._invalid_log_count == 1 or self._invalid_log_count % _INVALID_LOG_STRIDE == 0:
                self.get_logger().error(
                    f"first image encoding {msg.encoding!r} does not match pixel_format "
                    f"{self._pixel_format!r}; waiting for matching frame "
                    f"(consecutive={self._consecutive_invalid})"
                )
            return False

        self._effective_resolution = Resolution(msg.width, msg.height)
        self.get_logger().info(
            f"resolution inferred from first image: "
            f"{self._effective_resolution.width}x{self._effective_resolution.height}"
        )

        try:
            self._recorder = self._create_recorder()
        except RuntimeError as exc:
            self.get_logger().error(
                f"deferred recorder creation failed: {exc}; "
                f"clearing pending start (call ~/start_session to retry)"
            )
            self._pending_start = False
            self._pending_mp4_path = None
            self._effective_resolution = None
            return False

        self.get_logger().info(f"recorder ready: selected_codec={self._recorder.selected_codec}")

        # _pending_start=True 일 때 _pending_mp4_path 는 _handle_start_session 에서 항상 설정된다.
        assert self._pending_mp4_path is not None
        pending_path: str = self._pending_mp4_path
        try:
            self._recorder.start(pending_path)
        except Exception as exc:
            self.get_logger().error(
                f"deferred start failed: {exc}; "
                f"clearing pending start (call ~/start_session to retry)"
            )
            self._pending_start = False
            self._pending_mp4_path = None
            return False

        self._current_mp4_path = pending_path
        self._pending_start = False
        self._pending_mp4_path = None
        self._consecutive_invalid = 0
        self._invalid_log_count = 0
        self.get_logger().info(
            f"deferred session started: path={pending_path} codec={self._recorder.selected_codec}"
        )
        return True

    def _msg_to_ndarray(self, msg: Image) -> np.ndarray:
        """`sensor_msgs/Image` 를 `numpy.ndarray` 로 변환한다.

        `bgr8` / `rgb8` 은 `(H, W, 3)`, `mono8` 은 `(H, W)` shape 을 반환한다.
        `msg.step` 이 `W * channels` 와 다르면 stride padding 이 붙어있는
        상태이므로 행별로 유효 픽셀만 슬라이싱한 뒤 contiguous 버퍼로
        복사한다.

        호출자는 encoding / 해상도가 이미 파라미터와 일치함을 보장해야 한다.

        Raises:
            ValueError: `msg.data` 가 `step * height` 미만이거나, `msg.step`
                이 한 행에 필요한 `width * channels` 보다 작은 경우.
        """
        height = msg.height
        width = msg.width
        step = msg.step

        if self._pixel_format == "mono8":
            channels = 1
            target_shape: tuple[int, ...] = (height, width)
        else:
            # bgr8 / rgb8
            channels = 3
            target_shape = (height, width, 3)

        expected_row = width * channels
        buffer = np.frombuffer(msg.data, dtype=np.uint8)
        if buffer.size < step * height:
            raise ValueError(
                f"image buffer too small: got {buffer.size} bytes, "
                f"expected at least {step * height}"
            )

        # mono8 처럼 step < expected_row 이면 stride padding 슬라이싱이 좁은 배열을 조용히 반환한다.
        # bgr8/rgb8 은 reshape(h,w,3) 단계에서 어차피 실패하지만, mono8 은 그대로 통과해 invalid
        # 정책을 우회하므로 여기서 명시적으로 거부한다.
        if step < expected_row:
            raise ValueError(f"image step {step} smaller than expected row {expected_row} "
                             f"(width={width}, channels={channels})")

        if step == expected_row:
            # stride padding 없음 — 단순 reshape 로 충분
            return buffer[: step * height].reshape(target_shape)

        # stride padding 존재 — 행별로 유효 픽셀만 추출
        rows = buffer[: step * height].reshape(height, step)
        trimmed = rows[:, :expected_row]
        if channels == 3:
            trimmed = trimmed.reshape(height, width, 3)
        return np.ascontiguousarray(trimmed)

    def _handle_invalid_frame(self, msg: Image) -> None:
        """불일치 프레임 처리.

        카운터를 증가시킨 뒤 억제 정책에 따라 ERROR 로그를 남기고, 연속
        불일치가 `_MAX_CONSECUTIVE_INVALID_FRAMES` 를 채우면 세션을 자동으로
        종료한다. 프레임은 recorder 에 전달되지 않는다.
        """
        self._consecutive_invalid += 1
        self._invalid_log_count += 1

        # 최초 1회 + 이후 _INVALID_LOG_STRIDE 건마다 1회 로그 (폭주 억제)
        if (
            self._invalid_log_count == 1
            or self._invalid_log_count % _INVALID_LOG_STRIDE == 0
        ):
            assert self._effective_resolution is not None  # invalid 검사 시점에 이미 확정됨
            self.get_logger().error(
                f"invalid frame dropped: "
                f"encoding={msg.encoding} size={msg.width}x{msg.height} "
                f"(expected encoding={self._pixel_format} "
                f"size={self._effective_resolution.width}x{self._effective_resolution.height}, "
                f"consecutive={self._consecutive_invalid} "
                f"total_dropped={self._invalid_log_count})"
            )

        if self._consecutive_invalid >= _MAX_CONSECUTIVE_INVALID_FRAMES:
            self._auto_stop_session()

    def _auto_stop_session(self) -> None:
        """불일치가 누적되어 세션을 자동 종료한다.

        현재 세션이 이미 종료된 경우(경합)에는 no-op 이다. 정상 종료와 동일하게
        `recorder.stop()` 을 호출하며, 예외는 로그만 남기고 삼킨다 (콜백
        컨텍스트에서 예외가 전파되면 executor 가 중단됨).
        """
        if self._recorder is None or self._recorder.state != "RECORDING":
            return

        previous_path = self._current_mp4_path
        self.get_logger().error(
            "auto-stopping session: "
            f"{_MAX_CONSECUTIVE_INVALID_FRAMES} consecutive invalid frames "
            f"(path={previous_path})"
        )

        try:
            returned_path = self._recorder.stop(timeout=_STOP_TIMEOUT_SEC)
        except Exception as exc:
            self.get_logger().error(f"auto stop failed: {exc}")
            returned_path = previous_path or ""

        self.get_logger().info(
            f"auto-stopped: path={returned_path} "
            f"state={self._recorder.state} "
            f"frames_written={self._recorder.frames_written}"
        )

        # 세션 상태 변수 리셋 — 사용자는 다음 start_session 으로 재시작 가능
        self._current_mp4_path = None
        self._consecutive_invalid = 0
        self._invalid_log_count = 0

    # ---------- Recorder construction ---------------------------------------

    def _create_recorder(self) -> FFMpegMp4Recorder:
        """`_effective_resolution` 을 바탕으로 `FFMpegMp4Recorder` 인스턴스를 생성한다.

        호출자는 `_effective_resolution` 이 이미 확정되어 있음을 보장해야 한다
        (resolution 파라미터가 지정되었거나 첫 이미지로 추론이 끝난 시점).

        Raises:
            RuntimeError: 생성자에서 `ValueError` 또는 `EncoderUnavailableError`
                가 발생한 경우. resolution 이 사전에 지정된 경로에서는
                `main()` 이 FATAL 로 보고하고 종료한다.
        """
        if self._effective_resolution is None:
            raise RuntimeError("internal error: _create_recorder called without effective resolution")

        try:
            # Recorder 는 stdlib logging.Logger 를 사용한다 (ROS2 비의존).
            # 노드 로거(rclpy.RcutilsLogger) 와는 호환되지 않으므로 기본값
            # (None → recorder 클래스 자체 stdlib 로거) 을 사용한다.
            # bitrate / gop_size / preset / preferred_hw_codec /
            # ffmpeg_binary / vaapi_device 는 recorder 의 기본값을 그대로
            # 사용한다.
            return FFMpegMp4Recorder(
                fps=self._fps,
                resolution=self._effective_resolution,
                pixel_format=self._pixel_format,
                encoder_mode=self._encoder_mode,
                queue_size=self._queue_size,
            )
        except EncoderUnavailableError as exc:
            raise RuntimeError(f"GPU encoder unavailable: {exc}") from exc
        except ValueError as exc:
            raise RuntimeError(f"invalid recorder configuration: {exc}") from exc

    # ---------- Lifecycle ---------------------------------------------------

    def destroy_node(self) -> None:
        """노드 종료 시 recorder 를 안전하게 정리한다.

        SIGINT / SIGTERM / `rclpy.shutdown()` 경로에서도 호출되도록 `main()`
        의 `finally` 블록에서 보장된다. 녹화 중이면 `stop()` 으로 finalize 한
        뒤 `shutdown()` 을 호출한다 (idempotent).
        """
        recorder = getattr(self, "_recorder", None)
        if recorder is not None:
            if recorder.state == "RECORDING":
                self.get_logger().info(
                    "destroy_node: stopping active session before shutdown"
                )
                try:
                    recorder.stop(timeout=_STOP_TIMEOUT_SEC)
                except Exception as exc:
                    self.get_logger().error(
                        f"destroy_node: recorder.stop failed: {exc}"
                    )
            try:
                recorder.shutdown()
            except Exception as exc:
                self.get_logger().error(
                    f"destroy_node: recorder.shutdown failed: {exc}"
                )

        super().destroy_node()

    # ---------- Output directory --------------------------------------------

    def _prepare_output_dir(self) -> None:
        """`output_dir` 이 존재하지 않으면 생성한다.

        Raises:
            RuntimeError: 디렉터리 생성에 실패한 경우 (권한 부족 등).
        """
        try:
            os.makedirs(self._output_dir, exist_ok=True)
        except OSError as exc:
            # main() 에서 FATAL 로그와 함께 종료 처리를 수행한다.
            raise RuntimeError(f"failed to create output_dir {self._output_dir!r}: {exc}") from exc


def main(args: Optional[list[str]] = None) -> None:
    """콘솔 엔트리 포인트.

    `SingleThreadedExecutor` 로 노드를 spin 한다. SIGINT / Ctrl+C /
    `ExternalShutdownException` 경로에서도 `destroy_node()` 와
    `rclpy.try_shutdown()` 이 반드시 호출되도록 finally 블록으로 보호한다.
    """
    rclpy.init(args=args)

    # Python logging(logger="rdfp.*") 출력을 ROS2 logger로 브리지한다.
    from ..logging_bridge import configure_logging_bridge
    configure_logging_bridge(package_logger_name='rdfp')

    node: Optional[ImageRecorderNode] = None
    try:
        node = ImageRecorderNode()
    except Exception as exc:
        # 노드 생성 실패. 노드 로거가 없을 수 있으므로 stderr 에 직접 기록.
        print(f"[FATAL] ImageRecorderNode init failed: {exc}", file=sys.stderr)
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


if __name__ == "__main__":
    main()
