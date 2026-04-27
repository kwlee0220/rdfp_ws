#!/usr/bin/env python3

"""ImageRecorderNode 단위 테스트.

Recorder 엔진(`FFMpegMp4Recorder`) 은 `unittest.mock` 으로 패치하여 노드의
ROS2 인터페이스 / 콜백 로직만 검증한다. 실제 ffmpeg 실행, 파일 생성은
Phase 10 통합 테스트에서 다룬다.
"""

from __future__ import annotations

from typing import Any, Iterator

import re
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from rdfp.recorder import image_recorder_node as node_module
from rdfp.recorder.exceptions import (
    EncoderUnavailableError,
    RecorderStateError,
)
from rdfp.recorder.image_recorder_node import (
    _MAX_CONSECUTIVE_INVALID_FRAMES,
    _STOP_TIMEOUT_SEC,
    ImageRecorderNode,
)
from rdfp.types import InvalidFrameError

try:
    from rdfp_msgs.srv import StartSession, StopSession
except ImportError:  # pragma: no cover
    StartSession = None  # type: ignore
    StopSession = None  # type: ignore


# ---------- Fixtures ---------------------------------------------------------


@pytest.fixture(scope="module", autouse=True)
def _rclpy_session() -> Iterator[None]:
    """rclpy 를 모듈 단위로 초기화/정리한다."""
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _make_mock_recorder(state: str = "IDLE") -> MagicMock:
    """recorder 인스턴스를 흉내내는 MagicMock 을 생성한다."""
    mock = MagicMock(name="FFMpegMp4Recorder")
    mock.state = state
    mock.selected_codec = "libx264"
    mock.frames_written = 0
    mock.frames_dropped = 0
    return mock


def _default_overrides(
    tmp_path: Any,
    **updates: Any,
) -> list[Parameter]:
    """테스트용 파라미터 override 목록."""
    params: dict[str, Any] = {
        "output_dir": str(tmp_path),
        "session_prefix": "test",
        "fps": 30,
        "resolution": "640x480",
        "pixel_format": "bgr8",
        "encoder_mode": "auto",
        "queue_size": 120,
    }
    params.update(updates)
    overrides: list[Parameter] = []
    for name, value in params.items():
        overrides.append(Parameter(name, value=value))
    return overrides


@pytest.fixture
def mock_recorder_class() -> Iterator[MagicMock]:
    """`FFMpegMp4Recorder` 클래스를 MagicMock 으로 패치한다."""
    with patch.object(node_module, "FFMpegMp4Recorder") as cls_mock:
        cls_mock.return_value = _make_mock_recorder()
        yield cls_mock


@pytest.fixture
def node(
    tmp_path: Any,
    mock_recorder_class: MagicMock,
) -> Iterator[ImageRecorderNode]:
    """기본 파라미터로 ImageRecorderNode 를 생성·정리한다."""
    n = ImageRecorderNode(parameter_overrides=_default_overrides(tmp_path))
    try:
        yield n
    finally:
        n.destroy_node()


# ---------- Helpers ----------------------------------------------------------


def _make_image_msg(
    width: int = 640,
    height: int = 480,
    encoding: str = "bgr8",
    step: int | None = None,
    stamp_sec: int = 1,
    stamp_nanosec: int = 500_000_000,
) -> Image:
    """테스트용 Image 메시지를 생성한다."""
    msg = Image()
    msg.header = Header(
        stamp=Time(sec=stamp_sec, nanosec=stamp_nanosec),
        frame_id="test",
    )
    msg.width = width
    msg.height = height
    msg.encoding = encoding
    channels = 1 if encoding == "mono8" else 3
    msg.step = step if step is not None else width * channels
    msg.data = bytes(msg.step * height)  # 전부 0 으로 채움
    return msg


# ---------- Tests ------------------------------------------------------------


class TestInit:

    def test_declares_parameters(
        self, node: ImageRecorderNode, tmp_path: Any
    ) -> None:
        """노드 init 시 모든 파라미터가 선언되고 값이 로드되어야 한다."""
        assert node._output_dir == str(tmp_path)
        assert node._session_prefix == "test"
        assert int(node._fps) == 30
        assert node._resolution.width == 640
        assert node._resolution.height == 480
        assert node._pixel_format == "bgr8"
        assert node._encoder_mode == "auto"
        assert node._queue_size == 120

    def test_creates_output_dir(
        self,
        tmp_path: Any,
        mock_recorder_class: MagicMock,
    ) -> None:
        """output_dir 가 존재하지 않으면 생성되어야 한다."""
        target = tmp_path / "nested" / "dir"
        assert not target.exists()
        n = ImageRecorderNode(
            parameter_overrides=_default_overrides(target)
        )
        try:
            assert target.is_dir()
        finally:
            n.destroy_node()

    def test_constructs_recorder_with_params(
        self,
        node: ImageRecorderNode,
        mock_recorder_class: MagicMock,
    ) -> None:
        """recorder 생성자에 올바른 인자들이 전달되어야 한다."""
        mock_recorder_class.assert_called_once()
        kwargs = mock_recorder_class.call_args.kwargs
        assert kwargs["fps"] == 30
        assert kwargs["pixel_format"] == "bgr8"
        assert kwargs["encoder_mode"] == "auto"
        assert kwargs["queue_size"] == 120
        # bitrate / gop_size / preset 등은 노드가 전달하지 않으므로
        # recorder 의 기본값이 사용된다.
        assert "bitrate" not in kwargs
        assert "gop_size" not in kwargs
        assert "preferred_hw_codec" not in kwargs

    def test_fails_on_encoder_unavailable(
        self,
        tmp_path: Any,
    ) -> None:
        """recorder 생성자가 EncoderUnavailableError 를 던지면 RuntimeError 로
        변환되어야 한다."""
        with patch.object(node_module, "FFMpegMp4Recorder") as cls_mock:
            cls_mock.side_effect = EncoderUnavailableError("no hw encoder")
            with pytest.raises(RuntimeError, match="GPU encoder unavailable"):
                ImageRecorderNode(
                    parameter_overrides=_default_overrides(
                        tmp_path, encoder_mode="gpu"
                    )
                )


@pytest.mark.skipif(StartSession is None, reason="rdfp_msgs not built")
class TestStartSession:

    def test_generates_correct_filename(
        self,
        node: ImageRecorderNode,
        tmp_path: Any,
    ) -> None:
        """파일명이 `<prefix>_YYYYMMDD-HHMMSS.SSS.mp4` 형식이어야 한다."""
        response = StartSession.Response()
        request = StartSession.Request()
        node._recorder.state = "IDLE"

        node._handle_start_session(request, response)

        assert response.success is True
        pattern = re.compile(
            r"^test_\d{8}-\d{6}\.\d{3}\.mp4$"
        )
        basename = response.mp4_path.rsplit("/", 1)[-1]
        assert pattern.match(basename), f"unexpected filename: {basename}"
        # recorder.start 에 생성된 경로가 전달되었는지
        node._recorder.start.assert_called_once_with(response.mp4_path)

    def test_rejects_when_already_recording(
        self,
        node: ImageRecorderNode,
    ) -> None:
        """이미 RECORDING 상태이면 실패 응답을 반환해야 한다."""
        node._recorder.state = "RECORDING"
        response = StartSession.Response()
        node._handle_start_session(StartSession.Request(), response)

        assert response.success is False
        assert response.mp4_path == ""
        node._recorder.start.assert_not_called()

    def test_propagates_file_exists_error(
        self,
        node: ImageRecorderNode,
    ) -> None:
        """recorder.start 의 FileExistsError 는 실패 응답으로 돌아야 한다."""
        node._recorder.state = "IDLE"
        node._recorder.start.side_effect = FileExistsError("exists")
        response = StartSession.Response()
        node._handle_start_session(StartSession.Request(), response)
        assert response.success is False
        assert response.mp4_path == ""


@pytest.mark.skipif(StopSession is None, reason="rdfp_msgs not built")
class TestStopSession:

    def test_calls_stop_with_5s_timeout(
        self,
        node: ImageRecorderNode,
        tmp_path: Any,
    ) -> None:
        """recorder.stop 이 5.0 초 timeout 으로 호출되어야 한다."""
        node._recorder.state = "RECORDING"
        expected_path = str(tmp_path / "test_20260101-000000.000.mp4")
        node._recorder.stop.return_value = expected_path
        # stop() 호출 이후 state 가 IDLE 로 바뀌는 것을 흉내낸다.

        def _stop_side_effect(timeout: float) -> str:
            node._recorder.state = "IDLE"
            return expected_path

        node._recorder.stop.side_effect = _stop_side_effect

        response = StopSession.Response()
        node._handle_stop_session(StopSession.Request(), response)

        node._recorder.stop.assert_called_once_with(timeout=_STOP_TIMEOUT_SEC)
        assert response.success is True
        assert response.mp4_path == expected_path

    def test_rejects_when_not_recording(
        self,
        node: ImageRecorderNode,
    ) -> None:
        """RECORDING 상태가 아니면 실패 응답을 반환해야 한다."""
        node._recorder.state = "IDLE"
        response = StopSession.Response()
        node._handle_stop_session(StopSession.Request(), response)
        assert response.success is False
        assert response.mp4_path == ""
        node._recorder.stop.assert_not_called()

    def test_reports_failed_state(
        self,
        node: ImageRecorderNode,
        tmp_path: Any,
    ) -> None:
        """stop() 반환 후 recorder.state 가 FAILED 이면 실패 응답이어야 한다."""
        node._recorder.state = "RECORDING"
        returned_path = str(tmp_path / "test.mp4")

        def _stop_side_effect(timeout: float) -> str:
            node._recorder.state = "FAILED"
            return returned_path

        node._recorder.stop.side_effect = _stop_side_effect
        response = StopSession.Response()
        node._handle_stop_session(StopSession.Request(), response)
        assert response.success is False
        assert response.mp4_path == ""


class TestImageCallback:

    def test_drops_when_not_recording(
        self,
        node: ImageRecorderNode,
    ) -> None:
        """녹화 중이 아니면 프레임을 recorder 에 전달하지 않는다."""
        node._recorder.state = "IDLE"
        node._on_image(_make_image_msg())
        node._recorder.write.assert_not_called()
        assert node._consecutive_invalid == 0

    def test_valid_frame_writes_and_resets_counter(
        self,
        node: ImageRecorderNode,
    ) -> None:
        """정상 프레임은 recorder.write 가 호출되고 카운터가 0 으로 리셋된다."""
        node._recorder.state = "RECORDING"
        node._consecutive_invalid = 3  # 인위적으로 설정
        node._on_image(_make_image_msg())
        node._recorder.write.assert_called_once()
        assert node._consecutive_invalid == 0

    def test_invalid_encoding_increments_counter(
        self,
        node: ImageRecorderNode,
    ) -> None:
        """encoding 불일치 프레임은 drop 되고 카운터가 증가한다."""
        node._recorder.state = "RECORDING"
        msg = _make_image_msg(encoding="rgb8")
        node._on_image(msg)
        assert node._consecutive_invalid == 1
        node._recorder.write.assert_not_called()

    def test_invalid_resolution_same_policy(
        self,
        node: ImageRecorderNode,
    ) -> None:
        """해상도 불일치도 encoding 불일치와 동일한 정책으로 drop 된다."""
        node._recorder.state = "RECORDING"
        msg = _make_image_msg(width=800, height=600)
        node._on_image(msg)
        assert node._consecutive_invalid == 1
        node._recorder.write.assert_not_called()

    def test_recorder_write_errors_do_not_raise(
        self,
        node: ImageRecorderNode,
    ) -> None:
        """`write()` 의 InvalidFrameError / RecorderStateError 는 swallow 된다."""
        node._recorder.state = "RECORDING"
        node._recorder.write.side_effect = InvalidFrameError("bad")
        node._on_image(_make_image_msg())  # 예외가 전파되지 않아야 함

        node._recorder.write.side_effect = RecorderStateError("race")
        node._on_image(_make_image_msg())  # 동일


class TestInvalidFrameHandling:

    def test_log_suppression_first_and_every_100th(
        self,
        node: ImageRecorderNode,
    ) -> None:
        """첫 1회 + 100건마다 1회 로그가 찍힌다."""
        # 카운터만 검증 (로그 스파이 대신) — 자동 종료 임계치를 회피하기 위해
        # 콜백 대신 `_handle_invalid_frame` 내부 카운터만 증가시키고 자동
        # 종료를 disable.
        # 자동 종료를 막기 위해 _auto_stop_session 을 mock 한다.
        node._recorder.state = "RECORDING"
        with patch.object(node, "_auto_stop_session"):
            logger = MagicMock()
            with patch.object(node, "get_logger", return_value=logger):
                # 250 회 drop
                msg = _make_image_msg(encoding="rgb8")
                for _ in range(250):
                    node._handle_invalid_frame(msg)

        # 로그 호출 횟수 = 1 (최초) + 2 (100, 200) = 3
        assert logger.error.call_count == 3
        assert node._invalid_log_count == 250

    def test_auto_stop_after_5_consecutive_invalid(
        self,
        node: ImageRecorderNode,
    ) -> None:
        """연속 5 프레임 불일치 시 자동 종료가 트리거된다."""
        node._recorder.state = "RECORDING"

        def _stop_side_effect(timeout: float) -> str:
            node._recorder.state = "IDLE"
            return "/tmp/test.mp4"

        node._recorder.stop.side_effect = _stop_side_effect
        node._current_mp4_path = "/tmp/test.mp4"

        for _ in range(_MAX_CONSECUTIVE_INVALID_FRAMES):
            node._on_image(_make_image_msg(encoding="rgb8"))

        # recorder.stop 호출 확인
        node._recorder.stop.assert_called_once_with(
            timeout=_STOP_TIMEOUT_SEC
        )
        # 세션 상태 리셋
        assert node._consecutive_invalid == 0
        assert node._invalid_log_count == 0
        assert node._current_mp4_path is None


class TestDestroyNode:

    def test_calls_stop_then_shutdown(
        self,
        tmp_path: Any,
        mock_recorder_class: MagicMock,
    ) -> None:
        """RECORDING 중에 destroy_node 가 호출되면 stop → shutdown 순서여야 한다."""
        n = ImageRecorderNode(
            parameter_overrides=_default_overrides(tmp_path)
        )
        n._recorder.state = "RECORDING"
        n.destroy_node()

        recorder = n._recorder
        recorder.stop.assert_called_once_with(timeout=_STOP_TIMEOUT_SEC)
        recorder.shutdown.assert_called_once()

    def test_idle_skips_stop_but_calls_shutdown(
        self,
        tmp_path: Any,
        mock_recorder_class: MagicMock,
    ) -> None:
        """IDLE 상태에서는 stop 을 호출하지 않고 shutdown 만 호출한다."""
        n = ImageRecorderNode(
            parameter_overrides=_default_overrides(tmp_path)
        )
        n._recorder.state = "IDLE"
        n.destroy_node()

        recorder = n._recorder
        recorder.stop.assert_not_called()
        recorder.shutdown.assert_called_once()


class TestFrameConversion:

    def test_bgr8_frame_conversion_shape(
        self,
        node: ImageRecorderNode,
    ) -> None:
        """bgr8 메시지는 (H, W, 3) uint8 ndarray 로 변환된다."""
        msg = _make_image_msg(width=640, height=480, encoding="bgr8")
        frame = node._msg_to_ndarray(msg)
        assert frame.shape == (480, 640, 3)
        assert frame.dtype == np.uint8

    def test_mono8_frame_conversion_shape(
        self,
        tmp_path: Any,
        mock_recorder_class: MagicMock,
    ) -> None:
        """mono8 메시지는 (H, W) uint8 ndarray 로 변환된다."""
        n = ImageRecorderNode(
            parameter_overrides=_default_overrides(
                tmp_path, pixel_format="mono8"
            )
        )
        try:
            msg = _make_image_msg(
                width=320, height=240, encoding="mono8"
            )
            frame = n._msg_to_ndarray(msg)
            assert frame.shape == (240, 320)
            assert frame.dtype == np.uint8
        finally:
            n.destroy_node()

    def test_frame_conversion_handles_stride_padding(
        self,
        node: ImageRecorderNode,
    ) -> None:
        """msg.step 이 width*channels 보다 크면 padding 을 제거해야 한다."""
        # width=640, channels=3 → 유효 행 크기 1920. step=2000 (padding 80).
        msg = _make_image_msg(
            width=640, height=480, encoding="bgr8", step=2000
        )
        # 각 행 앞 1920 바이트는 1, 뒤 80 바이트(padding)는 99 로 세팅
        buffer = np.zeros((480, 2000), dtype=np.uint8)
        buffer[:, :1920] = 1
        buffer[:, 1920:] = 99
        msg.data = buffer.tobytes()

        frame = node._msg_to_ndarray(msg)
        assert frame.shape == (480, 640, 3)
        # padding 이 제거되어 모든 값이 1 이어야 함
        assert np.all(frame == 1)
