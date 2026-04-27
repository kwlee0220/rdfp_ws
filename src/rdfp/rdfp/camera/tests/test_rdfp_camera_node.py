#!/usr/bin/env python3

"""RdfpCameraNode 단위 테스트.

OpenCvCamera 는 unittest.mock 으로 패치하여 노드의 세션 상태 전이 로직,
타이머 콜백, 리소스 정리만 검증한다.
"""

from __future__ import annotations

from typing import Iterator

from unittest.mock import MagicMock, patch

import numpy as np
import pytest

import rclpy
from rclpy.parameter import Parameter

from rdfp.camera.junk import rdfp_camera_node as node_module
from rdfp.camera.junk.rdfp_camera_node import RdfpCameraNode
from rdfp.types import Resolution

from rdfp_msgs.msg import SessionCommand  # type: ignore[import-not-found]


# ---------- Fixtures ---------------------------------------------------------


@pytest.fixture(scope="module", autouse=True)
def _rclpy_session() -> Iterator[None]:
    """rclpy 를 모듈 단위로 초기화/정리한다."""
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _default_overrides(**updates) -> list[Parameter]:
    """테스트용 파라미터 override 목록."""
    params: dict = {
        "camera_id": "0",
        "encoding": "bgr8",
        "frame_id": "test_frame",
    }
    params.update(updates)
    return [Parameter(name, value=value) for name, value in params.items()]


def _make_mock_camera() -> MagicMock:
    """OpenCvCamera 를 흉내내는 MagicMock 을 생성한다."""
    mock = MagicMock(name="OpenCvCamera")
    mock.is_opened = False
    mock.open.return_value = (Resolution(640, 480), 30.0)
    mock.read.return_value = np.zeros((480, 640, 3), dtype=np.uint8)
    mock.release.return_value = None
    return mock


@pytest.fixture
def mock_camera_class() -> Iterator[MagicMock]:
    """OpenCvCamera 클래스를 MagicMock 으로 패치한다."""
    with patch.object(node_module, "OpenCvCamera") as cls_mock:
        cls_mock.return_value = _make_mock_camera()
        yield cls_mock


@pytest.fixture
def node(mock_camera_class: MagicMock) -> Iterator[RdfpCameraNode]:
    """기본 파라미터로 RdfpCameraNode 를 생성·정리한다."""
    n = RdfpCameraNode(parameter_overrides=_default_overrides())
    # 퍼블리셔를 MagicMock 으로 교체하여 publish 호출을 검증할 수 있게 한다.
    n._image_pub = MagicMock(name="image_pub")
    try:
        yield n
    finally:
        n.destroy_node()


def _make_session_msg(state: str) -> SessionCommand:
    """테스트용 SessionCommand 메시지를 생성한다."""
    msg = SessionCommand()
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = 0
    msg.state = state
    msg.task_label = ""
    return msg


# ---------- Tests: 초기화 ---------------------------------------------------


class TestInit:

    def test_parameters_loaded(self, node: RdfpCameraNode) -> None:
        """파라미터가 올바르게 로드되어야 한다."""
        assert node._camera_id == 0
        assert node._encoding == "bgr8"
        assert node._frame_id == "test_frame"

    def test_initial_state(self, node: RdfpCameraNode) -> None:
        """초기 상태가 올바르게 설정되어야 한다."""
        assert node._publishing is False
        assert node._open_failed is False
        assert node._timer is None
        assert node._prev_state == "IDLE"

    def test_camera_not_opened_on_init(
        self, node: RdfpCameraNode, mock_camera_class: MagicMock,
    ) -> None:
        """초기화 시 카메라 open 이 호출되지 않아야 한다."""
        node._camera.open.assert_not_called()


# ---------- Tests: 상태 전이 -------------------------------------------------


class TestStateTransitions:

    def test_idle_to_in_session(self, node: RdfpCameraNode) -> None:
        """IDLE → IN_SESSION: 카메라 open + 타이머 생성."""
        node._on_session(_make_session_msg("IN_SESSION"))

        node._camera.open.assert_called_once()
        assert node._timer is not None
        assert node._publishing is False
        assert node._prev_state == "IN_SESSION"

    def test_in_session_to_in_episode(self, node: RdfpCameraNode) -> None:
        """IN_SESSION → IN_EPISODE: _publishing 활성화."""
        node._on_session(_make_session_msg("IN_SESSION"))
        node._on_session(_make_session_msg("IN_EPISODE"))

        assert node._publishing is True
        assert node._prev_state == "IN_EPISODE"

    def test_in_episode_to_in_session(self, node: RdfpCameraNode) -> None:
        """IN_EPISODE → IN_SESSION: _publishing 비활성화, 타이머 유지."""
        node._on_session(_make_session_msg("IN_SESSION"))
        node._on_session(_make_session_msg("IN_EPISODE"))
        node._on_session(_make_session_msg("IN_SESSION"))

        assert node._publishing is False
        assert node._timer is not None
        assert node._prev_state == "IN_SESSION"

    def test_in_session_to_idle(self, node: RdfpCameraNode) -> None:
        """IN_SESSION → IDLE: 타이머 취소 + 카메라 release."""
        node._on_session(_make_session_msg("IN_SESSION"))
        node._on_session(_make_session_msg("IDLE"))

        assert node._timer is None
        assert node._publishing is False
        node._camera.release.assert_called()
        assert node._prev_state == "IDLE"

    def test_idle_repeated(self, node: RdfpCameraNode) -> None:
        """IDLE → IDLE: 카메라가 이미 닫힌 상태에서도 오류 없이 처리."""
        node._on_session(_make_session_msg("IDLE"))
        node._on_session(_make_session_msg("IDLE"))

        assert node._prev_state == "IDLE"
        assert node._publishing is False

    def test_full_cycle(self, node: RdfpCameraNode) -> None:
        """IDLE → IN_SESSION → IN_EPISODE → IN_SESSION → IDLE 전체 사이클."""
        node._on_session(_make_session_msg("IN_SESSION"))
        assert node._timer is not None

        node._on_session(_make_session_msg("IN_EPISODE"))
        assert node._publishing is True

        node._on_session(_make_session_msg("IN_SESSION"))
        assert node._publishing is False
        assert node._timer is not None

        node._on_session(_make_session_msg("IDLE"))
        assert node._timer is None
        node._camera.release.assert_called()

    def test_unknown_state_ignored(self, node: RdfpCameraNode) -> None:
        """알 수 없는 state 값은 무시하고 _prev_state 를 변경하지 않는다."""
        node._on_session(_make_session_msg("UNKNOWN_STATE"))
        assert node._prev_state == "IDLE"

    def test_late_join_in_session(self, node: RdfpCameraNode) -> None:
        """late-join IN_SESSION: 카메라 open + 타이머 생성."""
        node._on_session(_make_session_msg("IN_SESSION"))

        node._camera.open.assert_called_once()
        assert node._timer is not None
        assert node._publishing is False

    def test_late_join_in_episode(self, node: RdfpCameraNode) -> None:
        """late-join IN_EPISODE: 카메라 open + 타이머 생성 + 발행 시작."""
        node._camera.is_opened = False
        node._on_session(_make_session_msg("IN_EPISODE"))

        node._camera.open.assert_called_once()
        assert node._timer is not None
        assert node._publishing is True
        assert node._prev_state == "IN_EPISODE"

    def test_late_join_in_episode_open_failure(
        self, node: RdfpCameraNode,
    ) -> None:
        """late-join IN_EPISODE에서 open 실패 시 발행하지 않는다."""
        node._camera.is_opened = False
        node._camera.open.side_effect = RuntimeError("device not found")
        node._on_session(_make_session_msg("IN_EPISODE"))

        assert node._open_failed is True
        assert node._publishing is False
        assert node._timer is None


# ---------- Tests: 카메라 open 실패 ------------------------------------------


class TestCameraOpenFailure:

    def test_open_failed_sets_flag(self, node: RdfpCameraNode) -> None:
        """camera.open() 이 실패하면 _open_failed 가 True 로 설정된다."""
        node._camera.open.side_effect = RuntimeError("device not found")
        node._on_session(_make_session_msg("IN_SESSION"))

        assert node._open_failed is True
        assert node._timer is None

    def test_in_episode_ignored_after_open_failure(
        self, node: RdfpCameraNode,
    ) -> None:
        """open 실패 후 IN_EPISODE 가 오면 _publishing 이 False 유지."""
        node._camera.open.side_effect = RuntimeError("device not found")
        node._on_session(_make_session_msg("IN_SESSION"))
        node._on_session(_make_session_msg("IN_EPISODE"))

        assert node._publishing is False
        assert node._open_failed is True

    def test_idle_resets_open_failed(self, node: RdfpCameraNode) -> None:
        """open 실패 후 IDLE 전이 시 _open_failed 가 리셋된다."""
        node._camera.open.side_effect = RuntimeError("device not found")
        node._on_session(_make_session_msg("IN_SESSION"))
        assert node._open_failed is True

        node._on_session(_make_session_msg("IDLE"))
        assert node._open_failed is False


# ---------- Tests: 타이머 콜백 -----------------------------------------------


class TestTimerCallback:

    def test_read_success_not_publishing(self, node: RdfpCameraNode) -> None:
        """_publishing=False 이면 read 는 하되 발행하지 않는다."""
        node._on_session(_make_session_msg("IN_SESSION"))
        assert node._publishing is False

        node._timer_callback()

        node._camera.read.assert_called_once()
        # 발행하지 않았으므로 publish 호출 없음
        node._image_pub.publish.assert_not_called()

    def test_read_success_publishing(self, node: RdfpCameraNode) -> None:
        """_publishing=True 이면 이미지를 발행한다."""
        node._on_session(_make_session_msg("IN_SESSION"))
        node._on_session(_make_session_msg("IN_EPISODE"))
        assert node._publishing is True

        node._timer_callback()

        node._camera.read.assert_called()
        node._image_pub.publish.assert_called_once()

        published_msg = node._image_pub.publish.call_args[0][0]
        assert published_msg.header.frame_id == "test_frame"
        assert published_msg.header.stamp.sec > 0 or published_msg.header.stamp.nanosec > 0

    def test_read_failure_no_publish(self, node: RdfpCameraNode) -> None:
        """read() 가 None 을 반환하면 발행하지 않는다."""
        node._on_session(_make_session_msg("IN_SESSION"))
        node._on_session(_make_session_msg("IN_EPISODE"))

        node._camera.read.return_value = None
        node._timer_callback()

        node._image_pub.publish.assert_not_called()


# ---------- Tests: destroy_node ----------------------------------------------


class TestDestroyNode:

    def test_destroy_releases_camera(
        self, mock_camera_class: MagicMock,
    ) -> None:
        """destroy_node() 시 카메라가 release 된다."""
        n = RdfpCameraNode(parameter_overrides=_default_overrides())

        # 세션 시작 → 타이머 생성
        n._on_session(_make_session_msg("IN_SESSION"))
        assert n._timer is not None

        n.destroy_node()

        n._camera.release.assert_called()

    def test_destroy_without_session(
        self, mock_camera_class: MagicMock,
    ) -> None:
        """세션 시작 없이 destroy_node() 해도 오류 없이 정리된다."""
        n = RdfpCameraNode(parameter_overrides=_default_overrides())
        n.destroy_node()

        n._camera.release.assert_called()
