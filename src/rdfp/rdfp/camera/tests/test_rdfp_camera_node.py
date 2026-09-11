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

from robot_control.types import Resolution

from rdfp.camera import rdfp_camera_node as node_module
from rdfp.camera.rdfp_camera_node import RdfpCameraNode

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


# ---------- Tests: 끊김 감지 / 재연결 ----------------------------------------


class TestReconnect:

    def test_initial_status_is_latched(self, node: RdfpCameraNode) -> None:
        """초기화 시 DISCONNECTED 를 한 번 실어 둔다(TRANSIENT_LOCAL latch)."""
        assert node._last_status == "DISCONNECTED"

    def test_status_not_republished_when_unchanged(self, node: RdfpCameraNode) -> None:
        """같은 상태를 반복 발행하지 않는다."""
        node._status_pub = MagicMock(name="status_pub")

        node._publish_status("CONNECTED")
        node._publish_status("CONNECTED")
        node._publish_status("CONNECTED")

        node._status_pub.publish.assert_called_once()

    def test_disconnect_pauses_timer_and_schedules_retry(self, node: RdfpCameraNode) -> None:
        """캡처 중 끊기면 타이머를 정지시키고 재연결 타이머를 건다."""
        node._on_session(_make_session_msg("IN_SESSION"))
        node._on_session(_make_session_msg("IN_EPISODE"))
        capture_timer = node._timer

        node._camera.read.return_value = None
        node._timer_callback()

        # 타이머는 정지만 하고 파기하지 않는다 (자기 콜백 안에서의 파기 금지)
        assert node._timer is capture_timer
        assert node._timer.is_canceled()
        assert node._retry_timer is not None
        assert not node._retry_timer.is_canceled()
        assert node._last_status == "DISCONNECTED"
        node._camera.release.assert_called()
        # 세션은 그대로 IN_EPISODE 이므로 발행 의사는 유지된다
        assert node._publishing is True

    def test_reconnect_does_not_duplicate_capture_timer(self, node: RdfpCameraNode) -> None:
        """재연결에 성공해도 캡처 타이머가 중복 생성되지 않는다."""
        node._on_session(_make_session_msg("IN_SESSION"))
        node._on_session(_make_session_msg("IN_EPISODE"))

        node._camera.read.return_value = None
        node._timer_callback()          # 끊김 감지 → 재연결 타이머 생성
        node._camera.read.return_value = np.zeros((480, 640, 3), dtype=np.uint8)

        node._retry_callback()          # 재연결 성공

        # 캡처 타이머 1개 + 재연결 타이머 1개 = 2개.
        # **개수로 세는 것이 요점이다** — `_timer` 만 보면 등록된 채 방치된 옛
        # 타이머를 놓친다. 시계 검사 타이머는 이 불변식과 무관하므로 뺀다.
        capture_and_retry = [t for t in node.timers if t is not node._sim_time_check_timer]
        assert len(capture_and_retry) == 2
        assert not node._timer.is_canceled()
        assert node._retry_timer.is_canceled()
        assert node._last_status == "CONNECTED"

    def test_retry_stops_when_session_left(self, node: RdfpCameraNode) -> None:
        """세션이 IDLE 로 돌아가면 재연결 시도를 멈춘다."""
        node._on_session(_make_session_msg("IN_SESSION"))
        node._camera.read.return_value = None
        node._timer_callback()
        assert node._retry_timer is not None

        node._on_session(_make_session_msg("IDLE"))
        assert node._retry_timer.is_canceled()

        open_calls = node._camera.open.call_count
        node._retry_callback()
        assert node._camera.open.call_count == open_calls

    def test_open_failure_schedules_retry(self, node: RdfpCameraNode) -> None:
        """open 실패 시에도 재연결 타이머가 걸린다."""
        node._camera.open.side_effect = RuntimeError("device not found")
        node._on_session(_make_session_msg("IN_SESSION"))

        assert node._open_failed is True
        assert node._retry_timer is not None
        assert not node._retry_timer.is_canceled()
        assert node._last_status == "ERROR"

    def test_reconnect_disabled_by_zero_interval(self, mock_camera_class: MagicMock) -> None:
        """reconnect_interval_sec=0 이면 재연결 타이머를 만들지 않는다."""
        n = RdfpCameraNode(
            parameter_overrides=_default_overrides(reconnect_interval_sec=0.0),
        )
        try:
            n._on_session(_make_session_msg("IN_SESSION"))
            n._camera.read.return_value = None
            n._timer_callback()

            assert n._retry_timer is None
        finally:
            n.destroy_node()


class TestEpisodeBoundary:
    """에피소드 경계 — 시작 공백·꼬리 감사·시계 불일치."""

    def test_in_episode_publishes_immediately(self, node: RdfpCameraNode) -> None:
        """**전이 즉시 한 장 나가야 한다.**

        타이머는 자유 주행이라 전이가 주기를 앞당기지 않는다. 그대로 두면 에피소드
        시작 직후 최대 한 주기(10 Hz면 100 ms) 동안 이미지가 없다 — 실측 86~92 ms.
        """
        node._camera.read.return_value = np.zeros((480, 640, 3), dtype=np.uint8)
        node._on_session(_make_session_msg("IN_SESSION"))
        # 공용 대역은 open() 뒤에도 is_opened 가 False 로 남는다 — 실제 카메라는
        # 열리면 True 다. 즉시 캡처는 그 상태를 전제하므로 여기서 맞춰 준다.
        node._camera.is_opened = True
        node._image_pub.publish.reset_mock()

        node._on_session(_make_session_msg("IN_EPISODE"))

        assert node._image_pub.publish.call_count == 1

    def test_immediate_capture_is_skipped_when_camera_is_not_open(
            self, node: RdfpCameraNode) -> None:
        """카메라가 닫혀 있으면 즉시 캡처를 하지 않는다 — open 경로가 먼저다."""
        node._camera.is_opened = False
        node._on_session(_make_session_msg("IN_EPISODE"))
        assert node._image_pub.publish.call_count == 0

    def test_tail_overshoot_is_logged(self, node: RdfpCameraNode) -> None:
        """정지 stamp 보다 늦은 이미지를 냈으면 **조용히 넘어가지 않는다.**

        막을 수는 없다 — 발행 시점에는 정지 stamp 를 모른다. 그래서 알리기라도 한다.
        """
        node._on_session(_make_session_msg("IN_SESSION"))
        node._on_session(_make_session_msg("IN_EPISODE"))
        node._last_published_ns = 5_000_000_000

        stop = _make_session_msg("IN_SESSION")
        stop.header.stamp.sec = 4
        stop.header.stamp.nanosec = 0
        with patch.object(node, 'get_logger') as logger:
            node._on_session(stop)

        assert logger.return_value.warning.called
        assert 'episode tail' in logger.return_value.warning.call_args[0][0]

    def test_no_warning_when_the_tail_is_clean(self, node: RdfpCameraNode) -> None:
        """정지 stamp 이전에 멈췄으면 경고하지 않는다 — 잡음이 되면 아무도 안 본다."""
        node._on_session(_make_session_msg("IN_SESSION"))
        node._on_session(_make_session_msg("IN_EPISODE"))
        node._last_published_ns = 3_000_000_000

        stop = _make_session_msg("IN_SESSION")
        stop.header.stamp.sec = 4
        stop.header.stamp.nanosec = 0
        with patch.object(node, 'get_logger') as logger:
            node._on_session(stop)

        assert not logger.return_value.warning.called

    def test_clock_mismatch_is_reported(self, node: RdfpCameraNode) -> None:
        """`/clock` 이 도는데 `use_sim_time` 이 꺼져 있으면 **에러로** 알린다.

        증상이 "이미지가 DB 에 하나도 없다"뿐이라 원인이 안 보이기 때문이다.
        """
        with patch.object(node, 'count_publishers', return_value=1), \
             patch.object(node, 'get_logger') as logger:
            node._check_clock_source()

        assert logger.return_value.error.called
        assert 'use_sim_time' in logger.return_value.error.call_args[0][0]
        assert node._sim_time_check_timer is None, '검사는 한 번만 돈다'

    def test_no_clock_mismatch_report_without_clock(self, node: RdfpCameraNode) -> None:
        """`/clock` 이 없으면 정상이다 — 실제 카메라 스택이 그렇다."""
        with patch.object(node, 'count_publishers', return_value=0), \
             patch.object(node, 'get_logger') as logger:
            node._check_clock_source()

        assert not logger.return_value.error.called


class TestSessionTopicIsGlobal:
    """**세션은 시스템에 하나다** — 로봇별 네임스페이스를 타면 안 된다 (규약 §2.5).

    소비자 쪽에서만 실제로 검증할 수 있다. 세션 노드는 싱글턴이라 namespace 인자를
    받지 않지만, 카메라·레코더는 로봇별 네임스페이스 안에 들어갈 수 있기 때문이다.
    """

    def test_subscription_ignores_the_node_namespace(self, mock_camera_class) -> None:
        """`/abc` 안에 있어도 `/session` 을 구독한다.

        상대(`session`)로 되돌아가면 `/abc/session` 을 찾아 **조용히 아무것도 못
        받는다** — 증상은 "카메라가 안 열린다" 하나뿐이다.
        """
        node = RdfpCameraNode(namespace='/abc', parameter_overrides=_default_overrides())
        try:
            assert node._session_sub.topic_name == '/session'
        finally:
            node.destroy_node()

    def test_image_topics_still_follow_the_namespace(self, mock_camera_class) -> None:
        """**세션만 예외다.** 이미지·상태는 로봇별로 갈려야 한다 — 전부 절대로
        바꿔 버리는 실수를 잡는다.
        """
        node = RdfpCameraNode(namespace='/abc', parameter_overrides=_default_overrides())
        try:
            assert node._status_pub.topic_name.startswith('/abc/')
        finally:
            node.destroy_node()
