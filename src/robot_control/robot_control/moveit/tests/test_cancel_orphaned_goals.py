"""기동 시 고아 goal 취소 — 이전 프로세스가 남긴 컨트롤러 궤적을 끊는다.

클라이언트가 죽어도 컨트롤러는 받은 궤적을 끝까지 실행하고, 새 클라이언트는 그 goal 의
핸들이 없어 `cancel()` 로 닿지 않는다. 트윈의 예전 구현은 클라이언트가 만들어지기 전에
불려 아무것도 하지 않았다.
"""

from __future__ import annotations

from typing import Any, Callable, Optional

import pytest

pytest.importorskip('rclpy')                                         # noqa: E402
pytest.importorskip('moveit_msgs')                                   # noqa: E402

from action_msgs.msg import GoalInfo                                 # noqa: E402
from action_msgs.srv import CancelGoal                               # noqa: E402
from robot_control.moveit.move_group_client import MoveGroupClient   # noqa: E402
from robot_control.moveit.move_group_jtc_client import MoveGroupJtcClient  # noqa: E402

_CANCEL_SERVICE = '/panda_arm_controller/follow_joint_trajectory/_action/cancel_goal'


class _Logger:
    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warnings: list[str] = []

    def info(self, msg: str) -> None:
        self.infos.append(msg)

    def warning(self, msg: str) -> None:
        self.warnings.append(msg)


class _Node:
    def __init__(self) -> None:
        self.logger = _Logger()

    def get_logger(self) -> _Logger:
        return self.logger


class _Future:
    def __init__(self, response: Any = None, error: Optional[Exception] = None) -> None:
        self._response = response
        self._error = error

    def add_done_callback(self, callback: Callable[['_Future'], None]) -> None:
        callback(self)

    def result(self) -> Any:
        if self._error is not None:
            raise self._error
        return self._response


class _CancelClient:
    """컨트롤러 `CancelGoal` 서비스 클라이언트 대역."""

    def __init__(self, *, available: bool, response: Any = None,
                 error: Optional[Exception] = None) -> None:
        self._available = available
        self._response = response
        self._error = error
        self.waits: list[float] = []
        self.requests: list[Any] = []

    def wait_for_service(self, timeout_sec: float) -> bool:
        self.waits.append(timeout_sec)
        return self._available

    def service_is_ready(self) -> bool:      # cancel() 이 쓰는 즉시 판정 — 여기서는 매칭 전
        return False

    def call_async(self, request: Any) -> _Future:
        self.requests.append(request)
        return _Future(self._response, self._error)


def _jtc(cancel_client: _CancelClient) -> MoveGroupJtcClient:
    """취소 경로가 읽는 상태만 채운 껍데기. 생성자는 ROS 클라이언트를 만든다."""
    client = object.__new__(MoveGroupJtcClient)
    client._node = _Node()
    client._closed = False
    client._controller_cancel_client = cancel_client
    client._controller_cancel_service = _CANCEL_SERVICE
    return client


def _response(n_goals: int) -> CancelGoal.Response:
    return CancelGoal.Response(goals_canceling=[GoalInfo() for _ in range(n_goals)])


def test_waits_for_the_service_that_a_fresh_client_has_not_matched_yet() -> None:
    """기동 직후에는 `service_is_ready()` 가 False 다 — 기다리지 않으면 매번 아무것도 안 한다."""
    cancel = _CancelClient(available=True, response=_response(0))

    assert _jtc(cancel).cancel_orphaned_goals(timeout_sec=2.0) is True

    assert cancel.waits == [2.0]
    assert len(cancel.requests) == 1


def test_sends_a_cancel_all_request() -> None:
    """`goal_id` · `stamp` 가 0 인 요청은 그 서버의 **모든** goal 취소다 — 핸들이 없어도 닿는다."""
    cancel = _CancelClient(available=True, response=_response(0))

    _jtc(cancel).cancel_orphaned_goals(timeout_sec=1.0)

    request = cancel.requests[0]
    assert list(request.goal_info.goal_id.uuid) == [0] * 16
    assert request.goal_info.stamp.sec == 0 and request.goal_info.stamp.nanosec == 0


def test_absent_controller_is_not_an_error() -> None:
    """컨트롤러가 아직 없으면 끊을 궤적도 없다."""
    cancel = _CancelClient(available=False)
    client = _jtc(cancel)

    assert client.cancel_orphaned_goals(timeout_sec=1.0) is False

    assert cancel.requests == []
    assert client._node.logger.warnings == []


def test_reports_how_many_goals_were_actually_cancelled() -> None:
    """끊긴 goal 이 있었다는 것은 이전 프로세스가 로봇을 움직이던 중이었다는 단서다."""
    client = _jtc(_CancelClient(available=True, response=_response(1)))

    client.cancel_orphaned_goals(timeout_sec=1.0)

    assert any('cancelled 1 orphaned' in m for m in client._node.logger.warnings)


def test_quiet_when_nothing_was_running() -> None:
    client = _jtc(_CancelClient(available=True, response=_response(0)))

    client.cancel_orphaned_goals(timeout_sec=1.0)

    assert client._node.logger.warnings == []
    assert any('no orphaned' in m for m in client._node.logger.infos)


def test_non_positive_timeout_is_rejected() -> None:
    with pytest.raises(ValueError):
        _jtc(_CancelClient(available=True)).cancel_orphaned_goals(timeout_sec=0.0)


def test_streaming_clients_have_nothing_to_cancel() -> None:
    """JGPC 는 스트리밍이 곧 실행이라 프로세스가 죽으면 실행도 끝난다 — 기본 구현은 no-op."""
    assert MoveGroupClient.cancel_orphaned_goals(object.__new__(MoveGroupJtcClient)) is False
