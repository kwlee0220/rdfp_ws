"""`session_control` 이 없어도 teleop 이 동작하는지 고정한다.

`session_control_node` 는 수집 계층(`rdfp`) 소속이라 제어 스택만 띄운 경우 존재하지
않는다. 그때 텔레오퍼레이션까지 못 쓰게 되면 안 된다.
"""

from __future__ import annotations

import pytest

pytest.importorskip('rclpy')

from rdfp.session.session_control_client import SessionControlClient  # noqa: E402


class _StubClient:
    def __init__(self, ready: bool) -> None:
        self._ready = ready
        self.calls: list = []

    def service_is_ready(self) -> bool:
        return self._ready

    def call_async(self, request):
        self.calls.append(request)
        raise AssertionError('미준비 서비스에는 call_async 가 불리면 안 된다')


class _StubLogger:
    def __init__(self) -> None:
        self.messages: list[str] = []

    def error(self, m): self.messages.append(m)
    def warning(self, m): self.messages.append(m)
    def info(self, m): self.messages.append(m)


def _client_with(ready: bool) -> SessionControlClient:
    """생성자를 우회해 최소 상태만 갖춘 인스턴스를 만든다."""
    obj = SessionControlClient.__new__(SessionControlClient)
    obj._logger = _StubLogger()
    stub = _StubClient(ready)
    for name in ('_start_session_cli', '_stop_session_cli', '_start_episode_cli',
                 '_stop_episode_cli', '_set_task_label_cli', '_get_session_state_cli'):
        setattr(obj, name, stub)
    return obj


def test_async_call_fails_fast_when_service_is_absent():
    """미준비 서비스의 비동기 호출은 **즉시 실패로 끝난다.**

    `call_async` 는 서버가 없어도 future 를 돌려주는데 그것이 영영 완료되지 않아
    콜백이 불리지 않는다 — 호출자는 응답 없이 기다리고 로그에도 단서가 없다.
    """
    client = _client_with(ready=False)
    got: list = []
    client.start_session_async(done_callback=lambda ok, msg: got.append((ok, msg)))
    assert len(got) == 1
    ok, msg = got[0]
    assert ok is False
    assert 'not ready' in msg


def test_wait_until_ready_returns_false_instead_of_raising():
    """`create()` 는 RuntimeError 를 내지만 이쪽은 False 를 돌려준다."""
    assert _client_with(ready=False).wait_until_ready(0.0) is False
    assert _client_with(ready=True).wait_until_ready(0.0) is True
