"""세션/에피소드 경계 연산의 입력 검증과 서비스 호출 매핑 테스트.

`session_control_node` 를 띄우지 않고 `SessionControlClient` 를 대역으로 바꾼다.
ROS 없이 돈다.
"""

from __future__ import annotations

from typing import Any

import pytest

from rdfp.twin.backends import (
    BackendUnavailable, PreconditionFailed, _start_episode, _start_session, _stop_episode,
    _stop_session, validate_inputs
)


class _FakeFuture:
    def __init__(self, success: bool = True, message: str = '') -> None:
        self._response = type('R', (), {'success': success, 'message': message})()

    def done(self) -> bool:
        return True

    def exception(self):
        return None

    def result(self):
        return self._response


class _FakeSessionClient:
    """호출을 기록하는 SessionControlClient 대역."""

    def __init__(self, ready: bool = True, success: bool = True, message: str = '') -> None:
        self._ready = ready
        self._success = success
        self._message = message
        self.calls: list[tuple[str, dict]] = []

    def is_ready(self) -> bool:
        return self._ready

    def _record(self, name: str, **kwargs) -> _FakeFuture:
        self.calls.append((name, kwargs))
        return _FakeFuture(self._success, self._message)

    def set_task_label_async(self, label):
        return self._record('set_task_label', label=label)

    def start_session_async(self):
        return self._record('start_session')

    def stop_session_async(self):
        return self._record('stop_session')

    def start_episode_async(self):
        return self._record('start_episode')

    def stop_episode_async(self, outcome='', metadata=''):
        return self._record('stop_episode', outcome=outcome, metadata=metadata)


class _FakeRuntime:
    def __init__(self, client: _FakeSessionClient) -> None:
        self._client = client

    def session_control(self) -> Any:
        return self._client


class _FakeSession:
    def __init__(self, **inputs: Any) -> None:
        self.inputs = inputs


def _run(handler, client, **inputs):
    return handler(_FakeRuntime(client), None, None, _FakeSession(**inputs), 5.0)


# ----- 호출 매핑 -----

def test_start_session_sets_the_label_before_opening() -> None:
    """set_task_label 은 IN_EPISODE 에서 거부되므로 IDLE 일 때 먼저 불러야 한다."""
    client = _FakeSessionClient()

    out = _run(_start_session, client, task_label='pick_red_cube')

    assert [name for name, _ in client.calls] == ['set_task_label', 'start_session']
    assert client.calls[0][1]['label'] == 'pick_red_cube'
    assert out['task_label'] == 'pick_red_cube'


def test_start_session_without_a_label_skips_set_task_label() -> None:
    client = _FakeSessionClient()

    _run(_start_session, client)

    assert [name for name, _ in client.calls] == ['start_session']


def test_stop_episode_forwards_outcome_and_serialises_metadata() -> None:
    """metadata 는 HTTP 로 object 를 받고 서비스로는 JSON 문자열로 보낸다."""
    client = _FakeSessionClient()

    _run(_stop_episode, client, outcome='failure', metadata={'seed': 42})

    name, kwargs = client.calls[0]
    assert name == 'stop_episode'
    assert kwargs['outcome'] == 'failure'
    assert kwargs['metadata'] == '{"seed": 42}'


def test_stop_episode_defaults_mean_no_verdict() -> None:
    """teleop 처럼 판정 수단이 없는 경로와 같은 상태다 — 실패가 아니다."""
    client = _FakeSessionClient()

    out = _run(_stop_episode, client)

    assert client.calls[0][1] == {'outcome': '', 'metadata': ''}
    assert out['outcome'] == ''


def test_simple_handlers_call_their_service() -> None:
    for handler, expected in ((_stop_session, 'stop_session'), (_start_episode, 'start_episode')):
        client = _FakeSessionClient()
        _run(handler, client)
        assert [name for name, _ in client.calls] == [expected]


# ----- 거부 매핑 -----

def test_rejection_surfaces_the_reason_from_the_node() -> None:
    """상태 기계의 거부는 실행 중단이 아니라 사전조건 불충족이다.

    `EXECUTION_ABORTED` 로 나가면 "실행하다 죽었다"로 읽혀 클라이언트가 재시도하게
    되는데, 실제로는 아무것도 실행되지 않았고 상태를 먼저 맞춰야 한다.
    """
    client = _FakeSessionClient(success=False, message='invalid command')

    with pytest.raises(PreconditionFailed, match='invalid command'):
        _run(_start_episode, client)


def test_missing_session_control_node_is_reported() -> None:
    client = _FakeSessionClient(ready=False)

    with pytest.raises(BackendUnavailable, match='session_control_node'):
        _run(_start_episode, client)


# ----- 입력 검증 -----

def test_outcome_accepts_only_the_two_verdicts() -> None:
    validate_inputs('stop_episode', {'outcome': 'success'})
    validate_inputs('stop_episode', {'outcome': 'failure'})
    validate_inputs('stop_episode', {})                       # 생략 = 판정 없음

    with pytest.raises(ValueError, match='outcome'):
        validate_inputs('stop_episode', {'outcome': 'ok'})
    with pytest.raises(ValueError, match='outcome'):
        validate_inputs('stop_episode', {'outcome': True})


def test_metadata_must_be_a_json_object() -> None:
    validate_inputs('stop_episode', {'metadata': {'seed': 1}})

    for bad in ([1, 2], 'text', 42):
        with pytest.raises(ValueError, match='metadata'):
            validate_inputs('stop_episode', {'metadata': bad})


def test_metadata_must_be_serialisable() -> None:
    with pytest.raises(ValueError, match='metadata'):
        validate_inputs('stop_episode', {'metadata': {'bad': object()}})
