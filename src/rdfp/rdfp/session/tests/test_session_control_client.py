#!/usr/bin/env python3

"""`SessionControlClient` 테스트 — 가이드 §6 · §8 의 계약.

**서버 없이 돈다.** rclpy 서비스 클라이언트를 대역으로 세워, 이 래퍼가 약속한
「예외를 (False, 사유) 로 바꾼다」·「타임아웃 0 은 무한 대기」·「서버가 없어도
호출자는 산다」를 확인한다.

이 래퍼가 감추는 함정이 둘이다. rclpy 는 `timeout_sec=0` 을 **즉시 반환**으로 다루므로
그대로 넘기면 서버가 이미 처리한 명령을 호출자가 실패로 오해한다. 그리고 서비스가
없을 때 `call_async` 가 돌려주는 future 는 **영영 완료되지 않는다** — 그래서
`service_is_ready()` 를 먼저 본다.
"""

from __future__ import annotations

from typing import Iterator

from unittest.mock import MagicMock, patch

import pytest

pytest.importorskip('rclpy', reason='requires ROS 2 runtime')

import rclpy                                                            # noqa: E402
from rclpy.node import Node                                             # noqa: E402

from rdfp.session import session_control_client as client_module        # noqa: E402
# **여기서만 모듈 경로로 가져온다** — 아래 재노출 테스트가 패키지 경로의 객체와
# 같은 것인지 확인해야 하므로, 둘을 각각 가져와야 한다.
from rdfp.session.session_control_client import SessionControlClient    # noqa: E402

SERVICES = ('start_session', 'stop_session', 'start_episode',
            'stop_episode', 'set_task_label')


@pytest.fixture(scope='module', autouse=True)
def _rclpy_session() -> Iterator[None]:
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def host() -> Iterator[Node]:
    n = Node('client_host')
    try:
        yield n
    finally:
        n.destroy_node()


def _fake_client(ready: bool = True) -> MagicMock:
    cli = MagicMock(name='service_client')
    cli.service_is_ready.return_value = ready
    cli.wait_for_service.return_value = ready
    return cli


@pytest.fixture
def client(host) -> Iterator[SessionControlClient]:
    """서비스 클라이언트를 전부 대역으로 바꾼 세션 클라이언트."""
    with patch.object(Node, 'create_client', side_effect=lambda *a, **k: _fake_client()):
        yield SessionControlClient.create(host, wait_timeout_sec=0.0)


def _respond(client: SessionControlClient, name: str, success: bool, message: str) -> MagicMock:
    """해당 서비스 호출이 (success, message) 로 응답하도록 세운다."""
    cli = getattr(client, f'_{name}_cli')
    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = MagicMock(success=success, message=message)
    cli.call_async.return_value = future
    return cli


# ---------- 준비 상태 ----------

def test_wait_until_ready_is_false_instead_of_raising(client):
    """**서버가 없어도 호출자는 살아야 한다** (§6.4).

    `teleop_keyboard` 는 제어 스택만 띄운 경우에도 동작해야 한다 — 세션 키만 꺼진다.
    """
    for name in SERVICES:
        getattr(client, f'_{name}_cli').service_is_ready.return_value = False
    assert client.wait_until_ready(0.0) is False


def test_is_ready_requires_all_five(client):
    """하나라도 없으면 준비된 것이 아니다 — 절반만 되는 상태가 더 나쁘다."""
    assert client.is_ready() is True
    client._stop_episode_cli.service_is_ready.return_value = False
    assert client.is_ready() is False


def test_create_raises_when_waiting_and_services_are_missing(host):
    """`create()` 는 대기를 켜면 실패를 **예외로** 낸다 — 세션이 목적인 노드용이다."""
    with patch.object(Node, 'create_client',
                      side_effect=lambda *a, **k: _fake_client(ready=False)):
        with pytest.raises(RuntimeError, match='not available'):
            SessionControlClient.create(host, wait_timeout_sec=0.01)


# ---------- 동기 호출 ----------

def test_missing_service_returns_a_reason_not_a_hang(client):
    """**`call_async` 로 가면 future 가 영영 안 온다** — 먼저 걸러 사유를 돌려준다."""
    client._start_session_cli.service_is_ready.return_value = False
    ok, message = client.start_session()
    assert ok is False and message == 'service not ready'
    client._start_session_cli.call_async.assert_not_called()


def test_server_rejection_is_passed_through(client):
    """서버의 거부 사유를 삼키면 호출자가 원인을 못 본다."""
    _respond(client, 'set_task_label', False, 'invalid command')
    assert client.set_task_label('x') == (False, 'invalid command')


def test_success_is_passed_through(client):
    _respond(client, 'start_session', True, '')
    assert client.start_session() == (True, '')


def test_timeout_returns_a_reason(client):
    """타임아웃은 **명령이 취소됐다는 뜻이 아니다** — 서버는 처리했을 수 있다."""
    cli = _respond(client, 'start_episode', True, '')
    cli.call_async.return_value.done.return_value = False
    ok, message = client.start_episode(timeout_sec=0.01)
    assert ok is False and message == 'service call timed out'


def test_zero_timeout_means_wait_forever(client):
    """rclpy 는 0 을 '즉시 반환'으로 다룬다 — `None` 으로 바꿔 넘겨야 한다.

    안 바꾸면 서버가 이미 처리한 명령에 timed out 을 돌려준다(실측).
    """
    _respond(client, 'start_session', True, '')
    with patch.object(client_module.rclpy, 'spin_until_future_complete') as spin:
        client.start_session(timeout_sec=0.0)
    assert spin.call_args.kwargs['timeout_sec'] is None


def test_positive_timeout_is_forwarded(client):
    _respond(client, 'start_session', True, '')
    with patch.object(client_module.rclpy, 'spin_until_future_complete') as spin:
        client.start_session(timeout_sec=2.5)
    assert spin.call_args.kwargs['timeout_sec'] == 2.5


def test_exception_becomes_a_reason(client):
    """**어떤 예외든 (False, 사유) 로 바꾼다** — 호출자가 try 로 감쌀 필요가 없다."""
    cli = _respond(client, 'stop_session', True, '')
    cli.call_async.return_value.result.side_effect = RuntimeError('boom')
    ok, message = client.stop_session()
    assert ok is False and 'boom' in message


def test_none_response_becomes_a_reason(client):
    cli = _respond(client, 'stop_session', True, '')
    cli.call_async.return_value.result.return_value = None
    ok, message = client.stop_session()
    assert ok is False and message


# ---------- 요청 구성 ----------

def test_stop_episode_forwards_outcome_and_metadata(client):
    """이 둘은 **이 전이에만** 실리므로 여기서 빠지면 되살릴 수 없다."""
    cli = _respond(client, 'stop_episode', True, '')
    client.stop_episode(outcome='success', metadata='{"k": 1}')
    request = cli.call_async.call_args.args[0]
    assert (request.outcome, request.metadata) == ('success', '{"k": 1}')


def test_stop_episode_defaults_are_empty(client):
    """비워 부르는 것이 '판정 없음'이다 — teleop 처럼 값을 줄 수단이 없는 경로용."""
    cli = _respond(client, 'stop_episode', True, '')
    client.stop_episode()
    request = cli.call_async.call_args.args[0]
    assert (request.outcome, request.metadata) == ('', '')


def test_none_label_is_sent_as_empty_string(client):
    """`None` 은 라벨 clear 다 — 그대로 넘기면 메시지 필드 대입에서 죽는다."""
    cli = _respond(client, 'set_task_label', True, '')
    client.set_task_label(None)
    assert cli.call_async.call_args.args[0].task_label == ''


# ---------- namespace ----------

def test_namespace_is_applied_to_every_service(host):
    """서버를 다른 namespace 로 띄웠으면 다섯 서비스가 **함께** 옮겨가야 한다.

    **로봇별 세션을 뜻하지 않는다** — 세션 서버는 시스템에 하나다(가이드 §4.3).
    로봇이 둘 이상인 것은 공동 작업으로 하나의 데이터를 만든다는 뜻이고, 다른 학습
    작업은 `ROS_DOMAIN_ID` 를 나눈다. 이 인자는 서버를 기본이 아닌 이름으로 띄운
    경우를 위한 것이며, 다섯 중 하나라도 빠지면 그 명령만 조용히 안 간다.
    """
    created: list[str] = []

    def spy(_type, name, **kwargs):
        created.append(name)
        return _fake_client()

    with patch.object(Node, 'create_client', side_effect=spy):
        SessionControlClient.create(host, namespace='/other/session_control',
                                    wait_timeout_sec=0.0)

    assert created == [f'/other/session_control/{n}' for n in SERVICES]


# ---------- 패키지 공개 API ----------

def test_client_is_exported_from_the_package():
    """`rdfp.session.SessionControlClient` 가 공개 경로다.

    **모듈 경로가 아니라 이쪽이 계약이다** — 호출자(teleop·트윈)가 전부 이 경로를
    쓰므로, 재노출을 지우거나 클래스를 다른 모듈로 옮기면 그들이 함께 깨진다.
    """
    import rdfp.session as pkg

    assert pkg.SessionControlClient is SessionControlClient
    assert 'SessionControlClient' in pkg.__all__


def test_package_does_not_pull_in_the_node():
    """노드 구현은 끌어올리지 않는다 — 클라이언트만 쓰려는 호출자의 부담이 된다."""
    import rdfp.session as pkg

    assert not hasattr(pkg, 'SessionControlNode')


def test_default_service_namespace_is_absolute(host):
    """**기본 서비스 경로도 절대다** — 세션 서버는 시스템에 하나다.

    상대로 두면 호출자가 로봇별 네임스페이스 안에 있을 때
    `/abc/session_control/start_session` 을 찾아 **영영 준비되지 않는다.**
    증상은 "세션 키가 안 먹는다" 하나뿐이라 원인이 안 보인다.
    """
    created: list[str] = []

    def spy(_type, name, **kwargs):
        created.append(name)
        return _fake_client()

    with patch.object(Node, 'create_client', side_effect=spy):
        SessionControlClient.create(host, wait_timeout_sec=0.0)

    assert created == [f'/session_control/{n}' for n in SERVICES]
