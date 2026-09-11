#!/usr/bin/env python3

"""`SessionControlNode` 상태 머신 테스트.

명세는 [docs/session/session_control_guide.md](../../../../../docs/session/session_control_guide.md)
§2.1 「서비스별 허용 상태」 표다. 그 표를 그대로 파라미터화해서 **표와 코드가
갈라지면 실패**하도록 만든다.

이 노드는 수집 전체의 시간 기준점이다. 여기서 잘못 발행된 한 줄이 rosbag 에 남고,
적재는 그 stamp 로 에피소드 창을 자르므로 **나중에 고칠 방법이 없다** — 그래서
거부는 조용해야 하고(상태도 토픽도 안 변해야 한다) 허용은 정확해야 한다.
"""

from __future__ import annotations

from typing import Iterator

from unittest.mock import MagicMock

import pytest

pytest.importorskip('rclpy', reason='requires ROS 2 runtime')

import rclpy                                                        # noqa: E402
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy           # noqa: E402
from rdfp_msgs.srv import GetSessionState, SetString, StopEpisode   # noqa: E402
from std_srvs.srv import Trigger                                    # noqa: E402

from rdfp.session.session_control_node import (                     # noqa: E402
    SessionControlNode, SessionState)

IDLE, IN_SESSION, IN_EPISODE = (
    SessionState.IDLE, SessionState.IN_SESSION, SessionState.IN_EPISODE)
ALL_STATES = (IDLE, IN_SESSION, IN_EPISODE)


@pytest.fixture(scope='module', autouse=True)
def _rclpy_session() -> Iterator[None]:
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def node() -> Iterator[SessionControlNode]:
    """발행을 가로챈 노드. 상태는 `_state` 로 직접 세운다."""
    n = SessionControlNode()
    n._pub = MagicMock(name='session_pub')
    try:
        yield n
    finally:
        n.destroy_node()


def _published(node: SessionControlNode) -> list:
    return [c.args[0] for c in node._pub.publish.call_args_list]


def _call(node: SessionControlNode, service: str, **kw):
    """서비스 핸들러를 직접 부른다 (spin 없이 계약만 본다)."""
    handlers = {
        'start_session': (node._handle_start_session, Trigger),
        'stop_session': (node._handle_stop_session, Trigger),
        'start_episode': (node._handle_start_episode, Trigger),
        'stop_episode': (node._handle_stop_episode, StopEpisode),
        'set_task_label': (node._handle_set_task_label, SetString),
        'get_session_state': (node._handle_get_state, GetSessionState),
    }
    handler, srv = handlers[service]
    request = srv.Request()
    for key, value in kw.items():
        setattr(request, key, value)
    return handler(request, srv.Response())


# ---------- §2.1 허용 상태 표 ----------

# (서비스, 허용 상태 집합) — 가이드 §2.1 을 그대로 옮긴 것이다.
ALLOWED = [
    ('start_session', {IDLE}),
    ('start_episode', {IN_SESSION}),
    ('stop_episode', {IN_EPISODE}),
    ('stop_session', {IN_SESSION, IN_EPISODE}),
    ('set_task_label', {IDLE, IN_SESSION}),
]


@pytest.mark.parametrize('service,allowed', ALLOWED)
@pytest.mark.parametrize('state', ALL_STATES)
def test_service_is_accepted_exactly_in_its_allowed_states(node, service, allowed, state):
    node._state = state
    response = _call(node, service)
    assert response.success is (state in allowed), (
        f'{service} in {state.value}: 가이드 §2.1 과 다르다')


@pytest.mark.parametrize('service,allowed', ALLOWED)
@pytest.mark.parametrize('state', ALL_STATES)
def test_rejection_changes_nothing(node, service, allowed, state):
    """**거부는 조용해야 한다** — 상태도 토픽도 변하지 않는다."""
    if state in allowed:
        pytest.skip('허용 상태')
    node._state = state
    node._task_label = 'keep-me'

    response = _call(node, service)

    assert response.success is False
    assert response.message == 'invalid command'
    assert node._state is state
    assert node._task_label == 'keep-me'
    assert _published(node) == []


def test_get_session_state_is_allowed_everywhere_and_publishes_nothing(node):
    """조회는 부작용이 없다 — 어느 상태에서도 되고 아무것도 안 낸다."""
    for state in ALL_STATES:
        node._state = state
        node._task_label = 'peek'
        response = _call(node, 'get_session_state')
        assert response.state == state.value
        assert response.task_label == 'peek'
        assert node._state is state
    assert _published(node) == []


# ---------- 전이가 무엇을 발행하나 ----------

def test_happy_path_publishes_the_expected_sequence(node):
    node._task_label = 'pick_block'
    _call(node, 'start_session')
    _call(node, 'start_episode')
    _call(node, 'stop_episode')
    _call(node, 'stop_session')

    assert [m.state for m in _published(node)] == [
        'IN_SESSION', 'IN_EPISODE', 'IN_SESSION', 'IDLE']
    assert {m.task_label for m in _published(node)} == {'pick_block'}
    assert node._state is IDLE


def test_stop_session_from_in_episode_publishes_two_messages(node):
    """**한 호출로 2단계다** (가이드 §2.1).

    depth 1 로 구독하면 앞의 `IN_SESSION` 을 놓치고 `IDLE` 만 받는다 — 그래서
    구독자는 `IDLE` 도 에피소드 종료로 처리해야 한다 (§3.3).
    """
    node._state = IN_EPISODE
    node._task_label = 'L'

    assert _call(node, 'stop_session').success is True

    assert [m.state for m in _published(node)] == ['IN_SESSION', 'IDLE']
    assert node._state is IDLE


def test_state_is_updated_after_publishing(node):
    """발행 시점에 이미 새 상태여야 한다 (§2.1 마지막 문단).

    구독자가 토픽을 받았을 때 서버에 조회하면 같은 상태가 나와야 하기 때문이다.
    """
    seen: list = []
    node._pub.publish.side_effect = lambda msg: seen.append(node._state)
    _call(node, 'start_session')
    assert seen == [IDLE], '발행 -> 전이 순서다 (SRS 4.4)'
    assert node._state is IN_SESSION


# ---------- stop_episode 의 outcome / metadata ----------

@pytest.mark.parametrize('outcome', ['', 'success', 'failure'])
def test_valid_outcomes_are_accepted(node, outcome):
    """`''` 는 실패가 아니라 **판정 없음**이다 — DB 에 NULL 로 들어간다."""
    node._state = IN_EPISODE
    assert _call(node, 'stop_episode', outcome=outcome).success is True
    assert _published(node)[-1].outcome == outcome


@pytest.mark.parametrize('outcome', ['ok', 'SUCCESS', 'true', 'fail'])
def test_unknown_outcome_is_rejected_before_publishing(node, outcome):
    """**여기서 막지 않으면 rosbag 에 남고 적재 때야 드러난다** — 그때는 못 고친다."""
    node._state = IN_EPISODE
    response = _call(node, 'stop_episode', outcome=outcome)
    assert response.success is False
    assert 'outcome must be one of' in response.message
    assert _published(node) == []
    assert node._state is IN_EPISODE, '거부는 전이를 일으키지 않는다'


@pytest.mark.parametrize('metadata', ['{}', '{"a": 1}', ''])
def test_metadata_accepts_json_objects_and_empty(node, metadata):
    node._state = IN_EPISODE
    assert _call(node, 'stop_episode', metadata=metadata).success is True


@pytest.mark.parametrize('metadata', ['[]', '3', '"str"', 'null', 'not json'])
def test_metadata_must_be_a_json_object(node, metadata):
    """배열·스칼라는 거부한다 — 적재가 키-값으로 펼치기 때문이다."""
    node._state = IN_EPISODE
    response = _call(node, 'stop_episode', metadata=metadata)
    assert response.success is False
    assert 'JSON object' in response.message
    assert _published(node) == []


def test_outcome_and_metadata_ride_only_on_the_episode_end(node):
    """다른 전이에는 실리면 안 된다 — 후처리기가 종료되는 에피소드에 귀속시킨다."""
    node._state = IN_EPISODE
    _call(node, 'stop_episode', outcome='success', metadata='{"k": 1}')
    _call(node, 'stop_session')

    end, session_end = _published(node)
    assert (end.outcome, end.metadata) == ('success', '{"k": 1}')
    assert (session_end.outcome, session_end.metadata) == ('', '')


# ---------- 라벨 ----------

def test_label_is_rejected_during_an_episode(node):
    """**진행 중인 에피소드의 라벨이 바뀌면 안 된다** (§2.2)."""
    node._state = IN_EPISODE
    node._task_label = 'original'
    assert _call(node, 'set_task_label', task_label='new').success is False
    assert node._task_label == 'original'


def test_label_survives_transitions(node):
    _call(node, 'set_task_label', task_label='pick')
    _call(node, 'start_session')
    _call(node, 'start_episode')
    assert _published(node)[-1].task_label == 'pick'


def test_label_can_be_cleared(node):
    node._task_label = 'old'
    assert _call(node, 'set_task_label', task_label='').success is True
    assert node._task_label == ''


def test_label_is_published_before_the_internal_update(node):
    """발행 -> 갱신 순서 (SRS 4.4). 발행된 메시지가 **새 라벨**을 담는다."""
    node._task_label = 'old'
    _call(node, 'set_task_label', task_label='new')
    assert _published(node)[-1].task_label == 'new'
    assert node._task_label == 'new'


# ---------- §3.2 메시지 · §3.3 QoS ----------

def test_every_message_carries_a_stamp(node):
    """**stamp 가 비면 적재가 에피소드 창 밖으로 버린다** — rosbag 에는 남는데 DB 에 없다."""
    _call(node, 'start_session')
    _call(node, 'start_episode')
    for msg in _published(node):
        assert (msg.header.stamp.sec, msg.header.stamp.nanosec) != (0, 0)


def test_publisher_qos_matches_the_contract():
    """`TRANSIENT_LOCAL` 이라야 늦게 붙은 구독자가 현재 상태를 받는다 (§3.3).

    `VOLATILE` 로 바뀌면 매칭 자체가 깨져 값이 영영 안 온다.
    """
    n = SessionControlNode()
    try:
        qos = n._pub.qos_profile
        assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
        assert qos.reliability == ReliabilityPolicy.RELIABLE
        assert qos.depth == 1
    finally:
        n.destroy_node()


def test_topic_is_absolute_and_immune_to_namespaces():
    """**`/session` 은 절대 이름이다 — 세션은 시스템에 하나다.**

    로봇별 네임스페이스를 도입해도 따라 붙으면 안 된다. 상대(`session`)로 두면
    `/abc/session` 이 되어 전역이 아니게 되고, `~/` 면 노드 이름까지 붙는다.
    로봇이 둘 이상인 것은 **공동 작업으로 하나의 학습 데이터를 만든다**는 뜻이며,
    다른 학습 작업은 `ROS_DOMAIN_ID` 를 나눈다 (규약 §2.5).
    """
    n = SessionControlNode()
    try:
        assert n._pub.topic_name == '/session'
    finally:
        n.destroy_node()


def test_topic_name_constant_is_absolute():
    """상수가 상대로 되돌아가면 네임스페이스 면역이 조용히 사라진다."""
    from rdfp.session import session_control_node as module
    assert module._DEFAULT_SESSION_TOPIC.startswith('/')
