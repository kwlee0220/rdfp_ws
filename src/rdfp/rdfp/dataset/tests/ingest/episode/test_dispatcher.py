"""episode.dispatcher 단위 테스트."""

from __future__ import annotations

from dataclasses import dataclass

from rdfp.dataset.ingest.episode.detector import Episode
from rdfp.dataset.ingest.episode.dispatcher import AssignedMessage, assign_and_dispatch


@dataclass
class _Msg:
    stamp_ns: int
    topic: str = '/topic'
    type_name: str = 'pkg/msg/Type'
    msg: object = None


def _collect(episodes, msgs):
    return list(assign_and_dispatch(episodes, msgs))


def test_empty_episodes_yields_nothing() -> None:
    out = _collect([], [_Msg(100), _Msg(200)])
    assert out == []


def test_single_episode_with_messages() -> None:
    eps = [Episode(100, 200, 'x')]
    msgs = [_Msg(50), _Msg(100), _Msg(150), _Msg(199), _Msg(200), _Msg(300)]
    out = _collect(eps, msgs)
    kinds = [k for k, _ in out]
    assert kinds == ['enter', 'message', 'message', 'message', 'exit']
    stamps = [p.stamp_ns for k, p in out if k == 'message']
    assert stamps == [100, 150, 199]  # 50 스킵, 200 반열림 제외, 300 스킵


def test_half_open_boundary() -> None:
    # start 는 포함 (>=), stop 은 제외 (<)
    eps = [Episode(100, 200, None)]
    out = _collect(eps, [_Msg(100), _Msg(200)])
    msgs = [p.stamp_ns for k, p in out if k == 'message']
    assert msgs == [100]


def test_empty_episode_still_emits_enter_exit() -> None:
    # 메시지가 전혀 포함되지 않아도 enter/exit 는 발행한다 (sessions 행 생성 보장).
    eps = [Episode(100, 200, None), Episode(300, 400, None)]
    # 모든 메시지가 구간 밖
    msgs = [_Msg(10), _Msg(250), _Msg(500)]
    out = _collect(eps, msgs)
    kinds = [k for k, _ in out]
    assert kinds == ['enter', 'exit', 'enter', 'exit']
    assert [p.episode_index for k, p in out if k == 'enter'] == [0, 1]


def test_multiple_episodes_assignment() -> None:
    eps = [Episode(100, 200, 'A'), Episode(300, 400, 'B')]
    msgs = [_Msg(150, '/a'), _Msg(250, '/gap'), _Msg(350, '/b'), _Msg(500, '/after')]
    out = _collect(eps, msgs)
    msg_items = [(p.episode_index, p.topic) for k, p in out if k == 'message']
    assert msg_items == [(0, '/a'), (1, '/b')]
    kinds = [k for k, _ in out]
    assert kinds == ['enter', 'message', 'exit', 'enter', 'message', 'exit']


def test_messages_before_first_episode_are_skipped() -> None:
    eps = [Episode(100, 200, None)]
    out = _collect(eps, [_Msg(10), _Msg(50), _Msg(150)])
    msg_items = [p.stamp_ns for k, p in out if k == 'message']
    assert msg_items == [150]


def test_message_type_preserved_in_assigned() -> None:
    eps = [Episode(0, 1000, None)]
    msgs = [_Msg(100, topic='/t', type_name='pkg/msg/Foo', msg='hello')]
    out = _collect(eps, msgs)
    msg_items = [p for k, p in out if k == 'message']
    assert len(msg_items) == 1
    am: AssignedMessage = msg_items[0]
    assert am.topic == '/t'
    assert am.type_name == 'pkg/msg/Foo'
    assert am.msg == 'hello'
    assert am.episode_index == 0
