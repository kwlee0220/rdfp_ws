"""episode.detector 단위 테스트."""

from __future__ import annotations

from rdfp.dataset.ingest.episode.detector import (
    Episode, SessionEvent,
    STATE_IDLE, STATE_IN_SESSION, STATE_IN_EPISODE,
    detect_episodes,
)


def _ev(ns: int, state: str, task: str = '') -> SessionEvent:
    return SessionEvent(stamp_ns=ns, state=state, task_label=task)


def test_normal_episode_cycle() -> None:
    # IDLE → IN_SESSION → IN_EPISODE → IN_SESSION → IDLE
    events = [
        _ev(0,    STATE_IDLE),
        _ev(100,  STATE_IN_SESSION, 'pick'),
        _ev(200,  STATE_IN_EPISODE, 'pick'),
        _ev(500,  STATE_IN_SESSION, 'pick'),
        _ev(501,  STATE_IDLE,       'pick'),
    ]
    eps = detect_episodes(events)
    assert eps == [Episode(200, 500, 'pick')]


def test_stop_session_two_step_transition() -> None:
    # IN_EPISODE 중 stop_session 호출: 2단 전이 (IN_SESSION → IDLE).
    events = [
        _ev(0,    STATE_IDLE),
        _ev(100,  STATE_IN_SESSION, 'demo'),
        _ev(200,  STATE_IN_EPISODE, 'demo'),
        _ev(400,  STATE_IN_SESSION, 'demo'),   # stop_msg (이전이 IN_EPISODE)
        _ev(401,  STATE_IDLE,       'demo'),   # 2단 전이의 두 번째, stop_msg 아님
    ]
    eps = detect_episodes(events)
    assert eps == [Episode(200, 400, 'demo')]


def test_idle_to_in_session_is_not_stop() -> None:
    # IDLE → IN_SESSION 의 IN_SESSION 은 stop_msg 가 아니다 (에피소드 없음).
    events = [
        _ev(0,    STATE_IDLE),
        _ev(100,  STATE_IN_SESSION, 'x'),
        _ev(200,  STATE_IDLE,       'x'),
    ]
    assert detect_episodes(events) == []


def test_multiple_episodes_in_one_session() -> None:
    events = [
        _ev(0,    STATE_IDLE),
        _ev(100,  STATE_IN_SESSION, 'A'),
        _ev(200,  STATE_IN_EPISODE, 'A'),
        _ev(300,  STATE_IN_SESSION, 'A'),      # end ep1
        _ev(400,  STATE_IN_EPISODE, 'A'),
        _ev(600,  STATE_IN_SESSION, 'A'),      # end ep2
        _ev(700,  STATE_IDLE,       'A'),
    ]
    eps = detect_episodes(events)
    assert eps == [Episode(200, 300, 'A'), Episode(400, 600, 'A')]


def test_unfinished_episode_is_dropped() -> None:
    # IN_EPISODE 로 끝나고 종료 경계가 없으면 폐기.
    events = [
        _ev(0,    STATE_IDLE),
        _ev(100,  STATE_IN_SESSION, 'x'),
        _ev(200,  STATE_IN_EPISODE, 'x'),
    ]
    assert detect_episodes(events) == []


def test_empty_task_label_becomes_none() -> None:
    events = [
        _ev(0,    STATE_IDLE),
        _ev(100,  STATE_IN_SESSION, ''),
        _ev(200,  STATE_IN_EPISODE, ''),
        _ev(300,  STATE_IN_SESSION, ''),
    ]
    eps = detect_episodes(events)
    assert eps == [Episode(200, 300, None)]


def test_stop_ts_not_after_start_is_skipped() -> None:
    # stop_ts <= start_ts 는 비정상으로 취급해 버린다.
    events = [
        _ev(200,  STATE_IN_EPISODE, 'x'),
        _ev(200,  STATE_IN_SESSION, 'x'),     # 동일 stamp → skipped
    ]
    assert detect_episodes(events) == []
