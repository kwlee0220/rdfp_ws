"""episode.detector 단위 테스트."""

from __future__ import annotations

from rdfp.dataset.ingest.episode.detector import (
    Episode, SessionEvent,
    STATE_IDLE, STATE_IN_SESSION, STATE_IN_EPISODE,
    detect_episodes, parse_metadata, parse_outcome,
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


# ----- outcome / metadata (에피소드 종료 이벤트에만 실려 온다) -----

def _stop_ev(ns: int, task: str = '', outcome: str = '', metadata: str = '') -> SessionEvent:
    return SessionEvent(stamp_ns=ns, state=STATE_IN_SESSION, task_label=task,
                        outcome=outcome, metadata=metadata)


def test_outcome_and_metadata_come_from_the_stop_event() -> None:
    """시작 이벤트가 아니라 종료 이벤트의 값이 에피소드에 붙는다."""
    events = [
        _ev(100, STATE_IN_SESSION, 'pick'),
        _ev(200, STATE_IN_EPISODE, 'pick'),
        _stop_ev(500, 'pick', outcome='failure', metadata='{"seed": 42}'),
    ]

    (ep,) = detect_episodes(events)

    assert ep.success is False
    assert ep.metadata == {'seed': 42}


def test_empty_outcome_means_no_verdict_not_failure() -> None:
    """teleop 처럼 판정 수단이 없는 경로는 빈 값으로 온다 — 실패로 기록하면 안 된다."""
    events = [
        _ev(100, STATE_IN_SESSION),
        _ev(200, STATE_IN_EPISODE),
        _stop_ev(500),
    ]

    (ep,) = detect_episodes(events)

    assert ep.success is None
    assert ep.metadata is None


def test_success_outcome_maps_to_true() -> None:
    events = [
        _ev(100, STATE_IN_SESSION),
        _ev(200, STATE_IN_EPISODE),
        _stop_ev(500, outcome='success'),
    ]

    assert detect_episodes(events)[0].success is True


def test_each_episode_keeps_its_own_outcome() -> None:
    events = [
        _ev(100, STATE_IN_SESSION),
        _ev(200, STATE_IN_EPISODE),
        _stop_ev(300, outcome='success', metadata='{"seed": 1}'),
        _ev(400, STATE_IN_EPISODE),
        _stop_ev(500, outcome='failure', metadata='{"seed": 2}'),
    ]

    first, second = detect_episodes(events)

    assert (first.success, first.metadata) == (True, {'seed': 1})
    assert (second.success, second.metadata) == (False, {'seed': 2})


def test_stop_session_recovery_path_leaves_no_verdict() -> None:
    """IN_EPISODE 에서 stop_session 하면 2단 전이로 닫히며 판정이 없다."""
    events = [
        _ev(100, STATE_IN_SESSION),
        _ev(200, STATE_IN_EPISODE),
        _stop_ev(500),            # stop_session 이 만든 IN_SESSION
        _ev(501, STATE_IDLE),
    ]

    (ep,) = detect_episodes(events)

    assert ep.success is None


def test_parse_outcome_downgrades_unknown_values_to_no_verdict() -> None:
    """모르는 값을 실패로 단정하면 성공한 에피소드가 학습셋에서 조용히 빠진다."""
    assert parse_outcome('success') is True
    assert parse_outcome('failure') is False
    assert parse_outcome('') is None
    assert parse_outcome('SUCCESS') is None
    assert parse_outcome('ok') is None


def test_parse_metadata_rejects_non_object_json() -> None:
    """jsonb 컬럼에 들어가므로 최상위는 object 여야 조회가 성립한다."""
    assert parse_metadata('{"a": 1}') == {'a': 1}
    assert parse_metadata('') is None
    assert parse_metadata('[1, 2]') is None
    assert parse_metadata('42') is None
    assert parse_metadata('not json') is None


def test_bad_metadata_does_not_drop_the_episode() -> None:
    """부가 정보 하나 때문에 관절·이미지가 담긴 에피소드를 잃으면 안 된다."""
    events = [
        _ev(100, STATE_IN_SESSION),
        _ev(200, STATE_IN_EPISODE),
        _stop_ev(500, outcome='success', metadata='{oops'),
    ]

    (ep,) = detect_episodes(events)

    assert ep.success is True        # outcome 은 살아 있다
    assert ep.metadata is None
