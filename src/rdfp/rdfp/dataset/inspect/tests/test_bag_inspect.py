"""bag 진단 — 합성 bag 으로 세션 트리 · 토픽 요약 · 스키마 · stamp 진단 · 열람을 본다 (ROS 없음)."""

from __future__ import annotations

import shutil

from rdfp.dataset.ingest.episode.from_bag import episodes_from_splits
from rdfp.dataset.inspect.bag_inspect import (
    NOTE_UNFINALIZED, diagnose_stamps, episode_windows, finalized_splits, flatten_message,
    mcap_paths_for_topic, read_schema_text, read_topic_rows, row_columns, scan_sessions,
    summarize_topics)
from rdfp.rosbag.tests.fixtures.make_synth_bag import (JointStateEventSpec, SessionEventSpec,
                                                       write_synth_bag)

S0 = 1_700_000_000_000_000_000
SEC = 1_000_000_000


def _bag(root):
    """에피소드 하나(10~20 초) · 관절 8건 — 창 안 5, 창 앞 2, stamp 0 짜리 1."""
    session = [
        SessionEventSpec(S0 + 0 * SEC, 'IN_SESSION', 'pick'),
        SessionEventSpec(S0 + 10 * SEC, 'IN_EPISODE', 'pick'),
        SessionEventSpec(S0 + 20 * SEC, 'IN_SESSION', 'pick'),
    ]
    joints = [JointStateEventSpec(S0 + s * SEC, position=[0.1 * s]) for s in (5, 7)]
    joints += [JointStateEventSpec(S0 + s * SEC, position=[0.1 * s]) for s in (11, 12, 13, 14, 15)]
    joints.append(JointStateEventSpec(0, position=[9.9]))
    return write_synth_bag(root, session_events=session, joint_state_events=joints)


def test_scan_sessions_shows_unfinalized_sessions_too(tmp_path) -> None:
    """`discover_splits` 가 조용히 빼는 세션이 여기서는 '미완료' 로 보인다."""
    done = _bag(tmp_path)
    broken = tmp_path / '2026-04-11' / 'session_2026-04-11_10-00-00'
    broken.mkdir()
    shutil.copy(next(done.glob('*.mcap')), broken / 'session_2026-04-11_10-00-00_0.mcap')

    sessions = scan_sessions(tmp_path)

    by_name = {s.name: s for s in sessions}
    assert by_name[done.name].finalized is True
    assert by_name[done.name].message_count == 11
    assert {t.name for t in by_name[done.name].topics} == {'/session', '/joint_states'}
    unfinished = by_name[broken.name]
    assert unfinished.finalized is False
    assert unfinished.note.startswith(NOTE_UNFINALIZED)
    # MCAP summary 에서 읽은 토픽 · 수는 완료 세션과 같다.
    counts = {t.name: t.message_count for t in unfinished.topics}
    assert counts == {'/session': 3, '/joint_states': 8}
    assert len(finalized_splits(sessions)) == 1


def test_summarize_topics_defaults_to_what_import_sees(tmp_path) -> None:
    done = _bag(tmp_path)
    broken = tmp_path / '2026-04-11' / 'session_2026-04-11_10-00-00'
    broken.mkdir()
    shutil.copy(next(done.glob('*.mcap')), broken / 'x_0.mcap')
    sessions = scan_sessions(tmp_path)

    only_done = {t.name: t for t in summarize_topics(sessions)}
    assert only_done['/joint_states'].message_count == 8
    assert only_done['/joint_states'].type_name == 'sensor_msgs/msg/JointState'
    assert only_done['/joint_states'].session_count == 1

    both = {t.name: t for t in summarize_topics(sessions, include_unfinalized=True)}
    assert both['/joint_states'].message_count == 16
    assert both['/joint_states'].session_count == 2


def test_schema_text_is_the_msg_definition(tmp_path) -> None:
    _bag(tmp_path)
    sessions = scan_sessions(tmp_path)
    mcap = mcap_paths_for_topic(sessions, '/joint_states')[0]

    schema = read_schema_text(mcap, '/joint_states')

    assert schema is not None
    assert schema.type_name == 'sensor_msgs/msg/JointState'
    assert 'float64[] position' in schema.text
    assert read_schema_text(mcap, '/nope') is None


def test_stamp_diagnosis_counts_like_import_filters(tmp_path) -> None:
    """창 안 5 · 창 앞 2 · stamp 0 하나 — 적재가 넣는 것은 5 건뿐이고 그것이 숫자로 보인다."""
    _bag(tmp_path)
    sessions = scan_sessions(tmp_path)
    windows = episode_windows(episodes_from_splits(finalized_splits(sessions)))
    assert windows == [(S0 + 10 * SEC, S0 + 20 * SEC)]

    diag = diagnose_stamps(mcap_paths_for_topic(sessions, '/joint_states'), '/joint_states',
                           windows=windows)

    assert diag.total == 8
    assert diag.without_stamp == 0
    assert diag.zero_stamp == 1
    assert diag.in_episode == 5
    assert diag.episodes_checked == 1
    assert diag.suspect is True          # stamp 0 이 하나 있다
    assert diag.first_stamp_ns == 0
    assert any('에피소드 창 안: 5' in line for line in diag.summary_lines())


def test_stamp_diagnosis_without_windows_is_not_suspect_when_clean(tmp_path) -> None:
    _bag(tmp_path)
    sessions = scan_sessions(tmp_path)
    diag = diagnose_stamps(mcap_paths_for_topic(sessions, '/session'), '/session')
    assert diag.total == 3
    assert diag.episodes_checked is None
    assert diag.suspect is False
    assert diag.mean_rate_hz is not None


def test_stamp_diagnosis_flags_nothing_in_any_window(tmp_path) -> None:
    """창을 봤는데 한 건도 안 들면 시계 불일치의 전형 — `suspect`."""
    _bag(tmp_path)
    sessions = scan_sessions(tmp_path)
    far = [(S0 + 1000 * SEC, S0 + 1010 * SEC)]
    diag = diagnose_stamps(mcap_paths_for_topic(sessions, '/session'), '/session', windows=far)
    assert diag.in_episode == 0
    assert diag.suspect is True


def test_read_topic_rows_pages_in_stamp_order(tmp_path) -> None:
    _bag(tmp_path)
    sessions = scan_sessions(tmp_path)
    paths = mcap_paths_for_topic(sessions, '/joint_states')

    first, more = read_topic_rows(paths, '/joint_states', limit=3)
    assert more is True
    assert [r.stamp_ns for r in first] == [0, S0 + 5 * SEC, S0 + 7 * SEC]
    assert first[1].fields['position'] == [0.5]
    assert 'header.stamp.sec' in row_columns(first)

    rest, more = read_topic_rows(paths, '/joint_states', offset=3, limit=100)
    assert more is False
    assert len(rest) == 5
    assert rest[0].stamp_ns == S0 + 11 * SEC

    windowed, _ = read_topic_rows(paths, '/joint_states',
                                  time_range_ns=(S0 + 10 * SEC, S0 + 20 * SEC))
    assert len(windowed) == 5


def test_flatten_message_truncates_long_sequences_and_bytes() -> None:
    class Stamp:
        __slots__ = ('sec', 'nanosec')

        def __init__(self) -> None:
            self.sec, self.nanosec = 1, 2

    class Header:
        __slots__ = ('stamp', 'frame_id')

        def __init__(self) -> None:
            self.stamp, self.frame_id = Stamp(), 'cam'

    class Msg:
        __slots__ = ('header', 'data', 'values')

        def __init__(self) -> None:
            self.header, self.data, self.values = Header(), b'\x00' * 5, list(range(20))

    flat = flatten_message(Msg(), max_items=4)
    assert flat['header.stamp.sec'] == 1
    assert flat['header.frame_id'] == 'cam'
    assert flat['data'] == '<5 bytes>'
    assert flat['values'] == [0, 1, 2, 3, '… (+16)']
