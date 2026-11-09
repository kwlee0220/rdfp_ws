"""DB 조회 가운데 커넥션 없이 도는 부분 — 에피소드 점검 · 파일 찾기 · 셀 렌더링.

질의 자체는 PostgreSQL 이 필요해 여기서는 보지 않는다 (다른 dataset 테스트와 같은 선택).
"""

from __future__ import annotations

import pytest

from rdfp.dataset.inspect.db_inspect import (
    MESSAGE_TABLES, EpisodeSummary, StreamInfo, _order_for, _resolve_mp4, episode_checks,
    recording_files_for, render_cell)
from rdfp.dataset.inspect.findings import LEVEL_INFO, LEVEL_WARNING


def _summary(**kw) -> EpisodeSummary:
    base = dict(id=7, start_ns=1_000, stop_ns=11_000_000_000, task_label='pick', success=True,
                metadata={'backend': 'panda_isaac'})
    base.update(kw)
    return EpisodeSummary(**base)


def _stream(frames_in_db=61, frame_count=61, resolved='/data/x.mp4') -> StreamInfo:
    return StreamInfo(topic='/cam', mp4_path='x.mp4', frame_count=frame_count,
                      frames_in_db=frames_in_db, resolved_path=resolved)


def test_message_tables_exclude_system_and_media() -> None:
    assert 'sessions' not in MESSAGE_TABLES
    assert 'topics' not in MESSAGE_TABLES
    assert 'image_frames' not in MESSAGE_TABLES
    assert 'joint_states' in MESSAGE_TABLES


def test_summary_properties() -> None:
    s = _summary(row_counts={'joint_states': 10, 'gripper_cmds': 2, 'image_frames': 61})
    assert s.duration_sec == pytest.approx(11.0, abs=1e-6)
    assert s.backend == 'panda_isaac'
    assert s.message_rows == 12
    assert s.frames == 61


def test_healthy_episode_has_no_warnings() -> None:
    s = _summary(row_counts={'joint_states': 10, 'gripper_cmds': 2, 'image_frames': 61},
                 streams=[_stream()])
    assert [f for f in episode_checks(s) if f.level == LEVEL_WARNING] == []


def test_empty_episode_is_the_clock_overlap_signature() -> None:
    s = _summary(row_counts={'image_frames': 61}, streams=[_stream()])
    codes = {f.code for f in episode_checks(s)}
    assert 'no_message_rows' in codes
    assert 'gripper_cmds_zero' not in codes      # 전부 0 일 때 그리퍼만 따로 말하지 않는다


def test_frames_missing_is_the_pixel_format_signature() -> None:
    s = _summary(row_counts={'joint_states': 10, 'gripper_cmds': 1},
                 streams=[_stream(frames_in_db=0)])
    codes = {f.code for f in episode_checks(s)}
    assert 'frames_missing' in codes
    assert 'frame_count_mismatch' not in codes   # frames_missing 이 이미 말했다


def test_gripper_zero_and_unpaired_motion() -> None:
    s = _summary(row_counts={'joint_states': 10, 'gripper_cmds': 0, 'image_frames': 3},
                 streams=[_stream(frames_in_db=3, frame_count=3)], unpaired_trajectories=1)
    codes = {f.code for f in episode_checks(s)}
    assert {'gripper_cmds_zero', 'motion_unpaired'} <= codes


def test_missing_file_and_no_video() -> None:
    s = _summary(row_counts={'joint_states': 1, 'gripper_cmds': 1, 'image_frames': 5},
                 streams=[_stream(frames_in_db=5, frame_count=5, resolved=None)])
    assert 'mp4_file_missing' in {f.code for f in episode_checks(s)}
    bare = _summary(row_counts={'joint_states': 1, 'gripper_cmds': 1})
    info = [f for f in episode_checks(bare) if f.level == LEVEL_INFO]
    assert [f.code for f in info] == ['no_video']


def test_resolve_mp4_tries_roots_in_order(tmp_path) -> None:
    (tmp_path / 'a').mkdir()
    (tmp_path / 'b').mkdir()
    (tmp_path / 'b' / 'x.mp4').write_bytes(b'')
    assert _resolve_mp4('x.mp4', [str(tmp_path / 'a'), str(tmp_path / 'b')]) == str(
        tmp_path / 'b' / 'x.mp4')
    assert _resolve_mp4('y.mp4', [str(tmp_path / 'a')]) is None
    absolute = tmp_path / 'b' / 'x.mp4'
    assert _resolve_mp4(str(absolute), []) == str(absolute)


def test_recording_files_include_sidecar_and_metadata(tmp_path) -> None:
    rec = tmp_path / 'recordings'
    rec.mkdir()
    for name in ('s.mp4', 's.jsonl', 's_metadata.json', 'other.mp4'):
        (rec / name).write_bytes(b'')
    files = recording_files_for('s.mp4', [str(rec)])
    assert sorted(f.name for f in files) == ['s.jsonl', 's.mp4', 's_metadata.json']


def test_recording_files_for_bag_encoded_episode_dir(tmp_path) -> None:
    videos = tmp_path / 'videos'
    (videos / 'episode_00000042').mkdir(parents=True)
    (videos / 'episode_00000042' / 'cam.mp4').write_bytes(b'')
    files = recording_files_for('episode_00000042/cam.mp4', [str(videos)])
    assert files == [videos / 'episode_00000042']


def test_render_cell_handles_arrays_and_truncates() -> None:
    assert render_cell(None) == ''
    assert render_cell([1.5, 2.0]) == '[1.5, 2.0]'
    assert render_cell({'a': 1}) == '{"a": 1}'
    long = render_cell('x' * 500, max_chars=10)
    assert long.endswith('…') and len(long) == 10


def test_order_for_prefers_stamp_columns() -> None:
    assert 'stamp_sec' in _order_for('joint_states').as_string(None)
    assert 'start_sec' in _order_for('sessions').as_string(None)
    assert _order_for('topics').as_string(None) == 'id'
