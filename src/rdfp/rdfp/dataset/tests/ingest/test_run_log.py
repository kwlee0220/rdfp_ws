"""run_log 단위 테스트."""

from __future__ import annotations

import json

from rdfp.dataset.ingest.run_log import RunLog


def _lines(path) -> list[dict]:
    return [json.loads(ln) for ln in path.read_text(encoding='utf-8').splitlines()]


def test_episode_done_record(tmp_path) -> None:
    log = RunLog(tmp_path / 'run.jsonl')
    log.episode_done(
        episode_id=7,
        start_ns=1_000_000_000, stop_ns=2_500_000_000,
        task_label='pick',
        row_counts={'pose_stampeds': 10, 'joint_states': 20},
        mp4_files=['episode_00000007/cam_image_raw.mp4'],
    )
    recs = _lines(tmp_path / 'run.jsonl')
    assert len(recs) == 1
    rec = recs[0]
    assert rec['event'] == 'episode_done'
    assert rec['episode_id'] == 7
    assert rec['task_label'] == 'pick'
    assert rec['row_counts'] == {'pose_stampeds': 10, 'joint_states': 20}
    assert rec['mp4_files'] == ['episode_00000007/cam_image_raw.mp4']


def test_episode_failed_record(tmp_path) -> None:
    log = RunLog(tmp_path / 'run.jsonl')
    log.episode_failed(
        start_ns=100, stop_ns=200, task_label='x', error='boom',
    )
    [rec] = _lines(tmp_path / 'run.jsonl')
    assert rec['event'] == 'episode_failed'
    assert rec['error'] == 'boom'


def test_appends_to_same_file(tmp_path) -> None:
    log = RunLog(tmp_path / 'run.jsonl')
    log.episode_skipped(episode_id=1, start_ns=0, stop_ns=1, task_label=None)
    log.episode_skipped(episode_id=2, start_ns=2, stop_ns=3, task_label=None)
    recs = _lines(tmp_path / 'run.jsonl')
    assert [r['episode_id'] for r in recs] == [1, 2]
