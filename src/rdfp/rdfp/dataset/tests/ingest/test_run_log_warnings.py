"""run_log.quality_warning 기록 검증."""

from __future__ import annotations

import json

from rdfp.dataset.ingest.run_log import RunLog


def test_quality_warning_record(tmp_path) -> None:
    log = RunLog(tmp_path / 'run.jsonl')
    log.quality_warning(
        episode_id=3, kind='stamp_regression',
        topic='/ee_pose_publisher/ee_pose', stamp_ns=1000, prev_stamp_ns=2000,
    )
    [rec] = [json.loads(ln) for ln in (tmp_path / 'run.jsonl').read_text().splitlines()]
    assert rec['event'] == 'quality_warning'
    assert rec['kind'] == 'stamp_regression'
    assert rec['topic'] == '/ee_pose_publisher/ee_pose'
    assert rec['stamp_ns'] == 1000
    assert rec['prev_stamp_ns'] == 2000
    assert rec['episode_id'] == 3


def test_quality_warning_episode_id_can_be_none(tmp_path) -> None:
    log = RunLog(tmp_path / 'run.jsonl')
    log.quality_warning(
        episode_id=None, kind='idle_gap',
        topic='/t', stamp_ns=5, prev_stamp_ns=1,
    )
    [rec] = [json.loads(ln) for ln in (tmp_path / 'run.jsonl').read_text().splitlines()]
    assert rec['episode_id'] is None
