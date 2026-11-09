"""CLI 명령 조립 · 진행률 파싱 — GUI 조작이 CLI 와 같은 명령이 되는지 (설계안 §6.4)."""

from __future__ import annotations

import sys

import pytest

from rdfp_gui.jobs import (ImportProgress, build_episode_delete_command, build_export_command,
                           build_import_command, build_reindex_command, episodes_spec,
                           parse_summary_json, rdfp_command)


def test_episodes_spec_compresses_runs() -> None:
    assert episodes_spec([]) == ''
    assert episodes_spec([7]) == '7'
    assert episodes_spec([3, 1, 2, 7, 9, 10]) == '1-3,7,9-10'
    assert episodes_spec([5, 5, 6]) == '5-6'


def test_rdfp_command_prefers_ros2_and_falls_back_to_python() -> None:
    assert rdfp_command('import', prefer_ros2=True) == ['ros2', 'run', 'rdfp', 'import']
    assert rdfp_command('export', prefer_ros2=False) == [
        sys.executable, '-m', 'rdfp.dataset.export_cmd']
    with pytest.raises(KeyError):
        rdfp_command('nope', prefer_ros2=False)


def test_import_command_is_the_plain_cli_when_nothing_is_selected() -> None:
    spec = build_import_command('/w/config/dataset_config.yaml', prefer_ros2=True)
    assert spec.argv == ['ros2', 'run', 'rdfp', 'import', '--config',
                         '/w/config/dataset_config.yaml']
    assert spec.command_line() == 'ros2 run rdfp import --config /w/config/dataset_config.yaml'
    assert spec.total_hint is None


def test_import_command_carries_selection_and_policy() -> None:
    spec = build_import_command('c.yaml', start_ns=[10, 20], topics=['/joint_states', '/ee_pose'],
                                on_existing='replace', prefer_ros2=True)
    assert spec.argv[4:] == ['--config', 'c.yaml', '--start-ns', '10', '--start-ns', '20',
                             '--topics', '/joint_states', '/ee_pose', '--on-existing', 'replace']
    assert spec.total_hint == 2


def test_export_command() -> None:
    spec = build_export_command('c.yaml', '/out', episode_ids=[1, 2, 3, 9], label='joint_command',
                                chunk_len=10, delta_sec=0.05, step_sec=0.1, camera_topic='/cam',
                                keep_untrustworthy=True, prefer_ros2=True)
    assert spec.argv == ['ros2', 'run', 'rdfp', 'export', '--config', 'c.yaml', '--out', '/out',
                         '--episodes', '1-3,9', '--label', 'joint_command', '--chunk-len', '10',
                         '--delta-sec', '0.05', '--step-sec', '0.1', '--camera-topic', '/cam',
                         '--keep-untrustworthy']


def test_delete_and_reindex_commands() -> None:
    spec = build_episode_delete_command('c.yaml', [12, 13], delete_files=True, prefer_ros2=True)
    assert spec.argv[4:] == ['--config', 'c.yaml', '--ids', '12-13', '--yes', '--delete-files']
    assert build_reindex_command('/bags/2026-09-20/session_x').argv == [
        'ros2', 'bag', 'reindex', '-s', 'mcap', '/bags/2026-09-20/session_x']


def test_import_progress_reads_the_pipeline_log() -> None:
    p = ImportProgress()
    assert p.feed('2026-11-04 [INFO] rdfp.dataset.import_cmd: found 1 finalized split(s)') is False
    assert p.feed('... detected 65 episode(s)') is True
    assert p.total == 65
    assert p.feed('... 3 episode(s) remain after filters') is True
    assert p.total == 3
    assert p.feed("... episode committed: id=83 rows={'joint_states': 1520} mp4=1") is True
    assert p.feed('... episode already present (start_ns=1, id=2); skipped') is True
    assert p.feed('... insert failed for topic=/x (type=y) in episode 2') is True
    assert (p.committed, p.skipped, p.failed, p.done) == (1, 1, 1, 3)
    assert p.last_episode_id == 83
    assert p.status() == '3/3 에피소드 (skip 1, fail 1)'


def test_export_progress_lines() -> None:
    p = ImportProgress()
    assert p.feed('exporting 4 episode(s) as joint_plan') is True
    assert p.total == 4
    assert p.feed('episode 7: 120 sample(s)') is True
    assert p.done == 1


def test_parse_summary_json_takes_the_trailing_object() -> None:
    out = 'noise\n{\n  "episodes": 2,\n  "inserted": {"joint_states": 10}\n}\n'
    assert parse_summary_json(out) == {'episodes': 2, 'inserted': {'joint_states': 10}}
    assert parse_summary_json('plain text') is None
    assert parse_summary_json('') is None
