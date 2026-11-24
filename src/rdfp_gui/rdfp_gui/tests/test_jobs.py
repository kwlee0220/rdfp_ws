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


# ----- 통합 (2026-11-23) — 매핑 · 토픽 교체 · LeRobot · 영상 복사 ----------------------------

def test_import_topic_replace_needs_topics() -> None:
    spec = build_import_command('c.yaml', topics=['/joint_states'], only_topics_replace=True,
                                prefer_ros2=True)
    assert spec.argv[-3:] == ['--topics', '/joint_states', '--only-topics-replace'][-3:]
    assert '--only-topics-replace' in spec.argv
    with pytest.raises(ValueError):
        build_import_command('c.yaml', only_topics_replace=True, prefer_ros2=True)


def test_mapping_commands_carry_their_confirmations() -> None:
    from rdfp_gui.jobs import build_mapping_command

    reg = build_mapping_command('c.yaml', 'register', 'a/msg/B', table='bs', prefer_ros2=True)
    assert reg.argv == ['ros2', 'run', 'rdfp', 'mapping', 'register', '--config', 'c.yaml',
                        '--type', 'a/msg/B', '--table', 'bs']
    unreg = build_mapping_command('c.yaml', 'unregister', 'a/msg/B', prefer_ros2=True)
    assert unreg.argv[-1] == '--yes'
    drop = build_mapping_command('c.yaml', 'drop', 'a/msg/B', confirm_table='bs',
                                 prefer_ros2=True)
    assert drop.argv[-3:] == ['--yes', '--confirm', 'bs']
    with pytest.raises(ValueError):
        build_mapping_command('c.yaml', 'drop', 'a/msg/B', prefer_ros2=True)
    with pytest.raises(ValueError):
        build_mapping_command('c.yaml', 'nuke', 'a/msg/B', prefer_ros2=True)


def test_export_copy_media_flag() -> None:
    spec = build_export_command('c.yaml', '/out', episode_ids=[1], copy_media=True,
                                prefer_ros2=True)
    assert spec.argv[-1] == '--copy-media'
    assert spec.env == {}


def test_lerobot_export_runs_in_the_lerobot_python(monkeypatch) -> None:
    from rdfp_gui.jobs import LEROBOT_PYTHON_ENV

    monkeypatch.delenv(LEROBOT_PYTHON_ENV, raising=False)
    with pytest.raises(ValueError, match=LEROBOT_PYTHON_ENV):
        build_export_command('c.yaml', '/out', episode_ids=[1], fmt='lerobot_v3')
    monkeypatch.setenv(LEROBOT_PYTHON_ENV, 'uv run --no-project --python /v/bin/python python')
    spec = build_export_command('c.yaml', '/out', episode_ids=[1, 2], fmt='lerobot_v3', fps=5,
                                max_skew_sec=0.5, copy_media=True)
    assert spec.argv[:7] == ['uv', 'run', '--no-project', '--python', '/v/bin/python',
                             'python', '-m']
    assert spec.argv[7] == 'rdfp.dataset.export_cmd'
    assert spec.argv[-6:] == ['--format', 'lerobot_v3', '--fps', '5', '--max-skew-sec', '0.5']
    assert '--copy-media' not in spec.argv            # LeRobot 은 언제나 영상을 품는다
    assert spec.env['PYTHONPATH'].endswith('src/rdfp') or 'rdfp' in spec.env['PYTHONPATH']
    assert spec.command_line().startswith('PYTHONPATH=')
