"""import · export 사전 점검 — DB 없이 도는 부분."""

from __future__ import annotations

import pytest

from rdfp.dataset.config import DatasetConfig
from rdfp.dataset.inspect.findings import LEVEL_ERROR, LEVEL_INFO, LEVEL_WARNING, has_errors, only
from rdfp.dataset.inspect.preflight import check_import, check_topics_file, overlapping_windows

TYPES = {
    '/session': 'rdfp_msgs/msg/SessionCommand',
    '/joint_states': 'sensor_msgs/msg/JointState',
    '/gripper_cmds': 'rdfp_msgs/msg/GripperCommand',
    '/isaac/camera/image_raw': 'sensor_msgs/msg/Image',
    '/camera_image/compressed': 'sensor_msgs/msg/CompressedImage',
}
WINDOWS = [(100, 200), (300, 400)]


def _cfg(**kw) -> DatasetConfig:
    base = {'rosbag_dir': '/tmp/bags', 'output_mp4_dir': '/tmp/videos'}
    base.update(kw)
    return DatasetConfig.model_validate(base)


def _codes(findings) -> set:
    return {f.code for f in findings}


def test_overlapping_windows_finds_pairs_regardless_of_order() -> None:
    assert overlapping_windows([(0, 10), (20, 30)]) == []
    assert overlapping_windows([(20, 30), (0, 25)]) == [(1, 0)]
    # 셋이 다 겹치면 세 쌍이다.
    assert len(overlapping_windows([(0, 100), (10, 20), (15, 50)])) == 3


def test_image_topic_with_recordings_dir_is_blocked() -> None:
    """image_streams UNIQUE 위반으로 죽는 조합 — 실행 전에 막는다."""
    findings = check_import(_cfg(recordings_dir='/tmp/rec'), topic_types=TYPES,
                            episode_windows=WINDOWS)
    assert 'image_conflict' in _codes(findings)
    assert has_errors(findings)


def test_image_topic_without_recordings_dir_is_fine() -> None:
    findings = check_import(_cfg(), topic_types=TYPES, episode_windows=WINDOWS)
    assert 'image_conflict' not in _codes(findings)
    assert not has_errors(findings)


def test_deselecting_the_image_topic_removes_the_conflict() -> None:
    chosen = [t for t in TYPES if t != '/isaac/camera/image_raw']
    findings = check_import(_cfg(recordings_dir='/tmp/rec'), topic_types=TYPES,
                            selected_topics=chosen, episode_windows=WINDOWS)
    codes = _codes(findings)
    assert 'image_conflict' not in codes
    assert 'partial_topics' in codes          # 일부만 골랐다는 경고는 남는다


def test_unmapped_topics_are_a_warning_not_an_error() -> None:
    findings = check_import(_cfg(), topic_types=TYPES, episode_windows=WINDOWS)
    unmapped = [f for f in findings if f.code == 'unmapped_topics']
    assert len(unmapped) == 1
    assert unmapped[0].level == LEVEL_WARNING
    assert '/camera_image/compressed' in unmapped[0].message


def test_missing_output_dir_is_an_error_with_the_cli_message() -> None:
    cfg = DatasetConfig.model_validate({'rosbag_dir': '/tmp/bags'})
    findings = check_import(cfg, topic_types=TYPES, episode_windows=WINDOWS)
    err = [f for f in findings if f.code == 'output_mp4_dir']
    assert err and err[0].level == LEVEL_ERROR
    assert err[0].message == 'output_mp4_dir is required for import; add it to the dataset config'


def test_no_episodes_is_an_error_but_unknown_is_not() -> None:
    assert 'no_episodes' in _codes(check_import(_cfg(), topic_types=TYPES, episode_windows=()))
    assert 'no_episodes' not in _codes(check_import(_cfg(), topic_types=TYPES,
                                                    episode_windows=None))


def test_overlapping_episodes_warn() -> None:
    findings = check_import(_cfg(), topic_types=TYPES, episode_windows=[(0, 100), (50, 150)])
    warn = [f for f in findings if f.code == 'overlapping_episodes']
    assert warn and warn[0].level == LEVEL_WARNING
    assert 'sim 시계' in warn[0].message


@pytest.mark.parametrize('policy, code, level', [
    ('skip', 'existing_skip', LEVEL_INFO),
    ('replace', 'existing_replace', LEVEL_WARNING),
    ('error', 'existing_error', LEVEL_ERROR),
])
def test_existing_episodes_follow_the_policy(policy, code, level) -> None:
    findings = check_import(_cfg(), topic_types=TYPES, episode_windows=WINDOWS,
                            existing_keys={100}, policy=policy)
    hit = [f for f in findings if f.code == code]
    assert hit and hit[0].level == level
    assert '1개' in hit[0].message


def test_existing_check_is_skipped_when_unknown() -> None:
    findings = check_import(_cfg(), topic_types=TYPES, episode_windows=WINDOWS,
                            existing_keys=None)
    assert not any(f.code.startswith('existing_') for f in findings)


# ----- topics_file ------------------------------------------------------------

def test_topics_file_same_as_reference(tmp_path) -> None:
    ref = tmp_path / 'config' / 'recording_topics.list'
    ref.parent.mkdir()
    ref.write_text('/session\n/joint_states\n')
    cfg = tmp_path / 'config' / 'dataset_config.yaml'
    cfg.write_text('rosbag_dir: /x\ntopics_file: recording_topics.list\n')

    findings = check_topics_file(cfg, reference=ref)
    assert _codes(findings) == {'topics_file_same'}


def test_topics_file_differs_from_reference(tmp_path) -> None:
    ref = tmp_path / 'recording_topics.list'
    ref.write_text('/session\n')
    other = tmp_path / 'other.list'
    other.write_text('/session\n')
    cfg = tmp_path / 'dataset_config.yaml'
    cfg.write_text(f'rosbag_dir: /x\ntopics_file: {other}\n')

    findings = check_topics_file(cfg, reference=ref)
    assert only(findings, LEVEL_WARNING)[0].code == 'topics_file_differs'


def test_topics_file_missing_and_absent(tmp_path) -> None:
    cfg = tmp_path / 'dataset_config.yaml'
    cfg.write_text('rosbag_dir: /x\ntopics_file: nope.list\n')
    assert _codes(check_topics_file(cfg)) == {'topics_file_missing'}
    cfg.write_text('rosbag_dir: /x\n')
    assert _codes(check_topics_file(cfg)) == {'topics_all'}
