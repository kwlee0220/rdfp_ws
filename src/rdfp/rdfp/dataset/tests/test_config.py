"""dataset.config 모듈 단위 테스트."""

from __future__ import annotations

import pytest

from rdfp.dataset.config import DatasetConfig, SESSION_TOPIC, load_dataset_config


def _base_dict() -> dict:
    return {
        'rosbag_dir': '/tmp/rosbag',
        'output_mp4_dir': '/tmp/out',
    }


def test_load_minimal_config() -> None:
    cfg = DatasetConfig.model_validate(_base_dict())
    assert cfg.rosbag_dir == '/tmp/rosbag'
    assert cfg.db.dsn_env == 'RDFP_DB_DSN'
    assert cfg.db.schema_ == 'public'
    assert cfg.on_existing_episode == 'skip'
    assert cfg.delete_splits_after_import is False


def test_delete_splits_flag_parsed() -> None:
    d = _base_dict()
    d['delete_splits_after_import'] = True
    cfg = DatasetConfig.model_validate(d)
    assert cfg.delete_splits_after_import is True


def test_effective_topics_auto_includes_session() -> None:
    d = _base_dict()
    d['topics'] = ['/ee_pose_publisher/ee_pose', '/joint_states']
    cfg = DatasetConfig.model_validate(d)
    assert SESSION_TOPIC in cfg.effective_topics()
    assert cfg.effective_topics()[0] == SESSION_TOPIC


def test_effective_topics_empty_stays_empty() -> None:
    cfg = DatasetConfig.model_validate(_base_dict())
    assert cfg.effective_topics() == []


def test_effective_topics_no_duplicate() -> None:
    d = _base_dict()
    d['topics'] = ['/session', '/ee_pose_publisher/ee_pose']
    cfg = DatasetConfig.model_validate(d)
    assert cfg.effective_topics().count('/session') == 1


def test_session_filter_date_format() -> None:
    d = _base_dict()
    d['session_filter'] = {'dates': ['2026/04/11']}
    with pytest.raises(Exception):
        DatasetConfig.model_validate(d)


def test_load_dataset_config_missing_file(tmp_path) -> None:
    with pytest.raises(FileNotFoundError):
        load_dataset_config(tmp_path / 'nope.yaml')


def test_load_dataset_config_roundtrip(tmp_path) -> None:
    cfg_path = tmp_path / 'dataset_config.yaml'
    cfg_path.write_text(
        'rosbag_dir: /tmp/rosbag\n'
        'output_mp4_dir: /tmp/out\n'
        'topics: [/ee_pose_publisher/ee_pose]\n'
    )
    cfg = load_dataset_config(cfg_path)
    assert '/ee_pose_publisher/ee_pose' in cfg.effective_topics()


