"""dataset.config 모듈 단위 테스트."""

from __future__ import annotations

import os

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


def test_path_fields_expand_env_and_user(monkeypatch, tmp_path) -> None:
    monkeypatch.setenv('RDFP_TEST_ROOT', '/data/rdfp')
    monkeypatch.setenv('HOME', str(tmp_path))
    cfg = DatasetConfig.model_validate({
        'rosbag_dir': '${RDFP_TEST_ROOT}/rosbag',
        'output_mp4_dir': '~/videos',
    })
    assert cfg.rosbag_dir == '/data/rdfp/rosbag'
    assert cfg.output_mp4_dir == os.path.join(str(tmp_path), 'videos')


def test_path_fields_fallback_when_unset(monkeypatch, tmp_path) -> None:
    monkeypatch.delenv('RDFP_TEST_UNSET', raising=False)
    monkeypatch.setenv('HOME', str(tmp_path))
    cfg = DatasetConfig.model_validate({
        'rosbag_dir': '${RDFP_TEST_UNSET:-/fallback/rosbag}',
        'output_mp4_dir': '${RDFP_TEST_UNSET:-~/videos}',
    })
    assert cfg.rosbag_dir == '/fallback/rosbag'
    assert cfg.output_mp4_dir == os.path.join(str(tmp_path), 'videos')


def test_path_fields_fallback_skipped_when_set(monkeypatch) -> None:
    monkeypatch.setenv('RDFP_TEST_SET', '/from/env')
    cfg = DatasetConfig.model_validate({
        'rosbag_dir': '${RDFP_TEST_SET:-/should/not/use}',
    })
    assert cfg.rosbag_dir == '/from/env'


def test_path_fields_no_expansion_when_plain(tmp_path) -> None:
    cfg = DatasetConfig.model_validate({
        'rosbag_dir': '/tmp/rosbag',
        'output_mp4_dir': None,
    })
    assert cfg.rosbag_dir == '/tmp/rosbag'
    assert cfg.output_mp4_dir is None


def test_load_dataset_config_roundtrip(tmp_path) -> None:
    cfg_path = tmp_path / 'dataset_config.yaml'
    cfg_path.write_text(
        'rosbag_dir: /tmp/rosbag\n'
        'output_mp4_dir: /tmp/out\n'
        'topics: [/ee_pose_publisher/ee_pose]\n'
    )
    cfg = load_dataset_config(cfg_path)
    assert '/ee_pose_publisher/ee_pose' in cfg.effective_topics()


def test_topics_file_relative_to_yaml(tmp_path) -> None:
    (tmp_path / 'topics.list').write_text(
        '# header comment\n'
        '\n'
        '/session\n'
        '/joint_states\n'
        '   # indented comment\n'
        '/camera/image_raw   \n'
    )
    cfg_path = tmp_path / 'dataset_config.yaml'
    cfg_path.write_text(
        'rosbag_dir: /tmp/rosbag\n'
        'topics_file: topics.list\n'
    )
    cfg = load_dataset_config(cfg_path)
    assert cfg.topics == ['/session', '/joint_states', '/camera/image_raw']


def test_topics_file_merges_with_inline_topics(tmp_path) -> None:
    (tmp_path / 'topics.list').write_text('/session\n/joint_states\n')
    cfg_path = tmp_path / 'dataset_config.yaml'
    cfg_path.write_text(
        'rosbag_dir: /tmp/rosbag\n'
        'topics_file: topics.list\n'
        'topics: [/joint_states, /ee_pose]\n'  # /joint_states 중복은 1회만 등장
    )
    cfg = load_dataset_config(cfg_path)
    assert cfg.topics == ['/session', '/joint_states', '/ee_pose']


def test_topics_file_missing_raises(tmp_path) -> None:
    cfg_path = tmp_path / 'dataset_config.yaml'
    cfg_path.write_text(
        'rosbag_dir: /tmp/rosbag\n'
        'topics_file: does_not_exist.list\n'
    )
    with pytest.raises(FileNotFoundError):
        load_dataset_config(cfg_path)


def test_topics_file_env_expansion(monkeypatch, tmp_path) -> None:
    topics_path = tmp_path / 'topics.list'
    topics_path.write_text('/session\n')
    monkeypatch.setenv('RDFP_TEST_TOPICS_DIR', str(tmp_path))
    cfg_path = tmp_path / 'dataset_config.yaml'
    cfg_path.write_text(
        'rosbag_dir: /tmp/rosbag\n'
        'topics_file: ${RDFP_TEST_TOPICS_DIR}/topics.list\n'
    )
    cfg = load_dataset_config(cfg_path)
    assert cfg.topics == ['/session']


