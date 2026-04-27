"""``init-db`` 독립 CLI 단위 테스트 (argparse + main 흐름)."""

from __future__ import annotations

import pytest

import rdfp.dataset.init_db_cmd as init_db_cmd_mod


def _parser():
    return init_db_cmd_mod._build_parser()


# ---- argparse ----------------------------------------------------------

def test_parser_with_dsn_env_arg() -> None:
    ns = _parser().parse_args(['--dsn-env', 'MY_DSN', '--schema', 'public'])
    assert ns.dsn_env == 'MY_DSN'
    assert ns.schema == 'public'
    assert ns.config is None
    assert ns.drop is False


def test_parser_with_config() -> None:
    ns = _parser().parse_args(['--config', 'dataset_config.yaml'])
    assert ns.config == 'dataset_config.yaml'
    # 기본값 유지.
    assert ns.dsn_env == 'RDFP_DB_DSN'
    assert ns.schema == 'public'


# ---- main flow ---------------------------------------------------------

def test_drop_requires_yes_in_non_tty(monkeypatch, tmp_path) -> None:
    """``--drop`` 단독 (--yes 없음, non-tty) 은 거부되어 exit 2."""
    monkeypatch.chdir(tmp_path)   # cwd 의 dataset_config.yaml 격리.
    called: list = []
    monkeypatch.setenv('CI', '1')   # non-interactive.
    monkeypatch.setenv('FAKE_DSN', 'postgresql://x')
    monkeypatch.setattr(
        init_db_cmd_mod, 'initialize_schema',
        lambda *a, **kw: called.append((a, kw)))
    rc = init_db_cmd_mod.main(['--dsn-env', 'FAKE_DSN', '--drop'])
    assert rc == 2
    assert called == []   # initialize_schema 는 호출되지 않음.


def test_drop_with_yes_calls_init(monkeypatch, tmp_path) -> None:
    monkeypatch.chdir(tmp_path)
    called: list = []
    monkeypatch.setenv('FAKE_DSN', 'postgresql://x')
    monkeypatch.setattr(
        init_db_cmd_mod, 'initialize_schema',
        lambda *a, **kw: called.append(kw))
    rc = init_db_cmd_mod.main(['--dsn-env', 'FAKE_DSN', '--drop', '--yes'])
    assert rc == 0
    assert len(called) == 1
    assert called[0]['drop_first'] is True


def test_missing_env_var_returns_2(monkeypatch, tmp_path, caplog) -> None:
    monkeypatch.chdir(tmp_path)
    monkeypatch.delenv('FAKE_DSN', raising=False)
    with caplog.at_level('ERROR'):
        rc = init_db_cmd_mod.main(['--dsn-env', 'FAKE_DSN'])
    assert rc == 2


def test_uses_config_db_section(monkeypatch, tmp_path) -> None:
    cfg_path = tmp_path / 'c.yaml'
    cfg_path.write_text(
        'rosbag_dir: /tmp/x\n'
        'output_mp4_dir: /tmp/y\n'
        'db: {dsn_env: CUSTOM_DSN, schema: custom}\n'
    )
    monkeypatch.setenv('CUSTOM_DSN', 'postgresql://example')
    captured: list = []
    monkeypatch.setattr(
        init_db_cmd_mod, 'initialize_schema',
        lambda dsn, schema, drop_first: captured.append((dsn, schema, drop_first)))
    rc = init_db_cmd_mod.main(['--config', str(cfg_path)])
    assert rc == 0
    assert captured == [('postgresql://example', 'custom', False)]


def test_auto_uses_cwd_dataset_config(monkeypatch, tmp_path) -> None:
    """--config 미지정 + cwd 의 dataset_config.yaml 존재 → 자동 로드."""
    cfg_path = tmp_path / 'dataset_config.yaml'
    cfg_path.write_text(
        'rosbag_dir: /tmp/x\n'
        'output_mp4_dir: /tmp/y\n'
        'db: {dsn_env: AUTO_DSN, schema: auto_schema}\n'
    )
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv('AUTO_DSN', 'postgresql://auto')
    captured: list = []
    monkeypatch.setattr(
        init_db_cmd_mod, 'initialize_schema',
        lambda dsn, schema, drop_first: captured.append((dsn, schema, drop_first)))
    rc = init_db_cmd_mod.main([])
    assert rc == 0
    assert captured == [('postgresql://auto', 'auto_schema', False)]
