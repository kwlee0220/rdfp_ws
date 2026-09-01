"""``init-db`` 독립 CLI 단위 테스트 (argparse + main 흐름)."""

from __future__ import annotations

import rdfp.dataset.cli_common as cli_common
import rdfp.dataset.init_db_cmd as init_db_cmd_mod


def _isolate_default_config(monkeypatch, tmp_path) -> None:
    """기본 설정 파일 탐색이 저장소의 실제 파일을 집지 않게 한다.

    `resolve_config_path` 는 `cli_common` 의 모듈 전역을 읽으므로 거기를 바꾼다.
    """
    from rdfp.dataset import cli_common

    monkeypatch.setattr(cli_common, 'DEFAULT_CONFIG_FILE_PATH',
                        tmp_path / 'config' / 'dataset_config.yaml')


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
    monkeypatch.chdir(tmp_path)
    # **chdir 만으로는 부족하다** — 기본 경로는 RDFP_HOME 기준 모듈 상수다.
    _isolate_default_config(monkeypatch, tmp_path)
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
    """`--drop --yes` 는 스키마 초기화를 호출하고 exit 0.

    기본 설정 파일을 차단해야 `--dsn-env` 가 실제로 쓰인다 — 안 그러면 저장소의
    `config/dataset_config.yaml` 이 잡혀 `RDFP_DB_DSN` 을 보게 되고, **그 변수가
    설정된 환경에서만 우연히 통과**한다.
    """
    monkeypatch.chdir(tmp_path)
    _isolate_default_config(monkeypatch, tmp_path)
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
    """`--dsn-env` 가 가리키는 환경변수가 없으면 exit 2.

    **기본 설정 파일을 함께 차단해야 이 경로가 실제로 돈다.** `cmd_init_db` 는
    설정 파일을 찾으면 `--dsn-env` 를 무시하고 `cfg.db.dsn_env` 를 쓴다. 이
    저장소에는 `config/dataset_config.yaml` 이 있고 `RDFP_HOME` 도 설정돼 있어
    `chdir` 만으로는 막을 수 없다 — 예전에는 그 설정의 DSN 으로 접속이 실패해서
    우연히 2가 나왔고, DB 가 살아나자 0을 반환하며 드러났다.
    """
    monkeypatch.chdir(tmp_path)
    _isolate_default_config(monkeypatch, tmp_path)
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


def test_auto_uses_default_config(monkeypatch, tmp_path) -> None:
    """--config 미지정 + 기본 경로의 dataset_config.yaml 존재 → 자동 로드."""
    cfg_path = tmp_path / 'dataset_config.yaml'
    cfg_path.write_text(
        'rosbag_dir: /tmp/x\n'
        'output_mp4_dir: /tmp/y\n'
        'db: {dsn_env: AUTO_DSN, schema: auto_schema}\n'
    )
    # 이 테스트는 반대로 **기본 경로가 발견되는** 상황을 만든다.
    monkeypatch.setattr(cli_common, 'DEFAULT_CONFIG_FILE_PATH', cfg_path)
    monkeypatch.setenv('AUTO_DSN', 'postgresql://auto')
    captured: list = []
    monkeypatch.setattr(
        init_db_cmd_mod, 'initialize_schema',
        lambda dsn, schema, drop_first: captured.append((dsn, schema, drop_first)))
    rc = init_db_cmd_mod.main([])
    assert rc == 0
    assert captured == [('postgresql://auto', 'auto_schema', False)]
