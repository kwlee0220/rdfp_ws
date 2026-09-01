"""``list`` 독립 CLI 단위 테스트 (argparse + main 흐름)."""

from __future__ import annotations


import rdfp.dataset.list_cmd as list_cmd_mod


def _isolate_default_config(monkeypatch, tmp_path) -> None:
    """기본 설정 파일 탐색이 저장소의 실제 파일을 집지 않게 한다.

    `resolve_config_path` 는 `cli_common` 의 모듈 전역을 읽으므로 거기를 바꾼다.
    """
    from rdfp.dataset import cli_common

    monkeypatch.setattr(cli_common, 'DEFAULT_CONFIG_FILE_PATH',
                        tmp_path / 'config' / 'dataset_config.yaml')


def _parser():
    return list_cmd_mod._build_parser()


# ---- argparse ----------------------------------------------------------

def test_list_parser_defaults() -> None:
    ns = _parser().parse_args([])
    assert ns.config is None
    assert ns.format == 'text'


def test_list_parser_accepts_config() -> None:
    ns = _parser().parse_args(['--config', 'x.yaml'])
    assert ns.config == 'x.yaml'


def test_list_parser_accepts_json_format() -> None:
    ns = _parser().parse_args(['--config', 'x.yaml', '--format', 'json'])
    assert ns.format == 'json'


# ---- main flow ---------------------------------------------------------

def test_main_without_config_and_no_cwd_file_returns_2(monkeypatch, tmp_path,
                                                       caplog) -> None:
    """`--config` 없이 실행했을 때, 기본 설정 파일도 없으면 exit 2.

    **`chdir` 만으로는 이 상황을 만들 수 없다.** `DEFAULT_CONFIG_FILE_PATH` 는
    `RDFP_HOME` 을 보고 **import 시점에 한 번** 정해지는 모듈 상수라 작업 디렉터리를
    옮겨도 그대로다. 이 저장소에는 `config/dataset_config.yaml` 이 실제로 있고
    `RDFP_HOME` 도 설정돼 있어, 예전에는 그 파일이 발견된 뒤 **DB 접속이 실패해서**
    우연히 2가 나왔다. DB 가 살아나자 0을 반환하며 드러났다.

    그래서 상수 자체를 존재하지 않는 경로로 돌린다.
    """
    monkeypatch.chdir(tmp_path)
    _isolate_default_config(monkeypatch, tmp_path)
    with caplog.at_level('ERROR'):
        rc = list_cmd_mod.main([])
    assert rc == 2
