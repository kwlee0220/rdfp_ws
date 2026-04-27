"""``stats`` 독립 CLI 단위 테스트 (argparse + main 흐름)."""

from __future__ import annotations

import pytest

import rdfp.dataset.stats_cmd as stats_cmd_mod


def _parser():
    return stats_cmd_mod._build_parser()


# ---- argparse ----------------------------------------------------------

def test_stats_parser_defaults() -> None:
    ns = _parser().parse_args([])
    assert ns.config is None
    assert ns.format == 'text'


def test_stats_parser_accepts_config_and_json() -> None:
    ns = _parser().parse_args(['--config', 'x.yaml', '--format', 'json'])
    assert ns.config == 'x.yaml'
    assert ns.format == 'json'


# ---- main flow ---------------------------------------------------------

def test_main_without_config_and_no_cwd_file_returns_2(monkeypatch, tmp_path,
                                                       caplog) -> None:
    """cwd 에 dataset_config.yaml 이 없으면 --config 없이 실행 시 exit 2."""
    monkeypatch.chdir(tmp_path)
    with caplog.at_level('ERROR'):
        rc = stats_cmd_mod.main([])
    assert rc == 2
