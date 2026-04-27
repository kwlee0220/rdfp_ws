"""``import`` 독립 CLI 단위 테스트 (argparse + main 흐름)."""

from __future__ import annotations

import pytest

import rdfp.dataset.import_cmd as import_cmd_mod


def _parser():
    return import_cmd_mod._build_parser()


# ---- argparse ----------------------------------------------------------

def test_import_parser_accepts_config() -> None:
    ns = _parser().parse_args(['--config', 'x.yaml'])
    assert ns.config == 'x.yaml'


def test_import_parser_config_optional() -> None:
    ns = _parser().parse_args([])
    assert ns.config is None


def test_import_parser_rejects_dry_run_flag() -> None:
    """제거된 ``--dry-run`` 플래그는 argparse 단계에서 거부된다."""
    with pytest.raises(SystemExit):
        _parser().parse_args(['--config', 'x.yaml', '--dry-run'])


# ---- main flow ---------------------------------------------------------

def test_main_auto_uses_cwd_dataset_config(monkeypatch, tmp_path) -> None:
    """--config 미지정 시 cwd 의 dataset_config.yaml 을 자동 사용한다.

    실제 파이프라인은 호출하지 않도록 ``import_cmd`` 의 ``discover_splits`` 만
    빈 리스트로 stub — cmd_import 가 ``_empty_summary()`` 로 즉시 종료한다.
    discover_splits 가 받는 ``cfg.rosbag_dir`` 로 config 가 cwd 에서 로드된
    것을 확인한다.
    """
    cfg_path = tmp_path / 'dataset_config.yaml'
    cfg_path.write_text(
        'rosbag_dir: /tmp/x\n'
        'output_mp4_dir: /tmp/y\n'
    )
    monkeypatch.chdir(tmp_path)
    called: list = []
    monkeypatch.setattr(
        import_cmd_mod, 'discover_splits',
        lambda rosbag_dir, dates=None, topics=None: (
            called.append(rosbag_dir) or []),
    )
    rc = import_cmd_mod.main([])
    assert rc == 0
    assert called == ['/tmp/x']


def test_main_without_config_and_no_cwd_file_returns_2(monkeypatch, tmp_path,
                                                       caplog) -> None:
    """cwd 에 dataset_config.yaml 이 없으면 --config 없이 실행 시 exit 2."""
    monkeypatch.chdir(tmp_path)
    with caplog.at_level('ERROR'):
        rc = import_cmd_mod.main([])
    assert rc == 2
