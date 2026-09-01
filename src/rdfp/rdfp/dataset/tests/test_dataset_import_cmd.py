"""``import`` 독립 CLI 단위 테스트 (argparse + main 흐름).

모듈 대부분은 ROS 없이 돈다. `cmd_import` 본문까지 들어가는 테스트만 예외로,
파이프라인 헬퍼가 lazy import 하는 `sensor_msgs` 가 필요해 개별 skip 한다.
"""

from __future__ import annotations

import importlib.util

import pytest

import rdfp.dataset.cli_common as cli_common
import rdfp.dataset.import_cmd as import_cmd_mod

# cmd_import 는 split 목록이 비어도 파이프라인 헬퍼를 먼저 import 하므로, stub 을
# 끼워도 ROS 의존이 남는다.
requires_ros = pytest.mark.skipif(
    importlib.util.find_spec('sensor_msgs') is None,
    reason='cmd_import lazy-imports the ingest pipeline (sensor_msgs)')


def _isolate_default_config(monkeypatch, tmp_path) -> None:
    """기본 설정 파일 탐색이 저장소의 실제 파일을 집지 않게 한다.

    `resolve_config_path` 는 `cli_common` 의 모듈 전역을 읽으므로 거기를 바꾼다.
    """
    from rdfp.dataset import cli_common

    monkeypatch.setattr(cli_common, 'DEFAULT_CONFIG_FILE_PATH',
                        tmp_path / 'config' / 'dataset_config.yaml')


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

@requires_ros
def test_main_auto_uses_default_config(monkeypatch, tmp_path) -> None:
    """--config 미지정 시 기본 경로(DEFAULT_CONFIG_FILE_PATH)의 설정을 자동 사용한다.

    실제 파이프라인은 호출하지 않도록 ``import_cmd`` 의 ``discover_splits`` 만
    빈 리스트로 stub — cmd_import 가 ``_empty_summary()`` 로 즉시 종료한다.
    discover_splits 가 받는 ``cfg.rosbag_dir`` 로 기본 경로의 config 가 로드된
    것을 확인한다.
    """
    cfg_path = tmp_path / 'dataset_config.yaml'
    cfg_path.write_text(
        'rosbag_dir: /tmp/x\n'
        'output_mp4_dir: /tmp/y\n'
    )
    monkeypatch.setattr(cli_common, 'DEFAULT_CONFIG_FILE_PATH', cfg_path)
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
        rc = import_cmd_mod.main([])
    assert rc == 2
