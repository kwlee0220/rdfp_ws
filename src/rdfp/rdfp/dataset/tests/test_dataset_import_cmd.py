"""``import`` 독립 CLI 단위 테스트 (argparse + main 흐름).

모듈 대부분은 ROS 없이 돈다. `cmd_import` 본문까지 들어가는 테스트만 예외로,
파이프라인 헬퍼가 lazy import 하는 `sensor_msgs` 가 필요해 개별 skip 한다.
"""

from __future__ import annotations

import argparse
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


@requires_ros
def test_cmd_import_prefers_root_from_caller(monkeypatch, tmp_path) -> None:
    """`rosbag export --rosbag-dir` 가 보낸 루트가 설정의 `rosbag_dir` 보다 앞선다.

    `export` 는 그 루트에서 에피소드를 골랐다. 적재가 설정의 루트를 다시 읽으면 고른 bag 과
    다른 bag 에서 같은 자연키를 찾게 된다.
    """
    cfg_path = tmp_path / 'dataset_config.yaml'
    cfg_path.write_text(
        'rosbag_dir: /tmp/from_config\n'
        'output_mp4_dir: /tmp/y\n'
    )
    called: list = []
    monkeypatch.setattr(
        import_cmd_mod, 'discover_splits',
        lambda rosbag_dir, dates=None, topics=None: (
            called.append(rosbag_dir) or []),
    )
    args = argparse.Namespace(start_ns_in=[1], rosbag_dir='/tmp/from_caller')

    rc = import_cmd_mod.cmd_import(args, str(cfg_path))

    assert rc == 0
    assert called == ['/tmp/from_caller']


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


# ---- 자동 삭제 · 병렬 실패 ----------------------------------------------

def test_split_deletion_blocker() -> None:
    """split 을 지워도 되는 것은 실패도, 필터로 빠진 에피소드도 없을 때뿐이다."""
    assert import_cmd_mod._split_deletion_blocker(0, 0) is None
    assert 'failed' in import_cmd_mod._split_deletion_blocker(1, 0)
    assert 'excluded 2' in import_cmd_mod._split_deletion_blocker(0, 2)


def _stub_pipeline(monkeypatch, tmp_path, *, failed: int, keep: int) -> tuple[str, list]:
    """`cmd_import` 의 무거운 단계(DB · 병렬 실행)를 대역으로 바꾼다.

    에피소드 둘을 감지하고 필터가 앞의 `keep` 개를 남긴다. 병렬 실행은 `failed` 개를
    실패로 보고한다. 돌려주는 목록에 `_delete_used_splits` 호출이 쌓인다.
    """
    from contextlib import contextmanager

    import rdfp.dataset.db.connection as connection_mod
    import rdfp.dataset.db.schema_check as schema_check_mod
    import rdfp.dataset.db.topics as topics_mod
    import rdfp.dataset.ingest.pipeline as pipeline_mod

    class _Conn:
        def commit(self) -> None:
            pass

    @contextmanager
    def _open(_db):
        yield _Conn()

    def _parallel(cfg, splits, episodes, topic_entries, topic_ids, image_topics,
                  run_log_path, summary) -> None:
        summary['episodes'] += len(episodes) - failed
        summary['failed'] += failed

    deleted: list = []
    monkeypatch.setattr(import_cmd_mod, 'discover_splits',
                        lambda rosbag_dir, dates=None, topics=None: ['split0'])
    monkeypatch.setattr(pipeline_mod, '_detect_all_episodes', lambda splits: ['ep1', 'ep2'])
    monkeypatch.setattr(pipeline_mod, '_apply_episode_filters', lambda eps, f: eps[:keep])
    monkeypatch.setattr(pipeline_mod, '_determine_topic_classification',
                        lambda splits, cfg: (set(), {}, {}))
    monkeypatch.setattr(pipeline_mod, '_run_parallel', _parallel)
    monkeypatch.setattr(pipeline_mod, '_delete_used_splits',
                        lambda splits: deleted.append(list(splits)) or len(splits))
    monkeypatch.setattr(connection_mod, 'open_connection', _open)
    monkeypatch.setattr(schema_check_mod, 'ensure_schema', lambda conn, tables: None)
    monkeypatch.setattr(topics_mod, 'upsert_topic_ids', lambda conn, topics: {})

    cfg_path = tmp_path / 'dataset_config.yaml'
    cfg_path.write_text(
        f'rosbag_dir: {tmp_path}/rosbag\n'
        f'output_mp4_dir: {tmp_path}/videos\n'
        'parallelism: 2\n'
        'delete_splits_after_import: true\n'
    )
    return str(cfg_path), deleted


@requires_ros
def test_parallel_failure_returns_1_and_keeps_splits(monkeypatch, tmp_path, capsys) -> None:
    """병렬 모드에서 에피소드가 실패하면 종료 코드 1 이고 split 을 지우지 않는다.

    예전에는 워커가 돌려준 `status='failed'` 를 집계가 버려 0 으로 끝났고, 자동 삭제가
    켜져 있으면 **실패한 에피소드의 원본까지** 지웠다.
    """
    cfg_path, deleted = _stub_pipeline(monkeypatch, tmp_path, failed=1, keep=2)

    rc = import_cmd_mod.cmd_import(argparse.Namespace(), cfg_path)

    assert rc == 1
    assert deleted == []
    out = capsys.readouterr().out
    assert '"failed": 1' in out
    assert '"delete_skipped"' in out


@requires_ros
def test_filtered_out_episodes_keep_splits(monkeypatch, tmp_path, capsys) -> None:
    """필터가 에피소드 일부를 뺐으면 지우지 않는다 — 같은 split 에 있을 수 있다.

    `rosbag export --episodes 3` 도 이 경로(`start_ns_in`)라, 예전에는 하나를 적재하고
    `rosbag_dir` 의 split 을 전부 지울 수 있었다.
    """
    cfg_path, deleted = _stub_pipeline(monkeypatch, tmp_path, failed=0, keep=1)

    rc = import_cmd_mod.cmd_import(argparse.Namespace(), cfg_path)

    assert rc == 0
    assert deleted == []
    assert 'excluded 1' in capsys.readouterr().out


@requires_ros
def test_clean_run_deletes_splits(monkeypatch, tmp_path) -> None:
    """실패도, 빠진 에피소드도 없으면 예전처럼 지운다."""
    cfg_path, deleted = _stub_pipeline(monkeypatch, tmp_path, failed=0, keep=2)

    rc = import_cmd_mod.cmd_import(argparse.Namespace(), cfg_path)

    assert rc == 0
    assert deleted == [['split0']]
