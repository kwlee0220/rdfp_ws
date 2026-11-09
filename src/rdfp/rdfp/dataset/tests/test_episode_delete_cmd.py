"""``episode-delete`` 독립 CLI 단위 테스트 (argparse + 확인 규칙 + main 흐름)."""

from __future__ import annotations

import pytest

import rdfp.dataset.episode_delete_cmd as mod


def test_parse_id_spec() -> None:
    assert mod.parse_id_spec('11,12-13') == [11, 12, 13]
    assert mod.parse_id_spec(' 3 , 1,1') == [1, 3]
    with pytest.raises(ValueError):
        mod.parse_id_spec('5-2')


def test_parser_requires_ids() -> None:
    with pytest.raises(SystemExit):
        mod._build_parser().parse_args(['--config', 'x.yaml'])
    ns = mod._build_parser().parse_args(['--ids', '1-3', '--delete-files', '--yes', '--dry-run'])
    assert ns.ids == '1-3'
    assert ns.delete_files and ns.yes and ns.dry_run


def test_main_without_config_returns_2(monkeypatch, tmp_path) -> None:
    from rdfp.dataset import cli_common

    monkeypatch.setattr(cli_common, 'DEFAULT_CONFIG_FILE_PATH', tmp_path / 'nope.yaml')
    assert mod.main(['--ids', '1']) == 2


def test_confirm_refuses_in_non_tty_without_yes(monkeypatch) -> None:
    """파괴적 명령의 공통 규칙 — 비대화형에서는 `--yes` 가 필수다."""
    import sys

    monkeypatch.setattr(sys.stdin, 'isatty', lambda: False, raising=False)
    assert mod._confirm(3, yes=False) is False
    assert mod._confirm(3, yes=True) is True


def test_confirm_requires_the_word(monkeypatch) -> None:
    import builtins
    import sys

    monkeypatch.setattr(sys.stdin, 'isatty', lambda: True, raising=False)
    monkeypatch.setattr(builtins, 'input', lambda prompt: 'yes')
    assert mod._confirm(1, yes=False) is False
    monkeypatch.setattr(builtins, 'input', lambda prompt: ' delete ')
    assert mod._confirm(1, yes=False) is True


def test_dry_run_prints_plan_and_deletes_nothing(monkeypatch, tmp_path, capsys) -> None:
    """`--dry-run` 은 영상 경로까지 보여 주고 DB 를 건드리지 않는다."""
    from contextlib import contextmanager

    from rdfp.dataset.inspect.db_inspect import EpisodeSummary, StreamInfo

    rec = tmp_path / 'rec'
    rec.mkdir()
    (rec / 'e.mp4').write_bytes(b'')
    (rec / 'e.jsonl').write_bytes(b'')
    cfg_path = tmp_path / 'dataset_config.yaml'
    cfg_path.write_text(f'rosbag_dir: /tmp/x\nrecordings_dir: {rec}\n')

    deleted: list = []

    @contextmanager
    def _open(_db):
        yield object()

    def _summaries(conn, *, mp4_roots=()):
        return [EpisodeSummary(id=5, start_ns=1_700_000_000_000_000_000,
                               stop_ns=1_700_000_010_000_000_000, task_label='pick',
                               success=True, metadata={}, row_counts={'joint_states': 3},
                               streams=[StreamInfo('/cam', 'e.mp4', 1, 1, str(rec / 'e.mp4'))])]

    monkeypatch.setattr(mod, 'open_connection', _open)
    monkeypatch.setattr(mod, 'episode_summaries', _summaries)
    monkeypatch.setattr(mod, 'delete_episodes', lambda conn, ids: deleted.append(ids) or 1)

    rc = mod.main(['--config', str(cfg_path), '--ids', '5', '--dry-run'])

    assert rc == 0
    assert deleted == []
    out = capsys.readouterr().out
    assert 'e.mp4' in out and 'e.jsonl' in out
    assert '1 episode(s)' in out


def test_unknown_id_is_a_usage_error(monkeypatch, tmp_path) -> None:
    from contextlib import contextmanager

    cfg_path = tmp_path / 'dataset_config.yaml'
    cfg_path.write_text('rosbag_dir: /tmp/x\n')

    @contextmanager
    def _open(_db):
        yield object()

    monkeypatch.setattr(mod, 'open_connection', _open)
    monkeypatch.setattr(mod, 'episode_summaries', lambda conn, *, mp4_roots=(): [])
    assert mod.main(['--config', str(cfg_path), '--ids', '9', '--yes']) == 2
