"""rosbag CLI 단위 테스트 (argparse 파싱 + list-topics / clear 출력 구조)."""

from __future__ import annotations

from rdfp.rosbag.cli import _build_parser


def test_no_subcommand_returns_2(capsys) -> None:
    import rdfp.rosbag.cli as cli_mod
    rc = cli_mod.main([])
    assert rc == 2


# ---------------------------------------------------------------------------
# list-topics
# ---------------------------------------------------------------------------

def test_list_topics_parser() -> None:
    ns = _build_parser().parse_args(
        ['list-topics', '--rosbag-dir', '/tmp/x',
         '--format', 'json', '--sort', 'count'],
    )
    assert ns.command == 'list-topics'
    assert ns.rosbag_dir == '/tmp/x'
    assert ns.format == 'json'
    assert ns.sort == 'count'


def test_list_topics_default_sort_is_name() -> None:
    ns = _build_parser().parse_args(['list-topics', '--rosbag-dir', '/tmp/x'])
    assert ns.sort == 'name'
    assert ns.format == 'text'


def test_list_topics_requires_rosbag_dir_or_env(monkeypatch, caplog) -> None:
    """`--rosbag-dir` 도 없고 RDFP_ROSBAG_DIR 도 없으면 exit 2 + 에러 로그."""
    import rdfp.rosbag.cli as cli_mod

    monkeypatch.delenv('RDFP_ROSBAG_DIR', raising=False)
    with caplog.at_level('ERROR'):
        rc = cli_mod.main(['list-topics'])
    assert rc == 2
    assert 'RDFP_ROSBAG_DIR' in caplog.text


def test_list_topics_uses_env_var_fallback(tmp_path, monkeypatch, capsys) -> None:
    """`--rosbag-dir` 미지정 시 RDFP_ROSBAG_DIR 환경변수가 사용된다."""
    import rdfp.rosbag.cli as cli_mod

    monkeypatch.setenv('RDFP_ROSBAG_DIR', str(tmp_path))
    captured: dict = {}

    def fake_discover(rosbag_dir):
        captured['rosbag_dir'] = rosbag_dir
        return []

    monkeypatch.setattr(cli_mod, 'discover_splits', fake_discover)

    rc = cli_mod.main(['list-topics'])
    assert rc == 0
    # 환경변수 값이 Path 로 변환되어 discover_splits 에 전달된다.
    from pathlib import Path
    assert captured['rosbag_dir'] == Path(tmp_path)


def test_list_topics_arg_overrides_env_var(tmp_path, monkeypatch) -> None:
    """`--rosbag-dir` 가 명시되면 환경변수보다 우선한다."""
    import rdfp.rosbag.cli as cli_mod

    monkeypatch.setenv('RDFP_ROSBAG_DIR', '/from/env')
    captured: dict = {}

    def fake_discover(rosbag_dir):
        captured['rosbag_dir'] = rosbag_dir
        return []

    monkeypatch.setattr(cli_mod, 'discover_splits', fake_discover)

    cli_mod.main(['list-topics', '--rosbag-dir', str(tmp_path)])
    from pathlib import Path
    assert captured['rosbag_dir'] == Path(tmp_path)


def test_list_topics_text_output(tmp_path, monkeypatch, capsys) -> None:
    """list-topics 가 total 라인 + ls -l 스타일 표 한 줄/토픽으로 출력한다."""
    import rdfp.rosbag.cli as cli_mod
    from datetime import datetime
    from types import SimpleNamespace

    local_ns = int(datetime(2024, 4, 11, 9, 0, 2).timestamp()) * 1_000_000_000
    splits = [SimpleNamespace(path='/tmp/a.mcap', session_name='sess_A', split_index=0)]

    def fake_inspect(path, per_topic):
        per_topic['/camera/image_raw'] = {
            'count': 1500, 'types': {'sensor_msgs/msg/Image'},
            'first_ns': local_ns, 'last_ns': local_ns + 50_000_000_000,
        }
        per_topic['/session'] = {
            'count': 45, 'types': {'rdfp_msgs/msg/SessionCommand'},
            'first_ns': local_ns, 'last_ns': local_ns + 50_000_000_000,
        }

    monkeypatch.setattr(cli_mod, 'discover_splits', lambda rosbag_dir: splits)
    monkeypatch.setattr(cli_mod, '_inspect_split_all_topics', fake_inspect)

    rc = cli_mod.main(['list-topics', '--rosbag-dir', str(tmp_path)])
    assert rc == 0
    out = capsys.readouterr().out
    assert 'total 2' in out
    assert 'COUNT' in out and 'RATE' in out and 'TOPIC' in out
    assert '/camera/image_raw' in out
    assert '/session' in out
    assert 'sensor_msgs/msg/Image' in out
    # name 정렬 기본: /camera/image_raw 가 /session 보다 먼저.
    idx_cam = out.index('/camera/image_raw')
    idx_sess = out.index('/session')
    assert idx_cam < idx_sess


def test_list_topics_sort_by_count(tmp_path, monkeypatch, capsys) -> None:
    import rdfp.rosbag.cli as cli_mod
    from types import SimpleNamespace

    splits = [SimpleNamespace(path='/tmp/a.mcap', session_name='s', split_index=0)]

    def fake_inspect(path, per_topic):
        per_topic['/low']  = {'count': 10,  'types': {'a/msg/X'},
                              'first_ns': 1, 'last_ns': 2}
        per_topic['/high'] = {'count': 999, 'types': {'a/msg/X'},
                              'first_ns': 1, 'last_ns': 2}

    monkeypatch.setattr(cli_mod, 'discover_splits', lambda rosbag_dir: splits)
    monkeypatch.setattr(cli_mod, '_inspect_split_all_topics', fake_inspect)

    cli_mod.main(['list-topics', '--rosbag-dir', str(tmp_path), '--sort', 'count'])
    out = capsys.readouterr().out
    # /high (count=999) 가 /low (count=10) 보다 먼저 나와야 한다.
    assert out.index('/high') < out.index('/low')


def test_list_topics_json_output(tmp_path, monkeypatch, capsys) -> None:
    import rdfp.rosbag.cli as cli_mod
    import json as json_mod
    from types import SimpleNamespace

    splits = [SimpleNamespace(path='/tmp/a.mcap', session_name='s', split_index=0)]

    def fake_inspect(path, per_topic):
        per_topic['/foo'] = {'count': 7, 'types': {'a/msg/X'},
                             'first_ns': 1_000_000_000, 'last_ns': 2_000_000_000}

    monkeypatch.setattr(cli_mod, 'discover_splits', lambda rosbag_dir: splits)
    monkeypatch.setattr(cli_mod, '_inspect_split_all_topics', fake_inspect)

    rc = cli_mod.main(
        ['list-topics', '--rosbag-dir', str(tmp_path), '--format', 'json'],
    )
    assert rc == 0
    decoded = json_mod.loads(capsys.readouterr().out)
    assert decoded['topic_count'] == 1
    assert decoded['topics'][0]['name'] == '/foo'
    assert decoded['topics'][0]['message_count'] == 7
    assert decoded['topics'][0]['rate_hz'] == 7.0   # 7 msgs / 1.0 s


def test_list_topics_empty_bag(tmp_path, monkeypatch, capsys) -> None:
    """split 이 전혀 없어도 exit 0, 'total 0' 만 출력."""
    import rdfp.rosbag.cli as cli_mod
    monkeypatch.setattr(cli_mod, 'discover_splits', lambda rosbag_dir: [])

    rc = cli_mod.main(['list-topics', '--rosbag-dir', str(tmp_path)])
    assert rc == 0
    assert 'total 0' in capsys.readouterr().out


# ---------------------------------------------------------------------------
# clear
# ---------------------------------------------------------------------------

def _make_session_tree(root, date: str, session: str, files=('data_0.mcap',)):
    """테스트용 `<root>/<date>/<session>/` 세션 디렉터리와 파일들을 만든다."""
    d = root / date / session
    d.mkdir(parents=True, exist_ok=True)
    for fname in files:
        (d / fname).write_bytes(b'x' * 16)
    return d


def test_clear_parser_defaults() -> None:
    ns = _build_parser().parse_args(['clear', '--rosbag-dir', '/tmp/x'])
    assert ns.command == 'clear'
    assert ns.rosbag_dir == '/tmp/x'
    assert ns.yes is False
    # --dry-run 은 더 이상 존재하지 않는다.
    assert not hasattr(ns, 'dry_run')


def test_clear_parser_flags() -> None:
    ns = _build_parser().parse_args(
        ['clear', '--rosbag-dir', '/tmp/x', '--yes'],
    )
    assert ns.yes is True


def test_clear_parser_rejects_dry_run_flag() -> None:
    """제거된 `--dry-run` 플래그는 argparse 단계에서 거부된다."""
    import pytest
    with pytest.raises(SystemExit):
        _build_parser().parse_args(
            ['clear', '--rosbag-dir', '/tmp/x', '--dry-run'],
        )


def test_clear_requires_rosbag_dir_or_env(monkeypatch, caplog) -> None:
    """`--rosbag-dir` 도 없고 RDFP_ROSBAG_DIR 도 없으면 exit 2 + 에러 로그."""
    import rdfp.rosbag.cli as cli_mod

    monkeypatch.delenv('RDFP_ROSBAG_DIR', raising=False)
    with caplog.at_level('ERROR'):
        rc = cli_mod.main(['clear'])
    assert rc == 2
    assert 'RDFP_ROSBAG_DIR' in caplog.text


def test_clear_uses_env_var_fallback(tmp_path, monkeypatch, capsys) -> None:
    """`--rosbag-dir` 미지정 시 RDFP_ROSBAG_DIR 환경변수가 사용된다."""
    import rdfp.rosbag.cli as cli_mod

    rosbag_root = tmp_path / 'rosbag'
    rosbag_root.mkdir()
    monkeypatch.setenv('RDFP_ROSBAG_DIR', str(rosbag_root))

    rc = cli_mod.main(['clear', '--yes'])
    assert rc == 0
    out = capsys.readouterr().out
    assert 'no session directories found' in out


def test_clear_with_yes_deletes_sessions(tmp_path, capsys) -> None:
    """`--yes` 는 TTY 확인 없이 세션 디렉터리를 모두 삭제한다."""
    import rdfp.rosbag.cli as cli_mod

    rosbag_root = tmp_path / 'rosbag'
    s1 = _make_session_tree(rosbag_root, '2026-04-20', 'session_2026-04-20_10-00-00')
    s2 = _make_session_tree(rosbag_root, '2026-04-21', 'session_2026-04-21_11-22-33')

    rc = cli_mod.main(['clear', '--rosbag-dir', str(rosbag_root), '--yes'])
    assert rc == 0
    out = capsys.readouterr().out
    assert 'deleted 2/2' in out
    # 세션 디렉터리 및 (비어진) 날짜 디렉터리 모두 제거.
    assert not s1.exists()
    assert not s2.exists()
    assert not (rosbag_root / '2026-04-20').exists()
    assert not (rosbag_root / '2026-04-21').exists()
    # rosbag_dir 자체는 유지.
    assert rosbag_root.is_dir()


def test_clear_non_interactive_without_yes_returns_2(
    tmp_path, monkeypatch, caplog,
) -> None:
    """non-TTY 환경에서 `--yes` 가 없으면 거부하고 exit 2 + 삭제되지 않는다."""
    import rdfp.rosbag.cli as cli_mod

    rosbag_root = tmp_path / 'rosbag'
    s1 = _make_session_tree(rosbag_root, '2026-04-20', 'session_2026-04-20_10-00-00')

    monkeypatch.setenv('CI', '1')    # non-interactive 로 강제.

    with caplog.at_level('ERROR'):
        rc = cli_mod.main(['clear', '--rosbag-dir', str(rosbag_root)])
    assert rc == 2
    assert s1.is_dir()               # 확정 없이 아무것도 삭제되지 않아야 한다.


def test_clear_no_sessions_returns_0(tmp_path, capsys) -> None:
    """세션 디렉터리가 없어도 성공 종료 (`0`)."""
    import rdfp.rosbag.cli as cli_mod

    rosbag_root = tmp_path / 'rosbag'
    rosbag_root.mkdir()

    rc = cli_mod.main(['clear', '--rosbag-dir', str(rosbag_root), '--yes'])
    assert rc == 0
    assert 'no session directories found' in capsys.readouterr().out


def test_clear_missing_rosbag_dir_returns_1(tmp_path, caplog) -> None:
    """`rosbag_dir` 자체가 존재하지 않으면 exit 1."""
    import rdfp.rosbag.cli as cli_mod

    bogus = tmp_path / 'does_not_exist'
    with caplog.at_level('ERROR'):
        rc = cli_mod.main(['clear', '--rosbag-dir', str(bogus), '--yes'])
    assert rc == 1


def test_clear_skips_non_session_dirs(tmp_path) -> None:
    """세션 디렉터리 네이밍 규칙과 맞지 않는 경로는 보존한다."""
    import rdfp.rosbag.cli as cli_mod

    rosbag_root = tmp_path / 'rosbag'
    valid = _make_session_tree(rosbag_root, '2026-04-20', 'session_2026-04-20_10-00-00')

    # 날짜 폴더 형식이 아닌 이웃 디렉터리 (e.g. 사용자가 복사해둔 백업).
    stray = rosbag_root / 'backup_snapshot'
    stray.mkdir()
    (stray / 'note.txt').write_text('keep me')

    rc = cli_mod.main(['clear', '--rosbag-dir', str(rosbag_root), '--yes'])
    assert rc == 0
    assert not valid.exists()
    assert stray.is_dir()           # 규칙 외 폴더는 보존된다.
    assert (stray / 'note.txt').is_file()


# ---------------------------------------------------------------------------
# _confirm_clear: 대화형 입력 검증 (TTY 모킹).
# ---------------------------------------------------------------------------

def _force_tty_no_ci(monkeypatch) -> None:
    """대화형 입력 분기로 진입시키기 위한 공통 모킹."""
    monkeypatch.delenv('CI', raising=False)
    monkeypatch.setattr('sys.stdin.isatty', lambda: True)


def test_confirm_clear_accepts_lowercase_y(monkeypatch, tmp_path) -> None:
    """'y' 입력은 진행을 승인한다."""
    import rdfp.rosbag.cli as cli_mod

    _force_tty_no_ci(monkeypatch)
    monkeypatch.setattr('builtins.input', lambda _prompt: 'y')
    assert cli_mod._confirm_clear(tmp_path) is True


def test_confirm_clear_accepts_uppercase_y(monkeypatch, tmp_path) -> None:
    """'Y' 도 동일하게 승인한다 (대소문자 무관)."""
    import rdfp.rosbag.cli as cli_mod

    _force_tty_no_ci(monkeypatch)
    monkeypatch.setattr('builtins.input', lambda _prompt: 'Y')
    assert cli_mod._confirm_clear(tmp_path) is True


def test_confirm_clear_rejects_other_inputs(monkeypatch, tmp_path) -> None:
    """'y'/'Y' 외의 입력은 모두 거부된다 (예: 'YES', 'yes', 빈 입력)."""
    import rdfp.rosbag.cli as cli_mod

    _force_tty_no_ci(monkeypatch)
    for ans in ('YES', 'yes', 'no', 'n', '', 'yeah'):
        monkeypatch.setattr('builtins.input', lambda _prompt, a=ans: a)
        assert cli_mod._confirm_clear(tmp_path) is False, f'should reject {ans!r}'
