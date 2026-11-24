"""작업 이력 (로컬 SQLite) — Qt 없음."""

from __future__ import annotations

from rdfp_gui.history import JobHistory, default_path


def test_history_records_start_and_finish(tmp_path) -> None:
    history = JobHistory(tmp_path / 'jobs.sqlite3')
    a = history.start('import', 'ros2 run rdfp import --config c.yaml')
    b = history.start('export (npz)', 'ros2 run rdfp export --out /x')
    history.finish(a, 0, {'episodes': 3, 'inserted': {'joint_states': 30}})
    records = history.recent()
    assert [r.id for r in records] == [b, a]           # 최근 것이 먼저
    assert records[1].state == '성공'
    assert records[1].summary['episodes'] == 3
    assert records[0].state == '실행 중 또는 중단됨'
    history.finish(b, 2)
    assert history.recent()[0].state == '실패 (exit 2)'


def test_default_path_follows_xdg_state_home(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv('XDG_STATE_HOME', str(tmp_path))
    assert default_path() == tmp_path / 'rdfp-dataset-gui' / 'jobs.sqlite3'
