"""화면 조각들을 offscreen 으로 띄워 본다 — 합성 bag 하나, DB 없음.

보는 것은 "뜨고, 채워지고, 안 죽는다" 다. 선의 좌표나 색을 픽셀로 확인하지는 않는다.
"""

from __future__ import annotations

import pytest

pytest.importorskip('PySide6.QtWidgets')

from PySide6.QtCore import Qt  # noqa: E402
from PySide6.QtWidgets import QDialogButtonBox  # noqa: E402

from rdfp.dataset.inspect.db_inspect import EpisodeSummary, StreamInfo  # noqa: E402
from rdfp.dataset.inspect.findings import LEVEL_ERROR, LEVEL_INFO, Finding  # noqa: E402
from rdfp.rosbag.tests.fixtures.make_synth_bag import (JointStateEventSpec,  # noqa: E402
                                                       SessionEventSpec, write_synth_bag)
from rdfp_gui.context import AppContext  # noqa: E402
from rdfp_gui.jobs import JobSpec  # noqa: E402
from rdfp_gui.models import RowsTableModel  # noqa: E402
from rdfp_gui.panels.common import FindingsList, TypedConfirmDialog  # noqa: E402
from rdfp_gui.snapshot import (DbSnapshot, EpisodeData, TableEntry, load_bag_snapshot,  # noqa: E402
                               load_db_snapshot)

S0 = 1_700_000_000_000_000_000
SEC = 1_000_000_000


@pytest.fixture
def ctx(tmp_path, monkeypatch) -> AppContext:
    monkeypatch.delenv('RDFP_DB_DSN', raising=False)
    bags = tmp_path / 'bags'
    bags.mkdir()
    write_synth_bag(
        bags,
        session_events=[SessionEventSpec(S0, 'IN_SESSION', 'pick'),
                        SessionEventSpec(S0 + 10 * SEC, 'IN_EPISODE', 'pick'),
                        SessionEventSpec(S0 + 20 * SEC, 'IN_SESSION', 'pick')],
        joint_state_events=[JointStateEventSpec(S0 + 12 * SEC, position=[0.1])])
    broken = bags / '2026-04-11' / 'session_2026-04-11_11-00-00'
    broken.mkdir()
    cfg = tmp_path / 'dataset_config.yaml'
    cfg.write_text(f'rosbag_dir: {bags}\noutput_mp4_dir: {tmp_path}/videos\n')
    return AppContext.load(str(cfg))


def _wait(qapp, bg, store, *, ms: int = 15000) -> None:
    """백그라운드 작업이 끝나고 큐에 쌓인 콜백이 GUI 스레드에서 처리될 때까지."""
    assert bg.wait(ms)
    for _ in range(5):
        qapp.processEvents()


def test_main_window_populates_from_bag_without_db(qapp, ctx) -> None:
    from rdfp_gui.app import MainWindow

    win = MainWindow(ctx)
    _wait(qapp, win.bg, win.store)
    assert win.store.bag is not None and win.store.bag.error is None
    assert win.store.db is not None and win.store.db.error      # DSN 이 없다
    assert win.store.episodes is not None and win.store.episodes.error

    mapping = win.mapping_panel
    assert set(mapping._topic_items) == {'/session', '/joint_states'}
    assert 'joint_states' in mapping._table_items
    # 매핑 선: /session → sessions, /joint_states → joint_states.
    assert len(mapping._canvas._links) == 2
    assert mapping.checked_topics() == ['/joint_states']

    # 토픽을 고르면 짝 테이블이 선택되고 상세가 채워진다.
    mapping._topics.setCurrentItem(mapping._topic_items['/joint_states'])
    _wait(qapp, win.bg, win.store)
    assert mapping._table_items['joint_states'].isSelected()
    assert 'joint_states' in mapping.detail._summary.toPlainText()
    assert 'float64[] position' in mapping.detail._schema.toPlainText()
    # 진단은 선택 시 자동으로 돌아 △/● 가 정해진다.
    assert win.store.bag.entry('/joint_states').diagnosis is not None

    # ③ 세션 탭: 완료 하나 · 미완료 하나.
    tree = win.session_panel._tree
    date_item = tree.topLevelItem(0)
    statuses = sorted(date_item.child(i).text(1) for i in range(date_item.childCount()))
    assert statuses == ['미완료', '완료']

    # ② 에피소드 탭: DB 가 없으니 bag 의 미적재 행만 (적재 여부 ?) 보인다.
    ep_tree = win.episode_panel._tree
    assert ep_tree.topLevelItemCount() == 1
    assert ep_tree.topLevelItem(0).text(12) == '적재 여부 ?'

    # ④ 설정 탭: 로드 OK + topics_file 없음(모든 토픽).
    codes = {win.config_panel._findings.item(i).toolTip()
             for i in range(win.config_panel._findings.count())}
    assert {'config_ok', 'topics_all'} <= codes
    win.close()


def test_import_dialog_builds_the_cli_and_blocks_on_errors(qapp, ctx) -> None:
    from rdfp_gui.panels.mapping_panel import ImportDialog
    from rdfp_gui.workers import Background

    bag = load_bag_snapshot(ctx.cfg)
    db = load_db_snapshot(ctx.cfg, bag)
    bg = Background()
    dialog = ImportDialog(ctx, bag, db, ['/joint_states'], bg)
    _wait(qapp, bg, None)
    ok = dialog._buttons.button(QDialogButtonBox.StandardButton.Ok)
    # DB 에 못 붙으므로(check_db → error) 실행은 막힌다 — 그래도 명령은 보인다.
    assert dialog._findings.has_errors()
    assert not ok.isEnabled()
    assert dialog.spec is not None
    assert '--config' in dialog.spec.command_line()
    # 전부 골랐으므로 --start-ns 도 --topics 도 없다.
    assert '--start-ns' not in dialog.spec.argv and '--topics' not in dialog.spec.argv
    # 정책을 바꾸면 명령에 실린다.
    dialog._policy.setCurrentIndex(dialog._policy.findData('replace'))
    assert '--on-existing' in dialog.spec.argv
    # 에피소드를 모두 빼면 OK 가 꺼진다.
    dialog._set_all(Qt.CheckState.Unchecked)
    assert dialog.selected_keys() == []


def test_episode_panel_shows_db_rows_and_warnings(qapp, ctx) -> None:
    from rdfp_gui.panels.episode_panel import EpisodePanel
    from rdfp_gui.store import DataStore
    from rdfp_gui.workers import Background

    bg = Background()
    store = DataStore(ctx, bg)
    panel = EpisodePanel(ctx, store, bg)
    s_ok = EpisodeSummary(id=1, start_ns=S0, stop_ns=S0 + 10 * SEC, task_label='pick', success=True,
                          metadata={'backend': 'panda_isaac'},
                          row_counts={'joint_states': 10, 'gripper_cmds': 1, 'image_frames': 5},
                          streams=[StreamInfo('/cam', 'a.mp4', 5, 5, '/x/a.mp4')])
    s_bad = EpisodeSummary(id=2, start_ns=S0 + 30 * SEC, stop_ns=S0 + 40 * SEC, task_label='pick',
                           success=None, metadata={}, row_counts={}, streams=[])
    checks = {1: [], 2: [Finding('warning', 'no_message_rows', 'empty')]}
    store.episodes = EpisodeData(summaries=[s_ok, s_bad], checks=checks)
    store.db = DbSnapshot(exported_keys={S0, S0 + 30 * SEC})
    store.episodes_changed.emit()
    tree = panel._tree
    assert tree.topLevelItemCount() == 2
    assert tree.topLevelItem(0).text(12) == '✓'
    assert tree.topLevelItem(1).text(12) == '⚠ 1'
    assert tree.topLevelItem(0).text(5) == 'panda_isaac'
    panel._only_warn.setChecked(True)
    assert tree.topLevelItemCount() == 1
    tree.topLevelItem(0).setSelected(True)
    assert panel.selected_ids() == [2]


def test_table_model_and_findings_widgets(qapp) -> None:
    model = RowsTableModel(['a', 'b'], [(1, [1.5, 2.0]), (None, {'k': 'v'})])
    assert model.rowCount() == 2 and model.columnCount() == 2
    assert model.data(model.index(0, 1)) == '[1.5, 2.0]'
    assert model.data(model.index(1, 0)) == ''
    assert '"k": "v"' in model.row_json(1)
    model.clear()
    assert model.rowCount() == 0

    findings = FindingsList()
    findings.set_findings([Finding(LEVEL_INFO, 'ok', 'fine'), Finding(LEVEL_ERROR, 'bad', 'nope')])
    assert findings.count() == 2 and findings.has_errors()
    findings.set_findings([])
    assert findings.count() == 1 and not findings.has_errors()


def test_typed_confirm_requires_the_exact_text(qapp) -> None:
    dialog = TypedConfirmDialog(None, '삭제', '정말?', 'session_x')
    ok = dialog._buttons.button(QDialogButtonBox.StandardButton.Ok)
    assert not ok.isEnabled()
    dialog._edit.setText('session_y')
    assert not ok.isEnabled()
    dialog._edit.setText(' session_x ')
    assert ok.isEnabled()


def test_job_panel_shows_the_command_and_progress(qapp) -> None:
    from rdfp_gui.jobs import JobRunner
    from rdfp_gui.panels.job_panel import JobPanel

    runner = JobRunner()
    panel = JobPanel(runner)
    spec = JobSpec(title='import', argv=['ros2', 'run', 'rdfp', 'import', '--config', 'c.yaml'],
                   kind='import', total_hint=3)
    panel.show_spec(spec)
    assert panel._command.text() == 'ros2 run rdfp import --config c.yaml'
    runner.started.emit(spec)
    assert panel._progress.maximum() == 3
    runner.progress.emit(2, 3, '2/3 에피소드')
    assert panel._progress.value() == 2
    runner.finished.emit(0, {'episodes': 3, 'inserted': {'joint_states': 30}})
    assert '요약' in panel._log.toPlainText()
    assert not panel._cancel.isEnabled()


def test_table_entry_detail_without_db(qapp) -> None:
    from rdfp.dataset.inspect.mapping import TableInfo
    from rdfp_gui.panels.detail_panel import DetailPanel

    panel = DetailPanel()
    info = TableInfo('joint_states', 'builtin', ('sensor_msgs/msg/JointState',), 'desc')
    entry = TableEntry(info, None, ['/joint_states'])
    panel.show_table(entry, None, None, db_error='no dsn')
    text = panel._summary.toPlainText()
    assert 'joint_states' in text and 'no dsn' in text
    assert not panel._browse.isEnabled()
