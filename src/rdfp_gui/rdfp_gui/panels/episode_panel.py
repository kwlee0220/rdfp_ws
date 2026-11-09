"""② 에피소드 탭 — DB 의 에피소드 + bag 의 미적재 에피소드를 한 표에, 점검 · 상세 · 삭제 · Export
(설계안 §4.5 · §4.6)."""

from __future__ import annotations

from typing import Optional

import json

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QAbstractItemView, QCheckBox, QComboBox, QDialog, QDialogButtonBox,
                               QDoubleSpinBox, QFileDialog, QHBoxLayout, QHeaderView, QLabel,
                               QLineEdit, QListWidget, QPlainTextEdit, QPushButton, QSpinBox,
                               QTextBrowser, QToolBar, QTreeWidget, QTreeWidgetItem, QVBoxLayout,
                               QWidget)

from rdfp.dataset.inspect.db_inspect import MESSAGE_TABLES, EpisodeSummary, recording_files_for
from rdfp.dataset.inspect.findings import LEVEL_WARNING
from rdfp.dataset.inspect.preflight import check_export

from ..context import AppContext, fmt_time
from ..jobs import build_episode_delete_command, build_export_command, episodes_spec
from ..snapshot import exported_flag
from ..store import DataStore
from ..workers import Background
from .browser_panel import BrowserDialog, DbSource
from .common import COLORS, FindingsList, TypedConfirmDialog, brush, esc, mono_font

_ROLE_ID = Qt.ItemDataRole.UserRole
_COLUMNS = ['id', '시작', '길이', 'task', '결과', '백엔드', '행 합', 'joint_states', 'gripper_cmds',
            'motion', 'scene', '영상', '검증', '세션 (bag)']


def _result_text(success: Optional[bool]) -> str:
    return 'success' if success is True else ('failure' if success is False else '-')


def export_preflight(cfg, ids, *, camera_topic: str, label: str) -> list:
    """`check_export` 를 커넥션과 함께 — 워커에서 부른다."""
    from rdfp.dataset.db.connection import open_connection

    with open_connection(cfg.db) as conn:
        findings = check_export(conn, ids, camera_topic=camera_topic, label=label)
        conn.rollback()
    return findings


class EpisodePanel(QWidget):
    job_requested = Signal(object)

    def __init__(self, ctx: AppContext, store: DataStore, bg: Background,
                 parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.ctx = ctx
        self.store = store
        self.bg = bg
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        toolbar = QToolBar()
        toolbar.addAction('새로 고침', store.refresh_episodes)
        toolbar.addSeparator()
        toolbar.addWidget(QLabel(' task: '))
        self._filter = QLineEdit()
        self._filter.setPlaceholderText('task_label 로 거른다')
        self._filter.setMaximumWidth(220)
        self._filter.textChanged.connect(self._populate)
        toolbar.addWidget(self._filter)
        self._only_warn = QCheckBox('경고만')
        self._only_warn.toggled.connect(self._populate)
        toolbar.addWidget(self._only_warn)
        self._show_bag = QCheckBox('미적재(bag) 보이기')
        self._show_bag.setChecked(True)
        self._show_bag.toggled.connect(self._populate)
        toolbar.addWidget(self._show_bag)
        toolbar.addSeparator()
        toolbar.addAction('상세', self._detail)
        toolbar.addAction('Export…', self._export)
        toolbar.addAction('삭제…', self._delete)
        self._count = QLabel('')
        toolbar.addSeparator()
        toolbar.addWidget(self._count)
        layout.addWidget(toolbar)

        self._tree = QTreeWidget()
        self._tree.setHeaderLabels(_COLUMNS)
        self._tree.setRootIsDecorated(False)
        self._tree.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self._tree.header().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self._tree.itemDoubleClicked.connect(lambda *_: self._detail())
        layout.addWidget(self._tree, 1)

        self._hint = QLabel('')
        self._hint.setWordWrap(True)
        layout.addWidget(self._hint)

        store.episodes_changed.connect(self._populate)
        store.bag_changed.connect(self._populate)
        store.db_changed.connect(self._populate)

    # ----- 채우기 --------------------------------------------------------------

    def _populate(self) -> None:
        self._tree.clear()
        data = self.store.episodes
        text = self._filter.text().strip()
        shown = warned = 0
        if data is not None:
            if data.error:
                self._hint.setText(f'DB: {data.error}')
            else:
                self._hint.setText('')
            for s in data.summaries:
                if text and text not in (s.task_label or ''):
                    continue
                warnings = data.warnings(s.id)
                if self._only_warn.isChecked() and not warnings:
                    continue
                self._tree.addTopLevelItem(self._db_item(s, warnings))
                shown += 1
                warned += bool(warnings)
        bag = self.store.bag
        keys = self.store.db.exported_keys if self.store.db else None
        unexported = 0
        if bag is not None and self._show_bag.isChecked() and not self._only_warn.isChecked():
            for be in bag.episodes:
                if exported_flag(be, keys) == 'yes':
                    continue
                if text and text not in be.task_label:
                    continue
                self._tree.addTopLevelItem(self._bag_item(be, keys))
                unexported += 1
        self._count.setText(f' DB {shown} (경고 {warned}) · 미적재 {unexported}')

    def _db_item(self, s: EpisodeSummary, warnings: list) -> QTreeWidgetItem:
        rc = s.row_counts
        motion = f'{rc.get("motion_trajectories", 0)}/{rc.get("motion_events", 0)}'
        if s.streams:
            missing = any(not st.file_exists for st in s.streams)
            video = f'{s.frames:,} 프레임 · {len(s.streams)} 카메라' + (' ⚠ 파일 없음' if missing else '')
        else:
            video = '없음'
        check = f'⚠ {len(warnings)}' if warnings else '✓'
        item = QTreeWidgetItem([str(s.id), fmt_time(s.start_ns), f'{s.duration_sec:.1f} s',
                                s.task_label or '-', _result_text(s.success), s.backend or '-',
                                f'{s.message_rows:,}', f'{rc.get("joint_states", 0):,}',
                                str(rc.get('gripper_cmds', 0)), motion,
                                str(rc.get('scene_objects', 0)), video, check, ''])
        item.setData(0, _ROLE_ID, s.id)
        if warnings:
            item.setToolTip(12, '\n'.join(f'• {w.message}' for w in warnings))
            item.setForeground(12, brush(LEVEL_WARNING))
            item.setForeground(0, brush(LEVEL_WARNING))
        return item

    def _bag_item(self, be, keys) -> QTreeWidgetItem:
        flag = exported_flag(be, keys)
        item = QTreeWidgetItem(['—', fmt_time(be.log_start_ns), f'{be.duration_ns / 1e9:.1f} s',
                                be.task_label or '-', _result_text(be.episode.success), '-', '', '',
                                '', '', '', '', '미적재' if flag == 'no' else '적재 여부 ?',
                                be.session_name])
        item.setData(0, _ROLE_ID, None)
        for col in range(len(_COLUMNS)):
            item.setForeground(col, brush('muted'))
        item.setToolTip(0, 'bag 에서 감지됐지만 DB 에 없다 — ① 탭의 Import 로 적재한다')
        return item

    def selected_ids(self) -> list:
        ids = []
        for item in self._tree.selectedItems():
            eid = item.data(0, _ROLE_ID)
            if eid is not None:
                ids.append(int(eid))
        return sorted(set(ids))

    # ----- 조작 ----------------------------------------------------------------

    def _detail(self) -> None:
        ids = self.selected_ids()
        if not ids or self.store.episodes is None:
            return
        s = self.store.episodes.by_id(ids[0])
        if s is None:
            return
        episodes = [(x.id, x.start_ns) for x in self.store.episodes.summaries]
        dialog = EpisodeDetailDialog(self.ctx, self.bg, s, self.store.episodes.checks.get(s.id, []),
                                     episodes, self)
        dialog.show()

    def _export(self) -> None:
        ids = self.selected_ids()
        if self.ctx.cfg is None or self.store.episodes is None:
            return
        if not ids:
            ids = [s.id for s in self.store.episodes.summaries if s.streams]
            if not ids:
                self._hint.setText('영상이 있는 에피소드가 없어 export 할 것이 없다')
                return
        summaries = [self.store.episodes.by_id(i) for i in ids]
        dialog = ExportDialog(self.ctx, self.bg, [s for s in summaries if s is not None], self)
        if dialog.exec() == QDialog.DialogCode.Accepted and dialog.spec is not None:
            self.job_requested.emit(dialog.spec)

    def _delete(self) -> None:
        ids = self.selected_ids()
        if not ids or self.ctx.cfg is None or self.store.episodes is None:
            return
        summaries = [s for s in (self.store.episodes.by_id(i) for i in ids) if s is not None]
        roots = self.ctx.mp4_roots()
        files = [f for s in summaries for st in s.streams
                 for f in recording_files_for(st.mp4_path, roots)]
        rows = sum(s.message_rows for s in summaries)
        frames = sum(s.frames for s in summaries)
        spec_text = episodes_spec(ids)
        extra = QWidget()
        box = QVBoxLayout(extra)
        box.setContentsMargins(0, 0, 0, 0)
        file_list = QListWidget()
        file_list.setMaximumHeight(120)
        for f in files:
            file_list.addItem(str(f))
        box.addWidget(QLabel(f'디스크의 영상 파일 {len(files)}개:'))
        box.addWidget(file_list)
        delete_files = QCheckBox('영상 파일도 지운다 (mp4 · sidecar · metadata)')
        box.addWidget(delete_files)
        text = (f'에피소드 <b>{len(ids)}</b>개 (id {esc(spec_text)}) 를 DB 에서 지운다 — 메시지 행 '
                f'<b>{rows:,}</b> · 프레임 <b>{frames:,}</b> 이 함께 사라진다 (ON DELETE CASCADE). '
                f'bag 은 건드리지 않는다 (`rosbag delete` 의 몫).')
        if TypedConfirmDialog.ask(self, '에피소드 삭제', text, spec_text, extra):
            self.job_requested.emit(build_episode_delete_command(
                str(self.ctx.config_path), ids, delete_files=delete_files.isChecked()))


class EpisodeDetailDialog(QDialog):
    def __init__(self, ctx: AppContext, bg: Background, s: EpisodeSummary, checks: list,
                 episodes: list, parent=None) -> None:
        super().__init__(parent)
        self.ctx = ctx
        self.bg = bg
        self.summary = s
        self._episodes = tuple(episodes)
        self.setWindowTitle(f'에피소드 {s.id} — {s.task_label or "-"}')
        self.resize(800, 650)
        layout = QVBoxLayout(self)

        browser = QTextBrowser()
        parts = [f'<p><b>id {s.id}</b> · {esc(fmt_time(s.start_ns))} → {esc(fmt_time(s.stop_ns))} '
                 f'({s.duration_sec:.1f} s) · task {esc(s.task_label or "-")} · 결과 '
                 f'{_result_text(s.success)} · 백엔드 {esc(s.backend or "-")}</p>']
        if checks:
            parts.append('<p><b>점검</b><br>' + '<br>'.join(
                f'<span style="color:{COLORS.get(c.level, "#000")}">'
                f'{"⚠" if c.level == "warning" else "ⓘ"} {esc(c.message)}</span>'
                for c in checks) + '</p>')
        if s.streams:
            parts.append('<p><b>영상</b><br>' + '<br>'.join(
                f'<code>{esc(st.topic)}</code> → {esc(st.mp4_path)} · 프레임 {st.frames_in_db:,}'
                f'/{st.frame_count:,}' + ('' if st.file_exists else
                                          f' <span style="color:{COLORS["error"]}">파일 없음</span>')
                for st in s.streams) + '</p>')
        browser.setHtml(''.join(parts))
        layout.addWidget(browser, 1)

        layout.addWidget(QLabel('테이블별 행 수 — 더블클릭하면 그 테이블의 이 에피소드 행을 본다'))
        self._tables = QTreeWidget()
        self._tables.setHeaderLabels(['테이블', '행'])
        self._tables.setRootIsDecorated(False)
        self._tables.header().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        for table in [*MESSAGE_TABLES, 'image_streams', 'image_frames']:
            n = s.row_counts.get(table, len(s.streams) if table == 'image_streams' else 0)
            item = QTreeWidgetItem([table, f'{n:,}'])
            if n == 0:
                item.setForeground(0, brush('muted'))
                item.setForeground(1, brush('muted'))
            self._tables.addTopLevelItem(item)
        self._tables.itemDoubleClicked.connect(self._browse)
        layout.addWidget(self._tables, 1)

        layout.addWidget(QLabel('metadata (sessions.metadata)'))
        meta = QPlainTextEdit()
        meta.setReadOnly(True)
        meta.setFont(mono_font())
        meta.setPlainText(json.dumps(s.metadata, ensure_ascii=False, indent=2)
                          if s.metadata else '{}')
        meta.setMaximumHeight(140)
        layout.addWidget(meta)

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        buttons.rejected.connect(self.reject)
        buttons.clicked.connect(lambda *_: self.accept())
        layout.addWidget(buttons)

    def _browse(self, item, _column) -> None:
        dialog = BrowserDialog(self.ctx, self.bg,
                               DbSource(item.text(0), episode_id=self.summary.id,
                                        episodes=self._episodes), self)
        dialog.show()


class ExportDialog(QDialog):
    """범위 → 사전 점검 → `export` 명령 (설계안 §4.6). npz 만 — LeRobot v3 는 미구현(§6.5)."""

    def __init__(self, ctx: AppContext, bg: Background, summaries: list, parent=None) -> None:
        super().__init__(parent)
        self.ctx = ctx
        self.bg = bg
        self.summaries = summaries
        self.ids = sorted(s.id for s in summaries)
        self.spec = None
        self._token: Optional[int] = None
        self.setWindowTitle(f'Export — 에피소드 {len(self.ids)}개')
        self.resize(820, 620)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel(f'<b>에피소드</b> {esc(episodes_spec(self.ids))}'))

        row = QHBoxLayout()
        row.addWidget(QLabel('출력 디렉터리'))
        self._out = QLineEdit(str(ctx.default_export_dir()))
        browse = QPushButton('찾기…')
        browse.clicked.connect(self._pick_dir)
        row.addWidget(self._out, 1)
        row.addWidget(browse)
        layout.addLayout(row)

        opts = QHBoxLayout()
        opts.addWidget(QLabel('형식'))
        self._format = QComboBox()
        self._format.addItem('npz (현행 — rdfp-train 이 읽는 형식)', 'npz')
        self._format.addItem('LeRobot v3 (미구현 — 설계안 §6.5)', 'lerobot_v3')
        self._format.model().item(1).setEnabled(False)
        opts.addWidget(self._format)
        opts.addWidget(QLabel('레이블'))
        self._label = QComboBox()
        self._label.addItem('joint_plan', 'joint_plan')
        self._label.addItem('joint_command', 'joint_command')
        opts.addWidget(self._label)
        opts.addWidget(QLabel('chunk_len'))
        self._chunk = QSpinBox()
        self._chunk.setRange(1, 200)
        self._chunk.setValue(20)
        opts.addWidget(self._chunk)
        opts.addWidget(QLabel('Δ'))
        self._delta = QDoubleSpinBox()
        self._delta.setDecimals(3)
        self._delta.setRange(0.0, 5.0)
        self._delta.setSingleStep(0.05)
        self._delta.setValue(0.1)
        opts.addWidget(self._delta)
        opts.addWidget(QLabel('T_a'))
        self._step = QDoubleSpinBox()
        self._step.setDecimals(3)
        self._step.setRange(0.001, 5.0)
        self._step.setSingleStep(0.05)
        self._step.setValue(0.1)
        opts.addWidget(self._step)
        layout.addLayout(opts)

        cam_row = QHBoxLayout()
        cam_row.addWidget(QLabel('카메라 (frame clock)'))
        self._camera = QComboBox()
        cameras = sorted({st.topic for s in summaries for st in s.streams})
        self._camera.addItem('(자동 — 카메라가 하나일 때)', '')
        for c in cameras:
            self._camera.addItem(c, c)
        if len(cameras) == 1:
            self._camera.setCurrentIndex(1)
        cam_row.addWidget(self._camera, 1)
        self._keep = QCheckBox('가정이 섞인 에피소드도 포함 (--keep-untrustworthy)')
        cam_row.addWidget(self._keep)
        layout.addLayout(cam_row)

        layout.addWidget(QLabel('<b>사전 점검</b>'))
        self._findings = FindingsList()
        layout.addWidget(self._findings, 1)
        layout.addWidget(QLabel('실행할 명령:'))
        self._command = QLineEdit()
        self._command.setReadOnly(True)
        self._command.setFont(mono_font())
        layout.addWidget(self._command)

        self._buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok
                                         | QDialogButtonBox.StandardButton.Cancel)
        self._buttons.button(QDialogButtonBox.StandardButton.Ok).setText('Export 실행')
        self._buttons.accepted.connect(self.accept)
        self._buttons.rejected.connect(self.reject)
        layout.addWidget(self._buttons)

        for w in (self._label, self._camera):
            w.currentIndexChanged.connect(lambda *_: self._preflight())
        for w in (self._chunk,):
            w.valueChanged.connect(lambda *_: self._rebuild())
        for w in (self._delta, self._step):
            w.valueChanged.connect(lambda *_: self._rebuild())
        self._out.textChanged.connect(lambda *_: self._rebuild())
        self._keep.toggled.connect(lambda *_: self._rebuild())
        self._preflight()

    def _pick_dir(self) -> None:
        path = QFileDialog.getExistingDirectory(self, '출력 디렉터리', self._out.text())
        if path:
            self._out.setText(path)

    def _preflight(self) -> None:
        self._rebuild()
        if self.ctx.cfg is None:
            return
        self._findings.set_findings([])
        self._buttons.button(QDialogButtonBox.StandardButton.Ok).setEnabled(False)
        if self._token is not None:
            self.bg.cancel(self._token)
        self._token = self.bg.run(
            lambda cfg, ids, cam, lab: export_preflight(cfg, ids, camera_topic=cam, label=lab),
            self._checked, self._check_failed, self.ctx.cfg, self.ids,
            self._camera.currentData() or '', self._label.currentData())

    def _checked(self, findings) -> None:
        self._token = None
        self._findings.set_findings(findings)
        self._buttons.button(QDialogButtonBox.StandardButton.Ok).setEnabled(
            not self._findings.has_errors())

    def _check_failed(self, text: str) -> None:
        from rdfp.dataset.inspect.findings import LEVEL_ERROR, Finding

        self._token = None
        self._findings.set_findings([Finding(LEVEL_ERROR, 'preflight_failed',
                                             text.strip().splitlines()[-1])])

    def _rebuild(self) -> None:
        self.spec = build_export_command(
            str(self.ctx.config_path), self._out.text().strip(), episode_ids=self.ids,
            label=self._label.currentData(), chunk_len=self._chunk.value(),
            delta_sec=self._delta.value(), step_sec=self._step.value(),
            camera_topic=self._camera.currentData() or '',
            keep_untrustworthy=self._keep.isChecked())
        self._command.setText(self.spec.command_line())


__all__ = ['EpisodeDetailDialog', 'EpisodePanel', 'ExportDialog', 'export_preflight']
