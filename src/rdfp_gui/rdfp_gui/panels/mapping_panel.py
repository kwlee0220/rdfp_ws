"""① 매핑 탭 — 왼쪽 bag 토픽 · 오른쪽 DB 테이블 · 가운데 매핑 선 · 아래 상세 (설계안 §3.1).

이 탭이 가장 먼저 눈에 띄게 할 것은 **매핑 없음(✖)** 토픽이다 — 적재가 조용히 무시하는 데이터다.
같은 타입의 토픽 둘(`/joint_states` · `/target_joint_cmds`)이 한 테이블로 모이는 것이 선 두 개로
보인다. Import 는 체크한 토픽 · 고른 에피소드로 `ros2 run rdfp import …` 를 만든다.
"""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import QPoint, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import (QAbstractItemView, QComboBox, QDialog, QDialogButtonBox, QHBoxLayout,
                               QHeaderView, QLabel, QLineEdit, QListWidget, QListWidgetItem,
                               QPushButton, QSplitter, QToolBar, QTreeWidget, QTreeWidgetItem,
                               QVBoxLayout, QWidget)

from rdfp.dataset.inspect.bag_inspect import read_schema_text
from rdfp.dataset.inspect.db_inspect import rows_per_topic, table_schema
from rdfp.dataset.inspect.mapping import STATUS_SUSPECT, STATUS_UNMAPPED, TableInfo
from rdfp.dataset.inspect.preflight import check_db, check_import

from ..context import AppContext, fmt_time
from ..jobs import build_import_command
from ..snapshot import BagSnapshot, DbSnapshot, TableEntry, TopicEntry, exported_flag
from ..store import DataStore
from ..workers import Background
from .browser_panel import BagSource, BrowserDialog, DbSource
from .common import COLORS, FindingsList, brush, color, mono_font
from .detail_panel import DetailPanel

_ROLE_KEY = Qt.ItemDataRole.UserRole
_SOURCE_GLYPH = {'builtin': '●', 'media': '◇', 'system': '○'}


class LinkCanvas(QWidget):
    """두 트리 사이에 매핑 선을 긋는다. 선택된 쪽의 선은 굵고 주황이다."""

    def __init__(self, left: QTreeWidget, right: QTreeWidget,
                 parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._left = left
        self._right = right
        self._links: list = []       # (left_item, right_item, status)
        self.setFixedWidth(72)
        for tree in (left, right):
            tree.verticalScrollBar().valueChanged.connect(self.update)
            tree.itemSelectionChanged.connect(self.update)
            tree.itemExpanded.connect(self.update)
            tree.itemCollapsed.connect(self.update)

    def set_links(self, links: list) -> None:
        self._links = list(links)
        self.update()

    def _anchor(self, tree: QTreeWidget, item: QTreeWidgetItem,
                right_edge: bool) -> Optional[QPoint]:
        rect = tree.visualItemRect(item)
        if not rect.isValid() or rect.height() <= 0:
            return None
        x = rect.right() if right_edge else rect.left()
        local = QPoint(x, rect.center().y())
        return self.mapFromGlobal(tree.viewport().mapToGlobal(local))

    def paintEvent(self, event) -> None:  # noqa: N802 — Qt 이름
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        width = self.width()
        for left_item, right_item, status in self._links:
            a = self._anchor(self._left, left_item, True)
            b = self._anchor(self._right, right_item, False)
            if a is None or b is None:
                continue
            ya = max(-10, min(self.height() + 10, a.y()))
            yb = max(-10, min(self.height() + 10, b.y()))
            highlighted = left_item.isSelected() or right_item.isSelected()
            pen = QPen(QColor(COLORS['highlight']) if highlighted else color(status))
            pen.setWidthF(2.6 if highlighted else 1.2)
            painter.setPen(pen)
            path = QPainterPath(QPoint(0, ya))
            path.cubicTo(width * 0.5, ya, width * 0.5, yb, width, yb)
            painter.drawPath(path)
        painter.end()


class MappingPanel(QWidget):
    job_requested = Signal(object)

    def __init__(self, ctx: AppContext, store: DataStore, bg: Background,
                 parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.ctx = ctx
        self.store = store
        self.bg = bg
        self._topic_items: dict = {}
        self._table_items: dict = {}
        self._current_topic: Optional[str] = None
        self._current_table: Optional[str] = None
        self._schema_cache: dict = {}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        toolbar = QToolBar()
        self._refresh_btn = toolbar.addAction('새로 고침', store.refresh_all)
        toolbar.addAction('전체 stamp 진단', self._diagnose_all)
        toolbar.addSeparator()
        self._import_action = toolbar.addAction('Import…', self._open_import)
        toolbar.addSeparator()
        self._bag_label = QLabel('bag: -')
        self._db_label = QLabel('DB: -')
        toolbar.addWidget(self._bag_label)
        toolbar.addWidget(QLabel('   '))
        toolbar.addWidget(self._db_label)
        layout.addWidget(toolbar)

        self._topics = QTreeWidget()
        self._topics.setHeaderLabels(['', '토픽', '타입', '메시지', '세션'])
        self._topics.setRootIsDecorated(False)
        self._topics.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self._topics.header().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self._tables = QTreeWidget()
        self._tables.setHeaderLabels(['', '테이블', '행', '출처', '들어오는 토픽'])
        self._tables.setRootIsDecorated(False)
        self._tables.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self._tables.header().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self._canvas = LinkCanvas(self._topics, self._tables)

        middle = QWidget()
        row = QHBoxLayout(middle)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(0)
        row.addWidget(self._topics, 1)
        row.addWidget(self._canvas)
        row.addWidget(self._tables, 1)

        self.detail = DetailPanel()
        self.detail.browse_requested.connect(self._browse)
        self.detail.diagnose_requested.connect(self._diagnose_current)

        splitter = QSplitter(Qt.Orientation.Vertical)
        splitter.addWidget(middle)
        splitter.addWidget(self.detail)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        layout.addWidget(splitter, 1)

        self._topics.itemSelectionChanged.connect(self._topic_selected)
        self._tables.itemSelectionChanged.connect(self._table_selected)
        store.bag_changed.connect(self._on_bag)
        store.db_changed.connect(self._on_db)
        store.diagnosis_changed.connect(self._on_diagnosis)

    # ----- 채우기 --------------------------------------------------------------

    def _on_bag(self) -> None:
        bag = self.store.bag
        self._topics.clear()
        self._topic_items = {}
        if bag is None:
            return
        self._bag_label.setText(f'bag: {bag.rosbag_dir}'
                                + (f'  ⚠ {bag.error}' if bag.error else ''))
        for entry in bag.topics:
            item = QTreeWidgetItem([entry.glyph, entry.name, entry.summary.type_name,
                                    f'{entry.summary.message_count:,}',
                                    str(entry.summary.session_count)])
            item.setData(0, _ROLE_KEY, entry.name)
            item.setToolTip(1, entry.mapping.reason)
            if entry.checkable:
                item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
                item.setCheckState(0, Qt.CheckState.Checked)
            self._paint_topic(item, entry)
            self._topics.addTopLevelItem(item)
            self._topic_items[entry.name] = item
        self._relink()

    def _paint_topic(self, item: QTreeWidgetItem, entry: TopicEntry) -> None:
        status = entry.mapping.status
        if entry.diagnosis is not None and entry.diagnosis.suspect and status != STATUS_UNMAPPED:
            status = STATUS_SUSPECT
        item.setText(0, entry.glyph)
        flagged = status in (STATUS_UNMAPPED, STATUS_SUSPECT)
        for col in range(5):
            if flagged:
                item.setForeground(col, brush(status))
            elif col == 0:
                item.setForeground(col, brush(status))
            else:
                item.setForeground(col, QBrush())      # 팔레트 기본값 (다크 테마에서도 읽힌다)
        if status == STATUS_UNMAPPED:
            item.setToolTip(0, '매핑 없음 — DB 에 들어가지 않는다')
        elif status == STATUS_SUSPECT:
            item.setToolTip(0, 'stamp 진단에 걸렸다 — 적재에서 빠지는 메시지가 있다')

    def _on_db(self) -> None:
        db = self.store.db
        self._tables.clear()
        self._table_items = {}
        if db is None:
            return
        self._db_label.setText('DB: ' + (f'⚠ {db.error}' if db.error else self.ctx.masked_dsn()))
        for entry in db.tables:
            incoming = ', '.join(entry.incoming)
            if not incoming and entry.info.source == 'builtin':
                incoming = '← 토픽 없음'
            rows = '—' if entry.rows is None else f'{entry.rows:,}'
            item = QTreeWidgetItem([_SOURCE_GLYPH.get(entry.info.source, '·'), entry.name, rows,
                                    entry.info.source, incoming])
            item.setData(0, _ROLE_KEY, entry.name)
            item.setToolTip(1, entry.info.description)
            if entry.rows is None:
                for col in range(5):
                    item.setForeground(col, brush('muted'))
                item.setToolTip(2, 'DB 에 테이블이 없다 (또는 DB 를 못 읽었다)')
            self._tables.addTopLevelItem(item)
            self._table_items[entry.name] = item
        for name in db.extra_tables:
            item = QTreeWidgetItem(['?', name, '', '(registry 밖)', ''])
            item.setData(0, _ROLE_KEY, name)
            for col in range(5):
                item.setForeground(col, brush('muted'))
            self._tables.addTopLevelItem(item)
        self._relink()

    def _relink(self) -> None:
        links = []
        bag = self.store.bag
        if bag is not None:
            for entry in bag.topics:
                table = entry.mapping.table
                left = self._topic_items.get(entry.name)
                right = self._table_items.get(table) if table else None
                if left is not None and right is not None:
                    links.append((left, right, entry.mapping.status))
        self._canvas.set_links(links)

    def _on_diagnosis(self, topic: str) -> None:
        bag = self.store.bag
        item = self._topic_items.get(topic)
        if bag is None or item is None:
            return
        entry = bag.entry(topic)
        if entry is not None:
            self._paint_topic(item, entry)
            if self._current_topic == topic:
                self.detail.show_topic(entry, self._schema_cache.get(topic))

    # ----- 선택 ----------------------------------------------------------------

    def _topic_selected(self) -> None:
        items = self._topics.selectedItems()
        if not items or self.store.bag is None:
            return
        topic = items[0].data(0, _ROLE_KEY)
        entry = self.store.bag.entry(topic)
        if entry is None:
            return
        self._current_topic = topic
        self._current_table = None
        # 반대쪽 짝을 강조한다.
        self._tables.blockSignals(True)
        self._tables.clearSelection()
        partner = self._table_items.get(entry.mapping.table) if entry.mapping.table else None
        if partner is not None:
            partner.setSelected(True)
        self._tables.blockSignals(False)
        self._canvas.update()

        diagnosing = entry.diagnosis is None and entry.mapping.ingested
        self.detail.show_topic(entry, self._schema_cache.get(topic), diagnosing=diagnosing)
        if topic not in self._schema_cache:
            paths = self.store.bag.mcap_paths(topic)
            if paths:
                self.bg.run(read_schema_text, lambda s, t=topic: self._schema_loaded(t, s), None,
                            paths[0], topic)
        if diagnosing:
            self.store.diagnose(topic)

    def _schema_loaded(self, topic: str, schema) -> None:
        self._schema_cache[topic] = schema
        if self._current_topic == topic and self.store.bag is not None:
            entry = self.store.bag.entry(topic)
            if entry is not None:
                diagnosing = entry.diagnosis is None and entry.mapping.ingested
                self.detail.show_topic(entry, schema, diagnosing=diagnosing)

    def _table_selected(self) -> None:
        items = self._tables.selectedItems()
        if not items or self.store.db is None:
            return
        table = items[0].data(0, _ROLE_KEY)
        entry = self.store.db.entry(table)
        if entry is None:
            # registry 밖의 테이블 — 보여 주되 매핑 정보는 없다.
            entry = TableEntry(TableInfo(table, 'unknown', (), 'registry 가 모르는 테이블'), None, [])
        self._current_table = table
        self._current_topic = None
        self._topics.blockSignals(True)
        self._topics.clearSelection()
        for name in entry.incoming:
            partner = self._topic_items.get(name)
            if partner is not None:
                partner.setSelected(True)
        self._topics.blockSignals(False)
        self._canvas.update()

        if entry.rows is None or self.ctx.cfg is None:
            self.detail.show_table(entry, None, None, db_error=self.store.db.error)
            return
        self.detail.show_table(entry)

        def load(cfg, name):
            from rdfp.dataset.db.connection import open_connection

            with open_connection(cfg.db) as conn:
                schema = table_schema(conn, name)
                topics = rows_per_topic(conn, name)
                conn.rollback()
            return schema, topics

        def done(result, name=table, e=entry) -> None:
            if self._current_table == name:
                self.detail.show_table(e, result[0], result[1])

        def fail(text, name=table, e=entry) -> None:
            if self._current_table == name:
                self.detail.show_table(e, None, None, db_error=text.strip().splitlines()[-1])

        self.bg.run(load, done, fail, self.ctx.cfg, table)

    # ----- 조작 ----------------------------------------------------------------

    def _diagnose_current(self) -> None:
        if self._current_topic:
            self.store.diagnose(self._current_topic)
            entry = self.store.bag.entry(self._current_topic) if self.store.bag else None
            if entry is not None:
                self.detail.show_topic(entry, self._schema_cache.get(self._current_topic),
                                       diagnosing=True)

    def _diagnose_all(self) -> None:
        self.store.diagnose_all()

    def _browse(self) -> None:
        if self._current_topic and self.store.bag is not None:
            entry = self.store.bag.entry(self._current_topic)
            if entry is None:
                return
            source = BagSource(self._current_topic, self.store.bag, entry.mapping.table)
            dialog = BrowserDialog(self.ctx, self.bg, source, self)
            dialog.show()
        elif self._current_table:
            episodes = ()
            if self.store.episodes is not None:
                episodes = tuple((s.id, s.start_ns) for s in self.store.episodes.summaries)
            source = DbSource(self._current_table, episodes=episodes)
            dialog = BrowserDialog(self.ctx, self.bg, source, self)
            dialog.show()

    def checked_topics(self) -> list:
        out = []
        for name, item in self._topic_items.items():
            if item.flags() & Qt.ItemFlag.ItemIsUserCheckable \
                    and item.checkState(0) == Qt.CheckState.Checked:
                out.append(name)
        return out

    def _open_import(self) -> None:
        if self.ctx.cfg is None or self.store.bag is None:
            self.store.status.emit('설정 또는 bag 을 아직 읽지 못했다')
            return
        dialog = ImportDialog(self.ctx, self.store.bag, self.store.db, self.checked_topics(),
                              self.bg, self)
        if dialog.exec() == QDialog.DialogCode.Accepted and dialog.spec is not None:
            self.job_requested.emit(dialog.spec)


class ImportDialog(QDialog):
    """토픽(체크) · 에피소드(선택) · 정책 → 사전 점검 → `import` 명령 (설계안 §4.4)."""

    def __init__(self, ctx: AppContext, bag: BagSnapshot, db: Optional[DbSnapshot],
                 topics: list, bg: Background, parent=None) -> None:
        super().__init__(parent)
        self.ctx = ctx
        self.bag = bag
        self.db = db
        self.topics = list(topics)
        self.bg = bg
        self.spec = None
        self._db_findings: list = []
        self.setWindowTitle('Import — rosbag → DB')
        self.resize(900, 700)
        layout = QVBoxLayout(self)

        layout.addWidget(QLabel(f'<b>적재 토픽 {len(self.topics)}개</b> (① 탭에서 체크한 것; '
                                f'/session 은 자동 포함)'))
        topic_list = QListWidget()
        topic_list.setMaximumHeight(110)
        for name in self.topics:
            entry = bag.entry(name)
            QListWidgetItem(f'{entry.glyph if entry else "·"} {name}', topic_list)
        layout.addWidget(topic_list)

        layout.addWidget(QLabel('<b>에피소드</b> — 기본으로 미적재만 체크된다'))
        self._episodes = QTreeWidget()
        self._episodes.setHeaderLabels(['', '#', '시작 (log time)', '길이', 'task', '적재', '세션'])
        self._episodes.setRootIsDecorated(False)
        self._episodes.header().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        keys = db.exported_keys if db is not None else None
        for i, be in enumerate(bag.episodes, start=1):
            flag = exported_flag(be, keys)
            item = QTreeWidgetItem(['', str(i), fmt_time(be.log_start_ns),
                                    f'{be.duration_ns / 1e9:.1f} s', be.task_label or '-', flag,
                                    be.session_name])
            item.setData(0, _ROLE_KEY, be.start_ns)
            item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
            item.setCheckState(0, Qt.CheckState.Unchecked if flag == 'yes'
                               else Qt.CheckState.Checked)
            if flag == 'yes':
                for col in range(7):
                    item.setForeground(col, brush('muted'))
            self._episodes.addTopLevelItem(item)
        layout.addWidget(self._episodes, 2)

        row = QHBoxLayout()
        select_all = QPushButton('전체 선택')
        select_none = QPushButton('전체 해제')
        select_all.clicked.connect(lambda: self._set_all(Qt.CheckState.Checked))
        select_none.clicked.connect(lambda: self._set_all(Qt.CheckState.Unchecked))
        row.addWidget(select_all)
        row.addWidget(select_none)
        row.addStretch(1)
        row.addWidget(QLabel('이미 있는 에피소드:'))
        self._policy = QComboBox()
        for p in ('skip', 'replace', 'error'):
            self._policy.addItem(p, p)
        self._policy.setCurrentIndex(self._policy.findData(ctx.cfg.on_existing_episode))
        row.addWidget(self._policy)
        layout.addLayout(row)

        layout.addWidget(QLabel('<b>사전 점검</b>'))
        self._findings = FindingsList()
        self._findings.setMaximumHeight(140)
        layout.addWidget(self._findings)

        layout.addWidget(QLabel('실행할 명령:'))
        self._command = QLineEdit()
        self._command.setReadOnly(True)
        self._command.setFont(mono_font())
        layout.addWidget(self._command)

        self._buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok
                                         | QDialogButtonBox.StandardButton.Cancel)
        self._buttons.button(QDialogButtonBox.StandardButton.Ok).setText('Import 실행')
        self._buttons.accepted.connect(self.accept)
        self._buttons.rejected.connect(self.reject)
        layout.addWidget(self._buttons)

        self._episodes.itemChanged.connect(lambda *_: self._recompute())
        self._policy.currentIndexChanged.connect(lambda *_: self._recompute())
        self._recompute()
        self.bg.run(check_db, self._db_checked, None, ctx.cfg)

    def _set_all(self, state) -> None:
        self._episodes.blockSignals(True)
        for i in range(self._episodes.topLevelItemCount()):
            self._episodes.topLevelItem(i).setCheckState(0, state)
        self._episodes.blockSignals(False)
        self._recompute()

    def _db_checked(self, findings) -> None:
        self._db_findings = list(findings)
        self._recompute()

    def selected_keys(self) -> list:
        out = []
        for i in range(self._episodes.topLevelItemCount()):
            item = self._episodes.topLevelItem(i)
            if item.checkState(0) == Qt.CheckState.Checked:
                out.append(int(item.data(0, _ROLE_KEY)))
        return out

    def _recompute(self) -> None:
        cfg = self.ctx.cfg
        keys = self.selected_keys()
        windows = [(be.start_ns, be.episode.stop_ns) for be in self.bag.episodes
                   if be.start_ns in set(keys)]
        policy = self._policy.currentData()
        all_ingestible = sorted(t.name for t in self.bag.topics if t.checkable)
        topic_types = self.bag.topic_types()
        findings = check_import(cfg, topic_types=topic_types,
                                selected_topics=[*self.topics, '/session'] if self.topics else [],
                                episode_windows=windows,
                                existing_keys=self.db.exported_keys if self.db else None,
                                policy=policy)
        findings = [*self._db_findings, *findings]
        self._findings.set_findings(findings)

        topics_arg = () if sorted(self.topics) == all_ingestible else tuple(self.topics)
        all_keys = [be.start_ns for be in self.bag.episodes]
        start_arg = () if sorted(keys) == sorted(all_keys) else tuple(keys)
        on_existing = policy if policy != cfg.on_existing_episode else None
        self.spec = build_import_command(str(self.ctx.config_path), start_ns=start_arg,
                                         topics=topics_arg, on_existing=on_existing)
        if not start_arg:
            self.spec.total_hint = len(keys) or None
        self._command.setText(self.spec.command_line())
        ok = self._buttons.button(QDialogButtonBox.StandardButton.Ok)
        ok.setEnabled(bool(keys) and not self._findings.has_errors())


__all__ = ['ImportDialog', 'LinkCanvas', 'MappingPanel']
