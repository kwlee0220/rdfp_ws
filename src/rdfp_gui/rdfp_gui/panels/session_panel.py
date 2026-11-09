"""③ 세션(bag) 탭 — 날짜 · 세션 · split 트리. **미완료 세션이 보인다** (설계안 §2 첫 함정)."""

from __future__ import annotations

from typing import Optional

import shutil

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QAbstractItemView, QHBoxLayout, QHeaderView, QLabel, QToolBar,
                               QTreeWidget, QTreeWidgetItem, QVBoxLayout, QWidget)

from rdfp.dataset.inspect.bag_inspect import SessionInfo

from ..context import AppContext, fmt_duration, fmt_time
from ..jobs import build_reindex_command
from ..snapshot import session_export_status
from ..store import DataStore
from ..workers import Background
from .common import TypedConfirmDialog, brush

_ROLE_PATH = Qt.ItemDataRole.UserRole
_ROLE_FINAL = Qt.ItemDataRole.UserRole + 1


class SessionPanel(QWidget):
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
        toolbar.addAction('새로 고침', lambda: store.refresh_bag(then_db=True))
        toolbar.addSeparator()
        self._reindex = toolbar.addAction('reindex (metadata.yaml 복구)', self._do_reindex)
        self._reindex.setEnabled(False)
        self._delete = toolbar.addAction('세션 디렉터리 삭제…', self._do_delete)
        self._delete.setEnabled(False)
        toolbar.addSeparator()
        self._label = QLabel('rosbag_dir: -')
        toolbar.addWidget(self._label)
        layout.addWidget(toolbar)

        self._tree = QTreeWidget()
        self._tree.setHeaderLabels(['이름', '상태', '메시지', '시작', '길이', 'split', '적재', '비고'])
        self._tree.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self._tree.header().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self._tree.itemSelectionChanged.connect(self._selection_changed)
        layout.addWidget(self._tree, 1)

        hint = QHBoxLayout()
        self._hint = QLabel('미완료 세션은 적재 대상이 아니다 — 녹화가 끝난 것이 확실하면 reindex 로 '
                            'metadata.yaml 을 되살린다. 세션 삭제는 bag 만 지운다 (DB 는 ② 탭).')
        self._hint.setWordWrap(True)
        hint.addWidget(self._hint)
        layout.addLayout(hint)

        store.bag_changed.connect(self._populate)
        store.db_changed.connect(self._populate)

    def _populate(self) -> None:
        self._tree.clear()
        bag = self.store.bag
        if bag is None:
            return
        self._label.setText(f'rosbag_dir: {bag.rosbag_dir}'
                            + (f'  ⚠ {bag.error}' if bag.error else ''))
        keys = self.store.db.exported_keys if self.store.db else None
        by_date: dict = {}
        for s in bag.sessions:
            by_date.setdefault(s.date_dir, []).append(s)
        for date, sessions in sorted(by_date.items()):
            messages = sum(s.message_count for s in sessions)
            mcaps = sum(len(s.mcap_paths) for s in sessions)
            date_item = QTreeWidgetItem([date, '', str(messages), '', '', str(mcaps), '', ''])
            self._tree.addTopLevelItem(date_item)
            for s in sessions:
                date_item.addChild(self._session_item(s, bag.episodes, keys))
            date_item.setExpanded(True)

    def _session_item(self, s: SessionInfo, episodes, keys) -> QTreeWidgetItem:
        status = '완료' if s.finalized else '미완료'
        item = QTreeWidgetItem([s.name, status, f'{s.message_count:,}', fmt_time(s.start_ns),
                                fmt_duration(s.duration_ns), str(len(s.mcap_paths)),
                                session_export_status(s, episodes, keys), s.note])
        item.setData(0, _ROLE_PATH, str(s.path))
        item.setData(0, _ROLE_FINAL, s.finalized)
        if not s.finalized:
            for col in range(8):
                item.setForeground(col, brush('warning'))
            item.setToolTip(7, s.note)
        for sp in s.splits:
            child = QTreeWidgetItem([sp.path.name, 'split', f'{sp.message_count:,}',
                                     fmt_time(sp.start_ns), fmt_duration(sp.end_ns - sp.start_ns),
                                     '', '', ''])
            child.setData(0, _ROLE_PATH, None)
            item.addChild(child)
        if not s.finalized:
            for p in s.mcap_paths:
                child = QTreeWidgetItem([p.name, 'mcap', '', '', '', '', '', 'metadata.yaml 없음'])
                child.setData(0, _ROLE_PATH, None)
                for col in range(8):
                    child.setForeground(col, brush('muted'))
                item.addChild(child)
        return item

    def _selected_session(self) -> Optional[QTreeWidgetItem]:
        items = self._tree.selectedItems()
        if not items or items[0].data(0, _ROLE_PATH) is None:
            return None
        return items[0]

    def _selection_changed(self) -> None:
        item = self._selected_session()
        self._delete.setEnabled(item is not None)
        self._reindex.setEnabled(item is not None and not item.data(0, _ROLE_FINAL))

    def _do_reindex(self) -> None:
        item = self._selected_session()
        if item is not None:
            self.job_requested.emit(build_reindex_command(item.data(0, _ROLE_PATH)))

    def _do_delete(self) -> None:
        item = self._selected_session()
        if item is None:
            return
        path = item.data(0, _ROLE_PATH)
        name = item.text(0)
        text = (f'bag 세션 디렉터리 <code>{path}</code> 를 통째로 지운다 — 메시지 {item.text(2)} 건, '
                f'split {item.text(5)}개. DB 의 에피소드는 그대로 남는다.')
        if not TypedConfirmDialog.ask(self, '세션 삭제', text, name):
            return

        def done(_result) -> None:
            self.store.status.emit(f'세션 삭제: {name}')
            self.store.refresh_bag(then_db=True)

        def fail(text: str) -> None:
            self.store.status.emit(f'세션 삭제 실패: {text.strip().splitlines()[-1]}')

        self.bg.run(shutil.rmtree, done, fail, path)


__all__ = ['SessionPanel']
