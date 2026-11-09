"""데이터 브라우저 — bag 메시지와 DB 행을 같은 모양의 표로 페이지 단위로 펼친다 (설계안 §4.2).

bag 쪽은 `read_topic_rows`(stamp 순, split 단위 읽기), DB 쪽은 `fetch_rows`(LIMIT/OFFSET). 둘 다
워커에서 읽는다. bag 쪽에서 에피소드를 고르면 **같은 에피소드 · 같은 토픽의 DB 행 수**를 나란히
보인다 — 빠진 것이 숫자로 보이는 자리다.
"""

from __future__ import annotations

from typing import Optional

from dataclasses import dataclass

from PySide6.QtWidgets import (QAbstractItemView, QComboBox, QDialog, QHBoxLayout, QHeaderView,
                               QLabel, QMessageBox, QPushButton, QSpinBox, QTableView, QVBoxLayout)

from rdfp.dataset.inspect.bag_inspect import read_topic_rows, row_columns
from rdfp.dataset.inspect.db_inspect import all_topics, fetch_rows

from ..context import AppContext, fmt_time
from ..models import RowsTableModel
from ..snapshot import BagSnapshot, db_row_count
from ..workers import Background
from .common import mono_font


@dataclass
class BagSource:
    topic: str
    bag: BagSnapshot
    table: Optional[str]           # 매핑된 테이블 — DB 행 수 비교에 쓴다


@dataclass
class DbSource:
    table: str
    episode_id: Optional[int] = None
    topic_id: Optional[int] = None
    episodes: tuple = ()           # (id, start_ns) 선택지


def _load_bag_page(paths, topic, window, offset, limit):
    rows, more = read_topic_rows(paths, topic, time_range_ns=window, offset=offset, limit=limit)
    columns = ['stamp_ns', 'log_time_ns', *row_columns(rows)]
    data = [(r.stamp_ns, r.log_time_ns, *[r.fields.get(c) for c in columns[2:]]) for r in rows]
    return columns, data, more


def _load_db_page(cfg, table, episode_id, topic_id, offset, limit):
    from rdfp.dataset.db.connection import open_connection

    with open_connection(cfg.db) as conn:
        page = fetch_rows(conn, table, episode_id=episode_id, topic_id=topic_id, offset=offset,
                          limit=limit)
        conn.rollback()
    return list(page.columns), list(page.rows), page.has_more


def _load_db_topics(cfg):
    from rdfp.dataset.db.connection import open_connection

    with open_connection(cfg.db) as conn:
        topics = all_topics(conn)
        conn.rollback()
    return topics


class BrowserDialog(QDialog):
    PAGE_SIZE = 1000

    def __init__(self, ctx: AppContext, bg: Background, source, parent=None) -> None:
        super().__init__(parent)
        self.ctx = ctx
        self.bg = bg
        self.source = source
        self._offset = 0
        self._has_more = False
        self._token: Optional[int] = None
        self.setWindowTitle(self._title())
        self.resize(1100, 650)

        layout = QVBoxLayout(self)
        controls = QHBoxLayout()
        controls.addWidget(QLabel('에피소드'))
        self._episode = QComboBox()
        self._episode.addItem('전체', None)
        controls.addWidget(self._episode)
        self._topic = QComboBox()
        if isinstance(source, DbSource):
            controls.addWidget(QLabel('토픽'))
            self._topic.addItem('전체', None)
            controls.addWidget(self._topic)
        controls.addWidget(QLabel('페이지 크기'))
        self._page_size = QSpinBox()
        self._page_size.setRange(50, 10000)
        self._page_size.setValue(self.PAGE_SIZE)
        controls.addWidget(self._page_size)
        controls.addStretch(1)
        self._prev = QPushButton('◀ 이전')
        self._next = QPushButton('다음 ▶')
        self._prev.clicked.connect(lambda: self._move(-1))
        self._next.clicked.connect(lambda: self._move(+1))
        controls.addWidget(self._prev)
        controls.addWidget(self._next)
        layout.addLayout(controls)

        self._status = QLabel('')
        self._compare = QLabel('')
        self._compare.setStyleSheet('color: #2b6cb0')
        status_row = QHBoxLayout()
        status_row.addWidget(self._status, 1)
        status_row.addWidget(self._compare)
        layout.addLayout(status_row)

        self._model = RowsTableModel()
        self._view = QTableView()
        self._view.setModel(self._model)
        self._view.setFont(mono_font())
        self._view.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._view.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Interactive)
        self._view.horizontalHeader().setDefaultSectionSize(140)
        self._view.doubleClicked.connect(self._show_row)
        layout.addWidget(self._view, 1)

        self._fill_filters()
        self._episode.currentIndexChanged.connect(self._reset)
        self._topic.currentIndexChanged.connect(self._reset)
        self._page_size.valueChanged.connect(self._reset)
        self._reset()

    def _title(self) -> str:
        if isinstance(self.source, BagSource):
            return f'bag 메시지 — {self.source.topic}'
        return f'DB 행 — {self.source.table}'

    def _fill_filters(self) -> None:
        if isinstance(self.source, BagSource):
            for i, be in enumerate(self.source.bag.episodes, start=1):
                label = (f'#{i} {fmt_time(be.log_start_ns)} · {be.duration_ns / 1e9:.1f}s '
                         f'· {be.task_label or "-"}')
                self._episode.addItem(label, (be.start_ns, be.episode.stop_ns))
        else:
            for eid, start_ns in self.source.episodes:
                self._episode.addItem(f'id {eid} · {fmt_time(start_ns)}', eid)
            if self.source.episode_id is not None:
                idx = self._episode.findData(self.source.episode_id)
                if idx >= 0:
                    self._episode.setCurrentIndex(idx)
            if self.ctx.cfg is not None:
                self.bg.run(_load_db_topics, self._set_topics, None, self.ctx.cfg)

    def _set_topics(self, topics) -> None:
        current = self._topic.currentData()
        self._topic.blockSignals(True)
        self._topic.clear()
        self._topic.addItem('전체', None)
        for t in topics:
            self._topic.addItem(f'{t.name} (id {t.id})', t.id)
        preset = self.source.topic_id if current is None else current
        if preset is not None:
            idx = self._topic.findData(preset)
            if idx >= 0:
                self._topic.setCurrentIndex(idx)
        self._topic.blockSignals(False)

    # ----- 페이지 --------------------------------------------------------------

    def _reset(self) -> None:
        self._offset = 0
        self._load()

    def _move(self, direction: int) -> None:
        size = self._page_size.value()
        if direction < 0:
            self._offset = max(0, self._offset - size)
        elif self._has_more:
            self._offset += size
        self._load()

    def _load(self) -> None:
        if self._token is not None:
            self.bg.cancel(self._token)
        self._status.setText('읽는 중…')
        self._prev.setEnabled(False)
        self._next.setEnabled(False)
        size = self._page_size.value()
        if isinstance(self.source, BagSource):
            window = self._episode.currentData()
            paths = self.source.bag.mcap_paths(self.source.topic)
            self._token = self.bg.run(_load_bag_page, self._loaded, self._failed, paths,
                                      self.source.topic, window, self._offset, size)
            self._compare_with_db(window)
        else:
            if self.ctx.cfg is None:
                self._status.setText('설정이 없어 DB 를 읽을 수 없다')
                return
            self._token = self.bg.run(_load_db_page, self._loaded, self._failed, self.ctx.cfg,
                                      self.source.table, self._episode.currentData(),
                                      self._topic.currentData(), self._offset, size)
            self._compare.setText('')

    def _compare_with_db(self, window) -> None:
        self._compare.setText('')
        src = self.source
        if not isinstance(src, BagSource) or src.table is None or self.ctx.cfg is None:
            return
        if not self.ctx.db_configured():
            return
        start_ns = window[0] if window else None

        def done(n) -> None:
            if n is None:
                self._compare.setText('DB 행 수: (못 읽음)')
            else:
                scope = '같은 에피소드 · 같은 토픽' if window else '전체 · 같은 토픽'
                self._compare.setText(f'DB 행 수 {n:,} ({scope}, 테이블 {src.table})')

        self.bg.run(
            lambda cfg, table, topic, s: db_row_count(cfg, table, topic_name=topic, start_ns=s),
            done, None, self.ctx.cfg, src.table, src.topic, start_ns)

    def _loaded(self, result) -> None:
        self._token = None
        columns, rows, more = result
        self._has_more = more
        self._model.set_page(columns, rows)
        self._view.resizeColumnsToContents()
        first = self._offset + 1 if rows else 0
        last = self._offset + len(rows)
        tail = ' (더 있음)' if more else ''
        self._status.setText(f'행 {first:,}–{last:,}{tail}')
        self._prev.setEnabled(self._offset > 0)
        self._next.setEnabled(more)

    def _failed(self, text: str) -> None:
        self._token = None
        last = text.strip().splitlines()[-1] if text.strip() else text
        self._status.setText(f'읽기 실패: {last}')

    def _show_row(self, index) -> None:
        if not index.isValid():
            return
        QMessageBox.information(self, f'행 {self._offset + index.row() + 1}',
                                self._model.row_json(index.row()))


__all__ = ['BagSource', 'BrowserDialog', 'DbSource']
