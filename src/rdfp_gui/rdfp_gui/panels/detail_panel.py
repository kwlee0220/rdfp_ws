"""상세 패널 — 토픽을 고르면 메시지 정의 · 통계 · 매핑, 테이블을 고르면 스키마 · 통계 (설계안 §3.2)."""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QHBoxLayout, QLabel, QPlainTextEdit, QPushButton, QTabWidget,
                               QTextBrowser, QVBoxLayout, QWidget)

from rdfp.dataset.inspect.bag_inspect import SchemaText
from rdfp.dataset.inspect.db_inspect import TableSchema
from rdfp.dataset.inspect.mapping import STATUS_MAPPED, STATUS_SPECIAL, STATUS_UNMAPPED

from ..context import fmt_time
from ..snapshot import TableEntry, TopicEntry
from .common import COLORS, esc, mono_font

_STATUS_TEXT = {STATUS_MAPPED: '매핑됨', STATUS_SPECIAL: '특수', STATUS_UNMAPPED: '매핑 없음'}

_NOT_IMPLEMENTED = ('동적 매핑(테이블 생성 · 등록 해제 · 테이블 삭제)은 미구현이다 — 설계안 §4.3 · §9 의 '
                    '열 생성 규칙 확정이 먼저다. 내장 매핑은 export · 재생이 의존해 GUI 에서 지우지 않는다.')


class DetailPanel(QWidget):
    browse_requested = Signal()
    diagnose_requested = Signal()

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self._header = QLabel('상세: (토픽이나 테이블을 고른다)')
        self._header.setStyleSheet('font-weight: bold')
        layout.addWidget(self._header)

        self._tabs = QTabWidget()
        self._summary = QTextBrowser()
        self._summary.setOpenExternalLinks(False)
        self._schema = QPlainTextEdit()
        self._schema.setReadOnly(True)
        self._schema.setFont(mono_font())
        self._tabs.addTab(self._summary, '요약')
        self._tabs.addTab(self._schema, '메시지 정의 / 스키마')
        layout.addWidget(self._tabs, 1)

        buttons = QHBoxLayout()
        self._browse = QPushButton('데이터 보기')
        self._browse.setEnabled(False)
        self._browse.clicked.connect(self.browse_requested)
        self._diagnose = QPushButton('stamp 진단')
        self._diagnose.setEnabled(False)
        self._diagnose.clicked.connect(self.diagnose_requested)
        self._create = QPushButton('테이블 생성…')
        self._create.setEnabled(False)
        self._create.setToolTip(_NOT_IMPLEMENTED)
        self._unmap = QPushButton('매핑 제거')
        self._unmap.setEnabled(False)
        self._unmap.setToolTip(_NOT_IMPLEMENTED)
        for b in (self._browse, self._diagnose, self._create, self._unmap):
            buttons.addWidget(b)
        buttons.addStretch(1)
        layout.addLayout(buttons)

    def clear(self, message: str = '상세: (토픽이나 테이블을 고른다)') -> None:
        self._header.setText(message)
        self._summary.setHtml('')
        self._schema.setPlainText('')
        self._browse.setEnabled(False)
        self._diagnose.setEnabled(False)

    def set_schema_text(self, text: str) -> None:
        self._schema.setPlainText(text)

    # ----- 토픽 ----------------------------------------------------------------

    def show_topic(self, entry: TopicEntry, schema: Optional[SchemaText] = None, *,
                   diagnosing: bool = False) -> None:
        m = entry.mapping
        s = entry.summary
        col = COLORS.get(m.status, '#000')
        self._header.setText(f'상세: {s.name} — {s.type_name}')
        parts = [f'<p><span style="color:{col}; font-size:larger">{entry.glyph}</span> '
                 f'<b>{esc(_STATUS_TEXT.get(m.status, m.status))}</b>']
        if m.table:
            parts.append(f' → 테이블 <code>{esc(m.table)}</code>')
            if m.writer:
                parts.append(f' · writer <code>{esc(m.writer)}</code>')
            if m.reader:
                parts.append(f' · reader <code>{esc(m.reader)}</code>')
        parts.append('</p>')
        parts.append(f'<p>{esc(m.reason)}</p>')
        parts.append(f'<p>메시지 수 <b>{s.message_count:,}</b> · 세션 {s.session_count}'
                     + (f' · 타입이 섞임: <code>{esc(s.type_name)}</code>' if len(s.types) > 1 else '')
                     + '</p>')
        if entry.diagnosis is not None:
            d = entry.diagnosis
            warn = (f' <span style="color:{COLORS["suspect"]}">△ 적재에서 빠지는 메시지가 있다</span>'
                    if d.suspect else '')
            parts.append(f'<p><b>stamp 진단</b>{warn}<br>'
                         + '<br>'.join(esc(line) for line in d.summary_lines()) + '</p>')
            if d.first_stamp_ns is not None:
                parts.append(f'<p style="color:#718096">첫 stamp '
                             f'{esc(fmt_time(d.first_stamp_ns))} · '
                             f'첫 log time {esc(fmt_time(d.first_log_ns))}</p>')
        elif diagnosing:
            parts.append('<p style="color:#718096">stamp 진단 중… (MCAP 을 읽는다)</p>')
        else:
            parts.append('<p style="color:#718096">stamp 진단 전 — [stamp 진단] 을 누른다.</p>')
        self._summary.setHtml(''.join(parts))
        if schema is not None:
            self._schema.setPlainText(f'# {schema.type_name} ({schema.encoding})\n\n{schema.text}')
        else:
            self._schema.setPlainText('(메시지 정의를 읽는 중…)')
        self._browse.setEnabled(True)
        self._diagnose.setEnabled(entry.diagnosis is None and not diagnosing)

    # ----- 테이블 ---------------------------------------------------------------

    def show_table(self, entry: TableEntry, schema: Optional[TableSchema] = None,
                   db_topics: Optional[list] = None, *, db_error: Optional[str] = None) -> None:
        info = entry.info
        self._header.setText(f'상세: 테이블 {info.name}')
        rows = '—' if entry.rows is None else f'{entry.rows:,}'
        parts = [f'<p><b>{esc(info.name)}</b> · 출처 {esc(info.source)} · 행 {rows}</p>',
                 f'<p>{esc(info.description)}</p>']
        if info.type_names:
            parts.append('<p>타입: ' + ', '.join(f'<code>{esc(t)}</code>' for t in info.type_names)
                         + '</p>')
        if entry.incoming:
            parts.append('<p>bag 에서 들어오는 토픽: '
                         + ', '.join(f'<code>{esc(t)}</code>' for t in entry.incoming) + '</p>')
        elif info.source == 'builtin':
            parts.append(f'<p style="color:{COLORS["muted"]}">bag 에 이 테이블로 가는 토픽이 없다.</p>')
        if db_topics:
            lines = [f'&nbsp;&nbsp;<code>{esc(tr.topic.name)}</code> (id {tr.topic.id})'
                     f' — {tr.rows:,}' for tr in db_topics]
            parts.append('<p>DB 의 topic_id 별 행 수:<br>' + '<br>'.join(lines) + '</p>')
        if db_error:
            parts.append(f'<p style="color:{COLORS["error"]}">DB: {esc(db_error)}</p>')
        if entry.rows is None and not db_error:
            parts.append(f'<p style="color:{COLORS["warning"]}">DB 에 이 테이블이 없다 — '
                         f'<code>ros2 run rdfp init-db</code> 로 만든다.</p>')
        self._summary.setHtml(''.join(parts))
        self._schema.setPlainText(schema.as_text() if schema is not None
                                  else ('(스키마를 읽는 중…)' if not db_error else ''))
        self._browse.setEnabled(entry.rows is not None)
        self._diagnose.setEnabled(False)


__all__ = ['DetailPanel']
