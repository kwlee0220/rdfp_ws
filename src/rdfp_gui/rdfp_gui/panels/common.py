"""패널들이 함께 쓰는 작은 위젯 — 점검 목록 · 타이핑 확인 다이얼로그 · 색 · 글꼴."""

from __future__ import annotations

from typing import Iterable, Optional

import html

from PySide6.QtCore import Qt
from PySide6.QtGui import QBrush, QColor, QFont
from PySide6.QtWidgets import (QDialog, QDialogButtonBox, QLabel, QLineEdit, QListWidget,
                               QListWidgetItem, QVBoxLayout, QWidget)

from rdfp.dataset.inspect.findings import LEVEL_ERROR, LEVEL_INFO, LEVEL_WARNING, Finding
from rdfp.dataset.inspect.mapping import (STATUS_MAPPED, STATUS_SPECIAL, STATUS_SUSPECT,
                                          STATUS_UNMAPPED)

#: 상태 · 점검 단계별 색. 매핑 탭의 선과 기호, 점검 목록, 에피소드 행이 같은 색을 쓴다.
COLORS = {
    STATUS_MAPPED: '#2b6cb0',
    STATUS_SPECIAL: '#718096',
    STATUS_UNMAPPED: '#c53030',
    STATUS_SUSPECT: '#dd6b20',
    LEVEL_ERROR: '#c53030',
    LEVEL_WARNING: '#dd6b20',
    LEVEL_INFO: '#2f855a',
    'muted': '#a0aec0',
    'highlight': '#f6ad55',
}

_LEVEL_PREFIX = {LEVEL_ERROR: '✖', LEVEL_WARNING: '⚠', LEVEL_INFO: 'ⓘ'}


def color(key: str) -> QColor:
    return QColor(COLORS.get(key, '#000000'))


def brush(key: str) -> QBrush:
    return QBrush(color(key))


def mono_font() -> QFont:
    font = QFont('Monospace')
    font.setStyleHint(QFont.StyleHint.TypeWriter)
    return font


def esc(text: object) -> str:
    """HTML 로 보일 때 이스케이프."""
    return html.escape(str(text))


class FindingsList(QListWidget):
    """점검 결과 목록. 단계별 색과 접두어로 한눈에 갈린다."""

    def set_findings(self, findings: Iterable[Finding]) -> None:
        self.clear()
        for f in findings:
            item = QListWidgetItem(f'{_LEVEL_PREFIX.get(f.level, "·")} {f.message}')
            item.setForeground(brush(f.level))
            item.setToolTip(f.code)
            item.setData(Qt.ItemDataRole.UserRole, f.level)
            self.addItem(item)
        if self.count() == 0:
            empty = QListWidgetItem('점검 항목 없음')
            empty.setForeground(brush('muted'))
            self.addItem(empty)

    def has_errors(self) -> bool:
        return any(self.item(i).data(Qt.ItemDataRole.UserRole) == LEVEL_ERROR
                   for i in range(self.count()))


class TypedConfirmDialog(QDialog):
    """파괴적 조작의 확인 — 보여 준 값을 **그대로 타이핑**해야 OK 가 켜진다 (설계안 §6.6)."""

    def __init__(self, parent: Optional[QWidget], title: str, text: str, expected: str,
                 extra: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle(title)
        self._expected = expected
        layout = QVBoxLayout(self)
        label = QLabel(text)
        label.setWordWrap(True)
        layout.addWidget(label)
        if extra is not None:
            layout.addWidget(extra)
        layout.addWidget(QLabel(f'계속하려면 아래에 <b>{esc(expected)}</b> 를 입력한다:'))
        self._edit = QLineEdit()
        self._edit.setFont(mono_font())
        self._edit.textChanged.connect(self._check)
        layout.addWidget(self._edit)
        self._buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok
                                         | QDialogButtonBox.StandardButton.Cancel)
        self._buttons.accepted.connect(self.accept)
        self._buttons.rejected.connect(self.reject)
        layout.addWidget(self._buttons)
        self._check('')

    def _check(self, text: str) -> None:
        ok = self._buttons.button(QDialogButtonBox.StandardButton.Ok)
        ok.setEnabled(text.strip() == self._expected)

    @staticmethod
    def ask(parent: Optional[QWidget], title: str, text: str, expected: str,
            extra: Optional[QWidget] = None) -> bool:
        dialog = TypedConfirmDialog(parent, title, text, expected, extra)
        return dialog.exec() == QDialog.DialogCode.Accepted


__all__ = ['COLORS', 'FindingsList', 'TypedConfirmDialog', 'brush', 'color', 'esc', 'mono_font']
