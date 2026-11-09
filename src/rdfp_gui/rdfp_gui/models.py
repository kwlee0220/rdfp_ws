"""표 모델 — 행 묶음을 `QTableView` 에 지연 없이 보인다. 페이지는 호출자가 바꿔 끼운다."""

from __future__ import annotations

from typing import Any, Optional, Sequence

import json

from PySide6.QtCore import QAbstractTableModel, QModelIndex, Qt

from rdfp.dataset.inspect.db_inspect import render_cell


class RowsTableModel(QAbstractTableModel):
    """열 이름 + 튜플 행. 셀은 `render_cell` 로 한 줄, 툴팁은 전체 JSON."""

    def __init__(self, columns: Sequence[str] = (), rows: Sequence[Sequence[Any]] = (),
                 parent=None) -> None:
        super().__init__(parent)
        self._columns = list(columns)
        self._rows = [tuple(r) for r in rows]

    def set_page(self, columns: Sequence[str], rows: Sequence[Sequence[Any]]) -> None:
        self.beginResetModel()
        self._columns = list(columns)
        self._rows = [tuple(r) for r in rows]
        self.endResetModel()

    def clear(self) -> None:
        self.set_page([], [])

    def rowCount(self, parent: QModelIndex = QModelIndex()) -> int:  # noqa: N802 — Qt 이름
        return 0 if parent.isValid() else len(self._rows)

    def columnCount(self, parent: QModelIndex = QModelIndex()) -> int:  # noqa: N802
        return 0 if parent.isValid() else len(self._columns)

    def data(self, index: QModelIndex, role: int = Qt.ItemDataRole.DisplayRole) -> Optional[str]:
        if not index.isValid():
            return None
        row = self._rows[index.row()]
        if index.column() >= len(row):
            return None
        value = row[index.column()]
        if role == Qt.ItemDataRole.DisplayRole:
            return render_cell(value)
        if role == Qt.ItemDataRole.ToolTipRole:
            return render_cell(value, max_chars=4000)
        if role == Qt.ItemDataRole.UserRole:
            return value
        return None

    def headerData(self, section: int, orientation: Qt.Orientation,  # noqa: N802
                   role: int = Qt.ItemDataRole.DisplayRole) -> Optional[str]:
        if role != Qt.ItemDataRole.DisplayRole:
            return None
        if orientation == Qt.Orientation.Horizontal:
            return self._columns[section] if section < len(self._columns) else None
        return str(section + 1)

    def value_at(self, row: int, column: int) -> Any:
        return self._rows[row][column]

    def row_json(self, row: int) -> str:
        return json.dumps(dict(zip(self._columns, self._rows[row])), ensure_ascii=False, indent=2,
                          default=str)


__all__ = ['RowsTableModel']
