from PySide6.QtCore import QAbstractTableModel, QSortFilterProxyModel, Qt
from PySide6.QtGui import QColor


class RecordsProxyModel(QSortFilterProxyModel):
    """Number visible rows independently of source order and filtering."""
    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if orientation == Qt.Vertical and role == Qt.DisplayRole:
            return str(section + 1) if 0 <= section < self.rowCount() else None
        return super().headerData(section, orientation, role)


class RecordsModel(QAbstractTableModel):
    """A bounded page of records. Database/MCAP I/O stays in worker processes."""
    def __init__(self, columns, parent=None):
        super().__init__(parent)
        self.columns, self.rows, self.highlight = columns, [], set()

    def replace(self, rows):
        self.beginResetModel()
        self.rows = rows
        self.highlight = set()
        self.endResetModel()

    def rowCount(self, parent=None):
        return len(self.rows)

    def columnCount(self, parent=None):
        return len(self.columns)

    def data(self, index, role=Qt.DisplayRole):
        if not index.isValid():
            return None
        if role == Qt.BackgroundRole and index.row() in self.highlight:
            return QColor('#d8eaff')
        value = self.rows[index.row()].get(self.columns[index.column()][0], '')
        if role in (Qt.DisplayRole, Qt.ToolTipRole):
            text = '' if value is None else str(value)
            return text if role == Qt.ToolTipRole else text[:180]
        return None

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role == Qt.DisplayRole:
            return self.columns[section][1] if orientation == Qt.Horizontal else str(section+1)

    def mark(self, predicate):
        self.highlight = {i for i, row in enumerate(self.rows) if predicate(row)}
        if self.rows:
            self.dataChanged.emit(self.index(0, 0), self.index(len(self.rows)-1, len(self.columns)-1))
