"""하단 작업 패널 — 진행률 · 로그 · **"이 조작과 같은 CLI 명령"** · 중단 · 작업 이력 (설계안 §3 · §6.4).

이력은 `rdfp_gui.history` 의 로컬 SQLite 에 남는다 (Codex Manager 의 Journal 을 가져왔다, 2026-11-23).
"""

from __future__ import annotations

from typing import Optional

from datetime import datetime

from PySide6.QtGui import QGuiApplication
from PySide6.QtWidgets import (QHBoxLayout, QHeaderView, QLabel, QLineEdit, QPlainTextEdit,
                               QProgressBar, QPushButton, QTabWidget, QTreeWidget,
                               QTreeWidgetItem, QVBoxLayout, QWidget)

from ..history import JobHistory
from ..jobs import JobRunner, JobSpec
from .common import brush, mono_font


class JobPanel(QWidget):
    def __init__(self, runner: JobRunner, parent: Optional[QWidget] = None, *,
                 history: Optional[JobHistory] = None) -> None:
        super().__init__(parent)
        self._runner = runner
        self._history = history
        self._job_id: Optional[int] = None
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        top = QHBoxLayout()
        self._title = QLabel('작업 없음')
        self._progress = QProgressBar()
        self._progress.setRange(0, 1)
        self._progress.setValue(0)
        self._progress.setFormat('%v/%m')
        self._status = QLabel('')
        self._cancel = QPushButton('중단')
        self._cancel.setEnabled(False)
        self._cancel.clicked.connect(runner.cancel)
        top.addWidget(self._title)
        top.addWidget(self._progress, 1)
        top.addWidget(self._status)
        top.addWidget(self._cancel)
        layout.addLayout(top)

        cmd = QHBoxLayout()
        cmd.addWidget(QLabel('$'))
        self._command = QLineEdit()
        self._command.setReadOnly(True)
        self._command.setFont(mono_font())
        self._command.setPlaceholderText('이 조작과 같은 CLI 명령이 여기에 보인다')
        copy = QPushButton('복사')
        copy.clicked.connect(self._copy)
        cmd.addWidget(self._command, 1)
        cmd.addWidget(copy)
        layout.addLayout(cmd)

        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setFont(mono_font())
        self._log.setMaximumBlockCount(5000)
        self._past = QTreeWidget()
        self._past.setHeaderLabels(['시작', '작업', '결과', '명령'])
        self._past.setRootIsDecorated(False)
        self._past.header().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self._past.itemDoubleClicked.connect(lambda item, _c: self._command.setText(item.text(3)))
        self._past.setToolTip('더블클릭하면 그 명령을 위 칸에 올린다 (복사해 터미널에서 다시 돌릴 수 있다)')
        tabs = QTabWidget()
        tabs.addTab(self._log, '로그')
        tabs.addTab(self._past, '작업 이력')
        layout.addWidget(tabs, 1)
        self._refresh_history()

        runner.started.connect(self._on_started)
        runner.line.connect(self._log.appendPlainText)
        runner.progress.connect(self._on_progress)
        runner.finished.connect(self._on_finished)

    def show_spec(self, spec: JobSpec) -> None:
        """실행 전에 명령을 보인다 — 사용자가 복사해 터미널에서 돌려도 같은 결과가 나와야 한다."""
        self._command.setText(spec.command_line())
        self._log.clear()

    def _copy(self) -> None:
        QGuiApplication.clipboard().setText(self._command.text())

    def _refresh_history(self) -> None:
        self._past.clear()
        if self._history is None:
            return
        for rec in self._history.recent():
            started = datetime.fromtimestamp(rec.started).strftime('%m-%d %H:%M:%S')
            item = QTreeWidgetItem([started, rec.title, rec.state, rec.command])
            if rec.exit_code not in (None, 0):
                item.setForeground(2, brush('error'))
            if rec.summary:
                item.setToolTip(2, ', '.join(f'{k}={v}' for k, v in rec.summary.items()
                                             if k != 'inserted'))
            self._past.addTopLevelItem(item)

    def _on_started(self, spec: JobSpec) -> None:
        if self._history is not None:
            self._job_id = self._history.start(spec.title, spec.command_line())
        self._title.setText(f'작업: {spec.title}')
        self._status.setText('')
        self._cancel.setEnabled(True)
        if spec.total_hint:
            self._progress.setRange(0, spec.total_hint)
            self._progress.setValue(0)
        else:
            self._progress.setRange(0, 0)     # 분모를 모르면 바쁨 표시

    def _on_progress(self, done: int, total: int, status: str) -> None:
        if total:
            self._progress.setRange(0, total)
            self._progress.setValue(min(done, total))
        self._status.setText(status)

    def _on_finished(self, code: int, summary: Optional[dict]) -> None:
        if self._history is not None and self._job_id is not None:
            self._history.finish(self._job_id, code, summary)
            self._job_id = None
            self._refresh_history()
        self._cancel.setEnabled(False)
        if self._progress.maximum() == 0:
            self._progress.setRange(0, 1)
            self._progress.setValue(1 if code == 0 else 0)
        elif code == 0:
            self._progress.setValue(self._progress.maximum())
        self._title.setText(f'작업 끝: {self._title.text().removeprefix("작업: ")} '
                            f'({"성공" if code == 0 else f"실패 exit {code}"})')
        if summary:
            inserted = summary.get('inserted') or {}
            rows = ' · '.join(f'{k} {v:,}' for k, v in sorted(inserted.items()) if v)
            self._log.appendPlainText(
                f'[gui] 요약: 에피소드 {summary.get("episodes", 0)} · skip {summary.get("skipped", 0)}'
                f' · replace {summary.get("replaced", 0)} · fail {summary.get("failed", 0)}'
                f' · mp4 {summary.get("mp4_files", 0)}' + (f'\n[gui] 행: {rows}' if rows else ''))


__all__ = ['JobPanel']
