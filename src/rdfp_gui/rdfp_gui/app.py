"""메인 윈도우 — 탭 넷 + 공통 하단 작업 패널 (설계안 §3)."""

from __future__ import annotations

from typing import Optional

import logging
import sys

from PySide6.QtCore import Qt
from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtWidgets import (QApplication, QFileDialog, QMainWindow, QMessageBox, QSplitter,
                               QTabWidget)

from .context import AppContext
from .jobs import JobRunner, JobSpec, workspace_cwd
from .panels.config_panel import ConfigPanel
from .panels.episode_panel import EpisodePanel
from .panels.job_panel import JobPanel
from .panels.mapping_panel import MappingPanel
from .panels.session_panel import SessionPanel
from .store import DataStore
from .workers import Background

_logger = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    def __init__(self, ctx: AppContext) -> None:
        super().__init__()
        self.ctx = ctx
        self.bg = Background(self)
        self.store = DataStore(ctx, self.bg, self)
        self.runner = JobRunner(self)
        self.setWindowTitle('rdfp 데이터셋 관리')
        self.resize(1400, 900)

        self.tabs = QTabWidget()
        self.mapping_panel = MappingPanel(ctx, self.store, self.bg)
        self.episode_panel = EpisodePanel(ctx, self.store, self.bg)
        self.session_panel = SessionPanel(ctx, self.store, self.bg)
        self.config_panel = ConfigPanel(ctx, self.store, self.bg)
        self.tabs.addTab(self.mapping_panel, '① 매핑')
        self.tabs.addTab(self.episode_panel, '② 에피소드')
        self.tabs.addTab(self.session_panel, '③ 세션 (bag)')
        self.tabs.addTab(self.config_panel, '④ 설정')

        self.job_panel = JobPanel(self.runner)
        splitter = QSplitter(Qt.Orientation.Vertical)
        splitter.addWidget(self.tabs)
        splitter.addWidget(self.job_panel)
        splitter.setStretchFactor(0, 4)
        splitter.setStretchFactor(1, 1)
        self.setCentralWidget(splitter)

        for panel in (self.mapping_panel, self.episode_panel, self.session_panel):
            panel.job_requested.connect(self.run_job)
        self.config_panel.config_changed.connect(self._on_config_changed)
        self.runner.finished.connect(self._on_job_finished)
        self.store.status.connect(self.statusBar().showMessage)

        self._build_menu()
        self._update_title()
        self.store.refresh_all()

    # ----- 메뉴 ----------------------------------------------------------------

    def _build_menu(self) -> None:
        file_menu = self.menuBar().addMenu('파일(&F)')
        open_act = QAction('설정 열기…', self)
        open_act.setShortcut(QKeySequence.StandardKey.Open)
        open_act.triggered.connect(self._open_config)
        refresh_act = QAction('새로 고침', self)
        refresh_act.setShortcut(QKeySequence(Qt.Key.Key_F5))
        refresh_act.triggered.connect(self.store.refresh_all)
        quit_act = QAction('끝내기', self)
        quit_act.setShortcut(QKeySequence.StandardKey.Quit)
        quit_act.triggered.connect(self.close)
        file_menu.addAction(open_act)
        file_menu.addAction(refresh_act)
        file_menu.addSeparator()
        file_menu.addAction(quit_act)

        help_menu = self.menuBar().addMenu('도움말(&H)')
        about = QAction('정보', self)
        about.triggered.connect(self._about)
        help_menu.addAction(about)

    def _about(self) -> None:
        QMessageBox.information(
            self, 'rdfp 데이터셋 관리',
            'rosbag2 녹화물 → PostgreSQL 적재 → 학습 샘플 export 를 관리한다.\n\n'
            '적재 · export 는 기존 `ros2 run rdfp import|export` 를 서브프로세스로 돌린다 — 하단 패널의 '
            '명령이 그대로 CLI 다.\n설계: docs/collect/데이터셋_관리_GUI_설계안.md')

    def _open_config(self) -> None:
        start = str(self.ctx.config_path.parent if self.ctx.config_path else '.')
        path, _ = QFileDialog.getOpenFileName(self, 'dataset_config.yaml 열기', start,
                                              'YAML (*.yaml *.yml);;모든 파일 (*)')
        if path:
            self.ctx.switch(path)
            self._on_config_changed()

    def _on_config_changed(self) -> None:
        self._update_title()
        self.config_panel.refresh()
        self.store.refresh_all()

    def _update_title(self) -> None:
        name = str(self.ctx.config_path) if self.ctx.config_path else '(설정 없음)'
        state = '' if self.ctx.loaded else f'  ⚠ {self.ctx.error}'
        self.setWindowTitle(f'rdfp 데이터셋 관리 — {name}{state}')

    # ----- 작업 ----------------------------------------------------------------

    def run_job(self, spec: JobSpec) -> None:
        if self.runner.busy:
            QMessageBox.warning(self, '작업 중',
                                f'이미 `{self.runner.current.title}` 가 돌고 있다. '
                                f'끝나거나 중단한 뒤 다시 시도한다.')
            return
        if spec.cwd is None:
            spec.cwd = workspace_cwd()
        self.job_panel.show_spec(spec)
        self.runner.start(spec)

    def _on_job_finished(self, code: int, summary: Optional[dict]) -> None:
        # 적재 · 삭제 · reindex 뒤에는 DB 와 bag 이 바뀌었으므로 다시 읽는다.
        self.store.refresh_all()
        if code != 0:
            self.statusBar().showMessage(f'작업이 실패했다 (exit {code}) — 하단 로그를 본다')

    def closeEvent(self, event) -> None:  # noqa: N802 — Qt 이름
        if self.runner.busy:
            reply = QMessageBox.question(
                self, '작업 중', '작업이 돌고 있다. 중단하고 끝낼까? (적재는 에피소드 단위로 되돌아간다)',
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            if reply != QMessageBox.StandardButton.Yes:
                event.ignore()
                return
            self.runner.cancel()
        event.accept()


def run(config_path: Optional[str]) -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    ctx = AppContext.load(config_path)
    if ctx.error:
        _logger.warning('config not loaded: %s', ctx.error)
    window = MainWindow(ctx)
    window.show()
    return app.exec()


__all__ = ['MainWindow', 'run']
