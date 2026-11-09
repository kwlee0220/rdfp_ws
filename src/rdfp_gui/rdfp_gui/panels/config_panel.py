"""④ 설정 탭 — `dataset_config.yaml` 보기 · 텍스트 편집 · Pydantic 검증 · topics_file 일치 · DB 연결
시험 (설계안 §4.7). 구조 편집기는 만들지 않는다 — 설정 모델이 바뀔 때마다 GUI 를 고치게 된다."""

from __future__ import annotations

from typing import Optional

import os
import tempfile
from pathlib import Path

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QHBoxLayout, QHeaderView, QLabel, QPlainTextEdit, QPushButton,
                               QSplitter, QTreeWidget, QTreeWidgetItem, QVBoxLayout, QWidget)

from rdfp.dataset.config import load_dataset_config
from rdfp.dataset.inspect.findings import LEVEL_ERROR, LEVEL_INFO, LEVEL_WARNING, Finding
from rdfp.dataset.inspect.preflight import check_db, check_topics_file

from ..context import AppContext
from ..store import DataStore
from ..workers import Background
from .common import FindingsList, brush, mono_font


def validate_text(text: str, config_path: Path) -> list:
    """편집 중인 YAML 을 저장하지 않고 검증한다 — 같은 디렉터리의 임시 파일로 읽어 `topics_file`
    상대 경로가 실제와 같게 풀리게 한다."""
    fd, tmp = tempfile.mkstemp(prefix='.dataset_config_check_', suffix='.yaml',
                               dir=str(config_path.parent))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            f.write(text)
        cfg = load_dataset_config(tmp)
    except (FileNotFoundError, ValueError) as exc:
        return [Finding(LEVEL_ERROR, 'config_invalid', str(exc))]
    finally:
        try:
            os.unlink(tmp)
        except OSError:
            pass
    findings = [Finding(LEVEL_INFO, 'config_ok',
                        f'설정 검증 통과 — 토픽 {len(cfg.topics)}개 · on_existing_episode '
                        f'{cfg.on_existing_episode} · parallelism {cfg.parallelism}')]
    for name in ('rosbag_dir', 'output_mp4_dir', 'recordings_dir'):
        value = getattr(cfg, name)
        if value and not Path(value).is_dir():
            findings.append(Finding(LEVEL_WARNING, f'{name}_missing', f'{name} 디렉터리가 없다: {value}'))
    if cfg.output_mp4_dir is None:
        findings.append(Finding(LEVEL_WARNING, 'output_mp4_dir',
                                'output_mp4_dir 이 없다 — import 가 안 된다'))
    return findings


class ConfigPanel(QWidget):
    config_changed = Signal()

    def __init__(self, ctx: AppContext, store: DataStore, bg: Background,
                 parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.ctx = ctx
        self.store = store
        self.bg = bg
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        head = QHBoxLayout()
        self._path = QLabel('')
        self._path.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        reload_btn = QPushButton('다시 읽기')
        reload_btn.clicked.connect(self.refresh)
        head.addWidget(QLabel('설정 파일:'))
        head.addWidget(self._path, 1)
        head.addWidget(reload_btn)
        layout.addLayout(head)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        self._editor = QPlainTextEdit()
        self._editor.setFont(mono_font())
        left_layout.addWidget(self._editor, 1)
        buttons = QHBoxLayout()
        validate_btn = QPushButton('검증')
        validate_btn.clicked.connect(self._validate)
        save_btn = QPushButton('저장')
        save_btn.clicked.connect(self._save)
        buttons.addWidget(validate_btn)
        buttons.addWidget(save_btn)
        buttons.addStretch(1)
        left_layout.addLayout(buttons)
        splitter.addWidget(left)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.addWidget(QLabel('<b>검증 · 점검</b>'))
        self._findings = FindingsList()
        right_layout.addWidget(self._findings, 1)
        right_layout.addWidget(QLabel('<b>경로</b>'))
        self._paths = QTreeWidget()
        self._paths.setHeaderLabels(['이름', '값', '존재'])
        self._paths.setRootIsDecorated(False)
        self._paths.header().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        right_layout.addWidget(self._paths)
        db_row = QHBoxLayout()
        self._db = QLabel('')
        self._db.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        db_btn = QPushButton('DB 연결 시험')
        db_btn.clicked.connect(self._test_db)
        db_row.addWidget(QLabel('<b>DB</b>'))
        db_row.addWidget(self._db, 1)
        db_row.addWidget(db_btn)
        right_layout.addLayout(db_row)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        layout.addWidget(splitter, 1)
        self.refresh()

    def refresh(self) -> None:
        path = self.ctx.config_path
        self._path.setText(str(path) if path else '(없음)')
        text = ''
        if path and path.is_file():
            text = path.read_text(encoding='utf-8')
        self._editor.setPlainText(text)
        self._show_static()

    def _show_static(self) -> None:
        findings = []
        if self.ctx.error:
            findings.append(Finding(LEVEL_ERROR, 'config_load', self.ctx.error))
        elif self.ctx.cfg is not None:
            findings.append(Finding(LEVEL_INFO, 'config_ok',
                                    f'설정 로드 — 토픽 {len(self.ctx.cfg.topics)}개'))
        if self.ctx.config_path and self.ctx.config_path.is_file():
            findings.extend(check_topics_file(self.ctx.config_path))
        self._findings.set_findings(findings)
        self._paths.clear()
        for name, value, exists in self.ctx.path_rows():
            shown = '' if exists is None else ('예' if exists else '아니오')
            item = QTreeWidgetItem([name, value or '(없음)', shown])
            if exists is False:
                item.setForeground(2, brush('warning'))
            self._paths.addTopLevelItem(item)
        self._db.setText(f'{self.ctx.dsn_env} = {self.ctx.masked_dsn()}')

    def _validate(self) -> None:
        if self.ctx.config_path is None:
            return
        findings = validate_text(self._editor.toPlainText(), self.ctx.config_path)
        findings.extend(check_topics_file(self.ctx.config_path))
        self._findings.set_findings(findings)

    def _save(self) -> None:
        if self.ctx.config_path is None:
            return
        self.ctx.config_path.write_text(self._editor.toPlainText(), encoding='utf-8')
        self.ctx.reload()
        self._show_static()
        self.config_changed.emit()

    def _test_db(self) -> None:
        if self.ctx.cfg is None:
            self._findings.set_findings([Finding(LEVEL_ERROR, 'config_load', '설정이 없다')])
            return
        self._db.setText(f'{self.ctx.dsn_env} = {self.ctx.masked_dsn()}  — 접속 중…')

        def done(findings: list) -> None:
            self._findings.set_findings(findings)
            self._db.setText(f'{self.ctx.dsn_env} = {self.ctx.masked_dsn()}  — '
                             + ('OK' if all(f.level != LEVEL_ERROR for f in findings) else '실패'))

        self.bg.run(check_db, done, None, self.ctx.cfg)


__all__ = ['ConfigPanel', 'validate_text']
