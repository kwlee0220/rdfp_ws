"""Dataset Manager UI. Every filesystem/DB operation runs outside the Qt thread."""
from __future__ import annotations

import base64
from pathlib import Path
from queue import Empty

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import (QAbstractItemView, QCheckBox, QComboBox, QDialog,
    QDialogButtonBox, QDoubleSpinBox, QFileDialog, QFormLayout, QHBoxLayout,
    QInputDialog, QLabel, QLineEdit, QMainWindow, QMessageBox, QPlainTextEdit,
    QPushButton, QSpinBox, QSplitter, QTableView, QTabWidget, QVBoxLayout, QWidget)

from ..management.common import dumps, parse_ids
from ..management.jobs import Journal, Worker
from .models import RecordsModel, RecordsProxyModel


class RecordsView(QWidget):
    def __init__(self, title, columns):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel(title))
        search = QLineEdit(placeholderText='목록 검색')
        layout.addWidget(search)
        self.model = RecordsModel(columns, self)
        self.proxy = RecordsProxyModel(self)
        self.proxy.setSourceModel(self.model)
        self.proxy.setFilterKeyColumn(-1)
        self.proxy.setFilterCaseSensitivity(Qt.CaseInsensitive)
        search.textChanged.connect(self.proxy.setFilterFixedString)
        self.table = QTableView()
        self.table.setModel(self.proxy)
        # Preserve the catalog/query order until the user selects a sort column.
        self.table.horizontalHeader().setSortIndicator(-1, Qt.AscendingOrder)
        self.table.setSortingEnabled(True)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.table)

    def replace(self, rows):
        self.model.replace(rows)
        self.table.resizeColumnsToContents()

    def selected(self):
        return [self.model.rows[self.proxy.mapToSource(i).row()]
                for i in self.table.selectionModel().selectedRows()]


class ExportDialog(QDialog):
    def __init__(self, ids, parent):
        super().__init__(parent)
        self.setWindowTitle('학습 데이터셋 Export')
        layout = QFormLayout(self)
        self.ids = QLineEdit(','.join(str(x) for x in ids))
        layout.addRow('에피소드 DB ID (예: 1-5,8)', self.ids)
        self.output = QLineEdit(placeholderText='아직 존재하지 않는 새 디렉터리')
        layout.addRow('출력 경로', self.output)
        self.format = QComboBox()
        self.format.addItems(['npz', 'lerobot_v3'])
        layout.addRow('형식', self.format)
        self.camera = QLineEdit(placeholderText='카메라가 하나면 생략 가능')
        layout.addRow('카메라 토픽', self.camera)
        self.label = QComboBox()
        self.label.addItems(['joint_plan', 'joint_command'])
        layout.addRow('Action 원천', self.label)
        self.chunk = QSpinBox(minimum=1, maximum=1000, value=20)
        layout.addRow('NPZ chunk 길이', self.chunk)
        self.delta = QDoubleSpinBox(minimum=0, maximum=60, value=.1, decimals=3)
        self.step = QDoubleSpinBox(minimum=.001, maximum=60, value=.1, decimals=3)
        layout.addRow('NPZ 첫 행동 오프셋 (초)', self.delta)
        layout.addRow('NPZ 행동 간격 (초)', self.step)
        self.fps = QSpinBox(minimum=1, maximum=120, value=10)
        self.skew = QDoubleSpinBox(minimum=.001, maximum=10, value=.2, decimals=3)
        layout.addRow('LeRobot 출력 fps', self.fps)
        layout.addRow('LeRobot 최대 영상 시차 (초)', self.skew)
        self.copy = QCheckBox('NPZ에 영상을 복사하여 포함')
        self.copy.setChecked(True)
        layout.addRow(self.copy)
        self.keep = QCheckBox('종료 이벤트가 없어 추정한 레이블도 포함')
        layout.addRow(self.keep)
        note = QLabel('LeRobot은 고정 시간격자로 다시 샘플링합니다.\n'
                      '학습 입력: 팔 관절 + 그리퍼 폭 / 행동: 팔 목표 + open·close·grasp one-hot.\n'
                      '검증 실패·취소 시 완성본을 게시하지 않습니다. 기존 결과는 덮어쓰지 않습니다.')
        note.setWordWrap(True)
        layout.addRow(note)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

    def request(self):
        if not self.output.text().strip():
            raise ValueError('출력 경로를 지정하세요.')
        return {'episode_ids': parse_ids(self.ids.text()), 'output': self.output.text().strip(), 'options': {
            'format': self.format.currentText(), 'label': self.label.currentText(),
            'camera_topic': self.camera.text().strip(), 'chunk_len': self.chunk.value(),
            'delta_sec': self.delta.value(), 'step_sec': self.step.value(), 'fps': self.fps.value(),
            'max_skew_sec': self.skew.value(), 'copy_media': self.copy.isChecked(),
            'keep_untrustworthy': self.keep.isChecked()}}


class ManagerWindow(QMainWindow):
    def __init__(self, config_path='', state_dir=None):
        super().__init__()
        self.setWindowTitle('RDFP Dataset Manager')
        self.resize(1350, 920)
        self.worker = None
        self.callback = None
        self.terminal = None
        self.journal = Journal(Path(state_dir or Path.home()/'.local/state/rdfp-dataset-manager')/'jobs.sqlite3')
        self.browser_request = None
        self.page = 0
        self.ep_page = 0
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        paths = QHBoxLayout()
        self.config = QLineEdit(str(config_path))
        paths.addWidget(QLabel('설정 YAML'))
        paths.addWidget(self.config)
        self._button(paths, '설정 선택', self.choose_config)
        self._button(paths, '관리 DB 준비', self.initialize)
        layout.addLayout(paths)
        sources = QHBoxLayout()
        self.source = QComboBox()
        self.source.setEditable(True)
        sources.addWidget(QLabel('녹화본'))
        sources.addWidget(self.source, 1)
        self._button(sources, '폴더 선택', self.choose_source)
        self._button(sources, 'MCAP 선택', self.choose_file)
        self._button(sources, '토픽 조회', self.refresh_topics)
        self._button(sources, 'DB 조회', self.refresh_db)
        layout.addLayout(sources)
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs, 3)
        self._mapping_tab()
        self._episodes_tab()
        self._media_tab()
        self.history = RecordsView('최근 200개 작업', [('action', '작업'), ('state', '상태'), ('started', '시작 epoch'), ('message', '결과')])
        self.tabs.addTab(self.history, '작업 이력')
        self.history.replace(self.journal.list())
        self.detail = QPlainTextEdit(readOnly=True)
        self.detail.setPlaceholderText('행을 선택하면 상세 정보를 보여줍니다. 데이터 조회는 원본·DB를 변경하지 않습니다.')
        self.detail.setMaximumHeight(210)
        layout.addWidget(self.detail, 1)
        self.log = QPlainTextEdit(readOnly=True)
        self.log.setMaximumHeight(100)
        self.log.setMaximumBlockCount(1500)
        layout.addWidget(self.log)
        bottom = QHBoxLayout()
        self.status = QLabel('준비')
        bottom.addWidget(self.status, 1)
        self.cancel = QPushButton('작업 취소')
        self.cancel.setEnabled(False)
        self.cancel.clicked.connect(self.cancel_job)
        bottom.addWidget(self.cancel)
        layout.addLayout(bottom)
        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.poll)
        self.timer.start()
        if config_path:
            QTimer.singleShot(0, lambda: self.start('config', {}, self.on_config))

    def _button(self, layout, text, slot):
        button = QPushButton(text)
        button.clicked.connect(slot)
        layout.addWidget(button)
        return button

    def _mapping_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        splitter = QSplitter(Qt.Horizontal)
        self.topics = RecordsView('bag에 저장된 토픽 (Ctrl/Shift로 여러 행 선택)', [
            ('topic', '토픽'), ('type', '메시지 타입'), ('count', 'bag 메시지 수'),
            ('status', '매핑 상태'), ('table_name', '대상 테이블')])
        self.tables = RecordsView('PostgreSQL 테이블', [('table_name', '테이블'), ('estimated_rows', '추정 행 수'), ('bytes', '크기 bytes')])
        splitter.addWidget(self.topics)
        splitter.addWidget(self.tables)
        splitter.setSizes([730, 520])
        layout.addWidget(splitter)
        self.topics.table.selectionModel().selectionChanged.connect(self.topic_selected)
        self.tables.table.selectionModel().selectionChanged.connect(self.table_selected)
        actions = QHBoxLayout()
        self._button(actions, '메시지 보기', self.browse_topic)
        self._button(actions, '연결된 DB 데이터', self.browse_mapped_topic)
        self._button(actions, '테이블 스키마', self.describe_table)
        self._button(actions, '테이블 데이터', self.browse_table)
        self._button(actions, '테이블 생성·등록', self.register)
        self._button(actions, '매핑 해제', lambda: self.active_mapping(False))
        self._button(actions, '다시 연결', lambda: self.active_mapping(True))
        self._button(actions, '전용 테이블 삭제', self.drop_custom)
        layout.addLayout(actions)
        imports = QHBoxLayout()
        self.replace_import = QCheckBox('이미 적재된 선택 토픽 교체')
        imports.addWidget(self.replace_import)
        self._button(imports, '선택 토픽·영상 Import...', self.plan_import)
        self._button(imports, 'bag 에피소드 확인', lambda: self.start('bag_episodes', {'source': self.source.currentText()}, self.show_detail))
        layout.addLayout(imports)
        self.tabs.addTab(page, '토픽·테이블')
        browser = QWidget()
        bl = QVBoxLayout(browser)
        self.rows = RecordsView('데이터: 한 번에 200건, 정렬·검색은 현재 페이지에 적용', [('value', '값')])
        self.rows.table.selectionModel().selectionChanged.connect(lambda: self.show_detail(self.rows.selected()))
        bl.addWidget(self.rows)
        controls = QHBoxLayout()
        self._button(controls, '이전 페이지', lambda: self.browse_page(-1))
        self._button(controls, '다음 페이지', lambda: self.browse_page(1))
        self.page_label = QLabel('')
        controls.addWidget(self.page_label)
        bl.addLayout(controls)
        self.tabs.addTab(browser, '데이터')

    def _episodes_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        self.episodes = RecordsView('DB 에피소드 (NULL 성패는 미판정)', [
            ('id', 'DB ID'), ('task_label', '작업'), ('start_sec', '시작 sec'),
            ('start_nanosec', '시작 ns'), ('stop_sec', '종료 sec'), ('success', '성공 여부')])
        layout.addWidget(self.episodes)
        actions = QHBoxLayout()
        self._button(actions, '목록 조회', lambda: self.load_episodes(0))
        self._button(actions, '이전', lambda: self.load_episodes(-1))
        self._button(actions, '다음', lambda: self.load_episodes(1))
        self._button(actions, '선택 에피소드 상세', self.episode_detail)
        self._button(actions, 'Export...', self.export)
        layout.addLayout(actions)
        self.tabs.addTab(page, '에피소드')

    def _media_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        self.media = RecordsView('별도 레코더 영상 (선택한 카메라 토픽만 Import)', [
            ('topic', '토픽'), ('episode_start_ns', '에피소드 시작 ns'), ('frames', '프레임 수'), ('exists', '파일 존재'), ('mp4', 'MP4')])
        layout.addWidget(self.media)
        actions = QHBoxLayout()
        self._button(actions, '레코더 목록 조회', lambda: self.start('media', {}, self.media.replace))
        self.frame = QSpinBox(minimum=0, maximum=2147483647)
        actions.addWidget(QLabel('프레임 번호'))
        actions.addWidget(self.frame)
        self._button(actions, '프레임 보기', self.preview_frame)
        layout.addLayout(actions)
        self.image = QLabel('영상을 선택하고 프레임 번호를 지정하세요.')
        self.image.setAlignment(Qt.AlignCenter)
        self.image.setMinimumHeight(180)
        layout.addWidget(self.image)
        self.tabs.addTab(page, '영상')

    def selected_one(self, view):
        rows = view.selected()
        if len(rows) != 1:
            QMessageBox.information(self, '선택', '항목 하나를 선택해 주세요.')
            return None
        return rows[0]

    def show_detail(self, value):
        self.detail.setPlainText(dumps(value))

    def choose_config(self):
        path, _ = QFileDialog.getOpenFileName(self, 'dataset 설정', self.config.text(), 'YAML (*.yaml *.yml)')
        if path:
            self.config.setText(path)
            self.start('config', {}, self.on_config)

    def on_config(self, cfg):
        self.show_detail(cfg)
        self.source.setEditText(cfg['rosbag_dir'])
        self.status.setText(f'DB: {cfg["dsn_env"]} / {cfg["schema"]} · '+('설정됨' if cfg['db_configured'] else 'DSN 환경변수 없음'))

    def choose_source(self):
        root = QFileDialog.getExistingDirectory(self, 'bag 또는 녹화 루트 폴더')
        if root:
            self.start('discover', {'root': root}, self.set_sources)

    def set_sources(self, paths):
        self.source.clear()
        self.source.addItems(paths)
        self.status.setText(f'녹화본 {len(paths)}개. 목록에서 선택 후 토픽 조회를 누르세요.')

    def choose_file(self):
        path, _ = QFileDialog.getOpenFileName(self, '완성된 MCAP 선택', '', 'MCAP (*.mcap)')
        if path:
            self.source.setEditText(path)

    def refresh_topics(self):
        self.start('scan', {'source': self.source.currentText()}, self.topics.replace)

    def refresh_db(self):
        self.start('tables', {}, self.tables.replace)

    def topic_selected(self, *_):
        selected = self.topics.selected()
        targets = {r.get('table_name') for r in selected}
        self.tables.model.mark(lambda r: r['table_name'] in targets)
        self.show_detail(selected)

    def table_selected(self, *_):
        names = {r['table_name'] for r in self.tables.selected()}
        self.topics.model.mark(lambda r: r.get('table_name') in names)

    def describe_table(self):
        row = self.selected_one(self.tables)
        if row:
            self.start('table_schema', {'table': row['table_name']}, self.show_detail)

    def browse_topic(self):
        row = self.selected_one(self.topics)
        if row:
            self.browser_request = ('bag_rows', {'source': self.source.currentText(), 'topic': row['topic']})
            self.page = 0
            self.browse_page(0)

    def browse_table(self):
        row = self.selected_one(self.tables)
        if row:
            text, ok = QInputDialog.getText(self, '에피소드 필터', '에피소드 DB ID (전체 조회는 빈 값)')
            if ok:
                if text and not text.isdigit():
                    QMessageBox.warning(self, '입력', '양의 정수 ID를 입력하세요.')
                    return
                self.browser_request = ('table_rows', {'table': row['table_name'], 'episode_id': int(text) if text else None})
                self.page = 0
                self.browse_page(0)

    def browse_mapped_topic(self):
        row = self.selected_one(self.topics)
        if row and row.get('table_name') and row['topic'] != '/session':
            self.browser_request = ('table_rows', {'table': row['table_name'], 'topic_name': row['topic']})
            self.page = 0
            self.browse_page(0)

    def browse_page(self, delta):
        if self.browser_request:
            self.page = max(0, self.page + delta)
            action, args = self.browser_request
            self.start(action, {**args, 'offset': self.page*200, 'limit': 200}, self.show_rows)

    def show_rows(self, rows):
        columns = list(rows[0]) if rows else ['value']
        self.rows.model.beginResetModel()
        self.rows.model.columns = [(c, c) for c in columns]
        self.rows.model.rows = rows
        self.rows.model.endResetModel()
        self.rows.table.resizeColumnsToContents()
        self.page_label.setText(f'{self.page+1}페이지 · {len(rows)}건')
        self.tabs.setCurrentIndex(1)

    def initialize(self):
        if QMessageBox.question(self, '관리 DB 준비', '설정 YAML의 DB schema에 dataset·관리 테이블을 추가합니다.\n'
                                '기존 행은 삭제하지 않습니다. 실행할까요?') == QMessageBox.Yes:
            self.start('initialize', {}, self.show_detail)

    def register(self):
        topic = self.selected_one(self.topics)
        if not topic:
            return
        name, ok = QInputDialog.getText(self, '범용 테이블 등록', '테이블명 (custom_으로 시작)', text='custom_messages')
        if not ok:
            return
        exists = any(r['table_name'] == name for r in self.tables.model.rows)
        if QMessageBox.question(self, '등록 확인', f'{topic["topic"]} → {name}\n'
                '메시지는 JSONB와 원본 CDR로 저장합니다. 학습 feature 등록과는 별개입니다.\n'
                + ('기존 Manager 테이블을 공유합니다.' if exists else '새 테이블을 생성합니다.')) == QMessageBox.Yes:
            self.start('register', {'topic': topic, 'table': name, 'create': not exists}, self.show_detail)

    def active_mapping(self, active):
        topic = self.selected_one(self.topics)
        if topic:
            self.start('mapping_active', {'topic': topic, 'active': active}, self.show_detail)

    def drop_custom(self):
        topic = self.selected_one(self.topics)
        if not topic:
            return
        table = topic.get('table_name', '')
        text, ok = QInputDialog.getText(self, '전용 테이블 삭제',
            f'{table}의 모든 행과 테이블을 삭제합니다. 복구하려면 별도 백업이 필요합니다.\n'
            '공유·기본 테이블은 삭제할 수 없습니다. 확인할 테이블명을 입력하세요.')
        if ok and text == table and table:
            self.start('drop_custom', {'topic': topic, 'confirmed_table': text}, self.show_detail)

    def plan_import(self):
        args = {'source': self.source.currentText(), 'selected_topics': [r['topic'] for r in self.topics.selected()],
                'media_topics': sorted({r['topic'] for r in self.media.selected()}), 'replace': self.replace_import.isChecked()}
        self.start('plan_import', args, self.confirm_import)

    def confirm_import(self, plan):
        self.show_detail(plan)
        dialog = QMessageBox(self)
        dialog.setWindowTitle('Import 계획 확인')
        dialog.setText(f'에피소드 {len(plan["episodes"])}개 · 메시지 {sum(plan["counts"].values())}건 · '
                       f'영상 {len(plan["media"])}개\n선택 토픽 교체: {plan["replace"]}\n'
                       f'구간 밖 제외: {plan["excluded"]["outside_episode"]}건\n'
                       '원본 bag·MP4는 삭제하지 않습니다. 실행할까요?')
        dialog.setDetailedText(dumps(plan))
        dialog.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        dialog.setDefaultButton(QMessageBox.No)
        if dialog.exec() == QMessageBox.Yes:
            self.start('import', {'plan': plan}, self.show_detail)

    def load_episodes(self, delta):
        self.ep_page = max(0, self.ep_page+delta) if delta else 0
        self.start('episodes', {'offset': self.ep_page*200}, self.episodes.replace)

    def episode_detail(self):
        row = self.selected_one(self.episodes)
        if row:
            self.start('episode_detail', {'episode_id': row['id']}, self.show_detail)

    def export(self):
        dialog = ExportDialog([r['id'] for r in self.episodes.selected()], self)
        if dialog.exec() == QDialog.Accepted:
            try:
                args = dialog.request()
            except ValueError as exc:
                QMessageBox.warning(self, '입력', str(exc))
                return
            self.start('export', args, self.show_detail)

    def preview_frame(self):
        row = self.selected_one(self.media)
        if row:
            self.start('preview_frame', {'path': row['mp4'], 'frame_index': self.frame.value()}, self.show_frame)

    def show_frame(self, result):
        pixmap = QPixmap()
        pixmap.loadFromData(base64.b64decode(result['png']))
        self.image.setPixmap(pixmap.scaled(850, 350, Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def start(self, action, args, callback):
        if self.worker:
            self.status.setText('작업이 실행 중입니다. 완료 또는 취소 후 다시 실행하세요.')
            return
        self.config.setEnabled(False)
        self.source.setEnabled(False)
        self.cancel.setEnabled(True)
        self.terminal = None
        self.callback = callback
        self.worker = Worker(self.config.text().strip(), {'action': action, 'args': args}, self.journal)
        self.status.setText(f'{action} 실행 중')
        self.log.appendPlainText(f'[{action}] 시작')

    def cancel_job(self):
        if self.worker:
            self.worker.cancel()
            self.status.setText('취소 요청됨 — 현재 단위 rollback을 기다립니다. 완료된 단위는 유지됩니다.')

    def poll(self):
        if not self.worker:
            return
        while True:
            try:
                kind, value = self.worker.queue.get_nowait()
            except Empty:
                break
            if kind == 'progress':
                self.log.appendPlainText(value.get('message', ''))
                self.status.setText(value.get('message', '실행 중'))
            else:
                self.terminal = (kind, value)
        if self.worker.process.is_alive():
            return
        if self.terminal is None:
            self.terminal = ('error', f'worker가 결과 없이 종료되었습니다 (exit={self.worker.process.exitcode}). 작업 이력과 DB를 확인하세요.')
            self.journal.finish(self.worker.job_id, 'INTERRUPTED', self.terminal[1])
        self.worker.close()
        self.worker = None
        self.config.setEnabled(True)
        self.source.setEnabled(True)
        self.cancel.setEnabled(False)
        self.history.replace(self.journal.list())
        kind, value = self.terminal
        callback = self.callback
        self.status.setText('완료' if kind == 'result' else str(value))
        self.log.appendPlainText('완료' if kind == 'result' else str(value))
        if kind == 'result':
            callback(value)
        elif kind == 'error':
            QMessageBox.warning(self, '작업 실패', str(value))

    def closeEvent(self, event):
        if self.worker:
            self.cancel_job()
            event.ignore()
            self.status.setText('작업 취소·정리가 끝난 뒤 창을 닫아 주세요.')
        else:
            event.accept()
