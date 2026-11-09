"""공유 상태 — bag · DB · 에피소드 스냅샷을 한 곳에 들고 바뀌면 패널에 알린다.

패널마다 따로 읽으면 같은 bag 을 세 번 훑고, 한쪽만 새로 고쳐져 화면이 서로 다른 말을 한다.
읽기는 `Background` 로 돌고, 완료 시그널을 패널이 구독한다.
"""

from __future__ import annotations

from typing import Callable, Optional

from PySide6.QtCore import QObject, Signal

from .context import AppContext
from .snapshot import (BagSnapshot, DbSnapshot, EpisodeData, diagnose_topic, load_bag_snapshot,
                       load_db_snapshot, load_episode_data)
from .workers import Background


class DataStore(QObject):
    bag_changed = Signal()
    db_changed = Signal()
    episodes_changed = Signal()
    diagnosis_changed = Signal(str)        # topic
    status = Signal(str)                   # 상태바 문구

    def __init__(self, ctx: AppContext, bg: Background, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self.ctx = ctx
        self.bg = bg
        self.bag: Optional[BagSnapshot] = None
        self.db: Optional[DbSnapshot] = None
        self.episodes: Optional[EpisodeData] = None
        self._diagnosing: set = set()

    # ----- 새로 고침 -----------------------------------------------------------

    def refresh_all(self) -> None:
        self.refresh_bag(then_db=True)
        self.refresh_episodes()

    def refresh_bag(self, *, then_db: bool = False) -> None:
        self.status.emit('bag 을 훑는 중…')

        def done(snapshot: BagSnapshot) -> None:
            self.bag = snapshot
            self.bag_changed.emit()
            self.status.emit(f'bag: 세션 {len(snapshot.sessions)} · 토픽 {len(snapshot.topics)} · '
                             f'에피소드 {len(snapshot.episodes)}'
                             + (f' — {snapshot.error}' if snapshot.error else ''))
            if then_db:
                self.refresh_db()

        self.bg.run(load_bag_snapshot, done, self._fail('bag'), self.ctx.cfg)

    def refresh_db(self) -> None:
        self.status.emit('DB 를 읽는 중…')

        def done(snapshot: DbSnapshot) -> None:
            self.db = snapshot
            self.db_changed.emit()
            if snapshot.error:
                self.status.emit(f'DB: {snapshot.error}')
            else:
                self.status.emit(f'DB: 테이블 {len(snapshot.tables)} · 적재된 에피소드 '
                                 f'{len(snapshot.exported_keys or ())}')

        self.bg.run(load_db_snapshot, done, self._fail('db'), self.ctx.cfg, self.bag)

    def refresh_episodes(self) -> None:
        def done(data: EpisodeData) -> None:
            self.episodes = data
            self.episodes_changed.emit()

        self.bg.run(load_episode_data, done, self._fail('episodes'), self.ctx.cfg)

    def diagnose(self, topic: str, on_done: Optional[Callable] = None) -> None:
        """토픽 하나의 stamp 진단. 결과는 `TopicEntry.diagnosis` 에 들어가고 시그널로 알린다."""
        if self.bag is None or topic in self._diagnosing:
            return
        entry = self.bag.entry(topic)
        if entry is None:
            return
        if entry.diagnosis is not None:
            if on_done:
                on_done(entry.diagnosis)
            return
        self._diagnosing.add(topic)
        bag = self.bag

        def done(diag) -> None:
            self._diagnosing.discard(topic)
            if self.bag is bag:
                entry.diagnosis = diag
                self.diagnosis_changed.emit(topic)
            if on_done:
                on_done(diag)

        def fail(text: str) -> None:
            self._diagnosing.discard(topic)
            self.status.emit(f'stamp 진단 실패 ({topic}): {text.strip().splitlines()[-1]}')

        self.bg.run(diagnose_topic, done, fail, bag, topic)

    def diagnose_all(self) -> None:
        if self.bag is None:
            return
        for entry in self.bag.topics:
            if entry.mapping.ingested and entry.diagnosis is None:
                self.diagnose(entry.name)

    def _fail(self, what: str) -> Callable[[str], None]:
        def handler(text: str) -> None:
            last = text.strip().splitlines()[-1] if text.strip() else text
            self.status.emit(f'{what} 읽기 실패: {last}')
        return handler


__all__ = ['DataStore']
