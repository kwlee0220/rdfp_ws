"""화면이 쓰는 모양으로 bag · DB 를 읽어 묶는다. **Qt 를 모른다** — 워커 스레드에서 돌고 테스트된다.

읽기는 전부 `rdfp.dataset.inspect` 를 부른다. 여기서 하는 일은 두 쪽을 **짝짓는 것**이다 — 토픽마다
매핑 상태를 붙이고, 테이블마다 어느 토픽이 들어오는지 붙이고, bag 의 에피소드에 적재 여부를 붙인다.
"""

from __future__ import annotations

from typing import Optional

import logging
from dataclasses import dataclass, field
from pathlib import Path

from rdfp.dataset.config import DatasetConfig
from rdfp.dataset.episode_key import fetch_exported_keys, split_ns
from rdfp.dataset.ingest.episode.from_bag import BagEpisode, episodes_from_splits
from rdfp.dataset.inspect.bag_inspect import (SessionInfo, StampDiagnosis, TopicSummary,
                                              diagnose_stamps, episode_windows, finalized_splits,
                                              mcap_paths_for_topic, scan_sessions, summarize_topics)
from rdfp.dataset.inspect.db_inspect import (EpisodeSummary, count_rows, episode_checks,
                                             episode_summaries, list_tables, table_row_counts)
from rdfp.dataset.inspect.findings import Finding
from rdfp.dataset.inspect.mapping import (STATUS_UNMAPPED, MappingInfo, TableInfo, builtin_tables,
                                          classify_topic, glyph_for)

_logger = logging.getLogger(__name__)


@dataclass
class TopicEntry:
    """① 매핑 탭 왼쪽 한 줄."""

    summary: TopicSummary
    mapping: MappingInfo
    diagnosis: Optional[StampDiagnosis] = None

    @property
    def name(self) -> str:
        return self.summary.name

    @property
    def glyph(self) -> str:
        return glyph_for(self.mapping, suspect=bool(self.diagnosis and self.diagnosis.suspect))

    @property
    def checkable(self) -> bool:
        """적재에 영향을 주는 토픽만 체크할 수 있다 — 매핑 없는 토픽은 어차피 무시된다."""
        return self.mapping.ingested and self.mapping.table != 'sessions'


@dataclass
class TableEntry:
    """① 매핑 탭 오른쪽 한 줄."""

    info: TableInfo
    rows: Optional[int]            # None 이면 DB 에 없다(또는 DB 를 못 봤다)
    incoming: list = field(default_factory=list)   # bag 에서 이 테이블로 가는 토픽 이름

    @property
    def name(self) -> str:
        return self.info.name


@dataclass
class BagSnapshot:
    rosbag_dir: Optional[Path]
    sessions: list = field(default_factory=list)
    topics: list = field(default_factory=list)          # TopicEntry
    episodes: list = field(default_factory=list)        # BagEpisode (log time 순)
    error: Optional[str] = None

    @property
    def windows(self) -> list:
        return episode_windows(self.episodes)

    def topic_types(self) -> dict:
        return {t.name: t.summary.types[0] if t.summary.types else '' for t in self.topics}

    def entry(self, topic: str) -> Optional[TopicEntry]:
        for t in self.topics:
            if t.name == topic:
                return t
        return None

    def mcap_paths(self, topic: str) -> list:
        return mcap_paths_for_topic(self.sessions, topic)


@dataclass
class DbSnapshot:
    tables: list = field(default_factory=list)          # TableEntry
    extra_tables: list = field(default_factory=list)    # DB 에만 있는 테이블
    exported_keys: Optional[set] = None                 # sessions 의 start_ns 전부. None = 못 봤다
    error: Optional[str] = None

    def entry(self, table: str) -> Optional[TableEntry]:
        for t in self.tables:
            if t.name == table:
                return t
        return None


@dataclass
class EpisodeData:
    summaries: list = field(default_factory=list)       # EpisodeSummary (DB)
    checks: dict = field(default_factory=dict)          # id → list[Finding]
    error: Optional[str] = None

    def by_id(self, episode_id: int) -> Optional[EpisodeSummary]:
        for s in self.summaries:
            if s.id == episode_id:
                return s
        return None

    def warnings(self, episode_id: int) -> list:
        return [f for f in self.checks.get(episode_id, ()) if f.level == 'warning']


# ----- bag ------------------------------------------------------------------

def load_bag_snapshot(cfg: Optional[DatasetConfig]) -> BagSnapshot:
    """세션 트리 · 토픽 요약(매핑 상태 포함) · 에피소드를 한 번에 읽는다. MCAP 본문은 안 푼다
    (`/session` 만 읽어 에피소드를 감지한다)."""
    if cfg is None:
        return BagSnapshot(rosbag_dir=None, error='config not loaded')
    root = Path(cfg.rosbag_dir)
    try:
        sessions = scan_sessions(root)
    except FileNotFoundError as exc:
        return BagSnapshot(rosbag_dir=root, error=str(exc))
    topics = [TopicEntry(summary=s, mapping=classify_topic(s.name, s.types[0] if s.types else ''))
              for s in summarize_topics(sessions)]
    episodes: list = []
    try:
        episodes = episodes_from_splits(finalized_splits(sessions))
    except Exception as exc:  # noqa: BLE001 — 잘린 split 하나가 목록 전체를 막으면 안 된다
        _logger.warning('episode detection failed: %s', exc)
        return BagSnapshot(rosbag_dir=root, sessions=sessions, topics=topics,
                           error=f'episode detection failed: {exc}')
    return BagSnapshot(rosbag_dir=root, sessions=sessions, topics=topics, episodes=episodes)


def diagnose_topic(bag: BagSnapshot, topic: str) -> StampDiagnosis:
    """토픽 하나의 stamp 진단 — 에피소드 창을 함께 본다. 느리므로 워커에서 부른다."""
    return diagnose_stamps(bag.mcap_paths(topic), topic, windows=bag.windows)


def unmapped_topics(bag: BagSnapshot) -> list:
    return [t.name for t in bag.topics if t.mapping.status == STATUS_UNMAPPED]


# ----- DB -------------------------------------------------------------------

def _incoming(bag: Optional[BagSnapshot], table: str) -> list:
    if bag is None:
        return []
    return [t.name for t in bag.topics if t.mapping.table == table]


def load_db_snapshot(cfg: Optional[DatasetConfig], bag: Optional[BagSnapshot]) -> DbSnapshot:
    """테이블 · 행 수 · 적재된 에피소드 키. DB 에 못 붙어도 테이블 목록(내장)은 돌려준다 — 오른쪽
    패널이 비면 매핑 선을 그릴 수 없다."""
    infos = builtin_tables()
    if cfg is None:
        return DbSnapshot(tables=[TableEntry(i, None, _incoming(bag, i.name)) for i in infos],
                          error='config not loaded')
    from rdfp.dataset.db.connection import open_connection

    try:
        with open_connection(cfg.db) as conn:
            present = list_tables(conn)
            counts = table_row_counts(conn, present)
            exported = fetch_exported_keys(conn) if 'sessions' in present else set()
            conn.rollback()
    except Exception as exc:  # noqa: BLE001 — 접속 실패 종류가 많다; 메시지만 보인다
        return DbSnapshot(tables=[TableEntry(i, None, _incoming(bag, i.name)) for i in infos],
                          error=str(exc))
    tables = [TableEntry(i, counts.get(i.name) if i.name in present else None,
                         _incoming(bag, i.name)) for i in infos]
    extra = sorted(set(present) - {i.name for i in infos})
    return DbSnapshot(tables=tables, extra_tables=extra, exported_keys=exported)


def load_episode_data(cfg: Optional[DatasetConfig]) -> EpisodeData:
    if cfg is None:
        return EpisodeData(error='config not loaded')
    from rdfp.dataset.db.connection import open_connection

    try:
        with open_connection(cfg.db) as conn:
            summaries = episode_summaries(conn, mp4_roots=cfg.mp4_roots())
            conn.rollback()
    except Exception as exc:  # noqa: BLE001
        return EpisodeData(error=str(exc))
    checks = {s.id: episode_checks(s) for s in summaries}
    return EpisodeData(summaries=summaries, checks=checks)


def exported_flag(episode: BagEpisode, exported_keys: Optional[set]) -> str:
    """bag 에피소드의 적재 여부 — `yes` · `no` · `?` (DB 를 못 봤다). `?` 를 `no` 로 뭉개면 재적재를
    유발한다."""
    if exported_keys is None:
        return '?'
    return 'yes' if episode.start_ns in exported_keys else 'no'


def db_row_count(cfg: DatasetConfig, table: str, *, topic_name: str,
                 start_ns: Optional[int]) -> Optional[int]:
    """같은 에피소드 · 같은 토픽의 DB 행 수 — bag 행 수와 나란히 보이기 위한 값. 못 구하면 ``None``."""
    from rdfp.dataset.db.connection import open_connection

    try:
        with open_connection(cfg.db) as conn:
            with conn.cursor() as cur:
                cur.execute('SELECT id FROM topics WHERE topic_name = %s', (topic_name,))
                row = cur.fetchone()
                if row is None:
                    return 0
                topic_id = int(row[0])
                episode_id = None
                if start_ns is not None:
                    sec, nsec = split_ns(start_ns)
                    cur.execute('SELECT id FROM sessions '
                                'WHERE start_sec = %s AND start_nanosec = %s', (sec, nsec))
                    found = cur.fetchone()
                    if found is None:
                        return 0
                    episode_id = int(found[0])
            n = count_rows(conn, table, episode_id=episode_id, topic_id=topic_id)
            conn.rollback()
            return n
    except Exception as exc:  # noqa: BLE001
        _logger.debug('db count failed: %s', exc)
        return None


def session_export_status(session: SessionInfo, episodes: list,
                          exported_keys: Optional[set]) -> str:
    """③ 세션 탭의 '적재' 열 — 그 세션의 에피소드 가운데 몇 개가 DB 에 있나."""
    mine = [e for e in episodes if e.session_name == session.name]
    if not mine:
        return '-' if session.finalized else '미완료'
    if exported_keys is None:
        return f'? / {len(mine)}'
    done = sum(1 for e in mine if e.start_ns in exported_keys)
    return f'{done} / {len(mine)}'


__all__ = [
    'BagSnapshot', 'DbSnapshot', 'EpisodeData', 'Finding', 'TableEntry', 'TopicEntry',
    'db_row_count', 'diagnose_topic', 'exported_flag', 'load_bag_snapshot', 'load_db_snapshot',
    'load_episode_data', 'session_export_status', 'unmapped_topics',
]
