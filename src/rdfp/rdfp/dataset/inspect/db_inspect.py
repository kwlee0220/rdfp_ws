"""DB 쪽 조회 — 테이블 · 스키마 · 행 수 · 에피소드 요약과 점검 · 행 열람 · 에피소드 삭제.

`list` · `stats` CLI 가 하던 질의를 여기로 모아 GUI 와 CLI 가 같은 질의를 쓴다 (설계안 §7).
모든 함수는 **열린 커넥션을 받는다**(`rdfp.dataset.db.connection.open_connection`) — 접속과
트랜잭션 경계는 호출자의 몫이다. 읽기 함수는 커밋하지 않고, :func:`delete_episodes` 도 커밋을
호출자에게 맡긴다 (확인 절차 뒤에 커밋해야 한다).

에피소드 점검(:func:`episode_checks`)은 설계안 §4.5 의 넷이다 — 전부 **"있어야 할 것이 0"** 류라
경고로만 표시하고 막지 않는다. 데이터가 조용히 비는 함정의 전형을 각 메시지에 적어 두었다.
"""

from __future__ import annotations

from typing import Any, Iterable, Optional, Sequence

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path

import psycopg
from psycopg import sql

from rdfp.dataset.db.schema_check import REQUIRED_COLUMNS
from rdfp.dataset.episode_key import join_ns

from .findings import LEVEL_INFO, LEVEL_WARNING, Finding
from .mapping import MEDIA_TABLES, SYSTEM_TABLES

_logger = logging.getLogger(__name__)

#: 에피소드 단위로 행 수를 세는 메시지 테이블 (sessions · topics · 영상 제외).
MESSAGE_TABLES: tuple = tuple(sorted(
    t for t in REQUIRED_COLUMNS if t not in SYSTEM_TABLES and t not in MEDIA_TABLES))


def _ident(name: str) -> sql.Identifier:
    return sql.Identifier(name)


# ----- 테이블 · 스키마 ------------------------------------------------------------

def list_tables(conn: psycopg.Connection) -> list[str]:
    """현재 스키마의 테이블 이름 전부 (적재가 모르는 테이블도 보인다)."""
    with conn.cursor() as cur:
        cur.execute("SELECT table_name FROM information_schema.tables "
                    "WHERE table_schema = current_schema() AND table_type = 'BASE TABLE' "
                    "ORDER BY table_name")
        return [str(r[0]) for r in cur.fetchall()]


def table_row_counts(conn: psycopg.Connection, tables: Iterable[str]) -> dict[str, Optional[int]]:
    """테이블별 `COUNT(*)`. 없는 테이블은 ``None`` 이다 — 0 과 구분한다."""
    out: dict[str, Optional[int]] = {}
    for table in tables:
        try:
            with conn.transaction():
                with conn.cursor() as cur:
                    cur.execute(sql.SQL('SELECT COUNT(*) FROM {}').format(_ident(table)))
                    out[table] = int(cur.fetchone()[0])
        except psycopg.Error as exc:
            _logger.debug('count failed for %s: %s', table, exc)
            out[table] = None
    return out


@dataclass(frozen=True)
class ColumnInfo:
    name: str
    data_type: str
    nullable: bool
    default: Optional[str]
    generated: Optional[str]


@dataclass(frozen=True)
class TableSchema:
    """`information_schema` · `pg_catalog` 로 읽은 테이블 정의."""

    table: str
    columns: tuple
    constraints: tuple
    indexes: tuple

    def as_text(self) -> str:
        """`\\d` 비슷한 텍스트 — 상세 패널과 CLI 가 그대로 보인다."""
        width = max([len(c.name) for c in self.columns] + [6])
        lines = [f'{"column":<{width}}  type                 null  default / generated']
        for c in self.columns:
            extra = c.generated and f'GENERATED AS {c.generated}' or (c.default or '')
            nullable = 'yes' if c.nullable else 'no'
            lines.append(f'{c.name:<{width}}  {c.data_type:<20} {nullable:<5} {extra}')
        if self.constraints:
            lines.append('')
            lines.append('constraints:')
            lines.extend(f'  {c}' for c in self.constraints)
        if self.indexes:
            lines.append('')
            lines.append('indexes:')
            lines.extend(f'  {i}' for i in self.indexes)
        return '\n'.join(lines)


def table_schema(conn: psycopg.Connection, table: str) -> TableSchema:
    """열 · 제약 · 인덱스를 읽는다. 열 타입은 배열이면 `float8[]` 처럼 원소 타입으로 보인다."""
    with conn.cursor() as cur:
        cur.execute(
            'SELECT column_name, data_type, udt_name, is_nullable, column_default, '
            '       is_generated, generation_expression '
            'FROM information_schema.columns '
            'WHERE table_schema = current_schema() AND table_name = %s '
            'ORDER BY ordinal_position', (table,))
        columns = []
        for name, data_type, udt, nullable, default, generated, expr in cur.fetchall():
            shown = f'{udt[1:]}[]' if data_type == 'ARRAY' and udt.startswith('_') else data_type
            columns.append(ColumnInfo(
                name=str(name), data_type=str(shown), nullable=(nullable == 'YES'),
                default=str(default) if default else None,
                generated=str(expr) if generated == 'ALWAYS' and expr else None))
        cur.execute('SELECT conname, pg_get_constraintdef(oid) FROM pg_constraint '
                    'WHERE conrelid = %s::regclass ORDER BY conname', (table,))
        constraints = tuple(f'{n}: {d}' for n, d in cur.fetchall())
        cur.execute('SELECT indexdef FROM pg_indexes '
                    'WHERE schemaname = current_schema() AND tablename = %s ORDER BY indexname',
                    (table,))
        indexes = tuple(str(r[0]) for r in cur.fetchall())
    return TableSchema(table=table, columns=tuple(columns), constraints=constraints,
                       indexes=indexes)


# ----- 토픽 ------------------------------------------------------------------

@dataclass(frozen=True)
class TopicRef:
    id: int
    name: str
    type_name: str


def all_topics(conn: psycopg.Connection) -> list[TopicRef]:
    with conn.cursor() as cur:
        cur.execute('SELECT id, topic_name, topic_type FROM topics ORDER BY topic_name')
        return [TopicRef(int(i), str(n), str(t)) for i, n, t in cur.fetchall()]


@dataclass(frozen=True)
class TopicRows:
    topic: TopicRef
    rows: int


def rows_per_topic(conn: psycopg.Connection, table: str) -> list[TopicRows]:
    """한 테이블에 어느 토픽이 몇 행 들어왔나 — 선 두 개가 한 테이블로 모이는 것이 여기서 보인다."""
    if table in SYSTEM_TABLES:
        return []
    query = sql.SQL('SELECT t.id, t.topic_name, t.topic_type, COUNT(*) FROM {} x '
                    'JOIN topics t ON t.id = x.topic_id GROUP BY 1, 2, 3 ORDER BY 2').format(
                        _ident(table))
    with conn.cursor() as cur:
        cur.execute(query)
        return [TopicRows(TopicRef(int(i), str(n), str(t)), int(c))
                for i, n, t, c in cur.fetchall()]


# ----- 행 열람 ---------------------------------------------------------------

@dataclass(frozen=True)
class RowPage:
    """데이터 브라우저의 한 페이지."""

    columns: tuple
    rows: tuple
    offset: int
    has_more: bool


def _where(episode_id: Optional[int], topic_id: Optional[int]) -> tuple[sql.Composable, list]:
    parts = []
    params: list = []
    if episode_id is not None:
        parts.append(sql.SQL('episode_id = %s'))
        params.append(episode_id)
    if topic_id is not None:
        parts.append(sql.SQL('topic_id = %s'))
        params.append(topic_id)
    if not parts:
        return sql.SQL(''), params
    return sql.SQL(' WHERE ') + sql.SQL(' AND ').join(parts), params


def _order_for(table: str) -> sql.Composable:
    """stamp 가 있는 테이블은 stamp 순, 아니면 id 순."""
    cols = REQUIRED_COLUMNS.get(table, [])
    if 'stamp_sec' in cols:
        return sql.SQL('stamp_sec, stamp_nanosec, id')
    if table == 'sessions':
        return sql.SQL('start_sec, start_nanosec, id')
    return sql.SQL('id')


def fetch_rows(conn: psycopg.Connection, table: str, *, episode_id: Optional[int] = None,
               topic_id: Optional[int] = None, offset: int = 0, limit: int = 1000) -> RowPage:
    """행 한 페이지. `has_more` 를 알기 위해 `limit + 1` 을 읽는다."""
    where, params = _where(episode_id, topic_id)
    query = sql.SQL('SELECT * FROM {}{} ORDER BY {} LIMIT %s OFFSET %s').format(
        _ident(table), where, _order_for(table))
    with conn.cursor() as cur:
        cur.execute(query, (*params, limit + 1, offset))
        rows = cur.fetchall()
        columns = tuple(d.name for d in cur.description or ())
    has_more = len(rows) > limit
    return RowPage(columns=columns, rows=tuple(rows[:limit]), offset=offset, has_more=has_more)


def count_rows(conn: psycopg.Connection, table: str, *, episode_id: Optional[int] = None,
               topic_id: Optional[int] = None) -> int:
    where, params = _where(episode_id, topic_id)
    with conn.cursor() as cur:
        cur.execute(sql.SQL('SELECT COUNT(*) FROM {}{}').format(_ident(table), where), params)
        return int(cur.fetchone()[0])


def render_cell(value: Any, *, max_chars: int = 200) -> str:
    """셀 하나를 한 줄 문자열로. 배열 · JSONB 는 JSON 으로 펼치고 길면 자른다."""
    if value is None:
        return ''
    if isinstance(value, (list, tuple, dict)):
        text = json.dumps(value, ensure_ascii=False, default=str)
    else:
        text = str(value)
    if len(text) > max_chars:
        return text[:max_chars - 1] + '…'
    return text


# ----- 에피소드 요약 · 점검 ----------------------------------------------------------

@dataclass(frozen=True)
class StreamInfo:
    """에피소드의 영상 한 벌 (`image_streams` 한 행)."""

    topic: str
    mp4_path: str
    frame_count: int
    frames_in_db: int
    resolved_path: Optional[str]

    @property
    def file_exists(self) -> bool:
        return self.resolved_path is not None


@dataclass
class EpisodeSummary:
    """에피소드 뷰의 한 행 (설계안 §4.5)."""

    id: int
    start_ns: int
    stop_ns: int
    task_label: Optional[str]
    success: Optional[bool]
    metadata: dict
    row_counts: dict = field(default_factory=dict)
    streams: list = field(default_factory=list)
    unpaired_trajectories: int = 0
    unpaired_events: int = 0

    @property
    def duration_sec(self) -> float:
        return (self.stop_ns - self.start_ns) / 1e9

    @property
    def backend(self) -> Optional[str]:
        value = self.metadata.get('backend') if isinstance(self.metadata, dict) else None
        return str(value) if value else None

    @property
    def message_rows(self) -> int:
        """메시지 테이블 행의 합 (영상 제외)."""
        return sum(self.row_counts.get(t, 0) for t in MESSAGE_TABLES)

    @property
    def frames(self) -> int:
        return self.row_counts.get('image_frames', 0)


def _resolve_mp4(mp4_path: str, roots: Sequence[str]) -> Optional[str]:
    """`mp4_path` 는 출력 루트 기준 상대 경로라 루트를 차례로 붙여 본다."""
    p = Path(mp4_path)
    if p.is_absolute():
        return str(p) if p.is_file() else None
    for root in roots:
        candidate = Path(root) / p
        if candidate.is_file():
            return str(candidate)
    return None


def episode_summaries(conn: psycopg.Connection, *, tables: Sequence[str] = MESSAGE_TABLES,
                      mp4_roots: Sequence[str] = ()) -> list[EpisodeSummary]:
    """DB 의 모든 에피소드를 테이블별 행 수 · 영상 · 연산 짝 맞춤과 함께 돌려준다.

    질의 수는 테이블 수에 비례하고 에피소드 수와 무관하다 (`GROUP BY episode_id`).
    없는 테이블은 건너뛴다 — 스키마가 옛것이어도 목록은 나와야 한다.
    """
    present = set(list_tables(conn))
    with conn.cursor() as cur:
        cur.execute('SELECT id, start_sec, start_nanosec, stop_sec, stop_nanosec, task_label, '
                    '       success, metadata FROM sessions ORDER BY start_sec, start_nanosec, id')
        summaries: dict[int, EpisodeSummary] = {}
        for row in cur.fetchall():
            eid, ssec, snsec, tsec, tnsec, task, success, meta = row
            summaries[int(eid)] = EpisodeSummary(
                id=int(eid), start_ns=join_ns(ssec, snsec), stop_ns=join_ns(tsec, tnsec),
                task_label=task, success=success,
                metadata=meta if isinstance(meta, dict) else {})

        for table in [*tables, 'image_frames']:
            if table not in present:
                continue
            cur.execute(sql.SQL('SELECT episode_id, COUNT(*) FROM {} GROUP BY episode_id').format(
                _ident(table)))
            for eid, n in cur.fetchall():
                s = summaries.get(int(eid))
                if s is not None:
                    s.row_counts[table] = int(n)

        if 'image_streams' in present:
            cur.execute(
                'SELECT s.episode_id, t.topic_name, s.mp4_path, s.frame_count, '
                '       (SELECT COUNT(*) FROM image_frames f '
                '        WHERE f.episode_id = s.episode_id AND f.topic_id = s.topic_id) '
                'FROM image_streams s JOIN topics t ON t.id = s.topic_id '
                'ORDER BY s.episode_id, t.topic_name')
            for eid, topic, mp4_path, frame_count, in_db in cur.fetchall():
                s = summaries.get(int(eid))
                if s is not None:
                    s.streams.append(StreamInfo(
                        topic=str(topic), mp4_path=str(mp4_path), frame_count=int(frame_count),
                        frames_in_db=int(in_db),
                        resolved_path=_resolve_mp4(str(mp4_path), mp4_roots)))

        if 'motion_trajectories' in present and 'motion_events' in present:
            cur.execute('SELECT t.episode_id, COUNT(*) FROM motion_trajectories t '
                        'LEFT JOIN motion_events e ON e.episode_id = t.episode_id '
                        '                         AND e.operation_id = t.operation_id '
                        'WHERE e.id IS NULL GROUP BY t.episode_id')
            for eid, n in cur.fetchall():
                s = summaries.get(int(eid))
                if s is not None:
                    s.unpaired_trajectories = int(n)
            # `failed` 만 짝 궤적이 없는 것이 정상이다 — 그 불변식이 유일한 근거다.
            cur.execute("SELECT e.episode_id, COUNT(*) FROM motion_events e "
                        "LEFT JOIN motion_trajectories t ON t.episode_id = e.episode_id "
                        "                              AND t.operation_id = e.operation_id "
                        "WHERE t.id IS NULL AND e.result <> 'failed' GROUP BY e.episode_id")
            for eid, n in cur.fetchall():
                s = summaries.get(int(eid))
                if s is not None:
                    s.unpaired_events = int(n)
    return list(summaries.values())


def episode_checks(s: EpisodeSummary) -> list[Finding]:
    """설계안 §4.5 의 점검. 경고만 하고 막지 않는다 — 전부 '있어야 할 것이 0' 류다."""
    out: list[Finding] = []
    if s.message_rows == 0:
        out.append(Finding(LEVEL_WARNING, 'no_message_rows',
                           '어느 메시지 테이블에도 행이 없다 — 시뮬레이터 재기동 뒤 sim 시계가 겹친 '
                           'bag 을 함께 적재했을 때의 전형이다 (에피소드는 있는데 속이 빈다).'))
    if s.streams and s.frames == 0:
        out.append(Finding(LEVEL_WARNING, 'frames_missing',
                           'image_streams 는 있는데 image_frames 가 0 이다 — 레코더 `pixel_format` '
                           '불일치(모든 프레임이 조용히 버려짐)의 전형이다. frame clock 이 서지 '
                           '않아 export 가 막힌다.'))
    if s.unpaired_trajectories or s.unpaired_events:
        out.append(Finding(LEVEL_WARNING, 'motion_unpaired',
                           f'motion_trajectories ↔ motion_events 의 operation_id 짝이 안 맞는다 '
                           f'(궤적만 {s.unpaired_trajectories} · failed 아닌 이벤트만 '
                           f'{s.unpaired_events}). `failed` 만 짝 궤적이 없어야 한다.'))
    if s.message_rows and s.row_counts.get('gripper_cmds', 0) == 0:
        out.append(Finding(LEVEL_WARNING, 'gripper_cmds_zero',
                           'gripper_cmds 가 0 행이다 — `header.stamp` 를 안 채운(또는 벽시계로 '
                           '찍은) 메시지가 에피소드 창 밖으로 떨어진 stamp 함정의 전형이다. '
                           '그리퍼를 안 쓴 에피소드라면 정상이다.'))
    for st in s.streams:
        if not st.file_exists:
            out.append(Finding(LEVEL_WARNING, 'mp4_file_missing',
                               f'{st.topic} 의 mp4 파일을 찾을 수 없다: {st.mp4_path} '
                               f'(recordings_dir · output_mp4_dir 아래 모두 없음).'))
        elif s.frames and st.frames_in_db != st.frame_count:
            # 프레임이 통째로 없는 경우는 위 `frames_missing` 이 이미 말했다.
            out.append(Finding(LEVEL_WARNING, 'frame_count_mismatch',
                               f'{st.topic}: image_streams.frame_count={st.frame_count} 인데 '
                               f'image_frames 는 {st.frames_in_db} 행이다.'))
    if not s.streams:
        out.append(Finding(LEVEL_INFO, 'no_video',
                           '영상이 등록되지 않았다 — 유효한 에피소드지만 frame clock 이 없어 export '
                           '대상이 아니다.'))
    return out


# ----- 삭제 ------------------------------------------------------------------

def mp4_paths_for(conn: psycopg.Connection, episode_ids: Sequence[int]) -> list[tuple[int, str]]:
    """지우기 **전에** 영상 경로를 뽑는다 — 행을 지운 뒤에는 어느 파일이 그 에피소드 것인지 알 수 없다."""
    if not episode_ids:
        return []
    with conn.cursor() as cur:
        cur.execute('SELECT episode_id, mp4_path FROM image_streams WHERE episode_id = ANY(%s) '
                    'ORDER BY episode_id, mp4_path', (list(episode_ids),))
        return [(int(e), str(p)) for e, p in cur.fetchall()]


def delete_episodes(conn: psycopg.Connection, episode_ids: Sequence[int]) -> int:
    """`sessions` 행을 지운다 — 자식 행은 FK CASCADE 로 함께 사라진다. **커밋하지 않는다.**"""
    if not episode_ids:
        return 0
    with conn.cursor() as cur:
        cur.execute('DELETE FROM sessions WHERE id = ANY(%s)', (list(episode_ids),))
        return int(cur.rowcount)


def recording_files_for(mp4_path: str, roots: Sequence[str]) -> list[Path]:
    """mp4 한 벌이 디스크에 남긴 파일들 — 레코더 출력은 mp4 · `.jsonl` sidecar · `_metadata.json`,
    bag 인코딩 출력은 `episode_<id>/` 디렉터리째다. 존재하는 것만 돌려준다."""
    out: list[Path] = []
    rel = Path(mp4_path)
    for root in roots:
        base = Path(root)
        candidate = base / rel
        if not candidate.exists():
            continue
        if rel.parts and rel.parts[0].startswith('episode_'):
            out.append(base / rel.parts[0])
            break
        out.append(candidate)
        for sibling in (candidate.with_suffix('.jsonl'),
                        candidate.with_name(candidate.stem + '_metadata.json')):
            if sibling.exists():
                out.append(sibling)
        break
    return out


__all__ = [
    'ColumnInfo', 'EpisodeSummary', 'MESSAGE_TABLES', 'RowPage', 'StreamInfo', 'TableSchema',
    'TopicRef', 'TopicRows', 'all_topics', 'count_rows', 'delete_episodes', 'episode_checks',
    'episode_summaries', 'fetch_rows', 'list_tables', 'mp4_paths_for', 'recording_files_for',
    'render_cell', 'rows_per_topic', 'table_row_counts', 'table_schema',
]
