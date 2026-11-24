"""에피소드 열기 · 적재 단위 · 출처 — 직렬(`pipeline`)과 병렬(`episode_worker`) 두 경로가 함께 쓴다.

두 경로가 이 판단을 각자 들고 있으면 한쪽만 뒤처진다(예전 `_open_episode` 가 두 벌이었다).
그래서 에피소드를 어떻게 열고 무엇을 건너뛰는지는 여기 한 곳에서 정한다.

**적재 단위 = (에피소드, 토픽)** 이다 (설계안 §4.4, 2026-11-23). 이미 있는 에피소드를 만나면:

| 경우 | 동작 |
|---|---|
| 출처가 다른 녹화본이다 | **거부** — sim 시계가 되감긴 녹화본이 같은 키를 낸 것이다. 예전에는 조용히 속이 빈 에피소드가 됐다 |
| 종료 시각이 다르다 | **거부** — 같은 키의 다른 에피소드다 |
| 출처 기록이 없다(이 구조 이전 적재) | 시작 · 종료 시각이 같으면 **연결(adopt)** 하고 계속한다 |
| `--only-topics-replace` | 고른 토픽의 행만 지우고 다시 넣는다. 다른 토픽과 `sessions.id` 는 그대로다 |
| `on_existing_episode: skip` | **이미 적재된 단위만** 건너뛰고 빠진 토픽을 채운다 |
| `replace` | 예전처럼 에피소드를 통째로 지우고 다시 넣는다 |
| `error` | 예전처럼 멈춘다 |

`skip` 이 단위 단위가 된 것이 동작 변화다 — 예전에는 에피소드를 통째로 건너뛰었다. 전 토픽이
이미 들어 있는 에피소드는 지금도 아무것도 안 넣으므로 결과가 같고, 다른 점은 **빠진 토픽이
채워진다**는 것뿐이다(새 토픽을 녹화 목록 · 동적 매핑에 더한 경우).

ROS 를 요구하지 않는다.
"""

from __future__ import annotations

from typing import Callable, Mapping, Optional

from dataclasses import dataclass, field

import psycopg
from psycopg import sql

from rdfp.dataset.db.dynamic import GenericJsonbWriter
from rdfp.dataset.db.registry import resolve_message_type
from rdfp.dataset.db.writers.base import WriterBase
from rdfp.dataset.db.writers.session_command import SessionWriter
from rdfp.dataset.episode_key import join_ns

from .episode.detector import Episode
from .exceptions import PostProcError

#: 이 구조 이전 DB 에는 메타 테이블이 없다. `import` 는 그때 init-db 안내와 함께 멈춘다.
META_HINT = ('run `ros2 run rdfp init-db` once to add the ingest metadata tables '
             '(idempotent — existing rows are kept)')


@dataclass(frozen=True)
class Origin:
    """에피소드가 나온 녹화본. `source_id` 는 세션 디렉터리 이름이다 (schema.sql 주석 참조)."""

    source_id: str
    location: str


@dataclass
class Opening:
    """에피소드를 연 결과.

    Attributes:
        episode_id: ``None`` 이면 에피소드를 통째로 건너뛴다 (`skip` 이 아니라 그럴 일은 없다 —
            예외로만 멈춘다). 유지하는 이유는 호출자 분기를 단순하게 두기 위해서다.
        skip_topics: 이미 적재돼 이번에 건너뛸 토픽.
        existed: 이미 있던 에피소드인가.
        replaced: `replace` 로 통째로 다시 만들었나.
        adopted: 출처 기록이 없던 에피소드를 이번에 연결했나.
        replace_topics: `--only-topics-replace` 로 지우고 다시 넣는 토픽.
    """

    episode_id: Optional[int]
    skip_topics: frozenset = frozenset()
    existed: bool = False
    replaced: bool = False
    adopted: bool = False
    replace_topics: frozenset = frozenset()
    unit_counts: dict = field(default_factory=dict)


def make_writer(conn: psycopg.Connection, table: str, type_name: str,
                topic_id: int) -> WriterBase:
    """registry 에 있으면 내장 writer, 없으면 동적(JSONB) writer.

    `topic_entries` 에는 매핑된 토픽만 들어오므로 registry 에 없다는 것은 곧 동적 매핑이다.
    """
    binding = resolve_message_type(type_name)
    if binding is not None:
        return binding.writer_cls(conn, table=table, topic_id=topic_id)
    return GenericJsonbWriter(conn, table=table, topic_id=topic_id)


# ----- 출처 ----------------------------------------------------------------------

def record_source(conn: psycopg.Connection, origin: Origin) -> None:
    with conn.cursor() as cur:
        cur.execute('INSERT INTO ingest_sources (source_id, location) VALUES (%s, %s) '
                    'ON CONFLICT (source_id) DO UPDATE SET location = EXCLUDED.location, '
                    'last_seen = now()', (origin.source_id, origin.location))


def source_of(conn: psycopg.Connection, episode_id: int) -> Optional[str]:
    with conn.cursor() as cur:
        cur.execute('SELECT source_id FROM episode_sources WHERE episode_id = %s', (episode_id,))
        row = cur.fetchone()
    return str(row[0]) if row else None


def _link_source(conn: psycopg.Connection, episode_id: int, origin: Origin, *,
                 adopted: bool) -> None:
    record_source(conn, origin)
    with conn.cursor() as cur:
        cur.execute('INSERT INTO episode_sources (episode_id, source_id, adopted) '
                    'VALUES (%s, %s, %s)', (episode_id, origin.source_id, adopted))


def _stop_ns(conn: psycopg.Connection, episode_id: int) -> int:
    with conn.cursor() as cur:
        cur.execute('SELECT stop_sec, stop_nanosec FROM sessions WHERE id = %s', (episode_id,))
        sec, nsec = cur.fetchone()
    return join_ns(sec, nsec)


# ----- 적재 단위 ------------------------------------------------------------------

def _unit_exists(conn: psycopg.Connection, episode_id: int, topic_id: int) -> bool:
    with conn.cursor() as cur:
        cur.execute('SELECT 1 FROM ingest_units WHERE episode_id = %s AND topic_id = %s',
                    (episode_id, topic_id))
        return cur.fetchone() is not None


def _legacy_rows(conn: psycopg.Connection, table: str, episode_id: int, topic_id: int) -> int:
    with conn.cursor() as cur:
        cur.execute(sql.SQL('SELECT count(*) FROM {} WHERE episode_id = %s AND topic_id = %s')
                    .format(sql.Identifier(table)), (episode_id, topic_id))
        return int(cur.fetchone()[0])


def complete_unit(conn: psycopg.Connection, episode_id: int, topic_id: int, table: str,
                  rows: int, *, adopted: bool = False) -> None:
    with conn.cursor() as cur:
        cur.execute('INSERT INTO ingest_units (episode_id, topic_id, table_name, rows, adopted) '
                    'VALUES (%s, %s, %s, %s, %s) ON CONFLICT (episode_id, topic_id) DO UPDATE '
                    'SET table_name = EXCLUDED.table_name, rows = EXCLUDED.rows, '
                    'adopted = EXCLUDED.adopted, completed_at = now()',
                    (episode_id, topic_id, table, rows, adopted))


def loaded_topics(conn: psycopg.Connection, episode_id: int,
                  units: Mapping[str, tuple]) -> set:
    """`units` (토픽 → (테이블, topic_id)) 가운데 이미 적재된 토픽.

    적재 이력이 없는데 행이 있으면 이 구조 이전에 들어간 것이다 — 적재된 것으로 보고 이력을
    `adopted` 로 채워 넣는다. 그래야 다음 실행부터는 이력만 보고 판단한다.
    """
    done: set = set()
    for topic, (table, topic_id) in units.items():
        if _unit_exists(conn, episode_id, topic_id):
            done.add(topic)
            continue
        rows = _legacy_rows(conn, table, episode_id, topic_id)
        if rows:
            complete_unit(conn, episode_id, topic_id, table, rows, adopted=True)
            done.add(topic)
    return done


def delete_unit(conn: psycopg.Connection, episode_id: int, topic_id: int, table: str) -> int:
    """한 단위의 행을 지운다. 영상(`image_streams`)이면 프레임 행도 함께 지운다."""
    with conn.cursor() as cur:
        if table == 'image_streams':
            cur.execute('DELETE FROM image_frames WHERE episode_id = %s AND topic_id = %s',
                        (episode_id, topic_id))
        cur.execute(sql.SQL('DELETE FROM {} WHERE episode_id = %s AND topic_id = %s')
                    .format(sql.Identifier(table)), (episode_id, topic_id))
        n = cur.rowcount
        cur.execute('DELETE FROM ingest_units WHERE episode_id = %s AND topic_id = %s',
                    (episode_id, topic_id))
    return int(n)


# ----- 에피소드 열기 ----------------------------------------------------------------

def open_episode(conn: psycopg.Connection, sess_writer: SessionWriter, ep: Episode, *,
                 origin: Optional[Origin], policy: str, units: Mapping[str, tuple],
                 replace_topics: Optional[frozenset] = None,
                 remove_episode_dir: Optional[Callable[[int], None]] = None) -> Opening:
    """에피소드 행을 확보하고 이번에 무엇을 넣을지 정한다. **커밋하지 않는다.**

    Args:
        origin: 이 에피소드가 나온 녹화본. ``None`` 이면 출처를 기록하지도 대조하지도 않는다
            (단위 시험 · 옛 호출자 호환).
        policy: `on_existing_episode` — skip · replace · error.
        units: 이번 실행에서 적재할 수 있는 토픽 → (테이블, topic_id).
        replace_topics: ``None`` 이 아니면 `--only-topics-replace` 다 — 이 토픽들은 지우고 다시
            넣고, 나머지는 건드리지 않는다.
        remove_episode_dir: `replace` 에서 bag 인코딩 mp4 디렉터리를 지우는 콜백.
    """
    existing = sess_writer.find_existing(ep.start_ns)
    if existing is None:
        episode_id = sess_writer.insert_episode(ep.start_ns, ep.stop_ns, ep.task_label,
                                                ep.success, ep.metadata)
        if origin is not None:
            _link_source(conn, episode_id, origin, adopted=False)
        return Opening(episode_id=episode_id)

    # ① 같은 키의 **다른** 에피소드인지부터 본다 — 정책과 무관하게 거부한다. `replace` 라도
    #    다른 녹화본의 에피소드를 지우면 안 된다.
    recorded = source_of(conn, existing) if origin is not None else None
    if origin is not None and recorded is not None and recorded != origin.source_id:
        raise PostProcError(
            f'episode key collision: start_ns={ep.start_ns} is already episode {existing} '
            f'from recording {recorded!r}, not {origin.source_id!r} — the sim clock was '
            f'probably rewound between recordings; import them into separate databases '
            f'or move the imported bag out of rosbag_dir')
    stop_ns = _stop_ns(conn, existing)
    if stop_ns != ep.stop_ns:
        raise PostProcError(
            f'episode key collision: start_ns={ep.start_ns} is already episode {existing} '
            f'with stop_ns={stop_ns}, but this bag says {ep.stop_ns}')
    adopted = False
    if origin is not None and recorded is None:
        _link_source(conn, existing, origin, adopted=True)
        adopted = True

    if replace_topics is not None:
        for topic in sorted(replace_topics):
            if topic in units:
                table, topic_id = units[topic]
                delete_unit(conn, existing, topic_id, table)
        keep = loaded_topics(conn, existing,
                             {t: v for t, v in units.items() if t not in replace_topics})
        return Opening(episode_id=existing, skip_topics=frozenset(keep), existed=True,
                       adopted=adopted, replace_topics=frozenset(replace_topics))

    if policy == 'error':
        raise PostProcError(f'episode already present (start_ns={ep.start_ns}, id={existing})')
    if policy == 'replace':
        sess_writer.delete_by_id(existing)
        if remove_episode_dir is not None:
            remove_episode_dir(existing)
        episode_id = sess_writer.insert_episode(ep.start_ns, ep.stop_ns, ep.task_label,
                                                ep.success, ep.metadata)
        if origin is not None:
            _link_source(conn, episode_id, origin, adopted=False)
        return Opening(episode_id=episode_id, existed=True, replaced=True)
    if policy == 'skip':
        done = loaded_topics(conn, existing, units)
        return Opening(episode_id=existing, skip_topics=frozenset(done), existed=True,
                       adopted=adopted)
    raise PostProcError(f'unknown on_existing_episode policy: {policy!r}')


def register_recordings(conn: psycopg.Connection, registrar, opening: Opening,
                        episode_ns: int, *,
                        only_topics: Optional[frozenset] = None) -> dict:
    """레코더 mp4 를 단위 규칙대로 등록한다 — 이미 등록된 카메라는 건너뛰고, 교체 대상은
    지우고 다시 넣는다. 등록한 단위는 `ingest_units` 에 적는다."""
    replace = opening.replace_topics if opening.existed else frozenset()
    registered = registrar.register(opening.episode_id, episode_ns, only_topics=only_topics,
                                    replace_topics=replace)
    for topic, topic_id, frames in registrar.last_registered:
        complete_unit(conn, opening.episode_id, topic_id, 'image_streams', frames)
        opening.unit_counts[topic] = frames
    return registered


def finish_units(conn: psycopg.Connection, opening: Opening, written: Mapping[str, tuple]) -> None:
    """메시지 writer 가 적재한 단위를 기록한다. `written` 은 토픽 → (테이블, topic_id, 행 수)."""
    for topic, (table, topic_id, rows) in written.items():
        if topic in opening.skip_topics:
            continue
        complete_unit(conn, opening.episode_id, topic_id, table, rows)
        opening.unit_counts[topic] = rows


__all__ = ['META_HINT', 'Opening', 'Origin', 'complete_unit', 'delete_unit',
           'finish_units', 'loaded_topics', 'make_writer', 'open_episode', 'record_source',
           'register_recordings', 'source_of']
