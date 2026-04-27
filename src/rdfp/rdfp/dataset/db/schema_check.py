"""DB 스키마 사전 검증.

후처리 시작 시점에 아래 항목을 확인한다 (설계서 5.3).
  1. `sessions` 테이블 존재 + 필수 컬럼 존재.
  2. `topics` 테이블 존재 + 필수 컬럼 존재.
  3. 대상 토픽의 메시지 타입에 대응되는 테이블이 존재.
  4. 각 비-session 테이블에 `episode_id` 컬럼이 존재하고 `sessions(id)` 를
     참조하는 FK 가 정의되어 있음.
  5. 각 비-session 테이블에 `topic_id` 컬럼이 존재하고 `topics(id)` 를
     참조하는 FK 가 정의되어 있음.

하나라도 실패하면 `SchemaCheckError` 를 발생시킨다.
"""

from __future__ import annotations

from typing import Iterable

import psycopg


# 각 대상 테이블의 필수 컬럼 리스트 (생성 컬럼·인덱스는 검증 대상 아님).
REQUIRED_COLUMNS: dict[str, list[str]] = {
    'sessions':        ['id', 'start_sec', 'start_nanosec', 'stop_sec', 'stop_nanosec',
                        'task_label'],
    'topics':          ['id', 'topic_name', 'topic_type'],
    'pose_stampeds':   ['id', 'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec',
                        'position', 'orientation'],
    'twist_stampeds':  ['id', 'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec',
                        'twist'],
    'joint_states':    ['id', 'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec',
                        'position', 'velocity', 'effort'],
    'joint_jogs':      ['id', 'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec',
                        'joint_names', 'displacements', 'velocities', 'duration'],
    'target_joint_states': ['id', 'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec',
                            'positions', 'velocities', 'accelerations', 'effort',
                            'tfs_sec', 'tfs_nanosec'],
    'gripper_cmds':    ['id', 'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec',
                        'command'],
    'gripper_states':  ['id', 'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec',
                        'position', 'effort', 'stalled', 'reached_goal'],
    'image_frames':    ['id', 'episode_id', 'topic_id', 'frame_index',
                        'stamp_sec', 'stamp_nanosec'],
    'image_streams':   ['id', 'episode_id', 'topic_id', 'mp4_path', 'codec',
                        'pixel_format', 'container_fps', 'frame_id',
                        'width', 'height', 'frame_count', 'created_at'],
}


# sessions 와 topics 는 자식 row 가 FK 로 참조하는 룩업 테이블이며, 비-session/
# 비-topics 테이블만 FK 검증 루프(_verify_fk) 의 대상이다.
_LOOKUP_TABLES: frozenset[str] = frozenset({'sessions', 'topics'})


class SchemaCheckError(RuntimeError):
    """스키마 검증 실패를 나타내는 예외."""


def ensure_schema(
    conn: psycopg.Connection,
    required_tables: Iterable[str],
) -> None:
    """요구되는 테이블/컬럼/FK 가 모두 갖춰졌는지 검증한다.

    Args:
        conn: 열려 있는 PostgreSQL 커넥션.
        required_tables: 반드시 존재해야 하는 테이블 이름 목록. `sessions` 는
            호출 측이 포함하지 않아도 자동 추가된다.

    Raises:
        SchemaCheckError: 테이블 누락, 컬럼 누락, 또는 episode_id/topic_id FK 누락.
    """
    needed = set(required_tables) | {'sessions', 'topics'}
    unknown = needed - set(REQUIRED_COLUMNS)
    if unknown:
        raise SchemaCheckError(
            f'schema check asked for unknown tables (not registered): {sorted(unknown)}'
        )

    with conn.cursor() as cur:
        cur.execute("""
            SELECT table_name, column_name
            FROM   information_schema.columns
            WHERE  table_schema = current_schema()
              AND  table_name = ANY(%s)
        """, (list(needed),))
        present: dict[str, set[str]] = {}
        for table, col in cur.fetchall():
            present.setdefault(table, set()).add(col)

    missing_tables = [t for t in needed if t not in present]
    if missing_tables:
        raise SchemaCheckError(
            f'required tables not found in schema: {sorted(missing_tables)}'
        )

    for table in needed:
        required = set(REQUIRED_COLUMNS[table])
        missing_cols = required - present[table]
        if missing_cols:
            raise SchemaCheckError(
                f'table {table!r} is missing columns: {sorted(missing_cols)}'
            )

    _verify_fk(conn, needed, column='episode_id', parent_table='sessions',
               parent_column='id')
    _verify_fk(conn, needed, column='topic_id', parent_table='topics',
               parent_column='id')


def _verify_fk(conn: psycopg.Connection, needed: Iterable[str], *,
               column: str, parent_table: str, parent_column: str,) -> None:
    """비-session/topics 테이블의 FK 컬럼이 지정 부모를 참조하는지 확인한다."""
    children = [t for t in needed if t not in _LOOKUP_TABLES]
    if not children:
        return
    with conn.cursor() as cur:
        cur.execute("""
            SELECT    tc.table_name
            FROM      information_schema.table_constraints  AS tc
            JOIN      information_schema.key_column_usage   AS kcu
                ON    tc.constraint_name = kcu.constraint_name
                AND   tc.table_schema    = kcu.table_schema
            JOIN      information_schema.constraint_column_usage AS ccu
                ON    tc.constraint_name = ccu.constraint_name
                AND   tc.table_schema    = ccu.table_schema
            WHERE     tc.constraint_type = 'FOREIGN KEY'
              AND     tc.table_schema    = current_schema()
              AND     tc.table_name      = ANY(%s)
              AND     kcu.column_name    = %s
              AND     ccu.table_name     = %s
              AND     ccu.column_name    = %s
        """, (children, column, parent_table, parent_column))
        ok = {row[0] for row in cur.fetchall()}

    missing = [t for t in children if t not in ok]
    if missing:
        raise SchemaCheckError(
            f'tables missing {column} FK to {parent_table}({parent_column}): '
            f'{sorted(missing)}'
        )


__all__ = ['REQUIRED_COLUMNS', 'SchemaCheckError', 'ensure_schema']
