"""sessions 테이블 writer.

다른 writer 와 달리 메시지 단위 append 가 아니라, 에피소드 감지 후
`(start_ns, stop_ns, task_label)` 을 받아 1 행을 INSERT 하고 생성된
`id` 를 반환한다. 반환된 `id` 가 비-session 테이블의 `episode_id` 로
사용된다.
"""

from __future__ import annotations

import psycopg


NS_PER_SEC = 1_000_000_000


class SessionWriter:
    """sessions 테이블에 에피소드 레코드를 INSERT 한다."""

    table = 'sessions'

    def __init__(self, conn: psycopg.Connection) -> None:
        self._conn = conn

    def insert_episode(
        self,
        start_ns: int,
        stop_ns: int,
        task_label: str | None,
    ) -> int:
        """에피소드를 INSERT 하고 생성된 `id` 를 반환한다.

        UNIQUE(start_sec, start_nanosec) 제약 위반 시 `psycopg.errors.UniqueViolation`
        이 발생한다. 상위(pipeline) 에서 on_existing_episode 정책에 따라 처리한다.
        """
        start_sec, start_nanosec = _split_ns(start_ns)
        stop_sec, stop_nanosec = _split_ns(stop_ns)
        sql = """
            INSERT INTO sessions
                (start_sec, start_nanosec, stop_sec, stop_nanosec, task_label)
            VALUES
                (%s, %s, %s, %s, %s)
            RETURNING id
        """
        with self._conn.cursor() as cur:
            cur.execute(sql, (start_sec, start_nanosec, stop_sec, stop_nanosec, task_label))
            row = cur.fetchone()
            return int(row[0])

    def find_existing(self, start_ns: int) -> int | None:
        """동일 `(start_sec, start_nanosec)` 에피소드의 `id` 를 조회한다."""
        start_sec, start_nanosec = _split_ns(start_ns)
        sql = 'SELECT id FROM sessions WHERE start_sec = %s AND start_nanosec = %s'
        with self._conn.cursor() as cur:
            cur.execute(sql, (start_sec, start_nanosec))
            row = cur.fetchone()
            return int(row[0]) if row else None

    def delete_by_id(self, episode_id: int) -> None:
        """`on_existing_episode = replace` 시 기존 에피소드를 제거한다.

        자식 테이블은 FK `ON DELETE CASCADE` 로 함께 삭제된다.
        """
        with self._conn.cursor() as cur:
            cur.execute('DELETE FROM sessions WHERE id = %s', (episode_id,))


def _split_ns(ns: int) -> tuple[int, int]:
    """epoch ns 를 `(sec, nanosec)` 로 분할한다 (음수 입력은 고려하지 않음)."""
    return ns // NS_PER_SEC, ns % NS_PER_SEC


__all__ = ['SessionWriter']
