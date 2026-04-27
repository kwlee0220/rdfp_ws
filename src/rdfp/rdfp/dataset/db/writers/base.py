"""배치 INSERT writer 의 공통 베이스."""

from __future__ import annotations

from typing import Any

import psycopg


class WriterBase:
    """에피소드 경계 혹은 버퍼 임계에서 flush 되는 배치 INSERT writer.

    서브클래스는 다음을 구현한다.
      * 클래스 속성 `table`, `columns` (episode_id, topic_id, stamp_sec,
        stamp_nanosec 등을 포함한 INSERT 컬럼 순서).
      * `row_values(episode_id, msg)` : 메시지 → 튜플 변환 (컬럼 순서).
        `topic_id` 는 `self.topic_id` 에서 읽어 튜플에 포함시킨다.

    `__init__` 의 `table` 인자로 인스턴스 단위 테이블명을 덮어쓸 수 있다
    (클래스 속성은 건드리지 않는다).
    """

    table: str = ''
    columns: tuple[str, ...] = ()

    def __init__(self, conn: psycopg.Connection, batch_size: int = 1000,
                 table: str | None = None, topic_id: int | None = None,) -> None:
        self._conn = conn
        self._batch_size = batch_size
        self._buffer: list[tuple[Any, ...]] = []
        self._inserted_since_reset: int = 0
        # INSERT 시 topic_id 컬럼에 채울 값. topics 테이블의 id 를 가리킨다.
        # 테스트 환경에서 row_values 변환만 검증할 때는 None 으로 두고 기대
        # 튜플에 None 을 그대로 반영한다.
        self.topic_id = topic_id
        if table is not None:
            # 클래스 기본 테이블명을 인스턴스 속성으로 덮어쓴다. 동일 writer
            # 클래스가 동시에 여러 테이블을 쓸 수 있도록 클래스 속성은 수정하지
            # 않는다 (shared mutable 방지).
            self.table = table

    def append(self, episode_id: int, msg: Any) -> None:
        """메시지를 현재 버퍼에 추가한다. 임계 초과 시 flush 한다."""
        self._buffer.append(self.row_values(episode_id, msg))
        if len(self._buffer) >= self._batch_size:
            self.flush()

    def flush(self) -> None:
        """버퍼에 쌓인 행을 executemany 로 일괄 INSERT 한다."""
        if not self._buffer:
            return
        placeholders = ', '.join(['%s'] * len(self.columns))
        col_list = ', '.join(self.columns)
        sql = f'INSERT INTO {self.table} ({col_list}) VALUES ({placeholders})'
        with self._conn.cursor() as cur:
            cur.executemany(sql, self._buffer)
        self._inserted_since_reset += len(self._buffer)
        self._buffer.clear()

    def consume_inserted_count(self) -> int:
        """마지막 reset 이후 실제로 INSERT 된 행 수를 반환하고 카운터를 0 으로 리셋한다."""
        n = self._inserted_since_reset
        self._inserted_since_reset = 0
        return n

    def drop_pending(self) -> None:
        """ROLLBACK 시 버퍼에 쌓여있던 (아직 flush 되지 않은) 행을 제거한다."""
        self._buffer.clear()
        self._inserted_since_reset = 0

    def row_values(self, episode_id: int, msg: Any) -> tuple[Any, ...]:
        raise NotImplementedError


def extract_stamp(msg: Any) -> tuple[int, int]:
    """메시지의 `header.stamp` 를 `(sec, nanosec)` 로 추출한다."""
    stamp = msg.header.stamp
    return int(stamp.sec), int(stamp.nanosec)


__all__ = ['WriterBase', 'extract_stamp']
