"""에피소드의 메시지를 DB 에서 읽어 리스트로 반환하는 모듈.

에피소드 id + 토픽 이름 또는 토픽 타입을 받아 해당 토픽의 row 를 stamp
오름차순으로 조회하고 ROS 2 메시지 인스턴스로 복원해 리스트로 반환한다.
타입별 복원은 `registry.MESSAGE_TYPE_REGISTRY` 에 등록된 `ReaderBase`
서브클래스가 담당하며, 각 reader 는 메시지 패키지를 lazy import 로 로드한다.

`header.frame_id` 는 DB 에 저장되지 않으므로 복원 메시지에서 설정하지
않으며, 메시지 기본값 ('') 이 유지된다.

공개 함수:
    read_topic_messages_by_name(conn, episode_id, topic_name)
    read_topic_messages_by_type(conn, episode_id, topic_type)
"""

from __future__ import annotations

from typing import Any, Optional

import psycopg
from psycopg import sql

from .readers.base import ReaderBase
from .registry import resolve_message_type


def read_topic_messages_by_name(conn: psycopg.Connection, episode_id: int,
                                topic_name: str) -> list[Any]:
    """지정 에피소드에서 지정 `topic_name` 의 메시지 리스트를 stamp 오름차순으로 반환한다.

    Args:
        conn: 열려있는 커넥션.
        episode_id: `sessions.id`.
        topic_name: 조회 대상 토픽 이름 (`topics.topic_name`).

    Returns:
        복원된 ROS 2 메시지 인스턴스 리스트 (stamp 오름차순). 해당 에피소드에
        row 가 없으면 빈 리스트.

    Raises:
        ValueError: topic 이 `topics` 에 없거나 등록된 바인딩이 없는 경우.
    """
    topic_id, topic_type = _lookup_topic_by_name(conn, topic_name)
    return _read_topic_messages(conn, episode_id, topic_type, topic_id=topic_id)


def read_topic_messages_by_type(conn: psycopg.Connection, episode_id: int,
                                topic_type: str) -> list[Any]:
    """지정 에피소드에서 지정 `topic_type` 의 메시지 리스트를 stamp 오름차순으로 반환한다.

    동일 타입을 쓰는 토픽이 여러 개라도 `topic_id` 필터 없이 대상 테이블을
    `ORDER BY stamp_sec, stamp_nanosec` 로 조회하므로, DB 가 모든 토픽의
    row 를 stamp 순으로 섞어서 반환한다.

    Args:
        conn: 열려있는 커넥션.
        episode_id: `sessions.id`.
        topic_type: ROS 2 메시지 타입 문자열 (예: 'geometry_msgs/msg/TwistStamped').

    Returns:
        복원된 ROS 2 메시지 인스턴스 리스트 (stamp 오름차순). 해당 에피소드에
        row 가 없으면 빈 리스트.

    Raises:
        ValueError: 등록된 바인딩이 없는 경우.
    """
    return _read_topic_messages(conn, episode_id, topic_type)


# ---------------------------------------------------------------------------
# 내부 공용 로직.
# ---------------------------------------------------------------------------

def _read_topic_messages(conn: psycopg.Connection, episode_id: int, topic_type: str,
                         topic_id: Optional[int] = None) -> list[Any]:
    """`topic_type` + 선택적 `topic_id` 에 해당하는 메시지 리스트를 반환한다.

    이름 모드는 `topic_id` 를 함께 넘겨 단일 토픽의 row 만 조회하고, 타입
    모드는 `topic_id=None` 으로 호출되어 해당 타입의 테이블 전체를 stamp
    순으로 조회한다. 실제 SQL 실행과 row 복원은 `_execute_query` 가 담당하며
    리스트 형태로 반환된다 (cursor 는 함수 반환 시점에 이미 닫혀 있다).

    Args:
        conn: 열려있는 커넥션.
        episode_id: `sessions.id`.
        topic_type: ROS 2 메시지 타입 문자열.
        topic_id: 조회 대상 토픽 id. None 이면 topic_id 조건 없이 조회한다.

    Returns:
        복원된 ROS 2 메시지 인스턴스 리스트 (stamp 오름차순). 해당 에피소드에
        row 가 없으면 빈 리스트.

    Raises:
        ValueError: `MESSAGE_TYPE_REGISTRY` 에 해당 타입에 대한 바인딩이
            없는 경우.
    """
    binding = resolve_message_type(topic_type)
    if binding is None:
        raise ValueError(f'no binding registered for type {topic_type!r}')
    return _execute_query(conn, episode_id, binding.table, binding.reader_cls, topic_id)


def _execute_query(conn: psycopg.Connection, episode_id: int,
                   table: str, reader_cls: type[ReaderBase],
                   topic_id: Optional[int]) -> list[Any]:
    """`table` 에서 `reader_cls.select_cols` 로 row 를 조회·복원해 리스트로 반환한다.

    topic_id 가 None 이면 topic_id 조건 없이 episode_id 만으로 조회한다. 결과
    전체를 한 번에 fetchall() 해서 리스트로 돌려주므로, cursor 수명은 함수
    반환 시점에 이미 종료되어 있다.

    Args:
        conn: 열려있는 커넥션.
        episode_id: `sessions.id`.
        table: 조회 대상 테이블 이름 (registry 의 `TypeBinding.table`).
        reader_cls: `ReaderBase` 서브클래스. `select_cols` 와 `build(row)` 를
            제공한다.
        topic_id: 조회 대상 topic_id. None 이면 topic_id 조건 없이 조회한다.

    Returns:
        복원된 ROS 2 메시지 인스턴스 리스트 (stamp 오름차순). 해당 에피소드에
        row 가 없으면 빈 리스트.
    """
    # 동적 컬럼/테이블 이름을 안전하게 주입하기 위해 psycopg.sql.Identifier 로
    # 감싼다. 파라미터 바인딩 (%s) 은 그대로 placeholder 형태로 둔다.
    cols_sql = sql.SQL(', ').join(sql.Identifier(c) for c in reader_cls.select_cols)
    table_sql = sql.Identifier(table)
    if topic_id is not None:
        where_sql = sql.SQL('episode_id = %s AND topic_id = %s')
        params: tuple = (episode_id, topic_id)
    else:
        where_sql = sql.SQL('episode_id = %s')
        params = (episode_id,)

    query = sql.SQL(
        'SELECT {cols} FROM {table} WHERE {where} '
        'ORDER BY stamp_sec ASC, stamp_nanosec ASC, id ASC'
    ).format(cols=cols_sql, table=table_sql, where=where_sql)

    with conn.cursor() as cur:
        cur.execute(query, params)
        rows = cur.fetchall()
    return [reader_cls.build(row) for row in rows]


def _lookup_topic_by_name(conn: psycopg.Connection, topic_name: str) -> tuple[int, str]:
    """`topic_name` 으로 `topics` 에서 단일 행을 조회해 `(topic_id, topic_type)` 을 반환한다.

    Args:
        conn: 열려있는 커넥션.
        topic_name: 조회 대상 토픽 이름 (`topics.topic_name`).

    Returns:
        `(topic_id, topic_type)` 2-tuple.

    Raises:
        ValueError: `topic_name` 이 `topics` 에 없을 때.
    """
    with conn.cursor() as cur:
        cur.execute('SELECT id, topic_type FROM topics WHERE topic_name = %s', (topic_name,))
        row = cur.fetchone()
    if row is None:
        raise ValueError(f'topic name {topic_name!r} not found in topics table')
    return (int(row[0]), str(row[1]))


__all__ = [
    'read_topic_messages_by_name',
    'read_topic_messages_by_type'
]
