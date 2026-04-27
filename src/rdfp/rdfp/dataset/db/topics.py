"""`topics` 룩업 테이블 upsert 헬퍼.

import 파이프라인 시작 시점에 적재 대상 토픽의 (이름, 타입) 을 `topics`
테이블에 등록하고, 각 토픽의 `id` 를 writer 에 주입한다. 이미 존재하는
행은 재사용하고 없는 행만 INSERT 한다 (ON CONFLICT DO NOTHING — 기존 행의
topic_type 은 보존되어 최초 등록 시점의 타입이 유지된다).
"""

from __future__ import annotations

from typing import Mapping

import psycopg


def upsert_topic_ids(conn: psycopg.Connection,
                     topic_types: Mapping[str, str]) -> dict[str, int]:
    """주어진 토픽 (이름, 타입) 을 `topics` 에 upsert 하고 `{topic_name: id}` 를 반환한다.

    `topic_name` 에 UNIQUE 제약이 걸려 있으므로 ON CONFLICT 로 멱등 처리한다.
    본 함수는 커밋을 수행하지 않는다 — 호출자 트랜잭션에 속한다.
    """
    items = sorted((n, t) for n, t in topic_types.items() if n)
    if not items:
        return {}
    names = [n for n, _ in items]
    sql_insert = (
        'INSERT INTO topics (topic_name, topic_type) VALUES (%s, %s) '
        'ON CONFLICT (topic_name) DO NOTHING'
    )
    sql_select = 'SELECT topic_name, id FROM topics WHERE topic_name = ANY(%s)'
    with conn.cursor() as cur:
        cur.executemany(sql_insert, items)
        cur.execute(sql_select, (names,))
        rows = cur.fetchall()
    return {str(name): int(tid) for name, tid in rows}


__all__ = ['upsert_topic_ids']
