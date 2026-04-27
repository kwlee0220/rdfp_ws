"""PostgreSQL 커넥션 헬퍼.

DSN 은 설정 파일에 평문으로 두지 않고 `db.dsn_env` 에 지정된 환경변수에서
읽어온다. 본 후처리기는 단일 스레드 배치 프로세스이므로 풀을 두지 않고
단일 커넥션 + 에피소드 단위 트랜잭션을 사용한다.
"""

from __future__ import annotations

from typing import Iterator

from contextlib import contextmanager

import psycopg
from psycopg import sql

from .config import DbConfig, resolve_dsn


@contextmanager
def open_connection(cfg: DbConfig) -> Iterator[psycopg.Connection]:
    """설정에서 DSN 을 확보하여 커넥션을 연다.

    `autocommit=False` 로 열고, `search_path` 를 `cfg.schema_` 로 설정한다.

    `SET search_path` 는 PostgreSQL 의 utility 명령이라 `%s` 파라미터 바인딩을
    지원하지 않는다 (`syntax error at or near "$1"` 로 실패). 식별자이므로
    `psycopg.sql.Identifier` 로 안전하게 quote 하여 SQL 에 삽입한다.
    """
    dsn = resolve_dsn(cfg)
    conn = psycopg.connect(dsn, autocommit=False)
    try:
        with conn.cursor() as cur:
            cur.execute(
                sql.SQL('SET search_path TO {}').format(sql.Identifier(cfg.schema_))
            )
        conn.commit()
        yield conn
    finally:
        conn.close()


__all__ = ['open_connection']
