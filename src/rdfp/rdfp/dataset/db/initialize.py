"""DB 스키마 초기화 로직.

`sql/schema.sql` (선택적으로 `sql/drop.sql`) 을 실행해 후처리기가 요구하는
테이블/인덱스/FK 를 구축한다. 스키마가 `public` 이 아닌 경우 대상 스키마를
먼저 `CREATE SCHEMA IF NOT EXISTS` 한다.

DDL 은 `CREATE TABLE IF NOT EXISTS` 로 작성되어 있으므로 같은 명령을 반복
실행해도 안전하다.
"""

from __future__ import annotations

import logging
from pathlib import Path

import psycopg


_logger = logging.getLogger(__name__)


def sql_dir() -> Path:
    """`schema.sql` / `drop.sql` 이 위치한 디렉터리를 반환한다.

    우선 ament_index 를 통해 설치 경로(`share/rdfp/dataset/sql`) 를 조회하고,
    실패하면 패키지 상대 경로로 fallback (개발 환경: `dataset/db/../sql` =
    `dataset/sql`).
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('rdfp'))
        candidate = share / 'dataset' / 'sql'
        if candidate.is_dir():
            return candidate
    except Exception:
        pass
    return Path(__file__).parent.parent / 'sql'


def read_schema_sql() -> str:
    """`sql/schema.sql` 의 전체 내용을 문자열로 반환한다."""
    p = sql_dir() / 'schema.sql'
    if not p.is_file():
        raise FileNotFoundError(f'schema.sql not found under {sql_dir()}')
    return p.read_text(encoding='utf-8')


def read_drop_sql() -> str:
    """`sql/drop.sql` 의 전체 내용을 문자열로 반환한다."""
    p = sql_dir() / 'drop.sql'
    if not p.is_file():
        raise FileNotFoundError(f'drop.sql not found under {sql_dir()}')
    return p.read_text(encoding='utf-8')


def initialize_schema(dsn: str, schema: str = 'public', *,
                      drop_first: bool = False,) -> None:
    """지정 DSN 의 DB 에 후처리기 스키마를 생성한다.

    Args:
        dsn: PostgreSQL DSN. 예: `postgresql://user:pass@host:5432/dbname`.
        schema: 스키마 이름. `public` 이 아니면 `CREATE SCHEMA IF NOT EXISTS`
            를 선행한다.
        drop_first: True 이면 `drop.sql` 을 먼저 실행해 기존 테이블을 제거한다.
            운영 환경에서는 주의해서 사용한다.

    Raises:
        psycopg.Error: DB 실행 오류.
    """
    schema_sql = read_schema_sql()
    drop_sql = read_drop_sql() if drop_first else None

    _logger.info('connecting to db (schema=%s, drop_first=%s)', schema, drop_first)
    with psycopg.connect(dsn, autocommit=False) as conn:
        with conn.cursor() as cur:
            if schema and schema != 'public':
                cur.execute(f'CREATE SCHEMA IF NOT EXISTS "{schema}"')
            cur.execute(f'SET search_path TO "{schema}"')
            if drop_sql is not None:
                _logger.warning('dropping existing tables in schema %r', schema)
                cur.execute(drop_sql)
            cur.execute(schema_sql)
        conn.commit()
    _logger.info('schema initialized')


__all__ = [
    'sql_dir',
    'read_schema_sql',
    'read_drop_sql',
    'initialize_schema',
]
