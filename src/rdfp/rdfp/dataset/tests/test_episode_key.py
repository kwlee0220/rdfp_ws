"""에피소드 자연키 — 테이블 이름 계약과 실 DB 조회.

`fetch_exported_keys` 의 SQL 은 **모킹으로 검증되지 않는다** — 가짜 커서는 어떤
테이블 이름이든 받아 주므로, 한때 있던 ``FROM episodes``(실제 이름은 `sessions`)
버그를 단위 시험이 하나도 잡지 못했다. 실 DB 적재 시험에서야 드러났다.

그래서 두 겹으로 막는다.

1. **스키마 계약 시험** — DB 없이 늘 돈다. `fetch_exported_keys` 가 실제로 만드는
   SQL 에서 테이블·컬럼 이름을 뽑아 `sql/schema.sql` 과 대조한다.
2. **실 DB 시험** — ``RDFP_TEST_DB_DSN`` 이 있을 때만 돈다. `RDFP_DB_DSN` 과
   **일부러 다른 이름**이다 — 셸에 남은 운영 DSN 으로 시험이 돌면 안 된다.
"""

from __future__ import annotations

import os
import re
import uuid
from pathlib import Path

import pytest

from rdfp.dataset.episode_key import (
    NS_PER_SEC, fetch_exported_keys, join_ns, key_range, split_ns,
)

SCHEMA_SQL = Path(__file__).resolve().parents[1] / 'sql' / 'schema.sql'
TEST_DSN_ENV = 'RDFP_TEST_DB_DSN'


# ----- 순수 함수 --------------------------------------------------------------


def test_split_and_join_are_inverse() -> None:
    ns = 1_757_000_000_123_456_789
    assert join_ns(*split_ns(ns)) == ns
    assert split_ns(ns) == (1_757_000_000, 123_456_789)
    assert split_ns(NS_PER_SEC) == (1, 0)


def test_key_range() -> None:
    assert key_range([]) == (None, None)
    assert key_range([30, 10, 20]) == (10, 30)
    assert key_range([7]) == (7, 7)


# ----- 스키마 계약 ------------------------------------------------------------


class _CapturingConn:
    """실행된 SQL 만 기록하고 빈 결과를 돌려주는 커넥션."""

    def __init__(self) -> None:
        self.sql: list[str] = []
        self.params: list[tuple] = []

    def cursor(self):
        this = self

        class _Cur:

            def __enter__(self):
                return self

            def __exit__(self, *exc):
                return False

            def execute(self, sql, params=()):
                this.sql.append(sql)
                this.params.append(params)

            def fetchall(self):
                return []

        return _Cur()


def _executed_sql(**kwargs) -> str:
    conn = _CapturingConn()
    fetch_exported_keys(conn, **kwargs)
    assert len(conn.sql) == 1
    return conn.sql[0]


def _schema_table_block(table: str) -> str:
    """`schema.sql` 의 `CREATE TABLE <table> (...)` 본문."""
    sql = SCHEMA_SQL.read_text(encoding='utf-8')
    m = re.search(rf'CREATE TABLE IF NOT EXISTS\s+{re.escape(table)}\s*\((.*?)\n\);',
                  sql, re.S)
    assert m is not None, f'schema.sql 에 {table} 테이블이 없다'
    return m.group(1)


def test_queried_table_exists_in_schema() -> None:
    """질의하는 테이블이 스키마에 실제로 있어야 한다.

    `episodes` 로 적었던 버그를 잡는 시험이다 — 개념상의 이름과 스키마의 이름이
    다르기 때문에 사람이 틀리기 쉽다.
    """
    m = re.search(r'\bFROM\s+(\w+)', _executed_sql())
    assert m is not None
    _schema_table_block(m.group(1))      # 없으면 assert 로 실패한다


def test_queried_columns_exist_in_that_table() -> None:
    """`SELECT` 컬럼과 `WHERE` 컬럼이 모두 그 테이블의 것이어야 한다."""
    sql = _executed_sql(start_ns_lo=0, start_ns_hi=NS_PER_SEC)
    table = re.search(r'\bFROM\s+(\w+)', sql).group(1)
    block = _schema_table_block(table)

    selected = re.search(r'SELECT\s+(.*?)\s+FROM', sql).group(1)
    columns = {c.strip() for c in selected.split(',')}
    columns |= set(re.findall(r'(\w+)\s*[<>]=?\s*%s', sql))

    for col in columns:
        assert re.search(rf'^\s*{col}\s', block, re.M), f'{table} 에 {col} 컬럼이 없다'


def test_natural_key_is_unique_in_schema() -> None:
    """자연키 조회가 성립하려면 `(start_sec, start_nanosec)` 이 유일해야 한다."""
    table = re.search(r'\bFROM\s+(\w+)', _executed_sql()).group(1)

    assert 'UNIQUE (start_sec, start_nanosec)' in _schema_table_block(table)


def test_bounds_narrow_on_start_sec_not_joined_ns() -> None:
    """ns 를 SQL 쪽에서 합치면 인덱스를 타지 못한다 — `start_sec` 으로 좁힌다."""
    lo, hi = 1_757_000_000_500_000_000, 1_757_000_009_500_000_000

    conn = _CapturingConn()
    fetch_exported_keys(conn, start_ns_lo=lo, start_ns_hi=hi)

    assert 'start_sec >= %s' in conn.sql[0] and 'start_sec <= %s' in conn.sql[0]
    # 하한은 내림, 상한은 +1 로 넉넉히 — 경계 에피소드를 잘라내면 안 된다.
    assert conn.params[0] == (split_ns(lo)[0], split_ns(hi)[0] + 1)


def test_no_where_clause_without_bounds() -> None:
    assert 'WHERE' not in _executed_sql()


# ----- 실 DB ------------------------------------------------------------------


@pytest.fixture()
def db_schema():
    """격리된 임시 스키마에 `sessions` 를 만들고, 끝나면 통째로 지운다.

    사용자의 기존 테이블을 절대 건드리지 않는다.
    """
    dsn = os.environ.get(TEST_DSN_ENV, '').strip()
    if not dsn:
        pytest.skip(f'{TEST_DSN_ENV} not set')
    psycopg = pytest.importorskip('psycopg')

    schema = f'rdfp_test_{uuid.uuid4().hex[:8]}'
    with psycopg.connect(dsn, autocommit=True) as conn:
        conn.execute(f'CREATE SCHEMA {schema}')
        try:
            conn.execute(f'SET search_path TO {schema}')
            conn.execute(SCHEMA_SQL.read_text(encoding='utf-8'))
            yield conn
        finally:
            conn.execute(f'DROP SCHEMA {schema} CASCADE')


def _insert(conn, start_ns: int, *, label: str = 'peg') -> None:
    sec, nanosec = split_ns(start_ns)
    conn.execute(
        'INSERT INTO sessions (start_sec, start_nanosec, stop_sec, stop_nanosec, task_label)'
        ' VALUES (%s, %s, %s, %s, %s)',
        (sec, nanosec, sec + 12, nanosec, label))


def test_fetch_exported_keys_against_real_db(db_schema) -> None:
    """적재된 `start_ns` 를 실제로 찾아낸다 — 테이블 이름이 틀리면 여기서 죽는다."""
    keys = [1_757_000_000_123_456_789, 1_757_000_200_000_000_000]
    for k in keys:
        _insert(db_schema, k)

    assert fetch_exported_keys(db_schema) == set(keys)


def test_fetch_exported_keys_empty_table(db_schema) -> None:
    assert fetch_exported_keys(db_schema) == set()


def test_fetch_exported_keys_honors_bounds(db_schema) -> None:
    """구간 밖은 빼고, 경계에 걸친 것은 남긴다."""
    inside = 1_757_000_100_500_000_000
    outside = 1_757_009_999_000_000_000
    _insert(db_schema, inside)
    _insert(db_schema, outside)

    got = fetch_exported_keys(db_schema, start_ns_lo=inside - NS_PER_SEC,
                              start_ns_hi=inside + NS_PER_SEC)

    assert got == {inside}
