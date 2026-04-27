"""db.initialize 단위 테스트.

실제 DB 연결은 실환경 검증에서 수행한다. 여기서는 SQL 파일 로드와
initialize_schema() 의 인자 처리 / 호출 흐름 (psycopg.connect mock) 을
검증한다.
"""

from __future__ import annotations

import pytest

from rdfp.dataset.db.initialize import (
    initialize_schema,
    read_drop_sql,
    read_schema_sql,
    sql_dir,
)


def test_sql_dir_exists() -> None:
    # 개발 환경 fallback (패키지 상대 경로) 또는 설치 경로 어느 쪽이든
    # 존재해야 한다.
    assert sql_dir().is_dir()


def test_read_schema_sql_contains_expected_tables() -> None:
    text = read_schema_sql()
    for table in ('sessions', 'pose_stampeds', 'twist_stampeds',
                  'joint_states', 'target_joint_states',
                  'gripper_cmds', 'gripper_states'):
        assert f'CREATE TABLE IF NOT EXISTS {table}' in text


def test_read_drop_sql_drops_known_tables() -> None:
    text = read_drop_sql()
    for table in ('sessions', 'pose_stampeds', 'twist_stampeds',
                  'joint_states', 'target_joint_states',
                  'gripper_cmds', 'gripper_states'):
        assert f'DROP TABLE IF EXISTS {table}' in text


# ---- initialize_schema() mock 기반 흐름 검증 ----


class _FakeCursor:

    def __init__(self, log: list) -> None:
        self._log = log

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def execute(self, sql, params=None):
        # 전체 SQL 원문을 그대로 저장해 테스트에서 substring 검색이 가능하도록 한다.
        self._log.append(sql)


class _FakeConn:

    def __init__(self, log: list) -> None:
        self._log = log
        self.committed = False

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def cursor(self):
        return _FakeCursor(self._log)

    def commit(self):
        self.committed = True
        self._log.append('__COMMIT__')


def test_initialize_public_schema(monkeypatch) -> None:
    log: list = []
    monkeypatch.setattr(
        'rdfp.dataset.db.initialize.psycopg.connect',
        lambda dsn, autocommit: _FakeConn(log),
    )
    initialize_schema('postgresql://x', schema='public')
    # SET search_path + schema.sql 실행이 포함되어야 한다.
    assert any('SET search_path TO "public"' in s for s in log)
    assert any('CREATE TABLE IF NOT EXISTS sessions' in s for s in log)
    assert '__COMMIT__' in log


def test_initialize_custom_schema_creates_schema(monkeypatch) -> None:
    log: list = []
    monkeypatch.setattr(
        'rdfp.dataset.db.initialize.psycopg.connect',
        lambda dsn, autocommit: _FakeConn(log),
    )
    initialize_schema('postgresql://x', schema='rdfp_v1')
    assert any('CREATE SCHEMA IF NOT EXISTS "rdfp_v1"' in s for s in log)
    assert any('SET search_path TO "rdfp_v1"' in s for s in log)


def test_initialize_with_drop_first_runs_drop_sql(monkeypatch) -> None:
    log: list = []
    monkeypatch.setattr(
        'rdfp.dataset.db.initialize.psycopg.connect',
        lambda dsn, autocommit: _FakeConn(log),
    )
    initialize_schema('postgresql://x', schema='public', drop_first=True)
    # drop.sql 의 DROP 문이 먼저, 그 뒤에 CREATE TABLE 이 실행되어야 한다.
    drop_idx = next(
        i for i, s in enumerate(log) if 'DROP TABLE IF EXISTS joint_states' in s
    )
    create_idx = next(
        i for i, s in enumerate(log) if 'CREATE TABLE IF NOT EXISTS sessions' in s
    )
    assert drop_idx < create_idx


def test_initialize_reraises_missing_sql(monkeypatch, tmp_path) -> None:
    # sql_dir() 이 존재하지 않는 경로를 반환하도록 강제.
    from rdfp.dataset.db import initialize as mod
    monkeypatch.setattr(mod, 'sql_dir', lambda: tmp_path / 'no_such')
    with pytest.raises(FileNotFoundError):
        read_schema_sql()
