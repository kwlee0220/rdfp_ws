"""message_reader 단위 테스트.

실제 DB 대신 `_FakeConn` / `_FakeCursor` 로 `cur.execute`, `cur.fetchone`,
그리고 cursor 순회를 흉내낸다. 테스트 전용 `ReaderBase` 서브클래스를
monkeypatch 로 `MESSAGE_TYPE_REGISTRY` 에 등록하여 ROS 2 메시지 패키지
의존성 없이 제너레이터 경로를 검증한다.
"""

from __future__ import annotations

from typing import Any

import pytest

from rdfp.dataset.db import message_reader


# --------------------------------------------------------------------------
# Fakes
# --------------------------------------------------------------------------

class _FakeCursor:
    """`with conn.cursor() as cur:` 를 흉내내는 최소 커서.

    `fake_conn.row_queue` (list[list]) 에서 매 `execute` 호출마다 한 세트의
    row 들을 꺼낸다. `fetchone` 은 첫 row, `fetchall` 은 전체 row 를 돌려준다.
    """

    def __init__(self, fake_conn: '_FakeConn') -> None:
        self._fake_conn = fake_conn
        self._current_rows: list[tuple] = []

    def __enter__(self) -> '_FakeCursor':
        return self

    def __exit__(self, *a: Any) -> bool:
        return False

    def execute(self, query: Any, params: Any = None) -> None:
        self._fake_conn.executed.append((query, params))
        if self._fake_conn.row_queue:
            self._current_rows = self._fake_conn.row_queue.pop(0)
        else:
            self._current_rows = []

    def fetchone(self) -> tuple | None:
        return self._current_rows[0] if self._current_rows else None

    def fetchall(self) -> list[tuple]:
        return list(self._current_rows)


class _FakeConn:
    """`cursor()` 만 제공하는 커넥션 스텁."""

    def __init__(self) -> None:
        self.executed: list[tuple[Any, Any]] = []
        self.row_queue: list[list[tuple]] = []

    def cursor(self) -> _FakeCursor:
        return _FakeCursor(self)


# --------------------------------------------------------------------------
# Fixtures
# --------------------------------------------------------------------------

TEST_TYPE = 'test/msg/Ping'
TEST_TABLE = 'test_pings'


@pytest.fixture
def fake_binding(monkeypatch: pytest.MonkeyPatch):
    """테스트 전용 TypeBinding 을 `MESSAGE_TYPE_REGISTRY` 에 등록한다."""
    from rdfp.dataset.db import registry
    from rdfp.dataset.db.readers.base import ReaderBase

    class _FakeReader(ReaderBase):
        select_cols = ('stamp_sec', 'stamp_nanosec', 'payload')

        @classmethod
        def build(cls, row: tuple[Any, ...]) -> Any:
            return row

    monkeypatch.setitem(
        registry.MESSAGE_TYPE_REGISTRY,
        TEST_TYPE,
        registry.TypeBinding(
            table=TEST_TABLE,
            writer_cls=object,  # writer 는 이 테스트에서 사용되지 않음.
            reader_cls=_FakeReader,
        ),
    )


# --------------------------------------------------------------------------
# _lookup_topic_by_name
# --------------------------------------------------------------------------

def test_lookup_topic_by_name_returns_tuple() -> None:
    conn = _FakeConn()
    conn.row_queue = [[(7, TEST_TYPE)]]
    result = message_reader._lookup_topic_by_name(conn, '/ping')
    assert result == (7, TEST_TYPE)
    # 단일 쿼리가 토픽 이름을 파라미터로 실행된다.
    assert len(conn.executed) == 1
    assert conn.executed[0][1] == ('/ping',)


def test_lookup_topic_by_name_raises_when_missing() -> None:
    conn = _FakeConn()
    conn.row_queue = [[]]
    with pytest.raises(ValueError, match='not found in topics'):
        message_reader._lookup_topic_by_name(conn, '/missing')


# --------------------------------------------------------------------------
# read_topic_messages_by_name
# --------------------------------------------------------------------------

def test_read_topic_messages_by_name_returns_rows(fake_binding) -> None:
    """이름 모드는 topics 조회 → 데이터 쿼리 2단계를 거쳐 row 리스트를 반환한다."""
    conn = _FakeConn()
    conn.row_queue = [
        [(5, TEST_TYPE)],                            # topics lookup
        [(10, 100_000_000, 'a'), (11, 0, 'b')],      # data rows
    ]
    results = message_reader.read_topic_messages_by_name(conn, 42, '/ping')
    assert results == [(10, 100_000_000, 'a'), (11, 0, 'b')]

    # 2개의 SQL 이 실행되었고 두 번째는 (episode_id, topic_id) 를 파라미터로 사용한다.
    assert len(conn.executed) == 2
    assert conn.executed[0][1] == ('/ping',)
    assert conn.executed[1][1] == (42, 5)


def test_read_topic_messages_by_name_unknown_topic_raises_immediately(
    fake_binding,
) -> None:
    """토픽이 없으면 호출 시점에 즉시 ValueError 가 발생한다."""
    conn = _FakeConn()
    conn.row_queue = [[]]   # topics lookup 결과 없음

    with pytest.raises(ValueError, match='not found in topics'):
        message_reader.read_topic_messages_by_name(conn, 1, '/missing')


# --------------------------------------------------------------------------
# read_topic_messages_by_type
# --------------------------------------------------------------------------

def test_read_topic_messages_by_type_returns_rows(fake_binding) -> None:
    """타입 모드는 topics 를 건너뛰고 곧바로 테이블을 조회한다."""
    conn = _FakeConn()
    conn.row_queue = [
        [(1, 0, 'x'), (2, 500, 'y'), (3, 0, 'z')],
    ]
    results = message_reader.read_topic_messages_by_type(conn, 99, TEST_TYPE)
    assert results == [(1, 0, 'x'), (2, 500, 'y'), (3, 0, 'z')]

    # 단 한 번의 SQL 이 episode_id 만을 파라미터로 실행된다 (topic_id 필터 없음).
    assert len(conn.executed) == 1
    assert conn.executed[0][1] == (99,)


def test_read_topic_messages_by_type_unknown_type_raises_immediately(
    fake_binding,
) -> None:
    """등록되지 않은 타입은 호출 시점에 즉시 ValueError 를 던진다."""
    conn = _FakeConn()
    with pytest.raises(ValueError, match='no binding registered'):
        message_reader.read_topic_messages_by_type(conn, 1, 'unknown/msg/Foo')
    # 에러는 검증 단계에서 발생하므로 SQL 은 실행되지 않았다.
    assert conn.executed == []


def test_read_topic_messages_by_type_no_rows_returns_empty_list(fake_binding) -> None:
    conn = _FakeConn()
    conn.row_queue = [[]]
    assert message_reader.read_topic_messages_by_type(conn, 1, TEST_TYPE) == []


# --------------------------------------------------------------------------
# SQL 쿼리 shape 검증
# --------------------------------------------------------------------------

def _rendered_sql(query: Any, conn: _FakeConn) -> str:
    """psycopg.sql.Composed / SQL 객체를 실제 DB 없이 문자열로 풀어낸다."""
    # conn 이 실제 psycopg 커넥션이 아니어도 as_string(None) 은 sql.SQL/Composed
    # 의 일부 하위 클래스에서 동작한다. 실패 시 str() fallback.
    try:
        return query.as_string(None)
    except Exception:
        return str(query)


def test_by_name_query_has_order_by_and_topic_id_filter(fake_binding) -> None:
    """이름 모드 데이터 쿼리에 `ORDER BY` + `topic_id = %s` 가 포함된다."""
    conn = _FakeConn()
    conn.row_queue = [[(5, TEST_TYPE)], []]
    message_reader.read_topic_messages_by_name(conn, 42, '/ping')

    data_query_sql = _rendered_sql(conn.executed[1][0], conn)
    assert 'ORDER BY' in data_query_sql
    assert 'topic_id' in data_query_sql
    assert 'episode_id' in data_query_sql


def test_by_type_query_has_order_by_without_topic_id_filter(fake_binding) -> None:
    """타입 모드 쿼리에 `ORDER BY` 가 있고 `topic_id` 필터는 없다."""
    conn = _FakeConn()
    conn.row_queue = [[]]
    message_reader.read_topic_messages_by_type(conn, 42, TEST_TYPE)

    query_sql = _rendered_sql(conn.executed[0][0], conn)
    assert 'ORDER BY' in query_sql
    assert 'episode_id' in query_sql
    assert 'topic_id' not in query_sql
