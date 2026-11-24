"""동적 매핑 — registry 에 writer 가 없는 메시지 타입을 **JSONB 로 담는** 길.

설계: `docs/collect/데이터셋_관리_GUI_설계안.md` §4.3. 정본은 두 곳이고 **코드가 이긴다** —
`db/registry.py` 의 `MESSAGE_TYPE_REGISTRY` 에 있는 타입은 `topic_tables` 에 적어도 쓰이지 않는다.
동적 매핑은 "아직 손으로 설계하지 않은 타입을 일단 담아 두는" 길이고, 자주 쓰이면 writer 를 써서
내장으로 올린다.

**열을 펼치지 않고 JSONB 한 열에 담는다** (2026-11-23 결정 — 설계안 부록 B 의 열 펼치기 안을
대신한다). 펼치는 규칙은 중첩 · 배열에서 결정할 것이 많고(§9-1) 틀리면 조용히 빈 열이 되지만,
JSONB 는 메시지를 그대로 담아 잃는 것이 없다. 값의 손실을 막는 표기는 :func:`json_value` 가
정한다 — `NaN`/`inf` 는 `{"$float": "nan"}`, 바이트열은 `{"$bytes": <base64>}` 다. 0 이나 null 로
바꾸지 않는다.

**키는 타입이다**(토픽이 아니다) — 내장 registry 와 같은 원칙이다. 같은 타입의 토픽들이 한 테이블에
들어가고 `topic_id` 로 갈린다. 등록할 때 본 `.msg` 정의의 지문을 저장해 두고, 정의가 다른 bag 은
적재가 거부한다.

ROS 를 요구하지 않는다 — `psycopg` 만 쓴다.
"""

from __future__ import annotations

from typing import Any, Optional

import base64
import hashlib
import math
import re
from dataclasses import dataclass

import psycopg
from psycopg import sql
from psycopg.types.json import Jsonb

from .registry import MESSAGE_TYPE_REGISTRY, SESSION_COMMAND_TYPE, is_image_type
from .writers.base import WriterBase, extract_stamp

#: 동적 테이블 이름 규칙. 경로 · SQL 식별자로 쓰이므로 좁게 받는다.
TABLE_NAME_RE = re.compile(r'^[a-z][a-z0-9_]{0,55}$')

#: 동적 테이블의 열. 공통 열은 내장 테이블과 같아야 에피소드 삭제(CASCADE) · `replace` 정책 ·
#: 에피소드 뷰가 그대로 먹는다.
GENERIC_COLUMNS = ('episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec', 'payload')


class DynamicMappingError(RuntimeError):
    """동적 매핑 등록 · 해제 · 삭제를 거부할 때."""


@dataclass(frozen=True)
class DynamicMapping:
    type_name: str
    table_name: str
    schema_hash: str
    active: bool


def schema_hash(type_name: str, schema_text: str) -> str:
    """`.msg` 정의의 지문. 타입 이름까지 넣어 같은 정의를 다른 이름으로 쓰는 경우도 가른다."""
    return hashlib.sha256(type_name.encode('utf-8') + b'\0'
                          + schema_text.encode('utf-8')).hexdigest()


def default_table_name(type_name: str) -> str:
    """`rdfp_msgs/msg/ClutchState` → `clutch_states` (설계안 §4.3.1 의 기본값)."""
    base = type_name.rsplit('/', 1)[-1]
    snake = re.sub(r'(?<=[a-z0-9])(?=[A-Z])', '_', base).lower()
    snake = re.sub(r'[^a-z0-9_]', '_', snake).strip('_') or 'messages'
    if not snake[0].isalpha():
        snake = f'm_{snake}'
    return (snake if snake.endswith('s') else snake + 's')[:56]


def validate_table_name(name: str) -> str:
    if not TABLE_NAME_RE.match(name):
        raise DynamicMappingError(
            f'invalid table name {name!r}: use lowercase letters, digits and _ (max 56), '
            f'starting with a letter')
    return name


def registrable(type_name: str) -> Optional[str]:
    """그 타입을 동적 매핑에 등록할 수 없는 이유. 되면 ``None``."""
    if type_name == SESSION_COMMAND_TYPE:
        return 'SessionCommand is the episode boundary, not a data table'
    if is_image_type(type_name):
        return 'images are stored as mp4 (image_streams), not rows'
    if type_name in MESSAGE_TYPE_REGISTRY:
        return f'{type_name} already has a built-in writer (registry wins)'
    return None


# ----- JSON 표기 ---------------------------------------------------------------

def json_value(value: Any) -> Any:
    """메시지 값을 **잃지 않는** JSON 값으로 바꾼다.

    `mcap_ros2` 가 만든 동적 메시지는 `__slots__` 가 필드 목록이다. 비유한 실수와 바이트열은
    태그를 붙인 객체로 남긴다 — JSON 에 그대로 표현되지 않는 값을 0 이나 null 로 바꾸면
    `getattr` 기본값 함정과 같은 종류의 조용한 손실이 된다.
    """
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else {'$float': repr(value)}
    if isinstance(value, (bytes, bytearray, memoryview)):
        return {'$bytes': base64.b64encode(bytes(value)).decode('ascii')}
    if isinstance(value, dict):
        return {str(k): json_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(v) for v in value]
    if hasattr(value, 'tolist'):
        return json_value(value.tolist())
    slots = getattr(type(value), '__slots__', None)
    if slots:
        return {name: json_value(getattr(value, name)) for name in slots
                if not name.startswith('_')}
    if hasattr(value, '__dict__'):
        return json_value(vars(value))
    return str(value)


# ----- writer -------------------------------------------------------------------

class GenericJsonbWriter(WriterBase):
    """동적 테이블 writer. 메시지 전체를 `payload` 한 열에 담는다.

    `header.stamp` 는 공통 열로 소비하지만 `payload` 에도 남는다 — 원본 그대로를 보는 것이
    이 경로의 목적이라 빼지 않는다.
    """

    columns = GENERIC_COLUMNS

    def row_values(self, episode_id: int, msg: Any) -> tuple:
        sec, nsec = extract_stamp(msg)
        return (episode_id, self.topic_id, sec, nsec, Jsonb(json_value(msg)))

    def flush(self) -> None:
        # 테이블 이름이 사용자 정의라 식별자로 인용한다 (WriterBase 는 고정 이름을 전제한다).
        if not self._buffer:
            return
        query = sql.SQL('INSERT INTO {} ({}) VALUES ({})').format(
            sql.Identifier(self.table),
            sql.SQL(', ').join(sql.Identifier(c) for c in self.columns),
            sql.SQL(', ').join(sql.Placeholder() * len(self.columns)))
        with self._conn.cursor() as cur:
            cur.executemany(query, self._buffer)
        self._inserted_since_reset += len(self._buffer)
        self._buffer.clear()


# ----- topic_tables ------------------------------------------------------------

def _has_registry_table(conn: psycopg.Connection) -> bool:
    with conn.cursor() as cur:
        cur.execute("SELECT to_regclass('topic_tables') IS NOT NULL")
        return bool(cur.fetchone()[0])


def load_mappings(conn: psycopg.Connection, *, active_only: bool = True) -> dict:
    """`topic_tables` 를 타입 → :class:`DynamicMapping` 으로. 표가 없는 옛 DB 면 빈 dict."""
    if not _has_registry_table(conn):
        return {}
    query = 'SELECT type_name, table_name, schema_hash, active FROM topic_tables'
    if active_only:
        query += ' WHERE active'
    with conn.cursor() as cur:
        cur.execute(query + ' ORDER BY type_name')
        return {r[0]: DynamicMapping(str(r[0]), str(r[1]), str(r[2]), bool(r[3]))
                for r in cur.fetchall()}


def _table_exists(conn: psycopg.Connection, table: str) -> bool:
    with conn.cursor() as cur:
        cur.execute('SELECT to_regclass(%s) IS NOT NULL',
                    (sql.Identifier(table).as_string(conn),))
        return bool(cur.fetchone()[0])


def create_table_sql(table: str) -> sql.Composed:
    """동적 테이블 DDL — 미리보기(GUI 확인 화면)와 실행이 같은 문장을 쓴다."""
    ident = sql.Identifier(table)
    return sql.SQL("""CREATE TABLE {table} (
    id              BIGSERIAL     PRIMARY KEY,
    episode_id      BIGINT        NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    topic_id        BIGINT        NOT NULL REFERENCES topics(id)   ON DELETE RESTRICT,
    stamp_sec       INTEGER       NOT NULL,
    stamp_nanosec   BIGINT        NOT NULL,
    stamp_ts        TIMESTAMPTZ   GENERATED ALWAYS AS
                        (to_timestamp(stamp_sec::double precision
                                      + stamp_nanosec::double precision / 1e9)) STORED,
    payload         JSONB         NOT NULL
);
CREATE INDEX ON {table} (episode_id);
CREATE INDEX ON {table} (topic_id);
CREATE INDEX ON {table} (stamp_ts);""").format(table=ident)


def register(conn: psycopg.Connection, type_name: str, table: str, schema_text: str) -> dict:
    """테이블을 만들고 매핑을 등록한다. **커밋하지 않는다.**

    같은 타입 · 같은 테이블 · 같은 정의로 다시 부르면 해제된 매핑을 다시 켠다(데이터는 그대로).
    """
    reason = registrable(type_name)
    if reason:
        raise DynamicMappingError(reason)
    validate_table_name(table)
    if not _has_registry_table(conn):
        raise DynamicMappingError('topic_tables not found; run `ros2 run rdfp init-db` first')
    digest = schema_hash(type_name, schema_text)
    with conn.cursor() as cur:
        cur.execute('SELECT table_name, schema_hash, active FROM topic_tables '
                    'WHERE type_name = %s', (type_name,))
        row = cur.fetchone()
        if row is not None:
            if (row[0], row[1]) != (table, digest):
                raise DynamicMappingError(
                    f'{type_name} is already mapped to {row[0]!r} with another definition; '
                    f'drop it first (changing a mapping in place is not supported)')
            cur.execute('UPDATE topic_tables SET active = TRUE WHERE type_name = %s',
                        (type_name,))
            return {'type_name': type_name, 'table_name': table, 'created': False,
                    'reactivated': not row[2]}
        cur.execute('SELECT type_name FROM topic_tables WHERE table_name = %s', (table,))
        if cur.fetchone() is not None:
            raise DynamicMappingError(f'table {table!r} is already used by another type')
    if _table_exists(conn, table):
        raise DynamicMappingError(f'table {table!r} already exists and is not a dynamic table')
    with conn.cursor() as cur:
        cur.execute(create_table_sql(table))
        cur.execute('INSERT INTO topic_tables (type_name, table_name, schema_hash, schema_text) '
                    'VALUES (%s, %s, %s, %s)', (type_name, table, digest, schema_text))
    return {'type_name': type_name, 'table_name': table, 'created': True, 'reactivated': False}


def unregister(conn: psycopg.Connection, type_name: str) -> dict:
    """등록 해제 — 다음 적재부터 그 타입을 무시한다. **테이블과 행은 남는다.** 커밋하지 않는다."""
    with conn.cursor() as cur:
        cur.execute('UPDATE topic_tables SET active = FALSE WHERE type_name = %s '
                    'RETURNING table_name', (type_name,))
        row = cur.fetchone()
    if row is None:
        raise DynamicMappingError(f'{type_name} is not a dynamic mapping')
    return {'type_name': type_name, 'table_name': row[0]}


def row_count(conn: psycopg.Connection, table: str) -> int:
    with conn.cursor() as cur:
        cur.execute(sql.SQL('SELECT count(*) FROM {}').format(sql.Identifier(table)))
        return int(cur.fetchone()[0])


def drop(conn: psycopg.Connection, type_name: str) -> dict:
    """동적 테이블을 지운다(`DROP … RESTRICT`). **이 표에 행이 있는 테이블만** — 내장 테이블은
    여기 없으므로 지울 수 없다. 커밋하지 않는다."""
    with conn.cursor() as cur:
        cur.execute('SELECT table_name FROM topic_tables WHERE type_name = %s', (type_name,))
        row = cur.fetchone()
    if row is None:
        raise DynamicMappingError(f'{type_name} is not a dynamic mapping; built-in tables '
                                  f'cannot be dropped from here')
    table = str(row[0])
    rows = row_count(conn, table) if _table_exists(conn, table) else 0
    with conn.cursor() as cur:
        cur.execute(sql.SQL('DROP TABLE IF EXISTS {} RESTRICT').format(sql.Identifier(table)))
        cur.execute('DELETE FROM ingest_units WHERE table_name = %s', (table,))
        cur.execute('DELETE FROM topic_tables WHERE type_name = %s', (type_name,))
    return {'type_name': type_name, 'table_name': table, 'deleted_rows': rows}


__all__ = [
    'DynamicMapping', 'DynamicMappingError', 'GENERIC_COLUMNS', 'GenericJsonbWriter',
    'TABLE_NAME_RE', 'create_table_sql', 'default_table_name', 'drop', 'json_value',
    'load_mappings', 'register', 'registrable', 'row_count', 'schema_hash', 'unregister',
    'validate_table_name',
]
