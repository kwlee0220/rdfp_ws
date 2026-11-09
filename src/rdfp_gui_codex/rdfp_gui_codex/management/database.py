"""PostgreSQL catalog and explicitly initialized manager metadata.

All manager writes use the same advisory lock. Existing dataset CLI writes do
not participate: run those separately, as documented in the operator guide.
"""
from __future__ import annotations

from contextlib import contextmanager
import re

import psycopg
from psycopg import sql
from psycopg.rows import dict_row

from rdfp.dataset.db.config import DbConfig, resolve_dsn
from rdfp.dataset.db.registry import MESSAGE_TYPE_REGISTRY
from .common import fingerprint


DDL = """
CREATE TABLE IF NOT EXISTS dm_sources (
    source_id TEXT PRIMARY KEY, location TEXT NOT NULL, files JSONB NOT NULL);
CREATE TABLE IF NOT EXISTS dm_episode_sources (
    episode_id BIGINT PRIMARY KEY REFERENCES sessions(id) ON DELETE CASCADE,
    source_id TEXT NOT NULL REFERENCES dm_sources(source_id));
CREATE TABLE IF NOT EXISTS dm_mappings (
    topic TEXT PRIMARY KEY, type_name TEXT NOT NULL, schema_hash TEXT NOT NULL,
    table_name TEXT NOT NULL, kind TEXT NOT NULL CHECK (kind IN ('builtin','jsonb')),
    active BOOLEAN NOT NULL DEFAULT TRUE, definition TEXT NOT NULL DEFAULT '');
CREATE TABLE IF NOT EXISTS dm_owned_tables (
    table_name TEXT PRIMARY KEY, relation_oid OID NOT NULL);
CREATE TABLE IF NOT EXISTS dm_imports (
    episode_id BIGINT NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    topic_id BIGINT NOT NULL REFERENCES topics(id), revision TEXT NOT NULL,
    table_name TEXT NOT NULL, rows BIGINT NOT NULL, completed_at TIMESTAMPTZ NOT NULL DEFAULT now(),
    PRIMARY KEY (episode_id, topic_id));
"""


def validate_name(name):
    if not re.fullmatch(r'[a-z][a-z0-9_]{0,55}', name):
        raise ValueError('테이블명은 영문 소문자로 시작하는 56자 이하 영문·숫자·밑줄로 지정하세요.')
    return name


class Database:
    def __init__(self, config: DbConfig):
        self.config = config

    @contextmanager
    def connection(self, *, write=False, snapshot=False):
        with psycopg.connect(resolve_dsn(self.config), row_factory=dict_row,
                             connect_timeout=5) as conn:
            conn.execute(sql.SQL('SET search_path TO {}').format(sql.Identifier(self.config.schema_)))
            conn.execute("SET statement_timeout TO '30s'")
            conn.execute("SET lock_timeout TO '5s'")
            conn.commit()
            if snapshot:
                conn.execute('SET TRANSACTION ISOLATION LEVEL REPEATABLE READ')
            if write:
                conn.execute('SELECT pg_advisory_xact_lock(hashtext(%s))',
                             ('rdfp_dataset_manager:' + self.config.schema_,))
            else:
                conn.execute('SET TRANSACTION READ ONLY')
            yield conn

    @staticmethod
    def ready(conn):
        return conn.execute("SELECT to_regclass('dm_mappings') IS NOT NULL AS ok").fetchone()['ok']

    @staticmethod
    def require_ready(conn):
        if not Database.ready(conn):
            raise ValueError('먼저 ‘관리 DB 준비’를 실행해 주세요. 기존 데이터는 보존됩니다.')

    def initialize(self):
        from rdfp.dataset.db.initialize import read_schema_sql
        with self.connection(write=True) as conn:
            conn.execute(sql.SQL('CREATE SCHEMA IF NOT EXISTS {}').format(sql.Identifier(self.config.schema_)))
            conn.execute(sql.SQL('SET search_path TO {}').format(sql.Identifier(self.config.schema_)))
            conn.execute(read_schema_sql())
            conn.execute(DDL)
        return {'message': '기존 dataset 스키마와 관리 테이블을 준비했습니다.'}

    def tables(self):
        with self.connection() as conn:
            return conn.execute("""SELECT c.relname AS table_name,
                    greatest(c.reltuples::bigint, 0) AS estimated_rows,
                    pg_total_relation_size(c.oid) AS bytes,
                    left(c.relname,3)='dm_' AS management
                FROM pg_class c JOIN pg_namespace n ON n.oid=c.relnamespace
                WHERE n.nspname=%s AND c.relkind IN ('r','p') ORDER BY c.relname""",
                                (self.config.schema_,)).fetchall()

    def describe(self, table):
        with self.connection() as conn:
            columns = conn.execute("""SELECT column_name, data_type, udt_name, is_nullable,
                    column_default FROM information_schema.columns
                    WHERE table_schema=%s AND table_name=%s ORDER BY ordinal_position""",
                                   (self.config.schema_, table)).fetchall()
            constraints = conn.execute("""SELECT conname, pg_get_constraintdef(c.oid) AS definition
                FROM pg_constraint c JOIN pg_class t ON t.oid=c.conrelid
                JOIN pg_namespace n ON n.oid=t.relnamespace
                WHERE n.nspname=%s AND t.relname=%s""", (self.config.schema_, table)).fetchall()
            indexes = conn.execute('SELECT indexname, indexdef FROM pg_indexes WHERE schemaname=%s AND tablename=%s',
                                   (self.config.schema_, table)).fetchall()
            return {'columns': columns, 'constraints': constraints, 'indexes': indexes}

    def browse(self, table, *, offset=0, limit=200, episode_id=None, topic_id=None, topic_name=None):
        if offset < 0 or not 1 <= limit <= 1000:
            raise ValueError('잘못된 페이지 범위입니다.')
        with self.connection() as conn:
            column_rows = conn.execute(
                'SELECT column_name,udt_name FROM information_schema.columns '
                'WHERE table_schema=%s AND table_name=%s ORDER BY ordinal_position',
                (self.config.schema_, table)).fetchall()
            columns = {r['column_name'] for r in column_rows}
            if not columns:
                raise ValueError('테이블을 찾을 수 없습니다.')
            clauses, args = [], []
            for key, value in [('episode_id', episode_id), ('topic_id', topic_id)]:
                if value is not None:
                    if key not in columns:
                        raise ValueError(f'{table}에는 {key} 열이 없습니다.')
                    clauses.append(sql.SQL('{}=%s').format(sql.Identifier(key)))
                    args.append(int(value))
            if topic_name:
                if 'topic_id' not in columns:
                    raise ValueError(f'{table}에는 topic_id 열이 없습니다.')
                clauses.append(sql.SQL('topic_id=(SELECT id FROM topics WHERE topic_name=%s)'))
                args.append(topic_name)
            where = sql.SQL(' WHERE ') + sql.SQL(' AND ').join(clauses) if clauses else sql.SQL('')
            order = sql.Identifier('id') if 'id' in columns else sql.SQL('ctid')
            projections = []
            for col in column_rows:
                name = sql.Identifier(col['column_name'])
                if col['udt_name'] == 'bytea':
                    projections.append(sql.SQL('substring({} from 1 for 256) AS {}').format(name,name))
                    projections.append(sql.SQL('octet_length({}) AS {}').format(
                        name,sql.Identifier(col['column_name']+'_total_bytes')))
                else:
                    projections.append(name)
            query = sql.SQL('SELECT {} FROM {}{} ORDER BY {} LIMIT %s OFFSET %s').format(
                sql.SQL(',').join(projections), sql.Identifier(self.config.schema_, table), where, order)
            return conn.execute(query, (*args, limit, offset)).fetchall()

    def episodes(self, limit=200, offset=0):
        with self.connection() as conn:
            if limit < 1 or limit > 1000 or offset < 0:
                raise ValueError('잘못된 페이지 범위입니다.')
            return conn.execute('SELECT * FROM sessions ORDER BY id LIMIT %s OFFSET %s',
                                (limit, offset)).fetchall()

    def episode_detail(self, episode_id):
        with self.connection() as conn:
            ep = conn.execute('SELECT * FROM sessions WHERE id=%s', (episode_id,)).fetchone()
            if ep is None:
                raise ValueError('에피소드가 없습니다.')
            tables = conn.execute("""SELECT table_name FROM information_schema.columns
                WHERE table_schema=%s AND column_name='episode_id' AND left(table_name,3)<>'dm_'""",
                                  (self.config.schema_,)).fetchall()
            counts = {}
            for row in tables:
                table = row['table_name']
                counts[table] = conn.execute(sql.SQL('SELECT count(*) AS n FROM {} WHERE episode_id=%s')
                                             .format(sql.Identifier(table)), (episode_id,)).fetchone()['n']
            return {'episode': ep, 'counts': counts,
                    'media': conn.execute('SELECT i.*,t.topic_name FROM image_streams i '
                                          'JOIN topics t ON t.id=i.topic_id WHERE episode_id=%s',
                                          (episode_id,)).fetchall()}

    @staticmethod
    def mapping(conn, topic):
        row = None
        if Database.ready(conn):
            row = conn.execute('SELECT * FROM dm_mappings WHERE topic=%s', (topic['topic'],)).fetchone()
        if row:
            if row['type_name'] != topic['type'] or row['schema_hash'] != topic['schema_hash']:
                return {**row, 'status': '스키마 충돌'}
            return {**row, 'status': '연결됨' if row['active'] else '비활성'}
        if topic['topic'] == '/session':
            return {'table_name': 'sessions', 'status': '에피소드 경계', 'kind': 'session'}
        binding = MESSAGE_TYPE_REGISTRY.get(topic['type'])
        if binding:
            return {'topic': topic['topic'], 'type_name': topic['type'],
                    'schema_hash': topic['schema_hash'], 'table_name': binding.table,
                    'kind': 'builtin', 'active': True, 'status': '연결됨'}
        return {'table_name': '', 'status': '미매핑', 'kind': ''}

    def mappings(self, topics):
        with self.connection() as conn:
            result = []
            for topic in topics:
                m = self.mapping(conn, topic)
                if topic.get('conflict'):
                    m['status'] = '스키마 충돌'
                if m['table_name']:
                    exists = conn.execute('SELECT to_regclass(%s) AS t',
                                          (sql.Identifier(self.config.schema_, m['table_name']).as_string(conn),)
                                          ).fetchone()['t']
                    if not exists:
                        m['status'] = '테이블 없음'
                result.append({**topic, 'mapping': m, 'table_name': m['table_name'], 'status': m['status']})
            return result

    @staticmethod
    def store_mapping(conn, topic, table, kind, active=True):
        conn.execute("""INSERT INTO dm_mappings(topic,type_name,schema_hash,table_name,kind,active,definition)
            VALUES(%s,%s,%s,%s,%s,%s,%s) ON CONFLICT(topic) DO UPDATE SET
            type_name=excluded.type_name,schema_hash=excluded.schema_hash,table_name=excluded.table_name,
            kind=excluded.kind,active=excluded.active,definition=excluded.definition""",
                     (topic['topic'], topic['type'], topic['schema_hash'], table, kind, active,
                      topic.get('definition', '')))

    def register(self, topic, table, *, create=True):
        validate_name(table)
        if not table.startswith('custom_'):
            raise ValueError('범용 테이블명은 custom_으로 시작해야 합니다.')
        if topic['topic'] == '/session' or topic.get('conflict') or not topic.get('schema_hash'):
            raise ValueError('이 토픽은 범용 매핑에 등록할 수 없습니다.')
        with self.connection(write=True) as conn:
            self.require_ready(conn)
            old = conn.execute('SELECT * FROM dm_mappings WHERE topic=%s', (topic['topic'],)).fetchone()
            if old and (old['table_name'], old['schema_hash']) != (table, topic['schema_hash']):
                raise ValueError('기존 매핑의 구조 변경은 지원하지 않습니다. 별도 토픽·DB에서 등록하세요.')
            peers = conn.execute('SELECT schema_hash FROM dm_mappings WHERE table_name=%s', (table,)).fetchall()
            if any(p['schema_hash'] != topic['schema_hash'] for p in peers):
                raise ValueError('다른 메시지 스키마가 사용하는 테이블입니다.')
            if create:
                conn.execute(sql.SQL("""CREATE TABLE {} (
                    id BIGSERIAL PRIMARY KEY,
                    episode_id BIGINT NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
                    topic_id BIGINT NOT NULL REFERENCES topics(id),
                    stamp_sec BIGINT NOT NULL, stamp_nanosec BIGINT NOT NULL,
                    log_time_ns BIGINT NOT NULL, source_file INTEGER NOT NULL, source_ordinal BIGINT NOT NULL,
                    payload JSONB NOT NULL, raw_cdr BYTEA NOT NULL)""").format(sql.Identifier(table)))
                conn.execute(sql.SQL('CREATE INDEX ON {} (episode_id,topic_id,stamp_sec,stamp_nanosec,id)')
                             .format(sql.Identifier(table)))
                conn.execute('INSERT INTO dm_owned_tables VALUES(%s,to_regclass(%s)::oid)', (table, table))
            else:
                # Only verified manager-created generic tables can be shared. Arbitrary
                # external schemas require a developer-supplied adapter, not guessed casts.
                owned = conn.execute('SELECT 1 FROM dm_owned_tables WHERE table_name=%s '
                                     'AND relation_oid=to_regclass(%s)::oid', (table, table)).fetchone()
                if not owned:
                    raise ValueError('등록된 Dataset Manager 범용 테이블만 재사용할 수 있습니다.')
            self.store_mapping(conn, topic, table, 'jsonb')
        return {'message': f'{topic["topic"]} → {table} 등록 완료'}

    def set_mapping_active(self, topic, active):
        with self.connection(write=True) as conn:
            self.require_ready(conn)
            mapping = self.mapping(conn, topic)
            if mapping['kind'] not in ('builtin', 'jsonb') or mapping['status'] == '스키마 충돌':
                raise ValueError('해제·활성화 가능한 매핑이 아닙니다.')
            self.store_mapping(conn, topic, mapping['table_name'], mapping['kind'], active)
        return {'message': '매핑을 활성화했습니다.' if active else '매핑을 해제했습니다. 데이터는 보존됩니다.'}

    def drop_custom(self, topic, *, confirmed_table):
        with self.connection(write=True) as conn:
            self.require_ready(conn)
            m = self.mapping(conn, topic)
            table = m['table_name']
            owned = conn.execute('SELECT 1 FROM dm_owned_tables WHERE table_name=%s '
                                 'AND relation_oid=to_regclass(%s)::oid', (table, table)).fetchone()
            if not owned or m['kind'] != 'jsonb' or confirmed_table != table:
                raise ValueError('확인한 사용자 전용 테이블만 삭제할 수 있습니다.')
            others = conn.execute('SELECT topic FROM dm_mappings WHERE table_name=%s AND topic<>%s',
                                  (table, topic['topic'])).fetchall()
            if others:
                raise ValueError('다른 토픽이 공유하는 테이블은 삭제할 수 없습니다.')
            count = conn.execute(sql.SQL('SELECT count(*) AS n FROM {}').format(sql.Identifier(table))).fetchone()['n']
            conn.execute(sql.SQL('DROP TABLE {} RESTRICT').format(sql.Identifier(table)))
            conn.execute('DELETE FROM dm_owned_tables WHERE table_name=%s', (table,))
            conn.execute('DELETE FROM dm_imports WHERE table_name=%s', (table,))
            conn.execute('DELETE FROM dm_mappings WHERE table_name=%s', (table,))
        return {'message': f'{table} 삭제 완료', 'deleted_rows': count}


def mapping_revision(mapping):
    return fingerprint({k: mapping[k] for k in ('topic', 'type_name', 'schema_hash', 'table_name', 'kind')})
