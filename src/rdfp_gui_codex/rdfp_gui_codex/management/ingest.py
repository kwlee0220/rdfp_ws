"""Selected-topic import without deleting/recreating an existing episode."""
from __future__ import annotations

from bisect import bisect_right
from pathlib import Path

from psycopg import sql
from psycopg.rows import tuple_row
from psycopg.types.json import Jsonb

from rdfp.dataset.db.registry import MESSAGE_TYPE_REGISTRY
from rdfp.dataset.ingest.media.recorder_registry import scan_recordings, RecorderRegistrar, _iter_sidecar
from . import catalog
from .common import Control, fingerprint, json_value, hash_file
from .database import Database, mapping_revision, validate_name


def media_catalog(root):
    if not root:
        return []
    result = []
    for stamp, records in scan_recordings(root).items():
        for rec in records:
            result.append({'episode_start_ns': stamp, 'topic': rec.topic, 'frames': rec.nframe,
                           'mp4': str(rec.mp4_path), 'sidecar': str(rec.sidecar_path),
                           'metadata': str(rec.metadata_path), 'type': rec.topic_type,
                           'exists': rec.mp4_path.is_file() and rec.sidecar_path.is_file()})
    return result


def plan_import(db: Database, source, selected_topics, *, starts=None,
                recordings_dir=None, media_topics=(), replace=False, control=None):
    control = control or Control()
    if not selected_topics and not media_topics:
        raise ValueError('적재할 토픽 또는 레코더 영상을 선택해 주세요.')
    source_id, files = catalog.source_identity(source, control)
    eps = catalog.episodes(source, control)
    if starts is not None:
        eps = [e for e in eps if e['start_ns'] in starts]
    if not eps:
        raise ValueError('선택 범위에 종료 경계가 있는 에피소드가 없습니다.')
    eps.sort(key=lambda e: e['start_ns'])
    if any(a['stop_ns'] > b['start_ns'] for a, b in zip(eps, eps[1:])):
        raise ValueError('에피소드 구간이 겹칩니다.')
    selected = set(selected_topics) - {'/session'}
    topics = [r for r in catalog.scan(source, control) if r['topic'] in selected]
    if {r['topic'] for r in topics} != selected:
        raise ValueError('선택한 토픽 중 bag에서 찾을 수 없는 항목이 있습니다.')
    if selected & set(media_topics):
        raise ValueError('bag 토픽과 레코더 영상의 이중 적재는 허용하지 않습니다.')
    with db.connection() as conn:
        db.require_ready(conn)
        mappings = []
        for topic in topics:
            m = db.mapping(conn, topic)
            if topic['conflict'] or m['status'] != '연결됨':
                raise ValueError(f'{topic["topic"]}: {m["status"]}. 먼저 매핑을 등록하세요.')
            validate_name(m['table_name'])
            columns = {r['column_name'] for r in conn.execute(
                'SELECT column_name FROM information_schema.columns WHERE table_schema=%s AND table_name=%s',
                (db.config.schema_, m['table_name']))}
            required = (set(MESSAGE_TYPE_REGISTRY[topic['type']].writer_cls.columns)
                        if m['kind'] == 'builtin' else {'payload', 'raw_cdr', 'episode_id', 'topic_id'})
            if not required <= columns:
                raise ValueError(f'{m["table_name"]}: 필요한 열이 없습니다: {sorted(required-columns)}')
            mappings.append({**m, 'definition': topic['definition']})
        for ep in eps:
            row = conn.execute('SELECT s.id,d.source_id,s.stop_sec,s.stop_nanosec FROM sessions s '
                               'LEFT JOIN dm_episode_sources d ON d.episode_id=s.id '
                               'WHERE s.start_sec=%s AND s.start_nanosec=%s',
                               divmod(ep['start_ns'], 10**9)).fetchone()
            if row and (row['source_id'] != source_id or
                        row['stop_sec'] * 10**9 + row['stop_nanosec'] != ep['stop_ns']):
                raise ValueError(f'에피소드 {row["id"]}: 다른 원본 또는 출처 미확인 데이터와 시각이 충돌합니다.')
    counts, rejected = {}, {'no_header': 0, 'outside_episode': 0}
    boundaries = [e['start_ns'] for e in eps]
    for msg in catalog.messages(source, selected, control):
        if msg.stamp_ns is None:
            rejected['no_header'] += 1
            continue
        index = bisect_right(boundaries, msg.stamp_ns) - 1
        if index < 0 or msg.stamp_ns >= eps[index]['stop_ns']:
            rejected['outside_episode'] += 1
        else:
            key = f'{eps[index]["start_ns"]}:{msg.topic}'
            counts[key] = counts.get(key, 0) + 1
    if rejected['no_header']:
        raise ValueError(f'header.stamp 없는 메시지 {rejected["no_header"]}건: browse는 가능하지만 적재 시간축을 정할 수 없습니다.')
    media = []
    for row in media_catalog(recordings_dir):
        if row['topic'] not in media_topics or row['episode_start_ns'] not in boundaries:
            continue
        if not row['exists']:
            raise ValueError(f'영상 파일 누락: {row["metadata"]}')
        ep = next(e for e in eps if e['start_ns'] == row['episode_start_ns'])
        count, previous = 0, None
        for frame_index, sec, ns in _iter_sidecar(Path(row['sidecar'])):
            control.check()
            stamp = sec * 10**9 + ns
            if (frame_index != count or not 0 <= ns < 10**9
                    or not ep['start_ns'] <= stamp < ep['stop_ns']
                    or (previous is not None and stamp <= previous)):
                raise ValueError(f'{row["sidecar"]}: 프레임 번호·시각·에피소드 구간이 일치하지 않습니다.')
            count += 1
            previous = stamp
        if count != row['frames'] or not count:
            raise ValueError(f'{row["metadata"]}: sidecar 프레임 수가 일치하지 않거나 비어 있습니다.')
        hashes = {k: hash_file(Path(row[k]), control) for k in ('mp4', 'sidecar', 'metadata')}
        media.append({**row, 'hashes': hashes})
    pairs = [(r['episode_start_ns'], r['topic']) for r in media]
    if len(set(pairs)) != len(pairs):
        raise ValueError('같은 에피소드·토픽에 레코더 영상이 여러 개 있습니다. 원본 폴더를 좁혀 주세요.')
    missing = [(e['start_ns'], t) for e in eps for t in media_topics if (e['start_ns'], t) not in pairs]
    if missing:
        raise ValueError(f'선택 에피소드에 레코더 영상이 없습니다: {missing[:5]}')
    plan = {'source': str(Path(source).expanduser().resolve()), 'source_id': source_id, 'files': files,
            'episodes': eps, 'mappings': mappings, 'counts': counts, 'excluded': rejected,
            'media': media, 'recordings_dir': recordings_dir, 'replace': bool(replace)}
    plan['digest'] = fingerprint(plan)
    return plan


def _topic_id(conn, name, type_name):
    row = conn.execute('SELECT id,topic_type FROM topics WHERE topic_name=%s', (name,)).fetchone()
    if row:
        if row['topic_type'] != type_name:
            raise ValueError(f'{name}: DB의 기존 메시지 타입과 다릅니다.')
        return row['id']
    return conn.execute('INSERT INTO topics(topic_name,topic_type) VALUES(%s,%s) RETURNING id',
                        (name, type_name)).fetchone()['id']


def _episode_id(conn, ep, source_id):
    start, stop = divmod(ep['start_ns'], 10**9), divmod(ep['stop_ns'], 10**9)
    row = conn.execute('SELECT s.*,d.source_id FROM sessions s LEFT JOIN dm_episode_sources d '
                       'ON d.episode_id=s.id WHERE s.start_sec=%s AND s.start_nanosec=%s FOR UPDATE OF s', start).fetchone()
    if row:
        if row['source_id'] != source_id or (row['stop_sec'], row['stop_nanosec']) != stop:
            raise ValueError('기존 에피소드 출처·경계 충돌: 기존 데이터를 교체하지 않았습니다.')
        return row['id']
    row = conn.execute('INSERT INTO sessions(start_sec,start_nanosec,stop_sec,stop_nanosec,task_label,success,metadata) '
                       'VALUES(%s,%s,%s,%s,%s,%s,%s) RETURNING id',
                       (*start, *stop, ep['task_label'], ep['success'], Jsonb(ep['metadata']))).fetchone()
    conn.execute('INSERT INTO dm_episode_sources VALUES(%s,%s)', (row['id'], source_id))
    return row['id']


def _prepare_unit(conn, ep_id, topic_id, revision, table, replace):
    old = conn.execute('SELECT * FROM dm_imports WHERE episode_id=%s AND topic_id=%s',
                       (ep_id, topic_id)).fetchone()
    if old and old['revision'] == revision and not replace:
        counted_table = 'image_frames' if table == 'image_streams' else table
        actual = conn.execute(sql.SQL('SELECT count(*) AS n FROM {} WHERE episode_id=%s AND topic_id=%s')
                              .format(sql.Identifier(counted_table)), (ep_id, topic_id)).fetchone()['n']
        if actual != old['rows']:
            raise ValueError(f'{table}: 적재 이력과 실제 행 수가 다릅니다. 선택 토픽 교체로 복구하세요.')
        return False
    if old and not replace:
        raise ValueError('적재 버전이 다릅니다. 선택 토픽 교체를 명시적으로 선택하세요.')
    exists = conn.execute(sql.SQL('SELECT 1 FROM {} WHERE episode_id=%s AND topic_id=%s LIMIT 1')
                          .format(sql.Identifier(table)), (ep_id, topic_id)).fetchone()
    if exists and not old:
        raise ValueError(f'{table}: 관리 이력이 없는 기존 행이 있습니다. 자동 병합하지 않습니다.')
    if old and old['table_name'] != table:
        raise ValueError('저장 대상 변경은 지원하지 않습니다.')
    if replace:
        conn.execute(sql.SQL('DELETE FROM {} WHERE episode_id=%s AND topic_id=%s').format(sql.Identifier(table)),
                     (ep_id, topic_id))
        if table == 'image_streams':
            conn.execute('DELETE FROM image_frames WHERE episode_id=%s AND topic_id=%s', (ep_id, topic_id))
    return True


def _complete_unit(conn, ep_id, topic_id, revision, table, rows):
    conn.execute('INSERT INTO dm_imports(episode_id,topic_id,revision,table_name,rows) VALUES(%s,%s,%s,%s,%s) '
                 'ON CONFLICT(episode_id,topic_id) DO UPDATE SET revision=excluded.revision, '
                 'table_name=excluded.table_name, rows=excluded.rows, completed_at=now()',
                 (ep_id, topic_id, revision, table, rows))


def execute_import(db: Database, plan, control=None):
    control = control or Control()
    if plan.get('digest') != fingerprint({k: v for k, v in plan.items() if k != 'digest'}):
        raise ValueError('적재 계획이 변경되었습니다. 다시 미리보기를 실행하세요.')
    source_id, _ = catalog.source_identity(plan['source'], control)
    if source_id != plan['source_id']:
        raise ValueError('원본 파일이 변경되었습니다. 계획을 다시 생성하세요.')
    before = {p: (p.stat().st_size, p.stat().st_mtime_ns) for p in catalog.source_files(plan['source'])}
    for media in plan['media']:
        for key, expected in media['hashes'].items():
            if hash_file(Path(media[key]), control) != expected:
                raise ValueError(f'레코더 파일이 변경되었습니다: {media[key]}')
    summary = {'episodes': 0, 'rows': 0, 'skipped_units': 0}
    for index, ep in enumerate(plan['episodes']):
        control.progress(f'에피소드 {index+1}/{len(plan["episodes"])} 적재', **summary)
        with db.connection(write=True) as conn:
            db.require_ready(conn)
            conn.execute('INSERT INTO dm_sources VALUES(%s,%s,%s) ON CONFLICT(source_id) DO NOTHING',
                         (source_id, plan['source'], Jsonb(plan['files'])))
            ep_id = _episode_id(conn, ep, source_id)
            active, counts, skipped = {}, {}, 0
            for m in plan['mappings']:
                current = db.mapping(conn, {'topic': m['topic'], 'type': m['type_name'], 'schema_hash': m['schema_hash']})
                if current['status'] != '연결됨' or mapping_revision(current) != mapping_revision(m):
                    raise ValueError('매핑이 변경되었습니다. 적재 계획을 다시 생성하세요.')
                tid = _topic_id(conn, m['topic'], m['type_name'])
                rev = mapping_revision(m)
                db.store_mapping(conn, {'topic': m['topic'], 'type': m['type_name'],
                                       'schema_hash': m['schema_hash'], 'definition': m.get('definition', '')},
                                 m['table_name'], m['kind'])
                if not _prepare_unit(conn, ep_id, tid, rev, m['table_name'], plan['replace']):
                    skipped += 1
                    continue
                writer = (MESSAGE_TYPE_REGISTRY[m['type_name']].writer_cls(conn, topic_id=tid)
                          if m['kind'] == 'builtin' else None)
                active[m['topic']] = (m, tid, rev, writer)
                counts[m['topic']] = 0
            for msg in catalog.messages(plan['source'], set(active), control):
                if msg.stamp_ns is None:
                    raise ValueError(f'{msg.topic}: header.stamp 없음')
                if not ep['start_ns'] <= msg.stamp_ns < ep['stop_ns']:
                    continue
                m, tid, rev, writer = active[msg.topic]
                if msg.schema_hash != m['schema_hash']:
                    raise ValueError(f'{msg.topic}: 메시지 스키마 변경')
                if writer:
                    writer.append(ep_id, msg.value)
                else:
                    conn.execute(sql.SQL('INSERT INTO {}(episode_id,topic_id,stamp_sec,stamp_nanosec,log_time_ns,'
                                         'source_file,source_ordinal,payload,raw_cdr) VALUES(%s,%s,%s,%s,%s,%s,%s,%s,%s)')
                                 .format(sql.Identifier(m['table_name'])),
                                 (ep_id, tid, *divmod(msg.stamp_ns, 10**9), msg.log_ns,
                                  msg.file_index, msg.ordinal, Jsonb(json_value(msg.value)), msg.raw))
                counts[msg.topic] += 1
            for topic, (m, tid, rev, writer) in active.items():
                if writer:
                    writer.flush()
                expected = plan['counts'].get(f'{ep["start_ns"]}:{topic}', 0)
                if counts[topic] != expected:
                    raise ValueError('원본 메시지 수가 적재 계획과 다릅니다.')
                _complete_unit(conn, ep_id, tid, rev, m['table_name'], counts[topic])
            # Register only explicitly selected recordings; never re-encode or delete originals.
            media_count = 0
            selected_media = [r for r in plan['media'] if r['episode_start_ns'] == ep['start_ns']]
            if selected_media:
                available = scan_recordings(plan['recordings_dir'])
                for row in selected_media:
                    tid = _topic_id(conn, row['topic'], row['type'])
                    rev = fingerprint(row['hashes'])
                    if not _prepare_unit(conn, ep_id, tid, rev, 'image_streams', plan['replace']):
                        skipped += 1
                        continue
                    records = [r for r in available.get(ep['start_ns'], [])
                               if str(r.metadata_path) == row['metadata']]
                    if len(records) != 1:
                        raise ValueError('레코더 기록이 변경되었습니다.')
                    # Legacy image writers fetch tuple rows. Restore factory immediately.
                    factory = conn.row_factory
                    try:
                        conn.row_factory = tuple_row
                        RecorderRegistrar(conn, plan['recordings_dir'],
                                          index={ep['start_ns']: records}).register(ep_id, ep['start_ns'])
                    finally:
                        conn.row_factory = factory
                    _complete_unit(conn, ep_id, tid, rev, 'image_streams', row['frames'])
                    media_count += row['frames']
            control.check()  # Cancellation wins before this episode's commit.
            if any((p.stat().st_size, p.stat().st_mtime_ns) != stamp for p, stamp in before.items()):
                raise ValueError('적재 중 원본 파일이 변경되었습니다. 현재 에피소드를 rollback합니다.')
        summary['episodes'] += 1
        summary['rows'] += sum(counts.values()) + media_count
        summary['skipped_units'] += skipped
        control.report({'message': f'에피소드 {ep_id} commit 완료', **summary})
    return summary
