"""Read-only MCAP catalog. Original order, not header-sorted, is retained."""
from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
import hashlib
from pathlib import Path

import yaml
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

from .common import Control, fingerprint, hash_file, json_value


def discover_recordings(root: str) -> list[str]:
    path = Path(root).expanduser().resolve()
    if path.is_file():
        return [str(path)]
    if not path.is_dir():
        raise ValueError(f'경로가 없습니다: {path}')
    if (path / 'metadata.yaml').is_file() or list(path.glob('*.mcap')):
        return [str(path)]
    return sorted(str(p.parent) for p in path.rglob('metadata.yaml'))


def source_files(source: str) -> list[Path]:
    path = Path(source).expanduser().resolve()
    if path.is_file():
        files = [path]
    elif (path / 'metadata.yaml').is_file():
        meta = yaml.safe_load((path / 'metadata.yaml').read_text()) or {}
        info = meta.get('rosbag2_bagfile_information', {})
        if info.get('storage_identifier', 'mcap') != 'mcap':
            raise ValueError('현재 Dataset Manager는 MCAP rosbag2를 지원합니다.')
        names = info.get('relative_file_paths') or [f['path'] for f in info.get('files', [])]
        files = [(path / name).resolve() for name in names]
        if any(not f.is_relative_to(path) for f in files):
            raise ValueError('metadata가 bag 디렉터리 밖의 파일을 참조합니다.')
    else:
        files = sorted(path.glob('*.mcap'))
    if not files or any(f.suffix != '.mcap' or not f.is_file() for f in files):
        raise ValueError('완성된 MCAP split 파일을 찾을 수 없습니다. 압축·누락 파일을 확인하세요.')
    if len(set(files)) != len(files):
        raise ValueError('metadata에 중복 split 경로가 있습니다.')
    return files


def schema_hash(schema):
    return hashlib.sha256(schema.name.encode() + b'\0' + schema.data).hexdigest()


@dataclass
class Message:
    file_index: int
    ordinal: int
    topic: str
    type_name: str
    schema_hash: str
    log_ns: int
    stamp_ns: int | None
    value: object
    raw: bytes


def messages(source: str, topics=None, control=None):
    control = control or Control()
    factory = DecoderFactory()
    for file_index, path in enumerate(source_files(source)):
        with path.open('rb') as stream:
            reader = make_reader(stream)
            for ordinal, (schema, channel, msg) in enumerate(reader.iter_messages(log_time_order=False)):
                control.check()
                if topics is not None and channel.topic not in topics:
                    continue
                if schema is None:
                    raise ValueError(f'{channel.topic}: MCAP 메시지 정의가 없습니다.')
                decoder = factory.decoder_for(channel.message_encoding, schema)
                if decoder is None:
                    raise ValueError(f'{channel.topic}: 지원하지 않는 직렬화 형식 {channel.message_encoding}')
                value = decoder(msg.data)
                stamp = getattr(getattr(value, 'header', None), 'stamp', None)
                stamp_ns = None
                if stamp is not None:
                    ns = int(stamp.nanosec)
                    if not 0 <= ns < 1_000_000_000:
                        raise ValueError(f'{channel.topic}: 잘못된 nanosec {ns}')
                    stamp_ns = int(stamp.sec) * 1_000_000_000 + ns
                yield Message(file_index, ordinal, channel.topic, schema.name,
                              schema_hash(schema), msg.log_time, stamp_ns, value, msg.data)


def scan(source: str, control=None):
    control = control or Control()
    topics = {}
    for path in source_files(source):
        control.progress(f'토픽 목록: {path.name}')
        with path.open('rb') as stream:
            reader = make_reader(stream)
            summary = reader.get_summary()
            if summary and summary.statistics:
                entries = [(summary.schemas.get(ch.schema_id), ch,
                            summary.statistics.channel_message_counts.get(ch.id, 0))
                           for ch in summary.channels.values()]
            else:
                counts, records = defaultdict(int), {}
                for schema, channel, _ in reader.iter_messages(log_time_order=False):
                    control.check()
                    counts[channel.id] += 1
                    records[channel.id] = (schema, channel)
                entries = [(s, c, counts[k]) for k, (s, c) in records.items()]
            for schema, channel, count in entries:
                key = channel.topic
                h = schema_hash(schema) if schema else ''
                row = topics.setdefault(key, {'topic': key, 'type': schema.name if schema else '',
                                             'count': 0, 'schema_hash': h,
                                             'definition': schema.data.decode(errors='replace') if schema else '',
                                             'encoding': channel.message_encoding, 'conflict': False})
                row['count'] += count
                row['conflict'] |= row['schema_hash'] != h
    return sorted(topics.values(), key=lambda row: row['topic'])


def browse(source, topic, offset=0, limit=200, control=None):
    if offset < 0 or not 1 <= limit <= 1000:
        raise ValueError('잘못된 페이지 범위입니다.')
    rows = []
    for index, msg in enumerate(messages(source, [topic], control)):
        if index < offset:
            continue
        rows.append({'file_index': msg.file_index, 'ordinal': msg.ordinal,
                     'log_time_ns': msg.log_ns, 'header_stamp_ns': msg.stamp_ns,
                     'value': json_value(msg.value, limit=100)})
        if len(rows) >= limit:
            break
    return rows


def source_identity(source, control=None):
    control = control or Control()
    files = []
    for path in source_files(source):
        control.progress(f'원본 확인: {path.name}')
        files.append({'name': path.name, 'size': path.stat().st_size,
                      'sha256': hash_file(path, control)})
    # Paths/names are locations, not identity. A moved/copied recording stays the same.
    return fingerprint([{'size': f['size'], 'sha256': f['sha256']} for f in files]), files


def episodes(source, control=None):
    from rdfp.dataset.ingest.episode.detector import SessionEvent, detect_episodes
    events, previous = [], None
    for msg in messages(source, ['/session'], control):
        if msg.type_name != 'rdfp_msgs/msg/SessionCommand' or msg.stamp_ns is None:
            raise ValueError('/session 타입 또는 header.stamp가 잘못되었습니다.')
        if previous is not None and msg.stamp_ns < previous:
            raise ValueError('/session 시각이 역행합니다. 녹화본을 clock 구간별로 분리해 주세요.')
        previous = msg.stamp_ns
        value = msg.value
        events.append(SessionEvent(msg.stamp_ns, str(value.state),
                                   str(getattr(value, 'task_label', '') or ''),
                                   str(getattr(value, 'outcome', '') or ''),
                                   str(getattr(value, 'metadata', '') or '')))
    result = [e._asdict() for e in detect_episodes(events)]
    if events and events[-1].state == 'IN_EPISODE':
        (control or Control()).report({'message': '종료 경계 없는 마지막 에피소드는 적재에서 제외됩니다.'})
    return result
