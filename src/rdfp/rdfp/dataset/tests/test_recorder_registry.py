#!/usr/bin/env python3

"""레코더 mp4 등록 (I4) — 출력 스캔과 `image_streams` · `image_frames` 적재.

**영상의 원본은 레코더 mp4 다** — rosbag 에는 영상을 담지 않는다. 적재는 이미
만들어져 있는 mp4 를 그 에피소드 행에 **붙이기만** 한다(좌표 공간 문서 권고 12).

지키는 것:

1. **붙이는 열쇠는 `episode_key`** — 파일 이름의 벽시계가 아니다. `use_sim_time`
   스택에서는 둘이 아예 무관하다.
2. **sidecar 줄 수 ≠ `nframe` 이면 올린다** — 조용히 넘기면 프레임 인덱스가 밀린
   채 적재되고, 학습 샘플의 영상과 관측이 어긋난다.
3. **깨진 sidecar 줄도 올린다** — mp4 N번째 프레임 ↔ sidecar N번째 줄이 이
   데이터의 전부다.
4. **`episode_key` 가 없는 옛 출력은 건너뛴다** — 억지로 붙이면 엉뚱한 에피소드에
   영상이 달린다.

DB 없이 도는 부분(스캔·검증)과 실 DB 왕복을 나눠 건다.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from rdfp.dataset.ingest.media.recorder_registry import (
    RecorderRegistrar, RecordingError, scan_recordings)

TOPIC = '/camera/image_raw'


def _write_recording(root: Path, *, prefix='session', ts='20260918-120000.000',
                     episode_sec=100, episode_nsec=0, nframe=3, frames=None,
                     topic=TOPIC, drop_key=False, make_mp4=True) -> Path:
    """레코더가 남기는 세 파일을 흉내 낸다."""
    root.mkdir(parents=True, exist_ok=True)
    mp4, side = f'{prefix}_{ts}.mp4', f'{prefix}_{ts}.jsonl'
    if make_mp4:
        (root / mp4).write_bytes(b'\x00' * 16)
    lines = frames if frames is not None else [
        {'frame_index': i, 'stamp': {'sec': episode_sec + 1 + i, 'nanosec': 0}}
        for i in range(nframe)]
    (root / side).write_text(''.join(json.dumps(x) + '\n' for x in lines), encoding='utf-8')
    meta = {
        'mp4': mp4, 'sidecar': side, 'topic': topic,
        'topic_type': 'sensor_msgs/msg/Image', 'codec': 'libx264', 'fps': 10,
        'input_format': 'rawvideo', 'resolution': '32x32', 'width': 32, 'height': 32,
        'encoding': 'bgr8', 'pixel_format': 'bgr8', 'frame_id': 'cam',
        'is_bigendian': False, 'nframe': nframe,
    }
    if not drop_key:
        meta['episode_key'] = {'sec': episode_sec, 'nanosec': episode_nsec}
    path = root / f'{prefix}_{ts}_metadata.json'
    path.write_text(json.dumps(meta), encoding='utf-8')
    return path


# ── 1. 스캔 (DB 불필요) ─────────────────────────────────────────────────────

def test_scan_indexes_by_episode_key(tmp_path) -> None:
    """**파일 이름이 아니라 `episode_key` 로 묶는다.**"""
    _write_recording(tmp_path, ts='20260918-120000.000', episode_sec=100)
    _write_recording(tmp_path, ts='20260918-130000.000', episode_sec=200)
    index = scan_recordings(tmp_path)

    assert sorted(index) == [100 * 10**9, 200 * 10**9]
    assert index[100 * 10**9][0].nframe == 3


def test_scan_groups_multiple_cameras_of_one_episode(tmp_path) -> None:
    """`file_prefix` 로 가른 카메라 여럿이 한 에피소드에 붙는다."""
    _write_recording(tmp_path, prefix='wrist', episode_sec=100, topic='/wrist/image_raw')
    _write_recording(tmp_path, prefix='overhead', episode_sec=100, topic='/top/image_raw')

    assert len(scan_recordings(tmp_path)[100 * 10**9]) == 2


def test_scan_skips_output_without_an_episode_key(tmp_path) -> None:
    """옛 형식이다 — 억지로 붙이면 엉뚱한 에피소드에 영상이 달린다."""
    _write_recording(tmp_path, drop_key=True)

    assert scan_recordings(tmp_path) == {}


def test_scan_of_a_missing_directory_is_empty(tmp_path) -> None:
    """영상 없이 수집한 데이터도 유효하다 — 오류가 아니다."""
    assert scan_recordings(tmp_path / 'nope') == {}


def test_nanosecond_part_is_part_of_the_key(tmp_path) -> None:
    _write_recording(tmp_path, episode_sec=100, episode_nsec=500_000_000)

    assert list(scan_recordings(tmp_path)) == [100 * 10**9 + 500_000_000]


# ── 2. 실 DB 등록 ───────────────────────────────────────────────────────────

psycopg = pytest.importorskip('psycopg', reason='needs psycopg')


@pytest.fixture
def db():
    """임시 DB 에 스키마를 올린다. peer 인증이 안 되면 건너뛴다."""
    import subprocess
    name = 'rdfp_i4_test'
    schema = Path(__file__).resolve().parents[1] / 'sql' / 'schema.sql'
    try:
        subprocess.run(['psql', '-d', 'postgres', '-q', '-c',
                        f'DROP DATABASE IF EXISTS {name};', '-c',
                        f'CREATE DATABASE {name};'], check=True,
                       capture_output=True, timeout=30)
        subprocess.run(['psql', '-d', name, '-q', '-v', 'ON_ERROR_STOP=1',
                        '-f', str(schema)], check=True, capture_output=True, timeout=60)
    except Exception as exc:                       # noqa: BLE001
        pytest.skip(f'no local postgres: {exc}')
    conn = psycopg.connect(f'dbname={name}')
    try:
        yield conn
    finally:
        conn.close()
        subprocess.run(['psql', '-d', 'postgres', '-q', '-c',
                        f'DROP DATABASE IF EXISTS {name};'], capture_output=True)


def _episode(conn, start_sec=100) -> int:
    with conn.cursor() as cur:
        cur.execute('INSERT INTO sessions (start_sec, start_nanosec, stop_sec, '
                    'stop_nanosec) VALUES (%s, 0, %s, 0) RETURNING id',
                    (start_sec, start_sec + 20))
        return cur.fetchone()[0]


def test_register_inserts_stream_and_frames(db, tmp_path) -> None:
    _write_recording(tmp_path, episode_sec=100, nframe=3)
    ep = _episode(db, 100)

    counts = RecorderRegistrar(db, tmp_path).register(ep, 100 * 10**9)

    assert counts == {'image_streams': 1, 'image_frames': 3}
    with db.cursor() as cur:
        cur.execute('SELECT mp4_path, codec, container_fps, width, height, frame_count '
                    'FROM image_streams WHERE episode_id=%s', (ep,))
        assert cur.fetchone() == ('session_20260918-120000.000.mp4', 'libx264', 10, 32, 32, 3)
        cur.execute('SELECT frame_index, stamp_sec FROM image_frames '
                    'WHERE episode_id=%s ORDER BY frame_index', (ep,))
        assert cur.fetchall() == [(0, 101), (1, 102), (2, 103)]


def test_register_creates_the_topic_row_itself(db, tmp_path) -> None:
    """**영상은 rosbag 에 없으므로 `topics` 행이 아직 없을 수 있다.**"""
    _write_recording(tmp_path, episode_sec=100)
    ep = _episode(db, 100)

    RecorderRegistrar(db, tmp_path).register(ep, 100 * 10**9)

    with db.cursor() as cur:
        cur.execute('SELECT topic_name, topic_type FROM topics')
        assert cur.fetchall() == [(TOPIC, 'sensor_msgs/msg/Image')]


def test_register_is_a_noop_without_a_matching_recording(db, tmp_path) -> None:
    """영상 없이 수집한 에피소드가 정상적으로 있을 수 있다 (헤드리스 등)."""
    _write_recording(tmp_path, episode_sec=999)
    ep = _episode(db, 100)

    assert RecorderRegistrar(db, tmp_path).register(ep, 100 * 10**9) == {}


def test_sidecar_line_count_must_match_nframe(db, tmp_path) -> None:
    """**핵심.** 어긋난 채 넣으면 프레임 인덱스가 밀려 영상과 관측이 안 맞는다."""
    _write_recording(tmp_path, episode_sec=100, nframe=5,
                     frames=[{'frame_index': i, 'stamp': {'sec': 101 + i, 'nanosec': 0}}
                             for i in range(3)])
    ep = _episode(db, 100)

    with pytest.raises(RecordingError, match='nframe'):
        RecorderRegistrar(db, tmp_path).register(ep, 100 * 10**9)


def test_a_malformed_sidecar_line_is_raised(db, tmp_path) -> None:
    path = _write_recording(tmp_path, episode_sec=100, nframe=2)
    (path.parent / 'session_20260918-120000.000.jsonl').write_text(
        '{"frame_index": 0, "stamp": {"sec": 1, "nanosec": 0}}\nnot json\n',
        encoding='utf-8')
    ep = _episode(db, 100)

    with pytest.raises(RecordingError, match='malformed sidecar'):
        RecorderRegistrar(db, tmp_path).register(ep, 100 * 10**9)


def test_a_missing_mp4_is_raised(db, tmp_path) -> None:
    """metadata 는 있는데 영상이 없으면 등록할 것이 없다 — 조용히 넘기면 안 된다."""
    _write_recording(tmp_path, episode_sec=100, make_mp4=False)
    ep = _episode(db, 100)

    with pytest.raises(RecordingError, match='mp4 not found'):
        RecorderRegistrar(db, tmp_path).register(ep, 100 * 10**9)


def test_two_cameras_land_as_two_streams(db, tmp_path) -> None:
    _write_recording(tmp_path, prefix='wrist', episode_sec=100, topic='/wrist/image_raw')
    _write_recording(tmp_path, prefix='top', episode_sec=100, topic='/top/image_raw')
    ep = _episode(db, 100)

    counts = RecorderRegistrar(db, tmp_path).register(ep, 100 * 10**9)

    assert counts['image_streams'] == 2 and counts['image_frames'] == 6


def test_inserted_counts_are_consumable_like_a_writer(db, tmp_path) -> None:
    """파이프라인 요약이 그대로 더할 수 있어야 한다 — 빼먹으면 0 으로 보고된다."""
    _write_recording(tmp_path, episode_sec=100)
    ep = _episode(db, 100)
    reg = RecorderRegistrar(db, tmp_path)
    reg.register(ep, 100 * 10**9)

    assert reg.consume_inserted_count() == {'image_streams': 1, 'image_frames': 3}
    assert reg.consume_inserted_count() == {}, '소비하면 0 으로 돌아간다'


def test_rollback_undoes_the_registration(db, tmp_path) -> None:
    """**같은 트랜잭션이어야 한다** — 따로 커밋하면 에피소드 없는 행이 남는다."""
    _write_recording(tmp_path, episode_sec=100)
    ep = _episode(db, 100)
    RecorderRegistrar(db, tmp_path).register(ep, 100 * 10**9)
    db.rollback()

    with db.cursor() as cur:
        cur.execute('SELECT count(*) FROM image_streams')
        assert cur.fetchone()[0] == 0
