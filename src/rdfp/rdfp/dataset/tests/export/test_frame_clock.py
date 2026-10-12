#!/usr/bin/env python3

"""frame clock (E1) — 카메라 프레임 시각열과 팔 행동 chunk 결합.

**카메라 프레임 하나가 학습 샘플 하나다.** 시각의 출처는 적재된 `image_frames` 이고
그 값은 레코더 sidecar 에서 왔으므로, `frame_index` 가 곧 mp4 안의 프레임 번호다 —
시각과 영상이 같은 근거를 갖는다.

지키는 것:

1. **카메라가 여럿이면 고르라고 요구한다** — 아무거나 고르면 영상과 레이블의 위상이
   조용히 어긋난다.
2. **영상이 없으면 올린다** — frame clock 없이 샘플을 만들 수 없다.
3. **간격은 설정 fps 가 아니라 실측이다** — 펑션베이 실측이 설정 10 Hz 에서
   9.54~9.60 Hz 였고 회차마다 달랐다.
4. **레이블과 결합하면 프레임마다 chunk 하나** — 에피소드 끝을 넘으면 `is_pad`.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

psycopg = pytest.importorskip('psycopg', reason='needs psycopg')

from rdfp.dataset.export.frame_clock import (                       # noqa: E402
    FrameClockError, arm_action_chunks, available_clock_topics, load_frame_clock)
from rdfp.dataset.export.motion_label import (                      # noqa: E402
    Operation, PlanLabel, Waypoint)

NS = 1_000_000_000


@pytest.fixture
def db():
    name = 'rdfp_e1_test'
    schema = Path(__file__).resolve().parents[2] / 'sql' / 'schema.sql'
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


def _episode(conn, start=100, stop=110) -> int:
    with conn.cursor() as cur:
        cur.execute('INSERT INTO sessions (start_sec, start_nanosec, stop_sec, '
                    'stop_nanosec) VALUES (%s,0,%s,0) RETURNING id', (start, stop))
        return cur.fetchone()[0]


def _stream(conn, episode_id, topic, stamps, *, fps=10) -> None:
    """한 카메라의 image_streams 1행 + image_frames N행."""
    with conn.cursor() as cur:
        cur.execute('INSERT INTO topics (topic_name, topic_type) VALUES (%s, %s) '
                    'ON CONFLICT (topic_name) DO NOTHING', (topic, 'sensor_msgs/msg/Image'))
        cur.execute('SELECT id FROM topics WHERE topic_name = %s', (topic,))
        tid = cur.fetchone()[0]
        cur.execute(
            'INSERT INTO image_streams (episode_id, topic_id, mp4_path, codec, '
            'pixel_format, container_fps, frame_id, width, height, frame_count) '
            'VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)',
            (episode_id, tid, f'{topic.strip("/").replace("/", "_")}.mp4', 'libx264',
             'bgr8', fps, 'cam', 640, 480, len(stamps)))
        for i, t in enumerate(stamps):
            cur.execute('INSERT INTO image_frames (episode_id, topic_id, frame_index, '
                        'stamp_sec, stamp_nanosec) VALUES (%s,%s,%s,%s,%s)',
                        (episode_id, tid, i, int(t), int(round((t % 1) * 1e9))))


# ── 1. 시계 고르기 ──────────────────────────────────────────────────────────

def test_a_single_camera_needs_no_choice(db) -> None:
    ep = _episode(db)
    _stream(db, ep, '/camera/image_raw', [100.0, 100.1, 100.2])

    clock = load_frame_clock(db, ep)

    assert clock.stream.topic == '/camera/image_raw'
    assert len(clock) == 3
    assert [f.index for f in clock.frames] == [0, 1, 2]


def test_two_cameras_must_be_disambiguated(db) -> None:
    """**아무거나 고르면 영상과 레이블의 위상이 조용히 어긋난다.**"""
    ep = _episode(db)
    _stream(db, ep, '/wrist/image_raw', [100.0, 100.1])
    _stream(db, ep, '/top/image_raw', [100.05, 100.15])

    with pytest.raises(FrameClockError, match='pass topic='):
        load_frame_clock(db, ep)

    assert load_frame_clock(db, ep, topic='/top/image_raw').frames[0].t == 100.05
    assert sorted(available_clock_topics(db, ep)) == ['/top/image_raw', '/wrist/image_raw']


def test_an_unknown_topic_is_refused(db) -> None:
    ep = _episode(db)
    _stream(db, ep, '/camera/image_raw', [100.0])

    with pytest.raises(FrameClockError, match='no image stream'):
        load_frame_clock(db, ep, topic='/nope')


def test_an_episode_without_video_is_refused(db) -> None:
    """frame clock 없이는 샘플을 만들 수 없다 — 원인을 가리키는 메시지여야 한다."""
    ep = _episode(db)

    with pytest.raises(FrameClockError, match='no registered image stream'):
        load_frame_clock(db, ep)


# ── 2. 시계의 성질 ──────────────────────────────────────────────────────────

def test_frames_carry_the_episode_window(db) -> None:
    """`episode_end` 는 chunk 의 `is_pad` 판정에 그대로 쓰인다."""
    ep = _episode(db, start=100, stop=110)
    _stream(db, ep, '/camera/image_raw', [100.5, 100.6])
    clock = load_frame_clock(db, ep)

    assert clock.episode_start_ns == 100 * NS
    assert clock.episode_end == 110.0


def test_interval_stats_measure_the_real_cadence(db) -> None:
    """**설정 fps 를 믿지 않는다** — 펑션베이는 설정 10 Hz 가 실측 9.5 Hz 였다."""
    ep = _episode(db)
    _stream(db, ep, '/camera/image_raw', [100.0, 100.1, 100.3], fps=10)
    st = load_frame_clock(db, ep).interval_stats()

    assert st['count'] == 2
    assert st['min'] == pytest.approx(0.1) and st['max'] == pytest.approx(0.2)
    assert st['hz'] == pytest.approx(1 / 0.15)


def test_interval_stats_of_a_single_frame_is_empty(db) -> None:
    ep = _episode(db)
    _stream(db, ep, '/camera/image_raw', [100.0])

    assert load_frame_clock(db, ep).interval_stats() == {}


def test_the_stream_points_at_the_mp4(db) -> None:
    """영상은 mp4 의 `frame_index` 번째 프레임이다 — 경로가 있어야 꺼낸다."""
    ep = _episode(db)
    _stream(db, ep, '/camera/image_raw', [100.0, 100.1])
    s = load_frame_clock(db, ep).stream

    assert s.mp4_path.endswith('.mp4')
    assert (s.width, s.height, s.fps, s.frame_count) == (640, 480, 10, 2)


# ── 3. 레이블과 결합 (E1 × E3) ──────────────────────────────────────────────

def _label(clock):
    op = Operation(
        t0=100.0, joint_names=('j1',),
        waypoints=(Waypoint(t=0.0, positions=(0.0,)), Waypoint(t=2.0, positions=(2.0,))),
        end=102.0, operation_id='a')
    return PlanLabel([op], joint_names=('j1',), initial_positions=(9.0,),
                     episode_end=clock.episode_end)


def test_one_chunk_per_frame(db) -> None:
    ep = _episode(db, start=100, stop=110)
    _stream(db, ep, '/camera/image_raw', [100.0, 100.5, 101.0])
    clock = load_frame_clock(db, ep)

    rows = arm_action_chunks(clock, _label(clock), k=2, step=0.5, delta=0.5)

    assert len(rows) == 3
    assert [r[0] for r in rows] == [0, 1, 2], 'frame_index 가 그대로 따라온다'
    # 프레임 0 (t=100.0): τ = 100.5, 101.0 → 0.5, 1.0
    assert [v[0] for v in rows[0][2]] == pytest.approx([0.5, 1.0])


def test_is_pad_rises_past_the_episode_end(db) -> None:
    ep = _episode(db, start=100, stop=102)
    _stream(db, ep, '/camera/image_raw', [101.5])
    clock = load_frame_clock(db, ep)

    _, _, values, is_pad = arm_action_chunks(clock, _label(clock),
                                             k=3, step=1.0, delta=0.5)[0]

    assert is_pad == (False, True, True), 'τ = 102.0 / 103.0 / 104.0'
    assert values[2][0] == pytest.approx(2.0), '패딩 값은 마지막 유효 자세가 이어진다'
