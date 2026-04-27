"""비디오 파이프라인 end-to-end 통합 테스트.

합성 rosbag 에 이미지 토픽을 포함시키고, FrameRouter 가 실제 mp4 를
ffmpeg 로 생성하는지, sidecar 정보 (image_streams / image_frames) 가
DBMS INSERT 호출로 전달되는지 확인한다. 실 PostgreSQL 의존을 피하기 위해
``executemany`` / ``execute`` 호출만 기록하는 fake conn 을 사용한다.
시스템 ffmpeg 가 없으면 자동으로 skip.
"""

from __future__ import annotations

import shutil
import subprocess

import pytest

from rdfp.rosbag.mcap_reader import iter_split_messages
from rdfp.rosbag.tests.fixtures.make_synth_bag import (
    ImageEventSpec,
    SessionEventSpec,
    write_synth_bag,
)
from rdfp.dataset.ingest.media.frame_router import FrameRouter


pytestmark = pytest.mark.skipif(
    shutil.which('ffmpeg') is None or shutil.which('ffprobe') is None,
    reason='ffmpeg/ffprobe not available',
)


class _FakeConn:
    """ImageStreamWriter 의 ``execute(... RETURNING id)`` 와
    WriterBase 의 ``executemany`` 모두를 기록하는 더미 커넥션.
    """

    def __init__(self) -> None:
        self.executed: list[tuple[str, object]] = []
        self.executemany_calls: list[tuple[str, list]] = []
        self._next_id: int = 1

    def cursor(self):
        this = self

        class _Cur:
            def __init__(self) -> None:
                self._last_id: int | None = None

            def __enter__(self):
                return self

            def __exit__(self, *a):
                return False

            def execute(self, sql, params=None):
                this.executed.append((sql, params))
                if 'RETURNING id' in sql:
                    self._last_id = this._next_id
                    this._next_id += 1
                else:
                    self._last_id = None

            def executemany(self, sql, params):
                this.executemany_calls.append((sql, list(params)))

            def fetchone(self):
                return (self._last_id,) if self._last_id is not None else None

        return _Cur()


def _ffprobe_frame_count(path) -> tuple[int, int, int]:
    """ffprobe 로 mp4 의 (frames, width, height) 를 읽어온다."""
    r = subprocess.run(
        ['ffprobe', '-v', 'error', '-select_streams', 'v:0',
         '-count_frames', '-show_entries', 'stream=nb_read_frames,width,height',
         '-of', 'default=noprint_wrappers=1:nokey=1', str(path)],
        capture_output=True, text=True, check=True,
    )
    lines = [ln for ln in r.stdout.splitlines() if ln.strip()]
    width, height, frames = map(int, lines)
    return frames, width, height


def test_image_topic_to_mp4_and_dbms(tmp_path) -> None:
    bag_root = tmp_path / 'bag'
    mp4_root = tmp_path / 'out'
    S = 1_712_345_678_000_000_000

    # 에피소드 [S+2s, S+5s) 에 Image 토픽 3 프레임.
    session_dir = write_synth_bag(
        bag_root,
        session_events=[
            SessionEventSpec(S,                     'IDLE',       ''),
            SessionEventSpec(S + 1_000_000_000,     'IN_SESSION', 'vid'),
            SessionEventSpec(S + 2_000_000_000,     'IN_EPISODE', 'vid'),
            SessionEventSpec(S + 5_000_000_000,     'IN_SESSION', 'vid'),
            SessionEventSpec(S + 5_001_000_000,     'IDLE',       'vid'),
        ],
        image_events=[
            ImageEventSpec(S + 2_500_000_000, '/cam/raw', frame_id='cam_raw_link',
                           width=32, height=24, encoding='bgr8', fill=(10, 20, 30)),
            ImageEventSpec(S + 3_000_000_000, '/cam/raw', frame_id='cam_raw_link',
                           width=32, height=24, encoding='bgr8', fill=(40, 50, 60)),
            ImageEventSpec(S + 4_000_000_000, '/cam/raw', frame_id='cam_raw_link',
                           width=32, height=24, encoding='bgr8', fill=(70, 80, 90)),
        ],
    )
    mcap = next(session_dir.glob('*.mcap'))

    conn = _FakeConn()
    router = FrameRouter(
        mp4_root, conn=conn, topic_ids={'/cam/raw': 7}, fps=30,
    )

    ep_start = S + 2_000_000_000
    ep_stop = S + 5_000_000_000
    router.open_episode(episode_id=1)
    for m in iter_split_messages(mcap):
        if m.topic != '/cam/raw':
            continue
        if not (ep_start <= m.stamp_ns < ep_stop):
            continue
        router.on_image(m.topic, m.type_name, m.msg)
    produced = router.finalize_episode()

    # mp4 파일 검증.
    assert len(produced) == 1, produced
    ep_dir = mp4_root / 'episode_00000001'
    raw_mp4 = ep_dir / 'cam_raw.mp4'
    assert raw_mp4.is_file()
    raw_frames, w, h = _ffprobe_frame_count(raw_mp4)
    assert raw_frames == 3
    assert (w, h) == (32, 24)

    # image_frames batch INSERT (executemany) 가 한 번 이상 호출되고 3 행을 담아야 한다.
    image_frame_calls = [
        (sql, params) for sql, params in conn.executemany_calls
        if 'image_frames' in sql
    ]
    assert image_frame_calls, conn.executemany_calls
    rows = [row for _, params in image_frame_calls for row in params]
    assert len(rows) == 3
    # (episode_id, topic_id, frame_index, stamp_sec, stamp_nanosec)
    assert [r[0] for r in rows] == [1, 1, 1]
    assert [r[1] for r in rows] == [7, 7, 7]
    assert [r[2] for r in rows] == [0, 1, 2]

    # image_streams INSERT (execute + RETURNING id) 가 한 번 호출되고 메타가 옳아야 한다.
    stream_calls = [(sql, params) for sql, params in conn.executed
                    if 'image_streams' in sql and params is not None]
    assert len(stream_calls) == 1
    _, sparams = stream_calls[0]
    # 컬럼 순서: episode_id, topic_id, mp4_path, codec, pixel_format,
    #           container_fps, frame_id, width, height, frame_count
    assert sparams[0] == 1            # episode_id
    assert sparams[1] == 7            # topic_id
    assert sparams[2] == 'episode_00000001/cam_raw.mp4'  # 상대 경로
    assert sparams[4] == 'bgr8'       # pixel_format (첫 프레임의 encoding)
    assert sparams[5] == 30           # container_fps
    assert sparams[6] == 'cam_raw_link'
    assert sparams[7] == 32           # width
    assert sparams[8] == 24           # height
    assert sparams[9] == 3            # frame_count


def test_empty_camera_episode_produces_no_mp4(tmp_path) -> None:
    router = FrameRouter(tmp_path, conn=_FakeConn(), topic_ids={}, fps=30)
    router.open_episode(episode_id=9)
    # on_image 호출 없음.
    produced = router.finalize_episode()
    assert produced == []
    ep_dir = tmp_path / 'episode_00000009'
    assert ep_dir.is_dir()
    # mp4 는 생성되지 않는다.
    assert list(ep_dir.iterdir()) == []


def test_abort_episode_removes_partial_files(tmp_path) -> None:
    router = FrameRouter(
        tmp_path, conn=_FakeConn(), topic_ids={'/cam/raw': 1}, fps=30,
    )
    router.open_episode(episode_id=11)
    # 1 프레임 기록 후 abort.
    from types import SimpleNamespace
    import numpy as np
    frame = np.zeros((8, 8, 3), dtype=np.uint8)
    msg = SimpleNamespace(
        height=8, width=8, encoding='bgr8',
        is_bigendian=0, step=24, data=frame.tobytes(),
        header=SimpleNamespace(frame_id='', stamp=SimpleNamespace(sec=0, nanosec=0)),
    )
    router.on_image('/cam/raw', 'sensor_msgs/msg/Image', msg)
    router.abort_episode()
    # 에피소드 디렉터리 전체가 제거된다.
    assert not (tmp_path / 'episode_00000011').exists()
