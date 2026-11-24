"""LeRobot v3 export 의 격자 샘플링 — `lerobot` 없이 도는 부분만 본다.

writer · loader 를 통한 끝까지의 검증은 `lerobot==0.4.4` 가 있는 파이썬(`rdfp-train` venv)에서
돌린다 — 그 환경에는 ROS 도 이 워크스페이스의 테스트 픽스처도 없다 (설계안 §6.5).
"""

from __future__ import annotations

import pytest

from rdfp.dataset.export.frame_clock import FrameClock, FrameRef, ImageStream
from rdfp.dataset.export.lerobot_v3 import export_lerobot_v3, grid_frames
from rdfp.dataset.export.writer import ExportError

NS = 1_000_000_000


def _clock(stamps_s, *, start=0.0, end=None):
    frames = tuple(FrameRef(index=i, stamp_ns=int(round(t * NS))) for i, t in enumerate(stamps_s))
    stream = ImageStream(topic='/cam', mp4_path='x.mp4', codec='h264', pixel_format='rgb8',
                         fps=10, width=4, height=4, frame_count=len(frames))
    end = end if end is not None else stamps_s[-1] + 0.1
    return FrameClock(episode_id=1, stream=stream, frames=frames,
                      episode_start_ns=int(start * NS),
                      episode_end_ns=int(end * NS))


def test_grid_picks_the_latest_frame_at_or_before_each_tick():
    # 실제 카메라가 9.6 Hz 로 흔들린다 — 10 Hz 격자에 얹으면 가끔 같은 프레임을 두 번 쓴다.
    clock = _clock([0.0, 0.104, 0.209, 0.313, 0.418])
    picks = list(grid_frames(clock, fps=10, max_skew_sec=0.2))
    assert [i for _, i, _ in picks] == [0, 0, 1, 2, 3]
    assert all(skew < 0.2 for _, _, skew in picks)
    assert picks[0][0] == 0 and picks[1][0] == NS // 10


def test_a_gap_beyond_max_skew_is_refused_not_filled():
    clock = _clock([0.0, 0.1, 0.9])
    with pytest.raises(ExportError, match='video gap'):
        list(grid_frames(clock, fps=10, max_skew_sec=0.2))


def test_non_increasing_stamps_are_refused():
    clock = _clock([0.0, 0.1, 0.1])
    with pytest.raises(ExportError, match='strictly increasing'):
        list(grid_frames(clock, fps=10, max_skew_sec=0.2))


def test_bad_arguments_fail_before_touching_the_db():
    with pytest.raises(ExportError, match='no episodes'):
        export_lerobot_v3(None, [], out_dir='/tmp/x', fps=10)
    with pytest.raises(ExportError, match='fps'):
        export_lerobot_v3(None, [1], out_dir='/tmp/x', fps=0)
