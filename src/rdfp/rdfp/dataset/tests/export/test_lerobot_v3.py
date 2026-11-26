"""LeRobot v3 export 의 격자 샘플링 — `lerobot` 없이 도는 부분만 본다.

writer · loader 를 통한 끝까지의 검증은 `lerobot==0.4.4` 가 있는 파이썬(`rdfp-train` venv)에서
돌린다 — 그 환경에는 ROS 도 이 워크스페이스의 테스트 픽스처도 없다 (설계안 §6.5).
"""

from __future__ import annotations

import json
import types

import numpy as np
import pytest

from rdfp.dataset.export.frame_clock import FrameClock, FrameRef, ImageStream
from rdfp.dataset.export import lerobot_v3
from rdfp.dataset.export.lerobot_v3 import ImageGapError, export_lerobot_v3, grid_frames
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


def test_a_gap_is_an_image_gap_error_and_still_an_export_error():
    clock = _clock([0.0, 0.1, 0.6, 0.7])
    with pytest.raises(ImageGapError) as info:
        list(grid_frames(clock, fps=10, max_skew_sec=0.2))
    assert isinstance(info.value, ExportError)
    assert info.value.skew_sec > 0.2


# ----- 시차 초과 에피소드만 빼고 계속한다 --------------------------------------
# lerobot · DB 없이 export 루프만 돌린다 — 데이터 접근 함수와 writer 를 가짜로 바꾼다.

class _FakeDataset:
    created = None

    def __init__(self, *_a, **_kw):          # 재읽기 — 쓴 행 수를 그대로 돌려준다
        self._n = _FakeDataset.created.meta.total_frames

    @classmethod
    def create(cls, *_a, **_kw):
        ds = cls.__new__(cls)
        ds.meta = types.SimpleNamespace(total_frames=0, total_episodes=0)
        ds.frames_by_episode = []
        ds._pending = 0
        cls.created = ds
        return ds

    def add_frame(self, frame):
        self._pending += 1
        self.meta.total_frames += 1

    def save_episode(self):
        self.frames_by_episode.append(self._pending)
        self._pending = 0
        self.meta.total_episodes += 1

    def finalize(self):
        pass

    def __len__(self):
        return self._n

    def __getitem__(self, i):
        return {}


class _FakeDecoder:
    def __init__(self, _path):
        pass

    def get(self, _index):
        return np.zeros((4, 4, 3), dtype=np.uint8)

    def close(self):
        pass


@pytest.fixture
def fake_env(monkeypatch):
    clocks = {1: _clock([0.0, 0.1, 0.6, 0.7]),            # 0.5 s 공백 — 제외 대상
              2: _clock([0.0, 0.1, 0.2, 0.3, 0.4])}
    label = types.SimpleNamespace(at=lambda t: (0.0, 0.0))
    grip = types.SimpleNamespace(one_hot=lambda t: (1.0, 0.0, 0.0))
    m = lerobot_v3
    monkeypatch.setattr(m, '_lerobot', lambda: _FakeDataset)
    monkeypatch.setattr(m, 'load_frame_clock', lambda conn, ep, topic=None: clocks[ep])
    monkeypatch.setattr(m, 'default_arm_joints', lambda *a, **kw: ('j1', 'j2'))
    monkeypatch.setattr(m, 'episode_backend', lambda conn, ep: None)
    monkeypatch.setattr(m, 'load_joint_series', lambda *a, **kw: None)
    monkeypatch.setattr(m, '_build_label', lambda *a, **kw: (label, 0, 0, 0, ()))
    monkeypatch.setattr(m, 'load_gripper_width', lambda conn, ep: None)
    monkeypatch.setattr(m, 'load_gripper_commands', lambda conn, ep: grip)
    monkeypatch.setattr(m, 'qpos', lambda joints, widths, t: (0.0, 0.0, 0.04))
    monkeypatch.setattr(m, '_task_label', lambda conn, ep: 'task')
    monkeypatch.setattr(m, 'resolve_media', lambda path, roots: path)
    monkeypatch.setattr(m, '_ForwardDecoder', _FakeDecoder)
    monkeypatch.setattr(m.importlib.metadata, 'version', lambda name: '0.4.4')
    return clocks


def test_an_episode_over_the_skew_limit_is_excluded_and_the_rest_exported(fake_env, tmp_path):
    out = tmp_path / 'lr'
    result = export_lerobot_v3(None, [1, 2], out_dir=str(out), fps=10, max_skew_sec=0.2)
    assert result['kept'] == [2]
    assert [e['episode_id'] for e in result['excluded']] == [1]
    assert result['excluded'][0]['reason'] == 'image_gap'
    assert result['excluded'][0]['skew_sec'] > 0.2
    # 제외한 에피소드의 행은 한 줄도 들어가지 않았다.
    assert _FakeDataset.created.frames_by_episode == [result['frames']]
    manifest = json.loads((out / 'manifest.json').read_text())
    assert manifest['excluded'][0]['reason'] == 'image_gap'


def test_every_episode_over_the_limit_leaves_nothing(fake_env, tmp_path):
    with pytest.raises(ExportError, match='nothing to export'):
        export_lerobot_v3(None, [1], out_dir=str(tmp_path / 'lr'), fps=10, max_skew_sec=0.2)
    assert not (tmp_path / 'lr').exists()
