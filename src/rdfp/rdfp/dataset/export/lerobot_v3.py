"""LeRobot v3 export — **교환용 파생물**이다. 학습 정본은 그대로 RDFP NPZ 다.

설계: `docs/collect/데이터셋_관리_GUI_설계안.md` §6.5 와 결정_기록 28 의 예외 항목.

LeRobot 로더는 프레임이 **고정 fps 격자**에 있다고 전제한다. 우리 frame clock 은 실제 카메라
stamp 라(펑션베이 9.54~9.60 Hz) 그대로 넣을 수 없어, 사용자가 고른 fps 격자 `t_i = t_0 + i/fps`
위에서 **다시 샘플링한다**:

* 영상 — `t_i` 이전의 가장 가까운 프레임(인과적). 그 시차가 `max_skew_sec` 를 넘으면 공백을
  메우지 않고 **에피소드를 거부한다**. 같은 프레임을 반복해 쓴 수와 최대 시차를 기록한다.
* 관측 — `qpos(t_i)` (팔 관절 선형 보간 + 그리퍼 폭 최근접 — NPZ 와 같은 함수).
* 행동 — `a(t_i)` = 레이블 함수를 격자 시각에서 평가한 값 + 그리퍼 one-hot. **NPZ 의 미리 계산한
  chunk 를 펴서 쓰지 않는다** — chunk 는 로더가 `delta_timestamps` 로 이어 붙인다.

그래서 NPZ 와 수치가 같지 않다. 무엇을 어떻게 바꿨는지는 `manifest.json` 에 남긴다.

**공식 `lerobot` writer 를 쓴다** — `lerobot==0.4.4` 가 `codebase_version: v3.0` 을 쓰고 다시 읽힌다는
것을 `rdfp-train` venv 에서 확인했다 (2026-11-23). ROS 파이썬에는 없으므로 **lazy import** 하고,
그 환경(`lerobot` + `psycopg`)에서 `python -m rdfp.dataset.export_cmd --format lerobot_v3` 로 돌린다.

영상은 **PyAV 로 앞으로만 디코드한다** — 격자 프레임 번호가 단조 증가라 탐색이 필요 없다
(`cv2.CAP_PROP_POS_FRAMES` 는 매번 디코더를 비워 4.8배 느리다 — CLAUDE.md).
"""

from __future__ import annotations

from typing import Any, Iterator, Optional, Sequence

import bisect
import importlib.metadata
import math

import numpy as np

from .dataset_meta import episode_backend
from .frame_clock import FrameClock, load_frame_clock
from .observation import (GRIPPER_SYMBOLS, load_gripper_commands, load_gripper_width,
                          load_joint_series, qpos)
from .writer import (DEFAULT_COMMAND_TOPIC, DEFAULT_JOINT_TOPIC, LABEL_COMMAND, LABEL_PLAN,
                     MEDIA_ROOT_DATASET, ExportError, _build_label, default_arm_joints,
                     resolve_media, staged_output, write_manifest)

NS = 1_000_000_000
#: `observation.images.<key>` 의 key. 카메라 하나를 쓴다 (NPZ 와 같은 frame clock 카메라).
CAMERA_KEY = 'observation.images.camera'


def grid_frames(clock: FrameClock, fps: int, max_skew_sec: float) -> Iterator[tuple]:
    """격자 시각마다 `(stamp_ns, frame_index, skew_sec)` — 인과적 최근 프레임.

    Raises:
        ExportError: 프레임 시각이 비었거나 증가하지 않을 때, 시차가 허용을 넘을 때.
    """
    stamps = [f.stamp_ns for f in clock.frames]
    if not stamps or any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise ExportError('camera stamps are empty or not strictly increasing')
    start = max(clock.episode_start_ns, stamps[0])
    end = min(clock.episode_end_ns, stamps[-1] + 1)
    count = math.ceil((end - start) * fps / NS)
    for i in range(count):
        stamp = start + (i * NS) // fps
        if stamp >= end:
            break
        idx = bisect.bisect_right(stamps, stamp) - 1
        skew = (stamp - stamps[idx]) / NS
        if skew > max_skew_sec:
            raise ExportError(f'video gap exceeds max_skew_sec: {skew:.3f} s at '
                              f'{(stamp - start) / NS:.3f} s into the episode')
        yield stamp, clock.frames[idx].index, skew


class _ForwardDecoder:
    """mp4 를 앞으로만 디코드하며 요청한 프레임 번호의 RGB 배열을 준다 (같은 번호 반복 허용)."""

    def __init__(self, path: str) -> None:
        import av

        self._container = av.open(path)
        self._frames = self._container.decode(video=0)
        self._index = -1
        self._current: Optional[np.ndarray] = None

    def get(self, index: int) -> np.ndarray:
        if index < self._index:
            raise ExportError(f'frame index went backwards: {index} < {self._index}')
        while self._index < index:
            try:
                frame = next(self._frames)
            except StopIteration as exc:
                raise ExportError(f'video ended before frame {index}') from exc
            self._index += 1
            if self._index == index:
                self._current = frame.to_ndarray(format='rgb24')
        assert self._current is not None
        return self._current

    def close(self) -> None:
        self._container.close()


def _lerobot():
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as exc:
        raise ExportError(
            'LeRobot v3 export needs `lerobot==0.4.4` (+ psycopg) in this Python; run it from '
            'the rdfp-train environment: python -m rdfp.dataset.export_cmd --format lerobot_v3 '
            '(see docs/collect/데이터셋_관리_GUI_설계안.md §6.5)') from exc
    return LeRobotDataset


def export_lerobot_v3(conn: Any, episode_ids: Sequence[int], *, out_dir: str, fps: int,
                      max_skew_sec: float = 0.2, label: str = LABEL_PLAN,
                      camera_topic: str = '', joint_topic: str = DEFAULT_JOINT_TOPIC,
                      command_topic: str = DEFAULT_COMMAND_TOPIC,
                      media_roots: Sequence[str] = (), repo_id: str = 'local/rdfp',
                      skip_untrustworthy: bool = True) -> dict:
    """에피소드들을 LeRobot v3 데이터셋으로 쓴다 (원자적 게시 · 재읽기 검증).

    Returns:
        요약 — `kept` · `excluded` · `frames` · 에피소드별 대응(`episodes`).
    """
    ids = list(episode_ids)
    if not ids:
        raise ExportError('no episodes to export')
    if not 1 <= int(fps) <= 120:
        raise ExportError(f'fps must be 1..120, got {fps}')
    if not (math.isfinite(max_skew_sec) and max_skew_sec > 0):
        raise ExportError('max_skew_sec must be > 0')
    LeRobotDataset = _lerobot()

    first = load_frame_clock(conn, ids[0], topic=camera_topic or None)
    camera = camera_topic or first.stream.topic
    arm = default_arm_joints(conn, ids[0], label=label, command_topic=command_topic)
    backend = episode_backend(conn, ids[0])

    kept: list = []
    excluded: list = []
    mapping: list = []
    with staged_output(out_dir) as staging:
        root = f'{staging}/dataset'   # LeRobotDataset.create 는 없는 디렉터리를 요구한다
        dataset = None
        for ep_id in ids:
            clock = load_frame_clock(conn, ep_id, topic=camera)
            if default_arm_joints(conn, ep_id, label=label, command_topic=command_topic) != arm:
                raise ExportError(f'episode {ep_id}: arm joints differ from episode {ids[0]}')
            if label == LABEL_COMMAND and episode_backend(conn, ep_id) != backend:
                raise ExportError('joint_command label: do not mix backends')
            joints = load_joint_series(conn, ep_id, joint_topic, joint_names=arm)
            label_fn, assumed, _overlap, _skipped, _notes = _build_label(
                conn, ep_id, clock, joints, label=label, arm_joints=arm,
                command_topic=command_topic)
            if assumed and skip_untrustworthy:
                excluded.append({'episode_id': ep_id, 'reason': 'assumed_full_duration',
                                 'count': assumed})
                continue
            widths = load_gripper_width(conn, ep_id)
            grip = load_gripper_commands(conn, ep_id)
            if dataset is None:
                dataset = LeRobotDataset.create(
                    repo_id, fps=int(fps), root=root, robot_type='panda', use_videos=True,
                    features={
                        'observation.state': {'dtype': 'float32', 'shape': (len(arm) + 1,),
                                              'names': [*arm, 'gripper_width_m']},
                        'action': {'dtype': 'float32', 'shape': (len(arm) + 3,),
                                   'names': [*arm, *GRIPPER_SYMBOLS]},
                        CAMERA_KEY: {'dtype': 'video',
                                     'shape': (clock.stream.height, clock.stream.width, 3),
                                     'names': ['height', 'width', 'channels']}})
            task = _task_label(conn, ep_id)
            video = resolve_media(clock.stream.mp4_path, media_roots)
            decoder = _ForwardDecoder(video)
            first_index = dataset.meta.total_frames
            count, repeats, max_skew, previous, origin = 0, 0, 0.0, None, None
            try:
                for stamp, frame_index, skew in grid_frames(clock, int(fps), max_skew_sec):
                    t = stamp / NS
                    state = np.asarray(qpos(joints, widths, t), dtype=np.float32)
                    action = np.asarray((*label_fn.at(t), *grip.one_hot(t)), dtype=np.float32)
                    if not (np.isfinite(state).all() and np.isfinite(action).all()):
                        raise ExportError(f'episode {ep_id}: non-finite state/action at '
                                          f't={t:.3f}')
                    dataset.add_frame({'observation.state': state, 'action': action,
                                       CAMERA_KEY: decoder.get(frame_index), 'task': task})
                    origin = stamp if origin is None else origin
                    repeats += int(previous == frame_index)
                    previous = frame_index
                    count += 1
                    max_skew = max(max_skew, skew)
            finally:
                decoder.close()
            if not count:
                raise ExportError(f'episode {ep_id}: no frames on the {fps} fps grid')
            dataset.save_episode()
            kept.append(ep_id)
            mapping.append({'source_episode_id': ep_id,
                            'episode_index': dataset.meta.total_episodes - 1,
                            'first_index': first_index, 'frames': count, 'origin_ns': origin,
                            'repeated_frames': repeats, 'max_image_skew_sec': max_skew})
        if dataset is None:
            raise ExportError(f'nothing to export after quality checks: {excluded}')
        dataset.finalize()
        # 다시 연다 — 쓰는 것과 읽히는 것은 별개다 (공식 loader 로 첫 · 끝 샘플을 꺼내 본다).
        loaded = LeRobotDataset(repo_id, root=root, video_backend='pyav')
        if len(loaded) != sum(m['frames'] for m in mapping):
            raise ExportError('LeRobot reload: frame count mismatch')
        for m in mapping:
            loaded[m['first_index']]
            loaded[m['first_index'] + m['frames'] - 1]
        write_manifest(staging, fmt='lerobot_v3', media_root=MEDIA_ROOT_DATASET,
                       requested=ids, kept=kept, excluded=excluded, extra={
                           'lerobot_version': importlib.metadata.version('lerobot'),
                           'dataset_dir': 'dataset', 'fps': int(fps),
                           'max_skew_sec': max_skew_sec, 'label': label,
                           'camera_topic': camera, 'arm_joints': list(arm),
                           'gripper_symbols': list(GRIPPER_SYMBOLS),
                           'action_meaning': 'a(t) on the fps grid; chunks are built by the '
                                             'loader (delta_timestamps), not precomputed',
                           'resampling': 'video: latest frame at or before t (causal); '
                                         'state: qpos(t) linear joints + nearest gripper width',
                           'episodes': mapping})
    return {'output': out_dir, 'kept': kept, 'excluded': excluded,
            'frames': sum(m['frames'] for m in mapping)}


def _task_label(conn: Any, episode_id: int) -> str:
    with conn.cursor() as cur:
        cur.execute('SELECT task_label FROM sessions WHERE id = %s', (episode_id,))
        row = cur.fetchone()
    return (row[0] if row else None) or 'unspecified'


__all__ = ['CAMERA_KEY', 'export_lerobot_v3', 'grid_frames']
