"""Atomic NPZ packaging and an optional official LeRobot v3 writer adapter."""
from __future__ import annotations

from bisect import bisect_right
from dataclasses import replace
import importlib.metadata
import json
import math
from pathlib import Path
import shutil
import tempfile

import numpy as np
from psycopg.rows import tuple_row

from rdfp.dataset.export.dataset_meta import ExportSpec, RunningStats, build_metadata, episode_backend
from rdfp.dataset.export.frame_clock import load_frame_clock
from rdfp.dataset.export.observation import GRIPPER_SYMBOLS, load_joint_series, load_gripper_width, load_gripper_commands, qpos
from rdfp.dataset.export.writer import _build_label, default_arm_joints, export_episode
from .common import Control, hash_file


def media_path(path, roots):
    value = Path(path)
    candidates = [value] if value.is_absolute() else [Path(r) / value for r in roots if r]
    found = [p.resolve() for p in candidates if p.is_file()]
    if not found:
        raise ValueError(f'영상 파일을 찾을 수 없습니다: {path}')
    if len(set(found)) != 1:
        raise ValueError(f'영상 루트에 동명 파일이 있습니다: {path}')
    return found[0]


def validate_options(options):
    result = {'format': 'npz', 'camera_topic': '', 'label': 'joint_plan', 'chunk_len': 20,
              'delta_sec': 0.1, 'step_sec': 0.1, 'fps': 10, 'max_skew_sec': 0.2,
              'copy_media': True, 'keep_untrustworthy': False, **options}
    if result['format'] not in ('npz', 'lerobot_v3') or result['label'] not in ('joint_plan', 'joint_command'):
        raise ValueError('지원하지 않는 export 형식 또는 label입니다.')
    for key in ('delta_sec', 'step_sec', 'max_skew_sec'):
        value = float(result[key])
        if not math.isfinite(value) or value < 0 or (key != 'delta_sec' and value == 0):
            raise ValueError(f'{key} 값을 확인하세요.')
        result[key] = value
    for key, maximum in [('fps', 120), ('chunk_len', 1000)]:
        value = result[key]
        if isinstance(value, bool) or int(value) != value or not 1 <= int(value) <= maximum:
            raise ValueError(f'{key}는 1~{maximum} 정수여야 합니다.')
        result[key] = int(value)
    return result


def grid_frames(clock, fps, max_skew_sec):
    """Causal image sampling; do not fill arbitrarily long gaps."""
    stamps = [f.stamp_ns for f in clock.frames]
    if not stamps or any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise ValueError('카메라 stamp가 비어 있거나 증가하지 않습니다.')
    start = max(clock.episode_start_ns, stamps[0])
    end = min(clock.episode_end_ns, stamps[-1] + 1)
    count = math.ceil((end - start) * fps / 10**9)
    for i in range(count):
        stamp = start + (i * 10**9 // fps)
        if stamp >= end:
            break
        idx = bisect_right(stamps, stamp) - 1
        skew = (stamp - stamps[idx]) / 10**9
        if skew > max_skew_sec:
            raise ValueError(f'영상 공백이 허용 시차를 넘었습니다: {skew:.6f}초')
        yield stamp, clock.frames[idx].index, skew


def export_dataset(db, episode_ids, output, options, *, media_roots=(), control=None):
    control = control or Control()
    options = validate_options(options)
    ids = sorted(set(int(x) for x in episode_ids))
    if not ids or any(x < 1 for x in ids):
        raise ValueError('내보낼 에피소드를 선택하세요.')
    out = Path(output).expanduser().resolve()
    if out.exists():
        raise ValueError('출력 경로가 이미 존재합니다. 새 데이터셋 경로를 지정하세요.')
    out.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f'.{out.name}-', dir=out.parent))
    target = staging / 'dataset'
    accepted, excluded, provenance, lines = [], [], [], []
    dataset = None
    try:
        # Manager mutations share this lock. Legacy CLI/direct SQL must be run separately.
        with db.connection(write=True, snapshot=True) as conn:
            rows = conn.execute('SELECT id FROM sessions WHERE id=ANY(%s)', (ids,)).fetchall()
            if {r['id'] for r in rows} != set(ids):
                raise ValueError('선택한 에피소드 중 DB에 없는 ID가 있습니다.')
            if db.ready(conn):
                provenance = conn.execute('SELECT s.id,d.source_id FROM sessions s LEFT JOIN dm_episode_sources d '
                                          'ON s.id=d.episode_id WHERE s.id=ANY(%s) ORDER BY s.id', (ids,)).fetchall()
            conn.row_factory = tuple_row
            first = load_frame_clock(conn, ids[0], topic=options['camera_topic'] or None)
            arm = default_arm_joints(conn, ids[0], label=options['label'])
            backend = episode_backend(conn, ids[0])
            spec = ExportSpec(label=options['label'], delta_sec=options['delta_sec'],
                              step_sec=options['step_sec'], chunk_len=options['chunk_len'],
                              arm_joints=arm, gripper_symbols=GRIPPER_SYMBOLS,
                              camera_topic=first.stream.topic,
                              camera_hz=first.interval_stats().get('hz', 0), backend=backend)
            obs_stats, action_stats = RunningStats(len(arm)+1), RunningStats(len(arm))
            if options['format'] == 'npz':
                target.mkdir()
            for index, ep_id in enumerate(ids):
                control.progress(f'Export 에피소드 {index+1}/{len(ids)}: {ep_id}')
                clock = load_frame_clock(conn, ep_id, topic=spec.camera_topic)
                if default_arm_joints(conn, ep_id, label=spec.label) != arm:
                    raise ValueError(f'{ep_id}: 팔 관절 구성·순서가 다릅니다.')
                if spec.label == 'joint_command' and episode_backend(conn, ep_id) != backend:
                    raise ValueError('joint_command export에는 여러 백엔드를 섞을 수 없습니다.')
                if not clock.frames or any(a.stamp_ns >= b.stamp_ns for a, b in zip(clock.frames, clock.frames[1:])):
                    raise ValueError(f'{ep_id}: 영상 frame clock이 비어 있거나 역행·중복됩니다.')
                if any(not clock.episode_start_ns <= f.stamp_ns < clock.episode_end_ns for f in clock.frames):
                    raise ValueError(f'{ep_id}: 영상 시각이 에피소드 범위 밖입니다.')
                video = media_path(clock.stream.mp4_path, media_roots)
                joints = load_joint_series(conn, ep_id, '/joint_states', joint_names=arm)
                label, assumed, _, _, _ = _build_label(conn, ep_id, clock, joints,
                    label=spec.label, arm_joints=arm, command_topic='/target_joint_cmds')
                if assumed and not options['keep_untrustworthy']:
                    excluded.append({'episode_id': ep_id, 'reason': 'motion 종료 이벤트 누락', 'count': assumed})
                    continue
                if options['format'] == 'npz':
                    line = export_episode(conn, ep_id, out_dir=str(target), spec=spec,
                                          qpos_stats=obs_stats, action_stats=action_stats)
                    with np.load(target / 'episodes' / f'{ep_id}.npz', allow_pickle=False) as check:
                        if not all(np.isfinite(check[k]).all() for k in ('qpos', 'action_arm', 'action_gripper', 't')):
                            raise ValueError(f'{ep_id}: 학습 배열에 NaN/Infinity가 있습니다.')
                    if options['copy_media']:
                        dest = target / 'videos' / f'{ep_id}.mp4'
                        dest.parent.mkdir(exist_ok=True)
                        with video.open('rb') as src, dest.open('wb') as dst:
                            while block := src.read(1024 * 1024):
                                control.check()
                                dst.write(block)
                        line = replace(line, mp4=str(dest.relative_to(target)))
                    else:
                        line = replace(line, mp4=str(video))
                    lines.append(line)
                else:
                    dataset, info = _lerobot_episode(dataset, target, conn, ep_id, clock, video,
                                                     arm, joints, label, options, control)
                    lines.append(info)
                accepted.append(ep_id)
            if not accepted:
                raise ValueError(f'품질 검사 후 내보낼 에피소드가 없습니다: {excluded}')
            control.check()
            if dataset is not None:
                dataset.finalize()
                from lerobot.datasets.lerobot_dataset import LeRobotDataset
                loaded = LeRobotDataset('local/rdfp', root=target, video_backend='pyav')
                for row in lines:
                    loaded[row['first_index']]
                    loaded[row['last_index']]
                if len(loaded) != sum(r['frames'] for r in lines):
                    raise ValueError('LeRobot 재읽기 샘플 수가 일치하지 않습니다.')
            else:
                meta = build_metadata(spec, obs_stats, action_stats, episodes=tuple(accepted),
                                      samples=sum(line.samples for line in lines))
                _write_json(target / 'metadata.json', meta.to_dict())
                with (target / 'episodes.jsonl').open('w') as stream:
                    for line in lines:
                        stream.write(json.dumps(line.to_dict(), ensure_ascii=False) + '\n')
            manifest = {'version': 1, 'format': options['format'], 'options': options,
                        'requested_episodes': ids, 'accepted_episodes': accepted, 'excluded': excluded,
                        'provenance': provenance, 'gripper_symbols': list(GRIPPER_SYMBOLS),
                        'action_meaning': 'a(t); future offsets belong to the loader' if dataset else 'precomputed chunks',
                        'files': {}, 'lerobot_episode_mapping': lines if dataset else None}
            if dataset:
                manifest['lerobot_version'] = importlib.metadata.version('lerobot')
            for p in sorted(target.rglob('*')):
                if p.is_file():
                    manifest['files'][str(p.relative_to(target))] = hash_file(p, control)
            _write_json(target / 'rdfp_manifest.json', manifest)
            control.check()
            if out.exists():
                raise ValueError('다른 작업이 출력 경로를 생성했습니다.')
            target.rename(out)
        return {'output': str(out), 'episodes': accepted, 'excluded': excluded}
    finally:
        # Finalized output is outside staging. Cancelled/failed partial data never looks complete.
        shutil.rmtree(staging, ignore_errors=True)


def _write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def _lerobot_episode(dataset, root, conn, ep_id, clock, video, arm, joints, label, options, control):
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as exc:
        raise ValueError('LeRobot v3 export에는 별도 환경의 lerobot==0.4.4가 필요합니다. 운영 가이드를 확인하세요.') from exc
    import cv2
    widths, commands = load_gripper_width(conn, ep_id), load_gripper_commands(conn, ep_id)
    if dataset is None:
        dataset = LeRobotDataset.create('local/rdfp', fps=options['fps'], root=root,
            robot_type='panda', use_videos=True, vcodec='h264', video_backend='pyav', features={
                'observation.state': {'dtype': 'float32', 'shape': (len(arm)+1,), 'names': [*arm, 'gripper_width_m']},
                'action': {'dtype': 'float32', 'shape': (len(arm)+3,), 'names': [*arm, *GRIPPER_SYMBOLS]},
                'observation.images.camera': {'dtype': 'video', 'shape': (clock.stream.height, clock.stream.width, 3),
                                              'names': ['height', 'width', 'channels']}})
    capture = cv2.VideoCapture(str(video))
    count, max_skew, previous, repeats = 0, 0.0, None, 0
    first_index = dataset.meta.total_frames
    task = conn.execute('SELECT task_label FROM sessions WHERE id=%s', (ep_id,)).fetchone()[0] or 'unspecified'
    origin = None
    try:
        for stamp, frame_index, skew in grid_frames(clock, options['fps'], options['max_skew_sec']):
            control.check()
            capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
            ok, bgr = capture.read()
            if not ok:
                raise ValueError(f'{video}: 프레임 {frame_index} 디코딩 실패')
            t = stamp / 10**9
            state = np.asarray(qpos(joints, widths, t), dtype=np.float32)
            action = np.asarray((*label.at(t), *commands.one_hot(t)), dtype=np.float32)
            if not np.isfinite(state).all() or not np.isfinite(action).all():
                raise ValueError(f'{ep_id}: 관측 또는 행동값이 유한하지 않습니다.')
            dataset.add_frame({'observation.state': state, 'action': action,
                               'observation.images.camera': cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), 'task': task})
            origin = stamp if origin is None else origin
            repeats += int(previous == frame_index)
            previous = frame_index
            count += 1
            max_skew = max(max_skew, skew)
        if not count:
            raise ValueError(f'{ep_id}: 생성할 영상 프레임이 없습니다.')
        dataset.save_episode()
    finally:
        capture.release()
    return dataset, {'source_episode_id': ep_id, 'episode_index': dataset.meta.total_episodes-1,
                     'first_index': first_index, 'last_index': first_index+count-1, 'frames': count,
                     'origin_ns': origin, 'max_image_skew_sec': max_skew, 'repeated_frames': repeats}
