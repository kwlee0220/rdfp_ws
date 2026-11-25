"""저장 형식 (E7) — 학습기가 읽을 파일로 내보낸다.

저장 형식 스키마는 [저장_형식.md §4](../../../../../docs/learning/저장_형식.md#4-저장-형식-스키마) 이다.

```
<dataset_root>/
├── metadata.json                 ← DatasetMetadata.to_dict() (E6)
├── episodes.jsonl                ← 에피소드 한 줄씩
└── episodes/<episode_id>.npz     ← 숫자
```

**영상은 이 트리에 복사하지 않는다** — `episodes.jsonl` 의 `mp4` 가 레코더 출력을
가리킨다. 경로는 `recordings_dir` 기준 **상대**다: 절대 경로로 적으면 학습 기계로
트리를 옮기는 순간 깨지는데, 학습은 수집 기계가 아닌 곳에서 도는 것이 정상이다.

**chunk 는 미리 계산해 저장한다**(설계 §3.3). frame clock 이 고정 격자가 아니라 실제
카메라 stamp 라(펑션베이 실측 9.54~9.60 Hz) 로더가 행을 이어 붙여 chunk 를 만들면
레이블이 조금씩 밀리는데, **틀린 양이 작아서 에러가 아니라 "학습이 잘 안 된다"로만
나타난다.** `plan(τ)` 는 시각의 함수라 τ_j 를 그대로 평가하면 그 문제가 사라진다.

**정규화는 걸지 않는다**(설계 §6.4). 통계만 `metadata.json` 에 남기고 값은 원값이다 —
에피소드를 더해 통계가 바뀌어도 저장본을 다시 안 써도 되고, 원값이라야 사람이 읽는다.
"""

from __future__ import annotations

from typing import Any, Iterable, Optional, Sequence

import dataclasses
import hashlib
import json
import os
import shutil
import uuid
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone

import numpy as np

from .command_label import CommandLabel
from .dataset_meta import ExportSpec, RunningStats, build_metadata, episode_backend
from .frame_clock import FrameClock, load_frame_clock
from .motion_label import PlanLabel, load_operations
from .observation import (GRIPPER_SYMBOLS, JointSeries, load_gripper_commands,
                          load_gripper_width, load_joint_series, qpos)

#: 레이블 종류. `ExportSpec.label` 에 그대로 들어간다.
LABEL_PLAN = 'joint_plan'
LABEL_COMMAND = 'joint_command'

#: 관절-계획(경우 B)의 원천 토픽과, 관절-명령(경우 A)의 원천 토픽.
DEFAULT_JOINT_TOPIC = '/joint_states'
DEFAULT_COMMAND_TOPIC = '/target_joint_cmds'


class ExportError(RuntimeError):
    """내보내기를 계속할 수 없을 때."""


#: `manifest.json` 의 `media_root` 값. 학습 로더(`rdfp-train`)가 mp4 를 어디 기준으로 찾을지 정한다.
MEDIA_ROOT_RECORDINGS = 'recordings_dir'   # episodes.jsonl 의 mp4 는 recordings_dir 기준 (기본)
MEDIA_ROOT_DATASET = 'dataset'             # --copy-media — mp4 를 트리 안 videos/ 로 복사했다
MANIFEST_NAME = 'manifest.json'


# ----- 원자적 게시 ---------------------------------------------------------

@contextmanager
def staged_output(out_dir: str):
    """출력을 옆의 임시 디렉터리에 쓰고, 끝까지 성공했을 때만 `out_dir` 로 **이름을 바꿔** 게시한다.

    실패 · 취소된 export 가 완성본처럼 보이면 학습이 반쪽 데이터셋을 읽는다(2026-11-23 — 예전에는
    `out_dir` 에 바로 써서 중간에 죽으면 일부 `.npz` 만 남았다). **이미 내용이 있는 `out_dir` 는
    거부한다** — 덮어쓰면 이전 결과와 섞인다. 빈 디렉터리는 받는다.
    """
    final = os.path.abspath(out_dir)
    if os.path.isdir(final) and os.listdir(final):
        raise ExportError(f'output directory is not empty: {final} — choose a new path')
    if os.path.exists(final) and not os.path.isdir(final):
        raise ExportError(f'output path exists and is not a directory: {final}')
    parent = os.path.dirname(final)
    os.makedirs(parent, exist_ok=True)
    staging = os.path.join(parent, f'.{os.path.basename(final)}.partial-{uuid.uuid4().hex[:8]}')
    os.makedirs(staging)
    try:
        yield staging
        if os.path.isdir(final):
            os.rmdir(final)        # 비어 있음을 위에서 확인했다
        os.rename(staging, final)
    finally:
        if os.path.isdir(staging):
            shutil.rmtree(staging, ignore_errors=True)


def sha256_file(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, 'rb') as fp:
        for block in iter(lambda: fp.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def write_manifest(root: str, *, fmt: str, media_root: str, requested: Sequence[int],
                   kept: Sequence[int], excluded: Sequence[dict], extra: Optional[dict] = None
                   ) -> dict:
    """`manifest.json` — 무엇을 요청했고 무엇이 들어갔으며 어느 파일이 어떤 내용인가.

    파일별 sha256 이 있어 학습 기계로 옮긴 뒤 손상 · 누락을 확인할 수 있다.
    """
    files = {}
    for dirpath, _dirs, names in os.walk(root):
        for name in sorted(names):
            path = os.path.join(dirpath, name)
            files[os.path.relpath(path, root)] = sha256_file(path)
    manifest = {'version': 1, 'format': fmt,
                'created_at': datetime.now(timezone.utc).isoformat(),
                'media_root': media_root, 'requested_episodes': list(requested),
                'kept_episodes': list(kept), 'excluded': list(excluded),
                'files': dict(sorted(files.items()))}
    manifest.update(extra or {})
    with open(os.path.join(root, MANIFEST_NAME), 'w', encoding='utf-8') as fp:
        json.dump(manifest, fp, ensure_ascii=False, indent=2)
        fp.write('\n')
    return manifest


def resolve_media(mp4_path: str, roots: Sequence[str]) -> str:
    """`image_streams.mp4_path`(출력 루트 기준 상대)를 실제 파일로. 못 찾으면 `ExportError`."""
    if os.path.isabs(mp4_path):
        if os.path.isfile(mp4_path):
            return mp4_path
    else:
        for root in roots:
            candidate = os.path.join(root, mp4_path)
            if root and os.path.isfile(candidate):
                return candidate
    raise ExportError(f'mp4 not found: {mp4_path} (looked under {list(roots)})')


@dataclass(frozen=True)
class EpisodeExport:
    """`episodes.jsonl` 한 줄. **세 카운트가 이 줄의 존재 이유다.**

    `load_operations` 가 `LabelStats` 를 돌려주는 까닭이 "호출자가 그 에피소드를
    제외할 수 있게" 이므로, 파일에 안 남기면 그 값이 사라진다.
    """

    episode_id: int
    samples: int
    mp4: str
    #: 이벤트(R2)가 없어 **정상 종료로 가정한** 연산 수. 실제로는 잘렸을 수 있다.
    assumed_full_duration: int = 0
    #: 앞 연산이 끝나기 전에 시작한 연산 수. 팔 자원 락이 있으면 0 이어야 한다.
    overlapping: int = 0
    #: 궤적이 없어 시간축에서 뺀 연산 수. 짝이 없는 것이 **정상**이다(`failed`).
    skipped_failed: int = 0
    #: `sessions.metadata` 를 **그대로** 통과시킨다 (설계 §6.1).
    metadata: dict = dataclasses.field(default_factory=dict)
    notes: tuple = ()

    @property
    def trustworthy(self) -> bool:
        """가정으로 만든 레이블이 섞이지 않았는가.

        **정책: `assumed_full_duration > 0` 이면 기본 제외한다**(설계 §6.1).
        `overlapping` 은 경고만 한다 — 값 자체는 나중 연산이 이긴다는 규칙으로
        정해지므로 레이블이 없는 것은 아니다.
        """
        return self.assumed_full_duration == 0

    def to_dict(self) -> dict:
        return {'episode_id': self.episode_id, 'samples': self.samples,
                'mp4': self.mp4,
                'assumed_full_duration': self.assumed_full_duration,
                'overlapping': self.overlapping,
                'skipped_failed': self.skipped_failed,
                'metadata': self.metadata, 'notes': list(self.notes)}


@dataclass(frozen=True)
class EpisodeArrays:
    """한 에피소드의 `.npz` 내용. 모양은 설계 §6.2 다."""

    frame_index: Any     # [N] int32 — mp4 안의 프레임 번호
    t: Any               # [N] float64 — 프레임 시각(초)
    qpos: Any            # [N, n+1] float32 — 팔 관절 + 그리퍼 폭
    action_arm: Any      # [N, k, n] float32
    action_gripper: Any  # [N, k, 3] float32 one-hot
    is_pad: Any          # [N, k] bool

    def save(self, path: str) -> None:
        np.savez(path, frame_index=self.frame_index, t=self.t, qpos=self.qpos,
                 action_arm=self.action_arm, action_gripper=self.action_gripper,
                 is_pad=self.is_pad)


# ----- 레이블 만들기 -------------------------------------------------------

def _build_label(conn: Any, episode_id: int, clock: FrameClock, joints: JointSeries,
                 *, label: str, arm_joints: Sequence[str], command_topic: str):
    """레이블 함수와 `LabelStats` 성격의 카운트를 만든다.

    **두 레이블은 같은 인터페이스다**(`at` · `chunk`) — 그래서 이 함수가 무엇을
    돌려주든 아래 격자 코드는 갈라지지 않는다(E5 의 설계 의도).
    """
    start = clock.episode_start_ns / 1_000_000_000
    initial = joints.at(start)
    if label == LABEL_PLAN:
        operations, loaded = load_operations(conn, episode_id)
        plan = PlanLabel(operations, joint_names=arm_joints, initial_positions=initial,
                         episode_end=clock.episode_end)
        # **카운트가 두 군데서 나온다** — `load_operations` 는 이벤트가 없어 정상
        # 종료로 *가정한* 건수를, `PlanLabel` 은 궤적이 없어 뺀 건수와 겹친 건수를
        # 센다. 한쪽만 보면 조용히 절반을 잃는다.
        return (plan, loaded.assumed_full_duration, plan.stats.overlapping,
                plan.stats.skipped_failed,
                tuple(loaded.notes) + tuple(plan.stats.notes))
    if label == LABEL_COMMAND:
        series = load_joint_series(conn, episode_id, command_topic, joint_names=arm_joints)
        cmd = CommandLabel(series, initial_positions=initial,
                           episode_end=clock.episode_end)
        # ⚠️ **단일 백엔드로만 쓴다** — 백엔드마다 보간이 다르고 펑션베이는 중력
        # 보상 오프셋이 들어 있다. 그 사실이 metadata 에 없다는 것이 설계 §6.3 의
        # 열린 항목(Q13′)이라, 여기서 note 로라도 남긴다.
        note = (f'command label: gap threshold {cmd.gap_sec:.3f} s '
                f'(median period {cmd.period_sec:.3f} s); single-backend only — '
                f'gravity-offset inclusion is not recorded in the spec (E7 Q13-prime)')
        return cmd, 0, 0, 0, (note,)
    raise ExportError(f'unknown label {label!r}; expected {LABEL_PLAN!r} or {LABEL_COMMAND!r}')


def default_arm_joints(conn: Any, episode_id: int, *, label: str,
                       command_topic: str = DEFAULT_COMMAND_TOPIC) -> tuple:
    """팔 관절 집합과 순서를 **행동 채널에서** 가져온다.

    ⚠️ **`/joint_states` 에서 가져오면 안 된다** — 거기에는 그리퍼 손가락 관절이
    섞여 있다(mock 실측: `panda_finger_joint1` · `2` 가 팔 7축과 함께 온다). 그것을
    팔 관절로 쓰면 ① 실행 궤적과 이름이 안 맞아 레이블이 서지 않고, ② `qpos` 가
    그리퍼를 **두 번** 싣는다(손가락 관절 + `width`).

    그래서 정본은 행동 채널이다 — 관절-계획은 실행 궤적(R1), 관절-명령은
    `/target_joint_cmds` 다. 고른 레이블의 채널에 이름이 없으면 **넘겨짚지 않고 올린다**
    (다른 채널로 대체하지 않는다): 이름으로 손가락을 걸러내는 것은 백엔드마다 다른
    추측이고, 틀려도 조용하다.
    """
    if label == LABEL_PLAN:
        with conn.cursor() as cur:
            cur.execute('SELECT joint_names FROM motion_trajectories '
                        'WHERE episode_id = %s ORDER BY stamp_sec, stamp_nanosec LIMIT 1',
                        (episode_id,))
            row = cur.fetchone()
        if row and row[0]:
            return tuple(row[0])
        raise ExportError(
            f'episode {episode_id} has no motion_trajectories to take the arm joint '
            f'order from; pass arm_joints explicitly (--arm-joints)')
    series = load_joint_series(conn, episode_id, command_topic)
    return series.joint_names


def _relative_mp4(mp4_path: str, recordings_dir: Optional[str]) -> str:
    """mp4 를 `recordings_dir` 기준 상대 경로로 만든다 (설계 §6.1).

    밖에 있으면 **원래 경로를 그대로 둔다** — 억지로 `..` 를 붙이면 옮길 때 더
    깨지기 쉬운 경로가 된다.
    """
    if not recordings_dir:
        return mp4_path
    try:
        rel = os.path.relpath(os.path.abspath(mp4_path), os.path.abspath(recordings_dir))
    except ValueError:  # 윈도우에서 드라이브가 다른 경우
        return mp4_path
    return mp4_path if rel.startswith(os.pardir) else rel


# ----- 에피소드 하나 -------------------------------------------------------

def export_episode(conn: Any, episode_id: int, *, out_dir: str, spec: ExportSpec,
                   qpos_stats: Optional[RunningStats] = None,
                   action_stats: Optional[RunningStats] = None,
                   joint_topic: str = DEFAULT_JOINT_TOPIC,
                   command_topic: str = DEFAULT_COMMAND_TOPIC,
                   recordings_dir: Optional[str] = None) -> EpisodeExport:
    """에피소드 하나를 `<out_dir>/episodes/<id>.npz` 로 내보낸다.

    Args:
        spec: 격자와 규약. `chunk_len` · `step_sec` · `delta_sec` · `arm_joints` ·
            `camera_topic` 을 쓴다. **`chunk_len` 은 고려 중인 최댓값으로 둔다** —
            `τ_j` 가 k 와 무관하므로 k=20 으로 저장한 `[:10]` 은 k=10 으로 뽑은 것과
            같은 값이라, 나중에 잘라 쓰면 재export 가 필요 없다(설계 §5).
        qpos_stats / action_stats: 주면 이 에피소드의 값을 **누적**한다. 여러
            에피소드의 통계를 모아 `metadata.json` 을 만들 때 쓴다.

    Returns:
        `episodes.jsonl` 에 적을 한 줄.
    """
    clock = load_frame_clock(conn, episode_id, topic=spec.camera_topic or None)
    if not len(clock):
        raise ExportError(f'episode {episode_id} has no camera frames')

    arm = tuple(spec.arm_joints) or default_arm_joints(
        conn, episode_id, label=spec.label, command_topic=command_topic)
    # 관측은 **팔 관절만** 골라 읽는다 — `/joint_states` 에 손가락이 섞여 있어도
    # 이름으로 인덱싱하므로 그리퍼가 `qpos` 에 두 번 실리지 않는다.
    joints = load_joint_series(conn, episode_id, joint_topic, joint_names=arm)
    widths = load_gripper_width(conn, episode_id)
    grip = load_gripper_commands(conn, episode_id)

    label, assumed, overlapping, skipped, notes = _build_label(
        conn, episode_id, clock, joints, label=spec.label, arm_joints=arm,
        command_topic=command_topic)

    n, k = len(arm), spec.chunk_len
    rows = len(clock)
    frame_index = np.empty(rows, dtype=np.int32)
    taus = np.empty(rows, dtype=np.float64)
    obs = np.empty((rows, n + 1), dtype=np.float32)
    act_arm = np.empty((rows, k, n), dtype=np.float32)
    act_grip = np.zeros((rows, k, len(GRIPPER_SYMBOLS)), dtype=np.float32)
    pad = np.empty((rows, k), dtype=bool)

    for i, frame in enumerate(clock.frames):
        t = frame.t
        frame_index[i] = frame.index
        taus[i] = t
        obs[i] = qpos(joints, widths, t)
        values, is_pad = label.chunk(t, k=k, step=spec.step_sec, delta=spec.delta_sec)
        act_arm[i] = values
        pad[i] = is_pad
        # 그리퍼는 팔과 **같은 τ_j 격자**에서 뽑는다 — 다르면 한 샘플 안에서 팔과
        # 손이 서로 다른 시각을 가리킨다.
        for j in range(k):
            act_grip[i, j] = grip.one_hot(t + spec.delta_sec + j * spec.step_sec)

    if qpos_stats is not None:
        qpos_stats.update_many(obs.tolist())
    if action_stats is not None:
        # 행동 통계는 **첫 스텝**으로 센다 — chunk 는 이웃끼리 겹쳐 같은 값을 k 번
        # 세게 되고, 그러면 겹치는 구간에 가중치가 실려 분포가 틀어진다.
        action_stats.update_many(act_arm[:, 0, :].tolist())

    episodes_dir = os.path.join(out_dir, 'episodes')
    os.makedirs(episodes_dir, exist_ok=True)
    EpisodeArrays(frame_index=frame_index, t=taus, qpos=obs, action_arm=act_arm,
                  action_gripper=act_grip, is_pad=pad).save(
        os.path.join(episodes_dir, f'{episode_id}.npz'))

    gaps = clock.interval_stats()
    if gaps and gaps.get('hz'):
        notes = notes + (f'camera frame rate measured {gaps["hz"]:.2f} Hz '
                         f'(gaps {gaps["min"]:.3f}~{gaps["max"]:.3f} s)',)
    return EpisodeExport(episode_id=episode_id, samples=rows,
                         mp4=_relative_mp4(clock.stream.mp4_path, recordings_dir),
                         assumed_full_duration=assumed, overlapping=overlapping,
                         skipped_failed=skipped,
                         metadata=_session_metadata(conn, episode_id), notes=notes)


def _stream_path(conn: Any, episode_id: int, camera_topic: str) -> str:
    """그 에피소드 · 카메라의 `image_streams.mp4_path` (출력 루트 기준 상대)."""
    return load_frame_clock(conn, episode_id, topic=camera_topic or None).stream.mp4_path


def _session_metadata(conn: Any, episode_id: int) -> dict:
    """`sessions.metadata` 를 **통째로** 가져온다.

    키를 골라 옮기지 않는 이유는 에피소드 metadata 규약(R4)이 아직 자라는 중이라서다
    — 골라서 옮기면 R4 가 바뀔 때마다 여기를 고쳐야 한다(설계 §6.1).
    """
    with conn.cursor() as cur:
        cur.execute('SELECT metadata FROM sessions WHERE id = %s', (episode_id,))
        row = cur.fetchone()
    meta = (row or [None])[0]
    return dict(meta) if isinstance(meta, dict) else {}


# ----- 데이터셋 -----------------------------------------------------------

def export_dataset(conn: Any, episode_ids: Iterable[int], *, out_dir: str,
                   label: str = LABEL_PLAN, delta_sec: float = 0.1,
                   step_sec: float = 0.1, chunk_len: int = 20,
                   arm_joints: Sequence[str] = (), camera_topic: str = '',
                   joint_topic: str = DEFAULT_JOINT_TOPIC,
                   command_topic: str = DEFAULT_COMMAND_TOPIC,
                   recordings_dir: Optional[str] = None,
                   skip_untrustworthy: bool = True,
                   copy_media: bool = False,
                   media_roots: Sequence[str] = ()) -> tuple:
    """에피소드 여러 개를 하나의 데이터셋 트리로 내보낸다.

    **`skip_untrustworthy` 는 파일을 안 쓰는 것이 아니라 통계와 목록에서 뺀다** —
    이미 쓴 `.npz` 는 남겨 둬야 왜 뺐는지 들여다볼 수 있다.

    **원자적으로 게시한다** (:func:`staged_output`) — 끝까지 성공해야 `out_dir` 가 생기고,
    `manifest.json` 에 파일별 sha256 이 남는다. `copy_media` 면 mp4 를 트리 안 `videos/` 로
    복사하고 `episodes.jsonl` 의 `mp4` 가 트리 기준이 된다(`manifest.media_root = 'dataset'`) —
    학습 기계로 옮길 때 recordings 디렉터리를 따로 챙기지 않아도 된다.

    Returns:
        `(DatasetMetadata, [EpisodeExport])`. 뒤엣것은 **뺀 것까지 전부**이고 각
        줄의 `trustworthy` 로 구분한다.
    """
    ids = list(episode_ids)
    if not ids:
        raise ExportError('no episodes to export')
    roots = [r for r in (*media_roots, recordings_dir) if r]
    with staged_output(out_dir) as staging:
        meta, lines = _export_into(conn, ids, staging, label=label, delta_sec=delta_sec,
                                   step_sec=step_sec, chunk_len=chunk_len,
                                   arm_joints=arm_joints, camera_topic=camera_topic,
                                   joint_topic=joint_topic, command_topic=command_topic,
                                   recordings_dir=recordings_dir,
                                   skip_untrustworthy=skip_untrustworthy,
                                   copy_media=copy_media, media_roots=roots)
        write_manifest(staging, fmt='rdfp_npz',
                       media_root=MEDIA_ROOT_DATASET if copy_media else MEDIA_ROOT_RECORDINGS,
                       requested=ids, kept=meta.episodes,
                       excluded=[{'episode_id': e.episode_id,
                                  'reason': 'assumed_full_duration'}
                                 for e in lines if e.episode_id not in meta.episodes])
    return meta, lines


def _export_into(conn: Any, ids: list, out_dir: str, *, label: str, delta_sec: float,
                 step_sec: float, chunk_len: int, arm_joints: Sequence[str], camera_topic: str,
                 joint_topic: str, command_topic: str, recordings_dir: Optional[str],
                 skip_untrustworthy: bool, copy_media: bool, media_roots: Sequence[str]) -> tuple:
    """`export_dataset` 의 본문 — `out_dir` 는 임시 디렉터리다."""

    first = load_frame_clock(conn, ids[0], topic=camera_topic or None)
    arm = tuple(arm_joints) or default_arm_joints(
        conn, ids[0], label=label, command_topic=command_topic)
    hz = (first.interval_stats() or {}).get('hz', 0.0)
    spec = ExportSpec(label=label, delta_sec=delta_sec, step_sec=step_sec,
                      chunk_len=chunk_len, arm_joints=arm,
                      gripper_symbols=GRIPPER_SYMBOLS,
                      camera_topic=camera_topic or first.stream.topic,
                      camera_hz=hz, backend=episode_backend(conn, ids[0]))

    qpos_stats = RunningStats(dim=len(arm) + 1)
    action_stats = RunningStats(dim=len(arm))
    lines, kept, samples = [], [], 0
    for episode_id in ids:
        # 통계는 에피소드마다 따로 모았다가 **포함한 에피소드만** 합친다 — 신뢰성은
        # 에피소드를 다 읽은 뒤에야 알 수 있으므로, 바로 누적하면 제외한 에피소드가
        # 정규화 통계에 섞인다.
        ep_qpos = RunningStats(dim=qpos_stats.dim)
        ep_action = RunningStats(dim=action_stats.dim)
        line = export_episode(conn, episode_id, out_dir=out_dir, spec=spec,
                              qpos_stats=ep_qpos, action_stats=ep_action,
                              joint_topic=joint_topic, command_topic=command_topic,
                              recordings_dir=recordings_dir)
        if copy_media:
            src = resolve_media(_stream_path(conn, episode_id, spec.camera_topic), media_roots)
            rel = os.path.join('videos', f'{episode_id}.mp4')
            os.makedirs(os.path.join(out_dir, 'videos'), exist_ok=True)
            shutil.copyfile(src, os.path.join(out_dir, rel))
            line = dataclasses.replace(line, mp4=rel)
        lines.append(line)
        if line.trustworthy or not skip_untrustworthy:
            kept.append(line.episode_id)
            samples += line.samples
            qpos_stats.merge(ep_qpos)
            action_stats.merge(ep_action)

    meta = build_metadata(spec, qpos_stats, action_stats, episodes=tuple(kept),
                          samples=samples)
    dropped = [e.episode_id for e in lines if e.episode_id not in kept]
    if dropped:
        meta = dataclasses.replace(meta, notes=meta.notes + (
            f'{len(dropped)} episode(s) excluded for assumed-full-duration '
            f'operations: {dropped}',))
    if label == LABEL_COMMAND:
        meta = dataclasses.replace(meta, notes=meta.notes + (
            'joint_command label: do not mix backends — interpolation differs and '
            'FunctionBay commands carry a gravity-compensation offset that this spec '
            'does not record (E7 Q13-prime)',))

    with open(os.path.join(out_dir, 'metadata.json'), 'w', encoding='utf-8') as fp:
        json.dump(meta.to_dict(), fp, ensure_ascii=False, indent=2)
        fp.write('\n')
    with open(os.path.join(out_dir, 'episodes.jsonl'), 'w', encoding='utf-8') as fp:
        for line in lines:
            fp.write(json.dumps(line.to_dict(), ensure_ascii=False) + '\n')
    return meta, lines


def load_episode(path: str) -> EpisodeArrays:
    """내보낸 `.npz` 를 다시 읽는다. 시험과 검증이 쓴다."""
    with np.load(path) as z:
        return EpisodeArrays(frame_index=z['frame_index'], t=z['t'], qpos=z['qpos'],
                             action_arm=z['action_arm'],
                             action_gripper=z['action_gripper'], is_pad=z['is_pad'])


__all__ = ['DEFAULT_COMMAND_TOPIC', 'DEFAULT_JOINT_TOPIC', 'EpisodeArrays',
           'EpisodeExport', 'ExportError', 'LABEL_COMMAND', 'LABEL_PLAN', 'MANIFEST_NAME',
           'MEDIA_ROOT_DATASET', 'MEDIA_ROOT_RECORDINGS', 'default_arm_joints',
           'export_dataset', 'export_episode', 'load_episode', 'resolve_media',
           'sha256_file', 'staged_output', 'write_manifest']
