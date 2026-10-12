"""저장 형식 (E7) — 학습기가 읽을 파일로 내보낸다.

설계는 [E7_저장_형식_설계.md](../../../../docs/learning/저장_형식.md) §6 이다.

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
import json
import os
from dataclasses import dataclass

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
    `/target_joint_cmds` 다. 둘 다 없으면 **넘겨짚지 않고 올린다**: 이름으로
    손가락을 걸러내는 것은 백엔드마다 다른 추측이고, 틀려도 조용하다.
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
                   skip_untrustworthy: bool = True) -> tuple:
    """에피소드 여러 개를 하나의 데이터셋 트리로 내보낸다.

    **`skip_untrustworthy` 는 파일을 안 쓰는 것이 아니라 통계와 목록에서 뺀다** —
    이미 쓴 `.npz` 는 남겨 둬야 왜 뺐는지 들여다볼 수 있다.

    Returns:
        `(DatasetMetadata, [EpisodeExport])`. 뒤엣것은 **뺀 것까지 전부**이고 각
        줄의 `trustworthy` 로 구분한다.
    """
    ids = list(episode_ids)
    if not ids:
        raise ExportError('no episodes to export')
    os.makedirs(out_dir, exist_ok=True)

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
        line = export_episode(conn, episode_id, out_dir=out_dir, spec=spec,
                              qpos_stats=qpos_stats, action_stats=action_stats,
                              joint_topic=joint_topic, command_topic=command_topic,
                              recordings_dir=recordings_dir)
        lines.append(line)
        if line.trustworthy or not skip_untrustworthy:
            kept.append(line.episode_id)
            samples += line.samples

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
           'EpisodeExport', 'ExportError', 'LABEL_COMMAND', 'LABEL_PLAN',
           'default_arm_joints', 'export_dataset', 'export_episode', 'load_episode']
