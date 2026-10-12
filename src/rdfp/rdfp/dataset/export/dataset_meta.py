"""정규화와 데이터셋 metadata (E6).

두 가지를 만든다.

1. **차원별 정규화 통계** — `qpos` 와 팔 행동을 학습 데이터의 평균 · 표준편차로
   정규화한다(ACT 문서 §3.3). **그리퍼 one-hot 은 정규화하지 않는다** — 0/1 을
   표준화하면 심볼의 뜻이 사라진다.
2. **배치가 학습과 같은 규약을 쓰는지 검사할 수 있는 기록** — ACT 문서 §7 이
   *"배치 때의 카메라 주기 · δ(Δ) · T_a · k 가 학습 때와 같아야 한다"* 고 못 박는데,
   **어긋나도 에러가 안 난다.** 정책은 그냥 조금씩 틀린 곳을 겨냥할 뿐이다. 그래서
   :meth:`ExportSpec.assert_compatible` 로 **검사를 실행 가능하게** 만든다.

**통계는 스트리밍으로 누적한다**(Welford) — 에피소드가 쌓이면 모든 샘플을 메모리에
들고 있을 수 없고, 단순 누적합은 큰 값에서 정밀도를 잃는다.

이 모듈은 ROS 도 DB 도 모른다.
"""

from __future__ import annotations

from typing import Any, Iterable, Optional, Sequence

import math
from dataclasses import dataclass, field

#: 표준편차가 이보다 작으면 **정규화하지 않는다**(1.0 으로 둔다). 0 으로 나누면
#: `inf`/`NaN` 이 되어 학습이 조용히 망가진다.
STD_FLOOR = 1e-6


class MetadataError(RuntimeError):
    """학습과 배치의 규약이 어긋날 때. **조용히 넘기면 정책이 틀린 곳을 겨냥한다.**"""


@dataclass
class RunningStats:
    """차원별 평균 · 표준편차를 **한 번 훑으며** 누적한다 (Welford).

    ``NaN`` 은 **그 차원만 건너뛰고 센다** — 펑션베이는 그리퍼 폭을 못 구해 `NaN` 을
    내는데(0 은 '닫혀 있다'는 거짓말이라 그렇게 둔 것이다), 그것이 섞이면 평균과
    표준편차가 통째로 `NaN` 이 된다.
    """

    dim: int
    count: list = field(default_factory=list)
    _mean: list = field(default_factory=list)
    _m2: list = field(default_factory=list)
    #: 차원별로 건너뛴 `NaN` 개수. 0 이 아니면 데이터를 들여다볼 근거다.
    nan_count: list = field(default_factory=list)

    def __post_init__(self) -> None:
        if self.dim <= 0:
            raise ValueError(f'dim must be positive, got {self.dim}')
        for name in ('count', '_mean', '_m2', 'nan_count'):
            if not getattr(self, name):
                setattr(self, name, [0.0] * self.dim if name != 'count'
                        else [0] * self.dim)
        self.nan_count = [0] * self.dim if not any(self.nan_count) else self.nan_count

    def update(self, values: Sequence[float]) -> None:
        """표본 하나를 더한다."""
        if len(values) != self.dim:
            raise ValueError(f'expected {self.dim} values, got {len(values)}')
        for i, v in enumerate(values):
            v = float(v)
            if math.isnan(v):
                self.nan_count[i] += 1
                continue
            self.count[i] += 1
            delta = v - self._mean[i]
            self._mean[i] += delta / self.count[i]
            self._m2[i] += delta * (v - self._mean[i])

    def update_many(self, rows: Iterable[Sequence[float]]) -> None:
        for row in rows:
            self.update(row)

    @property
    def mean(self) -> tuple:
        """차원별 평균. 표본이 없으면 0.0."""
        return tuple(m if c else 0.0 for m, c in zip(self._mean, self.count))

    @property
    def std(self) -> tuple:
        """차원별 표준편차(모표준편차). 표본이 2 개 미만이면 0.0."""
        return tuple(math.sqrt(m2 / c) if c > 1 else 0.0
                     for m2, c in zip(self._m2, self.count))


@dataclass(frozen=True)
class Normalizer:
    """차원별 z-정규화. **연속값에만 쓴다** — one-hot 은 그대로 둔다.

    Attributes:
        mean / std: 차원별 통계.
        degenerate: **표준편차가 바닥값 미만인 차원**의 번호. 그 차원은 정규화하지
            않는다(나누면 `inf` 가 된다). ⚠️ **움직이지 않은 관절이라는 뜻**이라
            데이터가 한쪽으로 치우쳤다는 신호이기도 하다 — 비어 있는 것이 정상이다.
    """

    mean: tuple
    std: tuple
    degenerate: tuple = ()

    @classmethod
    def from_stats(cls, stats: RunningStats, *, floor: float = STD_FLOOR) -> 'Normalizer':
        mean, std = stats.mean, stats.std
        bad = tuple(i for i, s in enumerate(std) if s < floor)
        return cls(mean=tuple(mean),
                   std=tuple(1.0 if i in bad else s for i, s in enumerate(std)),
                   degenerate=bad)

    def apply(self, values: Sequence[float]) -> tuple:
        return tuple((v - m) / s for v, m, s in zip(values, self.mean, self.std))

    def invert(self, values: Sequence[float]) -> tuple:
        """배치에서 정책 출력을 되돌린다. :meth:`apply` 의 역이다."""
        return tuple(v * s + m for v, m, s in zip(values, self.mean, self.std))

    def to_dict(self) -> dict:
        return {'mean': list(self.mean), 'std': list(self.std),
                'degenerate': list(self.degenerate)}

    @classmethod
    def from_dict(cls, d: dict) -> 'Normalizer':
        return cls(mean=tuple(d['mean']), std=tuple(d['std']),
                   degenerate=tuple(d.get('degenerate') or ()))


@dataclass(frozen=True)
class ExportSpec:
    """샘플을 어떻게 만들었는가 — **배치가 그대로 따라야 하는 값들**.

    Attributes:
        label: ``'joint_plan'``(관절-계획, 1차 학습) 또는 ``'joint_command'``(비교 기준).
            **레이블이 다르면 배치 경로가 다르다** — 계획은 컨트롤러가 보간하고,
            명령은 제어 주기로 직접 넣는다.
        delta_sec: 첫 스텝 오프셋 (계획은 Δ, 명령은 δ).
        step_sec: 행동 스텝 T_a.
        chunk_len: chunk 길이 k.
        arm_joints: 팔 행동·관측의 관절 순서. **이름이 곧 차원의 뜻이다.**
        gripper_symbols: one-hot 순서. 배치가 argmax 로 되돌리므로 계약이다.
        camera_topic: frame clock 을 준 토픽.
        camera_hz: **실측** 프레임 주기. 설정값이 아니다.
        backend: 어느 스택에서 모은 데이터인가. ``None`` 이면 기록되지 않았다는 뜻이다
            (에피소드 metadata 규약 R4 가 붙으면 채워진다).
        kinematic_model_hash: 기구학 모델 지문. I2 가 붙으면 채워진다.
    """

    label: str
    delta_sec: float
    step_sec: float
    chunk_len: int
    arm_joints: tuple
    gripper_symbols: tuple
    camera_topic: str
    camera_hz: float
    backend: Optional[str] = None
    kinematic_model_hash: Optional[str] = None

    #: 배치와 반드시 같아야 하는 키들. 여기 없는 것(예: `camera_hz`)은 재현이
    #: 불가능하거나 근사로 충분한 값이다 — 아래 주석 참조.
    STRICT_KEYS = ('label', 'delta_sec', 'step_sec', 'chunk_len', 'arm_joints',
                   'gripper_symbols')

    def assert_compatible(self, other: 'ExportSpec', *,
                          camera_hz_tol: float = 0.2) -> None:
        """배치 규약이 학습과 같은지 검사한다 (ACT 문서 §7).

        **어긋나도 에러가 안 나는 것이 문제라서** 검사를 명시적으로 둔다 — 정책은
        조금씩 틀린 곳을 겨냥할 뿐이고, 그 증상은 "학습은 됐는데 잘 안 된다"다.

        ``camera_hz`` 는 **정확히 같기를 요구하지 않는다** — 실측값이라 회차마다
        흔들린다(펑션베이 9.54~9.60 Hz). 대신 상대 오차가 ``camera_hz_tol`` 을
        넘으면 올린다: 주기가 그만큼 달라지면 chunk 가 덮는 시간 범위가 달라진다.

        ``backend`` 는 **한쪽이라도 비어 있으면 넘어간다** — 아직 기록하지 않는
        데이터가 있기 때문이다. 둘 다 있는데 다르면 올린다(관절-명령 레이블은
        백엔드를 섞으면 안 된다).

        Raises:
            MetadataError: 어긋난 항목이 있을 때. **전부 모아서** 알린다.
        """
        bad = [f'{k}: train={getattr(self, k)!r} deploy={getattr(other, k)!r}'
               for k in self.STRICT_KEYS if getattr(self, k) != getattr(other, k)]
        if self.camera_hz > 0 and other.camera_hz > 0:
            rel = abs(other.camera_hz - self.camera_hz) / self.camera_hz
            if rel > camera_hz_tol:
                bad.append(f'camera_hz: train={self.camera_hz:.3f} '
                           f'deploy={other.camera_hz:.3f} (rel {rel:.1%})')
        if self.backend and other.backend and self.backend != other.backend:
            bad.append(f'backend: train={self.backend!r} deploy={other.backend!r}')
        if bad:
            raise MetadataError('deployment spec does not match training: '
                                + '; '.join(bad))

    def to_dict(self) -> dict:
        return {'label': self.label, 'delta_sec': self.delta_sec,
                'step_sec': self.step_sec, 'chunk_len': self.chunk_len,
                'arm_joints': list(self.arm_joints),
                'gripper_symbols': list(self.gripper_symbols),
                'camera_topic': self.camera_topic, 'camera_hz': self.camera_hz,
                'backend': self.backend,
                'kinematic_model_hash': self.kinematic_model_hash}

    @classmethod
    def from_dict(cls, d: dict) -> 'ExportSpec':
        return cls(label=d['label'], delta_sec=float(d['delta_sec']),
                   step_sec=float(d['step_sec']), chunk_len=int(d['chunk_len']),
                   arm_joints=tuple(d['arm_joints']),
                   gripper_symbols=tuple(d['gripper_symbols']),
                   camera_topic=d['camera_topic'], camera_hz=float(d['camera_hz']),
                   backend=d.get('backend'),
                   kinematic_model_hash=d.get('kinematic_model_hash'))


@dataclass(frozen=True)
class DatasetMetadata:
    """데이터셋 하나를 설명하는 기록 — 학습도 배치도 이것을 읽는다.

    Attributes:
        spec: 샘플 생성 규약.
        qpos: 관측 정규화 (팔 관절 + 그리퍼 폭).
        arm_action: 팔 행동 정규화. **그리퍼 one-hot 은 없다** — 정규화하지 않는다.
        episodes: 포함된 에피소드 id.
        samples: 총 샘플 수.
        notes: 사람이 읽을 경고 (움직이지 않은 관절, `NaN` 개수 등).
    """

    spec: ExportSpec
    qpos: Normalizer
    arm_action: Normalizer
    episodes: tuple = ()
    samples: int = 0
    notes: tuple = ()

    def to_dict(self) -> dict:
        return {'spec': self.spec.to_dict(), 'qpos': self.qpos.to_dict(),
                'arm_action': self.arm_action.to_dict(),
                'episodes': list(self.episodes), 'samples': self.samples,
                'notes': list(self.notes)}

    @classmethod
    def from_dict(cls, d: dict) -> 'DatasetMetadata':
        return cls(spec=ExportSpec.from_dict(d['spec']),
                   qpos=Normalizer.from_dict(d['qpos']),
                   arm_action=Normalizer.from_dict(d['arm_action']),
                   episodes=tuple(d.get('episodes') or ()),
                   samples=int(d.get('samples') or 0),
                   notes=tuple(d.get('notes') or ()))


def build_metadata(spec: ExportSpec, qpos_stats: RunningStats,
                   action_stats: RunningStats, *, episodes: Sequence[int] = (),
                   samples: int = 0) -> DatasetMetadata:
    """누적한 통계로 데이터셋 metadata 를 만든다.

    **경고를 사람이 읽을 문장으로 남긴다** — 움직이지 않은 관절이나 `NaN` 이 섞인
    차원은 에러가 아니지만 데이터를 들여다볼 근거다. 숫자만 남기면 아무도 안 본다.
    """
    qpos = Normalizer.from_stats(qpos_stats)
    action = Normalizer.from_stats(action_stats)
    notes = []
    names = list(spec.arm_joints)
    for i in qpos.degenerate:
        what = names[i] if i < len(names) else ('gripper width' if i == len(names)
                                                else f'dim {i}')
        notes.append(f'qpos dimension {i} ({what}) never varied; left unnormalised')
    for i in action.degenerate:
        what = names[i] if i < len(names) else f'dim {i}'
        notes.append(f'arm action dimension {i} ({what}) never varied; left unnormalised')
    for i, n in enumerate(qpos_stats.nan_count):
        if n:
            notes.append(f'qpos dimension {i} had {n} NaN sample(s) (skipped in stats)')
    if spec.backend is None:
        notes.append('backend is unknown — episode metadata did not record it (R4)')
    if spec.kinematic_model_hash is None:
        notes.append('kinematic model fingerprint is unknown (I2 not in place)')
    return DatasetMetadata(spec=spec, qpos=qpos, arm_action=action,
                           episodes=tuple(episodes), samples=samples,
                           notes=tuple(notes))


def episode_backend(conn: Any, episode_id: int) -> Optional[str]:
    """`sessions.metadata` 에서 `backend` 를 읽는다. 없으면 ``None``.

    **기록돼 있지 않은 것이 지금은 정상이다** — 에피소드 metadata 규약(R4)이 아직
    없다. 채워지면 이 함수가 그대로 값을 준다.
    """
    with conn.cursor() as cur:
        cur.execute('SELECT metadata FROM sessions WHERE id = %s', (episode_id,))
        row = cur.fetchone()
    meta = (row or [None])[0]
    return meta.get('backend') if isinstance(meta, dict) else None


__all__ = ['DatasetMetadata', 'ExportSpec', 'MetadataError', 'Normalizer',
           'RunningStats', 'STD_FLOOR', 'build_metadata', 'episode_backend']
