"""관절-계획 레이블 함수 `plan(τ)` 와 action chunk (E3).

1차 학습의 팔 행동이다(수집 설계 D2). **계획은 연산이 실행될 때만 있으므로** 주기
토픽이 아니라 이벤트로 녹화하고(D3 · D4), 빈 구간은 여기서 규칙으로 채운다(D5).

τ 의 위치에 따라 값이 다르다 ([ACT 문서](../../../../docs/learning/학습_샘플_사양.md) §5.2):

===========================  ==========================================================
τ 의 위치                    plan(τ)
===========================  ==========================================================
연산 s 실행 중               연산 s 의 실행 궤적을 ``τ − t₀ₛ`` 에서 보간
연산 s 가 끝난 뒤            정상 종료면 궤적의 끝점, 중단·실패면 **중단 시각의 궤적 값**
첫 연산 이전                 에피소드 시작 시점의 관절 관측
===========================  ==========================================================

**뒤의 두 줄이 한 규칙으로 합쳐진다** — 연산의 끝 시각을 `t₀ + truncated_at` 으로 두면
"끝난 뒤"는 곧 "궤적을 `truncated_at` 에서 평가한 값"이다. 정상 종료는 `truncated_at`
이 궤적 길이와 같으므로 자연히 끝점이 된다. 그래서 아래 구현에 분기가 하나뿐이다.

⚠️ **연산 사이를 직전 값으로 채우는 것과, 움직이는 동안 경유점을 반복하는 것은 다르다.**
뒤쪽은 금지다 — 정책이 "현재 위치"를 명령하도록 배워 **배치 때 팔이 전진하지 않는다**
(수집 설계 §4 의 B 레이블 규칙 표). 앞쪽은 사실에 맞는다: 그 구간에는 컨트롤러가 실제로
마지막 설정값을 붙들고 있다.

**이 모듈은 ROS 도 DB 도 모른다.** 순수 계산이라 시뮬레이터 없이 시험한다 —
`load_operations` 만 커넥션을 받으며, 그것도 인자로만 쓴다.
"""

from __future__ import annotations

from typing import Any, Iterable, Optional, Sequence

import bisect
from dataclasses import dataclass, field

#: 궤적을 넘기지도 못한 연산. 실행 궤적이 없으므로 시간축에 자리를 차지하지 않는다.
RESULT_FAILED = 'failed'


@dataclass(frozen=True)
class Waypoint:
    """실행 궤적의 경유점 하나.

    Attributes:
        t: **궤적 시작으로부터의 경과 초** (`time_from_start`). 절대 시각이 아니다.
        positions: 관절 위치. 순서는 소속 :class:`Operation` 의 ``joint_names`` 다.
        velocities: 있으면 3차 Hermite, 없으면 선형으로 보간한다. **빈 것을 0 으로
            채우면 안 된다** — 0 은 "그 점에서 멈춰 있으라"는 뜻이라 곡선이 달라진다.
    """

    t: float
    positions: tuple
    velocities: tuple = ()


@dataclass(frozen=True)
class Operation:
    """연산 하나 — 실행 궤적(R1)과 연산 이벤트(R2)를 합친 것.

    Attributes:
        t0: 궤적의 0초가 시작된 **절대 시각**. R1 의 `header.stamp` 다.
        joint_names: ``waypoints`` 의 관절 순서.
        waypoints: 경유점. 시각 오름차순이어야 한다.
        end: 실제로 멈춘 **절대 시각** = ``t0 + truncated_at``. 정상 종료면 궤적 끝이다.
        result: R2 의 결과. ``'failed'`` 는 궤적이 없어 시간축에서 빠진다.
        operation_id: 두 채널을 이은 값. 진단용이다.
    """

    t0: float
    joint_names: tuple
    waypoints: tuple
    end: float
    result: str = 'succeeded'
    operation_id: str = ''

    @property
    def duration(self) -> float:
        """궤적 길이(초) — 마지막 경유점의 시각."""
        return self.waypoints[-1].t if self.waypoints else 0.0


@dataclass
class LabelStats:
    """레이블을 만들며 드러난 것. **조용히 넘기면 안 되는 것만 센다.**"""

    operations: int = 0
    #: 궤적이 없어 시간축에서 뺀 연산 (`failed`). 짝이 없는 것이 정상이다.
    skipped_failed: int = 0
    #: 이벤트가 없어 **정상 종료로 가정한** 연산. 실제로는 잘렸을 수 있다.
    assumed_full_duration: int = 0
    #: 앞 연산이 끝나기 전에 시작한 연산. 팔 자원 락이 있으면 0 이어야 한다.
    overlapping: int = 0
    notes: list = field(default_factory=list)


class PlanLabel:
    """에피소드 하나의 관절-계획 레이블 함수.

    Args:
        operations: 그 에피소드의 연산들. 순서는 상관없다 — ``t0`` 로 정렬한다.
        joint_names: **레이블 벡터의 관절 순서(정본)**. 연산마다 순서가 다를 수 있어
            여기에 맞춰 재배열한다. 어긋나면 **에러가 아니라 관절이 뒤바뀐 레이블**이
            되므로 생성 시점에 막는다.
        initial_positions: 에피소드 시작 시점의 관절 관측. **첫 연산 이전 구간의 값**이다.
        episode_end: 에피소드 끝 시각. chunk 의 ``is_pad`` 판정에 쓴다. ``None`` 이면
            패딩이 없다고 본다.

    Raises:
        ValueError: 관절 이름이 안 맞거나 ``initial_positions`` 길이가 다를 때.
    """

    def __init__(self, operations: Iterable[Operation], *, joint_names: Sequence[str],
                 initial_positions: Sequence[float],
                 episode_end: Optional[float] = None) -> None:
        self.joint_names = tuple(joint_names)
        if len(initial_positions) != len(self.joint_names):
            raise ValueError(
                f'initial_positions has {len(initial_positions)} values but '
                f'{len(self.joint_names)} joint names were given')
        self.initial_positions = tuple(float(v) for v in initial_positions)
        self.episode_end = episode_end
        self.stats = LabelStats()

        ops = []
        for op in operations:
            self.stats.operations += 1
            # 궤적이 없는 연산은 시간축에 자리가 없다 — 움직인 적이 없기 때문이다.
            if op.result == RESULT_FAILED or not op.waypoints:
                self.stats.skipped_failed += 1
                continue
            ops.append(self._reorder(op))
        ops.sort(key=lambda o: o.t0)
        for prev, cur in zip(ops, ops[1:]):
            if cur.t0 < prev.end:
                self.stats.overlapping += 1
        self._ops = tuple(ops)
        self._starts = [o.t0 for o in self._ops]

    # ----- 조회 ----------------------------------------------------------

    def at(self, tau: float) -> tuple:
        """시각 ``tau`` 에서 계획상 팔이 있어야 할 관절 자세."""
        if not self._ops or tau < self._starts[0]:
            return self.initial_positions
        # **t₀ ≤ tau 인 마지막 연산**을 고른다. 겹침이 있어도 나중 것이 이긴다 —
        # 실제로 나중에 보낸 명령이 앞의 것을 덮기 때문이다.
        op = self._ops[bisect.bisect_right(self._starts, tau) - 1]
        cut = op.end - op.t0
        # 끝난 뒤라면 **끝 시각에서 평가한다** — 정상 종료면 궤적 끝점이고,
        # 중단이면 중단 시각의 궤적 값이다 (D5 의 두 줄이 한 식으로 합쳐진다).
        return _sample(op.waypoints, min(max(tau - op.t0, 0.0), max(cut, 0.0)))

    def chunk(self, t: float, *, k: int, step: float,
              delta: float) -> tuple[tuple, tuple]:
        """관측 시각 ``t`` 의 action chunk 를 만든다.

        ``τ_j = t + delta + j × step`` (j = 0 … k-1) 에서 :meth:`at` 을 평가한다.

        **경유점 번호가 아니라 시각으로 뽑는다.** 경유점 간격은 계획 종류와 속도
        배율에 따라 100 ms 가 아니고(데카르트는 `10 × vs` Hz), 카메라와 위상도 다르다.

        **chunk 가 다음 연산으로 넘어가도 자르지 않는다** — 시연이 실제로 이어서 한
        동작이므로 다음 연산의 계획이 그대로 들어오는 것이 맞다(ACT 문서 §5.4).

        Returns:
            ``(values, is_pad)``. ``values`` 는 k 개의 관절 튜플, ``is_pad`` 는
            τ_j 가 에피소드 끝을 넘었는지다. 패딩 구간의 값은 마지막 유효 자세가
            그대로 이어진다(:meth:`at` 이 끝점을 유지하므로 따로 채우지 않는다).

        Raises:
            ValueError: ``k`` 나 ``step`` 이 양수가 아닐 때.
        """
        if k <= 0:
            raise ValueError(f'k must be positive, got {k}')
        if step <= 0:
            raise ValueError(f'step must be positive, got {step}')
        taus = [t + delta + j * step for j in range(k)]
        values = tuple(self.at(tau) for tau in taus)
        if self.episode_end is None:
            return values, tuple(False for _ in taus)
        return values, tuple(tau > self.episode_end for tau in taus)

    # ----- 내부 ----------------------------------------------------------

    def _reorder(self, op: Operation) -> Operation:
        """연산의 경유점을 정본 관절 순서로 재배열한다.

        **조용히 넘기지 않는다** — 관절이 하나라도 빠지면 레이블 벡터의 어느 칸이
        다른 관절 값이 되는데, 값의 범위가 비슷해 데이터를 열어봐도 안 드러난다.
        """
        if tuple(op.joint_names) == self.joint_names:
            return op
        try:
            index = [list(op.joint_names).index(n) for n in self.joint_names]
        except ValueError as exc:
            raise ValueError(
                f'operation {op.operation_id!r} is missing joints: expected '
                f'{list(self.joint_names)}, got {list(op.joint_names)}') from exc
        points = tuple(
            Waypoint(t=w.t,
                     positions=tuple(w.positions[i] for i in index),
                     velocities=tuple(w.velocities[i] for i in index) if w.velocities else ())
            for w in op.waypoints)
        return Operation(t0=op.t0, joint_names=self.joint_names, waypoints=points,
                         end=op.end, result=op.result, operation_id=op.operation_id)


def _sample(waypoints: Sequence[Waypoint], rel: float) -> tuple:
    """궤적을 상대 시각 ``rel`` 에서 평가한다.

    경유점에 속도가 있으면 **3차 Hermite**, 없으면 선형이다. MoveIt 의 시간
    파라미터화는 경유점마다 속도를 채우므로 보통 Hermite 로 간다 — 선형으로 두면
    경유점 사이에서 속도가 계단이 되어 실제 궤적과 다른 곡선이 된다.
    """
    if rel <= waypoints[0].t:
        return tuple(waypoints[0].positions)
    if rel >= waypoints[-1].t:
        return tuple(waypoints[-1].positions)
    times = [w.t for w in waypoints]
    i = max(0, bisect.bisect_right(times, rel) - 1)
    a, b = waypoints[i], waypoints[min(i + 1, len(waypoints) - 1)]
    span = b.t - a.t
    if span <= 0:
        return tuple(a.positions)
    s = (rel - a.t) / span
    if a.velocities and b.velocities:
        return _hermite(a, b, span, s)
    return tuple(p0 + (p1 - p0) * s for p0, p1 in zip(a.positions, b.positions))


def _hermite(a: Waypoint, b: Waypoint, span: float, s: float) -> tuple:
    """3차 Hermite 보간 — 위치와 속도를 둘 다 맞춘다."""
    s2, s3 = s * s, s * s * s
    h00 = 2 * s3 - 3 * s2 + 1
    h10 = s3 - 2 * s2 + s
    h01 = -2 * s3 + 3 * s2
    h11 = s3 - s2
    return tuple(h00 * p0 + h10 * span * v0 + h01 * p1 + h11 * span * v1
                 for p0, v0, p1, v1 in zip(a.positions, a.velocities,
                                           b.positions, b.velocities))


def load_operations(conn: Any, episode_id: int) -> tuple[list, LabelStats]:
    """한 에피소드의 연산을 DB 에서 읽어 :class:`Operation` 목록으로 만든다.

    **`operation_id` 로 두 테이블을 잇는다** — FK 가 아니라 인덱스다. 계획에서 실패한
    연산은 이벤트만 있고 궤적이 없으며(그것이 ``'failed'`` 의 정의다), 반대로 이벤트
    발행에만 실패하면 궤적만 남는다. 둘 다 정상이므로 **OUTER JOIN 이어야 한다.**

    이벤트가 없는 궤적은 **정상 종료로 가정한다** — 잘렸는지 알 수 없지만, 빼 버리면
    그 구간을 직전 연산의 끝점이 채워 더 틀린 레이블이 된다. 가정한 건수를
    :class:`LabelStats` 로 돌려주므로 호출자가 그 에피소드를 제외할 수 있다.
    """
    stats = LabelStats()
    sql = """
        SELECT tr.stamp_sec, tr.stamp_nanosec, tr.joint_names, tr.points,
               tr.duration_sec, tr.operation_id,
               ev.result, ev.truncated_at_sec
        FROM motion_trajectories tr
        LEFT JOIN motion_events ev
               ON ev.episode_id = tr.episode_id AND ev.operation_id = tr.operation_id
        WHERE tr.episode_id = %s
        ORDER BY tr.stamp_sec, tr.stamp_nanosec
    """
    with conn.cursor() as cur:
        cur.execute(sql, (episode_id,))
        rows = cur.fetchall()

    operations = []
    for sec, nsec, names, points, duration, op_id, result, cut in rows:
        stats.operations += 1
        t0 = sec + nsec / 1_000_000_000
        if result is None:
            stats.assumed_full_duration += 1
            result, cut = 'succeeded', duration
        if cut is None:
            cut = duration or 0.0
        operations.append(Operation(
            t0=t0, joint_names=tuple(names or []),
            waypoints=tuple(Waypoint(t=float(p.get('t', 0.0)),
                                     positions=tuple(p.get('positions') or ()),
                                     velocities=tuple(p.get('velocities') or ()))
                            for p in (points or [])),
            end=t0 + float(cut), result=str(result), operation_id=str(op_id or '')))
    if stats.assumed_full_duration:
        stats.notes.append(
            f'{stats.assumed_full_duration} trajectory(ies) had no matching event; '
            f'assumed they ran to full duration')
    return operations, stats


__all__ = ['LabelStats', 'Operation', 'PlanLabel', 'Waypoint', 'load_operations']
