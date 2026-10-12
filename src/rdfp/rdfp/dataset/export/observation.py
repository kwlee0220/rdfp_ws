"""관측 `qpos` 와 그리퍼 행동 (E2 · E4).

ACT 샘플의 나머지 절반이다 — 팔 행동은 `motion_label` 이, 기준 시각열은 `frame_clock`
이 맡는다.

======================  ====================================================
채널                    frame clock 시각 t 에서의 값
======================  ====================================================
팔 관측 (`/joint_states`)  t 에서 **선형 보간** (연속·조밀)
그리퍼 관측 (`width`)      t 에 **가장 가까운** 샘플 (주기 발행)
그리퍼 행동 (`gripper_cmds`) τ 이전의 **마지막 명령을 0차 유지** (이벤트)
======================  ====================================================

**그리퍼 상태를 `/joint_states` 의 손가락 관절에서 읽지 않는다** — 펑션베이는 TF
성립용 고정값(0.04)을 주입해 "항상 열려 있다"고 거짓말한다. `width` 는 관절값이
아니라 **개구 폭(m)** 이고, 못 구하면 `NaN` 이다(0 은 '닫혀 있다'는 거짓말이 된다).

**그리퍼 행동은 one-hot 이다** — ACT 는 연속값에 L1 을 쓰지만 그리퍼 명령은 심볼이다.
목표 폭 같은 숫자로 바꾸면 "명령은 심볼, 관측은 물리량" 이라는 결정과 어긋난다.

이 모듈은 ROS 를 모른다 — 로더만 DB 커넥션을 받는다.
"""

from __future__ import annotations

from typing import Any, Optional, Sequence

import bisect
import math
from dataclasses import dataclass

NS = 1_000_000_000

#: 그리퍼 명령 심볼의 one-hot 순서. **배치가 argmax 로 되돌리므로 순서가 계약이다.**
GRIPPER_SYMBOLS = ('open', 'close', 'grasp')

#: 첫 그리퍼 명령 이전 구간의 값 (Q7 결정, 2026-09-18). 아래 주석 참조.
GRIPPER_PRIOR = 'open'


class ObservationError(RuntimeError):
    """관측을 만들 수 없을 때. **조용히 0 으로 채우지 않는다.**"""


@dataclass(frozen=True)
class JointSeries:
    """관절 관측의 시간열 — 이름 순서를 정본에 맞춘 뒤의 값이다."""

    joint_names: tuple
    stamps_ns: tuple
    positions: tuple

    def at(self, t: float) -> tuple:
        """시각 ``t`` 에서 **선형 보간**한 관절 위치.

        범위 밖은 **양 끝 값을 유지한다**(Q8 결정) — 첫 프레임이 첫 관측보다 이르거나
        마지막 프레임이 마지막 관측보다 늦는 일이 실제로 생긴다(카메라와 관절이 다른
        주기로 발행된다). 그 프레임을 **버리지 않는 이유**는 영상이 이미 있기
        때문이다: 버리면 mp4 프레임과 샘플 수가 어긋나 `frame_index` 가 밀린다.
        """
        if not self.stamps_ns:
            raise ObservationError('joint series is empty')
        ns = int(round(t * NS))
        if ns <= self.stamps_ns[0]:
            return self.positions[0]
        if ns >= self.stamps_ns[-1]:
            return self.positions[-1]
        i = bisect.bisect_right(self.stamps_ns, ns) - 1
        t0, t1 = self.stamps_ns[i], self.stamps_ns[i + 1]
        p0, p1 = self.positions[i], self.positions[i + 1]
        span = t1 - t0
        if span <= 0:
            return p0
        s = (ns - t0) / span
        return tuple(a + (b - a) * s for a, b in zip(p0, p1))


@dataclass(frozen=True)
class GripperWidthSeries:
    """그리퍼 개구 폭의 시간열. 값이 없으면 ``NaN`` 이다."""

    stamps_ns: tuple
    widths: tuple

    def at(self, t: float) -> float:
        """시각 ``t`` 에 **가장 가까운** 샘플의 폭.

        보간하지 않는 이유는 개폐가 계단에 가깝고(램프 구간이 짧다) 보간이 없는
        중간값을 만들기 때문이다. 샘플이 없으면 ``NaN`` 이다 — **0 을 돌려주면
        '닫혀 있다'는 거짓말**이 된다(Q8 결정).
        """
        if not self.stamps_ns:
            return math.nan
        ns = int(round(t * NS))
        i = bisect.bisect_left(self.stamps_ns, ns)
        if i == 0:
            return self.widths[0]
        if i >= len(self.stamps_ns):
            return self.widths[-1]
        before, after = self.stamps_ns[i - 1], self.stamps_ns[i]
        return self.widths[i - 1] if (ns - before) <= (after - ns) else self.widths[i]


@dataclass(frozen=True)
class GripperCommandSeries:
    """그리퍼 명령(심볼)의 이벤트 열."""

    stamps_ns: tuple
    goals: tuple

    def at(self, t: float) -> str:
        """시각 ``t`` 이전의 **마지막 명령을 0차 유지**한다.

        심볼은 경유점이 아니라 **유지되는 의도 상태**다 — `grasp` 를 한 번 보내면
        다음 명령까지 쥐고 있는 것이 사실이다. (팔에서 0차 유지를 금지하는 것과
        다른 이야기다: 그쪽은 움직이는 동안 경유점을 반복하는 경우다.)

        **첫 명령 이전 구간은 :data:`GRIPPER_PRIOR` 다** (Q7 결정, 2026-09-18) —
        수집 루프가 에피소드마다 `ready` 로 시작하고 그 직전에 그리퍼를 여는 것이
        관례이며(펑션베이는 `fb_ready.goto_ready()` 가 강제한다), 무엇보다 **닫힘을
        기본값으로 두면 정책이 "시작하자마자 쥐라"를 배운다.** 실제 상태가 다르면
        `width` 관측이 그것을 드러낸다.
        """
        if not self.stamps_ns:
            return GRIPPER_PRIOR
        ns = int(round(t * NS))
        i = bisect.bisect_right(self.stamps_ns, ns) - 1
        return GRIPPER_PRIOR if i < 0 else self.goals[i]

    def one_hot(self, t: float) -> tuple:
        """:meth:`at` 의 결과를 :data:`GRIPPER_SYMBOLS` 순서의 one-hot 으로.

        모르는 심볼이면 **전부 0** 이다 — 임의의 칸을 세우면 정책이 없는 명령을
        배운다. 그런 값이 있으면 데이터를 의심할 근거이므로 조용히 고치지 않는다.
        """
        goal = self.at(t)
        return tuple(1.0 if goal == sym else 0.0 for sym in GRIPPER_SYMBOLS)


def qpos(joints: JointSeries, widths: GripperWidthSeries, t: float) -> tuple:
    """ACT 의 관측 벡터 — 팔 관절 + 그리퍼 폭 하나 (ACT 문서 §3.1)."""
    return joints.at(t) + (widths.at(t),)


# ----- DB 로더 --------------------------------------------------------------

def load_joint_series(conn: Any, episode_id: int, topic: str, *,
                      joint_names: Optional[Sequence[str]] = None) -> JointSeries:
    """`joint_states` 에서 한 토픽의 시간열을 읽는다.

    Args:
        topic: `/joint_states`(관측) 또는 `/target_joint_cmds`(비교 기준 action).
            **두 토픽이 같은 테이블에 있고 `topic_id` 로만 갈린다.**
        joint_names: 뽑을 관절과 그 순서(정본). ``None`` 이면 첫 행의 순서를 쓴다.
            **관측에는 손가락 관절이 섞여 있으므로 보통 팔 7축을 명시한다.**

    Raises:
        ObservationError: 행이 없거나, 이름이 안 적혀 있거나(I3 이전 데이터),
            요청한 관절이 빠졌을 때. **조용히 순서로 때우지 않는다** — 값의 범위가
            비슷해 뒤바뀌어도 데이터에서 안 드러난다.
    """
    sql = ('SELECT j.stamp_sec, j.stamp_nanosec, j.joint_names, j.position '
           'FROM joint_states j JOIN topics t ON t.id = j.topic_id '
           'WHERE j.episode_id = %s AND t.topic_name = %s '
           'ORDER BY j.stamp_sec, j.stamp_nanosec')
    with conn.cursor() as cur:
        cur.execute(sql, (episode_id, topic))
        rows = cur.fetchall()
    if not rows:
        raise ObservationError(
            f'episode {episode_id} has no {topic!r} rows in joint_states')

    first_names = list(rows[0][2] or [])
    if not first_names:
        raise ObservationError(
            f'episode {episode_id}: {topic!r} rows have no joint_names — the publisher '
            f'did not fill JointState.name (or the data predates the column)')
    wanted = tuple(joint_names) if joint_names else tuple(first_names)

    stamps, values = [], []
    for sec, nsec, names, position in rows:
        names = list(names or [])
        try:
            idx = [names.index(n) for n in wanted]
        except ValueError as exc:
            raise ObservationError(
                f'episode {episode_id}: {topic!r} row is missing joints — wanted '
                f'{list(wanted)}, got {names}') from exc
        stamps.append(int(sec) * NS + int(nsec))
        values.append(tuple(float(position[i]) for i in idx))
    return JointSeries(joint_names=wanted, stamps_ns=tuple(stamps),
                       positions=tuple(values))


def load_gripper_width(conn: Any, episode_id: int) -> GripperWidthSeries:
    """`gripper_states` 에서 개구 폭의 시간열을 읽는다. 행이 없으면 빈 열이다."""
    sql = ('SELECT stamp_sec, stamp_nanosec, width FROM gripper_states '
           'WHERE episode_id = %s ORDER BY stamp_sec, stamp_nanosec')
    with conn.cursor() as cur:
        cur.execute(sql, (episode_id,))
        rows = cur.fetchall()
    return GripperWidthSeries(
        stamps_ns=tuple(int(s) * NS + int(n) for s, n, _ in rows),
        widths=tuple(math.nan if w is None else float(w) for _, _, w in rows))


def load_gripper_commands(conn: Any, episode_id: int) -> GripperCommandSeries:
    """`gripper_cmds` 에서 명령 심볼의 이벤트 열을 읽는다."""
    sql = ('SELECT stamp_sec, stamp_nanosec, goal FROM gripper_cmds '
           'WHERE episode_id = %s ORDER BY stamp_sec, stamp_nanosec')
    with conn.cursor() as cur:
        cur.execute(sql, (episode_id,))
        rows = cur.fetchall()
    return GripperCommandSeries(
        stamps_ns=tuple(int(s) * NS + int(n) for s, n, _ in rows),
        goals=tuple(str(g or '') for _, _, g in rows))


__all__ = ['GRIPPER_PRIOR', 'GRIPPER_SYMBOLS', 'GripperCommandSeries',
           'GripperWidthSeries', 'JointSeries', 'ObservationError',
           'load_gripper_commands', 'load_gripper_width', 'load_joint_series', 'qpos']
