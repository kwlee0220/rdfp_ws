"""관절-명령 레이블 (E5) — **비교 기준**이다.

1차 학습의 팔 행동은 관절-계획(`motion_label`)이고, 이쪽은 같은 DB 에서 뽑는
**단일 백엔드 비교 기준**이다(수집 설계 §0.4, 권고 3). 컨트롤러로 실제로 나간
명령값(`/target_joint_cmds`)을 frame clock 격자에서 보간한다.

관절-계획과 세 가지가 다르다.

============  ==========================  ==========================
              관절-계획 (`motion_label`)   관절-명령 (여기)
============  ==========================  ==========================
원천          연산마다 한 건인 궤적        제어 주기의 조밀한 명령열
공백          연산 사이 — 끝점 유지        명령이 끊긴 구간 — 직전 유지
백엔드 혼합    **된다**                    **안 된다** (아래)
============  ==========================  ==========================

⚠️ **단일 백엔드로만 쓴다.** 명령값에는 백엔드마다 다른 보간(isaac 은 JTC 스플라인
100 Hz, 펑션베이는 선형 50 Hz)과 **펑션베이의 중력 보상 오프셋**이 들어 있다. 섞으면
같은 자세가 백엔드마다 다른 레이블이 된다.

⚠️ **펑션베이는 어느 명령으로 학습했는지가 배치를 가른다.** 보상이 든
`/target_joint_cmds` 로 학습했으면 배치 때 보상을 다시 걸지 않고, 보상 전 명령으로
학습했으면 배치 경로에서 건다. 어긋나면 오프셋이 **두 번 들어가거나 빠진다.**

인터페이스는 :class:`~rdfp.dataset.export.motion_label.PlanLabel` 과 같다 —
`arm_action_chunks()` 에 그대로 바꿔 끼워 두 레이블을 같은 격자에서 비교한다.

이 모듈은 ROS 도 DB 도 모른다.
"""

from __future__ import annotations

from typing import Optional, Sequence

import bisect
import statistics

from .observation import JointSeries

NS = 1_000_000_000

#: 공백 판정 임계의 기본 배수. 앞뒤 명령 간격이 **명령 주기의 이 배수**를 넘으면
#: 보간하지 않고 직전 명령을 유지한다 (ACT 문서 §4.2; 값은 §9 의 2 로 열려 있다).
DEFAULT_GAP_FACTOR = 2.0


class CommandLabel:
    """관절-명령 레이블 함수.

    Args:
        series: `/target_joint_cmds` 의 시간열
            (:func:`~rdfp.dataset.export.observation.load_joint_series` 로 읽는다).
        initial_positions: **첫 명령 이전 구간의 값.** 에피소드 시작 시점의 관절
            관측을 준다 — 관절-계획의 "첫 연산 이전" 과 같은 상황이다.
        episode_end: 에피소드 끝(초). chunk 의 ``is_pad`` 판정에 쓴다.
        gap_sec: 공백 판정 임계(초). ``None`` 이면 **명령 간격의 중앙값 ×
            :data:`DEFAULT_GAP_FACTOR`** 로 정한다 — isaac 100 Hz 와 펑션베이 50 Hz
            의 주기가 달라 호출자가 백엔드를 알지 않아도 되게 한다. 추정값은
            :attr:`gap_sec` 로 확인한다.

    Raises:
        ValueError: 명령이 하나도 없거나 ``initial_positions`` 길이가 다를 때.
    """

    def __init__(self, series: JointSeries, *, initial_positions: Sequence[float],
                 episode_end: Optional[float] = None,
                 gap_sec: Optional[float] = None) -> None:
        if not series.stamps_ns:
            raise ValueError('command series is empty; there is nothing to label')
        if len(initial_positions) != len(series.joint_names):
            raise ValueError(
                f'initial_positions has {len(initial_positions)} values but the series '
                f'has {len(series.joint_names)} joints {list(series.joint_names)}')
        self.series = series
        self.joint_names = series.joint_names
        self.initial_positions = tuple(float(v) for v in initial_positions)
        self.episode_end = episode_end
        gaps = [(b - a) / NS for a, b in zip(series.stamps_ns, series.stamps_ns[1:])]
        #: 추정한 명령 주기(초) — 간격의 **중앙값**이다. 평균이 아닌 이유는 연산
        #: 사이의 긴 공백 몇 개가 평균을 통째로 끌어올리기 때문이다.
        self.period_sec = statistics.median(gaps) if gaps else 0.0
        self.gap_sec = (gap_sec if gap_sec is not None
                        else self.period_sec * DEFAULT_GAP_FACTOR)

    # ----- 조회 ----------------------------------------------------------

    def at(self, tau: float) -> tuple:
        """시각 ``tau`` 에서 컨트롤러로 나간 관절 명령값.

        구간 안이면 **선형 보간**, 공백이면 **직전 명령 유지**다.

        **공백을 보간하면 없던 움직임이 생긴다** — 명령은 팔이 움직이는 동안에만
        나오므로, 앞 동작의 끝과 다음 동작의 시작을 이으면 그 사이에 팔이 미끄러진
        것처럼 보인다. 컨트롤러는 실제로 마지막 설정값을 붙들고 있으므로 유지가
        사실에 맞다.
        """
        stamps = self.series.stamps_ns
        ns = int(round(tau * NS))
        if ns < stamps[0]:
            return self.initial_positions
        if ns >= stamps[-1]:
            return self.series.positions[-1]
        i = bisect.bisect_right(stamps, ns) - 1
        t0, t1 = stamps[i], stamps[i + 1]
        p0 = self.series.positions[i]
        if self.gap_sec > 0 and (t1 - t0) / NS > self.gap_sec:
            return p0                      # 공백 — 직전 명령을 유지한다
        span = t1 - t0
        if span <= 0:
            return p0
        s = (ns - t0) / span
        return tuple(a + (b - a) * s
                     for a, b in zip(p0, self.series.positions[i + 1]))

    def chunk(self, t: float, *, k: int, step: float,
              delta: float) -> tuple[tuple, tuple]:
        """관측 시각 ``t`` 의 action chunk. :class:`PlanLabel.chunk` 와 같은 모양이다.

        ``δ`` 를 두는 이유는 **t 의 명령이 관측보다 먼저 나가 팔이 이미 쫓고 있는
        값**이기 때문이다 — 관측 뒤에 나갈 명령부터 레이블로 쓴다. 기본은 제어 한
        주기이며(:attr:`period_sec` 로 추정값을 본다) 학습과 배치가 **같은 값**을
        써야 한다.

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

    def gap_intervals(self) -> tuple:
        """공백으로 판정되는 구간들 — ``(시작 초, 끝 초)``.

        **연산 사이의 공백이 여기 드러난다.** 개수가 연산 수와 크게 다르면 명령이
        빠졌다는 뜻이므로 그 에피소드를 의심할 근거가 된다(ACT 문서 §9 의 2 가
        재려는 분포이기도 하다).
        """
        if self.gap_sec <= 0:
            return ()
        stamps = self.series.stamps_ns
        return tuple((a / NS, b / NS) for a, b in zip(stamps, stamps[1:])
                     if (b - a) / NS > self.gap_sec)


__all__ = ['DEFAULT_GAP_FACTOR', 'CommandLabel']
