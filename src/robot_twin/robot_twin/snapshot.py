"""상태 변수 스냅샷과 품질 판정 (설계서 4.1, 4.2, 5.1).

스냅샷은 **불변 객체**다. ROS 콜백은 새 스냅샷을 만들어 캐시의 참조 하나만
교체하고, HTTP 스레드는 그 참조를 읽는다. CPython 에서 속성 참조 대입은 GIL 하에서
원자적이므로 단일 변수 조회에는 락이 필요 없다 (설계서 2.2).

ROS 의존성이 없다 — 원본 메시지는 불투명한 객체로 보관하며, 해석은
:mod:`robot_twin.serialize` 가 담당한다.
"""

from __future__ import annotations

from typing import Any, Optional

import itertools
import time
from dataclasses import dataclass
from enum import Enum


class Quality(str, Enum):
    """조회 응답의 품질 판정 (설계서 5.1)."""

    OK = 'OK'
    STALE = 'STALE'
    NO_DATA = 'NO_DATA'
    SOURCE_UNAVAILABLE = 'SOURCE_UNAVAILABLE'
    ERROR = 'ERROR'


class Reason(str, Enum):
    """``SOURCE_UNAVAILABLE`` 의 구체적 사유 (설계서 5.1).

    품질 값은 하나로 유지해 클라이언트 분기를 단순하게 두고, 진단이 필요할 때만
    이 값을 본다.
    """

    NO_PUBLISHER = 'NO_PUBLISHER'
    TF_LOOKUP_FAILED = 'TF_LOOKUP_FAILED'
    SERVICE_UNAVAILABLE = 'SERVICE_UNAVAILABLE'
    SOURCE_NODE_DOWN = 'SOURCE_NODE_DOWN'


@dataclass(frozen=True)
class Snapshot:
    """특정 시점의 변수 값과 수신 메타데이터.

    Attributes:
        msg: 원본 ROS 메시지 참조. **변환하지 않는다** — rclpy 는 콜백마다 새 객체를
            역직렬화해 넘기므로 참조 보관이 안전하며, JSON 변환은 조회 시점에
            지연 수행한다 (설계서 2.2).
        received_at: ROS 콜백에서 찍은 **wall clock** 수신 시각(초).
        gen: 단조 증가 세대 번호. ETag 와 직렬화 메모이즈 키로 함께 쓴다.
        error: 값 획득 자체는 됐으나 후처리에 실패한 경우의 메시지.
    """

    msg: Any
    received_at: float
    gen: int
    error: Optional[str] = None


class GenerationCounter:
    """스냅샷 세대 번호 발급기.

    ``itertools.count`` 의 ``next`` 는 CPython 에서 원자적이므로 별도 락이 없다.
    """

    def __init__(self, start: int = 1) -> None:
        self._counter = itertools.count(start)

    def next(self) -> int:
        """다음 세대 번호를 반환한다."""
        return next(self._counter)


@dataclass(frozen=True)
class VariableStatus:
    """조회 시점에 판정된 변수 상태.

    Attributes:
        quality: 품질 판정.
        age_ms: 수신 후 경과 시간(ms). 값이 없으면 ``None``.
        reason: ``SOURCE_UNAVAILABLE`` 일 때의 사유.
        error: ``ERROR`` 일 때의 설명.
    """

    quality: Quality
    age_ms: Optional[int] = None
    reason: Optional[Reason] = None
    error: Optional[str] = None


def judge(snapshot: Optional[Snapshot], *, staleness_ms: Optional[int],
          source_available: bool = True, reason: Optional[Reason] = None,
          now: Optional[float] = None) -> VariableStatus:
    """스냅샷과 소스 가용성으로 품질을 판정한다 (설계서 5.1).

    판정 순서가 중요하다. 소스 자체를 못 쓰는 상황은 값의 나이보다 우선한다 —
    퍼블리셔가 없는 것과 퍼블리셔가 멈춘 것은 age 만 보면 구분되지 않기 때문이다.

    Args:
        snapshot: 캐시된 스냅샷. 아직 수신 전이면 ``None``.
        staleness_ms: staleness 임계값. ``None`` 이면 검사하지 않는다 — 이벤트성
            토픽이나 정적 소스는 값이 오래된 것이 정상이다.
        source_available: 소스(퍼블리셔/서비스/TF)가 사용 가능한지.
        reason: ``source_available`` 이 ``False`` 일 때의 사유.
        now: 현재 wall clock 시각(초). 테스트에서 주입한다.

    Returns:
        판정 결과.
    """
    if not source_available:
        return VariableStatus(Quality.SOURCE_UNAVAILABLE, reason=reason)

    if snapshot is None:
        return VariableStatus(Quality.NO_DATA)

    if snapshot.error is not None:
        return VariableStatus(Quality.ERROR, error=snapshot.error)

    # age 는 항상 wall clock `received_at` 기준으로만 계산한다. 메시지의
    # header.stamp 를 섞으면 use_sim_time 환경에서 값이 무의미해진다 (설계서 4.2).
    now = time.time() if now is None else now
    # 부동소수 표현 오차로 599.9999… 같은 값이 나오므로 절삭이 아니라 반올림한다.
    age_ms = max(0, round((now - snapshot.received_at) * 1000.0))

    if staleness_ms is not None and age_ms > staleness_ms:
        return VariableStatus(Quality.STALE, age_ms=age_ms)

    return VariableStatus(Quality.OK, age_ms=age_ms)


__all__ = [
    'GenerationCounter', 'Quality', 'Reason', 'Snapshot', 'VariableStatus', 'judge'
]
