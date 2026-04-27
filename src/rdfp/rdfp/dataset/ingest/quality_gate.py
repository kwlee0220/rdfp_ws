"""품질 게이트.

각 메시지를 훑으며 다음을 감지해 JSONL 로그에 `quality_warning` 로 남긴다.
에피소드 자체를 제외하거나 실패시키지는 않는다.

  * stamp_regression : 토픽별로 `header.stamp` 가 역행하는 경우.
  * idle_gap         : 토픽 내 인접 메시지의 stamp 간격이 임계 초과.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class QualityWarning:
    """품질 경고 한 건."""

    kind: str                  # 'stamp_regression' | 'idle_gap'
    topic: str
    stamp_ns: int
    prev_stamp_ns: int
    episode_id: int | None


class QualityGate:
    """토픽별 stamp 시퀀스를 검사한다.

    에피소드 단위로 사용한다 (`reset_episode` 로 상태를 비운 뒤 메시지를
    흘려넣고, 에피소드 종료 시 수집된 경고를 `pop_warnings()` 로 꺼낸다).
    """

    def __init__(
        self,
        *,
        check_stamp_regression: bool = True,
        idle_gap_ns: int = 0,
    ) -> None:
        self._check_regression = bool(check_stamp_regression)
        self._idle_gap_ns = int(idle_gap_ns)
        self._last_stamp: dict[str, int] = {}
        self._warnings: list[QualityWarning] = []
        self._episode_id: int | None = None

    def reset_episode(self, episode_id: int | None) -> None:
        self._last_stamp.clear()
        self._warnings.clear()
        self._episode_id = episode_id

    def on_message(self, topic: str, stamp_ns: int) -> None:
        prev = self._last_stamp.get(topic)
        if prev is not None:
            if self._check_regression and stamp_ns < prev:
                self._warnings.append(QualityWarning(
                    kind='stamp_regression', topic=topic,
                    stamp_ns=stamp_ns, prev_stamp_ns=prev,
                    episode_id=self._episode_id,
                ))
            elif self._idle_gap_ns > 0 and (stamp_ns - prev) > self._idle_gap_ns:
                self._warnings.append(QualityWarning(
                    kind='idle_gap', topic=topic,
                    stamp_ns=stamp_ns, prev_stamp_ns=prev,
                    episode_id=self._episode_id,
                ))
        self._last_stamp[topic] = stamp_ns

    def pop_warnings(self) -> list[QualityWarning]:
        out = list(self._warnings)
        self._warnings.clear()
        return out


__all__ = ['QualityGate', 'QualityWarning']
