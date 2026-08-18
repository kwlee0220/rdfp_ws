"""`/session` 토픽의 상태 머신에서 에피소드 구간을 추출한다.

규칙 (설계서 7.1):
  * `state == 'IN_EPISODE'` 이면서 직전 state 가 `IN_EPISODE` 가 아닌 메시지
    → start_msg. 해당 메시지의 header.stamp 가 start_ns 가 된다.
  * `state == 'IN_SESSION'` 이면서 직전 state 가 `IN_EPISODE` 인 메시지
    → stop_msg. 해당 메시지의 header.stamp 가 stop_ns 가 된다.
  * 그 외 전이는 무시한다. 특히 `IDLE → IN_SESSION` 에서의 IN_SESSION 은
    stop_msg 가 아니다.
"""

from __future__ import annotations

from typing import Iterable, NamedTuple

import json
import logging


_logger = logging.getLogger(__name__)


STATE_IDLE = 'IDLE'
STATE_IN_SESSION = 'IN_SESSION'
STATE_IN_EPISODE = 'IN_EPISODE'


class SessionEvent(NamedTuple):
    """에피소드 감지기에 입력되는 세션 메시지.

    `outcome` / `metadata` 는 **에피소드 종료 전이에서만** 채워져 온다 — 다른
    전이에서는 발행 측이 빈 문자열로 남긴다 (`rdfp_msgs/SessionCommand` 참고).
    """

    stamp_ns: int
    state: str
    task_label: str
    # '' | 'success' | 'failure'. '' 는 실패가 아니라 판정 없음이다.
    outcome: str = ''
    # JSON object 문자열. 없으면 ''.
    metadata: str = ''


class Episode(NamedTuple):
    """감지된 에피소드 구간."""

    start_ns: int
    stop_ns: int
    task_label: str | None
    # 작업 성패. None 은 **판정 없음**이며 실패가 아니다.
    success: bool | None = None
    # 재현·분석용 부가 정보 (seed, scene, 초기 물체 배치 등).
    metadata: dict | None = None


def parse_outcome(outcome: str) -> bool | None:
    """`SessionCommand.outcome` 을 DB 의 `success` 값으로 정규화한다.

    알 수 없는 값은 **판정 없음(None)** 으로 낮춘다. 발행 측이 검증하므로 여기까지
    올 일이 없지만, 온다면 그것을 실패로 단정하는 편이 더 나쁘다 — 성공한 에피소드가
    실패로 기록되면 학습셋에서 조용히 빠진다.
    """
    if outcome == 'success':
        return True
    if outcome == 'failure':
        return False
    return None


def parse_metadata(metadata: str) -> dict | None:
    """`SessionCommand.metadata`(JSON object 문자열)를 dict 로 바꾼다.

    비었거나 object 가 아니면 ``None``. 적재를 실패시키지 않는 이유는, 부가 정보
    하나 때문에 에피소드 본체(관절·이미지)를 잃는 것이 훨씬 큰 손실이기 때문이다.
    """
    if not metadata:
        return None
    try:
        parsed = json.loads(metadata)
    except (TypeError, ValueError):
        _logger.warning('episode metadata is not valid JSON; storing NULL: %r', metadata)
        return None
    if not isinstance(parsed, dict):
        _logger.warning('episode metadata is not a JSON object; storing NULL: %r', metadata)
        return None
    return parsed


def detect_episodes(events: Iterable[SessionEvent]) -> list[Episode]:
    """`/session` 이벤트 시퀀스에서 에피소드 목록을 추출한다.

    Args:
        events: stamp 오름차순으로 정렬된 `SessionEvent` 의 iterable.

    Returns:
        감지된 에피소드 목록. 종료 경계를 찾지 못한 마지막 에피소드는 버린다
        (경고 로깅).
    """
    episodes: list[Episode] = []
    prev_state: str | None = None
    current_start_ns: int | None = None
    current_task: str | None = None

    for ev in events:
        state = ev.state
        stamp = ev.stamp_ns

        if state == STATE_IN_EPISODE and prev_state != STATE_IN_EPISODE:
            current_start_ns = stamp
            current_task = ev.task_label or None
        elif state == STATE_IN_SESSION and prev_state == STATE_IN_EPISODE:
            if current_start_ns is None:
                _logger.warning(
                    'stop_msg observed without a recorded start_msg at stamp=%d', stamp,
                )
            else:
                if stamp <= current_start_ns:
                    _logger.warning(
                        'episode stop_ts (%d) not after start_ts (%d); skipping',
                        stamp, current_start_ns,
                    )
                else:
                    # outcome/metadata 는 **종료 이벤트**에 실려 온다. 시작
                    # 이벤트에는 없으므로 여기서 읽어야 한다.
                    episodes.append(Episode(
                        start_ns=current_start_ns,
                        stop_ns=stamp,
                        task_label=current_task,
                        success=parse_outcome(ev.outcome),
                        metadata=parse_metadata(ev.metadata),
                    ))
            current_start_ns = None
            current_task = None
        # 그 외 전이는 무시 (IDLE → IN_SESSION 포함).

        prev_state = state

    if current_start_ns is not None:
        _logger.warning(
            'unfinished episode discarded: start_ns=%d (no stop_msg before end of stream)',
            current_start_ns,
        )

    return episodes


__all__ = [
    'STATE_IDLE',
    'STATE_IN_SESSION',
    'STATE_IN_EPISODE',
    'SessionEvent',
    'Episode',
    'detect_episodes',
    'parse_metadata',
    'parse_outcome',
]
