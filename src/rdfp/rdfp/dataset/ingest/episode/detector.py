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

import logging


_logger = logging.getLogger(__name__)


STATE_IDLE = 'IDLE'
STATE_IN_SESSION = 'IN_SESSION'
STATE_IN_EPISODE = 'IN_EPISODE'


class SessionEvent(NamedTuple):
    """에피소드 감지기에 입력되는 세션 메시지."""

    stamp_ns: int
    state: str
    task_label: str


class Episode(NamedTuple):
    """감지된 에피소드 구간."""

    start_ns: int
    stop_ns: int
    task_label: str | None


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
                    episodes.append(Episode(
                        start_ns=current_start_ns,
                        stop_ns=stamp,
                        task_label=current_task,
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
]
