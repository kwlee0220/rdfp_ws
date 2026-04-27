from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Iterator, Protocol

from std_msgs.msg import Header


class StampedMessage(Protocol):
    header: Header


@dataclass
class ReplayStream:
    """재생 대상 토픽 하나를 나타낸다 (lazy / iterator 기반).

    publish 루프가 streaming merge (heapq.merge) 로 한 메시지씩 소비할 수 있도록
    iterator 형태로 노출한다. 메시지 타입과 첫 stamp 는 ``first_message`` 에서
    peek 하고, 전체 시퀀스는 ``iterator`` 에서 순차로 yield 된다 (``first_message``
    포함). 외부 자원(예: ``cv2.VideoCapture``) 은 ``close()`` 로 정리한다.

    Attributes:
        topic_name: 발행 대상 토픽 이름.
        first_message: peek 용 첫 메시지. publisher 의 메시지 타입과 anchor stamp
            계산에 사용된다.
        iterator: ``first_message`` 를 포함한 전체 메시지 시퀀스 (stamp ASC).
        close: source 가 보유한 외부 자원을 해제하는 콜러블 (idempotent).
        expected_count: 진단용 전체 메시지 수 (lazy source 도 메타 카운트로 알 수
            있는 경우 채운다). 알 수 없으면 ``None``. publish 루프의 동작에는
            영향을 주지 않는다.
    """

    topic_name: str
    first_message: StampedMessage
    iterator: Iterator[StampedMessage]
    close: Callable[[], None] = field(default=lambda: None)
    expected_count: int | None = None


__all__ = ['StampedMessage', 'ReplayStream']
