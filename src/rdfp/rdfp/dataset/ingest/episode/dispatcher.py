"""에피소드 구간과 메시지 스트림을 sweep-line 으로 매칭한다.

설계서 8.1 의 구현. 에피소드 리스트(stamp 오름차순) 와 stamp 오름차순
메시지 스트림을 동시에 훑어 각 메시지를 소속 에피소드에 배정한다.
구간 규칙은 `[start_ns, stop_ns)` 반열림.

에피소드 경계에서 발생하는 전환 이벤트는 `enter`/`exit` 로 노출해
writer flush / mp4 sink open-close 에 사용한다. 메시지가 전혀 없는
에피소드도 `enter` + `exit` 이벤트는 반드시 발행된다 (sessions 레코드
생성이 보장되어야 하기 때문이다).
"""

from __future__ import annotations

from typing import Any, Iterable, Iterator, NamedTuple

from .detector import Episode


class AssignedMessage(NamedTuple):
    """특정 에피소드에 배정된 메시지."""

    episode_index: int     # episodes 리스트 내 인덱스 (0 기반)
    stamp_ns: int
    topic: str
    type_name: str
    msg: Any


class DispatchEvent(NamedTuple):
    """에피소드 전환/완료 이벤트."""

    kind: str              # 'enter' | 'exit'
    episode_index: int


def assign_and_dispatch(episodes: list[Episode], messages: Iterable[Any],) -> Iterator[tuple[str, Any]]:
    """에피소드 진입/이탈 이벤트와 배정 메시지를 교대로 yield 한다.

    출력 스트림 스키마:
        ('enter',   DispatchEvent(episode_index=0))
        ('message', AssignedMessage(episode_index=0, ...))
        ...
        ('exit',    DispatchEvent(episode_index=0))
        ('enter',   DispatchEvent(episode_index=1))
        ...
        ('exit',    DispatchEvent(episode_index=1))

    메시지가 어떤 에피소드 구간에도 속하지 않으면 yield 하지 않고 건너뛴다
    (구간 사이 혹은 구간 외부). 반면 메시지가 하나도 포함되지 않은 에피소드도
    `enter` / `exit` 이벤트는 발행된다.

    Args:
        episodes: `detect_episodes` 의 결과. stamp 오름차순이어야 한다.
        messages: `stamp_ns`, `topic`, `type_name`, `msg` 속성을 가진 객체의
            iterable (예: `TimedMessage`). stamp 오름차순이어야 한다.
    """
    if not episodes:
        return

    it = iter(messages)
    next_msg = next(it, None)

    for ep_idx, ep in enumerate(episodes):
        # 현재 에피소드 시작 이전의 메시지는 스킵한다 (이전 에피소드와 겹치는
        # 상황은 `detect_episodes` 가 만들어내지 않으므로 여기서 발생하지 않는다).
        while next_msg is not None and next_msg.stamp_ns < ep.start_ns:
            next_msg = next(it, None)

        yield 'enter', DispatchEvent(kind='enter', episode_index=ep_idx)

        while next_msg is not None and next_msg.stamp_ns < ep.stop_ns:
            yield 'message', AssignedMessage(
                episode_index=ep_idx,
                stamp_ns=next_msg.stamp_ns,
                topic=next_msg.topic,
                type_name=next_msg.type_name,
                msg=next_msg.msg,
            )
            next_msg = next(it, None)

        yield 'exit', DispatchEvent(kind='exit', episode_index=ep_idx)


__all__ = ['AssignedMessage', 'DispatchEvent', 'assign_and_dispatch']
