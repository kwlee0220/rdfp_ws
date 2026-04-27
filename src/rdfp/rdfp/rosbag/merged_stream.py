"""여러 split 파일의 메시지를 `header.stamp` 기준으로 스트리밍 머지한다.

각 split 은 이미 stamp 오름차순으로 정렬된 `TimedMessage` 를 yield 하므로,
`heapq.merge` 로 키 함수만 주어 여러 split 을 합친다.
"""

from __future__ import annotations

from typing import Iterable, Iterator

import heapq

from .mcap_reader import TimedMessage, iter_split_messages


def merge_split_streams(split_paths: Iterable[str], *,
                        topics: list[str] | None = None,
                        time_range_ns: tuple[int, int] | None = None,) -> Iterator[TimedMessage]:
    """여러 split 의 메시지를 하나의 stamp 오름차순 스트림으로 결합한다.

    Args:
        split_paths: `.mcap` 경로 iterable.
        topics: 선행 토픽 필터 (`iter_split_messages` 에 그대로 전달).
        time_range_ns: 선행 시간 필터 (`iter_split_messages` 에 그대로 전달).

    Yields:
        stamp 오름차순의 `TimedMessage`.
    """
    iterators = [
        iter_split_messages(p, topics=topics, time_range_ns=time_range_ns)
        for p in split_paths
    ]
    # heapq.merge 는 key 함수를 지원 (Python 3.5+). 안정성은 같은 키에서 삽입
    # 순서를 유지하므로 split 순서가 tie-breaker 가 된다.
    yield from heapq.merge(*iterators, key=lambda m: (m.stamp_ns, m.topic))


__all__ = ['merge_split_streams']
