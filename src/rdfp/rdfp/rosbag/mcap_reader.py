"""MCAP split 파일을 읽어 `header.stamp` 기준 메시지 스트림을 생성한다.

`mcap_ros2` 가 MCAP 파일에 내장된 ROS 2 IDL 스키마를 바탕으로 메시지를
동적으로 역직렬화해 주므로 `rdfp_msgs` 등 소스 메시지 패키지를 파이썬
환경에 설치할 필요가 없다. 본 모듈은 각 split 을 메모리에 로드한 뒤
`header.stamp` 로 안정 정렬하여 yield 한다.
"""

from __future__ import annotations

from typing import Any, Iterator, NamedTuple

import logging
from pathlib import Path

from mcap_ros2.reader import McapROS2Message, read_ros2_messages


_logger = logging.getLogger(__name__)


class TimedMessage(NamedTuple):
    """역직렬화된 ROS 2 메시지 + 정렬 키."""

    stamp_ns: int          # header.stamp 를 ns 로 합친 값
    topic: str             # 토픽 이름 (예: '/ee_pose_publisher/ee_pose')
    type_name: str         # 메시지 타입 (예: 'geometry_msgs/msg/PoseStamped')
    msg: Any               # mcap_ros2 가 동적 생성한 메시지 인스턴스
    log_time_ns: int       # rosbag 에 기록된 시각 (참고용)


def iter_split_messages(split_path: str | Path, *,
                        topics: list[str] | None = None,
                        time_range_ns: tuple[int, int] | None = None,) -> Iterator[TimedMessage]:
    """단일 MCAP split 파일에서 `header.stamp` 오름차순으로 메시지를 yield 한다.

    Args:
        split_path: `.mcap` 파일 경로.
        topics: 대상 토픽 목록. None 이면 모든 토픽.
        time_range_ns: `(start_ns, end_ns)` 로 `header.stamp` 기준 필터. None 이면
            시간 필터를 적용하지 않는다. end_ns 는 exclusive.

    Yields:
        `TimedMessage`. `header.stamp` 가 없는 메시지는 건너뛴다.

    Notes:
        * `mcap_ros2.read_ros2_messages` 의 `start_time`/`end_time` 은 `log_time`
          기준이므로 본 함수는 이 파라미터를 사용하지 않고 `header.stamp` 로
          재필터링한다.
        * 한 split 의 전체 메시지를 메모리에 적재 후 정렬한다. 운영 방안의
          1시간 상한 split 을 전제로 하며, 경로 변경이 필요하면 상위에서
          스트리밍 머지 경로를 교체한다.
    """
    buffered: list[TimedMessage] = []
    topic_filter = list(topics) if topics else None
    lo, hi = (time_range_ns[0], time_range_ns[1]) if time_range_ns else (None, None)

    for rec in read_ros2_messages(str(split_path), topics=topic_filter):
        stamp_ns = _extract_header_stamp_ns(rec)
        if stamp_ns is None:
            # header.stamp 가 없으면 정렬 불가능하므로 건너뛴다.
            _logger.debug(
                "skip message without header.stamp: topic=%s type=%s log_time_ns=%d",
                rec.channel.topic, rec.schema.name, rec.log_time_ns,
            )
            continue
        if lo is not None and stamp_ns < lo:
            continue
        if hi is not None and stamp_ns >= hi:
            continue
        buffered.append(TimedMessage(
            stamp_ns=stamp_ns,
            topic=rec.channel.topic,
            type_name=rec.schema.name,
            msg=rec.ros_msg,
            log_time_ns=rec.log_time_ns,
        ))

    buffered.sort(key=lambda m: (m.stamp_ns, m.topic))
    yield from buffered


def _extract_header_stamp_ns(rec: McapROS2Message) -> int | None:
    """`header.stamp` 를 epoch ns 로 합쳐서 반환한다. 필드가 없으면 None."""
    msg = rec.ros_msg
    header = getattr(msg, 'header', None)
    if header is None:
        return None
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return None
    sec = getattr(stamp, 'sec', None)
    nsec = getattr(stamp, 'nanosec', None)
    if sec is None or nsec is None:
        return None
    return int(sec) * 1_000_000_000 + int(nsec)


__all__ = ['TimedMessage', 'iter_split_messages']
