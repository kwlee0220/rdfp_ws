"""geometry_msgs/msg/TwistStamped → twist_stampeds."""

from __future__ import annotations

from typing import Any

from .base import WriterBase, extract_stamp


class TwistStampedWriter(WriterBase):
    """TwistStamped 메시지를 twist_stampeds 테이블에 적재한다."""

    table = 'twist_stampeds'
    columns = ('episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec', 'twist')

    def row_values(self, episode_id: int, msg: Any) -> tuple[Any, ...]:
        sec, nsec = extract_stamp(msg)
        lin = msg.twist.linear
        ang = msg.twist.angular
        return (
            episode_id, self.topic_id, sec, nsec,
            [
                float(lin.x), float(lin.y), float(lin.z),
                float(ang.x), float(ang.y), float(ang.z),
            ],
        )


__all__ = ['TwistStampedWriter']
