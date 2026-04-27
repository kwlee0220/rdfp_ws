"""geometry_msgs/msg/PoseStamped → pose_stampeds."""

from __future__ import annotations

from typing import Any

from .base import WriterBase, extract_stamp


class PoseStampedWriter(WriterBase):
    """PoseStamped 메시지를 pose_stampeds 테이블에 적재한다."""

    table = 'pose_stampeds'
    columns = (
        'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec',
        'position', 'orientation',
    )

    def row_values(self, episode_id: int, msg: Any) -> tuple[Any, ...]:
        sec, nsec = extract_stamp(msg)
        p = msg.pose.position
        o = msg.pose.orientation
        return (
            episode_id, self.topic_id, sec, nsec,
            [float(p.x), float(p.y), float(p.z)],
            [float(o.x), float(o.y), float(o.z), float(o.w)],
        )


__all__ = ['PoseStampedWriter']
