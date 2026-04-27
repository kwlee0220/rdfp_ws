"""rdfp_msgs/msg/GripperState → gripper_states."""

from __future__ import annotations

from typing import Any

from .base import WriterBase, extract_stamp


class GripperStateWriter(WriterBase):
    """GripperState 메시지를 gripper_states 테이블에 적재한다."""

    table = 'gripper_states'
    columns = (
        'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec',
        'position', 'effort', 'stalled', 'reached_goal',
    )

    def row_values(self, episode_id: int, msg: Any) -> tuple[Any, ...]:
        sec, nsec = extract_stamp(msg)
        return (
            episode_id, self.topic_id, sec, nsec,
            float(msg.position),
            float(msg.effort),
            bool(msg.stalled),
            bool(msg.reached_goal),
        )


__all__ = ['GripperStateWriter']
