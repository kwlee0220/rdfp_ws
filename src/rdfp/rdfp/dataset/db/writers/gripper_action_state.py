"""rdfp_msgs/msg/GripperActionState → gripper_action_states."""

from __future__ import annotations

from typing import Any

from .base import WriterBase, extract_stamp


class GripperActionStateWriter(WriterBase):
    """GripperActionState 메시지를 gripper_action_states 테이블에 적재한다."""

    table = 'gripper_action_states'
    columns = (
        'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec',
        'position', 'effort', 'stalled', 'reached_goal', 'status',
    )

    def row_values(self, episode_id: int, msg: Any) -> tuple[Any, ...]:
        sec, nsec = extract_stamp(msg)
        return (
            episode_id, self.topic_id, sec, nsec,
            float(msg.position),
            float(msg.effort),
            bool(msg.stalled),
            bool(msg.reached_goal),
            # status 도입 이전에 녹화된 bag 은 필드가 없으므로 UNKNOWN(0) 으로 적재한다.
            int(getattr(msg, 'status', 0)),
        )


__all__ = ['GripperActionStateWriter']
