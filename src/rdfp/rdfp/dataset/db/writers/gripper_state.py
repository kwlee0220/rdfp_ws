"""rdfp_msgs/msg/GripperState → gripper_states."""

from __future__ import annotations

from typing import Any

from .base import WriterBase, extract_stamp


class GripperStateWriter(WriterBase):
    """GripperState 메시지를 gripper_states 테이블에 적재한다."""

    table = 'gripper_states'
    columns = (
        'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec',
        'goal', 'width', 'stalled', 'at_goal',
    )

    def row_values(self, episode_id: int, msg: Any) -> tuple[Any, ...]:
        sec, nsec = extract_stamp(msg)
        # NaN 을 0 으로 바꾸지 않는다 — '모른다'와 '닫혀 있다'는 다르다.
        return (
            episode_id, self.topic_id, sec, nsec,
            str(msg.goal or ''), float(msg.width),
            bool(msg.stalled), bool(msg.at_goal),
        )


__all__ = ['GripperStateWriter']
