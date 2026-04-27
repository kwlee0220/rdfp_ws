"""rdfp_msgs/msg/GripperCommand → gripper_cmds."""

from __future__ import annotations

from typing import Any

from .base import WriterBase, extract_stamp


class GripperCommandWriter(WriterBase):
    """GripperCommand 메시지를 gripper_cmds 테이블에 적재한다."""

    table = 'gripper_cmds'
    columns = (
        'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec', 'command',
    )

    def row_values(self, episode_id: int, msg: Any) -> tuple[Any, ...]:
        sec, nsec = extract_stamp(msg)
        return (
            episode_id, self.topic_id, sec, nsec,
            str(msg.command),
        )


__all__ = ['GripperCommandWriter']
