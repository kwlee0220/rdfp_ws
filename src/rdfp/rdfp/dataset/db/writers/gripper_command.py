"""rdfp_msgs/msg/GripperCommand → gripper_cmds."""

from __future__ import annotations

from typing import Any

from .base import WriterBase, extract_stamp


class GripperCommandWriter(WriterBase):
    """GripperCommand 메시지를 gripper_cmds 테이블에 적재한다."""

    table = 'gripper_cmds'
    columns = (
        'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec',
        'position', 'max_effort', 'label',
    )

    def row_values(self, episode_id: int, msg: Any) -> tuple[Any, ...]:
        sec, nsec = extract_stamp(msg)
        # label 은 선택 필드다. 없는 구현을 만나도 적재가 멈추지 않게 방어한다.
        return (
            episode_id, self.topic_id, sec, nsec,
            float(msg.position), float(msg.max_effort), str(getattr(msg, 'label', '') or ''),
        )


__all__ = ['GripperCommandWriter']
