"""rdfp_msgs/msg/GripperCommand → gripper_cmds."""

from __future__ import annotations

from typing import Any

from .base import WriterBase, extract_stamp


class GripperCommandWriter(WriterBase):
    """GripperCommand 메시지를 gripper_cmds 테이블에 적재한다."""

    table = 'gripper_cmds'
    columns = ('episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec', 'goal')

    def row_values(self, episode_id: int, msg: Any) -> tuple[Any, ...]:
        sec, nsec = extract_stamp(msg)
        # 구형 bag 은 position/max_effort 를 갖고 label 이 비어 있을 수 있다.
        # 그 경우도 적재는 멈추지 않게 두되, 빈 label 은 그대로 남긴다 —
        # 숫자를 되살려 추측하지 않는다.
        return (episode_id, self.topic_id, sec, nsec, str(getattr(msg, 'label', '') or ''))


__all__ = ['GripperCommandWriter']
