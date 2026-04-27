"""gripper_states → rdfp_msgs/msg/GripperState."""

from __future__ import annotations

from typing import Any

from .base import ReaderBase


class GripperStateReader(ReaderBase):
    """gripper_states 테이블 행을 GripperState 메시지로 복원한다."""

    select_cols = ('stamp_sec', 'stamp_nanosec', 'position', 'effort', 'stalled', 'reached_goal')

    @classmethod
    def build(cls, row: tuple[Any, ...]) -> Any:
        from rdfp_msgs.msg import GripperState

        sec, nsec, position, effort, stalled, reached_goal = row
        msg = GripperState()
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nsec)
        msg.position = float(position)
        msg.effort = float(effort)
        msg.stalled = bool(stalled)
        msg.reached_goal = bool(reached_goal)
        return msg


__all__ = ['GripperStateReader']
