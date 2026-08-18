"""gripper_action_states → rdfp_msgs/msg/GripperActionState."""

from __future__ import annotations

from typing import Any

from .base import ReaderBase


class GripperActionStateReader(ReaderBase):
    """gripper_action_states 테이블 행을 GripperActionState 메시지로 복원한다."""

    select_cols = ('stamp_sec', 'stamp_nanosec', 'position', 'effort', 'stalled',
                   'reached_goal', 'status')

    @classmethod
    def build(cls, row: tuple[Any, ...]) -> Any:
        from rdfp_msgs.msg import GripperActionState

        sec, nsec, position, effort, stalled, reached_goal, status = row
        msg = GripperActionState()
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nsec)
        msg.position = float(position)
        msg.effort = float(effort)
        msg.stalled = bool(stalled)
        msg.reached_goal = bool(reached_goal)
        msg.status = int(status)
        return msg


__all__ = ['GripperActionStateReader']
