"""gripper_cmds → rdfp_msgs/msg/GripperCommand."""

from __future__ import annotations

from typing import Any

from .base import ReaderBase


class GripperCommandReader(ReaderBase):
    """gripper_cmds 테이블 행을 GripperCommand 메시지로 복원한다."""

    select_cols = ('stamp_sec', 'stamp_nanosec', 'command')

    @classmethod
    def build(cls, row: tuple[Any, ...]) -> Any:
        from rdfp_msgs.msg import GripperCommand

        sec, nsec, command = row
        msg = GripperCommand()
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nsec)
        msg.command = str(command)
        return msg


__all__ = ['GripperCommandReader']
