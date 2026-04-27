"""joint_states → sensor_msgs/msg/JointState."""

from __future__ import annotations

from typing import Any

from .base import ReaderBase


class JointStateReader(ReaderBase):
    """joint_states 테이블 행을 JointState 메시지로 복원한다."""

    select_cols = ('stamp_sec', 'stamp_nanosec', 'position', 'velocity', 'effort')

    @classmethod
    def build(cls, row: tuple[Any, ...]) -> Any:
        from sensor_msgs.msg import JointState

        sec, nsec, position, velocity, effort = row
        msg = JointState()
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nsec)
        msg.position = [float(x) for x in (position or [])]
        msg.velocity = [float(x) for x in (velocity or [])]
        msg.effort = [float(x) for x in (effort or [])]
        return msg


__all__ = ['JointStateReader']
