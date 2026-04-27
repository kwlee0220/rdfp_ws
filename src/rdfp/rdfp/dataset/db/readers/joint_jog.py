"""joint_jogs → control_msgs/msg/JointJog."""

from __future__ import annotations

from typing import Any

from .base import ReaderBase


class JointJogReader(ReaderBase):
    """joint_jogs 테이블 행을 JointJog 메시지로 복원한다."""

    select_cols = (
        'stamp_sec', 'stamp_nanosec',
        'joint_names', 'displacements', 'velocities', 'duration',
    )

    @classmethod
    def build(cls, row: tuple[Any, ...]) -> Any:
        from control_msgs.msg import JointJog

        sec, nsec, joint_names, displacements, velocities, duration = row
        msg = JointJog()
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nsec)
        msg.joint_names = [str(x) for x in (joint_names or [])]
        msg.displacements = [float(x) for x in (displacements or [])]
        msg.velocities = [float(x) for x in (velocities or [])]
        msg.duration = float(duration)
        return msg


__all__ = ['JointJogReader']
