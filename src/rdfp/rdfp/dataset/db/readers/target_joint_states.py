"""target_joint_states → rdfp_msgs/msg/TargetJointStates."""

from __future__ import annotations

from typing import Any

from .base import ReaderBase


class TargetJointStatesReader(ReaderBase):
    """target_joint_states 테이블 행을 TargetJointStates 메시지로 복원한다."""

    select_cols = (
        'stamp_sec', 'stamp_nanosec',
        'positions', 'velocities', 'accelerations', 'effort',
        'tfs_sec', 'tfs_nanosec',
    )

    @classmethod
    def build(cls, row: tuple[Any, ...]) -> Any:
        from rdfp_msgs.msg import TargetJointStates

        (sec, nsec, positions, velocities, accelerations, effort,
         tfs_sec, tfs_nsec) = row
        msg = TargetJointStates()
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nsec)
        msg.point.positions = [float(x) for x in (positions or [])]
        msg.point.velocities = [float(x) for x in (velocities or [])]
        msg.point.accelerations = [float(x) for x in (accelerations or [])]
        msg.point.effort = [float(x) for x in (effort or [])]
        msg.point.time_from_start.sec = int(tfs_sec)
        msg.point.time_from_start.nanosec = int(tfs_nsec)
        return msg


__all__ = ['TargetJointStatesReader']
