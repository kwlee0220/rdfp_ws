"""pose_stampeds → geometry_msgs/msg/PoseStamped."""

from __future__ import annotations

from typing import Any

from .base import ReaderBase


class PoseStampedReader(ReaderBase):
    """pose_stampeds 테이블 행을 PoseStamped 메시지로 복원한다."""

    select_cols = ('stamp_sec', 'stamp_nanosec', 'position', 'orientation')

    @classmethod
    def build(cls, row: tuple[Any, ...]) -> Any:
        from geometry_msgs.msg import PoseStamped

        sec, nsec, position, orientation = row
        msg = PoseStamped()
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nsec)
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.x = float(orientation[0])
        msg.pose.orientation.y = float(orientation[1])
        msg.pose.orientation.z = float(orientation[2])
        msg.pose.orientation.w = float(orientation[3])
        return msg


__all__ = ['PoseStampedReader']
