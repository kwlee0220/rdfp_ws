"""twist_stampeds → geometry_msgs/msg/TwistStamped."""

from __future__ import annotations

from typing import Any

from .base import ReaderBase


class TwistStampedReader(ReaderBase):
    """twist_stampeds 테이블 행을 TwistStamped 메시지로 복원한다."""

    select_cols = ('stamp_sec', 'stamp_nanosec', 'twist')

    @classmethod
    def build(cls, row: tuple[Any, ...]) -> Any:
        from geometry_msgs.msg import TwistStamped

        sec, nsec, twist = row
        msg = TwistStamped()
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nsec)
        msg.twist.linear.x = float(twist[0])
        msg.twist.linear.y = float(twist[1])
        msg.twist.linear.z = float(twist[2])
        msg.twist.angular.x = float(twist[3])
        msg.twist.angular.y = float(twist[4])
        msg.twist.angular.z = float(twist[5])
        return msg


__all__ = ['TwistStampedReader']
