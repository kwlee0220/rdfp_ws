"""motion_events → rdfp_msgs/msg/MotionEvent."""

from __future__ import annotations

from typing import Any

from .base import ReaderBase


class MotionEventReader(ReaderBase):
    """motion_events 행을 MotionEvent 메시지로 복원한다."""

    select_cols = ('stamp_sec', 'stamp_nanosec', 'operation_id', 'operation', 'result',
                   'reason', 'truncated_at_sec', 'target_kind', 'target_joint_names',
                   'target_joint_values', 'target_position', 'target_orientation',
                   'target_frame')

    @classmethod
    def build(cls, row: tuple[Any, ...]) -> Any:
        from rdfp_msgs.msg import MotionEvent

        (sec, nsec, operation_id, operation, result, reason, cut, kind,
         joint_names, joint_values, position, orientation, frame) = row
        msg = MotionEvent()
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nsec)
        msg.header.frame_id = ''
        msg.operation_id = str(operation_id or '')
        msg.operation = str(operation or '')
        msg.result = str(result or '')
        msg.reason = str(reason or '')
        total_ns = int(round(float(cut or 0.0) * 1_000_000_000))
        msg.truncated_at.sec = total_ns // 1_000_000_000
        msg.truncated_at.nanosec = total_ns % 1_000_000_000
        msg.target_kind = str(kind or '')
        msg.target_joint_names = [str(n) for n in (joint_names or [])]
        msg.target_joint_values = [float(v) for v in (joint_values or [])]
        msg.target_frame = str(frame or '')
        if position and orientation:
            # ROS 규약 xyzw 로 저장돼 있다 — 순서를 바꾸면 조용히 틀린 자세가 된다.
            msg.target_pose.position.x = float(position[0])
            msg.target_pose.position.y = float(position[1])
            msg.target_pose.position.z = float(position[2])
            msg.target_pose.orientation.x = float(orientation[0])
            msg.target_pose.orientation.y = float(orientation[1])
            msg.target_pose.orientation.z = float(orientation[2])
            msg.target_pose.orientation.w = float(orientation[3])
        return msg


__all__ = ['MotionEventReader']
