"""rdfp_msgs/msg/TargetJointStates → target_joint_states.

JointTrajectoryPoint 단일 point 의 4개 배열(positions/velocities/accelerations/
effort) 과 time_from_start (sec/nanosec) 를 평탄화하여 한 행에 적재한다.
배열이 비어 있어도 빈 배열 그대로 저장한다 (DB 스키마는 NOT NULL 이지만 길이
0 의 ``DOUBLE PRECISION[]`` 은 허용).
"""

from __future__ import annotations

from typing import Any

from .base import WriterBase, extract_stamp


class TargetJointStatesWriter(WriterBase):
    """TargetJointStates 메시지를 target_joint_states 테이블에 적재한다."""

    table = 'target_joint_states'
    columns = (
        'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec',
        'positions', 'velocities', 'accelerations', 'effort',
        'tfs_sec', 'tfs_nanosec',
    )

    def row_values(self, episode_id: int, msg: Any) -> tuple[Any, ...]:
        sec, nsec = extract_stamp(msg)
        p = msg.point
        tfs = getattr(p, 'time_from_start', None)
        tfs_sec = int(getattr(tfs, 'sec', 0)) if tfs is not None else 0
        tfs_nsec = int(getattr(tfs, 'nanosec', 0)) if tfs is not None else 0
        return (
            episode_id, self.topic_id, sec, nsec,
            [float(x) for x in (p.positions or [])],
            [float(x) for x in (p.velocities or [])],
            [float(x) for x in (p.accelerations or [])],
            [float(x) for x in (p.effort or [])],
            tfs_sec, tfs_nsec,
        )


__all__ = ['TargetJointStatesWriter']
