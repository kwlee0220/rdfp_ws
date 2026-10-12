"""motion_trajectories → rdfp_msgs/msg/MotionTrajectory."""

from __future__ import annotations

from typing import Any

from .base import ReaderBase


class MotionTrajectoryReader(ReaderBase):
    """motion_trajectories 행을 MotionTrajectory 메시지로 복원한다.

    `header.stamp` 는 **t₀**(궤적의 0초)이고, 안쪽 `trajectory.header` 는 비운다 —
    시각을 두 곳에 두면 어느 쪽이 t₀ 인지 모호해진다(메시지 계약).
    """

    select_cols = ('stamp_sec', 'stamp_nanosec', 'operation_id',
                   'velocity_scaling', 'time_scaling', 'joint_names', 'points')

    @classmethod
    def build(cls, row: tuple[Any, ...]) -> Any:
        from rdfp_msgs.msg import MotionTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint

        sec, nsec, operation_id, vs, ts, joint_names, points = row
        msg = MotionTrajectory()
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nsec)
        msg.header.frame_id = ''
        msg.operation_id = str(operation_id or '')
        msg.velocity_scaling = float(vs)
        msg.time_scaling = float(ts)
        msg.trajectory.joint_names = [str(n) for n in (joint_names or [])]
        msg.trajectory.points = [_build_point(JointTrajectoryPoint, p)
                                 for p in (points or [])]
        return msg


def _build_point(point_cls: type, item: dict[str, Any]) -> Any:
    """jsonb 원소 하나를 경유점으로 복원한다.

    빈 속도·가속도는 **빈 채로 둔다** — 0 으로 채우면 "그 점에서 멈춰 있으라"는 뜻이
    되어 보간 결과가 실제 궤적과 달라진다. 누락 키는 기본값으로 둬 과거 행도 복원된다.
    """
    p = point_cls()
    p.positions = [float(v) for v in (item.get('positions') or [])]
    p.velocities = [float(v) for v in (item.get('velocities') or [])]
    p.accelerations = [float(v) for v in (item.get('accelerations') or [])]
    total_ns = int(round(float(item.get('t', 0.0)) * 1_000_000_000))
    p.time_from_start.sec = total_ns // 1_000_000_000
    p.time_from_start.nanosec = total_ns % 1_000_000_000
    return p


__all__ = ['MotionTrajectoryReader']
