"""rdfp_msgs/msg/MotionTrajectory → motion_trajectories."""

from __future__ import annotations

from typing import Any

from psycopg.types.json import Jsonb

from .base import WriterBase, extract_stamp


class MotionTrajectoryWriter(WriterBase):
    """실행 궤적(L2)을 motion_trajectories 테이블에 적재한다.

    **연산마다 한 행이고 궤적 전체가 그 한 행에 들어간다.** 경유점마다 행을 나누지
    않는 이유는 scene_objects 와 같다 — reader 계약이 row 1개 → 메시지 1개다.

    ⚠️ `stamp` 는 발행 시각이 아니라 **궤적의 0초가 시작된 시각(t₀)** 이다. export 가
    `t₀ + points[i].t` 로 경유점의 절대 시각을 만들므로, 이 값이 틀리면 학습 레이블이
    통째로 밀린다.
    """

    table = 'motion_trajectories'
    columns = (
        'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec',
        'operation_id', 'velocity_scaling', 'time_scaling', 'joint_names', 'points',
    )

    def row_values(self, episode_id: int, msg: Any) -> tuple[Any, ...]:
        sec, nsec = extract_stamp(msg)
        jt = msg.trajectory
        return (
            episode_id, self.topic_id, sec, nsec,
            str(msg.operation_id),
            float(msg.velocity_scaling), float(msg.time_scaling),
            [str(n) for n in (jt.joint_names or [])],
            # dict/list 를 그대로 넘기면 psycopg 가 어느 타입으로 보낼지 모른다.
            Jsonb([_point_to_dict(p) for p in (jt.points or [])]),
        )


def _point_to_dict(point: Any) -> dict[str, Any]:
    """경유점 하나를 jsonb 원소로 바꾼다.

    `t` 는 **궤적 시작으로부터의 경과 초**(`time_from_start`)다. 속도·가속도는 계획
    방식에 따라 비어 있을 수 있어 **빈 배열을 그대로 둔다** — 0 으로 채우면 "정지해
    있으라"는 뜻이 되어 3차 Hermite 보간이 실제와 다른 곡선을 만든다.
    """
    d = point.time_from_start
    return {
        't': d.sec + d.nanosec / 1_000_000_000,
        'positions': [float(v) for v in (point.positions or [])],
        'velocities': [float(v) for v in (point.velocities or [])],
        'accelerations': [float(v) for v in (point.accelerations or [])],
    }


__all__ = ['MotionTrajectoryWriter']
