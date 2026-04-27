"""sensor_msgs/msg/JointState → joint_states."""

from __future__ import annotations

from typing import Any

from .base import WriterBase, extract_stamp


class JointStateWriter(WriterBase):
    """JointState 메시지를 joint_states 테이블에 적재한다.

    `position`/`velocity`/`effort` 중 일부가 빈 배열이어도 그대로 저장한다.
    DB 스키마는 `NOT NULL` 이지만 빈 배열 (`'{}'::DOUBLE PRECISION[]`) 을 허용한다.
    """

    table = 'joint_states'
    columns = (
        'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec',
        'position', 'velocity', 'effort',
    )

    def row_values(self, episode_id: int, msg: Any) -> tuple[Any, ...]:
        sec, nsec = extract_stamp(msg)
        return (
            episode_id, self.topic_id, sec, nsec,
            [float(x) for x in (msg.position or [])],
            [float(x) for x in (msg.velocity or [])],
            [float(x) for x in (msg.effort or [])],
        )


__all__ = ['JointStateWriter']
