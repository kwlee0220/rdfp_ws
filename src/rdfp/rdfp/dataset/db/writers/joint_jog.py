"""control_msgs/msg/JointJog → joint_jogs."""

from __future__ import annotations

from typing import Any

from .base import WriterBase, extract_stamp


class JointJogWriter(WriterBase):
    """JointJog 메시지를 joint_jogs 테이블에 적재한다.

    `joint_names` 와 `displacements`/`velocities` 중 한 쪽은 비어 있을 수 있으나
    DB 스키마는 `NOT NULL` 이므로 빈 배열 (`'{}'`) 로 저장한다.
    """

    table = 'joint_jogs'
    columns = (
        'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec',
        'joint_names', 'displacements', 'velocities', 'duration',
    )

    def row_values(self, episode_id: int, msg: Any) -> tuple[Any, ...]:
        sec, nsec = extract_stamp(msg)
        return (
            episode_id, self.topic_id, sec, nsec,
            [str(x) for x in (msg.joint_names or [])],
            [float(x) for x in (msg.displacements or [])],
            [float(x) for x in (msg.velocities or [])],
            float(msg.duration),
        )


__all__ = ['JointJogWriter']
