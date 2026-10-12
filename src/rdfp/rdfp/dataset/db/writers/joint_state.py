"""sensor_msgs/msg/JointState → joint_states."""

from __future__ import annotations

from typing import Any

from .base import WriterBase, extract_stamp


class JointStateWriter(WriterBase):
    """JointState 메시지를 joint_states 테이블에 적재한다.

    `position`/`velocity`/`effort` 중 일부가 빈 배열이어도 그대로 저장한다.
    DB 스키마는 `NOT NULL` 이지만 빈 배열 (`'{}'::DOUBLE PRECISION[]`) 을 허용한다.

    **`name` 을 함께 남긴다** (2026-09-18) — 배열 순서 규약만으로는 어느 칸이 어느
    관절인지 알 수 없고, rosbag 은 적재 뒤 지우므로 버리면 되살릴 수 없다.
    발행자가 안 채웠으면 **빈 채로** 둔다 — 그것도 사실이고, export 가 그 에피소드를
    걸러낼 근거가 된다.
    """

    table = 'joint_states'
    columns = (
        'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec',
        'joint_names', 'position', 'velocity', 'effort',
    )

    def row_values(self, episode_id: int, msg: Any) -> tuple[Any, ...]:
        sec, nsec = extract_stamp(msg)
        return (
            episode_id, self.topic_id, sec, nsec,
            [str(n) for n in (msg.name or [])],
            [float(x) for x in (msg.position or [])],
            [float(x) for x in (msg.velocity or [])],
            [float(x) for x in (msg.effort or [])],
        )


__all__ = ['JointStateWriter']
