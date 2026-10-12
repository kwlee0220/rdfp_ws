"""joint_states → sensor_msgs/msg/JointState."""

from __future__ import annotations

from typing import Any

from .base import ReaderBase


class JointStateReader(ReaderBase):
    """joint_states 테이블 행을 JointState 메시지로 복원한다."""

    select_cols = ('stamp_sec', 'stamp_nanosec', 'joint_names',
                   'position', 'velocity', 'effort')

    @classmethod
    def build(cls, row: tuple[Any, ...]) -> Any:
        from sensor_msgs.msg import JointState

        sec, nsec, names, position, velocity, effort = row
        msg = JointState()
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nsec)
        # **이름을 되살린다** — 예전에는 버려서 재생된 메시지의 `name` 이 늘 비었고,
        # `target_joint_cmds_executor` 가 파라미터 폴백으로 때워야 했다.
        msg.name = [str(n) for n in (names or [])]
        msg.position = [float(x) for x in (position or [])]
        msg.velocity = [float(x) for x in (velocity or [])]
        msg.effort = [float(x) for x in (effort or [])]
        return msg


__all__ = ['JointStateReader']
