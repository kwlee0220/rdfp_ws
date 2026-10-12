"""rdfp_msgs/msg/GripperCommand → gripper_cmds."""

from __future__ import annotations

from typing import Any

from .base import WriterBase, extract_stamp


class GripperCommandWriter(WriterBase):
    """GripperCommand 메시지를 gripper_cmds 테이블에 적재한다."""

    table = 'gripper_cmds'
    columns = ('episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec', 'goal')

    def row_values(self, episode_id: int, msg: Any) -> tuple[Any, ...]:
        sec, nsec = extract_stamp(msg)
        # ⚠️ **필드 이름은 `goal` 이다.** 한때 `label` 이었고 이 writer 가 그 이름을
        # `getattr(msg, 'label', '')` 로 읽고 있었다 — **기본값이 있어 이름이 틀려도
        # 예외가 안 난다.** 그 결과 모든 그리퍼 명령이 빈 문자열로 적재됐고, 증상은
        # export 에서 one-hot 이 전부 0 이 되는 것뿐이었다(2026-09-19 실 수집에서
        # 드러났다). 구형 bag 을 위해 `label` 을 폴백으로만 남긴다.
        goal = getattr(msg, 'goal', None)
        if goal is None:
            goal = getattr(msg, 'label', '')
        return (episode_id, self.topic_id, sec, nsec, str(goal or ''))


__all__ = ['GripperCommandWriter']
