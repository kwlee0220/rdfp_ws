"""rdfp_msgs/msg/MotionEvent → motion_events."""

from __future__ import annotations

from typing import Any, Optional

from .base import WriterBase, extract_stamp

#: `target_kind` 가 이 값일 때만 pose 컬럼을 채운다.
_TARGET_KIND_POSE = 'pose'


class MotionEventWriter(WriterBase):
    """연산 이벤트를 motion_events 테이블에 적재한다.

    ⚠️ `stamp` 는 **레이블 구간의 끝이 아니다** — 결과를 받은 시각이라 goal tolerance
    확인과 팔의 물리 지연이 들어 있다. 구간 끝은 짝 궤적의 t₀ 에 `truncated_at_sec` 을
    더해 얻는다.

    `result` 어휘를 여기서 검사하지 않는다. rosbag 은 적재 뒤 지우므로, 예상 못 한 값
    하나가 에피소드 트랜잭션을 되돌리면 그 데이터를 되살릴 수 없다 — 어휘는 발행부가
    지키고 적재는 사실대로 담는다.
    """

    table = 'motion_events'
    columns = (
        'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec',
        'operation_id', 'operation', 'result', 'reason', 'truncated_at_sec',
        'target_kind', 'target_joint_names', 'target_joint_values',
        'target_position', 'target_orientation', 'target_frame',
    )

    def row_values(self, episode_id: int, msg: Any) -> tuple[Any, ...]:
        sec, nsec = extract_stamp(msg)
        cut = msg.truncated_at
        kind = str(msg.target_kind)
        position, orientation = _pose_arrays(msg) if kind == _TARGET_KIND_POSE else (None, None)
        return (
            episode_id, self.topic_id, sec, nsec,
            str(msg.operation_id), str(msg.operation), str(msg.result), str(msg.reason),
            cut.sec + cut.nanosec / 1_000_000_000,
            kind,
            [str(n) for n in (msg.target_joint_names or [])],
            [float(v) for v in (msg.target_joint_values or [])],
            position, orientation, str(msg.target_frame),
        )


def _pose_arrays(msg: Any) -> tuple[Optional[list], Optional[list]]:
    """데카르트 목표를 `[x, y, z]` 와 `[x, y, z, w]` 로 편다.

    orientation 은 **ROS 규약 xyzw 순서**다. 바꾸면 4개 float 에 unit norm 이라 어떤
    검사도 통과하면서 '그럴듯하게 틀린 자세'가 된다 (scene_objects 와 같은 함정).
    """
    p, o = msg.target_pose.position, msg.target_pose.orientation
    return ([float(p.x), float(p.y), float(p.z)],
            [float(o.x), float(o.y), float(o.z), float(o.w)])


__all__ = ['MotionEventWriter']
