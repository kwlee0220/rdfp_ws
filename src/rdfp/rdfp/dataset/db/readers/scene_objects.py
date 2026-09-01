"""scene_objects → rdfp_msgs/msg/SceneObjects."""

from __future__ import annotations

from typing import Any

from .base import ReaderBase


class SceneObjectsReader(ReaderBase):
    """scene_objects 테이블 행을 SceneObjects 메시지로 복원한다.

    다른 reader 와 달리 `frame_id` 를 복원한다 — 이 토픽은 좌표 기준 프레임이
    값의 의미를 좌우하므로 DB 에도 저장되어 있다.
    """

    select_cols = ('stamp_sec', 'stamp_nanosec', 'frame_id', 'objects')

    @classmethod
    def build(cls, row: tuple[Any, ...]) -> Any:
        from rdfp_msgs.msg import SceneObject, SceneObjects

        sec, nsec, frame_id, objects = row
        msg = SceneObjects()
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nsec)
        msg.header.frame_id = str(frame_id or '')
        msg.objects = [_build_object(SceneObject, o) for o in (objects or [])]
        return msg


def _build_object(scene_object_cls: type, item: dict[str, Any]) -> Any:
    """jsonb 원소 하나를 SceneObject 로 복원한다.

    누락 키는 메시지 기본값으로 둔다 — 적재 시점보다 필드가 늘어난 경우
    (과거 행) 복원 자체가 실패하지 않게 한다.
    """
    obj = scene_object_cls()
    obj.name = str(item.get('name', ''))
    obj.type = str(item.get('type', ''))
    obj.dimensions = [float(d) for d in (item.get('dimensions') or [])]
    position = item.get('position') or [0.0, 0.0, 0.0]
    orientation = item.get('orientation') or [0.0, 0.0, 0.0, 1.0]
    obj.pose.position.x = float(position[0])
    obj.pose.position.y = float(position[1])
    obj.pose.position.z = float(position[2])
    obj.pose.orientation.x = float(orientation[0])
    obj.pose.orientation.y = float(orientation[1])
    obj.pose.orientation.z = float(orientation[2])
    obj.pose.orientation.w = float(orientation[3])
    # `fixture` 도입 이전에 적재된 행에는 이 키가 없다. 기본값 false 는
    # '조작 대상' 이라 planning scene 이 비는 쪽으로 틀리며, 그 편이 즉시 드러난다.
    obj.fixture = bool(item.get('fixture', False))
    return obj


__all__ = ['SceneObjectsReader']
