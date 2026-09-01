"""rdfp_msgs/msg/SceneObjects → scene_objects."""

from __future__ import annotations

from typing import Any

from psycopg.types.json import Jsonb

from .base import WriterBase, extract_stamp


class SceneObjectsWriter(WriterBase):
    """SceneObjects 메시지를 scene_objects 테이블에 적재한다.

    한 메시지가 한 행이며 물체 배열은 `objects` jsonb 컬럼에 통째로 들어간다.
    물체마다 행을 나누지 않는 이유는 스키마 주석에 있다 (reader 계약이 row
    1개 → 메시지 1개, `dimensions` 길이가 종류마다 다름).

    `header.frame_id` 를 함께 남긴다. 다른 writer 는 frame_id 를 버리지만,
    백엔드가 world→base 변환을 빠뜨린 경우 pose 값만으로는 드러나지 않기
    때문이다.
    """

    table = 'scene_objects'
    columns = (
        'episode_id', 'topic_id', 'stamp_sec', 'stamp_nanosec',
        'frame_id', 'objects',
    )

    def row_values(self, episode_id: int, msg: Any) -> tuple[Any, ...]:
        sec, nsec = extract_stamp(msg)
        objects = [_object_to_dict(o) for o in (msg.objects or [])]
        return (
            episode_id, self.topic_id, sec, nsec,
            str(msg.header.frame_id),
            # dict/list 를 그대로 넘기면 psycopg 가 어느 타입으로 보낼지 알 수
            # 없으므로 명시적으로 jsonb 로 어댑트한다.
            Jsonb(objects),
        )


def _object_to_dict(obj: Any) -> dict[str, Any]:
    """SceneObject 하나를 jsonb 원소로 변환한다.

    orientation 은 **ROS 규약 xyzw 순서**로 담는다. 순서를 바꾸면 4개 float 에
    unit norm 이라 어떤 검사도 통과하면서 '그럴듯하게 틀린 자세'가 된다.

    `fixture` 를 함께 남긴다 — 자동 라벨링이 '블록이 탁자 위에 놓였는가'를 판정할 때
    어느 것이 지지면인지 알아야 하는데, 그것을 데이터 밖의 설정 파일에서 다시 찾으면
    파일이 바뀐 뒤 라벨이 조용히 틀린다.
    """
    pos = obj.pose.position
    ori = obj.pose.orientation
    return {
        'name': str(obj.name),
        'type': str(obj.type),
        'dimensions': [float(d) for d in (obj.dimensions or [])],
        'position': [float(pos.x), float(pos.y), float(pos.z)],
        'orientation': [float(ori.x), float(ori.y), float(ori.z), float(ori.w)],
        'fixture': bool(obj.fixture),
    }


__all__ = ['SceneObjectsWriter']
