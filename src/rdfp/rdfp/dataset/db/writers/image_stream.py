"""image_streams 테이블 INSERT 헬퍼.

다른 writer 들과 달리 메시지당 한 행이 아니라 ``(episode_id, topic_id)``
조합당 한 행만 발생하므로 ``WriterBase`` 의 배치 INSERT 패턴을 사용하지
않는다. 보통 sink 의 finalize 시점에 한 번 호출된다.
"""

from __future__ import annotations

import psycopg


class ImageStreamWriter:
    """`(episode_id, topic_id)` 단위 mp4 글로벌 메타를 image_streams 에 적재한다.

    호출자가 commit / rollback 을 관리한다 (다른 writer 와 동일).
    """

    table: str = 'image_streams'
    columns: tuple[str, ...] = (
        'episode_id', 'topic_id', 'mp4_path', 'codec', 'pixel_format',
        'container_fps', 'frame_id', 'width', 'height', 'frame_count',
    )

    def __init__(self, conn: psycopg.Connection, *,
                 table: str | None = None) -> None:
        self._conn = conn
        if table is not None:
            self.table = table

    def insert(self, *, episode_id: int, topic_id: int, mp4_path: str, codec: str,
               pixel_format: str, container_fps: int, frame_id: str,
               width: int, height: int, frame_count: int) -> int:
        """글로벌 메타 한 행을 INSERT 하고 새 row id 를 반환한다.

        Args:
            episode_id: ``sessions.id`` FK.
            topic_id: ``topics.id`` FK.
            mp4_path: mp4 파일 경로 (출력 루트에 대한 상대경로 권장).
            codec: mp4 컨테이너 비디오 코덱 (예: ``'h264'``).
            pixel_format: 입력 프레임 픽셀 포맷 (예: ``'bgr8'``).
            container_fps: mp4 컨테이너 CFR.
            frame_id: ROS 메시지 ``header.frame_id`` (없으면 빈 문자열).
            width: 프레임 너비 (px).
            height: 프레임 높이 (px).
            frame_count: 인코딩된 총 프레임 수.

        Returns:
            새로 INSERT 된 ``image_streams.id``.
        """
        placeholders = ', '.join(['%s'] * len(self.columns))
        col_list = ', '.join(self.columns)
        sql = (
            f'INSERT INTO {self.table} ({col_list}) '
            f'VALUES ({placeholders}) RETURNING id'
        )
        row = (
            int(episode_id), int(topic_id), str(mp4_path), str(codec),
            str(pixel_format), int(container_fps), str(frame_id),
            int(width), int(height), int(frame_count),
        )
        with self._conn.cursor() as cur:
            cur.execute(sql, row)
            new_id = cur.fetchone()[0]
        return int(new_id)


__all__ = ['ImageStreamWriter']
