"""sensor_msgs/msg/Image 의 mp4 sidecar → image_frames.

`Mp4ImageRecorder` 가 mp4 파일에 저장된 프레임의 ``frame_index`` /
``stamp`` 메타정보를 ``image_frames`` 테이블에 적재할 때 사용한다.

다른 writer 와 달리 ``frame_index`` 는 메시지 자체에 포함되지 않고
recorder 가 별도로 부여하는 값이므로, ``WriterBase.append()`` 대신
``append_frame(episode_id, msg, frame_index)`` 를 사용하도록 강제한다.
"""

from __future__ import annotations

from typing import Any

from .base import WriterBase, extract_stamp


class ImageFrameWriter(WriterBase):
    """이미지 메시지의 stamp 와 frame_index 를 image_frames 테이블에 적재한다.

    UNIQUE (episode_id, topic_id, frame_index) 제약을 두므로 동일 episode/topic
    에 대해 frame_index 를 0 부터 단조 증가시키는 것은 호출자(recorder) 의 책임이다.
    """

    table = 'image_frames'
    columns = (
        'episode_id', 'topic_id', 'frame_index', 'stamp_sec', 'stamp_nanosec',
    )

    def append_frame(self, episode_id: int, msg: Any, frame_index: int) -> None:
        """이미지 메시지와 frame_index 를 받아 한 행을 버퍼에 추가한다.

        Args:
            episode_id: ``sessions.id`` FK.
            msg: ``sensor_msgs/msg/Image`` (혹은 ``header.stamp`` 를 가진 메시지).
            frame_index: mp4 파일 내 0-기반 프레임 인덱스. 호출자가 단조 증가
                시켜야 한다.
        """
        sec, nsec = extract_stamp(msg)
        self._buffer.append((episode_id, self.topic_id, frame_index, sec, nsec))
        if len(self._buffer) >= self._batch_size:
            self.flush()

    def row_values(self, episode_id: int, msg: Any) -> tuple[Any, ...]:
        """``WriterBase`` 호환을 위해 정의되지만 사용을 금지한다.

        ``frame_index`` 가 메시지에 포함되지 않으므로 ``append_frame()`` 을
        사용해야 한다.
        """
        raise NotImplementedError(
            'ImageFrameWriter requires explicit frame_index; use append_frame() instead'
        )


__all__ = ['ImageFrameWriter']
