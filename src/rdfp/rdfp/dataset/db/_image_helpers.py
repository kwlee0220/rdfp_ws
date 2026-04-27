"""image_streams + image_frames + mp4 reader 들이 공통으로 쓰는 helper.

``image_message_reader`` (lazy ReplayStream) 와 ``mp4_image_replayer``
(time-aware standalone) 가 모두 사용한다. 공개 API 가 아니므로
외부에서 직접 import 하지 말 것 (`_` prefix 모듈).
"""

from __future__ import annotations

from typing import Optional

import psycopg

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image

from rdfp.types import ImageMetadata, Resolution, to_ros_image_msg


SUPPORTED_PIXEL_FORMATS: frozenset[str] = frozenset({
    'bgr8', 'rgb8', 'bgra8', 'rgba8', 'mono8',
})


def lookup_topic_id(conn: psycopg.Connection, topic_name: str) -> int:
    """``topics`` 에서 ``topic_name`` 의 id 를 조회한다.

    Raises:
        ValueError: 토픽이 등록되지 않은 경우.
    """
    with conn.cursor() as cur:
        cur.execute('SELECT id FROM topics WHERE topic_name = %s', (topic_name,))
        row = cur.fetchone()
    if row is None:
        raise ValueError(f'topic name {topic_name!r} not found in topics table')
    return int(row[0])


def read_image_stream_meta(conn: psycopg.Connection, episode_id: int,
                           topic_id: int) -> dict:
    """``image_streams`` 의 한 행을 dict 로 반환한다.

    Raises:
        ValueError: 행이 없는 경우.
    """
    sql = ('SELECT mp4_path, pixel_format, frame_id, width, height, frame_count '
           'FROM image_streams WHERE episode_id = %s AND topic_id = %s')
    with conn.cursor() as cur:
        cur.execute(sql, (episode_id, topic_id))
        row = cur.fetchone()
    if row is None:
        raise ValueError(
            f'no image_streams row for episode_id={episode_id}, topic_id={topic_id}'
        )
    return {
        'mp4_path': str(row[0]), 'pixel_format': str(row[1]),
        'frame_id': str(row[2]), 'width': int(row[3]), 'height': int(row[4]),
        'frame_count': int(row[5]),
    }


def read_image_frames_meta(conn: psycopg.Connection, episode_id: int,
                           topic_id: int) -> list[dict]:
    """``image_frames`` 의 모든 행을 ``frame_index`` 오름차순 list 로 반환한다."""
    sql = ('SELECT frame_index, stamp_sec, stamp_nanosec '
           'FROM image_frames WHERE episode_id = %s AND topic_id = %s '
           'ORDER BY frame_index')
    with conn.cursor() as cur:
        cur.execute(sql, (episode_id, topic_id))
        rows = cur.fetchall()
    return [
        {'frame_index': int(r[0]), 'stamp_sec': int(r[1]), 'stamp_nanosec': int(r[2])}
        for r in rows
    ]


def channels_for(pixel_format: str) -> int:
    """픽셀 포맷의 채널 수."""
    return {
        'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1,
    }[pixel_format]


def build_image_metadata(stream: dict) -> ImageMetadata:
    """``image_streams`` 행 dict 으로부터 ``ImageMetadata`` 를 생성한다."""
    pixel_format = stream['pixel_format']
    return ImageMetadata(
        frame_id=stream['frame_id'],
        resolution=Resolution(stream['width'], stream['height']),
        encoding=pixel_format,
        is_bigendian=0,
        step=stream['width'] * channels_for(pixel_format),
    )


def stamp_to_ns(frame_meta: dict) -> int:
    """``image_frames`` row dict 의 stamp 를 정수 나노초로 변환."""
    return int(frame_meta['stamp_sec']) * 1_000_000_000 + int(frame_meta['stamp_nanosec'])


def decode_one_frame(cap, frame_meta: dict, pixel_format: str,
                     metadata: ImageMetadata) -> Optional[Image]:
    """mp4 에서 한 프레임을 읽어 ``Image`` 메시지로 복원한다.

    ``cv2.VideoCapture.read()`` 가 실패하면 ``None`` 을 반환한다 (mp4 가
    image_frames 행 수보다 짧은 경우 등).
    """
    pixels = decode_one_ndarray(cap, pixel_format)
    if pixels is None:
        return None
    return build_image_msg(metadata, frame_meta, pixels)


def decode_one_ndarray(cap, pixel_format: str):
    """mp4 에서 한 프레임만 디코딩하여 픽셀 ndarray 를 반환한다.

    Image 메시지 wrap 비용 (특히 rclpy 의 ``msg.data = bytes`` setter 비용)
    이 큰 use case 에서 메시지 생성을 분리하기 위한 lower-level 헬퍼.

    Returns:
        지정 ``pixel_format`` 에 맞춘 ``numpy.ndarray``. ``cv2.VideoCapture.read()``
        가 실패하면 ``None``.
    """
    ok, bgr = cap.read()
    if not ok:
        return None
    return convert_from_bgr(bgr, pixel_format)


def build_image_msg(metadata: ImageMetadata, frame_meta: dict,
                    pixels) -> Image:
    """``ndarray`` + (stamp 정보 dict) → ``sensor_msgs/Image`` 메시지.

    ``decode_one_ndarray`` 와 짝을 이룬다.
    """
    stamp = Time()
    stamp.sec = int(frame_meta['stamp_sec'])
    stamp.nanosec = int(frame_meta['stamp_nanosec'])
    return to_ros_image_msg(metadata, stamp, pixels)


def convert_from_bgr(bgr, pixel_format: str):
    """OpenCV BGR ndarray 를 지정 픽셀 포맷으로 변환한다.

    mp4 디코딩 결과는 항상 BGR 이므로, 4채널 픽셀 포맷의 알파는
    ``cv2.cvtColor`` 가 0xFF 로 채워준다.
    """
    import cv2

    if pixel_format == 'bgr8':
        return bgr
    if pixel_format == 'rgb8':
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    if pixel_format == 'bgra8':
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2BGRA)
    if pixel_format == 'rgba8':
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGBA)
    if pixel_format == 'mono8':
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    raise ValueError(f'unsupported pixel_format: {pixel_format!r}')


__all__ = [
    'SUPPORTED_PIXEL_FORMATS',
    'lookup_topic_id',
    'read_image_stream_meta',
    'read_image_frames_meta',
    'channels_for',
    'build_image_metadata',
    'stamp_to_ns',
    'decode_one_frame',
    'decode_one_ndarray',
    'build_image_msg',
    'convert_from_bgr',
]
