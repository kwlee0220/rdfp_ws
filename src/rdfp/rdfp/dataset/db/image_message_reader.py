"""Image 토픽 replay 를 위한 mp4 + DBMS sidecar reader (lazy iteration).

`image_streams` (글로벌 메타) + `image_frames` (프레임별 stamp) + mp4 파일을
조합하여 ``sensor_msgs/Image`` 메시지를 한 프레임씩 lazily 복원하는
``ReplayStream`` 을 반환한다. mp4 의 N번째 프레임은
``image_frames.frame_index = N`` 행과 1:1 매칭된다.

색공간 처리:
    mp4 컨테이너는 yuv420p 인코딩이라 OpenCV 디코딩 결과는 항상 BGR 이다.
    본 reader 는 ``image_streams.pixel_format`` 에 맞춰 형식만 복원하며,
    원본이 4채널 (bgra8/rgba8) 이었던 경우 알파 채널은 0xFF 로 채운다
    (mp4 인코딩 단계에서 알파가 손실됐으므로).

메모리:
    프레임 메타 (frame_index, stamp) 는 SQL 결과만큼 메모리에 적재한다
    (행당 24바이트 수준). mp4 픽셀 데이터는 ``next()`` 호출 시점에 한
    프레임씩 디코딩하므로 시점당 메모리는 (frame size × 1) 수준.

주의:
    본 reader 는 publish 타이밍 (sleep 등) 을 다루지 않는다. 시간 동기화된
    Image 시퀀스가 필요하면 ``Mp4ImageReplayer`` 를 사용한다.
"""

from __future__ import annotations

from typing import Final, Iterator

import logging
import queue
import threading
from pathlib import Path

import psycopg

from sensor_msgs.msg import Image

from rdfp.dataset.types import ReplayStream

from ._image_helpers import (
    SUPPORTED_PIXEL_FORMATS,
    build_image_metadata,
    decode_one_frame,
    lookup_topic_id,
    read_image_frames_meta,
    read_image_stream_meta,
)


_logger = logging.getLogger(__name__)

# 큐 종료 신호 (None 이 아닌 unique sentinel).
_QUEUE_END: Final[object] = object()

# Prefetch 큐 깊이. 너무 크면 메모리 압박, 너무 작으면 디코더 스레드 backpressure.
# 4 ≈ 10MB @ 1080p — 디코드 30~50ms 지터를 흡수하기에 충분.
_PREFETCH_QUEUE_SIZE: Final[int] = 4


def open_image_replay_source(conn: psycopg.Connection, episode_id: int,
                             topic_name: str,
                             mp4_root: Path) -> ReplayStream | None:
    """지정 에피소드/토픽의 mp4 파일을 lazy ``ReplayStream`` 으로 연다.

    image_streams + image_frames 메타는 즉시 읽지만 mp4 디코딩은 ``iterator``
    의 ``next()`` 호출 시점에 한 프레임씩 수행되어 메모리 사용량을 O(1) 로
    유지한다. ``ReplayStream.close()`` 는 ``cv2.VideoCapture`` 를 해제한다.

    Args:
        conn: 열려 있는 PostgreSQL 커넥션.
        episode_id: ``sessions.id``.
        topic_name: 조회 대상 이미지 토픽 이름.
        mp4_root: ``image_streams.mp4_path`` 의 기준 루트.

    Returns:
        ``ReplayStream`` 인스턴스. 해당 에피소드에 image_frames row 가 없으면
        ``None``.

    Raises:
        ValueError: topic 이 ``topics`` 에 없거나, ``image_streams`` 행이 없거나,
            지원하지 않는 ``pixel_format`` 인 경우.
        RuntimeError: mp4 파일을 열 수 없거나 첫 프레임 디코딩에 실패한 경우.
    """
    import cv2   # lazy import: cv2 미설치 환경에서도 모듈 import 자체는 가능

    topic_id = lookup_topic_id(conn, topic_name)
    stream = read_image_stream_meta(conn, episode_id, topic_id)
    frames_meta = read_image_frames_meta(conn, episode_id, topic_id)
    if not frames_meta:
        return None

    pixel_format = stream['pixel_format']
    if pixel_format not in SUPPORTED_PIXEL_FORMATS:
        raise ValueError(
            f'unsupported pixel_format in image_streams: {pixel_format!r}; '
            f'expected one of {sorted(SUPPORTED_PIXEL_FORMATS)}'
        )

    mp4_path = Path(mp4_root) / stream['mp4_path']
    if not mp4_path.is_file():
        raise RuntimeError(f'mp4 file not found: {mp4_path}')

    cap = cv2.VideoCapture(str(mp4_path))
    if not cap.isOpened():
        raise RuntimeError(f'failed to open mp4: {mp4_path}')

    metadata = build_image_metadata(stream)

    # 첫 프레임을 즉시 디코딩하여 peek 용으로 확보한다. 실패 시 capture 해제 후 raise.
    first_msg = decode_one_frame(cap, frames_meta[0], pixel_format, metadata)
    if first_msg is None:
        cap.release()
        raise RuntimeError(
            f'failed to decode first frame of mp4: {mp4_path} '
            f'(image_frames had {len(frames_meta)} row(s))'
        )

    # 나머지 프레임은 백그라운드 스레드가 prefetch 한다. 이렇게 하면 publish 루프가
    # cv2.read() / cvtColor 의 블로킹 비용 (10~50ms/frame) 을 떠안지 않아, 다중
    # 토픽 replay 시 비-image 토픽의 발행 타이밍이 image 디코딩에 의해 burst 가 되지
    # 않는다. 이는 MoveIt Servo 처럼 ``incoming_command_timeout`` 을 가진 소비자가
    # gap 으로 인해 halt 되는 문제를 방지한다.
    frame_q: queue.Queue = queue.Queue(maxsize=_PREFETCH_QUEUE_SIZE)
    stop_flag = threading.Event()

    def decoder_loop() -> None:
        try:
            for fm in frames_meta[1:]:
                if stop_flag.is_set():
                    break
                msg = decode_one_frame(cap, fm, pixel_format, metadata)
                if msg is None:
                    # mp4 가 image_frames row 수보다 짧으면 남은 stamp 는 폐기.
                    # (recorder 가 1:1 매치를 보장하므로 정상 경로에서는 발생하지 않음)
                    break
                # 큐가 가득 차면 backpressure: stop_flag 폴링하면서 대기.
                while not stop_flag.is_set():
                    try:
                        frame_q.put(msg, timeout=0.1)
                        break
                    except queue.Full:
                        continue
        except Exception:   # noqa: BLE001
            _logger.exception('image decoder thread crashed')
        finally:
            # 정상/비정상 종료 모두 sentinel 을 넣어 consumer 가 빠져나오게 한다.
            try:
                frame_q.put(_QUEUE_END, timeout=1.0)
            except queue.Full:
                pass

    decoder_thread = threading.Thread(
        target=decoder_loop, name=f'image-decoder-{topic_name}', daemon=True,
    )
    decoder_thread.start()

    closed_flag: list[bool] = [False]

    def close() -> None:
        if closed_flag[0]:
            return
        closed_flag[0] = True
        stop_flag.set()
        # 디코더 스레드가 frame_q.put 에 block 중이면 한 번 비워서 깨운다.
        try:
            while True:
                frame_q.get_nowait()
        except queue.Empty:
            pass
        decoder_thread.join(timeout=2.0)
        cap.release()

    def iterator() -> Iterator[Image]:
        try:
            yield first_msg
            while True:
                msg = frame_q.get()
                if msg is _QUEUE_END:
                    break
                yield msg
        finally:
            close()

    return ReplayStream(
        topic_name=topic_name, first_message=first_msg,
        iterator=iterator(), close=close,
        expected_count=len(frames_meta),
    )


def is_image_topic_in_db(conn: psycopg.Connection, topic_name: str) -> bool:
    """`topics` 테이블에서 ``topic_name`` 의 타입을 조회해 Image 여부를 반환한다.

    토픽이 존재하지 않으면 False 를 반환한다 (본 함수는 falsy 분기에 사용되며,
    실제 read 호출이 ValueError 로 별도 보고한다).
    """
    with conn.cursor() as cur:
        cur.execute('SELECT topic_type FROM topics WHERE topic_name = %s', (topic_name,))
        row = cur.fetchone()
    if row is None:
        return False
    return str(row[0]) == 'sensor_msgs/msg/Image'


__all__ = ['open_image_replay_source', 'is_image_topic_in_db']
