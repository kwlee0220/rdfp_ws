"""mp4 + image_streams + image_frames → 단일 토픽 Image 재발송.

`TopicMessageReplayer` 와 동일한 인터페이스 (``start(start_time,
first_history_time)`` / ``stop()`` / ``get_first_stamp()``) 를 단일 이미지
토픽에 대해 제공한다. 차이점은 입력이 mp4 + DB sidecar 라는 것 뿐이며,
시간축 변환 공식은 동일하다 — 임의의 메시지 (stamp = S) 는
``start_time + (S - first_history_time)`` 시각에 발행된다. 단,
``S < first_history_time`` 인 메시지는 publish 하지 않고 무시한다.

설계:
    * ``__init__`` — image_streams + image_frames 메타를 사전 로드하고
      mp4 파일 경로를 검증한다. publisher 도 즉시 생성한다 (메시지 타입은
      ``sensor_msgs/Image`` 로 고정).
    * ``start(start_time, first_history_time)`` — cv2 capture 를 열고
      디코더 스레드 + publisher 스레드를 기동한 뒤 즉시 반환한다.
        - 디코더 스레드: mp4 frame 을 순서대로 디코딩해 bounded 큐에
          enqueue 한다 (queue full 시 backpressure).
        - publisher 스레드: ``start_time`` 까지 ROS clock 으로 wait 한 뒤,
          큐에서 (ndarray, frame_meta) 를 꺼내 ``start_time + (stamp -
          first_history_time)`` 시각에 ``Image`` 메시지를 publish 한다.
          stamp 는 wall-clock 시간축으로 shift 한다 (Servo 의
          ``incoming_command_timeout`` 대응).
    * ``stop()`` — cooperative cancellation 신호만 보내고 즉시 반환한다.
    * ``close()`` — stop + 디코더/publisher 스레드 join + cv2 capture 해제.

스레드 모델 (start 후):
    * 디코더 스레드 — cv2.read 의 30~50ms 블로킹을 publish timing 에서
      분리하기 위해 별도. queue (default depth 4) 로 backpressure.
    * publisher 스레드 — stamp 기반 wait + msg wrap + publish.
    * publisher 는 ``Node.create_publisher`` (rclpy 가 thread-safe).

주의:
    * ``stop_event.wait`` 는 monotonic 기반이고 ROS clock 은 다를 수 있다.
      매 100ms 마다 ROS clock 을 재확인해 누적 drift 는 흡수하지만, 시뮬
      시간이 정지된 경우에는 ``_wait_until_ros_time`` 이 진행하지 않는다.
"""

from __future__ import annotations

from typing import Any, Final, Optional

import logging
import queue
import threading
from pathlib import Path

import psycopg

from builtin_interfaces.msg import Time
from rclpy.node import Node, Publisher
from sensor_msgs.msg import Image

from rdfp.types import ImageMetadata, to_ros_image_msg

from ._image_helpers import (
    SUPPORTED_PIXEL_FORMATS,
    build_image_metadata,
    decode_one_ndarray,
    lookup_topic_id,
    read_image_frames_meta,
    read_image_stream_meta,
    stamp_to_ns,
)


_logger = logging.getLogger(__name__)

# 큐 종료 신호 (None 이 아닌 unique sentinel).
_QUEUE_END: Final[object] = object()

# Decode 큐의 기본 깊이. 너무 크면 메모리 압박, 너무 작으면 디코더 backpressure.
# 4 ≈ 10MB @ 1080p — 30~50ms decode jitter 흡수에 충분.
_DEFAULT_DECODE_QUEUE_SIZE: Final[int] = 4

# ROS clock polling interval (sec) — stop_event 즉응 + clock drift 흡수.
_CLOCK_POLL_SEC: Final[float] = 0.1


def _time_to_ns(t: Any) -> int:
    """`builtin_interfaces.msg.Time` 또는 `rclpy.time.Time` 을 ns 로 환산."""
    if hasattr(t, 'nanoseconds'):
        return int(t.nanoseconds)
    return int(t.sec) * 1_000_000_000 + int(t.nanosec)


class Mp4ImageReplayer:
    """mp4 + image_streams + image_frames → 단일 이미지 토픽으로 재발송.

    상세는 모듈 docstring 참조.

    Lifecycle:
        ``__init__`` → 메타 사전 로드 + mp4 검증 + publisher 생성
        ``start(start_time, first_history_time)`` → 디코더/publisher 스레드 기동
        ``stop()`` → 중단 신호 (즉시 반환)
        ``close()`` → 워커 join + cv2 capture 해제 (idempotent)
    """

    def __init__(self, node: Node, conn: psycopg.Connection, episode_id: int, topic_name: str,
                 mp4_root: Path, *,
                 decode_queue_size: int = _DEFAULT_DECODE_QUEUE_SIZE,
                 publish_queue: int = 10,
                 logger: Optional[logging.Logger] = None) -> None:
        """메타 (image_streams + image_frames) 사전 로드 + publisher 생성.

        Args:
            node: ROS 노드. publisher 생성과 ROS clock 조회에 사용된다.
            conn: 열려 있는 PostgreSQL 커넥션. 본 생성자 동안에만 사용된다.
            episode_id: ``sessions.id``.
            topic_name: 발행 대상 이미지 토픽 이름.
            mp4_root: ``image_streams.mp4_path`` 의 기준 raoot.
            decode_queue_size: 디코더 → publisher 큐 깊이 (양의 정수).
            publish_queue: publisher 의 큐 깊이.
            logger: 진단 로거. 생략 시 모듈 로거.

        Raises:
            ValueError: topic 이 ``topics`` 에 없거나, ``image_streams`` 행이
                없거나, ``image_frames`` row 가 0 개거나, 지원하지 않는
                ``pixel_format`` 인 경우.
            RuntimeError: mp4 파일을 찾을 수 없는 경우.
        """
        self._node = node
        self._logger = logger or _logger
        self._decode_queue_size = max(1, int(decode_queue_size))

        topic_id = lookup_topic_id(conn, topic_name)
        self._stream: dict = read_image_stream_meta(conn, episode_id, topic_id)
        self._frames_meta: list[dict] = read_image_frames_meta(
            conn, episode_id, topic_id)
        if not self._frames_meta:
            raise ValueError(
                f'no image_frames row for episode_id={episode_id}, '
                f'topic_id={topic_id} (topic={topic_name!r})')

        self._pixel_format: str = self._stream['pixel_format']
        if self._pixel_format not in SUPPORTED_PIXEL_FORMATS:
            raise ValueError(
                f'unsupported pixel_format in image_streams: {self._pixel_format!r}; '
                f'expected one of {sorted(SUPPORTED_PIXEL_FORMATS)}')

        self._mp4_path: Path = Path(mp4_root) / self._stream['mp4_path']
        if not self._mp4_path.is_file():
            raise RuntimeError(f'mp4 file not found: {self._mp4_path}')

        self._metadata: ImageMetadata = build_image_metadata(self._stream)
        self._topic_name: str = topic_name
        self._episode_id: int = episode_id

        # Publisher 는 즉시 생성 (start 마다 새로 만들지 않음).
        self._publisher: Publisher = node.create_publisher(Image, topic_name, publish_queue)

        # Runtime — start() 에서 셋.
        self._cap = None
        self._decode_queue: Optional[queue.Queue] = None
        self._decoder_thread: Optional[threading.Thread] = None
        self._publisher_thread: Optional[threading.Thread] = None
        self._stop_event: Optional[threading.Event] = None
        # 디코더가 큐 sentinel 을 못 넣고 종료한 경우에도 publisher 가 무한 대기에
        # 빠지지 않도록, 디코더 종료 신호를 별도로 둔다.
        self._decoder_done: threading.Event = threading.Event()
        self._published_count: int = 0
        # 디코더/퍼블리셔 스레드에서 처음 발생한 예외. join() 이 re-raise 한다.
        self._error: Optional[BaseException] = None
        self._closed: bool = False

    # ---------- properties --------------------------------------------------

    @property
    def topic_name(self) -> str:
        return self._topic_name

    @property
    def episode_id(self) -> int:
        return self._episode_id

    @property
    def metadata(self) -> ImageMetadata:
        """``image_streams`` 으로부터 빌드된 공통 ``ImageMetadata``."""
        return self._metadata

    @property
    def expected_count(self) -> int:
        """전체 frame 수 (= image_frames 행 수)."""
        return len(self._frames_meta)

    @property
    def first_stamp_ns(self) -> int:
        """첫 프레임의 stamp (ns)."""
        return stamp_to_ns(self._frames_meta[0])

    @property
    def last_stamp_ns(self) -> int:
        """마지막 프레임의 stamp (ns)."""
        return stamp_to_ns(self._frames_meta[-1])

    @property
    def duration_sec(self) -> float:
        """첫 ↔ 마지막 stamp 간 간격 (초)."""
        return (self.last_stamp_ns - self.first_stamp_ns) / 1e9

    @property
    def published_count(self) -> int:
        """현재까지 publish 된 메시지 수."""
        return self._published_count

    @property
    def is_running(self) -> bool:
        return (self._publisher_thread is not None
                and self._publisher_thread.is_alive())

    @property
    def error(self) -> Optional[BaseException]:
        """디코더/퍼블리셔 스레드에서 처음 발생한 예외 (없으면 None).

        ``is_running`` 이 True 인 동안에도 부분 실패 (예: 디코더만 죽음) 를
        조기에 감지하기 위해 실시간으로 갱신된다. ``join()`` 은 종료 후
        이 값을 자동으로 re-raise 한다.
        """
        return self._error

    def get_first_stamp(self) -> Time:
        """첫 프레임의 stamp 를 ``builtin_interfaces.msg.Time`` 으로 반환.

        ``TopicMessageReplayer.get_first_stamp`` 와 동일한 시맨틱 — 보통
        이 값을 ``start(..., first_history_time=...)`` 의 anchor 로 그대로
        넘겨 첫 프레임이 ``start_time`` 시각에 발행되도록 한다.
        """
        ns = self.first_stamp_ns
        t = Time()
        t.sec = int(ns // 1_000_000_000)
        t.nanosec = int(ns % 1_000_000_000)
        return t

    # ---------- lifecycle ---------------------------------------------------

    def start(self, start_time: Time, first_history_time: Time) -> None:
        """디코더 + publisher 스레드를 띄워 비동기로 재발송을 시작한다 (즉시 반환).

        시간축 변환 공식은 ``TopicMessageReplayer`` 와 동일하다 — 임의의
        프레임 (stamp = S) 은 ``start_time + (S - first_history_time)``
        시각에 publish 된다. ``S < first_history_time`` 인 프레임은
        publish 하지 않고 무시한다.

        ``cap.isOpened()`` 가 True 여도 깨진 mp4 / 0-frame mp4 의 경우 첫
        ``read()`` 가 실패할 수 있으므로, 첫 프레임을 즉시 디코딩하여 검증
        한다 (성공한 첫 프레임은 큐에 미리 채워둔다).

        한 번만 호출 가능. ``close()`` 이후 재시작은 지원하지 않는다 (새
        인스턴스를 만들 것).

        Args:
            start_time: 발행을 시작하는 wall-clock 시각 (ROS clock 기준).
                ``builtin_interfaces.msg.Time`` 또는 ``rclpy.time.Time``.
            first_history_time: 위 ``start_time`` 에 대응되는 저장 시간축
                상의 기준 시각. 보통 ``get_first_stamp()`` 결과를 넘긴다.

        Raises:
            RuntimeError: 이미 ``start()`` 가 호출되었거나, ``close()`` 이후
                호출된 경우, mp4 파일을 열 수 없는 경우, 첫 프레임 디코딩이
                실패한 경우.
        """
        if self._closed:
            raise RuntimeError('Mp4ImageReplayer is already closed')
        if self._publisher_thread is not None:
            raise RuntimeError('Mp4ImageReplayer is already started')

        import cv2

        cap = cv2.VideoCapture(str(self._mp4_path))
        if not cap.isOpened():
            raise RuntimeError(f'failed to open mp4: {self._mp4_path}')

        # 첫 프레임을 즉시 디코딩하여 mp4 가 실제로 디코딩 가능한지 검증한다.
        # cap.isOpened() 만으로는 헤더는 정상이지만 본문이 깨진 mp4, 0-frame
        # mp4, 또는 image_frames 메타와 mismatch 된 mp4 를 잡지 못한다.
        first_pixels = decode_one_ndarray(cap, self._pixel_format)
        if first_pixels is None:
            cap.release()
            raise RuntimeError(
                f'failed to decode first frame of mp4: {self._mp4_path} '
                f'(image_frames had {len(self._frames_meta)} row(s); '
                f'mp4 may be corrupted or empty)')

        self._cap = cap
        self._decode_queue = queue.Queue(maxsize=self._decode_queue_size)
        self._stop_event = threading.Event()
        self._decoder_done.clear()
        self._published_count = 0

        # 검증된 첫 프레임을 큐에 미리 넣어둔다. 디코더 스레드는 인덱스 1 부터.
        # 큐 항목 형식: (ndarray, frame_meta_dict).
        self._decode_queue.put_nowait((first_pixels, self._frames_meta[0]))

        self._decoder_thread = threading.Thread(
            target=self._decoder_loop,
            name=f'mp4-image-decoder-{self._topic_name}', daemon=True)
        self._decoder_thread.start()

        start_ns = _time_to_ns(start_time)
        anchor_ns = _time_to_ns(first_history_time)
        self._publisher_thread = threading.Thread(
            target=self._publisher_loop,
            args=(start_ns, anchor_ns, self._stop_event),
            name=f'mp4-image-publisher-{self._topic_name}', daemon=True)
        self._publisher_thread.start()

        self._logger.debug(
            'Mp4ImageReplayer started: topic=%s frames=%d duration=%.3fs',
            self._topic_name, self.expected_count, self.duration_sec)

    def stop(self) -> None:
        """진행 중인 재발송에 중단 신호를 보낸다 (즉시 반환, 워커 join 안 함)."""
        if self._stop_event is not None:
            self._stop_event.set()

    def join(self, timeout: Optional[float] = None) -> None:
        """publisher 스레드의 종료를 기다린다 (idempotent — 워커 없으면 no-op).

        스레드 내부에서 캐치된 예외가 있고 publisher 가 실제로 종료된 경우
        해당 예외를 re-raise 한다 (``timeout`` 으로 일찍 반환된 경우는
        raise 하지 않음 — 호출자가 다시 ``join()`` 하거나 ``error`` property
        로 확인할 수 있다).
        """
        if self._publisher_thread is not None:
            self._publisher_thread.join(timeout=timeout)
            if not self._publisher_thread.is_alive() and self._error is not None:
                raise self._error

    def close(self) -> None:
        """디코더/publisher 종료 + cv2 capture 해제 (idempotent).

        스레드가 2초 안에 종료되지 못한 경우 use-after-free 를 막기 위해 해당
        스레드가 쓰는 리소스의 파괴를 건너뛴다 (Python 은 스레드 강제 종료
        수단이 없으므로, leak 이 use-after-free 보다 안전한 차선책 — node
        종료 시 함께 정리됨).
            * decoder 스레드가 살아 있으면 ``cv2.VideoCapture.release()`` skip
              (디코더가 ``self._cap`` 을 쓰는 중일 수 있음).
            * publisher 스레드가 살아 있으면 ``destroy_publisher()`` skip
              (publisher 가 ``self._publisher`` 를 쓰는 중일 수 있음).
        """
        if self._closed:
            return
        self._closed = True
        if self._stop_event is not None:
            self._stop_event.set()

        # 디코더가 frame_q.put 에 block 중이면 큐를 한 번 비워 깨운다.
        if self._decode_queue is not None:
            try:
                while True:
                    self._decode_queue.get_nowait()
            except queue.Empty:
                pass

        publisher_alive = False
        if self._publisher_thread is not None:
            self._publisher_thread.join(timeout=2.0)
            publisher_alive = self._publisher_thread.is_alive()
            if publisher_alive:
                self._logger.warning(
                    'mp4 image publisher thread did not terminate within 2s; '
                    'skipping destroy_publisher to avoid use-after-free '
                    '(topic=%s, publisher will leak until node shutdown)',
                    self._topic_name)

        decoder_alive = False
        if self._decoder_thread is not None:
            self._decoder_thread.join(timeout=2.0)
            decoder_alive = self._decoder_thread.is_alive()
            if decoder_alive:
                self._logger.warning(
                    'mp4 image decoder thread did not terminate within 2s; '
                    'skipping cv2 capture release to avoid use-after-free '
                    '(topic=%s, capture will leak until process exit)',
                    self._topic_name)

        # cv2 capture 해제 — decoder 가 죽은 경우에만.
        if self._cap is not None and not decoder_alive:
            self._cap.release()
            self._cap = None

        # Publisher destroy — publisher 스레드가 죽은 경우에만.
        # rclpy 의 destroy_publisher 는 동일 객체에 두 번 호출해도 무해하다.
        if self._publisher is not None and not publisher_alive:
            try:
                self._node.destroy_publisher(self._publisher)
            except Exception:   # noqa: BLE001
                self._logger.exception(
                    'destroy_publisher failed (topic=%s)', self._topic_name)
            self._publisher = None   # type: ignore[assignment]

    def __enter__(self) -> 'Mp4ImageReplayer':
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def __del__(self) -> None:
        # GC 시 최후 안전장치로 정리 시도.
        try:
            self.close()
        except Exception:
            pass

    # ---------- 디코더 스레드 ----------------------------------------------

    def _decoder_loop(self) -> None:
        """mp4 에서 frame 을 순서대로 디코딩해 큐에 enqueue 한다.

        첫 프레임 (index 0) 은 ``start()`` 에서 이미 큐에 push 되었으므로
        본 스레드는 인덱스 1 부터 진행한다. 큐가 가득 차면 backpressure 로
        ``put`` 이 대기한다 (디코더가 publisher 보다 너무 앞서가지 않게).

        큐 항목 형식: ``(ndarray, frame_meta_dict)``. Image msg wrap 은
        publisher 측에서 수행한다.
        """
        assert self._stop_event is not None and self._decode_queue is not None
        try:
            for fm in self._frames_meta[1:]:
                if self._stop_event.is_set():
                    break
                pixels = decode_one_ndarray(self._cap, self._pixel_format)
                if pixels is None:
                    # mp4 가 image_frames row 수보다 짧으면 종료.
                    self._logger.warning(
                        'mp4 decode returned None at frame_index=%d (mp4 shorter '
                        'than image_frames=%d); ending early for topic %s',
                        fm['frame_index'], len(self._frames_meta), self._topic_name)
                    break
                item = (pixels, fm)
                while not self._stop_event.is_set():
                    try:
                        self._decode_queue.put(item, timeout=0.1)
                        break
                    except queue.Full:
                        continue
        except Exception as exc:   # noqa: BLE001
            self._logger.exception(
                'mp4 image decoder thread crashed for topic %s', self._topic_name)
            self._record_error(exc)
        finally:
            self._decoder_done.set()
            # 부수적으로 sentinel 도 시도 — 큐에 자리가 있다면 publisher 가 즉시
            # 깨어나는 fast path. 실패해도 _decoder_done 으로 충분.
            if self._decode_queue is not None:
                try:
                    self._decode_queue.put_nowait(_QUEUE_END)
                except queue.Full:
                    pass

    # ---------- publisher 스레드 -------------------------------------------

    def _publisher_loop(self, start_ns: int, anchor_ns: int,
                        stop_event: threading.Event) -> None:
        """publisher 본체: start_time wait → frame loop → cadence 재현 publish.

        시간축 변환은 ``TopicMessageReplayer._run`` 과 동일 — 임의의
        프레임 (stamp = S) 은 ``start_ns + (S - anchor_ns)`` 시각에 publish.
        ``S < anchor_ns`` 인 프레임은 publish 하지 않고 무시한다 (anchor 보다
        이른 stamp 의 메시지는 시간축 상 "과거" 로 간주).

        예외는 모두 잡아 로그만 남기고 깔끔하게 종료한다.
        """
        skipped = 0
        try:
            # 1) start_time 까지 wait (ROS clock 기준).
            if self._wait_until_ros_time(start_ns, stop_event):
                self._logger.info(
                    'mp4 replay stopped before start_time (topic=%s)',
                    self._topic_name)
                return

            # 2) 큐에서 한 frame 씩 꺼내며 stamp 기반 wait + publish.
            while not stop_event.is_set():
                item = self._dequeue_one(stop_event)
                if item is None:
                    break
                pixels, fm = item

                stamp_ns = stamp_to_ns(fm)
                # anchor 보다 이른 stamp 의 프레임은 무시한다.
                if stamp_ns < anchor_ns:
                    skipped += 1
                    continue
                target_ns = start_ns + (stamp_ns - anchor_ns)
                if self._wait_until_ros_time(target_ns, stop_event):
                    break

                # stamp shift — Servo 처럼 incoming_command_timeout 가진 소비자
                # 가 fresh 로 인식하도록 wall-clock 시간축 stamp 로 갱신한다.
                stamp = Time()
                stamp.sec = int(target_ns // 1_000_000_000)
                stamp.nanosec = int(target_ns % 1_000_000_000)
                msg = to_ros_image_msg(self._metadata, stamp, pixels)
                self._publisher.publish(msg)
                self._published_count += 1

            self._logger.info(
                'mp4 replay finished: %d/%d frame(s) published, %d skipped '
                '(topic=%s)',
                self._published_count, self.expected_count, skipped,
                self._topic_name)
        except Exception as exc:   # noqa: BLE001
            self._logger.exception(
                'mp4 image publisher loop crashed for topic %s', self._topic_name)
            self._record_error(exc)

    def _record_error(self, exc: BaseException) -> None:
        """디코더/퍼블리셔 스레드에서 잡힌 첫 예외만 보관하고 다른 스레드도 정리시킨다.

        thread-safe — 두 스레드가 동시에 호출해도 첫 예외가 우선되며 (CPython GIL
        하에서 단일 attribute 할당은 atomic), stop_event 셋으로 다른 스레드가
        조속히 빠져나오게 한다. ``join()`` 이 종료 후 이 값을 re-raise.
        """
        if self._error is None:
            self._error = exc
        if self._stop_event is not None:
            self._stop_event.set()

    def _dequeue_one(self, stop_event: threading.Event) -> Optional[tuple]:
        """큐에서 (ndarray, frame_meta) 하나를 꺼낸다.

        sentinel / 디코더 종료 / stop 시 ``None`` 반환. 디코더가 큐 가득 찬
        상태로 종료되어 sentinel 이 유실된 경우에도 ``_decoder_done`` event
        polling 으로 무한 대기에 빠지지 않는다.
        """
        assert self._decode_queue is not None
        while True:
            try:
                item = self._decode_queue.get(timeout=0.03)
                break
            except queue.Empty:
                if stop_event.is_set():
                    return None
                if self._decoder_done.is_set():
                    return None
                continue
        if item is _QUEUE_END:
            return None
        return item

    def _wait_until_ros_time(self, target_ns: int,
                             stop_event: threading.Event) -> bool:
        """ROS clock 이 ``target_ns`` 에 도달할 때까지 wait.

        ``stop_event`` 즉응을 위해 100ms 단위로 polling 하며, 매 polling
        마다 ROS clock 을 다시 읽어 monotonic 기반 wait 의 누적 drift 를
        흡수한다.

        Returns:
            True 면 stop_event 셋으로 중단됨, False 면 정상 도달 (target 이
            이미 과거였던 경우 포함).
        """
        while not stop_event.is_set():
            now_ns = self._node.get_clock().now().nanoseconds
            wait_ns = target_ns - now_ns
            if wait_ns <= 0:
                return False
            wait_sec = min(wait_ns / 1e9, _CLOCK_POLL_SEC)
            if stop_event.wait(timeout=wait_sec):
                return True
        return True


__all__ = ['Mp4ImageReplayer']
