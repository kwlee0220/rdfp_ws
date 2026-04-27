"""에피소드의 복수 비이미지 토픽 메시지를 stamp 정렬 후 원본 cadence 로 재발송.

`replay_cmd.py` 의 dedicated-thread-per-topic 패턴과 달리, 본 클래스는
``heapq.merge`` 로 단일 정렬 시퀀스를 만든 뒤 한 워커 스레드에서 차례로
발행한다. servo command 처럼 작은 메시지들의 묶음 재생에 적합하다.

이미지 토픽은 받지 않는다 — 이미지 ReplayStream 의 close() 가 frame_q.get()
에서 블록된 consumer 를 깨운다는 보장이 약하고, 큰 이미지의 DDS 송신 비용이
다른 토픽 cadence 를 늦출 수 있다. 이미지는 토픽당 ``Mp4ImageReplayer``
인스턴스를 따로 띄울 것 (``__init__`` 에서 이미지 토픽 발견 시 ValueError).

설계:
    * ``__init__`` — 토픽별 ``ReplayStream`` 을 사전 로드하고 publisher 를
      생성한다. ``message_reader`` 결과를 list iterator 로 wrap.
    * ``start(start_time, first_history_time)`` — 워커 스레드를 띄워 즉시
      반환한다. 워커는 ``start_time`` 까지 ROS clock 으로 wait 한 뒤,
      호출자가 지정한 ``first_history_time`` 을 anchor 로 잡고 각 메시지를
      ``start_time + (stamp - first_history_time)`` 시각에 발행한다 (즉,
      "저장된 시각 first_history_time 이 곧 wall-clock 의 start_time 이라고
      가정"). ``stamp < first_history_time`` 인 메시지는 publish 하지 않고
      무시한다.
    * ``stop()`` — cooperative cancellation 신호만 보내고 즉시 반환한다.

스레드 모델:
    * 워커 스레드는 한 개. heap merge 결과를 차례로 publish 한다.
    * publisher 는 ``Node.create_publisher`` (rclpy 가 thread-safe).

주의:
    * ``stop_event.wait`` 는 monotonic 기반이고 ROS clock 은 다를 수 있다.
      매 100ms 마다 ROS clock 을 재확인해 누적 drift 는 흡수하지만, 시뮬
      시간이 정지된 경우에는 ``_wait_until_ros_time`` 이 진행하지 않는다.
"""

from __future__ import annotations

from typing import Any, Iterator, Optional

import heapq
import logging
import threading

import psycopg

from builtin_interfaces.msg import Time
from rclpy.node import Node, Publisher

from rdfp.dataset.types import ReplayStream, StampedMessage

from .image_message_reader import is_image_topic_in_db
from .message_reader import read_topic_messages_by_name


_logger = logging.getLogger(__name__)

# ROS clock polling interval (sec) — stop_event 즉응 + clock drift 흡수.
_CLOCK_POLL_SEC = 0.1


def _stamp_ns(msg: StampedMessage) -> int:
    """`header.stamp` 를 정수 나노초로 환산한다."""
    s = msg.header.stamp
    return int(s.sec) * 1_000_000_000 + int(s.nanosec)


def _time_to_ns(t: Any) -> int:
    """`builtin_interfaces.msg.Time` 또는 `rclpy.time.Time` 을 ns 로 환산."""
    if hasattr(t, 'nanoseconds'):
        return int(t.nanoseconds)
    return int(t.sec) * 1_000_000_000 + int(t.nanosec)


class TopicMessageReplayer:
    """에피소드의 복수 토픽을 stamp 기준 merge 후 원본 cadence 로 재발송.

    Lifecycle (one-shot — 재시작 불가, 새 인스턴스를 만들 것):
        ``__init__`` → 토픽별 ReplayStream 사전 로드 + publisher 생성
        ``start(start_time)`` → 워커 스레드 기동 (즉시 반환, 한 번만 호출 가능)
        ``stop()`` → 중단 신호 (즉시 반환)
        ``close()`` → 워커 join + ReplayStream close (idempotent)
    """

    def __init__(self, node: Node, conn: psycopg.Connection,
                 episode_id: int, topic_names: list[str], *,
                 qos_depth: int = 10,
                 logger: Optional[logging.Logger] = None) -> None:
        """토픽별 ReplayStream 사전 로드 + publisher 생성.

        Args:
            node: ROS 노드. publisher 생성과 ROS clock 조회에 사용된다.
            conn: 열려있는 PostgreSQL 커넥션. 본 생성자 동안에만 사용된다.
            episode_id: ``sessions.id``.
            topic_names: 재생할 토픽 이름 목록. 이미지 토픽은 허용하지 않는다
                (별도 ``Mp4ImageReplayer`` 사용).
            qos_depth: publisher 의 큐 깊이.
            logger: 진단 로거. 생략 시 모듈 로거.

        Raises:
            ValueError: episode 가 ``sessions`` 에 없는 경우, 토픽이
                ``topics`` 에 없거나 등록된 binding 이 없는 경우,
                이미지 토픽이 포함된 경우, 또는 요청된 모든 토픽이 해당
                에피소드에서 빈 source 인 경우 (사용 불가능한 replayer
                생성을 방지).
        """
        self._node = node
        self._logger = logger or _logger
        self._episode_id = episode_id
        self._requested_topic_names = list(topic_names)

        # 1) 에피소드 존재 확인.
        with conn.cursor() as cur:
            cur.execute('SELECT 1 FROM sessions WHERE id = %s', (episode_id,))
            if cur.fetchone() is None:
                raise ValueError(f'episode id {episode_id} not found in sessions')

        # 2) 이미지 토픽 거부. close() 시 frame_q.get() 에서 블록된 consumer 를
        #    깨운다는 보장이 약해 use-after-destroy 위험이 있으므로 사전 차단.
        for topic in self._requested_topic_names:
            if is_image_topic_in_db(conn, topic):
                raise ValueError(
                    f'image topic {topic!r} is not supported by '
                    f'TopicMessageReplayer; use Mp4ImageReplayer instead')

        # 3) 토픽별 ReplayStream 로드. 부분 로드 후 실패 시 이미 연 stream 들은 close.
        self._streams: list[ReplayStream] = []
        try:
            for topic in self._requested_topic_names:
                s = self._open_stream(conn, episode_id, topic)
                if s is None:
                    self._logger.warning(
                        'episode %d has no messages on topic %s; skipping',
                        episode_id, topic)
                    continue
                self._streams.append(s)
        except Exception:
            for s in self._streams:
                try:
                    s.close()
                except Exception:   # noqa: BLE001
                    pass
            raise

        # 모든 토픽이 빈 source 였다면 사용 불가능한 replayer — 생성 중단.
        # 이렇게 막지 않으면 호출자가 ``get_first_stamp()`` / ``start()`` 에서
        # 'no streams' RuntimeError 를 만나, 같이 묶인 다른 replayer (예: 이미지)
        # 까지 연쇄 실패시킬 수 있다.
        if not self._streams:
            raise ValueError(
                f'episode {episode_id} has no messages on any of the requested '
                f'topics: {self._requested_topic_names}')

        # 3) publisher 는 first_message 의 타입으로 미리 만들어둔다.
        self._publishers: dict[str, Publisher] = {
            s.topic_name: node.create_publisher(
                type(s.first_message), s.topic_name, qos_depth)
            for s in self._streams
        }

        # 4) 워커 상태.
        self._stop_event: Optional[threading.Event] = None
        self._worker_thread: Optional[threading.Thread] = None
        self._published_count: int = 0
        # 워커 스레드에서 발생한 예외. join() 이 re-raise 한다.
        self._error: Optional[BaseException] = None
        self._closed: bool = False

    # ---------- properties --------------------------------------------------

    @property
    def topic_names(self) -> list[str]:
        """실제로 stream 이 열린 토픽 이름 목록 (skip 된 토픽 제외)."""
        return [s.topic_name for s in self._streams]

    @property
    def expected_count(self) -> int:
        """ReplayStream 들의 expected_count 합 (unknown(None) 은 제외)."""
        return sum(s.expected_count for s in self._streams
                   if s.expected_count is not None)

    @property
    def published_count(self) -> int:
        """현재까지 publish 된 메시지 수."""
        return self._published_count

    @property
    def is_running(self) -> bool:
        return self._worker_thread is not None and self._worker_thread.is_alive()

    @property
    def error(self) -> Optional[BaseException]:
        """워커 스레드에서 발생한 예외 (없으면 None).

        ``is_running`` 이 True 인 동안에도 부분 실패를 조기에 감지하기 위해
        실시간으로 갱신된다. ``join()`` 은 종료 후 이 값을 자동으로 re-raise.
        """
        return self._error

    def get_first_stamp(self) -> Time:
        """적재된 모든 메시지 중 가장 이른 stamp 를 반환한다.

        각 ``ReplayStream.iterator`` 는 stamp ASC 가 보장되므로 ``first_message``
        들의 stamp 중 최솟값이 곧 전체 메시지의 가장 이른 stamp 다 (heap merge
        결과의 첫 항목과 동일). 반환 타입은 ``builtin_interfaces.msg.Time`` 으로,
        그대로 ``start(start_time)`` 의 anchor 로 넘겨 원본 stamp 시각에 재생을
        시작할 수 있다.

        Raises:
            RuntimeError: 열린 stream 이 하나도 없는 경우.
        """
        if not self._streams:
            raise RuntimeError('no streams loaded')
        earliest_ns = min(_stamp_ns(s.first_message) for s in self._streams)
        t = Time()
        t.sec = int(earliest_ns // 1_000_000_000)
        t.nanosec = int(earliest_ns % 1_000_000_000)
        return t

    # ---------- lifecycle ---------------------------------------------------

    def start(self, start_time: Time, first_history_time: Time) -> None:
        """워커 스레드를 띄워 비동기로 재생을 시작한다 (즉시 반환).

        ``first_history_time`` 은 저장된 메시지 시간축 상의 anchor 로,
        "저장된 시각 ``first_history_time`` 이 wall-clock 의 ``start_time``
        에 해당한다" 고 가정한다는 의미다. 따라서 임의의 메시지 (stamp = S)
        의 발행 시각은 ``start_time + (S - first_history_time)`` 이 된다.
        보통 ``first_history_time`` 으로 ``get_first_stamp()`` 를 넘기면
        첫 메시지가 ``start_time`` 에 발행되어 원본 cadence 가 그대로
        재현된다. 더 이른 ``first_history_time`` 을 주면 모든 메시지가
        그만큼 늦게 발행되고, 더 늦은 값을 주면 ``S < first_history_time``
        에 해당하는 앞쪽 메시지들은 publish 되지 않고 무시된다.

        한 번만 호출 가능 — ``ReplayStream.iterator`` 는 일회성 소비이고
        ``_publish`` 가 메시지 stamp 를 in-place 로 수정하기 때문이다.
        재생을 다시 하려면 새 인스턴스를 만들어야 한다.

        Args:
            start_time: 발행 작업을 시작하는 wall-clock 시각 (ROS clock 기준).
                ``builtin_interfaces.msg.Time`` 또는 ``rclpy.time.Time``.
            first_history_time: 위 ``start_time`` 에 대응되는 저장 시간축 상의
                기준 시각. 동일한 두 타입을 모두 받는다.

        Raises:
            RuntimeError: 이미 ``start()`` 가 호출되었거나, 열린 stream 이
                하나도 없거나, ``close()`` 이후 호출된 경우.
        """
        if self._closed:
            raise RuntimeError('TopicMessageReplayer is already closed')
        if self._worker_thread is not None:
            raise RuntimeError('TopicMessageReplayer is already started')
        if not self._streams:
            raise RuntimeError('no streams to replay')

        self._stop_event = threading.Event()
        self._published_count = 0
        start_ns = _time_to_ns(start_time)
        anchor_ns = _time_to_ns(first_history_time)
        self._worker_thread = threading.Thread(
            target=self._run, args=(start_ns, anchor_ns, self._stop_event),
            name='topic-message-replayer', daemon=True)
        self._worker_thread.start()

    def stop(self) -> None:
        """진행 중인 재생에 중단 신호를 보낸다 (즉시 반환, 워커 join 안 함)."""
        if self._stop_event is not None:
            self._stop_event.set()

    def join(self, timeout: Optional[float] = None) -> None:
        """워커 스레드의 종료를 기다린다 (idempotent — 워커 없으면 no-op).

        스레드 내부에서 캐치된 예외가 있고 워커가 실제로 종료된 경우 해당
        예외를 re-raise 한다 (``timeout`` 으로 일찍 반환된 경우는 raise 하지
        않음 — 호출자가 다시 ``join()`` 하거나 ``error`` property 로 확인할
        수 있다).
        """
        if self._worker_thread is not None:
            self._worker_thread.join(timeout=timeout)
            if not self._worker_thread.is_alive() and self._error is not None:
                raise self._error

    def close(self) -> None:
        """워커 중단 + 모든 ReplayStream close + publisher destroy (idempotent).

        종료 순서가 중요하다 — 워커가 stream iterator 의 블로킹 read 안에
        있을 수 있어, stream 을 먼저 깨워야 stop_event 만으로 빠져나오지
        못한 워커도 정리된다. 그 후 publisher 를 destroy 해야 워커가
        뒤늦게 깨어 destroyed publisher 를 건드리는 use-after-destroy 를
        막을 수 있다.
            1) ``stop_event`` 셋 (cooperative)
            2) ``ReplayStream.close()`` — iterator 블로킹 해제
            3) 워커 join — 정상 종료 또는 iterator 예외로 종료
            4) ``destroy_publisher`` — **워커가 죽은 경우에만** 실행. 2초
               안에 빠져나오지 못하면 publisher 를 leak 시킨다 (Python 은
               스레드 강제 종료 수단이 없으므로, leak 이 use-after-destroy
               보다 안전한 차선책 — node 종료 시 함께 정리됨).
        """
        if self._closed:
            return
        self._closed = True

        # 1) Cooperative cancel.
        self.stop()

        # 2) Stream 들을 먼저 닫아 iterator 안에서 블록된 워커도 깨운다.
        #    (워커가 이미 정상 종료했어도 idempotent — close 가 no-op).
        for s in self._streams:
            try:
                s.close()
            except Exception:   # noqa: BLE001
                self._logger.exception(
                    'stream close failed: topic=%s', s.topic_name)

        # 3) 워커가 종료될 때까지 대기. stream close 이후엔 다음 next() 호출에서
        #    예외가 raise 되어 _run 의 try/except 로 잡히고 곧 빠져나온다.
        worker_alive = False
        if self._worker_thread is not None:
            self._worker_thread.join(timeout=2.0)
            worker_alive = self._worker_thread.is_alive()
            if worker_alive:
                self._logger.warning(
                    'replayer worker did not terminate within 2s; skipping '
                    'publisher destroy to avoid use-after-destroy. '
                    'publishers will leak until node shutdown.')

        # 4) 워커가 죽은 뒤에만 publisher destroy. 살아 있으면 leak 시키고 종료.
        if not worker_alive:
            for topic_name, pub in self._publishers.items():
                try:
                    self._node.destroy_publisher(pub)
                except Exception:   # noqa: BLE001
                    self._logger.exception(
                        'destroy_publisher failed (topic=%s)', topic_name)
            self._publishers.clear()

    def __enter__(self) -> 'TopicMessageReplayer':
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    # ---------- 내부 -------------------------------------------------------

    def _open_stream(self, conn: psycopg.Connection, episode_id: int,
                     topic_name: str) -> Optional[ReplayStream]:
        """비이미지 토픽의 ReplayStream 을 연다. 빈 source 면 None.

        이미지 토픽은 ``__init__`` 에서 사전 거부되므로 여기까지 오지 않는다.
        """
        messages = read_topic_messages_by_name(conn, episode_id, topic_name)
        if not messages:
            return None
        return ReplayStream(
            topic_name=topic_name, first_message=messages[0],
            iterator=iter(messages), expected_count=len(messages))

    def _run(self, start_ns: int, anchor_stamp_ns: int,
             stop_event: threading.Event) -> None:
        """워커 본체: start_time wait → heap merge → cadence 재현 publish.

        ``anchor_stamp_ns`` (호출자가 ``first_history_time`` 으로 지정) 가
        시간축 변환의 기준이다. 즉, 임의의 메시지 (stamp = S) 의 발행
        target 은 ``start_ns + (S - anchor_stamp_ns)``. ``S < anchor_stamp_ns``
        인 메시지는 publish 하지 않고 무시한다 (anchor 보다 이른 stamp 의
        메시지는 시간축 상 "과거" 로 간주).

        예외는 모두 잡아 로그만 남기고 깔끔하게 종료한다 (caller 가
        stop_event 셋이나 close() 로 회수).
        """
        skipped = 0
        try:
            # 1) start_time 까지 wait (ROS clock 기준).
            if self._wait_until_ros_time(start_ns, stop_event):
                self._logger.info('replay stopped before start_time')
                return

            # 2) 토픽 정보를 잃지 않도록 (topic, msg) 튜플로 wrap 한 뒤 heapq.merge.
            #    각 ReplayStream.iterator 는 stamp ASC 가 보장되므로 merge 결과도
            #    전역 stamp ASC 로 정렬된다.
            wrapped = [self._wrap_with_topic(s) for s in self._streams]
            merged = heapq.merge(*wrapped, key=lambda tm: _stamp_ns(tm[1]))

            # 3) 모든 메시지를 동일 공식 (target = start_ns + (msg_ns - anchor)) 으로
            #    처리한다. 첫 메시지를 따로 다룰 필요가 없다 — anchor 가 호출자
            #    지정이라 첫 메시지 stamp 와 같다는 보장이 없다.
            for topic, msg in merged:
                if stop_event.is_set():
                    break
                msg_ns = _stamp_ns(msg)
                # anchor 보다 이른 stamp 의 메시지는 무시한다.
                if msg_ns < anchor_stamp_ns:
                    skipped += 1
                    continue
                target_ns = start_ns + (msg_ns - anchor_stamp_ns)
                if self._wait_until_ros_time(target_ns, stop_event):
                    break
                self._publish(topic, msg, anchor_stamp_ns, start_ns)

            self._logger.info(
                'replay finished: %d message(s) published, %d skipped',
                self._published_count, skipped)
        except Exception as exc:   # noqa: BLE001
            self._logger.exception('TopicMessageReplayer worker crashed')
            if self._error is None:
                self._error = exc
            stop_event.set()

    @staticmethod
    def _wrap_with_topic(stream: ReplayStream) -> Iterator[tuple[str, StampedMessage]]:
        """ReplayStream.iterator 의 각 메시지를 (topic_name, msg) 튜플로 yield."""
        topic_name = stream.topic_name
        for msg in stream.iterator:
            yield (topic_name, msg)

    def _publish(self, topic_name: str, msg: StampedMessage,
                 anchor_stamp_ns: int, start_ns: int) -> None:
        """msg.header.stamp 를 ``start_ns + (msg_stamp - anchor)`` 로 shift 후 publish.

        ``anchor_stamp_ns`` 는 호출자가 지정한 ``first_history_time`` (저장
        시간축 anchor). ``start_ns`` 는 wall-clock anchor. Servo 처럼
        ``incoming_command_timeout`` 을 가진 소비자가 fresh 로 인식하도록
        stamp 를 wall-clock 시간축으로 갱신한다 (replay_cmd / replay_gui_cmd
        와 동일 정책).
        """
        msg_ns = _stamp_ns(msg)
        shifted_ns = start_ns + (msg_ns - anchor_stamp_ns)
        msg.header.stamp.sec = int(shifted_ns // 1_000_000_000)
        msg.header.stamp.nanosec = int(shifted_ns % 1_000_000_000)
        self._publishers[topic_name].publish(msg)
        self._published_count += 1

    def _wait_until_ros_time(self, target_ns: int,
                             stop_event: threading.Event) -> bool:
        """ROS clock 이 ``target_ns`` 에 도달할 때까지 wait.

        ``stop_event`` 즉응을 위해 100ms 단위로 polling 하며, 매 polling
        마다 ROS clock 을 다시 읽어 monotonic 기반 wait 의 누적 drift 를
        흡수한다.

        Returns:
            True 면 stop_event 셋으로 중단됨, False 면 정상 도달.
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


__all__ = ['TopicMessageReplayer']
