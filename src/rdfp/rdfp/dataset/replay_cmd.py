"""`replay` 독립 CLI — 지정 에피소드의 기록된 토픽(들)을 라이브 발행.

이전에는 ``dataset replay`` 서브커맨드였으나, ROS 2 런타임(``rclpy``)이
필요한 무거운 경로를 별도 entry_point 로 분리하여 ``dataset`` (init-db /
stats / list) 와 독립적으로 동작하도록 했다. 본 모듈 자체는 ROS sourced
환경에서만 import 가능하다 (rclpy / std_msgs / sensor_msgs 등 전이 의존).

토픽별로 ``ReplayStream`` 을 열어 (이미지는 mp4 + image_streams + image_frames
조합, 그 외는 message_reader) k-way merge 로 stamp 오름차순 streaming
publish 한다. 이미지 프레임은 ``next()`` 시점에 한 장씩 디코딩되어 메모리
사용량이 O(1) 로 유지된다.

공개 엔트리:
    * :func:`main` — argparse 진입점 (setup.py 의 ``replay`` console_script).
    * :func:`cmd_replay` — 파싱된 ``args`` 와 확정된 ``config_path`` 로 실행.
"""

from __future__ import annotations

from typing import Any

import argparse
import logging
import sys
import threading
import time
from pathlib import Path

from rclpy.node import Publisher

from rdfp.moveit.servo_client import ServoClient

from .cli_common import (
    DEFAULT_CONFIG_FILENAME, add_common_args, add_config_arg,
    configure_logging, load_dataset_or_fail, resolve_config_path,
)
from .db.connection import open_connection
from .db.image_message_reader import (
    is_image_topic_in_db, open_image_replay_source,
)
from .db.message_reader import read_topic_messages_by_name
from .types import ReplayStream, StampedMessage


def _stamp_ns(msg: StampedMessage) -> int:
    """`header.stamp` 를 정수 나노초로 환산한다."""
    stamp = msg.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _open_stream(conn, episode_id: int, topic_name: str,
                 mp4_root: Path | None) -> ReplayStream | None:
    """토픽 종류에 맞는 ``ReplayStream`` 을 연다.

    이미지 토픽이면 mp4 + sidecar 기반 lazy 스트림, 그 외는 message_reader
    결과를 list 기반 ReplayStream 으로 wrap. 빈 source 또는 mp4_root 미설정
    (이미지 토픽인 경우) 시 ``None``.
    """
    if is_image_topic_in_db(conn, topic_name):
        if mp4_root is None:
            logging.error(
                'output_mp4_dir is required to replay image topic %s; '
                'set it in the dataset config', topic_name,
            )
            return None
        return open_image_replay_source(conn, episode_id, topic_name, mp4_root)

    messages = read_topic_messages_by_name(conn, episode_id, topic_name)
    if not messages:
        return None
    return ReplayStream(
        topic_name=topic_name, first_message=messages[0],
        iterator=iter(messages), expected_count=len(messages),
    )


def cmd_replay(args: argparse.Namespace, config_path: str) -> int:
    """`replay` 서브커맨드: 지정 에피소드의 기록된 토픽(들)을 재발행한다.

    ``--topic`` 으로 여러 토픽을 지정하면 각 토픽의 ``ReplayStream`` 을 구성
    한다. 이후 내부 지역 함수 ``replay`` 가 전체 메시지를 stamp 오름차순으로
    streaming merge 하여 원본 상대 타이밍대로 각 publisher 에 재생한다. 각
    메시지의 ``header.stamp`` 는 재생 시작 시점의 ROS clock 기준으로 offset
    되어 MoveIt Servo 등 ``incoming_command_timeout`` 을 가진 소비자가
    "fresh" 로 인식한다.

    ``config_path`` 는 호출측(cli) 에서 이미 확정된 경로를 넘겨받는다.
    """
    import rclpy

    cfg = load_dataset_or_fail(config_path)
    if cfg is None:
        return 2

    # 토픽별 ReplayStream 을 생성한다. 이미지 토픽은 mp4 + image_frames lazy 디코딩.
    # rclpy 초기화 전에 연결이 끝나도록 먼저 수행한다.
    mp4_root = Path(cfg.output_mp4_dir) if cfg.output_mp4_dir else None
    streams: list[ReplayStream] = []
    try:
        with open_connection(cfg.db) as conn:
            with conn.cursor() as cur:
                cur.execute('SELECT 1 FROM sessions WHERE id = %s', (args.episode_id,))
                if cur.fetchone() is None:
                    logging.error('episode id %d not found in sessions', args.episode_id)
                    return 2
            for topic in args.topic:
                try:
                    s = _open_stream(conn, args.episode_id, topic, mp4_root)
                except ValueError as exc:
                    logging.error('%s', exc)
                    _close_all(streams)
                    return 2
                except RuntimeError as exc:
                    logging.error('%s', exc)
                    _close_all(streams)
                    return 1
                if s is None:
                    logging.warning('episode %d has no messages on topic %s; skipping',
                                    args.episode_id, topic)
                    continue
                streams.append(s)
            conn.rollback()
    except Exception:
        logging.exception('failed to open replay streams from DB')
        _close_all(streams)
        return 1

    if not streams:
        logging.warning('episode %d has no replayable messages on any of %s',
                        args.episode_id, args.topic)
        return 0

    per_topic_expected = ', '.join(
        f'{s.topic_name}={s.expected_count if s.expected_count is not None else "?"}'
        for s in streams
    )
    logging.info(
        'replaying episode %d across %d topic(s): [%s]',
        args.episode_id, len(streams), per_topic_expected,
    )

    rclpy.init()
    node = rclpy.create_node('dataset_replay')

    # publish 전에 servo 를 기동한다. 이미 기동된 경우에도 start_servo 는 무해하며,
    # 서비스 미준비나 호출 실패 시에는 경고만 남기고 진행한다 (사용자가 수동으로
    # 서보를 띄워 둔 시나리오도 지원하기 위함).
    servo_client = ServoClient.create(node)
    if servo_client.wait_for_services_ready(timeout_sec=5.0):
        started, servo_message = servo_client.start()
        if started:
            logging.info('servo started: %s', servo_message)
        else:
            logging.warning(
                'servo start failed (%s); proceeding with publish anyway', servo_message,
            )
    else:
        logging.warning('servo services not ready; proceeding with publish anyway')

    def replay() -> None:
        # 토픽별 publisher 확보 — 메시지 타입은 first_message 에서 유도.
        publisher_dict: dict[str, Publisher] = {
            s.topic_name: node.create_publisher(type(s.first_message), s.topic_name, 10)
            for s in streams
        }

        # 공통 anchor: 원본 타임라인의 첫 stamp ↔ 재생 시작 시점의 ROS clock.
        # 모든 스레드가 동일 anchor 를 사용해 토픽 간 상대 타이밍을 보존한다.
        first_ns: int = min(_stamp_ns(s.first_message) for s in streams)
        now_at_start_ns: int = node.get_clock().now().nanoseconds
        wall_start = time.monotonic()
        stop_event = threading.Event()
        published_per_topic: dict[str, int] = {s.topic_name: 0 for s in streams}

        # 토픽 간 발행 격리 — 각 ReplayStream 마다 dedicated 스레드를 띄운다.
        # 이미지처럼 큰 메시지의 DDS 전송 비용이 다른 토픽의 cadence 를 늦추지
        # 않도록 (특히 MoveIt Servo 의 incoming_command_timeout 트립을 방지).
        threads: list[threading.Thread] = []
        for s in streams:
            t = threading.Thread(
                target=_publish_one_stream_loop,
                args=(s, publisher_dict[s.topic_name], first_ns, wall_start,
                      now_at_start_ns, stop_event, published_per_topic),
                name=f'replay-pub-{s.topic_name}', daemon=True,
            )
            threads.append(t)
            t.start()

        try:
            for t in threads:
                t.join()
        finally:
            stop_event.set()
            for t in threads:
                t.join(timeout=2.0)

        per_topic_published = ', '.join(
            f'{t}={n}' for t, n in published_per_topic.items()
        )
        logging.info('replay done: published per topic = [%s]', per_topic_published)

    try:
        replay()
    except KeyboardInterrupt:
        logging.info('replay interrupted by user')
    finally:
        _close_all(streams)
        node.destroy_node()
        rclpy.try_shutdown()

    logging.info('replay finished')
    return 0


def _close_all(streams: list[ReplayStream]) -> None:
    """모든 stream 의 close() 를 호출한다 (예외는 삼킨다)."""
    for s in streams:
        try:
            s.close()
        except Exception:   # noqa: BLE001
            logging.exception('stream close failed: topic=%s', s.topic_name)


def _publish_one_stream_loop(stream: ReplayStream, pub: Publisher,
                             first_ns: int, wall_start: float, now_at_start_ns: int,
                             stop_event: threading.Event,
                             counters: dict[str, int]) -> None:
    """단일 ``ReplayStream`` 을 자체 스레드에서 원본 타이밍대로 발행한다.

    ``counters`` 는 토픽별 카운트 dict 으로, 본 스레드는 자기 키
    (``stream.topic_name``) 만 갱신한다 (CPython dict 단일 키 갱신은 GIL
    하에서 안전).
    """
    topic_name = stream.topic_name
    try:
        for msg in stream.iterator:
            if stop_event.is_set():
                return
            msg_ns = _stamp_ns(msg)
            wait = wall_start + (msg_ns - first_ns) / 1e9 - time.monotonic()
            if wait > 0:
                if stop_event.wait(timeout=wait):
                    return
            shifted_ns = now_at_start_ns + (msg_ns - first_ns)
            msg.header.stamp.sec = int(shifted_ns // 1_000_000_000)
            msg.header.stamp.nanosec = int(shifted_ns % 1_000_000_000)
            pub.publish(msg)
            counters[topic_name] += 1
    except Exception:   # noqa: BLE001
        logging.exception('publisher thread crashed for topic %s', topic_name)


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='replay',
        description='Replay an episode\'s recorded topics back onto live ROS 2 '
                    'topics with original cadence (requires ROS 2 runtime).',
    )
    add_common_args(p)
    add_config_arg(p, required=False)
    p.add_argument('episode_id', type=int,
                   help='DB id of the episode (sessions.id) to replay')
    p.add_argument('--topic', nargs='+',
                   default=['/servo_node/delta_twist_cmds'],
                   help='one or more target topics. 모든 토픽의 메시지를 stamp '
                        '순으로 merge 하여 해당 publisher 로 재생한다 '
                        '(default: /servo_node/delta_twist_cmds)')
    return p


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    configure_logging(args.log_level)

    config_path = resolve_config_path(args)
    if config_path is None:
        logging.error(
            'no --config given and %s not found in the current working directory',
            DEFAULT_CONFIG_FILENAME)
        return 2

    try:
        return cmd_replay(args, config_path)
    except ImportError as exc:
        logging.error(
            'replay requires ROS 2 runtime (rclpy). source install/setup.bash '
            'first. Original error: %s', exc)
        return 2


__all__ = ['cmd_replay', 'main']


if __name__ == '__main__':
    sys.exit(main())
