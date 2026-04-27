#!/usr/bin/env python3
"""Mp4ImageReplayer (push API) 사용 예제.

지정 episode 와 image 토픽에 대해 mp4 + image_streams + image_frames 를
조합하여 ``sensor_msgs/Image`` 메시지를 원본 cadence 로 토픽에 publish 하고,
같은 토픽을 본 프로세스에서 subscribe 하여 OpenCV 윈도우에 표시한다.

전제 조건:
    * dataset import 가 한 번 이상 수행되어 image_streams / image_frames
      테이블에 행이 적재되어 있어야 한다.
    * mp4 파일이 ``output_mp4_dir`` 하위 ``image_streams.mp4_path`` 경로에
      존재해야 한다.
    * 환경변수 ``RDFP_DB_DSN`` (또는 dataset_config.yaml 의 ``db.dsn_env``) 에
      유효한 PostgreSQL DSN 이 설정되어 있어야 한다.

Usage:
    python3 -m rdfp.samples.sample_mp4_image_replayer
    python3 -m rdfp.samples.sample_mp4_image_replayer \\
        --config /etc/rdfp/dataset_config.yaml \\
        --episode-id 42 --topic /camera/image_raw
    python3 -m rdfp.samples.sample_mp4_image_replayer --no-display

키 입력 (display 모드):
    q / ESC — 즉시 종료

Requirements:
    * opencv-python, cv_bridge, sensor_msgs, builtin_interfaces, rclpy
    * psycopg[binary]
    * dataset_config.yaml 또는 RDFP_DB_DSN
"""

from __future__ import annotations

from typing import Optional, Sequence

import argparse
import logging
import sys
import threading
import time
from pathlib import Path

import cv2
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from rdfp.dataset.cli_common import load_dataset_or_fail
from rdfp.dataset.db.connection import open_connection
from rdfp.dataset.db.mp4_image_replayer import Mp4ImageReplayer


_DEFAULT_TOPIC = '/camera/image_raw'


def _to_bgr8(image: Image) -> 'cv2.typing.MatLike':
    """sensor_msgs/Image 를 OpenCV BGR8 ndarray 로 변환.

    cv_bridge 가 있으면 그것을 사용하고, 없으면 image.encoding / data 로부터
    직접 numpy reshape 한다 (Mp4ImageReplayer 가 채우는 인코딩만 지원).
    """
    try:
        from cv_bridge import CvBridge
        return CvBridge().imgmsg_to_cv2(image, desired_encoding='bgr8')
    except Exception:
        import numpy as np

        h, w = int(image.height), int(image.width)
        encoding = str(image.encoding).lower()
        data = bytes(image.data)
        if encoding == 'bgr8':
            return np.frombuffer(data, np.uint8).reshape(h, w, 3).copy()
        if encoding == 'rgb8':
            arr = np.frombuffer(data, np.uint8).reshape(h, w, 3)
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if encoding == 'bgra8':
            arr = np.frombuffer(data, np.uint8).reshape(h, w, 4)
            return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        if encoding == 'rgba8':
            arr = np.frombuffer(data, np.uint8).reshape(h, w, 4)
            return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        if encoding == 'mono8':
            arr = np.frombuffer(data, np.uint8).reshape(h, w)
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        raise RuntimeError(f'unsupported encoding for display fallback: {encoding!r}')


def _draw_overlay(frame, *, idx: int, total: int, stamp_ns: int, fps: float) -> None:
    """프레임 위에 디버그 오버레이 텍스트를 그린다 (in-place)."""
    text_lines = [
        f'frame {idx + 1}/{total}',
        f'stamp {stamp_ns / 1e9:.3f}s',
        f'fps {fps:.1f}',
    ]
    y = 22
    for line in text_lines:
        cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 1, cv2.LINE_AA)
        y += 22


def _resolve_episode_id(conn, requested: Optional[int]) -> int:
    """``--episode-id`` 가 주어지지 않았으면 sessions 의 첫 (가장 오래된) episode 사용."""
    if requested is not None:
        return requested
    with conn.cursor() as cur:
        cur.execute('SELECT id FROM sessions ORDER BY id ASC LIMIT 1')
        row = cur.fetchone()
    if row is None:
        raise RuntimeError(
            'no sessions found in DB; run "dataset import" first or specify '
            '--episode-id explicitly')
    return int(row[0])


class _LatestFrameSink:
    """subscriber 콜백이 채우는 최신 프레임 holder (스레드 안전)."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frame: Optional[Image] = None
        self._received: int = 0

    def set(self, msg: Image) -> None:
        with self._lock:
            self._frame = msg
            self._received += 1

    def take(self) -> Optional[Image]:
        with self._lock:
            f, self._frame = self._frame, None
            return f

    @property
    def received(self) -> int:
        with self._lock:
            return self._received


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', help='dataset_config.yaml 경로 (생략 시 자동 탐색)')
    parser.add_argument('--episode-id', type=int,
                        help='재생할 sessions.id (생략 시 가장 오래된 episode)')
    parser.add_argument('--topic', default=_DEFAULT_TOPIC,
                        help=f'재생할 image 토픽 (default: {_DEFAULT_TOPIC})')
    parser.add_argument('--queue-size', type=int, default=4,
                        help='디코더 → publisher 큐 깊이 (default: 4)')
    parser.add_argument('--qos-depth', type=int, default=10,
                        help='ROS publisher/subscriber 큐 깊이 (default: 10)')
    parser.add_argument('--start-delay', type=float, default=0.5,
                        help='subscribe 가 자리 잡을 시간 (sec). 너무 짧으면 첫 '
                             '몇 프레임 유실 가능 (default: 0.5)')
    parser.add_argument('--window-name', default='Mp4ImageReplayer',
                        help='OpenCV 윈도우 이름')
    parser.add_argument('--no-display', action='store_true',
                        help='화면 출력 없이 처리량만 측정 (헤드리스/CI)')
    parser.add_argument('--no-overlay', action='store_true',
                        help='프레임 위 디버그 텍스트(frame/stamp/fps) 그리지 않음')
    parser.add_argument('--log-level', default='info',
                        choices=['debug', 'info', 'warning', 'error'])
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format='%(asctime)s %(levelname)s %(name)s: %(message)s')

    cfg = load_dataset_or_fail(args.config or '')
    if cfg is None:
        return 2
    if cfg.output_mp4_dir is None:
        logging.error('output_mp4_dir is required in dataset config to play mp4')
        return 2
    mp4_root = Path(cfg.output_mp4_dir)

    # ROS init + node + executor (subscribe 로 자기 publish 를 받기 위해 필수).
    rclpy.init()
    node = rclpy.create_node('sample_mp4_image_replayer')
    sink = _LatestFrameSink()
    node.create_subscription(Image, args.topic, sink.set, qos_profile_sensor_data)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(
        target=executor.spin, name='rclpy-executor', daemon=True)
    spin_thread.start()

    replayer: Optional[Mp4ImageReplayer] = None
    window_open = False
    rc = 0
    try:
        try:
            with open_connection(cfg.db) as conn:
                episode_id = _resolve_episode_id(conn, args.episode_id)
                try:
                    replayer = Mp4ImageReplayer(
                        node, conn, episode_id=episode_id, topic_name=args.topic,
                        mp4_root=mp4_root, decode_queue_size=args.queue_size,
                        publish_queue=args.qos_depth)
                except (ValueError, RuntimeError) as exc:
                    logging.error('failed to open replayer: %s', exc)
                    return 2
                conn.rollback()
        except Exception:
            logging.exception('DB error while preparing replayer')
            return 1

        logging.info(
            'replayer ready: episode=%d topic=%s frames=%d duration=%.3fs '
            'mp4=%s',
            episode_id, args.topic, replayer.expected_count,
            replayer.duration_sec, replayer._mp4_path)

        if not args.no_display:
            cv2.namedWindow(args.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(
                args.window_name, replayer.metadata.resolution.width,
                replayer.metadata.resolution.height)
            window_open = True

        # subscribe wiring 이 자리잡을 짧은 여유 + start_time anchor 잡기.
        # subscribe 직후 publish 하면 첫 몇 프레임을 받지 못할 수 있다 (DDS).
        time.sleep(args.start_delay)

        now_ros = node.get_clock().now().to_msg()
        first_history = replayer.get_first_stamp()
        replayer.start(start_time=now_ros, first_history_time=first_history)
        logging.info(
            'replay started: start_time=%d.%09d first_history=%d.%09d',
            now_ros.sec, now_ros.nanosec, first_history.sec, first_history.nanosec)

        run_start = time.monotonic()
        fps_window_start = run_start
        fps_window_count = 0
        fps_display = 0.0
        displayed = 0

        try:
            while replayer.is_running or sink.received > displayed:
                msg = sink.take()
                if msg is None:
                    time.sleep(0.005)
                    continue

                fps_window_count += 1
                now = time.monotonic()
                elapsed = now - fps_window_start
                if elapsed >= 1.0:
                    fps_display = fps_window_count / elapsed
                    fps_window_start = now
                    fps_window_count = 0
                displayed += 1

                stamp_ns = (int(msg.header.stamp.sec) * 1_000_000_000
                            + int(msg.header.stamp.nanosec))
                if args.no_display:
                    if displayed % 30 == 0:
                        logging.info(
                            'progress: published=%d displayed=%d (fps=%.1f)',
                            replayer.published_count, displayed, fps_display)
                    continue

                try:
                    bgr = _to_bgr8(msg)
                except Exception as exc:   # noqa: BLE001
                    logging.warning('frame conversion failed at idx=%d: %s',
                                    displayed - 1, exc)
                    continue

                if not args.no_overlay:
                    _draw_overlay(
                        bgr, idx=displayed - 1, total=replayer.expected_count,
                        stamp_ns=stamp_ns, fps=fps_display)
                cv2.imshow(args.window_name, bgr)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    logging.info('user requested exit (key=%d)', key)
                    break
        except KeyboardInterrupt:
            logging.info('interrupted by user (Ctrl-C)')

        # 실행 시간이 1초 미만이면 sliding window 가 트립되지 않아 fps_display 가 0
        # 으로 남으므로, 최종 fps 는 항상 displayed/elapsed 로 다시 계산.
        total_elapsed = time.monotonic() - run_start
        final_fps = (displayed / total_elapsed) if total_elapsed > 0 else 0.0
        logging.info(
            'replay finished: published=%d displayed=%d in %.3fs (fps≈%.1f)',
            replayer.published_count, displayed, total_elapsed, final_fps)
    except Exception:
        logging.exception('sample_mp4_image_replayer crashed')
        rc = 1
    finally:
        if window_open:
            cv2.destroyAllWindows()
        if replayer is not None:
            replayer.close()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
