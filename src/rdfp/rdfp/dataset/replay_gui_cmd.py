"""Replay 제어용 GUI 노드.

dataset DB 에 적재된 에피소드 목록을 표시하고, 사용자가 선택한 에피소드의
지정 토픽들을 원본 타이밍으로 재발행한다. ``/camera/image_raw`` 토픽을
GUI 영역에 실시간 표시하여 재생 상황을 시각적으로 확인할 수 있다.
"초기화" 버튼은 ``MoveGroupClient.move_to_named_target_async`` 를 호출해
SRDF 의 named target (예: ``ready``) 으로 로봇을 이동시킨다.

스레딩 모델:
    * tkinter mainloop 은 메인 스레드.
    * rclpy executor (MultiThreadedExecutor) 는 백그라운드 스레드.
    * 재생 로직은 worker 스레드에서 발행 시퀀스를 sleep 으로 동기화하면서
      executor 가 publish 를 처리한다 (publisher 자체는 thread-safe).
    * 메인 스레드 ↔ ROS 콜백/워커 간 데이터 교환은 lock + ``Tk.after``.
"""

from __future__ import annotations

from typing import Any, Optional

import argparse
import base64
import logging
import os
import sys
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge, CvBridgeError
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from rdfp.dataset.cli_common import load_dataset_or_fail
from rdfp.dataset.db.connection import open_connection
from rdfp.dataset.db.image_message_reader import is_image_topic_in_db
from rdfp.dataset.db.mp4_image_replayer import Mp4ImageReplayer
from rdfp.dataset.db.registry import (
    IMAGE_MESSAGE_TYPES, MESSAGE_TYPE_REGISTRY, resolve_message_type,
)
from rdfp.dataset.db.topic_message_replayer import TopicMessageReplayer
from rdfp.moveit.move_group_client import MoveGroupClient
from rdfp.moveit.servo_client import ServoClient


_DEFAULT_IMAGE_TOPIC = '/camera/image_raw'
_DEFAULT_INIT_TARGET = 'ready'
# 카메라 이미지 표시 영역의 기본 크기. 720p (1280x720) 를 기본으로 한다.
_DEFAULT_DISPLAY_W = 1280
_DEFAULT_DISPLAY_H = 720
# 이미지 갱신 주기 (ms). 30 ms ≈ 33 Hz; 카메라 publish rate 보다 약간 낮게 잡아도
# 표시 지연이 누적되지 않는다.
_IMAGE_REFRESH_MS = 33
# 좌측 컨트롤 패널의 권장 너비 (px). 윈도우 기본 폭을 결정한다.
_LEFT_PANEL_WIDTH = 460

# replay 시작까지 부여할 prep 지연 (초). 이 시간 동안 디코더 prefetch + DDS
# discovery 가 완료되어, 모든 replayer 가 동시에 시작했을 때 첫 프레임이
# 부드럽게 흐르도록 한다.
_REPLAY_START_DELAY_SEC = 2.0


# ---------------------------------------------------------------------------
# ROS 2 노드.
# ---------------------------------------------------------------------------

class ReplayControlNode(Node):
    """GUI 와 함께 동작하는 replay 제어 노드."""

    def __init__(self, config_path: str) -> None:
        super().__init__('replay_gui')

        # --- 파라미터 ---
        self.declare_parameter('image_topic', _DEFAULT_IMAGE_TOPIC)
        self.declare_parameter('init_named_target', _DEFAULT_INIT_TARGET)
        self._image_topic: str = self.get_parameter('image_topic').value
        self._init_named_target: str = self.get_parameter('init_named_target').value

        # --- DB 설정 로드 ---
        cfg = load_dataset_or_fail(config_path)
        if cfg is None:
            raise RuntimeError(f'failed to load dataset config: {config_path}')
        self._cfg = cfg

        # --- 이미지 표시용 ---
        self._bridge = CvBridge()
        self._latest_frame_lock = threading.Lock()
        self._latest_frame_bgr: Optional[np.ndarray] = None
        self._cv_bridge_fail_logged: set[str] = set()

        self._image_sub = self.create_subscription(
            Image, self._image_topic, self._on_image, qos_profile_sensor_data,
        )

        # --- MoveGroupClient ---
        self._move_group = MoveGroupClient(self)

        # --- ServoClient (replay 시작 직전마다 servo 를 start 하기 위한 핸들).
        # service client 등 상태를 들고 있으므로 인스턴스 자체는 한 번만 만들고
        # 재사용한다.
        self._servo_client: ServoClient = ServoClient.create(self)

        # --- replay worker 상태 ---
        self._replay_thread: Optional[threading.Thread] = None
        self._replay_stop_event: Optional[threading.Event] = None
        # 워커 마지막 종료 사유 (GUI 가 polling).
        self._replay_status_lock = threading.Lock()
        self._replay_status: dict[str, Any] = {'state': 'idle', 'message': ''}

        self.get_logger().info(
            f'ReplayControlNode initialized: image_topic={self._image_topic}, '
            f'init_named_target={self._init_named_target!r}, '
            f'config={config_path}'
        )

    # ----- 이미지 콜백 / 최신 프레임 조회 ---------------------------------

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            encoding = msg.encoding
            if encoding not in self._cv_bridge_fail_logged:
                self._cv_bridge_fail_logged.add(encoding)
                self.get_logger().error(
                    f'cv_bridge conversion failed (encoding={encoding!r}): {exc}'
                )
            return
        with self._latest_frame_lock:
            self._latest_frame_bgr = frame

    def get_latest_frame(self) -> Optional[np.ndarray]:
        """최신 BGR 프레임의 사본을 반환한다 (없으면 None)."""
        with self._latest_frame_lock:
            return None if self._latest_frame_bgr is None else self._latest_frame_bgr.copy()

    # ----- DB 조회 ---------------------------------------------------------

    def list_episodes(self) -> list[dict]:
        """sessions 테이블에서 에피소드를 start_ts ASC 로 조회한다."""
        episodes: list[dict] = []
        with open_connection(self._cfg.db) as conn:
            with conn.cursor() as cur:
                cur.execute(
                    'SELECT id, start_ts, stop_ts, task_label '
                    'FROM sessions ORDER BY start_ts ASC'
                )
                for ep_id, start_ts, stop_ts, task_label in cur.fetchall():
                    duration = (stop_ts - start_ts).total_seconds()
                    episodes.append({
                        'id': int(ep_id),
                        'start_ts': start_ts,
                        'duration_sec': round(duration, 3),
                        'task_label': task_label or '',
                    })
            conn.rollback()
        return episodes

    def list_replayable_topics(self) -> list[str]:
        """`topics` 테이블에서 replay 가능한 토픽 이름을 반환한다.

        message-writer 가 등록된 일반 토픽 (`MESSAGE_TYPE_REGISTRY`) 과
        mp4 + DBMS sidecar 로 재구성 가능한 이미지 토픽 (`IMAGE_MESSAGE_TYPES`)
        을 모두 포함한다.
        """
        result: list[str] = []
        with open_connection(self._cfg.db) as conn:
            with conn.cursor() as cur:
                cur.execute(
                    'SELECT topic_name, topic_type FROM topics ORDER BY topic_name'
                )
                for topic_name, topic_type in cur.fetchall():
                    if (resolve_message_type(topic_type) is not None
                            or topic_type in IMAGE_MESSAGE_TYPES):
                        result.append(str(topic_name))
            conn.rollback()
        return result

    def get_episode_detail(self, episode_id: int) -> Optional[dict]:
        """sessions 한 행을 조회해 dict 로 반환한다 (없으면 None)."""
        with open_connection(self._cfg.db) as conn:
            with conn.cursor() as cur:
                cur.execute(
                    'SELECT id, start_ts, stop_ts, task_label '
                    'FROM sessions WHERE id = %s', (episode_id,),
                )
                row = cur.fetchone()
            conn.rollback()
        if row is None:
            return None
        ep_id, start_ts, stop_ts, task_label = row
        return {
            'id': int(ep_id),
            'start_ts': start_ts,
            'stop_ts': stop_ts,
            'duration_sec': round((stop_ts - start_ts).total_seconds(), 3),
            'task_label': task_label or '',
        }

    def list_topics_for_episode(self, episode_id: int) -> list[dict]:
        """해당 에피소드에 row 가 있는 토픽들을 반환한다.

        registry 의 모든 테이블 + ``image_frames`` (이미지 토픽) 을 순회하면서
        ``episode_id`` 로 필터된 행 수를 topic_id 별로 GROUP BY 하고,
        ``topics`` 테이블과 매핑해 (topic_name, topic_type, message_count,
        table) 형태로 모은다. 결과는 토픽 이름 오름차순 정렬.
        """
        result: list[dict] = []
        with open_connection(self._cfg.db) as conn:
            with conn.cursor() as cur:
                cur.execute('SELECT id, topic_name, topic_type FROM topics')
                id_to_topic: dict[int, tuple[str, str]] = {
                    int(tid): (str(name), str(typ))
                    for tid, name, typ in cur.fetchall()
                }

            # message-writer 토픽: 각 binding 의 테이블에서 카운트.
            for type_name, binding in MESSAGE_TYPE_REGISTRY.items():
                table = binding.table
                try:
                    with conn.cursor() as cur:
                        cur.execute(
                            f'SELECT topic_id, COUNT(*) FROM {table} '
                            'WHERE episode_id = %s GROUP BY topic_id',
                            (episode_id,),
                        )
                        rows = cur.fetchall()
                except Exception as e:   # noqa: BLE001
                    # 테이블 자체가 없거나 컬럼 누락 등은 조용히 건너뛴다.
                    self.get_logger().debug(f'skip table {table}: {e}')
                    conn.rollback()
                    continue
                for topic_id, count in rows:
                    info = id_to_topic.get(int(topic_id))
                    if info is None:
                        continue
                    name, typ = info
                    result.append({
                        'topic_name': name, 'topic_type': typ,
                        'message_count': int(count), 'table': table,
                    })

            # 이미지 토픽: image_frames 에서 frame 수를 카운트.
            try:
                with conn.cursor() as cur:
                    cur.execute(
                        'SELECT topic_id, COUNT(*) FROM image_frames '
                        'WHERE episode_id = %s GROUP BY topic_id',
                        (episode_id,),
                    )
                    img_rows = cur.fetchall()
            except Exception as e:   # noqa: BLE001
                self.get_logger().debug(f'skip table image_frames: {e}')
                conn.rollback()
                img_rows = []
            for topic_id, count in img_rows:
                info = id_to_topic.get(int(topic_id))
                if info is None:
                    continue
                name, typ = info
                result.append({
                    'topic_name': name, 'topic_type': typ,
                    'message_count': int(count), 'table': 'image_frames',
                })
            conn.rollback()

        result.sort(key=lambda d: d['topic_name'])
        return result

    # ----- Replay --------------------------------------------------------

    def replay_state(self) -> dict[str, Any]:
        """현재 replay 상태(`state`, `message`) 의 사본을 반환한다."""
        with self._replay_status_lock:
            return dict(self._replay_status)

    def _set_replay_status(self, state: str, message: str = '') -> None:
        with self._replay_status_lock:
            self._replay_status = {'state': state, 'message': message}

    def is_replay_running(self) -> bool:
        return self._replay_thread is not None and self._replay_thread.is_alive()

    def start_replay(self, episode_id: int, topics: list[str]) -> None:
        """worker 스레드를 띄워 비동기로 replay 를 시작한다 (즉시 반환).

        실제 DB 조회 / mp4 디코딩 / publish 모두 worker 스레드에서 수행된다.
        본 메서드는 기본적인 인자 검증만 하고 즉시 반환하므로, GUI 메인
        스레드는 무거운 이미지 디코딩에 의해 블록되지 않는다.

        Raises:
            RuntimeError: 다른 replay 가 이미 실행 중일 때.
            ValueError: 인자가 비었을 때.
        """
        if self.is_replay_running():
            raise RuntimeError('replay is already running')
        if not topics:
            raise ValueError('no topics selected for replay')

        self._replay_stop_event = threading.Event()
        self._set_replay_status(
            'starting', f'episode={episode_id}, topics={len(topics)}',
        )
        self._replay_thread = threading.Thread(
            target=self._replay_worker,
            args=(episode_id, list(topics), self._replay_stop_event),
            name='replay-worker',
            daemon=True,
        )
        self._replay_thread.start()

    def stop_replay(self) -> None:
        """진행 중인 replay 에 중단 신호를 보낸다 (즉시 반환)."""
        if self._replay_stop_event is not None:
            self._replay_stop_event.set()

    def _ensure_servo_started(self, timeout_sec: float = 5.0) -> None:
        """replay_cmd 와 동일한 규약으로 replay 시작 직전에 servo 를 기동한다.

        본 노드는 ``MultiThreadedExecutor`` 가 다른 스레드에서 spin 중이므로
        ``ServoClient.wait_for_services_ready`` / ``ServoClient.start`` 의
        ``rclpy.spin_once`` 기반 동기 헬퍼를 그대로 호출하면 "node already
        added to executor" 충돌이 난다. 따라서 ``ServoClient`` 가 보유한
        service client 만 재사용하고, 동기 부분은 ``service_is_ready`` 폴링
        + ``Future.add_done_callback`` + ``Event.wait`` 패턴으로 직접 구현한다.

        실패는 경고 로그만 남기고 무시한다 (이미 servo 가 외부에서 기동된
        시나리오도 지원한다 — replay_cmd.cmd_replay 와 동일한 규약).
        """
        deadline = time.monotonic() + timeout_sec
        start_client = self._servo_client.start_client

        # 1) start_servo 서비스 준비 대기 (non-spinning poll).
        while not start_client.service_is_ready():
            if time.monotonic() >= deadline:
                self.get_logger().warning(
                    'servo services not ready; proceeding with publish anyway'
                )
                return
            time.sleep(0.1)

        # 2) start_servo 호출 (executor 가 다른 스레드에서 future 를 채운다).
        try:
            from std_srvs.srv import Trigger
            future = start_client.call_async(Trigger.Request())
        except Exception as exc:   # noqa: BLE001
            self.get_logger().warning(
                f'servo start call_async failed ({exc}); proceeding anyway'
            )
            return

        done_event = threading.Event()
        future.add_done_callback(lambda _f: done_event.set())
        remains = max(0.1, deadline - time.monotonic())
        if not done_event.wait(timeout=remains):
            self.get_logger().warning(
                f'servo start timed out after {timeout_sec:.1f}s; proceeding anyway'
            )
            return

        try:
            response = future.result()
        except Exception as exc:   # noqa: BLE001
            self.get_logger().warning(
                f'servo start failed ({exc}); proceeding with publish anyway'
            )
            return

        if response.success:
            self.get_logger().info(f'servo started: {response.message}')
        else:
            self.get_logger().warning(
                f'servo start failed ({response.message}); '
                'proceeding with publish anyway'
            )

    def _replay_worker(self, episode_id: int, topics: list[str],
                       stop_event: threading.Event) -> None:
        """워커 스레드: replayer 구성 → servo → start → 모니터링.

        본 스레드는 다음을 수행한다:
            1) 토픽 분류 후 이미지는 토픽당 ``Mp4ImageReplayer`` 1 개,
               그 외는 모아서 ``TopicMessageReplayer`` 1 개를 만든다.
            2) servo 를 best-effort 로 기동한다 (실패 시 경고만).
            3) 모든 replayer 의 ``get_first_stamp`` 중 가장 이른 값을
               ``first_history_time`` anchor 로 잡고, ``now + 3s`` 를
               ``start_time`` 으로 모든 replayer 의 ``start()`` 를 호출한다.
               각 replayer 가 자체 워커로 wait 후 동시에 publish 시작.
            4) 모든 replayer 가 종료하거나 ``stop_event`` 가 셋 될 때까지
               polling. 중단 시 모든 replayer 에 ``stop()`` 신호.
            5) finally 에서 모든 replayer 의 ``close()`` 를 호출해 publisher
               까지 정리 (반복 replay 시 publisher 누수 방지).
        """
        image_replayers: list[Mp4ImageReplayer] = []
        msg_replayer: Optional[TopicMessageReplayer] = None
        try:
            # 1) replayer 구성.
            try:
                image_replayers, msg_replayer = self._build_replayers(
                    episode_id, topics)
            except ValueError as exc:
                self.get_logger().error(f'failed to build replayers: {exc}')
                self._set_replay_status('error', str(exc))
                return
            except Exception as exc:   # noqa: BLE001
                self.get_logger().exception('failed to build replayers')
                self._set_replay_status('error', str(exc))
                return

            all_replayers: list[Any] = list(image_replayers)
            if msg_replayer is not None:
                all_replayers.append(msg_replayer)
            if not all_replayers:
                msg = (f'episode {episode_id} has no replayable replayers '
                       f'for topics {topics}')
                self.get_logger().warning(msg)
                self._set_replay_status('failed', msg)
                return

            self.get_logger().info(
                f'replaying episode {episode_id}: {len(image_replayers)} image '
                f'replayer(s) + {1 if msg_replayer else 0} message replayer; '
                f'expected_count_total={self._sum_expected(all_replayers)}'
            )

            # 2) servo 기동 (실패 시 경고만 남기고 계속).
            self._ensure_servo_started(timeout_sec=5.0)

            # 3) anchor 계산 + 모든 replayer 동시 start.
            first_history_time = self._earliest_stamp(all_replayers)
            start_time = (self.get_clock().now()
                          + Duration(seconds=_REPLAY_START_DELAY_SEC)).to_msg()
            self.get_logger().info(
                f'start_time={start_time.sec}.{start_time.nanosec:09d} '
                f'(now+{_REPLAY_START_DELAY_SEC:.1f}s), '
                f'first_history={first_history_time.sec}.'
                f'{first_history_time.nanosec:09d}'
            )
            for r in all_replayers:
                r.start(start_time, first_history_time)

            self._set_replay_status(
                'running', f'episode={episode_id}, replayers={len(all_replayers)}')

            # 4) 모니터링 — 모든 replayer 가 종료할 때까지 0.2 초 단위로 polling.
            while any(r.is_running for r in all_replayers):
                if stop_event.wait(timeout=0.2):
                    for r in all_replayers:
                        r.stop()
                    break

            # 모든 워커가 자연 종료했더라도 join 으로 race 차단.
            for r in all_replayers:
                r.join(timeout=2.0)

            published = sum(r.published_count for r in all_replayers)
            per_replayer = ', '.join(
                f'{self._replayer_label(r)}={r.published_count}'
                for r in all_replayers)
            if stop_event.is_set():
                self._set_replay_status(
                    'stopped', f'published {published} [{per_replayer}]')
                self.get_logger().info(
                    f'replay stopped by user: {published} published '
                    f'[{per_replayer}]')
            else:
                self._set_replay_status(
                    'done', f'published {published} [{per_replayer}]')
                self.get_logger().info(
                    f'replay finished: {published} message(s) published '
                    f'[{per_replayer}]')
        except Exception as exc:   # noqa: BLE001
            self.get_logger().exception('replay worker crashed')
            self._set_replay_status('error', str(exc))
        finally:
            # 모든 replayer close — publisher destroy 포함 (누수 방지).
            for r in image_replayers:
                try:
                    r.close()
                except Exception:   # noqa: BLE001
                    self.get_logger().exception(
                        f'replayer close failed: topic={r.topic_name}')
            if msg_replayer is not None:
                try:
                    msg_replayer.close()
                except Exception:   # noqa: BLE001
                    self.get_logger().exception('msg replayer close failed')

    def _build_replayers(self, episode_id: int,
                         topics: list[str]
                         ) -> tuple[list[Mp4ImageReplayer],
                                    Optional[TopicMessageReplayer]]:
        """토픽을 이미지/비이미지로 분류해 각각 replayer 를 만든다.

        episode 미존재는 ValueError. 개별 토픽 실패 (빈 source / mp4 미존재 /
        binding 미등록 등) 는 warning 후 skip 하여 부분 성공을 허용한다.
        """
        image_replayers: list[Mp4ImageReplayer] = []
        msg_topics: list[str] = []
        mp4_root = (Path(self._cfg.output_mp4_dir)
                    if self._cfg.output_mp4_dir else None)

        with open_connection(self._cfg.db) as conn:
            with conn.cursor() as cur:
                cur.execute('SELECT 1 FROM sessions WHERE id = %s', (episode_id,))
                if cur.fetchone() is None:
                    raise ValueError(f'episode id {episode_id} not found')

            # 1) 토픽 분류 + 이미지 토픽은 토픽당 Mp4ImageReplayer 1 개.
            for topic in topics:
                if is_image_topic_in_db(conn, topic):
                    if mp4_root is None:
                        self.get_logger().warning(
                            f'skip image topic {topic}: output_mp4_dir not set')
                        continue
                    try:
                        r = Mp4ImageReplayer(
                            self, conn, episode_id=episode_id,
                            topic_name=topic, mp4_root=mp4_root)
                        image_replayers.append(r)
                    except (ValueError, RuntimeError) as e:
                        self.get_logger().warning(
                            f'skip image topic {topic}: {e}')
                else:
                    msg_topics.append(topic)

            # 2) 비이미지 토픽들은 모아서 TopicMessageReplayer 1 개.
            msg_replayer: Optional[TopicMessageReplayer] = None
            if msg_topics:
                try:
                    msg_replayer = TopicMessageReplayer(
                        self, conn, episode_id=episode_id,
                        topic_names=msg_topics)
                except (ValueError, RuntimeError) as e:
                    self.get_logger().warning(
                        f'skip message replayer: {e} (topics={msg_topics})')

            conn.rollback()
        return image_replayers, msg_replayer

    @staticmethod
    def _earliest_stamp(replayers: list[Any]) -> Time:
        """모든 replayer 의 ``get_first_stamp()`` 중 가장 이른 값을 반환한다.

        모든 replayer 가 동일한 anchor 를 받아야 토픽 간 상대 cadence 가
        보존된다. 단일 토픽의 stamp 시간축 상의 최소값.
        """
        stamps = [r.get_first_stamp() for r in replayers]
        return min(stamps,
                   key=lambda t: int(t.sec) * 1_000_000_000 + int(t.nanosec))

    @staticmethod
    def _sum_expected(replayers: list[Any]) -> int:
        """진단용 — 모든 replayer 의 expected_count 합."""
        return sum(int(r.expected_count) for r in replayers)

    @staticmethod
    def _replayer_label(replayer: Any) -> str:
        """로그용 라벨 — 이미지면 토픽 이름, 메시지 replayer 면 'msg'."""
        if isinstance(replayer, Mp4ImageReplayer):
            return replayer.topic_name
        return 'msg'

    # ----- Initialize (move_to_named_target) ------------------------------

    def initialize_robot_async(self) -> None:
        """SRDF named target 으로 비동기 이동을 시작한다.

        실패는 로그/상태로만 표시한다 (반환값 없음).
        """
        name = self._init_named_target
        self.get_logger().info(f'initialize: moving to named target {name!r}...')
        try:
            future = self._move_group.move_to_named_target_async(name)
        except Exception as exc:   # noqa: BLE001
            self.get_logger().error(f'initialize failed to start: {exc}')
            return

        def _on_done(_fut) -> None:
            try:
                _fut.result()
                self.get_logger().info(f'initialize done: reached {name!r}')
            except Exception as exc:   # noqa: BLE001
                self.get_logger().error(f'initialize failed: {exc}')

        future.add_done_callback(_on_done)

    # ----- 정리 -----------------------------------------------------------

    def shutdown(self) -> None:
        self.stop_replay()
        if self._replay_thread is not None:
            self._replay_thread.join(timeout=2.0)
        try:
            self._move_group.close()
        except Exception:   # noqa: BLE001
            pass


# ---------------------------------------------------------------------------
# tkinter GUI.
# ---------------------------------------------------------------------------

class EpisodeDetailWindow(tk.Toplevel):
    """선택된 에피소드의 상세 정보 + 포함 토픽 목록을 보여주는 별도 윈도우."""

    def __init__(self, parent: tk.Misc, detail: dict, topics: list[dict]) -> None:
        super().__init__(parent)
        self.title(f'Episode {detail["id"]} — detail')
        self.geometry('720x520')
        # 메인 윈도우와 독립적으로 동작 (모달 아님 — 여러 episode 동시 비교 가능).
        self.transient(parent)

        # --- 상단: 에피소드 메타 ---
        meta_frame = ttk.LabelFrame(self, text='Session', padding=8)
        meta_frame.pack(fill='x', padx=8, pady=(8, 4))
        rows = [
            ('ID', str(detail['id'])),
            ('Start TS', _format_ts(detail['start_ts'])),
            ('Stop TS', _format_ts(detail['stop_ts'])),
            ('Duration', f'{detail["duration_sec"]:.3f} s'),
            ('Task label', detail['task_label'] or '(none)'),
        ]
        for r, (k, v) in enumerate(rows):
            ttk.Label(meta_frame, text=k + ':',
                      font=('TkDefaultFont', 9, 'bold')).grid(
                row=r, column=0, sticky='w', padx=(0, 8), pady=2)
            ttk.Label(meta_frame, text=v).grid(row=r, column=1, sticky='w', pady=2)
        meta_frame.columnconfigure(1, weight=1)

        # --- 중단: 토픽 테이블 ---
        topic_frame = ttk.LabelFrame(
            self, text=f'Topics in this episode ({len(topics)})', padding=4,
        )
        topic_frame.pack(fill='both', expand=True, padx=8, pady=(4, 4))

        cols = ('topic', 'type', 'count', 'table')
        tree = ttk.Treeview(topic_frame, columns=cols, show='headings')
        for c, w, anchor in (
            ('topic', 240, 'w'),
            ('type', 220, 'w'),
            ('count', 80, 'e'),
            ('table', 140, 'w'),
        ):
            tree.heading(c, text=c.upper())
            tree.column(c, width=w, anchor=anchor)
        tree.pack(side='left', fill='both', expand=True)
        sb = ttk.Scrollbar(topic_frame, orient='vertical', command=tree.yview)
        tree.configure(yscrollcommand=sb.set)
        sb.pack(side='right', fill='y')

        if not topics:
            tree.insert('', 'end', values=('(no recorded topics)', '', '', ''))
        else:
            for t in topics:
                tree.insert('', 'end', values=(
                    t['topic_name'], t['topic_type'],
                    f'{t["message_count"]:,}', t['table'],
                ))

        # --- 하단: 닫기 버튼 ---
        btn_row = ttk.Frame(self)
        btn_row.pack(fill='x', padx=8, pady=(4, 8))
        ttk.Button(btn_row, text='Close', command=self.destroy).pack(side='right')


def _format_ts(ts: Any) -> str:
    """`datetime` / 그 외 객체를 사람이 읽기 쉬운 문자열로 변환한다."""
    if isinstance(ts, datetime):
        return ts.strftime('%Y-%m-%d %H:%M:%S')
    return str(ts)


class ReplayControlGui:
    """tkinter mainloop 을 호스팅하는 GUI 컨트롤러."""

    def __init__(self, root: tk.Tk, node: ReplayControlNode) -> None:
        self._root = root
        self._node = node

        self._root.title('rdfp Replay GUI')
        # 좌측 컨트롤 + 우측 이미지(1280x720) + 상단/하단 패딩 + 상태바를
        # 한 화면에 담을 수 있는 기본 윈도우 크기.
        win_w = _LEFT_PANEL_WIDTH + _DEFAULT_DISPLAY_W + 40
        win_h = _DEFAULT_DISPLAY_H + 100
        self._root.geometry(f'{win_w}x{win_h}')

        # 상태 변수.
        self._episodes: list[dict] = []
        self._available_topics: list[str] = []
        # 토픽별 체크 상태. _refresh_topics 가 갱신한다.
        self._topic_vars: dict[str, tk.BooleanVar] = {}
        self._image_imgtk: Optional[tk.PhotoImage] = None   # GC 방지 보유.

        self._build_layout()
        # 부팅 시점에 DB 조회.
        self._refresh_episodes()
        self._refresh_topics()

        # 이미지 / 상태 주기 갱신.
        self._root.after(_IMAGE_REFRESH_MS, self._tick_image)
        self._root.after(500, self._tick_status)

    # ----- 레이아웃 ----------------------------------------------------

    def _build_layout(self) -> None:
        # 좌: 에피소드 + 토픽 + 버튼 (고정 폭) / 우: 이미지(1280x720) + 상태.
        # 좌측은 weight=0 + minsize 로 컨트롤 폭을 고정하고, 우측이 윈도우 폭
        # 변화를 흡수해 이미지가 더 큰 모니터에서 자연스럽게 확장되도록 한다.
        self._root.columnconfigure(0, weight=0, minsize=_LEFT_PANEL_WIDTH)
        self._root.columnconfigure(1, weight=1, minsize=_DEFAULT_DISPLAY_W + 16)
        self._root.rowconfigure(0, weight=1)

        left = ttk.Frame(self._root, padding=8, width=_LEFT_PANEL_WIDTH)
        left.grid(row=0, column=0, sticky='nsew')
        # 좌측 컬럼은 자식 크기에 따라 좁아지지 않도록 폭을 잠근다.
        left.grid_propagate(False)
        right = ttk.Frame(self._root, padding=8)
        right.grid(row=0, column=1, sticky='nsew')

        # --- 좌측: 에피소드 목록 ---
        ep_frame = ttk.LabelFrame(left, text='Episodes', padding=4)
        ep_frame.pack(fill='both', expand=True)

        cols = ('id', 'start_ts', 'duration', 'label')
        self._ep_tree = ttk.Treeview(ep_frame, columns=cols, show='headings', height=12)
        for c, w, anchor in (
            ('id', 60, 'e'), ('start_ts', 160, 'w'),
            ('duration', 80, 'e'), ('label', 160, 'w'),
        ):
            self._ep_tree.heading(c, text=c.upper())
            self._ep_tree.column(c, width=w, anchor=anchor)
        self._ep_tree.pack(side='left', fill='both', expand=True)
        sb = ttk.Scrollbar(ep_frame, orient='vertical', command=self._ep_tree.yview)
        self._ep_tree.configure(yscrollcommand=sb.set)
        sb.pack(side='right', fill='y')
        # 행 더블클릭 → 에피소드 상세 윈도우.
        self._ep_tree.bind('<Double-Button-1>', self._on_episode_double_click)

        # --- 좌측: 토픽 선택 (체크박스 리스트, 스크롤 가능) ---
        tp_frame = ttk.LabelFrame(
            left, text='Topics to replay', padding=4,
        )
        tp_frame.pack(fill='both', expand=False, pady=(8, 0))

        # Canvas + 내부 Frame 패턴으로 스크롤 가능한 체크박스 컨테이너를 만든다.
        canvas = tk.Canvas(tp_frame, height=180, highlightthickness=0)
        canvas.pack(side='left', fill='both', expand=True)
        sb2 = ttk.Scrollbar(tp_frame, orient='vertical', command=canvas.yview)
        sb2.pack(side='right', fill='y')
        canvas.configure(yscrollcommand=sb2.set)

        self._topic_check_frame = ttk.Frame(canvas)
        self._topic_check_window = canvas.create_window(
            (0, 0), window=self._topic_check_frame, anchor='nw',
        )
        # Frame 크기 변화 → Canvas scrollregion 갱신.
        self._topic_check_frame.bind(
            '<Configure>',
            lambda e: canvas.configure(scrollregion=canvas.bbox('all')),
        )
        # Canvas 폭 변화 → 내부 Frame 폭도 맞춰 늘려 한 줄 한 항목으로 표시.
        canvas.bind(
            '<Configure>',
            lambda e: canvas.itemconfigure(self._topic_check_window, width=e.width),
        )
        self._topic_canvas = canvas

        # 마우스 휠 스크롤. Windows/macOS 는 <MouseWheel> + delta, X11/Linux 는
        # <Button-4>(up) / <Button-5>(down) 로 분기된다. 캔버스 자체와 내부 Frame,
        # 그 안에 들어갈 Checkbutton 들에서도 휠 이벤트가 캔버스로 라우팅되도록
        # `<Enter>` 시 root 레벨에 바인딩하고 `<Leave>` 시 해제한다.
        self._bind_mousewheel(canvas)

        # --- 좌측: 토픽 일괄 선택 행 ---
        topic_btn_row = ttk.Frame(left)
        topic_btn_row.pack(fill='x', pady=(4, 0))
        ttk.Button(topic_btn_row, text='Select all',
                   command=self._select_all_topics).pack(side='left')
        ttk.Button(topic_btn_row, text='Clear all',
                   command=self._select_no_topics).pack(side='left', padx=(8, 0))

        # --- 좌측: 실행 버튼 행 ---
        btn_row = ttk.Frame(left)
        btn_row.pack(fill='x', pady=(8, 0))
        ttk.Button(btn_row, text='Refresh', command=self._refresh_all).pack(side='left')
        self._replay_btn = ttk.Button(btn_row, text='Replay',
                                      command=self._on_replay_clicked)
        self._replay_btn.pack(side='left', padx=(8, 0))
        self._stop_btn = ttk.Button(btn_row, text='Stop',
                                    command=self._on_stop_clicked, state='disabled')
        self._stop_btn.pack(side='left', padx=(8, 0))
        ttk.Button(btn_row, text='위치 초기화',
                   command=self._on_init_clicked).pack(side='right')

        # --- 우측: 이미지 표시 (기본 1280x720 보장) ---
        img_frame = ttk.LabelFrame(
            right, text=f'Camera ({self._node._image_topic})', padding=4,
            width=_DEFAULT_DISPLAY_W, height=_DEFAULT_DISPLAY_H,
        )
        img_frame.pack(fill='both', expand=True)
        # 자식이 작아도 LabelFrame 이 1280x720 크기를 유지하도록 propagate off.
        img_frame.pack_propagate(False)
        self._image_label = ttk.Label(img_frame, anchor='center',
                                      text='(no image yet)', background='#222',
                                      foreground='#888')
        self._image_label.pack(fill='both', expand=True)

        # --- 우측: 상태 라벨 ---
        st_frame = ttk.LabelFrame(right, text='Status', padding=4)
        st_frame.pack(fill='x', pady=(8, 0))
        self._status_var = tk.StringVar(value='idle')
        ttk.Label(st_frame, textvariable=self._status_var).pack(anchor='w')

    # ----- 데이터 로드 -------------------------------------------------

    def _refresh_all(self) -> None:
        self._refresh_episodes()
        self._refresh_topics()

    def _refresh_episodes(self) -> None:
        try:
            self._episodes = self._node.list_episodes()
        except Exception as exc:   # noqa: BLE001
            messagebox.showerror('DB error', f'failed to list episodes: {exc}')
            self._episodes = []
        self._ep_tree.delete(*self._ep_tree.get_children())
        for e in self._episodes:
            ts = e['start_ts']
            ts_str = (
                ts.strftime('%Y-%m-%d %H:%M:%S')
                if isinstance(ts, datetime)
                else str(ts)
            )
            self._ep_tree.insert(
                '', 'end', iid=str(e['id']),
                values=(e['id'], ts_str, f'{e["duration_sec"]:.3f}', e['task_label']),
            )

    def _refresh_topics(self) -> None:
        try:
            self._available_topics = self._node.list_replayable_topics()
        except Exception as exc:   # noqa: BLE001
            messagebox.showerror('DB error', f'failed to list topics: {exc}')
            self._available_topics = []

        # 기존 위젯/변수 정리.
        for child in self._topic_check_frame.winfo_children():
            child.destroy()
        self._topic_vars.clear()

        # 토픽별 체크박스 동적 생성. 기본값은 모두 미선택 (사용자가 명시적으로
        # 골라야 의도하지 않은 토픽이 재생되지 않는다).
        for t in self._available_topics:
            var = tk.BooleanVar(value=False)
            self._topic_vars[t] = var
            ttk.Checkbutton(
                self._topic_check_frame, text=t, variable=var,
            ).pack(anchor='w', padx=4, pady=1, fill='x')

        # scrollregion 즉시 반영 (Configure 이벤트 대기 없이).
        self._topic_check_frame.update_idletasks()
        self._topic_canvas.configure(scrollregion=self._topic_canvas.bbox('all'))

    def _select_all_topics(self) -> None:
        for var in self._topic_vars.values():
            var.set(True)

    def _select_no_topics(self) -> None:
        for var in self._topic_vars.values():
            var.set(False)

    # ----- 마우스 휠 스크롤 -------------------------------------------

    def _bind_mousewheel(self, canvas: tk.Canvas) -> None:
        """캔버스에 마우스 휠 스크롤을 연결한다 (플랫폼 분기 포함).

        Windows/macOS 는 ``<MouseWheel>`` 이벤트에 부호 있는 `delta` 가
        실리고, X11/Linux 는 별도의 ``<Button-4>`` (up) / ``<Button-5>``
        (down) 이벤트로 들어온다. 자식 위젯 (Checkbutton 등) 위에서도
        휠이 동작하도록 마우스 진입 시점에 root 레벨에 바인딩하고, 영역을
        벗어나면 해제하여 다른 스크롤 가능 컨테이너와 충돌하지 않게 한다.
        """
        def _on_mousewheel(event: tk.Event) -> str:   # type: ignore[type-arg]
            # delta 부호: Windows/macOS 는 ±120 단위, macOS 는 ±1 일 수도 있음.
            if event.delta != 0:
                step = -1 if event.delta > 0 else 1
            elif getattr(event, 'num', None) == 4:
                step = -1   # Linux: scroll up
            elif getattr(event, 'num', None) == 5:
                step = 1    # Linux: scroll down
            else:
                return ''
            canvas.yview_scroll(step, 'units')
            return 'break'

        def _on_enter(_e: tk.Event) -> None:   # type: ignore[type-arg]
            # bind_all 로 자식 위젯 (Checkbutton) 위에서도 휠을 가로챈다.
            canvas.bind_all('<MouseWheel>', _on_mousewheel)
            canvas.bind_all('<Button-4>', _on_mousewheel)
            canvas.bind_all('<Button-5>', _on_mousewheel)

        def _on_leave(_e: tk.Event) -> None:   # type: ignore[type-arg]
            canvas.unbind_all('<MouseWheel>')
            canvas.unbind_all('<Button-4>')
            canvas.unbind_all('<Button-5>')

        canvas.bind('<Enter>', _on_enter)
        canvas.bind('<Leave>', _on_leave)

    # ----- 버튼 핸들러 -------------------------------------------------

    def _selected_episode_id(self) -> Optional[int]:
        sel = self._ep_tree.selection()
        if not sel:
            return None
        return int(sel[0])

    def _selected_topics(self) -> list[str]:
        # 체크박스가 켜진 토픽만, 원래 표시 순서를 유지하여 반환한다.
        return [t for t in self._available_topics if self._topic_vars[t].get()]

    def _on_replay_clicked(self) -> None:
        ep_id = self._selected_episode_id()
        if ep_id is None:
            messagebox.showwarning('Replay', 'Select an episode first.')
            return
        topics = self._selected_topics()
        if not topics:
            messagebox.showwarning('Replay', 'Select at least one topic.')
            return
        try:
            self._node.start_replay(ep_id, topics)
        except (RuntimeError, ValueError) as exc:
            messagebox.showerror('Replay', str(exc))
            return
        self._replay_btn.config(state='disabled')
        self._stop_btn.config(state='normal')

    def _on_stop_clicked(self) -> None:
        self._node.stop_replay()

    def _on_init_clicked(self) -> None:
        # async — 결과는 ROS 로그로 노출되며, GUI 는 즉시 반환한다.
        self._node.initialize_robot_async()

    def _on_episode_double_click(self, _event: tk.Event) -> None:   # type: ignore[type-arg]
        ep_id = self._selected_episode_id()
        if ep_id is None:
            return
        try:
            detail = self._node.get_episode_detail(ep_id)
            topics = self._node.list_topics_for_episode(ep_id)
        except Exception as exc:   # noqa: BLE001
            messagebox.showerror('Episode detail',
                                 f'failed to load detail: {exc}')
            return
        if detail is None:
            messagebox.showerror('Episode detail',
                                 f'episode id {ep_id} not found')
            return
        EpisodeDetailWindow(self._root, detail, topics)

    # ----- 주기 콜백 ---------------------------------------------------

    def _tick_image(self) -> None:
        try:
            frame = self._node.get_latest_frame()
            if frame is not None:
                self._render_frame(frame)
        finally:
            self._root.after(_IMAGE_REFRESH_MS, self._tick_image)

    def _render_frame(self, frame_bgr: np.ndarray) -> None:
        # 라벨의 현재 크기에 맞춰 비율을 유지하며 리사이즈.
        target_w = max(self._image_label.winfo_width(), _DEFAULT_DISPLAY_W)
        target_h = max(self._image_label.winfo_height(), _DEFAULT_DISPLAY_H)
        h, w = frame_bgr.shape[:2]
        scale = min(target_w / max(w, 1), target_h / max(h, 1))
        if scale > 0 and scale != 1.0:
            frame_bgr = cv2.resize(
                frame_bgr,
                (max(1, int(w * scale)), max(1, int(h * scale))),
                interpolation=cv2.INTER_AREA,
            )

        ok, buf = cv2.imencode('.png', frame_bgr)
        if not ok:
            return
        b64 = base64.b64encode(buf.tobytes())
        # 새 PhotoImage 를 만들어 라벨에 갱신. 이전 객체는 할당 해제로 GC.
        self._image_imgtk = tk.PhotoImage(data=b64)
        self._image_label.configure(image=self._image_imgtk, text='')

    def _tick_status(self) -> None:
        try:
            st = self._node.replay_state()
            running = self._node.is_replay_running()
            self._status_var.set(f'replay: {st["state"]} — {st["message"]}')
            if not running:
                self._replay_btn.config(state='normal')
                self._stop_btn.config(state='disabled')
        finally:
            self._root.after(500, self._tick_status)

    # ----- 진입 -----------------------------------------------------------

    def run(self) -> None:
        self._root.protocol('WM_DELETE_WINDOW', self._on_close)
        self._root.mainloop()

    def _on_close(self) -> None:
        try:
            self._node.stop_replay()
        finally:
            self._root.destroy()


# ---------------------------------------------------------------------------
# Entry point.
# ---------------------------------------------------------------------------

def _resolve_default_config() -> Optional[str]:
    candidate = Path.cwd() / 'dataset_config.yaml'
    return str(candidate) if candidate.is_file() else None


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Replay control GUI node')
    parser.add_argument(
        '--config', default=_resolve_default_config(),
        help='dataset_config.yaml 경로. 미지정 시 cwd 의 dataset_config.yaml.',
    )
    # ros2 run 은 `--ros-args ...` 를 따로 분리하므로 그 외 인자만 처리한다.
    parsed, _ = parser.parse_known_args(args=args)

    if not parsed.config:
        print('[FATAL] --config not given and dataset_config.yaml not found in cwd',
              file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(2)

    node: Optional[ReplayControlNode] = None
    executor: Optional[MultiThreadedExecutor] = None
    spin_thread: Optional[threading.Thread] = None
    try:
        node = ReplayControlNode(parsed.config)
    except Exception as exc:
        print(f'[FATAL] ReplayControlNode init failed: {exc}', file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(1)

    # rclpy executor 를 백그라운드 스레드에서 spin 하고, tkinter 는 메인 스레드.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def _spin() -> None:
        try:
            executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass

    spin_thread = threading.Thread(target=_spin, name='rclpy-executor', daemon=True)
    spin_thread.start()

    try:
        root = tk.Tk()
        gui = ReplayControlGui(root, node)
        gui.run()
    finally:
        try:
            node.shutdown()
        except Exception:   # noqa: BLE001
            pass
        # executor 종료 → spin 스레드 자연 종료.
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
