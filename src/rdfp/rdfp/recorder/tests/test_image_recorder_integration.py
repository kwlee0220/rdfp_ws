#!/usr/bin/env python3

"""ImageRecorderNode 통합 테스트.

실제 `FFMpegMp4Recorder` 와 ffmpeg subprocess 를 사용하여 end-to-end 동작을
검증한다. 모든 테스트는 in-process publisher node 를 함께 띄워 합성 프레임을
주입한다. ffmpeg 바이너리가 시스템에 설치되어 있어야 한다.
"""

from __future__ import annotations

from typing import Any, Iterator, Optional

import os
import shutil
import subprocess
import threading
import time

import pytest

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time

from rdfp.recorder.image_recorder_node import ImageRecorderNode

try:
    from rdfp_msgs.srv import StartSession, StopSession
except ImportError:  # pragma: no cover
    StartSession = None  # type: ignore
    StopSession = None  # type: ignore


pytestmark = pytest.mark.skipif(
    shutil.which("ffmpeg") is None or StartSession is None,
    reason="ffmpeg or rdfp_msgs unavailable",
)


# ---------- Synthetic publisher helper ---------------------------------------


class SyntheticImagePublisher(Node):
    """타이머 기반 합성 이미지 발행자.

    주어진 fps / resolution / encoding 으로 단조 증가하는 픽셀 값을 채운
    프레임을 발행한다. 실제 카메라가 없는 환경에서 통합 테스트용으로 사용한다.
    """

    def __init__(
        self,
        topic: str,
        width: int,
        height: int,
        encoding: str,
        fps: float,
    ) -> None:
        super().__init__("synthetic_image_publisher")
        self._width = width
        self._height = height
        self._encoding = encoding
        self._channels = 1 if encoding == "mono8" else 3
        self._counter = 0
        self._publisher = self.create_publisher(
            Image, topic, qos_profile_sensor_data
        )
        self.create_timer(1.0 / fps, self._publish_one)

    def _publish_one(self) -> None:
        msg = Image()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = Time(sec=now.sec, nanosec=now.nanosec)
        msg.header.frame_id = "synthetic"
        msg.width = self._width
        msg.height = self._height
        msg.encoding = self._encoding
        msg.step = self._width * self._channels
        # 매 프레임마다 카운터 값으로 채워 단조 증가 픽셀 패턴 생성
        value = self._counter % 256
        size = self._height * msg.step
        msg.data = bytes([value] * size)
        self._counter += 1
        self._publisher.publish(msg)

    @property
    def published_count(self) -> int:
        return self._counter


# ---------- Fixtures ---------------------------------------------------------


@pytest.fixture(scope="module", autouse=True)
def _rclpy_session() -> Iterator[None]:
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _default_overrides(
    output_dir: Any,
    width: int = 320,
    height: int = 240,
    fps: int = 30,
    pixel_format: str = "bgr8",
    session_prefix: str = "itest",
) -> list[Parameter]:
    return [
        Parameter("output_dir", value=str(output_dir)),
        Parameter("session_prefix", value=session_prefix),
        Parameter("fps", value=fps),
        Parameter("resolution", value=f"{width}x{height}"),
        Parameter("pixel_format", value=pixel_format),
        Parameter("encoder_mode", value="cpu"),  # libx264 강제 → probe 회피
        Parameter("queue_size", value=120),
    ]


def _spin_in_background(executor: MultiThreadedExecutor) -> threading.Thread:
    """executor 를 별도 스레드에서 spin 한다."""
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return thread


def _call_service(
    node: Node,
    srv_type: type,
    srv_name: str,
    request: Any,
    timeout_sec: float = 5.0,
) -> Any:
    """노드의 서비스를 동기로 호출한다 (executor 가 별도 스레드에서 spin 중이라고 가정)."""
    client = node.create_client(srv_type, srv_name)
    assert client.wait_for_service(timeout_sec=timeout_sec), (
        f"service {srv_name} not available within {timeout_sec}s"
    )
    future = client.call_async(request)
    end = time.monotonic() + timeout_sec
    while time.monotonic() < end:
        if future.done():
            return future.result()
        time.sleep(0.05)
    raise TimeoutError(
        f"service {srv_name} did not respond within {timeout_sec}s"
    )


# ---------- Tests ------------------------------------------------------------


def test_end_to_end_bgr8_30fps(tmp_path: Any) -> None:
    """기본 시나리오: 30fps bgr8 320x240 프레임을 ~2초간 녹화."""
    publisher = SyntheticImagePublisher(
        topic="image",
        width=320,
        height=240,
        encoding="bgr8",
        fps=30.0,
    )
    recorder_node = ImageRecorderNode(
        parameter_overrides=_default_overrides(
            tmp_path, session_prefix="bgr8"
        )
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(publisher)
    executor.add_node(recorder_node)
    spin_thread = _spin_in_background(executor)

    try:
        time.sleep(0.5)  # 노드/서비스 준비 대기
        start_resp = _call_service(
            publisher,
            StartSession,
            "/image_recorder/start_session",
            StartSession.Request(),
        )
        assert start_resp.success is True, "start_session must succeed"
        assert start_resp.mp4_path != ""

        time.sleep(2.0)  # 약 60 프레임 발행 기간

        stop_resp = _call_service(
            publisher,
            StopSession,
            "/image_recorder/stop_session",
            StopSession.Request(),
        )
        assert stop_resp.success is True, "stop_session must succeed"
        assert os.path.isfile(stop_resp.mp4_path)
        assert os.path.getsize(stop_resp.mp4_path) > 0
        # recorder 의 통계 확인
        assert recorder_node._recorder.frames_written > 0
    finally:
        executor.shutdown()
        spin_thread.join(timeout=3.0)
        recorder_node.destroy_node()
        publisher.destroy_node()


def test_end_to_end_encoding_mismatch_auto_stop(tmp_path: Any) -> None:
    """파라미터 pixel_format 과 다른 encoding 을 발행하면 자동 종료 트리거."""
    publisher = SyntheticImagePublisher(
        topic="image",
        width=320,
        height=240,
        encoding="bgr8",  # publisher 는 bgr8
        fps=30.0,
    )
    recorder_node = ImageRecorderNode(
        parameter_overrides=_default_overrides(
            tmp_path, pixel_format="rgb8", session_prefix="mismatch"
        )
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(publisher)
    executor.add_node(recorder_node)
    spin_thread = _spin_in_background(executor)

    try:
        time.sleep(0.5)
        start_resp = _call_service(
            publisher,
            StartSession,
            "/image_recorder/start_session",
            StartSession.Request(),
        )
        assert start_resp.success is True

        # 5프레임 이상 누적되면 자동 종료가 트리거된다 (~166ms @ 30fps).
        # STOPPING 은 일시적인 전이 상태이므로 IDLE/FAILED 까지 대기한다.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if recorder_node._recorder.state in ("IDLE", "FAILED"):
                break
            time.sleep(0.05)

        state = recorder_node._recorder.state
        assert state in ("IDLE", "FAILED"), (
            f"auto-stop should have fired; state={state}"
        )
        assert recorder_node._consecutive_invalid == 0  # 리셋 확인
        assert recorder_node._current_mp4_path is None
    finally:
        executor.shutdown()
        spin_thread.join(timeout=3.0)
        recorder_node.destroy_node()
        publisher.destroy_node()


def test_end_to_end_restart_session(tmp_path: Any) -> None:
    """동일 노드 인스턴스로 두 번 연속 start/stop 수행 시 두 개의 MP4 가 생성된다."""
    publisher = SyntheticImagePublisher(
        topic="image",
        width=320,
        height=240,
        encoding="bgr8",
        fps=30.0,
    )
    recorder_node = ImageRecorderNode(
        parameter_overrides=_default_overrides(
            tmp_path, session_prefix="restart"
        )
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(publisher)
    executor.add_node(recorder_node)
    spin_thread = _spin_in_background(executor)

    paths: list[str] = []
    try:
        time.sleep(0.5)
        for _ in range(2):
            start_resp = _call_service(
                publisher,
                StartSession,
                "/image_recorder/start_session",
                StartSession.Request(),
            )
            assert start_resp.success is True
            time.sleep(1.0)
            stop_resp = _call_service(
                publisher,
                StopSession,
                "/image_recorder/stop_session",
                StopSession.Request(),
            )
            assert stop_resp.success is True
            paths.append(stop_resp.mp4_path)
            # 파일명 충돌을 방지하기 위해 약간의 시간차
            time.sleep(0.05)
    finally:
        executor.shutdown()
        spin_thread.join(timeout=3.0)
        recorder_node.destroy_node()
        publisher.destroy_node()

    assert len(paths) == 2
    assert paths[0] != paths[1], "second session must produce a different file"
    for p in paths:
        assert os.path.isfile(p)
        assert os.path.getsize(p) > 0


def test_sigint_finalizes_mp4(tmp_path: Any) -> None:
    """별도 프로세스로 노드를 띄운 뒤 SIGINT 로 종료 → MP4 가 finalize 되어야 한다."""
    # publisher 는 같은 프로세스 안에서 spin (test process)
    publisher = SyntheticImagePublisher(
        topic="image",
        width=320,
        height=240,
        encoding="bgr8",
        fps=30.0,
    )
    pub_executor = MultiThreadedExecutor(num_threads=1)
    pub_executor.add_node(publisher)
    pub_thread = _spin_in_background(pub_executor)

    recorder_proc: Optional[subprocess.Popen] = None
    try:
        # rdfp 의 image_recorder_node 를 별도 프로세스로 기동한다.
        env = os.environ.copy()
        recorder_proc = subprocess.Popen(
            [
                "ros2", "run", "rdfp", "image_recorder_node",
                "--ros-args",
                "-p", f"output_dir:={tmp_path}",
                "-p", "session_prefix:=sigint",
                "-p", "fps:=30",
                "-p", "resolution:=320x240",
                "-p", "pixel_format:=bgr8",
                "-p", "encoder_mode:=cpu",
            ],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )

        # 노드가 준비될 때까지 대기
        time.sleep(2.0)

        # start_session 호출
        start_resp = _call_service(
            publisher,
            StartSession,
            "/image_recorder/start_session",
            StartSession.Request(),
            timeout_sec=10.0,
        )
        assert start_resp.success is True
        mp4_path = start_resp.mp4_path

        time.sleep(1.5)

        # 자식 프로세스(python image_recorder_node) 에 SIGINT 전송.
        # `ros2 run` wrapper 는 자식에게 시그널을 forward 하지 않을 수 있으므로
        # 손자(python 프로세스) 에 직접 보낸다.
        subprocess.run(
            ["pkill", "-INT", "-f", "/lib/rdfp/image_recorder_node"],
            check=False,
        )

        # finalize 대기 (최대 8초)
        try:
            recorder_proc.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            recorder_proc.kill()
            pytest.fail("recorder did not exit after SIGINT within 8s")

        # finalize 된 파일 검증
        assert os.path.isfile(mp4_path), f"mp4 not found: {mp4_path}"
        assert os.path.getsize(mp4_path) > 0
    finally:
        if recorder_proc is not None and recorder_proc.poll() is None:
            recorder_proc.kill()
            recorder_proc.wait(timeout=2.0)
        pub_executor.shutdown()
        pub_thread.join(timeout=3.0)
        publisher.destroy_node()
