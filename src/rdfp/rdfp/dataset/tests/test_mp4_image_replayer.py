"""Mp4ImageReplayer (push API) 단위 테스트.

push 모델로 전환된 이후의 검증:
    * ``__init__`` — 메타 사전 로드 + publisher 생성. 잘못된 토픽/스트림/mp4
      경로는 즉시 raise.
    * ``start(start_time, first_history_time)`` — 디코더 + publisher 두 스레드를
      띄우고 즉시 반환. mp4 가 깨졌으면 첫 프레임 검증 단계에서 RuntimeError.
    * publisher 스레드 — frame_index 순서대로 publish, mp4 가 image_frames 보다
      짧아도 가용 frame 만 publish 후 깨끗이 종료.
    * ``stop()`` / ``close()`` — cooperative cancellation; 디코더가 큐 가득 차서
      block 중이거나 publisher 가 start_time wait 중이라도 시간 내 풀려난다.
    * ``get_first_stamp()`` — 첫 프레임 stamp 를 ``builtin_interfaces.msg.Time``
      으로 반환.

ROS 의존을 피하기 위해 ``_FakeNode`` / ``_FakePublisher`` / ``_FakeClock`` 으로
publisher 와 ROS clock 을 모두 stub 한다. ``to_ros_image_msg`` 도 stub 해서
``sensor_msgs/Image`` 직접 생성/직렬화 비용을 우회한다.
"""

from __future__ import annotations

from typing import Any, Optional

import sys
import threading
import time
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from builtin_interfaces.msg import Time


# --------------------------------------------------------------------------
# DB / cv2 fakes
# --------------------------------------------------------------------------

class _FakeCursor:
    def __init__(self, fake_conn: '_FakeConn') -> None:
        self._fake_conn = fake_conn
        self._current_rows: list[tuple] = []

    def __enter__(self) -> '_FakeCursor':
        return self

    def __exit__(self, *a: Any) -> bool:
        return False

    def execute(self, query: Any, params: Any = None) -> None:
        self._fake_conn.executed.append((query, params))
        if self._fake_conn.row_queue:
            self._current_rows = self._fake_conn.row_queue.pop(0)
        else:
            self._current_rows = []

    def fetchone(self) -> tuple | None:
        return self._current_rows[0] if self._current_rows else None

    def fetchall(self) -> list[tuple]:
        return list(self._current_rows)


class _FakeConn:
    def __init__(self) -> None:
        self.executed: list[tuple[Any, Any]] = []
        self.row_queue: list[list[tuple]] = []

    def cursor(self) -> _FakeCursor:
        return _FakeCursor(self)


# --------------------------------------------------------------------------
# Node / publisher / clock fakes
# --------------------------------------------------------------------------

class _FakePublisher:
    """``Node.create_publisher`` 가 반환하는 stub. publish 결과는 sink list 에 모인다."""

    def __init__(self, topic: str, sink: list) -> None:
        self._topic = topic
        self._sink = sink

    def publish(self, msg: Any) -> None:
        self._sink.append((self._topic, msg))


class _FakeTime:
    """rclpy.time.Time stub — ``nanoseconds`` 속성만 노출한다."""

    def __init__(self, ns: int) -> None:
        self.nanoseconds = ns


class _FakeClock:
    """ROS clock stub — ``now_ns`` 가 반환할 ns 값을 외부에서 조정할 수 있다.

    기본값은 ``2**62`` 으로, 모든 target 시각이 과거가 되어 ``_wait_until_ros_time``
    이 즉시 반환되도록 한다 (timing 의존 없는 push 검증용).
    """

    def __init__(self, now_ns: int = 2**62) -> None:
        self.now_ns = now_ns

    def now(self) -> _FakeTime:
        return _FakeTime(self.now_ns)


class _FakeNode:
    def __init__(self, now_ns: int = 2**62) -> None:
        self.clock = _FakeClock(now_ns=now_ns)
        self.published: list[tuple[str, Any]] = []
        self.destroyed_publishers: list[_FakePublisher] = []

    def create_publisher(self, msg_type: Any, topic: str, qos: Any) -> _FakePublisher:
        return _FakePublisher(topic, self.published)

    def destroy_publisher(self, pub: _FakePublisher) -> None:
        self.destroyed_publishers.append(pub)

    def get_clock(self) -> _FakeClock:
        return self.clock


# --------------------------------------------------------------------------
# Fixtures / helpers
# --------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def stub_cv2(monkeypatch: pytest.MonkeyPatch) -> MagicMock:
    """``import cv2`` 를 sys.modules 에서 가로챈다 — start() 의 lazy import 시점에 hit."""
    fake_cv2 = MagicMock(name='cv2')
    monkeypatch.setitem(sys.modules, 'cv2', fake_cv2)
    return fake_cv2


@pytest.fixture(autouse=True)
def stub_to_ros_image_msg(monkeypatch: pytest.MonkeyPatch) -> None:
    """``to_ros_image_msg`` 를 stub — 실제 sensor_msgs/Image 생성/직렬화 회피."""
    from rdfp.dataset.db import mp4_image_replayer as m

    def fake_to_ros_image_msg(metadata: Any, stamp: Any, pixels: Any) -> Any:
        msg = MagicMock(name='Image')
        msg.metadata = metadata
        msg.stamp = stamp
        msg.pixels = pixels
        return msg

    monkeypatch.setattr(m, 'to_ros_image_msg', fake_to_ros_image_msg)


def _make_conn(mp4_path: Path, frames: list[tuple],
               width: int = 4, height: int = 4) -> _FakeConn:
    """``_FakeConn`` 에 lookup → stream → frames 순서로 row 를 큐잉한다.

    ``frames`` 는 ``[(frame_index, stamp_sec, stamp_nanosec), ...]``.
    """
    conn = _FakeConn()
    conn.row_queue = [
        [(1,)],
        [(mp4_path.name, 'bgr8', 'cam_link', width, height, len(frames))],
        list(frames),
    ]
    return conn


def _make_mp4(tmp_path: Path) -> Path:
    p = tmp_path / 'cam.mp4'
    p.write_bytes(b'fake-mp4')
    return p


def _stub_videocapture(stub_cv2: MagicMock, isopened: bool = True,
                       read_returns=(True, None)) -> MagicMock:
    fake_cap = MagicMock(name='VideoCapture')
    fake_cap.isOpened.return_value = isopened
    fake_cap.read.return_value = read_returns
    stub_cv2.VideoCapture.return_value = fake_cap
    return fake_cap


def _patch_decode_sequential(monkeypatch: pytest.MonkeyPatch,
                             succeeded: int | None = None) -> None:
    """``decode_one_ndarray`` 를 patch — N 번째 호출까지만 ndarray, 이후 None."""
    from rdfp.dataset.db import mp4_image_replayer as m

    counter = {'n': 0}

    def fake_decode(cap: Any, pixel_format: Any) -> Optional[Any]:
        counter['n'] += 1
        if succeeded is not None and counter['n'] > succeeded:
            return None
        return MagicMock(name=f'pixels-{counter["n"]}')

    monkeypatch.setattr(m, 'decode_one_ndarray', fake_decode)


def _zero_time() -> Time:
    t = Time()
    t.sec = 0
    t.nanosec = 0
    return t


# --------------------------------------------------------------------------
# __init__ tests
# --------------------------------------------------------------------------

def test_init_raises_on_missing_topic(tmp_path: Path) -> None:
    """``topics`` 에 토픽이 없으면 __init__ 단계에서 ValueError."""
    from rdfp.dataset.db import mp4_image_replayer as m

    conn = _FakeConn()
    conn.row_queue = [[]]
    node = _FakeNode()
    with pytest.raises(ValueError, match='not found in topics'):
        m.Mp4ImageReplayer(node, conn, episode_id=1, topic_name='/missing',
                           mp4_root=tmp_path)


def test_init_raises_on_missing_image_streams_row(tmp_path: Path) -> None:
    """``image_streams`` 행이 없으면 __init__ 단계에서 ValueError."""
    from rdfp.dataset.db import mp4_image_replayer as m

    conn = _FakeConn()
    conn.row_queue = [
        [(1,)],
        [],
    ]
    node = _FakeNode()
    with pytest.raises(ValueError, match='no image_streams row'):
        m.Mp4ImageReplayer(node, conn, episode_id=1, topic_name='/cam',
                           mp4_root=tmp_path)


def test_init_raises_on_missing_mp4(tmp_path: Path) -> None:
    """mp4 파일이 디스크에 없으면 __init__ 단계에서 RuntimeError."""
    from rdfp.dataset.db import mp4_image_replayer as m

    conn = _FakeConn()
    conn.row_queue = [
        [(1,)],
        [('missing.mp4', 'bgr8', '', 4, 4, 1)],
        [(0, 100, 0)],
    ]
    node = _FakeNode()
    with pytest.raises(RuntimeError, match='mp4 file not found'):
        m.Mp4ImageReplayer(node, conn, episode_id=1, topic_name='/cam',
                           mp4_root=tmp_path)


def test_init_raises_on_empty_image_frames(tmp_path: Path) -> None:
    """``image_frames`` row 가 0 개면 __init__ 단계에서 ValueError."""
    from rdfp.dataset.db import mp4_image_replayer as m

    mp4 = _make_mp4(tmp_path)
    conn = _make_conn(mp4, frames=[])
    node = _FakeNode()
    with pytest.raises(ValueError, match='no image_frames row'):
        m.Mp4ImageReplayer(node, conn, episode_id=1, topic_name='/cam',
                           mp4_root=tmp_path)


def test_init_creates_publisher(tmp_path: Path) -> None:
    """``__init__`` 이 publisher 를 즉시 생성하며 ``Image`` 타입을 사용한다."""
    from rdfp.dataset.db import mp4_image_replayer as m
    from sensor_msgs.msg import Image

    mp4 = _make_mp4(tmp_path)
    conn = _make_conn(mp4, [(0, 100, 0)])
    node = _FakeNode()
    create_pub = MagicMock(side_effect=node.create_publisher)
    node.create_publisher = create_pub   # type: ignore[method-assign]

    m.Mp4ImageReplayer(node, conn, episode_id=1, topic_name='/cam', mp4_root=tmp_path)
    create_pub.assert_called_once()
    msg_type, topic_name, qos = create_pub.call_args[0]
    assert msg_type is Image
    assert topic_name == '/cam'
    assert qos == 10


# --------------------------------------------------------------------------
# start() tests
# --------------------------------------------------------------------------

def test_start_raises_when_videocapture_not_opened(
    tmp_path: Path, stub_cv2: MagicMock,
) -> None:
    """``isOpened=False`` → ``start()`` 가 RuntimeError."""
    from rdfp.dataset.db import mp4_image_replayer as m

    mp4 = _make_mp4(tmp_path)
    conn = _make_conn(mp4, [(0, 100, 0)])
    _stub_videocapture(stub_cv2, isopened=False)

    r = m.Mp4ImageReplayer(_FakeNode(), conn, episode_id=1, topic_name='/cam',
                           mp4_root=tmp_path)
    with pytest.raises(RuntimeError, match='failed to open mp4'):
        r.start(_zero_time(), _zero_time())
    assert not r.is_running


def test_start_raises_when_first_frame_decode_fails(
    tmp_path: Path, stub_cv2: MagicMock,
) -> None:
    """``cap.isOpened()=True`` 인데 첫 ``read()`` 가 실패하면 RuntimeError + cap 해제."""
    from rdfp.dataset.db import mp4_image_replayer as m

    mp4 = _make_mp4(tmp_path)
    conn = _make_conn(mp4, [(0, 100, 0), (1, 100, 1_000_000)])
    fake_cap = _stub_videocapture(stub_cv2, isopened=True, read_returns=(False, None))

    r = m.Mp4ImageReplayer(_FakeNode(), conn, episode_id=1, topic_name='/cam',
                           mp4_root=tmp_path)
    with pytest.raises(RuntimeError, match='failed to decode first frame'):
        r.start(_zero_time(), _zero_time())
    assert fake_cap.release.called
    assert not r.is_running


def test_double_start_raises(
    tmp_path: Path, stub_cv2: MagicMock, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """이미 ``start()`` 한 인스턴스에 두 번째 ``start()`` 는 RuntimeError."""
    from rdfp.dataset.db import mp4_image_replayer as m

    mp4 = _make_mp4(tmp_path)
    conn = _make_conn(mp4, [(i, 100, i * 1_000_000) for i in range(3)])
    _stub_videocapture(stub_cv2)
    _patch_decode_sequential(monkeypatch)

    r = m.Mp4ImageReplayer(_FakeNode(), conn, episode_id=1, topic_name='/cam',
                           mp4_root=tmp_path)
    r.start(_zero_time(), _zero_time())
    try:
        with pytest.raises(RuntimeError, match='already started'):
            r.start(_zero_time(), _zero_time())
    finally:
        r.close()


def test_start_after_close_raises(
    tmp_path: Path, stub_cv2: MagicMock, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """``close()`` 이후 ``start()`` 호출은 RuntimeError."""
    from rdfp.dataset.db import mp4_image_replayer as m

    mp4 = _make_mp4(tmp_path)
    conn = _make_conn(mp4, [(0, 100, 0)])
    _stub_videocapture(stub_cv2)
    _patch_decode_sequential(monkeypatch)

    r = m.Mp4ImageReplayer(_FakeNode(), conn, episode_id=1, topic_name='/cam',
                           mp4_root=tmp_path)
    r.close()
    with pytest.raises(RuntimeError, match='already closed'):
        r.start(_zero_time(), _zero_time())


# --------------------------------------------------------------------------
# Publish flow tests
# --------------------------------------------------------------------------

def test_publishes_all_frames_in_order(
    tmp_path: Path, stub_cv2: MagicMock, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """모든 프레임이 frame_index 순서대로 publish 되며 published_count 와 일치한다."""
    from rdfp.dataset.db import mp4_image_replayer as m

    mp4 = _make_mp4(tmp_path)
    conn = _make_conn(mp4, [(i, 100, i * 1_000_000) for i in range(5)])
    _stub_videocapture(stub_cv2)
    _patch_decode_sequential(monkeypatch)
    node = _FakeNode()

    r = m.Mp4ImageReplayer(node, conn, episode_id=1, topic_name='/cam',
                           mp4_root=tmp_path)
    r.start(_zero_time(), _zero_time())
    r.join(timeout=2.0)

    assert not r.is_running
    assert r.published_count == 5
    assert len(node.published) == 5
    # 모두 같은 토픽으로
    assert all(topic == '/cam' for topic, _ in node.published)
    r.close()


def test_publishes_image_msg_with_shifted_stamp(
    tmp_path: Path, stub_cv2: MagicMock, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """publish 된 메시지의 stamp 는 wall-clock 시간축 (start_time + offset) 으로 shift."""
    from rdfp.dataset.db import mp4_image_replayer as m

    mp4 = _make_mp4(tmp_path)
    # frame 0: stamp 100s, frame 1: 100s + 100ms
    conn = _make_conn(mp4, [(0, 100, 0), (1, 100, 100_000_000)])
    _stub_videocapture(stub_cv2)
    _patch_decode_sequential(monkeypatch)
    node = _FakeNode()

    r = m.Mp4ImageReplayer(node, conn, episode_id=1, topic_name='/cam',
                           mp4_root=tmp_path)
    # start_time = 5s, first_history_time = 100s (= 첫 frame stamp)
    start_time = Time(); start_time.sec = 5; start_time.nanosec = 0
    first_history = r.get_first_stamp()
    assert first_history.sec == 100 and first_history.nanosec == 0
    r.start(start_time, first_history)
    r.join(timeout=2.0)

    assert r.published_count == 2
    # 첫 프레임: stamp = 5s + (100s - 100s) = 5s
    msg0 = node.published[0][1]
    assert msg0.stamp.sec == 5 and msg0.stamp.nanosec == 0
    # 두 번째: stamp = 5s + (100.1s - 100s) = 5.1s
    msg1 = node.published[1][1]
    assert msg1.stamp.sec == 5 and msg1.stamp.nanosec == 100_000_000
    r.close()


def test_mp4_shorter_than_image_frames(
    tmp_path: Path, stub_cv2: MagicMock, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """디코더가 N+1 번째 프레임에서 None 반환 시 가용 frame 만 publish 후 종료."""
    from rdfp.dataset.db import mp4_image_replayer as m

    mp4 = _make_mp4(tmp_path)
    # image_frames 5 개, mp4 는 3 frame 만 디코딩 가능.
    conn = _make_conn(mp4, [(i, 100, i * 1_000_000) for i in range(5)])
    _stub_videocapture(stub_cv2)
    _patch_decode_sequential(monkeypatch, succeeded=3)
    node = _FakeNode()

    r = m.Mp4ImageReplayer(node, conn, episode_id=1, topic_name='/cam',
                           mp4_root=tmp_path)
    r.start(_zero_time(), _zero_time())
    r.join(timeout=2.0)

    # 1 (start eager) + 2 (decoder) = 3 frames
    assert r.published_count == 3
    assert len(node.published) == 3
    r.close()


def test_non_contiguous_frame_indices_pass_through(
    tmp_path: Path, stub_cv2: MagicMock, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """frame_index 가 누락/중복/비연속이어도 image_frames row 순서대로 publish 된다.

    Mp4ImageReplayer 는 frame_index 의 의미를 검증하지 않고 list 순서대로 한
    프레임씩 처리한다 (mp4 의 N번째 프레임 ↔ image_frames 의 N번째 row).
    """
    from rdfp.dataset.db import mp4_image_replayer as m

    mp4 = _make_mp4(tmp_path)
    frames = [
        (0, 100, 0),
        (2, 100, 200_000_000),
        (5, 100, 500_000_000),
        (5, 100, 500_000_001),
        (10, 100, 1_000_000_000),
    ]
    conn = _make_conn(mp4, frames)
    _stub_videocapture(stub_cv2)
    _patch_decode_sequential(monkeypatch)
    node = _FakeNode()

    r = m.Mp4ImageReplayer(node, conn, episode_id=1, topic_name='/cam',
                           mp4_root=tmp_path)
    r.start(_zero_time(), _zero_time())
    r.join(timeout=2.0)

    assert r.published_count == 5
    assert r.expected_count == 5
    assert r.first_stamp_ns == 100 * 1_000_000_000 + 0
    assert r.last_stamp_ns == 100 * 1_000_000_000 + 1_000_000_000
    r.close()


# --------------------------------------------------------------------------
# get_first_stamp tests
# --------------------------------------------------------------------------

def test_get_first_stamp_returns_first_frame_stamp(tmp_path: Path) -> None:
    """``get_first_stamp()`` 은 첫 frame 의 stamp 를 builtin Time 으로 반환한다."""
    from rdfp.dataset.db import mp4_image_replayer as m

    mp4 = _make_mp4(tmp_path)
    conn = _make_conn(mp4, [(0, 200, 500_000_000),
                            (1, 200, 600_000_000)])
    r = m.Mp4ImageReplayer(_FakeNode(), conn, episode_id=1, topic_name='/cam',
                           mp4_root=tmp_path)
    t = r.get_first_stamp()
    assert isinstance(t, Time)
    assert t.sec == 200 and t.nanosec == 500_000_000


# --------------------------------------------------------------------------
# stop / close tests
# --------------------------------------------------------------------------

def test_close_is_idempotent(
    tmp_path: Path, stub_cv2: MagicMock, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """``close()`` 를 여러 번 호출해도 안전 (cv2 release 한 번만)."""
    from rdfp.dataset.db import mp4_image_replayer as m

    mp4 = _make_mp4(tmp_path)
    conn = _make_conn(mp4, [(0, 100, 0)])
    fake_cap = _stub_videocapture(stub_cv2)
    _patch_decode_sequential(monkeypatch)

    r = m.Mp4ImageReplayer(_FakeNode(), conn, episode_id=1, topic_name='/cam',
                           mp4_root=tmp_path)
    r.start(_zero_time(), _zero_time())
    r.join(timeout=2.0)
    r.close()
    r.close()
    r.close()
    assert fake_cap.release.call_count == 1


def test_context_manager_closes(
    tmp_path: Path, stub_cv2: MagicMock, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """``with`` 블록 종료 시 close 가 자동 호출된다."""
    from rdfp.dataset.db import mp4_image_replayer as m

    mp4 = _make_mp4(tmp_path)
    conn = _make_conn(mp4, [(0, 100, 0), (1, 100, 1_000_000)])
    fake_cap = _stub_videocapture(stub_cv2)
    _patch_decode_sequential(monkeypatch)

    with m.Mp4ImageReplayer(
        _FakeNode(), conn, episode_id=1, topic_name='/cam', mp4_root=tmp_path,
    ) as r:
        r.start(_zero_time(), _zero_time())
        r.join(timeout=2.0)

    assert fake_cap.release.called


def test_close_unblocks_decoder_in_full_queue(
    tmp_path: Path, stub_cv2: MagicMock, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """publisher 가 start_time wait 중이라 큐를 비우지 않을 때 close() 가 양쪽 모두 푼다.

    클럭을 0 으로 두고 start_time 을 먼 미래로 잡으면 publisher 가 wait 에 갇힌다.
    그 사이 디코더는 큐 (size=2) 를 채우고 더 못 넣어 backpressure. close() 가
    1초 안에 두 스레드를 모두 종료해야 한다.
    """
    from rdfp.dataset.db import mp4_image_replayer as m

    mp4 = _make_mp4(tmp_path)
    conn = _make_conn(mp4, [(i, 100, i * 1_000_000) for i in range(100)])
    _stub_videocapture(stub_cv2)
    _patch_decode_sequential(monkeypatch)
    # 클럭은 1ns (사실상 0). start_time 은 1년 후.
    node = _FakeNode(now_ns=1)

    r = m.Mp4ImageReplayer(node, conn, episode_id=1, topic_name='/cam',
                           mp4_root=tmp_path, decode_queue_size=2)
    far_future = Time()
    far_future.sec = 365 * 24 * 3600
    far_future.nanosec = 0
    r.start(far_future, _zero_time())

    time.sleep(0.2)
    decoder = r._decoder_thread
    publisher = r._publisher_thread
    assert decoder is not None and decoder.is_alive()
    assert publisher is not None and publisher.is_alive()

    t0 = time.monotonic()
    r.close()
    elapsed = time.monotonic() - t0

    assert not decoder.is_alive()
    assert not publisher.is_alive()
    assert elapsed < 1.0, f'close() took too long: {elapsed:.3f}s'


def test_stop_during_start_time_wait(
    tmp_path: Path, stub_cv2: MagicMock, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """``stop()`` 호출 시 publisher 가 start_time wait 중이라도 시간 내 빠져나온다."""
    from rdfp.dataset.db import mp4_image_replayer as m

    mp4 = _make_mp4(tmp_path)
    conn = _make_conn(mp4, [(i, 100, i * 1_000_000) for i in range(3)])
    _stub_videocapture(stub_cv2)
    _patch_decode_sequential(monkeypatch)
    node = _FakeNode(now_ns=1)

    r = m.Mp4ImageReplayer(node, conn, episode_id=1, topic_name='/cam',
                           mp4_root=tmp_path)
    far_future = Time()
    far_future.sec = 365 * 24 * 3600
    far_future.nanosec = 0
    r.start(far_future, _zero_time())

    time.sleep(0.15)   # 워커들이 wait/decode 에 들어가도록
    t0 = time.monotonic()
    r.stop()
    r.join(timeout=2.0)
    elapsed = time.monotonic() - t0

    assert not r.is_running
    assert elapsed < 1.0, f'stop+join took too long: {elapsed:.3f}s'
    # 한 frame 도 publish 되지 않았어야 한다 (start_time 이 미래라 wait 중에 stop).
    assert r.published_count == 0
    r.close()


def test_stop_before_start_is_noop(tmp_path: Path) -> None:
    """``start()`` 전에 ``stop()`` 호출은 safe (no-op)."""
    from rdfp.dataset.db import mp4_image_replayer as m

    mp4 = _make_mp4(tmp_path)
    conn = _make_conn(mp4, [(0, 100, 0)])
    r = m.Mp4ImageReplayer(_FakeNode(), conn, episode_id=1, topic_name='/cam',
                           mp4_root=tmp_path)
    r.stop()   # no exception
    assert not r.is_running
    r.close()
