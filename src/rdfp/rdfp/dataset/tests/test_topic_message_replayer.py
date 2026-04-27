"""TopicMessageReplayer 단위 테스트.

검증 항목:
    * 이미지 토픽이 ``topic_names`` 에 포함되면 ``__init__`` 단계에서 ValueError.
    * ``start()`` 두 번째 호출 시 RuntimeError ("already started").
    * 워커 스레드 내부에서 예외가 발생하면 ``join()`` 이 해당 예외를 re-raise.
    * 실행 중 ``close()`` 가 워커를 정지시키고 publisher destroy 까지 안전하게 정리.

ROS 의존을 피하기 위해 ``_FakeNode`` / ``_FakePublisher`` / ``_FakeClock`` 으로
publisher 와 ROS clock 을 stub 한다. DB 의존은 ``is_image_topic_in_db`` /
``read_topic_messages_by_name`` 을 monkeypatch 로 stub 한다.
"""

from __future__ import annotations

from typing import Any, Optional

import threading
import time

import pytest


# --------------------------------------------------------------------------
# DB / message fakes
# --------------------------------------------------------------------------

class _FakeCursor:
    def __init__(self, fake_conn: '_FakeConn') -> None:
        self._fake_conn = fake_conn

    def __enter__(self) -> '_FakeCursor':
        return self

    def __exit__(self, *a: Any) -> bool:
        return False

    def execute(self, query: Any, params: Any = None) -> None:
        self._fake_conn.executed.append((query, params))

    def fetchone(self) -> tuple | None:
        # 에피소드 존재 확인용 — 항상 (1,) 반환.
        return (1,)


class _FakeConn:
    def __init__(self) -> None:
        self.executed: list[tuple[Any, Any]] = []

    def cursor(self) -> _FakeCursor:
        return _FakeCursor(self)


class _FakeStamp:
    def __init__(self, sec: int = 0, nanosec: int = 0) -> None:
        self.sec = sec
        self.nanosec = nanosec


class _FakeHeader:
    def __init__(self, sec: int = 0, nanosec: int = 0) -> None:
        self.stamp = _FakeStamp(sec, nanosec)


class _FakeMsg:
    """``header.stamp`` 만 가진 최소 stub. publisher type 으로도 사용된다."""

    def __init__(self, sec: int = 0, nanosec: int = 0) -> None:
        self.header = _FakeHeader(sec, nanosec)


# --------------------------------------------------------------------------
# Node / publisher / clock fakes
# --------------------------------------------------------------------------

class _FakePublisher:
    def __init__(self, topic: str, sink: list,
                 raise_exc: Optional[BaseException] = None) -> None:
        self._topic = topic
        self._sink = sink
        self._raise_exc = raise_exc

    def publish(self, msg: Any) -> None:
        if self._raise_exc is not None:
            raise self._raise_exc
        self._sink.append((self._topic, msg))


class _FakeTime:
    def __init__(self, ns: int) -> None:
        self.nanoseconds = ns


class _FakeClock:
    def __init__(self, now_ns: int = 2**62) -> None:
        self.now_ns = now_ns

    def now(self) -> _FakeTime:
        return _FakeTime(self.now_ns)


class _FakeNode:
    def __init__(self, now_ns: int = 2**62,
                 publish_exc: Optional[BaseException] = None) -> None:
        self.clock = _FakeClock(now_ns=now_ns)
        self.published: list[tuple[str, Any]] = []
        self.destroyed_publishers: list[_FakePublisher] = []
        self._publish_exc = publish_exc

    def create_publisher(self, msg_type: Any, topic: str, qos: Any) -> _FakePublisher:
        return _FakePublisher(topic, self.published, raise_exc=self._publish_exc)

    def destroy_publisher(self, pub: _FakePublisher) -> None:
        self.destroyed_publishers.append(pub)

    def get_clock(self) -> _FakeClock:
        return self.clock


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------

def _patch_db(monkeypatch: pytest.MonkeyPatch, *,
              image_topics: Optional[set[str]] = None,
              messages_by_topic: Optional[dict[str, list[Any]]] = None) -> None:
    """``is_image_topic_in_db`` 와 ``read_topic_messages_by_name`` 을 stub.

    ``image_topics`` 에 들어있는 토픽은 이미지로 간주된다 (그 외는 비이미지).
    ``messages_by_topic`` 은 비이미지 토픽별 메시지 리스트를 반환하도록 한다.
    """
    from rdfp.dataset.db import topic_message_replayer as m

    image_set = image_topics or set()
    msgs = messages_by_topic or {}

    def fake_is_image(conn: Any, topic_name: str) -> bool:
        return topic_name in image_set

    def fake_read(conn: Any, episode_id: int, topic_name: str) -> list[Any]:
        return list(msgs.get(topic_name, []))

    monkeypatch.setattr(m, 'is_image_topic_in_db', fake_is_image)
    monkeypatch.setattr(m, 'read_topic_messages_by_name', fake_read)


def _zero_time() -> Any:
    """``builtin_interfaces.msg.Time`` 대용 — sec/nanosec 만 있으면 ``_time_to_ns`` 가 받는다."""
    from builtin_interfaces.msg import Time
    t = Time()
    t.sec = 0
    t.nanosec = 0
    return t


# --------------------------------------------------------------------------
# 0) 빈 source 가드
# --------------------------------------------------------------------------

def test_init_raises_when_all_topics_empty(monkeypatch: pytest.MonkeyPatch) -> None:
    """모든 요청 토픽이 빈 source 면 __init__ 단계에서 ValueError.

    이렇게 막지 않으면 빈 replayer 가 생성되어 ``get_first_stamp()`` 에서
    뒤늦게 RuntimeError 가 나고, 같이 묶인 이미지 replayer 까지 연쇄 실패.
    """
    from rdfp.dataset.db import topic_message_replayer as m

    _patch_db(monkeypatch, messages_by_topic={})   # 어느 토픽도 메시지 없음.

    node = _FakeNode()
    conn = _FakeConn()
    with pytest.raises(ValueError, match='no messages on any'):
        m.TopicMessageReplayer(
            node, conn, episode_id=1, topic_names=['/cmd_vel', '/joint_states'])
    assert node.destroyed_publishers == []   # publisher 도 만들어지지 않음.


def test_init_succeeds_when_any_topic_has_messages(
    monkeypatch: pytest.MonkeyPatch) -> None:
    """일부 토픽만 메시지가 있어도 replayer 생성은 성공한다 (skip + warn)."""
    from rdfp.dataset.db import topic_message_replayer as m

    # /cmd_vel 만 메시지가 있고 /joint_states 는 비어 있음.
    _patch_db(monkeypatch, messages_by_topic={'/cmd_vel': [_FakeMsg(0, 0)]})

    node = _FakeNode()
    conn = _FakeConn()
    r = m.TopicMessageReplayer(
        node, conn, episode_id=1, topic_names=['/cmd_vel', '/joint_states'])
    # 빈 토픽은 skip 되고 cmd_vel 만 stream 으로 잡힘.
    assert r.topic_names == ['/cmd_vel']
    r.close()


# --------------------------------------------------------------------------
# 1) 이미지 토픽 거부
# --------------------------------------------------------------------------

def test_init_rejects_image_topic(monkeypatch: pytest.MonkeyPatch) -> None:
    """이미지 토픽이 ``topic_names`` 에 포함되면 __init__ 에서 ValueError."""
    from rdfp.dataset.db import topic_message_replayer as m

    _patch_db(monkeypatch,
              image_topics={'/cam/image_raw'},
              messages_by_topic={'/cmd_vel': [_FakeMsg()]})

    node = _FakeNode()
    conn = _FakeConn()
    with pytest.raises(ValueError, match='image topic'):
        m.TopicMessageReplayer(
            node, conn, episode_id=1,
            topic_names=['/cmd_vel', '/cam/image_raw'])


def test_init_rejects_image_topic_does_not_create_publisher(
    monkeypatch: pytest.MonkeyPatch) -> None:
    """이미지 거부 시 어떤 publisher 도 생성되지 않아야 한다 (early raise)."""
    from rdfp.dataset.db import topic_message_replayer as m

    _patch_db(monkeypatch, image_topics={'/cam'})

    node = _FakeNode()
    conn = _FakeConn()
    with pytest.raises(ValueError):
        m.TopicMessageReplayer(node, conn, episode_id=1, topic_names=['/cam'])
    assert node.destroyed_publishers == []
    # publish 된 메시지도 없어야 함 (publisher 자체가 만들어지지 않음).


# --------------------------------------------------------------------------
# 2) 두 번째 start() 거부
# --------------------------------------------------------------------------

def test_start_twice_raises(monkeypatch: pytest.MonkeyPatch) -> None:
    """``start()`` 가 두 번째 호출되면 RuntimeError ('already started')."""
    from rdfp.dataset.db import topic_message_replayer as m

    _patch_db(monkeypatch, messages_by_topic={'/t': [_FakeMsg(0, 0)]})

    # 클럭을 매우 작은 값으로 두면 start_ns 가 미래라 워커가 wait 에 들어감 →
    # 정상 종료 전에 두 번째 start() 를 시도할 수 있다. 하지만 _worker_thread is
    # not None 가드는 워커 종료 여부와 무관하게 거부하므로, 워커가 끝난 뒤에도
    # 동일하게 거부됨을 검증하는 게 더 안전하다.
    node = _FakeNode(now_ns=2**62)   # 모든 target 이 과거 → 즉시 publish + 종료.
    conn = _FakeConn()
    r = m.TopicMessageReplayer(node, conn, episode_id=1, topic_names=['/t'])

    r.start(_zero_time(), _zero_time())
    r.join(timeout=2.0)
    # 정상 종료 후라도 재시작 차단.
    with pytest.raises(RuntimeError, match='already started'):
        r.start(_zero_time(), _zero_time())
    r.close()


def test_start_concurrent_call_raises(monkeypatch: pytest.MonkeyPatch) -> None:
    """워커가 실행 중일 때 ``start()`` 두 번째 호출도 RuntimeError."""
    from rdfp.dataset.db import topic_message_replayer as m

    _patch_db(monkeypatch, messages_by_topic={'/t': [_FakeMsg(0, 0)]})

    # 클럭을 0 으로 두고 start_ns 를 매우 큰 값으로 주면 워커가 무한 wait.
    node = _FakeNode(now_ns=0)
    conn = _FakeConn()
    r = m.TopicMessageReplayer(node, conn, episode_id=1, topic_names=['/t'])

    far_future = _zero_time()
    far_future.sec = 365 * 24 * 3600   # 1 년 후.
    r.start(far_future, _zero_time())
    try:
        # 워커가 wait 에 들어갈 시간을 준다.
        time.sleep(0.05)
        assert r.is_running
        with pytest.raises(RuntimeError, match='already started'):
            r.start(_zero_time(), _zero_time())
    finally:
        r.close()


# --------------------------------------------------------------------------
# 3) 워커 예외 → join() re-raise
# --------------------------------------------------------------------------

def test_join_reraises_worker_exception(monkeypatch: pytest.MonkeyPatch) -> None:
    """publish 단계에서 예외가 발생하면 ``join()`` 이 해당 예외를 re-raise."""
    from rdfp.dataset.db import topic_message_replayer as m

    _patch_db(monkeypatch, messages_by_topic={'/t': [_FakeMsg(0, 0)]})

    boom = RuntimeError('boom from publisher')
    node = _FakeNode(now_ns=2**62, publish_exc=boom)
    conn = _FakeConn()
    r = m.TopicMessageReplayer(node, conn, episode_id=1, topic_names=['/t'])

    r.start(_zero_time(), _zero_time())
    with pytest.raises(RuntimeError, match='boom from publisher'):
        r.join(timeout=2.0)
    # error property 도 동일 예외를 노출.
    assert r.error is boom
    r.close()


def test_error_property_visible_before_join(monkeypatch: pytest.MonkeyPatch) -> None:
    """워커가 죽은 직후 ``error`` property 로 비차단 조회 가능."""
    from rdfp.dataset.db import topic_message_replayer as m

    _patch_db(monkeypatch, messages_by_topic={'/t': [_FakeMsg(0, 0)]})

    boom = ValueError('x')
    node = _FakeNode(now_ns=2**62, publish_exc=boom)
    conn = _FakeConn()
    r = m.TopicMessageReplayer(node, conn, episode_id=1, topic_names=['/t'])

    r.start(_zero_time(), _zero_time())
    # 워커가 publish 하다 죽을 때까지 대기 (timeout 으로 폴링).
    deadline = time.time() + 2.0
    while time.time() < deadline and r.error is None:
        time.sleep(0.01)
    assert r.error is boom
    r.close()


# --------------------------------------------------------------------------
# 4) 실행 중 close() 가 worker stop + publisher 정리
# --------------------------------------------------------------------------

def test_close_stops_running_worker_and_destroys_publishers(
    monkeypatch: pytest.MonkeyPatch) -> None:
    """워커가 wait 중일 때 close() 가 stop + join + destroy 까지 처리."""
    from rdfp.dataset.db import topic_message_replayer as m

    _patch_db(monkeypatch, messages_by_topic={
        '/t1': [_FakeMsg(0, 0)],
        '/t2': [_FakeMsg(0, 0)]})

    # 워커가 _wait_until_ros_time 에서 무한 polling 하도록 클럭을 0 으로 고정.
    node = _FakeNode(now_ns=0)
    conn = _FakeConn()
    r = m.TopicMessageReplayer(node, conn, episode_id=1, topic_names=['/t1', '/t2'])

    far_future = _zero_time()
    far_future.sec = 365 * 24 * 3600
    r.start(far_future, _zero_time())
    try:
        time.sleep(0.05)   # 워커가 wait 에 들어갈 시간.
        assert r.is_running
    finally:
        r.close()

    # close() 후 워커는 죽고, publisher 두 개가 모두 destroy 되어야 한다.
    assert not r.is_running
    assert len(node.destroyed_publishers) == 2


def test_close_is_idempotent(monkeypatch: pytest.MonkeyPatch) -> None:
    """``close()`` 는 두 번 호출되어도 한 번만 동작 — destroy_publisher 도 1 회만."""
    from rdfp.dataset.db import topic_message_replayer as m

    _patch_db(monkeypatch, messages_by_topic={'/t': [_FakeMsg(0, 0)]})

    node = _FakeNode(now_ns=2**62)
    conn = _FakeConn()
    r = m.TopicMessageReplayer(node, conn, episode_id=1, topic_names=['/t'])

    r.start(_zero_time(), _zero_time())
    r.join(timeout=2.0)

    r.close()
    first = list(node.destroyed_publishers)
    r.close()   # 두 번째 close 는 no-op.
    assert node.destroyed_publishers == first


def test_close_after_start_failure_is_safe(monkeypatch: pytest.MonkeyPatch) -> None:
    """이미지 거부로 __init__ 이 실패해도, close() 호출 없이 깔끔히 종료."""
    from rdfp.dataset.db import topic_message_replayer as m

    _patch_db(monkeypatch, image_topics={'/cam'})

    node = _FakeNode()
    conn = _FakeConn()
    with pytest.raises(ValueError):
        m.TopicMessageReplayer(node, conn, episode_id=1, topic_names=['/cam'])
    # __init__ 실패 시 publisher / 스레드 자원이 만들어지지 않음 → 별도 정리 불필요.
    assert node.destroyed_publishers == []
