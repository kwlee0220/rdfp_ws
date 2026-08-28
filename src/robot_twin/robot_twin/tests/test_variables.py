"""`robot_twin.variables` 의 정적 소스 경로 단위 테스트 (설계서 4.3).

토픽 구독 경로는 실제 Node 가 필요하지만, 정적 소스는 resolver 를 주입받으므로
가짜 노드만으로 검증할 수 있다. 모듈이 rclpy 를 import 하므로 ROS 를 source 하지
않은 환경에서는 자체 skip 한다.
"""

from __future__ import annotations

from typing import Any, Optional

import threading

import pytest

pytest.importorskip('rclpy', reason='robot_twin.variables imports rclpy')

from robot_twin.config import VariableConfig  # noqa: E402
from robot_twin.snapshot import Quality, Reason  # noqa: E402
from robot_twin.variables import (  # noqa: E402
    STATIC_RETRY_SEC, StaticSourceUnavailable, VariableCache
)


class FakeLogger:
    """노드 로거 대역 — 호출만 삼킨다."""

    def info(self, msg: str) -> None:
        pass

    def warning(self, msg: str) -> None:
        pass

    def error(self, msg: str) -> None:
        pass


class FakeNode:
    """``VariableCache`` 가 실제로 쓰는 표면만 갖춘 가짜 노드."""

    def get_logger(self) -> FakeLogger:
        return FakeLogger()

    def create_subscription(self, *args: Any, **kwargs: Any) -> object:
        raise AssertionError('static source must not create a subscription')

    def count_publishers(self, topic: str) -> int:
        return 0


class FakeClock:
    """수동으로 진행시키는 시계."""

    def __init__(self, now: float = 1000.0) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


def make_cache(resolver: Optional[Any], clock: Optional[FakeClock] = None,
               *, method: Any = 'get_all_named_targets') -> tuple[VariableCache, FakeClock]:
    """정적 변수 하나만 가진 캐시를 만든다."""
    clock = clock or FakeClock()
    config = VariableConfig.model_validate({
        'name': 'named_targets',
        'source': {'type': 'static', 'backend': {'method': method}, 'timeout_sec': 2.0},
        'staleness_ms': None
    })
    return VariableCache(FakeNode(), [config], clock=clock, static_resolver=resolver), clock


NAMED_TARGETS = {'panda_arm': ['extended', 'ready'], 'hand': ['close', 'open']}


def test_start_does_not_fetch_static_source() -> None:
    """기동 시점 조회는 금지다 — 트윈이 move_group 보다 먼저 뜰 수 있다."""
    calls: list[str] = []

    def resolver(method: str, timeout: float) -> Any:
        calls.append(method)
        return NAMED_TARGETS

    cache, _ = make_cache(resolver)

    cache.start()

    assert calls == []
    # 조회를 거치지 않은 상태 — `/health` 가 보는 값이기도 하다.
    assert cache.entry('named_targets').status().quality is Quality.NO_DATA


def test_first_read_fetches_lazily() -> None:
    cache, _ = make_cache(lambda m, t: NAMED_TARGETS)

    body = cache.read('named_targets')

    assert body['quality'] == Quality.OK.value
    assert body['value'] == NAMED_TARGETS
    # 정적 소스는 헤더가 없으므로 stamp 를 싣지 않는다.
    assert body['stamp'] is None


def test_resolver_receives_configured_method_and_timeout() -> None:
    calls: list[tuple[str, float]] = []

    def resolver(method: str, timeout: float) -> Any:
        calls.append((method, timeout))
        return NAMED_TARGETS

    cache, _ = make_cache(resolver)
    cache.read('named_targets')

    assert calls == [('get_all_named_targets', 2.0)]


def test_value_is_cached_indefinitely() -> None:
    """성공한 값은 자동 무효화하지 않는다 — 두 번째 조회는 백엔드를 부르지 않는다."""
    calls: list[str] = []

    def resolver(method: str, timeout: float) -> Any:
        calls.append(method)
        return NAMED_TARGETS

    cache, clock = make_cache(resolver)
    cache.read('named_targets')
    clock.now += 86400.0
    body = cache.read('named_targets')

    assert calls == ['get_all_named_targets']
    # staleness_ms 가 None 이므로 하루가 지나도 STALE 이 아니다.
    assert body['quality'] == Quality.OK.value


def test_refresh_forces_refetch() -> None:
    values = [NAMED_TARGETS, {'panda_arm': ['ready']}]
    calls: list[str] = []

    def resolver(method: str, timeout: float) -> Any:
        calls.append(method)
        return values[min(len(calls) - 1, len(values) - 1)]

    cache, _ = make_cache(resolver)

    first = cache.read('named_targets')
    second = cache.read('named_targets', refresh=True)

    assert first['value'] == NAMED_TARGETS
    assert second['value'] == {'panda_arm': ['ready']}
    assert len(calls) == 2


def test_client_not_ready_reports_source_node_down_and_retries_later() -> None:
    """백엔드가 늦게 떠도 트윈 재시작 없이 회복되어야 한다."""
    ready = {'value': False}

    def resolver(method: str, timeout: float) -> Any:
        if not ready['value']:
            raise StaticSourceUnavailable('MoveGroup client is not ready')
        return NAMED_TARGETS

    cache, clock = make_cache(resolver)

    body = cache.read('named_targets')
    assert body['quality'] == Quality.SOURCE_UNAVAILABLE.value
    assert body['reason'] == Reason.SOURCE_NODE_DOWN.value
    assert body['value'] is None

    ready['value'] = True
    clock.now += STATIC_RETRY_SEC + 0.1
    body = cache.read('named_targets')

    assert body['quality'] == Quality.OK.value
    assert body['value'] == NAMED_TARGETS


def test_failure_is_not_retried_before_the_backoff_elapses() -> None:
    """백엔드가 죽어 있는 동안 매 조회마다 timeout 만큼 블로킹되면 안 된다."""
    calls: list[str] = []

    def resolver(method: str, timeout: float) -> Any:
        calls.append(method)
        raise StaticSourceUnavailable('not ready')

    cache, clock = make_cache(resolver)

    cache.read('named_targets')
    clock.now += STATIC_RETRY_SEC - 0.1
    cache.read('named_targets')

    assert len(calls) == 1

    clock.now += 0.2
    cache.read('named_targets')

    assert len(calls) == 2


def test_backend_error_reports_service_unavailable() -> None:
    def resolver(method: str, timeout: float) -> Any:
        raise TimeoutError('SRDF query timed out')

    cache, _ = make_cache(resolver)
    body = cache.read('named_targets')

    assert body['quality'] == Quality.SOURCE_UNAVAILABLE.value
    assert body['reason'] == Reason.SERVICE_UNAVAILABLE.value


def test_failed_refresh_keeps_the_cached_value() -> None:
    """정적 값은 런타임에 변하지 않는다 — 갱신 실패가 멀쩡한 값을 가리면 안 된다."""
    state = {'fail': False}

    def resolver(method: str, timeout: float) -> Any:
        if state['fail']:
            raise TimeoutError('backend went away')
        return NAMED_TARGETS

    cache, _ = make_cache(resolver)
    cache.read('named_targets')

    state['fail'] = True
    body = cache.read('named_targets', refresh=True)

    assert body['quality'] == Quality.OK.value
    assert body['value'] == NAMED_TARGETS


def test_missing_backend_method_is_a_config_error() -> None:
    """설정 오류는 저절로 낫지 않으므로 재시도 대상이 아니라 ERROR 다."""
    cache, _ = make_cache(lambda m, t: NAMED_TARGETS, method=None)

    body = cache.read('named_targets')

    assert body['quality'] == Quality.ERROR.value
    assert body['value'] is None
    # 원인 없이 ERROR 만 내보내면 클라이언트가 진단할 수 없다.
    assert 'backend.method' in body['error']['message']


def test_no_resolver_reports_source_node_down() -> None:
    cache, _ = make_cache(None)

    body = cache.read('named_targets')

    assert body['quality'] == Quality.SOURCE_UNAVAILABLE.value
    assert body['reason'] == Reason.SOURCE_NODE_DOWN.value


def test_concurrent_first_reads_fetch_once() -> None:
    """폴링 클라이언트가 N 명이어도 최초 조회는 한 번만 나가야 한다."""
    calls: list[str] = []
    entered = threading.Event()
    release = threading.Event()

    def resolver(method: str, timeout: float) -> Any:
        calls.append(method)
        entered.set()
        release.wait(timeout=2.0)
        return NAMED_TARGETS

    cache, _ = make_cache(resolver)
    results: list[Any] = []

    def read() -> None:
        results.append(cache.read('named_targets'))

    threads = [threading.Thread(target=read) for _ in range(5)]
    threads[0].start()
    # 첫 스레드가 resolver 안에 들어간 뒤 나머지를 풀어 경합을 만든다.
    assert entered.wait(timeout=2.0)
    for t in threads[1:]:
        t.start()
    release.set()
    for t in threads:
        t.join(timeout=2.0)

    assert len(calls) == 1
    assert all(r['value'] == NAMED_TARGETS for r in results)


def test_batch_read_fetches_static_source() -> None:
    cache, _ = make_cache(lambda m, t: NAMED_TARGETS)

    body = cache.read_many(['named_targets'])

    assert body['named_targets']['value'] == NAMED_TARGETS


def test_etag_changes_after_refresh_returns_a_new_value() -> None:
    """값이 바뀌면 조건부 요청이 304 로 막히면 안 된다."""
    values = [NAMED_TARGETS, {'panda_arm': ['ready']}]
    state = {'i': 0}

    def resolver(method: str, timeout: float) -> Any:
        value = values[state['i']]
        state['i'] = min(state['i'] + 1, len(values) - 1)
        return value

    cache, _ = make_cache(resolver)
    cache.read('named_targets')
    before = cache.entry('named_targets').etag
    cache.read('named_targets', refresh=True)

    assert cache.entry('named_targets').etag != before
