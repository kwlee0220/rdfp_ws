"""`robot_twin.session` 단위 테스트 — ROS 없이 동작한다."""

from __future__ import annotations

import pytest

from robot_twin.errors import ErrorCode, TwinError
from robot_twin.session import Phase, SessionStore, Status


class FakeClock:
    """테스트용 수동 시계 — 보관소 전체가 이 하나의 원천을 쓴다."""

    def __init__(self, now: float = 1000.0) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def _store(**kwargs) -> tuple[SessionStore, FakeClock]:
    clock = FakeClock()
    kwargs.setdefault('endpoint_prefix', '/api/v1/robot_twins/panda01')
    return SessionStore(clock=clock, **kwargs), clock


def test_create_issues_session_endpoint() -> None:
    store, clock = _store()

    s = store.create(operation='move_to_pose', resources=('arm',), inputs={})

    assert s.status is Status.RUNNING
    assert s.phase is Phase.PLANNING
    assert store.endpoint_of(s.id).endswith(f'/operations/sessions/{s.id}')
    # 권한 개념을 도입하지 않았으므로 항상 None 이다 (설계서 8.4).
    assert s.created_by is None


def test_same_resource_is_rejected_with_409() -> None:
    store, clock = _store()
    first = store.create(operation='move_to_joints', resources=('arm',), inputs={})

    with pytest.raises(TwinError) as exc:
        store.create(operation='move_to_pose', resources=('arm',), inputs={})

    err = exc.value
    assert err.code is ErrorCode.RESOURCE_BUSY
    assert err.http_status == 409
    assert err.details['occupied_by'] == store.endpoint_of(first.id)
    assert err.details['operation'] == 'move_to_joints'


def test_different_resources_run_in_parallel() -> None:
    store, clock = _store()

    store.create(operation='move_to_pose', resources=('arm',), inputs={})
    gripper = store.create(operation='move_gripper', resources=('gripper',), inputs={})

    assert gripper.status is Status.RUNNING


def test_null_resource_never_blocks() -> None:
    store, clock = _store()
    store.create(operation='move_to_pose', resources=('arm',), inputs={})

    a = store.create(operation='probe', resources=(), inputs={})
    b = store.create(operation='probe', resources=(), inputs={})

    assert a.id != b.id


def test_finish_releases_the_lock() -> None:
    store, clock = _store()
    first = store.create(operation='move_to_pose', resources=('arm',), inputs={})

    store.finish(first.id, Status.COMPLETED, outputs={'ok': True})
    second = store.create(operation='move_to_joints', resources=('arm',), inputs={})

    assert second.status is Status.RUNNING


def test_cancel_keeps_running_and_sets_canceling_phase() -> None:
    """로봇은 감속 정지에 시간이 걸리므로 즉시 종료시키지 않는다 (설계서 6.4)."""
    store, clock = _store()
    s = store.create(operation='move_to_pose', resources=('arm',), inputs={})

    store.request_cancel(s.id)

    assert s.status is Status.RUNNING
    assert s.phase is Phase.CANCELING
    assert s.cancel_requested is True


def test_cancelled_session_carries_no_error() -> None:
    store, clock = _store()
    s = store.create(operation='move_to_pose', resources=('arm',), inputs={})

    store.finish(s.id, Status.CANCELLED, error_code=ErrorCode.TIMEOUT,
                 error_message='should be ignored')

    assert s.error_code is None
    assert s.error_message is None


def test_first_terminal_reason_wins() -> None:
    """워치독과 실제 완료가 경합할 수 있다."""
    store, clock = _store()
    s = store.create(operation='move_to_pose', resources=('arm',), inputs={})

    store.finish(s.id, Status.FAILED, error_code=ErrorCode.TIMEOUT)
    store.finish(s.id, Status.COMPLETED)

    assert s.status is Status.FAILED
    assert s.error_code is ErrorCode.TIMEOUT


def test_estop_blocks_new_sessions_and_marks_running_ones() -> None:
    store, clock = _store()
    running = store.create(operation='move_to_pose', resources=('arm',), inputs={})

    affected = store.engage_estop()

    assert [s.id for s in affected] == [running.id]
    assert running.phase is Phase.CANCELING
    with pytest.raises(TwinError) as exc:
        store.create(operation='move_gripper', resources=('gripper',), inputs={})
    assert exc.value.code is ErrorCode.ESTOP_ENGAGED
    assert exc.value.http_status == 409

    store.release_estop()
    assert store.create(operation='move_gripper', resources=('gripper',), inputs={}) is not None


def test_remaining_ms_and_retry_after() -> None:
    store, clock = _store()
    first = store.create(operation='move_to_pose', resources=('arm',), inputs={})
    store.update(first.id, planned_duration_ms=20000)
    clock.advance(5.0)

    with pytest.raises(TwinError) as exc:
        store.create(operation='move_to_joints', resources=('arm',), inputs={})

    assert exc.value.details['estimated_remaining_ms'] == 15000
    assert exc.value.retry_after_sec == pytest.approx(15.0)


def test_resources_view() -> None:
    store, clock = _store()
    s = store.create(operation='move_to_pose', resources=('arm',), inputs={})

    view = store.resources()

    assert view['arm']['state'] == 'BUSY'
    assert view['arm']['session'] == store.endpoint_of(s.id)
    assert view['gripper']['state'] == 'FREE'


def test_terminal_sessions_expire_after_retention() -> None:
    store, clock = _store(retention_sec=900.0)
    s = store.create(operation='move_to_pose', resources=('arm',), inputs={})
    store.finish(s.id, Status.COMPLETED)

    clock.advance(899.0)
    assert store.get(s.id) is not None

    clock.advance(2.0)
    assert store.get(s.id) is None


def test_running_sessions_are_not_evicted_by_count_limit() -> None:
    """RUNNING 은 자원당 최대 1개라 누적되지 않는다 (설계서 6.12)."""
    store, clock = _store(max_sessions=2)
    keep = store.create(operation='move_to_pose', resources=('arm',), inputs={})

    for _ in range(5):
        done = store.create(operation='probe', resources=(), inputs={})
        store.finish(done.id, Status.COMPLETED)
        clock.advance(1.0)

    assert store.get(keep.id) is not None
    assert len(store.list(status=Status.COMPLETED)) == 2


def test_list_filters() -> None:
    store, clock = _store()
    a = store.create(operation='move_to_pose', resources=('arm',), inputs={})
    b = store.create(operation='move_gripper', resources=('gripper',), inputs={})
    store.finish(b.id, Status.COMPLETED)

    assert [s.id for s in store.list(operation='move_to_pose')] == [a.id]
    assert [s.id for s in store.list(status=Status.RUNNING)] == [a.id]
    assert len(store.list()) == 2


# ----- 다중 자원 락 -----

def test_multi_resource_lock_blocks_on_any_held_resource() -> None:
    """reset_scene 은 scene+arm 을 함께 잡는다 — 팔이 바쁘면 시작할 수 없다."""
    store = SessionStore(endpoint_prefix='/api')
    store.create(operation='move_linear', resources=('arm',), inputs={})

    with pytest.raises(TwinError):
        store.create(operation='reset_scene', resources=('scene', 'arm'), inputs={})


def test_multi_resource_lock_is_all_or_nothing() -> None:
    """일부만 잡히면 그 자원이 영원히 묶인다 — 검사와 점유가 같은 락 안에 있어야 한다."""
    store = SessionStore(endpoint_prefix='/api')
    store.create(operation='move_linear', resources=('arm',), inputs={})

    with pytest.raises(TwinError):
        store.create(operation='reset_scene', resources=('scene', 'arm'), inputs={})

    # scene 은 잡히지 않았어야 한다.
    store.create(operation='scene_only', resources=('scene',), inputs={})


def test_multi_resource_lock_releases_everything() -> None:
    store = SessionStore(endpoint_prefix='/api')
    session = store.create(operation='reset_scene', resources=('scene', 'arm'), inputs={})

    store.finish(session.id, Status.COMPLETED)

    store.create(operation='move_linear', resources=('arm',), inputs={})
    store.create(operation='scene_only', resources=('scene',), inputs={})


def test_reset_scene_blocks_arm_operations_while_running() -> None:
    """사전조건이 아니라 락인 이유 — 리셋 중 팔이 그 공간으로 들어오면 안 된다."""
    store = SessionStore(endpoint_prefix='/api')
    store.create(operation='reset_scene', resources=('scene', 'arm'), inputs={})

    with pytest.raises(TwinError):
        store.create(operation='move_linear', resources=('arm',), inputs={})


def test_known_resources_drives_the_resources_view() -> None:
    """이름을 코드에 두면 자원을 늘릴 때 응답에서 조용히 빠진다."""
    store = SessionStore(endpoint_prefix='/api', known_resources=('arm', 'gripper', 'scene'))

    assert set(store.resources()) == {'arm', 'gripper', 'scene'}
