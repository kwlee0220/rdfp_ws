"""`robot_twin.runtime` 의 전제 조건 검사 · 실패 분류 · 기동 순서 테스트.

노드나 executor 없이 런타임의 상태 처리와 백엔드 호출을 검증한다.
ROS가 필요한 클라이언트 초기화 테스트만 의존성이 없으면 건너뛴다.
"""

from __future__ import annotations

from typing import Any, Optional

import threading
import time
from types import SimpleNamespace

import pytest

from robot_twin import backend_registry  # noqa: E402
from robot_twin.config import OperationConfig  # noqa: E402
from robot_twin.errors import ErrorCode, TwinError  # noqa: E402
from robot_twin.runtime import RobotTwinRuntime  # noqa: E402
from robot_twin.session import SessionStore, Status  # noqa: E402

ARM_OP = OperationConfig(name='move_to_named_target', kind='async', resource='arm',
                         backend={'method': 'move_to_named_target_async'})
GRIPPER_OP = OperationConfig(
    name='move_gripper_to_target', kind='sync', resource='gripper',
    backend={'topic': '/gripper_cmds',
             'topic_type': 'rdfp_msgs/msg/GripperCommand',
             'result_variable': 'gripper_last_command_result',
             'labels': ['open']}
)
SESSION_OP = OperationConfig(name='start_episode', kind='sync',
                             backend={'session_control': 'start_episode'})


class _SessionClient:
    def __init__(self, *, ready: bool) -> None:
        self._ready = ready

    def is_ready(self) -> bool:
        return self._ready


class _Logger:
    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warnings: list[str] = []
        self.errors: list[str] = []

    def info(self, msg: str) -> None:
        self.infos.append(msg)

    def warning(self, msg: str) -> None:
        self.warnings.append(msg)

    def error(self, msg: str) -> None:
        self.errors.append(msg)


def _runtime(*, move_group: Optional[object],
             session_control: Optional[object] = None) -> RobotTwinRuntime:
    """`_check_preconditions` 가 읽는 상태만 채운 껍데기 런타임."""
    runtime = object.__new__(RobotTwinRuntime)
    runtime._mg_lock = threading.Lock()
    runtime._move_group = move_group
    runtime._move_group_error = None if move_group else 'not built yet'
    runtime._session_control = session_control
    runtime._node = object()
    runtime._logger = _Logger()
    return runtime


def test_arm_operation_requires_move_group() -> None:
    with pytest.raises(TwinError) as exc:
        _runtime(move_group=None)._check_preconditions(ARM_OP)

    assert exc.value.http_status == 503


def test_gripper_operation_does_not_require_move_group() -> None:
    """그리퍼는 액션을 직접 호출한다 — 팔 스택이 없어도 쓸 수 있어야 한다.

    예전에는 전제 조건이 연산별로 갈리지 않아, MoveIt 이 아직 안 뜬 동안 그리퍼
    연산까지 `503` 으로 거부되었다.
    """
    _runtime(move_group=None)._check_preconditions(GRIPPER_OP)   # 예외가 없어야 한다


def test_arm_operation_passes_once_move_group_is_ready() -> None:
    _runtime(move_group=object())._check_preconditions(ARM_OP)


# ----- named target — 없는 이름은 접수 단계에서 `400` --------------------------

class _NamedTargets:
    """`get_named_targets` 만 가진 MoveGroup 클라이언트 대역. 받은 인자를 남긴다."""

    def __init__(self, names: Optional[list[str]] = None, *,
                 error: Optional[Exception] = None) -> None:
        self._names = names
        self._error = error
        self.calls: list[dict] = []

    def get_named_targets(self, **kwargs: Any) -> list[str]:
        self.calls.append(kwargs)
        if self._error is not None:
            raise self._error
        return self._names


def test_unknown_named_target_is_refused_with_400() -> None:
    """예전에는 접수된 뒤 실행 단계에서 `EXECUTION_ABORTED` 로 끝나 컨트롤러 이상처럼 보였다."""
    runtime = _runtime(move_group=_NamedTargets(['extended', 'ready']))

    with pytest.raises(TwinError) as exc:
        runtime._validate_inputs(ARM_OP, {'target': 'home'})

    assert exc.value.code is ErrorCode.INVALID_INPUT
    assert "'home'" in str(exc.value)
    assert "['extended', 'ready']" in str(exc.value)


def test_known_named_target_is_accepted() -> None:
    client = _NamedTargets(['extended', 'ready'])

    _runtime(move_group=client)._validate_inputs(ARM_OP, {'target': 'ready'})

    # 접수는 HTTP 스레드에서 일어난다 — 자체 spin 을 하면 이중 spin 이 된다.
    assert client.calls[0]['externally_spun'] is True


def test_named_target_check_is_skipped_when_the_list_is_unavailable() -> None:
    """목록을 모르는데 거부하면 올바른 이름까지 `400` 이 된다 — 실행 단계에 맡긴다."""
    runtime = _runtime(move_group=_NamedTargets(error=TimeoutError('SRDF lookup timed out')))

    runtime._validate_inputs(ARM_OP, {'target': 'ready'})

    assert any('SRDF lookup timed out' in m for m in runtime._logger.warnings)


# ----- 세션 연산 — 세션을 만들기 전에 거절한다 --------------------------------

def test_session_operation_without_the_node_is_refused_with_503() -> None:
    """아무것도 실행되지 않았다 — `200 + EXECUTION_ABORTED` 가 아니다.

    예전에는 세션을 만든 뒤 핸들러에서 걸려 docstring 과 달리 `EXECUTION_ABORTED` 로
    나갔다. 클라이언트가 "실행하다 죽었다"로 읽고 재시도하게 된다.
    """
    runtime = _runtime(move_group=object(), session_control=_SessionClient(ready=False))

    with pytest.raises(TwinError) as exc:
        runtime._check_preconditions(SESSION_OP)

    assert exc.value.http_status == 503
    assert exc.value.code is ErrorCode.PRECONDITION_FAILED
    assert 'session_control_node' in exc.value.message


def test_session_operation_without_the_collection_layer_is_refused_with_503(monkeypatch) -> None:
    monkeypatch.setattr(backend_registry, 'load_provider', lambda name: None)

    with pytest.raises(TwinError) as exc:
        _runtime(move_group=object())._check_preconditions(SESSION_OP)

    assert exc.value.http_status == 503
    assert 'rdfp' in exc.value.message


def test_session_operation_does_not_need_move_group() -> None:
    """세션 경계는 팔과 무관하다 — MoveIt 이 늦게 떠도 세션은 열 수 있어야 한다."""
    _runtime(move_group=None, session_control=_SessionClient(ready=True))._check_preconditions(
        SESSION_OP)


def test_arm_operation_does_not_look_at_session_control() -> None:
    _runtime(move_group=object(), session_control=_SessionClient(ready=False))._check_preconditions(
        ARM_OP)


def _factory_recorder(created: list[dict]) -> Any:
    def _factory(node: Any, **kwargs: Any) -> _SessionClient:
        created.append(kwargs)
        return _SessionClient(ready=True)
    return _factory


def test_session_client_is_created_at_startup_when_session_operations_exist(monkeypatch) -> None:
    """갓 만든 서비스 클라이언트는 매칭 전이라 첫 요청에서 만들면 떠 있는 노드를 없다고 본다."""
    created: list[dict] = []
    monkeypatch.setattr(backend_registry, 'load_provider', lambda name: _factory_recorder(created))
    runtime = _runtime(move_group=None)
    runtime.config = SimpleNamespace(operations=[ARM_OP, SESSION_OP])

    runtime._start_session_control()

    # 블로킹 대기 없이 만든다 — 노드가 없을 때 기동이 묶이면 안 된다.
    assert created == [{'wait_timeout_sec': 0.0}]


def test_startup_creates_the_session_client(monkeypatch) -> None:
    """`start()` 가 실제로 그 경로를 탄다 — 메서드만 있고 안 불리면 첫 요청이 `503` 이다."""
    created: list[dict] = []
    monkeypatch.setattr(backend_registry, 'load_provider', lambda name: _factory_recorder(created))
    runtime = _runtime(move_group=None)
    runtime.config = SimpleNamespace(operations=[SESSION_OP])
    runtime.variables = SimpleNamespace(start=lambda: None, refresh_availability=lambda: None)
    runtime._command_publishers = {}
    runtime._node = SimpleNamespace(create_timer=lambda period, callback: None)

    runtime.start()

    assert len(created) == 1


def test_session_client_is_not_created_without_session_operations(monkeypatch) -> None:
    created: list[dict] = []
    monkeypatch.setattr(backend_registry, 'load_provider', lambda name: _factory_recorder(created))
    runtime = _runtime(move_group=None)
    runtime.config = SimpleNamespace(operations=[ARM_OP, GRIPPER_OP])

    runtime._start_session_control()

    assert created == []


# ----- 실패 분류 — 계획 실패만 `PLANNING_FAILED` -----------------------------

def _executing_runtime(exc: Exception) -> RobotTwinRuntime:
    runtime = _runtime(move_group=object())
    runtime.sessions = SessionStore()

    def _dispatch(op: OperationConfig, session: Any, *, timeout: float) -> dict:
        raise exc

    runtime._dispatch = _dispatch
    runtime._stop_outputs = dict
    return runtime


def _run(runner: str, exc: Exception) -> Any:
    runtime = _executing_runtime(exc)
    session = runtime.sessions.create(operation=ARM_OP.name, resources=('arm',), inputs={})
    getattr(runtime, runner)(ARM_OP, session)
    return runtime.sessions.get(session.id)


@pytest.mark.parametrize('runner', ['_run_sync', '_run_async'])
def test_planning_failure_is_reported_as_planning_failed(runner: str) -> None:
    """로봇이 움직이지 않았고 같은 목표로는 결과가 같다 — 클라이언트는 목표를 바꿔야 한다."""
    errors = pytest.importorskip('robot_control.errors')

    done = _run(runner, errors.PlanningError('no IK solution'))

    assert done.status is Status.FAILED
    assert done.error_code is ErrorCode.PLANNING_FAILED
    assert 'no IK solution' in done.error_message


@pytest.mark.parametrize('runner', ['_run_sync', '_run_async'])
def test_execution_failure_stays_execution_aborted(runner: str) -> None:
    done = _run(runner, RuntimeError('controller aborted the trajectory'))

    assert done.error_code is ErrorCode.EXECUTION_ABORTED


def test_cancelled_planning_failure_is_still_cancelled() -> None:
    """취소를 요청했으면 백엔드 예외는 그 결과다 — 실패 코드를 붙이지 않는다."""
    errors = pytest.importorskip('robot_control.errors')
    runtime = _executing_runtime(errors.PlanningError('preempted by cancel'))
    session = runtime.sessions.create(operation=ARM_OP.name, resources=('arm',), inputs={})
    runtime.sessions.request_cancel(session.id)

    runtime._run_async(ARM_OP, session)

    done = runtime.sessions.get(session.id)
    assert done.status is Status.CANCELLED
    assert done.error_code is None


def test_async_session_reports_executing_from_the_start() -> None:
    """계획 국면은 트윈에서 보이지 않는다 — `PLANNING` 은 응답에 나온 적이 없었다."""
    session = SessionStore().create(operation=ARM_OP.name, resources=('arm',), inputs={})

    assert session.phase.value == 'EXECUTING'


# ----- 기동 — 고아 goal 정리는 클라이언트를 내놓기 전에 ----------------------

class _MoveGroup:
    def __init__(self, runtime: RobotTwinRuntime, *, error: Optional[Exception] = None) -> None:
        self._runtime = runtime
        self._error = error
        self.published_at_cancel: list[Any] = []

    def cancel_orphaned_goals(self, timeout_sec: float) -> bool:
        self.published_at_cancel.append(self._runtime._move_group)
        if self._error is not None:
            raise self._error
        return True


def _start(monkeypatch, *, error: Optional[Exception] = None) -> tuple[RobotTwinRuntime, Any]:
    factory = pytest.importorskip('robot_control.moveit.move_group_factory')
    runtime = _runtime(move_group=None)
    runtime.config = SimpleNamespace(moveit=SimpleNamespace(
        move_group_mode='jtc', client_kwargs=dict, gravity_compensator=lambda: None))
    client = _MoveGroup(runtime, error=error)
    monkeypatch.setattr(factory, 'create_move_group_client', lambda node, **kw: client)

    runtime.start_move_group_async()

    deadline = time.monotonic() + 5.0
    while runtime._move_group is None and runtime._move_group_error in (None, 'not built yet'):
        assert time.monotonic() < deadline, 'MoveGroup init thread did not finish'
        time.sleep(0.01)
    return runtime, client


def test_orphaned_goals_are_cancelled_before_the_client_is_published(monkeypatch) -> None:
    """내놓은 뒤에 끊으면 그 사이에 접수된 새 연산의 궤적까지 끊는다.

    예전 구현은 클라이언트가 만들어지기 **전에** 불려 아무것도 하지 않았다.
    """
    runtime, client = _start(monkeypatch)

    assert client.published_at_cancel == [None]
    assert runtime._move_group is client


def test_failed_orphan_cancel_does_not_block_readiness(monkeypatch) -> None:
    runtime, client = _start(monkeypatch, error=RuntimeError('cancel service exploded'))

    assert runtime._move_group is client
    assert any('cancel service exploded' in m for m in runtime._logger.warnings)


# ----- 시간 초과 — 끝내기 전에 정지를 보낸다 ---------------------------------

class _ArmClient:
    """`cancel()` 호출을 세는 MoveGroup 클라이언트 대역. 부를 때의 연산 세션 상태를 남긴다."""

    def __init__(self, *, halts: bool = True) -> None:
        self._halts = halts
        self.cancels = 0
        self.terminal_at_cancel: list[bool] = []
        self.session: Optional[Any] = None

    def cancel(self) -> bool:
        self.cancels += 1
        if self.session is not None:
            self.terminal_at_cancel.append(self.session.is_terminal)
        return self._halts


def _overdue(op: OperationConfig, *, halts: bool = True, elapsed: float = 100.0,
             inputs: Optional[dict] = None) -> tuple[RobotTwinRuntime, _ArmClient, Any]:
    """`elapsed` 초가 지난 연산 세션 하나를 가진 런타임. 기본 상한은 60초다."""
    client = _ArmClient(halts=halts)
    runtime = _runtime(move_group=client)
    runtime.sessions = SessionStore(clock=lambda: 0.0)
    runtime.config = SimpleNamespace(operation=lambda name: op)
    runtime._clock = lambda: elapsed
    runtime._stop_outputs = dict
    session = runtime.sessions.create(operation=op.name, resources=tuple(op.resource_names),
                                      inputs=inputs or {})
    client.session = session
    return runtime, client, session


def test_watchdog_stops_the_arm_before_ending_an_overdue_session() -> None:
    """예전에는 끝내기만 했다 — 로봇은 계속 움직이는데 자원이 풀리고 취소도 안 됐다.

    정지가 끝내기 **전**이어야 한다. 끝내면 자원이 풀려 그 사이에 접수된 새 연산의
    동작까지 끊을 수 있다.
    """
    runtime, client, session = _overdue(ARM_OP)

    runtime._tick_watchdog()

    assert client.cancels == 1
    assert client.terminal_at_cancel == [False]
    assert session.status is Status.FAILED
    assert session.error_code is ErrorCode.TIMEOUT
    assert 'stop sent' in session.error_message


@pytest.mark.parametrize('op', [
    GRIPPER_OP,
    SESSION_OP,
    OperationConfig(name='reset_scene', kind='sync', resource=['scene', 'arm']),
    OperationConfig(name='move_gripper', kind='async', resource='gripper'),
])
def test_unsupported_cancel_never_stops_the_arm(op) -> None:
    runtime, client, session = _overdue(op)

    with pytest.raises(TwinError) as exc:
        runtime.cancel_session(session)

    assert exc.value.http_status == 501
    assert exc.value.code is ErrorCode.CANCEL_UNSUPPORTED
    assert client.cancels == 0
    assert session.status is Status.RUNNING
    assert not session.cancel_requested
    assert session.phase.value == 'EXECUTING'


@pytest.mark.parametrize('halts', [True, False])
def test_arm_cancel_marks_requested_only_after_delivery(halts) -> None:
    runtime, client, session = _overdue(ARM_OP, halts=halts)

    if halts:
        runtime.cancel_session(session)
    else:
        with pytest.raises(TwinError) as exc:
            runtime.cancel_session(session)
        assert exc.value.code is ErrorCode.CANCEL_UNSUPPORTED

    assert client.cancels == 1
    assert session.cancel_requested is halts
    assert session.phase.value == ('CANCELING' if halts else 'EXECUTING')


@pytest.mark.parametrize('fails', [False, True])
def test_undelivered_estop_does_not_turn_later_completion_into_cancellation(fails) -> None:
    runtime, client, session = _overdue(ARM_OP, halts=False)

    result = runtime.engage_estop()

    assert result == {'locked': True, 'halted': False, 'in_flight_sessions': 1}
    assert runtime.sessions.estop_engaged
    assert session.status is Status.RUNNING
    assert not session.cancel_requested
    assert session.phase.value == 'EXECUTING'

    def dispatch(*args, **kwargs):
        if fails:
            raise RuntimeError('controller failed')
        return {'closed_loop': False}

    runtime._dispatch = dispatch
    runtime._run_async(ARM_OP, session)
    assert session.status is (Status.FAILED if fails else Status.COMPLETED)
    assert client.cancels == 1


def test_delivered_estop_keeps_failure_reason_after_worker_returns() -> None:
    runtime, client, session = _overdue(ARM_OP)

    assert runtime.engage_estop()['halted'] is True
    runtime._dispatch = lambda *args, **kwargs: {}
    runtime._run_async(ARM_OP, session)

    assert client.cancels == 1
    assert session.status is Status.FAILED
    assert session.error_code is ErrorCode.ESTOP_ENGAGED


def test_watchdog_does_not_stop_the_arm_for_a_non_arm_session() -> None:
    """정지는 팔 전체에 걸린다 — 그리퍼의 시간 초과로 다른 연산 세션의 팔 동작을 끊으면 안 된다."""
    runtime, client, session = _overdue(GRIPPER_OP)

    runtime._tick_watchdog()

    assert client.cancels == 0
    assert session.error_code is ErrorCode.TIMEOUT
    assert 'stop' not in session.error_message


def test_watchdog_reports_an_undeliverable_stop() -> None:
    runtime, client, session = _overdue(ARM_OP, halts=False)

    runtime._tick_watchdog()

    assert session.error_code is ErrorCode.TIMEOUT
    assert 'could NOT' in session.error_message
    assert any('could NOT be halted' in m for m in runtime._logger.errors)


def test_watchdog_honours_max_duration_sec() -> None:
    runtime, client, session = _overdue(ARM_OP, elapsed=100.0, inputs={'max_duration_sec': 200})

    runtime._tick_watchdog()

    assert client.cancels == 0
    assert session.status is Status.RUNNING


def _timing_out(runtime: RobotTwinRuntime, seen: list[float]) -> None:
    def _dispatch(op: OperationConfig, session: Any, *, timeout: float) -> dict:
        seen.append(timeout)
        raise TimeoutError('move_to_named_target did not finish')

    runtime._dispatch = _dispatch


def test_server_timeout_stops_the_arm() -> None:
    runtime, client, session = _overdue(ARM_OP, elapsed=0.0)
    _timing_out(runtime, [])

    runtime._run_async(ARM_OP, session)

    assert client.cancels == 1
    assert session.error_code is ErrorCode.TIMEOUT
    assert 'stop sent' in session.error_message


def test_server_timeout_after_the_watchdog_does_not_stop_again() -> None:
    """워치독이 끝낸 뒤에는 자원이 풀려 다른 연산 세션이 팔을 쓰고 있을 수 있다."""
    runtime, client, session = _overdue(ARM_OP)
    runtime._tick_watchdog()
    _timing_out(runtime, [])

    runtime._run_async(ARM_OP, session)

    assert client.cancels == 1                      # 워치독의 한 번뿐이다
    assert 'max_duration_sec' in session.error_message


@pytest.mark.parametrize('inputs,expected', [({}, 60.0), ({'max_duration_sec': 150}, 150.0)])
def test_async_wait_uses_the_same_limit_as_the_watchdog(inputs, expected) -> None:
    """예전에는 `default_timeout_sec` 로 고정이라 `max_duration_sec` 를 늘려도 먼저 끊겼다."""
    runtime, _, session = _overdue(ARM_OP, elapsed=0.0, inputs=inputs)
    seen: list[float] = []
    _timing_out(runtime, seen)

    runtime._run_async(ARM_OP, session)

    assert seen == [expected]
