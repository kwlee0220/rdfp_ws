"""ROS 런타임 — 변수 캐시와 연산 백엔드를 API 에 연결한다 (설계서 2.2, 2.6, 6장).

본 모듈이 rclpy 와 :mod:`robot_control.moveit` 을 잇는 유일한 지점이다. API 계층은 이
클래스가 제공하는 표면만 보므로 ROS 를 직접 모른다.

스레드 규칙
-----------
- ROS executor 는 **별도 스레드**에서 spin 한다.
- HTTP 핸들러는 캐시와 세션 보관소만 만진다.
- ``MoveGroupClient`` 의 **동기 메서드를 호출하지 않는다.** ``*_async`` 계열만
  쓰고 ``externally_spun=True`` 를 전달한다 — executor 가 다른 스레드에서 Node 를
  spin 중임을 알려야 한다 (설계서 2.3).
"""

from __future__ import annotations

from typing import Any, Callable, Optional

import inspect
import threading
import time

from rclpy.node import Node

from robot_twin.backends import PreconditionFailed
from robot_twin.config import OperationConfig, TwinConfig
from robot_twin.errors import (
    ErrorCode, cancel_unsupported, invalid_input, precondition_failed
)
from robot_twin.session import Phase, Session, SessionStore, Status
from robot_twin.variables import StaticSourceUnavailable, VariableCache

# 소스 가용성(퍼블리셔 존재 여부) 갱신 주기. age 만으로는 "퍼블리셔 없음"과
# "퍼블리셔 멈춤"을 구분할 수 없어 그래프를 직접 조회해야 한다 (설계서 5.1).
_AVAILABILITY_PERIOD_SEC = 2.0

# 워치독 점검 주기. `max_duration_sec` 초과 세션을 강제 중단한다 (설계서 6.5).
_WATCHDOG_PERIOD_SEC = 1.0


class RobotTwinRuntime:
    """트윈의 ROS 측 런타임."""

    def __init__(self, node: Node, config: TwinConfig, *,
                 clock: Callable[[], float] = time.time) -> None:
        self.config = config
        self._node = node
        self._clock = clock
        self._logger = node.get_logger()

        base = f'/api/v1/robot_twins/{config.twin.id}'
        self.sessions = SessionStore(
            retention_sec=config.sessions.retention_sec,
            max_sessions=config.sessions.max_sessions,
            endpoint_prefix=base, clock=clock,
            # `/resources` 에 실을 이름은 설정에서 모은다 — 코드에 목록을 두면
            # 자원을 늘릴 때 응답에서 조용히 빠진다.
            known_resources=tuple(sorted(
                {name for op in config.operations for name in op.resource_names}))
        )
        self.variables = VariableCache(node, config.variables, clock=clock,
                                       static_resolver=self._fetch_static)

        # 명령 토픽 퍼블리셔. `backend.topic` 을 선언한 연산마다 기동 시 하나씩
        # 만들어 둔다 (`_start_command_publishers`).
        self._command_publishers: dict[str, Any] = {}

        # MoveGroup 클라이언트는 기동 4단계에서 백그라운드로 만든다. 실패해도
        # 프로세스는 유지되어야 `/health` 로 상태를 알릴 수 있다 (설계서 2.6).
        self._move_group: Optional[Any] = None
        self._move_group_error: Optional[str] = None
        self._mg_lock = threading.Lock()

        # 세션/에피소드 경계 제어. 트윈은 `/session` 을 직접 발행하지 않고
        # `session_control_node` 의 클라이언트가 된다 — 상태 기계를 한 곳에 두기
        # 위함이다. TRANSIENT_LOCAL 발행자가 둘이 되면 늦게 붙은 recorder 가 두
        # 발행자의 latched 샘플을 각각 받아 조용히 어긋난다.
        self._session_control: Optional[Any] = None

    # ------------------------------------------------------------------
    # 기동
    # ------------------------------------------------------------------

    def start(self) -> None:
        """구독·발행을 등록하고 주기 타이머를 건다 (기동 2단계)."""
        self.variables.start()
        self._start_command_publishers()
        self._node.create_timer(_AVAILABILITY_PERIOD_SEC, self.variables.refresh_availability)
        self._node.create_timer(_WATCHDOG_PERIOD_SEC, self._tick_watchdog)

    def _start_command_publishers(self) -> None:
        """``backend.topic`` 을 선언한 연산의 퍼블리셔를 **기동 시점에** 만든다.

        호출 시점에 만들면 DDS 매칭 전에 publish 하게 되어 **첫 명령이 조용히
        사라진다.** 구독자가 붙을 시간을 벌기 위해 미리 열어 둔다.
        """
        from robot_twin.variables import import_message_type

        for op in self.config.operations:
            topic = op.backend.get('topic')
            type_name = op.backend.get('topic_type')
            if not topic or not type_name:
                continue
            try:
                msg_type = import_message_type(type_name)
            except ValueError as exc:
                self._logger.error(f"operation '{op.name}': {exc}")
                continue
            self._command_publishers[topic] = self._node.create_publisher(msg_type, topic, 10)
            self._logger.info(f"operation '{op.name}' -> {topic} ({type_name})")

    def command_publisher(self, topic: str) -> Any:
        """기동 시 만들어 둔 명령 퍼블리셔. 없으면 ``None``."""
        return self._command_publishers.get(topic)

    def session_control(self) -> Any:
        """`session_control_node` 클라이언트. 최초 호출 시 만든다.

        **`wait_timeout_sec=0` 으로 만든다.** 생성자의 기본 동작은 6개 서비스가
        준비될 때까지 블로킹인데, 그러면 `session_control_node` 가 아직 없을 때
        HTTP 요청 스레드가 그대로 묶인다. 준비 여부는 호출 시점에 `is_ready()` 로
        보고 `PRECONDITION_FAILED` 로 거절한다.

        MoveGroup 과 달리 기동 시 미리 만들지 않는 이유는, 서비스 클라이언트 생성이
        가볍고 세션 연산을 쓰지 않는 트윈도 있기 때문이다.

        제공자(`rdfp` 수집 계층)가 설치되어 있지 않으면 ``None`` 을 돌려준다 —
        트윈은 제어 계층 위에 있으므로 수집 계층을 직접 import 하지 않는다
        (:mod:`robot_twin.backend_registry`).
        """
        if self._session_control is None:
            from robot_twin.backend_registry import load_provider
            factory = load_provider('session_control')
            if factory is None:
                return None
            self._session_control = factory(self._node, wait_timeout_sec=0.0)
        return self._session_control

    def start_move_group_async(self) -> None:
        """MoveGroup 클라이언트를 백그라운드 스레드에서 만든다 (기동 4단계).

        HTTP 서버는 이미 떠 있으므로, 여기서 실패해도 ``/health`` 가 그 사실을
        드러낸다. 연산 요청은 ``PRECONDITION_FAILED`` 로 거부된다.
        """
        def _build() -> None:
            mode = self.config.moveit.move_group_mode
            try:
                # 지연 import — ROS 없이 config/session 모듈을 쓰는 경로를 막지 않는다.
                from robot_control.moveit.move_group_factory import create_move_group_client
                # mode 는 항상 명시한다. 'auto' 는 설정 단계에서 이미 거부된다.
                client = create_move_group_client(self._node, mode=mode)
                with self._mg_lock:
                    self._move_group = client
                    self._move_group_error = None
                self._logger.info(f'MoveGroup client ready (mode={mode})')
            except Exception as exc:
                with self._mg_lock:
                    self._move_group = None
                    self._move_group_error = str(exc)
                self._logger.error(f'MoveGroup client init failed (mode={mode}): {exc}')

        threading.Thread(target=_build, name='twin-movegroup-init', daemon=True).start()

    # ------------------------------------------------------------------
    # 상태 변수 — API 표면
    # ------------------------------------------------------------------

    def _fetch_static(self, method: str, timeout: float) -> Any:
        """정적 소스의 백엔드 메서드를 호출한다 (설계서 4.3).

        :class:`~robot_twin.variables.VariableCache` 에 주입되는 resolver 다. 캐시가
        런타임을 직접 참조하면 순환 import 가 되므로 호출 가능 객체 하나만 넘긴다.

        ``externally_spun=True`` 를 넘길 수 있는 메서드에는 반드시 넘긴다 — 이
        호출은 HTTP 스레드에서 일어나는데 executor 가 다른 스레드에서 같은 Node 를
        spin 중이므로, 백엔드가 자체 spin 을 하면 이중 spin 이 된다 (설계서 2.3).

        Args:
            method: ``MoveGroupClient`` 의 메서드 이름.
            timeout: 호출 상한(초).

        Returns:
            메서드의 반환값.

        Raises:
            StaticSourceUnavailable: 클라이언트가 아직 준비되지 않았을 때. 캐시가
                이를 일시적 상황으로 보고 나중에 다시 시도한다.
            ValueError: 설정에 적힌 메서드가 클라이언트에 없을 때.
        """
        with self._mg_lock:
            client = self._move_group
        if client is None:
            raise StaticSourceUnavailable('MoveGroup client is not ready')

        fn = getattr(client, method, None)
        if not callable(fn):
            raise ValueError(f"MoveGroup client has no method '{method}'")

        # 백엔드 메서드의 서명을 강제하지 않는다 — 받는 키워드만 골라 넘긴다.
        params = inspect.signature(fn).parameters
        kwargs: dict[str, Any] = {}
        if 'timeout' in params:
            kwargs['timeout'] = timeout
        if 'externally_spun' in params:
            kwargs['externally_spun'] = True
        return fn(**kwargs)

    def read_variable(self, name: str, *, refresh: bool = False) -> Optional[dict[str, Any]]:
        """단일 변수 envelope.

        Args:
            name: 변수 이름.
            refresh: 정적 소스의 캐시를 강제로 다시 채운다 (설계서 4.3). 정적
                소스는 자동 무효화가 없으므로 이것이 트윈 재시작 외의 유일한 갱신
                수단이다.
        """
        return self.variables.read(name, refresh=refresh)

    def read_variables(self, names: list[str]) -> dict[str, Any]:
        """배치 조회."""
        return self.variables.read_many(names)

    def variable_etag(self, name: str) -> Optional[str]:
        """변수의 현재 ETag."""
        entry = self.variables.entry(name)
        return None if entry is None else entry.etag

    # ------------------------------------------------------------------
    # 헬스
    # ------------------------------------------------------------------

    def health(self) -> dict[str, Any]:
        """ROS 연결·준비 상태 (설계서 5.1 의 "둘 다 확인해야 한다").

        정적 소스는 **여기서 조회하지 않는다** — health 는 블로킹되면 안 된다.
        따라서 아직 한 번도 읽히지 않은 정적 변수는 ``NO_DATA`` 로 보이며, 해당
        변수를 실제로 조회하면 그때 채워진다 (설계서 4.3의 lazy 정책).
        """
        with self._mg_lock:
            mg_ready = self._move_group is not None
            mg_error = self._move_group_error

        return {
            'twin': self.config.twin.id,
            'ros': 'CONNECTED',
            'move_group': 'READY' if mg_ready else 'NOT_READY',
            'move_group_mode': self.config.moveit.move_group_mode,
            'move_group_error': mg_error,
            'estop': 'ENGAGED' if self.sessions.estop_engaged else 'RELEASED',
            'variables': {
                name: self.variables.entry(name).status().quality.value
                for name in self.variables.names()
            },
            'running_sessions': len(self.sessions.running())
        }

    # ------------------------------------------------------------------
    # 연산
    # ------------------------------------------------------------------

    def start_operation(self, name: str, inputs: dict[str, Any]) -> Session:
        """연산을 시작한다.

        순서가 중요하다 — **전제 조건 검사와 입력 검증을 자원 락보다 먼저** 한다.
        그래야 잘못된 요청이 자원을 잠갔다 푸는 낭비가 없다.

        Raises:
            TwinError: 전제 조건 미충족(``503``), 입력 오류(``400``),
                자원 점유(``409``), E-stop(``409``).
        """
        op = self.config.operation(name)
        if op is None:
            raise invalid_input(f'unknown operation: {name}')

        self._check_preconditions(op)
        self._validate_inputs(op, inputs)

        session = self.sessions.create(operation=op.name, resources=op.resource_names,
                                       inputs=inputs)
        # 감사 로그 — 인증이 없으므로 "누가"는 API 계층이 IP 로 남긴다 (설계서 8.2).
        self._logger.info(f"operation '{op.name}' started (session={session.id}, inputs={inputs})")

        if op.kind == 'sync':
            self._run_sync(op, session)
        else:
            threading.Thread(target=self._run_async, args=(op, session),
                             name=f'twin-op-{session.id}', daemon=True).start()
        return session

    def _stop_backend(self) -> bool:
        """실행 중인 동작을 실제로 멈춘다.

        Returns:
            정지 요청을 전달했으면 ``True``. 중단할 동작이 없거나 수단이 없으면
            ``False``.

        ``MoveGroupClient.cancel()`` 이 구현별 중단 수단을 캡슐화한다 — JTC 는
        추적 중인 액션 goal 취소, JGPC 는 스트리밍 정지. **비동기다**: ``True`` 는
        취소를 보냈다는 뜻이지 로봇이 이미 멈췄다는 뜻이 아니다.
        """
        with self._mg_lock:
            client = self._move_group
        if client is None:
            return False

        # 두 구현 모두 실제로 로봇을 멈추는 경로가 확인되었다 (실측).
        #
        # - JTC : 컨트롤러의 `FollowJointTrajectory` 액션에 취소를 보낸다.
        #         상위 `MoveGroup` 액션 취소만으로는 멈추지 않는다.
        # - JGPC: `stop_streaming()` 이 명령 발행을 멈춘다.
        try:
            return bool(client.cancel())
        except Exception as exc:
            self._logger.warning(f'cancel failed: {exc}')
            return False

    def cancel_session(self, session: Session) -> None:
        """취소를 접수하고 백엔드에 전달한다.

        **정지를 전달할 수단이 없으면 취소를 접수하지 않고 거부한다.** 접수하고
        ``CANCELLED`` 로 보고하면 로봇은 계속 움직이는데 클라이언트는 멈췄다고 믿게
        되어, 안전 관점에서 기능 부재보다 나쁘다.

        Raises:
            TwinError: 정지 수단이 없을 때 (``501``).
        """
        if not self._stop_backend():
            self._logger.warning(
                f'cancel rejected (session={session.id}): no stop mechanism for this backend'
            )
            raise cancel_unsupported(session.operation)

        self.sessions.request_cancel(session.id)
        self._logger.info(f'cancel requested (session={session.id})')

    def engage_estop(self) -> dict[str, Any]:
        """비상정지.

        **새 연산을 거부하는 잠금은 항상 확실히 동작한다.** 반면 이미 실행 중인
        동작을 멈추는 것은 백엔드에 정지 수단이 있을 때만 가능하다.

        Returns:
            ``halted`` 가 ``False`` 면 진행 중이던 동작은 **계속 실행된다.** 응답이
            이 사실을 그대로 전달해야 한다.
        """
        affected = self.sessions.engage_estop()
        halted = self._stop_backend()

        if halted:
            self._logger.warning(f'E-STOP engaged; halted {len(affected)} session(s)')
            for session in affected:
                self.sessions.finish(
                    session.id, Status.FAILED, error_code=ErrorCode.ESTOP_ENGAGED,
                    error_message='aborted by emergency stop', outputs=self._stop_outputs()
                )
        else:
            # 세션을 종료 상태로 바꾸지 않는다 — 로봇이 실제로 멈추지 않았으므로
            # FAILED 로 표시하면 상태가 거짓이 된다. 동작은 스스로 끝난다.
            self._logger.error(
                f'E-STOP engaged but {len(affected)} in-flight motion(s) CANNOT be halted: '
                'no stop mechanism for this backend. The robot keeps moving. '
                'Use a physical emergency stop'
            )

        return {'locked': True, 'halted': halted, 'in_flight_sessions': len(affected)}

    def release_estop(self) -> None:
        """비상정지 해제."""
        self.sessions.release_estop()
        self._logger.warning('E-STOP released')

    def cancel_all_on_startup(self) -> None:
        """기동 시 고아 goal 을 정리한다 (설계서 6.13).

        트윈이 죽어도 MoveIt 쪽 goal 은 계속 실행 중일 수 있다. 로봇이 움직이는데
        취소권을 가진 주체가 없는 상태를 만들지 않기 위해 기동 시 일괄 중단한다.
        """
        with self._mg_lock:
            client = self._move_group
        stop = getattr(client, 'stop_streaming', None) if client is not None else None
        if callable(stop):
            try:
                stop()
                self._logger.info('cancelled in-flight goals on startup')
            except Exception as exc:
                self._logger.warning(f'startup goal cancel failed: {exc}')

    # ------------------------------------------------------------------
    # 내부 — 검사
    # ------------------------------------------------------------------

    def _check_preconditions(self, op: OperationConfig) -> None:
        """시작 전 전제 조건을 검사한다 (설계서 6.8).

        **비용 0 인 로컬 조회만** 한다. 컨트롤러 상태(`list_controllers` 서비스)는
        조회하지 않는다 — 왕복 지연이 생기고 캐시하면 판정이 stale 해진다.

        **연산이 실제로 쓰는 백엔드만 본다.** 그리퍼 연산은 액션을 직접 호출하므로
        MoveGroup 준비 여부와 무관하다 — 팔 스택이 늦게 뜨거나 아예 없어도 그리퍼는
        쓸 수 있어야 한다.

        액션 서버 ready 여부(설계서 6.8 표의 두 번째 항목)는 **검사하지 않는다.**
        확인하려면 여기서 액션 클라이언트를 만들어야 하는데, 갓 만든 클라이언트는
        아직 매칭 전이라 ``server_is_ready()`` 가 ``False`` 를 돌려주어 멀쩡한 서버를
        미준비로 오판한다. 그 확인은 실행 단계의 ``wait_for_server`` 가 맡으며, 없으면
        ``EXECUTION_ABORTED`` 로 보고된다.
        """
        from robot_twin.backends import requires_move_group

        if not requires_move_group(op):
            return

        with self._mg_lock:
            ready = self._move_group is not None
            error = self._move_group_error

        if not ready:
            detail = f': {error}' if error else ' (still initializing)'
            raise precondition_failed(f'MoveGroup client is not ready{detail}')

    def _validate_inputs(self, op: OperationConfig, inputs: dict[str, Any]) -> None:
        """입력을 로봇에 닿기 전에 검증한다 (설계서 6.9).

        JSON Schema 가 선언되어 있으면 그것으로 1차 검증한다. 스키마 라이브러리가
        없으면 검증을 건너뛴다 — 선택적 의존성으로 두어 기동을 막지 않는다.
        """
        # 1차: JSON Schema (선언적, 선택적 의존성). 없으면 건너뛴다.
        if op.inputs_schema:
            try:
                import jsonschema
            except ImportError:
                self._logger.warning(
                    f"jsonschema not installed; schema validation skipped for '{op.name}'"
                )
            else:
                try:
                    jsonschema.validate(inputs, op.inputs_schema)
                except jsonschema.ValidationError as exc:
                    raise invalid_input(f'{op.name}: {exc.message}') from exc

        # 2차: 로봇 도메인 규칙. JSON Schema 로 표현하기 어려운 것들이며,
        # jsonschema 설치 여부와 무관하게 **항상** 수행한다.
        from robot_twin.backends import validate_inputs as validate_domain
        try:
            validate_domain(op.name, inputs, op)
        except ValueError as exc:
            raise invalid_input(f'{op.name}: {exc}') from exc

    # ------------------------------------------------------------------
    # 내부 — 실행
    # ------------------------------------------------------------------

    def _run_sync(self, op: OperationConfig, session: Session) -> None:
        """동기 연산을 실행한다 — 상한 안에 끝내고 세션을 종료 상태로 만든다."""
        try:
            outputs = self._dispatch(op, session, timeout=op.sync_timeout_sec)
            self.sessions.finish(session.id, Status.COMPLETED, outputs=outputs)
        except TimeoutError as exc:
            self.sessions.finish(session.id, Status.FAILED, error_code=ErrorCode.TIMEOUT,
                                 error_message=str(exc))
        except PreconditionFailed as exc:
            # 백엔드가 현재 상태에서 명령을 거절했다 — 실행이 중단된 것이 아니라
            # 아무것도 실행되지 않았다. 클라이언트는 재시도가 아니라 상태를 맞춰야 한다.
            self.sessions.finish(session.id, Status.FAILED,
                                 error_code=ErrorCode.PRECONDITION_FAILED,
                                 error_message=str(exc))
        except Exception as exc:
            self.sessions.finish(session.id, Status.FAILED,
                                 error_code=ErrorCode.EXECUTION_ABORTED, error_message=str(exc))

    def _run_async(self, op: OperationConfig, session: Session) -> None:
        """비동기 연산을 워커 스레드에서 실행한다."""
        self.sessions.update(session.id, phase=Phase.EXECUTING)
        try:
            outputs = self._dispatch(op, session, timeout=op.default_timeout_sec)
            if session.cancel_requested:
                self.sessions.finish(session.id, Status.CANCELLED, outputs=outputs)
            else:
                self.sessions.finish(session.id, Status.COMPLETED, outputs=outputs)
        except TimeoutError as exc:
            self.sessions.finish(session.id, Status.FAILED, error_code=ErrorCode.TIMEOUT,
                                 error_message=str(exc), outputs=self._stop_outputs())
        except Exception as exc:
            if session.cancel_requested:
                # 취소를 요청했으면 백엔드 예외는 그 결과다. 클라이언트 요청에 의한
                # 종료이므로 CANCELLED 이며 error 를 붙이지 않는다 (설계서 6.8).
                # E-stop·워치독은 이미 자기 사유로 세션을 종료시켰으므로 여기 오지
                # 않는다 — finish() 는 먼저 도달한 종료 사유를 유지한다.
                self.sessions.finish(session.id, Status.CANCELLED,
                                     outputs=self._stop_outputs())
            else:
                self.sessions.finish(session.id, Status.FAILED,
                                     error_code=ErrorCode.EXECUTION_ABORTED,
                                     error_message=str(exc), outputs=self._stop_outputs())

    def _dispatch(self, op: OperationConfig, session: Session, *,
                  timeout: float) -> dict[str, Any]:
        """연산 백엔드를 호출한다.

        현재는 rdfp 에 이미 존재하는 백엔드만 연결한다. ``move_to_pose``(자유 계획)
        와 ``move_gripper``(위치 지정) 는 rdfp 쪽 공개 API 가 아직 없어 미구현이며,
        호출하면 ``BackendUnavailable`` 이 난다 (설계서 9장).
        """
        from robot_twin.backends import dispatch
        with self._mg_lock:
            client = self._move_group
        return dispatch(self, client, op, session, timeout=timeout)

    def _stop_outputs(self) -> dict[str, Any]:
        """중단 시점의 로봇 상태를 담는다 (설계서 6.4, 6.7).

        취소·워치독·E-stop 으로 멈춘 로봇은 경로 중간의 불확정 자세에 있으므로,
        클라이언트가 이를 다룰 수 있도록 정지 지점을 알려준다.
        """
        outputs: dict[str, Any] = {}
        for name in ('ee_pose', 'joint_states'):
            entry = self.variables.entry(name)
            if entry is None or entry.snapshot is None:
                continue
            try:
                outputs['stopped_at' if name == 'ee_pose' else 'final_joints'] = entry.value()
            except Exception:  # 정지 지점 보고 실패가 종료 처리를 막아서는 안 된다
                continue
        return outputs

    # ------------------------------------------------------------------
    # 내부 — 워치독
    # ------------------------------------------------------------------

    def _tick_watchdog(self) -> None:
        """``max_duration_sec`` 초과 세션을 강제 중단한다 (설계서 6.5).

        폴링 방식이라 트윈은 클라이언트 생존을 알 수 없다. 클라이언트가 죽어도
        로봇이 계속 움직이는 것을 막는 유일한 장치다.
        """
        now = self._clock()
        for session in self.sessions.running():
            op = self.config.operation(session.operation)
            if op is None:
                continue
            limit = float(session.inputs.get('max_duration_sec') or op.default_timeout_sec)
            if (now - session.started_at) <= limit:
                continue

            self._logger.warning(
                f'watchdog: session {session.id} exceeded {limit}s; aborting'
            )
            self.sessions.request_cancel(session.id)
            # 클라이언트 요청이 아니므로 CANCELLED 가 아니라 FAILED 다 (설계서 6.4).
            self.sessions.finish(
                session.id, Status.FAILED, error_code=ErrorCode.TIMEOUT,
                error_message=f'exceeded max_duration_sec={limit}',
                outputs=self._stop_outputs()
            )


__all__ = ['RobotTwinRuntime']
