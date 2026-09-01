"""연산 세션과 자원 락 (설계서 6.1, 6.3, 6.12).

모든 비동기 연산은 **다중 세션 연산**이다. 실행 요청마다 세션이 생성되고
``session_endpoint`` 가 항상 발급된다. extern_op 의 ``IDLE`` 상태는 사용하지 않는다
— op-endpoint 상태 조회를 제공하지 않기 때문이다.

자원 배타성은 세션 모델이 아니라 **세션 생성 시점의 admission control** 이 보장한다.
통과하면 세션을 만들고, 통과하지 못하면 세션을 만들지 않고 ``409`` 로 거부한다.

ROS 의존성이 없으므로 ROS 없이 테스트할 수 있다.
"""

from __future__ import annotations

from typing import Any, Callable, Optional

import threading
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum

from robot_twin.errors import ErrorCode, resource_busy


class Status(str, Enum):
    """extern_op 의 연산 상태.

    ``IDLE`` 은 정의하지 않는다 — 다중 세션 모델에서는 등장하지 않는다 (설계서 6.1).
    """

    RUNNING = 'RUNNING'
    COMPLETED = 'COMPLETED'
    FAILED = 'FAILED'
    CANCELLED = 'CANCELLED'


class Phase(str, Enum):
    """진행 국면 (설계서 6.6).

    extern_op 의 상태 5종을 늘리지 않고 국면을 표현하기 위한 확장 필드다.
    ``CANCELING`` 동안 ``status`` 는 ``RUNNING`` 을 유지한다.
    """

    PLANNING = 'PLANNING'
    EXECUTING = 'EXECUTING'
    CANCELING = 'CANCELING'


_TERMINAL = frozenset({Status.COMPLETED, Status.FAILED, Status.CANCELLED})


@dataclass
class Session:
    """비동기 연산 1회 실행의 컨텍스트.

    Attributes:
        created_by: 인증 주체. 현재는 항상 ``None`` — 권한 개념을 도입하지 않았다
            (설계서 8.4). 나중에 채워 넣을 때 스키마가 바뀌지 않도록 자리만 둔다.
    """

    id: str
    operation: str
    # 점유한 자원 전체. 대부분 0~1개지만 `reset_scene` 처럼 둘을 함께 잡는 연산이
    # 있다 — scene 을 바꾸는 동안 팔이 그 공간으로 들어오면 안 되기 때문이다.
    resources: tuple[str, ...]
    inputs: dict[str, Any]
    started_at: float
    status: Status = Status.RUNNING
    phase: Optional[Phase] = Phase.PLANNING
    progress: Optional[int] = None
    message: Optional[str] = None
    outputs: dict[str, Any] = field(default_factory=dict)
    error_code: Optional[ErrorCode] = None
    error_message: Optional[str] = None
    ended_at: Optional[float] = None
    created_by: Optional[str] = None
    # 예상 총 소요 시간(ms). 계획 완료 시 궤적 duration 으로 채워지며 409 응답의
    # `estimated_remaining_ms` 산출에 쓰인다.
    planned_duration_ms: Optional[int] = None
    # 취소 요청이 접수되었는지. 워치독·E-stop 에 의한 중단도 이 경로를 지난다.
    cancel_requested: bool = False

    @property
    def is_terminal(self) -> bool:
        """종료 상태에 도달했는지."""
        return self.status in _TERMINAL

    def remaining_ms(self, *, now: Optional[float] = None) -> Optional[int]:
        """예상 잔여 시간(ms). 산출 근거가 없으면 ``None``."""
        if self.planned_duration_ms is None or self.is_terminal:
            return None
        now = time.time() if now is None else now
        elapsed_ms = (now - self.started_at) * 1000.0
        return max(0, int(self.planned_duration_ms - elapsed_ms))


class SessionStore:
    """세션 보관소 — 수명주기와 자원 락을 함께 관리한다.

    자원 락과 세션 생성이 **하나의 임계 구역**이어야 한다. 둘을 분리하면 두 요청이
    동시에 락 검사를 통과해 같은 자원에 두 세션을 만들 수 있다.
    """

    def __init__(self, *, retention_sec: float = 900.0, max_sessions: int = 200,
                 endpoint_prefix: str = '',
                 known_resources: tuple[str, ...] = ('arm', 'gripper'),
                 clock: Callable[[], float] = time.time) -> None:
        """보관소를 초기화한다.

        Args:
            retention_sec: 종료 세션 보존 기간(초).
            max_sessions: 종료 세션 개수 상한. 초과분은 LRU 로 축출한다.
            endpoint_prefix: ``session_endpoint`` 생성에 쓰는 URL 접두사.
            known_resources: ``/resources`` 응답에 실을 자원 이름 전체. 설정에서
                수집해 넘긴다 — 여기에 하드코딩하면 자원을 늘릴 때 조용히 빠진다.
            clock: 시각 공급자. 보관소 전체가 **하나의 시간 원천**을 쓰도록 주입한다
                — 생성·종료·축출이 서로 다른 시계를 보면 축출 판정이 어긋난다.
        """
        self._clock = clock
        self._retention_sec = retention_sec
        self._max_sessions = max_sessions
        self._prefix = endpoint_prefix.rstrip('/')
        self.known_resources = tuple(known_resources)
        self._lock = threading.RLock()
        self._sessions: dict[str, Session] = {}
        # 자원 이름 → 점유 중인 세션 id.
        self._holders: dict[str, str] = {}
        self._estop = False

    # ------------------------------------------------------------------
    # 엔드포인트
    # ------------------------------------------------------------------

    def endpoint_of(self, session_id: str) -> str:
        """세션의 ``session_endpoint`` URL 을 만든다."""
        return f'{self._prefix}/operations/sessions/{session_id}'

    # ------------------------------------------------------------------
    # E-stop
    # ------------------------------------------------------------------

    @property
    def estop_engaged(self) -> bool:
        """비상정지 잠금 상태인지."""
        with self._lock:
            return self._estop

    def engage_estop(self) -> list[Session]:
        """비상정지를 걸고, 진행 중이던 세션 목록을 반환한다.

        반환된 세션들의 실제 중단(액션 취소)은 호출자가 수행한다. 상태 전이는
        :meth:`finish` 로 ``FAILED`` + ``ESTOP_ENGAGED`` 를 기록한다 (설계서 6.4).
        """
        with self._lock:
            self._estop = True
            running = [s for s in self._sessions.values() if not s.is_terminal]
            for s in running:
                s.cancel_requested = True
                s.phase = Phase.CANCELING
            return running

    def release_estop(self) -> None:
        """비상정지 잠금을 해제한다."""
        with self._lock:
            self._estop = False

    # ------------------------------------------------------------------
    # 생성 — admission control
    # ------------------------------------------------------------------

    def create(self, *, operation: str, resources: tuple[str, ...],
               inputs: dict[str, Any]) -> Session:
        """자원 락을 획득하고 세션을 생성한다.

        Args:
            operation: 연산 이름.
            resources: 점유할 자원 이름들. 비면 아무것도 점유하지 않는다.
                **전부 잡히거나 하나도 안 잡히거나** 둘 중 하나다.
            inputs: 입력 인자 (감사 로그용으로 보관한다).

        Returns:
            생성된 세션.

        Raises:
            TwinError: 비상정지 중이거나(``409``) 자원이 점유 중일 때(``409``).
                이 경우 **세션을 만들지 않는다.**
        """
        now = self._clock()
        with self._lock:
            if self._estop:
                from robot_twin.errors import estop_engaged
                raise estop_engaged()

            # **전부 검사한 뒤 전부 점유한다.** 같은 락 안에서 일어나므로 부분
            # 획득이나 락 순서 문제가 생기지 않는다 — 다중 자원 락의 통상적인
            # 비용이 이 구조에서는 발생하지 않는다.
            for name in resources:
                holder_id = self._holders.get(name)
                holder = self._sessions.get(holder_id) if holder_id else None
                if holder is not None and not holder.is_terminal:
                    raise resource_busy(
                        name, holder.operation, self.endpoint_of(holder.id),
                        holder.remaining_ms(now=now)
                    )

            session = Session(
                id=uuid.uuid4().hex[:12], operation=operation, resources=tuple(resources),
                inputs=dict(inputs), started_at=now
            )
            self._sessions[session.id] = session
            for name in resources:
                self._holders[name] = session.id

            self._evict()
            return session

    # ------------------------------------------------------------------
    # 갱신 · 종료
    # ------------------------------------------------------------------

    def update(self, session_id: str, **fields: Any) -> Optional[Session]:
        """진행 중 세션의 필드를 갱신한다. 종료된 세션은 건드리지 않는다."""
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None or session.is_terminal:
                return session
            for key, value in fields.items():
                setattr(session, key, value)
            return session

    def finish(self, session_id: str, status: Status, *,
               outputs: Optional[dict[str, Any]] = None,
               error_code: Optional[ErrorCode] = None,
               error_message: Optional[str] = None) -> Optional[Session]:
        """세션을 종료 상태로 전이시키고 자원 락을 푼다.

        이미 종료된 세션은 그대로 둔다 — 워치독과 실제 완료가 경합할 수 있으므로
        **먼저 도달한 종료 사유를 유지**한다.
        """
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None or session.is_terminal:
                return session

            session.status = status
            session.phase = None
            session.ended_at = self._clock()
            if outputs is not None:
                session.outputs = outputs
            # CANCELLED 에는 error 를 붙이지 않는다 (설계서 6.8).
            if status is not Status.CANCELLED:
                session.error_code = error_code
                session.error_message = error_message

            for name in session.resources:
                if self._holders.get(name) == session.id:
                    del self._holders[name]

            self._evict()
            return session

    def request_cancel(self, session_id: str) -> Optional[Session]:
        """취소를 접수한다 (설계서 6.4).

        로봇은 감속 정지에 시간이 걸리므로 여기서 종료시키지 않는다. ``status`` 는
        ``RUNNING`` 을 유지한 채 ``phase`` 만 ``CANCELING`` 으로 바꾼다.
        """
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None or session.is_terminal:
                return session
            session.cancel_requested = True
            session.phase = Phase.CANCELING
            return session

    # ------------------------------------------------------------------
    # 조회
    # ------------------------------------------------------------------

    def get(self, session_id: str) -> Optional[Session]:
        """세션을 조회한다. 없거나 만료·축출되었으면 ``None``."""
        with self._lock:
            self._evict()
            return self._sessions.get(session_id)

    def list(self, *, operation: Optional[str] = None,
             status: Optional[Status] = None) -> list[Session]:
        """세션 목록을 최신순으로 반환한다."""
        with self._lock:
            self._evict()
            items = list(self._sessions.values())
        if operation is not None:
            items = [s for s in items if s.operation == operation]
        if status is not None:
            items = [s for s in items if s.status is status]
        return sorted(items, key=lambda s: s.started_at, reverse=True)

    def resources(self, *, names: Optional[tuple[str, ...]] = None
                  ) -> dict[str, dict[str, Any]]:
        """자원별 점유 상태를 반환한다 (설계서 6.3).

        시작 전 확인 편의를 위한 것이며 배타성 보장 수단이 아니다 — 조회와 시작
        사이의 경합은 남으므로 클라이언트는 ``409`` 처리를 생략할 수 없다.

        Args:
            names: 조회할 자원 이름. ``None`` 이면 **설정에 선언된 자원 전체**를
                호출자가 넘긴 것으로 본다. 이름 목록을 여기에 하드코딩하면 자원을
                늘릴 때 응답에서 조용히 빠진다.
        """
        if names is None:
            names = self.known_resources
        now = self._clock()
        out: dict[str, dict[str, Any]] = {}
        with self._lock:
            for name in names:
                holder_id = self._holders.get(name)
                holder = self._sessions.get(holder_id) if holder_id else None
                if holder is None or holder.is_terminal:
                    out[name] = {'state': 'FREE'}
                    continue
                out[name] = {
                    'state': 'BUSY',
                    'operation': holder.operation,
                    'session': self.endpoint_of(holder.id),
                    'estimated_remaining_ms': holder.remaining_ms(now=now)
                }
        return out

    def running(self) -> list[Session]:
        """진행 중인 세션 목록."""
        with self._lock:
            return [s for s in self._sessions.values() if not s.is_terminal]

    # ------------------------------------------------------------------
    # 내부
    # ------------------------------------------------------------------

    def _evict(self) -> None:
        """만료·초과된 **종료 세션**을 제거한다 (설계서 6.12).

        ``RUNNING`` 세션은 개수 상한과 무관하게 유지한다 — 자원당 최대 1개이므로
        누적되지 않는다. 호출자가 ``self._lock`` 을 잡은 상태여야 한다.
        """
        now = self._clock()

        expired = [
            sid for sid, s in self._sessions.items()
            if s.is_terminal and s.ended_at is not None
            and (now - s.ended_at) > self._retention_sec
        ]
        for sid in expired:
            del self._sessions[sid]

        terminal = [s for s in self._sessions.values() if s.is_terminal]
        overflow = len(terminal) - self._max_sessions
        if overflow > 0:
            # 오래 전에 끝난 것부터 축출한다 (LRU).
            for s in sorted(terminal, key=lambda x: x.ended_at or 0.0)[:overflow]:
                self._sessions.pop(s.id, None)


__all__ = ['Phase', 'Session', 'SessionStore', 'Status']
