#!/usr/bin/env python3

from __future__ import annotations

from typing import Iterable

import logging
import threading
from enum import Enum

from .exceptions import RecorderStateError


class RecorderState(Enum):
    """FFMpegMp4Recorder 의 생명주기 상태.

    - IDLE: 생성 직후 또는 stop() 완료 후 대기 상태
    - RECORDING: start() 성공 후 프레임 기록이 활성화된 상태
    - STOPPING: stop() 진행 중 (finalize 단계)
    - FAILED: 마지막 녹화가 비정상 종료된 상태 (start 로 복귀 가능)
    - SHUTDOWN: 모든 자원이 반환된 종착 상태
    """

    IDLE = "IDLE"
    RECORDING = "RECORDING"
    STOPPING = "STOPPING"
    FAILED = "FAILED"
    SHUTDOWN = "SHUTDOWN"

    def __str__(self) -> str:
        return self.value


# 각 상태에서 전이가 허용되는 목적 상태 집합 정의
_ALLOWED_TRANSITIONS: dict[RecorderState, frozenset[RecorderState]] = {
    RecorderState.IDLE: frozenset({
        RecorderState.RECORDING,
        RecorderState.SHUTDOWN,
    }),
    RecorderState.RECORDING: frozenset({
        RecorderState.STOPPING,
        RecorderState.FAILED,
        RecorderState.SHUTDOWN,
    }),
    RecorderState.STOPPING: frozenset({
        RecorderState.IDLE,
        RecorderState.FAILED,
        RecorderState.SHUTDOWN,
    }),
    RecorderState.FAILED: frozenset({
        RecorderState.RECORDING,
        RecorderState.SHUTDOWN,
    }),
    # 종착 상태이므로 전이 불가
    RecorderState.SHUTDOWN: frozenset(),
}


class RecorderStateMachine:
    """FFMpegMp4Recorder 의 상태와 상태 전이를 재진입 락으로 보호하는 헬퍼.

    복합 연산(상태 검사 + 전이)이 필요한 경우 `lock` 속성을 통해 외부에서
    동일한 락을 점유할 수 있다 (RLock 이므로 동일 스레드의 중첩 획득 안전).

    허용되는 전이 규칙:
        IDLE       → RECORDING, SHUTDOWN
        RECORDING  → STOPPING, FAILED, SHUTDOWN
        STOPPING   → IDLE, FAILED, SHUTDOWN
        FAILED     → RECORDING, SHUTDOWN
        SHUTDOWN   → (종착, 전이 불가)

    설계 근거: `STOPPING` 은 finalize 를 거쳐 `IDLE` 로 복귀하거나, 이상
    종료 시 `FAILED` 로 전환되어야 하므로 `RECORDING` 으로의 직접 복귀는
    금지한다. `FAILED` 에서 `start()` 로 다시 녹화를 시작할 수 있으므로
    `RECORDING` 으로의 전이만 허용한다.

    스레드 안전성:
        모든 public 메서드는 내부 락으로 보호된다. 단, `state` / `is_shutdown()`
        의 반환값은 **스냅샷**이므로 호출 직후 다른 스레드에 의해 실제 상태가
        바뀔 수 있다. TOCTOU 회피가 필요한 경우 `lock` 속성으로 외부에서
        명시적으로 락을 잡고 사용해야 한다.
    """

    def __init__(self, logger: logging.Logger) -> None:
        """
        Args:
            logger: 상태 전이 이벤트를 DEBUG 레벨로 기록할 로거.
        """
        self._state: RecorderState = RecorderState.IDLE
        self._lock: threading.RLock = threading.RLock()
        self._logger = logger

    @property
    def lock(self) -> threading.RLock:
        """상태 관련 복합 연산을 원자화할 때 사용하는 재진입 락."""
        return self._lock

    @property
    def state(self) -> RecorderState:
        """현재 상태의 스냅샷을 반환한다.

        반환 직후 다른 스레드에 의해 실제 상태가 바뀔 수 있으므로, 상태 검사와
        그에 따른 동작을 원자적으로 수행해야 한다면 `lock` 속성을 통해 외부에서
        명시적으로 락을 점유해야 한다.
        """
        with self._lock:
            return self._state

    def is_shutdown(self) -> bool:
        """현재 상태가 `SHUTDOWN` 인지 여부의 스냅샷을 반환한다.

        `state` 프로퍼티와 동일한 스냅샷 제약이 적용된다 (TOCTOU 주의).
        """
        with self._lock:
            return self._state is RecorderState.SHUTDOWN

    def require_not_shutdown(self) -> None:
        """현재 상태가 `SHUTDOWN` 이면 즉시 예외를 발생시킨다.

        `shutdown()` 이후의 재호출을 거부해야 하는 public 진입점에서 가드로
        사용한다. `require()` 로도 동일한 효과를 얻을 수 있지만, SHUTDOWN 만
        따로 차단하고 나머지 상태는 허용하는 케이스가 많아 별도 메서드로
        제공한다.

        Raises:
            RecorderStateError: 현재 상태가 `SHUTDOWN` 인 경우.
        """
        with self._lock:
            if self._state is RecorderState.SHUTDOWN:
                raise RecorderStateError(
                    "operation not allowed in state SHUTDOWN; recorder is terminated"
                )

    def require(self, *allowed: RecorderState) -> None:
        """현재 상태가 허용 목록에 포함되는지 검증한다.

        Public 메서드 진입 시 상태 사전조건을 강제하는 데 사용한다.
        예: `self._state_machine.require(RecorderState.RECORDING)` 은 `write()`
        호출이 RECORDING 상태에서만 허용됨을 보장한다.

        Args:
            *allowed: 허용되는 상태 목록. 하나 이상을 전달해야 한다.

        Raises:
            RecorderStateError: 현재 상태가 `allowed` 에 포함되지 않는 경우.
                예외 메시지에 현재 상태명과 허용 목록이 포함된다.
        """
        with self._lock:
            if self._state not in allowed:
                allowed_names = [s.value for s in allowed]
                raise RecorderStateError(
                    f"operation not allowed in state {self._state.value}; "
                    f"expected one of {allowed_names}"
                )

    def transition(self, target: RecorderState) -> RecorderState:
        """현재 상태에서 `target` 상태로 전이한다.

        `target` 이 현재 상태와 동일하면 no-op 으로 취급하고 현재 상태를 그대로
        반환한다. 허용되지 않은 전이 요청은 예외로 거부된다 (허용 규칙은 클래스
        docstring 참조).

        Args:
            target: 목표 상태.

        Returns:
            전이 후의 상태 (동일 상태로의 no-op 인 경우에도 현재 상태).

        Raises:
            RecorderStateError: 현재 상태에서 `target` 으로의 전이가 허용되지
                않는 경우.
        """
        with self._lock:
            current = self._state
            if target is current:
                return current
            allowed = _ALLOWED_TRANSITIONS.get(current, frozenset())
            if target not in allowed:
                raise RecorderStateError(
                    f"invalid state transition: {current.value} -> {target.value}"
                )
            self._state = target
            self._logger.debug(
                "recorder state transition: %s -> %s",
                current.value,
                target.value,
            )
            return target

    def try_transition(
        self,
        target: RecorderState,
        *,
        from_states: Iterable[RecorderState] | None = None,
    ) -> bool:
        """조건부 상태 전이를 시도하고 성공 여부를 반환한다.

        `transition()` 과 달리 예외를 던지지 않고 실패 시 `False` 를 반환하므로,
        writer/stderr drainer 스레드처럼 상태 경합 상황에서 "전이가 가능하면
        하고, 이미 다른 스레드가 상태를 바꿨다면 조용히 포기" 하는 패턴에
        적합하다. 예: writer 가 BrokenPipe 를 감지하여 RECORDING→FAILED 로
        전이하려는데 이미 메인 스레드가 STOPPING 으로 전환한 경우.

        Args:
            target: 목표 상태.
            from_states: 전이 허용 출발 상태 필터. `None` 이면 필터 없이
                `transition()` 의 기본 허용 규칙만 적용된다.

        Returns:
            전이에 성공하면 `True`, `from_states` 필터에 걸리거나 허용되지
            않은 전이여서 실패하면 `False`.
        """
        with self._lock:
            if from_states is not None and self._state not in from_states:
                return False
            try:
                self.transition(target)
                return True
            except RecorderStateError:
                return False
