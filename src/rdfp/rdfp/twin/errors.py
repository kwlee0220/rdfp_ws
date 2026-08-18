"""연산 오류 코드 체계 (설계서 6.8).

extern_op 4.4.1 의 구분을 그대로 따른다.

- **세션이 생성된 뒤의 실패** → ``200`` + ``status: FAILED`` + ``error.code``
- **세션이 생성되지 않은 거부** → 4xx / 5xx. ``status`` 자체가 없다.

``CANCELLED`` 에는 ``error`` 를 붙이지 않는다 — 클라이언트 자신의 요청에 의한 정상
종료이므로 오류가 아니다.
"""

from __future__ import annotations

from typing import Optional

from enum import Enum


class ErrorCode(str, Enum):
    """시스템 간 해석 가능한 안정적 오류 코드."""

    # --- 세션 생성 후 실패 (200 + FAILED) ---
    PLANNING_FAILED = 'PLANNING_FAILED'
    EXECUTION_ABORTED = 'EXECUTION_ABORTED'
    TIMEOUT = 'TIMEOUT'
    PREEMPTED = 'PREEMPTED'
    ROS_UNAVAILABLE = 'ROS_UNAVAILABLE'

    # --- 세션 미생성 거부 (4xx / 5xx) ---
    RESOURCE_BUSY = 'RESOURCE_BUSY'
    PRECONDITION_FAILED = 'PRECONDITION_FAILED'
    INVALID_INPUT = 'INVALID_INPUT'
    # 정지 요청을 전달할 대상이 없어 취소를 접수하지 못할 때.
    CANCEL_UNSUPPORTED = 'CANCEL_UNSUPPORTED'

    # --- 양쪽 모두에 등장 ---
    # 실행 중 비상정지로 중단이면 200 + FAILED, 잠금 상태의 신규 요청이면 409.
    ESTOP_ENGAGED = 'ESTOP_ENGAGED'


class TwinError(Exception):
    """세션을 만들지 않고 요청을 거부할 때 발생시킨다.

    API 계층이 이 예외를 잡아 ``http_status`` 로 응답한다.

    Attributes:
        code: 안정적 오류 코드.
        http_status: 응답 HTTP 상태 코드.
        details: 응답 ``error`` 객체에 병합할 추가 필드.
        retry_after_sec: ``Retry-After`` 헤더 값(초).
    """

    def __init__(self, code: ErrorCode, message: str, *, http_status: int,
                 details: Optional[dict] = None,
                 retry_after_sec: Optional[float] = None) -> None:
        super().__init__(message)
        self.code = code
        self.message = message
        self.http_status = http_status
        self.details = details or {}
        self.retry_after_sec = retry_after_sec

    def to_body(self) -> dict:
        """extern_op 형식의 오류 응답 본문을 만든다."""
        return {'error': {'code': self.code.value, 'message': self.message, **self.details}}


def resource_busy(resource: str, operation: str, session_endpoint: str,
                  remaining_ms: Optional[int]) -> TwinError:
    """자원이 점유 중일 때의 ``409`` (설계서 6.3).

    큐잉하지 않고 즉시 거부하되, 점유자와 예상 잔여 시간을 함께 주어 클라이언트가
    언제 재시도할지 판단할 수 있게 한다.
    """
    details: dict = {'resource': resource, 'operation': operation,
                     'occupied_by': session_endpoint}
    retry_after: Optional[float] = None
    if remaining_ms is not None:
        details['estimated_remaining_ms'] = remaining_ms
        # 초 단위 올림. 0 이면 즉시 재시도가 몰리므로 최소 1 초를 둔다.
        retry_after = max(1.0, remaining_ms / 1000.0)

    return TwinError(
        ErrorCode.RESOURCE_BUSY, f"resource '{resource}' is occupied by {operation}",
        http_status=409, details=details, retry_after_sec=retry_after
    )


def estop_engaged() -> TwinError:
    """비상정지 잠금 상태에서 새 연산을 요청했을 때의 ``409`` (설계서 6.4)."""
    return TwinError(
        ErrorCode.ESTOP_ENGAGED, 'emergency stop is engaged; release it before running operations',
        http_status=409
    )


def cancel_unsupported(operation: str) -> TwinError:
    """정지 요청을 전달할 대상이 없을 때의 ``501``.

    ``MoveGroupClient.cancel()`` 로 중단하는 경로 자체는 구현되어 있다 —
    JTC 는 컨트롤러의 ``FollowJointTrajectory`` 액션에 취소를 보내고, JGPC 는
    명령 스트리밍을 멈춘다. 이 예외는 그 경로가 **이번 호출에서 동작하지 못한**
    경우에 쓴다.

    - MoveGroup 클라이언트가 아직 준비되지 않았다 (ROS 미연결 등).
    - ``cancel()`` 이 ``False`` 를 돌려줬다 — 중단할 대상이 없거나(JGPC: 스트리밍
      중이 아니고 추적 중인 goal 도 없음) 컨트롤러의 취소 서비스가 없다(JTC).
    - ``cancel()`` 이 예외를 냈다.

    **취소를 접수하고 CANCELLED 로 보고하면 로봇은 계속 움직이는데 클라이언트는
    멈췄다고 믿는다.** 안전 관점에서 이는 기능 부재보다 나쁘므로 명시적으로 거부한다.
    """
    return TwinError(
        ErrorCode.CANCEL_UNSUPPORTED,
        (f"operation '{operation}' could not be interrupted: no stop could be delivered to "
         'the controller. The motion may run to completion. Use a physical emergency stop '
         'to halt the robot'),
        http_status=501
    )


def precondition_failed(message: str) -> TwinError:
    """백엔드가 준비되지 않았을 때의 ``503`` (설계서 6.8).

    실행이 시작조차 되지 않았으므로 ``200 + FAILED`` 가 아니다. 검사는 비용 0 인
    로컬 조회로만 하므로 매 요청마다 수행한다.
    """
    return TwinError(ErrorCode.PRECONDITION_FAILED, message, http_status=503)


def invalid_input(message: str) -> TwinError:
    """입력 검증 실패의 ``400`` (설계서 6.9).

    로봇에 닿기 전에 거른다. 검증 없이 MoveIt 까지 보내면 수 초 뒤에 불친절한
    메시지로 실패한다.
    """
    return TwinError(ErrorCode.INVALID_INPUT, message, http_status=400)


__all__ = [
    'ErrorCode', 'TwinError', 'cancel_unsupported', 'estop_engaged', 'invalid_input',
    'precondition_failed', 'resource_busy'
]
