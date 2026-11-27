"""제어 계층의 공용 예외.

ROS 를 import 하지 않는다 — 트윈처럼 이 예외로 실패를 **분류만** 하는 쪽이 `rclpy` 없이도
읽을 수 있어야 한다. 그래서 MoveIt 오류 코드의 이름도 여기서 풀지 않고, 예외를 만드는 쪽
(`moveit_msgs` 를 가진 MoveGroup 클라이언트)이 이름까지 채워 넘긴다.
"""

from __future__ import annotations

from typing import Any, Optional


class _MoveItCoded(RuntimeError):
    """MoveIt 원본 오류 코드(`MoveItErrorCodes`)를 함께 싣는 실패.

    트윈의 오류 코드(`PLANNING_FAILED` 등)만으로는 충돌 · IK 실패 · 시작 자세 무효를 구분할 수
    없어, 호출자가 재시도할지 목표를 바꿀지 판단할 원인을 잃는다. 메시지 문자열에 숫자를 넣어
    두는 것으로는 부족하다 — 문구가 바뀌면 조용히 못 읽게 된다.

    Attributes:
        moveit_error_code: `MoveItErrorCodes.val`. MoveIt 이 코드를 주지 않은 실패면 ``None``.
        moveit_error_name: 그 코드의 이름(예: ``GOAL_IN_COLLISION``). 모르는 값이면 ``None``.
    """

    def __init__(self, message: str, *, moveit_error_code: Optional[int] = None,
                 moveit_error_name: Optional[str] = None) -> None:
        super().__init__(message)
        self.moveit_error_code = moveit_error_code
        self.moveit_error_name = moveit_error_name


class PlanningError(_MoveItCoded):
    """MoveIt 이 경로를 **계획하지 못했다** — 로봇은 움직이지 않았다.

    IK 실패 · 충돌 · 도달 불가(MoveIt 오류 코드), 데카르트 경로의 계획 비율 미달, move_group 의
    plan-only goal 거부가 여기 해당한다. 실행 중 실패(컨트롤러 tolerance 위반 등)와 갈라야
    호출자가 "목표를 바꿀 것인가 / 다시 시도할 것인가"를 판단할 수 있다 — 예전에는 둘 다 같은
    `RuntimeError` 라 메시지 문자열로만 구분됐다.

    `RuntimeError` 를 상속하므로 기존 ``except RuntimeError`` 호출자는 그대로 동작한다.
    """


class ExecutionError(_MoveItCoded):
    """계획한 궤적의 **실행이 MoveIt 오류 코드로 끝났다** — 로봇이 경로 중간에 있을 수 있다.

    MoveGroup · ExecuteTrajectory 액션 결과의 `error_code` 가 `SUCCESS` 가 아닐 때다(컨트롤러 실패 ·
    시작 상태 이탈 등). 코드가 없는 실행 실패(goal 거부 등)는 여전히 일반 `RuntimeError` 다.
    """


def moveit_error_details(exc: Optional[BaseException]) -> Optional[dict[str, Any]]:
    """예외(와 그 원인 사슬)에서 MoveIt 원본 오류 코드를 꺼낸다. 없으면 ``None``.

    호출 경로에 따라 원래 예외가 다른 예외의 ``__cause__`` 로 감싸여 올라오므로 사슬을 따라간다.
    """
    seen = set()
    while exc is not None and id(exc) not in seen:
        seen.add(id(exc))
        code = getattr(exc, 'moveit_error_code', None)
        if isinstance(code, int) and not isinstance(code, bool):
            details: dict[str, Any] = {'moveit_error_code': code}
            name = getattr(exc, 'moveit_error_name', None)
            if isinstance(name, str) and name:
                details['moveit_error_name'] = name
            return details
        exc = exc.__cause__ or exc.__context__
    return None


__all__ = ['ExecutionError', 'PlanningError', 'moveit_error_details']
