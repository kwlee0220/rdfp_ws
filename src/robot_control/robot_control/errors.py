"""제어 계층의 공용 예외.

ROS 를 import 하지 않는다 — 트윈처럼 이 예외로 실패를 **분류만** 하는 쪽이 `rclpy` 없이도
읽을 수 있어야 한다.
"""

from __future__ import annotations


class PlanningError(RuntimeError):
    """MoveIt 이 경로를 **계획하지 못했다** — 로봇은 움직이지 않았다.

    IK 실패 · 충돌 · 도달 불가(MoveIt 오류 코드), 데카르트 경로의 계획 비율 미달, move_group 의
    plan-only goal 거부가 여기 해당한다. 실행 중 실패(컨트롤러 tolerance 위반 등)와 갈라야
    호출자가 "목표를 바꿀 것인가 / 다시 시도할 것인가"를 판단할 수 있다 — 예전에는 둘 다 같은
    `RuntimeError` 라 메시지 문자열로만 구분됐다.

    `RuntimeError` 를 상속하므로 기존 ``except RuntimeError`` 호출자는 그대로 동작한다.
    """


__all__ = ['PlanningError']
