"""계획 실패는 `PlanningError` 로 나간다 — 실행 실패와 갈라야 호출자가 대응을 고른다.

예전에는 둘 다 같은 `RuntimeError` 라 메시지 문자열로만 구분됐고, 트윈이 정의해 둔
`PLANNING_FAILED` 가 한 번도 나오지 않았다. 계획 실패면 로봇이 움직이지 않았고 같은
목표로 다시 불러도 결과가 같으므로, 클라이언트는 재시도가 아니라 목표를 바꿔야 한다.
"""

from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from robot_control.errors import PlanningError

_ERRORS_SOURCE = Path(__file__).resolve().parents[2] / 'errors.py'


def test_planning_error_keeps_existing_runtime_error_handlers_working() -> None:
    """``except RuntimeError`` 로 받던 기존 호출자가 그대로 동작해야 한다."""
    assert issubclass(PlanningError, RuntimeError)


def test_errors_module_does_not_import_ros() -> None:
    """트윈은 실패를 분류만 하므로 이 모듈을 `rclpy` 없이 읽을 수 있어야 한다."""
    tree = ast.parse(_ERRORS_SOURCE.read_text(encoding='utf-8'))
    roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            roots.add(node.module.split('.')[0])

    assert roots <= {'__future__'}, roots


# ----- MoveIt 결과 해석 ------------------------------------------------------

def _plan_result(code: int, *, points: int = 1) -> SimpleNamespace:
    trajectory = SimpleNamespace(joint_trajectory=SimpleNamespace(points=[object()] * points))
    return SimpleNamespace(error_code=SimpleNamespace(val=code), planned_trajectory=trajectory)


def test_moveit_error_code_is_a_planning_error() -> None:
    pytest.importorskip('moveit_msgs')
    from moveit_msgs.msg import MoveItErrorCodes
    from robot_control.moveit.move_group_client import _extract_planned_trajectory

    with pytest.raises(PlanningError, match='failed with code'):
        _extract_planned_trajectory(_plan_result(MoveItErrorCodes.PLANNING_FAILED), 'ready')


def test_empty_trajectory_is_a_planning_error() -> None:
    pytest.importorskip('moveit_msgs')
    from moveit_msgs.msg import MoveItErrorCodes
    from robot_control.moveit.move_group_client import _extract_planned_trajectory

    with pytest.raises(PlanningError, match='empty trajectory'):
        _extract_planned_trajectory(_plan_result(MoveItErrorCodes.SUCCESS, points=0), 'ready')
