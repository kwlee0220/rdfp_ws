"""`MockGripperNode` 의 판정 로직을 고정한다.

`Node.__init__` 을 피하고 판정 메서드만 빌려 쓴다 — ROS 없이도 돌아야 하는 것은
아니지만(rclpy 를 import 한다), 액션 서버·토픽 없이 로직만 검증하기 위해서다.
"""

from __future__ import annotations

import math

import pytest

pytest.importorskip('rclpy')

from robot_control.gripper.mock_gripper_node import MockGripperNode  # noqa: E402


class _Stub:
    """판정에 필요한 상태만 갖춘 대역."""

    def __init__(self, goal: str, width: float, tolerance: float = 0.005) -> None:
        self._goal = goal
        self._width = width
        self._width_tolerance = tolerance
        self._targets = {'open': (0.035, 10.0), 'close': (0.0, 10.0), 'grasp': (0.0, 30.0)}

    _at_goal = MockGripperNode._at_goal
    _stalled = MockGripperNode._stalled


def test_open_reaches_goal_at_target_width():
    assert _Stub('open', 0.035)._at_goal() is True
    assert _Stub('open', 0.020)._at_goal() is False


def test_close_reaches_goal_when_fully_closed():
    assert _Stub('close', 0.0)._at_goal() is True


def test_grasp_never_succeeds_on_mock():
    """mock 은 `stalled` 을 판정할 수 없으므로 `grasp` 의 `at_goal` 이 항상 false 다.

    **이것이 옳다.** mock 의 물체는 물리를 갖지 않아 파지 성패를 관측할 수 없다.
    위치 기준 정의였다면 목표 폭(0.0)에 도달해 true 가 됐을 텐데, 물리가 없는
    백엔드에서 그것은 "성공했다"는 거짓말이다.
    """
    # 목표 폭에 정확히 도달했는데도 false 다 — 위치가 아니라 의도로 판정하기 때문.
    assert _Stub('grasp', 0.0)._at_goal() is False
    assert _Stub('grasp', 0.02)._at_goal() is False


def test_no_command_yet_is_not_at_goal():
    """`goal` 이 '' 이면 판정 대상이 없다."""
    assert _Stub('', 0.035)._at_goal() is False


def test_unknown_width_is_not_at_goal():
    """폭을 모르면(NaN) 도달을 주장하지 않는다."""
    assert _Stub('open', math.nan)._at_goal() is False


def test_unknown_goal_is_not_at_goal():
    """`targets` 에 없는 심볼은 판정하지 않는다 — 명령 자체가 거부됐어야 한다."""
    assert _Stub('pinch', 0.035)._at_goal() is False


def test_stalled_is_always_false_on_mock():
    """'물지 않았다'가 아니라 **'모른다'** 는 뜻이다."""
    assert _Stub('grasp', 0.0)._stalled() is False
