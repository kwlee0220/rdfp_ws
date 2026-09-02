"""`GripperActionNode` 의 판정 로직을 고정한다.

`Node.__init__` 을 피하고 판정 메서드만 빌려 쓴다 — ROS 없이도 돌아야 하는 것은
아니지만(rclpy 를 import 한다), 액션 서버·토픽 없이 로직만 검증하기 위해서다.
"""

from __future__ import annotations

import math

import pytest

pytest.importorskip('rclpy')

from robot_control.gripper.gripper_action_node import GripperActionNode  # noqa: E402


class _Stub:
    """판정에 필요한 상태만 갖춘 대역."""

    def __init__(self, goal: str, width: float, tolerance: float = 0.005) -> None:
        self._goal = goal
        self._width = width
        self._width_tolerance = tolerance
        # `targets` 는 액션 goal 에 실을 **관절값**이다. 관측되는 width 는 그 2배다.
        self._targets = {'open': (0.035, 10.0), 'close': (0.0, 10.0), 'grasp': (0.0, 30.0)}
        self._width_scale = 2.0
        # 판정 파라미터의 기본(= 판정 안 함). `at_goal` 검사는 그 상태를 전제한다.
        self._effort = 0.0
        self._velocity = 0.0
        self._stall_effort = 0.0
        self._stall_velocity = 0.0

    _at_goal = GripperActionNode._at_goal
    _stalled = GripperActionNode._stalled
    _target_width = GripperActionNode._target_width


def test_open_reaches_goal_at_target_width():
    """비교 기준은 **개구 폭**이다 — 관절값 0.035 의 목표는 폭 0.070 이다."""
    assert _Stub('open', 0.070)._at_goal() is True
    assert _Stub('open', 0.040)._at_goal() is False


def test_open_does_not_match_the_raw_joint_value():
    """관절값과 폭을 그대로 견주면 `open` 은 영영 도달하지 못한다 (회귀).

    `targets['open']` 은 0.035(관절값)이고 손이 다 열렸을 때 `width` 는 0.070 이다.
    두 단위를 직접 비교하던 구현에서는 이 값이 `True` 였고, 실제로 열린 손은
    `False` 였다 — 판정이 정확히 뒤집혀 있었다.
    """
    assert _Stub('open', 0.035)._at_goal() is False


def test_close_reaches_goal_when_fully_closed():
    assert _Stub('close', 0.0)._at_goal() is True


def test_grasp_never_succeeds_without_stall_sensing():
    """`stalled` 을 판정하지 않는 스택에서는 `grasp` 의 `at_goal` 이 항상 false 다.

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


def test_stalled_is_false_when_no_threshold_is_configured():
    """'물지 않았다'가 아니라 **'모른다'** 는 뜻이다 — mock 이 그 상태로 남는다."""
    assert _Stub('grasp', 0.0)._stalled() is False


# ----------------------------------------------------------------------
# stalled — 파라미터로 켜고, 속도 조건은 선택이다
#
# 수치는 Isaac 실측(2026-09-02)에서 왔다: 빈손 폐쇄 + 팔 흔듦 |effort| ≤ 0.13
# (과도 0.50), 파지 유지 22.4 N·m, 그리고 **파지 중에도 |vel| 이 0.26 까지 간다.**
# ----------------------------------------------------------------------

class _StallStub:
    def __init__(self, effort=0.0, velocity=0.0,
                 stall_effort=0.0, stall_velocity=0.0) -> None:
        self._effort = effort
        self._velocity = velocity
        self._stall_effort = stall_effort
        self._stall_velocity = stall_velocity

    _stalled = GripperActionNode._stalled


def test_stall_is_not_judged_without_a_threshold():
    """기본값은 '판정 안 함' — mock 이 그 상태로 남는다."""
    assert _StallStub(effort=22.4)._stalled() is False


def test_grasping_effort_is_stalled():
    """Isaac 실측 파지값."""
    assert _StallStub(effort=22.4, stall_effort=1.0)._stalled() is True


def test_empty_close_with_the_arm_swinging_is_not_stalled():
    """거짓 양성 최악값 — 빈손으로 닫은 채 팔을 흔들 때다."""
    assert _StallStub(effort=0.503, stall_effort=1.0)._stalled() is False


def test_velocity_gate_is_off_by_default():
    """Isaac 은 파지 중에도 손가락이 떨린다 — 속도 조건을 걸면 파지를 놓친다."""
    assert _StallStub(effort=22.4, velocity=0.26, stall_effort=1.0)._stalled() is True


def test_velocity_gate_applies_when_enabled():
    """켠 스택에서는 움직이는 동안을 물림으로 읽지 않는다."""
    stub = _StallStub(effort=22.4, velocity=0.26,
                      stall_effort=1.0, stall_velocity=0.001)
    assert stub._stalled() is False
    stub._velocity = 0.0
    assert stub._stalled() is True


def test_missing_effort_reads_as_not_judged():
    """`/joint_states` 에 effort 가 없으면 0 으로 남아 판정이 서지 않는다."""
    assert _StallStub(effort=0.0, stall_effort=1.0)._stalled() is False
