"""`Robotiq2FGripperNode` 의 판정 로직을 고정한다.

`Node.__init__` 을 피하고 판정 메서드만 빌려 쓴다 — 토픽·시뮬레이터 없이 로직만
검증하기 위해서다.

수치는 펑션베이 실측에서 왔다 (`docs/simulation/functionbay_backend_design.md` §6.1):
빈손 폐쇄 토크 0.004 / 자유 이동 중 최대 0.845 / 파지 17.19 N·m.
"""

from __future__ import annotations

from typing import Optional

import math

import pytest

pytest.importorskip('rclpy')

from robot_control.gripper.robotiq_2f_gripper_node import Robotiq2FGripperNode  # noqa: E402

SIGNS = [1.0, 1.0, -1.0, -1.0, -1.0, 1.0]
CLOSED = 0.725


class _Stub:
    """판정에 필요한 상태만 갖춘 대역."""

    def __init__(self, goal: str = '', target: Optional[list] = None,
                 positions: Optional[list] = None, velocities: Optional[list] = None,
                 efforts: Optional[list] = None) -> None:
        self._goal = goal
        self._target = target
        self._positions = positions or []
        self._velocities = velocities if velocities is not None else [0.0] * 6
        self._efforts = efforts if efforts is not None else [0.0] * 6
        self._axis_signs = list(SIGNS)
        self._targets = {'open': 0.0, 'close': CLOSED, 'grasp': CLOSED}
        self._tolerance = 0.005
        self._stall_effort = 1.0
        self._stall_velocity = 0.001

    _command_array = Robotiq2FGripperNode._command_array
    _residual = Robotiq2FGripperNode._residual
    _stalled = Robotiq2FGripperNode._stalled
    _at_goal = Robotiq2FGripperNode._at_goal


def _spread(scalar: float) -> list:
    return [scalar * s for s in SIGNS]


# ----------------------------------------------------------------------
# 지령 생성 — 6축을 한꺼번에 채운다
# ----------------------------------------------------------------------

def test_command_spreads_one_scalar_over_every_axis():
    """한 축만 보내면 링키지가 어긋난 자세가 된다 — 시뮬레이터가 막아 주지 않는다."""
    assert _Stub()._command_array('close') == pytest.approx(
        [0.725, 0.725, -0.725, -0.725, -0.725, 0.725])
    assert _Stub()._command_array('open') == [0.0] * 6


def test_unknown_goal_has_no_command():
    """`targets` 에 없는 심볼은 지령을 만들지 않는다 — 호출자가 거부해야 한다."""
    assert _Stub()._command_array('pinch') is None


# ----------------------------------------------------------------------
# stalled — 신호 둘이 함께 서야 한다
# ----------------------------------------------------------------------

def test_grasping_torque_with_no_motion_is_stalled():
    """실측 파지: 토크 17.19 N·m, 속도 0."""
    s = _Stub(efforts=[0.0, 17.19, 12.0, 0.0, 8.4, 3.1], velocities=[0.0] * 6)
    assert s._stalled() is True


def test_acceleration_torque_while_moving_is_not_stalled():
    """자유 이동 중 최대 토크 0.845 N·m 는 임계 1.0 아래다 — 속도로도 갈린다."""
    s = _Stub(efforts=[0.845] * 6, velocities=[0.4] * 6)
    assert s._stalled() is False


def test_high_torque_while_still_moving_is_not_stalled():
    """토크만 보면 가속 구간을 파지로 오독한다 — 속도가 그것을 막는다."""
    s = _Stub(efforts=[5.0] * 6, velocities=[0.2] * 6)
    assert s._stalled() is False


def test_stopped_without_torque_is_not_stalled():
    """도달해 멈춘 것과 막혀 멈춘 것은 다르다 — 빈손 폐쇄 토크는 0.004 N·m 다."""
    s = _Stub(efforts=[0.004] * 6, velocities=[0.0] * 6)
    assert s._stalled() is False


def test_missing_effort_is_not_stalled():
    """보고에 effort 가 없으면 '모른다' — 계약대로 false 다."""
    assert _Stub(efforts=[])._stalled() is False


# ----------------------------------------------------------------------
# at_goal — 폭이 아니라 관절 잔차로 판정한다
# ----------------------------------------------------------------------

def test_close_reaches_goal_on_joint_residual():
    """**`width` 없이 판정한다** — 지령과 보고가 같은 관절 공간이기 때문이다."""
    s = _Stub(goal='close', target=_spread(CLOSED), positions=_spread(CLOSED))
    assert s._at_goal() is True


def test_open_reaches_goal_on_joint_residual():
    s = _Stub(goal='open', target=_spread(0.0), positions=_spread(0.0))
    assert s._at_goal() is True


def test_close_still_moving_is_not_at_goal():
    s = _Stub(goal='close', target=_spread(CLOSED), positions=_spread(0.30))
    assert s._at_goal() is False


def test_close_blocked_by_an_object_is_not_at_goal():
    """"빈손으로 닫아라"라고 했는데 뭔가 끼었으면 시킨 일을 이룬 것이 아니다."""
    s = _Stub(goal='close', target=_spread(CLOSED), positions=_spread(0.64),
              efforts=[17.0] * 6, velocities=[0.0] * 6)
    assert s._at_goal() is False


def test_grasp_succeeds_when_stalled_short_of_the_pose():
    """파지는 **목표 자세에 못 간 채 막혀 멈추는 것**이 성공이다."""
    s = _Stub(goal='grasp', target=_spread(CLOSED), positions=_spread(0.64),
              efforts=[17.19] * 6, velocities=[0.0] * 6)
    assert s._at_goal() is True


def test_grasp_closing_all_the_way_is_a_miss():
    """헛닫힘 — 목표 자세에 도달했는데 힘이 없다 = 잡은 것이 없다."""
    s = _Stub(goal='grasp', target=_spread(CLOSED), positions=_spread(CLOSED),
              efforts=[0.004] * 6, velocities=[0.0] * 6)
    assert s._at_goal() is False


def test_no_command_yet_is_not_at_goal():
    assert _Stub(goal='', positions=_spread(0.0))._at_goal() is False


def test_missing_report_is_not_at_goal():
    """보고를 못 받았으면 도달을 주장하지 않는다."""
    assert _Stub(goal='close', target=_spread(CLOSED), positions=[])._at_goal() is False


def test_residual_is_nan_without_a_command():
    """지령이 없으면 견줄 대상이 없다."""
    assert math.isnan(_Stub(positions=_spread(0.0))._residual())


def test_residual_is_nan_when_report_length_differs():
    """길이가 어긋난 보고를 잘라 쓰면 엉뚱한 축을 비교하게 된다."""
    s = _Stub(goal='close', target=_spread(CLOSED), positions=[0.725, 0.725])
    assert math.isnan(s._residual())
    assert s._at_goal() is False


def test_residual_takes_the_worst_axis():
    """한 축만 어긋나도 도달이 아니다 — 링키지가 어긋난 자세를 성공으로 읽지 않는다."""
    positions = _spread(CLOSED)
    positions[3] += 0.05
    s = _Stub(goal='close', target=_spread(CLOSED), positions=positions)
    assert s._residual() == pytest.approx(0.05)
    assert s._at_goal() is False
