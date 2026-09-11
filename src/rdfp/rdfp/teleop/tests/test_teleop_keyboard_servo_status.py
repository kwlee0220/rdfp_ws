#!/usr/bin/env python3

"""`teleop_keyboard` 가 servo 의 자기 진단을 사람에게 전달하는가.

**이것이 없으면 특이점·관절한계로 servo 가 스스로 멈춘 상황이 "키가 죽었다"로만
보인다.** Isaac 파지 자세에서 실제로 그렇게 오진했다 — `panda_joint6` 의 하한이
-0.0873 이라 파지 자세가 한계이자 손목 특이점 위였고, `+z` 명령이 옆으로 새다가
`JOINT_BOUND` 로 완전히 멈췄는데 화면에는 아무것도 안 나왔다.

servo 는 status 를 **매 주기** 내므로 그대로 찍으면 100 Hz 로 도배된다. 그래서
**전이 시점에만** 알린다 — 이 파일이 지키는 성질이 그것이다.
"""

from __future__ import annotations

import pytest

pytest.importorskip('rclpy', reason='requires ROS 2 runtime')

from rdfp.teleop.teleop_keyboard import TeleopKeyboard   # noqa: E402


class _Msg:
    def __init__(self, data: int) -> None:
        self.data = data


class _Logger:
    def __init__(self) -> None:
        self.warnings: list[str] = []
        self.infos: list[str] = []

    def warning(self, msg):
        self.warnings.append(str(msg))

    def info(self, msg):
        self.infos.append(str(msg))

    def error(self, msg):
        pass


class _Node:
    """`_on_servo_status` 만 빌려 쓰는 대역."""

    _on_servo_status = TeleopKeyboard._on_servo_status

    def __init__(self) -> None:
        self._servo_status = 0
        self._logger = _Logger()

    def get_logger(self):
        return self._logger


@pytest.fixture
def node():
    return _Node()


def test_nonzero_status_warns_once(node):
    """같은 코드가 반복돼도 한 번만 알린다 — servo 는 매 주기 낸다."""
    for _ in range(50):
        node._on_servo_status(_Msg(5))
    assert len(node._logger.warnings) == 1


def test_returning_to_no_warning_is_reported(node):
    """풀렸다는 것도 알려야 한다 — 경고만 남으면 아직 걸린 줄 안다."""
    node._on_servo_status(_Msg(1))
    node._on_servo_status(_Msg(0))
    assert len(node._logger.infos) == 1
    assert 'NO_WARNING' in node._logger.infos[0]


def test_startup_no_warning_is_silent(node):
    """초기값이 0 이므로 기동 직후의 NO_WARNING 은 아무 말도 하지 않는다."""
    node._on_servo_status(_Msg(0))
    assert node._logger.infos == []
    assert node._logger.warnings == []


def test_joint_bound_message_names_the_halt_and_the_recovery(node):
    """`JOINT_BOUND` 는 **한계에서 멀어지는 조그까지** 막는다 — 그 사실과 회복 수단.

    실측: joint6 이 하한에서 0.069 rad (servo 의 `joint_limit_margin` 0.1 안)일 때
    +0.5 rad/s 로 2 초를 밀어도 0.000 rad 움직였다. 회복은 servo 를 안 거치는
    MoveIt 경로('/')뿐이다.
    """
    node._on_servo_status(_Msg(5))
    text = node._logger.warnings[0]
    assert 'JOINT_BOUND' in text
    assert 'ALL motion' in text
    assert "'/'" in text


def test_singularity_message_explains_the_direction_drift(node):
    """특이점 감속은 '느려진다'가 아니라 **방향이 어긋난다**가 증상이다."""
    node._on_servo_status(_Msg(1))
    text = node._logger.warnings[0]
    assert 'SINGULARITY' in text
    assert 'deviate' in text


def test_unknown_code_does_not_crash(node):
    """servo 가 모르는 값을 내도 노드는 살아 있어야 한다."""
    node._on_servo_status(_Msg(99))
    assert 'UNKNOWN(99)' in node._logger.warnings[0]


def test_transition_between_two_nonzero_codes_is_reported(node):
    """특이점 감속 → 관절한계처럼 **원인이 바뀌는 것**을 놓치면 안 된다."""
    node._on_servo_status(_Msg(1))
    node._on_servo_status(_Msg(5))
    assert len(node._logger.warnings) == 2
    assert 'JOINT_BOUND' in node._logger.warnings[1]
