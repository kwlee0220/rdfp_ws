#!/usr/bin/env python3

"""`teleop_keyboard` 의 이동 속도 손잡이(`linear_step` / `angular_step`).

**servo 의 `scale.linear` 가 아니라 이쪽을 키우는 것이 맞다** — Isaac 실측에서
`scale.linear` 를 0.4 → 0.8 로 올리면 최대 속도는 1.2배 느는 대신 입력 대비
비례가 깨져 미세 조작을 잃는다 (CLAUDE.md). 0.4 는 전 구간이 비례하므로 입력
쪽을 키우는 편이 안전하다.

여기서 지키는 것은 **상한이 기본값의 상향을 막지 않는다**는 성질이다. 안전 상한이
기본값과 같으면 `_validate_parameters` 가 ValueError 로 노드를 죽여, 속도를 올리려는
시도가 "노드가 안 뜬다"로 나타난다 — 실제로 상한과 기본값이 둘 다 0.5 였다.
"""

from __future__ import annotations

import pytest

pytest.importorskip('rclpy', reason='requires ROS 2 runtime')

from rdfp.teleop.teleop_keyboard import TeleopKeyboard, _DEFAULT_TASK_LIST   # noqa: E402

# 기본값은 코드에만 있다 — launch/YAML 어디도 `linear_step` 을 설정하지 않으므로
# 이 값이 곧 실효값이다. 0.25 는 2026-09-07 servo 래칫 수정(같은 입력에 2.7배 빨라짐)
# 뒤 수정 전 체감(≈30 mm/s)으로 되맞춘 값이다 — docs/teleop/servo_vs_planned_motion.md §4.1.
DEFAULT_LINEAR_STEP = 0.25
DEFAULT_ANGULAR_STEP = 0.25


class _Logger:
    def info(self, *_a, **_k):
        pass

    def warning(self, *_a, **_k):
        pass

    def error(self, *_a, **_k):
        pass


class _Node:
    """`_validate_parameters` 만 빌려 쓰는 대역."""

    _validate_parameters = TeleopKeyboard._validate_parameters
    MAX_LINEAR_VELOCITY = TeleopKeyboard.MAX_LINEAR_VELOCITY
    MAX_ANGULAR_VELOCITY = TeleopKeyboard.MAX_ANGULAR_VELOCITY
    EMERGENCY_LINEAR_LIMIT = TeleopKeyboard.EMERGENCY_LINEAR_LIMIT
    EMERGENCY_ANGULAR_LIMIT = TeleopKeyboard.EMERGENCY_ANGULAR_LIMIT

    def __init__(self, linear_step, angular_step) -> None:
        self.rate_hz = 100.0
        self.linear_step = linear_step
        self.angular_step = angular_step
        self.deadman_ttl_sec = 0.06
        self.frame_id = 'panda_link0'
        self.tasks = list(_DEFAULT_TASK_LIST)

    def get_logger(self):
        return _Logger()


def test_defaults_pass_validation():
    """기본값으로 노드가 뜬다 — 상한과 같으면 여기서 죽는다."""
    _Node(DEFAULT_LINEAR_STEP, DEFAULT_ANGULAR_STEP)._validate_parameters()


def test_default_leaves_headroom_above_it():
    """기본값 위로 여유가 있어야 사용자가 더 올릴 수 있다."""
    assert TeleopKeyboard.MAX_LINEAR_VELOCITY > DEFAULT_LINEAR_STEP
    assert TeleopKeyboard.MAX_ANGULAR_VELOCITY > DEFAULT_ANGULAR_STEP


def test_step_stays_within_servo_unitless_range():
    """servo 는 `command_in_type: unitless` 로 [-1, 1] 을 받는다 — 넘으면 무의미하다."""
    assert TeleopKeyboard.EMERGENCY_LINEAR_LIMIT <= 1.0
    assert DEFAULT_LINEAR_STEP <= TeleopKeyboard.EMERGENCY_LINEAR_LIMIT


@pytest.mark.parametrize('step', [0.0, -0.1])
def test_nonpositive_linear_step_rejected(step):
    with pytest.raises(ValueError, match='linear_step'):
        _Node(step, DEFAULT_ANGULAR_STEP)._validate_parameters()


def test_linear_step_above_ceiling_rejected():
    """상한 자체는 살아 있어야 한다 — 올린 것은 상한이지 검사가 아니다."""
    over = TeleopKeyboard.MAX_LINEAR_VELOCITY + 0.1
    with pytest.raises(ValueError, match='linear_step'):
        _Node(over, DEFAULT_ANGULAR_STEP)._validate_parameters()


def test_angular_step_above_ceiling_rejected():
    over = TeleopKeyboard.MAX_ANGULAR_VELOCITY + 0.1
    with pytest.raises(ValueError, match='angular_step'):
        _Node(DEFAULT_LINEAR_STEP, over)._validate_parameters()
