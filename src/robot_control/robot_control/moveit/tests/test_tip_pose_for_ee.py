#!/usr/bin/env python3

"""EE 프레임 → tip link 변환 — **149 mm 를 조용히 틀리는 것을 막는다.**

데카르트 목표는 MoveIt 그룹의 tip link 기준으로 해석되는데, 사람이 보는 프레임은
보통 그것이 아니다. 펑션베이는 `/ee_pose` 가 `grasp_center` 를 가리키고 tip
(`panda_link8`) 과 **149 mm** 떨어져 있다 — 그 값을 그대로 목표로 넣으면 팔이 그만큼
엉뚱한 곳으로 간다. 에러가 아니라 "왜 여기로 가지" 로 나타난다.
"""

from __future__ import annotations

import math

import pytest

from robot_control.moveit.utils import tip_pose_for_ee

IDENTITY = (0.0, 0.0, 0.0, 1.0)
# 펑션베이 실측 (2026-09-11): tip → grasp_center 는 z +149.3 mm 이고 z 둘레 90° 다.
# **회전은 정확한 값을 쓴다** — TF 가 찍어 주는 `0.70711` 을 그대로 적으면 노름이
# 1.0000018 이라 왕복 시험이 9e-6 으로 어긋난다 (구현이 아니라 상수 탓이다).
FB_TRANSLATION = (0.0, 0.0003, 0.14927)
FB_ROTATION = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))


def test_no_offset_is_a_no_op():
    """오프셋이 없으면 그대로다 — mock·Isaac 처럼 EE 가 곧 tip 인 경우."""
    position, rotation = tip_pose_for_ee((0.5, 0.1, 0.3), IDENTITY, (0, 0, 0), IDENTITY)

    assert position == pytest.approx((0.5, 0.1, 0.3))
    assert rotation == pytest.approx(IDENTITY)


def test_pure_translation_subtracts_along_the_tip_axes():
    """자세가 항등이면 tip 은 EE 목표에서 오프셋만큼 **뒤로** 물러난다."""
    position, _ = tip_pose_for_ee((0.5, 0.0, 0.3), IDENTITY, (0.0, 0.0, 0.15), IDENTITY)

    assert position == pytest.approx((0.5, 0.0, 0.15))


def test_offset_follows_the_commanded_orientation():
    """**오프셋은 목표 자세를 따라 돈다** — 위치만 빼면 자세가 바뀔 때 틀린다.

    EE 가 z 둘레로 180° 돌아 있으면 z 오프셋은 여전히 z 지만, x 오프셋은 뒤집힌다.
    """
    half_turn = (0.0, 0.0, 1.0, 0.0)          # z 둘레 180°
    position, _ = tip_pose_for_ee((0.5, 0.0, 0.3), half_turn, (0.1, 0.0, 0.0), IDENTITY)

    assert position == pytest.approx((0.6, 0.0, 0.3), abs=1e-9)


def test_round_trip_returns_the_ee_pose():
    """tip 자세로 되짚으면 원래 EE 목표가 나와야 한다 — 변환이 가역이어야 한다."""
    from robot_control.moveit.utils import _quaternion_multiply, _rotate

    goal_position = (0.50, -0.10, 0.12)
    goal_orientation = (0.0, 1.0, 0.0, 0.0)          # 연직 하향

    tip_position, tip_rotation = tip_pose_for_ee(
        goal_position, goal_orientation, FB_TRANSLATION, FB_ROTATION)

    back_position = tuple(t + s for t, s in
                          zip(tip_position, _rotate(tip_rotation, FB_TRANSLATION)))
    back_orientation = _quaternion_multiply(tip_rotation, FB_ROTATION)

    assert back_position == pytest.approx(goal_position, abs=1e-9)
    assert back_orientation == pytest.approx(goal_orientation, abs=1e-9)


def test_functionbay_offset_moves_the_tip_by_149_mm():
    """**실측 회귀.** `grasp_center` 목표와 tip 지령은 149 mm 떨어져야 한다.

    0 에 가까우면 변환이 안 걸린 것이고, 그대로 두면 팔이 149 mm 어긋난 곳에 선다.
    """
    goal = (0.50, 0.0, 0.082)
    tip_position, _ = tip_pose_for_ee(goal, (0.0, 1.0, 0.0, 0.0),
                                      FB_TRANSLATION, FB_ROTATION)

    distance = math.dist(goal, tip_position)

    assert distance == pytest.approx(0.1493, abs=0.0005)


def test_rotation_is_composed_not_ignored():
    """tip 자세 = EE 자세 ⊗ (tip→EE 회전)⁻¹ — 90° 를 빠뜨리면 개폐축이 어긋난다."""
    _, tip_rotation = tip_pose_for_ee((0, 0, 0), IDENTITY, (0, 0, 0), FB_ROTATION)

    # 항등 EE 자세에 90° 오프셋이면 tip 은 −90° 여야 한다.
    assert tip_rotation == pytest.approx((0.0, 0.0, -0.70711, 0.70711), abs=1e-5)
