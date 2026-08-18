"""pose_math 단위 테스트 (ROS 불필요).

쿼터니언 순서·합성 오류는 크래시가 아니라 '그럴듯하게 틀린 자세'로 나타난다.
노름 검사로는 잡히지 않으므로 **알려진 회전**으로 고정한다.
"""

from __future__ import annotations

import math

import pytest

from rdfp.scene.pose_math import IDENTITY_QUAT, compose_pose, quat_multiply, quat_rotate


# z축 +90° (ROS xyzw). x축 → y축.
Z90 = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
# x축 180°. pick 예제의 DOWN 과 같은 값이며, Isaac 의 identity 를 그대로 옮겼을 때
# 나오는 값이기도 하다 — 그래서 눈으로는 오류가 드러나지 않는다.
X180 = (1.0, 0.0, 0.0, 0.0)


def _close(actual, expected, tol: float = 1e-9) -> None:
    assert actual == pytest.approx(expected, abs=tol)


# ----- quat_rotate -----

def test_identity_rotation_leaves_the_vector_alone() -> None:
    _close(quat_rotate(IDENTITY_QUAT, (0.4, -0.1, 0.2)), (0.4, -0.1, 0.2))


def test_z90_maps_x_axis_to_y_axis() -> None:
    """축이 섞이는 회전이라야 순서 오류가 드러난다."""
    _close(quat_rotate(Z90, (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0))


def test_z90_maps_y_axis_to_negative_x() -> None:
    _close(quat_rotate(Z90, (0.0, 1.0, 0.0)), (-1.0, 0.0, 0.0))


def test_x180_flips_y_and_z() -> None:
    _close(quat_rotate(X180, (0.1, 0.2, 0.3)), (0.1, -0.2, -0.3))


def test_rotation_preserves_length() -> None:
    out = quat_rotate(Z90, (0.3, 0.4, 0.5))
    assert math.hypot(*out) == pytest.approx(math.hypot(0.3, 0.4, 0.5))


# ----- quat_multiply -----

def test_identity_is_neutral_on_both_sides() -> None:
    _close(quat_multiply(IDENTITY_QUAT, Z90), Z90)
    _close(quat_multiply(Z90, IDENTITY_QUAT), Z90)


def test_two_z90_rotations_make_z180() -> None:
    _close(quat_multiply(Z90, Z90), (0.0, 0.0, 1.0, 0.0))


def test_multiply_is_not_commutative() -> None:
    """교환법칙이 성립하지 않으므로 인자 순서(부모, 자식)가 중요하다."""
    assert quat_multiply(Z90, X180) != quat_multiply(X180, Z90)


def test_composed_rotation_matches_sequential_rotation() -> None:
    vec = (0.3, 0.0, 0.1)
    combined = quat_multiply(Z90, X180)

    _close(quat_rotate(combined, vec), quat_rotate(Z90, quat_rotate(X180, vec)))


# ----- compose_pose -----

def test_compose_with_identity_parent_is_a_plain_offset() -> None:
    position, orientation = compose_pose(
        (1.0, 2.0, 3.0), IDENTITY_QUAT, (0.1, 0.0, 0.0), IDENTITY_QUAT)

    _close(position, (1.1, 2.0, 3.0))
    _close(orientation, IDENTITY_QUAT)


def test_child_offset_is_rotated_by_the_parent() -> None:
    """부모가 z축 90° 돌아 있으면 자식의 +x 오프셋은 부모의 +y 로 간다.

    이 합성을 빼먹으면 오프셋을 가진 복합 물체의 위치가 조용히 틀린다.
    """
    position, orientation = compose_pose(
        (1.0, 0.0, 0.0), Z90, (0.2, 0.0, 0.0), IDENTITY_QUAT)

    _close(position, (1.0, 0.2, 0.0))
    _close(orientation, Z90)


def test_orientations_compose_as_well_as_positions() -> None:
    _, orientation = compose_pose((0.0, 0.0, 0.0), Z90, (0.0, 0.0, 0.0), Z90)

    _close(orientation, (0.0, 0.0, 1.0, 0.0))


def test_compose_is_associative_over_three_frames() -> None:
    """TF ∘ (object ∘ primitive) 와 (TF ∘ object) ∘ primitive 가 같아야 한다."""
    a_pos, a_quat = (0.5, 0.0, 0.1), Z90
    b_pos, b_quat = (0.2, 0.1, 0.0), X180
    c_pos, c_quat = (0.0, 0.3, 0.05), Z90

    inner_pos, inner_quat = compose_pose(b_pos, b_quat, c_pos, c_quat)
    left = compose_pose(a_pos, a_quat, inner_pos, inner_quat)

    outer_pos, outer_quat = compose_pose(a_pos, a_quat, b_pos, b_quat)
    right = compose_pose(outer_pos, outer_quat, c_pos, c_quat)

    _close(left[0], right[0])
    _close(left[1], right[1])
