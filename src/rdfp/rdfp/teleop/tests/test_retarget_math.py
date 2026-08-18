"""retarget_math 순수 수학 유틸리티 테스트 (ROS 미소싱 환경에서 실행 가능)."""

from __future__ import annotations

import math

import pytest

from rdfp.teleop.retarget_math import (
    clamp_to_box, compute_target, lpf_alpha, quat_conjugate, quat_from_rpy,
    quat_multiply, quat_normalize, quat_slerp, rotate_vec
)

_IDENTITY = (0.0, 0.0, 0.0, 1.0)


def _quat_close(q1, q2, tol=1e-9):
    """두 쿼터니언이 같은 회전을 나타내는지 확인한다 (부호 모호성 허용)."""
    dot = sum(a * b for a, b in zip(q1, q2))
    return abs(abs(dot) - 1.0) < tol


def _vec_close(v1, v2, tol=1e-9):
    return all(abs(a - b) < tol for a, b in zip(v1, v2))


class TestQuatBasics:
    def test_multiply_identity(self):
        q = quat_from_rpy(0.3, -0.2, 0.7)
        assert _quat_close(quat_multiply(q, _IDENTITY), q)
        assert _quat_close(quat_multiply(_IDENTITY, q), q)

    def test_conjugate_is_inverse(self):
        q = quat_from_rpy(0.5, 0.1, -0.9)
        assert _quat_close(quat_multiply(q, quat_conjugate(q)), _IDENTITY)

    def test_normalize_zero_raises(self):
        with pytest.raises(ValueError):
            quat_normalize((0.0, 0.0, 0.0, 0.0))

    def test_from_rpy_yaw90(self):
        # yaw +90°: x축 단위벡터가 y축으로 회전한다.
        q = quat_from_rpy(0.0, 0.0, math.pi / 2)
        assert _vec_close(rotate_vec(q, (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0), tol=1e-12)

    def test_rotate_vec_roundtrip(self):
        q = quat_from_rpy(0.2, 0.4, -0.6)
        v = (0.3, -0.7, 1.1)
        restored = rotate_vec(quat_conjugate(q), rotate_vec(q, v))
        assert _vec_close(restored, v, tol=1e-12)


class TestSlerp:
    def test_endpoints(self):
        q1 = quat_from_rpy(0.0, 0.0, 0.0)
        q2 = quat_from_rpy(0.0, 0.0, 1.0)
        assert _quat_close(quat_slerp(q1, q2, 0.0), q1)
        assert _quat_close(quat_slerp(q1, q2, 1.0), q2)

    def test_midpoint_half_angle(self):
        q1 = _IDENTITY
        q2 = quat_from_rpy(0.0, 0.0, 1.0)
        mid = quat_slerp(q1, q2, 0.5)
        assert _quat_close(mid, quat_from_rpy(0.0, 0.0, 0.5), tol=1e-9)

    def test_shortest_path_with_flipped_sign(self):
        q1 = _IDENTITY
        q2 = quat_from_rpy(0.0, 0.0, 0.4)
        q2_neg = tuple(-c for c in q2)
        # 부호가 뒤집힌 동일 회전에 대해서도 최단 경로 보간이어야 한다.
        assert _quat_close(quat_slerp(q1, q2_neg, 0.5), quat_from_rpy(0.0, 0.0, 0.2))


class TestLpfAlpha:
    def test_disabled_cutoff_returns_one(self):
        assert lpf_alpha(0.02, 0.0) == 1.0
        assert lpf_alpha(0.02, -1.0) == 1.0

    def test_zero_dt_returns_zero(self):
        assert lpf_alpha(0.0, 3.0) == 0.0

    def test_alpha_in_unit_range_and_monotonic(self):
        a_slow = lpf_alpha(0.02, 1.0)
        a_fast = lpf_alpha(0.02, 10.0)
        assert 0.0 < a_slow < a_fast < 1.0


class TestClampToBox:
    def test_inside_unchanged(self):
        p = (0.1, 0.2, 0.3)
        assert clamp_to_box(p, (-1.0, -1.0, -1.0), (1.0, 1.0, 1.0)) == p

    def test_outside_clamped(self):
        p = (2.0, -3.0, 0.5)
        assert clamp_to_box(p, (-1.0, -1.0, 0.0), (1.0, 1.0, 1.0)) == (1.0, -1.0, 0.5)


class TestComputeTarget:
    """클러치 앵커 기반 상대 매핑의 성질을 검증한다."""

    L0_POS = (0.5, 0.1, 0.3)
    L0_QUAT = quat_from_rpy(0.0, 0.0, 0.2)
    F0_POS = (0.4, -0.2, 0.6)
    F0_QUAT = quat_from_rpy(0.1, 0.0, -0.5)

    def test_at_anchor_returns_follower_anchor(self):
        # L = L₀ 이면 target = F₀ (engage 순간 목표 점프 없음).
        p, q = compute_target(
            self.L0_POS, self.L0_QUAT, self.L0_POS, self.L0_QUAT,
            self.F0_POS, self.F0_QUAT, _IDENTITY, 1.0)
        assert _vec_close(p, self.F0_POS)
        assert _quat_close(q, self.F0_QUAT)

    def test_pure_translation_identity_align(self):
        leader_pos = (self.L0_POS[0] + 0.1, self.L0_POS[1] - 0.05, self.L0_POS[2] + 0.02)
        p, q = compute_target(
            leader_pos, self.L0_QUAT, self.L0_POS, self.L0_QUAT,
            self.F0_POS, self.F0_QUAT, _IDENTITY, 1.0)
        expected = (self.F0_POS[0] + 0.1, self.F0_POS[1] - 0.05, self.F0_POS[2] + 0.02)
        assert _vec_close(p, expected, tol=1e-12)
        assert _quat_close(q, self.F0_QUAT)

    def test_position_scale(self):
        leader_pos = (self.L0_POS[0] + 0.1, self.L0_POS[1], self.L0_POS[2])
        p, _ = compute_target(
            leader_pos, self.L0_QUAT, self.L0_POS, self.L0_QUAT,
            self.F0_POS, self.F0_QUAT, _IDENTITY, 2.5)
        assert _vec_close(p, (self.F0_POS[0] + 0.25, self.F0_POS[1], self.F0_POS[2]), tol=1e-12)

    def test_align_rotates_displacement(self):
        # R_align = yaw +90°: leader 의 +x 변위가 follower 의 +y 변위로 매핑된다.
        align = quat_from_rpy(0.0, 0.0, math.pi / 2)
        leader_pos = (self.L0_POS[0] + 0.1, self.L0_POS[1], self.L0_POS[2])
        p, _ = compute_target(
            leader_pos, self.L0_QUAT, self.L0_POS, self.L0_QUAT,
            self.F0_POS, self.F0_QUAT, align, 1.0)
        assert _vec_close(p, (self.F0_POS[0], self.F0_POS[1] + 0.1, self.F0_POS[2]), tol=1e-12)

    def test_relative_rotation_identity_align(self):
        # leader 가 z축으로 +0.3rad 돌면 follower 목표도 F₀ 대비 z축 +0.3rad 회전한다.
        leader_quat = quat_multiply(quat_from_rpy(0.0, 0.0, 0.3), self.L0_QUAT)
        _, q = compute_target(
            self.L0_POS, leader_quat, self.L0_POS, self.L0_QUAT,
            self.F0_POS, self.F0_QUAT, _IDENTITY, 1.0)
        expected = quat_multiply(quat_from_rpy(0.0, 0.0, 0.3), self.F0_QUAT)
        assert _quat_close(q, expected)

    def test_relative_rotation_with_align(self):
        # R_align = yaw +90°: leader base 의 x축 회전이 follower base 의 y축 회전으로
        # 켤레 변환(conjugation)되어 매핑된다.
        align = quat_from_rpy(0.0, 0.0, math.pi / 2)
        leader_quat = quat_multiply(quat_from_rpy(0.4, 0.0, 0.0), self.L0_QUAT)
        _, q = compute_target(
            self.L0_POS, leader_quat, self.L0_POS, self.L0_QUAT,
            self.F0_POS, self.F0_QUAT, align, 1.0)
        expected = quat_multiply(quat_from_rpy(0.0, 0.4, 0.0), self.F0_QUAT)
        assert _quat_close(q, expected)

    def test_mirror_image_via_align_yaw_pi(self):
        # R_align = yaw 180°: 거울상 매핑 — leader +x 변위가 follower -x 로 간다.
        align = quat_from_rpy(0.0, 0.0, math.pi)
        leader_pos = (self.L0_POS[0] + 0.1, self.L0_POS[1] + 0.05, self.L0_POS[2])
        p, _ = compute_target(
            leader_pos, self.L0_QUAT, self.L0_POS, self.L0_QUAT,
            self.F0_POS, self.F0_QUAT, align, 1.0)
        expected = (self.F0_POS[0] - 0.1, self.F0_POS[1] - 0.05, self.F0_POS[2])
        assert _vec_close(p, expected, tol=1e-12)

    def test_output_quaternion_is_normalized(self):
        leader_quat = quat_from_rpy(0.7, -0.3, 1.2)
        _, q = compute_target(
            (1.0, 2.0, 3.0), leader_quat, self.L0_POS, self.L0_QUAT,
            self.F0_POS, self.F0_QUAT, quat_from_rpy(0.1, 0.2, 0.3), 1.5)
        norm = math.sqrt(sum(c * c for c in q))
        assert abs(norm - 1.0) < 1e-12
