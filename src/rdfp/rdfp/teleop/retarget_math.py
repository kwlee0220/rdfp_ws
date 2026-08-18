"""Leader-follower retargeting 의 순수 수학 유틸리티.

ROS 의존성이 없어 단위테스트를 ROS 미소싱 환경에서도 실행할 수 있다.
쿼터니언은 ``(x, y, z, w)`` 튜플, 벡터는 ``(x, y, z)`` 튜플로 표현한다.

매핑 공식 (docs/teleop/leader_follower_mirroring_design.md §3.2):

- ``Δp = R_align · (p_L − p_L₀)``
- ``p_target = p_F₀ + s · Δp``
- ``ΔR = R_align · R_L·R_L₀⁻¹ · R_align⁻¹``
- ``R_target = ΔR · R_F₀``
"""

from __future__ import annotations

from typing import Tuple

import math

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]


def quat_multiply(q1: Quat, q2: Quat) -> Quat:
    """Hamilton 곱 ``q1 * q2`` 를 계산한다."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    )


def quat_conjugate(q: Quat) -> Quat:
    """단위 쿼터니언의 켤레(=역원)를 반환한다."""
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_normalize(q: Quat) -> Quat:
    """쿼터니언을 단위 크기로 정규화한다."""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError('Cannot normalize a zero-norm quaternion')
    return (x / n, y / n, z / n, w / n)


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quat:
    """roll-pitch-yaw (rad, extrinsic XYZ) 를 쿼터니언으로 변환한다."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy
    )


def rotate_vec(q: Quat, v: Vec3) -> Vec3:
    """벡터 ``v`` 를 쿼터니언 ``q`` 로 회전한다 (``v' = q v q*``)."""
    qv = (v[0], v[1], v[2], 0.0)
    rx, ry, rz, _ = quat_multiply(quat_multiply(q, qv), quat_conjugate(q))
    return (rx, ry, rz)


def quat_slerp(q1: Quat, q2: Quat, t: float) -> Quat:
    """``q1`` 에서 ``q2`` 로의 구면 선형 보간을 계산한다 (``t`` ∈ [0, 1])."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    dot = x1 * x2 + y1 * y2 + z1 * z2 + w1 * w2
    # 최단 경로를 위해 내적이 음수면 한쪽 부호를 뒤집는다.
    if dot < 0.0:
        x2, y2, z2, w2 = -x2, -y2, -z2, -w2
        dot = -dot
    if dot > 0.9995:
        # 매우 가까우면 선형 보간 후 정규화로 대체한다.
        return quat_normalize((
            x1 + (x2 - x1) * t,
            y1 + (y2 - y1) * t,
            z1 + (z2 - z1) * t,
            w1 + (w2 - w1) * t
        ))
    theta = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta = math.sin(theta)
    a = math.sin((1.0 - t) * theta) / sin_theta
    b = math.sin(t * theta) / sin_theta
    return (a * x1 + b * x2, a * y1 + b * y2, a * z1 + b * z2, a * w1 + b * w2)


def lpf_alpha(dt: float, cutoff_hz: float) -> float:
    """1차 저역통과 필터의 보간 계수 ``alpha`` 를 계산한다.

    ``y = y_prev + alpha * (x - y_prev)`` 에 사용한다. ``cutoff_hz <= 0`` 이면
    필터를 끄는 의미로 1.0 을 반환한다.
    """
    if cutoff_hz <= 0.0:
        return 1.0
    if dt <= 0.0:
        return 0.0
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    return dt / (dt + rc)


def clamp_to_box(p: Vec3, box_min: Vec3, box_max: Vec3) -> Vec3:
    """점 ``p`` 를 축정렬 박스 ``[box_min, box_max]`` 내부로 clamp 한다."""
    return (
        min(max(p[0], box_min[0]), box_max[0]),
        min(max(p[1], box_min[1]), box_max[1]),
        min(max(p[2], box_min[2]), box_max[2])
    )


def compute_target(leader_pos: Vec3, leader_quat: Quat,
                   anchor_leader_pos: Vec3, anchor_leader_quat: Quat,
                   anchor_follower_pos: Vec3, anchor_follower_quat: Quat,
                   align_quat: Quat, position_scale: float) -> tuple[Vec3, Quat]:
    """클러치 앵커 기준 상대 매핑으로 follower 목표 pose 를 계산한다.

    Args:
        leader_pos / leader_quat: 현재 leader EE pose (leader base frame).
        anchor_leader_pos / anchor_leader_quat: engage 시점의 leader EE pose (L₀).
        anchor_follower_pos / anchor_follower_quat: engage 시점의 follower EE pose (F₀).
        align_quat: leader base → follower base 방향 정렬 회전 (R_align).
        position_scale: 위치 변위 스케일 계수 (s).

    Returns:
        ``(p_target, q_target)`` — follower base frame 기준 목표 pose.
    """
    # Δp = R_align · (p_L − p_L₀),  p_target = p_F₀ + s·Δp
    dp_leader = (
        leader_pos[0] - anchor_leader_pos[0],
        leader_pos[1] - anchor_leader_pos[1],
        leader_pos[2] - anchor_leader_pos[2]
    )
    dp = rotate_vec(align_quat, dp_leader)
    p_target = (
        anchor_follower_pos[0] + position_scale * dp[0],
        anchor_follower_pos[1] + position_scale * dp[1],
        anchor_follower_pos[2] + position_scale * dp[2]
    )

    # ΔR = R_align · R_L·R_L₀⁻¹ · R_align⁻¹,  R_target = ΔR · R_F₀
    q_rel = quat_multiply(leader_quat, quat_conjugate(anchor_leader_quat))
    q_rel_aligned = quat_multiply(quat_multiply(align_quat, q_rel), quat_conjugate(align_quat))
    q_target = quat_normalize(quat_multiply(q_rel_aligned, anchor_follower_quat))
    return p_target, q_target


__all__ = [
    'Vec3', 'Quat',
    'quat_multiply', 'quat_conjugate', 'quat_normalize', 'quat_from_rpy',
    'rotate_vec', 'quat_slerp', 'lpf_alpha', 'clamp_to_box', 'compute_target'
]
