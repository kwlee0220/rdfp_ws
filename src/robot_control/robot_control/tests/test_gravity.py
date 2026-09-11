#!/usr/bin/env python3

"""중력 보상 — **조용히 틀릴 수 있는 것만** 고정한다.

이 값이 틀려도 에러가 아니라 "팔이 몇 mm 어긋난 곳에 선다"로 나타난다. 특히 **부호**를
뒤집으면 보상이 처짐을 두 배로 만드는데 증상은 "켰더니 더 나빠졌다" 뿐이다 — 2026-09-11
에 실제로 그랬고, **구현과 시험을 같은 추론으로 쓴 탓에 단위 시험이 못 잡았다.** 그래서
이제 처짐(`deficit`)과 보상값(`offset`)을 나누고, 실측과 맞추는 것은 처짐 쪽이다.
"""

from __future__ import annotations

import pytest

from robot_control.gravity import (
    GravityCompensator, describe, load_gravity_compensator)

READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
# 2026-09-11 실측 (RecurDyn-260907 · r2 · ready 정착). `실제 − 지령`, 단위 rad.
MEASURED_READY_DEFICIT = [0.0000, 0.0009, 0.0005, -0.0097, -0.0007, -0.0008, 0.0000]


@pytest.fixture(scope='module')
def comp():
    return load_gravity_compensator('functionbay')


def test_profile_provides_a_compensator(comp):
    assert comp is not None
    assert len(comp.joint_names) == 7


def test_backends_without_the_key_get_none():
    """**`None` 이 정상 경로다** — mock·Isaac 은 컨트롤러가 중력을 스스로 잡는다."""
    assert load_gravity_compensator('mock') is None


# ----- 실측 대조 — 이 모듈이 존재하는 이유다 ----------------------------------

def test_deficit_matches_the_measured_sag(comp):
    """**핵심 시험.** 예측 처짐 `τ_g/Kp` 가 `ready` 실측 정착 오차와 맞는가.

    실측은 일곱 관절 전부 0.03 mrad 안에서 일치했다. 1 mrad 허용치면 모델이 망가지는
    것(질량 누락, 축 부호 반전, 가지 빠짐)은 다 걸리면서 빌드별 미세차는 통과한다.
    """
    predicted = comp.deficit(READY)

    for name, p, m in zip(comp.joint_names, predicted, MEASURED_READY_DEFICIT):
        assert p == pytest.approx(m, abs=0.001), f'{name}: {p:+.5f} vs 실측 {m:+.5f}'


def test_gravity_neutral_joints_are_zero(comp):
    """연직축 회전이라 중력 토크가 없는 축은 **정확히 0** 이어야 한다.

    0 이 아니면 축 방향이나 트리 구성이 틀린 것이다 — 실측도 ±0.0000 이었다.
    """
    deficit = comp.deficit(READY)

    assert deficit[0] == pytest.approx(0.0, abs=1e-6), 'joint1'
    assert deficit[6] == pytest.approx(0.0, abs=1e-6), 'joint7'


def test_joint4_carries_the_largest_load(comp):
    """`ready` 에서 중력 모멘트가 가장 큰 축은 joint4 다 (실측 처짐 −9.7 mrad)."""
    deficit = comp.deficit(READY)

    assert abs(deficit[3]) == pytest.approx(max(abs(v) for v in deficit))
    assert deficit[3] < 0, '아래로 처지는 방향이라 음수다'


# ----- 부호 — 뒤집으면 오차가 두 배가 된다 -------------------------------------

def test_offset_is_the_opposite_of_the_deficit(comp):
    """**보상값 = −처짐.** 이 한 줄이 2026-09-11 의 두 배 오차를 막는다."""
    for d, o in zip(comp.deficit(READY), comp.offset(READY)):
        assert o == pytest.approx(-d)


def test_compensation_commands_above_the_target(comp):
    """**처질 만큼 미리 더 올려 지령한다.**

    `실제 = 지령 + 처짐` 이고 joint4 의 처짐은 음수(−0.0097)이므로, 실제가 `ready` 에
    서게 하려면 지령이 `ready` 보다 **더 양수** 여야 한다.
    """
    commanded = comp.compensate(READY)

    assert commanded[3] > READY[3], 'joint4 지령이 원 목표보다 양수여야 한다'
    assert commanded[3] - READY[3] == pytest.approx(-comp.deficit(READY)[3])


def test_compensate_leaves_neutral_joints_untouched(comp):
    commanded = comp.compensate(READY)

    assert commanded[0] == pytest.approx(READY[0])
    assert commanded[6] == pytest.approx(READY[6])


# ----- 모델 구조 ---------------------------------------------------------------

def test_torque_is_divided_by_kp_only_in_the_deficit(comp):
    """`τ_g` 는 `Kp` 와 무관하고, 나누는 것은 `deficit` 뿐이다."""
    for t, d, k in zip(comp.torque(READY), comp.deficit(READY), comp.kp):
        assert d == pytest.approx(t / k)


def test_horizontal_upper_arm_loads_joint2_far_more(comp):
    """모멘트 암이 길어지면 토크가 커진다 — 자세 의존이 실제로 계산되는가.

    상완을 눕히면(j2 −π/2) 팔 무게가 joint2 에서 먼 쪽으로 나간다. 세운 자세와 비교해
    한 자릿수 차이가 나야 한다 (계산값 51.8 대 5.1 N·m). 비슷하면 자세를 안 쓰고 있다.
    """
    upright = comp.torque([0.0, 0.0, 0.0, -0.1, 0.0, 1.571, 0.0])[1]
    horizontal = comp.torque([0.0, -1.571, 0.0, -0.1, 0.0, 1.571, 0.0])[1]

    assert abs(horizontal) > 5.0 * abs(upright), \
        f'수평 {horizontal:+.2f} vs 직립 {upright:+.2f} N·m — 자세가 반영되지 않는다'


def test_torque_is_not_constant_across_poses(comp):
    """자세마다 달라야 한다. 같으면 FK 가 관절값을 안 쓰고 있는 것이다."""
    a = comp.torque(READY)
    b = comp.torque([0.0, -1.571, 0.0, -0.1, 0.0, 1.571, 0.0])

    assert max(abs(x - y) for x, y in zip(a, b)) > 1.0


# ----- 거부 경로 ---------------------------------------------------------------

def test_wrong_joint_count_is_rejected(comp):
    """조용히 일부만 보상하면 엉뚱한 관절에 오프셋이 실린다."""
    with pytest.raises(ValueError, match='expected'):
        comp.torque([0.0] * 5)


def test_non_positive_kp_is_rejected(comp):
    with pytest.raises(ValueError, match='kp'):
        GravityCompensator(comp.path, comp.joint_names, 0.0)


def test_unknown_joint_is_rejected(comp):
    with pytest.raises(ValueError, match='unknown joint'):
        GravityCompensator(comp.path, ['panda_joint1', 'no_such_joint'], 2000.0)


def test_scale_is_applied(comp):
    half = GravityCompensator(comp.path, comp.joint_names, comp.kp, scale=0.5)

    assert half.deficit(READY)[3] == pytest.approx(comp.deficit(READY)[3] * 0.5)


# ----- 표시 -------------------------------------------------------------------

def test_describe_mentions_kp_and_urdf(comp):
    text = describe('functionbay')

    assert '2000' in text
    assert 'panda_ftsensor_robotiq' in text


def test_describe_says_so_when_absent():
    assert '없음' in describe('mock')
