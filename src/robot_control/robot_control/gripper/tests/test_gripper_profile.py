#!/usr/bin/env python3

"""그리퍼 명령 규약 — **조용히 틀릴 수 있는 것만** 고정한다.

`/input/gripper_joint` 는 이름 없는 `Float64MultiArray` 라 배열 순서·단위·packing 이
곧 계약인데, 그 계약이 **씬마다 다르다**. 틀려도 에러가 아니라 "그리퍼가 거의 안
움직인다"로 나타난다 — 라디안을 도로 읽는 씬에 라디안을 보내면 `π/180 = 1.745%` 만
움직인다 (2026-09-10 실측).
"""

from __future__ import annotations

import json
import math

import pytest

from robot_control.gripper.profile import (
    describe, gripper_profile_path, load_gripper_profile,
    load_gripper_profile_file)


@pytest.fixture(scope='module')
def profile():
    return load_gripper_profile('functionbay')


def test_the_file_is_reachable():
    import os

    path = gripper_profile_path('functionbay')
    assert path and os.path.isfile(path), path


def test_axis_count_matches_the_signs(profile):
    assert profile.axis_count == len(profile.axis_signs)


# ----- 단위 — 틀리면 1.7% 만 움직인다 -----------------------------------------

def test_degree_profile_converts_the_command(profile):
    """**핵심 회귀 시험.** r2 는 도로 받는다 — 라디안을 그대로 보내면 π/180 만 간다."""
    assert profile.command_units == 'deg', '이 시험은 r2(RecurDyn) 프로파일 전제다'

    first = profile.command(0.725)[0]

    assert first == pytest.approx(math.degrees(0.725), abs=1e-6)
    assert first > 40.0, f'라디안을 그대로 내보내고 있다 ({first})'


def test_report_is_read_as_radians_not_degrees(profile):
    """**보고는 어느 규약에서도 라디안이다** — 환산은 명령 방향뿐이다."""
    assert profile.scalar_from_report([0.725, -0.725]) == pytest.approx(0.725)


# ----- packing — 슬롯을 잘못 채우면 속도·힘 자리에 각도가 들어간다 -------------

def test_pvf_packing_puts_zero_in_velocity_and_force(profile):
    """관절당 `(위치, 속도, 힘)`. **힘 슬롯이 0 이 아니면 물체를 밀어붙인다.**"""
    assert profile.command_packing == 'pvf', '이 시험은 r2 프로파일 전제다'

    data = profile.command(0.3)

    assert len(data) == 3 * profile.axis_count
    assert data[1] == 0.0 and data[2] == 0.0
    assert data[4] == 0.0 and data[5] == 0.0
    assert data[0] == pytest.approx(-data[3])


def test_position_packing_is_plain(tmp_path):
    """t1(Crisp) 쪽 — 6축을 라디안 그대로 나열한다."""
    path = tmp_path / 'crisp.json'
    path.write_text(json.dumps({'axis_signs': [1, 1, -1, -1, -1, 1],
                                'command_units': 'rad', 'command_packing': 'position'}),
                    encoding='utf-8')
    crisp = load_gripper_profile_file(str(path))

    assert crisp.command(0.725) == [0.725, 0.725, -0.725, -0.725, -0.725, 0.725]


# ----- 거부 경로 — 조용히 진행하면 명령이 통째로 오해된다 ----------------------

def test_axis_count_mismatch_is_refused(profile):
    """씬을 바꾸고 파일을 안 바꿨을 때 **가장 먼저 틀리는 값이 축 수**다."""
    with pytest.raises(ValueError, match='reports'):
        profile.verify_report([0.0] * 6)

    profile.verify_report([0.0] * profile.axis_count)      # 맞으면 조용히 통과한다


def test_unknown_unit_is_rejected(tmp_path):
    path = tmp_path / 'bad.json'
    path.write_text(json.dumps({'axis_signs': [1, -1], 'command_units': 'turns'}),
                    encoding='utf-8')

    with pytest.raises(ValueError, match='command_units'):
        load_gripper_profile_file(str(path))


def test_unknown_packing_is_rejected(tmp_path):
    path = tmp_path / 'bad.json'
    path.write_text(json.dumps({'axis_signs': [1, -1], 'command_packing': 'interleaved'}),
                    encoding='utf-8')

    with pytest.raises(ValueError, match='command_packing'):
        load_gripper_profile_file(str(path))


def test_declared_count_must_match_the_signs(tmp_path):
    path = tmp_path / 'bad.json'
    path.write_text(json.dumps({'axis_signs': [1, -1], 'axis_count': 6}), encoding='utf-8')

    with pytest.raises(ValueError, match='axis_count'):
        load_gripper_profile_file(str(path))


def test_missing_file_is_not_an_empty_profile():
    """"규약이 필요 없는 백엔드"와 "아직 안 적은 백엔드"가 구별되어야 한다."""
    with pytest.raises(FileNotFoundError, match='gripper.profile_file'):
        load_gripper_profile('mock')


def test_non_finite_command_is_refused(profile):
    """NaN 을 내보내면 **전 관절이 발산하고 되돌아오지 않는다** (2026-09-09 실측)."""
    with pytest.raises(ValueError, match='non-finite'):
        profile.command(float('nan'))


# ----- 출처 — 어느 씬 값인지 사람이 대조할 수 있어야 한다 ---------------------

def test_provenance_is_recorded(profile):
    assert profile.solver, '어느 솔버 씬인지 적혀 있어야 한다'
    assert profile.scene, '어느 씬인지 적혀 있어야 한다'


def test_describe_shows_the_convention(profile):
    text = describe('functionbay')

    assert profile.command_units in text
    assert profile.command_packing in text
    assert profile.solver in text
