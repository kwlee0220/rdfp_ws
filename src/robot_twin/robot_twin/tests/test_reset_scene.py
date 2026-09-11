"""`reset_scene` 의 샘플링·검증 테스트 (ROS 불필요)."""

from __future__ import annotations

import math

import pytest

from robot_twin.backends import _sample_axis, _sample_scene
from robot_twin.config import OperationConfig


ONE_CUBE = {'objects': [{'name': 'cube_0', 'type': 'box', 'size': [0.05, 0.05, 0.05],
                         'x': [0.35, 0.55], 'y': [-0.15, 0.15], 'z': 0.025}]}


def test_same_seed_gives_the_same_placement() -> None:
    assert _sample_scene(ONE_CUBE, 42) == _sample_scene(ONE_CUBE, 42)


def test_different_seeds_move_the_object() -> None:
    """랜덤화가 실제로 일어나야 수집 루프가 의미를 갖는다."""
    a = _sample_scene(ONE_CUBE, 1)[0]['position']
    b = _sample_scene(ONE_CUBE, 2)[0]['position']

    assert (a['x'], a['y']) != (b['x'], b['y'])


def test_sampled_values_stay_inside_the_declared_range() -> None:
    for seed in range(30):
        pos = _sample_scene(ONE_CUBE, seed)[0]['position']
        assert 0.35 <= pos['x'] <= 0.55
        assert -0.15 <= pos['y'] <= 0.15
        assert pos['z'] == 0.025          # 숫자는 고정값이다


def test_geometry_is_copied_verbatim() -> None:
    """dimensions 순서는 SolidPrimitive 규약이라 재배열하지 않는다."""
    obj = _sample_scene(ONE_CUBE, 0)[0]

    assert obj['name'] == 'cube_0'
    assert obj['type'] == 'box'
    assert obj['dimensions'] == [0.05, 0.05, 0.05]


def test_name_is_derived_when_omitted() -> None:
    out = _sample_scene({'objects': [{'type': 'sphere', 'size': [0.02]}]}, 0)

    assert out[0]['name'] == 'sphere_0'


def test_empty_recipe_yields_an_empty_scene() -> None:
    """물체가 없는 것도 'scene 을 비운다'는 유효한 요청이다."""
    assert _sample_scene({'objects': []}, 0) == []


# ----- 축 지정 -----

def test_axis_accepts_a_fixed_number() -> None:
    assert _sample_axis(None, 0.025, 'z') == 0.025


def test_axis_rejects_an_inverted_range() -> None:
    """조용히 뒤집어 쓰면 의도한 범위 밖에 물체가 놓인다."""
    import random

    with pytest.raises(ValueError, match='inverted'):
        _sample_axis(random.Random(0), [0.5, 0.3], 'cube_0.x')


def test_axis_rejects_a_malformed_spec() -> None:
    import random

    for bad in ([0.1], [0.1, 0.2, 0.3], 'near', None, True):
        with pytest.raises(ValueError, match='cube_0.x'):
            _sample_axis(random.Random(0), bad, 'cube_0.x')


# ----- enum 파생 -----

def test_scene_enum_is_derived_from_the_recipe_table() -> None:
    """두 곳에 적으면 조용히 어긋난다 — 레시피 표가 단일 출처다."""
    op = OperationConfig(
        name='reset_scene', kind='sync',
        backend={'scenes': {'b': {}, 'a': {}}},
        inputs_schema={'type': 'object', 'properties': {'scene': {'type': 'string'}}})

    assert op.inputs_schema['properties']['scene']['enum'] == ['a', 'b']


def test_mismatched_scene_enum_fails_at_load() -> None:
    with pytest.raises(ValueError, match='does not match backend.scenes'):
        OperationConfig(
            name='reset_scene', kind='sync',
            backend={'scenes': {'a': {}}},
            inputs_schema={'type': 'object',
                           'properties': {'scene': {'type': 'string', 'enum': ['a', 'b']}}})


# ----- 자원 -----

def test_resource_accepts_a_list() -> None:
    op = OperationConfig(name='reset_scene', kind='sync', resource=['scene', 'arm'])

    assert op.resource_names == ('scene', 'arm')


def test_resource_accepts_a_bare_string() -> None:
    """단일 문자열도 같은 형태로 읽혀야 호출부가 두 경우를 나눠 다루지 않는다."""
    assert OperationConfig(name='x', kind='sync', resource='arm').resource_names == ('arm',)
    assert OperationConfig(name='x', kind='sync').resource_names == ()


# ---------- 자세(yaw) ----------
#
# 위치만 흩어 놓으면 물체가 늘 같은 방향을 보고, 파지 자세가 한 번도 안 바뀐다.
# **대칭을 넘겨 뽑지 않는 것이 요점이다** — 정육면체는 90° 마다 같은 모양이라
# [0, 360) 으로 뽑으면 같은 자세를 네 번 세는 셈이다.

RECIPE = ONE_CUBE

CUBE_WITH_YAW = {'objects': [{'name': 'block_a', 'type': 'box', 'size': [0.05, 0.05, 0.05],
                              'x': 0.4, 'y': 0.0, 'z': 0.425, 'yaw': [0, 90]}]}


def _yaw_deg(quat: dict) -> float:
    return math.degrees(math.atan2(2.0 * (quat['w'] * quat['z']),
                                   1.0 - 2.0 * quat['z'] * quat['z']))


def test_yaw_absent_means_no_rotation() -> None:
    """`yaw` 를 안 쓴 레시피(실린더 등)는 단위 쿼터니언이어야 한다."""
    obj = _sample_scene(RECIPE, 0)[0]

    assert obj['orientation'] == {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    assert obj['yaw_deg'] == 0.0


def test_yaw_is_sampled_inside_the_declared_range() -> None:
    seen = [_sample_scene(CUBE_WITH_YAW, seed)[0]['yaw_deg'] for seed in range(40)]

    assert all(0.0 <= value <= 90.0 for value in seen), seen
    assert len(set(seen)) > 30, '씨앗이 달라도 각도가 거의 안 바뀐다'


def test_yaw_becomes_a_z_only_quaternion() -> None:
    """탁자 위 물체를 눕히면 위에서 잡는 경로가 성립하지 않는다 — z축만 돈다."""
    for seed in range(10):
        quat = _sample_scene(CUBE_WITH_YAW, seed)[0]['orientation']
        assert quat['x'] == 0.0 and quat['y'] == 0.0
        assert abs(quat['x'] ** 2 + quat['y'] ** 2 + quat['z'] ** 2 + quat['w'] ** 2 - 1.0) < 1e-9


def test_quaternion_matches_the_reported_angle() -> None:
    """`yaw_deg` 는 사람이 읽는 값이고 재현의 근거는 쿼터니언이다 — 둘이 어긋나면 안 된다."""
    for seed in range(10):
        obj = _sample_scene(CUBE_WITH_YAW, seed)[0]
        assert abs(_yaw_deg(obj['orientation']) - obj['yaw_deg']) < 1e-6


def test_yaw_follows_the_seed() -> None:
    assert (_sample_scene(CUBE_WITH_YAW, 5)[0]['yaw_deg']
            == _sample_scene(CUBE_WITH_YAW, 5)[0]['yaw_deg'])
    assert (_sample_scene(CUBE_WITH_YAW, 5)[0]['yaw_deg']
            != _sample_scene(CUBE_WITH_YAW, 6)[0]['yaw_deg'])


def test_yaw_accepts_a_fixed_number() -> None:
    recipe = {'objects': [dict(CUBE_WITH_YAW['objects'][0], yaw=30.0)]}

    assert _sample_scene(recipe, 0)[0]['yaw_deg'] == 30.0
