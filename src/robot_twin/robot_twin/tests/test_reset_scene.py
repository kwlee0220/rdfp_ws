"""`reset_scene` 의 샘플링·검증 테스트 (ROS 불필요)."""

from __future__ import annotations

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
    """물체가 없는 것도 '씬을 비운다'는 유효한 요청이다."""
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
