"""`robot_twin.serialize` 단위 테스트 — ROS 없이 동작한다.

ROS 메시지는 ``get_fields_and_field_types`` 를 갖는 가짜 객체로 대체한다.
직렬화 규약 자체를 검증하는 것이 목적이므로 실제 rosidl 타입이 필요 없다.
"""

from __future__ import annotations

from typing import Any

import math

import pytest

from robot_twin.serialize import (
    SerializationError, apply_enums, extract_stamp, is_ros_message,
    project_joint_state_map, project_scene_object_map, to_jsonable
)


class FakeMsg:
    """rosidl 메시지 흉내 — 필드 메타데이터를 제공한다."""

    def __init__(self, **fields: Any) -> None:
        self._fields = fields
        for k, v in fields.items():
            setattr(self, k, v)

    def get_fields_and_field_types(self) -> dict[str, str]:
        return {k: 'unused' for k in self._fields}


def test_is_ros_message() -> None:
    assert is_ros_message(FakeMsg(a=1)) is True
    assert is_ros_message({'a': 1}) is False


def test_nested_message_becomes_nested_dict() -> None:
    msg = FakeMsg(header=FakeMsg(frame_id='panda_link0'), value=1.5)

    assert to_jsonable(msg) == {'header': {'frame_id': 'panda_link0'}, 'value': 1.5}


def test_nan_and_inf_become_null() -> None:
    """JSON 에는 NaN / Inf 리터럴이 없다."""
    msg = FakeMsg(a=math.nan, b=math.inf, c=-math.inf, d=1.0)

    assert to_jsonable(msg) == {'a': None, 'b': None, 'c': None, 'd': 1.0}


def test_large_int_becomes_string() -> None:
    """JS Number 정밀도를 넘는 정수는 문자열로 낸다."""
    safe = 2 ** 53 - 1
    msg = FakeMsg(small=42, edge=safe, big=safe + 1, negative=-(safe + 1))

    out = to_jsonable(msg)

    assert out['small'] == 42
    assert out['edge'] == safe
    assert out['big'] == str(safe + 1)
    assert out['negative'] == str(-(safe + 1))


def test_bool_is_not_treated_as_int() -> None:
    assert to_jsonable(FakeMsg(flag=True)) == {'flag': True}


def test_bytes_are_reduced_to_length() -> None:
    """이미지 등 바이트 배열은 상태 변수로 노출하지 않는다."""
    assert to_jsonable(FakeMsg(data=b'abcd')) == {'data': {'__bytes__': 4}}


def test_sequences_and_tolist() -> None:
    class FakeArray:
        def tolist(self) -> list[float]:
            return [1.0, math.nan]

    assert to_jsonable(FakeMsg(a=[1, 2], b=(3, 4))) == {'a': [1, 2], 'b': [3, 4]}
    assert to_jsonable(FakeArray()) == [1.0, None]


def test_extract_stamp_keeps_sec_nanosec() -> None:
    """ISO8601 로 바꾸지 않는다 — 정밀도 손실과 sim clock 문제 때문이다."""
    msg = FakeMsg(header=FakeMsg(stamp=FakeMsg(sec=1785664613, nanosec=206514897)))

    assert extract_stamp(msg) == {'sec': 1785664613, 'nanosec': 206514897}


def test_extract_stamp_returns_none_without_header() -> None:
    assert extract_stamp(FakeMsg(value=1)) is None


def test_project_joint_state_map() -> None:
    value = {'name': ['j1', 'j2'], 'position': [0.1, 0.2], 'velocity': [0.0, 0.0],
             'effort': [], 'header': {'frame_id': ''}}

    out = project_joint_state_map(value)

    assert out['position'] == {'j1': 0.1, 'j2': 0.2}
    assert out['velocity'] == {'j1': 0.0, 'j2': 0.0}
    # 빈 배열은 키 자체를 없앤다.
    assert 'effort' not in out
    # name 은 map 의 키가 되었으므로 별도 필드로 남기지 않는다.
    assert 'name' not in out
    assert out['header'] == {'frame_id': ''}


def test_project_rejects_empty_name() -> None:
    """재생 경로에서 name 이 비어 오는 사례가 실재한다 (설계서 5.7)."""
    with pytest.raises(SerializationError) as exc:
        project_joint_state_map({'name': [], 'position': [0.1]})
    assert 'name' in str(exc.value)


def test_project_rejects_length_mismatch() -> None:
    with pytest.raises(SerializationError) as exc:
        project_joint_state_map({'name': ['j1'], 'position': [0.1, 0.2]})
    assert 'position' in str(exc.value)


def test_project_rejects_scalar_name() -> None:
    """비어 있지 않은 스칼라 name 은 truthy 라 통과하고, name 필드만 소실됐었다."""
    with pytest.raises(SerializationError) as exc:
        project_joint_state_map({'name': 'cube_0', 'pose': {'x': 0.4}})
    assert 'list' in str(exc.value)


def test_project_rejects_unmapped_parallel_array() -> None:
    """화이트리스트 밖 병렬 배열은 map 이 되지 못한 채 name 만 사라져 복원 불가였다."""
    with pytest.raises(SerializationError) as exc:
        project_joint_state_map({'name': ['a', 'b'], 'temperature': [21.0, 22.5]})
    assert 'temperature' in str(exc.value)


def test_project_ignores_arrays_of_a_different_length() -> None:
    """길이가 다르면 애초에 대응 관계가 없으므로 실패시키지 않는다."""
    out = project_joint_state_map({'name': ['j1', 'j2'], 'position': [0.1, 0.2],
                                   'other': [1.0]})

    assert out['position'] == {'j1': 0.1, 'j2': 0.2}
    assert out['other'] == [1.0]


def test_apply_enums_maps_symbol_and_keeps_raw() -> None:
    enums = {'status': {4: 'SUCCEEDED', 6: 'ABORTED'}}

    out = apply_enums({'status': 4, 'position': 0.04}, enums)

    assert out['status'] == 'SUCCEEDED'
    assert out['status_value'] == 4
    assert out['position'] == 0.04


def test_apply_enums_leaves_unknown_values() -> None:
    out = apply_enums({'status': 99}, {'status': {4: 'SUCCEEDED'}})

    assert out['status'] == 99
    assert 'status_value' not in out


def test_apply_enums_noop_without_mapping() -> None:
    assert apply_enums({'a': 1}, {}) == {'a': 1}


# ----- scene_object_map (배열-of-구조체 → 이름 map) -----

def test_project_scene_object_map() -> None:
    value = {'header': {'frame_id': 'panda_link0'},
             'objects': [{'name': 'cube_0', 'type': 'box', 'dimensions': [0.05]},
                         {'name': 'ball_0', 'type': 'sphere', 'dimensions': [0.02]}]}

    out = project_scene_object_map(value)

    assert set(out['objects']) == {'cube_0', 'ball_0'}
    assert out['objects']['cube_0']['type'] == 'box'
    # name 은 map 의 키가 되었으므로 원소에 남기지 않는다 (joint_states 와 같은 처리).
    assert 'name' not in out['objects']['cube_0']
    # 나머지 필드는 그대로 통과한다.
    assert out['header'] == {'frame_id': 'panda_link0'}


def test_project_scene_object_map_accepts_an_empty_scene() -> None:
    """물체가 없는 것도 '씬이 비었다'는 유효한 상태다."""
    out = project_scene_object_map({'header': {}, 'objects': []})

    assert out['objects'] == {}


def test_project_scene_object_map_rejects_duplicate_names() -> None:
    """조용히 덮어쓰면 물체 하나가 데이터에서 사라진다."""
    with pytest.raises(SerializationError) as exc:
        project_scene_object_map(
            {'objects': [{'name': 'cube_0'}, {'name': 'cube_0'}]})
    assert 'duplicate' in str(exc.value)


def test_project_scene_object_map_rejects_missing_name() -> None:
    with pytest.raises(SerializationError) as exc:
        project_scene_object_map({'objects': [{'type': 'box'}]})
    assert 'name' in str(exc.value)


def test_project_scene_object_map_rejects_a_non_list_field() -> None:
    with pytest.raises(SerializationError) as exc:
        project_scene_object_map({'objects': {'cube_0': {}}})
    assert 'list' in str(exc.value)


def test_projection_table_covers_every_configurable_value() -> None:
    """설정 Literal 과 디스패치 표가 어긋나면 KeyError 로 죽는다."""
    from typing import get_args

    from robot_twin.config import VariableConfig
    from robot_twin.variables import PROJECTIONS

    field = VariableConfig.model_fields['projection']
    allowed = {v for arg in get_args(field.annotation) for v in get_args(arg)}

    assert allowed == set(PROJECTIONS)
