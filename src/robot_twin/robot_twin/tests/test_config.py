"""`robot_twin.config` 단위 테스트 — ROS 없이 동작한다."""

from __future__ import annotations

from typing import Any

import textwrap
from pathlib import Path

import pytest
from pydantic import ValidationError

from robot_twin.config import TwinConfig, load_config


MINIMAL: dict[str, Any] = {
    'twin': {'id': 'panda01'},
    'moveit': {'move_group_mode': 'jtc'}
}

# 배포 설정(`config/robot_twin_panda01.yaml`)의 그리퍼 연산과 같은 모양이다.
GRIPPER_BACKEND: dict[str, Any] = {'topic': '/gripper_control/gripper_cmds',
                                   'topic_type': 'rdfp_msgs/msg/GripperCommand',
                                   'result_variable': 'gripper_last_command_result'}
GRIPPER_LABELS: list = ['open', 'close']


def _gripper_op(target_schema: dict[str, Any]) -> dict[str, Any]:
    return {'name': 'move_gripper_to_target', 'kind': 'sync', 'resource': 'gripper',
            'backend': {**GRIPPER_BACKEND, 'labels': GRIPPER_LABELS},
            'inputs_schema': {'type': 'object', 'required': ['target'],
                              'properties': {'target': target_schema}}}


def _target_schema(cfg: TwinConfig) -> dict[str, Any]:
    return cfg.operation('move_gripper_to_target').inputs_schema['properties']['target']


def test_minimal_config_uses_defaults() -> None:
    cfg = TwinConfig.model_validate(MINIMAL)

    assert cfg.twin.id == 'panda01'
    assert cfg.http.host == '0.0.0.0'
    assert cfg.http.port == 8801
    assert cfg.http.tls is False
    assert cfg.sessions.retention_sec == 900.0
    assert cfg.sessions.max_sessions == 200


def test_move_group_mode_auto_is_rejected() -> None:
    """'auto' 는 토픽 그래프 lookup 이라 조용히 오판한다 (설계서 2.6)."""
    raw = {**MINIMAL, 'moveit': {'move_group_mode': 'auto'}}

    with pytest.raises(ValidationError) as exc:
        TwinConfig.model_validate(raw)
    assert 'auto' in str(exc.value)


def test_move_group_mode_is_required() -> None:
    with pytest.raises(ValidationError):
        TwinConfig.model_validate({'twin': {'id': 'panda01'}, 'moveit': {}})


def test_tls_true_is_rejected() -> None:
    """설정만 켜고 평문으로 뜨는 상황을 막는다."""
    raw = {**MINIMAL, 'http': {'tls': True}}

    with pytest.raises(ValidationError):
        TwinConfig.model_validate(raw)


def test_unknown_key_is_rejected() -> None:
    """extra='forbid' 로 오타를 조용히 넘기지 않는다."""
    raw = {**MINIMAL, 'htp': {'port': 9000}}

    with pytest.raises(ValidationError):
        TwinConfig.model_validate(raw)


def test_topic_source_requires_topic_and_msg() -> None:
    raw = {**MINIMAL, 'variables': [{'name': 'js', 'source': {'type': 'topic'}}]}

    with pytest.raises(ValidationError) as exc:
        TwinConfig.model_validate(raw)
    assert 'topic' in str(exc.value)


def test_tf_source_requires_frames() -> None:
    raw = {**MINIMAL, 'variables': [{'name': 'ee', 'source': {'type': 'tf'}}]}

    with pytest.raises(ValidationError):
        TwinConfig.model_validate(raw)


def test_duplicate_variable_names_are_rejected() -> None:
    src = {'type': 'topic', 'topic': '/joint_states', 'msg': 'sensor_msgs/msg/JointState'}
    raw = {**MINIMAL, 'variables': [{'name': 'js', 'source': src},
                                    {'name': 'js', 'source': src}]}

    with pytest.raises(ValidationError) as exc:
        TwinConfig.model_validate(raw)
    assert 'duplicate' in str(exc.value)


def test_derived_source_must_reference_known_variable() -> None:
    raw = {**MINIMAL, 'variables': [
        {'name': 'grip', 'source': {'type': 'derived', 'source_variable': 'nope',
                                    'extract': 'panda_finger_joint1'}}
    ]}

    with pytest.raises(ValidationError) as exc:
        TwinConfig.model_validate(raw)
    assert 'unknown variable' in str(exc.value)


def test_variable_name_must_be_url_safe() -> None:
    src = {'type': 'topic', 'topic': '/t', 'msg': 'std_msgs/msg/String'}
    raw = {**MINIMAL, 'variables': [{'name': 'a,b', 'source': src}]}

    with pytest.raises(ValidationError):
        TwinConfig.model_validate(raw)


def test_lookup_helpers() -> None:
    src = {'type': 'topic', 'topic': '/joint_states', 'msg': 'sensor_msgs/msg/JointState'}
    raw = {**MINIMAL,
           'variables': [{'name': 'js', 'source': src}],
           'operations': [_gripper_op({'type': 'string'})]}
    cfg = TwinConfig.model_validate(raw)

    assert cfg.variable('js') is not None
    assert cfg.variable('missing') is None
    assert cfg.operation('move_gripper_to_target').resource == 'gripper'
    assert cfg.operation('missing') is None


def test_target_enum_is_derived_from_backend_labels() -> None:
    """목표 이름의 단일 출처는 backend.labels 다.

    카탈로그가 backend 를 노출하지 않으므로 enum 이 클라이언트의 유일한 발견
    경로다. 설정에 두 번 적지 않도록 여기서 채운다.
    """
    cfg = TwinConfig.model_validate({**MINIMAL, 'operations': [_gripper_op({'type': 'string'})]})

    assert _target_schema(cfg) == {'type': 'string', 'enum': ['close', 'open']}


def test_deriving_the_enum_does_not_mutate_the_input() -> None:
    """호출자의 dict 를 바꾸지 않는다 — 모듈 상수를 공유하는 호출부가 있다."""
    raw = _gripper_op({'type': 'string'})

    TwinConfig.model_validate({**MINIMAL, 'operations': [raw]})

    assert raw['inputs_schema']['properties']['target'] == {'type': 'string'}


def test_matching_explicit_target_enum_is_accepted() -> None:
    """이미 적혀 있고 일치하면 그대로 둔다 — 순서는 보지 않는다."""
    op = _gripper_op({'type': 'string', 'enum': ['open', 'close']})
    cfg = TwinConfig.model_validate({**MINIMAL, 'operations': [op]})

    assert _target_schema(cfg)['enum'] == ['open', 'close']


@pytest.mark.parametrize('enum', [
    ['open'],                    # 빠졌다
    ['open', 'close', 'half'],   # 없는 목표가 있다
    'open',                      # 리스트가 아니다
])
def test_mismatched_target_enum_is_rejected(enum: Any) -> None:
    """드리프트를 기동 시점에 실패시킨다 (호출 시점이 아니라)."""
    op = _gripper_op({'type': 'string', 'enum': enum})

    with pytest.raises(ValidationError) as exc:
        TwinConfig.model_validate({**MINIMAL, 'operations': [op]})
    assert 'backend.labels' in str(exc.value)


def test_schema_without_target_property_is_left_alone() -> None:
    """`target` 입력이 없는 연산의 스키마는 건드리지 않는다."""
    op = {'name': 'other', 'kind': 'sync', 'backend': {'labels': GRIPPER_LABELS},
          'inputs_schema': {'type': 'object', 'properties': {'width': {'type': 'number'}}}}
    cfg = TwinConfig.model_validate({**MINIMAL, 'operations': [op]})

    assert cfg.operation('other').inputs_schema['properties'] == {'width': {'type': 'number'}}


def test_operation_without_labels_keeps_its_schema() -> None:
    op = {'name': 'move_to_named_target', 'kind': 'async',
          'backend': {'method': 'move_to_named_target_async'},
          'inputs_schema': {'type': 'object', 'properties': {'target': {'type': 'string'}}}}
    cfg = TwinConfig.model_validate({**MINIMAL, 'operations': [op]})

    assert cfg.operation('move_to_named_target').inputs_schema['properties']['target'] == {
        'type': 'string'}


def test_shipped_config_derives_gripper_labels() -> None:
    """배포되는 설정 파일 자체를 한 번 로드한다.

    단위 테스트가 전부 인라인 dict 라 실제 YAML 의 오타를 잡을 그물이 없었다.
    """
    path = Path(__file__).resolve().parents[3] / 'config' / 'robot_twin_panda01.yaml'
    if not path.exists():
        pytest.skip(f'shipped config not found: {path}')

    cfg = load_config(path)
    op = cfg.operation('move_gripper_to_target')

    assert sorted(op.backend['labels']) == ['close', 'grasp', 'open']
    assert op.inputs_schema['properties']['target']['enum'] == ['close', 'grasp', 'open']
    # **숫자는 설정에 없다.** 그리퍼 종속이라 gripper_control_node 가 갖는다.
    assert not any(isinstance(v, dict) for v in op.backend['labels'])


def test_load_config_from_file(tmp_path) -> None:
    path = tmp_path / 'twin.yaml'
    path.write_text(textwrap.dedent("""
        twin:
          id: panda01
        http:
          port: 8802
        moveit:
          move_group_mode: jgpc
    """).strip(), encoding='utf-8')

    cfg = load_config(path)

    assert cfg.http.port == 8802
    assert cfg.moveit.move_group_mode == 'jgpc'


def test_load_config_missing_file(tmp_path) -> None:
    with pytest.raises(FileNotFoundError):
        load_config(tmp_path / 'nope.yaml')


def test_load_config_rejects_non_mapping(tmp_path) -> None:
    path = tmp_path / 'twin.yaml'
    path.write_text('- just\n- a\n- list\n', encoding='utf-8')

    with pytest.raises(ValueError):
        load_config(path)
