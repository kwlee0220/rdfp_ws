#!/usr/bin/env python3

"""`Backend` — 프로파일 값과 백엔드 고유 코드를 잇는 층.

**백엔드마다 갈리는 것이 두 가지다** — 값(`config/backends/*.yaml`)과 객체 생성 코드
(이 클래스들). 값만 다른 것은 YAML 에, 값으로 표현할 수 없는 것만 코드에 둔다.

여기서 지키는 성질:

1. `factory` 가 가리키는 클래스가 실제로 만들어진다.
2. servo 파라미터가 **launch 헬퍼의 규약**(중첩 dict, 접두사 없음)과 맞는다.
3. `null` 블록이 "없다" 로 다뤄진다.
4. 경로가 패키지 share 기준으로 풀린다.
"""

from __future__ import annotations

import os

import pytest

from robot_control.backends import Backend, get_backend
from robot_control.backends.isaac import COMMANDED_JOINT_STATE_TOPIC, IsaacBackend


# ----- factory 선택 ----------------------------------------------------------

def test_factory_key_selects_the_class():
    assert isinstance(get_backend('isaac'), IsaacBackend)


def test_backends_without_a_factory_use_the_base_class():
    """고유 코드가 없으면 값만 읽는 기본 구현이다 — 빈 하위 클래스를 만들지 않는다.

    펑션베이가 여기 있는 것이 중요하다. 한때 `FunctionbayBackend` 가 있었지만 servo 출력
    형식은 `servo.command_out_type` 이, 딸려 오는 `publish_joint_*` 는
    `_float64_multi_array_rule` 이 갖게 되어 남은 코드가 없다.
    """
    assert type(get_backend('mock')) is Backend
    assert type(get_backend('mock_jgpc')) is Backend
    assert type(get_backend('functionbay')) is Backend


def test_unknown_backend_is_rejected():
    with pytest.raises(ValueError, match='backend'):
        get_backend('issac')


def test_bad_factory_spec_is_rejected():
    """`module:Class` 가 아니면 거부한다 — 오타가 import 시점의 알 수 없는 오류로
    번지지 않게."""
    from robot_control.backends import base

    base.BACKEND_PROFILES['_bad'] = {'factory': 'robot_control.backends.isaac'}
    try:
        with pytest.raises(ValueError, match="'module:Class'"):
            get_backend('_bad')
    finally:
        base.BACKEND_PROFILES.pop('_bad', None)


# ----- servo 파라미터 ---------------------------------------------------------

def test_servo_keys_carry_no_prefix():
    """launch 헬퍼가 `moveit_servo` 안쪽에 합치므로 접두사를 붙이면 **이중이 되어
    조용히 안 먹는다.**"""
    for name in ('isaac', 'functionbay', 'mock_jgpc'):
        for key in get_backend(name).servo_parameters():
            assert not key.startswith('moveit_servo'), f'{name}: {key}'
            assert '.' not in key, f'{name}: {key} — 중첩 dict 로 쓴다'


def test_isaac_maps_joint_source_to_a_topic():
    """**값 하나가 파라미터 하나로 안 간다.** 프로파일은 '무엇을 기준으로 삼는가',
    servo 는 '어느 토픽을 읽는가' — 그 사이 변환이 코드다.
    """
    params = get_backend('isaac').servo_parameters()

    assert params['joint_topic'] == COMMANDED_JOINT_STATE_TOPIC
    assert params['scale'] == {'linear': 0.4}


def test_functionbay_must_disable_velocities():
    """**선택이 아니다.** Float64MultiArray 출력에서 위치와 속도를 함께 내면 servo 의
    파라미터 검증이 실패해 노드가 아예 안 뜬다.
    """
    params = get_backend('functionbay').servo_parameters()

    assert params['command_out_type'] == 'std_msgs/Float64MultiArray'
    assert params['publish_joint_velocities'] is False
    assert params['command_out_topic'] == '/servo_node/commands'


def test_mock_overrides_nothing():
    assert get_backend('mock').servo_parameters() == {}


def test_apply_merges_into_the_nested_dict_without_mutating():
    original = {'moveit_servo': {'scale': {'linear': 0.9, 'rotational': 0.8}, 'keep': 1}}

    merged = get_backend('isaac').apply_servo_parameters(original)

    assert merged['moveit_servo']['scale']['linear'] == 0.4, '백엔드 값이 이긴다'
    assert merged['moveit_servo']['scale']['rotational'] == 0.8, \
        '형제 값이 사라지면 안 된다 — 통째로 갈아치우면 회전 배율을 잃는다'
    assert merged['moveit_servo']['keep'] == 1, '나머지는 남는다'
    assert original['moveit_servo']['scale']['linear'] == 0.9, '원본을 바꾸지 않는다'


# ----- null 블록 --------------------------------------------------------------

def test_missing_block_reads_as_absent():
    fb = get_backend('functionbay')

    assert fb.uses_ros2_control is False
    assert fb.hardware_type is None
    assert fb.controllers_file() is None
    # 없는 블록의 예는 `camera` 가 아니다 — 펑션베이에도 카메라는 있다.
    assert fb.value('ros2_control', 'controllers_file', 'fallback') == 'fallback'


def test_ros2_control_backends_report_their_hardware_type():
    assert get_backend('isaac').hardware_type == 'isaac'
    assert get_backend('mock').hardware_type == 'mock_components'
    assert get_backend('isaac').uses_ros2_control is True


# ----- 경로 -------------------------------------------------------------------

def test_paths_resolve_under_the_package_share():
    path = get_backend('isaac').controllers_file()

    assert path is not None and os.path.isabs(path)
    assert os.path.isfile(path), path
    assert path.endswith('config/panda_isaac_ros2_controllers.yaml')


def test_missing_path_stays_none():
    """`None` 은 "호출자가 기본값을 쓴다" 는 뜻이다 — mock 은 moveit_resources 것이다."""
    assert get_backend('mock').controllers_file() is None
    assert get_backend('mock').scene_file() is None


def test_scene_file_resolves():
    assert os.path.isfile(get_backend('isaac').scene_file())


# ----- use_sim_time -----------------------------------------------------------

def test_use_sim_time_matches_the_backend():
    """빠뜨리면 스탬프가 벽시계로 찍혀 에피소드 경계와 어긋난다."""
    assert get_backend('isaac').use_sim_time is True
    assert get_backend('mock').use_sim_time is False


# ----- Float64MultiArray 규칙 -------------------------------------------------

def test_float64_output_forces_the_publish_flags():
    """**백엔드 특성이 아니라 servo 의 제약이다.**

    그 형식에서 positions 와 velocities 를 함께 발행하도록 두면 servo 의 파라미터
    검증이 실패해 **노드가 아예 안 뜬다.** 백엔드마다 되적으면 새 백엔드에서 빠뜨리고,
    증상은 "servo 가 안 뜬다" 뿐이라 원인을 안 가리킨다.
    """
    for name in ('mock_jgpc', 'functionbay'):
        params = get_backend(name).servo_parameters()
        assert params['command_out_type'] == 'std_msgs/Float64MultiArray'
        assert params['publish_joint_positions'] is True
        assert params['publish_joint_velocities'] is False
        assert params['publish_joint_accelerations'] is False


def test_the_rule_does_not_apply_to_other_output_types():
    """기본 출력(JointTrajectory)에는 이 제약이 없다 — 걸면 오히려 이상해진다."""
    params = get_backend('isaac').servo_parameters()

    assert 'publish_joint_velocities' not in params


def test_command_out_topic_comes_from_the_profile():
    assert get_backend('mock_jgpc').servo_parameters()['command_out_topic'] == \
        '/panda_arm_controller/commands'
    assert get_backend('functionbay').servo_parameters()['command_out_topic'] == \
        '/servo_node/commands'


# ----- arm_command 접근자 ------------------------------------------------------

def test_arm_command_is_reachable_like_other_blocks():
    """`arm_command` 는 로더가 **최상위로 평탄화**한다 — 다른 블록과 다르다.

    그 비대칭을 모르고 `value('arm_command', 'topic')` 으로 읽으면 조용히 `None` 이
    나오고, launch 는 remap 대상이 없어 기동 중에 죽는다. 실제로 그렇게 깨졌다.
    """
    fb = get_backend('functionbay')

    assert fb.arm_command_mode == 'jgpc'
    assert fb.arm_command_topic == '/input/panda_joint'
    assert fb.arm_command_format == 'joint_state'
    assert fb.arm_command_joint_names == [f'panda_joint{i}' for i in range(1, 8)]
    # 블록으로는 안 잡힌다 — 그래서 접근자가 있다.
    assert fb.block('arm_command') is None


def test_jtc_backends_have_no_arm_command_channel():
    isaac = get_backend('isaac')

    assert isaac.arm_command_mode == 'jtc'
    assert isaac.arm_command_topic is None
    assert isaac.arm_command_joint_names == []


def test_joint_names_are_copied():
    """호출자가 바꿔도 프로파일이 오염되면 안 된다."""
    names = get_backend('functionbay').arm_command_joint_names
    names.append('panda_joint8')

    assert len(get_backend('functionbay').arm_command_joint_names) == 7


# ----- motion.velocity_scaling — 전역 기본 속도 -------------------------------

def test_functionbay_declares_a_global_velocity_scaling():
    """0.4 는 실측으로 고른 값이다 (프로파일 주석 참고)."""
    assert get_backend('functionbay').velocity_scaling == 0.4


def test_backends_without_the_block_have_no_opinion():
    """`None` 은 "이 백엔드에는 없다" — 호출자가 코드 기본값(1.0)을 쓴다.

    여기서 `0.0` 이나 `1.0` 을 돌려주면 **"프로파일이 정했다"와 "안 정했다"를 구별할 수
    없어** 팩토리가 인자 우선순위를 지킬 수 없다.
    """
    for name in ('mock', 'mock_jgpc', 'isaac'):
        assert get_backend(name).velocity_scaling is None, name


def test_velocity_scaling_is_a_float():
    """YAML 이 `0.4` 를 문자열로 싣더라도 숫자로 나와야 한다."""
    value = get_backend('functionbay').velocity_scaling
    assert isinstance(value, float) and 0.0 < value <= 1.0
