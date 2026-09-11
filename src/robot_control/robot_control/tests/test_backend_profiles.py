#!/usr/bin/env python3

"""백엔드 프로파일 — 파일에서 읽는 표.

**백엔드 하나 = 파일 하나**이고, 파일 이름이 곧 `backend` 이름이다. 여기서 지키는
성질은 넷이다.

1. 파일 내용이 실제 백엔드 구성과 맞는다.
2. **`rclpy` 없이 읽힌다** — 설정만 보는 경로(트윈 `config.py`)가 ROS 를 요구하면 안 된다.
3. 모르는 키를 조용히 무시하지 않는다 — "설정했는데 안 먹는다" 가 된다.
4. 우선순위가 **명시 인자 > 프로파일 > 전역 기본값** 이다.
"""

from __future__ import annotations

import os
import textwrap

import pytest

from robot_control import backend_profiles as bp


# ----- 파일 내용 -------------------------------------------------------------

def test_isaac_is_jtc_and_carries_no_command_channel():
    """**Isaac 은 JTC 다** (2026-09-05).

    옛 bridge 설정(`jgpc` + `/isaac/arm_command`)이 남아 있으면 컨트롤러를 **우회해**
    같은 채널에 끼어들어 서로 싸운다 — 지금 그 토픽은 `TopicBasedSystem` 이 발행한다.
    """
    p = bp.BACKEND_PROFILES['isaac']

    assert p['arm_command_mode'] == 'jtc'
    for stale in ('arm_command_topic', 'arm_command_format', 'arm_command_joint_names'):
        assert stale not in p, f'{stale} 은 bridge 시절 잔재다'


def test_functionbay_carries_all_four():
    """넷 중 하나만 빠져도 실패 방식이 다르고, 전부 조용하다.

    mode 를 빠뜨리면 계획만 되고 실행이 안 되며, joint_names 를 빠뜨리면 계획은
    `fraction 1.0` 으로 되고 스트리밍에서 파라미터 조회를 기다리다 죽는다.
    """
    p = bp.BACKEND_PROFILES['functionbay']

    assert p['arm_command_mode'] == 'jgpc'
    assert p['arm_command_topic'] == '/input/panda_joint'
    assert p['arm_command_format'] == 'joint_state'
    assert p['arm_command_joint_names'] == [f'panda_joint{i}' for i in range(1, 8)]


def test_mock_stacks_are_two_distinct_profiles():
    """mock 계열은 스택이 둘이라 **프로파일도 둘**이다.

    하나로 묶고 `auto` 로 두면 판별이 디스커버리에 좌우되고, 그 오판은 조용하다.
    """
    assert bp.BACKEND_PROFILES['mock']['arm_command_mode'] == 'jtc'
    assert bp.BACKEND_PROFILES['mock_jgpc']['arm_command_mode'] == 'jgpc'


def test_every_expected_backend_has_a_file():
    assert set(bp.backends()) >= {'mock', 'mock_jgpc', 'isaac', 'functionbay'}


def test_no_profile_defers_to_runtime_detection():
    """**`auto` 라는 백엔드는 없다.**

    프로파일 표가 있는 이유가 런타임 판별의 조용한 오판을 없애는 것인데, 그 표 안에
    판별로 되돌아가는 항목을 두면 앞뒤가 안 맞는다. 붙어 있는 스택에 맞추고 싶으면
    `backend` 없이 `mode='auto'` 를 쓴다 — 그것은 저수준 경로다.
    """
    assert 'auto' not in bp.backends()
    for name, profile in bp.BACKEND_PROFILES.items():
        assert profile.get('arm_command_mode') in ('jtc', 'jgpc'), \
            f'{name}: 확정 모드여야 한다'


# ----- ROS 없이 읽힌다 --------------------------------------------------------

def test_module_does_not_import_rclpy():
    """설정만 읽는 경로가 ROS 를 요구하면 안 된다.

    트윈의 `config.py` 가 `backend:` 를 펼칠 때 이 모듈을 부른다 — `rclpy` 를 끌어오면
    ROS 없는 환경에서 **설정 파일조차 못 읽는다**. 실제로 그렇게 깨진 적이 있다.
    """
    import ast

    source = ast.parse(open(bp.__file__, encoding='utf-8').read())
    imported = set()
    for node in ast.walk(source):
        if isinstance(node, ast.Import):
            imported |= {a.name.split('.')[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split('.')[0])
    assert 'rclpy' not in imported


# ----- 파싱 ------------------------------------------------------------------

def _write(tmp_path, name, text):
    path = tmp_path / f'{name}.yaml'
    path.write_text(textwrap.dedent(text), encoding='utf-8')
    return str(tmp_path)


def test_unknown_top_level_key_is_rejected(tmp_path):
    """조용히 무시하면 "설정했는데 안 먹는다" 가 된다."""
    d = _write(tmp_path, 'x', """
        name: x
        arm_commnad:          # 오타
          mode: jtc
    """)
    with pytest.raises(ValueError, match='unknown key'):
        bp.load_profiles(d)


def test_unknown_arm_command_key_is_rejected(tmp_path):
    d = _write(tmp_path, 'x', """
        name: x
        arm_command:
          moed: jtc           # 오타
    """)
    with pytest.raises(ValueError, match='unknown arm_command key'):
        bp.load_profiles(d)


def test_name_must_match_the_file_name(tmp_path):
    """참조는 **파일 이름**으로 한다 — 어긋나면 문서와 실제가 갈린다."""
    d = _write(tmp_path, 'isaac', """
        name: isaac_sim
        arm_command: { mode: jtc }
    """)
    with pytest.raises(ValueError, match='does not match the file name'):
        bp.load_profiles(d)


def test_name_may_be_omitted(tmp_path):
    """파일 이름이 정본이므로 `name` 은 없어도 된다."""
    d = _write(tmp_path, 'x', 'arm_command: { mode: jtc }\n')

    assert bp.load_profiles(d)['x'] == {'arm_command_mode': 'jtc'}


def test_empty_directory_is_an_error(tmp_path):
    """빈 표를 조용히 돌려주면 모든 backend 참조가 "모르는 이름" 으로 죽는다."""
    with pytest.raises(FileNotFoundError, match='no backend profiles'):
        bp.load_profiles(str(tmp_path))


def test_non_yaml_files_are_ignored(tmp_path):
    """디렉터리에 README.md 가 함께 있다."""
    (tmp_path / 'README.md').write_text('설명', encoding='utf-8')
    d = _write(tmp_path, 'x', 'arm_command: { mode: jtc }\n')

    assert set(bp.load_profiles(d)) == {'x'}


# ----- 우선순위 ---------------------------------------------------------------

def test_profile_fills_every_key():
    assert set(bp.resolve_backend_channel('functionbay')) == set(
        ['arm_command_mode', 'arm_command_topic', 'arm_command_format',
         'arm_command_joint_names'])


def test_explicit_argument_beats_the_profile():
    assert bp.resolve_backend_channel(
        'isaac', arm_command_mode='jgpc')['arm_command_mode'] == 'jgpc'


@pytest.mark.parametrize('empty', [None, '', '   '])
def test_empty_override_means_unset(empty):
    assert bp.resolve_backend_channel(
        'isaac', arm_command_mode=empty)['arm_command_mode'] == 'jtc'


def test_unknown_backend_is_rejected():
    with pytest.raises(ValueError, match='backend'):
        bp.resolve_backend_channel('issac')


# ----- 파일 위치 --------------------------------------------------------------

def test_profiles_dir_prefers_the_installed_copy():
    """배포된 값이 실제로 쓰이는 값이어야 한다.

    그 대가로 소스만 고치고 빌드를 빠뜨리면 옛 값이 읽힌다 — README 의 주의다.
    """
    assert os.path.isdir(bp.profiles_dir())
