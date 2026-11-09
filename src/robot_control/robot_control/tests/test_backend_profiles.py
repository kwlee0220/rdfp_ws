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

import json
import os
import textwrap

import pytest
import yaml

from robot_control import backend_profiles as bp


# ----- 파일 내용 -------------------------------------------------------------

def test_isaac_is_jtc_and_carries_no_command_channel():
    """**Isaac 은 JTC 다** (2026-09-05).

    옛 bridge 설정(`jgpc` + `/isaac/arm_command`)이 남아 있으면 컨트롤러를 **우회해**
    같은 채널에 끼어들어 서로 싸운다 — 지금 그 토픽은 `TopicBasedSystem` 이 발행한다.
    """
    p = bp.BACKEND_PROFILES['panda_isaac']

    assert p['arm_command_mode'] == 'jtc'
    for stale in ('arm_command_topic', 'arm_command_format', 'arm_command_joint_names'):
        assert stale not in p, f'{stale} 은 bridge 시절 잔재다'


def test_functionbay_carries_all_four():
    """넷 중 하나만 빠져도 실패 방식이 다르고, 전부 조용하다.

    mode 를 빠뜨리면 계획만 되고 실행이 안 되며, joint_names 를 빠뜨리면 계획은
    `fraction 1.0` 으로 되고 스트리밍에서 파라미터 조회를 기다리다 죽는다.
    """
    p = bp.BACKEND_PROFILES['panda_functionbay']

    assert p['arm_command_mode'] == 'jgpc'
    assert p['arm_command_topic'] == '/input/panda_joint'
    assert p['arm_command_format'] == 'joint_state'
    assert p['arm_command_joint_names'] == [f'panda_joint{i}' for i in range(1, 8)]


def test_mock_stacks_are_two_distinct_profiles():
    """mock 계열은 스택이 둘이라 **프로파일도 둘**이다.

    하나로 묶고 `auto` 로 두면 판별이 디스커버리에 좌우되고, 그 오판은 조용하다.
    """
    assert bp.BACKEND_PROFILES['panda_mock']['arm_command_mode'] == 'jtc'
    assert bp.BACKEND_PROFILES['panda_mock_jgpc']['arm_command_mode'] == 'jgpc'


def test_every_expected_backend_has_a_file():
    assert set(bp.backends()) >= {'panda_mock', 'panda_mock_jgpc', 'panda_isaac',
                                  'panda_functionbay'}


def test_no_profile_defers_to_runtime_detection():
    """**모든 프로파일이 확정 모드를 갖는다.**

    프로파일 표가 있는 이유가 런타임 판별의 조용한 오판을 없애는 것인데, 그 표 안에
    판별로 되돌아가는 항목을 두면 앞뒤가 안 맞는다. 프로파일에 없는 스택이면
    `backend` 없이 `mode='jtc'`/`'jgpc'` 를 명시한다 — 그것은 저수준 경로다.
    """
    assert 'auto' not in bp.backends()
    for name, profile in bp.BACKEND_PROFILES.items():
        assert profile.get('arm_command_mode') in ('jtc', 'jgpc'), \
            f'{name}: 확정 모드여야 한다'


def test_arm_command_mode_has_no_fallback():
    """**빠뜨린 모드를 짐작하지 않는다.**

    예전 폴백은 `'auto'` 였고, 그것이 런타임 판별로 되돌아가 펑션베이를 JTC 로
    오판하는 길을 열어 두고 있었다 — 그 스택엔 액션 서버가 없어 **계획은 되고 실행만
    조용히 안 된다.** 이제 `None` 이고, 팩토리가 그 사실을 말한다.
    """
    assert bp._FALLBACKS['arm_command_mode'] is None


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
    d = _write(tmp_path, 'panda_isaac', """
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
    assert set(bp.resolve_backend_channel('panda_functionbay')) == set(
        ['arm_command_mode', 'arm_command_topic', 'arm_command_format',
         'arm_command_joint_names'])


def test_explicit_argument_beats_the_profile():
    assert bp.resolve_backend_channel(
        'panda_isaac', arm_command_mode='jgpc')['arm_command_mode'] == 'jgpc'


@pytest.mark.parametrize('empty', [None, '', '   '])
def test_empty_override_means_unset(empty):
    assert bp.resolve_backend_channel(
        'panda_isaac', arm_command_mode=empty)['arm_command_mode'] == 'jtc'


def test_unknown_backend_is_rejected():
    with pytest.raises(ValueError, match='backend'):
        bp.resolve_backend_channel('issac')


# ----- 파일 위치 --------------------------------------------------------------

def test_profiles_dir_prefers_the_installed_copy():
    """배포된 값이 실제로 쓰이는 값이어야 한다.

    그 대가로 소스만 고치고 빌드를 빠뜨리면 옛 값이 읽힌다 — README 의 주의다.
    """
    assert os.path.isdir(bp.profiles_dir())


# ----- 시뮬레이터 씬 (`sim_scene` · `sim_scenes`) --------------------------------

_TWO_SCENES = """
    name: x
    sim_scene: {active}
    sim_scenes:
      a:
        solver: SolverA
        gripper:
          profile_file: config/a.json
        motion:
          gravity_compensation: null
      b:
        solver: SolverB
        gripper:
          profile_file: config/b.json
        motion:
          gravity_compensation:
            kp: 20000.0
    motion:
      velocity_scaling: 0.4
"""


def test_the_chosen_scene_is_laid_over_the_common_blocks(tmp_path):
    """**바꾸는 곳은 `sim_scene` 한 줄이다** — 씬 값이 공통 블록 위에 키 단위로 얹힌다."""
    d = _write(tmp_path, 'x', _TWO_SCENES.format(active='b'))
    p = bp.load_profiles(d)['x']

    assert p['solver'] == 'SolverB'
    assert p['sim_scene'] == 'b'
    assert p['gripper'] == {'profile_file': 'config/b.json'}
    assert p['motion'] == {'velocity_scaling': 0.4, 'gravity_compensation': {'kp': 20000.0}}


def test_null_means_this_scene_has_none(tmp_path):
    d = _write(tmp_path, 'x', _TWO_SCENES.format(active='a'))
    p = bp.load_profiles(d)['x']

    assert p['solver'] == 'SolverA'
    assert not p['motion']['gravity_compensation']


def test_an_unknown_sim_scene_is_rejected(tmp_path):
    d = _write(tmp_path, 'x', _TWO_SCENES.format(active='c'))
    with pytest.raises(ValueError, match="sim_scene 'c' is not one of"):
        bp.load_profiles(d)


def test_every_scene_must_set_the_same_keys(tmp_path):
    """한 씬에만 있는 키는 다른 씬으로 바꿨을 때 조용히 빠진다 — '없다' 는 `null` 로 적는다."""
    d = _write(tmp_path, 'x', """
        name: x
        sim_scene: a
        sim_scenes:
          a:
            solver: SolverA
            motion:
              gravity_compensation: null
          b:
            solver: SolverB
    """)
    with pytest.raises(ValueError, match='same keys'):
        bp.load_profiles(d)


def test_a_key_in_both_the_common_block_and_a_scene_is_rejected(tmp_path):
    """두 곳에 적으면 어느 쪽이 이기는지 헷갈린다."""
    d = _write(tmp_path, 'x', """
        name: x
        sim_scene: a
        sim_scenes:
          a:
            gripper:
              profile_file: config/a.json
        gripper:
          profile_file: config/common.json
    """)
    with pytest.raises(ValueError, match='both in the common block'):
        bp.load_profiles(d)


def test_solver_in_both_places_is_rejected(tmp_path):
    d = _write(tmp_path, 'x', """
        name: x
        solver: Top
        sim_scene: a
        sim_scenes:
          a:
            solver: SolverA
    """)
    with pytest.raises(ValueError, match='solver is set both'):
        bp.load_profiles(d)


def test_an_unknown_scene_key_is_rejected(tmp_path):
    d = _write(tmp_path, 'x', """
        name: x
        sim_scene: a
        sim_scenes:
          a:
            servo:
              linear_scale: 0.8
    """)
    with pytest.raises(ValueError, match='unknown sim_scenes.a key'):
        bp.load_profiles(d)


def test_solver_reaches_the_flat_profile():
    """**회귀 시험.** 2026-10-20 까지 로더가 `solver` 를 싣지 않아, 그리퍼 규약 파일과의
    대조(`gripper.profile._verify_solver`)가 늘 `None` 을 보고 **한 번도 작동하지 않았다.**"""
    p = bp.BACKEND_PROFILES['panda_functionbay']

    assert p.get('solver'), 'solver 가 프로파일에 안 실렸다 — 규약 대조가 건너뛰어진다'
    assert p.get('sim_scene'), 'sim_scene 이 프로파일에 안 실렸다 — 기동 시 씬 대조가 안 된다'


def test_every_functionbay_scene_matches_its_gripper_convention():
    """**활성 씬만이 아니라 모든 씬**이 자기 그리퍼 규약 파일과 맞아야 한다.

    활성이 아닌 씬은 로드 시점 대조를 안 거치므로, 낡아도 그 씬으로 바꾸는 날에야 드러난다.
    """
    config = os.path.dirname(bp.profiles_dir())
    path = os.path.join(bp.profiles_dir(), 'panda_functionbay.yaml')
    with open(path, encoding='utf-8') as handle:
        raw = yaml.safe_load(handle)

    assert raw['sim_scene'] in raw['sim_scenes']
    for name, scene in raw['sim_scenes'].items():
        path = os.path.join(config, os.path.basename(scene['gripper']['profile_file']))
        with open(path, encoding='utf-8') as handle:
            convention = json.load(handle)
        assert convention['solver'] == scene['solver'], name
        assert convention['scene'] == name, name

        gravity = scene['motion']['gravity_compensation']
        if gravity:
            assert gravity['kp'] > 0, name
            assert gravity['urdf_package'] and gravity['urdf_relative_path'], name


# ---------- 이름은 `<로봇>_<환경>` — 옛 이름은 한동안 받는다 (2026-10-21) ----------
#
# 프로파일 값은 환경만이 아니라 로봇에도 묶여 있다(관절 이름 · 컨트롤러 · 프레임 · 그리퍼).
# 환경 이름(`isaac`)만 쓰면 두 번째 로봇이 올 때 한 이름이 두 조합을 가리킨다.

def test_backends_lists_only_the_robot_qualified_names():
    names = set(bp.backends())
    assert {'panda_mock', 'panda_mock_jgpc', 'panda_isaac', 'panda_functionbay'} <= names
    assert not names & set(bp.LEGACY_BACKEND_NAMES), '순회는 옛 이름을 내지 않는다'


def test_every_legacy_name_points_at_an_existing_profile():
    for old, new in bp.LEGACY_BACKEND_NAMES.items():
        assert new in bp.backends(), (old, new)


@pytest.mark.parametrize('old, new', sorted(bp.LEGACY_BACKEND_NAMES.items()))
def test_lookup_accepts_a_legacy_name_silently(old, new):
    """조회는 조용히 받는다 — 경고는 사용자 입력을 받는 입구가 한 번만 낸다."""
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter('error')
        assert old in bp.BACKEND_PROFILES
        assert bp.BACKEND_PROFILES[old] is bp.BACKEND_PROFILES[new]
        assert bp.BACKEND_PROFILES.get(old) is bp.BACKEND_PROFILES[new]


def test_canonical_backend_warns_on_a_legacy_name():
    with pytest.warns(FutureWarning, match="'isaac' is deprecated; use 'panda_isaac'"):
        assert bp.canonical_backend('isaac') == 'panda_isaac'


def test_canonical_backend_leaves_other_names_alone():
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter('error')
        assert bp.canonical_backend('panda_isaac') == 'panda_isaac'
        assert bp.canonical_backend('no_such') == 'no_such'      # 판정은 호출자 몫이다


def test_resolve_channel_accepts_a_legacy_name_with_a_warning():
    with pytest.warns(FutureWarning):
        old = bp.resolve_backend_channel('functionbay')
    assert old == bp.resolve_backend_channel('panda_functionbay')


def test_get_backend_maps_a_legacy_name_to_the_current_one():
    """객체의 `name` 은 늘 지금 이름이다 — 팩토리 · 로그로 흘러가는 이름이 하나로 모인다."""
    from robot_control.backends import get_backend
    with pytest.warns(FutureWarning):
        backend = get_backend('isaac')
    assert backend.name == 'panda_isaac'
