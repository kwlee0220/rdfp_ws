#!/usr/bin/env python3

"""README §2.0 의 **백엔드 횡단표가 실제와 맞는가.**

표를 손으로 맞추면 낡는다 — 그리고 낡은 표는 없는 표보다 나쁘다. 값을 믿고 그대로 쓰다
"설정했는데 안 먹는다" 가 되는데, 그 증상은 표를 의심하게 만들지 않기 때문이다.

그래서 네 launch 를 **실제로 만들어** 셀 하나하나를 대조하고, 값이 갈리는데 표에 없는
인자까지 잡는다. launch 를 고치면 표를 안 고친 쪽이 여기서 깨진다.

`panda_gazebo` 는 뺐다 — 이 환경에 `ros_gz_sim` 이 없어 launch 로드 자체가 실패한다.
"""

from __future__ import annotations

from typing import Optional

import os
import re

import pytest

pytest.importorskip('launch')
pytest.importorskip('launch_ros')

from launch.actions import DeclareLaunchArgument                    # noqa: E402
from launch.substitutions import LaunchConfiguration                # noqa: E402

from robot_control.backend_profiles import BACKEND_PROFILES         # noqa: E402

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_LAUNCH_DIR = os.path.join(_REPO, 'launch')
_README = os.path.join(_LAUNCH_DIR, 'README.md')

# 표의 열 순서. 헤더와 대조하므로 여기만 고치면 안 되고 README 도 함께 고쳐야 한다.
_COLUMNS = ['panda_mock', 'panda_jgpc_mock', 'panda_isaac', 'panda_functionbay']

# 모든 백엔드에서 같아 표에서 뺀 인자. **여기 없는데 값이 갈리면 표에 빠진 것이다.**
_UNIFORM = {'log_level', 'base_frame', 'publish_rate', 'enable_scene', 'scene_publish_rate'}

# 값이 절대 경로라 표에 싣기에 길다. §2.2 가 상대 경로로 적고 있다.
_TOO_LONG_FOR_THE_TABLE = {'controllers_file', 'camera_id'}

_NOT_DECLARED = '—'


def _load(name: str):
    import importlib.util
    import sys

    path = os.path.join(_LAUNCH_DIR, f'{name}.launch.py')
    if not os.path.isfile(path):
        pytest.skip('설치 트리에는 launch 소스가 없다')
    spec = importlib.util.spec_from_file_location(f'_matrix_{name}', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
    finally:
        sys.modules.pop(spec.name, None)
    return module


def _render(default) -> Optional[str]:
    """기본값을 표의 셀과 같은 문자열로 만든다.

    `LaunchConfiguration` 은 `$이름` 으로 적는다 — **파생이라는 사실 자체가 정보**이고,
    그것을 리터럴로 풀어 적으면 표가 파생을 숨기게 된다 (펑션베이의 `enable_camera` 가
    `enable_image_viewer` 를 따라간다는 것이 그 예다).
    """
    if default is None:
        return None
    parts = []
    for piece in default:
        if isinstance(piece, LaunchConfiguration):
            parts.append('$' + ''.join(getattr(x, 'text', str(x))
                                       for x in piece.variable_name))
        else:
            parts.append(getattr(piece, 'text', f'<{type(piece).__name__}>'))
    return ''.join(parts)


@pytest.fixture(scope='module')
def actual() -> dict:
    """`{launch 이름: {인자: 기본값 문자열}}`."""
    out = {}
    for name in _COLUMNS:
        module = _load(name)
        out[name] = {action.name: _render(action.default_value)
                     for action in module.generate_launch_description().entities
                     if isinstance(action, DeclareLaunchArgument)}
    return out


@pytest.fixture(scope='module')
def table() -> dict:
    """README 의 표를 `{인자: {launch 이름: 셀}}` 로 읽는다."""
    if not os.path.isfile(_README):
        pytest.skip('설치 트리에는 README 가 없다')
    text = open(_README, encoding='utf-8').read()
    block = re.search(r'<!-- BACKEND-MATRIX:BEGIN -->(.*?)<!-- BACKEND-MATRIX:END -->',
                      text, re.S)
    assert block, 'BACKEND-MATRIX 마커를 찾지 못했다 — 표를 지웠다면 이 테스트도 지운다'

    rows = [line.strip() for line in block.group(1).splitlines()
            if line.strip().startswith('|')]
    header = [cell.strip() for cell in rows[0].strip('|').split('|')]
    assert header[1:] == [f'`{name}`' for name in _COLUMNS], header

    out = {}
    for row in rows[2:]:            # 0=헤더, 1=구분선
        cells = [cell.strip() for cell in row.strip('|').split('|')]
        argument = cells[0].strip('`')
        out[argument] = dict(zip(_COLUMNS, cells[1:]))
    return out


def _cell_to_expected(cell: str) -> Optional[str]:
    return None if cell == _NOT_DECLARED else cell.strip('`')


def test_the_table_is_not_empty(table):
    """마커만 남고 내용이 비면 아래 검사가 조용히 아무 일도 안 한다."""
    assert len(table) >= 10, len(table)


def test_every_cell_matches_the_launch(table, actual):
    """셀 하나하나를 실제 기본값과 대조한다."""
    wrong = []
    for argument, row in table.items():
        for name, cell in row.items():
            expected = _cell_to_expected(cell)
            if argument not in actual[name]:
                got = None
            else:
                got = actual[name][argument]
            if got != expected:
                wrong.append(f'{argument} / {name}: 표={expected!r} 실제={got!r}')
    assert not wrong, '표와 실제가 다르다:\n  ' + '\n  '.join(wrong)


def test_no_diverging_argument_is_missing_from_the_table(table, actual):
    """**값이 갈리는데 표에 없으면** 그것이야말로 표가 놓친 것이다.

    빠진 인자는 "이 스택에서만 안 되네" 로 나타나고, 그때 표를 보면 아예 안 적혀 있어
    원인을 안 가리킨다.
    """
    everywhere = sorted(set().union(*(set(v) for v in actual.values())))
    missing = []
    for argument in everywhere:
        if argument in table or argument in _UNIFORM or argument in _TOO_LONG_FOR_THE_TABLE:
            continue
        values = {name: actual[name].get(argument) for name in _COLUMNS}
        if len(set(values.values())) > 1:
            missing.append(f'{argument}: {values}')
    assert not missing, '백엔드마다 다른데 표에 없다:\n  ' + '\n  '.join(missing)


def test_uniform_arguments_really_are_uniform(table, actual):
    """표에서 뺀 '공통' 인자가 정말 공통인가 — 갈리면 빼 두면 안 된다."""
    diverged = []
    for argument in sorted(_UNIFORM):
        values = {name: actual[name].get(argument) for name in _COLUMNS}
        if len(set(values.values())) > 1:
            diverged.append(f'{argument}: {values}')
    assert not diverged, "공통이라고 표에서 뺐는데 갈린다:\n  " + '\n  '.join(diverged)


def test_profile_derived_values_agree_with_the_launch(actual):
    """표가 "프로파일에서 온다" 고 적은 값들이 실제로 그런가.

    launch 가 프로파일에서 파생하지 않고 손으로 되적으면 두 곳이 갈리는데, 그 어긋남은
    에러가 아니라 "명령이 안 먹는다" 로 나타난다.
    """
    checks = [
        ('panda_isaac', 'isaac', 'camera_image_topic', ('camera', 'image_topic')),
        ('panda_isaac', 'isaac', 'servo_linear_scale', ('servo', 'linear_scale')),
        ('panda_isaac', 'isaac', 'servo_joint_source', ('servo', 'joint_source')),
        ('panda_functionbay', 'functionbay', 'camera_image_topic', ('camera', 'image_topic')),
        ('panda_functionbay', 'functionbay', 'camera_compressed_topic',
         ('camera', 'compressed_topic')),
        ('panda_functionbay', 'functionbay', 'servo_linear_scale', ('servo', 'linear_scale')),
    ]
    for launch_name, backend, argument, (block, key) in checks:
        profile_value = str((BACKEND_PROFILES[backend].get(block) or {})[key])
        assert actual[launch_name][argument] == profile_value, \
            f'{launch_name}: {argument} 가 프로파일 {backend}.{block}.{key} 와 다르다'


def test_camera_source_row_matches_the_profiles():
    """프로파일 횡단표의 `camera.source` 행."""
    expected = {'mock': 'device', 'mock_jgpc': 'device',
                'isaac': 'native', 'functionbay': 'compressed'}
    for name, source in expected.items():
        assert (BACKEND_PROFILES[name].get('camera') or {}).get('source') == source, name
