"""펑션베이의 **raw 발행자 배선** — 조용한 0 프레임 / 빈 창을 막는 계약.

이 시뮬레이터는 압축(JPEG)만 발행한다. raw 를 요구하는 소비자(`image_viewer_node`)가
있으면 `image_transport/republish` 가 되살려 줘야 하는데, **타입이 다르면 ROS 2 는
연결만 안 하고 오류를 내지 않으므로** 증상이 "빈 창" · "0 프레임" 뿐이다.

그래서 raw 발행자는 `enable_camera` **하나**가 켜고, 그 기본값은 **raw 소비자에서
파생된다.** 예전에는 디코더가 `enable_image_viewer` 조건으로 떠서, 헤드리스로 돌리려고
뷰어를 끄면 발행자가 사라져 레코더가 조용히 0 프레임을 담았다.

> **여기서 검사하는 대상이 바뀌었다 (2026-09-13).** 예전에는 묶음 launch
> `rdfp_panda_functionbay` 를 봤고, 계약도 *"include 한 백엔드에
> `enable_image_viewer:=false` 를 명시적으로 넘긴다"* 였다 — 두 계층이 같은 인자
> 이름을 써서 창이 둘 뜨는 문제였다. 그 묶음을 삭제하면서 **include 자체가 없어져
> 그 계약은 소멸했고**, 남은 것은 제어 계층 `panda_functionbay` 안의 파생 로직이다.
"""

from __future__ import annotations

import importlib.util
import pathlib

import pytest

pytest.importorskip('launch')
pytest.importorskip('launch_ros')
pytest.importorskip('robot_control')

from launch import LaunchContext                                     # noqa: E402
from launch.actions import DeclareLaunchArgument                     # noqa: E402
from launch.utilities import perform_substitutions                   # noqa: E402

from ament_index_python.packages import get_package_share_directory  # noqa: E402

_LAUNCH_FILE = (pathlib.Path(get_package_share_directory('robot_control'))
                / 'launch' / 'panda_functionbay.launch.py')


@pytest.fixture(scope='module')
def launch_module():
    """launch 파일을 모듈로 읽는다."""
    if not _LAUNCH_FILE.is_file():
        pytest.skip(f'{_LAUNCH_FILE} 없음 (colcon build 를 먼저 돌린다)')
    spec = importlib.util.spec_from_file_location('panda_functionbay_launch', _LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _ctx(**config) -> LaunchContext:
    ctx = LaunchContext()
    for key, value in config.items():
        ctx.launch_configurations[key] = value
    return ctx


def _declared(launch_module, name: str) -> DeclareLaunchArgument:
    """최상위에서 선언된 argument 하나."""
    found = [e for e in launch_module.generate_launch_description().entities
             if isinstance(e, DeclareLaunchArgument) and e.name == name]
    assert len(found) == 1, f'{name}: {found}'
    return found[0]


def _resolve(arg: DeclareLaunchArgument, **config) -> str:
    return perform_substitutions(_ctx(**config), list(arg.default_value))


# ----- enable_camera 는 raw 소비자에서 파생된다 --------------------------------


def test_viewer_pulls_the_raw_publisher_up(launch_module):
    """뷰어를 켜면 디코더가 함께 뜬다 — 안 그러면 '빈 창' 이다."""
    declared = _declared(launch_module, 'enable_camera')

    assert _resolve(declared, enable_image_viewer='true') == 'true'
    assert _resolve(declared, enable_image_viewer='false') == 'false'


def test_enable_camera_is_a_separate_argument(launch_module):
    """파생은 **기본값**일 뿐이라 명시하면 그쪽이 이긴다.

    이것이 성립해야 `enable_image_viewer:=false` 인 헤드리스 수집에서도
    `enable_camera:=true` 로 raw 를 살릴 수 있다. 둘을 한 인자로 합치면 그 조합이
    사라진다.
    """
    declared = _declared(launch_module, 'enable_camera')

    assert declared.name == 'enable_camera'
    # 뷰어 인자와 **다른** 인자다.
    assert _declared(launch_module, 'enable_image_viewer') is not declared


# ----- 각 노드는 자기 스위치가 켠다 --------------------------------------------


def test_viewer_is_gated_by_enable_image_viewer(launch_module):
    viewer = launch_module.create_image_viewer_node_for_functionbay()

    assert viewer.condition.evaluate(_ctx(enable_image_viewer='true')) is True
    assert viewer.condition.evaluate(_ctx(enable_image_viewer='false')) is False


def test_republish_is_gated_by_enable_camera():
    """디코더는 `enable_camera` 가 켠다 — 뷰어 인자가 아니다."""
    from robot_control.launch_helpers.camera import create_camera_republish_node

    republish = create_camera_republish_node()

    assert republish.condition.evaluate(_ctx(enable_camera='true')) is True
    assert republish.condition.evaluate(_ctx(enable_camera='false')) is False


def test_control_layer_uses_the_plain_viewer(launch_module):
    """제어 계층은 세션을 모르는 평범한 뷰어를 쓴다 — 수집 계층 것과 다른 노드다."""
    viewer = launch_module.create_image_viewer_node_for_functionbay()

    executable = str(viewer._Node__node_executable)
    assert 'image_viewer_node' in executable
    assert 'rdfp_image_viewer_node' not in executable
