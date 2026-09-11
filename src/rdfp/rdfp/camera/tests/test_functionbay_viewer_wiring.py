"""`rdfp_panda_functionbay` 의 뷰어 배선 테스트 — 창이 두 개 뜨지 않아야 한다.

**include 된 launch 는 부모의 launch configuration 을 물려받고 자기
`DeclareLaunchArgument` 의 기본값으로 되돌리지 않는다** (2026-09-08 실측). 두 계층이
같은 `enable_image_viewer` 이름을 쓰게 된 뒤로, 넘기지 않으면 `:=true` 가 백엔드까지
새어 들어가 제어 계층의 평범한 뷰어까지 뜬다 — 같은 영상을 띄우는 창이 둘이 되고
오류는 없다. 그래서 **명시적으로 `false` 를 전달하는 것**이 계약이다.
"""

from __future__ import annotations

import importlib.util
import pathlib

import pytest

pytest.importorskip('launch')
pytest.importorskip('launch_ros')
pytest.importorskip('robot_control')

from launch import LaunchContext                                     # noqa: E402
from launch.actions import IncludeLaunchDescription                  # noqa: E402
from launch.utilities import perform_substitutions                   # noqa: E402
from launch_ros.actions import Node                                  # noqa: E402

_LAUNCH_FILE = (pathlib.Path(__file__).resolve().parents[3]
                / 'launch' / 'rdfp_panda_functionbay.launch.py')


@pytest.fixture(scope='module')
def launch_module():
    """launch 파일을 모듈로 읽는다 — 설치본이 아니라 **소스**를 본다."""
    spec = importlib.util.spec_from_file_location('rdfp_panda_functionbay_launch', _LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _perform(value) -> str:
    subs = value if isinstance(value, list) else [value]
    return perform_substitutions(LaunchContext(), subs)


def test_backend_include_turns_the_control_layer_viewer_off(launch_module):
    """백엔드에 `enable_image_viewer=false` 를 **명시적으로** 넘긴다 (docstring 참고)."""
    include = launch_module._included_backend()
    assert isinstance(include, IncludeLaunchDescription)
    forwarded = dict(include.launch_arguments)
    assert 'enable_image_viewer' in forwarded, sorted(forwarded)
    # 관심 있는 값만 푼다 — 나머지는 부모 스코프의 LaunchConfiguration 이라 여기서 못 푼다.
    assert _perform(forwarded['enable_image_viewer']) == 'false'


def test_exactly_one_viewer_and_one_republish(launch_module):
    """뷰어는 세션 오버레이 하나, republish 도 하나다."""
    entities = launch_module.generate_launch_description().entities
    nodes = [e for e in entities if isinstance(e, Node)]
    viewers = [n for n in nodes if 'image_viewer' in str(n._Node__node_executable)]
    republishers = [n for n in nodes if 'republish' in str(n._Node__node_executable)]
    assert len(viewers) == 1, viewers
    assert 'rdfp_image_viewer_node' in str(viewers[0]._Node__node_executable)
    assert len(republishers) == 1, republishers


def test_viewer_pulls_the_raw_publisher_up(launch_module):
    """뷰어를 켜면 raw 발행자가 **파생으로** 함께 뜬다.

    예전에는 둘이 `enable_image_viewer` 라는 **같은 인자**에 묶여 있었다. 그것이
    사고였다 — 뷰어를 끄면 raw 가 사라져 헤드리스 수집에서 레코더가 조용히 0 프레임을
    담았다. 지금은 raw 발행자가 `enable_camera` 를 보고, `enable_camera` 의 기본값이
    **모든 raw 소비자**에서 파생된다. 그래서 소비자를 늘려도 결합이 되살아나지 않으면서
    "뷰어를 켰는데 빈 창" 도 생기지 않는다.
    """
    entities = launch_module.generate_launch_description().entities
    nodes = [e for e in entities if isinstance(e, Node)]
    viewer = _only(nodes, 'image_viewer')
    republish = _only(nodes, 'republish')

    # 뷰어는 자기 인자를, raw 발행자는 `enable_camera` 를 본다.
    assert viewer.condition.evaluate(_ctx(enable_image_viewer='true')) is True
    assert viewer.condition.evaluate(_ctx(enable_image_viewer='false')) is False
    assert republish.condition.evaluate(_ctx(enable_camera='true')) is True
    assert republish.condition.evaluate(_ctx(enable_camera='false')) is False

    # 그 둘을 잇는 것이 `enable_camera` 의 기본값이다.
    declared = _declared(entities, 'enable_camera')
    assert _resolve(declared, enable_image_viewer='true',
                    image_recorder_input_format='mjpeg') == 'true'
    assert _resolve(declared, enable_image_viewer='false',
                    image_recorder_input_format='mjpeg') == 'false'


def test_raw_recorder_also_pulls_the_publisher_up(launch_module):
    """뷰어가 꺼져 있어도 **레코더가 raw 를 원하면** 발행자가 뜬다.

    이 항목이 예전 구조가 놓쳤던 경우다 — 헤드리스(`enable_image_viewer:=false`) 수집에서
    raw 발행자가 사라져 레코더가 조용히 0 프레임을 담았다.
    """
    entities = launch_module.generate_launch_description().entities
    declared = _declared(entities, 'enable_camera')

    assert _resolve(declared, enable_image_viewer='false',
                    image_recorder_input_format='rawvideo') == 'true'


def test_recorder_defaults_to_the_compressed_stream(launch_module):
    """기본은 `mjpeg` 다 — 그 경로는 raw 토픽(9.2 MB/s)을 아예 만들지 않는다."""
    entities = launch_module.generate_launch_description().entities
    declared = _declared(entities, 'image_recorder_input_format')

    assert _resolve(declared) == 'mjpeg'


def _only(nodes, key):
    found = [n for n in nodes if key in str(n._Node__node_executable)]
    assert len(found) == 1, f'{key}: {found}'
    return found[0]


def _declared(entities, name):
    from launch.actions import DeclareLaunchArgument
    found = [e for e in entities
             if isinstance(e, DeclareLaunchArgument) and e.name == name]
    assert len(found) == 1, f'{name} 선언이 {len(found)}개다'
    return found[0]


def _ctx(**config) -> LaunchContext:
    ctx = LaunchContext()
    ctx.launch_configurations.update(config)
    return ctx


def _resolve(declared, **config) -> str:
    return perform_substitutions(_ctx(**config), list(declared.default_value))


def test_topic_defaults_come_from_the_backend_profile(launch_module):
    """토픽 이름을 여기 되적으면 제어 계열 `panda_functionbay` 와 갈린다.

    갈려도 각 launch 안에서는 짝이 맞아 **동작하므로 안 드러난다** — 드러나는 것은
    두 계열을 갈아탔을 때 "영상이 안 담긴다" / "명령이 안 먹는다" 뿐이다. 그래서
    되적힌 값 옆에 "같아야 한다" 는 주석을 다는 것으로는 부족하고, 같은 프로파일에서
    파생시킨다.
    """
    from robot_control.backends import get_backend

    backend = get_backend('functionbay')
    defaults = {}
    for action in launch_module.generate_launch_description().entities:
        if type(action).__name__ != 'DeclareLaunchArgument':
            continue
        raw = action.default_value
        defaults[action.name] = (
            ''.join(getattr(part, 'text', str(part)) for part in raw) if raw else None)

    assert defaults['camera_image_topic'] == backend.value('camera', 'image_topic')
    assert defaults['camera_compressed_topic'] == backend.value('camera', 'compressed_topic')
    assert defaults['target_joint_cmds_input_topic'] == backend.arm_command_topic


def test_conditioned_nodes_come_before_the_backend_include(launch_module):
    """**순서가 계약이다** — 조건을 가진 이 계층의 노드는 include 앞에 와야 한다.

    `IncludeLaunchDescription.visit()` 은 `launch_arguments` 를 `SetLaunchConfiguration`
    액션으로 돌려주고, launch 서비스가 그것을 **같은 스코프에서** 실행한다. 그래서 백엔드에
    넘긴 `enable_image_viewer:=false` · `enable_camera:=false` 가 부모로 새고, include
    **뒤에** 있는 노드는 그 값을 본다 — 뷰어와 raw 발행자가 **오류 없이 안 뜬다.**

    `GroupAction(scoped=True)` 로 감싸는 길은 **막혀 있다**: 감싸면 include 가 끝날 때
    스코프가 pop 되는데 백엔드는 노드를 `OnProcessExit` 로 나중에 띄우므로, 그 시점에
    `servo_linear_scale` 이 사라져 launch 가 죽는다 (2026-09-09 실측).
    """
    from launch.actions import IncludeLaunchDescription

    entities = launch_module.generate_launch_description().entities
    include_at = [i for i, e in enumerate(entities)
                  if isinstance(e, IncludeLaunchDescription)]
    assert len(include_at) == 1, include_at
    include_at = include_at[0]

    for index, entity in enumerate(entities):
        if isinstance(entity, Node) and entity.condition is not None:
            assert index < include_at, (
                f'{entity._Node__node_executable} 가 include 뒤에 있다 — '
                '새어 나온 인자 때문에 조건이 false 가 된다')


def test_leaked_arguments_do_not_disable_this_layer(launch_module):
    """실제 방문 순서를 흉내 내 **뷰어가 켜진 채 남는지** 본다.

    조건 평가는 방문 시점에 일어나므로, include 가 값을 덮어쓰기 전에 지나가야 한다.
    """
    from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                                SetLaunchConfiguration)

    ctx = LaunchContext()
    started = []
    for entity in launch_module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(ctx)
        elif isinstance(entity, IncludeLaunchDescription):
            for sub in (entity.visit(ctx) or []):
                # launch 서비스는 반환된 SetLaunchConfiguration 을 같은 스코프에서 실행한다.
                if isinstance(sub, SetLaunchConfiguration):
                    sub.visit(ctx)
        elif isinstance(entity, Node):
            if entity.condition is None or entity.condition.evaluate(ctx):
                started.append(str(entity._Node__node_executable))

    assert any('image_viewer' in name for name in started), started
    assert any('republish' in name for name in started), started
