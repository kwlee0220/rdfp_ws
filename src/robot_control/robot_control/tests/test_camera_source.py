#!/usr/bin/env python3

"""raw 이미지 발행자를 **누가 고르는가** — 프로파일의 `camera.source` 다.

소비자(뷰어·레코더)는 어느 노드가 떴는지 몰라야 한다. 그래야 백엔드를 바꿔도
`camera_image_topic` 하나만 구독하면 되고, 제어 계층 노드가 백엔드를 알게 되는 일이
없다. 설계: `docs/camera/compressed_image_pipeline_design.md`.
"""

from __future__ import annotations

import pytest

pytest.importorskip('launch')
pytest.importorskip('launch_ros')

from launch import LaunchContext                                        # noqa: E402
from launch.actions import DeclareLaunchArgument                        # noqa: E402
from launch.utilities import perform_substitutions                      # noqa: E402

from robot_control.backends import get_backend                          # noqa: E402
from robot_control.launch_helpers.camera import (                       # noqa: E402
    create_raw_image_source_node, declare_camera_enable_argument)
from robot_control.launch_helpers.controller_startup import _chain_or_shutdown  # noqa: E402


def _executable(node) -> str:
    return str(node._Node__node_executable)


# ----- source -> 노드 --------------------------------------------------------

@pytest.mark.parametrize('backend,executable', [
    ('mock', 'camera_node'),
    ('mock_jgpc', 'camera_node'),
    ('functionbay', 'republish'),
    ('isaac', None),
])
def test_source_picks_the_publisher(backend, executable):
    """`camera.source` 하나가 무엇이 뜰지 정한다."""
    node = create_raw_image_source_node(get_backend(backend))

    if executable is None:
        assert node is None, 'native 는 시뮬레이터가 raw 를 직접 낸다 — 우리 노드가 없다'
    else:
        assert _executable(node) == executable


def test_every_profile_declares_a_camera_source():
    """빠뜨리면 `None` 이 되어 **조용히 아무것도 안 뜬다.**"""
    from robot_control.backend_profiles import BACKEND_PROFILES

    for name in BACKEND_PROFILES:
        backend = get_backend(name)
        if backend.block('camera') is None:
            continue        # 카메라가 없는 백엔드는 정당하다
        assert backend.camera_source in ('device', 'compressed', 'native'), \
            f'{name}: camera.source 가 없거나 모르는 값이다'


def test_unknown_source_is_rejected():
    """조용히 아무것도 안 띄우면 증상이 "이미지가 안 온다" 뿐이라 원인을 안 가리킨다."""
    backend = get_backend('mock')
    backend._profile['camera'] = {'source': 'typo', 'image_topic': '/x'}

    with pytest.raises(ValueError, match='unknown camera.source'):
        create_raw_image_source_node(backend)


def test_compressed_backend_declares_both_names():
    """압축 백엔드는 이름이 둘이다 — 소비자가 갈리기 때문이다."""
    backend = get_backend('functionbay')

    assert backend.camera_source == 'compressed'
    assert backend.value('camera', 'compressed_topic')
    assert backend.value('camera', 'image_topic')


# ----- enable_camera 기본값이 raw 소비자에서 파생된다 --------------------------

def _default_of(argument: DeclareLaunchArgument, **config) -> str:
    context = LaunchContext()
    for key, value in config.items():
        context.launch_configurations[key] = value
    return perform_substitutions(context, list(argument.default_value))


def test_no_consumer_means_off():
    """소비자가 없으면 raw 를 만들지 않는다 — 9.2 MB/s 가 그냥 안 생긴다."""
    assert _default_of(declare_camera_enable_argument()) == 'false'


def test_single_consumer_follows_it():
    """뷰어를 켜면 디코더가 함께 뜬다 — "빈 창" 을 없애는 것이 이 파생의 목적이다."""
    argument = declare_camera_enable_argument(('enable_image_viewer', 'true'))

    assert _default_of(argument, enable_image_viewer='true') == 'true'
    assert _default_of(argument, enable_image_viewer='false') == 'false'


def test_any_consumer_turns_it_on():
    """**둘 중 하나라도** raw 를 원하면 켠다.

    예전에는 디코더가 `enable_image_viewer` 하나에 묶여 있어, 헤드리스 수집에서 뷰어를
    끄면 raw 발행자가 사라져 **레코더가 조용히 0 프레임을 담았다.**
    """
    argument = declare_camera_enable_argument(('enable_image_viewer', 'true'),
                                              ('image_recorder_input_format', 'rawvideo'))
    both_off = {'enable_image_viewer': 'false', 'image_recorder_input_format': 'mjpeg'}

    assert _default_of(argument, **both_off) == 'false'
    assert _default_of(argument, **{**both_off, 'enable_image_viewer': 'true'}) == 'true'
    assert _default_of(argument,
                       **{**both_off, 'image_recorder_input_format': 'rawvideo'}) == 'true'


# ----- native 백엔드의 None 이 기동 체인을 깨지 않는다 -------------------------

def test_none_actions_are_skipped_in_the_startup_chain():
    """`native` 는 `None` 을 돌려준다 — 그대로 목록에 들어가면 launch 가 기동 중에 죽는다."""
    handler = _chain_or_shutdown(['a', None, 'b'], 'spawner')
    event = type('E', (), {'returncode': 0})()

    assert handler.__closure__ is not None      # 클로저 형태가 바뀌면 아래가 무의미하다
    assert _run(handler, event) == ['a', 'b']


def _run(handler, event):
    # `_chain_or_shutdown` 은 핸들러 함수를 돌려준다 (OnProcessExit 의 on_exit).
    return handler(event, None)
