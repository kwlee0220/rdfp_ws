"""압축 이미지를 raw 로 되살리는 `camera_republish` 배선 테스트.

**조용히 틀릴 수 있는 것만 고정한다** — remap 이름, 위치 인자 순서, 조건 인자 이름.
셋 중 어느 하나가 어긋나도 ROS 2 는 오류를 내지 않고 "화면이 비어 있다"로만 나타난다.
"""

from __future__ import annotations

import pytest

pytest.importorskip('launch')
pytest.importorskip('launch_ros')

from launch import LaunchContext                                     # noqa: E402
from launch.utilities import perform_substitutions                   # noqa: E402

from robot_control.launch_helpers.camera import (                    # noqa: E402
    create_camera_republish_node,
    declare_camera_republish_arguments,
)

COMPRESSED = '/camera_image/compressed'
RAW = '/camera_image'


def _context(enable_camera: str = 'true') -> LaunchContext:
    ctx = LaunchContext()
    ctx.launch_configurations.update({
        'camera_compressed_topic': COMPRESSED,
        'camera_image_topic': RAW,
        'enable_camera': enable_camera,
    })
    return ctx


def _cmd_words(node, ctx) -> list[str]:
    """명령줄에서 지금 확정 가능한 조각만 모은다.

    ``--ros-args`` 뒤에 붙는 부분은 실행 시점에만 정해지는 ``LocalSubstitution`` 이라
    여기서는 풀 수 없다 — remap 은 ``expanded_remapping_rules`` 로 따로 본다.
    """
    words = []
    for part in node.cmd:
        try:
            words.append(perform_substitutions(ctx, list(part)))
        except AttributeError:
            pass
    return words


def test_republish_subscribes_the_transport_suffixed_topic():
    """**`in/compressed` 를 remap 한다 — `in` 이 아니다.**

    image_transport 의 구독 플러그인은 base 토픽에 transport 접미사를 붙인 완성된
    이름으로 구독한다. `in` 만 remap 하면 한 장도 받지 못하고 **오류도 없다.**
    """
    node = create_camera_republish_node()
    ctx = _context()
    node._perform_substitutions(ctx)
    rules = dict(node.expanded_remapping_rules)
    assert rules.get('in/compressed') == COMPRESSED, rules
    assert 'in' not in rules
    assert rules.get('out') == RAW, rules


def test_republish_positional_transport_args_come_in_order():
    """`republish <in_transport> <out_transport>` 순서다. 뒤집으면 raw 를 압축으로 읽는다."""
    node = create_camera_republish_node()
    ctx = _context()
    node._perform_substitutions(ctx)
    words = _cmd_words(node, ctx)
    assert 'republish' in words[0], words
    assert words[1:3] == ['compressed', 'raw'], words


def test_republish_is_gated_by_enable_camera():
    """**`enable_image_viewer` 가 아니라 `enable_camera` 다.**

    뷰어 인자에 묶여 있던 것이 사고였다 — 헤드리스로 돌리려고 뷰어를 끄면 raw 발행자가
    사라져 서비스 구동 레코더가 조용히 0 프레임을 담았다. `enable_camera` 는 **모든**
    raw 소비자에서 기본값이 파생되므로 소비자를 늘려도 그 결합이 되살아나지 않는다
    (`declare_camera_enable_argument`).

    조건 자체는 남아 있어야 한다 — 없으면 헤드리스에서도 9.2 MB/s 를 흘린다.
    """
    node = create_camera_republish_node()
    assert node.condition is not None, '조건이 없으면 헤드리스에서도 9.2 MB/s 를 흘린다'
    assert node.condition.evaluate(_context('true')) is True
    assert node.condition.evaluate(_context('false')) is False


def test_compressed_topic_default_comes_from_the_caller():
    """압축 토픽 이름은 **시뮬레이터가 정하는 값**이라 helper 가 기본값을 갖지 않는다."""
    args = declare_camera_republish_arguments(COMPRESSED)
    assert len(args) == 1
    arg = args[0]
    assert arg.name == 'camera_compressed_topic'
    assert arg.default_value[0].text == COMPRESSED
