#!/usr/bin/env python3

"""펑션베이 수집 launch 의 레코더 배선 — **압축을 직접 받는가.**

레코더가 둘이고 능력이 다르다.

| 노드 | 구동 | 구독 | `input_format` |
|---|---|---|---|
| `image_recorder_node` | 서비스 | `sensor_msgs/Image` **만** | 없다 |
| `rdfp_image_recorder` | `/session` | 형식에 따라 갈린다 | 있다 |

**앞의 것을 쓰면 `camera_republish` 에 매달린다.** 그 노드는 뷰어와 짝으로 뜨므로
`enable_image_viewer:=false` 로 헤드리스 수집을 하면 raw 발행자가 사라지고 **레코더가
조용히 0 프레임을 담는다** — 오류도 경고도 없다. 2026-09-08 까지 실제로 그랬다.

launch 를 실행하지 않고 소스를 AST 로 읽는다 — `launch_ros` 의 내부 표현에 의존하면
버전이 바뀔 때 검사가 먼저 깨진다.
"""

from __future__ import annotations

import ast
import os

import pytest

_LAUNCH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))),
    'launch', 'rdfp_panda_functionbay.launch.py')


def _recorder_node_call() -> ast.Call:
    """`image_recorder_node = Node(...)` 의 호출 노드."""
    if not os.path.isfile(_LAUNCH):
        pytest.skip('설치 트리에는 launch 소스가 없다')
    tree = ast.parse(open(_LAUNCH, encoding='utf-8').read())
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name)
                and node.targets[0].id == 'image_recorder_node'
                and isinstance(node.value, ast.Call)):
            return node.value
    raise AssertionError('image_recorder_node 대입을 찾지 못했다')


def _kwarg(call: ast.Call, name: str):
    for kw in call.keywords:
        if kw.arg == name:
            return kw.value
    return None


def test_uses_the_session_driven_recorder():
    """서비스 구동 레코더는 압축을 못 받는다 — 그것을 쓰면 republish 에 매달린다."""
    executable = _kwarg(_recorder_node_call(), 'executable')

    assert isinstance(executable, ast.Constant)
    assert executable.value == 'rdfp_image_recorder', (
        "`image_recorder_node` 는 sensor_msgs/Image 만 구독하고 input_format 이 없다")


def test_input_format_is_an_argument_not_a_literal():
    """`input_format` 은 인자에서 온다 — 기본값은 `mjpeg` 다.

    상수로 박아 두면 raw 로 담을 방법이 없고, 인자로 열어 두면 `enable_camera`
    기본값이 그것을 보고 파생될 수 있다(`rawvideo` 면 raw 발행자를 끌어올린다).
    """
    params = _kwarg(_recorder_node_call(), 'parameters')
    assert isinstance(params, ast.List) and params.elts
    entries = {}
    for key, value in zip(params.elts[0].keys, params.elts[0].values):
        if isinstance(key, ast.Constant):
            entries[key.value] = value

    assert 'input_format' in entries, 'input_format 을 안 주면 raw 를 구독한다'
    source = ast.unparse(entries['input_format'])
    assert 'image_recorder_input_format' in source, source


def test_subscribes_to_the_compressed_topic():
    """`image` remap 이 **압축** 토픽을 가리켜야 한다.

    `input_format` 만 바꾸고 remap 을 raw 로 두면 타입이 달라 **ROS 2 가 연결만 안 하고
    오류도 안 낸다** — 증상은 다시 "0 프레임" 이다.
    """
    remaps = _kwarg(_recorder_node_call(), 'remappings')
    assert isinstance(remaps, ast.List)

    targets = {}
    for pair in remaps.elts:
        src = pair.elts[0]
        dst = pair.elts[1]
        if isinstance(src, ast.Constant):
            targets[src.value] = ast.unparse(dst)

    assert 'image' in targets
    assert 'camera_compressed_topic' in targets['image'], targets['image']


def test_no_auto_start_argument_remains():
    """세션 구동 레코더에는 `auto_start` 가 없다 — 남겨 두면 조용히 무시된다.

    녹화는 `/session` 이 `IN_EPISODE` 일 때 자동으로 돈다.
    """
    source = open(_LAUNCH, encoding='utf-8').read()

    assert 'image_recorder_auto_start' not in source
