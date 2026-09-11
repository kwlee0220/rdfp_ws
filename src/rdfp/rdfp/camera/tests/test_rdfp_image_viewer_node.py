#!/usr/bin/env python3

"""`RdfpImageViewerNode` — **세션 채널 계약** 테스트.

이 노드는 화면(OpenCV 윈도우)을 열므로 전체를 단위 테스트하기 어렵다. 여기서는
헤드리스에서도 확인할 수 있는 것 하나를 지킨다 — **세션 토픽이 전역이라는 것.**

세션은 시스템에 하나다 (규약 §2.5). 로봇이 둘 이상인 것은 공동 작업으로 하나의 학습
데이터를 만든다는 뜻이므로, 이 노드가 로봇별 네임스페이스 안에 들어가도 세션 토픽은
따라 붙으면 안 된다. 상대(`session`)로 되돌아가면 `/abc/session` 을 찾아 **오버레이가
영영 IDLE 로 남는다** — 화면만 보고는 원인을 알 수 없다.
"""

from __future__ import annotations

from typing import Iterator

from unittest.mock import patch

import pytest

pytest.importorskip('rclpy', reason='requires ROS 2 runtime')

import rclpy                                                              # noqa: E402

from rdfp.camera import rdfp_image_viewer_node as node_module             # noqa: E402
from rdfp.camera.rdfp_image_viewer_node import RdfpImageViewerNode        # noqa: E402


@pytest.fixture(scope='module', autouse=True)
def _rclpy_session() -> Iterator[None]:
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def _no_window() -> Iterator[None]:
    """OpenCV 윈도우 생성을 막는다 — 화면 없는 환경에서 죽는다."""
    with patch.object(node_module, 'cv2', create=True):
        yield


def test_session_topic_constant_is_absolute():
    """상수가 상대로 되돌아가면 네임스페이스 면역이 조용히 사라진다."""
    assert node_module._DEFAULT_SESSION_TOPIC.startswith('/')


def test_session_subscription_ignores_the_node_namespace(_no_window):
    node = RdfpImageViewerNode(namespace='/abc')
    try:
        assert node._session_sub.topic_name == '/session'
    finally:
        node.destroy_node()
