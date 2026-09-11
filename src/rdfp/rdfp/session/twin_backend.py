"""robot twin 용 세션 제어 백엔드 제공자.

`robot_twin` 은 제어 계층 위에 있고 수집 계층(`rdfp`) 아래에 있어서, 세션/에피소드
연산을 위해 이쪽을 직접 import 할 수 없다. 대신 `setup.py` 의
``robot_twin.backends`` entry point 그룹에 이 팩토리를 등록해 두면 트윈이 조회해
쓴다. 자세한 배경은 :mod:`robot_twin.backend_registry` 를 본다.
"""

from __future__ import annotations

from typing import Any

from rclpy.node import Node


def create_session_control_client(node: Node, *, wait_timeout_sec: float = 0.0) -> Any:
    """`SessionControlClient` 를 만들어 돌려준다.

    Args:
        node: 클라이언트가 붙을 ROS 노드.
        wait_timeout_sec: 서비스 준비 대기 시간(초). 트윈은 ``0`` 으로 호출한다 —
            기본값처럼 블로킹하면 `session_control_node` 가 없을 때 HTTP 요청
            스레드가 묶인다.

    Returns:
        `SessionControlClient` 인스턴스.
    """
    from rdfp.session import SessionControlClient
    return SessionControlClient(node, wait_timeout_sec=wait_timeout_sec)
