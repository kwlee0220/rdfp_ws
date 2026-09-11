"""백엔드 객체 — 프로파일 값과 백엔드 고유 코드를 한곳에 모은다.

    from robot_control.backends import get_backend

    backend = get_backend('isaac')
    client = backend.create_move_group_client(node)

`rclpy` 를 끌어오지 않으므로 launch 파일과 설정만 읽는 도구에서도 쓸 수 있다.
"""

from __future__ import annotations

from .base import Backend, get_backend

__all__ = ['Backend', 'get_backend']
