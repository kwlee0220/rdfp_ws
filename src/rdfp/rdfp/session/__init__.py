#!/usr/bin/env python3

"""세션 제어 — 상태 머신 노드와 그 클라이언트.

`SessionControlClient` 만 재노출한다. 노드(`session_control_node`)와 트윈 백엔드
(`twin_backend`)는 **진입점으로만 쓰이므로** 여기서 끌어올리지 않는다 — 끌어올리면
클라이언트 하나 쓰려는 호출자가 노드 구현까지 import 하게 된다.

계약과 사용법: `docs/session/session_control_guide.md`
"""

from __future__ import annotations

from .session_control_client import SessionControlClient

__all__ = [
    "SessionControlClient",
]
