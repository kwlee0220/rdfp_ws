"""선택적 백엔드 제공자 조회 — 계층 역행 의존을 없애는 seam.

트윈은 제어 계층(`robot_control`) 위에 있고 수집 계층(`rdfp`) **아래**에 있다.
그런데 세션/에피소드 경계 연산은 수집 계층의 `session_control_node` 를 호출해야
한다. 트윈이 `rdfp.session` 을 직접 import 하면 아래 계층이 위 계층에 의존하게
되어, 제어 스택만 설치한 환경에서 트윈이 import 조차 되지 않는다.

그래서 **제공자가 스스로 등록하고 트윈은 조회만 한다**. 수집 계층이
``robot_twin.backends`` entry point 그룹에 팩토리를 등록해 두면 여기서 찾아
쓰고, 설치되어 있지 않으면 해당 연산만 비활성화된다.

    # rdfp/setup.py
    'robot_twin.backends': [
        'session_control = rdfp.session.twin_backend:create_session_control_client',
    ]
"""

from __future__ import annotations

from typing import Any, Callable, Optional

import logging

ENTRY_POINT_GROUP = 'robot_twin.backends'

_logger = logging.getLogger(__name__)

# 조회 결과 캐시. 값이 `None` 인 것과 아직 조회하지 않은 것을 구분해야 하므로
# 딕셔너리에 키가 있는지로 판단한다.
_cache: dict[str, Optional[Callable[..., Any]]] = {}


def load_provider(name: str) -> Optional[Callable[..., Any]]:
    """entry point 그룹에서 제공자 팩토리를 찾는다. 없으면 ``None``.

    Args:
        name: entry point 이름 (예: ``'session_control'``).

    Returns:
        팩토리 callable. 제공자가 설치되어 있지 않거나 로드에 실패하면 ``None``.
    """
    if name in _cache:
        return _cache[name]

    factory: Optional[Callable[..., Any]] = None
    try:
        from importlib.metadata import entry_points
        # Python 3.10+ 의 select() API 를 쓴다.
        for ep in entry_points(group=ENTRY_POINT_GROUP):
            if ep.name == name:
                factory = ep.load()
                break
    except Exception as exc:      # noqa: BLE001 - 제공자 부재는 정상 경로다
        _logger.warning(f"failed to load backend provider '{name}': {exc}")
        factory = None

    _cache[name] = factory
    return factory


def clear_cache() -> None:
    """조회 캐시를 비운다 (테스트용)."""
    _cache.clear()
