"""트윈의 `fixtures` 정적 변수 — 고정물 목록을 어디서 어떻게 싣는가.

구멍·트레이는 `/tf` 에도 `/scene/objects` 에도 없다. 이 변수가 없으면 에이전트는
*"다른 hole 로 옮겨라"* 의 목적지를 **알 길이 없다** — 그래서 확인하는 것은
① 제공자가 MoveGroup 없이도 값을 주는가, ② 설정 오류를 재시도로 뭉개지 않는가,
③ 제공자 이름이 클라이언트 메서드를 가리지 않는가 다.
"""

from __future__ import annotations

from typing import Any, Optional

import pytest

from robot_twin.runtime import _STATIC_PROVIDERS, _fetch_fixtures


class _FakeMoveIt:
    def __init__(self, backend: Optional[str]) -> None:
        self.backend = backend


class _FakeConfig:
    def __init__(self, backend: Optional[str]) -> None:
        self.moveit = _FakeMoveIt(backend)


class _FakeRuntime:
    """설정만 갖는 런타임. **MoveGroup 클라이언트가 없다** — 그것이 요점이다."""

    def __init__(self, backend: Optional[str]) -> None:
        self.config = _FakeConfig(backend)


def test_fixtures_are_fetched_without_a_move_group_client() -> None:
    """파일 읽기라 MoveIt 기동을 기다릴 이유가 없다.

    `_fetch_static` 의 일반 경로는 클라이언트가 없으면 `StaticSourceUnavailable` 로
    거절한다. 제공자를 그 앞에 둔 이유가 이것이다.
    """
    pytest.importorskip('robot_control.scene.fixtures')
    payload = _fetch_fixtures(_FakeRuntime('functionbay'))

    assert payload['frame'] == 'panda_link0'
    assert set(payload['fixtures']) == {'peg_hole', 'peg_tray'}


def test_hole_entry_point_is_what_the_agent_aims_at() -> None:
    """*"다른 hole"* 의 목적지가 되는 값이다 — 실측(2026-09-09)과 맞는가."""
    pytest.importorskip('robot_control.scene.fixtures')
    hole = _fetch_fixtures(_FakeRuntime('functionbay'))['fixtures']['peg_hole']

    assert hole['entry_point']['z'] == pytest.approx(0.028)
    assert hole['depth'] == pytest.approx(0.025)
    assert hole['inner_diameter'] == pytest.approx(0.017)


def test_missing_backend_is_a_config_error_not_a_retry() -> None:
    """`StaticSourceUnavailable` 이면 캐시가 **영영 다시 시도한다.**

    백엔드 이름이 없는 것은 시간이 지나도 안 고쳐지므로 그 자리가 아니다.
    """
    with pytest.raises(ValueError, match='moveit.backend'):
        _fetch_fixtures(_FakeRuntime(None))


def test_provider_names_do_not_shadow_move_group_methods() -> None:
    """⚠️ 제공자를 **먼저** 보므로 이름이 겹치면 클라이언트 메서드가 조용히 가려진다.

    `get_all_named_targets` 같은 이름을 제공자 표에 실수로 넣으면 SRDF 조회가 통째로
    바뀌는데 에러는 나지 않는다.
    """
    client = pytest.importorskip('robot_control.moveit.move_group_client')
    shadowed = [name for name in _STATIC_PROVIDERS
                if hasattr(client.MoveGroupClient, name)]
    assert shadowed == [], f'MoveGroupClient 메서드를 가린다: {shadowed}'


def test_functionbay_config_declares_the_fixtures_variable() -> None:
    """설정에 없으면 코드가 아무리 맞아도 `/variables` 에 안 나온다."""
    import os

    import yaml

    path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))), 'config', 'robot_twin_panda_functionbay.yaml')
    raw: Any = yaml.safe_load(open(path, encoding='utf-8'))
    names = {v['name']: v for v in raw['variables']}

    assert 'fixtures' in names
    source = names['fixtures']['source']
    assert source['type'] == 'static'
    assert source['backend']['method'] in _STATIC_PROVIDERS
    assert names['fixtures']['staleness_ms'] is None      # 런타임 불변
