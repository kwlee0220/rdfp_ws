"""변수 메타의 시계 기준 ``stamp_clock`` (설계서 4.2).

응답 ``stamp`` 의 시계는 백엔드마다 다르다(Isaac 은 시뮬레이션 시각, 펑션베이는 벽시계). 카탈로그가 그것을
알려 줘야 클라이언트가 ``stamp`` 를 다른 시각과 비교해도 되는지 안다. 값은 **선언값**이다 — 트윈 설정이
채우고, 발행 노드가 다른 시계를 쓰는 변수만 YAML 에 적는다.

모듈 최상단에서 ``importorskip`` 하지 않는다 — 이 디렉터리에서는 그 skip 이 수집 전체를 끝낸다(test_api 참고).
"""

from __future__ import annotations

from typing import Any, Optional

from pathlib import Path

import pytest

from robot_twin.config import TwinConfig, load_config

_TWINS = Path(__file__).resolve().parents[2] / 'config' / 'twins'


def _config(*, use_sim_time: bool, variables: list[dict]) -> TwinConfig:
    return TwinConfig.model_validate({
        'twin': {'id': 't1'},
        'ros': {'use_sim_time': use_sim_time},
        'moveit': {'move_group_mode': 'jtc'},
        'variables': variables,
    })


def _topic(name: str, stamp_clock: Optional[str] = None) -> dict[str, Any]:
    var: dict[str, Any] = {'name': name, 'source': {'type': 'topic', 'topic': f'/{name}',
                                                    'msg': 'sensor_msgs/msg/JointState'}}
    if stamp_clock is not None:
        var['stamp_clock'] = stamp_clock
    return var


def _static(name: str, stamp_clock: Optional[str] = None) -> dict[str, Any]:
    var: dict[str, Any] = {'name': name, 'source': {'type': 'static',
                                                    'backend': {'method': 'get_all_named_targets'}}}
    if stamp_clock is not None:
        var['stamp_clock'] = stamp_clock
    return var


@pytest.mark.parametrize('use_sim_time, expected', [(True, 'sim'), (False, 'wall')])
def test_topic_default_follows_use_sim_time(use_sim_time: bool, expected: str) -> None:
    cfg = _config(use_sim_time=use_sim_time, variables=[_topic('joint_states')])
    assert cfg.variable('joint_states').stamp_clock == expected


def test_declared_clock_wins() -> None:
    """발행 노드가 트윈과 다른 시계로 찍는 변수만 적는다."""
    cfg = _config(use_sim_time=True, variables=[_topic('raw', 'wall')])
    assert cfg.variable('raw').stamp_clock == 'wall'


def test_static_source_has_no_stamp() -> None:
    cfg = _config(use_sim_time=True, variables=[_static('named_targets')])
    assert cfg.variable('named_targets').stamp_clock == 'none'


def test_static_source_rejects_a_clock() -> None:
    with pytest.raises(ValueError, match='static source has no stamp'):
        _config(use_sim_time=False, variables=[_static('named_targets', 'wall')])


def test_unknown_clock_is_rejected() -> None:
    with pytest.raises(ValueError):
        _config(use_sim_time=False, variables=[_topic('joint_states', 'gps')])


def test_catalog_reports_the_clock_and_null_for_no_stamp() -> None:
    pytest.importorskip('fastapi')
    from robot_twin.api import variable_catalog

    cfg = _config(use_sim_time=True, variables=[_topic('joint_states'), _static('named_targets')])
    catalog = {v['name']: v for v in variable_catalog(cfg)}
    assert catalog['joint_states']['stamp_clock'] == 'sim'
    assert catalog['named_targets']['stamp_clock'] is None


@pytest.mark.parametrize('twin, expected', [('panda01', 'sim'), ('rdfp01', 'wall'), ('mock01', 'wall')])
def test_shipped_twins_resolve_topic_clocks(twin: str, expected: str) -> None:
    """출하 트윈은 예외 선언 없이 백엔드 시계를 따른다 — Isaac 은 발행 노드가 전부 `use_sim_time` 을 받는다."""
    pytest.importorskip('robot_control.backend_profiles')
    cfg = load_config(_TWINS / f'{twin}.yaml')
    for var in cfg.variables:
        assert var.stamp_clock == ('none' if var.source.type == 'static' else expected), var.name


# ---------- 기동 — 헤더 없는 타입은 none 으로 바로잡는다 ----------------------

class _Logger:
    def __init__(self) -> None:
        self.infos: list[str] = []

    def info(self, msg: str) -> None:
        self.infos.append(msg)

    def warning(self, msg: str) -> None:
        pass

    def error(self, msg: str) -> None:
        pass


class _Node:
    def __init__(self) -> None:
        self.logger = _Logger()

    def get_logger(self) -> _Logger:
        return self.logger

    def create_subscription(self, *args: Any, **kwargs: Any) -> object:
        return object()

    def count_publishers(self, topic: str) -> int:
        return 1


@pytest.mark.parametrize('msg, expected', [('std_msgs/msg/String', 'none'),
                                           ('sensor_msgs/msg/JointState', 'wall')])
def test_headerless_message_type_is_corrected_at_start(msg: str, expected: str) -> None:
    pytest.importorskip('rclpy')
    pytest.importorskip('std_msgs')
    from robot_twin.variables import VariableCache

    cfg = _config(use_sim_time=False, variables=[{
        'name': 'v', 'source': {'type': 'topic', 'topic': '/v', 'msg': msg}}])
    node = _Node()
    VariableCache(node, cfg.variables).start()
    assert cfg.variable('v').stamp_clock == expected
    assert any('has no header' in m for m in node.logger.infos) is (expected == 'none')
