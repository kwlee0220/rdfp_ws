"""`move_linear` 의 계획 비율 — 입력 `min_fraction` 과 출력 `fraction`.

출력: 핸들러는 오래전부터 `fraction` 을 담도록 짜여 있었지만 클라이언트가 그 값을 버려
**한 번도 실리지 않았다.** 그래서 여기서는 가짜 결과가 아니라 실제
`CartesianPathResult` 를 돌려주는 클라이언트로 두 계층의 계약을 함께 본다.

입력: 예전에는 클라이언트 라이브러리 기본(0.60)이 그대로 쓰여, 요청 거리의 40 % 를
못 간 이동도 `COMPLETED` 였다. 트윈의 기본은 0.95 이고 호출자만 낮출 수 있다.
"""

from __future__ import annotations

from typing import Any

from types import SimpleNamespace

import pytest

pytest.importorskip('geometry_msgs')
moveit = pytest.importorskip('robot_control.moveit.move_group_client')

from rclpy.task import Future  # noqa: E402

from robot_twin.backends import _move_linear, validate_inputs  # noqa: E402
from robot_twin.config import OperationConfig  # noqa: E402
from robot_twin.session import SessionStore  # noqa: E402

OP = OperationConfig(name='move_linear', kind='async', resource='arm',
                     backend={'method': 'follow_trajectory_async'})
POSE = {'position': {'x': 0.4, 'y': 0.0, 'z': 0.3},
        'orientation': {'x': 1.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}}


class _Client:
    """받은 인자를 남기는 클라이언트 대역."""

    def __init__(self, result: Any) -> None:
        self._result = result
        self.kwargs: dict[str, Any] = {}

    def follow_trajectory_async(self, waypoints: list, **kwargs: Any) -> Future:
        self.kwargs = kwargs
        future = Future()
        future.set_result(self._result)
        return future


def _runtime(mode: str) -> Any:
    """tip 프레임만 아는 런타임 — 프레임 변환 없이 핸들러 흐름만 태운다."""
    moveit_config = SimpleNamespace(ee_frame=None, tip_frame='panda_link8',
                                    move_group_mode=mode)
    return SimpleNamespace(config=SimpleNamespace(moveit=moveit_config),
                           variables=SimpleNamespace(entry=lambda name: None))


def _run(mode: str, result: Any) -> dict:
    session = SessionStore().create(operation=OP.name, resources=('arm',),
                                    inputs={'pose': POSE})
    return _move_linear(_runtime(mode), _Client(result), OP, session, 5.0)


def _threshold_sent(**inputs: Any) -> float:
    """핸들러가 클라이언트에 넘긴 최소 계획 비율."""
    client = _Client(moveit.CartesianPathResult(fraction=1.0))
    session = SessionStore().create(operation=OP.name, resources=('arm',),
                                    inputs={'pose': POSE, **inputs})
    _move_linear(_runtime('jtc'), client, OP, session, 5.0)
    return client.kwargs['fraction_threshold']


def test_min_fraction_defaults_to_strict() -> None:
    """생략하면 0.95 다 — 라이브러리 기본 0.60 이 새어 나가면 안 된다."""
    assert _threshold_sent() == 0.95


def test_min_fraction_is_passed_through() -> None:
    assert _threshold_sent(min_fraction=0.6) == 0.6


@pytest.mark.parametrize('bad', [0, 0.0, -0.1, 1.5, True, '0.9'])
def test_min_fraction_out_of_range_is_refused_before_moving(bad: Any) -> None:
    """`jsonschema` 가 없어도 걸러야 한다 — 0 이면 한 점도 없는 경로가 `COMPLETED` 가 된다."""
    with pytest.raises(ValueError, match='min_fraction'):
        validate_inputs(OP.name, {'pose': POSE, 'min_fraction': bad})


def test_partial_path_is_reported_as_fraction() -> None:
    outputs = _run('jtc', moveit.CartesianPathResult(fraction=0.75))

    assert outputs['fraction'] == 0.75
    assert outputs['closed_loop'] is True


def test_streaming_result_reports_fraction_too() -> None:
    """JGPC 는 open loop 라 `fraction` 이 도달을 뜻하지 않는다 — 그래도 원인 진단에 쓴다."""
    outputs = _run('jgpc', moveit.CartesianPathResult(fraction=1.0, published=42))

    assert outputs['fraction'] == 1.0
    assert outputs['closed_loop'] is False
