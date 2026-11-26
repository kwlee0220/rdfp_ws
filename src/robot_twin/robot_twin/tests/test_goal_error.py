"""이동 결과의 ``goal_error`` (설계서 6.7).

closed loop(JTC)에서만 싣고, 구할 수 있는 항목만 싣는다. 트윈은 이 값으로 결과를
뒤집지 않는다. ``move_linear`` 는 ``ee_pose`` 가 아니라 TF 의 tip 자세와 비교한다 —
``ee_pose`` 의 프레임은 백엔드마다 달라 고정 오프셋이 오차로 잡히기 때문이다.
"""

from __future__ import annotations

from typing import Any, Optional

import math
from types import SimpleNamespace

import pytest

pytest.importorskip('geometry_msgs')
pytest.importorskip('rclpy')

from rclpy.task import Future  # noqa: E402

from robot_twin.backends import (  # noqa: E402
    _move_linear, _move_to_joints, _move_to_named_target, _quaternion_angle
)
from robot_twin.config import OperationConfig  # noqa: E402
from robot_twin.session import SessionStore  # noqa: E402

READY = {'panda_joint1': 0.0, 'panda_joint4': -2.356}
DOWN = {'x': 1.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
POSE = {'position': {'x': 0.4, 'y': 0.0, 'z': 0.3}, 'orientation': DOWN}


def _done(result: Any = None) -> Future:
    future = Future()
    future.set_result(result)
    return future


class _Client:
    frame_id = 'panda_link0'

    def __init__(self, cached: Optional[dict] = None) -> None:
        self._cached = cached

    def move_to_joints_async(self, joints: dict, **kwargs: Any) -> Future:
        return _done()

    def move_to_named_target_async(self, name: str, **kwargs: Any) -> Future:
        return _done()

    def cached_named_target_values(self, name: str) -> Optional[dict]:
        return self._cached

    def follow_trajectory_async(self, waypoints: list, **kwargs: Any) -> Future:
        return _done(SimpleNamespace(fraction=1.0))


class _Entry:
    def __init__(self, value: dict) -> None:
        self.snapshot = object()
        self._value = value

    def value(self) -> dict:
        return self._value

    def status(self) -> SimpleNamespace:
        return SimpleNamespace(age_ms=12)


def _runtime(mode: str, joints: Optional[dict] = None,
             tip: Optional[tuple] = None) -> SimpleNamespace:
    entries = {}
    if joints is not None:
        entries['joint_states'] = _Entry({'position': joints})
    moveit = SimpleNamespace(ee_frame=None, tip_frame='panda_link8', move_group_mode=mode)
    return SimpleNamespace(
        config=SimpleNamespace(moveit=moveit),
        variables=SimpleNamespace(entry=entries.get),
        lookup_frame_pose=lambda parent, child: tip,
    )


def _session(name: str, inputs: dict) -> Any:
    return SessionStore().create(operation=name, resources=('arm',), inputs=inputs)


def _joints(runtime: Any, goal: dict) -> dict:
    op = OperationConfig(name='move_to_joints', kind='async', resource='arm',
                         backend={'method': 'move_to_joints_async'})
    return _move_to_joints(runtime, _Client(), op, _session(op.name, {'joints': goal}), 5.0)


def test_joint_error_is_the_worst_joint() -> None:
    measured = {'panda_joint1': 0.01, 'panda_joint4': -2.336, 'panda_finger_joint1': 0.04}
    outputs = _joints(_runtime('jtc', measured), READY)
    assert outputs['goal_error'] == {'joint_max_rad': pytest.approx(0.02)}


def test_open_loop_gets_no_goal_error() -> None:
    """JGPC 는 반환 시점에 아직 움직이고 있을 수 있어 그 오차는 도달을 뜻하지 않는다."""
    outputs = _joints(_runtime('jgpc', dict(READY)), READY)
    assert 'goal_error' not in outputs
    assert outputs['closed_loop'] is False


def test_missing_joint_means_no_joint_error() -> None:
    """일부 관절만으로 최댓값을 내면 빠진 관절의 오차가 숨는다."""
    outputs = _joints(_runtime('jtc', {'panda_joint1': 0.0}), READY)
    assert 'goal_error' not in outputs


def test_named_target_uses_cached_srdf_values() -> None:
    op = OperationConfig(name='move_to_named_target', kind='async', resource='arm',
                         backend={'method': 'move_to_named_target_async'})
    measured = {'panda_joint1': 0.0, 'panda_joint4': -2.306}
    outputs = _move_to_named_target(_runtime('jtc', measured), _Client(cached=READY), op,
                                    _session(op.name, {'target': 'ready'}), 5.0)
    assert outputs['goal_error'] == {'joint_max_rad': pytest.approx(0.05)}


def test_named_target_without_cache_has_no_goal_error() -> None:
    op = OperationConfig(name='move_to_named_target', kind='async', resource='arm',
                         backend={'method': 'move_to_named_target_async'})
    outputs = _move_to_named_target(_runtime('jtc', dict(READY)), _Client(cached=None), op,
                                    _session(op.name, {'target': 'ready'}), 5.0)
    assert 'goal_error' not in outputs


def _linear(runtime: Any) -> dict:
    op = OperationConfig(name='move_linear', kind='async', resource='arm',
                         backend={'method': 'follow_trajectory_async'})
    return _move_linear(runtime, _Client(), op, _session(op.name, {'pose': POSE}), 5.0)


def test_linear_compares_tip_pose_from_tf() -> None:
    # 목표에서 z 로 3 mm 위, x 축으로 0.02 rad 기울어진 자세.
    half = 0.01
    tilted = (math.cos(half), 0.0, 0.0, math.sin(half))   # DOWN 에 x 축 회전을 얹은 것
    outputs = _linear(_runtime('jtc', tip=((0.4, 0.0, 0.303), tilted)))
    error = outputs['goal_error']
    assert error['position_m'] == pytest.approx(0.003)
    assert error['orientation_rad'] == pytest.approx(0.02, abs=1e-9)
    assert outputs['fraction'] == 1.0


def test_linear_without_tf_has_no_goal_error() -> None:
    """TF 가 늦어도 이동은 성공으로 보고되고, 오차만 빠진다."""
    outputs = _linear(_runtime('jtc', tip=None))
    assert 'goal_error' not in outputs


def test_quaternion_angle_ignores_sign() -> None:
    assert _quaternion_angle((0, 0, 0, 1), (0, 0, 0, -1)) == pytest.approx(0.0)
    quarter = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert _quaternion_angle((0, 0, 0, 1), quarter) == pytest.approx(math.pi / 2)
