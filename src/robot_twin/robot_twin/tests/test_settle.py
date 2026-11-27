"""이동 뒤 정착 대기 (설계서 6.7 "정착 대기").

판정 규칙(:class:`SettleDetector`), 대기 루프(:func:`wait_until_settled`), 이동 핸들러가
결과에 ``settled`` · ``settle_ms`` · ``goal_error`` 를 싣는 방식, 입력 검증과 설정 채움을
본다. ROS 없이 돈다 — 핸들러는 ``move_to_joints`` 로 시험한다(``move_linear`` 는
``geometry_msgs`` 가 필요하다).
"""

from __future__ import annotations

from typing import Any, Optional

import math
import time
from dataclasses import dataclass
from types import SimpleNamespace

import pytest

from robot_twin.backends import _move_to_joints, validate_inputs
from robot_twin.config import MoveItConfig, OperationConfig, SettleConfig
from robot_twin.session import Phase, SessionStore
from robot_twin.settle import PoseSample, SettleDetector, wait_until_settled
from robot_twin.snapshot import Quality

DOWN = (1.0, 0.0, 0.0, 0.0)
GOAL = {'panda_joint1': 0.0, 'panda_joint4': -2.356}


def _sample(x: float, t: float, *, stamp: Optional[int] = None,
            orientation: tuple = DOWN) -> PoseSample:
    return PoseSample((x, 0.0, 0.3), orientation,
                      int(t * 1e9) if stamp is None else stamp, t)


def _detector(quiet: float = 0.6) -> SettleDetector:
    return SettleDetector(position_tol_m=0.0002, orientation_tol_rad=0.002, quiet_sec=quiet)


# ---------- 판정 규칙 ----------------------------------------------------------

def test_still_for_quiet_sec_is_settled() -> None:
    det = _detector()
    assert not det.feed(_sample(0.4, 0.0))
    assert not det.feed(_sample(0.4001, 0.3))
    assert det.feed(_sample(0.40015, 0.6))


def test_moving_restarts_the_window() -> None:
    det = _detector()
    det.feed(_sample(0.400, 0.0))
    det.feed(_sample(0.401, 0.5))            # 1 mm 움직였다 — 여기서 구간을 새로 연다
    assert not det.feed(_sample(0.401, 0.9))
    assert det.feed(_sample(0.401, 1.1))


def test_rotation_alone_counts_as_motion() -> None:
    """`panda_joint7` 회전은 EE 위치를 거의 안 바꾼다 — 자세를 안 보면 정착으로 오판한다."""
    det = _detector()
    det.feed(_sample(0.4, 0.0))
    half = 0.005                              # 0.01 rad 회전
    turned = (math.cos(half), 0.0, 0.0, math.sin(half))
    assert not det.feed(_sample(0.4, 0.7, orientation=turned))


def test_republished_stamp_is_not_evidence() -> None:
    """TF 가 멈추면 같은 값을 같은 stamp 로 계속 낸다 — 그것으로 정착을 주장하면 안 된다."""
    det = _detector()
    det.feed(_sample(0.4, 0.0, stamp=100))
    for t in (0.3, 0.6, 0.9, 1.2):
        assert not det.feed(_sample(0.4, t, stamp=100))


def test_stamp_going_backwards_restarts_the_window() -> None:
    det = _detector()
    det.feed(_sample(0.4, 0.0, stamp=1000))
    assert not det.feed(_sample(0.4, 0.7, stamp=10))     # 되감겼다 — 새 구간
    assert det.feed(_sample(0.4, 1.3, stamp=20))


@pytest.mark.parametrize('field', ['position_tol_m', 'orientation_tol_rad', 'quiet_sec'])
def test_detector_rejects_non_positive_thresholds(field: str) -> None:
    kwargs = {'position_tol_m': 0.0002, 'orientation_tol_rad': 0.002, 'quiet_sec': 0.6}
    kwargs[field] = 0.0
    with pytest.raises(ValueError, match=field):
        SettleDetector(**kwargs)


# ---------- 대기 루프 ----------------------------------------------------------

@dataclass(frozen=True)
class _Snap:
    gen: int
    received_at: float
    x: float
    stamp_ns: int
    quality: Quality = Quality.OK


class _Clock:
    def __init__(self) -> None:
        self.now = 1000.0

    def sleep(self, sec: float) -> None:
        self.now += sec


class _PoseEntry:
    """`ee_pose` 캐시 슬롯 대역 — 시각에 따라 위치를 내는 함수로 스냅샷을 만든다."""

    def __init__(self, clock: _Clock, x_at, *, period: float = 0.05,
                 quality: Quality = Quality.OK, frozen_stamp: bool = False) -> None:
        self._clock = clock
        self._x_at = x_at
        self._period = period
        self._quality = quality
        self._frozen = frozen_stamp
        self._start = clock.now

    @property
    def snapshot(self) -> _Snap:
        k = int((self._clock.now - self._start) / self._period)
        t = self._start + k * self._period
        stamp = 1 if self._frozen else int(t * 1e9)
        return _Snap(k + 1, t, self._x_at(t - self._start), stamp, self._quality)

    def status(self, snap: _Snap) -> SimpleNamespace:
        return SimpleNamespace(quality=snap.quality)

    def value(self, snap: _Snap) -> dict:
        return {'pose': {'position': {'x': snap.x, 'y': 0.0, 'z': 0.3},
                         'orientation': dict(zip('xyzw', DOWN))}}

    def stamp(self, snap: _Snap) -> dict:
        return {'sec': snap.stamp_ns // 1_000_000_000, 'nanosec': snap.stamp_ns % 1_000_000_000}


def _wait(entry: _PoseEntry, clock: _Clock, *, timeout: float = 5.0, quiet: float = 0.6,
          should_stop=lambda: False):
    return wait_until_settled(entry, timeout_sec=timeout, position_tol_m=0.0002,
                              orientation_tol_rad=0.002, quiet_sec=quiet,
                              should_stop=should_stop, clock=lambda: clock.now,
                              monotonic=lambda: clock.now, sleep=clock.sleep)


def _approach(t: float) -> float:
    """1초 동안 10 mm 움직인 뒤 멈춘다."""
    return 0.4 + 0.01 * min(t, 1.0)


def test_waits_until_the_arm_stops() -> None:
    clock = _Clock()
    result = _wait(_PoseEntry(clock, _approach), clock)
    assert result.settled
    # 1.0 s 에 멈추고 quiet 0.6 s 를 채운 뒤다.
    assert 1600 <= result.settle_ms <= 1750


def test_times_out_while_still_moving() -> None:
    clock = _Clock()
    result = _wait(_PoseEntry(clock, lambda t: 0.4 + 0.01 * t), clock, timeout=2.0)
    assert not result.settled
    assert result.settle_ms is None


def test_frozen_stamp_never_settles() -> None:
    clock = _Clock()
    result = _wait(_PoseEntry(clock, lambda t: 0.4, frozen_stamp=True), clock, timeout=2.0)
    assert not result.settled


def test_stale_quality_never_settles() -> None:
    clock = _Clock()
    entry = _PoseEntry(clock, lambda t: 0.4, quality=Quality.STALE)
    assert not _wait(entry, clock, timeout=2.0).settled


def test_cancel_ends_the_wait_at_once() -> None:
    clock = _Clock()
    started = clock.now
    result = _wait(_PoseEntry(clock, lambda t: 0.4 + 0.01 * t), clock, timeout=30.0,
                   should_stop=lambda: clock.now - started > 0.2)
    assert not result.settled
    assert clock.now - started < 0.5


# ---------- 이동 핸들러 --------------------------------------------------------

class _Future:
    def done(self) -> bool:
        return True

    def exception(self) -> None:
        return None

    def result(self) -> None:
        return None


class _Client:
    def move_to_joints_async(self, joints: dict, **kwargs: Any) -> _Future:
        return _Future()


class _JointEntry:
    snapshot = object()

    def __init__(self, joints: dict) -> None:
        self._joints = joints

    def value(self) -> dict:
        return {'position': self._joints}

    def status(self) -> SimpleNamespace:
        return SimpleNamespace(age_ms=5)


class _StillPose:
    """정착을 즉시 판정하게 하는 `ee_pose` 대역 — 호출마다 새 세대 · 새 stamp 를 낸다."""

    def __init__(self) -> None:
        self._gen = 0

    @property
    def snapshot(self) -> SimpleNamespace:
        self._gen += 1
        # 실제 시각보다 앞선 수신 시각이라 대기 시작 이후 표본으로 읽힌다.
        return SimpleNamespace(gen=self._gen, received_at=time.time() + self._gen * 0.4)

    def status(self, snap: Any = None) -> SimpleNamespace:
        return SimpleNamespace(quality=Quality.OK, age_ms=3)

    def value(self, snap: Any = None) -> dict:
        return {'pose': {'position': {'x': 0.4, 'y': 0.0, 'z': 0.3},
                         'orientation': dict(zip('xyzw', DOWN))}}

    def stamp(self, snap: Any) -> dict:
        return {'sec': snap.gen, 'nanosec': 0}


def _runtime(mode: str, *, settle: SettleConfig, pose: Optional[Any] = None) -> SimpleNamespace:
    entries = {'joint_states': _JointEntry(dict(GOAL))}
    if pose is not None:
        entries['ee_pose'] = pose
    store = SessionStore()
    moveit = SimpleNamespace(ee_frame=None, tip_frame='panda_link8', move_group_mode=mode,
                             settle=settle)
    runtime = SimpleNamespace(config=SimpleNamespace(moveit=moveit),
                              variables=SimpleNamespace(entry=entries.get), sessions=store)
    return runtime


def _run(runtime: SimpleNamespace, inputs: dict) -> tuple[dict, Any]:
    op = OperationConfig(name='move_to_joints', kind='async', resource='arm',
                         backend={'method': 'move_to_joints_async'})
    session = runtime.sessions.create(operation=op.name, resources=('arm',),
                                      inputs={'joints': GOAL, **inputs})
    phases = []
    original = runtime.sessions.update

    def _update(session_id: str, **fields: Any):
        phases.append(fields.get('phase'))
        return original(session_id, **fields)

    runtime.sessions.update = _update
    outputs = _move_to_joints(runtime, _Client(), op, session, 30.0)
    return outputs, phases


def test_zero_timeout_keeps_the_immediate_result() -> None:
    """0 이면 기다리지 않는다 — `settled` 가 없고, JGPC 에는 `goal_error` 도 없다."""
    outputs, phases = _run(_runtime('jgpc', settle=SettleConfig(), pose=_StillPose()), {})
    assert 'settled' not in outputs and 'settle_ms' not in outputs
    assert 'goal_error' not in outputs
    assert phases == []


def test_jgpc_gets_goal_error_once_settled() -> None:
    outputs, phases = _run(_runtime('jgpc', settle=SettleConfig(), pose=_StillPose()),
                           {'settle_timeout_sec': 5.0})
    assert outputs['settled'] is True
    assert outputs['settle_ms'] >= 0
    assert outputs['goal_error'] == {'joint_max_rad': pytest.approx(0.0)}
    assert phases == [Phase.SETTLING]


def test_profile_default_applies_when_the_input_is_omitted() -> None:
    runtime = _runtime('jgpc', settle=SettleConfig(timeout_sec=5.0), pose=_StillPose())
    outputs, _ = _run(runtime, {})
    assert outputs['settled'] is True


def test_explicit_zero_beats_the_profile_default() -> None:
    runtime = _runtime('jgpc', settle=SettleConfig(timeout_sec=5.0), pose=_StillPose())
    outputs, _ = _run(runtime, {'settle_timeout_sec': 0})
    assert 'settled' not in outputs


def test_not_settled_drops_goal_error_even_for_jtc() -> None:
    """정착하지 못한 값은 JTC 여도 도달 정도를 말하지 못한다."""
    outputs, _ = _run(_runtime('jtc', settle=SettleConfig(), pose=None),
                      {'settle_timeout_sec': 5.0})
    assert outputs['settled'] is False
    assert 'settle_ms' not in outputs
    assert 'goal_error' not in outputs


def test_wait_never_runs_past_the_operation_limit() -> None:
    """대기는 워치독 시한 안에서만 한다 — 시한을 다 쓴 세션은 기다리지 않는다."""
    runtime = _runtime('jgpc', settle=SettleConfig(), pose=_StillPose())
    op = OperationConfig(name='move_to_joints', kind='async', resource='arm',
                         backend={'method': 'move_to_joints_async'})
    session = runtime.sessions.create(operation=op.name, resources=('arm',),
                                      inputs={'joints': GOAL, 'settle_timeout_sec': 5.0})
    session.started_at -= 30.0
    outputs = _move_to_joints(runtime, _Client(), op, session, 30.0)
    assert outputs['settled'] is False


# ---------- 입력 검증 · 설정 ---------------------------------------------------

@pytest.mark.parametrize('value', [-1, float('nan'), True, '5'])
def test_invalid_settle_timeout_is_rejected(value: Any) -> None:
    with pytest.raises(ValueError, match='settle_timeout_sec'):
        validate_inputs('move_to_joints', {'joints': GOAL, 'settle_timeout_sec': value})


@pytest.mark.parametrize('value', [0, 2.5])
def test_valid_settle_timeout_is_accepted(value: float) -> None:
    validate_inputs('move_to_joints', {'joints': GOAL, 'settle_timeout_sec': value})


def test_settle_defaults_do_not_wait() -> None:
    cfg = MoveItConfig(move_group_mode='jtc')
    assert cfg.settle.timeout_sec == 0.0


def test_explicit_settle_block_is_validated() -> None:
    with pytest.raises(ValueError):
        MoveItConfig(move_group_mode='jtc', settle={'quiet_sec': 0})


def test_functionbay_profile_fills_the_settle_block() -> None:
    """정착 기준은 백엔드의 성질이라 프로파일에서 온다 (펑션베이는 씬별로 둔다)."""
    pytest.importorskip('robot_control.backend_profiles')
    from robot_control.backend_profiles import BACKEND_PROFILES

    expected = BACKEND_PROFILES['panda_functionbay']['motion']['settle']
    cfg = MoveItConfig(backend='panda_functionbay')
    assert cfg.settle.quiet_sec == pytest.approx(expected['quiet_sec'])
    assert cfg.settle.timeout_sec == 0.0


def test_backend_without_settle_keeps_the_code_default() -> None:
    pytest.importorskip('robot_control.backend_profiles')
    assert MoveItConfig(backend='panda_isaac').settle == SettleConfig()
