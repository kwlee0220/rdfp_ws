"""이동 뒤 정착 대기 (설계서 6.7 "정착 대기").

실행이 끝난 뒤 ``ee_pose`` 가 멈출 때까지 기다린다. JGPC 는 명령 발행이 끝난 시점에
결과를 만들고 그때 팔이 아직 움직이고 있을 수 있어, 그 시점의 오차는 도달 정도를
말하지 못한다. 정착을 판정한 뒤 재면 두 스택(JTC · JGPC)의 ``goal_error`` 가 같은 뜻
(정착 후 잔차)을 갖는다.

판정 규칙은 클라이언트의 ``wait_until_still`` 과 같다.

1. 실행 종료 뒤 **새로 수신한** 표본만 쓴다 — 캐시에 남아 있던 종료 이전 표본은 근거가
   되지 않는다.
2. ``header.stamp`` 가 증가하지 않은 표본(같은 원본의 재발행)은 근거가 아니다 — TF 가
   멈추면 ``ee_pose`` 는 같은 값을 같은 stamp 로 계속 낸다(sim 시계 되감김).
   stamp 가 거꾸로 가면 구간을 새로 시작한다.
3. 품질이 ``OK`` 가 아닌 표본에서는 구간을 새로 시작한다.
4. 위치와 자세가 구간 시작 표본의 허용량 안에 ``quiet_sec`` 동안 머물면 정착이다.
   자세도 보는 이유는 ``panda_joint7`` 처럼 EE 위치를 거의 바꾸지 않는 회전이 있기
   때문이다.

판정(:class:`SettleDetector`)은 ROS 를 모른다 — 대기 루프(:func:`wait_until_settled`)만
변수 캐시를 읽는다.
"""

from __future__ import annotations

from typing import Any, Callable, Optional

import math
import time
from dataclasses import dataclass

# 대기 루프의 폴링 주기(초). `_await` 와 같은 값이다.
_POLL_SEC = 0.02


@dataclass(frozen=True)
class PoseSample:
    """정착 판정에 쓰는 표본 하나."""

    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    # 원본 ``header.stamp`` (ns). 헤더가 없으면 ``None`` 이고 stamp 검사를 건너뛴다.
    stamp_ns: Optional[int]
    # 트윈이 수신한 벽시계 시각(초).
    received_at: float


@dataclass(frozen=True)
class SettleResult:
    """정착 대기 결과."""

    settled: bool
    # 대기 시작(실행 종료)부터 정착 판정까지(ms). 정착하지 못했으면 ``None``.
    settle_ms: Optional[int] = None


def orientation_angle(a: tuple, b: tuple) -> float:
    """두 쿼터니언 사이의 회전각(rad, 0~π). 부호가 반대인 같은 회전은 0 이다."""
    norm_a = math.sqrt(sum(x * x for x in a))
    norm_b = math.sqrt(sum(x * x for x in b))
    if norm_a == 0.0 or norm_b == 0.0:
        return math.pi
    dot = abs(sum(x * y for x, y in zip(a, b))) / (norm_a * norm_b)
    return 2.0 * math.acos(min(1.0, dot))


class SettleDetector:
    """표본을 받아 정착 여부를 판정한다.

    구간 시작 표본(anchor)에서 위치 ``position_tol_m`` · 자세 ``orientation_tol_rad``
    안에 ``quiet_sec`` 동안 머물면 정착이다. 벗어나면 그 표본이 새 anchor 가 된다.
    """

    def __init__(self, *, position_tol_m: float, orientation_tol_rad: float,
                 quiet_sec: float) -> None:
        for name, value in (('position_tol_m', position_tol_m),
                            ('orientation_tol_rad', orientation_tol_rad),
                            ('quiet_sec', quiet_sec)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive, got {value!r}')
        self._position_tol = position_tol_m
        self._orientation_tol = orientation_tol_rad
        self._quiet = quiet_sec
        self._anchor: Optional[PoseSample] = None
        self._last_stamp: Optional[int] = None

    def reset(self) -> None:
        """구간을 버린다 — 무효 표본을 받았을 때 부른다."""
        self._anchor = None

    def feed(self, sample: PoseSample) -> bool:
        """표본 하나를 넣고 정착했으면 ``True`` 를 돌려준다."""
        if sample.stamp_ns is not None and self._last_stamp is not None:
            if sample.stamp_ns == self._last_stamp:
                # 같은 원본의 재발행이다 — 정착 근거로 세지 않는다.
                return False
            if sample.stamp_ns < self._last_stamp:
                self._last_stamp = sample.stamp_ns
                self._anchor = sample
                return False
        if sample.stamp_ns is not None:
            self._last_stamp = sample.stamp_ns

        anchor = self._anchor
        if anchor is None or not self._within(anchor, sample):
            self._anchor = sample
            return False
        return sample.received_at - anchor.received_at >= self._quiet

    def _within(self, anchor: PoseSample, sample: PoseSample) -> bool:
        if math.dist(anchor.position, sample.position) > self._position_tol:
            return False
        return orientation_angle(anchor.orientation, sample.orientation) <= self._orientation_tol


def pose_sample(entry: Any, snapshot: Any) -> Optional[PoseSample]:
    """``ee_pose`` 변수의 스냅샷 하나를 표본으로 바꾼다. 쓸 수 없으면 ``None``."""
    from robot_twin.snapshot import Quality

    if entry.status(snapshot).quality is not Quality.OK:
        return None
    try:
        value = entry.value(snapshot)
        pose = value['pose']
        p, q = pose['position'], pose['orientation']
        position = (float(p['x']), float(p['y']), float(p['z']))
        orientation = (float(q['x']), float(q['y']), float(q['z']), float(q['w']))
    except Exception:       # noqa: BLE001 — 형식이 다르면 판정 근거가 아니다
        return None
    if not all(math.isfinite(v) for v in position + orientation):
        return None
    stamp = entry.stamp(snapshot)
    stamp_ns = None if stamp is None else int(stamp['sec']) * 1_000_000_000 + int(
        stamp['nanosec'])
    return PoseSample(position, orientation, stamp_ns, float(snapshot.received_at))


def wait_until_settled(entry: Any, *, timeout_sec: float, position_tol_m: float,
                       orientation_tol_rad: float, quiet_sec: float,
                       should_stop: Callable[[], bool] = lambda: False,
                       clock: Callable[[], float] = time.time,
                       monotonic: Callable[[], float] = time.monotonic,
                       sleep: Callable[[float], None] = time.sleep) -> SettleResult:
    """``entry``(``ee_pose``)가 멈출 때까지 최대 ``timeout_sec`` 기다린다.

    Args:
        entry: ``ee_pose`` 변수의 캐시 슬롯(:class:`robot_twin.variables.VariableEntry`).
        timeout_sec: 최대 대기 시간(초). 0 이하면 기다리지 않고 정착하지 않은 것으로 끝낸다.
        should_stop: ``True`` 를 돌려주면 대기를 바로 끝낸다 — 취소 · 세션 종료.
        clock: 수신 시각(``received_at``)과 같은 벽시계.

    Returns:
        정착했으면 ``settled=True`` 와 걸린 시간.
    """
    detector = SettleDetector(position_tol_m=position_tol_m,
                              orientation_tol_rad=orientation_tol_rad, quiet_sec=quiet_sec)
    started_wall = clock()
    deadline = monotonic() + max(0.0, timeout_sec)
    last_gen = None
    while monotonic() < deadline and not should_stop():
        snapshot = entry.snapshot
        if (snapshot is not None and snapshot.gen != last_gen
                and snapshot.received_at >= started_wall):
            last_gen = snapshot.gen
            sample = pose_sample(entry, snapshot)
            if sample is None:
                detector.reset()
            elif detector.feed(sample):
                return SettleResult(True, max(0, round((clock() - started_wall) * 1000.0)))
        sleep(_POLL_SEC)
    return SettleResult(False)


__all__ = ['PoseSample', 'SettleDetector', 'SettleResult', 'orientation_angle', 'pose_sample',
           'wait_until_settled']
