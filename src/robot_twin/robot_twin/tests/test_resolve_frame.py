"""`_resolve_frame` — EE 프레임 목표를 tip link 지령으로 옮기는 변환.

**이 변환이 없거나 방향이 틀리면 에러가 아니라 "엉뚱한 곳으로 간다"** 로 나타난다
(펑션베이 149 mm, mock·Isaac 107 mm). 그래서 여기서 확인하는 것은 세 가지다 —
기본이 EE 프레임인가, tip 을 명시하면 손대지 않는가, 모르는 이름을 거절하는가.
"""

from __future__ import annotations

from typing import Any, Optional

import math

import pytest

from robot_twin.backends import _resolve_frame

# grasp_center 실측 오프셋 (2026-09-11, `panda_link8` -> `grasp_center`):
# 평행이동 149.3 mm, 회전은 z 축 90°.
OFFSET = ((0.0, 0.0003, 0.14927),
          (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)))

POSE = {'position': {'x': 0.4, 'y': 0.1, 'z': 0.3},
        'orientation': {'x': 1.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}}


class _FakeMoveIt:
    def __init__(self, ee: Optional[str], tip: str = 'panda_link8') -> None:
        self.ee_frame = ee
        self.tip_frame = tip


class _FakeConfig:
    def __init__(self, moveit: Any) -> None:
        self.moveit = moveit


class _FakeRuntime:
    """`lookup_tip_to_ee` 만 갖는 최소 런타임. TF 는 값으로 주입한다."""

    def __init__(self, ee: Optional[str], offset: Any = OFFSET) -> None:
        self.config = _FakeConfig(_FakeMoveIt(ee))
        self._offset = offset
        self.lookups = 0

    def lookup_tip_to_ee(self, timeout_sec: float = 2.0) -> Any:
        self.lookups += 1
        return self._offset


def _distance(a: dict, b: dict) -> float:
    return math.sqrt(sum((a['position'][k] - b['position'][k]) ** 2 for k in 'xyz'))


def test_tip_frame_is_passed_through_untouched() -> None:
    """tip 을 명시하면 **기존 동작 그대로** — TF 조회조차 하지 않는다."""
    runtime = _FakeRuntime('grasp_center')
    assert _resolve_frame(runtime, POSE, 'panda_link8') is POSE
    assert runtime.lookups == 0


def test_default_frame_is_the_ee_frame() -> None:
    """`frame` 을 생략하면 EE 기준이다 — `ee_pose` 를 그대로 넣는 것이 흔한 사용이라서다.

    ⚠️ 이것이 **기본값**인 것이 중요하다. tip 이 기본이면 `ee_pose` 를 읽어 그대로
    넣는 호출자가 149 mm 빗나가는데, 그 증상은 에러를 내지 않는다.
    """
    runtime = _FakeRuntime('grasp_center')
    out = _resolve_frame(runtime, POSE, None)
    assert runtime.lookups == 1
    assert _distance(out, POSE) == pytest.approx(0.14927, abs=1e-5)


def test_explicit_ee_frame_matches_the_default() -> None:
    runtime = _FakeRuntime('grasp_center')
    assert _resolve_frame(runtime, POSE, 'grasp_center') == _resolve_frame(runtime, POSE, None)


def test_unknown_frame_is_rejected() -> None:
    """변환할 수 없는 이름을 조용히 통과시키면 다시 149 mm 문제가 된다."""
    runtime = _FakeRuntime('grasp_center')
    with pytest.raises(ValueError, match='unknown frame'):
        _resolve_frame(runtime, POSE, 'panda_link0')


def test_frame_is_rejected_when_the_twin_has_no_ee_frame() -> None:
    """EE 프레임을 선언하지 않은 스택에서 EE 이름을 주면 **거절한다.**

    예전 `frame_id` 입력이 그랬듯 조용히 무시하면 "설정했는데 안 먹는다" 가 된다.
    """
    runtime = _FakeRuntime(None)
    with pytest.raises(ValueError, match='unknown frame'):
        _resolve_frame(runtime, POSE, 'grasp_center')
    assert _resolve_frame(runtime, POSE, None) is POSE


def test_ee_equal_to_tip_is_identity() -> None:
    """둘이 같은 스택은 변환이 항등이라 TF 를 보지 않는다."""
    runtime = _FakeRuntime('panda_link8')
    assert _resolve_frame(runtime, POSE, None) is POSE
    assert runtime.lookups == 0
