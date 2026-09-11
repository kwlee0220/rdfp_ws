#!/usr/bin/env python3

"""그리퍼 **규약** — `/input/gripper_joint` 배열을 어떻게 채우는가.

    from robot_control.gripper.profile import load_gripper_profile

    profile = load_gripper_profile('functionbay')
    profile.command(0.725)        # 심볼이 아니라 스칼라 s [rad] 를 받는다

그 토픽은 `Float64MultiArray` 라 이름이 없고 **배열 순서가 곧 계약**인데, 그 순서와
단위가 씬마다 다르다. 같은 펑션베이 백엔드 안에서도 `t1`(Crisp)은 6관절·라디안·위치
나열이고 `r2`(RecurDyn)은 2관절·**도**·관절당 `(p, v, f)` 다.

⚠️ **틀리면 에러가 아니라 "그리퍼가 거의 안 움직인다"** 로 나타난다. 라디안을 도로
읽는 씬에 라디안을 보내면 `π/180 = 1.745%` 만 움직인다 — 2026-09-10 에 실제로 그랬다.
그래서 `axis_count` 를 선언하고 :meth:`GripperProfile.verify_report` 가 실제 보고와
대조한다. 단위와 packing 은 보고에서 읽을 수 없어 선언에 의존한다.

**이 모듈은 `rclpy` 를 import 하지 않는다** — 값만 읽는 도구에서도 쓸 수 있어야 한다.
"""

from __future__ import annotations

from typing import Optional

import json
import math
import os

from robot_control.backend_profiles import BACKEND_PROFILES

# 지원하는 규약. 모르는 값은 **거부한다** — 조용히 기본값으로 떨어지면 그 순간
# 명령이 통째로 잘못 해석된다.
_UNITS = {'rad': 1.0, 'deg': 180.0 / math.pi}
_PACKINGS = ('position', 'pvf')


class GripperProfile:
    """한 씬의 그리퍼 명령 규약."""

    def __init__(self, raw: dict, path: str) -> None:
        self.path = path
        self.solver: Optional[str] = raw.get('solver')
        self.scene: Optional[str] = raw.get('scene')
        self.measured_on: Optional[str] = raw.get('measured_on')
        self.build: Optional[str] = raw.get('build')

        self.axis_signs: list = [float(v) for v in raw['axis_signs']]
        # **선언된 축 수가 부호 개수와 다르면 거부한다.** 둘이 어긋난 파일은 어느 쪽이
        # 맞는지 알 수 없고, 그대로 쓰면 배열 길이가 틀린 명령을 내보낸다.
        self.axis_count: int = int(raw.get('axis_count', len(self.axis_signs)))
        if self.axis_count != len(self.axis_signs):
            raise ValueError(f'{path}: axis_count={self.axis_count} 인데 axis_signs 는 '
                             f'{len(self.axis_signs)}개다')

        self.command_units: str = str(raw.get('command_units', 'rad'))
        if self.command_units not in _UNITS:
            raise ValueError(f'{path}: command_units={self.command_units!r} 를 모른다 '
                             f'(아는 것: {sorted(_UNITS)})')
        self.command_packing: str = str(raw.get('command_packing', 'position'))
        if self.command_packing not in _PACKINGS:
            raise ValueError(f'{path}: command_packing={self.command_packing!r} 를 모른다 '
                             f'(아는 것: {list(_PACKINGS)})')

        self.targets: dict = {str(k): float(v) for k, v in (raw.get('targets') or {}).items()}
        self.gain_kp: Optional[float] = _optional_float(raw.get('gain_kp'))
        self.stall_effort: float = float(raw.get('stall_effort', 0.0))
        self.stall_velocity: float = float(raw.get('stall_velocity', 0.0))
        self.position_tolerance: float = float(raw.get('position_tolerance', 0.005))
        # 목표까지 **몇 초에 걸쳐 나눠 보낼 것인가.** 0 이면 한 번에 던진다(계단).
        # 시뮬레이터의 그리퍼 서보는 계단 지령을 자기 최대 속도로 쫓아가므로, 천천히
        # 물리려면 지령 자체를 나눠 보내야 한다 — `fb_gripper.set_span` 이 쓰던 방식이다.
        self.command_ramp_sec: float = float(raw.get('command_ramp_sec', 0.0))

    def __repr__(self) -> str:
        return (f'GripperProfile(solver={self.solver!r}, scene={self.scene!r}, '
                f'axes={self.axis_count}, units={self.command_units!r}, '
                f'packing={self.command_packing!r})')

    def joint_targets(self, scalar: float) -> list:
        """구동 스칼라 `s` [rad] 를 **관절별 목표각 [rad]** 으로 펼친다."""
        return [scalar * sign for sign in self.axis_signs]

    def command(self, scalar: float) -> list:
        """구동 스칼라 `s` [rad] 를 **그대로 발행할 배열**로 바꾼다.

        단위 환산과 packing 이 여기서 일어난다. `pvf` 는 관절당 `(위치, 속도, 힘)` 이며
        속도·힘은 0 으로 둔다 — 위치 지령만 쓰고, 힘 슬롯에 0 이 아닌 값을 넣으면
        그리퍼가 물체를 밀어붙인다.

        Raises:
            ValueError: 유한하지 않은 값이 섞였을 때. **NaN 을 내보내면 그리퍼만이
                아니라 전 관절이 발산하고 되돌아오지 않는다** — 복구는 시뮬레이터 쪽
                브리지 재시작뿐이다 (2026-09-09 실측).
        """
        values = [v * _UNITS[self.command_units] for v in self.joint_targets(scalar)]
        if not all(math.isfinite(v) for v in values):
            raise ValueError(f'{self.path}: refusing a non-finite gripper command {values}')
        if self.command_packing == 'pvf':
            return [v for value in values for v in (value, 0.0, 0.0)]
        return values

    def scalar_from_report(self, positions) -> float:
        """보고된 관절값에서 구동 스칼라 `s` 를 되짚는다.

        **보고는 어느 규약에서도 라디안이다** — 도로 바꾸는 것은 명령 방향뿐이다.
        """
        if len(positions) != self.axis_count:
            return float('nan')
        return sum(p * s for p, s in zip(positions, self.axis_signs)) / self.axis_count

    def verify_report(self, positions) -> None:
        """실제 보고와 선언이 맞는지 본다. **어긋나면 거부한다.**

        씬을 바꾸고 이 파일을 안 바꿨을 때 **가장 먼저 틀리는 값이 축 수**다. 경고만
        하고 넘어가면 명령이 통째로 잘못 해석된 채 "거의 안 움직인다"로만 보인다.
        """
        if len(positions) != self.axis_count:
            raise ValueError(
                f'{self.path}: declares axis_count={self.axis_count} but the simulator '
                f'reports {len(positions)} — this profile does not match the running '
                f'scene (declared solver {self.solver!r}, scene {self.scene!r}). '
                f'Point gripper.profile_file at the right file.')


def _optional_float(value) -> Optional[float]:
    return None if value is None else float(value)


def gripper_profile_path(backend: str) -> Optional[str]:
    """프로파일의 `gripper.profile_file` 을 절대 경로로. 없으면 `None`."""
    from robot_control.backends import get_backend

    return get_backend(backend).share_path(
        (BACKEND_PROFILES[backend].get('gripper') or {}).get('profile_file'))


def load_gripper_profile(backend: str) -> GripperProfile:
    """백엔드의 그리퍼 규약을 읽는다.

    Raises:
        FileNotFoundError: 키가 없거나 파일이 없을 때. **빈 것을 돌려주지 않는다** —
            그러면 "규약이 필요 없는 백엔드"와 "아직 안 적은 백엔드"가 구별되지 않고,
            기본값으로 조용히 진행돼 명령이 잘못 해석된다.
    """
    path = gripper_profile_path(backend)
    if not path:
        raise FileNotFoundError(
            f'{backend}: 프로파일에 gripper.profile_file 이 없다 — 그리퍼 규약이 '
            '따로 필요 없는 백엔드이거나 아직 안 적은 것이다')
    if not os.path.isfile(path):
        raise FileNotFoundError(f'{backend}: {path} 가 없다 (colcon build 를 다시 돌렸나?)')
    profile = load_gripper_profile_file(path)
    _verify_solver(backend, profile)
    return profile


def _verify_solver(backend: str, profile: GripperProfile) -> None:
    """백엔드가 선언한 솔버와 파일의 솔버가 같은지 본다.

    한쪽만 고치는 실수를 **로드 시점에** 잡는다 — 런타임까지 가면 증상이 "그리퍼가
    거의 안 움직인다" 뿐이라 원인에 도달하기 어렵다.
    """
    declared = BACKEND_PROFILES[backend].get('solver')
    if declared and profile.solver and declared != profile.solver:
        raise ValueError(
            f'{profile.path}: solver={profile.solver!r} 인데 백엔드 {backend!r} 는 '
            f'{declared!r} 라고 선언한다 — 둘 중 하나가 낡았다')


def load_gripper_profile_file(path: str) -> GripperProfile:
    """파일 하나를 읽는다. 프로파일을 거치지 않으므로 테스트·도구가 쓴다."""
    with open(path, encoding='utf-8') as handle:
        return GripperProfile(json.load(handle), path)


def describe(backend: str) -> str:
    """사람이 읽을 요약."""
    profile = load_gripper_profile(backend)
    lines = [f'{backend} 그리퍼 규약 — 솔버 {profile.solver or "미기재"} · '
             f'씬 {profile.scene or "미기재"} (측정 {profile.measured_on or "미기재"})',
             f'  축 {profile.axis_count}개, 부호 {profile.axis_signs}',
             f'  명령 단위 {profile.command_units} · packing {profile.command_packing}',
             f'  targets {profile.targets}',
             f'  출처: {profile.path}']
    example = profile.targets.get('close')
    if example is not None:
        lines.append(f'  close({example}) → {[round(v, 4) for v in profile.command(example)]}')
    return '\n'.join(lines)


__all__ = ['GripperProfile', 'describe', 'gripper_profile_path', 'load_gripper_profile',
           'load_gripper_profile_file']
