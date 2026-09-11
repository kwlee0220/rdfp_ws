#!/usr/bin/env python3

"""중력 보상 — **지령에 `τ_g(q)/Kp` 를 미리 더해 처짐을 없앤다.**

    from robot_control.gravity import load_gravity_compensator

    comp = load_gravity_compensator('functionbay')
    q_cmd = comp.compensate(q_desired)      # 이 값을 발행한다

펑션베이의 관절 제어는 **중력 보상이 없는 유한 강성 위치 제어**다. 그래서 정상상태에서
제어 토크와 중력 토크가 균형을 이루는 지점에 선다::

    Kp · (지령 − 실제) + τ_g(q) = 0      →      실제 − 지령 = τ_g(q) / Kp

`Kp` 는 씬 XML 이 알려 준다 (`*__Manipulator_1_vm.xml` 의 `<Gain Kp=...>`; `t1`·`r2`
모두 **2000**). 그러므로 **지령에 `τ_g/Kp` 를 더하면 실제가 원하던 자리에 선다.**

실측 대조 (2026-09-11, `RecurDyn-260907` · `r2`, `ready` 자세, 단위 mrad)::

              j1     j2     j3      j4     j5     j6     j7
    예측    0.00   0.89   0.49   -9.67  -0.70  -0.80   0.00
    실측    0.00   0.90   0.50   -9.70  -0.70  -0.80   0.00

**일곱 관절이 0.03 mrad 안에서 맞는다.** 처짐은 잡음이 아니라 계산 가능한 함수다.

한계 셋
-------

- **부하는 모델에 없다.** 물체를 쥐어도 실측 차이가 작았다(손끝 수평 −7.22 대 −7.14 mm)
  ㅡ 지금 대상(16 mm peg)에서는 무시한다. 무거운 물체를 들면 다시 봐야 한다.
- **속도 성분은 안 잡는다.** 이동 중 오차가 정지 대비 커지는 몫은 `Kd` 항이며 여기서
  다루지 않는다 (실측: j2 정지 12.7 → 이동 중 20.2 mrad).
- **시뮬레이터가 중력 보상을 넣으면 이중 보정이 된다** (벤더 A-1). 그때는 프로파일의
  `motion.gravity_compensation` 을 지운다 — 켜고 끄는 것이 설정 한 줄이어야 하는 이유다.

**이 모듈은 `rclpy` 를 import 하지 않는다** — 값만 다루는 경로에서도 쓸 수 있어야 한다.
"""

from __future__ import annotations

from typing import Optional

import math
import os

import numpy as np

from robot_control.backend_profiles import BACKEND_PROFILES

GRAVITY = np.array([0.0, 0.0, -9.81])


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_rotation(axis, angle: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(a))
    if norm == 0.0:
        return np.eye(3)
    a = a / norm
    skew = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


class GravityCompensator:
    """URDF 의 질량·COM 으로 `τ_g(q)` 를 풀고 `Δq = τ_g/Kp` 를 준다."""

    def __init__(self, urdf_path: str, joint_names, kp, *, root_link: Optional[str] = None,
                 scale: float = 1.0) -> None:
        from urdf_parser_py.urdf import URDF

        self.path = urdf_path
        self.joint_names: list = list(joint_names)
        if not self.joint_names:
            raise ValueError('joint_names must not be empty')
        self.scale = float(scale)

        kp_list = [float(kp)] * len(self.joint_names) if np.isscalar(kp) else \
            [float(v) for v in kp]
        if len(kp_list) != len(self.joint_names):
            raise ValueError(f'kp has {len(kp_list)} entries but there are '
                             f'{len(self.joint_names)} joints')
        if any(v <= 0.0 for v in kp_list):
            raise ValueError(f'every kp must be > 0, got {kp_list}')
        self.kp = np.array(kp_list)

        self._robot = URDF.from_xml_file(urdf_path)
        self._links = {link.name: link for link in self._robot.links}
        self._joints = {joint.name: joint for joint in self._robot.joints}
        missing = [n for n in self.joint_names if n not in self._joints]
        if missing:
            raise ValueError(f'{urdf_path}: unknown joint(s) {missing}')

        self._children: dict = {}
        for joint in self._robot.joints:
            self._children.setdefault(joint.parent, []).append(joint)
        children_names = {joint.child for joint in self._robot.joints}
        roots = [name for name in self._links if name not in children_names]
        self.root_link = root_link or (roots[0] if roots else None)
        if self.root_link is None:
            raise ValueError(f'{urdf_path}: cannot determine the root link')
        # **가지마다 미리 모아 둔다** — 매 호출 재귀하면 스트리밍 주기에 못 맞춘다.
        self._below = {name: self._collect_below(name) for name in self.joint_names}

    def __repr__(self) -> str:
        return (f'GravityCompensator({os.path.basename(self.path)!r}, '
                f'joints={len(self.joint_names)}, kp={self.kp[0]:g}, scale={self.scale:g})')

    def _collect_below(self, joint_name: str) -> list:
        """그 관절보다 **아래쪽 모든 링크**. 그리퍼 가지도 포함해야 질량이 안 빠진다."""
        out, stack = [], [self._joints[joint_name].child]
        while stack:
            link = stack.pop()
            out.append(link)
            stack.extend(joint.child for joint in self._children.get(link, []))
        return out

    def _forward(self, q) -> tuple:
        """링크별 (위치, 회전) 과 대상 관절의 (원점, 축) — 전부 root 기준."""
        values = dict(zip(self.joint_names, (float(v) for v in q)))
        frames = {self.root_link: (np.zeros(3), np.eye(3))}
        axes: dict = {}
        stack = [self.root_link]
        while stack:
            parent = stack.pop()
            p_parent, r_parent = frames[parent]
            for joint in self._children.get(parent, []):
                origin = joint.origin
                xyz = np.array(origin.xyz, dtype=float) if origin else np.zeros(3)
                rpy = np.array(origin.rpy, dtype=float) if origin else np.zeros(3)
                position = p_parent + r_parent @ xyz
                rotation = r_parent @ _rpy_to_matrix(*rpy)
                if joint.type in ('revolute', 'continuous') and joint.axis is not None:
                    world_axis = rotation @ np.array(joint.axis, dtype=float)
                    if joint.name in values:
                        axes[joint.name] = (position.copy(), world_axis)
                        rotation = rotation @ _axis_rotation(joint.axis, values[joint.name])
                    # 대상 밖의 회전 관절(그리퍼 등)은 0 으로 둔다.
                frames[joint.child] = (position, rotation)
                stack.append(joint.child)
        return frames, axes

    def torque(self, q) -> np.ndarray:
        """자세 `q` 에서의 중력 토크 `τ_g` (N·m), 관절 순서는 `joint_names`."""
        if len(q) != len(self.joint_names):
            raise ValueError(f'q has {len(q)} entries, expected {len(self.joint_names)}')
        frames, axes = self._forward(q)
        tau = np.zeros(len(self.joint_names))
        for index, name in enumerate(self.joint_names):
            origin, axis = axes[name]
            moment = np.zeros(3)
            for link_name in self._below[name]:
                link = self._links.get(link_name)
                if link is None or link.inertial is None or not link.inertial.mass:
                    continue
                position, rotation = frames[link_name]
                com_local = np.array(link.inertial.origin.xyz, dtype=float) \
                    if link.inertial.origin else np.zeros(3)
                com = position + rotation @ com_local
                moment += np.cross(com - origin, link.inertial.mass * GRAVITY)
            tau[index] = float(axis @ moment)
        return tau

    def deficit(self, q) -> np.ndarray:
        """**처짐** `Δq = τ_g(q)/Kp` — 실제가 지령에서 벗어나는 양 (`실제 − 지령`).

        `ready` 에서 joint4 가 −9.67 mrad 로, 실측(−9.70)과 맞는다.
        """
        return self.scale * self.torque(q) / self.kp

    def offset(self, q) -> np.ndarray:
        """지령에 **더할** 값 = `−처짐`.

        ⚠️ **부호가 뒤집히면 오차가 두 배가 된다.** `실제 = 지령 + 처짐` 이므로
        `실제 = q` 가 되려면 `지령 = q − 처짐` 이다. 2026-09-11 에 이 부호를 반대로
        넣었다가 실기에서 오차가 **−7.33 → −14.86 mm 로 정확히 두 배**가 됐다.
        단위 시험은 못 잡았다 — 구현과 시험을 같은(틀린) 추론으로 썼기 때문이다.
        """
        return -self.deficit(q)

    def compensate(self, q) -> list:
        """`q` 를 보상된 **지령값**으로 바꾼다 (`q − τ_g/Kp`).

        `τ_g` 는 원하는 자세에서 계산한다 — 보상량이 작아(수 mrad) 그 차이로 인한
        토크 변화는 2차 항이다.
        """
        return [float(v) for v in (np.asarray(q, dtype=float) + self.offset(q))]


def _resolve_urdf(spec: dict) -> str:
    package = spec.get('urdf_package')
    relative = spec.get('urdf_relative_path')
    if not package or not relative:
        raise ValueError('gravity_compensation needs urdf_package and urdf_relative_path')
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(get_package_share_directory(package), relative)


def load_gravity_compensator(backend: str) -> Optional[GravityCompensator]:
    """백엔드의 중력 보상 설정을 읽는다. **키가 없으면 `None`** — 보상하지 않는다.

    `None` 이 정상 경로다. mock·Isaac 은 컨트롤러가 중력을 스스로 잡으므로 이 키가
    없고, 펑션베이도 벤더가 A-1 을 반영하면 키를 지워 끈다.
    """
    motion = BACKEND_PROFILES[backend].get('motion') or {}
    spec = motion.get('gravity_compensation')
    if not spec:
        return None
    path = _resolve_urdf(spec)
    if not os.path.isfile(path):
        raise FileNotFoundError(f'{backend}: {path} 가 없다 (colcon build 를 다시 돌렸나?)')
    # 순서는 **명령 배열의 순서**와 같아야 한다 — 프로파일이 그것을 이미 갖고 있으므로
    # 두 곳에 적지 않는다 (`arm_command` 블록은 `arm_command_*` 로 평탄화된다).
    joint_names = spec.get('joint_names') or BACKEND_PROFILES[backend].get(
        'arm_command_joint_names')
    if not joint_names:
        raise ValueError(f'{backend}: gravity_compensation 에 joint_names 가 없고 '
                         'arm_command_joint_names 로도 못 채운다')
    return GravityCompensator(path, joint_names, spec.get('kp', 2000.0),
                              root_link=spec.get('root_link'),
                              scale=float(spec.get('scale', 1.0)))


def describe(backend: str) -> str:
    """사람이 읽을 요약."""
    comp = load_gravity_compensator(backend)
    if comp is None:
        return f'{backend}: 중력 보상 없음 (motion.gravity_compensation 미설정)'
    zeros = [0.0] * len(comp.joint_names)
    return '\n'.join([
        f'{backend} 중력 보상 — Kp {comp.kp[0]:g}, scale {comp.scale:g}',
        f'  URDF: {comp.path}',
        f'  관절: {comp.joint_names}',
        '  q=0 에서 τ_g = ' + ', '.join(f'{v:+.2f}' for v in comp.torque(zeros)) + ' N·m'])


__all__ = ['GravityCompensator', 'describe', 'load_gravity_compensator']
