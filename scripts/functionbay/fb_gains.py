#!/usr/bin/env python3
"""시뮬레이터 **제어 게인**을 읽어 우리 설정과 대조한다 (체크리스트 A-19).

    ./fb_gains.py                                   # 기본 경로
    ./fb_gains.py /mnt/d/MMR-Build_260910/MMR_Database
    FB_MMR_DATABASE=/mnt/d/... ./fb_gains.py

인자: [MMR_Database 경로]   (인자 > 환경변수 `FB_MMR_DATABASE` > 기본값)

**왜 보는가.** 우리는 중력 보상을 `Δq = −τ_g(q)/Kp` 로 거는데, 그 `Kp` 는 **시뮬레이터
씬 XML 에서 옮겨 적은 값**이다(`<씬>__Manipulator_1_vm.xml` 의 `<Gain Kp=...>`). 새 빌드가
그 값을 바꾸면 **보상량이 그만큼 틀어지는데 에러가 나지 않는다** — 증상은 "팔이 몇 mm
어긋난 곳에 선다" 뿐이다.

**같은 빌드 안에서도 씬마다 다르다.** 실측(`MMR-Build_260907`):

    t1 · r2   Kp 2000   ← 우리가 쓰는 값
    r1        Kp  200   ← **1/10**. 이 씬을 띄우면 보상이 1/10 만 걸린다
    indy7_01  Kp 20000  (다른 로봇)

**활성 씬은 `Database/SolverConfig/SolverConfig.json` 이 알려 준다** (`MsFile`). 그래서
이 스크립트는 어느 씬이 떠 있는지까지 자동으로 판단한다 — 팔을 움직이지 않는다.

배경: `docs/simulation/functionbay_gravity_compensation.md` §4.3
"""
from __future__ import annotations

from typing import Optional

import json
import os
import re
import sys

DEFAULT_DATABASE = '/mnt/d/MMR-Build_260907/MMR_Database'
BACKEND = 'functionbay'
# 2026-09-11 실측. 새 빌드에서 달라지면 이 표와 비교해 드러낸다.
KNOWN = {
    ('t1', 'arm'): 2000.0, ('t1', 'gripper'): 200.0,
    ('r2', 'arm'): 2000.0, ('r2', 'gripper'): 200.0,
    ('r1', 'arm'): 200.0, ('r1', 'gripper'): 200.0}


def gains_of(path: str) -> tuple:
    """`<Gain Kp="..." Kd="..."/>` 에서 값 목록을 뽑는다."""
    with open(path, encoding='utf-8-sig', errors='ignore') as handle:
        text = handle.read()
    match = re.search(r'<Gain\s+Kp="([^"]*)"\s+Kd="([^"]*)"', text)
    if not match:
        return [], []
    return ([float(v) for v in match.group(1).split()],
            [float(v) for v in match.group(2).split()])


def active_scene(database: str) -> Optional[str]:
    """`SolverConfig.json` 의 `MsFile` 에서 활성 씬 이름을 얻는다."""
    path = os.path.join(database, 'Database', 'SolverConfig', 'SolverConfig.json')
    if not os.path.isfile(path):
        return None
    with open(path, encoding='utf-8-sig', errors='ignore') as handle:
        config = json.load(handle)
    systems = config.get('RobotSystems') or []
    if not systems:
        return None
    ms = str(systems[0].get('MsFile', ''))
    return ms[:-len('_ms.xml')] if ms.endswith('_ms.xml') else (ms or None)


def scan(database: str) -> dict:
    """씬별 팔·그리퍼 게인을 모은다."""
    folder = os.path.join(database, 'Database', 'VariableModel')
    if not os.path.isdir(folder):
        raise SystemExit(f'{folder} 가 없다 — MMR_Database 경로를 확인한다')
    found: dict = {}
    for name in sorted(os.listdir(folder)):
        match = re.match(r'(.+?)__(Manipulator|Gripper6?)_\d+_vm\.xml$', name)
        if not match:
            continue
        scene, kind = match.group(1), match.group(2)
        role = 'arm' if kind == 'Manipulator' else 'gripper'
        kp, kd = gains_of(os.path.join(folder, name))
        if not kp:
            continue
        # 한 씬에 Gripper 와 Gripper6 가 함께 있으면 **축이 많은 쪽**이 실제 배선이다.
        previous = found.get((scene, role))
        if previous is None or len(kp) > len(previous[0]):
            found[(scene, role)] = (kp, kd, name)
    return found


def profile_kp() -> Optional[float]:
    """우리 프로파일이 쓰는 `Kp`. 중력 보상이 꺼져 있으면 `None`."""
    from robot_control.backend_profiles import BACKEND_PROFILES

    spec = (BACKEND_PROFILES[BACKEND].get('motion') or {}).get('gravity_compensation')
    return None if not spec else float(spec.get('kp', 2000.0))


def main() -> None:
    database = (sys.argv[1] if len(sys.argv) > 1
                else os.environ.get('FB_MMR_DATABASE', DEFAULT_DATABASE))
    print(f'MMR_Database: {database}')
    found = scan(database)
    scene = active_scene(database)
    print(f'활성 씬 (SolverConfig.json): {scene or "판별 실패"}\n')

    print(f'{"씬":<10}{"역할":<9}{"축":>4}{"Kp":>10}{"Kd":>9}   기록값 대조')
    for (name, role), (kp, kd, source) in sorted(found.items()):
        known = KNOWN.get((name, role))
        if known is None:
            note = '(기록 없음)'
        elif abs(kp[0] - known) < 1e-9:
            note = '일치'
        else:
            note = f'**바뀜 — 기록 {known:g}**'
        flag = ' ←활성' if name == scene else ''
        print(f'{name:<10}{role:<9}{len(kp):>4}{kp[0]:>10.0f}{kd[0]:>9.0f}   {note}{flag}')
        if len(set(kp)) > 1:
            print(f'{"":<23}⚠ 축마다 다르다: {kp}')

    print()
    ours = profile_kp()
    if ours is None:
        print('우리 설정: 중력 보상 **꺼짐** (motion.gravity_compensation 없음)')
        print('  → 벤더가 A-1 을 반영했다면 정상이다. 아니면 켜야 한다.')
        return
    if scene is None:
        raise SystemExit('활성 씬을 판별하지 못해 대조할 수 없다 — SolverConfig.json 을 본다')
    entry = found.get((scene, 'arm'))
    if entry is None:
        raise SystemExit(f'{scene} 씬의 Manipulator 게인을 못 찾았다')
    actual = entry[0][0]
    print(f'우리 설정 Kp = {ours:g}   ·   활성 씬({scene}) 실제 Kp = {actual:g}')
    if abs(actual - ours) < 1e-9:
        print('  ✅ 일치 — 중력 보상이 올바른 게인으로 걸린다')
        return
    ratio = actual / ours
    print(f'  ⚠️ **불일치 (실제/설정 = {ratio:.2f})** — 보상이 '
          f'{"과" if ratio < 1 else "미"}보정된다')
    print(f'     config/backends/functionbay.yaml 의 '
          f'motion.gravity_compensation.kp 를 {actual:g} 로 고치고 재빌드한다.')
    raise SystemExit(1)


if __name__ == '__main__':
    main()
