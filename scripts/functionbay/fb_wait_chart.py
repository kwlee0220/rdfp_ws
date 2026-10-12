#!/usr/bin/env python3
"""시뮬레이션 시작 직후 자동으로 도는 **SignalChart 가 끝날 때까지** 기다린다.

`/scene/start` 로 시뮬레이션을 시작하면 씬의 Operation 블록이 가리키는 차트
(`<씬>__panda_robotiq_ref_1_vm.xml` 의 `Chart_File_Name`)가 재생되어 팔·그리퍼가
움직인다 — `MMR-Build_260929` 에서 t1 은 `panda_peginhole_file`(11 s, 끝에 peg 를 문다),
r2 는 `panda_peginhole_g2`(20 s, 11 s 뒤 9 s 동안 그리퍼를 연다).

**차트가 끝나기 전에 `/input/*` 로 명령을 보내면 차트가 끊긴다.** 시뮬레이터는 첫 토픽
명령에서 `토픽 명령 감지 → 토픽 모드 전환 (이후 커맨드 파일 무시)` 로 바꾼다(Player.log
`[PandaCommandTopicApplier]`). 그래서 팔·그리퍼를 움직이는 작업은 이 스크립트가 끝난
뒤에 한다. 로봇 스택은 기동만으로는 `/input/*` 에 아무것도 내지 않으므로 먼저 띄워도 된다.

**끝났다는 판정은 두 조건을 함께 본다.**

1. 팔과 그리퍼 보고가 `STILL_SEC` 동안 `STILL_RAD` 안에서 멈춰 있다 — 차트 안의 가장 긴
   정지 구간(1 s)보다 길게 잡는다. 수신 공백(정지, 체크리스트 A-17)은 멈춤으로 세지 않는다.
2. 팔이 차트의 **끝 자세**에 와 있다 — 또는 이 스크립트를 띄운 뒤 차트 길이만큼 지났다
   (토픽 명령으로 차트가 이미 끊긴 경우 끝 자세에 오지 않으므로).

차트 길이와 끝 자세는 차트 파일에서 읽는다. 활성 씬은 `SolverConfig.json` 의 `MsFile` 로
안다 — 시뮬레이션을 시작해야 그 씬으로 바뀌므로 **`/scene/start` 뒤에** 돌린다.

    ./fb_wait_chart.py                              # 기본 경로
    ./fb_wait_chart.py /mnt/d/MMR-Build_260929/MMR_Database
    ./fb_wait_chart.py --timeout 120

인자: [MMR_Database 경로] [--timeout <초>, 기본 120]   (경로: 인자 > 환경변수 `FB_MMR_DATABASE` > 기본값)
종료 코드: 0 = 끝났다 · 1 = 시간 초과 · 2 = 보고가 안 온다 · 3 = 차트를 못 읽었다
"""
from __future__ import annotations

import collections
import math
import os
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from sensor_msgs.msg import JointState

import fb_gains

ARM_TOPIC = '/output/panda_joint'
GRIPPER_TOPIC = '/output/gripper_joint'
# 멈춤 판정 — 차트 안의 가장 긴 정지(4~5 s 구간, 1 s)보다 길게 잡는다.
STILL_SEC = 3.0
STILL_RAD = 0.0005
# 멈춤 창 안에 이만큼은 보고가 와야 한다 (50 Hz 의 절반). 정지(A-17) 공백을 멈춤으로 세지 않으려는 것이다.
MIN_RATE_HZ = 25.0
# 끝 자세 판정 — 처짐(보상 없는 t1 에서 약 0.001 rad)보다 넉넉하게.
END_POSE_RAD = 0.01
FIRST_REPORT_TIMEOUT_SEC = 10.0
DEFAULT_TIMEOUT_SEC = 120.0
PROGRESS_SEC = 2.0


def chart_of(database: str, scene: str) -> tuple[str, float, list[float]]:
    """씬의 차트 이름 · 길이(s) · 팔 끝 자세(rad)를 차트 파일에서 읽는다."""
    folder = os.path.join(database, 'Database')
    vm = os.path.join(folder, 'VariableModel', f'{scene}__panda_robotiq_ref_1_vm.xml')
    with open(vm, encoding='utf-8-sig', errors='ignore') as handle:
        operation = ET.fromstring(handle.read())
    name = next((e.get('Chart_File_Name') for e in operation.iter() if e.get('Chart_File_Name')), None)
    if not name:
        raise SystemExit(f'{vm} 에 Chart_File_Name 이 없다')
    path = os.path.join(folder, 'SignalChart', f'{name}.xml')
    with open(path, encoding='utf-8-sig', errors='ignore') as handle:
        chart = ET.fromstring(handle.read())
    end = max(float(t) for e in chart.iter() if e.get('Time') for t in e.get('Time').split())
    arm = chart.find('Arm')
    if arm is None:
        raise SystemExit(f'{path} 에 <Arm> 이 없다')
    joints = sorted(arm.findall('Joint'), key=lambda j: int(j.get('Index')))
    final = [math.radians(float(j.find('Position').get('Value').split()[-1])) for j in joints]
    return name, end, final


def spread(window) -> float:
    """창 안의 축별 (최대 − 최소) 가운데 가장 큰 값."""
    columns = list(zip(*(positions for _, positions in window)))
    return max((max(c) - min(c)) for c in columns) if columns else math.inf


def still(history, now: float) -> tuple[bool, float]:
    """최근 `STILL_SEC` 동안 멈춰 있었는지와 그 창의 변화폭을 돌려준다.

    기록이 창 전체를 덮어야 판정한다 — 건수만 보면 구독 직후 1 s 남짓한 기록으로도 통과해
    차트 안의 짧은 정지 구간을 끝으로 오판한다(2026-10-01 첫 실행에서 1.3 s 만에 통과했다).
    """
    if not history or now - history[0][0] < STILL_SEC:
        return False, math.inf
    window = [(t, p) for t, p in history if now - t <= STILL_SEC]
    if len(window) < STILL_SEC * MIN_RATE_HZ:
        return False, math.inf
    width = spread(window)
    return width <= STILL_RAD, width


def main() -> int:
    args = list(sys.argv[1:])
    timeout = DEFAULT_TIMEOUT_SEC
    if '--timeout' in args:
        i = args.index('--timeout')
        timeout = float(args[i + 1])
        del args[i:i + 2]
    database = args[0] if args else os.environ.get('FB_MMR_DATABASE', fb_gains.DEFAULT_DATABASE)

    scene = fb_gains.active_scene(database)
    if scene is None:
        print(f'활성 씬을 판별하지 못했다 — {database}/Database/SolverConfig/SolverConfig.json 을 본다')
        return 3
    try:
        name, chart_sec, final = chart_of(database, scene)
    except (OSError, ET.ParseError, ValueError, AttributeError) as exc:
        print(f'차트를 못 읽었다 ({exc}) — 경로: {database}')
        return 3
    print(f'씬 {scene} · 차트 {name} · 길이 {chart_sec:g} s · 끝 자세 {[round(q, 4) for q in final]}')
    print(f'판정: 팔·그리퍼가 {STILL_SEC:g} s 동안 {STILL_RAD} rad 안에서 멈춤 + (끝 자세 {END_POSE_RAD} rad 안 '
          f'또는 {chart_sec:g} s 경과)')

    rclpy.init()
    node = rclpy.create_node('fb_wait_chart')
    histories: dict[str, collections.deque] = {ARM_TOPIC: collections.deque(maxlen=1000),
                                               GRIPPER_TOPIC: collections.deque(maxlen=1000)}
    axes: dict[str, int] = {}

    def on_report(topic: str, msg: JointState) -> None:
        histories[topic].append((time.monotonic(), list(msg.position)))
        axes[topic] = len(msg.position)

    for topic in histories:
        node.create_subscription(JointState, topic, lambda m, t=topic: on_report(t, m), 50)

    began = time.monotonic()
    last_progress = 0.0
    code = 1
    try:
        while time.monotonic() - began < timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            arm = histories[ARM_TOPIC]
            if not arm:
                if now - began > FIRST_REPORT_TIMEOUT_SEC:
                    print(f'{ARM_TOPIC} 보고가 {FIRST_REPORT_TIMEOUT_SEC:g} s 동안 안 왔다 — '
                          '시뮬레이션을 시작했나(/scene/start), endpoint 가 떠 있나')
                    code = 2
                    break
                continue
            arm_still, arm_width = still(arm, now)
            grip_still, grip_width = still(histories[GRIPPER_TOPIC], now)
            distance = max(abs(a - b) for a, b in zip(arm[-1][1], final))
            elapsed = now - began
            at_end = distance <= END_POSE_RAD
            if arm_still and grip_still and (at_end or elapsed >= chart_sec):
                reason = '끝 자세에 멈췄다' if at_end else f'{chart_sec:g} s 가 지났고 멈췄다 (끝 자세와 {distance:.4f} rad)'
                print(f'[{elapsed:5.1f} s] 차트 종료 — {reason}. 그리퍼 축 {axes.get(GRIPPER_TOPIC, 0)}')
                code = 0
                break
            if now - last_progress >= PROGRESS_SEC:
                last_progress = now
                print(f'[{elapsed:5.1f} s] 끝 자세까지 {distance:.4f} rad · 팔 변화 {arm_width:.4f} · '
                      f'그리퍼 변화 {grip_width:.4f} rad')
        else:
            print(f'{timeout:g} s 안에 끝나지 않았다')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
