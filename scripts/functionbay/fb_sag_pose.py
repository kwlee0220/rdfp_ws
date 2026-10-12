#!/usr/bin/env python3
"""이름 있는 자세(`ready`/`extended`)에서 **관절 처짐**을 보상 OFF/ON 으로 나란히 잰다.

`fb_sag_map.py` 는 고정물 지점을 **높이별(데카르트)** 로 재는 계측기라 "펴짐 자세의
joint2 처짐" 같은 **관절 공간** 값은 못 낸다. `fb_move.py` 는 `MoveGroupJgpcClient`
스트리밍 경로라 `motion.gravity_compensation` 이 얹혀 처짐이 0 으로 나온다 — 옛 회차
수치(RecurDyn +0.1428 / Crisp +0.0109 rad)와 비교하려면 **보상을 타지 않아야** 한다.

그래서 `/input/panda_joint` 로 **직접 발행**한다. 옛 회차가 쓰던 경로와 같다.

    ./fb_sag_pose.py                 # extended 자세를 보상 OFF → ON 으로 잰다
    ./fb_sag_pose.py ready           # ready 자세로
    ./fb_sag_pose.py extended --off  # 보상 OFF 만 (ON 단계를 건너뛴다)

⚠️ **`velocity` 를 비우면 안 된다** — 시뮬레이터가 명령을 통째로 무시한다 (2026-09-14).
⚠️ 측정이 끝나면 **항상 `ready` 로 되돌린다.** 펴진 채로 두면 다음 작업의 시작 자세가
   달라지고, 처짐이 누적되는 자세에 팔을 세워 두게 된다.
"""

from __future__ import annotations

from typing import Optional

import sys
import time

import fb_ready
import rclpy
from sensor_msgs.msg import JointState

POSES = {
    'ready': list(fb_ready.READY),
    # SRDF `extended` 는 팔이 **위로 곧게 선** 자세다 — 모멘트 암이 짧아 중력 토크가
    # 오히려 작다(모델상 joint2 오프셋 −0.00155). 이름만 보고 "펴짐 = 고토크"로 읽으면
    # 안 된다.
    'extended': [0.0, 0.0, 0.0, 0.0, 0.0, 1.571, 0.785],
    # 진짜 수평 — joint2 의 모멘트 암이 최대가 되는 자세다(모델상 오프셋 +0.02573,
    # `extended` 의 16배). 옛 회차의 "펴짐 (수평쪽)" 기준선과 같은 성격이라 비교는
    # 이쪽으로 해야 한다. FK 로 경로를 확인했다 — 최저점 z=+0.360 m 로 탁자에서
    # 31 cm 이상 뜨고, 팔이 x 음의 방향으로 가므로 작업면의 물체에서 멀어진다.
    'horizontal': [0.0, -1.571, 0.0, -0.0698, 0.0, 1.571, 0.785],
}

RAMP_SEC = 6.0
SETTLE_SEC = 8.0
SAMPLE_SEC = 2.0


def publish(pub, positions: list[float]) -> None:
    """`velocity` 를 0 으로 채워 발행한다 — 비우면 시뮬레이터가 무시한다."""
    msg = JointState()
    msg.name = list(fb_ready.ARM)
    msg.position = list(positions)
    msg.velocity = [0.0] * len(fb_ready.ARM)
    pub.publish(msg)


def command(pub, target: list[float], compensate: bool) -> list[float]:
    """보상 여부에 따라 실제로 내보낼 지령을 만든다."""
    return fb_ready.compensated(target) if compensate else list(target)


def ramp_to(node, pub, start: list[float], target: list[float], compensate: bool) -> None:
    steps = max(1, int(RAMP_SEC * fb_ready.RATE_HZ))
    for i in range(steps + 1):
        a = i / steps
        q = [start[j] + (target[j] - start[j]) * a for j in range(len(target))]
        publish(pub, command(pub, q, compensate))
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(fb_ready.COMMAND_PERIOD_SEC)


def hold_and_sample(node, cur: dict, pub, target: list[float], compensate: bool) -> list[list[float]]:
    """목표를 계속 내보내며 마지막 `SAMPLE_SEC` 구간의 실제값을 모은다."""
    end = time.time() + SETTLE_SEC
    samples: list[list[float]] = []
    while time.time() < end:
        publish(pub, command(pub, target, compensate))
        rclpy.spin_once(node, timeout_sec=0.0)
        if time.time() > end - SAMPLE_SEC:
            q = [cur.get(n) for n in fb_ready.ARM]
            if all(v is not None for v in q):
                samples.append([float(v) for v in q])
        time.sleep(fb_ready.COMMAND_PERIOD_SEC)
    return samples


def report(label: str, target: list[float], samples: list[list[float]]) -> Optional[float]:
    """관절별 잔차(실제 − 지령)를 찍고 |잔차| 최대를 돌려준다."""
    if not samples:
        print('  %s: 표본 없음 — 판정 불가' % label)
        return None
    n = len(samples)
    mean = [sum(s[j] for s in samples) / n for j in range(len(target))]
    print('  %s (표본 %d)' % (label, n))
    worst = 0.0
    for j, name in enumerate(fb_ready.ARM):
        resid = mean[j] - target[j]
        worst = max(worst, abs(resid))
        mark = '  ←' if abs(resid) >= 0.005 else ''
        print('    %-14s 지령 %+8.4f  실제 %+8.4f  잔차 %+8.4f rad%s' % (
            name, target[j], mean[j], resid, mark))
    print('    |잔차| 최대 = %.4f rad' % worst)
    return worst


def main() -> int:
    only_off = fb_ready.take_flag('--off')
    pose = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else 'extended'
    if pose not in POSES:
        print('알 수 없는 자세: %s (%s 중 하나)' % (pose, ', '.join(POSES)))
        return 2
    target = POSES[pose]

    rclpy.init()
    node = rclpy.create_node('fb_sag_pose')
    cur, pub = fb_ready.attach(node)

    print('=' * 78)
    print('자세별 처짐 측정 — pose=%s  보상=%s' % (pose, 'OFF 만' if only_off else 'OFF → ON'))
    print('  목표 %s' % ' '.join('%+.3f' % v for v in target))
    print('=' * 78)

    start = fb_ready.wait_for_state(node, cur)
    print('시작 자세 %s' % ' '.join('%+.4f' % v for v in start))

    # 1단계 — 보상 OFF 로 목표까지 간 뒤 정착시킨다.
    print('\n[1] 보상 OFF — 램프 %.0fs → 정착 %.0fs' % (RAMP_SEC, SETTLE_SEC))
    ramp_to(node, pub, start, target, compensate=False)
    off_worst = report('보상 OFF', target, hold_and_sample(node, cur, pub, target, False))

    on_worst = None
    if not only_off:
        # 2단계 — 같은 자세에서 지령에만 보상을 얹는다 (팔은 그대로 있다).
        print('\n[2] 보상 ON — 같은 자세에서 지령만 바꾼다, 정착 %.0fs' % SETTLE_SEC)
        on_worst = report('보상 ON', target, hold_and_sample(node, cur, pub, target, True))

    # 되돌리기 — 펴진 채로 두지 않는다.
    print('\n[3] ready 로 복귀')
    here = fb_ready.wait_for_state(node, cur)
    ramp_to(node, pub, here, list(fb_ready.READY), compensate=False)
    time.sleep(1.0)

    print('\n' + '=' * 78)
    if off_worst is not None:
        print('요약  %s  보상 OFF |잔차|최대 = %.4f rad' % (pose, off_worst))
    if on_worst is not None:
        print('      %s  보상 ON  |잔차|최대 = %.4f rad' % (pose, on_worst))
    print('=' * 78)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
