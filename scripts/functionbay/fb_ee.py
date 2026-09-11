#!/usr/bin/env python3
"""EE 를 수직으로 ±지정량 움직여 **방향 의존 오차**를 잰다 (체크리스트 B-9).

**이번 회차에서 가장 중요한 측정이다.** 벤더 A-1(중력 보상)의 개선 여부가 여기서
갈린다. 기준선(2026-09-01)은 방향에 따라 오차 부호가 뒤집혔다.

    상승 지령 +20 mm  →  실제 +10.0 mm   (중력이 거스름: **부족**)
    하강 지령 -20 mm  →  실제 -30.0 mm   (중력이 도움: **초과**)

이것이 심각한 이유는 **우리 쪽 보정으로 흡수되지 않기** 때문이다. 자세 의존 편향은
자세별 보정표로 덮을 여지가 있지만, 방향 의존은 같은 목표점에 어느 쪽에서 접근했는지에
따라 값이 달라진다. 한 방향에서 얻은 계수를 반대 방향에 적용하면 실제로 발진했다.
**대칭이 되었으면 A-1 을 닫을 수 있다.**

**측정과 지령은 반드시 같은 링크여야 한다 — 그래서 `/ee_pose` 를 쓰지 않는다.**
데카르트 목표는 MoveIt 그룹의 tip link(`panda_arm` → `panda_link8`) 기준으로 해석되는데
`/ee_pose` 의 프레임은 launch 의 `ee_frame` 이 정한다. 펑션베이는 2026-09-04 부터
`grasp_center`(`panda_link8` 대비 **z −149.2 mm**)를 쓰므로, `/ee_pose` 를 읽어 목표를
만들면 **오프셋만큼 엉뚱한 곳을 지령한다.** 실제로 그렇게 깨졌다 — +20 mm 상승 지령이
−129 mm 하강이 되어 상승·하강 시행이 **둘 다 하강**이 되고, 방향 의존을 전혀 재지 못했다.
그래서 `panda_link0 → panda_link8` 을 TF 로 직접 조회한다. 기준선(2026-09-01)도 당시
`/ee_pose` 가 `panda_hand`(= `panda_link8` 과 같은 원점)였으므로 이래야 비교가 성립한다.

`/ee_pose` 와 `/output/endeffector` 는 참고용으로만 함께 기록한다 — 우리 스택은
`/output/endeffector` 를 쓰지 않는다(벤더 B-4 는 2026-09-04 철회).

**`/ee_pose` 는 독립 검증이 아니다.** `/output/panda_joint` → TF → FK 로 나온 값이라
관절값이 틀리면 EE 도 같이 틀린다. 여기서는 "지령한 만큼 갔는가" 만 본다.

    ./fb_ee.py                 # ±20 mm, 각 2회
    ./fb_ee.py 0.02 2          # 이동량(m), 반복 횟수
    ./fb_ee.py 0.02 2 --no-ready
    ./fb_ee.py 0.02 2 --keep-grasp   # 그리퍼를 건드리지 않는다 (팔만 쓰는 시험)

인자: [이동량(m), 기본 0.02] [반복, 기본 2] [--no-ready] [--keep-grasp] [--ramp <초>]

`--keep-grasp` 는 **그리퍼 모델이 우리 전제와 다른 빌드**에서 쓴다 — RecurDyn 빌드는
`/output/gripper_joint` 를 2축으로 보고하는데 `fb_gripper` 는 6축을 전제하므로 개방
자체가 불가능하다. 그리퍼가 아무것도 물고 있지 않다는 것을 `|effort|` 로 먼저 확인하고
쓴다 (물고 있으면 팔이 걸려 측정이 무의미해진다).
"""
from __future__ import annotations

from typing import Optional

import copy
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

import fb_ready

from robot_control.moveit.move_group_factory import create_move_group_client

EE_TOPIC = '/ee_pose'
SIM_EE_TOPIC = '/output/endeffector'
# 측정·지령 프레임. **MoveIt 그룹의 tip link 와 같아야 한다** (docstring 참조).
BASE_FRAME = 'panda_link0'
TIP_FRAME = 'panda_link8'
# 시뮬레이터가 보간하지 않으므로 스트리밍 주기를 명시한다 (launch 문서와 같은 값).
PUBLISH_RATE_HZ = 50.0
SETTLE_SEC = 3.0


class _Watch:
    """`/ee_pose` 와 `/output/endeffector` 를 함께 지켜본다."""

    def __init__(self, node):
        self.ours: Optional[Pose] = None
        self.sim: Optional[Pose] = None
        node.create_subscription(PoseStamped, EE_TOPIC, self._on_ours, 20)
        node.create_subscription(PoseStamped, SIM_EE_TOPIC, self._on_sim, 20)

    def _on_ours(self, msg):
        self.ours = copy.deepcopy(msg.pose)

    def _on_sim(self, msg):
        self.sim = copy.deepcopy(msg.pose)


def _spin(node, seconds: float) -> None:
    end = time.time() + seconds
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def _tip_pose(node, tf_buffer: Buffer, timeout: float = 10.0) -> Pose:
    """`BASE_FRAME → TIP_FRAME` 을 TF 로 읽어 `Pose` 로 돌려준다.

    `/ee_pose` 를 쓰지 않는 이유는 docstring 에 있다 — 지령 프레임과 어긋나면
    측정이 성립하지 않는다.
    """
    end = time.time() + timeout
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            tf = tf_buffer.lookup_transform(BASE_FRAME, TIP_FRAME, Time()).transform
        except Exception:
            continue
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (
            tf.translation.x, tf.translation.y, tf.translation.z)
        pose.orientation = tf.rotation
        return pose
    raise SystemExit(f'no TF {BASE_FRAME} -> {TIP_FRAME}')


def _dz(a: Pose, b: Pose) -> float:
    return b.position.z - a.position.z


def main() -> None:
    skip_ready = fb_ready.take_flag('--no-ready')
    keep_grasp = fb_ready.take_flag('--keep-grasp')
    ramp_sec = fb_ready.DEFAULT_RAMP_SEC
    if '--ramp' in sys.argv:
        i = sys.argv.index('--ramp')
        ramp_sec = float(sys.argv[i + 1])
        del sys.argv[i:i + 2]
    step_m = float(sys.argv[1]) if len(sys.argv) > 1 else 0.02
    repeats = int(sys.argv[2]) if len(sys.argv) > 2 else 2

    rclpy.init()
    node = rclpy.create_node('fb_ee')
    cur, pub = fb_ready.attach(node)
    if skip_ready:
        fb_ready.wait_for_state(node, cur)
        print('[ready] 건너뜀 (--no-ready)')
    else:
        fb_ready.goto_ready(node, cur, pub, ramp_sec=ramp_sec, open_gripper=not keep_grasp)
    home = fb_ready.wait_for_state(node, cur)

    watch = _Watch(node)
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    _spin(node, 1.0)

    # auto 는 /panda_arm_controller/commands 를 찾는데 이 스택에는 없다 — 백엔드를
    # 명시하면 mode 와 명령 채널을 `BACKEND_PROFILES` 가 채운다.
    client = create_move_group_client(node, backend='functionbay')
    client.wait_until_ready(timeout_sec=30.0)
    print(f'client: {type(client).__name__}\n')

    print(f'{"방향":>6} {"지령(mm)":>10} {"실제(mm)":>10} {"달성률":>8}   기준선')
    results: dict[str, list[float]] = {'up': [], 'down': []}
    for k in range(repeats):
        for label, sign in (('up', 1.0), ('down', -1.0)):
            before = _tip_pose(node, tf_buffer)
            goal = copy.deepcopy(before)
            goal.position.z = before.position.z + sign * step_m
            client.follow_trajectory_streamed([goal], publish_rate=PUBLISH_RATE_HZ)
            _spin(node, SETTLE_SEC)
            after = _tip_pose(node, tf_buffer)
            moved_mm = _dz(before, after) * 1000.0
            cmd_mm = sign * step_m * 1000.0
            results[label].append(moved_mm)
            base = '+10.0 mm (부족)' if sign > 0 else '-30.0 mm (초과)'
            print(f'{label:>6} {cmd_mm:>10.1f} {moved_mm:>10.2f} '
                  f'{moved_mm / cmd_mm * 100:>7.0f}%   {base}')
            # 다음 회차가 같은 자세에서 출발하도록 매번 되돌린다.
            fb_ready.goto_ready(node, cur, pub, ramp_sec=2.0, verbose=False,
                                open_gripper=not keep_grasp)
            _spin(node, 0.5)

    print()
    if results['up'] and results['down']:
        up = statistics.fmean(results['up'])
        down = statistics.fmean(results['down'])
        cmd = step_m * 1000.0
        print(f'상승 평균 {up:+.2f} mm / 지령 {cmd:+.1f} mm  → 달성률 {up / cmd * 100:.0f}%')
        print(f'하강 평균 {down:+.2f} mm / 지령 {-cmd:+.1f} mm  → 달성률 {down / -cmd * 100:.0f}%')
        asym = abs(abs(up) - abs(down))
        print(f'상승·하강 크기 차 {asym:.2f} mm  '
              f'(기준선 20.0 mm — 상승 10.0 vs 하강 30.0)')
        if asym < 0.2 * cmd:
            print('→ **방향 의존이 사라졌다.** 벤더 A-1 을 닫을 근거가 된다.')
        else:
            print('→ 방향 의존이 남아 있다. 벤더 A-1 유지.')

    tip = _tip_pose(node, tf_buffer).position
    print()
    print(f'참고 — 같은 순간의 EE 값들 (지령 프레임은 {TIP_FRAME})')
    print(f'  {BASE_FRAME} -> {TIP_FRAME:<16} x={tip.x:+.4f} y={tip.y:+.4f} z={tip.z:+.4f}')
    if watch.ours is not None:
        o = watch.ours.position
        print(f'  /ee_pose (launch ee_frame)      x={o.x:+.4f} y={o.y:+.4f} z={o.z:+.4f}'
              f'   → tip 대비 z {o.z - tip.z:+.4f}')
    if watch.sim is not None:
        m = watch.sim.position
        print(f'  /output/endeffector (미사용)    x={m.x:+.4f} y={m.y:+.4f} z={m.z:+.4f}'
              f'   → tip 대비 z {m.z - tip.z:+.4f}')

    fb_ready.goto_ready(node, cur, pub, ramp_sec=3.0, open_gripper=not keep_grasp)
    print(f'\n기준 자세 대비 최대 차 '
          f'{max(abs(float(cur[j]) - home[i]) for i, j in enumerate(fb_ready.ARM)):.4f} rad')
    client.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
