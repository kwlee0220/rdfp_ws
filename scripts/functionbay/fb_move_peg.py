#!/usr/bin/env python3
"""peg 을 한 고정물에서 다른 고정물로 **옮겨 넣는다** (peg-in-hole 왕복).

    ./fb_move_peg.py                 # 반대쪽으로 (현재 위치에서 자동 판단)
    ./fb_move_peg.py peg_tray        # 목적지를 명시
    ./fb_move_peg.py peg_hole --dry-run
    ./fb_move_peg.py --travel-z 0.18 --vs 0.10

인자: [목적지 이름] [--dry-run] [--no-ready] [--travel-z <m>] [--vs <배율>]

**하강은 한 번에 한다.** 여러 번 끊어 내리던 것은 중력 보상이 없던 시절의 회피책이다 —
그때는 하강이 목표보다 6.4 mm 덜 내려가 peg 이 구멍 턱에 눌렸다. 보상이 켜진 지금은
tcp 도달 오차가 **+0.05 mm** 라 끊을 이유가 없다 (실측: 11~13초 1회 하강, 삽입 오차
0.4 mm 안, 왕복 양방향 재현).

세 가지가 이 스크립트의 실체다.

**① 파지 오프셋 보정 — peg 을 정렬한다, tcp 가 아니다.**
그리퍼는 peg 을 정확히 중심에 물지 않는다 (실측 y 로 **−0.36 mm**, 방향 무관하게 일정).
반경 여유가 **0.5 mm** 뿐이라 그 0.36 mm 를 무시하면 안 들어간다. 실제로 오프셋
+1.46 mm 를 무시했다가 입구 턱에 걸렸다. 쥔 상태에서 `peg − tcp` 를 재고
`tcp = 구멍중심 − 오프셋` 으로 지령한다.

**② 기하 사전 확인 — 손끝이 고정물 상면 위에 남는가.**
peg 이 그리퍼 안에서 밀려 올라가 있으면 완전 삽입에 **손가락을 구멍 안에 넣어야** 하는데
안지름이 17 mm 라 불가능하다. 그 상태로 밀어 넣다 peg 이 튕겨 나간 적이 있다. 시작 전에
막고 **다시 잡으라고** 알린다.

**③ 앉음 확인 — 놓기 전에 실제로 앉았는지 본다.**
안 앉은 채(눌렸거나 떠 있거나) 놓으면 **시뮬레이터가 물체를 씬 밖으로 튕겨낸다**
(x +188 mm, z −3900 m 까지 가속. 재현 4/4). 그리퍼 `effort` 로는 안 잡히고 물체 TF 를
봐야 보인다. 펑션베이는 `/scene/reset` 이 없어 복구는 시뮬레이터 재시작뿐이다.

배경: `docs/simulation/functionbay_gravity_compensation.md`
"""
from __future__ import annotations

from typing import Optional

import math
import sys
import time

import numpy as np
import rclpy
import fb_grasp as FG
import fb_gripper as G
import fb_ready as R
from rdfp_msgs.msg import SceneObjects
from robot_control.moveit.move_group_factory import create_move_group_client
from robot_control.scene.fixtures import load_fixtures

BACKEND = 'functionbay'
OBJECT = 'peg'
DEFAULT_TRAVEL_Z = 0.150
DEFAULT_VS = 0.08
# peg 이 앉았다고 볼 높이 허용치. 구멍 깊이가 25 mm 라 3 mm 면 "들어갔다"와
# "턱에 걸렸다"를 확실히 가른다.
SEAT_TOL = 0.003
# 쥔 채로 이만큼 밀려 올라가면 어딘가에 닿은 것이다.
RIDE_LIMIT = 0.004
# 원기둥이 이보다 기울어져 있으면 서 있는 것으로 보지 않는다.
UPRIGHT_LIMIT_DEG = 15.0


def take_flag(name: str) -> bool:
    if name in sys.argv:
        sys.argv.remove(name)
        return True
    return False


def take_option(name: str, default):
    if name not in sys.argv:
        return default
    index = sys.argv.index(name)
    value = sys.argv[index + 1]
    del sys.argv[index:index + 2]
    return value


def tilt_deg(obj) -> float:
    """원기둥 축이 연직에서 벗어난 각. 0 이면 서 있다."""
    q = obj.pose.orientation
    axis_z = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    return float(math.degrees(math.acos(min(1.0, abs(axis_z)))))


class Mover:
    """이송 한 벌. 모든 이동이 이 클래스를 거치므로 판정도 한 곳에 모인다."""

    def __init__(self, node) -> None:
        self.node = node
        self.approach = FG.Approach(node)
        self.approach.spin(3.0)
        self.gripper_state, self.gripper_pub = G.attach(node)
        G.wait_for_state(node, self.gripper_state)
        self.scene: dict = {}
        node.create_subscription(
            SceneObjects, '/scene/objects',
            lambda m: self.scene.update({o.name: o for o in m.objects}), 10)
        self.client = create_move_group_client(node, backend=BACKEND)
        self.client.wait_until_ready(timeout_sec=30.0)
        self.approach.spin(2.5)
        self.fixtures = load_fixtures(BACKEND)

    # ----- 관측 -----

    def object(self):
        obj = self.scene.get(OBJECT)
        if obj is None:
            raise SystemExit(f'/scene/objects 에 {OBJECT!r} 이 없다 — scene 노드를 본다')
        return obj

    def position(self) -> np.ndarray:
        p = self.object().pose.position
        return np.array([p.x, p.y, p.z])

    def show(self, tag: str) -> None:
        p, t = self.position(), self.approach.tcp()
        print(f'  {tag:20s} peg=({p[0]:+.4f},{p[1]:+.4f},{p[2]:+.4f})  tcp z={t[2]:+.4f}  '
              f'간격={p[2] - t[2]:+.4f}  |eff|={G.max_effort(self.gripper_state):.3f}')

    # ----- 이동 -----

    def move(self, xyz, *, velocity_scaling: float = 0.15, settle: float = 3.0) -> None:
        """**정착까지 기다린다** — 스트리밍은 open loop 라 반환이 도달을 뜻하지 않는다."""
        self.client.follow_trajectory_streamed(
            [self.approach.tip_pose_for(list(xyz))], publish_rate=50.0,
            velocity_scaling=velocity_scaling)
        self.approach.spin(settle)

    def grasp(self, obj) -> float:
        """물체 위로 내려가 잡는다. 파지 후의 `peg − tcp` 간격을 돌려준다."""
        xy = self.position()[:2]
        floor, name, _ = self.fixtures.floor_at(xy[0], xy[1])
        height = FG.grasp_height(obj, floor)
        print(f'  출발 고정물 {name or "탁자"} (바닥 {floor:+.4f}) → grasp_center {height:.4f}')
        self.move([xy[0], xy[1], FG.APPROACH_Z], settle=2.5)
        self.move([xy[0], xy[1], height], velocity_scaling=0.25, settle=3.0)
        error = self.approach.tcp() - np.array([xy[0], xy[1], height])
        print(f'  접근 오차 수평 {float(np.hypot(error[0], error[1])) * 1000:.2f} mm  '
              f'z {error[2] * 1000:+.2f} mm')
        G.set_span(self.node, self.gripper_state, self.gripper_pub, G.S_CLOSE,
                   ramp_sec=2.0, hold_sec=3.0, verbose=False)
        self.approach.spin(1.5)
        return float(self.position()[2] - self.approach.tcp()[2])

    def aim_for(self, fixture_xy, gap: float) -> list:
        """**peg 이** 구멍 중심에 오도록 tcp 목표 xy 를 만든다."""
        offset = self.position()[:2] - self.approach.tcp()[:2]
        print(f'  파지 오프셋 ({offset[0] * 1000:+.2f}, {offset[1] * 1000:+.2f}) mm '
              f'— 반경 여유 0.5 mm 라 무시하면 안 들어간다')
        return [float(fixture_xy[0] - offset[0]), float(fixture_xy[1] - offset[1])]


def check_geometry(fixture, gap: float) -> float:
    """완전 삽입이 기하적으로 가능한가. 목표 tcp z 를 돌려준다.

    Raises:
        SystemExit: 손끝이 고정물 상면 아래로 들어가야 할 때. peg 이 그리퍼 안에서
            밀려 올라간 것이며, **그대로 밀어 넣으면 물체가 튕겨 나간다.**
    """
    floor = fixture.absolute_z('floor_z') if 'floor_z' in fixture.geometry else 0.003
    half = 0.025
    target = floor + half - gap
    tip = target - FG.FINGERTIP_OFFSET
    top = fixture.top_z()
    print(f'  간격 {gap * 1000:+.2f} mm → 완전 삽입 시 tcp {target:.4f}, 손끝 {tip:+.4f}')
    print(f'  {fixture.name} 상면 {top:+.4f} → 여유 {(tip - top) * 1000:+.1f} mm')
    if tip <= top:
        raise SystemExit(
            f'손끝이 {fixture.name} 상면 아래로 들어가야 완전 삽입이 된다 — 불가능하다.\n'
            f'  peg 이 그리퍼 안에서 밀려 올라갔다 (간격 {gap * 1000:+.1f} mm, 정상 약 −54).\n'
            f'  놓았다 다시 잡은 뒤 재시도한다.')
    return target


def main() -> None:
    dry_run = take_flag('--dry-run')
    skip_ready = take_flag('--no-ready')
    travel_z = float(take_option('--travel-z', DEFAULT_TRAVEL_Z))
    velocity_scaling = float(take_option('--vs', DEFAULT_VS))
    destination: Optional[str] = sys.argv[1] if len(sys.argv) > 1 else None

    rclpy.init()
    node = rclpy.create_node('fb_move_peg')
    mover = Mover(node)

    obj = mover.object()
    tilt = tilt_deg(obj)
    if tilt > UPRIGHT_LIMIT_DEG:
        raise SystemExit(f'peg 이 {tilt:.1f}° 기울어져 있다 — 서 있는 물체만 다룬다')

    here = mover.position()
    _, source, _ = mover.fixtures.floor_at(here[0], here[1])
    if destination is None:
        others = [n for n in mover.fixtures if n != source]
        if len(others) != 1:
            raise SystemExit(f'목적지를 정할 수 없다 — 고정물 {sorted(mover.fixtures)} '
                             f'중 하나를 인자로 준다')
        destination = others[0]
    if destination == source:
        raise SystemExit(f'peg 이 이미 {destination} 에 있다')
    fixture = mover.fixtures[destination]
    target_xy = np.array(fixture.entry_point()[:2])
    print(f'peg ({here[0]:+.4f}, {here[1]:+.4f}, {here[2]:+.4f}) 기울기 {tilt:.1f}°  '
          f'{source or "탁자"} → {destination}')

    if dry_run:
        print('--dry-run — 움직이지 않는다')
        node.destroy_node()
        rclpy.shutdown()
        return

    cur, pub = R.attach(node)
    if not skip_ready:
        R.goto_ready(node, cur, pub, ramp_sec=4.0, verbose=False)
        mover.approach.spin(2.0)
    mover.show('시작')

    print('\n=== 1. 집기 ===')
    gap = mover.grasp(obj)
    mover.show('파지')

    print('\n=== 2. 기하 확인 ===')
    target_z = check_geometry(fixture, gap)

    print(f'\n=== 3. 이송 → {destination} ===')
    up = mover.approach.tcp().copy()
    up[2] = travel_z
    mover.move(up.tolist(), velocity_scaling=0.12, settle=3.0)
    aim = mover.aim_for(target_xy, gap)
    mover.move(aim + [travel_z], velocity_scaling=0.15, settle=3.5)
    p = mover.position()
    print(f'  정렬 후 peg 수평 오차 ({(p[0] - target_xy[0]) * 1000:+.2f}, '
          f'{(p[1] - target_xy[1]) * 1000:+.2f}) mm')

    print(f'\n=== 4. 하강 (한 번에, {travel_z:.3f} → {target_z:.4f}) ===')
    began = time.time()
    mover.move(aim + [target_z], velocity_scaling=velocity_scaling, settle=4.0)
    p, t = mover.position(), mover.approach.tcp()
    ride = (p[2] - t[2]) - gap
    print(f'  {time.time() - began:.1f}s  tcp {t[2]:+.4f} '
          f'(오차 {(t[2] - target_z) * 1000:+.2f} mm)  밀림 {ride * 1000:+.2f} mm')
    mover.show('하강 완료')

    print('\n=== 5. 앉음 확인 ===')
    entry = fixture.absolute_z('entry_z')
    seated = abs(p[2] - entry) < SEAT_TOL and ride <= RIDE_LIMIT
    print(f'  peg z={p[2]:+.4f} (목표 {entry:.4f} ± {SEAT_TOL * 1000:.0f} mm), '
          f'밀림 {ride * 1000:+.2f} mm → '
          f'{"앉음 — 개방한다" if seated else "**미착석 — 개방하지 않는다**"}')
    if not seated:
        print('  쥔 채로 둔다. 놓으면 시뮬레이터가 물체를 씬 밖으로 튕겨낸다 '
              '(복구는 재시작뿐).')
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    G.set_span(node, mover.gripper_state, mover.gripper_pub, G.S_OPEN,
               ramp_sec=2.0, hold_sec=2.5, verbose=False)
    for seconds in (0.0, 3.0, 7.0):
        mover.approach.spin(3.0 if seconds else 0.5)
        mover.show(f'개방 t≈{seconds:.0f}s')

    retreat = mover.approach.tcp().copy()
    retreat[2] = FG.APPROACH_Z
    mover.move(retreat.tolist(), settle=2.0)
    if not skip_ready:
        R.goto_ready(node, cur, pub, ramp_sec=4.0, verbose=False)
        mover.approach.spin(2.0)
    mover.show('복귀')

    p = mover.position()
    print(f'\n삽입 오차 ({(p[0] - target_xy[0]) * 1000:+.2f}, '
          f'{(p[1] - target_xy[1]) * 1000:+.2f}, {(p[2] - entry) * 1000:+.2f}) mm')
    mover.client.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
