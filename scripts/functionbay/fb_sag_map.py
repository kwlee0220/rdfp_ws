#!/usr/bin/env python3
"""고정물 지점의 **처짐(지령 대비 실제 부족분)** 을 높이별로 재고 계수를 뽑는다.

⚠️ **2026-09-11 부터 일상 경로에서 쓰지 않는다.** 백엔드의 중력 보상
(`motion.gravity_compensation`)이 처짐을 스트리밍 단계에서 없애므로 지점별 계수가
필요 없어졌다. 이 도구는 **중력 보상이 꺼진 상태의 처짐을 재는 계측기**로 남긴다 —
벤더가 A-1 을 반영했는지 판별할 때나 `scale` 을 정할 때 쓴다
(`docs/simulation/functionbay_gravity_compensation.md` §5).

보상이 켜진 채로 돌리면 처짐이 0 에 가깝게 나온다 — 그것이 정상이다.

    ./fb_sag_map.py            # 잰다 (팔이 움직인다)

인자: [--final-vs <배율>]

**결과는 화면으로만 낸다.** 저장하는 곳이 없다 — 지점별 계수를 파일에 담아 지령에 얹던
경로는 2026-09-11 에 중력 보상으로 대체되어 삭제됐다. 여기서 나오는 값은 **그때그때
판단하는 계측값**이다.

각 측정은 그 지점 위 `PERCH_Z` 에서 내려오는 **2단계**로 한다 — 실제 루틴이 그렇게
움직이고, 작은 지령이 안 먹는 데드밴드도 피한다.

⚠️ **목표 자세를 절대값으로 만든다.** 현재 자세를 읽어 쓰면 처짐이 자세도 틀어놓기
때문에 오차가 누적된다 — 2026-09-09 에 그렇게 이동 16회 만에 접근축이 연직에서
**22.3° 기울었고**, 그 상태에서 잰 Δy 는 부호까지 반대였다. 매 측정의 기울기를 함께
찍는 것은 그 재발을 바로 드러내기 위해서다 (정상값 약 1.25°).

**손끝이 고정물 상단보다 아래로 가지 않게** 최저 목표를 제한한다. 그 아래는 구멍
칼라에 부딪히므로, 필요하면 물체를 치우고 `TARGET_Z` 를 바꿔 잰다.
"""
from __future__ import annotations

import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.time import Time
import fb_grasp
from robot_control.moveit.move_group_factory import create_move_group_client
from robot_control.scene.fixtures import load_fixtures
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

BACKEND = 'functionbay'
# **측정 높이는 고정하지 않는다.** 처짐이 솔버마다 자릿수가 다르기 때문이다 — Crisp 은
# 약 5 mm 인데 RecurDyn 은 **약 88 mm** 다. 고정 높이를 쓰면 한쪽에서 손끝이 탁자를
# 파고든다 (2026-09-09 에 실제로 그랬다: 지령 0.080 → 실제 −0.0065).
#
# 그래서 **정찰 자세에서 처짐을 먼저 재고** 그 값으로 안전한 구간을 계산한다.
SCOUT_Z = 0.300           # 어느 솔버에서도 확실히 뜨는 높이
SPAN = 0.050              # 측정 구간의 높이 폭
STEPS = 4                 # 구간 안 측정점 수
SAFETY_MARGIN = 0.012     # 손끝이 고정물 상단에서 이만큼은 떠 있어야 한다
# 캘리브레이션 자세 = **파지 자세**여야 한다. `fb_grasp` 와 같은 값을 쓴다 — 자세가
# 다르면 관절 구성이 달라져 처짐도 달라지므로, 다른 자세에서 잰 계수는 못 쓴다.
#
# ⚠️ 예전에는 여기 자체 `YAW` 가 있었고 그것이 **`panda_link8` 기준**이라 `fb_grasp` 의
# 개폐축 기준과 **90° 어긋나 있었다.** 지금은 변환까지 `fb_grasp.tip_pose()` 하나가 한다.
GRASP_YAW = fb_grasp.DEFAULT_GRASP_YAW
# 손끝 여유 확인용 (보수적인 쪽 값).
FINGERTIP = 0.044
CONTACT_NM = 0.5
DEFAULT_FINAL_VS = 0.25

_BASE, _TIP, _TCP = 'panda_link0', 'panda_link8', 'grasp_center'


def take_option(name: str, default: float) -> float:
    if name not in sys.argv:
        return default
    index = sys.argv.index(name)
    value = float(sys.argv[index + 1])
    del sys.argv[index:index + 2]
    return value


class Probe:
    """TF 조회와 목표 생성. 자세는 **절대값**으로만 만든다."""

    def __init__(self, node) -> None:
        self._node = node
        self._buf = Buffer()
        self._listener = TransformListener(self._buf, node)
        self._gripper: list = []
        node.create_subscription(JointState, '/output/gripper_joint',
                                 self._gripper.append, 10)

    def spin(self, seconds: float) -> None:
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.02)

    def tcp(self) -> np.ndarray:
        t = self._buf.lookup_transform(_BASE, _TCP, Time()).transform.translation
        return np.array([t.x, t.y, t.z])

    def tilt_deg(self) -> float:
        """접근축이 연직에서 벗어난 각. **누적이 재발하면 여기서 커진다.**"""
        q = self._buf.lookup_transform(_BASE, _TCP, Time()).transform.rotation
        approach_z = 1 - 2 * (q.x * q.x + q.y * q.y)
        return float(np.degrees(np.arccos(min(1.0, abs(approach_z)))))

    def effort(self) -> float:
        return max(abs(v) for v in self._gripper[-1].effort) if self._gripper else 0.0

    def pose_for(self, xyz) -> Pose:
        """**파지 스크립트와 같은 변환을 쓴다** — 안 그러면 캘리브레이션 자세와 실제
        파지 자세가 달라져 계수가 안 맞는다."""
        return fb_grasp.tip_pose(self._buf, xyz, GRASP_YAW)


def fit(z_values, deltas) -> tuple:
    """`Δ = a + b·z`. 상수로 충분하면 `b=0` 으로 돌려준다 (임계 0.15 mm)."""
    z_values = np.asarray(z_values)
    deltas = np.asarray(deltas)
    constant_error = float(np.abs(deltas - deltas.mean()).max())
    if constant_error < 0.00015:
        return float(deltas.mean()), 0.0, constant_error
    slope, intercept = np.polyfit(z_values, deltas, 1)
    residual = float(np.abs(deltas - (intercept + slope * z_values)).max())
    return float(intercept), float(slope), residual


def main() -> None:
    final_vs = take_option('--final-vs', DEFAULT_FINAL_VS)

    rclpy.init()
    node = rclpy.create_node('fb_sag_map')
    probe = Probe(node)
    probe.spin(3.0)

    fixtures = load_fixtures(BACKEND)
    client = create_move_group_client(node, backend=BACKEND)
    client.wait_until_ready(timeout_sec=30.0)

    top_z = max(fixtures[name].entry_point()[2] for name in fixtures)
    scout_x, scout_y, _ = fixtures[sorted(fixtures)[0]].entry_point()

    # --- 정찰: 이 솔버의 처짐을 먼저 잰다 ---
    client.follow_trajectory_streamed([probe.pose_for([scout_x, scout_y, SCOUT_Z])],
                                      publish_rate=50.0)
    probe.spin(3.0)
    sag = probe.tcp()[2] - SCOUT_Z
    # 실제 손끝이 안전선 위에 있으려면 **지령**은 처짐만큼 더 높아야 한다.
    floor = top_z + FINGERTIP + SAFETY_MARGIN - sag
    targets = [round(floor + SPAN * i / (STEPS - 1), 4) for i in range(STEPS)][::-1]
    print(f'정찰 z={SCOUT_Z:.3f} → 실제 {probe.tcp()[2]:+.4f}  '
          f'**처짐 {sag*1000:+.1f} mm**  (기울기 {probe.tilt_deg():.2f}°)')
    print(f'고정물 상단 {top_z:.4f} + 손끝 {FINGERTIP:.3f} + 여유 {SAFETY_MARGIN:.3f} '
          f'− 처짐  →  지령 하한 {floor:.4f}')
    print(f'측정 높이 {targets}   개폐축 yaw {GRASP_YAW:+.4f} rad '
          f'({np.degrees(GRASP_YAW):+.1f}°)\n')
    perch_z = max(targets) + 0.030

    measured = {}
    for name in sorted(fixtures):
        target_x, target_y, _ = fixtures[name].entry_point()
        print(f'=== {name}  (x,y)=({target_x:+.4f},{target_y:+.4f}) ===')
        print('  목표 z    실제 z    Δz(mm)    Δx(mm)   Δy(mm)   |수평|  기울기')
        rows = []
        for z in targets:
            client.follow_trajectory_streamed(
                [probe.pose_for([target_x, target_y, perch_z])], publish_rate=50.0)
            probe.spin(2.5)
            future = client.follow_trajectory_async(
                [probe.pose_for([target_x, target_y, z])],
                publish_rate=50.0, velocity_scaling=final_vs)
            began = time.time()
            hit = False
            while not future.done() and time.time() - began < 60.0:
                rclpy.spin_once(node, timeout_sec=0.02)
                if probe.effort() > CONTACT_NM:
                    client.stop_streaming()
                    hit = True
                    break
            probe.spin(3.0)
            actual = probe.tcp()
            fingertip = actual[2] - FINGERTIP
            if fingertip < top_z:
                # **실측으로 확인한다.** 예측만 믿으면 처짐이 예상과 다를 때 파고든다.
                raise SystemExit(
                    f'손끝이 z={fingertip:+.4f} 로 고정물 상단 {top_z:.4f} 아래다 — '
                    f'중단한다. SCOUT_Z 를 올리거나 SAFETY_MARGIN 을 키운다')
            delta = actual - np.array([target_x, target_y, z])
            rows.append((z, delta))
            print(f'  {z:.4f}  {actual[2]:+.4f}  {delta[2] * 1000:+7.2f}  '
                  f'{delta[0] * 1000:+8.2f} {delta[1] * 1000:+8.2f}  '
                  f'{np.hypot(delta[0], delta[1]) * 1000:6.2f}  {probe.tilt_deg():5.2f}°'
                  f'{"  접촉!" if hit else ""}')
        measured[name] = rows
        print()

    print('=== 적합 계수 (Δ = a + b·z, 단위 m) ===')
    for name, rows in measured.items():
        z_values = [r[0] for r in rows]
        print(f'  "{name}": z_range [{min(z_values):.3f}, {max(z_values):.3f}]')
        for index, axis in enumerate('xyz'):
            a, b, residual = fit(z_values, [r[1][index] for r in rows])
            print(f'    "delta_{axis}": {{ "a": {a:.8f}, "b": {b:.8f} }}'
                  f'   잔차 {residual * 1000:.4f} mm')

    client.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
