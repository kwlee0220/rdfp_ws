#!/usr/bin/env python3
"""`/scene/objects` 의 물체 위로 가서 **파지 자세**를 만든다 (그리퍼는 닫지 않는다).

    ./fb_grasp.py                  # 첫 물체, 기본값
    ./fb_grasp.py peg              # 이름으로 고른다
    ./fb_grasp.py peg --final-vs 0.1   # 최종 하강만 더 느리게
    ./fb_grasp.py --no-ready       # ready 복귀를 건너뛴다 (이미 위에 있을 때)

인자: [물체 이름] [--final-vs <배율>] [--grasp-z <m>] [--no-ready] [--dry-run]

2단계로 접근한다.

    1단계 (빠름)  ready → 물체 바로 위        프로파일의 velocity_scaling (펑션베이 0.4)
    2단계 (느림)  순수 수직 하강              --final-vs (기본 0.25) + 접촉 감시

수직으로만 내려가므로 2단계에서 물체를 옆으로 훑지 않고, 접촉 감시도 이 구간에만 걸면
된다. **`--dry-run`** 은 계산한 목표만 찍고 팔을 움직이지 않는다.

⚠️ **`grasp_center` 는 손끝이 아니라 파지 중심이다** — 실제 손끝은 약 40 mm 아래다.
그것을 모르고 파지 높이를 정했다가 손끝이 탁자를 파고든 적이 있다 (2026-09-09).
아래 상수의 근거와 파지 높이 산정법은 `docs/gripper/grasp_center_frame.md` 를 본다.
"""
from __future__ import annotations

from typing import Optional

import math
import sys
import time

import fb_ready
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.time import Time
from rdfp_msgs.msg import SceneObjects
from robot_control.moveit.move_group_factory import create_move_group_client
from robot_control.scene.fixtures import load_fixtures
from robot_control.moveit.utils import downward_quaternion
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

# `grasp_center` 에서 실제 손끝까지 (그리퍼 열림 실측 39~40 mm, 충돌 메시 44 mm).
# **여유는 큰 쪽으로 잡는다** — 닫을수록 손가락이 세워져 44 mm 에 가까워진다.
FINGERTIP_OFFSET = 0.044
# 파지 지점에서 손끝이 바닥에서 떠 있어야 하는 최소 높이.
FINGERTIP_CLEARANCE = 0.010
# 접근 단계에서 물체 위에 두는 높이 (grasp_center 기준).
APPROACH_Z = 0.120
# **처짐 보정은 여기서 하지 않는다** — 백엔드의 중력 보상이 스트리밍 경로에서 이미
# 처리한다 (`motion.gravity_compensation`, docs/simulation/functionbay_gravity_compensation.md).
#
# 2026-09-11 이전에는 지점별 처짐 캘리브레이션(`motion.sag_calibration_file`)을 여기서
# 얹었다. 중력 보상과 **함께 쓰면 이중 보정**이 되므로 뺐다. 되살리려면 그 문서 §6.1 을
# 본다 — 다만 둘 중 하나만 쓴다.
# 이 값을 넘으면 접촉으로 보고 즉시 스트리밍을 끊는다 (빈손 기준선 0.01 N·m).
CONTACT_NM = 0.5
# 최종 하강의 속도 배율. 1단계는 프로파일의 전역값을 그대로 쓴다.
DEFAULT_FINAL_VS = 0.25
# **개폐축**(손가락 둘을 잇는 축)의 방향, base 의 +x 에서 잰 각. 0 이면 base x 축과
# 나란하다.
#
# ⚠️ **이 값은 `panda_link8` 의 yaw 가 아니다.** 데카르트 목표의 자세는 tip link 기준인데
# `grasp_center` 는 그것을 z 둘레로 **+90° 돌린** 프레임이다(정적 TF 의 `yaw=1.5708`).
# 예전에는 이 상수가 link8 yaw 였고 주석만 "개폐축"이라 적혀 있어, −135° 를 주면 개폐축이
# +135° 로 나왔다. 지금은 이름대로 개폐축이고 변환은 `tip_pose_for` 가 한다.
#
# `ready` 는 개폐축이 +135°(= −45°) 다. 축에 맞춘 0° 로 두는 것은 손목을 90° 더 돌리는
# 대신 화면·로그에서 방향이 바로 읽히기 때문이다. **원기둥 peg 에는 어느 값이든 결과가
# 같다** — 축대칭이라 개폐 방향이 파지·삽입에 영향을 주지 않는다.
# **+π/2 이고 −π/2 가 아닌 것이 중요하다.** 2지 그리퍼는 180° 대칭이라 두 값이 같은
# 자세를 뜻하지만 **손목 해가 다르다.** `ready`(joint7 +0.785)에서 출발할 때:
#   −π/2 → joint7 이 +2.889 로 **상한(+2.897)까지 0.5°** — 카테시안 계획이 12.5% 에서 끊긴다
#   +π/2 → joint7 이 −0.021 로 양쪽 2.88 rad 여유
# 개폐축 실측은 둘 다 −90.0° 로 같다 (2026-09-09 실측).
DEFAULT_GRASP_YAW = math.pi / 2.0

_BACKEND = 'functionbay'
_SCENE_TOPIC = '/scene/objects'
_GRIPPER_REPORT = '/output/gripper_joint'
_BASE, _TIP, _TCP = 'panda_link0', 'panda_link8', 'grasp_center'


def take_flag(name: str) -> bool:
    """위치 인자 파싱 전에 플래그를 걷어낸다 (`fb_ready` 와 같은 방식)."""
    if name in sys.argv:
        sys.argv.remove(name)
        return True
    return False


def take_option(name: str, default: float) -> float:
    if name not in sys.argv:
        return default
    index = sys.argv.index(name)
    value = float(sys.argv[index + 1])
    del sys.argv[index:index + 2]
    return value


def tip_quaternion(tf_buffer, grasp_yaw: float):
    """**개폐축 yaw → `panda_link8` 이 가져야 할 쿼터니언.**

    데카르트 목표의 자세는 MoveIt 그룹의 tip link(`panda_link8`) 기준인데 `grasp_center`
    는 그것을 z 둘레로 돌린 프레임이다. 그 상대 회전을 **TF 에서 읽는다** — 90° 를 상수로
    박으면 description 이 바뀔 때(§6.2 B안) 조용히 어긋난다.

    `fb_grasp` 와 `fb_sag_map` 이 **같은 함수를 써야** 캘리브레이션과 파지가 같은 자세를
    쓴다. 예전에는 각자 `downward_quaternion(YAW)` 를 불렀는데 그 `YAW` 가 한쪽은 개폐축,
    한쪽은 link8 기준이라 **90° 어긋나 있었다.**
    """
    rotation = tf_buffer.lookup_transform(_TIP, _TCP, Time()).transform.rotation
    offset_yaw = math.atan2(
        2.0 * (rotation.x * rotation.y + rotation.z * rotation.w),
        1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2))
    return downward_quaternion(grasp_yaw - offset_yaw)


def rotation_matrix(quaternion) -> np.ndarray:
    x, y, z, w = quaternion
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def tip_pose(tf_buffer, tcp_xyz, grasp_yaw: float = DEFAULT_GRASP_YAW) -> Pose:
    """`grasp_center` 목표 → `panda_link8` 지령 Pose. 자세는 **절대값**이다."""
    quaternion = tip_quaternion(tf_buffer, grasp_yaw)
    offset = tf_buffer.lookup_transform(_TIP, _TCP, Time()).transform.translation
    world = rotation_matrix(quaternion) @ np.array([offset.x, offset.y, offset.z])
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = [
        float(v) for v in (np.array(tcp_xyz) - world)]
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = quaternion
    return pose


class Approach:
    """TF·scene 조회와 좌표 변환을 한곳에 모은다."""

    def __init__(self, node) -> None:
        self._node = node
        self._buf = Buffer()
        self._listener = TransformListener(self._buf, node)
        self._scene: list = []
        self._gripper: list = []
        node.create_subscription(SceneObjects, _SCENE_TOPIC, self._scene.append, 10)
        node.create_subscription(JointState, _GRIPPER_REPORT, self._gripper.append, 10)

    def spin(self, seconds: float) -> None:
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.02)

    def tcp(self) -> np.ndarray:
        t = self._buf.lookup_transform(_BASE, _TCP, Time()).transform.translation
        return np.array([t.x, t.y, t.z])

    def effort(self) -> float:
        return max(abs(v) for v in self._gripper[-1].effort) if self._gripper else 0.0

    def find(self, name: Optional[str]):
        if not self._scene:
            raise SystemExit(f'{_SCENE_TOPIC} 를 못 받았다 — scene 노드가 떠 있나?')
        objects = self._scene[-1].objects
        if not objects:
            raise SystemExit(f'{_SCENE_TOPIC} 에 물체가 없다')
        if name is None:
            return objects[0]
        for obj in objects:
            if obj.name == name:
                return obj
        raise SystemExit(f'{name!r} 가 없다 — 있는 것: {[o.name for o in objects]}')

    def tip_pose_for(self, tcp_xyz, grasp_yaw: float = DEFAULT_GRASP_YAW) -> Pose:
        """**`grasp_center` 목표를 `panda_link8` 지령으로 바꾼다.**

        데카르트 목표는 MoveIt 그룹의 tip link 기준으로 해석되는데 `grasp_center` 와
        149 mm 떨어져 있고 z 둘레로도 돌아가 있다. `/ee_pose` 값을 그대로 목표로 넣으면
        그만큼 엉뚱한 곳을 지령한다 (`fb_ee.py` 의 같은 경고).

        ⚠️ **자세는 절대값으로 만든다 — 현재 자세를 읽어 쓰지 않는다.**
        처짐은 자세도 틀어놓으므로, 틀어진 현재 자세를 다음 목표로 주면 오차가 쌓인다.
        실측(2026-09-09): 그렇게 16회 이동했더니 접근축이 연직에서 **22.3° 기울었다.**
        """
        return tip_pose(self._buf, tcp_xyz, grasp_yaw)


def vertical_half_extent(obj) -> float:
    """물체가 중심에서 위아래로 얼마나 뻗는가 — **자세를 반영한다.**

    ⚠️ `dimensions[0]`(원기둥 길이)을 그냥 수직 크기로 쓰면 **누운 물체에서 틀린다.**
    50 mm peg 이 누우면 수직 크기는 길이가 아니라 지름(16 mm)이다. 그렇게 계산하면
    바닥을 17 mm 낮게 잡아 손끝 목표가 탁자 아래로 내려간다 (2026-09-09 발견).

    원기둥의 정확한 값은 축 방향 `a` 에 대해 ``|a_z|·(L/2) + √(1-a_z²)·r`` 이다.
    """
    length, radius = obj.dimensions[0], obj.dimensions[1]
    q = obj.pose.orientation
    # 원기둥 축은 로컬 z — 회전행렬 3열의 z 성분만 필요하다.
    axis_z = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    horizontal = max(0.0, 1.0 - axis_z * axis_z) ** 0.5
    return abs(axis_z) * (length / 2.0) + horizontal * radius


def grasp_height(obj, floor_z: float = 0.0) -> float:
    """`grasp_center` 를 어디에 둘 것인가.

    **두 조건을 따로 본다.** 손끝이 바닥을 안 치는가(`FINGERTIP_OFFSET` + 여유), 그리고
    패드가 물체를 덮는가. 파지 중심을 물체 중심에 맞추려 하면 안 된다 — 실제로 잡는 것은
    56.5 mm 짜리 패드 면이라 물체가 그 구간에 들어오기만 하면 된다.

    ⚠️ **`floor_z` 는 그 (x, y) 의 실효 바닥이지 탁자면이 아니다.** 고정물에 꽂힌 물체는
    자기 바닥이 고정물 **속**에 있어서, 물체 바닥만 보면 손끝을 고정물 안으로 내린다 —
    2026-09-10 에 구멍에 꽂힌 peg 을 잡다가 칼라를 **15 mm 파고들었다**. 물체 바닥과
    실효 바닥 중 **높은 쪽**을 쓴다. 바닥은 `FixtureSet.floor_at()` 이 준다.
    """
    bottom = max(obj.pose.position.z - vertical_half_extent(obj), floor_z)
    return bottom + FINGERTIP_OFFSET + FINGERTIP_CLEARANCE


def report(label: str, seconds: float, moved: float, tcp, target=None) -> None:
    line = (f'[{label}] {seconds:5.1f}s  {moved * 1000:6.1f} mm '
            f'→ {moved * 1000 / max(seconds, 1e-3):5.0f} mm/s   '
            f'tcp=({tcp[0]:+.4f},{tcp[1]:+.4f},{tcp[2]:+.4f})')
    if target is not None:
        line += f'  오차 {np.linalg.norm(tcp - target) * 1000:5.2f} mm'
    print(line, flush=True)


def main() -> None:
    skip_ready = take_flag('--no-ready')
    dry_run = take_flag('--dry-run')
    final_vs = take_option('--final-vs', DEFAULT_FINAL_VS)
    grasp_z_override = take_option('--grasp-z', float('nan'))
    name = sys.argv[1] if len(sys.argv) > 1 else None

    rclpy.init()
    node = rclpy.create_node('fb_grasp')
    approach = Approach(node)
    approach.spin(3.0)

    obj = approach.find(name)
    peg = np.array([obj.pose.position.x, obj.pose.position.y])
    # **손끝이 닿는 바닥은 탁자가 아닐 수 있다** — 고정물 위/안이면 그 윗면이다.
    fixtures = load_fixtures(_BACKEND)
    floor_z, floor_src, floor_unknown = fixtures.floor_at(peg[0], peg[1])
    grasp_z = (grasp_z_override if grasp_z_override == grasp_z_override
               else grasp_height(obj, floor_z))
    print(f'대상 {obj.name!r} ({obj.type}) xy=({peg[0]:+.4f},{peg[1]:+.4f})  '
          f'높이 {obj.dimensions[0] * 1000:.0f} mm')
    print(f'실효 바닥 z = {floor_z:+.4f} '
          f'({floor_src + " 상면" if floor_src else "탁자면"})')
    if floor_unknown:
        print(f'  ⚠️ 발자국을 몰라 판정 못 한 고정물: {floor_unknown} — '
              f'그 위라면 바닥이 더 높을 수 있다 (geometry.outer_extent_xy 를 채운다)')
    print(f'파지 grasp_center z = {grasp_z:.4f}  '
          f'→ 손끝 z ≈ {grasp_z - FINGERTIP_OFFSET:+.4f} '
          f'(실효 바닥에서 {(grasp_z - FINGERTIP_OFFSET - floor_z) * 1000:+.0f} mm)')
    fingertip_z = grasp_z - FINGERTIP_OFFSET
    if fingertip_z < floor_z:
        raise SystemExit(
            f'파지 높이가 손끝을 실효 바닥 아래로 보낸다 — 거부한다.\n'
            f'  손끝 {fingertip_z:+.4f} < 바닥 {floor_z:+.4f} '
            f'({floor_src + " 상면" if floor_src else "탁자면"}), '
            f'{(fingertip_z - floor_z) * 1000:+.1f} mm.\n'
            '  누운 원기둥처럼 수직 크기가 작은 물체는 손끝 여유를 확보할 수 없다. '
            '--grasp-z 로 명시하거나 물체를 세운다.')

    if dry_run:
        print('--dry-run — 움직이지 않는다')
        return

    # **속도는 프로파일이 정한다** — `motion.velocity_scaling` (펑션베이 0.4).
    # 여기서 인자로 주면 그쪽이 이기므로 일부러 주지 않는다.
    client = create_move_group_client(node, backend=_BACKEND)
    client.wait_until_ready(timeout_sec=30.0)

    if not skip_ready:
        # `ready` 는 그리퍼 개방까지 함께 한다 — 접촉을 남기면 팔이 걸리고, 그것을
        # "명령 경로가 죽었다"로 오진한다 (체크리스트 §B 서문).
        current, publisher = fb_ready.attach(node)
        fb_ready.goto_ready(node, current, publisher)
        approach.spin(2.0)

    start = approach.tcp()
    began = time.time()
    client.follow_trajectory_streamed(
        [approach.tip_pose_for([peg[0], peg[1], APPROACH_Z])],
        publish_rate=50.0)
    elapsed = time.time() - began
    approach.spin(3.0)
    above = approach.tcp()
    report('1단계 접근', elapsed, float(np.linalg.norm(above - start)), above)

    # 2단계 — 순수 수직 하강. 최종 자세의 처짐(z·x)을 여기서 보정한다.
    began = time.time()
    future = client.follow_trajectory_async(
        [approach.tip_pose_for([peg[0], peg[1], grasp_z])],
        publish_rate=50.0, velocity_scaling=final_vs)
    aborted = False
    while not future.done() and time.time() - began < 120.0:
        rclpy.spin_once(node, timeout_sec=0.02)
        if approach.effort() > CONTACT_NM:
            client.stop_streaming()
            aborted = True
            print(f'  ** 접촉 {approach.effort():.3f} N·m — 하강 중단 **', flush=True)
            break
    elapsed = time.time() - began
    approach.spin(3.0)
    final = approach.tcp()
    report(f'2단계 하강(vs={final_vs})', elapsed, float(above[2] - final[2]), final,
           target=np.array([peg[0], peg[1], grasp_z]))

    horizontal = float(np.hypot(final[0] - peg[0], final[1] - peg[1]))
    print(f'\n{"** 중단됨 **  " if aborted else ""}'
          f'수평 편차 {horizontal * 1000:.2f} mm (반지름 {obj.dimensions[1] * 1000:.0f} mm)  '
          f'z 오차 {(final[2] - grasp_z) * 1000:+.1f} mm  '
          f'손끝 z {final[2] - FINGERTIP_OFFSET:+.4f}  '
          f'effort {approach.effort():.3f} N·m')

    client.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
