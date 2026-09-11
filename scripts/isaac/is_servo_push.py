#!/usr/bin/env python3
"""servo 에 순수 +z twist 를 넣고 손끝이 **실제로 어디로 갔는지** 잰다.

``docs/teleop/servo_vs_planned_motion.md`` §2 의 A(잡은 채) / B(놓은 뒤) 행을 재현하고,
§3 "아직 안 가른 것" — ``panda_joint5`` 의 회전이 servo 가 **명령한** 것인지 물체에
**끌려간** 것인지 — 를 같은 실행에서 가른다. 절차 전체는
``docs/teleop/servo_vs_planned_motion.md`` 부록 A 다.

    ./is_servo_push.py --label A --save-start /tmp/start_A.json --csv /tmp/push_A.csv
    ./is_servo_push.py --label B --csv /tmp/push_B.csv

순서:

    0. /ee_pose · /joint_states 스냅샷 (before)
    1. start_servo
    2. TwistStamped(frame_id=panda_link0, linear.z=--input) 를 --rate Hz 로 --seconds 초
       발행한다. 발행하는 동안
         - /servo_node/status 를 **상시 구독**해 본 코드를 전부 모은다 (`--once` 로
           찍으면 안 되는 이유가 문서 §1 에 있다)
         - /panda_arm_controller/joint_trajectory 의 **명령값**과 /joint_states 의
           **실제값**을 나란히 기록한다 (joint5)
    3. 0 twist 한 번 → stop_servo
    4. 정착 뒤 스냅샷 (after) → 수평/수직/비율, 관절 변화, joint5 명령 vs 실제

**`teleop_keyboard` 가 떠 있으면 안 된다.** 기동 시 servo 를 자동 시작해 이 스크립트가
재려는 흐름(문서 §5)을 미리 만들고, 같은 twist 토픽에 자기 0 을 섞어 넣는다.

**twist 값은 m/s 가 아니다.** servo 가 `command_in_type: unitless` 라 [-1, 1] 의
비율이고 실제 속도는 launch 의 `servo_linear_scale`(Isaac 기본 0.4)이 정한다 —
입력 0.5 는 약 24.5 mm/s 다 (Isaac 실측 2026-09-06).

종료 코드: 측정을 마치면 0, 입력 토픽·서비스가 없어 측정하지 못하면 2. 값의 좋고
나쁨은 판정하지 않는다 — 표를 채우는 도구다.
"""
from __future__ import annotations

from typing import Optional

import argparse
import bisect
import csv
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

ARM_JOINTS = [f'panda_joint{i}' for i in range(1, 8)]
TWIST_TOPIC = '/servo_node/delta_twist_cmds'
STATUS_TOPIC = '/servo_node/status'
COMMAND_TOPIC = '/panda_arm_controller/joint_trajectory'
# servo 가 IK 증분을 더하는 **기준** 상태. 분해(IK 몫/되읽기 몫)는 이 시계열로 한다 —
# `servo_joint_source:=commanded` 스택이면 /joint_states_commanded 를 준다.
DEFAULT_BASE_TOPIC = '/joint_states'
FRAME_ID = 'panda_link0'
# 문서 §2 에서 유독 크게 돈 관절. 명령/실제 시계열은 이것만 기록한다.
FOCUS_JOINT = 'panda_joint5'

STATUS_NAME = {
    -1: 'INVALID', 0: 'NO_WARNING',
    1: 'DECELERATE_FOR_APPROACHING_SINGULARITY', 2: 'HALT_FOR_SINGULARITY',
    3: 'DECELERATE_FOR_COLLISION', 4: 'HALT_FOR_COLLISION', 5: 'JOINT_BOUND',
    6: 'DECELERATE_FOR_LEAVING_SINGULARITY'}

# joint5 판별 문턱. 이보다 작게 움직였으면 어느 쪽이라고 말하지 않는다.
MIN_ACTUAL_RAD = 0.03


class Probe(Node):
    def __init__(self, base_topic: str = DEFAULT_BASE_TOPIC) -> None:
        # 스택이 use_sim_time 이라 twist 의 stamp 도 sim 시계로 찍는다.
        super().__init__('is_servo_push',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.ee: Optional[PoseStamped] = None
        self.joints: dict = {}
        self.recording = False
        self.status_seen: set = set()
        self.cmd_count = 0
        self.cmd_series: list = []     # (sim t, joint5 명령값)
        self.act_series: list = []     # (sim t, joint5 실제값 — 전후 스냅샷과 관절 변화용)
        self.base_series: list = []    # (sim t, joint5 기준값 — servo 가 되읽는 것, 분해용)
        self.create_subscription(PoseStamped, '/ee_pose', self._on_ee, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        if base_topic == '/joint_states':
            self.base_series = self.act_series
        else:
            self.create_subscription(JointState, base_topic, self._on_base, 10)
        self.create_subscription(Int8, STATUS_TOPIC, self._on_status, 10)
        self.create_subscription(JointTrajectory, COMMAND_TOPIC, self._on_command, 50)
        self.twist_pub = self.create_publisher(TwistStamped, TWIST_TOPIC, 10)
        self.start_cli = self.create_client(Trigger, '/servo_node/start_servo')
        self.stop_cli = self.create_client(Trigger, '/servo_node/stop_servo')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_ee(self, msg: PoseStamped) -> None:
        self.ee = msg

    def _on_joints(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self.joints[name] = pos
        if self.recording and FOCUS_JOINT in self.joints:
            self.act_series.append((self._now(), self.joints[FOCUS_JOINT]))

    def _on_base(self, msg: JointState) -> None:
        if self.recording and FOCUS_JOINT in msg.name:
            self.base_series.append((self._now(), msg.position[msg.name.index(FOCUS_JOINT)]))

    def _on_status(self, msg: Int8) -> None:
        if self.recording:
            self.status_seen.add(int(msg.data))

    def _on_command(self, msg: JointTrajectory) -> None:
        if not self.recording:
            return
        self.cmd_count += 1
        if FOCUS_JOINT in msg.joint_names and msg.points:
            index = msg.joint_names.index(FOCUS_JOINT)
            point = msg.points[-1]
            if index < len(point.positions):
                self.cmd_series.append((self._now(), point.positions[index]))

    # ---- 보조 ----
    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_inputs(self, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.ee is not None and all(j in self.joints for j in ARM_JOINTS):
                return True
        return False

    def call(self, client, timeout: float = 5.0) -> tuple:
        if not client.wait_for_service(timeout_sec=timeout):
            return False, f'{client.srv_name} not available'
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            return False, 'no response'
        return future.result().success, future.result().message

    def snapshot(self) -> dict:
        p = self.ee.pose.position
        q = self.ee.pose.orientation
        # 베이스 z 둘레 yaw (rad). angular 측정에서 회전량을 재는 데 쓴다.
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return {'ee': (p.x, p.y, p.z), 'yaw': yaw,
                'joints': {j: self.joints[j] for j in ARM_JOINTS}}

    def publish_twist(self, z: float, angular: bool = False) -> None:
        """angular 면 linear.z 대신 angular.z (베이스 z 둘레 회전) 에 넣는다."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        if angular:
            msg.twist.angular.z = float(z)
        else:
            msg.twist.linear.z = float(z)
        self.twist_pub.publish(msg)


def _series_delta(series: list) -> Optional[float]:
    if len(series) < 2:
        return None
    return series[-1][1] - series[0][1]


def _decompose(cmd_series: list, act_series: list) -> Optional[dict]:
    """명령 변화를 **IK 몫**과 **되읽기 몫**으로 가른다.

    servo 는 매 주기 *측정된* 관절값에 IK 증분을 더해 다음 명령을 만든다. 그래서 명령의
    변화는 두 몫으로 나뉜다:

        IK 몫     = 명령(k) − 그 직전 측정값      servo 가 이번 주기에 **더한** 양
        되읽기 몫 = 그 직전 측정값 − 명령(k−1)    팔이 이전 명령보다 **처진** 양을 servo 가 되읽은 양

    문서 §3 의 두 갈래("명령에 들어 있다" / "명령은 0 인데 실제만")는 두 번째 몫을 놓친다 —
    처짐이 되읽혀 **명령 안으로** 들어오기 때문이다. 2026-09-07 Isaac 실측: 잡은 채 +z 에서
    joint5 는 IK 몫 **+0.117**, 되읽기 몫 **−0.337**, 합 −0.220 rad — servo 의 IK 는 오히려
    반대로 밀었고, 옆샘은 부하로 처진 측정값이 매 주기 명령에 적분된 것이다. 빈손은
    +0.053 / −0.047 로 상쇄된다.
    """
    if len(cmd_series) < 2 or not act_series:
        return None
    act_t = [t for t, _ in act_series]
    ik, ratchet = [], []
    for k in range(1, len(cmd_series)):
        t, q = cmd_series[k]
        i = bisect.bisect_left(act_t, t) - 1
        if i < 0:
            continue
        meas = act_series[i][1]
        ik.append(q - meas)
        ratchet.append(meas - cmd_series[k - 1][1])
    if not ik:
        return None
    return {'ik': sum(ik), 'ratchet': sum(ratchet), 'cycles': len(ik)}


def _joint5_verdict(parts: Optional[dict], act_delta: float) -> str:
    if abs(act_delta) < MIN_ACTUAL_RAD:
        return f'움직임이 작아 판별하지 않는다 (|Δ실제| < {MIN_ACTUAL_RAD * 1000:.0f} mrad)'
    if parts is None:
        return '명령값을 못 받았다 — servo 출력 토픽이 다른가'
    ik, ratchet = parts['ik'], parts['ratchet']
    if abs(ratchet) >= 2.0 * abs(ik) and ratchet * act_delta > 0:
        return ('되읽기(래칫) — 부하로 처진 측정값을 servo 가 매 주기 되읽어 명령에 적분한다. '
                'servo 에는 목표가 없어 되돌릴 기준이 없다 → 물체를 든 채로는 servo 를 쓰지 않는다')
    if abs(ik) >= 2.0 * abs(ratchet):
        return 'servo 의 IK 몫이 지배적 — IK 선택 (servo 파라미터 쪽)'
    return '두 몫이 섞였다 — 시계열(--csv) 을 본다'


def report(label: str, args: argparse.Namespace, before: dict, after: dict,
           node: Probe) -> None:
    bx, by, bz = before['ee']
    ax, ay, az = after['ee']
    dx, dy, dz = (ax - bx) * 1000, (ay - by) * 1000, (az - bz) * 1000
    horizontal = math.hypot(dx, dy)
    ratio = horizontal / abs(dz) if abs(dz) > 1e-6 else float('inf')

    what = 'angular.z 돌리기' if args.angular else '+z 밀기'
    print(f'servo {what} — {label}  (input {args.input}, {args.seconds}s @ {args.rate} Hz; '
          f'실제 속도는 launch 의 servo_*_scale 이 정한다)')
    print(f'  EE 이동         수평 {horizontal:6.1f} mm   수직 {dz:+6.1f} mm   '
          f'수평/수직 {ratio:.2f}   (dx {dx:+.1f}, dy {dy:+.1f})')
    if args.angular:
        dyaw = math.degrees(after['yaw'] - before['yaw'])
        dyaw = (dyaw + 180.0) % 360.0 - 180.0
        print(f'  EE yaw 회전     {dyaw:+6.1f} deg  ({dyaw / args.seconds:+.1f} deg/s)')
    deltas = {j: after['joints'][j] - before['joints'][j] for j in ARM_JOINTS}
    print('  관절 변화(rad)  ' + '  '.join(
        f'{j[-1]}:{d:+.3f}' + ('*' if j == FOCUS_JOINT else '') for j, d in deltas.items()))
    seen = ', '.join(f'{c}={STATUS_NAME.get(c, "?")}' for c in sorted(node.status_seen)) or '없음'
    print(f'  servo status    {seen}   (명령 {node.cmd_count}건, 발행 중 상시 구독)')

    cmd_delta = _series_delta(node.cmd_series)
    act_delta = deltas[FOCUS_JOINT]
    parts = _decompose(node.cmd_series, node.base_series)
    cmd_text = 'n/a' if cmd_delta is None else f'{cmd_delta:+.3f}'
    if parts is None:
        split = ''
    else:
        per_cycle = parts["ratchet"] / parts["cycles"] * 1000
        split = (f' = IK 몫 {parts["ik"]:+.3f} + 되읽기 몫 {parts["ratchet"]:+.3f} '
                 f'({parts["cycles"]} 주기, 되읽기 {per_cycle:+.1f} mrad/주기)')
    print(f'  {FOCUS_JOINT}    실제 Δ {act_delta:+.3f} rad / 명령 Δ {cmd_text}{split}'
          f'  [기준 {args.base_topic}]')
    print(f'                  → {_joint5_verdict(parts, act_delta)}')


def main() -> int:
    parser = argparse.ArgumentParser(description='servo 에 +z twist 를 넣고 손끝 이동을 잰다')
    parser.add_argument('--label', default='?', help='표의 행 이름 (A/B 등)')
    parser.add_argument('--input', type=float, default=0.5, help='twist linear.z (unitless)')
    parser.add_argument('--angular', action='store_true',
                        help='linear.z 대신 angular.z 에 --input 을 넣는다 (yaw 회전량을 잰다)')
    parser.add_argument('--seconds', type=float, default=1.5, help='발행 시간')
    parser.add_argument('--rate', type=float, default=30.0, help='발행 주기 Hz')
    parser.add_argument('--settle', type=float, default=1.0, help='stop_servo 뒤 정착 대기')
    parser.add_argument('--save-start', metavar='JSON',
                        help='before 관절값을 저장한다 (B 를 같은 자세에서 재기 위해)')
    parser.add_argument('--csv', metavar='CSV', help='joint5 명령/실제 시계열을 저장한다')
    parser.add_argument('--base-topic', default=DEFAULT_BASE_TOPIC,
                        help='servo 가 되읽는 기준 상태 토픽 (commanded 스택이면 '
                             '/joint_states_commanded). 분해는 이 시계열로 한다')
    args = parser.parse_args()

    rclpy.init()
    node = Probe(args.base_topic)
    try:
        if not node.wait_inputs(10.0):
            print('ERROR: /ee_pose 또는 /joint_states 가 오지 않는다 — 스택이 떠 있나', file=sys.stderr)
            return 2
        before = node.snapshot()
        if args.save_start:
            with open(args.save_start, 'w', encoding='utf-8') as handle:
                json.dump(before['joints'], handle, indent=2)
            print(f'  시작 관절값 저장: {args.save_start}')

        ok, message = node.call(node.start_cli)
        if not ok:
            print(f'ERROR: start_servo 실패: {message}', file=sys.stderr)
            return 2
        end = time.monotonic() + 5.0
        while node.twist_pub.get_subscription_count() == 0 and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.twist_pub.get_subscription_count() == 0:
            print(f'ERROR: {TWIST_TOPIC} 에 구독자가 없다 — servo 가 떠 있나', file=sys.stderr)
            return 2

        node.recording = True
        period = 1.0 / args.rate
        next_tick = time.monotonic()
        stop_at = next_tick + args.seconds
        while time.monotonic() < stop_at:
            node.publish_twist(args.input, args.angular)
            next_tick += period
            while time.monotonic() < next_tick:
                rclpy.spin_once(node, timeout_sec=0.005)
        # 정지는 0 을 **한 번** 보내고 servo 를 세운다 (teleop 과 같은 순서).
        node.publish_twist(0.0, args.angular)
        node.spin_for(0.2)
        node.recording = False
        ok, message = node.call(node.stop_cli)
        if not ok:
            print(f'WARNING: stop_servo 실패: {message}', file=sys.stderr)
        node.spin_for(args.settle)
        after = node.snapshot()

        report(args.label, args, before, after, node)
        if args.csv:
            with open(args.csv, 'w', newline='', encoding='utf-8') as handle:
                writer = csv.writer(handle)
                writer.writerow(['t_sim', 'kind', FOCUS_JOINT])
                for t, v in node.cmd_series:
                    writer.writerow([f'{t:.4f}', 'cmd', f'{v:.6f}'])
                for t, v in node.act_series:
                    writer.writerow([f'{t:.4f}', 'actual', f'{v:.6f}'])
            print(f'  시계열 저장: {args.csv} (cmd {len(node.cmd_series)} / actual '
                  f'{len(node.act_series)} 행)')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
