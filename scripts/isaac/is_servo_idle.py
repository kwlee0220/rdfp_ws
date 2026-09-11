#!/usr/bin/env python3
"""servo 가 **명령 없이도** 팔을 끌고 가는지 잰다.

``docs/teleop/servo_vs_planned_motion.md`` §5 를 재현한다 — `start_servo` 만 하고
twist·JointJog 를 한 건도 보내지 않는다. 절차 전체는
``docs/teleop/servo_vs_planned_motion.md`` 부록 A 다.

    ./is_servo_idle.py                 # 10 초 관찰, stop_servo 뒤 5 초 더
    ./is_servo_idle.py --seconds 30
    ./is_servo_idle.py --prime         # 0 twist 를 **한 번** 준 뒤 쉰다
    ./is_servo_idle.py --prime 0.3     # +z 0.3 으로 1 초 민 뒤 0 을 한 번 주고 쉰다
                                       #   (teleop 에서 키를 눌렀다 뗀 상황)

`--prime` 이 없으면 start_servo 뒤 아무것도 보내지 않는다. 유휴 구간에는 twist 토픽의
**다른 발행자 수**를 함께 찍는다 — 0 이 아니면 "명령 0 건" 조건이 아니다 (보통
`teleop_keyboard` 가 떠 있는 것이다).

순서:

    0. /joint_states 스냅샷
    1. start_servo
    2. --seconds 동안 아무것도 보내지 않고
         - /panda_arm_controller/joint_trajectory 건수 (문서: 10 초에 293 건)
         - 관절 변화 (문서: 초당 약 3 mrad, 중력 처짐 방향)
         - /panda_arm_controller/controller_state 의 추종 오차 최댓값
           (문서: 0 — 목표 자체가 걸어가므로 컨트롤러로는 안 보인다)
    3. stop_servo → --after 초 동안 다시 센다 (0 건 / 0 mrad 여야 한다)

**팔은 `ready` 에 두고 시작한다.** 자세마다 처짐 방향과 크기가 다르므로, 문서의 수치와
비교하려면 같은 자세여야 한다.

**`teleop_keyboard` 가 떠 있으면 안 된다** — 기동 시 servo 를 자동 시작하므로 이 측정의
"명령 0 건" 조건이 깨진다.

종료 코드: 측정을 마치면 0, 토픽·서비스가 없어 측정하지 못하면 2.
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

ARM_JOINTS = [f'panda_joint{i}' for i in range(1, 8)]
COMMAND_TOPIC = '/panda_arm_controller/joint_trajectory'
TWIST_TOPIC = '/servo_node/delta_twist_cmds'
STATE_TOPIC = '/panda_arm_controller/controller_state'


class Probe(Node):
    def __init__(self) -> None:
        super().__init__('is_servo_idle',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.joints: dict = {}
        self.cmd_count = 0
        self.max_error = 0.0
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self.create_subscription(JointTrajectory, COMMAND_TOPIC, self._on_command, 50)
        self.create_subscription(JointTrajectoryControllerState, STATE_TOPIC,
                                 self._on_state, 10)
        self.start_cli = self.create_client(Trigger, '/servo_node/start_servo')
        self.stop_cli = self.create_client(Trigger, '/servo_node/stop_servo')
        self.twist_pub = self.create_publisher(TwistStamped, TWIST_TOPIC, 10)

    def publish_twist(self, z: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'panda_link0'
        msg.twist.linear.z = float(z)
        self.twist_pub.publish(msg)

    def prime(self, z: float, seconds: float) -> None:
        """z 가 0 이면 0 twist 를 **한 번**, 아니면 z 로 seconds 초 밀고 0 을 한 번 —
        teleop 에서 키를 눌렀다 떼는 것과 같은 순서다."""
        if z != 0.0:
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                self.publish_twist(z)
                self.spin_for(1.0 / 30.0)
        self.publish_twist(0.0)

    def other_twist_publishers(self) -> int:
        """유휴 조건 검증 — 이 노드 말고 twist 토픽에 발행자가 있는가."""
        return max(0, self.count_publishers(TWIST_TOPIC) - 1)

    def _on_joints(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self.joints[name] = pos

    def _on_command(self, _msg: JointTrajectory) -> None:
        self.cmd_count += 1

    def _on_state(self, msg: JointTrajectoryControllerState) -> None:
        if msg.error.positions:
            self.max_error = max(self.max_error, max(abs(e) for e in msg.error.positions))

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_joints(self, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(j in self.joints for j in ARM_JOINTS):
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
        return {j: self.joints[j] for j in ARM_JOINTS}

    def observe(self, seconds: float) -> dict:
        """건수·오차를 0 에서 다시 세며 seconds 동안 지켜본다."""
        self.cmd_count = 0
        self.max_error = 0.0
        before = self.snapshot()
        self.spin_for(seconds)
        after = self.snapshot()
        deltas = {j: (after[j] - before[j]) * 1000.0 for j in ARM_JOINTS}   # mrad
        worst = max(deltas, key=lambda j: abs(deltas[j]))
        return {'count': self.cmd_count, 'deltas': deltas, 'worst': worst,
                'max_error': self.max_error, 'seconds': seconds,
                'others': self.other_twist_publishers()}


def _print(title: str, obs: dict) -> None:
    sec = obs['seconds']
    print(f'{title} — {sec:.1f} s')
    print(f'  명령 발행         {obs["count"]} 건 ({obs["count"] / sec:.1f} Hz)')
    print('  관절 변화(mrad)   ' + '  '.join(
        f'{j[-1]}:{d:+.1f}' for j, d in obs['deltas'].items()))
    worst = obs['worst']
    print(f'  최대 |Δ|          {abs(obs["deltas"][worst]):.1f} mrad ({worst}) '
          f'≈ {abs(obs["deltas"][worst]) / sec:.2f} mrad/s')
    print(f'  컨트롤러 추종 오차 최대 {obs["max_error"] * 1000:.2f} mrad')
    others = obs['others']
    print(f'  twist 토픽의 다른 발행자 {others} 개'
          + ('' if others == 0 else '  ← 명령 0 건 조건이 아니다 (teleop_keyboard?)'))


def main() -> int:
    parser = argparse.ArgumentParser(description='servo 무명령 흐름 측정')
    parser.add_argument('--seconds', type=float, default=10.0, help='start_servo 뒤 관찰 시간')
    parser.add_argument('--after', type=float, default=5.0, help='stop_servo 뒤 관찰 시간')
    parser.add_argument('--prime', nargs='?', type=float, const=0.0, default=None,
                        metavar='Z', help='start_servo 뒤 twist 를 준 뒤 쉰다 — 값이 없으면 0 을 '
                                          '한 번, Z 면 +z Z 로 --prime-seconds 밀고 0 을 한 번')
    parser.add_argument('--prime-seconds', type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = Probe()
    try:
        if not node.wait_joints(10.0):
            print('ERROR: /joint_states 가 오지 않는다 — 스택이 떠 있나', file=sys.stderr)
            return 2
        ok, message = node.call(node.start_cli)
        if not ok:
            print(f'ERROR: start_servo 실패: {message}', file=sys.stderr)
            return 2
        # servo 가 첫 발행을 시작할 시간을 조금 준다. 이 구간은 세지 않는다.
        node.spin_for(0.5)
        if args.prime is not None:
            end = time.monotonic() + 5.0
            while node.twist_pub.get_subscription_count() == 0 and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            node.prime(args.prime, args.prime_seconds)
            node.spin_for(0.2)

        if args.prime is None:
            title = 'servo 활성, 명령 0 건'
        elif args.prime == 0.0:
            title = 'servo 활성, 0 twist 한 번 뒤 무명령'
        else:
            title = f'servo 활성, +z {args.prime} 로 {args.prime_seconds}s 민 뒤 0 한 번, 이후 무명령'
        running = node.observe(args.seconds)
        _print(title, running)
        print('  (문서 §5: 10 초에 293 건, 초당 ~3 mrad, 추종 오차 0)')

        ok, message = node.call(node.stop_cli)
        if not ok:
            print(f'WARNING: stop_servo 실패: {message}', file=sys.stderr)
        node.spin_for(0.3)
        stopped = node.observe(args.after)
        _print('stop_servo 뒤', stopped)
        print('  (0 건 / 0 mrad 이어야 한다)')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
