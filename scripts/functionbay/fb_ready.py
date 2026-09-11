#!/usr/bin/env python3
"""B 단계 실험을 **항상 그리퍼를 열고 `ready` 자세에서 시작**하게 하는 공용 모듈 겸 단독 스크립트.

**로봇이 실제로 움직이는 작업은 예외 없이 이 순서를 밟는다 — ① 그리퍼 개방 ② `ready` 이동.**
`goto_ready()` 가 둘을 함께 수행하므로, 팔을 움직이는 스크립트는 이것만 부르면 된다.

**왜 그리퍼를 먼저 여는가**

접촉을 남긴 채 팔을 옮기면 **팔이 안 움직이는 것을 "명령 경로가 죽었다"로 오진한다**
(체크리스트 §0 함정 5). 실제로 그렇게 오진한 적이 있다 — 차트가 끝난 직후에는 그리퍼가
구멍에 꽂힌 peg 를 물고 있어서(`|effort|` 6.8~7.2 N·m) 팔이 구멍에 걸린다. 물체를 끌고
가다 씬을 망가뜨리는 것도 막는다. 개방은 `fb_gripper.release_contact()` 가 맡는다.

**왜 매번 `ready` 로 옮기는가**

1. **B 단계 측정값은 대부분 자세에 의존한다.** 중력 처짐(B-3·B-4)은 모멘트 암에
   비례하고 실효 가동범위(B-10)도 자세를 탄다. 시작 자세가 회차마다 다르면
   기준선과 비교하는 것 자체가 성립하지 않는다.
2. **`fb_hold.py` 는 현재 자세를 그대로 목표로 준다.** 처진 만큼 더 처지므로
   되돌리지 않고 반복하면 누적되어 자기충돌 자세까지 간다.
3. **작업면에 물체가 놓인 채로 시작하면 팔이 그것을 건드린다.** `ready` 는 팔을
   접어 세운 자세라 작업면에서 떨어져 있다.

**MoveIt 을 쓰지 않는다.** 자기충돌 자세에서는 MoveIt 이 시작 상태를 무효로 판정해
계획 자체가 실패하므로(`Motion planning start tree could not be initialized!`)
`MoveGroupClient` 로는 빠져나올 수 없다. 그래서 원시 ``JointState`` 를 직접 민다.
시뮬레이터가 내부 보간을 하지 않으므로 큰 도약을 한 번에 주지 않고 50 Hz 로 선형
보간한다.

**경로는 관절공간 직선이다.** 데카르트 직선이 아니므로 시작 자세에 따라 EE 가
예상 밖으로 휘어 지나갈 수 있다. 그래서 이동 전에 관절별 변화량을 먼저 출력한다 —
큰 값이 보이면 `Ctrl-C` 로 멈추고 `ramp_sec` 을 늘려 다시 건다.

    ./fb_ready.py              # 그리퍼를 열고 5초에 걸쳐 ready 로 이동
    ./fb_ready.py 10           # 램프 시간 지정 (물체 근처면 길게)
    ./fb_ready.py --keep-grasp # 그리퍼를 열지 않는다 (파지 유지 실험 전용)

`--keep-grasp` 는 **파지 상태에서 팔을 움직이는 것이 실험의 목적일 때만** 쓴다 (예: 파지가
팔 이동 중에 유지되는지 보는 체크리스트 §E 항목). 그 밖에는 쓰지 않는다.

다른 스크립트에서는 이렇게 쓴다.

    import fb_ready
    cur, pub = fb_ready.attach(node)
    fb_ready.goto_ready(node, cur, pub)      # 그리퍼 개방까지 함께 한다
"""
from __future__ import annotations

from typing import Optional

import sys
import time

import fb_gripper
import rclpy
from sensor_msgs.msg import JointState

ARM = [f'panda_joint{i}' for i in range(1, 8)]
# SRDF 의 panda_arm `ready` group_state 와 같은 값이다.
READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
REPORT_TOPIC = '/output/panda_joint'
COMMAND_TOPIC = '/input/panda_joint'
RATE_HZ = 50.0
# 시뮬레이터 수신 주기(50 Hz) 에 맞춘다 (문서 §10.3).
COMMAND_PERIOD_SEC = 1.0 / RATE_HZ
# moveit_resources_panda_description 의 URDF 한계. **시뮬레이터 실효 한계는 이보다
# 안쪽이고 자세에 따라 변한다** (joint7 은 약 1.64 rad 에서 막힌 실측이 있다). 그래서
# 이 표는 "어느 방향에 여유가 더 많은가" 를 고르는 데만 쓰고, 절대 한계로 믿지 않는다.
LIMITS = {
    'panda_joint1': (-2.9671, 2.9671), 'panda_joint2': (-1.8326, 1.8326),
    'panda_joint3': (-2.9671, 2.9671), 'panda_joint4': (-3.1416, 0.0873),
    'panda_joint5': (-2.9671, 2.9671), 'panda_joint6': (-0.0873, 3.8223),
    'panda_joint7': (-2.9671, 2.9671)}

DEFAULT_RAMP_SEC = 5.0
# 목표에 도달한 뒤에도 계속 명령해 정착시킨다. 놓으면 중력으로 다시 처진다.
DEFAULT_SETTLE_SEC = 4.0
# 이 값을 넘는 관절 변화가 있으면 경고한다. 물체를 쓸고 갈 수 있는 크기다.
LARGE_MOVE_RAD = 1.0


def attach(node) -> tuple[dict[str, float], object]:
    """관절 상태 구독과 명령 발행자를 붙이고 ``(현재값 dict, publisher)`` 를 돌려준다.

    dict 는 콜백이 계속 갱신하므로 호출자가 그대로 들고 있으면 된다.
    """
    cur: dict[str, float] = {}
    node.create_subscription(JointState, REPORT_TOPIC,
                             lambda m: cur.update(zip(m.name or ARM, m.position)), 50)
    pub = node.create_publisher(JointState, COMMAND_TOPIC, 10)
    return cur, pub


def free_direction(joint: str, value: float) -> float:
    """한계까지 여유가 더 많은 쪽의 부호를 고른다.

    한 방향으로만 밀면 그 관절이 한계 근처일 때 **무반응으로 오진한다.** 실제로
    joint7 이 1.4949 rad 인 자세에서 `+0.030` 지령에 0.0048 만 움직여 그렇게 읽혔다.
    """
    lo, hi = LIMITS[joint]
    return 1.0 if (hi - value) >= (value - lo) else -1.0


def wait_for_state(node, cur: dict[str, float], timeout_sec: float = 5.0) -> list[float]:
    """첫 관절 상태가 올 때까지 기다렸다가 ARM 순서 리스트로 돌려준다."""
    deadline = time.time() + timeout_sec
    while rclpy.ok() and time.time() < deadline and not cur:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not cur:
        raise SystemExit(f'no JointState received on {REPORT_TOPIC}')
    return [float(cur[j]) for j in ARM]


def goto_ready(node, cur: dict[str, float], pub, ramp_sec: float = DEFAULT_RAMP_SEC,
               settle_sec: float = DEFAULT_SETTLE_SEC, verbose: bool = True,
               open_gripper: bool = True) -> Optional[list[float]]:
    """**그리퍼를 열고** `ready` 까지 관절공간 직선으로 램프한 뒤 정착시킨다.

    돌려주는 값은 관절별 정착 오차(실제 − `ready`)다. 중력 처짐이 있으므로 0 이
    아닌 것이 정상이며, 기준선은 `ready` 자세에서 joint4 −0.0097 rad 다.

    `open_gripper=False` 는 **파지 상태에서 팔을 움직이는 것이 실험의 목적일 때만** 쓴다.
    그 밖에는 접촉을 남긴 채 팔을 옮기게 되어 오진과 씬 파손을 부른다 (모듈 docstring).

    그리퍼 개방을 자세 판독보다 **먼저** 한다 — 접촉이 풀리면 부하가 바뀌어 팔이 실제로
    움직인다 (실측 0.0089 rad). 개방 전 자세를 시작점으로 잡으면 그만큼 틀어진다.
    """
    if open_gripper:
        fb_gripper.release_contact(node, verbose=verbose)
    start = wait_for_state(node, cur)
    deltas = [READY[i] - start[i] for i in range(7)]
    if verbose:
        print('[ready] 이동 전 관절 변화량 (rad):')
        for i, joint in enumerate(ARM):
            flag = '  ← 큰 이동' if abs(deltas[i]) >= LARGE_MOVE_RAD else ''
            print(f'[ready]   {joint}: {start[i]:+.4f} → {READY[i]:+.4f}  ({deltas[i]:+.4f}){flag}')
        print(f'[ready] 최대 변화 {max(abs(d) for d in deltas):.4f} rad, '
              f'{ramp_sec:.1f}초에 걸쳐 이동한다')

    msg = JointState()
    msg.name = list(ARM)

    started = time.time()
    next_publish = started
    while rclpy.ok():
        elapsed = time.time() - started
        if elapsed > ramp_sec:
            break
        if time.time() >= next_publish:
            alpha = min(elapsed / ramp_sec, 1.0) if ramp_sec > 0 else 1.0
            msg.position = [start[i] + deltas[i] * alpha for i in range(7)]
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            next_publish += COMMAND_PERIOD_SEC
        rclpy.spin_once(node, timeout_sec=0.002)

    msg.position = list(READY)
    deadline = time.time() + settle_sec
    next_publish = time.time()
    while rclpy.ok() and time.time() < deadline:
        if time.time() >= next_publish:
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            next_publish += COMMAND_PERIOD_SEC
        rclpy.spin_once(node, timeout_sec=0.002)

    final = [float(cur[j]) for j in ARM]
    errors = [final[i] - READY[i] for i in range(7)]
    if verbose:
        worst = max(range(7), key=lambda i: abs(errors[i]))
        print(f'[ready] 도착. 최대 정착 오차 {ARM[worst]} {errors[worst]:+.4f} rad '
              f'(중력 처짐이라 0 이 아닌 것이 정상)')
    return errors


def ensure_ready(node, ramp_sec: float = DEFAULT_RAMP_SEC, verbose: bool = True,
                 open_gripper: bool = True) -> tuple[dict[str, float], object]:
    """구독·발행을 붙이고 **그리퍼를 열어** `ready` 로 옮긴 뒤 재사용할 핸들을 돌려준다."""
    cur, pub = attach(node)
    goto_ready(node, cur, pub, ramp_sec=ramp_sec, verbose=verbose, open_gripper=open_gripper)
    return cur, pub


def take_flag(name: str) -> bool:
    """``sys.argv`` 에서 플래그를 떼어내고 있었는지 알려준다.

    기존 스크립트가 전부 위치 인자를 쓰므로, 위치 파싱 전에 먼저 걷어낸다.
    """
    if name in sys.argv:
        sys.argv.remove(name)
        return True
    return False


def main() -> None:
    # 위치 인자보다 먼저 걷어낸다 (take_flag docstring).
    keep_grasp = take_flag('--keep-grasp')
    ramp_sec = float(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_RAMP_SEC
    rclpy.init()
    node = rclpy.create_node('fb_ready')
    cur, pub = attach(node)
    errors = goto_ready(node, cur, pub, ramp_sec=ramp_sec, open_gripper=not keep_grasp)
    print('관절별 정착 오차 (실제 - ready):')
    for i, joint in enumerate(ARM):
        print(f'  {joint}: {errors[i]:+.4f} rad')
    print(f'최대 |오차| = {max(abs(e) for e in errors):.4f} rad')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
