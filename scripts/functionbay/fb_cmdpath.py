#!/usr/bin/env python3
"""명령 경로가 살아 있는지 **팔과 그리퍼를 나눠서** 판정한다 — 벤더 A-4 분류용.

`fb_raw.py` 는 팔 7관절을 0.2 rad 씩 흔들어 반응을 본다. 이 스크립트는 그 앞 단계다.
목적이 다르다 — **어느 경로가 죽었는지 가르는 것**이지 성능을 재는 게 아니라서,
팔은 중력 토크가 없는 관절만 아주 조금(0.02~0.03 rad) 건드리고 곧바로 되돌린다.

**왜 나눠서 봐야 하는가.** 2026-09-03 에 그리퍼만 죽고 팔은 정확히 반응하는 상태를
관측했다 (그리퍼 5가지 명령 형식 × 양방향 전부 무반응, 같은 시각 팔 joint7 은 지령
+0.030 에 실제 +0.030002). 벤더 A-4 는 "명령 경로 전체 무반응" 으로 적혀 있어서
이 형태를 담지 못한다. **팔이라는 대조군이 있어야 오진이 안 섞인다.**

판정에 앞서 세 가지를 먼저 확인한다 — 이걸 빼면 엉뚱한 결론이 난다.

* **시뮬레이션이 돌고 있는가** — `header.stamp` 가 진행하는지 본다. 멈춰 있으면
  명령이 안 먹는 게 당연하다.
* **`/input/*` 에 구독자가 있는가** — 0 이면 Unity 미연결이라 판정 자체가 무의미하다.
* **그리퍼가 접촉 중인가** — `|effort|` > 1 N·m 이면 물체에 닿아 있다. 무반응처럼
  보이는 원인이 고착이 아니라 접촉일 수 있다 (체크리스트 §0 함정 5).

    ./fb_cmdpath.py              # 팔·그리퍼 둘 다
    ./fb_cmdpath.py --arm-only   # 그리퍼를 건드리면 안 될 때

인자: [--arm-only] [--gripper-only]
"""
from __future__ import annotations

import math
import time

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import fb_gripper
import fb_ready

ARM = fb_ready.ARM
GRIP_NAMES = fb_gripper.GRIPPER_JOINTS
# 중력 토크가 0 인 관절만 쓴다. 처짐과 반응을 구별할 필요가 없어진다 (§10.2b).
# 크기만 정하고 **부호는 자세를 보고 고른다** — 한계 근처에서 한 방향만 밀면
# 무반응으로 오진한다 (§0 함정 4).
ARM_TESTS = [('joint7 손목 롤', 6, 0.03), ('joint1 베이스 요', 0, 0.02)]
# 반응으로 인정할 최소 이동. 관측 잡음은 1e-7 rad 수준이라 여유가 크다.
MOVED_RAD = 0.005
DRIVE_SEC = 4.0
SETTLE_SEC = 2.0
ALIVE_CHECK_SEC = 3.0


def _arm_command(node, name, position) -> JointState:
    """팔 명령. `/input/panda_joint` 는 `sensor_msgs/JointState` 다."""
    msg = JointState()
    msg.name = list(name)
    msg.position = list(position)
    msg.header.stamp = node.get_clock().now().to_msg()
    return msg


def _gripper_command(node, name, position) -> Float64MultiArray:
    """그리퍼 명령. `/input/gripper_joint` 는 `std_msgs/Float64MultiArray` 다.

    **`name` 을 받지만 쓰지 않는다** — 이 메시지에는 관절 이름 필드가 없고 배열 순서가
    곧 관절 순서다. `_drive` 의 서명을 팔과 맞추기 위해 자리만 지킨다.
    """
    return fb_gripper.make_command(position)


def _drive(node, pub, name, position, seconds=DRIVE_SEC, make_msg=_arm_command):
    """`make_msg` 가 타입을 정한다 — 팔과 그리퍼는 명령 타입이 다르다."""
    end = time.time() + seconds
    nxt = time.time()
    while rclpy.ok() and time.time() < end:
        if time.time() >= nxt:
            pub.publish(make_msg(node, name, position))
            nxt += fb_ready.COMMAND_PERIOD_SEC
        rclpy.spin_once(node, timeout_sec=0.002)
    end = time.time() + SETTLE_SEC
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def _sim_alive(node) -> bool:
    """`header.stamp` 가 진행하는지 본다. 멈춰 있으면 명령 판정이 무의미하다."""
    rows = []
    sub = node.create_subscription(
        JointState, fb_ready.REPORT_TOPIC,
        lambda m: rows.append((time.time(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)),
        50)
    end = time.time() + ALIVE_CHECK_SEC
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
    node.destroy_subscription(sub)
    if len(rows) < 2:
        print('[전제] /output/panda_joint 수신 없음 — Unity 미연결')
        return False
    stamp_span = rows[-1][1] - rows[0][1]
    recv_span = rows[-1][0] - rows[0][0]
    alive = stamp_span > 0.5 * recv_span
    print(f'[전제] 수신 {len(rows)}건, stamp {stamp_span:.2f}s / 실시간 {recv_span:.2f}s '
          f'→ {"시뮬레이션 진행 중" if alive else "**시뮬레이션이 멈춰 있다**"}')
    return alive


def check_arm(node) -> bool:
    """팔 명령 경로를 판정하고 **반드시 원래 자세로 되돌린다.**"""
    cur, pub = fb_ready.attach(node)
    base = fb_ready.wait_for_state(node, cur)
    print(f'[팔] 구독자 {node.count_subscribers(fb_ready.COMMAND_TOPIC)}, '
          f'기준 자세 {[round(v, 4) for v in base]}')
    responded = 0
    for label, idx, delta in ARM_TESTS:
        sign = fb_ready.free_direction(ARM[idx], base[idx])
        signed = sign * delta
        target = list(base)
        target[idx] += signed
        _drive(node, pub, ARM, target)
        moved = float(cur[ARM[idx]]) - base[idx]
        ratio = moved / signed if signed else 0.0
        ok = abs(moved) > MOVED_RAD
        responded += ok
        note = ''
        if ok and ratio < 0.5:
            note = '  ← 이동률이 낮다. 실효 한계 근처일 수 있다'
        print(f'[팔]   {label:<16} 지령 {signed:+.3f} → 실제 {moved:+.6f} rad '
              f'({ratio * 100:3.0f}%)  {"반응" if ok else "**무반응**"}{note}')
    _drive(node, pub, ARM, base)          # 원위치 — 진단이 자세를 바꾸면 안 된다
    final = [float(cur[j]) for j in ARM]
    print(f'[팔] 원위치 완료, 기준 대비 최대 차 '
          f'{max(abs(final[i] - base[i]) for i in range(7)):.6f} rad')
    return responded > 0


def check_gripper(node) -> bool:
    """그리퍼 명령 경로를 판정한다. **양방향으로 본다.**

    **반대 방향도 시험한다** — 물체에 막혀 못 열리는 것과 명령이 안 먹는 것은
    한 방향만 보면 구별되지 않는다. 실제로 개방 방향에만 저항이 있어(체크리스트 B-12)
    여는 쪽만 보면 명령 경로가 죽은 것으로 오진한다.
    """
    state, pub = fb_gripper.attach(node)
    fb_gripper.wait_for_state(node, state)
    base = list(state['pos'])
    span = fb_gripper.current_span(state)
    eff = fb_gripper.max_effort(state)
    # SignalChart 는 각도를 **도(deg)** 로 적는다. 대조하려면 같은 단위로 내야 한다.
    print(f'[그리퍼] 구독자 {node.count_subscribers(fb_gripper.COMMAND_TOPIC)}, '
          f's={span:.4f} rad = {math.degrees(span):.6f} deg, '
          f'최대 |effort| {eff:.3g} N·m ({fb_gripper.effort_state(eff)})')
    if eff > fb_gripper.DIVERGED_EFFORT_NM or eff != eff:
        print('[그리퍼] **경고: effort 가 물리적으로 불가능한 크기다.** 명령 반응 판정보다')
        print('[그리퍼]   물리 발산을 먼저 해결해야 한다. 모델·차트 설정을 되돌린다.')
    # 여는 쪽과 닫는 쪽 모두 여유가 남는 값을 고른다.
    open_s = max(fb_gripper.S_OPEN, span - 0.15)
    close_s = min(fb_gripper.S_CLOSE, span + 0.05)
    # 예전에는 `name` 을 채운 것과 비운 것을 나눠 시험했다. 명령 타입이
    # `Float64MultiArray` 라 **그 필드가 없으므로** 구분이 사라졌다.
    trials = [('닫는 방향', close_s), ('여는 방향', open_s)]
    responded = 0
    for label, target_s in trials:
        # **여유가 없는 시험은 건너뛴다.** 이미 한계에 붙어 있으면 움직일 여지가 없어
        # 무반응으로 찍히는데, 그것은 명령 경로의 문제가 아니다.
        if abs(target_s - span) <= MOVED_RAD:
            print(f'[그리퍼]   {label:<20} s→{target_s:.3f}  현재값과 같아 건너뜀')
            continue
        _drive(node, pub, GRIP_NAMES, [target_s * p for p in fb_gripper.PATTERN],
               make_msg=_gripper_command)
        moved = max(abs(a - b) for a, b in zip(state['pos'], base))
        ok = moved > MOVED_RAD
        responded += ok
        print(f'[그리퍼]   {label:<20} s→{target_s:.3f}  최대 이동 {moved:.6f} rad  '
              f'{"반응" if ok else "**무반응**"}')
        if ok:
            break                          # 하나라도 먹으면 경로는 살아 있다
    return responded > 0


def main() -> None:
    arm_only = fb_ready.take_flag('--arm-only')
    gripper_only = fb_ready.take_flag('--gripper-only')

    rclpy.init()
    node = rclpy.create_node('fb_cmdpath')
    print('=' * 78)
    print('명령 경로 판정 — 팔은 0.02~0.03 rad 만 움직이고 되돌린다')
    print('=' * 78)
    if not _sim_alive(node):
        raise SystemExit('시뮬레이션이 진행하지 않는다 — 명령 판정을 건너뛴다')

    arm_ok = None if gripper_only else check_arm(node)
    grip_ok = None if arm_only else check_gripper(node)

    print()
    if arm_ok is None or grip_ok is None:
        print('판정: 팔 %s / 그리퍼 %s' % (
            {True: '정상', False: '무반응', None: '(생략)'}[arm_ok],
            {True: '정상', False: '무반응', None: '(생략)'}[grip_ok]))
    elif arm_ok and grip_ok:
        print('판정: **정상** — 양쪽 경로 모두 명령을 받는다')
    elif arm_ok and not grip_ok:
        print('판정: **그리퍼만 무반응** — 팔이 대조군이므로 접촉·부하로는 설명되지 않는다.')
        print('      **A-4(고착)로 단정하지 말 것.** 2026-09-03 사례는 원인이 시뮬레이터가')
        print('      아니라 **모델 설정**이었고, Unity 재시작으로도 풀리지 않았다.')
        print('      먼저 확인할 것 — VariableModel/t1__panda_robotiq_ref_1_vm.xml 의')
        print('      <Input Type="SignalChart"> 가 가리키는 차트를 열어, 그 <Gripper> 절이')
        print('      유지하는 각도가 위에 찍힌 deg 값과 같은지 본다. 같으면 차트가 잡고')
        print('      있는 것이라 ROS 명령이 이길 수 없다 (실측: 37.242257 deg = 0.65 rad).')
    elif grip_ok and not arm_ok:
        print('판정: **팔만 무반응** — 관측된 적 없는 형태다. 차트의 <Arm> 절도 함께 본다.')
    else:
        print('판정: **양쪽 다 무반응 — Unity 수신 경로 전체 고착 (벤더 A-4)**')
        print('      이쪽이 진짜 A-4 다. 복구 수단은 Unity 재시작뿐이다.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
