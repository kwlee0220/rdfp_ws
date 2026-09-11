#!/usr/bin/env python3
"""그리퍼를 열고 닫는다 — B-12 ~ B-14 측정과 실험 전 접촉 해제에 함께 쓴다.

**6축을 모두 채워 보낸다.** 시뮬레이터는 mimic 을 강제하지 않으므로 구동 관절 하나만
보내면 나머지가 따라오지 않는다 (체크리스트 B-12 / open_work §7 ④). 명령은
``s * [+1, +1, -1, -1, -1, +1]`` 형태이며 ``s`` 는 0(열림) ~ 0.725(닫힘)다.

**`effort` 로 접촉을 판정한다.** 빈손 기준선은 0.004 N·m, 파지는 17.19 N·m 다
(체크리스트 B-14). **1 N·m 를 넘으면 무언가에 닿아 있다** — 그 상태로 팔을 움직이면
물체를 끌고 가거나 픽스처를 쓰러뜨린다. 그래서 B 단계를 시작하기 전에 이 스크립트로
먼저 열어 접촉을 끊는다.

시뮬레이터가 내부 보간을 하지 않으므로 목표를 한 번에 던지지 않고 50 Hz 로 램프한다.

    ./fb_gripper.py open              # s → 0
    ./fb_gripper.py close             # s → 0.725
    ./fb_gripper.py 0.30              # s 직접 지정
    ./fb_gripper.py open --ramp 3     # 천천히 (물체를 쥐고 있을 때)

인자: [open|close|<s>] [--ramp <초>, 기본 2] [--hold <초>, 기본 3]
"""
from __future__ import annotations

from typing import Optional

import math
import sys
import time

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

GRIPPER_JOINTS = [f'gripper_joint{i}' for i in range(1, 7)]
# 구동값 s 에 곱하는 부호. 실측 패턴 (open_work §7 ④).
PATTERN = [1.0, 1.0, -1.0, -1.0, -1.0, 1.0]
S_OPEN = 0.0
S_CLOSE = 0.725
REPORT_TOPIC = '/output/gripper_joint'        # sensor_msgs/JointState
# **명령은 `std_msgs/Float64MultiArray` 다 — 팔과 타입이 다르다.** 팔의
# `/input/panda_joint` 는 `JointState` 라서 그 패턴을 복사하면 조용히 깨진다. ROS 2 는
# 같은 이름에 다른 타입의 퍼블리셔를 허용하고 **연결만 안 될 뿐 오류를 내지 않으므로**,
# 증상이 "그리퍼가 ROS 명령을 무시한다"로 보인다 (2026-09-03 에 실제로 그렇게 오진해
# 체크리스트 B-12~14 를 '측정 불가'로 닫았다). 타입을 바꾸기 전에 반드시
# `ros2 topic info /input/gripper_joint` 로 시뮬레이터가 구독하는 타입을 확인한다.
COMMAND_TOPIC = '/input/gripper_joint'        # std_msgs/Float64MultiArray
RATE_HZ = 50.0
COMMAND_PERIOD_SEC = 1.0 / RATE_HZ
DEFAULT_RAMP_SEC = 2.0
DEFAULT_HOLD_SEC = 3.0
# 이 값을 넘는 |effort| 는 접촉으로 본다 (빈손 0.004 / 파지 17.19 N·m).
CONTACT_EFFORT_NM = 1.0
# 이 값을 넘으면 접촉이 아니라 **수치 발산**이다. 물리적으로 나올 수 없는 크기다.
# 2026-09-03 에 시그널차트의 <Gripper> 절을 주석 처리했더니 PD 제어기가 참조를 잃고
# 4.1e28 N·m 로 폭발했다. 이걸 '접촉'으로 읽으면 원인을 영영 못 찾는다.
DIVERGED_EFFORT_NM = 1.0e3

# **씬마다 그리퍼 규약이 다르다 — 보고 축 수로 판별한다** (2026-09-10 실측).
#
# `t1`(Crisp) 은 6관절을 그대로 라디안으로 받는데, `r2`(RecurDyn) 은 2관절이고
# **관절당 (position, velocity, force) 3값을 평평하게 이어 붙이며 각도가 도(degree)** 다.
# 씬 XML 의 `<Gripper Number="N">` 이 관절 수가 아니라 **값 개수**(관절 수 × 3)인 것과
# 맞는다 — t1 은 18(=6×3), r2 는 6(=2×3) 이다.
#
# 단위를 몰라서 라디안을 보내면 `π/180 = 1.745%` 만 움직이는데, 에러가 아니라 "지령이
# 거의 반영되지 않는다"로 보인다. 벤더 요청서에 열려 있던 *"그리퍼 지령 미반영
# 1.6~4%"* 가 정확히 이것이었다. 도로 바꿔 보내면 오차 0.0001 rad 이하로 선다.
#
# **팔은 라디안이다** — 같은 로봇의 두 채널이 각도 단위가 다르다.
GRIPPER_PROFILES = {
    6: {'label': 't1/Crisp 6축', 'signs': PATTERN, 'degrees': False, 'pvf': False},
    2: {'label': 'r2/RecurDyn 2축', 'signs': [1.0, -1.0], 'degrees': True, 'pvf': True}
}


def profile_for(state: dict) -> dict:
    """보고된 축 수로 규약을 고른다. 모르는 축 수는 **거부한다.**"""
    count = len(state.get('pos') or [])
    if count not in GRIPPER_PROFILES:
        raise SystemExit(
            f'{REPORT_TOPIC} reports {count} axes; known conventions are '
            f'{sorted(GRIPPER_PROFILES)}. Add a profile before driving this gripper.')
    return GRIPPER_PROFILES[count]


def effort_state(value: float) -> str:
    """최대 |effort| 를 사람이 읽는 판정으로 바꾼다."""
    if value != value or value > DIVERGED_EFFORT_NM:      # NaN 또는 발산
        return '**수치 발산 — 물리가 깨졌다**'
    return '접촉 중' if value > CONTACT_EFFORT_NM else '접촉 없음'


def make_command(positions, profile: Optional[dict] = None) -> Float64MultiArray:
    """관절 목표각(**라디안**)을 명령 메시지로 감싼다.

    `Float64MultiArray` 에는 header 도 관절 이름도 없다 — **배열 순서가 곧 계약**이며
    그 순서는 `profile` 이 정한다 (`GRIPPER_PROFILES` 주석 참조).

    `profile` 이 `None` 이면 값을 그대로 싣는다 — 규약 변환 없이 배열을 직접 만드는
    호출자(`fb_cmdpath`)를 위한 경로다.
    """
    values = [float(v) for v in positions]
    if profile is not None:
        if profile['degrees']:
            values = [math.degrees(v) for v in values]
        if profile['pvf']:
            # 관절당 (position, velocity, force). 속도·힘은 0 으로 둔다 — 위치 지령만
            # 쓰며, 힘 슬롯에 0 이 아닌 값을 넣으면 그리퍼가 물체를 밀어붙인다.
            values = [v for value in values for v in (value, 0.0, 0.0)]
    # ⚠️ **NaN·inf 를 절대 내보내지 않는다.** 시뮬레이터가 이 값을 솔버 상태에 넣으면
    # 그리퍼만이 아니라 **전 관절이 NaN 으로 발산하고 되돌아오지 않는다** — 복구는
    # `ros_tcp_endpoint` 재시작뿐이다. 2026-09-09 에 실제로 그랬다: 2축 빌드에서
    # `current_span()` 이 축 수 불일치로 NaN 을 돌려주었고, 그것이 램프 시작값으로
    # 들어가 명령 6개를 통째로 오염시켰다. 팔 스크립트는 전부 `goto_ready()` 에서
    # 그리퍼를 열므로 **팔 시험을 시작하는 것만으로 시뮬레이터가 죽었다.**
    if not all(math.isfinite(v) for v in values):
        raise SystemExit(
            f'refusing to publish a non-finite gripper command: {values} — '
            f'this diverges the simulator and only an endpoint restart recovers it. '
            f'Check {REPORT_TOPIC} and the matching GRIPPER_PROFILES entry.')
    msg = Float64MultiArray()
    msg.data = values
    return msg


def attach(node) -> tuple[dict[str, list], object]:
    """그리퍼 상태 구독과 명령 발행자를 붙인다."""
    state: dict[str, list] = {}
    node.create_subscription(
        JointState, REPORT_TOPIC,
        lambda m: state.update({'pos': list(m.position), 'eff': list(m.effort),
                                'vel': list(m.velocity)}), 50)
    pub = node.create_publisher(Float64MultiArray, COMMAND_TOPIC, 10)
    return state, pub


def wait_for_state(node, state: dict[str, list], timeout_sec: float = 5.0) -> dict[str, list]:
    deadline = time.time() + timeout_sec
    while rclpy.ok() and time.time() < deadline and not state:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not state:
        raise SystemExit(f'no JointState received on {REPORT_TOPIC}')
    return state


def max_effort(state: dict[str, list]) -> float:
    eff = state.get('eff') or []
    return max((abs(e) for e in eff), default=0.0)


def current_span(state: dict[str, list], profile: Optional[dict] = None) -> float:
    """보고된 관절값에서 구동값 s 를 되짚는다 (부호를 벗겨 평균낸다).

    **보고는 어느 규약에서도 라디안이다** — 도로 바꾸는 것은 명령 방향뿐이다.
    `profile` 을 안 주면 축 수로 찾고, 모르는 축 수면 `NaN` 을 준다.
    """
    pos = state.get('pos') or []
    signs = (profile or GRIPPER_PROFILES.get(len(pos), {})).get('signs')
    if not signs or len(pos) != len(signs):
        return float('nan')
    return sum(p * s for p, s in zip(pos, signs)) / len(signs)


def set_span(node, state: dict[str, list], pub, target_s: float,
             ramp_sec: float = DEFAULT_RAMP_SEC, hold_sec: float = DEFAULT_HOLD_SEC,
             verbose: bool = True) -> Optional[float]:
    """구동값 s 를 현재값에서 목표까지 램프한 뒤 유지한다. 최대 |effort| 를 돌려준다."""
    wait_for_state(node, state)
    # **규약을 보고 축 수로 정한다.** 모르는 축 수면 `profile_for` 가 거부한다 — 그냥
    # 진행하면 `current_span()` 이 NaN 을 주고 그 NaN 이 명령으로 나간다.
    profile = profile_for(state)
    start_s = current_span(state, profile)
    before_eff = max_effort(state)
    if verbose:
        print(f'[gripper] 규약 {profile["label"]} '
              f'({"도" if profile["degrees"] else "라디안"}, '
              f'{"관절당 p/v/f" if profile["pvf"] else "위치만"})')
        print(f'[gripper] s {start_s:.4f} → {target_s:.4f}, {ramp_sec:.1f}초 램프')
        print(f'[gripper] 이동 전 최대 |effort| = {before_eff:.3g} N·m '
              f'({effort_state(before_eff)})')

    started = time.time()
    next_publish = started
    while rclpy.ok():
        elapsed = time.time() - started
        if elapsed > ramp_sec:
            break
        if time.time() >= next_publish:
            alpha = min(elapsed / ramp_sec, 1.0) if ramp_sec > 0 else 1.0
            s = start_s + (target_s - start_s) * alpha
            pub.publish(make_command([s * v for v in profile['signs']], profile))
            next_publish += COMMAND_PERIOD_SEC
        rclpy.spin_once(node, timeout_sec=0.002)

    hold_msg = make_command([target_s * v for v in profile['signs']], profile)
    deadline = time.time() + hold_sec
    next_publish = time.time()
    while rclpy.ok() and time.time() < deadline:
        if time.time() >= next_publish:
            pub.publish(hold_msg)
            next_publish += COMMAND_PERIOD_SEC
        rclpy.spin_once(node, timeout_sec=0.002)

    after_eff = max_effort(state)
    if verbose:
        print(f'[gripper] 도착 s={current_span(state, profile):.4f}, '
              f'최대 |effort| = {after_eff:.3g} N·m ({effort_state(after_eff)})')
        print(f'[gripper] 관절: {[round(v, 4) for v in (state.get("pos") or [])]}')
        print(f'[gripper] effort: {[round(v, 3) for v in (state.get("eff") or [])]}')
    return after_eff


def release_contact(node, ramp_sec: float = DEFAULT_RAMP_SEC,
                    verbose: bool = True) -> float:
    """그리퍼를 열어 접촉을 끊는다. 팔을 움직이기 전에 부른다."""
    state, pub = attach(node)
    return set_span(node, state, pub, S_OPEN, ramp_sec=ramp_sec, verbose=verbose)


def _parse_opt(name: str, default: float) -> float:
    if name in sys.argv:
        i = sys.argv.index(name)
        value = float(sys.argv[i + 1])
        del sys.argv[i:i + 2]
        return value
    return default


def main() -> None:
    ramp_sec = _parse_opt('--ramp', DEFAULT_RAMP_SEC)
    hold_sec = _parse_opt('--hold', DEFAULT_HOLD_SEC)
    arg = sys.argv[1] if len(sys.argv) > 1 else 'open'
    target = {'open': S_OPEN, 'close': S_CLOSE}.get(arg)
    if target is None:
        target = float(arg)
    if not 0.0 <= target <= S_CLOSE:
        raise SystemExit(f's must be within [0, {S_CLOSE}], got {target}')

    rclpy.init()
    node = rclpy.create_node('fb_gripper')
    state, pub = attach(node)
    set_span(node, state, pub, target, ramp_sec=ramp_sec, hold_sec=hold_sec)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
