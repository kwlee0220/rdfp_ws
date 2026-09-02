#!/usr/bin/env python3
"""Isaac 백엔드 **Phase 3 수용 기준**(scene 객체)을 검사한다.

    0. 재생 상태   — Isaac 이 Play 중인가
    1. 토픽 수신   — /scene/objects 가 오는가 (QoS 매칭 포함)
    2. 물체 목록   — JSON 정의의 물체가 모두, 이름·종류·크기까지 맞게 오는가
    3. 좌표 변환   — pose 가 **panda_link0 기준**인가 (world 값과 다른지로 판정)
    4. 쿼터니언    — 회전 없는 물체가 (0,0,0,1) 로, 45° 회전 물체가 맞게 오는가

**4번이 이 단계의 핵심이다.** Isaac 은 wxyz(스칼라 우선), ROS 는 xyzw 다. 순서를
틀려도 norm 이 1 이라 어떤 검사도 통과하고 결과는 '그럴듯하게 틀린 자세'가 된다.
그래서 norm 이 아니라 **알려진 자세**로 검증한다 — `rdfp_msgs/SceneObject.msg` 가
지시하는 방법이다.

    ./is_check_phase3.py

종료 코드: 전부 통과하면 0, 하나라도 실패하면 1.
"""
from __future__ import annotations

import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from ament_index_python.packages import get_package_share_directory
from rdfp_msgs.msg import SceneObjects
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

SCENE_TOPIC = '/scene/objects'
BASE_FRAME = 'panda_link0'
DISCOVERY_TIMEOUT_SEC = 15.0
POSITION_TOLERANCE = 0.02
QUAT_TOLERANCE = 0.02
DIMENSION_TOLERANCE = 1e-6

_SCENE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class Phase3Checker(Node):
    def __init__(self) -> None:
        super().__init__('is_check_phase3',
                         parameter_overrides=[Parameter('use_sim_time', value=False)])
        self.latest: SceneObjects = None
        self.clock_samples: list[float] = []
        self.create_subscription(SceneObjects, SCENE_TOPIC, self._on_scene, _SCENE_QOS)
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        # Isaac 이 실제로 어떤 프레임 이름으로 내보내는지 모아 둔다. 물체가 안 잡힐 때
        # "이름이 다른 것"과 "TF 자체가 없는 것"을 가르는 데 쓴다.
        self.tf_frames: set = set()
        self.create_subscription(TFMessage, '/tf', self._on_tf, 50)

    def _on_clock(self, msg: Clock) -> None:
        self.clock_samples.append(msg.clock.sec + msg.clock.nanosec * 1e-9)

    def _on_tf(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            self.tf_frames.add(transform.child_frame_id)

    def _on_scene(self, msg: SceneObjects) -> None:
        self.latest = msg


def _fmt(ok: bool) -> str:
    return '통과' if ok else '실패'


def _load_config() -> dict:
    path = os.path.join(get_package_share_directory('robot_control'),
                        'config', 'isaac_scene.json')
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def check_playback(node: Phase3Checker) -> bool:
    deadline = time.time() + DISCOVERY_TIMEOUT_SEC
    advancing = False
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.clock_samples) >= 2 and node.clock_samples[-1] > node.clock_samples[0]:
            advancing = True
            break
    print(f'  0. 재생 상태          : {_fmt(advancing)} — /clock 샘플 {len(node.clock_samples)}개')
    if not advancing:
        print('       → Isaac 이 정지 상태다. **Play(▶) 를 누른다.**')
    return advancing


def check_topic(node: Phase3Checker) -> bool:
    deadline = time.time() + DISCOVERY_TIMEOUT_SEC
    while rclpy.ok() and time.time() < deadline and node.latest is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    ok = node.latest is not None
    detail = (f'물체 {len(node.latest.objects)}개, frame_id={node.latest.header.frame_id!r}'
              if ok else '수신 없음')
    print(f'  1. {SCENE_TOPIC:<18}: {_fmt(ok)} — {detail}')
    if not ok:
        # publisher 를 **발견은 했는지**가 갈림길이다.
        #   0개  → 전송/디스커버리 문제 (서로 못 본다)
        #   1개+ → QoS 불일치 (보이는데 데이터가 안 온다)
        publishers = node.count_publishers(SCENE_TOPIC)
        print(f'       발견된 publisher: {publishers}개')
        if publishers:
            print('       → publisher 는 보이는데 데이터가 안 온다 = QoS 불일치.')
            print('          /scene/objects 는 RELIABLE / TRANSIENT_LOCAL / depth=1 이다')
        else:
            print('       → publisher 자체가 안 보인다. enable_scene:=true 인지, '
                  '노드가 살아 있는지 확인한다')
    # 프레임 이름은 통과 여부와 무관하게 항상 보고한다 — 물체가 안 잡힐 때
    # "이름이 다르다"와 "TF 가 아예 없다"를 즉시 가른다.
    robot_frames = {f for f in node.tf_frames if f.startswith('panda_')}
    other_frames = sorted(node.tf_frames - robot_frames)
    print(f'       /tf 프레임: 로봇 {len(robot_frames)}개, 그 외 {other_frames}')
    return ok


def check_objects(node: Phase3Checker, config: dict) -> bool:
    # **`dynamic: true` 인 것만 기대한다.** `/scene/objects` 는 조작 대상 채널이고,
    # 발행 노드가 환경 물체(탁자 등)를 걸러낸다 (2026-09-01 결정,
    # docs/scene/scene_objects_guide.md). 이 필터가 생기기 전에 쓰인 검사라 전체를
    # 기대했고, 그래서 정상 동작이 '누락: table' 로 나왔다 (2026-09-02 재검증).
    expected = {obj['name']: obj for obj in config['objects'] if obj.get('dynamic', False)}
    received = {obj.name: obj for obj in node.latest.objects}

    missing = sorted(set(expected) - set(received))
    problems: list[str] = []
    for name, spec in expected.items():
        obj = received.get(name)
        if obj is None:
            continue
        if obj.type != spec['type']:
            problems.append(f'{name}: type {obj.type!r} != {spec["type"]!r}')
        if len(obj.dimensions) != len(spec['dimensions']) or any(
                abs(a - b) > DIMENSION_TOLERANCE
                for a, b in zip(obj.dimensions, spec['dimensions'])):
            problems.append(f'{name}: dimensions {list(obj.dimensions)} != {spec["dimensions"]}')

    ok = not missing and not problems
    print(f'  2. 물체 목록          : {_fmt(ok)} — 기대 {len(expected)}개 / 수신 {len(received)}개')
    for name in missing:
        print(f'       누락: {name}')
    for problem in problems:
        print(f'       {problem}')
    return ok


def check_frame(node: Phase3Checker, config: dict) -> bool:
    """pose 가 world 가 아니라 로봇 베이스 기준인가."""
    if node.latest.header.frame_id != BASE_FRAME:
        print(f'  3. 좌표 기준          : {_fmt(False)} — frame_id '
              f'{node.latest.header.frame_id!r} != {BASE_FRAME!r}')
        return False

    # world 기준 값(JSON)과 수신 값이 같으면 변환이 빠진 것이다. 현재 스택은
    # world → panda_link0 가 항등이라 값 자체는 같을 수 있으므로, 다르면 확실히
    # 변환된 것이고 같으면 '항등 변환'으로 보고 통과시킨다 — 판정은 frame_id 로 한다.
    received = {obj.name: obj for obj in node.latest.objects}
    rows = []
    for spec in config['objects']:
        obj = received.get(spec['name'])
        if obj is None:
            continue
        delta = max(abs(obj.pose.position.x - spec['position'][0]),
                    abs(obj.pose.position.y - spec['position'][1]),
                    abs(obj.pose.position.z - spec['position'][2]))
        rows.append(f'{spec["name"]}:{delta:.3f}')
    print(f'  3. 좌표 기준          : {_fmt(True)} — frame_id={BASE_FRAME}, '
          f'world 정의와의 차이 {" ".join(rows)}')
    return True


def check_quaternion(node: Phase3Checker, config: dict) -> bool:
    """알려진 자세로 쿼터니언 규약을 검증한다 (norm 으로는 못 잡는다)."""
    received = {obj.name: obj for obj in node.latest.objects}
    ok = True
    for spec in config['objects']:
        obj = received.get(spec['name'])
        if obj is None:
            continue
        want = spec['orientation']
        got = [obj.pose.orientation.x, obj.pose.orientation.y,
               obj.pose.orientation.z, obj.pose.orientation.w]
        # q 와 -q 는 같은 회전이다.
        diff = min(max(abs(a - b) for a, b in zip(want, got)),
                   max(abs(a + b) for a, b in zip(want, got)))
        good = diff <= QUAT_TOLERANCE
        ok = ok and good
        angle = 2.0 * math.degrees(math.acos(min(1.0, abs(got[3]))))
        print(f'       {spec["name"]:>8} xyzw={["%+.3f" % v for v in got]} '
              f'(z축 {angle:5.1f}°) {"" if good else "← 기대 " + str(want)}')
    print(f'  4. 쿼터니언 규약      : {_fmt(ok)} — 기대값과 xyzw 로 일치하는가')
    if not ok:
        print('       → wxyz/xyzw 순서가 뒤집혔을 수 있다. norm 검사로는 잡히지 않는다')
    return ok


def main() -> int:
    rclpy.init()
    node = Phase3Checker()
    config = _load_config()
    print('Isaac Phase 3 수용 기준 검사')

    results = [check_playback(node)]
    if results[0]:
        results.append(check_topic(node))
    if all(results):
        results.append(check_objects(node, config))
        results.append(check_frame(node, config))
        results.append(check_quaternion(node, config))
    node.destroy_node()

    passed = sum(1 for r in results if r)
    print(f'\n{passed}/{len(results)} 통과')
    rclpy.shutdown()
    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
