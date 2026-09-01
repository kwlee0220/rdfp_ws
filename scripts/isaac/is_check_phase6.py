#!/usr/bin/env python3
"""Isaac 백엔드 **Phase 6 수용 기준**(물리 파지)을 검사한다.

    0. 재생 상태   — Isaac 이 Play 중인가
    1. 기준 자세   — 그리퍼를 열고 `ready` 로 **계획 없이** 되돌린다
    2. 접근·하강   — 블록 위로 이동한 뒤 파지 높이까지 내려간다
    3. 물림        — 손가락이 0 이 아니라 **블록 두께에서** 멈추는가
    4. 들어올림    — 블록이 그리퍼를 따라 올라오는가
    5. 유지        — 몇 초 뒤에도 떨어지지 않는가

**3번과 4번이 이 단계의 핵심이다.**

3번 — Phase 2 는 자유공간에서 손가락이 움직이는 것만 봤다. 닫힘(0.0)을 명령했을 때
손가락이 **0.0 까지 가버리면** 블록을 놓쳤거나 뚫고 지나간 것이고, 블록 반폭
(0.025) 근처에서 멈추면 실제로 문 것이다. 액션이 `stalled` 로 돌아오는 것이 정상이며
그것이 실패가 아니다 — `gripper_action_bridge` 가 그렇게 구분한다.

4번 — 문 것과 드는 것은 다르다. 마찰이 모자라면 손가락 사이에서 미끄러져 제자리에
남는다. `/scene/objects` 의 블록 z 가 그리퍼를 따라 올라오는지로 판정한다.

**1번이 계획을 쓰지 않는 이유가 있다.** 이 검사는 팔이 어디에 있든 시작할 수 있어야
하는데, 앞선 실행이 손가락을 테이블이나 블록 안에 남겨 두면 **시작 자세가 충돌**이라
MoveIt 이 계획 자체를 거부한다(`INVALID_MOTION_PLAN`, -2). scene 을 동기화한 뒤로 생긴
상황이며, 빠져나오려면 계획을 거치지 않는 경로가 필요하다 — 관절 명령을 직접
보간해 흘린다. 복구 뒤 `/check_state_validity` 로 충돌이 실제로 풀렸는지 확인한다.

**MoveIt 의 planning scene 과는 무관하다.** 여기서 보는 것은 PhysX 안의 물리다.
조작 대상 블록은 planning scene 에 넣지 않으므로(정적 물체만 넣는다) 파지에
attach/detach 관리가 필요 없고, 이 검사도 그것을 쓰지 않는다.

    ./is_check_phase6.py

종료 코드: 전부 통과하면 0, 하나라도 실패하면 1.
"""
from __future__ import annotations

from typing import Optional

import copy
import math
import time

import rclpy
import tf2_ros
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from control_msgs.action import GripperCommand as GripperCommandAction
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from rdfp_msgs.msg import SceneObjects
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

ARM_JOINT_NAMES = [f'panda_joint{i}' for i in range(1, 8)]
ARM_COMMAND_TOPIC = '/isaac/arm_command'
ANCHOR_POSE = 'ready'

BASE_FRAME = 'panda_link0'
# 카테시안 waypoint 는 group 의 tip link 기준이다. `panda_arm` 의 tip 은
# `panda_link8` 이며 `panda_hand` 와 **원점을 공유한다**(yaw 만 -45° 다르다).
TIP_FRAME = 'panda_link8'

# panda_link8 원점에서 손끝 중심까지의 거리(m). URDF 에서 finger 관절 원점이
# panda_hand 기준 z=0.0584 이고 손가락 메시가 거기서 ~0.045 더 뻗는다.
# 이 값이 틀리면 파지 높이가 그만큼 어긋나 블록을 밀어내거나 헛문다.
TCP_OFFSET = 0.1034

# 어느 블록을 집는가. **회전이 없는 것을 고른다** — block_b 는 z축 45° 회전이라
# 평행 조가 면이 아니라 모서리를 물게 된다(대각 0.0707 m 는 최대 개구 0.08 안에
# 들어와서 '집히긴 하는' 상태가 되어 판정을 흐린다).
TARGET_BLOCK = 'block_a'
BLOCK_HALF_WIDTH = 0.025

APPROACH_HEIGHT = 0.10     # 파지 높이 위로 이만큼에서 접근한다
LIFT_HEIGHT = 0.10         # 문 뒤 이만큼 들어올린다
HOLD_SEC = 3.0             # 들어올린 뒤 이만큼 버티는지 본다

OPEN_WIDTH = 0.04
CLOSED_WIDTH = 0.0
# 물린 폭이 이 범위 밖이면 파지가 아니다. 0 에 가까우면 놓친 것이고, 열림에
# 가까우면 닫히지 않은 것이다.
GRIP_WIDTH_MIN = 0.015
GRIP_WIDTH_MAX = 0.035
# 명령 상승분 대비 이만큼은 실제로 올라와야 "들렸다"고 본다.
MIN_LIFT = 0.07
# 유지 구간에서 이 이상 내려가면 미끄러진 것이다.
MAX_SLIP = 0.02

FINGER_JOINTS = ['panda_finger_joint1', 'panda_finger_joint2']
JOINT_STATE_TOPIC = '/joint_states'
SCENE_TOPIC = '/scene/objects'
GRIPPER_ACTION = '/panda_hand_controller/gripper_cmd'
DISCOVERY_TIMEOUT_SEC = 15.0
ACTION_TIMEOUT_SEC = 15.0
# 개루프 복귀에 쓸 시간·주파수. 느리게 흘려야 시뮬레이터가 따라온다.
RECOVER_SEC = 3.0
RECOVER_RATE = 50.0
RECOVER_TOLERANCE = 0.05


class Phase6Checker(Node):
    """블록·손가락 상태를 관측하고 그리퍼를 부르는 검사 노드."""

    def __init__(self) -> None:
        super().__init__('is_check_phase6',
                         parameter_overrides=[Parameter('use_sim_time', value=False)])
        self.positions: dict[str, float] = {}
        self.clock_samples: list[float] = []
        self.objects: dict[str, Pose] = {}

        self.create_subscription(JointState, JOINT_STATE_TOPIC, self._on_state, 50)
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        # `/scene/objects` 는 TRANSIENT_LOCAL 이다. volatile 로 구독하면 매칭 자체가
        # 되지 않아 값이 영영 오지 않는다 — 에러도 경고도 없다.
        self.create_subscription(
            SceneObjects, SCENE_TOPIC, self._on_scene,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # 계획을 거치지 않는 복귀 경로. MoveIt 이 시작 자세를 거부해도 여기는 통한다.
        self._arm = self.create_publisher(JointState, ARM_COMMAND_TOPIC, 10)
        self._validity = self.create_client(GetStateValidity, '/check_state_validity')
        self.action = ActionClient(self, GripperCommandAction, GRIPPER_ACTION)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    # ----- 콜백 ---------------------------------------------------------

    def _on_clock(self, msg: Clock) -> None:
        self.clock_samples.append(msg.clock.sec + msg.clock.nanosec * 1e-9)

    def _on_state(self, msg: JointState) -> None:
        if msg.name:
            self.positions.update(zip(msg.name, msg.position))

    def _on_scene(self, msg: SceneObjects) -> None:
        # 순서는 보장되지 않으므로 인덱스가 아니라 이름으로 담는다.
        for obj in msg.objects:
            self.objects[obj.name] = obj.pose

    # ----- 관측 ---------------------------------------------------------

    def spin(self, seconds: float) -> None:
        deadline = time.time() + seconds
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def width(self) -> float:
        return self.positions.get(FINGER_JOINTS[0], float('nan'))

    def block_z(self) -> float:
        pose = self.objects.get(TARGET_BLOCK)
        return pose.position.z if pose is not None else float('nan')

    def block_pose(self) -> Optional[Pose]:
        return self.objects.get(TARGET_BLOCK)

    def tip_pose(self, timeout_sec: float = 5.0) -> Optional[Pose]:
        """현재 tip link 의 pose 를 TF 로 읽는다.

        ``Time()`` 은 '가장 최근'을 뜻하며 노드 시계를 보지 않는다 — 이 노드가
        wall clock 인데 TF stamp 는 sim time 이어도 그대로 동작한다.
        """
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                tf = self.tf_buffer.lookup_transform(
                    BASE_FRAME, TIP_FRAME, rclpy.time.Time(),
                    timeout=Duration(seconds=0.0))
            except Exception:
                continue
            pose = Pose()
            pose.position.x = tf.transform.translation.x
            pose.position.y = tf.transform.translation.y
            pose.position.z = tf.transform.translation.z
            pose.orientation = tf.transform.rotation
            return pose
        return None

    # ----- 동작 ---------------------------------------------------------

    def send_gripper(self, position: float) -> Optional[GripperCommandAction.Result]:
        """그리퍼 액션 목표를 보내고 결과를 돌려준다. 거부되면 ``None``."""
        goal = GripperCommandAction.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 50.0
        send_future = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=ACTION_TIMEOUT_SEC)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=ACTION_TIMEOUT_SEC)
        wrapper = result_future.result()
        return wrapper.result if wrapper is not None else None

    def stream_to_joints(self, targets: dict) -> None:
        """현재 자세에서 목표까지 선형 보간해 관절 명령을 직접 흘린다.

        **MoveIt 을 거치지 않는다.** 시작 자세가 충돌이면 계획은 거부되지만 이
        경로는 통하므로, 박힌 상태에서 빠져나오는 유일한 수단이다. 개루프이므로
        도달은 호출자가 확인해야 한다.
        """
        names = list(targets)
        start = {j: self.positions.get(j, 0.0) for j in names}
        steps = max(1, int(RECOVER_SEC * RECOVER_RATE))
        period = 1.0 / RECOVER_RATE
        for i in range(steps + 1):
            ratio = i / steps
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = names
            msg.position = [start[j] + (targets[j] - start[j]) * ratio for j in names]
            self._arm.publish(msg)
            deadline = time.time() + period
            while rclpy.ok() and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.005)

    def state_is_valid(self) -> tuple[Optional[bool], list]:
        """현재 자세가 planning scene 과 충돌하는지 묻는다.

        ``(valid, 접촉쌍)`` 을 돌려준다. 서비스가 없으면 ``(None, [])``.
        """
        if not self._validity.wait_for_service(timeout_sec=5.0):
            return None, []
        request = GetStateValidity.Request()
        request.group_name = 'panda_arm'
        state = RobotState()
        state.joint_state.name = list(self.positions)
        state.joint_state.position = [self.positions[j] for j in self.positions]
        state.is_diff = False
        request.robot_state = state
        future = self._validity.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None:
            return None, []
        contacts = [f'{c.contact_body_1}<->{c.contact_body_2}' for c in response.contacts]
        return response.valid, contacts


def _fmt(ok: bool) -> str:
    return '통과' if ok else '실패'


def _hand_points_down(orientation) -> float:
    """tip 의 로컬 +z 축이 world -z 와 이루는 각도(도)를 돌려준다.

    파지 높이를 ``TCP_OFFSET`` 만큼 위로 잡는 계산은 손이 **아래를 본다**는 전제에
    기대고 있다. 전제가 깨지면 조용히 엉뚱한 높이를 집으므로 각도를 재서 알린다.
    """
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    # 회전행렬 3열 = 로컬 z축의 world 표현.
    zx = 2.0 * (x * z + w * y)
    zy = 2.0 * (y * z - w * x)
    zz = 1.0 - 2.0 * (x * x + y * y)
    # world -z 와의 내적.
    dot = max(-1.0, min(1.0, -zz / math.sqrt(zx * zx + zy * zy + zz * zz)))
    return math.degrees(math.acos(dot))


# ----- 검사 -------------------------------------------------------------


def check_playback(node: Phase6Checker) -> bool:
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


def _warn_home_pose_drift(srdf_targets: dict) -> None:
    """`isaac_scene.json` 의 `home_pose` 가 SRDF 와 어긋나면 알린다.

    Isaac 쪽 스크립트는 ROS 도 MoveIt 도 없는 곳에서 돌아 SRDF 를 읽을 수 없다.
    그래서 시작 자세가 두 곳에 적히는데, 어긋나면 **Play 직후의 자세만 조용히
    달라진다** — 검사는 개루프로 복구하므로 통과해 버린다. 여기서 대조한다.
    """
    import json
    import os

    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('robot_control'),
                            'config', 'isaac_scene.json')
        with open(path, encoding='utf-8') as f:
            home = json.load(f).get('home_pose') or {}
    except Exception:
        return
    if not home:
        return

    drift = {j: (v, srdf_targets[j]) for j, v in home.items()
             if j in srdf_targets and abs(v - srdf_targets[j]) > 1e-3}
    missing = sorted(set(srdf_targets) - set(home))
    if drift or missing:
        print('       주의: isaac_scene.json 의 home_pose 가 SRDF 와 다르다')
        for joint, (a, b) in sorted(drift.items()):
            print(f'          {joint}: json={a:+.4f} srdf={b:+.4f}')
        if missing:
            print(f'          json 에 없는 관절: {missing}')
        print('          → Play 직후 자세만 어긋난다. set_home_pose.py 재실행 전에 맞춘다')


def check_anchor(node: Phase6Checker, client) -> bool:
    """그리퍼를 열고 `ready` 로 되돌린 뒤, 충돌이 풀렸는지 확인한다.

    **계획을 쓰지 않는다.** 앞선 실행이 손가락을 물체 안에 남겨 두면 시작 자세가
    충돌이라 MoveIt 이 거부한다 — 그 상태에서 빠져나오는 것이 이 단계의 일이다.
    """
    if not node.action.wait_for_server(timeout_sec=DISCOVERY_TIMEOUT_SEC):
        print(f'  1. 기준 자세          : {_fmt(False)} — {GRIPPER_ACTION} 없음')
        print('       → enable_gripper:=true 로 스택을 띄웠는지 확인한다')
        return False

    # 먼저 연다. 조 사이에 물체가 낀 채로 팔을 올리면 물체를 끌고 간다.
    node.send_gripper(OPEN_WIDTH)
    node.spin(1.0)

    # SRDF 의 group_state 를 그대로 쓴다 — 값을 여기 적으면 출처가 둘이 된다.
    # 공개 API 에 '이름 -> 관절값' 접근자가 없어 캐시된 내부 조회를 쓴다.
    try:
        targets = client._get_named_state_joint_values(ANCHOR_POSE, timeout=DISCOVERY_TIMEOUT_SEC)
    except Exception as exc:
        print(f'  1. 기준 자세          : {_fmt(False)} — SRDF 조회 실패: '
              f'{type(exc).__name__}: {exc}')
        return False

    _warn_home_pose_drift(targets)

    node.stream_to_joints(targets)
    node.spin(1.0)

    worst = max((abs(node.positions.get(j, 0.0) - v) for j, v in targets.items()), default=9.9)
    reached = worst <= RECOVER_TOLERANCE
    opened = abs(node.width() - OPEN_WIDTH) <= 0.005
    valid, contacts = node.state_is_valid()

    ok = reached and opened and valid is not False
    print(f'  1. 기준 자세          : {_fmt(ok)} — {ANCHOR_POSE} 관절오차 {worst:.4f} rad, '
          f'그리퍼 폭 {node.width():.4f} m, 충돌없음 {valid}')
    if not reached:
        print('       → 개루프 복귀가 목표에 못 미쳤다. 팔 drive 게인을 확인한다')
    if not opened:
        print('       → 손가락이 열리지 않으면 물체가 조 사이에 낀 상태다')
    if valid is False:
        print(f'       → 아직 충돌 중이다: {", ".join(contacts[:4])}')
        print('          Isaac scene 을 이 자세로 저장해 두면 Stop/Play 로 깨끗이 시작한다')
    return ok


def check_approach(node: Phase6Checker, client) -> tuple[bool, Optional[Pose]]:
    """블록 위로 접근한 뒤 파지 높이까지 내려간다. 파지 자세를 함께 돌려준다."""
    node.spin(2.0)
    block = node.block_pose()
    if block is None:
        print(f'  2. 접근·하강          : {_fmt(False)} — {SCENE_TOPIC} 에 '
              f'{TARGET_BLOCK} 이 없다')
        print('       → enable_scene 과 Isaac 그래프의 PHASE(>=3)를 확인한다')
        return False, None

    tip = node.tip_pose()
    if tip is None:
        print(f'  2. 접근·하강          : {_fmt(False)} — TF {BASE_FRAME} -> {TIP_FRAME} 없음')
        return False, None

    tilt = _hand_points_down(tip.orientation)
    if tilt > 15.0:
        print(f'       주의: 손이 아래를 보지 않는다 ({tilt:.1f}°). '
              f'파지 높이 계산이 어긋난다')

    grasp = copy.deepcopy(tip)
    grasp.position.x = block.position.x
    grasp.position.y = block.position.y
    grasp.position.z = block.position.z + TCP_OFFSET

    pre = copy.deepcopy(grasp)
    pre.position.z += APPROACH_HEIGHT

    try:
        client.follow_trajectory_streamed([pre], publish_rate=50.0)
        node.spin(0.5)
        client.follow_trajectory_streamed([grasp], publish_rate=50.0)
        node.spin(1.0)
    except Exception as exc:
        print(f'  2. 접근·하강          : {_fmt(False)} — {type(exc).__name__}: {exc}')
        print('       → 계획 비율이 낮으면 블록이 팔의 작업영역 밖이다')
        return False, None

    reached = node.tip_pose()
    error = float('nan')
    if reached is not None:
        error = math.dist(
            (reached.position.x, reached.position.y, reached.position.z),
            (grasp.position.x, grasp.position.y, grasp.position.z))
    ok = error <= 0.02
    print(f'  2. 접근·하강          : {_fmt(ok)} — 목표 '
          f'({grasp.position.x:.3f}, {grasp.position.y:.3f}, {grasp.position.z:.3f}), '
          f'오차 {error:.4f} m, 손 기울기 {tilt:.1f}°')
    if not ok:
        print('       → 스트리밍은 개루프다. 도달 오차가 크면 drive 게인을 의심한다')
    return ok, grasp


def check_grip(node: Phase6Checker) -> bool:
    """닫기 명령을 보내고 손가락이 블록 두께에서 멈추는지 본다."""
    result = node.send_gripper(CLOSED_WIDTH)
    if result is None:
        print(f'  3. 물림               : {_fmt(False)} — 그리퍼 목표가 거부됐다')
        return False
    node.spin(1.0)

    width = node.width()
    ok = GRIP_WIDTH_MIN <= width <= GRIP_WIDTH_MAX
    print(f'  3. 물림               : {_fmt(ok)} — 폭 {width:.4f} m '
          f'(기준 {GRIP_WIDTH_MIN}~{GRIP_WIDTH_MAX}, 블록 반폭 {BLOCK_HALF_WIDTH}), '
          f'stalled={result.stalled}')
    if width < GRIP_WIDTH_MIN:
        print('       → 끝까지 닫혔다. 블록을 놓쳤거나 접근 위치가 어긋났다')
    elif width > GRIP_WIDTH_MAX:
        print('       → 닫히다 말았다. 손가락 drive 의 maxForce 를 확인한다')
    return ok


def check_lift(node: Phase6Checker, client, grasp: Pose) -> tuple[bool, bool]:
    """들어올리고, 버티는지 본다."""
    node.spin(1.0)
    before = node.block_z()

    lift = copy.deepcopy(grasp)
    lift.position.z += LIFT_HEIGHT
    try:
        client.follow_trajectory_streamed([lift], publish_rate=50.0)
    except Exception as exc:
        print(f'  4. 들어올림           : {_fmt(False)} — {type(exc).__name__}: {exc}')
        return False, False
    node.spin(1.5)

    peak = node.block_z()
    risen = peak - before
    lifted = risen >= MIN_LIFT
    print(f'  4. 들어올림           : {_fmt(lifted)} — 블록 z {before:.4f} -> {peak:.4f} '
          f'(+{risen:.4f} m, 명령 +{LIFT_HEIGHT}, 기준 ≥ {MIN_LIFT})')
    if not lifted:
        print('       → 그리퍼만 올라가고 블록은 남았다. 손끝 마찰과 파지력을 본다')
        print('          (tune_grasp.py 를 Stop 상태에서 돌리고 다시 Play 했는지 확인)')

    node.spin(HOLD_SEC)
    held = node.block_z()
    slip = peak - held
    holding = lifted and slip <= MAX_SLIP
    print(f'  5. 유지 ({HOLD_SEC:.0f}초)        : {_fmt(holding)} — '
          f'z {peak:.4f} -> {held:.4f} (낙하 {slip:.4f} m, 기준 ≤ {MAX_SLIP})')
    if lifted and not holding:
        print('       → 들렸다가 미끄러졌다. 마찰이 모자라거나 파지력이 약하다')

    return lifted, holding


def main() -> int:
    from rclpy.parameter import Parameter as RclParameter

    from robot_control.moveit.move_group_factory import create_move_group_client

    rclpy.init()
    node = Phase6Checker()
    print('Isaac Phase 6 수용 기준 검사 (물리 파지)')

    results = [check_playback(node)]
    planner = None
    client = None
    try:
        if results[0]:
            planner = rclpy.create_node(
                'is_check_phase6_planner',
                parameter_overrides=[RclParameter('use_sim_time', value=True)])
            client = create_move_group_client(
                planner, mode='jgpc', arm_command_topic=ARM_COMMAND_TOPIC,
                arm_command_joint_names=ARM_JOINT_NAMES, arm_command_format='joint_state')

            results.append(check_anchor(node, client))
        if all(results):
            approached, grasp = check_approach(node, client)
            results.append(approached)
        if all(results):
            results.append(check_grip(node))
        if all(results):
            lifted, holding = check_lift(node, client, grasp)
            results.extend([lifted, holding])
    finally:
        if planner is not None:
            planner.destroy_node()
        node.destroy_node()

    passed = sum(1 for r in results if r)
    print(f'\n{passed}/{len(results)} 통과')
    rclpy.shutdown()
    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
