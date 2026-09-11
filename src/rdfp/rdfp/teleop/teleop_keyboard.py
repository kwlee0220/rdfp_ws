#!/usr/bin/env python3
"""Keyboard Cartesian teleop with gripper, episode, and task management.

Extends the base keyboard teleop (panda_servo_teleop) with:
- Gripper open/close (g/h)
- Episode start/end ([/])
- Task selection (1-5) and outcome marking (=/-).
"""
import sys
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import Int8
from rdfp_msgs.msg import GripperCommand

from robot_control.moveit import backends, create_move_group_client
from std_srvs.srv import Trigger

from robot_control.moveit.servo_client import ServoClient
from robot_control.ros2_utils import get_parameter, parse_float, parse_str, parse_str_list
from ..session import SessionControlClient
from .key_hold import HoldKeyTracker, TerminalRawMode


@dataclass
class KeyMapping:
    # Linear (Cartesian, TwistStamped 로 publish)
    forward: str = "j"        # +x
    back: str = "l"           # -x
    left: str = "i"           # +y
    right: str = "k"          # -y
    up: str = "q"             # +z
    down: str = "a"           # -z

    # Joint (panda_joint1 단일 조인트, JointJog 로 publish)
    joint1_plus: str = "'"    # (+joint1)
    joint1_minus: str = ";"   # (-joint1)

    # Angular (Cartesian, TwistStamped 의 angular 축)
    yaw_left: str = "y"    # +z
    yaw_right: str = "h"   # -z
    pitch_up: str = "t"    # +y
    pitch_down: str = "g"  # -y
    roll_left: str = "r"   # +x
    roll_right: str = "f"  # -x

    # Control
    deadman: str = " "    # space
    stop: str = "x"
    # '/' 키 — MoveIt SRDF 의 'ready' named target 으로 이동
    home: str = "/"

    # Gripper
    # '=' 는 벌어진 간격(open), '-' 는 줄어든 간격(close) 의미로 매핑한다.
    # '\\' 는 grasp — close 와 **판정이 다르다**. close 는 목표 자세 도달로,
    # grasp 는 `stalled`(물체에 막혀 더 안 닫힘)로 성공을 판정한다. 물건을 집을
    # 때는 grasp 를 써야 `at_goal` 이 제대로 선다 (docs/gripper/GripperNode_Design.md).
    gripper_open: str = "="
    gripper_close: str = "-"
    gripper_grasp: str = "\\"

    # Session (calls start_session / stop_session services).
    # '<' / '>' 는 Shift 조합, ',' / '.' 는 Shift 없이도 쓸 수 있도록 alias 로 제공한다.
    session_start: str = "<"
    session_start_alt: str = ","
    session_stop: str = ">"
    session_stop_alt: str = "."

    # Episode (calls start_episode / stop_episode services)
    episode_start: str = "["
    episode_end: str = "]"

    # Task clear (calls set_task_label with empty task_label)
    task_clear: str = "0"


HELP_TEXT = """
Keyboard Twist Teleop (forgeflow)
─────────────────────────────────────
  Motion (deadman auto-refresh):
    j/l: +x/-x   i/k: +y/-y   q/a: +z/-z
    ;/': panda_joint1 -/+
    r/f: roll    t/g: pitch   y/h: yaw
  Gripper:
    =: open   -: close   \\: grasp (물체를 집을 때)
  Session (split services on session_control):
    < or ,: start_session   > or .: stop_session
    [: start_episode   ]: stop_episode
  Task (set_task_label service):
    1-N: select task from 'tasks' parameter (see log below)
    0: clear task
  Control:
    SPACE: deadman   x: stop   /: move to 'ready' pose   Ctrl-C: quit
  Note: Servo auto-starts on initialization
  Note: If keys stop moving the arm, read the 'servo status' warning below —
        servo halts on joint limits and singularities. '/' recovers (MoveIt, not servo).
─────────────────────────────────────"""

_DEFAULT_TASK_LIST = ["touch", "pick_and_place", "push", "stack", "wipe"]
_SERVO_NODE_NAME = "/servo_node"  # Default name for the servo node to control
_DELTA_TWIST_TOPIC = "/servo_node/delta_twist_cmds"  # Default topic for TwistStamped commands
# JGPC 기본 명령 토픽. auto 판별도 이 토픽의 유무를 본다.
_DEFAULT_ARM_COMMAND_TOPIC = "/panda_arm_controller/commands"
# JTC 실행 경로의 액션. 서버 부재 확인에 쓴다.

# 백엔드별 arm 명령 채널 프로파일.
#
# `auto` 판별은 `/panda_arm_controller/commands` 토픽의 유무만 본다. 그 이름을 쓰지
# 않는 백엔드(펑션베이의 `/input/panda_joint`)는 알려주지 않으면 찾을 방법이 없고,
# 그러면 JTC 로 오판해 `/`(ready 이동)가 `CONTROL_FAILED`(-4) 로 실패한다 — 계획은
# 되고 실행만 안 된다.
#
# 그래서 값 세 개를 매번 손으로 주는 대신 `backend` 하나로 고른다.
_DELTA_JOINT_TOPIC = "/servo_node/delta_joint_cmds"  # JointJog topic for joint-level servo

# servo 가 매 주기 내는 자기 진단(`moveit_servo/StatusCode`). **이것을 안 보면
# 특이점·관절한계로 servo 가 스스로 멈춘 상황이 "키가 죽었다"로만 보인다** —
# 실제로 Isaac 파지 자세(`panda_joint6` 하한 -0.0873 에 근접)에서 그렇게 오진했다.
# 명령을 넣는 동안에만 갱신되므로 `ros2 topic echo --once` 로 확인하면 직전 값이
# latch 된 것을 보게 된다. 그래서 노드가 상시 구독해 전이 시점에 알린다.
_SERVO_STATUS_TOPIC = "/servo_node/status"

# 코드별 (이름, 조치). 조치가 비어 있으면 이름만 알린다.
_SERVO_STATUS_TEXT: dict = {
    -1: ("INVALID", "servo reported an invalid state"),
    1: ("DECELERATE_FOR_APPROACHING_SINGULARITY",
        "motion is scaled down near a singularity and may deviate from the commanded "
        "direction; press '/' to return to 'ready'"),
    2: ("HALT_FOR_SINGULARITY", "servo stopped at a singularity; press '/' to return to 'ready'"),
    3: ("DECELERATE_FOR_COLLISION", "motion is scaled down near a collision"),
    4: ("HALT_FOR_COLLISION", "servo stopped at a collision; press '/' to return to 'ready'"),
    5: ("JOINT_BOUND",
        "a joint reached its limit and servo halts ALL motion, including jogs away from "
        "the limit; press '/' to return to 'ready'"),
    6: ("DECELERATE_FOR_LEAVING_SINGULARITY", "motion is scaled down while leaving a singularity"),
}
_JOINT1_NAME = "panda_joint1"  # 조인트 단위 서보 대상 (좌/우 화살표 매핑)

# GripperNode 가 구독하는 명령 토픽. 이 토픽이 곧 학습 데이터의 action 채널이다.
# 키 입력을 **심볼**로 바꿔 발행한다 — 숫자는 그리퍼에 종속이라 다른 기구로 옮기면
# 틀린 값이 된다 (2026-09-01 결정, rdfp_msgs/msg/GripperCommand.msg 참조).
_GRIPPER_CMD_TOPIC = "/gripper_cmds"

# 키보드로 보낼 수 있는 그리퍼 심볼. 숫자는 `GripperNode` 의 `targets`
# 파라미터가 갖는다 — 명령 메시지에는 의도만 실린다.
#
# `grasp` 를 넣지 않은 것은 키보드 teleop 이 빈손 개폐가 기본 용도이기 때문이다.
# 필요하면 키를 하나 더 배정하고 여기에 추가한다.
# `GripperNode` 가 받는 심볼. **여기 없으면 발행 자체가 안 되고 경고만 남는다.**
# 노드 쪽 `targets.<goal>` 과 짝이 맞아야 한다 — 모르는 심볼은 노드가 거부한다.
_GRIPPER_LABELS = ("open", "close", "grasp")

_DEFAULT_RATE_HZ = 100.0
# deadman TTL 기본값. 마지막 모션 키 입력(자동반복 포함) 이후 이 시간만큼만
# 모션을 유지하고 만료되면 멈춘다. 너무 길면 키를 뗀 뒤에도 오래 코스팅하고,
# 너무 짧으면 OS 키보드 자동반복 간격(~33ms)을 못 메워 홀드 중 끊긴다.
# 100Hz 타이머 + 30Hz 자동반복 기준으로 자동반복 간격은 메우면서 정지는 빠르게
# 가져가도록 0.06s 로 둔다.
_DEFAULT_DEADMAN_TTL_SEC = 0.06


class TeleopKeyboard(Node):
    """Publishes TwistStamped, gripper commands, and dataset/task events."""

    # 안전 상한. **단위가 m/s 가 아니다** — servo 를 `command_in_type: unitless` 로
    # 쓰므로 twist 성분은 [-1, 1] 의 무차원 비율이고, 실제 속도는 servo 의
    # `scale.linear` 가 정한다 (Isaac 0.4 에서 입력 1.0 이 130 mm/s — 2026-09-07 래칫
    # 수정 후 값. 수정 전엔 되읽기가 이동량을 먹어 47.5 였다. CLAUDE.md 참조).
    # 상한이 기본값과 같으면 값을 올리는 순간 `_validate_parameters` 가 ValueError 로
    # 노드를 죽이므로, 무차원 천장(1.0) 아래에 여유를 두고 잡는다.
    MAX_LINEAR_VELOCITY = 0.8   # unitless - 경고/검증 임계
    MAX_ANGULAR_VELOCITY = 1.57  # rad/s - maximum safe angular velocity (π/2)

    # Emergency limits (even more conservative)
    EMERGENCY_LINEAR_LIMIT = 1.0   # unitless - servo 의 무차원 입력 천장, 하드 클램프
    EMERGENCY_ANGULAR_LIMIT = 3.14  # rad/s - absolute emergency stop limit (π)

    def __init__(self):
        super().__init__("teleop_keyboard")

        # --- Parameters ---
        self.declare_parameter("frame_id", "panda_link0")
        self.declare_parameter("rate_hz", _DEFAULT_RATE_HZ)
        # 기본 0.25 (2026-09-07). servo 의 래칫을 끊자 같은 입력에 직선 2.7배·회전 2.3배
        # 빨라져서(docs/teleop/servo_vs_planned_motion.md §4.1) 0.6 이던 것을 되맞췄다 —
        # 0.25 는 직선 32 mm/s · 회전 2.9 °/s 로 수정 전 체감(28.5 mm/s · 3.0 °/s)에 가깝다.
        # `scale.linear` 쪽을 낮추지 않는 이유: 비례가 유지되고, 그쪽은 servo 재기동이 필요하며,
        # 다른 servo 소비자(ee_twist·teleop_retarget)의 보정이 함께 흔들린다.
        self.declare_parameter("linear_step", 0.25)
        self.declare_parameter("angular_step", 0.25)
        self.declare_parameter("deadman_ttl_sec", _DEFAULT_DEADMAN_TTL_SEC)
        self.declare_parameter("tasks", _DEFAULT_TASK_LIST)
        # 빈 값이 "미설정" 이다 — 프로파일이 채우게 두고, 명시하면 그것이 이긴다.
        # **기본값이 없다.** 어느 스택에 붙는지는 사람이 안다 — 짐작하게 두면
        # 틀린 짐작이 조용히 통과하고, 증상은 "'/' 키가 안 먹는다" 로만 보인다.
        self.declare_parameter("backend", Parameter.Type.STRING)

        self.frame_id = get_parameter(self, "frame_id", parse_str, default="panda_link0")
        self.rate_hz = get_parameter(self, "rate_hz", parse_float, default=_DEFAULT_RATE_HZ)
        self.linear_step = get_parameter(self, "linear_step", parse_float, default=0.25)
        self.angular_step = get_parameter(self, "angular_step", parse_float, default=0.25)
        self.deadman_ttl_sec = get_parameter(self, "deadman_ttl_sec", parse_float,
                                             default=_DEFAULT_DEADMAN_TTL_SEC)
        self.tasks = get_parameter(self, "tasks",  parse_str_list, default=_DEFAULT_TASK_LIST)

        # --- arm 명령 채널 ('/' = ready 이동에 쓰인다) ---
        # **`backend` 하나로 끝난다.** 어느 스택이 JTC 인가와 명령 채널은
        # `robot_control.moveit.BACKEND_PROFILES` 가 갖고, 오판 경고도 팩토리가 낸다.
        #
        # 채널을 개별로 받던 파라미터 넷은 걷어냈다 — 실제로 값을 넣던 곳이 모두
        # 프로파일과 같은 값을 손으로 다시 적고 있었고, 그것이 표를 둔 이유를
        # 무너뜨린다. 새 스택은 프로파일에 한 항목을 더한다.
        self.backend = self._require_backend()
        self.get_logger().info(f"arm command channel: backend={self.backend}")

        # --- Publishers ---
        self.twist_pub = self.create_publisher(TwistStamped, _DELTA_TWIST_TOPIC, 10)
        # panda_joint1 단일 조인트 서보용 JointJog 퍼블리셔
        self.joint_jog_pub = self.create_publisher(JointJog, _DELTA_JOINT_TOPIC, 10)

        # --- Gripper command publisher ---
        # 하드웨어 접점은 GripperNode 가 담당한다. 본 노드는 키 입력을
        # 심볼로 바꿔 발행만 한다.
        self._gripper_pub = self.create_publisher(GripperCommand, _GRIPPER_CMD_TOPIC, 10)

        # --- servo 자기 진단 구독 ---
        # servo 는 매 주기 status 를 내므로 **전이 시점에만** 알린다. 매번 찍으면
        # 100 Hz 로 도배되어 오히려 안 보인다.
        self._servo_status = 0
        self.create_subscription(Int8, _SERVO_STATUS_TOPIC, self._on_servo_status, 10)

        # --- SessionControlClient (비동기 API 사용) ---
        # **없어도 뜬다.** `session_control_node` 는 수집 계층(rdfp) 소속이라 제어
        # 스택만 띄운 경우에는 존재하지 않는다. 그래도 텔레오퍼레이션 자체는 아무
        # 문제가 없으므로 세션 키만 비활성으로 두고 나머지는 그대로 쓴다.
        #
        # `create()` 의 기본 동작은 5 개 서비스를 기다리다 `RuntimeError` 를 내는
        # 것이라, 대기를 끄고 `wait_until_ready` 로 예외 없이 확인한다.
        session_wait_sec = float(
            self.declare_parameter("session_wait_sec", 2.0).value)
        self._session_client = SessionControlClient.create(self, wait_timeout_sec=0.0)
        self._session_available = self._session_client.wait_until_ready(session_wait_sec)
        if not self._session_available:
            self.get_logger().warning(
                "session_control services not found — session/episode/task keys are "
                "disabled. Start the collection layer if you need them "
                "(e.g. ros2 launch rdfp rdfp_collect.launch.py)")

        # --- Servo utils ---
        self.servo_utils = ServoClient.create(self, _SERVO_NODE_NAME)

        # --- MoveGroup 클라이언트 (Home 키 → 'ready' named target 이동) ---
        self._move_group = self._create_move_group()
        # Home 이동 진행 중 플래그. True 이면 timer 가 twist/joint_jog 발행을
        # 건너뛰어 move_group trajectory 와 충돌하지 않게 한다.
        self._home_in_progress = False

        # --- Key mapping ---
        self.keys = KeyMapping()
        self.motion_keys = {
            self.keys.forward, self.keys.back, self.keys.left, self.keys.right,
            self.keys.up, self.keys.down,
            self.keys.yaw_left, self.keys.yaw_right,
            self.keys.pitch_up, self.keys.pitch_down,
            self.keys.roll_left, self.keys.roll_right,
            self.keys.joint1_plus, self.keys.joint1_minus,
        }
        # Task selection keys: '1' through len(tasks)
        self.task_keys = {str(i + 1): name for i, name in enumerate(self.tasks[:9])}

        # session_control 서버가 있어야 동작하는 키 전체. 세션·에피소드·작업 라벨이
        # 모두 같은 노드의 서비스라 하나로 묶는다.
        km = self.keys
        self._session_keys = {
            km.session_start, km.session_start_alt,
            km.session_stop, km.session_stop_alt,
            km.episode_start, km.episode_end,
            km.task_clear,
            *self.task_keys.keys(),
        }

        # --- Twist state ---
        self.twist = TwistStamped()
        self.twist.header.frame_id = self.frame_id

        # --- JointJog state (panda_joint1 단독) ---
        self.joint_jog = JointJog()
        self.joint_jog.header.frame_id = self.frame_id
        self.joint_jog.joint_names = [_JOINT1_NAME]
        self.joint_jog.velocities = [0.0]

        self.get_logger().info(
            HELP_TEXT if self._session_available
            else HELP_TEXT.replace(
                "  Session (split services on session_control):",
                "  Session — DISABLED (session_control not found):"))
        if self.tasks:
            task_list = "  ".join(f"{i+1}:{name}" for i, name in enumerate(self.tasks[:9]))
            self.get_logger().info(f"Tasks: {task_list}")

        # --- Timer ---
        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.01
        self.timer = self.create_timer(period, self._on_timer)

        # --- 홀드(누른 동안만 움직임) ---
        # 터미널은 '키를 뗐다'를 안 알려 준다. 그 추정과 만료 시 0 을 한 번 보내는
        # 규칙을 `HoldKeyTracker` 가 갖는다 (`key_hold.py` 모듈 설명 참조).
        self._holds = HoldKeyTracker(self.motion_keys, ttl_sec=self.deadman_ttl_sec,
                                     hold_keys=(self.keys.deadman,))

        # Validate all parameters
        self._validate_parameters()

        # Auto-start servo after initialization
        self._setup_servo_auto_start()

    def destroy_node(self):
        super().destroy_node()

    # ── Keyboard input ──────────────────────────────────────────

    # ── Twist helpers ───────────────────────────────────────────

    def _zero_twist(self):
        self.twist.twist.linear.x = 0.0
        self.twist.twist.linear.y = 0.0
        self.twist.twist.linear.z = 0.0
        self.twist.twist.angular.x = 0.0
        self.twist.twist.angular.y = 0.0
        self.twist.twist.angular.z = 0.0

    def _apply_key_to_twist(self, key: str):
        self._zero_twist()
        ls = self.linear_step
        ang = self.angular_step
        km = self.keys

        if key == km.forward:
            self.twist.twist.linear.x = +ls
        elif key == km.back:
            self.twist.twist.linear.x = -ls
        elif key == km.left:
            self.twist.twist.linear.y = +ls
        elif key == km.right:
            self.twist.twist.linear.y = -ls
        elif key == km.up:
            self.twist.twist.linear.z = +ls
        elif key == km.down:
            self.twist.twist.linear.z = -ls
        elif key == km.roll_left:
            self.twist.twist.angular.x = +ang
        elif key == km.roll_right:
            self.twist.twist.angular.x = -ang
        elif key == km.pitch_up:
            self.twist.twist.angular.y = +ang
        elif key == km.pitch_down:
            self.twist.twist.angular.y = -ang
        elif key == km.yaw_left:
            self.twist.twist.angular.z = +ang
        elif key == km.yaw_right:
            self.twist.twist.angular.z = -ang

        # Apply velocity saturation for safety
        self._saturate_twist()

    # ── JointJog helpers (panda_joint1 단독 서보) ────────────────

    def _zero_joint_jog(self):
        self.joint_jog.velocities = [0.0]

    def _apply_key_to_joint_jog(self, key: str):
        """;/' 키를 panda_joint1 각속도로 변환한다."""
        self._zero_joint_jog()
        ang = self.angular_step
        km = self.keys
        if key == km.joint1_plus:
            self.joint_jog.velocities = [+ang]
        elif key == km.joint1_minus:
            self.joint_jog.velocities = [-ang]

    def _apply_key_to_motion(self, key: str):
        """키 입력을 twist 또는 joint_jog 로 분기한다. 두 명령은 상호 배타적으로
        한 쪽만 활성화되며, 나머지는 0 으로 유지된다."""
        km = self.keys
        if key in (km.joint1_plus, km.joint1_minus):
            self._zero_twist()
            self._apply_key_to_joint_jog(key)
        else:
            self._zero_joint_jog()
            self._apply_key_to_twist(key)

    # ── Gripper helpers ─────────────────────────────────────────

    def _call_gripper(self, command: str) -> None:
        """키 입력을 심볼 명령으로 `GripperNode` 에 발행한다.

        **숫자를 싣지 않는다.** 목표 폭(m) 은 그리퍼에 종속이라 노드의 `targets`
        파라미터가 갖는다.

        결과는 기다리지 않는다 — 하드웨어를 다루는 것은 `GripperNode` 의 몫이며,
        성패는 `/gripper_states` 의 `at_goal` 로 확인한다 (`/joint_states` 의
        finger joint 는 백엔드에 따라 거짓말을 한다).
        """
        if command not in _GRIPPER_LABELS:
            self.get_logger().warning(f"[gripper] unknown command {command!r}")
            return

        msg = GripperCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.goal = command
        self._gripper_pub.publish(msg)

        self.get_logger().info(f"[gripper] {command}")

    # ── Home helpers (MoveIt named target 'ready' 이동) ─────────────

    def _require_backend(self) -> str:
        """`backend` 파라미터를 읽는다. **없으면 뜨지 않는다.**

        기본값을 두면 틀린 짐작이 조용히 통과한다 — 예컨대 jgpc mock 스택에서 JTC 로
        잡히면 계획은 되고 실행만 안 되어 "'/' 키가 안 먹는다" 로만 보인다.
        """
        try:
            value = self.get_parameter("backend").value
        except Exception:       # noqa: BLE001 — 미설정이면 rclpy 가 예외를 던진다
            value = None
        if not value:
            raise ValueError(
                f"the 'backend' parameter is required; pick one of {list(backends())} "
                "(e.g. --ros-args -p backend:=isaac)")
        if value not in backends():
            raise ValueError(f"backend must be one of {list(backends())}, got {value!r}")
        return str(value)

    def _create_move_group(self):
        """`backend` 프로파일로 MoveGroup 클라이언트를 만든다.

        **어느 스택이 JTC 인가는 `robot_control.moveit` 이 안다** — 프로파일 표와
        JTC 오판 경고가 모두 팩토리에 있다. 여기서는 이름만 넘긴다.
        """
        return create_move_group_client(self, backend=self.backend)

    def _call_home(self) -> None:
        """MoveIt SRDF 의 'ready' named target 으로 이동 요청을 보낸다.

        이동은 비동기로 수행되며, 완료까지 수 초 가량 소요된다. 진행 중에는
        twist/joint_jog 발행이 중단되어 move_group trajectory 와의 충돌을
        방지한다. 재진입은 무시한다.
        """
        if self._home_in_progress:
            self.get_logger().warning("[home] already in progress, ignoring")
            return
        self._home_in_progress = True
        # 이동 동안 servo 입력이 컨트롤러를 방해하지 않도록 twist/joint_jog 를 0 으로 초기화.
        self._zero_twist()
        self._zero_joint_jog()

        # **발행을 멈추는 것만으로는 부족하다 — servo 자체를 정지시켜야 한다.**
        # 명령이 끊기면 servo 는 `incoming_command_timeout`(0.1s) 뒤 halt 궤적을
        # `num_outgoing_halt_msgs_to_publish`(4)회 **스스로 발행한다.** 그 궤적이
        # 같은 컨트롤러 토픽으로 가서 **move_group 궤적을 선점**하므로, 키로 팔을
        # 움직인 직후에 '/' 를 누르면 ready 로 가지 않는다 (실측: servo 를 멈추지
        # 않으면 복귀가 반복 실패, 멈추면 전부 성공).
        #
        # 서비스 호출은 **비동기**여야 한다. `ServoClient.stop()` 은 내부에서
        # `rclpy.spin_once` 를 돌리는데 여기는 타이머 콜백 안이라 재진입이 된다.
        # `pause`/`unpause` 는 쓰지 않는다 — `unpause_servo` 가 성공을 반환하면서도
        # 발행을 되살리지 못한다(scripts/isaac/is_recover.py).
        stop_cli = self.servo_utils.stop_client
        if stop_cli.service_is_ready():
            stop_cli.call_async(Trigger.Request()).add_done_callback(
                lambda _f: self._start_home_move())
        else:
            self.get_logger().warning("[home] stop_servo not available; moving anyway")
            self._start_home_move()

    def _start_home_move(self) -> None:
        """servo 정지 뒤 실제 이동을 건다."""
        try:
            future = self._move_group.move_to_named_target_async("ready")
        except Exception as e:   # noqa: BLE001
            self._home_in_progress = False
            self._resume_servo()
            self.get_logger().error(f"[home] failed to start: {e}")
            return
        future.add_done_callback(self._on_home_done)
        self.get_logger().info("[home] moving to 'ready' pose...")

    def _resume_servo(self) -> None:
        """이동이 끝났으니 servo 를 되살린다 (비동기, fire-and-forget)."""
        start_cli = self.servo_utils.start_client
        if start_cli.service_is_ready():
            start_cli.call_async(Trigger.Request())
        else:
            self.get_logger().warning("[home] start_servo not available; 이동 키가 안 먹을 수 있다")

    def _on_home_done(self, future) -> None:
        """'ready' 이동 완료 콜백. 결과를 로그에 남기고 플래그를 해제한다."""
        try:
            exc = future.exception()
            if exc is not None:
                self.get_logger().error(f"[home] failed: {exc}")
            else:
                self.get_logger().info("[home] reached 'ready' pose")
        except Exception as e:   # noqa: BLE001
            self.get_logger().error(f"[home] callback error: {e}")
        finally:
            self._resume_servo()
            self._home_in_progress = False

    # ── Session 서비스 완료 콜백 ──────────────────────────────────

    def _on_servo_status(self, msg: Int8) -> None:
        """servo status 가 바뀔 때만 알린다."""
        code = int(msg.data)
        if code == self._servo_status:
            return
        previous, self._servo_status = self._servo_status, code
        if code == 0:
            self.get_logger().info("servo status cleared (NO_WARNING)")
            return
        name, advice = _SERVO_STATUS_TEXT.get(code, (f"UNKNOWN({code})", ""))
        detail = f" — {advice}" if advice else ""
        self.get_logger().warning(f"servo status: {name}{detail} (was {previous})")

    def _on_trigger_done(self, label: str, success: bool, message: str) -> None:
        """Trigger 서비스(start/stop session/episode) 비동기 호출 완료 콜백."""
        if success:
            self.get_logger().info(self._success_message(label))
        else:
            reason = message or "rejected by server"
            self.get_logger().warning(
                f"Failed to execute '{label}': {reason}"
            )

    def _on_set_task_done(self, requested: str, success: bool, message: str) -> None:
        """set_task_label 서비스 비동기 호출 완료 콜백."""
        if success:
            if requested:
                self.get_logger().info(f"Task set to '{requested}'")
            else:
                self.get_logger().info("Task cleared")
        else:
            reason = message or "rejected by server"
            target = f"'{requested}'" if requested else "(clear)"
            self.get_logger().warning(
                f"Failed to set task to {target}: {reason}"
            )

    @staticmethod
    def _success_message(label: str) -> str:
        """명령별 성공 로그 메시지."""
        messages = {
            "start_session": "Session started",
            "stop_session": "Session stopped",
            "start_episode": "Episode started",
            "stop_episode": "Episode stopped",
        }
        return messages.get(label, f"Command '{label}' succeeded")

    # ── One-shot key handlers ───────────────────────────────────

    def _handle_oneshot_key(self, key: str) -> bool:
        """Handle keys that fire once per press. Returns True if handled."""
        km = self.keys

        # Gripper — GripperNode 가 구독하는 명령 토픽에 발행.
        if key == km.gripper_open:
            self._call_gripper("open")
            return True
        if key == km.gripper_close:
            self._call_gripper("close")
            return True
        if key == km.gripper_grasp:
            self._call_gripper("grasp")
            return True

        # Home — MoveIt 'ready' named target 으로 이동
        if key == km.home:
            self._call_home()
            return True

        # 세션 계열은 서버가 있어야 한다. 없으면 **조용히 무시하지 않고** 알린다 —
        # 키가 먹지 않는 것과 서버가 없는 것을 구분할 수 없으면 원인을 찾을 수 없다.
        if key in self._session_keys and not self._session_available:
            self.get_logger().warning(
                f"'{key}' ignored: session_control not available")
            return True

        # Session lifecycle
        if key in (km.session_start, km.session_start_alt):
            self._session_client.start_session_async(
                done_callback=lambda ok, msg: self._on_trigger_done("start_session", ok, msg),
            )
            return True
        if key in (km.session_stop, km.session_stop_alt):
            self._session_client.stop_session_async(
                done_callback=lambda ok, msg: self._on_trigger_done("stop_session", ok, msg),
            )
            return True

        # Episode lifecycle
        if key == km.episode_start:
            self._session_client.start_episode_async(
                done_callback=lambda ok, msg: self._on_trigger_done("start_episode", ok, msg),
            )
            return True
        if key == km.episode_end:
            # outcome/metadata 를 비워 호출한다 — 키 하나로 끝내는 UI 라 성패를 줄
            # 수단이 없고, 그 결과가 곧 '판정 없음'(DB 의 success=NULL)이다.
            self._session_client.stop_episode_async(
                done_callback=lambda ok, msg: self._on_trigger_done("stop_episode", ok, msg),
            )
            return True

        # Task clear
        if key == km.task_clear:
            self._session_client.set_task_label_async(
                task_label=None,
                done_callback=lambda ok, msg: self._on_set_task_done("", ok, msg),
            )
            return True

        # Task selection (1-N)
        if key in self.task_keys:
            label = self.task_keys[key]
            self._session_client.set_task_label_async(
                task_label=label,
                done_callback=lambda ok, msg, t=label: self._on_set_task_done(t, ok, msg),
            )
            return True

        # Help commands ('h' 는 yaw_right angular 매핑에 사용되므로 '?' 만 사용)
        if key == '?':
            self.get_logger().info(self._generate_help_message())
            return True

        return False

    # ── Main timer callback ─────────────────────────────────────

    def _on_timer(self):
        """Timer callback with comprehensive exception handling."""
        try:
            dt = self.timer.timer_period_ns * 1e-9
            state = self._holds.tick(dt)

            # 모션·홀드 키가 아닌 것은 트래커가 그대로 넘겨준다 — 순서를 유지한다.
            for key in state.other_keys:
                try:
                    if self._handle_oneshot_key(key):
                        continue

                    if key not in self._get_all_valid_keys():
                        self._handle_unknown_key(key)
                        continue

                    # 정지 키: 홀드를 끊고 0 을 직접 한 번 보낸 뒤 이번 틱을 끝낸다.
                    # 트래커의 `just_released` 로 또 보내면 0 이 두 번 나가 servo 의
                    # 정지 판정이 오히려 늦어지므로 `cancel()` 이 그것을 막는다.
                    if key == self.keys.stop:
                        self._holds.cancel()
                        self._publish_zero()
                        return

                except Exception as e:
                    self.get_logger().warning(f"Failed to process key '{key}': {e}")
                    # Continue execution - don't let key processing errors stop the timer

            # **Home 이동 중에는 servo 로 아무것도 보내지 않는다.** 0 이라도 보내면
            # servo 가 깨어나 같은 컨트롤러 토픽에 궤적을 내고 **move_group 궤적을
            # 선점한다** — '/' 를 눌러도 ready 로 안 가는 증상이 된다. 그래서 아래
            # 홀드 처리보다 **먼저** 걸러야 한다.
            if self._home_in_progress:
                return

            if state.just_released:
                # 침묵하지 않고 0 을 **한 번** 보낸다. 근거는 `key_hold` 모듈 설명.
                self._publish_zero()
                return
            if not state.active:
                return

            # 홀드 중: 마지막 모션 키를 명령으로 바꾼다. `key` 가 None 이면 홀드 키
            # (SPACE)만 눌린 상태이므로 0 을 유지한다.
            try:
                if state.key is None:
                    self._zero_twist()
                    self._zero_joint_jog()
                else:
                    self._apply_key_to_motion(state.key)

                self._publish_twist_safely()
                self._publish_joint_jog_safely()

            except Exception as e:
                self.get_logger().warning(f"Failed to generate/publish motion: {e}")
                # Publish zero twist + joint_jog as safety fallback
                try:
                    self._publish_zero()
                except Exception as fallback_e:
                    self.get_logger().error(f"Critical: Even safety fallback failed: {fallback_e}")

        except Exception as e:
            # Catch-all for any other unexpected errors
            self.get_logger().error(f"Critical timer callback error: {e}")
            # Don't re-raise - let the timer continue running

    def _publish_zero(self):
        """twist 와 joint_jog 를 0 으로 채워 함께 발행한다."""
        self._zero_twist()
        self._zero_joint_jog()
        self._publish_twist_safely()
        self._publish_joint_jog_safely()

    def _publish_twist_safely(self):
        """Safely publish twist message with proper error handling."""
        try:
            # Apply final safety check before publishing
            self._saturate_twist()

            # Only update timestamp - frame_id is set once during initialization
            self.twist.header.stamp = self.get_clock().now().to_msg()
            self.twist_pub.publish(self.twist)
        except Exception as e:
            self.get_logger().error(f"Failed to publish twist message: {e}")
            # Don't re-raise - this is a non-critical error for safety

    def _publish_joint_jog_safely(self):
        """Safely publish JointJog message with proper error handling."""
        try:
            self.joint_jog.header.stamp = self.get_clock().now().to_msg()
            self.joint_jog_pub.publish(self.joint_jog)
        except Exception as e:
            self.get_logger().error(f"Failed to publish joint_jog message: {e}")

    def _saturate_twist(self):
        """Apply velocity saturation to ensure safe operation.

        Clamps all velocity components to safe limits and logs warnings
        if values exceed normal operating ranges.
        """
        # Get current twist values
        linear = self.twist.twist.linear
        angular = self.twist.twist.angular

        # Check for excessive velocities (log warnings)
        if (abs(linear.x) > self.MAX_LINEAR_VELOCITY
                or abs(linear.y) > self.MAX_LINEAR_VELOCITY
                or abs(linear.z) > self.MAX_LINEAR_VELOCITY):
            self.get_logger().warning(
                "High linear command detected: "
                f"[{linear.x:.3f}, {linear.y:.3f}, {linear.z:.3f}] (unitless)"
            )

        if (abs(angular.x) > self.MAX_ANGULAR_VELOCITY
                or abs(angular.y) > self.MAX_ANGULAR_VELOCITY
                or abs(angular.z) > self.MAX_ANGULAR_VELOCITY):
            self.get_logger().warning(
                "High angular velocity detected: "
                f"[{angular.x:.3f}, {angular.y:.3f}, {angular.z:.3f}] rad/s"
            )

        # Apply emergency limits (hard clamp)
        linear.x = max(-self.EMERGENCY_LINEAR_LIMIT, min(self.EMERGENCY_LINEAR_LIMIT, linear.x))
        linear.y = max(-self.EMERGENCY_LINEAR_LIMIT, min(self.EMERGENCY_LINEAR_LIMIT, linear.y))
        linear.z = max(-self.EMERGENCY_LINEAR_LIMIT, min(self.EMERGENCY_LINEAR_LIMIT, linear.z))

        angular.x = max(-self.EMERGENCY_ANGULAR_LIMIT, min(self.EMERGENCY_ANGULAR_LIMIT, angular.x))
        angular.y = max(-self.EMERGENCY_ANGULAR_LIMIT, min(self.EMERGENCY_ANGULAR_LIMIT, angular.y))
        angular.z = max(-self.EMERGENCY_ANGULAR_LIMIT, min(self.EMERGENCY_ANGULAR_LIMIT, angular.z))

    # ── State helpers ────────────────────────────────────────────

    def _get_all_valid_keys(self) -> set:
        """Get all valid keys that can be processed."""
        km = self.keys
        valid_keys = {
            # Motion keys (Cartesian)
            km.forward, km.back, km.left, km.right, km.up, km.down,
            km.yaw_left, km.yaw_right, km.pitch_up, km.pitch_down,
            km.roll_left, km.roll_right,
            # Joint-level motion keys
            km.joint1_plus, km.joint1_minus,
            # Control keys
            km.deadman, km.stop, km.home,
            # Gripper keys
            km.gripper_open, km.gripper_close, km.gripper_grasp,
            # Session keys (primary + alt)
            km.session_start, km.session_stop,
            km.session_start_alt, km.session_stop_alt,
            # Episode keys
            km.episode_start, km.episode_end,
            # Task clear key
            km.task_clear,
        }

        # Add task selection keys (1-9)
        valid_keys.update(self.task_keys.keys())

        return valid_keys

    def _get_key_category(self, key: str) -> str:
        """Get the category of a valid key for better help."""
        km = self.keys

        if key in self.motion_keys:
            return "motion"
        elif key in [km.deadman, km.stop]:
            return "control"
        elif key in [km.gripper_open, km.gripper_close, km.gripper_grasp]:
            return "gripper"
        elif key in [km.session_start, km.session_stop,
                     km.session_start_alt, km.session_stop_alt]:
            return "session"
        elif key in [km.episode_start, km.episode_end]:
            return "episode"
        elif key == km.task_clear:
            return "task_clear"
        elif key in self.task_keys:
            return "task_selection"
        else:
            return "unknown"

    def _generate_help_message(self) -> str:
        """Generate a concise help message with valid keys."""
        task_hint = "  ".join(
            f"{i + 1}:{name}" for i, name in enumerate(self.tasks[:9])
        )
        return f"""
Valid Keys:
  Motion: j/l (±x), i/k (±y), q/a (±z), ;/' (panda_joint1 ±)
  Angular: r/f (roll), t/g (pitch), y/h (yaw)
  Control: SPACE (deadman), x (stop), / (move to 'ready')
  Gripper: = (open), - (close), \\ (grasp)
  Session: < or , (start_session), > or . (stop_session)
  Episode: [ (start_episode), ] (stop_episode)
  Tasks: {task_hint}   0 (clear)
  Note: Servo auto-starts on initialization
  Press Ctrl+C to quit
        """.strip()

    def _handle_unknown_key(self, key: str):
        """Handle unknown key input with helpful feedback."""
        # Filter out common non-printable characters
        if not key.isprintable() or key in ['\n', '\r', '\t', '\x1b']:
            return  # Silently ignore non-printable keys

        suggestions: list[str] = []

        # Check for common typos or similar keys
        key_lower = key.lower()
        if key_lower in ['z', 'c', 'v', 'b', 'n', 'm']:
            suggestions.append("Try 'x' to stop, SPACE as deadman")
        elif key_lower in ['1', '2', '3', '4', '5', '6', '7', '8', '9']:
            if key in self.task_keys:
                return  # Valid task key, shouldn't be here
            elif int(key) <= len(self.tasks):
                suggestions.append(f"Task {key} exists - this might be a processing error")
            else:
                suggestions.append(f"Only tasks 1-{len(self.tasks)} are available")
        elif key_lower in ['[', ']', '{', '}', '<', '>', ',', '.']:
            suggestions.append(
                "Session: '<'/',' start_session, '>'/'.' stop_session; "
                "Episode: '[' start_episode, ']' stop_episode"
            )
        elif key_lower.isalpha() and key_lower not in ['j', 'l', 'i', 'k', 'q', 'a',
                                                       'r', 'f', 't', 'g', 'y', 'h',
                                                       'x']:
            suggestions.append(
                "Motion: j/l (±x), i/k (±y), q/a (±z), ;/' (panda_joint1 ±); "
                "Angular: r/f (roll), t/g (pitch), y/h (yaw)"
            )

        message_parts = [f"Unknown key '{key}'"]
        if suggestions:
            message_parts.append(f"Hint: {suggestions[0]}")
        message_parts.append("Press '?' for full key reference")

        self.get_logger().info("\\n".join(message_parts))

    def _validate_parameters(self):
        """Validate all parameters and raise ValueError if any are invalid."""

        # Validate rate_hz
        if self.rate_hz <= 0:
            raise ValueError(f"rate_hz must be positive, got {self.rate_hz}")
        if self.rate_hz > 1000:
            raise ValueError(f"rate_hz too high (max 1000), got {self.rate_hz}")

        # Validate linear_step against safety limits
        if self.linear_step <= 0:
            raise ValueError(f"linear_step must be positive, got {self.linear_step}")
        if self.linear_step > self.MAX_LINEAR_VELOCITY:
            raise ValueError(
                f"linear_step exceeds safety limit "
                f"(max {self.MAX_LINEAR_VELOCITY}, unitless), got {self.linear_step}"
            )
        if self.linear_step > self.EMERGENCY_LINEAR_LIMIT:
            raise ValueError(
                f"linear_step exceeds emergency limit "
                f"(max {self.EMERGENCY_LINEAR_LIMIT}, unitless), got {self.linear_step}"
            )

        # Validate angular_step against safety limits
        if self.angular_step <= 0:
            raise ValueError(f"angular_step must be positive, got {self.angular_step}")
        if self.angular_step > self.MAX_ANGULAR_VELOCITY:
            raise ValueError(
                f"angular_step exceeds safety limit "
                f"(max {self.MAX_ANGULAR_VELOCITY:.2f} rad/s), got {self.angular_step}"
            )
        if self.angular_step > self.EMERGENCY_ANGULAR_LIMIT:
            raise ValueError(
                f"angular_step exceeds emergency limit "
                f"(max {self.EMERGENCY_ANGULAR_LIMIT:.2f} rad/s), got {self.angular_step}"
            )

        # Validate deadman_ttl_sec
        if self.deadman_ttl_sec <= 0:
            raise ValueError(f"deadman_ttl_sec must be positive, got {self.deadman_ttl_sec}")
        if self.deadman_ttl_sec > 10.0:
            raise ValueError(f"deadman_ttl_sec too large (max 10s), got {self.deadman_ttl_sec}")

        # Validate string parameters
        if not self.frame_id.strip():
            raise ValueError("frame_id cannot be empty")

        # Validate tasks list
        if not self.tasks:
            raise ValueError("tasks list cannot be empty")
        if len(self.tasks) > 9:
            raise ValueError(f"tasks list too long (max 9), got {len(self.tasks)}")

        # Check for duplicate tasks
        if len(set(self.tasks)) != len(self.tasks):
            duplicates = [task for task in self.tasks if self.tasks.count(task) > 1]
            raise ValueError(f"Duplicate tasks found: {list(set(duplicates))}")

        # Check for empty task names
        empty_tasks = [i for i, task in enumerate(self.tasks) if not task.strip()]
        if empty_tasks:
            raise ValueError(f"Empty task names at indices: {empty_tasks}")

        self.get_logger().info("Parameters validated successfully:")
        self.get_logger().info(f"  rate_hz: {self.rate_hz} Hz")
        self.get_logger().info(f"  linear_step: {self.linear_step} (unitless servo input)")
        self.get_logger().info(f"  angular_step: {self.angular_step} rad/s")
        self.get_logger().info(f"  deadman_ttl: {self.deadman_ttl_sec} s")
        self.get_logger().info(f"  tasks: {len(self.tasks)} configured")

    def _setup_servo_auto_start(self):
        """Setup automatic servo startup using servo_utils."""
        # Create a one-shot timer to start servo after initialization
        self._servo_auto_start_timer = self.create_timer(2.0, self._auto_start_servo_callback)

    def _auto_start_servo_callback(self):
        """Timer callback to automatically start servo using servo_utils."""
        # Cancel the timer (one-shot operation)
        self._servo_auto_start_timer.cancel()

        # Use servo_utils to auto-start servo
        success = self.servo_utils.auto_start()

        if success:
            self.get_logger().info("[teleop] Servo auto-start completed successfully!")
        else:
            self.get_logger().warning(
                "[teleop] ⚠️ Servo auto-start failed. You may need to manually start servo node."
            )


def main():
    if not sys.stdin.isatty():
        print(
            "teleop_keyboard requires an interactive TTY stdin. "
            "Please run it from a terminal.",
            file=sys.stderr,
        )
        return

    rclpy.init()

    with TerminalRawMode():
        node = TeleopKeyboard()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
