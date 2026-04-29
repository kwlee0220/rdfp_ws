#!/usr/bin/env python3
"""Keyboard Cartesian teleop with gripper, episode, and task management.

Extends the base keyboard teleop (panda_servo_teleop) with:
- Gripper open/close (g/h)
- Episode start/end ([/])
- Task selection (1-5) and outcome marking (=/-).
"""
import sys
import select
import termios
import tty
from dataclasses import dataclass
from typing import List  # noqa: UP035

from rdfp.teleop.session_teleop import _DEFAULT_TASKS
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_srvs.srv import Trigger

from ..moveit.move_group_client import MoveGroupClient
from ..moveit.servo_client import ServoClient
from ..ros2_utils import get_parameter, parse_float, parse_str, parse_str_list
from ..session.session_control_client import SessionControlClient


class TerminalRawMode:
    """Context manager for safely handling terminal raw mode.

    Automatically restores terminal settings when exiting the context,
    even if exceptions occur.
    """

    def __enter__(self):
        self._stdin_fd = sys.stdin.fileno()
        self._old_term = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Restore terminal settings regardless of how we exit."""
        try:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_term)
        except Exception:
            # Fail silently if terminal restoration fails
            pass


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
    gripper_open: str = "="
    gripper_close: str = "-"

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
    =: open   -: close
  Session (split services on session_control):
    < or ,: start_session   > or .: stop_session
    [: start_episode   ]: stop_episode
  Task (set_task_label service):
    1-N: select task from 'tasks' parameter (see log below)
    0: clear task
  Control:
    SPACE: deadman   x: stop   /: move to 'ready' pose   Ctrl-C: quit
  Note: Servo auto-starts on initialization
─────────────────────────────────────"""

_DEFAULT_TASK_LIST =  ["touch", "pick_and_place", "push", "stack", "wipe"]
_SERVO_NODE_NAME = "/servo_node"  # Default name for the servo node to control
_DELTA_TWIST_TOPIC = "/servo_node/delta_twist_cmds"  # Default topic for TwistStamped commands
_DELTA_JOINT_TOPIC = "/servo_node/delta_joint_cmds"  # JointJog topic for joint-level servo
_JOINT1_NAME = "panda_joint1"  # 조인트 단위 서보 대상 (좌/우 화살표 매핑)

# gripper_control_node 가 제공하는 Trigger 서비스. 기본 node 이름은
# 'gripper_control' 이므로 ``~/open_gripper`` 는 /gripper_control/open_gripper 로
# resolve 된다.
_GRIPPER_OPEN_SERVICE = "/gripper_control/open_gripper"
_GRIPPER_CLOSE_SERVICE = "/gripper_control/close_gripper"

_DEFAULT_RATE_HZ = 100.0
_DEFAULT_DEADMAN_TTL_SEC = 0.1


class TeleopKeyboard(Node):
    """Publishes TwistStamped, gripper commands, and dataset/task events."""

    # Safety velocity limits (absolute maximum values)
    MAX_LINEAR_VELOCITY = 0.5   # m/s - maximum safe linear velocity
    MAX_ANGULAR_VELOCITY = 1.57  # rad/s - maximum safe angular velocity (π/2)

    # Emergency limits (even more conservative)
    EMERGENCY_LINEAR_LIMIT = 1.0   # m/s - absolute emergency stop limit
    EMERGENCY_ANGULAR_LIMIT = 3.14  # rad/s - absolute emergency stop limit (π)

    def __init__(self):
        super().__init__("teleop_keyboard")

        # --- Parameters ---
        self.declare_parameter("frame_id", "panda_link0")
        self.declare_parameter("rate_hz", _DEFAULT_RATE_HZ)
        self.declare_parameter("linear_step", 0.5)
        self.declare_parameter("angular_step", 0.50)
        self.declare_parameter("deadman_ttl_sec", _DEFAULT_DEADMAN_TTL_SEC)
        self.declare_parameter("tasks", _DEFAULT_TASK_LIST)

        self.frame_id = get_parameter(self, "frame_id", parse_str, default="panda_link0")
        self.rate_hz = get_parameter(self, "rate_hz", parse_float, default=_DEFAULT_RATE_HZ)
        self.linear_step = get_parameter(self, "linear_step", parse_float, default=0.5)
        self.angular_step = get_parameter(self, "angular_step", parse_float, default=0.50)
        self.deadman_ttl_sec = get_parameter(self, "deadman_ttl_sec", parse_float,
                                             default=_DEFAULT_DEADMAN_TTL_SEC)
        self.tasks = get_parameter(self, "tasks",  parse_str_list, default=_DEFAULT_TASK_LIST)

        # --- Publishers ---
        self.twist_pub = self.create_publisher(TwistStamped, _DELTA_TWIST_TOPIC, 10)
        # panda_joint1 단일 조인트 서보용 JointJog 퍼블리셔
        self.joint_jog_pub = self.create_publisher(JointJog, _DELTA_JOINT_TOPIC, 10)

        # --- Gripper service clients ---
        # 실제 액션 호출/토픽 발행은 gripper_control_node 가 담당한다. 본 노드는
        # 키 입력을 Trigger 서비스 호출로 변환만 한다.
        self._gripper_open_cli = self.create_client(Trigger, _GRIPPER_OPEN_SERVICE)
        self._gripper_close_cli = self.create_client(Trigger, _GRIPPER_CLOSE_SERVICE)

        # --- SessionControlClient (비동기 API 사용) ---
        self._session_client = SessionControlClient.create(self)

        # --- Servo utils ---
        self.servo_utils = ServoClient.create(self, _SERVO_NODE_NAME)

        # --- MoveGroup 클라이언트 (Home 키 → 'ready' named target 이동) ---
        self._move_group = MoveGroupClient(self)
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

        # --- Twist state ---
        self.twist = TwistStamped()
        self.twist.header.frame_id = self.frame_id

        # --- JointJog state (panda_joint1 단독) ---
        self.joint_jog = JointJog()
        self.joint_jog.header.frame_id = self.frame_id
        self.joint_jog.joint_names = [_JOINT1_NAME]
        self.joint_jog.velocities = [0.0]

        self.get_logger().info(HELP_TEXT)
        if self.tasks:
            task_list = "  ".join(f"{i+1}:{name}" for i, name in enumerate(self.tasks[:9]))
            self.get_logger().info(f"Tasks: {task_list}")

        # --- Timer ---
        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.01
        self.timer = self.create_timer(period, self._on_timer)

        # --- Deadman state ---
        self._deadman_ttl = 0.0
        self._last_key = None

        # Validate all parameters
        self._validate_parameters()

        # Auto-start servo after initialization
        self._setup_servo_auto_start()


    def destroy_node(self):
        super().destroy_node()

    # ── Keyboard input ──────────────────────────────────────────

    def _read_key_nonblocking(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not dr:
            return None
        ch = sys.stdin.read(1)
        # ESC 수신 시 화살표 등 escape sequence (\x1b[X) 일 가능성이 있으므로
        # 뒤따르는 바이트를 추가로 읽어 합쳐 반환한다. 타임아웃이 너무 짧으면
        # OS 스케줄링/터미널 버퍼 타이밍으로 인해 3 바이트가 분리 수신되어
        # '[' 가 episode_start 키로, 'D' 가 unknown key 로 오인식될 수 있다.
        # 50ms 로 두면 분리 수신 확률이 크게 낮아진다.
        if ch == "\x1b":
            dr2, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not dr2:
                return ch
            ch2 = sys.stdin.read(1)
            if ch2 != "[":
                return ch + ch2
            dr3, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not dr3:
                return ch + ch2
            ch3 = sys.stdin.read(1)
            return ch + ch2 + ch3
        return ch

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
        """gripper_control_node 의 Trigger 서비스를 비동기로 호출한다.

        실제 토픽 발행과 액션 호출은 gripper_control_node 내부에서 수행된다.
        본 노드는 키 입력을 서비스 호출로만 변환한다. 서비스가 아직 뜨지
        않았다면 경고만 찍고 스킵한다 (매 키 누름마다 블로킹 대기하지 않음).
        """
        client = (
            self._gripper_open_cli if command == "open" else self._gripper_close_cli
        )
        if not client.service_is_ready():
            self.get_logger().warning(
                f"[gripper] {command} service not ready; is gripper_control_node running?"
            )
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda fut: self._on_gripper_service_done(command, fut)
        )

    def _on_gripper_service_done(self, command: str, future) -> None:
        """gripper Trigger 서비스 호출 결과 처리 (비동기 콜백)."""
        try:
            result = future.result()
        except Exception as e:   # noqa: BLE001
            self.get_logger().warning(f"[gripper] {command} service call failed: {e}")
            return
        if result is None:
            self.get_logger().warning(f"[gripper] {command} service returned no result")
            return
        if result.success:
            self.get_logger().info(f"[gripper] {command}: {result.message or 'ok'}")
        else:
            self.get_logger().warning(
                f"[gripper] {command} rejected: {result.message or 'unknown'}"
            )

    # ── Home helpers (MoveIt named target 'ready' 이동) ─────────────

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
        try:
            future = self._move_group.move_to_named_target_async("ready")
        except Exception as e:   # noqa: BLE001
            self._home_in_progress = False
            self.get_logger().error(f"[home] failed to start: {e}")
            return
        future.add_done_callback(self._on_home_done)
        self.get_logger().info("[home] moving to 'ready' pose...")

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
            self._home_in_progress = False

    # ── Session 서비스 완료 콜백 ──────────────────────────────────

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

        # Gripper — gripper_control_node 의 Trigger 서비스 위임 호출.
        if key == km.gripper_open:
            self._call_gripper("open")
            return True
        if key == km.gripper_close:
            self._call_gripper("close")
            return True

        # Home — MoveIt 'ready' named target 으로 이동
        if key == km.home:
            self._call_home()
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
            # Update deadman timer
            dt = self.timer.timer_period_ns * 1e-9
            self._deadman_ttl = max(0.0, self._deadman_ttl - dt)

            # Read keyboard input
            key = self._read_key_nonblocking()
            if key is not None:
                try:
                    # One-shot keys (gripper, episode, task) are handled immediately
                    if self._handle_oneshot_key(key):
                        return

                    # Check if this is a valid motion or control key
                    valid_keys = self._get_all_valid_keys()
                    if key not in valid_keys:
                        self._handle_unknown_key(key)
                        return  # Don't process unknown keys further

                    self._last_key = key

                    # Refresh deadman on SPACE or motion key
                    if key == self.keys.deadman or key in self.motion_keys:
                        self._deadman_ttl = self.deadman_ttl_sec

                    # Stop key: publish zero once (twist + joint_jog 모두)
                    if key == self.keys.stop:
                        self._zero_twist()
                        self._zero_joint_jog()
                        self._publish_twist_safely()
                        self._publish_joint_jog_safely()
                        return

                except Exception as e:
                    self.get_logger().warning(f"Failed to process key '{key}': {e}")
                    # Continue execution - don't let key processing errors stop the timer

            if self._deadman_ttl <= 0.0:
                return

            # Home 이동 중에는 servo 명령 발행을 중단하여 move_group trajectory 와
            # 충돌하지 않게 한다.
            if self._home_in_progress:
                return

            # Deadman active: generate twist / joint_jog from last motion key
            try:
                cmd_key = self._last_key
                if cmd_key is None or cmd_key == self.keys.deadman:
                    self._zero_twist()
                    self._zero_joint_jog()
                else:
                    self._apply_key_to_motion(cmd_key)

                self._publish_twist_safely()
                self._publish_joint_jog_safely()

            except Exception as e:
                self.get_logger().warning(f"Failed to generate/publish motion: {e}")
                # Publish zero twist + joint_jog as safety fallback
                try:
                    self._zero_twist()
                    self._zero_joint_jog()
                    self._publish_twist_safely()
                    self._publish_joint_jog_safely()
                except Exception as fallback_e:
                    self.get_logger().error(f"Critical: Even safety fallback failed: {fallback_e}")

        except Exception as e:
            # Catch-all for any other unexpected errors
            self.get_logger().error(f"Critical timer callback error: {e}")
            # Don't re-raise - let the timer continue running

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
        if (abs(linear.x) > self.MAX_LINEAR_VELOCITY or
            abs(linear.y) > self.MAX_LINEAR_VELOCITY or
            abs(linear.z) > self.MAX_LINEAR_VELOCITY):
            self.get_logger().warning(
                f"High linear velocity detected: [{linear.x:.3f}, {linear.y:.3f}, {linear.z:.3f}] m/s"
            )

        if (abs(angular.x) > self.MAX_ANGULAR_VELOCITY or
            abs(angular.y) > self.MAX_ANGULAR_VELOCITY or
            abs(angular.z) > self.MAX_ANGULAR_VELOCITY):
            self.get_logger().warning(
                f"High angular velocity detected: [{angular.x:.3f}, {angular.y:.3f}, {angular.z:.3f}] rad/s"
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
            km.gripper_open, km.gripper_close,
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
        elif key in [km.gripper_open, km.gripper_close]:
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
  Control: SPACE (deadman), x (stop)
  Gripper: = (open), - (close)
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
                f"linear_step exceeds safety limit (max {self.MAX_LINEAR_VELOCITY} m/s), got {self.linear_step}"
            )
        if self.linear_step > self.EMERGENCY_LINEAR_LIMIT:
            raise ValueError(
                f"linear_step exceeds emergency limit (max {self.EMERGENCY_LINEAR_LIMIT} m/s), got {self.linear_step}"
            )

        # Validate angular_step against safety limits
        if self.angular_step <= 0:
            raise ValueError(f"angular_step must be positive, got {self.angular_step}")
        if self.angular_step > self.MAX_ANGULAR_VELOCITY:
            raise ValueError(
                f"angular_step exceeds safety limit (max {self.MAX_ANGULAR_VELOCITY:.2f} rad/s), got {self.angular_step}"
            )
        if self.angular_step > self.EMERGENCY_ANGULAR_LIMIT:
            raise ValueError(
                f"angular_step exceeds emergency limit (max {self.EMERGENCY_ANGULAR_LIMIT:.2f} rad/s), got {self.angular_step}"
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
        self.get_logger().info(f"  linear_step: {self.linear_step} m/s")
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
