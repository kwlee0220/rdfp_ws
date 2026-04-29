#!/usr/bin/env python3

"""SessionTeleop 모듈.

키보드 입력으로 세션/에피소드 라이프사이클과 태스크 라벨을 제어하는 ROS2 노드.
teleop_keyboard 에서 세션·태스크 관련 기능만 분리한 경량 버전이다.
세션 제어는 SessionControlClient 의 비동기 API 를 활용한다.

키 바인딩:
  {/} : start_session / stop_session
  [/] : start_episode / stop_episode
  1-9 : 태스크 선택 (tasks 파라미터 기반)
  0   : 태스크 클리어
  x   : 종료
"""

from __future__ import annotations

from typing import Optional

import sys
import select
import termios
import tty
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from ..ros2_utils import get_parameter, parse_int, parse_str_list
from ..session.session_control_client import SessionControlClient


_DEFAULT_TASKS = ["touch", "pick_and_place", "push", "stack", "wipe"]


class TerminalRawMode:
    """터미널 raw 모드 컨텍스트 매니저.

    컨텍스트 종료 시(예외 포함) 터미널 설정을 자동 복원한다.
    """

    def __enter__(self):
        self._stdin_fd = sys.stdin.fileno()
        self._old_term = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """터미널 설정을 복원한다."""
        try:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_term)
        except Exception:
            pass


@dataclass
class SessionKeyMapping:
    """세션/태스크 제어 키 매핑."""

    # 세션 라이프사이클
    session_start: str = "{"
    session_stop: str = "}"

    # 에피소드 라이프사이클
    episode_start: str = "["
    episode_end: str = "]"

    # 태스크 클리어
    task_clear: str = "0"

    # 종료
    stop: str = "x"


HELP_TEXT = """
Session Teleop
-------------------------------------
  Session:
    {: start_session   }: stop_session
    [: start_episode   ]: stop_episode
  Task (set_task_label service):
    1-N: select task from 'tasks' parameter
    0: clear task
  Control:
    x: quit
-------------------------------------"""


class SessionTeleop(Node):
    """키보드로 세션/에피소드/태스크를 제어하는 텔레옵 노드."""

    def __init__(self):
        super().__init__("session_teleop")

        # --- 파라미터 ---
        self.declare_parameter("rate_hz", 30)
        self.declare_parameter("tasks", _DEFAULT_TASKS)

        self.rate_hz = get_parameter(self, "rate_hz", parse_int, default=30)
        self.tasks = get_parameter(self, "tasks",  parse_str_list, default=_DEFAULT_TASKS)

        # --- SessionControlClient (비동기 API 사용) ---
        self._session_client = SessionControlClient.create(self)

        # --- 키 매핑 ---
        self.keys = SessionKeyMapping()
        self.task_keys = {str(i + 1): name for i, name in enumerate(self.tasks[:9])}

        # --- 도움말 출력 ---
        self.get_logger().info(HELP_TEXT)
        if self.tasks:
            task_list = "  ".join(
                f"{i + 1}:{name}" for i, name in enumerate(self.tasks[:9])
            )
            self.get_logger().info(f"Tasks: {task_list}")

        # --- 타이머 ---
        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.05
        self.timer = self.create_timer(period, self._on_timer)

        # --- 종료 플래그 ---
        self._quit = False

    # -- 키보드 입력 ----------------------------------------------------------

    def _read_key_nonblocking(self) -> Optional[str]:
        """논블로킹 키 입력을 읽는다."""
        dr, _, _ = select.select([sys.stdin], [], [], 0.0)
        if dr:
            return sys.stdin.read(1)
        return None

    # -- 서비스 완료 콜백 -----------------------------------------------------

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

    # -- 로그 메시지 빌더 -----------------------------------------------------

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

    # -- 키 핸들링 ------------------------------------------------------------

    def _handle_key(self, key: str) -> bool:
        """키 입력을 처리한다. 처리된 경우 True를 반환한다."""
        km = self.keys

        # 종료
        if key == km.stop:
            self._quit = True
            return True

        # 세션 라이프사이클
        if key == km.session_start:
            self._session_client.start_session_async(
                done_callback=lambda ok, msg: self._on_trigger_done("start_session", ok, msg),
            )
            return True
        if key == km.session_stop:
            self._session_client.stop_session_async(
                done_callback=lambda ok, msg: self._on_trigger_done("stop_session", ok, msg),
            )
            return True

        # 에피소드 라이프사이클
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

        # 태스크 클리어
        if key == km.task_clear:
            self._session_client.set_task_label_async(
                task_label=None,
                done_callback=lambda ok, msg: self._on_set_task_done("", ok, msg),
            )
            return True

        # 태스크 선택 (1-N)
        if key in self.task_keys:
            label = self.task_keys[key]
            self._session_client.set_task_label_async(
                task_label=label,
                done_callback=lambda ok, msg, t=label: self._on_set_task_done(t, ok, msg),
            )
            return True

        return False

    # -- 타이머 콜백 ----------------------------------------------------------

    def _on_timer(self) -> None:
        """타이머 콜백: 키 입력을 폴링하여 처리한다."""
        key = self._read_key_nonblocking()
        if key is None:
            return

        if self._handle_key(key):
            return

        # 인식되지 않는 키
        if key.isprintable() and key not in ('\n', '\r', '\t'):
            self.get_logger().info(
                f"Unknown key '{key}'. "
                f"Press '{{}}/{{}}' for session, '[/]' for episode, "
                f"1-{min(len(self.tasks), 9)} for task, 0 to clear."
            )

    @property
    def quit_requested(self) -> bool:
        """종료가 요청되었는지 반환한다."""
        return self._quit


def main() -> None:
    """콘솔 엔트리 포인트."""
    if not sys.stdin.isatty():
        print(
            "session_teleop requires an interactive TTY stdin. "
            "Please run it from a terminal.",
            file=sys.stderr,
        )
        return

    rclpy.init()

    node: Optional[SessionTeleop] = None
    try:
        with TerminalRawMode():
            node = SessionTeleop()
            try:
                while rclpy.ok() and not node.quit_requested:
                    rclpy.spin_once(node, timeout_sec=0.05)
            except KeyboardInterrupt:
                pass
    except RuntimeError as exc:
        print(f"[FATAL] SessionTeleop init failed: {exc}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
