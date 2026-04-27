#!/usr/bin/env python3
"""session_control_node 의 6 개 서비스를 래핑하는 Python 클라이언트.

SessionControlClient 는 호출자의 ROS2 노드를 주입받아 그 위에 6 개의 서비스
클라이언트를 생성하고, 각 서비스에 대한 **동기/비동기** 호출 메서드를 제공한다.
생성 시점에 6 개 서비스가 모두 준비될 때까지 블로킹 대기한다.

사용 예:

    client = SessionControlClient.create(node, wait_timeout_sec=10.0)

    # 동기 호출 (노드가 아직 executor 에서 spin 중이 아닐 때만 사용 가능)
    ok, msg = client.start_session()

    # 비동기 호출 (타이머/콜백 내부에서 블로킹 없이 사용)
    client.start_session_async(done_callback=on_done)

    # task clear
    client.set_task_label(None)
"""

from __future__ import annotations

from typing import Callable

import rclpy
from rclpy.client import Client
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import Trigger

from rdfp_msgs.srv import GetSessionState, SetString


# 기본 서비스 네임스페이스. session_control_node 의 기본 노드 이름과 일치.
_DEFAULT_NAMESPACE: str = "session_control"

# 초기화 시 서비스 ready 대기 기본 타임아웃 (초).
_DEFAULT_WAIT_TIMEOUT: float = 10.0

# 개별 서비스 호출 기본 타임아웃 (초).
_DEFAULT_CALL_TIMEOUT: float = 5.0

# 동기 호출에서 공통으로 쓰는 실패 메시지 상수.
_NOT_READY_MSG: str = "service not ready"
_CALL_TIMEOUT_MSG: str = "service call timed out"
_NO_RESPONSE_MSG: str = "no response"


class SessionControlClient:
    """session_control_node 의 6 개 서비스를 래핑하는 Python 클라이언트.

    호출자의 `rclpy.node.Node` 를 주입받아 그 위에 서비스 클라이언트를 생성
    한다. 생성 시점에 6 개 서비스 모두 준비될 때까지 블로킹 대기하며, 타임
    아웃 안에 준비되지 않으면 `RuntimeError` 를 발생시킨다.

    동기 메서드(`start_session` 등)는 내부에서
    `rclpy.spin_until_future_complete` 를 호출하므로, **호출자 노드가 이미
    다른 스레드의 executor 에서 spin 중이거나 서비스 콜백 내부에서 호출할
    경우 데드락이 발생할 수 있다**. 그 경우에는 `*_async` 변형을 사용해야
    한다.
    """

    def __init__(
        self,
        node: Node,
        namespace: str = _DEFAULT_NAMESPACE,
        wait_timeout_sec: float = _DEFAULT_WAIT_TIMEOUT,
    ) -> None:
        """SessionControlClient 를 초기화한다.

        Args:
            node: 서비스 클라이언트를 붙일 호출자의 ROS2 노드.
            namespace: session_control_node 의 노드 이름(서비스 prefix).
                기본값은 'session_control'. 절대 경로(`/ns/session_control`)도
                허용된다.
            wait_timeout_sec: 6 개 서비스가 모두 준비될 때까지 대기할 총
                타임아웃. 0 이하이면 대기하지 않는다.

        Raises:
            RuntimeError: 타임아웃 안에 하나 이상의 서비스가 준비되지 않은 경우.
        """
        self._node = node
        self._logger = node.get_logger()

        # 서비스 이름 구성: namespace 앞에 '/' 가 없으면 상대 경로로 취급되어
        # 호출자 노드의 네임스페이스에 붙는다.
        prefix = namespace.rstrip("/")

        self._start_session_cli = node.create_client(
            Trigger, f"{prefix}/start_session"
        )
        self._stop_session_cli = node.create_client(
            Trigger, f"{prefix}/stop_session"
        )
        self._start_episode_cli = node.create_client(
            Trigger, f"{prefix}/start_episode"
        )
        self._stop_episode_cli = node.create_client(
            Trigger, f"{prefix}/stop_episode"
        )
        self._set_task_label_cli = node.create_client(
            SetString, f"{prefix}/set_task_label"
        )
        self._get_session_state_cli = node.create_client(
            GetSessionState, f"{prefix}/get_session_state"
        )

        # 초기화 시점에 6 개 서비스 모두 ready 가 될 때까지 블로킹 대기한다.
        if wait_timeout_sec > 0.0:
            self._wait_for_services_ready(wait_timeout_sec)

    @classmethod
    def create(cls, node: Node, namespace: str = _DEFAULT_NAMESPACE,
               wait_timeout_sec: float = _DEFAULT_WAIT_TIMEOUT,) -> SessionControlClient:
        """SessionControlClient 인스턴스를 생성한다 (팩토리 메서드).

        생성자와 동일한 시그니처이며, 프로젝트 내 다른 *Client 클래스와의
        스타일 통일을 위해 제공된다.

        Args:
            node: 서비스 클라이언트를 붙일 호출자의 ROS2 노드.
            namespace: session_control_node 의 노드 이름(서비스 prefix).
                기본값은 'session_control'. 절대 경로(`/ns/session_control`)도
                허용된다.
            wait_timeout_sec: 6 개 서비스가 모두 준비될 때까지 대기할 총
                타임아웃. 0 이하이면 대기하지 않는다.

        Returns:
            SessionControlClient 인스턴스.
        """
        return cls(node, namespace=namespace, wait_timeout_sec=wait_timeout_sec)

    # ------------------------------------------------------------------
    # 동기 API
    # ------------------------------------------------------------------

    def start_session(self, timeout_sec: float = _DEFAULT_CALL_TIMEOUT) -> tuple[bool, str]:
        """start_session 서비스를 동기로 호출한다.

        Args:
            timeout_sec: 서비스 호출 타임아웃(초). 0 이하이면 타임아웃 없이 무한 대기한다.
        Returns:
            (success, message) 튜플. 서버가 거부하면 success=False, message 에
            서버가 반환한 사유가 담긴다. 서비스가 ready 가 아니거나 호출이
            타임아웃되면 success=False 와 내부 사유 메시지가 반환된다.
        """
        return self._call_trigger_sync(self._start_session_cli, timeout_sec)

    def stop_session(self, timeout_sec: float = _DEFAULT_CALL_TIMEOUT) -> tuple[bool, str]:
        """stop_session 서비스를 동기로 호출한다.

        `IN_EPISODE` 상태에서 호출되면 서버가 원자적으로 에피소드 종료 후
        세션 종료를 수행한다 (단일 호출로 2 단계 전이).
        Args:
            timeout_sec: 서비스 호출 타임아웃(초). 0 이하이면 타임아웃 없이 무한 대기한다.
        Returns:
            (success, message) 튜플. 서버가 거부하면 success=False, message 에
            서버가 반환한 사유가 담긴다. 서비스가 ready 가 아니거나 호출이
            타임아웃되면 success=False 와 내부 사유 메시지가 반환된다
        """
        return self._call_trigger_sync(self._stop_session_cli, timeout_sec)

    def start_episode(self, timeout_sec: float = _DEFAULT_CALL_TIMEOUT) -> tuple[bool, str]:
        """start_episode 서비스를 동기로 호출한다.

        Args:
            timeout_sec: 서비스 호출 타임아웃(초). 0 이하이면 타임아웃 없이 무한 대기한다.
        Returns:
            (success, message) 튜플. 서버가 거부하면 success=False, message 에
            서버가 반환한 사유가 담긴다. 서비스가 ready 가 아니거나 호출이
            타임아웃되면 success=False 와 내부 사유 메시지가 반환된다.
        """
        return self._call_trigger_sync(self._start_episode_cli, timeout_sec)

    def stop_episode(self, timeout_sec: float = _DEFAULT_CALL_TIMEOUT) -> tuple[bool, str]:
        """stop_episode 서비스를 동기로 호출한다.

        Args:
            timeout_sec: 서비스 호출 타임아웃(초). 0 이하이면 타임아웃 없이 무한 대기한다.
        Returns:
            (success, message) 튜플. 서버가 거부하면 success=False, message 에
            서버가 반환한 사유가 담긴다. 서비스가 ready 가 아니거나 호출이
            타임아웃되면 success=False 와 내부 사유 메시지가 반환된다.
        """
        return self._call_trigger_sync(self._stop_episode_cli, timeout_sec)

    def set_task_label(self, task_label: str | None,
                       timeout_sec: float = _DEFAULT_CALL_TIMEOUT,) -> tuple[bool, str]:
        """set_task_label 서비스를 동기로 호출한다.

        Args:
            task_label: 설정할 task label. `None` 을 전달하면 내부적으로
                빈 문자열로 변환되어 task clear 동작을 수행한다.
        Returns:
            (success, message) 튜플. 서버가 거부하면 success=False, message 에
            서버가 반환한 사유가 담긴다. 서비스가 ready 가 아니거나 호출이
            타임아웃되면 success=False 와 내부 사유 메시지가 반환된다
        """
        client = self._set_task_label_cli
        if not client.service_is_ready():
            return False, _NOT_READY_MSG

        request = SetString.Request()
        request.task_label = "" if task_label is None else task_label

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)

        if not future.done():
            return False, _CALL_TIMEOUT_MSG

        try:
            response = future.result()
        except Exception as exc:
            self._logger.error(f'[session_control_client] set_task_label raised: {exc}')
            return False, f'exception: {exc}'
        if response is None:
            return False, _NO_RESPONSE_MSG
        return bool(response.success), str(response.message)

    def get_session_state(self, timeout_sec: float = _DEFAULT_CALL_TIMEOUT) -> tuple[str, str]:
        """get_session_state 서비스를 동기로 호출한다.

        Args:
            timeout_sec: 서비스 호출 타임아웃(초). 0 이하이면 타임아웃 없이 무한 대기한다.
        Returns:
            (state, task_label) 튜플. 서비스가 ready 가 아니거나 호출이
            타임아웃되면 `('', '')` 를 반환하고 warning 로그를 남긴다.
            `''` 와 "실제 IDLE/빈 라벨" 을 엄밀하게 구분해야 한다면
            `get_session_state_async` 를 써서 직접 future 를 다루거나,
            이 메서드 호출 전후로 상태를 명시적으로 추적하라.
        """
        client = self._get_session_state_cli
        if not client.service_is_ready():
            self._logger.warning(
                "[session_control_client] get_session_state: service not ready"
            )
            return "", ""

        future = client.call_async(GetSessionState.Request())
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=timeout_sec
        )

        if not future.done():
            self._logger.warning(
                "[session_control_client] get_session_state: call timed out"
            )
            return "", ""

        try:
            response = future.result()
        except Exception as exc:
            self._logger.error(
                f'[session_control_client] get_session_state raised: {exc}'
            )
            return "", ""
        if response is None:
            return "", ""
        return str(response.state), str(response.task_label)

    # ------------------------------------------------------------------
    # 비동기 API
    # ------------------------------------------------------------------

    def start_session_async(self, done_callback: Callable[[bool, str], None] | None = None,) -> Future:
        """start_session 서비스를 비동기로 호출한다.

        Args:
            done_callback: 응답 수신 시 호출할 콜백. 인자는 `(success, message)`.
                서비스 호출이 예외로 실패하면 `(False, "exception: ...")` 로
                호출되고, 응답이 없으면 `(False, "no response")` 로 호출된다.
                None 이면 콜백을 등록하지 않고 raw future 만 반환된다.

        Returns:
            `rclpy.task.Future`. raw future 에 직접 `add_done_callback` 을
            추가하거나 `result()` 로 원본 `Trigger.Response` 에 접근할 수 있다.
            서비스가 ready 가 아니어도 호출은 성공하며, 대신 future 가
            오래 대기하다가 호출자의 executor 가 멈출 때 정리된다. 먼저
            `is_ready()` 로 확인하고 호출할지 여부는 호출자가 결정한다.
        """
        return self._call_trigger_async(self._start_session_cli, done_callback)

    def stop_session_async(self, done_callback: Callable[[bool, str], None] | None = None,) -> Future:
        """stop_session 서비스를 비동기로 호출한다."""
        return self._call_trigger_async(self._stop_session_cli, done_callback)

    def start_episode_async(self, done_callback: Callable[[bool, str], None] | None = None,) -> Future:
        """start_episode 서비스를 비동기로 호출한다."""
        return self._call_trigger_async(self._start_episode_cli, done_callback)

    def stop_episode_async(self, done_callback: Callable[[bool, str], None] | None = None,) -> Future:
        """stop_episode 서비스를 비동기로 호출한다."""
        return self._call_trigger_async(self._stop_episode_cli, done_callback)

    def set_task_label_async(self, task_label: str | None,
                             done_callback: Callable[[bool, str], None] | None = None,) -> Future:
        """set_task_label 서비스를 비동기로 호출한다.

        Args:
            task_label: 설정할 task label. `None` 을 전달하면 내부적으로
                빈 문자열로 변환되어 task clear 동작을 수행한다.
            done_callback: 응답 수신 시 호출할 콜백. 인자는 `(success, message)`.
        """
        request = SetString.Request()
        request.task_label = "" if task_label is None else task_label

        future = self._set_task_label_cli.call_async(request)
        if done_callback is not None:
            future.add_done_callback(
                lambda fut, cb=done_callback:
                    self._invoke_bool_message_callback(cb, fut)
            )
        return future

    def get_session_state_async(self, done_callback: Callable[[str, str], None] | None = None,) -> Future:
        """get_session_state 서비스를 비동기로 호출한다.

        Args:
            done_callback: 응답 수신 시 호출할 콜백. 인자는 `(state, task_label)`.
                호출이 예외로 실패하거나 응답이 없으면 `('', '')` 로 호출된다.
        """
        future = self._get_session_state_cli.call_async(
            GetSessionState.Request()
        )
        if done_callback is not None:
            future.add_done_callback(
                lambda fut, cb=done_callback:
                    self._invoke_state_callback(cb, fut)
            )
        return future

    # ------------------------------------------------------------------
    # 서비스 ready / 유틸리티
    # ------------------------------------------------------------------

    def is_ready(self) -> bool:
        """6 개 서비스가 모두 ready 상태인지 확인한다.

        Returns:
            모든 서비스가 ready 면 True, 하나라도 아니면 False.
        """
        return all(
            client.service_is_ready()
            for client in (
                self._start_session_cli,
                self._stop_session_cli,
                self._start_episode_cli,
                self._stop_episode_cli,
                self._set_task_label_cli,
                self._get_session_state_cli,
            )
        )

    # ------------------------------------------------------------------
    # 내부 헬퍼
    # ------------------------------------------------------------------

    def _wait_for_services_ready(self, total_timeout_sec: float) -> None:
        """6 개 서비스가 모두 ready 가 될 때까지 블로킹 대기한다.

        각 서비스에 대해 **남은 시간 예산** 을 부여하며, 하나라도 타임아웃
        안에 준비되지 않으면 `RuntimeError` 를 발생시킨다.
        """
        deadline_ns = self._node.get_clock().now().nanoseconds + int(
            total_timeout_sec * 1e9
        )
        clients: list[tuple[str, Client]] = [
            ("start_session", self._start_session_cli),
            ("stop_session", self._stop_session_cli),
            ("start_episode", self._start_episode_cli),
            ("stop_episode", self._stop_episode_cli),
            ("set_task_label", self._set_task_label_cli),
            ("get_session_state", self._get_session_state_cli),
        ]

        for name, client in clients:
            remaining_ns = deadline_ns - self._node.get_clock().now().nanoseconds
            remaining_sec = max(0.0, remaining_ns / 1e9)
            if not client.wait_for_service(timeout_sec=remaining_sec):
                raise RuntimeError(
                    f"SessionControlClient: service '{name}' not available "
                    f"within {total_timeout_sec:.1f}s"
                )

        self._logger.info("[session_control_client] all 6 services ready")

    def _call_trigger_sync(self, client: Client, timeout_sec: float) -> tuple[bool, str]:
        """Trigger 서비스를 동기로 호출하는 내부 헬퍼."""
        if not client.service_is_ready():
            return False, _NOT_READY_MSG

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)

        if not future.done():
            return False, _CALL_TIMEOUT_MSG

        try:
            response = future.result()
        except Exception as exc:
            self._logger.error(f'[session_control_client] service call raised: {exc}')
            return False, f'exception: {exc}'
        if response is None:
            return False, _NO_RESPONSE_MSG
        return bool(response.success), str(response.message)

    def _call_trigger_async(self, client: Client,
                            done_callback: Callable[[bool, str], None] | None,) -> Future:
        """Trigger 서비스를 비동기로 호출하는 내부 헬퍼."""
        future = client.call_async(Trigger.Request())
        if done_callback is not None:
            future.add_done_callback(
                lambda fut, cb=done_callback:
                    self._invoke_bool_message_callback(cb, fut)
            )
        return future

    def _invoke_bool_message_callback(self, callback: Callable[[bool, str], None],
                                      future: Future,) -> None:
        """Trigger/SetString future 의 결과를 (bool, str) 로 콜백에 전달."""
        try:
            response = future.result()
        except Exception as exc:
            self._logger.error(f"[session_control_client] service call raised: {exc}")
            callback(False, f"exception: {exc}")
            return

        if response is None:
            callback(False, _NO_RESPONSE_MSG)
            return

        callback(bool(response.success), str(response.message))

    def _invoke_state_callback(self, callback: Callable[[str, str], None], future: Future,) -> None:
        """get_session_state future 의 결과를 (str, str) 로 콜백에 전달."""
        try:
            response = future.result()
        except Exception as exc:
            self._logger.error(f"[session_control_client] get_session_state raised: {exc}")
            callback("", "")
            return

        if response is None:
            callback("", "")
            return

        callback(str(response.state), str(response.task_label))
