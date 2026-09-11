#!/usr/bin/env python3
"""session_control_node 의 제어 서비스 5 개를 래핑하는 Python 클라이언트.

SessionControlClient 는 호출자의 ROS2 노드를 주입받아 그 위에 서비스 클라이언트를
만들고, 각 서비스에 대한 **동기/비동기** 호출 메서드를 제공한다. 생성 시점에 서비스
5 개가 모두 준비될 때까지 블로킹 대기한다 (``wait_timeout_sec=0`` 으로 끌 수 있다).

래핑하는 것은 상태를 **바꾸는** 서비스뿐이다 — ``start_session`` · ``stop_session`` ·
``start_episode`` · ``stop_episode`` · ``set_task_label``. 서버의 ``get_session_state`` 는
래핑하지 않는다. 현재 상태는 ``session`` 토픽(TRANSIENT_LOCAL) 으로 읽는 것이 정본이라
호출처가 하나도 없었고, 서비스 조회는 CLI 진단용으로만 남긴다.

사용 예:

    client = SessionControlClient.create(node, wait_timeout_sec=10.0)

    # 동기 호출 — 노드가 아직 executor 에서 spin 중이 아닐 때만
    ok, msg = client.start_session()

    # 비동기 호출 — 타이머/콜백 안에서
    client.start_session_async(done_callback=on_done)

    # task clear
    client.set_task_label(None)

    # 세션 노드가 없어도 계속 동작해야 하는 호출자 (teleop_keyboard)
    client = SessionControlClient.create(node, wait_timeout_sec=0.0)
    if not client.wait_until_ready(2.0):
        ...  # 세션 기능만 끈다
"""

from __future__ import annotations

from typing import Callable, Optional

import rclpy
from rclpy.client import Client
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import Trigger

from rdfp_msgs.srv import SetString, StopEpisode


# 기본 서비스 네임스페이스. session_control_node 의 기본 노드 이름과 일치한다 —
# 서버가 서비스를 `~/` 로 열기 때문에 `/session_control/<name>` 이 된다.
# **절대 경로다 — 세션 서버는 시스템에 하나다.** 상대로 두면 호출자 노드가 로봇별
# 네임스페이스 안에 있을 때 `/abc/session_control/...` 를 찾아 영영 준비되지 않는다
# (docs/topic_naming_contract.md §2.5). 토픽과 같은 이유다.
_DEFAULT_NAMESPACE: str = '/session_control'

# 초기화 시 서비스 ready 대기 기본 타임아웃 (초).
_DEFAULT_WAIT_TIMEOUT: float = 10.0

# 개별 서비스 호출 기본 타임아웃 (초).
_DEFAULT_CALL_TIMEOUT: float = 5.0

# 실패 메시지 상수. 호출자는 문자열이 아니라 success 로 분기한다.
_NOT_READY_MSG: str = 'service not ready'
_CALL_TIMEOUT_MSG: str = 'service call timed out'
_NO_RESPONSE_MSG: str = 'no response'

# 비동기 완료 콜백의 형. 인자는 (success, message).
DoneCallback = Callable[[bool, str], None]


class SessionControlClient:
    """session_control_node 의 제어 서비스 5 개를 래핑하는 Python 클라이언트.

    호출자의 `rclpy.node.Node` 를 주입받아 그 위에 서비스 클라이언트를 만든다. 자체
    노드가 아니다. 생성 시점에 서비스가 모두 준비될 때까지 블로킹 대기하며, 타임아웃
    안에 준비되지 않으면 `RuntimeError` 를 낸다.

    **동기 메서드는 노드가 외부 executor 에서 spin 중이 아닐 때만 쓴다.** 내부에서
    `rclpy.spin_until_future_complete(node, ...)` 를 부르므로, 같은 노드가 다른 스레드에서
    spin 중이거나 콜백 안에서 부르면 데드락이다. 그 경우에는 `*_async` 를 쓴다.

    **비동기 메서드는 서버가 없으면 즉시 실패로 끝낸다.** `call_async` 는 미준비 서비스에도
    future 를 돌려주는데 그것이 영영 완료되지 않아 콜백이 불리지 않는다 — 다섯 메서드
    모두 호출 직전에 `service_is_ready()` 를 보고, 아니면 `(False, 'service not ready')`
    로 콜백을 부른 뒤 완료된 future 를 돌려준다.
    """

    def __init__(self, node: Node, namespace: str = _DEFAULT_NAMESPACE,
                 wait_timeout_sec: float = _DEFAULT_WAIT_TIMEOUT) -> None:
        """SessionControlClient 를 초기화한다.

        Args:
            node: 서비스 클라이언트를 붙일 호출자의 ROS2 노드.
            namespace: session_control_node 의 노드 이름(서비스 prefix). 기본값은
                'session_control'. 절대 경로(`/ns/session_control`)도 허용된다.
            wait_timeout_sec: 서비스 5 개가 모두 준비될 때까지 대기할 총 타임아웃.
                0 이하이면 대기하지 않는다 — 이어서 `wait_until_ready()` 로 예외 없이
                확인할 수 있다.

        Raises:
            RuntimeError: 타임아웃 안에 하나 이상의 서비스가 준비되지 않은 경우.
        """
        self._node = node
        self._logger = node.get_logger()

        # 앞에 '/' 가 없으면 상대 경로라 호출자 노드의 네임스페이스에 붙는다.
        prefix = namespace.rstrip('/')

        self._start_session_cli = node.create_client(Trigger, f'{prefix}/start_session')
        self._stop_session_cli = node.create_client(Trigger, f'{prefix}/stop_session')
        self._start_episode_cli = node.create_client(Trigger, f'{prefix}/start_episode')
        self._stop_episode_cli = node.create_client(StopEpisode, f'{prefix}/stop_episode')
        self._set_task_label_cli = node.create_client(SetString, f'{prefix}/set_task_label')

        if wait_timeout_sec > 0.0:
            self._wait_for_services_ready(wait_timeout_sec)

    @classmethod
    def create(cls, node: Node, namespace: str = _DEFAULT_NAMESPACE,
               wait_timeout_sec: float = _DEFAULT_WAIT_TIMEOUT) -> SessionControlClient:
        """생성자와 같은 시그니처의 팩토리. 다른 `*Client` 와의 스타일 통일용이다."""
        return cls(node, namespace=namespace, wait_timeout_sec=wait_timeout_sec)

    # ------------------------------------------------------------------
    # 동기 API
    # ------------------------------------------------------------------

    def start_session(self, timeout_sec: float = _DEFAULT_CALL_TIMEOUT) -> tuple[bool, str]:
        """start_session 을 동기로 부른다. 인자·반환은 `_call_sync` 를 본다."""
        return self._call_sync(self._start_session_cli, Trigger.Request(), timeout_sec)

    def stop_session(self, timeout_sec: float = _DEFAULT_CALL_TIMEOUT) -> tuple[bool, str]:
        """stop_session 을 동기로 부른다.

        `IN_EPISODE` 에서 부르면 서버가 에피소드 종료와 세션 종료를 한 호출로 처리한다.
        """
        return self._call_sync(self._stop_session_cli, Trigger.Request(), timeout_sec)

    def start_episode(self, timeout_sec: float = _DEFAULT_CALL_TIMEOUT) -> tuple[bool, str]:
        """start_episode 를 동기로 부른다."""
        return self._call_sync(self._start_episode_cli, Trigger.Request(), timeout_sec)

    def stop_episode(self, outcome: str = '', metadata: str = '',
                     timeout_sec: float = _DEFAULT_CALL_TIMEOUT) -> tuple[bool, str]:
        """stop_episode 를 동기로 부른다.

        다섯 서비스 중 유일하게 `Trigger` 가 아니다 — 종료 시점에만 알 수 있는 성패·
        부가정보를 받아 `SessionCommand` 로 흘려야 rosbag 에 기록되기 때문이다.

        Args:
            outcome: 작업 성패 (`''` / `'success'` / `'failure'`). **`''` 는 실패가 아니라
                '판정 없음'** 이며 DB 에 NULL 로 들어간다. teleop 처럼 값을 줄 수단이 없는
                경로는 비운 채 부른다.
            metadata: 부가 정보의 JSON object 문자열 (seed, scene, 초기 배치 등). 없으면
                `''`. 최상위가 object 가 아니면 서버가 거부한다.
            timeout_sec: 서비스 호출 타임아웃(초). 0 이하이면 무한 대기.
        Returns:
            (success, message). success 는 **명령 수용 여부**이며 작업의 성패가 아니다
            (그것은 인자 `outcome` 이다). 서버가 값을 거부하면 message 에 사유가 담긴다.
        """
        return self._call_sync(self._stop_episode_cli,
                               self._stop_episode_request(outcome, metadata), timeout_sec)

    def set_task_label(self, task_label: Optional[str],
                       timeout_sec: float = _DEFAULT_CALL_TIMEOUT) -> tuple[bool, str]:
        """set_task_label 을 동기로 부른다.

        Args:
            task_label: 설정할 task label. `None` 은 빈 문자열로 바꿔 보내며 task clear 다.
            timeout_sec: 서비스 호출 타임아웃(초). 0 이하이면 무한 대기.
        Returns:
            (success, message). `IN_EPISODE` 에서는 서버가 거부한다.
        """
        return self._call_sync(self._set_task_label_cli,
                               self._set_task_label_request(task_label), timeout_sec)

    # ------------------------------------------------------------------
    # 비동기 API
    # ------------------------------------------------------------------

    def start_session_async(self, done_callback: Optional[DoneCallback] = None) -> Future:
        """start_session 을 비동기로 부른다.

        Args:
            done_callback: 응답 수신 시 `(success, message)` 로 불린다. 서버가 없으면
                즉시 `(False, 'service not ready')`, 호출이 예외로 끝나면
                `(False, 'exception: ...')`, 응답이 없으면 `(False, 'no response')`.
                None 이면 콜백 없이 raw future 만 돌려준다.

        Returns:
            `rclpy.task.Future`. `result()` 로 원본 응답에 접근할 수 있다. 서버가 없을 때는
            결과가 `None` 인 **완료된** future 다.
        """
        return self._call_async(self._start_session_cli, Trigger.Request(), done_callback)

    def stop_session_async(self, done_callback: Optional[DoneCallback] = None) -> Future:
        """stop_session 을 비동기로 부른다. 콜백 규약은 `start_session_async` 와 같다."""
        return self._call_async(self._stop_session_cli, Trigger.Request(), done_callback)

    def start_episode_async(self, done_callback: Optional[DoneCallback] = None) -> Future:
        """start_episode 를 비동기로 부른다. 콜백 규약은 `start_session_async` 와 같다."""
        return self._call_async(self._start_episode_cli, Trigger.Request(), done_callback)

    def stop_episode_async(self, outcome: str = '', metadata: str = '',
                           done_callback: Optional[DoneCallback] = None) -> Future:
        """stop_episode 를 비동기로 부른다.

        인자 의미는 :meth:`stop_episode` 와 같다. ``done_callback`` 이 세 번째 인자이므로
        키워드로 넘긴다.
        """
        return self._call_async(self._stop_episode_cli,
                                self._stop_episode_request(outcome, metadata), done_callback)

    def set_task_label_async(self, task_label: Optional[str],
                             done_callback: Optional[DoneCallback] = None) -> Future:
        """set_task_label 을 비동기로 부른다. `None` 은 task clear 다."""
        return self._call_async(self._set_task_label_cli,
                                self._set_task_label_request(task_label), done_callback)

    # ------------------------------------------------------------------
    # 준비 상태
    # ------------------------------------------------------------------

    def is_ready(self) -> bool:
        """서비스 5 개가 모두 ready 인지 블로킹 없이 확인한다."""
        return all(client.service_is_ready() for _, client in self._all_clients())

    def wait_until_ready(self, timeout_sec: float) -> bool:
        """서비스가 모두 준비됐는지 **예외 없이** 확인한다.

        `create()` 는 실패를 `RuntimeError` 로 내지만 이쪽은 ``False`` 로 돌려준다.
        `session_control_node` 는 수집 계층(`rdfp`) 소속이라 제어 스택만 띄운 경우
        존재하지 않는데, 그때도 계속 동작해야 하는 호출자를 위한 것이다.

        Args:
            timeout_sec: 총 대기 한도(초). 0 이하이면 대기 없이 현재 상태만 본다.

        Returns:
            서비스 5 개가 모두 준비됐으면 ``True``.
        """
        try:
            if timeout_sec > 0.0:
                self._wait_for_services_ready(timeout_sec)
            return self.is_ready()
        except RuntimeError:
            return False

    # ------------------------------------------------------------------
    # 내부 헬퍼
    # ------------------------------------------------------------------

    def _all_clients(self) -> list[tuple[str, Client]]:
        """(서비스 이름, 클라이언트) 목록. 속성에서 매번 읽는다 — 테스트가 속성을 바꿔 끼운다."""
        return [
            ('start_session', self._start_session_cli),
            ('stop_session', self._stop_session_cli),
            ('start_episode', self._start_episode_cli),
            ('stop_episode', self._stop_episode_cli),
            ('set_task_label', self._set_task_label_cli),
        ]

    @staticmethod
    def _stop_episode_request(outcome: str, metadata: str) -> StopEpisode.Request:
        request = StopEpisode.Request()
        request.outcome = outcome
        request.metadata = metadata
        return request

    @staticmethod
    def _set_task_label_request(task_label: Optional[str]) -> SetString.Request:
        request = SetString.Request()
        request.task_label = '' if task_label is None else task_label
        return request

    def _wait_for_services_ready(self, total_timeout_sec: float) -> None:
        """서비스 5 개가 모두 ready 가 될 때까지 블로킹 대기한다.

        예산은 **공유된다** — 각 서비스에 남은 시간 전체를 주므로 앞의 것이 오래 걸리면
        뒤의 것에 남는 시간이 줄어든다. 하나라도 시간 안에 준비되지 않으면
        `RuntimeError` 를 내며, 메시지에 그 서비스 이름이 들어 있다.
        """
        deadline_ns = self._node.get_clock().now().nanoseconds + int(total_timeout_sec * 1e9)
        for name, client in self._all_clients():
            remaining_ns = deadline_ns - self._node.get_clock().now().nanoseconds
            remaining_sec = max(0.0, remaining_ns / 1e9)
            if not client.wait_for_service(timeout_sec=remaining_sec):
                raise RuntimeError(
                    f"SessionControlClient: service '{name}' not available "
                    f'within {total_timeout_sec:.1f}s')
        self._logger.info('[session_control_client] all 5 services ready')

    def _call_sync(self, client: Client, request, timeout_sec: float) -> tuple[bool, str]:
        """(success, message) 응답을 갖는 서비스를 동기로 부른다.

        Args:
            timeout_sec: 0 이하이면 **무한 대기**다. rclpy 는 0 을 '즉시 반환' 으로 다루므로
                `None` 으로 바꿔 넘긴다 — 그대로 넘기면 응답이 오기 전에 timed out 을
                돌려주는데 서버는 이미 명령을 처리한 뒤라 호출자가 실패로 오해한다 (실측).

        Returns:
            (success, message). 서버가 거부하면 success=False 와 서버의 사유, 서비스가
            없으면 'service not ready', 시간 안에 응답이 없으면 'service call timed out'.
            **타임아웃은 명령이 취소됐다는 뜻이 아니다** — 서버는 처리했을 수 있다.
        """
        if not client.service_is_ready():
            return False, _NOT_READY_MSG

        future = client.call_async(request)
        spin_timeout = None if timeout_sec <= 0.0 else timeout_sec
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=spin_timeout)

        if not future.done():
            return False, _CALL_TIMEOUT_MSG

        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - 어떤 예외든 (False, 사유) 로 바꾼다
            self._logger.error(f'[session_control_client] service call raised: {exc}')
            return False, f'exception: {exc}'
        if response is None:
            return False, _NO_RESPONSE_MSG
        return bool(response.success), str(response.message)

    def _call_async(self, client: Client, request,
                    done_callback: Optional[DoneCallback]) -> Future:
        """(success, message) 응답을 갖는 서비스를 비동기로 부른다.

        `Trigger` / `SetString` / `StopEpisode` 는 요청 타입만 다르고 응답 구조가 같으므로
        요청 객체만 받아 공통 처리한다.

        **서버가 없으면 즉시 실패로 끝낸다.** `call_async` 는 미준비 서비스에도 future 를
        돌려주는데 그것이 **영영 완료되지 않아** 콜백이 불리지 않는다 — 호출자는 아무 응답
        없이 기다리게 되고 로그에도 단서가 남지 않는다.
        """
        if not client.service_is_ready():
            future = Future()
            future.set_result(None)
            if done_callback is not None:
                done_callback(False, _NOT_READY_MSG)
            return future

        future = client.call_async(request)
        if done_callback is not None:
            future.add_done_callback(
                lambda fut, cb=done_callback: self._invoke_done_callback(cb, fut))
        return future

    def _invoke_done_callback(self, callback: DoneCallback, future: Future) -> None:
        """future 의 결과를 (success, message) 로 콜백에 전달한다."""
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self._logger.error(f'[session_control_client] service call raised: {exc}')
            callback(False, f'exception: {exc}')
            return

        if response is None:
            callback(False, _NO_RESPONSE_MSG)
            return

        callback(bool(response.success), str(response.message))
