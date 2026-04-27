"""SessionControlNode 모듈.

세션/에피소드 생명주기를 관리하는 ROS2 노드. 외부 클라이언트는 5 개의
서비스(`start_session`, `stop_session`, `start_episode`, `stop_episode`,
`set_task_label`)로 제어 명령을 전달하고, 본 노드는 내부 상태 머신을 갱신한
뒤 변경된 상태와 task_label 을 `session` 토픽으로 발행한다.

상세 스펙은 `session_control_srs.md`, 개발 절차는
`session_control_plan.md` 를 참조한다.
"""

from __future__ import annotations

from typing import Optional

import sys
from enum import Enum

from rdfp.ros2_utils import SYSTEM_QOS
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger

from rdfp_msgs.msg import SessionCommand
from rdfp_msgs.srv import GetSessionState, SetString


class SessionState(Enum):
    """SessionControlNode 의 상태."""
    IDLE = 'IDLE'
    IN_SESSION = 'IN_SESSION'
    IN_EPISODE = 'IN_EPISODE'



_DEFAULT_SESSION_TOPIC = "session"
_INVALID_COMMAND_MSG: str = 'invalid command'


class SessionControlNode(Node):
    """세션 제어 명령을 서비스로 수신하여 상태 변경을 토픽으로 발행하는 노드.

    상태 전이 규칙과 토픽 발행 규칙은 `session_control_srs.md` 4 절을
    그대로 따른다. 단일 스레드 executor 사용을 전제로 하므로 상태 보호용
    lock 은 두지 않는다.
    """

    def __init__(self) -> None:
        super().__init__('session_control')

        # 내부 상태 변수 초기화 (SRS 4.2).
        self._state: SessionState = SessionState.IDLE
        self._task_label: str = ''

        # Publisher QoS: 늦게 붙은 구독자도 직전 상태를 즉시 수신하도록
        # TRANSIENT_LOCAL / RELIABLE / depth=1 조합을 사용한다 (SRS 3.3.2).
        self._pub = self.create_publisher(SessionCommand, _DEFAULT_SESSION_TOPIC, SYSTEM_QOS)

        # 6 개의 분할된 서비스. std_srvs/Trigger 를 사용하는 4 개와
        # task_label 파라미터를 받는 set_task_label 1 개, 상태 조회 1 개.
        # '~/' prefix 로 /session_control/<name> 형태로 노출한다.
        self._start_session_srv = self.create_service(
            Trigger, '~/start_session', self._handle_start_session,
        )
        self._stop_session_srv = self.create_service(
            Trigger, '~/stop_session', self._handle_stop_session,
        )
        self._start_episode_srv = self.create_service(
            Trigger, '~/start_episode', self._handle_start_episode,
        )
        self._stop_episode_srv = self.create_service(
            Trigger, '~/stop_episode', self._handle_stop_episode,
        )
        self._set_task_label_srv = self.create_service(
            SetString, '~/set_task_label', self._handle_set_task_label,
        )
        self._get_srv = self.create_service(
            GetSessionState,
            '~/get_session_state',
            self._handle_get_state,
        )

        # 초기 상태 발행: 늦게 붙은 구독자가 TRANSIENT_LOCAL 로 즉시 수신.
        self._publish(self._state, self._task_label)

        self.get_logger().info(
            "SessionControlNode started (state=IDLE, task_label='')"
        )

    # ------------------------------------------------------------------
    # 서비스 콜백
    # ------------------------------------------------------------------

    def _handle_start_session(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        # IDLE 에서만 허용 (SRS 4.3).
        if self._state is not SessionState.IDLE:
            return self._reject('start_session', response)

        self._publish(SessionState.IN_SESSION, self._task_label)
        self._transition(SessionState.IN_SESSION, 'start_session')
        return self._ok(response)

    def _handle_stop_session(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        # IN_SESSION 에서는 단일 발행으로 IDLE 전이.
        if self._state is SessionState.IN_SESSION:
            self._publish(SessionState.IDLE, self._task_label)
            self._transition(SessionState.IDLE, 'stop_session')
            return self._ok(response)

        # IN_EPISODE 에서는 SRS 4.3 에 따라 (IN_SESSION, <L>) -> (IDLE, <L>)
        # 순서로 2 회 발행한 뒤 IDLE 로 전이한다. 에피소드 종료와 세션 종료를
        # 구독자가 순서대로 관찰할 수 있도록 하기 위함이다.
        if self._state is SessionState.IN_EPISODE:
            self._publish(SessionState.IN_SESSION, self._task_label)
            self._publish(SessionState.IDLE, self._task_label)
            self._transition(SessionState.IDLE, 'stop_session')
            return self._ok(response)

        return self._reject('stop_session', response)

    def _handle_start_episode(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        # IN_SESSION 에서만 허용 (SRS 4.3).
        if self._state is not SessionState.IN_SESSION:
            return self._reject('start_episode', response)

        self._publish(SessionState.IN_EPISODE, self._task_label)
        self._transition(SessionState.IN_EPISODE, 'start_episode')
        return self._ok(response)

    def _handle_stop_episode(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        # IN_EPISODE 에서만 허용 (SRS 4.3).
        if self._state is not SessionState.IN_EPISODE:
            return self._reject('stop_episode', response)

        self._publish(SessionState.IN_SESSION, self._task_label)
        self._transition(SessionState.IN_SESSION, 'stop_episode')
        return self._ok(response)

    def _handle_set_task_label(
        self,
        request: SetString.Request,
        response: SetString.Response,
    ) -> SetString.Response:
        label = request.task_label

        # set_task_label 은 IDLE / IN_SESSION 에서만 허용된다 (SRS 4.3).
        if self._state is SessionState.IN_EPISODE:
            return self._reject(f"set_task_label='{label}'", response)

        # 토픽 발행 후 내부 task_label 갱신 (SRS 4.4: 발행 -> 전이/갱신 순서).
        self._publish(self._state, label)
        self._task_label = label
        self.get_logger().info(f"task_label set to '{label}'")
        return self._ok(response)

    def _handle_get_state(
        self,
        request: GetSessionState.Request,
        response: GetSessionState.Response,
    ) -> GetSessionState.Response:
        # 내부 상태를 변경하지 않고 현재 state / task_label 을 그대로 반환.
        response.state = self._state.value
        response.task_label = self._task_label
        return response

    # ------------------------------------------------------------------
    # 내부 헬퍼
    # ------------------------------------------------------------------

    def _publish(self, state: SessionState, task_label: str) -> None:
        msg = SessionCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.state = state.value
        msg.task_label = task_label
        self._pub.publish(msg)

    def _transition(self, next_state: SessionState, command: str) -> None:
        prev = self._state
        self._state = next_state
        self.get_logger().info(
            f'state transition: {prev.value} -> {next_state.value} '
            f'(command={command})'
        )

    def _ok(self, response):
        # Trigger.Response 와 SetString.Response 는 동일한 필드 구조(success, message)를
        # 가지므로 단일 헬퍼로 처리할 수 있다.
        response.success = True
        response.message = ''
        return response

    def _reject(self, command: str, response):
        self.get_logger().warning(
            f"invalid command '{command}' in state {self._state.value}"
        )
        response.success = False
        response.message = _INVALID_COMMAND_MSG
        return response


def main(args: Optional[list[str]] = None) -> None:
    """Console entry point for session_control_node.

    ``rclpy.init()`` 호출 직후부터 외곽 ``try/finally`` 로 감싸 ``__init__`` 또는
    ``executor.add_node`` 단계에서 ``KeyboardInterrupt`` / 예외가 발생해도
    ``destroy_node()`` 와 ``rclpy.try_shutdown()`` 이 반드시 실행되도록 한다.
    """
    rclpy.init(args=args)

    node: Optional[SessionControlNode] = None
    try:
        try:
            node = SessionControlNode()
        except Exception as exc:
            print(
                f'[FATAL] SessionControlNode init failed: {exc}',
                file=sys.stderr,
            )
            sys.exit(1)

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
