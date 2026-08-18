"""teleop_retarget 의 클러치 인터페이스를 래핑하는 Python 클라이언트.

호출자의 :class:`rclpy.node.Node` 를 주입받아 그 위에 서비스 클라이언트와
상태 구독을 올린다 (``ServoClient`` / ``SessionControlClient`` 와 같은 방식).

래핑 대상
---------

===========================  ==========================  ==========================
이름                         타입                        용도
===========================  ==========================  ==========================
``<node>/clutch``            ``std_srvs/SetBool``        engage / disengage
``<node>/clutch_state``      ``rdfp_msgs/ClutchState``   상태 변경 구독
``<node>/get_clutch_state``  ``std_srvs/Trigger``        동기 조회
===========================  ==========================  ==========================

직접 쓸 때 틀리기 쉬운 두 가지를 감춘다.

1. ``clutch_state`` 는 **TRANSIENT_LOCAL** 이다. 기본 QoS 로 구독하면 래치된
   값을 못 받아 **조용히 아무것도 안 온다**.
2. 동기 호출은 내부에서 ``rclpy.spin_until_future_complete`` 를 쓰므로, 이미
   spin 중인 노드(GUI mainloop, 키 입력 루프)에서 부르면 데드락이 난다. 그런
   경우 ``*_async`` 를 쓴다.

사용 예
-------

.. code-block:: python

   client = ClutchClient.create(node)
   client.on_change(lambda engaged, reason: print(engaged, reason))

   client.engage_async()          # 페달 press
   client.disengage_async()       # 페달 release
   client.toggle_async()          # 캐시된 상태를 보고 반대로

   if client.engaged:             # 마지막 수신 상태 (None = 아직 미수신)
       ...
"""

from __future__ import annotations

from typing import Callable, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future

from rdfp_msgs.msg import ClutchState
from std_srvs.srv import SetBool, Trigger


# teleop_retarget 노드의 기본 이름. 서비스/토픽 prefix 로 쓰인다.
_DEFAULT_NODE_NAME = 'teleop_retarget'

# 동기 호출 기본 타임아웃(초).
_DEFAULT_CALL_TIMEOUT = 5.0

# 상태 토픽 QoS. 발행 측(teleop_retarget)과 반드시 일치해야 한다.
_CLUTCH_STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)

_NOT_READY_MSG = 'clutch service not ready'
_CALL_TIMEOUT_MSG = 'clutch service call timed out'
_NO_RESPONSE_MSG = 'no response'


class ClutchClient:
    """teleop_retarget 클러치의 제어·상태를 함께 다루는 클라이언트."""

    def __init__(self, node: Node, node_name: str = _DEFAULT_NODE_NAME) -> None:
        """클라이언트를 초기화한다.

        서비스 준비를 기다리지 않는다 — 페달·GUI 는 teleop_retarget 보다 먼저
        떠 있을 수 있고, 그때 생성자가 막히면 곤란하기 때문이다. 준비 여부는
        :meth:`wait_for_service` 로 확인한다.

        Args:
            node: 클라이언트를 붙일 호출자 노드.
            node_name: teleop_retarget 노드 이름. 앞의 ``/`` 는 없어도 된다.
        """
        self._node = node
        prefix = node_name if node_name.startswith('/') else f'/{node_name}'
        self._prefix = prefix.rstrip('/')

        self._clutch_cli = node.create_client(SetBool, f'{self._prefix}/clutch')
        self._state_cli = node.create_client(Trigger, f'{self._prefix}/get_clutch_state')
        self._state_sub = node.create_subscription(
            ClutchState, f'{self._prefix}/clutch_state', self._on_state, _CLUTCH_STATE_QOS)

        # 마지막으로 수신한 상태. 아직 못 받았으면 None.
        self._engaged: Optional[bool] = None
        self._reason: str = ''
        self._callbacks: list[Callable[[bool, str], None]] = []

    @classmethod
    def create(cls, node: Node, node_name: str = _DEFAULT_NODE_NAME) -> 'ClutchClient':
        """ClutchClient 인스턴스를 생성한다."""
        return cls(node, node_name)

    # ------------------------------------------------------------------
    # 상태
    # ------------------------------------------------------------------

    @property
    def engaged(self) -> Optional[bool]:
        """마지막으로 수신한 engaged 여부. 아직 상태를 못 받았으면 ``None``."""
        return self._engaged

    @property
    def reason(self) -> str:
        """마지막 상태 메시지. engaged 면 ``'engaged'``, 아니면 해제 사유."""
        return self._reason

    def on_change(self, callback: Callable[[bool, str], None]) -> None:
        """상태 변경 콜백을 등록한다. ``callback(engaged, reason)`` 로 호출된다.

        등록 시점에 이미 상태를 받아 두었으면 **즉시 한 번 호출**한다. GUI 가
        늦게 붙어도 현재 상태를 바로 그릴 수 있게 하기 위함이다.
        """
        self._callbacks.append(callback)
        if self._engaged is not None:
            callback(self._engaged, self._reason)

    def _on_state(self, msg: ClutchState) -> None:
        """상태 토픽 콜백. 캐시를 갱신하고 등록된 콜백에 전파한다."""
        self._engaged = bool(msg.engaged)
        self._reason = str(msg.reason)
        for cb in self._callbacks:
            try:
                cb(self._engaged, self._reason)
            except Exception as exc:  # noqa: BLE001 - 콜백 실패가 구독을 죽이면 안 된다
                self._node.get_logger().warning(f'clutch state callback failed: {exc}')

    def wait_for_service(self, timeout_sec: float = _DEFAULT_CALL_TIMEOUT) -> bool:
        """``clutch`` 서비스가 준비될 때까지 대기한다."""
        return self._clutch_cli.wait_for_service(timeout_sec=timeout_sec)

    # ------------------------------------------------------------------
    # 제어 — 비동기 (이미 spin 중인 노드에서 쓴다)
    # ------------------------------------------------------------------

    def engage_async(self, done_callback: Optional[Callable[[bool, str], None]] = None
                     ) -> Optional[Future]:
        """클러치를 잡는다. 서비스가 준비되지 않았으면 ``None`` 을 반환한다."""
        return self._call_async(True, done_callback)

    def disengage_async(self, done_callback: Optional[Callable[[bool, str], None]] = None
                        ) -> Optional[Future]:
        """클러치를 놓는다."""
        return self._call_async(False, done_callback)

    def toggle_async(self, done_callback: Optional[Callable[[bool, str], None]] = None
                     ) -> Optional[Future]:
        """캐시된 상태의 반대로 전환한다.

        상태를 아직 못 받았으면 engage 를 시도한다 (초기 상태는 disengaged 이고,
        teleop_retarget 이 TRANSIENT_LOCAL 로 기동 직후 발행하므로 정상적으로는
        곧 채워진다).
        """
        return self._call_async(not bool(self._engaged), done_callback)

    def _call_async(self, engage: bool,
                    done_callback: Optional[Callable[[bool, str], None]]) -> Optional[Future]:
        """`clutch` 서비스를 비동기 호출한다."""
        if not self._clutch_cli.service_is_ready():
            self._node.get_logger().warning(f'{_NOT_READY_MSG}: {self._prefix}/clutch')
            if done_callback is not None:
                done_callback(False, _NOT_READY_MSG)
            return None

        request = SetBool.Request()
        request.data = engage
        future = self._clutch_cli.call_async(request)
        if done_callback is not None:
            future.add_done_callback(lambda fut: self._forward(fut, done_callback))
        return future

    def _forward(self, future: Future, done_callback: Callable[[bool, str], None]) -> None:
        """서비스 응답을 ``(success, message)`` 로 풀어 콜백에 넘긴다."""
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            done_callback(False, f'clutch call failed: {exc}')
            return
        if response is None:
            done_callback(False, _NO_RESPONSE_MSG)
            return
        done_callback(bool(response.success), str(response.message))

    # ------------------------------------------------------------------
    # 제어·조회 — 동기 (spin 하지 않는 스크립트용)
    # ------------------------------------------------------------------

    def engage(self, timeout_sec: float = _DEFAULT_CALL_TIMEOUT) -> Tuple[bool, str]:
        """클러치를 잡고 ``(success, message)`` 를 반환한다."""
        return self._call_sync(True, timeout_sec)

    def disengage(self, timeout_sec: float = _DEFAULT_CALL_TIMEOUT) -> Tuple[bool, str]:
        """클러치를 놓고 ``(success, message)`` 를 반환한다."""
        return self._call_sync(False, timeout_sec)

    def get_state(self, timeout_sec: float = _DEFAULT_CALL_TIMEOUT) -> Tuple[bool, str]:
        """현재 상태를 동기 조회한다.

        Returns:
            ``(engaged, message)``. 조회 실패 시 ``(False, <사유>)``.
        """
        if not self._state_cli.service_is_ready():
            return False, _NOT_READY_MSG
        future = self._state_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        if not future.done():
            return False, _CALL_TIMEOUT_MSG
        response = future.result()
        if response is None:
            return False, _NO_RESPONSE_MSG
        return bool(response.success), str(response.message)

    def _call_sync(self, engage: bool, timeout_sec: float) -> Tuple[bool, str]:
        """`clutch` 서비스를 동기 호출한다."""
        if not self._clutch_cli.service_is_ready():
            return False, _NOT_READY_MSG
        request = SetBool.Request()
        request.data = engage
        future = self._clutch_cli.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        if not future.done():
            return False, _CALL_TIMEOUT_MSG
        response = future.result()
        if response is None:
            return False, _NO_RESPONSE_MSG
        return bool(response.success), str(response.message)


__all__ = ['ClutchClient']
