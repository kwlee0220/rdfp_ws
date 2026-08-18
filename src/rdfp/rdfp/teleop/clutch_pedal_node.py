"""USB HID 풋페달로 teleop 클러치를 제어하는 노드.

``evdev`` 로 ``/dev/input/event*`` 를 직접 읽어 **키 누름과 뗌을 모두** 감지한다.
터미널 raw/cbreak 모드로는 뗌을 알 수 없어 hold(데드맨) 방식을 구현할 수 없기
때문이다.

동작 방식 (``mode`` 파라미터)
------------------------------

``hold`` (기본)
    밟는 동안만 engage. 떼면 즉시 disengage — **데드맨**. 밟고 있는 동안
    ``pedal_heartbeat`` 를 ``heartbeat_rate`` 로 발행한다.

``toggle``
    밟을 때마다 engage/disengage 전환. 뗌 이벤트는 무시하고 하트비트도 보내지
    않는다. 데드맨이 아니므로 안전이 걸리는 용도에는 권하지 않는다.

데드맨을 완성하려면 teleop_retarget 쪽 ``pedal_timeout`` 도 켜야 한다
--------------------------------------------------------------------

본 노드가 죽거나 USB 가 빠지면 disengage 를 보낼 주체가 사라져 클러치가 물린
채로 남는다. 그래서 ``hold`` 모드에서는 밟고 있는 동안 하트비트를 계속 보내고,
``teleop_retarget`` 은 ``pedal_timeout`` 안에 하트비트가 없으면 자동 해제한다.
**``pedal_timeout`` 이 0(기본)이면 이 보호가 동작하지 않는다.**

.. code-block:: bash

   ros2 run rdfp teleop_retarget --ros-args -p pedal_timeout:=0.3

장치 지정
---------

``device_path`` 를 주면 그대로 열고, 비어 있으면 ``device_name`` 부분 문자열로
찾는다. ``/dev/input/eventN`` 번호는 재부팅·재연결 시 바뀌므로
``/dev/input/by-id/...`` 심볼릭 링크나 ``device_name`` 매칭을 권한다.

.. code-block:: bash

   # 연결된 장치 이름 확인
   grep -E "^N: Name" /proc/bus/input/devices

   # 어떤 키코드를 보내는지 확인 (페달을 밟아 본다)
   python3 -m evdev.evtest

파라미터
--------

==================  =======================  ==========================================
이름                기본값                   설명
==================  =======================  ==========================================
device_path         ''                       장치 경로. 비우면 device_name 으로 탐색
device_name         'pedal'                  장치 이름 부분 문자열 (대소문자 무시)
key_code            ''                       대상 키 이름(예: 'KEY_A', 'BTN_LEFT').
                                             비우면 **아무 키나** 페달로 취급한다
mode                'hold'                   'hold' | 'toggle'
heartbeat_rate      10.0                     hold 모드에서 하트비트 발행 주기(Hz)
clutch_node_name    'teleop_retarget'        클러치 노드 이름 (서비스/토픽 prefix)
grab                False                    True 면 장치를 독점(EVIOCGRAB)해 키
                                             입력이 데스크톱/터미널로 새지 않는다
reconnect_interval  2.0                      장치 재탐색 주기(초). 0 이하면 재시도 안 함
==================  =======================  ==========================================
"""

from __future__ import annotations

from typing import Any, Optional

import select

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty

from rdfp.ros2_utils import get_parameter, parse_bool, parse_float, parse_str
from rdfp.teleop.clutch_client import ClutchClient


_MODE_HOLD = 'hold'
_MODE_TOGGLE = 'toggle'
_VALID_MODES = (_MODE_HOLD, _MODE_TOGGLE)

# evdev EV_KEY 이벤트의 value. 2(자동반복)는 새 누름이 아니므로 무시한다.
_KEY_RELEASE = 0
_KEY_PRESS = 1

# 장치 이벤트 폴링 주기(초).
_POLL_PERIOD = 0.01


def _import_evdev() -> Any:
    """evdev 를 지연 import 한다. 없으면 설치 방법을 담은 오류를 낸다."""
    try:
        import evdev
    except ImportError as exc:  # pragma: no cover - 환경 의존
        raise RuntimeError(
            'python3-evdev is required for clutch_pedal_node. '
            'Install it with: sudo apt install python3-evdev'
        ) from exc
    return evdev


class ClutchPedalNode(Node):
    """USB HID 풋페달의 누름/뗌을 클러치 engage/disengage 로 옮긴다."""

    def __init__(self) -> None:
        super().__init__('clutch_pedal')

        self.declare_parameter('device_path', '')
        self.declare_parameter('device_name', 'pedal')
        self.declare_parameter('key_code', '')
        self.declare_parameter('mode', _MODE_HOLD)
        self.declare_parameter('heartbeat_rate', 10.0)
        self.declare_parameter('clutch_node_name', 'teleop_retarget')
        self.declare_parameter('grab', False)
        self.declare_parameter('reconnect_interval', 2.0)

        self._device_path = get_parameter(self, 'device_path', parse_str)
        self._device_name = get_parameter(self, 'device_name', parse_str)
        self._key_code = get_parameter(self, 'key_code', parse_str)
        self._mode = get_parameter(self, 'mode', parse_str)
        self._grab = get_parameter(self, 'grab', parse_bool)
        self._reconnect_interval = get_parameter(self, 'reconnect_interval', parse_float)

        if self._mode not in _VALID_MODES:
            raise ValueError(f"'mode' must be one of {_VALID_MODES}, got {self._mode!r}")

        heartbeat_rate = get_parameter(self, 'heartbeat_rate', parse_float)
        if self._mode == _MODE_HOLD and heartbeat_rate <= 0.0:
            raise ValueError(f"'heartbeat_rate' must be > 0 in hold mode, got {heartbeat_rate}")

        self._evdev = _import_evdev()
        clutch_node_name = get_parameter(self, 'clutch_node_name', parse_str)
        self._client = ClutchClient.create(self, clutch_node_name)

        prefix = clutch_node_name if clutch_node_name.startswith('/') else f'/{clutch_node_name}'
        self._heartbeat_pub = self.create_publisher(
            Empty, f'{prefix.rstrip("/")}/pedal_heartbeat', 10)

        self._device: Optional[Any] = None
        self._pressed = False

        self._poll_timer = self.create_timer(_POLL_PERIOD, self._poll_device)
        # hold 모드에서만 하트비트를 보낸다. toggle 은 데드맨이 아니다.
        self._heartbeat_timer = None
        if self._mode == _MODE_HOLD:
            self._heartbeat_timer = self.create_timer(1.0 / heartbeat_rate, self._publish_heartbeat)
        if self._reconnect_interval > 0.0:
            self._reconnect_timer = self.create_timer(self._reconnect_interval, self._ensure_device)

        self.get_logger().info(
            f'ClutchPedalNode started: mode={self._mode!r}, '
            f'device={self._device_path or self._device_name!r}, '
            f'key={self._key_code or "(any)"}, clutch={prefix}, '
            f'heartbeat={heartbeat_rate if self._mode == _MODE_HOLD else "off"}'
        )
        self._ensure_device()

    # ------------------------------------------------------------------
    # 장치 연결
    # ------------------------------------------------------------------

    def _find_device(self) -> Optional[Any]:
        """device_path 또는 device_name 으로 입력 장치를 찾는다."""
        evdev = self._evdev
        if self._device_path:
            try:
                return evdev.InputDevice(self._device_path)
            except OSError as exc:
                self.get_logger().warning(f'cannot open {self._device_path}: {exc}')
                return None

        needle = self._device_name.lower()
        for path in evdev.list_devices():
            try:
                dev = evdev.InputDevice(path)
            except OSError:
                continue
            if needle in dev.name.lower():
                return dev
            dev.close()
        return None

    def _ensure_device(self) -> None:
        """장치가 없으면 다시 찾아 연다."""
        if self._device is not None:
            return
        device = self._find_device()
        if device is None:
            return
        if self._grab:
            try:
                device.grab()
            except OSError as exc:
                self.get_logger().warning(f'cannot grab {device.path}: {exc}')
        self._device = device
        self.get_logger().info(f'pedal connected: {device.path} ({device.name})')

    def _drop_device(self, reason: str) -> None:
        """장치를 놓고, 밟힌 상태였으면 즉시 해제한다.

        USB 가 빠졌을 때 클러치가 물린 채로 남지 않게 하는 것이 핵심이다.
        """
        if self._device is not None:
            try:
                self._device.close()
            except OSError:
                pass
            self._device = None
        self.get_logger().warning(f'pedal disconnected: {reason}')
        if self._pressed:
            self._pressed = False
            self._client.disengage_async(self._log_result)

    # ------------------------------------------------------------------
    # 이벤트 처리
    # ------------------------------------------------------------------

    def _poll_device(self) -> None:
        """장치 이벤트를 논블로킹으로 읽어 처리한다."""
        device = self._device
        if device is None:
            return
        try:
            ready, _, _ = select.select([device.fd], [], [], 0.0)
            if not ready:
                return
            for event in device.read():
                self._handle_event(event)
        except OSError as exc:
            self._drop_device(str(exc))

    def _handle_event(self, event: Any) -> None:
        """EV_KEY 이벤트를 클러치 명령으로 옮긴다."""
        evdev = self._evdev
        if event.type != evdev.ecodes.EV_KEY:
            return
        # value 2 는 자동반복이라 새 누름이 아니다.
        if event.value not in (_KEY_PRESS, _KEY_RELEASE):
            return
        if self._key_code and not self._matches_key(event.code):
            return

        if event.value == _KEY_PRESS:
            self._on_press()
        else:
            self._on_release()

    def _matches_key(self, code: int) -> bool:
        """이벤트 코드가 key_code 파라미터와 일치하는지 확인한다."""
        names = self._evdev.ecodes.bytype[self._evdev.ecodes.EV_KEY].get(code)
        if names is None:
            return False
        if isinstance(names, str):
            names = [names]
        return self._key_code in names

    def _on_press(self) -> None:
        """페달을 밟았을 때."""
        if self._pressed:
            return
        self._pressed = True
        if self._mode == _MODE_HOLD:
            self.get_logger().info('pedal pressed -> engage')
            self._client.engage_async(self._log_result)
        else:
            self.get_logger().info('pedal pressed -> toggle')
            self._client.toggle_async(self._log_result)

    def _on_release(self) -> None:
        """페달을 뗐을 때. toggle 모드에서는 무시한다."""
        if not self._pressed:
            return
        self._pressed = False
        if self._mode == _MODE_HOLD:
            self.get_logger().info('pedal released -> disengage')
            self._client.disengage_async(self._log_result)

    def _publish_heartbeat(self) -> None:
        """밟고 있는 동안 하트비트를 발행한다 (hold 모드 전용)."""
        if self._pressed:
            self._heartbeat_pub.publish(Empty())

    def _log_result(self, success: bool, message: str) -> None:
        """클러치 서비스 응답을 로그로 남긴다."""
        if success:
            self.get_logger().info(f'clutch: {message}')
        else:
            self.get_logger().warning(f'clutch call failed: {message}')

    def shutdown(self) -> None:
        """종료 시 클러치를 놓는다 (정상 종료 경로 한정)."""
        if self._pressed:
            self._pressed = False
            self._client.disengage(timeout_sec=1.0)
        if self._device is not None:
            try:
                self._device.close()
            except OSError:
                pass
            self._device = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ClutchPedalNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
