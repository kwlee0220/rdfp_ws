#!/usr/bin/env python3

"""RdfpCameraNode 모듈.

세션 상태에 따라 카메라를 제어하고 이미지를 발행하는 ROS2 노드.

``SessionControlNode`` 가 발행하는 ``session`` 토픽을 구독하여 상태 전이에
따라 ``OpenCvCamera`` 의 open/release 와 캡처 타이머를 관리한다.
``IN_EPISODE`` 구간에서만 ``sensor_msgs/Image`` 를 ``image`` 토픽으로 발행하고,
``IN_SESSION`` 구간에서는 캡처만 수행(프레임 버림)한다.

세션 상태에 종속되므로 제어 계층(`robot_control`)이 아니라 수집 계층에 둔다.
카메라 하드웨어 추상화는 `robot_control.camera.opencv_camera` 를 그대로 쓴다.
"""

from __future__ import annotations

from typing import Any, Optional

import sys

import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from rdfp_msgs.msg import SessionCommand  # type: ignore[import-not-found]

from robot_control.camera.camera_utils import mask_camera_id_for_log, parse_camera_id
from robot_control.camera.opencv_camera import OpenCvCamera
from robot_control.ros2_utils import (SYSTEM_QOS, get_optional_parameter, get_parameter,
                                      log_periodic, parse_float, parse_stripped_str)
from robot_control.types import Fps, Resolution


_READ_FAIL_LOG_INTERVAL_SEC: float = 5.0
_CONVERT_FAIL_LOG_INTERVAL_SEC: float = 5.0
_DEFAULT_FALLBACK_FPS: float = 30.0
_DEFAULT_RECONNECT_INTERVAL_SEC: float = 3.0

# camera_node.py 와 동일한 private namespace(~/) 기본 토픽명을 사용한다.
_DEFAULT_IMAGE_TOPIC = '~/image_raw'
_DEFAULT_CAMERA_INFO_TOPIC = '~/camera_info'
_DEFAULT_CAMERA_STATUS_TOPIC = '~/camera_status'

# camera_status 토픽에 싣는 상태 문자열 (camera_node.py 와 동일한 어휘).
_STATUS_CONNECTED = 'CONNECTED'
_STATUS_DISCONNECTED = 'DISCONNECTED'
_STATUS_ERROR = 'ERROR'

# 카메라가 열려 있어야 하는 세션 상태.
_ACTIVE_STATES = ('IN_SESSION', 'IN_EPISODE')


class RdfpCameraNode(Node):
    """세션 상태에 따라 카메라 캡처·이미지 발행을 제어하는 노드.

    ``session`` 토픽의 state 값에 따라 동작이 결정된다:

    - ``IDLE`` — 카메라 닫힘, 타이머 없음.
    - ``IN_SESSION`` — 카메라 열림, 타이머로 캡처 루프 동작(프레임 버림).
    - ``IN_EPISODE`` — 캡처한 프레임을 ``image`` 토픽으로 발행.

    캡처 중 카메라 연결이 끊기면 캡처 타이머를 멈추고
    ``reconnect_interval_sec`` 주기로 재연결을 시도한다. 재연결에 성공하면
    세션 상태를 그대로 유지한 채 발행이 재개된다.
    """

    def __init__(self, **node_kwargs: Any) -> None:
        """노드를 초기화한다.

        Args:
            **node_kwargs: ``rclpy.node.Node.__init__`` 에 전달되는 추가 키워드
                인자. 테스트에서 ``parameter_overrides`` 주입 용도로 사용한다.
        """
        super().__init__('rdfp_camera_node', **node_kwargs)

        # 1. 파라미터 선언 및 로드
        self._declare_parameters()
        self._load_parameters()

        # 2. OpenCvCamera 인스턴스 생성 (open은 세션 시작 시 수행)
        self._camera = OpenCvCamera(self._camera_id, resolution=self._resolution, fps=self._fps)

        # 3. 퍼블리셔 (image / camera_info / status)
        self._image_pub = self.create_publisher(Image, _DEFAULT_IMAGE_TOPIC,
                                                qos_profile_sensor_data)
        self._camera_info_pub = self.create_publisher(CameraInfo, _DEFAULT_CAMERA_INFO_TOPIC,
                                                      qos_profile_sensor_data)

        # status 토픽은 late-joiner 가 최신 상태를 즉시 받을 수 있도록 TRANSIENT_LOCAL 사용
        self._status_pub = self.create_publisher(String, _DEFAULT_CAMERA_STATUS_TOPIC, SYSTEM_QOS)

        # 4. CvBridge
        self._bridge = CvBridge()

        # 5. 내부 상태 변수
        self._publishing: bool = False
        self._open_failed: bool = False
        self._timer = None
        self._retry_timer = None
        self._prev_state: str = 'IDLE'
        self._last_status: Optional[str] = None
        self._last_read_fail_log_ts: float = 0.0
        self._last_convert_fail_log_ts: float = 0.0
        # CameraInfo 매트릭스 계산에 사용하는 실제 카메라 해상도
        self._actual_resolution: Optional[Resolution] = None

        # TRANSIENT_LOCAL 이므로 초기 상태를 한 번 실어 둔다 — 그러지 않으면
        # 첫 세션 전이 전까지 late-joiner 가 latch 된 값을 하나도 못 받는다.
        self._publish_status(_STATUS_DISCONNECTED)

        # 6. 세션 토픽 구독 (SessionControlNode 발행 QoS와 동일)
        self._session_sub = self.create_subscription(SessionCommand, 'session', self._on_session,
                                                     SYSTEM_QOS)

        # 로그용 마스킹 camera_id (RTSP URL 자격증명 보호)
        self._masked_camera_id = mask_camera_id_for_log(self._camera_id)

        # 파라미터 요약 로그
        self.get_logger().info(
            f'RdfpCameraNode initialized: '
            f'camera_id={self._masked_camera_id} '
            f'fps={self._fps} '
            f'resolution={self._resolution} '
            f'encoding={self._encoding} '
            f'frame_id={self._frame_id} '
            f'reconnect_interval_sec={self._reconnect_interval_sec}'
        )

    # -- 파라미터 -----------------------------------------------------------

    def _declare_parameters(self) -> None:
        """ROS2 파라미터를 선언한다."""
        self.declare_parameter('camera_id', value='0',
                               descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('fps', value=None,
                               descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('resolution', value=None,
                               descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('encoding', 'bgr8')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('reconnect_interval_sec', _DEFAULT_RECONNECT_INTERVAL_SEC)

    def _load_parameters(self) -> None:
        """파라미터 값을 인스턴스 속성으로 로드한다."""
        self._camera_id = get_parameter(self, 'camera_id', parse_camera_id)
        self._fps: Optional[Fps] = get_optional_parameter(self, 'fps', Fps.parse)
        self._resolution: Optional[Resolution] = get_optional_parameter(self, 'resolution',
                                                                        Resolution.parse)
        self._encoding = get_parameter(self, 'encoding', parse_stripped_str)
        self._frame_id = get_parameter(self, 'frame_id', parse_stripped_str)
        # 0 이하이면 재연결을 시도하지 않는다.
        self._reconnect_interval_sec = get_parameter(self, 'reconnect_interval_sec', parse_float)

    # -- 세션 토픽 콜백 -----------------------------------------------------

    def _on_session(self, msg: SessionCommand) -> None:
        """세션 토픽 콜백: state 값에 따라 상태 전이 핸들러를 호출한다."""
        state = msg.state
        prev = self._prev_state

        if state == 'IDLE':
            self._handle_idle()
        elif state == 'IN_SESSION':
            self._handle_in_session(prev)
        elif state == 'IN_EPISODE':
            self._handle_in_episode()
        else:
            self.get_logger().warning(f'unknown session state: {state}')
            return

        self._prev_state = state

    def _handle_idle(self) -> None:
        """IDLE 상태 처리: 타이머 취소, 카메라 해제, 플래그 리셋."""
        self._publishing = False
        self._cancel_timer()
        self._cancel_retry_timer()
        self._camera.release()
        self._actual_resolution = None
        self._open_failed = False
        self._publish_status(_STATUS_DISCONNECTED)
        self.get_logger().info('session IDLE: camera released')

    def _handle_in_session(self, prev_state: str) -> None:
        """IN_SESSION 전이를 처리한다.

        IDLE → IN_SESSION: 카메라 open 후 캡처 타이머를 생성한다.
        IN_EPISODE → IN_SESSION: 이미지 발행만 중단하고 캡처는 계속한다.
        """
        if prev_state == 'IDLE':
            self._open_camera_and_start_timer()

        elif prev_state == 'IN_EPISODE':
            self._publishing = False
            self.get_logger().info('session IN_SESSION: publishing stopped')

    def _handle_in_episode(self) -> None:
        """IN_EPISODE 전이를 처리한다.

        카메라가 아직 열려 있지 않으면(late-join 시나리오) open + 타이머
        생성을 먼저 수행한다. open 에 실패한 상태라면 발행을 시작하지 않는다 —
        재연결 타이머가 살아 있다면 다음 IN_EPISODE 전이에서 다시 시도된다.
        """
        if self._open_failed:
            self.get_logger().warning('session IN_EPISODE ignored: camera open had failed')
            return

        # late-join: 카메라가 열려 있지 않으면 먼저 open + 타이머 생성
        if not self._camera.is_opened:
            self._open_camera_and_start_timer()
            if self._open_failed:
                return

        self._publishing = True
        self.get_logger().info('session IN_EPISODE: publishing started')

    def _open_camera_and_start_timer(self) -> None:
        """카메라를 열고 캡처 타이머를 생성한다.

        실패 시 ``_open_failed`` 를 설정하고 ``ERROR`` status 를 발행한 뒤
        재연결 타이머를 건다.
        """
        # 기존 타이머를 먼저 정리한다 — 재연결/late-join 경로에서 타이머가
        # 중복 생성되면 캡처·발행 주기가 그대로 배가 된다.
        self._cancel_timer()

        try:
            actual_resolution, actual_fps = self._camera.open()
        except RuntimeError as exc:
            self.get_logger().error(f'camera open failed: {exc}')
            self._open_failed = True
            self._actual_resolution = None
            self._publish_status(_STATUS_ERROR)
            self._schedule_reconnect()
            return

        self._cancel_retry_timer()
        self._open_failed = False
        self._last_read_fail_log_ts = 0.0
        self._actual_resolution = actual_resolution

        # 타이머 주기 산출
        if actual_fps > 0:
            period = 1.0 / actual_fps
        else:
            period = 1.0 / _DEFAULT_FALLBACK_FPS

        self._timer = self.create_timer(period, self._timer_callback)
        self._publish_status(_STATUS_CONNECTED)
        self.get_logger().info(f'camera opened (resolution={actual_resolution}, '
                               f'fps={actual_fps:.1f})')

    # -- 타이머 관리 ---------------------------------------------------------

    def _cancel_timer(self) -> None:
        """캡처 타이머를 취소·파기한다.

        **캡처 타이머 콜백 안에서 호출하면 안 된다** — 실행 중인 타이머를
        스스로 파기하게 된다. 끊김 감지 경로는 `_pause_timer()` 로 정지만
        시키고, 파기는 다른 콜백 컨텍스트(세션 콜백/재연결 타이머)에서 한다.
        """
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None

    def _pause_timer(self) -> None:
        """캡처 타이머를 정지만 시킨다(파기하지 않는다)."""
        if self._timer is not None and not self._timer.is_canceled():
            self._timer.cancel()

    def _schedule_reconnect(self) -> None:
        """재연결 타이머를 건다. 이미 동작 중이면 아무것도 하지 않는다.

        타이머는 한 번 만들면 재사용한다(cancel/reset) — 끊김이 반복될 때마다
        새 타이머를 만들면 노드에 타이머가 쌓인다.
        """
        if self._reconnect_interval_sec <= 0.0:
            return

        if self._retry_timer is None:
            self._retry_timer = self.create_timer(self._reconnect_interval_sec,
                                                  self._retry_callback)
        elif self._retry_timer.is_canceled():
            self._retry_timer.reset()
        else:
            return

        self.get_logger().info(f'reconnect scheduled '
                               f'(every {self._reconnect_interval_sec:.1f}s)')

    def _cancel_retry_timer(self) -> None:
        """재연결 타이머를 정지시킨다(파기는 destroy_node 에서 한다)."""
        if self._retry_timer is not None and not self._retry_timer.is_canceled():
            self._retry_timer.cancel()

    def _retry_callback(self) -> None:
        """재연결 타이머 콜백: 세션이 살아 있는 동안 카메라 open 을 재시도한다."""
        if self._prev_state not in _ACTIVE_STATES:
            self._cancel_retry_timer()
            return

        self.get_logger().info(f'retrying camera open (camera_id={self._masked_camera_id})')
        self._open_camera_and_start_timer()

    # -- 타이머 콜백 ---------------------------------------------------------

    def _timer_callback(self) -> None:
        """캡처 타이머 콜백. read() 후 ``_publishing`` 이면 이미지/카메라정보를 발행한다."""
        frame = self._camera.read()
        # stamp 는 캡처 직후에 찍는다 — 변환·발행 뒤에 찍으면 cv_bridge 변환
        # 시간만큼 밀린 값이 데이터셋에 그대로 남는다.
        stamp = self.get_clock().now().to_msg()

        if frame is None:
            self._last_read_fail_log_ts = log_periodic(
                self.get_logger().warning,
                f'frame read failed (camera_id={self._masked_camera_id})',
                self._last_read_fail_log_ts,
                _READ_FAIL_LOG_INTERVAL_SEC,
            )
            # 카메라 연결이 끊긴 경우 캡처를 멈추고 재연결로 넘어간다.
            if not self._camera.is_opened:
                self._handle_disconnect()
            return

        if not self._publishing:
            return

        try:
            ros_image = self._bridge.cv2_to_imgmsg(frame, encoding=self._encoding)
        except Exception as exc:
            self._last_convert_fail_log_ts = log_periodic(
                self.get_logger().error,
                f'cv_bridge conversion failed (encoding={self._encoding}): {exc}',
                self._last_convert_fail_log_ts,
                _CONVERT_FAIL_LOG_INTERVAL_SEC,
            )
            return

        # image 와 camera_info 를 동일한 stamp / frame_id 로 발행
        ros_image.header.stamp = stamp
        ros_image.header.frame_id = self._frame_id
        self._image_pub.publish(ros_image)

        camera_info = self._create_camera_info_msg()
        if camera_info is not None:
            camera_info.header.stamp = stamp
            self._camera_info_pub.publish(camera_info)

    def _handle_disconnect(self) -> None:
        """캡처 중 카메라 연결이 끊긴 상황을 처리한다.

        캡처 타이머를 멈추고 자원을 해제한 뒤 재연결 타이머를 건다.
        ``_publishing`` 은 건드리지 않는다 — 세션은 여전히 ``IN_EPISODE`` 일 수
        있고, 재연결에 성공하면 그대로 발행이 재개되어야 한다.
        """
        self._pause_timer()
        self._camera.release()
        self._actual_resolution = None
        self._publish_status(_STATUS_DISCONNECTED)
        self.get_logger().error(f'camera connection lost (camera_id={self._masked_camera_id})')
        self._schedule_reconnect()

    # -- CameraInfo / status 헬퍼 -----------------------------------------

    def _create_camera_info_msg(self) -> Optional[CameraInfo]:
        """현재 해상도 기준의 기본 CameraInfo 메시지를 생성한다.

        실제 카메라 내부 파라미터(캘리브레이션) 는 수행하지 않으며,
        관례에 따라 ``fx = fy = max(W, H)``, ``cx = W/2``, ``cy = H/2`` 를
        채운다. 해상도가 아직 확정되지 않았다면 ``None`` 을 반환한다.
        """
        resolution = self._actual_resolution
        if resolution is None:
            return None

        info = CameraInfo()
        info.width = resolution.width
        info.height = resolution.height
        info.header.frame_id = self._frame_id

        focal_length = float(max(resolution.width, resolution.height))
        cx = resolution.width / 2.0
        cy = resolution.height / 2.0

        info.k = [
            focal_length, 0.0, cx,
            0.0, focal_length, cy,
            0.0, 0.0, 1.0
        ]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.distortion_model = 'plumb_bob'
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            focal_length, 0.0, cx, 0.0,
            0.0, focal_length, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        return info

    def _publish_status(self, status: str) -> None:
        """카메라 상태 문자열을 status 토픽으로 발행한다.

        직전과 같은 상태면 발행하지 않는다 — 끊긴 카메라에서 캡처 주기마다
        DISCONNECTED 가 초당 fps 회 쏟아지는 것을 막는다.
        """
        if status == self._last_status:
            return

        try:
            msg = String()
            msg.data = status
            self._status_pub.publish(msg)
            self._last_status = status
            self.get_logger().debug(f'status published: {status}')
        except Exception as exc:
            self.get_logger().warning(f'failed to publish status: {exc}')

    # -- 라이프사이클 --------------------------------------------------------

    def destroy_node(self) -> None:
        """노드 종료 시 타이머를 취소하고 카메라를 해제한다."""
        self._cancel_timer()
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self.destroy_timer(self._retry_timer)
            self._retry_timer = None
        self._camera.release()
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    """콘솔 엔트리 포인트.

    ``SingleThreadedExecutor`` 로 노드를 spin 한다. SIGINT / Ctrl+C /
    ``ExternalShutdownException`` 경로에서도 ``destroy_node()`` 와
    ``rclpy.try_shutdown()`` 이 호출되도록 finally 블록으로 보호한다.
    """
    rclpy.init(args=args)

    from robot_control.logging_bridge import configure_logging_bridge
    configure_logging_bridge(package_logger_name='rdfp')

    node: Optional[RdfpCameraNode] = None
    try:
        node = RdfpCameraNode()
    except Exception as exc:
        print(f'[FATAL] RdfpCameraNode init failed: {exc}', file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(1)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
