#!/usr/bin/env python3

"""ImageViewerNode 모듈.

ROS2 이미지 토픽을 OpenCV 윈도우에 표시하는 단순 뷰어 노드.
``resolution`` 파라미터가 지정되면 해당 크기로 resize 하여 표시하고,
미지정 시 첫 수신 이미지의 크기를 그대로 사용한다.
"""

from __future__ import annotations

from typing import Any, Optional

import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from ..ros2_utils import get_optional_parameter, log_periodic
from ..types import Resolution


_DEFAULT_INPUT_TOPIC = 'image'
_DEFAULT_WINDOW_NAME = 'image_viewer'
# GUI 런타임 에러(cv2.error) 로그 rate-limit 간격 (초)
_DISPLAY_FAIL_LOG_INTERVAL_SEC = 5.0


class ImageViewerNode(Node):
    """이미지 토픽을 구독하여 OpenCV 윈도우에 표시하는 노드.

    파라미터:
        resolution (optional): 표시할 이미지 크기 ``WIDTHxHEIGHT``. 미지정 시
            첫 수신 이미지의 크기를 사용하며, 입력 크기와 다르면 resize 한다.
    """

    def __init__(self, node_name: str = 'image_viewer_node',
                 window_name: str = _DEFAULT_WINDOW_NAME,
                 **node_kwargs: Any) -> None:
        """노드를 초기화한다.

        Args:
            node_name: ROS2 노드 이름. 서브클래스에서 다른 이름을 지정할 수 있다.
            window_name: OpenCV 윈도우 이름. 서브클래스에서 재정의 가능.
            **node_kwargs: ``rclpy.node.Node.__init__`` 에 전달되는 키워드 인자.
                테스트에서 ``parameter_overrides`` 주입 용도로 사용한다.
        """
        super().__init__(node_name, **node_kwargs)

        # 파라미터 선언 및 로드
        self.declare_parameter('resolution', value=None, descriptor=ParameterDescriptor(dynamic_typing=True))
        self._resolution: Optional[Resolution] = get_optional_parameter(self, 'resolution', Resolution.parse)

        # 내부 상태
        self._bridge = CvBridge()
        self._window_name = window_name
        self._window_created: bool = False
        # 첫 입력 이미지를 기준으로 결정될 표시 해상도
        self._display_resolution: Optional[Resolution] = self._resolution
        # 런타임 GUI 에러 로그 rate-limit 타임스탬프
        self._last_display_fail_log_ts: float = 0.0
        # cv_bridge 변환에 실패한 인코딩을 인코딩별 1회만 로그하기 위한 기록
        self._cv_bridge_fail_logged: set[str] = set()

        # GUI 백엔드 사전 점검 (headless/X 미설정 환경에서 fail-fast)
        self._create_window()

        # 이미지 구독자
        self._image_sub = self.create_subscription(Image, _DEFAULT_INPUT_TOPIC, self._on_image,
                                                   qos_profile_sensor_data,)

        self.get_logger().info(f'ImageViewerNode initialized: topic={_DEFAULT_INPUT_TOPIC} '
                               f'resolution={self._resolution}')

    def _create_window(self) -> None:
        """OpenCV 윈도우를 생성하고 GUI 백엔드를 사전 점검한다.

        headless/X 미설정 환경 등에서는 ``cv2.error`` 가 발생하므로, 기동 시점에
        ``waitKey`` 까지 호출하여 이벤트 루프 동작을 확인하고 실패 시
        ``RuntimeError`` 로 변환해 fail-fast 한다.

        Raises:
            RuntimeError: GUI 백엔드 초기화에 실패한 경우.
        """
        try:
            cv2.namedWindow(self._window_name, cv2.WINDOW_AUTOSIZE)
            cv2.waitKey(1)
        except cv2.error as exc:
            raise RuntimeError(f'OpenCV GUI backend is not available '
                               f'(headless environment or missing display): {exc}') from exc
        self._window_created = True

    def _on_image(self, msg: Image) -> None:
        """이미지 토픽 콜백: 메시지를 bgr8 로 변환하고 화면에 표시한다."""
        try:
            # cv_bridge 가 rgb8/mono8 등 일반적인 인코딩을 bgr8 로 자동 변환한다.
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            # 동일 publisher 의 encoding 은 대체로 고정이므로 encoding 별 1회만 경고한다.
            encoding = msg.encoding
            if encoding not in self._cv_bridge_fail_logged:
                self._cv_bridge_fail_logged.add(encoding)
                self.get_logger().error(
                    f'cv_bridge conversion failed (encoding={encoding!r}): {exc}. '
                    f'Frames with this encoding will be ignored.'
                )
            return

        # 표시 해상도 결정: 파라미터로 지정되지 않았다면 첫 입력 이미지 크기 사용
        if self._display_resolution is None:
            height, width = frame.shape[:2]
            self._display_resolution = Resolution(width, height)
            self.get_logger().info(f'Display resolution set from first frame: {self._display_resolution}')

        # resize / imshow / waitKey 는 X/Wayland 연결 끊김 등으로 런타임 중 실패할 수 있다.
        # 예외가 구독 콜백 밖으로 전파되면 노드가 비정상 종료되므로 여기서 방어한다.
        try:
            height, width = frame.shape[:2]
            target = self._display_resolution
            if width != target.width or height != target.height:
                frame = cv2.resize(frame, (target.width, target.height), interpolation=cv2.INTER_AREA)
            # 서브클래스 확장 지점: 프레임에 오버레이 등을 덧입힌 결과를 표시한다.
            frame = self._decorate_frame(frame)
            cv2.imshow(self._window_name, frame)
            # GUI 이벤트 루프 처리 (윈도우가 멈추지 않도록 필수)
            cv2.waitKey(1)
        except cv2.error as exc:
            self._last_display_fail_log_ts = log_periodic(self.get_logger().warning,
                f'OpenCV HighGUI failure (display lost or backend error): {exc}',
                self._last_display_fail_log_ts,
                _DISPLAY_FAIL_LOG_INTERVAL_SEC,
            )

    def _decorate_frame(self, frame: np.ndarray) -> np.ndarray:
        """표시 직전 프레임을 가공하기 위한 서브클래스 확장 훅.

        기본 구현은 원본 프레임을 그대로 반환한다. 서브클래스는 텍스트
        오버레이, 도형 그리기 등 추가 렌더링을 수행한 결과 프레임을
        반환하면 된다.

        Args:
            frame: resize 가 적용된 ``bgr8`` 프레임.

        Returns:
            화면에 표시할 프레임. 서브클래스가 수정하지 않는다면 입력과 동일.
        """
        return frame

    def destroy_node(self) -> None:
        """노드 종료 시 OpenCV 윈도우를 정리한다.

        Qt/GTK 백엔드는 ``destroyWindow`` 만으로 close 이벤트가 처리되지 않을
        수 있어, ``waitKey`` 를 수회 호출하여 HighGUI 이벤트 루프를 펌프한다.
        """
        if self._window_created:
            try:
                # 자신이 생성한 윈도우만 닫는다. cv2.destroyAllWindows() 는 같은
                # 프로세스의 다른 뷰어까지 함께 닫으므로 호출하지 않는다.
                cv2.destroyWindow(self._window_name)
                for _ in range(4):
                    cv2.waitKey(1)
            except cv2.error:
                pass
            self._window_created = False
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    """콘솔 엔트리 포인트.

    ``SingleThreadedExecutor`` 로 spin 하며, SIGINT/Ctrl-C 및
    ``ExternalShutdownException`` 경로에서도 ``destroy_node()`` 와
    ``rclpy.try_shutdown()`` 이 실행되도록 finally 로 보호한다.
    """
    rclpy.init(args=args)

    from ..logging_bridge import configure_logging_bridge
    configure_logging_bridge(package_logger_name='rdfp')

    node: Optional[ImageViewerNode] = None
    try:
        node = ImageViewerNode()
    except Exception as exc:
        print(f'[FATAL] ImageViewerNode init failed: {exc}', file=sys.stderr)
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
