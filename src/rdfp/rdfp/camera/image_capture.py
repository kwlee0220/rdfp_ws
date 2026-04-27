#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
import cv2

from ..ros2_utils import (
    get_optional_parameter,
    get_parameter,
    log_periodic,
    parse_int,
    parse_str,
)
from ..types import Fps, Resolution
from .reconnecting_camera import ReconnectingCamera
from .camera_utils import parse_camera_id

DEFAULT_CAMERA_ID = 0
DEFAULT_FPS = 5
DEFAULT_TOPIC = '/stor/real/image/jpeg'
DEFAULT_FRAME_ID = 'camera'
DEFAULT_JPEG_QUALITY = 80  # 0~100
DEFAULT_QUEUE_DEPTH = 10
DEFAULT_RECONNECT_INTERVAL = 5.0    # 재연결 시도 간격 (5초)
DEFAULT_MAX_RECONNECT_ATTEMPTS = 0  # 최대 재연결 시도 횟수 (0: 재연결 시도하지 않음)
ERROR_LOG_INTERVAL_SEC = 5.0


class ImageCaptureNode(Node):
    """
    USB 카메라에서 직접 이미지를 캡처하고 JPEG으로 압축하여 발행하는 노드.
    재연결 로직은 ReconnectingCamera에 위임한다.
    """

    def __init__(self):
        super().__init__('image_capture')

        # 파라미터 선언
        self.declare_parameter('camera_id', DEFAULT_CAMERA_ID)
        self.declare_parameter('resolution')
        self.declare_parameter('fps', DEFAULT_FPS)
        self.declare_parameter('topic', DEFAULT_TOPIC)
        self.declare_parameter('frame_id', DEFAULT_FRAME_ID)
        self.declare_parameter('jpeg_quality', DEFAULT_JPEG_QUALITY)

        # 파라미터 가져오기
        self.camera_id = get_parameter(self, 'camera_id', parse_camera_id)
        self.resolution = get_optional_parameter(self, 'resolution', Resolution.parse)
        self.fps = get_parameter(self, 'fps', Fps, default=Fps(DEFAULT_FPS))
        self.topic = get_parameter(self, 'topic', parse_str, default=DEFAULT_TOPIC)
        self.frame_id = get_parameter(self, 'frame_id', parse_str, default=DEFAULT_FRAME_ID)
        self.jpeg_quality = get_parameter(self, 'jpeg_quality', parse_int,
                                          default=DEFAULT_JPEG_QUALITY)

        # jpeg_quality 범위 검사
        if self.jpeg_quality < 0 or self.jpeg_quality > 100:
            self.get_logger().error('jpeg_quality must be between 0 and 100.')
            raise ValueError('jpeg_quality must be between 0 and 100')

        # 카메라 초기화
        self._camera = ReconnectingCamera(
            self.camera_id,
            self.resolution,
            self.fps,
            reconnect_interval=DEFAULT_RECONNECT_INTERVAL,
            max_attempts=DEFAULT_MAX_RECONNECT_ATTEMPTS,
        )

        # JPEG 압축 이미지 발행자 생성
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=DEFAULT_QUEUE_DEPTH,
        )
        self.publisher = self.create_publisher(
            CompressedImage,
            self.topic,
            qos_profile
        )

        # 타이머 생성
        self.timer = self.create_timer(1.0 / float(self.fps), self.timer_callback)
        self._last_read_error_log_ts = 0.0
        self._last_encode_error_log_ts = 0.0

        self.get_logger().info(
            f'Started ImageCaptureNode: '
            f'camera_id={self.camera_id}, '
            f'resolution={self.resolution}, '
            f'fps={self.fps}, '
            f'topic={self.topic}, '
            f'jpeg_quality={self.jpeg_quality}'
        )

    def timer_callback(self):
        """카메라에서 이미지를 캡처하여 JPEG으로 압축하고 발행한다."""
        if not self._camera.is_available:
            return
        
        try:
            # 카메라에서 프레임 읽기 (timeout=0: 즉시 반환)
            frame = self._camera.read(timeout_sec=0)
            if frame is None:
                self._last_read_error_log_ts = log_periodic(
                    self.get_logger().error,
                    'Failed to read frame.',
                    self._last_read_error_log_ts,
                    ERROR_LOG_INTERVAL_SEC,
                )
                return

            # 이미지 크기 조정 (numpy shape 은 (H, W), resize 는 (W, H))
            if self.resolution is not None and frame.shape[:2] != (
                self.resolution.height, self.resolution.width
            ):
                frame = cv2.resize(frame, self.resolution)

            # 이미지를 JPEG으로 인코딩
            encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            success, jpeg_data = cv2.imencode('.jpg', frame, encode_params)

            if not success:
                self._last_encode_error_log_ts = log_periodic(
                    self.get_logger().error,
                    'JPEG encoding failed.',
                    self._last_encode_error_log_ts,
                    ERROR_LOG_INTERVAL_SEC,
                )
                return

            # CompressedImage 메시지 생성 및 발행
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.header.frame_id = self.frame_id
            compressed_msg.format = 'jpeg'
            compressed_msg.data = jpeg_data.tobytes()

            self.publisher.publish(compressed_msg)

        except ConnectionError:
            # 카메라가 CLOSED 상태 — 타이머를 중지하고 노드 종료
            self.timer.cancel()
            self.get_logger().error('Camera is closed. Shutting down.')
            raise SystemExit(1)
        except Exception as e:
            self.get_logger().error(f'Error during image processing: {e}')

    def destroy_node(self):
        """노드 종료 시 카메라 리소스 해제"""
        try:
            self._camera.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    image_capture_node = None
    try:
        image_capture_node = ImageCaptureNode()
        rclpy.spin(image_capture_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        get_logger('image_capture').error(f'Node exception: {e}')
    finally:
        if image_capture_node is not None:
            image_capture_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
