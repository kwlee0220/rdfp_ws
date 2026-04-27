#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from ..ros2_utils import get_optional_parameter, get_parameter, parse_bool, parse_stripped_str
from ..ros2_utils import SENSOR_QOS, SYSTEM_QOS
from ..types import Fps, Resolution
from .camera_utils import parse_camera_id, mask_camera_id_for_log

try:
    from turbojpeg import TJPF_BGR, TurboJPEG
except ImportError:
    TJPF_BGR = None
    TurboJPEG = None

from ..logging_bridge import configure_logging_bridge
from .opencv_camera import OpenCvCamera


_DEFAULT_RAW_IMAGE_TOPIC = "~/image_raw"
_DEFAULT_COMPRESSED_IMAGE_TOPIC = "~/image_compressed"
_DEFAULT_CAMERA_INFO_TOPIC = "~/camera_info"
_DEFAULT_CAMERA_STATUS_TOPIC = "~/camera_status"


class CameraNode(Node):
    """OpenCV 기반 카메라 이미지를 ROS2 토픽으로 발행하는 노드.

    opencv_camera.py를 활용하여 카메라에서 이미지를 읽고,
    sensor_msgs/Image 메시지로 변환하여 ROS2 토픽으로 발행한다.
    """

    def __init__(self) -> None:
        """CameraNode를 초기화한다."""
        super().__init__('camera_node')

        self.get_logger().info("Initializing CameraNode")

        # 멤버 변수 초기화
        self._camera: OpenCvCamera | None = None
        self._bridge = CvBridge()
        self._timer = None
        self._jpeg_encoder = None
        self._jpeg_pixel_format: int | None = None

        # QoS 프로파일 설정
        self._sensor_qos = SENSOR_QOS
        self._system_qos = SYSTEM_QOS

        # 파라미터 선언 및 초기화
        self._declare_and_validate_parameters()

        # 카메라 초기화
        self._initialize_camera()

        # ROS2 퍼블리셔 초기화
        self._initialize_publishers()

        # 타이머 시작
        self._start_publishing_timer()

        self.get_logger().info("CameraNode initialization completed")

    def _cleanup(self) -> None:
        """리소스 정리 및 카메라 해제."""
        self.get_logger().info("Cleaning up resources")

        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

        if self._camera is not None:
            self._camera.release()
            self._camera = None

        self._jpeg_encoder = None
        self._jpeg_pixel_format = None

        self.get_logger().info("Resource cleanup completed")

    def _declare_and_validate_parameters(self) -> None:
        """ROS2 파라미터들을 선언하고 유효성을 검사한다."""
        self.get_logger().info("Declaring and validating parameters")

        # 기본 이미지 토픽은 파라미터가 아닌 remap 대상으로 고정한다.
        self._image_topic = _DEFAULT_RAW_IMAGE_TOPIC
        self._camera_info_topic = _DEFAULT_CAMERA_INFO_TOPIC
        self._status_topic = _DEFAULT_CAMERA_STATUS_TOPIC

        # 선택적 파라미터 선언
        self.declare_parameter('camera_id', value='0', descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('fps', value=None, descriptor=ParameterDescriptor(dynamic_typing=True),)
        self.declare_parameter('resolution', value=None, descriptor=ParameterDescriptor(dynamic_typing=True),)
        self.declare_parameter('compress_image', False)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('encoding', 'bgr8')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        # 파라미터 값 가져오기 및 유효성 검사
        self._camera_id = get_parameter(self, 'camera_id', parse_camera_id)
        self._fps = get_optional_parameter(self, 'fps', Fps.parse)
        self._resolution = get_optional_parameter(self, 'resolution', Resolution.parse)
        self._compress_image = get_parameter(self, 'compress_image', parse_bool)
        if self._compress_image:
            self._image_topic = _DEFAULT_COMPRESSED_IMAGE_TOPIC
        self._frame_id = get_parameter(self, 'frame_id', parse_stripped_str)
        self._encoding = get_parameter(self, 'encoding', parse_stripped_str)
        self._use_sim_time = get_parameter(self, 'use_sim_time', parse_bool)

        masked_camera_id = mask_camera_id_for_log(self._camera_id)
        self.get_logger().info(f"Parameters validated: camera_id={masked_camera_id}, "
                              f"image_topic='{self._image_topic}', "
                              f"fps={self._fps}, "
                              f"resolution={self._resolution}, "
                              f"compress_image={self._compress_image}, "
                              f"frame_id='{self._frame_id}', "
                              f"camera_info_topic='{self._camera_info_topic}', "
                              f"status_topic='{self._status_topic}'")

    def _initialize_camera(self) -> None:
        """opencv_camera.py를 활용하여 카메라를 초기화한다."""
        self.get_logger().info("Initializing camera")

        try:
            # OpenCvCamera 객체 생성
            self._camera = OpenCvCamera(self._camera_id, resolution=self._resolution, fps=self._fps)

            # 카메라 연결 시도
            actual_resolution, actual_fps = self._camera.open()

            # 실제 설정값 로깅
            if self._resolution is not None and actual_resolution != self._resolution:
                self.get_logger().warning(
                    f"Requested resolution {self._resolution} differs from actual resolution {actual_resolution}"
                )

            fps_tolerance = 0.5
            if self._fps is not None and abs(actual_fps - self._fps) > fps_tolerance:
                self.get_logger().warning(
                    f"Requested FPS {self._fps} differs from actual FPS {actual_fps}"
                )

            # 실제 FPS에 따른 타이머 주기 재조정
            if actual_fps > 0:
                self._timer_period = 1.0 / actual_fps
                self._actual_fps = actual_fps
            else:
                fallback_fps = self._fps if self._fps is not None else 30.0
                self._timer_period = 1.0 / fallback_fps
                self._actual_fps = fallback_fps

            self._actual_resolution = actual_resolution

            self.get_logger().info(
                f"Camera initialized successfully: actual_resolution={actual_resolution}, "
                f"actual_fps={actual_fps}"
            )

        except Exception as e:
            error_msg = f"Camera initialization failed: {e}"
            self.get_logger().error(error_msg)
            self._cleanup()
            raise RuntimeError(error_msg)

    def _initialize_publishers(self) -> None:
        """ROS2 퍼블리셔들을 초기화한다."""
        self.get_logger().info("Initializing publishers")

        try:
            # 이미지 토픽 퍼블리셔
            if self._compress_image:
                if TurboJPEG is None or TJPF_BGR is None:
                    raise RuntimeError(
                        "compress_image=true requires PyTurboJPEG (pip install PyTurboJPEG)"
                    )
                self._jpeg_encoder = TurboJPEG()
                self._jpeg_pixel_format = int(TJPF_BGR)
                self._image_pub = self.create_publisher(
                    CompressedImage,
                    self._image_topic,
                    qos_profile=self._sensor_qos
                )
                image_type = "sensor_msgs/CompressedImage(jpeg)"
            else:
                self._image_pub = self.create_publisher(Image, self._image_topic, qos_profile=self._sensor_qos)
                image_type = "sensor_msgs/Image"

            # 카메라 정보 토픽 퍼블리셔 (파라미터 또는 자동 계산된 값 사용)
            self._camera_info_pub = self.create_publisher(CameraInfo, self._camera_info_topic,
                                                          qos_profile=self._sensor_qos)

            # 상태 토픽 퍼블리셔
            self._status_pub = self.create_publisher(String, self._status_topic, qos_profile=self._system_qos)

            self.get_logger().info(
                f"Publishers initialized: image='{self._image_topic}' ({image_type}), "
                f"camera_info='{self._camera_info_topic}', status='{self._status_topic}'"
            )

        except Exception as e:
            error_msg = f"Publisher initialization failed: {e}"
            self.get_logger().error(error_msg)
            self._cleanup()
            raise RuntimeError(error_msg)

    def _create_camera_info_msg(self) -> CameraInfo:
        """CameraInfo 메시지를 생성한다."""
        camera_info = CameraInfo()

        # 기본 정보 설정
        camera_info.width = self._actual_resolution.width
        camera_info.height = self._actual_resolution.height

        # 프레임 ID 설정
        camera_info.header.frame_id = self._frame_id

        # 기본 카메라 매트릭스 (캘리브레이션되지 않은 상태)
        # 관례: fx = fy = max(width, height), cx = width/2, cy = height/2
        focal_length = float(max(self._actual_resolution.width, self._actual_resolution.height))
        cx = self._actual_resolution.width / 2.0
        cy = self._actual_resolution.height / 2.0

        # K 매트릭스: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        camera_info.k = [
            focal_length, 0.0, cx,
            0.0, focal_length, cy,
            0.0, 0.0, 1.0
        ]

        # 왜곡 계수 (기본값: 없음)
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.distortion_model = "plumb_bob"

        # R 매트릭스: 단위 행렬
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # P 매트릭스: [fx, 0, cx, Tx, 0, fy, cy, Ty, 0, 0, 1, 0]
        camera_info.p = [
            focal_length, 0.0, cx, 0.0,
            0.0, focal_length, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        return camera_info

    def _start_publishing_timer(self) -> None:
        """이미지 발행을 위한 타이머를 시작한다."""
        self.get_logger().info(f"Starting timer with period {self._timer_period:.3f}s (FPS: {self._actual_fps})")

        self._timer = self.create_timer(self._timer_period, self._timer_callback)

        # 초기 상태 발행
        self._publish_status("CONNECTED")

    def _timer_callback(self) -> None:
        """타이머 콜백: 카메라에서 이미지를 읽고 ROS2 토픽으로 발행한다."""
        try:
            camera = self._camera
            if camera is None:
                self.get_logger().error("Camera is not initialized")
                self._publish_status("DISCONNECTED")
                return

            # 카메라에서 프레임 읽기
            frame = camera.read()
            if frame is None:
                self.get_logger().warning("Failed to read frame from camera")

                # 카메라 연결 상태 확인
                if not camera.is_opened:
                    self.get_logger().error("Camera connection lost")
                    self._publish_status("DISCONNECTED")
                    self._handle_camera_error()
                    return

                return

            # 현재 시간 가져오기
            current_time = self.get_clock().now()

            # OpenCV 이미지를 ROS Image 메시지로 변환
            try:
                if self._compress_image:
                    if self._jpeg_encoder is None or self._jpeg_pixel_format is None:
                        self.get_logger().error("JPEG encoder is not initialized")
                        return

                    compressed_image = CompressedImage()
                    compressed_image.header.stamp = current_time.to_msg()
                    compressed_image.header.frame_id = self._frame_id
                    compressed_image.format = 'jpeg'
                    compressed_image.data = self._jpeg_encoder.encode(
                        frame,
                        pixel_format=self._jpeg_pixel_format,
                    )
                    self._image_pub.publish(compressed_image)
                else:
                    ros_image = self._bridge.cv2_to_imgmsg(frame, encoding=self._encoding)
                    ros_image.header.stamp = current_time.to_msg()
                    ros_image.header.frame_id = self._frame_id

                    # 이미지 발행
                    self._image_pub.publish(ros_image)

            except Exception as e:
                self.get_logger().warning(f"Failed to convert or publish image: {e}")
                return

            # CameraInfo 메시지 생성 및 발행
            try:
                camera_info = self._create_camera_info_msg()
                camera_info.header.stamp = current_time.to_msg()
                self._camera_info_pub.publish(camera_info)

            except Exception as e:
                self.get_logger().warning(f"Failed to publish camera info: {e}")

        except Exception as e:
            self.get_logger().error(f"Timer callback error: {e}")
            self._handle_camera_error()

    def _publish_status(self, status: str) -> None:
        """상태 토픽에 상태 메시지를 발행한다."""
        try:
            status_msg = String()
            status_msg.data = status
            self._status_pub.publish(status_msg)
            self.get_logger().debug(f"Status published: {status}")
        except Exception as e:
            self.get_logger().warning(f"Failed to publish status: {e}")

    def _handle_camera_error(self) -> None:
        """카메라 에러 발생 시 처리한다."""
        self.get_logger().error("Handling camera error - shutting down node")
        self._publish_status("ERROR")
        self._cleanup()
        raise SystemExit(1)


def main(args=None) -> None:
    """OpenCV 카메라 노드의 메인 함수."""
    rclpy.init(args=args)
    logger = get_logger('camera_node.main')
    node: CameraNode | None = None

    # Python logging(logger="rdfp.*") 출력을 ROS2 logger로 브리지한다.
    configure_logging_bridge(package_logger_name='rdfp')

    try:
        # 노드 생성
        node = CameraNode()

        # ROS2 스핀
        rclpy.spin(node)

    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received, shutting down...")
    except Exception as e:
        logger.error(f"Fatal error: {e}")
    finally:
        # 정리 작업
        if node is not None:
            try:
                node._cleanup()
            except Exception:
                pass

            try:
                node.destroy_node()
            except Exception:
                pass

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()