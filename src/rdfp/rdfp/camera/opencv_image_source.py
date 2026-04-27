#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import logging

from cv_bridge import CvBridge
from rclpy.clock import Clock
from sensor_msgs.msg import Image

from ..types import Fps, Resolution
from .opencv_camera import OpenCvCamera
from .ros2_image_source import ImageMetadata, Ros2ImageSource


class OpenCvImageSource(Ros2ImageSource):
    """OpenCV 카메라에서 프레임을 읽어 sensor_msgs/Image 메시지로 반환하는 이미지 소스.

    내부적으로 ``OpenCvCamera`` 를 composition으로 사용하며, 읽은 프레임을
    ``cv_bridge`` 를 통해 ``sensor_msgs/Image`` 로 변환한다. 변환된 메시지의
    ``header.stamp`` 는 생성자에 전달된 ``Clock`` 으로부터 얻는다.
    """

    def __init__(self, camera_id: int | str, *,
                 clock: Clock,
                 resolution: str | tuple[int, int] | Resolution | None = None,
                 fps: float | Fps | None = None,
                 frame_id: str = 'camera_link') -> None:
        """
        Args:
            camera_id: 카메라 디바이스 ID (음이 아닌 정수) 또는 영상 소스 경로 (비어있지 않은 문자열).
            clock: ``header.stamp`` 생성을 위한 ROS2 Clock. 일반적으로 ``node.get_clock()`` 을 전달한다.
            resolution: ``Resolution`` 인스턴스, ``(width, height)`` 튜플 또는
                ``"WIDTHxHEIGHT"`` 형식의 문자열. 생략 시 카메라 기본 해상도 사용.
            fps: 초당 프레임 수 (0보다 큰 수). 생략 시 카메라 기본 FPS 사용.
            frame_id: 발행되는 Image 메시지의 ``header.frame_id`` 값.

        Raises:
            ValueError: camera_id, resolution 또는 fps가 유효하지 않은 값인 경우.
        """
        self._logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
        self._camera = OpenCvCamera(camera_id, resolution=resolution, fps=fps)
        self._clock = clock
        self._bridge = CvBridge()

        resol = Resolution.parse(resolution, "resolution") if resolution is not None \
                else Resolution(640, 480)  # OpenCV 기본 해상도 가정
        self._metadata: ImageMetadata = ImageMetadata(frame_id=frame_id,
                                                      resolution=resol,
                                                      encoding="bgr8",
                                                      is_bigendian=0,
                                                      step=resol.width * 3)

    @property
    def camera_id(self) -> int | str:
        """설정된 카메라 ID."""
        return self._camera.camera_id
    
    @property
    def metadata(self) -> ImageMetadata:
        """이미지 소스 메타데이터를 반환한다."""
        if not self._camera.is_opened:
            raise RuntimeError("Camera is not opened. Metadata is unavailable.")
        return self._metadata

    @property
    def is_opened(self) -> bool:
        """카메라가 열려 있는지 여부."""
        return self._camera.is_opened


    def open(self) -> ImageMetadata:
        """내부 카메라를 열고 메타데이터를 반환한다.

        Returns:
            ImageMetadata: 이미지 소스 메타데이터.

        Raises:
            RuntimeError: 이미지 소스 열기에 실패한 경우.
        """
        actual_resolution, actual_fps = self._camera.open()
        if actual_resolution != self._metadata.resolution:
            self._metadata.resolution = actual_resolution
            self._metadata.step = actual_resolution.width * 3  # encoding이 bgr8로 고정되어 있다고 가정
        self._logger.info(f"Camera opened: id={self._camera.camera_id} "
                          f"requested_resolution={self._metadata.resolution} "
                          f"actual_resolution={actual_resolution} "
                          f"requested_fps={self._camera._fps} "
                          f"actual_fps={actual_fps}")
        return self._metadata


    def read(self) -> Optional[Image]:
        """카메라에서 프레임을 읽어 sensor_msgs/Image 메시지로 변환하여 반환한다.

        ``header.stamp`` 는 생성자에 전달된 ``Clock`` 의 현재 시각으로,
        ``header.frame_id`` 는 ``frame_id`` 파라미터 값으로 채운다.

        Returns:
            Optional[Image]: 성공 시 sensor_msgs/Image 메시지, 실패 시 None.
        """
        frame = self._camera.read()
        if frame is None:
            return None

        try:
            ros_image = self._bridge.cv2_to_imgmsg(frame, encoding=self.metadata.encoding)
        except Exception as e:
            self._logger.warning(f"Failed to convert frame to sensor_msgs/Image: {e}")
            return None

        ros_image.header.stamp = self._clock.now().to_msg()
        ros_image.header.frame_id = self.metadata.frame_id
        return ros_image

    def close(self) -> None:
        """카메라 리소스를 해제한다.

        이미 닫혀 있는 상태에서 호출해도 예외를 발생시키지 않는다.
        """
        self._camera.release()

    def __enter__(self) -> 'OpenCvImageSource':
        """Context manager 진입: 카메라를 열고 self를 반환한다."""
        self.open()
        return self

    def __exit__(self, *_) -> None:
        """Context manager 종료: 카메라 리소스를 해제한다."""
        self.close()
