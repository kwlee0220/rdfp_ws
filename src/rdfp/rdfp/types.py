#!/usr/bin/env python3
"""rdfp 패키지 공용 타입 정의."""

from __future__ import annotations

from typing import Any, NamedTuple

from dataclasses import dataclass
from array import array

import numpy as np

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image


class Resolution(NamedTuple):
    """프레임 해상도 (width, height) 순서.

    OpenCV `VideoWriter` / `cv2.resize` 와 동일한 `(width, height)` 순서를
    따르며, numpy shape `(height, width)` 과 반대임에 유의한다. 값 검증은
    이 타입을 소비하는 쪽에서 수행한다.
    """

    width: int
    height: int

    @classmethod
    def parse(cls, value: Any, name: str) -> "Resolution":
        """값을 `Resolution` 으로 파싱한다.

        Args:
            value: `"WIDTHxHEIGHT"` 문자열, `(width, height)` 튜플 또는
                기존 `Resolution` 인스턴스.
            name: 오류 메시지에 사용할 파라미터 이름.

        Returns:
            파싱된 `Resolution` 인스턴스.

        Raises:
            ValueError: 형식이 잘못되었거나 0 이하의 값이 포함된 경우.
        """
        if isinstance(value, Resolution):
            if value.width <= 0 or value.height <= 0:
                raise ValueError(
                    f"{name} width and height must be positive integers: {value}"
                )
            return value
        if isinstance(value, tuple):
            if len(value) != 2:
                raise ValueError(f"{name} must be (width, height) tuple: {value}")
            width, height = value
            if (
                not isinstance(width, int) or isinstance(width, bool)
                or not isinstance(height, int) or isinstance(height, bool)
                or width <= 0 or height <= 0
            ):
                raise ValueError(
                    f"{name} width and height must be positive integers: {value}"
                )
            return cls(width, height)
        if isinstance(value, str):
            try:
                width, height = map(int, value.lower().split("x"))
                if width <= 0 or height <= 0:
                    raise ValueError
            except (ValueError, AttributeError):
                raise ValueError(
                    f"{name} must be like 1280x720 with positive integers"
                )
            return cls(width, height)
        raise ValueError(
            f"{name} must be a string like 1280x720 or a (width, height) tuple"
        )

    def __repr__(self) -> str:
        return f'{self.width}x{self.height}'


class Fps(float):
    """0보다 큰 초당 프레임 수(FPS) 타입.

    `float`의 서브클래스이므로 모든 float 연산이 그대로 동작한다.

    Examples:
        >>> fps = Fps(30.0)
        >>> fps * 2
        60.0
        >>> Fps(-1)
        ValueError: fps must be > 0, got -1
    """

    def __new__(cls, value: Any, name: str = "fps") -> "Fps":
        """FPS 값을 생성한다.

        Args:
            value: 0보다 큰 실수로 변환 가능한 값.
            name: 예외 메시지에 사용할 파라미터 이름.

        Returns:
            검증된 `Fps` 인스턴스.

        Raises:
            ValueError: 변환 불가능하거나 0 이하인 경우.
        """
        try:
            v = float(value)
        except (TypeError, ValueError):
            raise ValueError(f"{name} must be a number, got {value!r}")
        if v <= 0:
            raise ValueError(f"{name} must be > 0, got {value}")
        return super().__new__(cls, v)
    
    @staticmethod
    def parse(value: Any, name: str = "fps") -> "Fps":
        """값을 `Fps` 로 파싱한다.

        Args:
            value: 0보다 큰 실수로 변환 가능한 값.
            name: 예외 메시지에 사용할 파라미터 이름.

        Returns:
            검증된 `Fps` 인스턴스.

        Raises:
            ValueError: 변환 불가능하거나 0 이하인 경우.
        """
        return Fps(value, name)
    
    def to_int(self) -> int:
        """FPS 값을 가장 가까운 정수로 반환한다."""
        return round(self)


@dataclass(slots=True,kw_only=True)
class ImageMetadata:
    frame_id: str
    resolution: Resolution
    encoding: str
    is_bigendian: int
    step: int


def to_ros_image_msg(metadata: ImageMetadata, stamp: Time,
                     pixels: np.ndarray|array) -> Image:
    """메타데이터와 픽셀 데이터를 ROS `sensor_msgs/Image` 메시지로 합성한다.

    Args:
        metadata: 이미지 메타데이터.
        stamp: 메시지 헤더에 사용할 타임스탬프.
        pixels: `(height, width, 3)` shape, `uint8` dtype 의 BGR8 픽셀 데이터.
                OpenCV `VideoWriter` 와 동일한 형식이다.

    Returns:
        ROS `sensor_msgs/Image` 메시지.

    Note:
        ``msg.data = bytes(...)`` 직접 대입은 rclpy 의 uint8[] setter 가 element
        검증 fallback 경로로 떨어져 1MB 당 수십 ms 가 소요된다. 본 함수는
        ``array.array('B', ...)`` 로 한 번 감싸서 setter 의 fast path (typecode
        만 확인) 를 강제 — 540p 1.5MB 기준 ~100ms → ~5ms 로 ~20x 단축.
        호출자가 이미 ``array.array('B', ...)`` 를 넘겨주면 추가 wrap 비용도
        없도록 단락한다.
    """
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = metadata.frame_id
    msg.height = metadata.resolution.height
    msg.width = metadata.resolution.width
    msg.encoding = metadata.encoding
    msg.is_bigendian = metadata.is_bigendian
    msg.step = metadata.step
    if isinstance(pixels, array) and pixels.typecode == 'B':
        # 이미 setter fast path 에 맞는 형태 — 그대로 대입.
        msg.data = pixels
    else:
        msg.data = array('B', pixels.tobytes())
    return msg


def split_ros_image_msg(msg: Image) -> tuple[ImageMetadata, array]:
    """ROS `sensor_msgs/Image` 메시지에서 메타데이터와 픽셀 데이터를 추출한다."""
    metadata = ImageMetadata(
        frame_id=msg.header.frame_id,
        resolution=Resolution(msg.width, msg.height),
        encoding=msg.encoding,
        is_bigendian=msg.is_bigendian,
        step=msg.step,
    )
    return metadata, msg.data


class InvalidFrameError(ValueError):
    """입력 프레임이 recorder/sink 의 설정(shape, dtype, encoding 등)과 불일치할 때 발생."""


__all__ = [
    "Fps", "Resolution", "ImageMetadata",
    "to_ros_image_msg", "split_ros_image_msg",
    "InvalidFrameError",
]