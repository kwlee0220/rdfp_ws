#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

from abc import ABC, abstractmethod

from sensor_msgs.msg import Image

from ..types import ImageMetadata


class Ros2ImageSource(ABC):
    """ROS2 sensor_msgs/Image를 직접 생성하는 이미지 소스 추상 클래스.

    구체 구현체는 이미지 소스를 열고 프레임을 읽어 sensor_msgs/Image
    메시지로 반환해야 한다.
    """

    @abstractmethod
    def open(self) -> ImageMetadata:
        """이미지 소스 객체를 개방하고 메타데이터를 반환한다.

        Returns:
            ImageMetadata: 이미지 소스 메타데이터.

        Raises:
            RuntimeError: 이미지 소스 열기에 실패한 경우.
        """
        ...

    @property
    @abstractmethod
    def metadata(self) -> ImageMetadata:
        """이미지 소스 메타데이터를 반환한다.

        Returns:
            ImageMetadata: 이미지 소스 메타데이터.

        Raises:
            RuntimeError: 이미지 소스가 열려 있지 않거나 메타데이터를 가져올 수 없는 경우.
        """
        ...

    @abstractmethod
    def read(self) -> Optional[Image]:
        """이미지 소스에서 프레임을 읽어 sensor_msgs/Image 메시지로 반환한다.

        반환되는 메시지의 ``header.stamp`` 와 ``header.frame_id`` 는 구현체가
        적절히 채워야 한다.

        Returns:
            Optional[Image]: 성공 시 sensor_msgs/Image 메시지, 실패 시 None.
        """
        ...

    @abstractmethod
    def close(self) -> None:
        """이미지 소스 리소스를 해제한다.

        이미 닫혀 있는 상태에서 호출해도 예외를 발생시키지 않아야 한다.
        """
        ...

    @property
    @abstractmethod
    def is_opened(self) -> bool:
        """이미지 소스가 열려 있는지 여부.

        Returns:
            bool: 이미지 소스가 열려 있으면 True, 그렇지 않으면 False.
        """
        ...
