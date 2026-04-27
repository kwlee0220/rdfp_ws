#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import logging
import os

import cv2
import numpy as np

from ..ros2_utils import log_periodic
from ..types import Fps, Resolution
from .camera_utils import parse_camera_id

_READ_FAIL_LOG_INTERVAL_SEC = 5.0


class OpenCvCamera:
    """OpenCV VideoCapture를 사용한 카메라 제어 클래스.

    카메라 열기, 프레임 읽기, 리소스 해제를 담당한다.
    """

    def __init__(self, camera_id: int|str, *,
                 resolution: str | tuple[int, int] | Resolution | None = None,
                 fps: float | Fps | None = None,) -> None:
        """
        Args:
            camera_id: 카메라 디바이스 ID (음이 아닌 정수) 또는 영상 소스 경로 (비어있지 않은 문자열)
            resolution: `Resolution` 인스턴스, `(width, height)` 튜플 또는
                `"WIDTHxHEIGHT"` 형식의 문자열. 생략 시 카메라 기본 해상도 사용
            fps: 초당 프레임 수 (0보다 큰 수)
                생략 시 카메라 기본 FPS 사용

        Notes:
            resolution/fps를 생략한 경우 실제 적용된 값은 open()의 반환값
            (actual_resolution, actual_fps)으로 확인할 수 있다.

        Raises:
            ValueError: camera_id, resolution 또는 fps가 유효하지 않은 값인 경우
        """
        # 인스턴스 변수들을 먼저 초기화 (예외 발생 시 __del__ 안전성 확보)
        self._cap: Optional[cv2.VideoCapture] = None
        self._logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")

        # camera_id 유효성 검사
        camera_id = parse_camera_id(camera_id, "camera_id")

        # resolution 유효성 검사
        parsed_resolution: Optional[Resolution] = None
        if resolution is not None:
            parsed_resolution = Resolution.parse(resolution, "resolution")

        # fps 유효성 검사
        if fps is not None and not isinstance(fps, Fps):
            fps = Fps(fps, "fps")

        self._camera_id = camera_id
        self._resolution: Optional[Resolution] = parsed_resolution
        self._fps = fps
        self._last_read_fail_log_ts: float = 0.0

    @property
    def is_opened(self) -> bool:
        """카메라가 열려 있는지 여부.

        Returns:
            bool: 카메라가 열려 있으면 True, 그렇지 않으면 False
        """
        return self._cap is not None and self._cap.isOpened()

    @property
    def camera_id(self) -> int|str:
        """설정된 카메라 ID.

        Returns:
            int|str: 설정된 카메라 ID
        """
        return self._camera_id

    @property
    def resolution(self) -> Resolution | None:
        """요청한 해상도 또는 None.

        None인 경우 카메라 기본 해상도를 사용한다.
        실제 설정된 해상도는 open() 메서드의 반환값에서 확인할 수 있다.

        Returns:
            Resolution | None: 요청한 해상도 또는 None
        """
        return self._resolution

    @property
    def fps(self) -> float | None:
        """요청한 FPS 또는 None.

        None인 경우 카메라 기본 FPS를 사용한다.
        실제 설정된 FPS는 open() 메서드의 반환값에서 확인할 수 있다.

        Returns:
            float | None: 요청한 FPS 또는 None
        """
        return self._fps

    def open(self) -> tuple[Resolution, float]:
        """카메라를 열고 해상도/FPS를 설정한다.

        resolution/fps를 지정하지 않으면 카메라 기본 설정을 사용한다.

        요청한 해상도나 FPS가 카메라에서 지원되지 않을 경우, 카메라가 지원하는
        가장 가까운 값으로 자동 조정되며 warning 메시지가 로그에 출력된다.

        Returns:
            tuple[Resolution, float]: ``(actual_resolution, actual_fps)`` 튜플.

        Raises:
            RuntimeError: 카메라가 이미 열린 상태이거나 열기에 실패한 경우.
        """
        if self._cap is not None:
            if self._cap.isOpened():
                raise RuntimeError("Camera is already opened")
            else:
                self._cap.release()
                self._cap = None

        cap = cv2.VideoCapture(self._camera_id)
        if not cap.isOpened():
            cap.release()
            detail = _diagnose_open_failure(self._camera_id)
            raise RuntimeError(f"Failed to open camera (camera_id={self._camera_id}): {detail}")

        if self._resolution is not None:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._resolution.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._resolution.height)
        if self._fps is not None:
            cap.set(cv2.CAP_PROP_FPS, self._fps)

        # 실제 설정된 값 읽기
        actual_resolution = Resolution(
            int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        )
        actual_fps = float(cap.get(cv2.CAP_PROP_FPS))

        # 요청한 값과 실제 설정된 값 비교 (경고만 출력)
        if self._resolution is not None and actual_resolution != self._resolution:
            self._logger.warning(
                f"Requested resolution {self._resolution} differs from actual resolution {actual_resolution}"
            )

        fps_tolerance = 0.5
        if self._fps is not None and abs(actual_fps - self._fps) > fps_tolerance:
            self._logger.warning(f"Requested FPS {self._fps} differs from actual FPS {actual_fps}")

        self._cap = cap
        return (actual_resolution, actual_fps)

    def read(self) -> Optional[np.ndarray]:
        """카메라에서 프레임을 읽는다.

        Returns:
            성공 시 프레임 (numpy.ndarray), 실패 시 None
        """
        if self._cap is None or not self._cap.isOpened():
            self._logger.warning('read() called but camera is not opened')
            return None

        ret, frame = self._cap.read()
        if not ret or frame is None:
            self._last_read_fail_log_ts = log_periodic(
                self._logger.warning,
                f'Frame read failed (camera_id={self._camera_id})',
                self._last_read_fail_log_ts,
                _READ_FAIL_LOG_INTERVAL_SEC,
            )
            return None

        return frame

    def release(self) -> None:
        """카메라 리소스를 해제한다."""
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def __del__(self) -> None:
        """소멸자에서 리소스 해제를 보장한다.

        가비지 컬렉션 시 자동으로 호출되어 리소스 누수를 방지한다.
        """
        self.release()

    def __enter__(self) -> 'OpenCvCamera':
        """Context manager 진입: 카메라를 열고 self를 반환한다.

        Returns:
            OpenCvCamera: 열린 카메라 객체

        Raises:
            RuntimeError: 카메라 열기 실패 시
        """
        self.open()
        return self

    def __exit__(self, *_) -> None:
        """Context manager 종료: 카메라 리소스를 해제한다.

        Args:
            *_: 예외 정보 (exc_type, exc_val, exc_tb) - 사용하지 않음
        """
        self.release()


def _diagnose_open_failure(camera_id: int | str) -> str:
    """카메라 열기 실패 시 원인을 추정하여 진단 메시지를 반환한다."""
    if isinstance(camera_id, int):
        return (
            'device not connected, already in use by another process, '
            'or invalid device number'
        )

    if '://' in camera_id:
        return (
            'check network connectivity, credentials, or stream availability'
        )

    # 파일 경로로 간주
    if not os.path.exists(camera_id):
        return f'path does not exist: {camera_id}'
    if not os.access(camera_id, os.R_OK):
        return f'no read permission: {camera_id}'
    return f'file exists but failed to open: {camera_id}'
