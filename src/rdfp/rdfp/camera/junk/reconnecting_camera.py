#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional, Union

import time
import enum
import logging
import threading

import numpy as np

from ...types import Fps, Resolution
from ..opencv_camera import OpenCvCamera

logger = logging.getLogger(__name__)


class Status(enum.Enum):
    CONNECTING = "CONNECTING"
    CONNECTED = "CONNECTED"
    CLOSED = "CLOSED"


class ReconnectingCamera:
    """재연결 기능을 갖춘 카메라 클래스.

    OpenCvCamera를 composition으로 사용하며, 연결 실패 시
    백그라운드 스레드에서 주기적으로 재연결을 시도한다.
    ROS 의존성 없이 순수 Python threading으로 구현되어 있다.

    상태 보호와 대기/알림에 threading.Condition 하나를 사용한다.
    - read() 호출자: 연결 상태 변경을 대기
    - 재연결 스레드: 다음 시도까지 간격 대기 + 중지 신호 수신
    - close(): notify_all()로 양쪽 모두 깨움
    """

    def __init__(self, camera_id: Union[int, str],
                 resolution: Optional[Resolution],
                 fps: Fps, reconnect_interval: float = 5.0,
                 max_attempts: int = 12) -> None:
        """ReconnectingCamera를 초기화하고 카메라 연결을 시도한다.

        카메라가 연결되지 않으면 CONNECTING 상태로 전이되고,
        reconnect_interval 주기로 최대 max_attempts까지
        재연결을 시도하는 스레드를 시작한 뒤 즉시 반환된다.

        Args:
            camera_id: 카메라 디바이스 ID
            resolution: `Resolution` 인스턴스 또는 None
            fps: 초당 프레임 수
            reconnect_interval: 재연결 시도 간격 (초)
            max_attempts: 연결이 실패했을 때 이후 최대 재연결 시도 횟수.
                            0인 경우는 재연결 시도하지 않음 (즉시 CLOSED 상태로 전이)
                            음수인 경우는 무제한 재연결 시도
        """
        if reconnect_interval <= 0:
            raise ValueError('reconnect_interval must be greater than 0.')
        
        self._device = OpenCvCamera(camera_id, resolution=resolution, fps=fps)
        self._reconnect_interval = reconnect_interval
        self._max_attempts = max_attempts

        self._cond = threading.Condition()
        self._reconnect_thread = None

        with self._cond:
            try:
                self._device.open()
                self._status = Status.CONNECTED
                logger.info('Camera connected (initial connection).')
            except RuntimeError:
                self._status = Status.CONNECTING
                logger.warning('Initial connection failed. Starting reconnect loop.')
                self._start_reconnect_loop_locked()

    @property
    def status(self) -> Status:
        """현재 상태를 반환한다."""
        with self._cond:
            return self._status

    @property
    def is_available(self) -> bool:
        """카메라가 현재 연결되어 있는지 여부를 반환한다."""
        return self.status == Status.CONNECTED

    def read(self, timeout_sec: Optional[float] = 0) -> Optional[np.ndarray]:
        """카메라에서 프레임을 읽어온다.

        Condition의 lock을 유지한 상태에서 상태 확인과 I/O를 함께 수행한다.
        CONNECTING 상태에서 대기가 필요하면 cond.wait()가
        lock 해제 → 알림 대기 → lock 재획득을 원자적으로 처리한다.

        Args:
            timeout_sec: 연결 대기 시간 (초).
                - 0: 연결되지 않은 경우 즉시 None 반환
                - None: 연결될 때까지 무한 대기
                - N (>0): 최대 N초 대기 후 TimeoutError

        Returns:
            numpy.ndarray 또는 None: 성공 시 프레임, 실패 시 None

        Raises:
            ConnectionError: CLOSED 상태에서 호출된 경우
            TimeoutError: timeout_sec 초과 시 (CONNECTING 상태)
        """
        started = time.monotonic()
        with self._cond:
            while True:
                if self._status == Status.CLOSED:
                    raise ConnectionError('Camera is closed.')

                if self._status == Status.CONNECTED:
                    frame = self._read_and_check_locked()
                    if frame is not None or self._device.is_opened:
                        return frame

                # CONNECTING 상태 (또는 read 중 연결 끊김)
                if timeout_sec == 0:
                    return None

                if timeout_sec is not None:
                    remains = timeout_sec - (time.monotonic() - started)
                    if remains <= 0:
                        raise TimeoutError(f'timeout_sec={timeout_sec}')
                else:
                    remains = None

                self._cond.wait(timeout=remains)

    def close(self) -> None:
        """카메라 리소스를 해제하고 CLOSED 상태로 전이한다.

        재연결 스레드가 실행 중이면 중지시키고,
        대기 중인 read() 호출을 해제한다.
        """
        with self._cond:
            self._status = Status.CLOSED
            self._cond.notify_all()

        if self._reconnect_thread is not None:
            self._reconnect_thread.join(timeout=self._reconnect_interval + 1.0)
            self._reconnect_thread = None

        self._device.release()
        logger.info('Camera released. Status is now CLOSED.')

    def _read_and_check_locked(self) -> Optional[np.ndarray]:
        """프레임을 읽고, 연결 끊김 감지 시 재연결 루프를 시작한다.

        만일 카메라가 일시적으로 연결이 끊긴 경우, read()가 None을 반환할 수 있다.
        호출자가 self._cond의 lock을 보유한 상태에서 호출해야 한다.
        lock을 유지하므로 상태 전이가 원자적으로 처리된다.
        """
        frame = self._device.read()
        if frame is not None:
            return frame
        
        if not self._device.is_opened:
            if self._max_attempts == 0:
                self._status = Status.CLOSED
                self._cond.notify_all()
                logger.error('Camera disconnected.')
                return None
            self._status = Status.CONNECTING
            logger.warning('Camera disconnected. Starting reconnect loop.')
            self._start_reconnect_loop_locked()
        return None

    def _start_reconnect_loop_locked(self) -> None:
        """재연결 스레드를 시작한다."""

        # 아직 종료되지 않은 reconnect_thread가 있으면 종료될 때까지 기다린다.
        while self._reconnect_thread is not None:
            self._cond.wait()

        self._reconnect_thread = threading.Thread(target=self._reconnect_loop, daemon=True,)
        self._reconnect_thread.start()

    def _reconnect_loop(self) -> None:
        """백그라운드에서 주기적으로 재연결을 시도한다.

        각 시도 사이에 cond.wait(interval)로 대기한다.
        close()가 호출되면 notify_all()로 즉시 깨어나고,
        상태가 CLOSED이면 루프를 종료한다.
        """
        attempts_remains = self._max_attempts

        try:
            while True:
                with self._cond:
                    if self._status == Status.CLOSED:
                        return

                try:
                    self._device.open()
                except RuntimeError:
                    pass
                else:
                    with self._cond:
                        # 카메라 오픈하는 과정에서 close()가 호출될 수 있으므로 CLOSED 상태인지 먼저 확인한다.
                        if self._status == Status.CLOSED:
                            self._device.release()
                            return
                        self._status = Status.CONNECTED
                        self._cond.notify_all()
                    logger.info('Camera reconnected successfully.')
                    return

                # 카메라 재연결이 실패한 경우
                if self._max_attempts > 0:
                    attempts_remains -= 1
                    if attempts_remains == 0:
                        with self._cond:
                            if self._status != Status.CLOSED:
                                self._status = Status.CLOSED
                                self._cond.notify_all()
                        logger.error('Failed to connect camera.')
                        return
                    logger.info(f'Reconnect attempt remains {attempts_remains}.')

                # 다음 시도까지 대기 (close() 시 notify_all()로 즉시 깨어남)
                with self._cond:
                    if self._status == Status.CLOSED:
                        return
                    self._cond.wait(timeout=self._reconnect_interval)
        finally:
            logger.info('Exiting reconnect loop.')
            with self._cond:
                self._reconnect_thread = None
                self._cond.notify_all()
