#!/usr/bin/env python3

"""`OpenCvCamera` — 특히 **파일 끝 되감기** 테스트.

`cv2.VideoCapture` 를 대역으로 세운다. 실제 mp4 를 쓰면 ffmpeg 이 있는 환경에서만
돌고, 무엇보다 **되감기 판단의 갈림길(프레임 수·set 실패)을 재현할 수 없다.**

되감기가 없으면 노드는 살아 있는 채 아무것도 못 내고 초당 fps 회의 경고만 남긴다 —
증상이 "카메라가 조용하다"뿐이라 원인이 안 보인다. mock 스택이 mp4 를 카메라로 쓰므로
(`config/image_pipeline.yaml`) **영상 길이가 곧 스택의 수명**이 된다.
"""

from __future__ import annotations

from unittest.mock import patch

import numpy as np
import pytest

pytest.importorskip('rclpy', reason='opencv_camera → ros2_utils → rclpy')

import cv2                                                          # noqa: E402

from robot_control.camera.opencv_camera import OpenCvCamera         # noqa: E402

FRAME = np.zeros((4, 4, 3), dtype=np.uint8)


class FakeCapture:
    """읽을 프레임 수가 정해진 가짜 VideoCapture.

    `frame_count` 가 0 이하면 **라이브 소스**(장치·RTSP)를 뜻한다 — 되감을 것이 없다.
    """

    def __init__(self, *, frames: int = 3, frame_count: float = 3.0,
                 opened: bool = True, seek_ok: bool = True) -> None:
        self._frames = frames
        self._remaining = frames
        self._frame_count = frame_count
        self._opened = opened
        self._seek_ok = seek_ok
        self.seek_calls = 0
        self.released = False

    def isOpened(self) -> bool:                                     # noqa: N802
        return self._opened

    def read(self):
        if self._remaining <= 0:
            return False, None
        self._remaining -= 1
        return True, FRAME.copy()

    def get(self, prop):
        if prop == cv2.CAP_PROP_FRAME_COUNT:
            return self._frame_count
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            return 4.0
        if prop == cv2.CAP_PROP_FRAME_HEIGHT:
            return 4.0
        if prop == cv2.CAP_PROP_FPS:
            return 10.0
        return 0.0

    def set(self, prop, value):
        if prop == cv2.CAP_PROP_POS_FRAMES:
            self.seek_calls += 1
            if self._seek_ok:
                self._remaining = self._frames
            return self._seek_ok
        return True

    def release(self) -> None:
        self.released = True


def _opened_camera(cap: FakeCapture, camera_id: str = '/data/clip.mp4') -> OpenCvCamera:
    camera = OpenCvCamera(camera_id)
    with patch.object(cv2, 'VideoCapture', return_value=cap):
        camera.open()
    return camera


def _read_n(camera: OpenCvCamera, n: int) -> tuple[int, int]:
    ok = fail = 0
    for _ in range(n):
        if camera.read() is None:
            fail += 1
        else:
            ok += 1
    return ok, fail


# ---------- 파일 끝 되감기 ----------

def test_file_loops_instead_of_going_silent():
    """**핵심.** 3프레임짜리 파일을 10번 읽어도 한 번도 실패하지 않아야 한다."""
    cap = FakeCapture(frames=3, frame_count=3.0)
    camera = _opened_camera(cap)

    ok, fail = _read_n(camera, 10)

    assert (ok, fail) == (10, 0)
    assert cap.seek_calls == 3, '3프레임마다 한 번씩 되감는다'


def test_rewind_is_not_attempted_before_any_frame_was_read():
    """**한 프레임도 못 읽은 소스는 되감지 않는다.**

    깨진 파일에서 되감기와 실패를 무한히 반복하게 되고, 그동안 진짜 원인(파일이
    깨졌다)은 로그에 안 나온다.
    """
    cap = FakeCapture(frames=0, frame_count=99.0)
    camera = _opened_camera(cap)

    assert camera.read() is None
    assert cap.seek_calls == 0


def test_live_source_is_never_rewound():
    """장치·RTSP 는 프레임 수가 0 이하다 — 되감을 것이 없다."""
    cap = FakeCapture(frames=2, frame_count=0.0)
    camera = _opened_camera(cap, camera_id='rtsp://cam.local/stream')

    ok, fail = _read_n(camera, 4)

    assert (ok, fail) == (2, 2), '끊긴 라이브 소스는 실패로 보고해야 한다'
    assert cap.seek_calls == 0


def test_seek_failure_falls_back_to_reporting_failure():
    """되감기가 실패하면 성공한 척하지 않는다 — 한 번만 시도하고 실패로 넘긴다."""
    cap = FakeCapture(frames=1, frame_count=1.0, seek_ok=False)
    camera = _opened_camera(cap)

    assert camera.read() is not None
    assert camera.read() is None
    assert cap.seek_calls == 1


def test_rewind_survives_repeated_cycles():
    """되감기 뒤에도 계속 돌아야 한다 — 한 바퀴만 되고 마는 구현을 잡는다."""
    cap = FakeCapture(frames=2, frame_count=2.0)
    camera = _opened_camera(cap)

    ok, fail = _read_n(camera, 20)

    assert (ok, fail) == (20, 0)
    assert cap.seek_calls == 9


# ---------- 열기 / 상태 ----------

def test_open_failure_raises():
    """열지 못하면 조용히 넘어가지 않는다."""
    camera = OpenCvCamera('/data/missing.mp4')
    with patch.object(cv2, 'VideoCapture', return_value=FakeCapture(opened=False)):
        with pytest.raises(RuntimeError, match='Failed to open camera'):
            camera.open()


def test_open_twice_is_rejected():
    cap = FakeCapture()
    camera = _opened_camera(cap)
    with patch.object(cv2, 'VideoCapture', return_value=FakeCapture()):
        with pytest.raises(RuntimeError, match='already opened'):
            camera.open()


def test_read_before_open_returns_none():
    """열지 않은 카메라를 읽어도 예외가 아니라 None 이다 (타이머 콜백에서 불린다)."""
    assert OpenCvCamera('/data/clip.mp4').read() is None


def test_release_clears_the_handle():
    cap = FakeCapture()
    camera = _opened_camera(cap)
    camera.release()
    assert cap.released is True
    assert camera.is_opened is False


def test_release_is_idempotent():
    """정리 경로가 두 번 불려도 죽지 않아야 한다 (`_cleanup` + `__del__`)."""
    camera = _opened_camera(FakeCapture())
    camera.release()
    camera.release()


def test_is_opened_reflects_the_capture():
    cap = FakeCapture()
    camera = _opened_camera(cap)
    assert camera.is_opened is True
    cap._opened = False
    assert camera.is_opened is False


# ---------- 파라미터 검증 ----------

@pytest.mark.parametrize('bad', [-1, '', '   ', None])
def test_invalid_camera_id_is_rejected_at_construction(bad):
    """기동 단계에서 막는다 — 열어 본 뒤 실패하면 원인이 멀어진다."""
    with pytest.raises(ValueError):
        OpenCvCamera(bad)


def test_resolution_and_fps_are_applied_to_the_capture():
    cap = FakeCapture()
    camera = OpenCvCamera('/data/clip.mp4', resolution='4x4', fps=10.0)
    with patch.object(cv2, 'VideoCapture', return_value=cap):
        resolution, fps = camera.open()
    assert (resolution.width, resolution.height) == (4, 4)
    assert fps == 10.0


def test_actual_values_win_over_requested_ones():
    """카메라가 요청을 못 맞추면 **실제값**을 돌려줘야 한다 — 타이머 주기가 여기 걸린다."""
    cap = FakeCapture()
    camera = OpenCvCamera('/data/clip.mp4', resolution='640x480', fps=30.0)
    with patch.object(cv2, 'VideoCapture', return_value=cap):
        resolution, fps = camera.open()
    assert (resolution.width, resolution.height) == (4, 4), '대역은 4x4 를 보고한다'
    assert fps == 10.0
