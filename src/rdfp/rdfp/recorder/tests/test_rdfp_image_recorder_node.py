#!/usr/bin/env python3

"""`RdfpImageRecorderNode` 의 **세션 경계** 테스트.

이 노드의 정확성은 「지금 녹화 중인가」가 아니라 **세션 메시지의 stamp** 로 정해진다.
프레임을 `pending_image_queue` 에 담아 두었다가 전이 시점에 stamp 로 가르므로,
도착 순서가 밀려도 에피소드 창이 정확하다. 여기서 지키는 것은 셋이다:

  ① 시작 stamp **이전** 프레임은 기록하지 않는다
  ② 시작 stamp 이후 프레임은 (늦게 도착해도) 기록한다
  ③ 정지 stamp **이후** 프레임은 기록하지 않는다

그리고 **정지 경로가 둘이라는 것** — 세션 상태 기계는 `IN_EPISODE → IDLE` 을
허용한다. `IN_SESSION` 만 정지로 보면 그 경로에서 녹화가 영영 안 끝나 큐에 쌓인
프레임이 통째로 유실된다.
"""

from __future__ import annotations

import os

from typing import Iterator

from unittest.mock import patch

import numpy as np
import pytest

pytest.importorskip('rclpy', reason='requires ROS 2 runtime')

import rclpy                                                            # noqa: E402
from rclpy.parameter import Parameter                                   # noqa: E402
from rdfp_msgs.msg import SessionCommand                                # noqa: E402
from sensor_msgs.msg import Image                                       # noqa: E402

from rdfp.recorder import rdfp_image_recorder_node as node_module       # noqa: E402
from rdfp.recorder.rdfp_image_recorder_node import RdfpImageRecorderNode  # noqa: E402

WIDTH, HEIGHT = 4, 4


@pytest.fixture(scope='module', autouse=True)
def _rclpy_session() -> Iterator[None]:
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def node(tmp_path) -> Iterator[RdfpImageRecorderNode]:
    """recorder 를 대역으로 바꾼 노드. 파일은 만들지 않는다."""
    with patch.object(node_module, 'FFMpegMp4Recorder') as cls:
        rec = cls.return_value
        rec.state = 'STOPPED'
        rec.frames_written = 0
        rec.selected_codec = 'libx264'
        rec.stop.return_value = str(tmp_path / 'out.mp4')
        n = RdfpImageRecorderNode(parameter_overrides=[
            Parameter('output_dir', value=str(tmp_path)),
            Parameter('resolution', value=f'{WIDTH}x{HEIGHT}'),
            Parameter('fps', value=10),
            Parameter('pending_queue_length', value=4),
        ])
        try:
            yield n
        finally:
            n.destroy_node()


def _session(state: str, sec: int) -> SessionCommand:
    msg = SessionCommand()
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = 0
    msg.state = state
    return msg


def _image(sec: int, nanosec: int = 0) -> Image:
    msg = Image()
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.height, msg.width = HEIGHT, WIDTH
    msg.encoding = 'bgr8'
    msg.step = WIDTH * 3
    msg.data = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8).tobytes()
    return msg


def _written_stamps(node: RdfpImageRecorderNode) -> list[int]:
    """`_write_frame` 이 실제로 기록한 stamp 목록 (한 장도 없으면 빈 목록)."""
    return getattr(node, '_written_for_test', [])


@pytest.fixture(autouse=True)
def _capture_writes(monkeypatch):
    """`_write_frame` 을 가로채 어떤 stamp 가 기록됐는지 모은다."""
    original = RdfpImageRecorderNode._write_frame

    def spy(self, stamp_ns, frame):
        if not hasattr(self, '_written_for_test'):
            self._written_for_test = []
        self._written_for_test = self._written_for_test + [stamp_ns]
        return original(self, stamp_ns, frame)

    monkeypatch.setattr(RdfpImageRecorderNode, '_write_frame', spy)


# ---------- 정지 경로가 둘이다 ----------

@pytest.mark.parametrize('stop_state', ['IN_SESSION', 'IDLE'])
def test_both_stop_paths_finish_the_recording(node, stop_state):
    """**`IDLE` 도 정지다.** 에피소드 도중 `stop_session` 을 부르면 세션은
    `IN_SESSION` 을 거치지 않고 바로 `IDLE` 로 간다. 그 경로를 놓치면 녹화가
    영영 안 끝나고 큐에 쌓인 프레임이 통째로 사라진다.
    """
    node._on_session(_session('IN_EPISODE', 10))
    node._on_image(_image(11))
    node._on_session(_session(stop_state, 12))

    assert node._recording is False
    node._recorder.stop.assert_called_once()
    assert 11_000_000_000 in _written_stamps(node), '큐의 프레임이 flush 돼야 한다'


def test_stop_flushes_every_queued_frame_inside_the_window(node):
    node._on_session(_session('IN_EPISODE', 10))
    for sec in (11, 12, 13):
        node._on_image(_image(sec))
    node._on_session(_session('IN_SESSION', 20))

    assert _written_stamps(node) == [11_000_000_000, 12_000_000_000, 13_000_000_000]


# ---------- ① 시작 경계 ----------

def test_frames_before_start_stamp_are_dropped(node):
    """세션 시작 **이전**에 찍힌 프레임은 큐에 있어도 버린다."""
    node._on_image(_image(5))        # 시작 전
    node._on_image(_image(6))        # 시작 전
    node._on_session(_session('IN_EPISODE', 10))
    node._on_image(_image(11))
    node._on_session(_session('IN_SESSION', 20))

    assert _written_stamps(node) == [11_000_000_000]


def test_frame_exactly_at_start_stamp_is_kept(node):
    """경계값은 **포함**이다 (`frame_ts >= start_ts`)."""
    node._on_image(_image(10))
    node._on_session(_session('IN_EPISODE', 10))
    node._on_session(_session('IN_SESSION', 20))

    assert _written_stamps(node) == [10_000_000_000]


# ---------- ③ 정지 경계 ----------

def test_frames_after_stop_stamp_are_not_recorded(node):
    """정지 stamp 이후 프레임은 큐에 남아 있어도 기록하지 않는다."""
    node._on_session(_session('IN_EPISODE', 10))
    node._on_image(_image(11))
    node._on_image(_image(30))       # 정지 이후 시각
    node._on_session(_session('IN_SESSION', 20))

    assert _written_stamps(node) == [11_000_000_000]


def test_late_frame_arriving_after_stop_is_rejected(node):
    """정지 처리가 끝난 뒤 도착한 뒤늦은 프레임은 파일이 이미 닫혀 받을 수 없다."""
    node._on_session(_session('IN_EPISODE', 10))
    node._on_session(_session('IN_SESSION', 20))
    before = list(_written_stamps(node))

    node._on_image(_image(15))       # 창 안이지만 너무 늦게 왔다

    assert _written_stamps(node) == before


# ---------- 상태 기계 ----------

def test_second_start_is_ignored_while_recording(node):
    node._on_session(_session('IN_EPISODE', 10))
    node._recorder.start.reset_mock()
    node._on_session(_session('IN_EPISODE', 11))
    node._recorder.start.assert_not_called()


def test_stop_without_recording_is_noop(node):
    node._on_session(_session('IN_SESSION', 10))
    node._recorder.stop.assert_not_called()


def test_consecutive_episodes_use_their_own_windows(node):
    node._on_session(_session('IN_EPISODE', 10))
    node._on_image(_image(11))
    node._on_session(_session('IN_SESSION', 12))
    first = list(_written_stamps(node))

    node._recorder.state = 'STOPPED'
    node._on_image(_image(13))                    # 에피소드 사이 — 버려야 한다
    node._on_session(_session('IN_EPISODE', 20))
    node._on_image(_image(21))
    node._on_session(_session('IN_SESSION', 22))

    assert first == [11_000_000_000]
    assert 13_000_000_000 not in _written_stamps(node), '에피소드 사이 프레임이 섞였다'
    assert 21_000_000_000 in _written_stamps(node)


# ---------- 세션 채널은 전역이다 ----------

def test_session_subscription_ignores_the_node_namespace(tmp_path):
    """**로봇별 네임스페이스 안에 있어도 `/session` 을 구독한다** (규약 §2.5).

    세션은 시스템에 하나다 — 로봇이 둘 이상인 것은 공동 작업으로 하나의 학습
    데이터를 만든다는 뜻이다. 상대(`session`)로 되돌아가면 `/abc/session` 을 찾아
    **조용히 녹화가 시작되지 않는다.**
    """
    with patch.object(node_module, 'FFMpegMp4Recorder'):
        n = RdfpImageRecorderNode(namespace='/abc', parameter_overrides=[
            Parameter('output_dir', value=str(tmp_path)),
            Parameter('resolution', value=f'{WIDTH}x{HEIGHT}'),
        ])
        try:
            assert n._session_sub.topic_name == '/session'
            assert n._image_sub.topic_name.startswith('/abc/'), '이미지는 로봇별로 갈린다'
        finally:
            n.destroy_node()


# ---------- 파일명 접두사 ----------

def test_prefix_parameter_is_named_file_prefix(tmp_path):
    """**`session_prefix` 가 아니다.**

    `/session` 상태 머신의 세션과 무관하다 — 산출물은 **에피소드마다** 하나이고
    (한 세션 안에서 start/stop_episode 가 반복된다), 세션은 시스템에 하나다.
    이름이 되돌아가면 그 혼동이 되살아난다.
    """
    with patch.object(node_module, 'FFMpegMp4Recorder'):
        n = RdfpImageRecorderNode(parameter_overrides=[
            Parameter('output_dir', value=str(tmp_path)),
            Parameter('resolution', value=f'{WIDTH}x{HEIGHT}'),
        ])
        try:
            assert n.has_parameter('file_prefix')
            assert not n.has_parameter('session_prefix')
        finally:
            n.destroy_node()


def test_prefix_separates_files_in_one_directory(tmp_path):
    """한 디렉터리에 카메라 여러 대를 녹화하는 것이 이 값의 쓸모다."""
    with patch.object(node_module, 'FFMpegMp4Recorder'):
        n = RdfpImageRecorderNode(parameter_overrides=[
            Parameter('output_dir', value=str(tmp_path)),
            Parameter('resolution', value=f'{WIDTH}x{HEIGHT}'),
            Parameter('file_prefix', value='wrist'),
        ])
        try:
            assert n._build_output_path().split('/')[-1].startswith('wrist_')
            assert n._build_metadata_path().endswith('wrist_metadata.json')
        finally:
            n.destroy_node()


# ---------- 접두사 검증 (경로 탈출) ----------

@pytest.mark.parametrize('bad', ['../../tmp/pwn', 'nested/foo', '/abs/path',
                                 '', '.', '..', 'a\\b'])
def test_prefix_that_escapes_the_output_dir_is_rejected(tmp_path, bad):
    """**기동 시점에 막는다.**

    이 값은 파일명 조각으로 그대로 삽입되므로 `..` 나 경로 구분자가 들어가면
    `output_dir` 를 벗어나 쓴다. 첫 에피소드가 시작되고 나서 드러나면 **그 시연을
    잃는다** — 녹화 대상이 이미 지나가 버렸기 때문이다.
    """
    with patch.object(node_module, 'FFMpegMp4Recorder'):
        with pytest.raises(Exception) as exc:
            RdfpImageRecorderNode(parameter_overrides=[
                Parameter('output_dir', value=str(tmp_path)),
                Parameter('resolution', value=f'{WIDTH}x{HEIGHT}'),
                Parameter('file_prefix', value=bad),
            ])
    assert 'file_prefix' in str(exc.value)


@pytest.mark.parametrize('ok', ['session', 'wrist', 'overhead_cam', 'cam-1', 'a.b'])
def test_ordinary_prefixes_are_accepted(tmp_path, ok):
    """점이 들어간 이름(`a.b`)은 막지 않는다 — `.`/`..` 자체만 거른다."""
    with patch.object(node_module, 'FFMpegMp4Recorder'):
        n = RdfpImageRecorderNode(parameter_overrides=[
            Parameter('output_dir', value=str(tmp_path)),
            Parameter('resolution', value=f'{WIDTH}x{HEIGHT}'),
            Parameter('file_prefix', value=ok),
        ])
        try:
            assert n._file_prefix == ok
        finally:
            n.destroy_node()


def test_output_path_stays_inside_the_output_dir(tmp_path):
    """검증의 목적은 이 불변식 하나다."""
    with patch.object(node_module, 'FFMpegMp4Recorder'):
        n = RdfpImageRecorderNode(parameter_overrides=[
            Parameter('output_dir', value=str(tmp_path)),
            Parameter('resolution', value=f'{WIDTH}x{HEIGHT}'),
            Parameter('file_prefix', value='wrist'),
        ])
        try:
            for path in (n._build_output_path(), n._build_metadata_path()):
                assert os.path.realpath(path).startswith(os.path.realpath(str(tmp_path)))
        finally:
            n.destroy_node()
