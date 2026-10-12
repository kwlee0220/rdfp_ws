#!/usr/bin/env python3

"""`RdfpImageRecorderNode` 의 **압축 이미지 처리**.

이 노드는 `sensor_msgs/Image` 와 `sensor_msgs/CompressedImage` 를 모두 받고, 무엇을
ffmpeg 에 넘길지는 `input_format` 이 따로 정한다. 여기서 지키는 것은 셋이다.

  ① 압축이 와도 `rawvideo` 면 **내부에서 디코드**해 raw 와 같은 경로로 흐른다
  ② `mjpeg` 면 디코드하지 않고 JPEG 바이트가 그대로 흐른다
  ③ 어긋난 조합(`mjpeg` + `Image`, JPEG 아닌 압축, 크기 불일치)은 **버리고 경고한다**
     — 예외를 올리면 콜백에서 터져 노드가 통째로 내려가고, 증상은 "그 노드만 조용히
     무응답" 이 된다

**두 타입을 같은 이름으로 함께 구독할 수는 없다** — RMW 가 두 번째를 거부하며 그때
rclpy 가 보여 주는 말은 *"invalid allocator"* 라 원인을 안 가리킨다. 그래서 노드는
발행자의 타입을 보고 **하나만** 건다. ④ 가 그것을 지킨다.
"""

from __future__ import annotations

from typing import Iterator

from unittest.mock import patch

import numpy as np
import pytest

pytest.importorskip('rclpy', reason='requires ROS 2 runtime')
cv2 = pytest.importorskip('cv2', reason='decode path needs OpenCV')

import rclpy                                                            # noqa: E402
from rclpy.parameter import Parameter                                   # noqa: E402
from sensor_msgs.msg import CompressedImage, Image                      # noqa: E402

from rdfp.recorder import rdfp_image_recorder_node as node_module       # noqa: E402
from rdfp.recorder.rdfp_image_recorder_node import RdfpImageRecorderNode  # noqa: E402

WIDTH, HEIGHT = 16, 12


@pytest.fixture(scope='module', autouse=True)
def _rclpy_session() -> Iterator[None]:
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _make_node(tmp_path, **overrides) -> RdfpImageRecorderNode:
    params = [Parameter('output_dir', value=str(tmp_path)),
              Parameter('resolution', value=f'{WIDTH}x{HEIGHT}'),
              Parameter('fps', value=10)]
    params += [Parameter(k, value=v) for k, v in overrides.items()]
    return RdfpImageRecorderNode(parameter_overrides=params)


@pytest.fixture
def node(tmp_path) -> Iterator[RdfpImageRecorderNode]:
    with patch.object(node_module, 'FFMpegMp4Recorder') as cls:
        cls.return_value.state = 'STOPPED'
        n = _make_node(tmp_path)
        try:
            yield n
        finally:
            n.destroy_node()


@pytest.fixture
def mjpeg_node(tmp_path) -> Iterator[RdfpImageRecorderNode]:
    with patch.object(node_module, 'FFMpegMp4Recorder') as cls:
        cls.return_value.state = 'STOPPED'
        n = _make_node(tmp_path, input_format='mjpeg')
        try:
            yield n
        finally:
            n.destroy_node()


def _jpeg(width: int = WIDTH, height: int = HEIGHT) -> bytes:
    """디코드하면 `width x height` 가 되는 JPEG."""
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, :, 2] = 200                       # BGR 의 R 채널 — 색 변환 검증에 쓴다
    ok, buf = cv2.imencode('.jpg', frame)
    assert ok
    return buf.tobytes()


def _compressed(data: bytes, fmt: str = 'jpeg') -> CompressedImage:
    msg = CompressedImage()
    msg.format = fmt
    msg.data = data
    msg.header.stamp.sec = 100
    return msg


def _raw() -> Image:
    msg = Image()
    msg.width, msg.height = WIDTH, HEIGHT
    msg.encoding = 'bgr8'
    msg.step = WIDTH * 3
    msg.data = bytes(WIDTH * HEIGHT * 3)
    msg.header.stamp.sec = 100
    return msg


def _queued(node: RdfpImageRecorderNode) -> list:
    return [frame for _, frame in node._pending_queue]


# ----- ① rawvideo 로 오는 압축은 디코드된다 ----------------------------------


def test_compressed_is_decoded_into_ndarray(node) -> None:
    node._on_compressed_image(_compressed(_jpeg()))

    frames = _queued(node)
    assert len(frames) == 1
    assert isinstance(frames[0], np.ndarray), 'rawvideo 는 ndarray 를 기대한다'
    assert frames[0].shape == (HEIGHT, WIDTH, 3)


def test_decoded_frame_honours_rgb8(tmp_path) -> None:
    """`cv2.imdecode` 는 BGR 로 푼다 — rgb8 이면 뒤집어야 색이 맞다."""
    with patch.object(node_module, 'FFMpegMp4Recorder') as cls:
        cls.return_value.state = 'STOPPED'
        n = _make_node(tmp_path, pixel_format='rgb8')
        try:
            n._on_compressed_image(_compressed(_jpeg()))
            frame = _queued(n)[0]
            # 원본은 BGR 의 R 채널(index 2)만 채웠다 — rgb8 이면 index 0 으로 와야 한다.
            assert frame[0, 0, 0] > 150 and frame[0, 0, 2] < 100
        finally:
            n.destroy_node()


def test_decoded_size_mismatch_is_dropped(node) -> None:
    node._on_compressed_image(_compressed(_jpeg(WIDTH * 2, HEIGHT)))

    assert _queued(node) == [], '해상도가 다르면 버린다'


def test_undecodable_payload_is_dropped_not_raised(node) -> None:
    node._on_compressed_image(_compressed(b'not a jpeg at all'))

    assert _queued(node) == []


# ----- ② mjpeg 는 바이트를 그대로 흘린다 -------------------------------------


def test_mjpeg_passes_bytes_through(mjpeg_node) -> None:
    payload = _jpeg()

    mjpeg_node._on_compressed_image(_compressed(payload))

    frames = _queued(mjpeg_node)
    assert frames == [payload], 'mjpeg 는 디코드하지 않는다'


# ----- ③ 어긋난 조합 ----------------------------------------------------------


def test_non_jpeg_compression_is_dropped(node) -> None:
    node._on_compressed_image(_compressed(b'\x89PNG\r\n', fmt='png'))

    assert _queued(node) == []


def test_raw_image_under_mjpeg_is_dropped(mjpeg_node) -> None:
    """raw 를 JPEG 으로 넘길 수 없다 — 막지 않으면 매 프레임 write 가 실패한다."""
    mjpeg_node._on_image(_raw())

    assert _queued(mjpeg_node) == []


# ----- ④ 타입은 발행자를 보고 정한다 ------------------------------------------


def test_mjpeg_subscribes_without_waiting_for_a_publisher(mjpeg_node) -> None:
    """`mjpeg` 는 타입이 이미 정해져 있다 — 발행자를 기다리지 않는다."""
    assert mjpeg_node._image_sub is not None
    assert mjpeg_node._resolve_image_type() is CompressedImage


def test_type_is_unknown_until_a_publisher_appears(node) -> None:
    """발행자가 없으면 구독하지 않고 타이머로 다시 본다 — 조용히 죽지 않는다."""
    assert node._image_sub is None
    assert node._type_probe_timer is not None


def test_publisher_type_selects_the_subscription(tmp_path) -> None:
    """압축 발행자가 있으면 `rawvideo` 라도 `CompressedImage` 를 구독한다."""
    with patch.object(node_module, 'FFMpegMp4Recorder') as cls:
        cls.return_value.state = 'STOPPED'
        n = _make_node(tmp_path, image_topic='probe/image_raw')
        helper = rclpy.create_node('compressed_publisher_probe')
        pub = helper.create_publisher(CompressedImage, '/probe/image_raw', 1)
        try:
            for _ in range(50):
                if n._resolve_image_type() is CompressedImage:
                    break
                rclpy.spin_once(n, timeout_sec=0.05)
            n._subscribe_image()
            assert n._image_sub is not None
            assert n._image_sub.msg_type is CompressedImage
        finally:
            helper.destroy_publisher(pub)
            helper.destroy_node()
            n.destroy_node()
