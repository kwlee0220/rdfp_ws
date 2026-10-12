"""`CameraRestampNode` — 카메라 프레임을 관절 짝의 오프셋으로 재stamp 하는가."""

from __future__ import annotations

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('sensor_msgs')

import time                                                         # noqa: E402

import rclpy                                                        # noqa: E402
from sensor_msgs.msg import CompressedImage, TimeReference          # noqa: E402

from robot_control.functionbay.camera_restamp_node import CameraRestampNode   # noqa: E402

MS = 1_000_000
SIM0 = 4600 * 1_000_000_000              # 경과시간 회차의 sim stamp
EPOCH_OFFSET = 1_789_000_000 * 1_000_000_000


@pytest.fixture(scope='module', autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = CameraRestampNode()
    yield n
    n.destroy_node()


def _capture(node) -> list[CompressedImage]:
    sent: list[CompressedImage] = []
    node._publisher.publish = sent.append          # noqa: SLF001 - 테스트 목적
    return sent


def _ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _pair(sim_ns: int, offset_ns: int) -> TimeReference:
    msg = TimeReference()
    msg.time_ref.sec, msg.time_ref.nanosec = divmod(sim_ns, 1_000_000_000)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(sim_ns + offset_ns, 1_000_000_000)
    return msg


def _image(sim_ns: int) -> CompressedImage:
    msg = CompressedImage()
    msg.format = 'jpeg'
    msg.data = b'\xff\xd8'
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(sim_ns, 1_000_000_000)
    return msg


def test_holds_a_frame_until_a_later_joint_pair_arrives(node):
    """자기보다 sim stamp 가 늦은 관절 짝이 오기 전에는 내보내지 않는다."""
    sent = _capture(node)
    node._on_reference(_pair(SIM0, EPOCH_OFFSET))
    node._on_image(_image(SIM0 + 7 * MS))
    assert sent == []

    node._on_reference(_pair(SIM0 + 20 * MS, EPOCH_OFFSET))
    assert len(sent) == 1
    assert _ns(sent[0].header.stamp) == SIM0 + 7 * MS + EPOCH_OFFSET


def test_uses_the_offset_of_the_nearest_joint_by_sim_stamp(node):
    """정지 경계처럼 앞뒤 오프셋이 다르면 sim stamp 가 가까운 쪽을 쓴다."""
    sent = _capture(node)
    before, after = EPOCH_OFFSET, EPOCH_OFFSET + 900 * MS
    node._on_reference(_pair(SIM0, before))
    node._on_reference(_pair(SIM0 + 100 * MS, after))
    node._on_image(_image(SIM0 + 10 * MS))        # 앞쪽이 가깝다
    node._on_image(_image(SIM0 + 90 * MS))        # 뒤쪽이 가깝다
    assert [_ns(m.header.stamp) for m in sent] == [
        SIM0 + 10 * MS + before, SIM0 + 90 * MS + after]


def test_output_stamps_never_go_backwards_across_an_offset_drop(node):
    """정지 직후처럼 짝 오프셋이 크게 내려가도 출력 순서는 뒤집히지 않는다 (실측 −5.8 ms 재현)."""
    sent = _capture(node)
    node._on_reference(_pair(SIM0, EPOCH_OFFSET + 100 * MS))
    node._on_reference(_pair(SIM0 + 20 * MS, EPOCH_OFFSET))
    node._on_image(_image(SIM0 + 5 * MS))          # 앞 짝: +100 ms
    node._on_image(_image(SIM0 + 15 * MS))         # 뒤 짝: +0 ms → 그대로면 90 ms 역순
    stamps = [_ns(m.header.stamp) for m in sent]
    assert stamps[1] > stamps[0]
    assert node._monotonic.clamped == 1            # noqa: SLF001 - 테스트 목적


def test_falls_back_to_the_nearest_known_pair_after_max_hold(node):
    """짝을 끝내 못 받으면 최대 대기 뒤 알려진 가장 가까운 오프셋을 쓴다."""
    sent = _capture(node)
    node._on_reference(_pair(SIM0, EPOCH_OFFSET))
    node._on_image(_image(SIM0 + 30 * MS))
    assert sent == []
    node._max_hold_ns = 0                          # noqa: SLF001 - 테스트 목적
    node._flush()
    assert _ns(sent[0].header.stamp) == SIM0 + 30 * MS + EPOCH_OFFSET


def test_drops_frames_when_no_pair_ever_arrives(node):
    """관절 쪽이 아예 없으면 흔들리는 원 stamp 를 흘리지 않고 버린다."""
    sent = _capture(node)
    node._on_image(_image(SIM0))
    node._max_hold_ns = 0                          # noqa: SLF001 - 테스트 목적
    node._flush()
    assert sent == []
    assert not node._pending                        # noqa: SLF001 - 테스트 목적


def test_zero_stamp_frames_get_the_reception_time(node):
    """stamp 가 0 인 프레임은 정렬할 원본이 없어 수신 시각으로 내보낸다."""
    sent = _capture(node)
    node._on_image(_image(0))
    assert len(sent) == 1
    assert abs(_ns(sent[0].header.stamp) / 1e9 - time.time()) < 1.0
