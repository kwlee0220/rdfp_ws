"""`JointStateFusionNode` 변환 로직 테스트."""

from __future__ import annotations

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('sensor_msgs')

import time                                               # noqa: E402

import rclpy                                              # noqa: E402
from sensor_msgs.msg import JointState, TimeReference     # noqa: E402

from robot_control.functionbay.joint_state_fusion_node import (   # noqa: E402
    JointStateFusionNode,
)

ARM = [f'panda_joint{i}' for i in range(1, 8)]


@pytest.fixture(scope='module', autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = JointStateFusionNode()
    yield n
    n.destroy_node()


def _capture(node) -> list[JointState]:
    """발행 메시지를 가로채 담는다 (실제 DDS 왕복 없이 변환만 검증)."""
    sent: list[JointState] = []
    node._publisher.publish = sent.append      # noqa: SLF001 - 테스트 목적
    return sent


def _incoming(positions, **kw) -> JointState:
    msg = JointState()
    msg.position = list(positions)
    for k, v in kw.items():
        setattr(msg, k, list(v) if k in ('name', 'velocity', 'effort') else v)
    return msg


def test_assigns_names_by_order(node):
    """이름이 없는 입력에 배열 순서대로 이름을 붙인다."""
    sent = _capture(node)
    node._on_joint_state(_incoming([0.1 * i for i in range(7)]))

    assert len(sent) == 1
    out = sent[0]
    assert out.name == ARM + ['panda_finger_joint1']
    assert out.position == pytest.approx([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.04])


def test_drops_on_length_mismatch(node):
    """길이가 다르면 발행하지 않는다 — 순서 계약이 깨진 상태다."""
    sent = _capture(node)
    node._on_joint_state(_incoming([0.0] * 6))      # 7 이어야 한다
    assert sent == []


def test_uses_incoming_names_when_present(node):
    """입력이 이름을 채워 주면 그것을 신뢰한다."""
    sent = _capture(node)
    names = ['j1', 'j2']
    node._on_joint_state(_incoming([1.0, 2.0], name=names))

    assert sent[0].name == names + ['panda_finger_joint1']
    assert sent[0].position == pytest.approx([1.0, 2.0, 0.04])


def test_velocity_passthrough_and_padding(node):
    """속도는 길이가 맞을 때만 옮기고, 덧붙인 관절 몫은 0 으로 채운다."""
    sent = _capture(node)
    node._on_joint_state(_incoming([0.0] * 7, velocity=[1.0] * 7))
    assert sent[0].velocity == pytest.approx([1.0] * 7 + [0.0])

    sent.clear()
    node._on_joint_state(_incoming([0.0] * 7, velocity=[1.0] * 3))   # 길이 불일치
    assert list(sent[0].velocity) == []


def test_stamp_auto_fills_zero_stamp(node):
    """stamp 가 0 이면 노드 시계로 채운다 (시뮬레이터가 안 채우는 경우)."""
    sent = _capture(node)
    node._on_joint_state(_incoming([0.0] * 7))
    stamp = sent[0].header.stamp
    assert (stamp.sec, stamp.nanosec) != (0, 0)


def _elapsed(sec: int, nanosec: int = 0) -> JointState:
    """기동 후 경과시간으로 찍힌 입력 (2026-09-17 회차의 모습)."""
    msg = _incoming([0.0] * 7)
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    return msg


def _seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def test_aligned_is_the_default_and_moves_elapsed_stamps_to_the_host_clock(node):
    """기본값 `aligned` 는 경과시간 stamp 를 벽시계 축으로 옮긴다."""
    assert node._stamp_source == 'aligned'
    sent = _capture(node)
    pairs: list[TimeReference] = []
    node._reference_publisher.publish = pairs.append      # noqa: SLF001 - 테스트 목적

    node._on_joint_state(_elapsed(4600))
    assert abs(_seconds(sent[0].header.stamp) - time.time()) < 1.0
    # 짝은 (정렬 시각, 원 stamp) 이다 — 카메라 중계가 같은 오프셋을 쓰려면 둘 다 필요하다.
    assert (pairs[0].time_ref.sec, pairs[0].time_ref.nanosec) == (4600, 0)
    assert pairs[0].header.stamp == sent[0].header.stamp


def test_aligned_never_reorders_samples(node):
    """묶음으로 들어온 표본도 순서가 뒤집히지 않는다.

    간격 보존(창이 찬 뒤 20 ms)은 `test_stamp_alignment` 가 합성 스트림으로 본다. 여기서는
    한 묶음만 넣으므로 오프셋이 수렴 중이라 표본이 같은 시각 근처로 모이는 것이 정상이다.
    """
    sent = _capture(node)
    node._reference_publisher.publish = lambda _msg: None    # noqa: SLF001 - 테스트 목적
    for k in range(3):
        node._on_joint_state(_elapsed(4600, 20_000_000 * k))
    stamps = [_seconds(m.header.stamp) for m in sent]
    assert stamps == sorted(stamps)


def test_auto_keeps_the_incoming_stamp(node):
    """옛 동작 `auto` 는 0 이 아닌 stamp 를 그대로 둔다."""
    node._stamp_source = 'auto'                               # noqa: SLF001 - 테스트 목적
    node._aligner = None                                      # noqa: SLF001 - 테스트 목적
    sent = _capture(node)
    node._on_joint_state(_elapsed(4600, 5))
    assert (sent[0].header.stamp.sec, sent[0].header.stamp.nanosec) == (4600, 5)
