"""`JointStateFusionNode` 변환 로직 테스트."""

from __future__ import annotations

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('sensor_msgs')

import rclpy                                              # noqa: E402
from sensor_msgs.msg import JointState                    # noqa: E402

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
