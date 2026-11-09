"""기동 게이트 — 첫 관절 보고를 기다리고, 켜면 그리퍼 축 수로 씬을 대조한다.

씬 대조는 **시뮬레이터를 다른 씬으로 띄우고 프로파일 `sim_scene` 을 안 바꾼** 경우를 기동
시점에 잡으려는 것이다 — 그 전에는 스택이 멀쩡히 올라왔다가 그리퍼 명령이 거부될 때에야
드러났다(2026-10-20, 시뮬레이터 t1 · 프로파일 r2).
"""

from __future__ import annotations

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('sensor_msgs')

import rclpy                                                        # noqa: E402
from rclpy.parameter import Parameter                               # noqa: E402
from sensor_msgs.msg import JointState                              # noqa: E402

from robot_control.functionbay.readiness_gate_node import ReadinessGateNode   # noqa: E402


@pytest.fixture(scope='module', autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _gate(**params) -> ReadinessGateNode:
    overrides = [Parameter(name, value=value) for name, value in params.items()]
    return ReadinessGateNode(parameter_overrides=overrides)


def _report(axes: int) -> JointState:
    return JointState(position=[0.0] * axes)


def test_without_the_scene_check_the_arm_report_is_enough():
    gate = _gate()
    try:
        assert not gate.ready
        gate._on_message(_report(7))
        assert gate.ready
        assert not gate.scene_mismatch
    finally:
        gate.destroy_node()


def test_the_scene_check_also_waits_for_the_gripper():
    gate = _gate(gripper_topic='/output/gripper_joint', expected_gripper_axes=6, sim_scene='t1')
    try:
        gate._on_message(_report(7))
        assert not gate.ready, '그리퍼 보고를 안 받았는데 통과했다'
        assert gate.waiting_for() == '/output/gripper_joint'

        gate._on_gripper(_report(6))
        assert gate.ready
        assert not gate.scene_mismatch
    finally:
        gate.destroy_node()


def test_a_scene_mismatch_is_reported_with_the_fix():
    """시뮬레이터 t1(6축) · 프로파일 r2(2축) — 2026-10-20 에 실제로 난 조합이다."""
    gate = _gate(gripper_topic='/output/gripper_joint', expected_gripper_axes=2, sim_scene='r2')
    try:
        gate._on_message(_report(7))
        gate._on_gripper(_report(6))

        assert gate.scene_mismatch
        message = gate.mismatch_message()
        assert '6 gripper axes' in message and "'r2'" in message and 'expects 2' in message
        assert 'sim_scene' in message and 'panda_functionbay.yaml' in message
    finally:
        gate.destroy_node()


def test_only_the_first_gripper_report_counts():
    gate = _gate(gripper_topic='/output/gripper_joint', expected_gripper_axes=2, sim_scene='r2')
    try:
        gate._on_gripper(_report(2))
        gate._on_gripper(_report(6))
        assert not gate.scene_mismatch
    finally:
        gate.destroy_node()


def test_zero_expected_axes_turns_the_check_off():
    gate = _gate(gripper_topic='/output/gripper_joint', expected_gripper_axes=0)
    try:
        gate._on_message(_report(7))
        assert gate.ready
    finally:
        gate.destroy_node()
