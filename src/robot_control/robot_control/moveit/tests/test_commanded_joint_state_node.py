"""moveit/commanded_joint_state_node 의 병합 규칙.

**servo 의 래칫을 끊는 노드다.** servo 는 매 주기 측정값에 IK 증분을 더하므로 부하로 처진
만큼이 명령에 적분된다 (Isaac 실측: 100 g 을 쥐면 joint5 가 주기당 7.7 mrad, 1.5 초에
−0.22 rad → 손끝이 옆으로 30 mm). 이 노드가 servo 에게 측정값 대신 컨트롤러의 명령 위치를
주면 처짐은 JTC 의 추종 오차로만 남는다.

가장 위험한 실패 둘:

- **길이가 안 맞는 명령을 잘라 쓰는 것** — 관절이 밀린 채 servo 가 그 위에 적분한다.
- **손가락을 빼고 내는 것** — servo 의 상태 모니터가 완성되지 않아 조용히 기다리기만 한다.
"""

from __future__ import annotations

import pytest

pytest.importorskip('control_msgs', reason='requires ROS 2 runtime (control_msgs)')

from control_msgs.msg import JointTrajectoryControllerState      # noqa: E402
from sensor_msgs.msg import JointState                           # noqa: E402

from robot_control.moveit.commanded_joint_state_node import merge_commanded  # noqa: E402

ARM = [f'panda_joint{i}' for i in range(1, 8)]
FINGERS = ['panda_finger_joint1', 'panda_finger_joint2']


def _state(reference=None, desired=None, ref_vel=None, stamp_sec=12):
    msg = JointTrajectoryControllerState()
    msg.header.stamp.sec = stamp_sec
    msg.joint_names = list(ARM)
    if reference is not None:
        msg.reference.positions = list(reference)
    if ref_vel is not None:
        msg.reference.velocities = list(ref_vel)
    if desired is not None:
        msg.desired.positions = list(desired)
    return msg


def _measured(with_velocity=False):
    msg = JointState()
    msg.name = ARM + FINGERS
    # 측정값은 명령과 다르게 둔다 — 팔 관절에 이것이 새어 나오면 실패다.
    msg.position = [0.9] * 7 + [0.03, 0.04]
    if with_velocity:
        msg.velocity = [0.0] * 9
    return msg


def test_arm_comes_from_reference_and_fingers_pass_through():
    out = merge_commanded(_state(reference=[0.1] * 7), _measured())
    assert out is not None
    assert out.name == ARM + FINGERS
    assert out.position[:7] == pytest.approx([0.1] * 7)      # 명령이지 측정(0.9)이 아니다
    assert out.position[7:] == pytest.approx([0.03, 0.04])   # 손가락은 측정 통과


def test_falls_back_to_desired_when_reference_is_empty():
    out = merge_commanded(_state(desired=[0.2] * 7), _measured())
    assert out is not None
    assert out.position[:7] == pytest.approx([0.2] * 7)


def test_reference_wins_over_desired():
    out = merge_commanded(_state(reference=[0.1] * 7, desired=[0.2] * 7), _measured())
    assert out.position[:7] == pytest.approx([0.1] * 7)


def test_no_commanded_positions_means_no_message():
    assert merge_commanded(_state(), _measured()) is None


def test_length_mismatch_is_dropped_not_truncated():
    """6 개짜리 명령을 7 관절에 잘라 쓰면 관절이 밀린다 — 버려야 한다."""
    assert merge_commanded(_state(reference=[0.1] * 6), _measured()) is None
    assert merge_commanded(_state(desired=[0.2] * 8), _measured()) is None


def test_waits_for_measured_joint_states():
    """손가락이 빠진 상태를 내면 servo 가 영영 기다린다."""
    assert merge_commanded(_state(reference=[0.1] * 7), None) is None


def test_velocity_only_when_both_sides_have_it():
    with_both = merge_commanded(_state(reference=[0.1] * 7, ref_vel=[0.5] * 7),
                                _measured(with_velocity=True))
    assert len(with_both.velocity) == 9
    assert with_both.velocity[:7] == pytest.approx([0.5] * 7)

    measured_only = merge_commanded(_state(reference=[0.1] * 7), _measured(with_velocity=True))
    assert len(measured_only.velocity) == 0

    command_only = merge_commanded(_state(reference=[0.1] * 7, ref_vel=[0.5] * 7), _measured())
    assert len(command_only.velocity) == 0     # 한쪽만 있으면 길이가 어긋나므로 뺀다


def test_stamp_is_the_controllers():
    """servo 는 sim 시계로 신선도를 본다 — 컨트롤러가 찍은 stamp 를 그대로 넘긴다."""
    out = merge_commanded(_state(reference=[0.1] * 7, stamp_sec=77), _measured())
    assert out.header.stamp.sec == 77
