"""관절 목표 보완 — 지정하지 않은 관절을 현재값으로 고정한다.

보완하지 않으면 목표가 자세 하나가 아니라 "지정한 관절만 만족하는 자세의 **집합**"이
되고, 플래너가 그중 아무거나 고른다. 실측에서 ``panda_joint1`` 하나만 준 호출이
나머지 6축을 최대 3.5 rad 움직여 엔드이펙터가 로봇 뒤쪽 위로 넘어갔다.
"""

from __future__ import annotations

from typing import Any

import pytest

pytest.importorskip('rclpy')                                        # noqa: E402
pytest.importorskip('moveit_msgs')                                  # noqa: E402

from robot_control.moveit.move_group_client import MoveGroupClient   # noqa: E402

ARM_JOINTS = [f'panda_joint{i}' for i in range(1, 8)]
READY = {'panda_joint1': 0.0, 'panda_joint2': -0.785, 'panda_joint3': 0.0,
         'panda_joint4': -2.356, 'panda_joint5': 0.0, 'panda_joint6': 1.571,
         'panda_joint7': 0.785}
# `/joint_states` 는 그룹 밖 관절도 함께 싣는다.
JOINT_STATES = {**READY, 'panda_finger_joint1': 0.04, 'panda_finger_joint2': 0.04}


class _Logger:
    def __init__(self) -> None:
        self.messages: list[str] = []

    def info(self, msg: str) -> None:
        self.messages.append(msg)


class _Node:
    def __init__(self) -> None:
        self._logger = _Logger()

    def get_logger(self) -> _Logger:
        return self._logger


class _Client:
    """`MoveGroupClient` 의 보완 로직만 떼어 쓰기 위한 대역.

    `MoveGroupClient` 는 추상 클래스이고 생성자가 ROS 클라이언트를 만든다. 여기서
    검증하려는 것은 병합 규칙이므로 SRDF 조회와 `/joint_states` 수신만 대체한다.
    """

    def __init__(self, *, group_states: dict[str, Any], current: dict[str, float],
                 group: str = 'panda_arm') -> None:
        self._moveit_group_name = group
        self._node = _Node()
        self._group_states = group_states
        self._current = current
        self.state_reads = 0

    def _ensure_srdf_cache(self, *, timeout: float, externally_spun: bool = False):
        return self._group_states, list(self._group_states)

    def _current_joint_positions(self, *, timeout: float, externally_spun: bool = False):
        self.state_reads += 1
        return self._current

    group_joint_names = MoveGroupClient.group_joint_names
    _complete_joint_values = MoveGroupClient._complete_joint_values


def _client(**overrides: Any) -> _Client:
    base: dict[str, Any] = {
        # SRDF 의 group_state 는 그룹의 관절을 전부 나열한다 — 그 키가 곧 그룹 관절이다.
        'group_states': {'panda_arm': {'ready': dict(READY),
                                       'extended': dict(READY)},
                         'hand': {'open': {'panda_finger_joint1': 0.035}}},
        'current': dict(JOINT_STATES),
    }
    base.update(overrides)
    return _Client(**base)


# ----- 그룹 관절 목록 -----

def test_group_joints_come_from_the_srdf_group_state() -> None:
    assert sorted(_client().group_joint_names()) == sorted(ARM_JOINTS)


def test_any_group_state_gives_the_same_joint_set() -> None:
    """어느 상태를 보든 관절 집합은 같다 — 첫 항목을 써도 된다."""
    c = _client(group_states={'panda_arm': {'transport': dict(READY)}})
    assert sorted(c.group_joint_names()) == sorted(ARM_JOINTS)


def test_missing_group_state_is_an_explicit_error() -> None:
    """조용히 부분 제약으로 되돌아가면 예측 불가 동작이 재발한다."""
    c = _client(group_states={'hand': {'open': {'panda_finger_joint1': 0.035}}})
    with pytest.raises(RuntimeError, match='group_state'):
        c.group_joint_names()


# ----- 보완 -----

def test_unspecified_joints_are_held_at_their_current_value() -> None:
    c = _client()

    out = c._complete_joint_values({'panda_joint1': 0.5})

    assert out['panda_joint1'] == 0.5
    for name in ARM_JOINTS[1:]:
        assert out[name] == READY[name]


def test_specified_values_win_over_the_current_state() -> None:
    c = _client()

    out = c._complete_joint_values({'panda_joint2': 0.1})

    assert out['panda_joint2'] == 0.1


def test_joints_outside_the_group_are_not_added() -> None:
    """finger 관절이 섞이면 계획이 실패한다."""
    out = _client()._complete_joint_values({'panda_joint1': 0.5})

    assert sorted(out) == sorted(ARM_JOINTS)


def test_a_full_specification_is_returned_untouched() -> None:
    """전 관절을 준 호출은 현재 상태를 읽을 이유가 없다."""
    c = _client()
    full = {name: 0.1 for name in ARM_JOINTS}

    assert c._complete_joint_values(full) == full
    assert c.state_reads == 0


def test_a_joint_missing_from_joint_states_is_an_explicit_error() -> None:
    c = _client(current={'panda_joint1': 0.0})

    with pytest.raises(RuntimeError, match='panda_joint2'):
        c._complete_joint_values({'panda_joint1': 0.5})


def test_held_joints_are_logged() -> None:
    """무엇을 고정했는지 로그에 남아야 나중에 자세를 설명할 수 있다."""
    c = _client()

    c._complete_joint_values({'panda_joint1': 0.5})

    assert any('panda_joint7' in m for m in c._node.get_logger().messages)
