#!/usr/bin/env python3

"""`create_move_group_client` — 백엔드 이름으로 구현을 고른다.

**프로파일 파일의 내용과 해석 규칙은 `test_backend_profiles.py` 가 본다.** 여기서는
그 결과로 *어떤 클라이언트가 만들어지는가*만 본다.

1. 백엔드가 JTC/JGPC 중 맞는 구현으로 이어진다.
2. `arm_command_format` 은 **JGPC 생성자에만** 간다 — JTC 생성자는 그 인자를 모른다.
3. JTC 로 결정됐는데 액션 서버가 없으면 **기동 시점에 알린다.**
4. 스택보다 먼저 뜨는 호출자는 그 경고를 끌 수 있다.
"""

from __future__ import annotations

from typing import Optional

import pytest

pytest.importorskip('rclpy', reason='requires ROS 2 runtime')

from robot_control.moveit import move_group_factory as f     # noqa: E402


# ----- 구현 선택 -------------------------------------------------------------

class _Logger:
    def __init__(self):
        self.errors, self.infos = [], []

    def error(self, msg):
        self.errors.append(str(msg))

    def info(self, msg):
        self.infos.append(str(msg))

    def warning(self, msg):
        pass


class _Node:
    """토픽 그래프만 흉내내는 대역."""

    def __init__(self, *, publishers: Optional[dict] = None):
        self._pub = publishers or {}
        self._logger = _Logger()

    def get_logger(self):
        return self._logger

    def get_publishers_info_by_topic(self, topic):
        return self._pub.get(topic, [])

    def get_subscriptions_info_by_topic(self, topic):
        return []


JTC_STATUS = '/panda_arm_controller/follow_joint_trajectory/_action/status'


def test_isaac_backend_builds_a_jtc_client(monkeypatch):
    built = {}
    monkeypatch.setattr(f, 'MoveGroupJtcClient', lambda node, **kw: built.setdefault('jtc', kw))
    node = _Node(publishers={JTC_STATUS: ['server']})

    f.create_move_group_client(node, backend='isaac')

    assert 'jtc' in built


def test_functionbay_backend_builds_a_jgpc_client_with_its_channel(monkeypatch):
    built = {}
    monkeypatch.setattr(f, 'MoveGroupJgpcClient',
                        lambda node, **kw: built.setdefault('jgpc', kw))

    f.create_move_group_client(_Node(), backend='functionbay')

    assert built['jgpc']['arm_command_topic'] == '/input/panda_joint'
    assert built['jgpc']['arm_command_joint_names'] == \
        [f'panda_joint{i}' for i in range(1, 8)]
    assert built['jgpc']['arm_command_format'] == 'joint_state'


def test_format_never_reaches_the_jtc_constructor(monkeypatch):
    """`arm_command_format` 은 JGPC 전용이다 — JTC 생성자는 이 인자를 모른다.

    그대로 넘기면 `TypeError` 로 노드가 안 뜬다.
    """
    built = {}
    monkeypatch.setattr(f, 'MoveGroupJtcClient', lambda node, **kw: built.setdefault('jtc', kw))
    node = _Node(publishers={JTC_STATUS: ['server']})

    f.create_move_group_client(node, backend='isaac', arm_command_format='joint_state')

    assert 'arm_command_format' not in built['jtc']


# ----- JTC 오판 경고 ---------------------------------------------------------

def test_missing_jtc_action_server_is_reported(monkeypatch):
    """계획은 되고 **실행만 조용히 안 되는** 조합을 기동 시점에 알린다."""
    monkeypatch.setattr(f, 'MoveGroupJtcClient', lambda node, **kw: object())
    node = _Node()                                   # status 토픽 발행자가 없다

    f.create_move_group_client(node, backend='isaac')

    assert any('FollowJointTrajectory' in e for e in node.get_logger().errors)


def test_present_jtc_action_server_is_quiet(monkeypatch):
    monkeypatch.setattr(f, 'MoveGroupJtcClient', lambda node, **kw: object())
    node = _Node(publishers={JTC_STATUS: ['server']})

    f.create_move_group_client(node, backend='isaac')

    assert node.get_logger().errors == []


def test_action_server_check_uses_the_hidden_status_topic():
    """**`ros2 action list` 로는 부족하다** — 클라이언트만 있어도 목록에 나온다
    (`moveit_simple_controller_manager` 가 그 경우다). 서버는 `_action/status` 를
    발행하므로 그 발행자 수로 판단한다.
    """
    assert f.jtc_action_server_present(_Node()) is False
    assert f.jtc_action_server_present(_Node(publishers={JTC_STATUS: ['s']})) is True


def test_action_server_warning_can_be_suppressed(monkeypatch):
    """**스택보다 먼저 뜨는 호출자는 꺼야 한다** (`robot_twin` 이 그 경우다).

    그때는 서버가 없는 것이 정상이라 매 기동 오경보가 되고, 그런 로그는 사람을
    길들여 진짜 오류까지 흘려보게 만든다.
    """
    monkeypatch.setattr(f, 'MoveGroupJtcClient', lambda node, **kw: object())
    node = _Node()

    f.create_move_group_client(node, backend='isaac', check_action_server=False)

    assert node.get_logger().errors == []


# ----- velocity_scaling: 인자 > 프로파일 > 코드 기본값 -------------------------

def test_profile_velocity_scaling_reaches_the_client(monkeypatch):
    """`backend` 만 줬을 때 프로파일의 `motion.velocity_scaling` 이 실린다."""
    captured = {}

    class _Spy:
        def __init__(self, node, **kwargs):
            captured.update(kwargs)

    monkeypatch.setattr(f, 'MoveGroupJgpcClient', _Spy)
    f.create_move_group_client(_Node(), backend='functionbay')

    assert captured['velocity_scaling'] == 0.4


def test_explicit_velocity_scaling_beats_the_profile(monkeypatch):
    """**명시 인자가 이긴다.** 안 그러면 한 번 정한 전역값을 못 벗어난다."""
    captured = {}

    class _Spy:
        def __init__(self, node, **kwargs):
            captured.update(kwargs)

    monkeypatch.setattr(f, 'MoveGroupJgpcClient', _Spy)
    f.create_move_group_client(_Node(), backend='functionbay', velocity_scaling=0.1)

    assert captured['velocity_scaling'] == 0.1


def test_no_profile_value_leaves_the_constructor_default(monkeypatch):
    """프로파일에 없으면 **키를 아예 안 넘긴다** — 생성자 기본값(1.0)이 살아야 한다."""
    captured = {}

    class _Spy:
        def __init__(self, node, **kwargs):
            captured.update(kwargs)

    monkeypatch.setattr(f, 'MoveGroupJtcClient', _Spy)
    f.create_move_group_client(_Node(), backend='mock', check_action_server=False)

    assert 'velocity_scaling' not in captured
