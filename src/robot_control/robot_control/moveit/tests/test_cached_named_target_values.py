"""캐시된 SRDF 에서 named target 관절값을 꺼내는 공개 API.

트윈이 이동 결과의 ``goal_error`` 를 계산할 때 쓴다. 조회 요청을 보내면 결과 보고가
서비스 왕복에 묶이므로 **캐시만** 읽고, 없으면 ``None`` 으로 물러난다.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')                                        # noqa: E402
pytest.importorskip('moveit_msgs')                                  # noqa: E402

from robot_control.moveit.move_group_client import MoveGroupClient   # noqa: E402

READY = {'panda_joint1': 0.0, 'panda_joint4': -2.356}


def _stub(group_states, group: str = 'panda_arm') -> SimpleNamespace:
    return SimpleNamespace(_group_states=group_states, _moveit_group_name=group,
                           _frame_id='panda_link0')


def test_returns_copy_of_cached_values() -> None:
    stub = _stub({'panda_arm': {'ready': READY}})
    values = MoveGroupClient.cached_named_target_values(stub, 'ready')
    assert values == READY
    values['panda_joint1'] = 9.9
    assert READY['panda_joint1'] == 0.0


def test_none_when_cache_not_loaded() -> None:
    assert MoveGroupClient.cached_named_target_values(_stub(None), 'ready') is None


def test_none_for_unknown_name_or_other_group() -> None:
    stub = _stub({'hand': {'open': {'panda_finger_joint1': 0.04}}})
    assert MoveGroupClient.cached_named_target_values(stub, 'open') is None
    stub = _stub({'panda_arm': {'ready': READY}})
    assert MoveGroupClient.cached_named_target_values(stub, 'extended') is None


def test_frame_id_property() -> None:
    assert MoveGroupClient.frame_id.fget(_stub({})) == 'panda_link0'
