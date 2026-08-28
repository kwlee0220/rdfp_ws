"""`robot_twin.runtime` 의 전제 조건 검사 테스트.

`runtime` 은 최상위에서 `rclpy` 를 import 하므로 ROS 없이는 자체 skip 한다. 검사
대상인 `_check_preconditions` 는 인스턴스 상태 세 개만 읽으므로, 노드나 executor
없이 껍데기 인스턴스로 확인한다.
"""

from __future__ import annotations

from typing import Optional

import threading

import pytest

pytest.importorskip('rclpy', reason='robot_twin.runtime imports rclpy')

from robot_twin.config import OperationConfig  # noqa: E402
from robot_twin.errors import TwinError  # noqa: E402
from robot_twin.runtime import RobotTwinRuntime  # noqa: E402

ARM_OP = OperationConfig(name='move_to_named_target', kind='async', resource='arm',
                         backend={'method': 'move_to_named_target_async'})
GRIPPER_OP = OperationConfig(
    name='move_gripper_to_target', kind='sync', resource='gripper',
    backend={'topic': '/gripper_control/gripper_cmds',
             'topic_type': 'rdfp_msgs/msg/GripperCommand',
             'result_variable': 'gripper_last_command_result',
             'targets': {'open': {'position': 0.04}}}
)


def _runtime(*, move_group: Optional[object]) -> RobotTwinRuntime:
    """`_check_preconditions` 가 읽는 상태만 채운 껍데기 런타임."""
    runtime = object.__new__(RobotTwinRuntime)
    runtime._mg_lock = threading.Lock()
    runtime._move_group = move_group
    runtime._move_group_error = None if move_group else 'not built yet'
    return runtime


def test_arm_operation_requires_move_group() -> None:
    with pytest.raises(TwinError) as exc:
        _runtime(move_group=None)._check_preconditions(ARM_OP)

    assert exc.value.http_status == 503


def test_gripper_operation_does_not_require_move_group() -> None:
    """그리퍼는 액션을 직접 호출한다 — 팔 스택이 없어도 쓸 수 있어야 한다.

    예전에는 전제 조건이 연산별로 갈리지 않아, MoveIt 이 아직 안 뜬 동안 그리퍼
    연산까지 `503` 으로 거부되었다.
    """
    _runtime(move_group=None)._check_preconditions(GRIPPER_OP)   # 예외가 없어야 한다


def test_arm_operation_passes_once_move_group_is_ready() -> None:
    _runtime(move_group=object())._check_preconditions(ARM_OP)
