"""`externally_spun` 이 계획 경로 끝까지 전달되는지 검사한다.

**executor 가 다른 스레드에서 노드를 돌리고 있을 때(로봇 트윈이 그렇다) 이 값이
빠지면 연산이 영원히 멈춘다.** 지정하지 않은 관절을 현재값으로 채우려면
``/joint_states`` 한 건을 받아야 하는데, 그 대기가 기본값(`False`)에서는 노드를
**직접 spin** 한다. 콜백은 이미 executor 쪽으로 가므로 여기서는 영영 오지 않고,
예외도 타임아웃도 없이 오퍼레이션이 RUNNING 인 채로 남는다.

실제로 그렇게 막혔다 — 트윈의 `move_to_joints` 가 팔을 움직이지 않고 계속 RUNNING
이었으며, `plan_joints_async` 가 인자를 아예 받지 않는 것이 원인이었다.
"""

from __future__ import annotations

from typing import Any

import inspect

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('moveit_msgs')

from robot_control.moveit.move_group_client import MoveGroupClient        # noqa: E402
from robot_control.moveit.move_group_jgpc_client import MoveGroupJgpcClient  # noqa: E402

READY = {f'panda_joint{i}': 0.0 for i in range(1, 8)}


class _Logger:
    def info(self, msg: str) -> None:
        pass

    def error(self, msg: str) -> None:
        pass


class _Node:
    def get_logger(self) -> _Logger:
        return _Logger()


class _GoalClient:
    def send_goal_async(self, _goal):
        from rclpy.task import Future
        return Future()


class _Client:
    """`plan_joints_async` 만 떼어 쓰기 위한 대역.

    검증 대상은 **`externally_spun` 이 `_complete_joint_values` 까지 가는가** 하나다.
    그래서 그 아래(`/joint_states` 수신)는 호출 인자만 기록하고 값을 돌려준다.
    """

    def __init__(self) -> None:
        self._node = _Node()
        self._closed = False
        self._moveit_group_name = 'panda_arm'
        self._move_group_client = _GoalClient()
        self._velocity_scaling = 1.0
        self.seen: list[Any] = []

    def _require_open(self) -> None:
        pass

    def _resolve_velocity_scaling(self, value):
        return value if value is not None else self._velocity_scaling

    def _complete_joint_values(self, values, *, timeout: float = 10.0,
                               externally_spun: bool = False):
        self.seen.append(externally_spun)
        return dict(values)

    plan_joints_async = MoveGroupClient.plan_joints_async


# ----- 서명 -----

def test_plan_joints_async_accepts_externally_spun():
    params = inspect.signature(MoveGroupClient.plan_joints_async).parameters
    assert 'externally_spun' in params, (
        'plan_joints_async 가 externally_spun 을 받지 않으면 트윈에서 영원히 멈춘다')
    assert params['externally_spun'].default is False


def test_jgpc_streamed_async_accepts_externally_spun():
    params = inspect.signature(MoveGroupJgpcClient.move_to_joints_streamed_async).parameters
    assert 'externally_spun' in params


# ----- 전달 -----

def test_externally_spun_reaches_joint_completion():
    client = _Client()
    client.plan_joints_async({'panda_joint1': 0.4}, externally_spun=True)
    assert client.seen == [True], (
        '_complete_joint_values 까지 전달되지 않으면 그 안에서 노드를 직접 spin 한다')


def test_default_is_self_spinning():
    """기본값은 종전과 같아야 한다 — 스스로 spin 하는 스크립트 경로가 깨지면 안 된다."""
    client = _Client()
    client.plan_joints_async({'panda_joint1': 0.4})
    assert client.seen == [False]


def test_jgpc_forwards_externally_spun_to_planning():
    """JGPC 의 스트리밍 경로가 계획 단계로 값을 넘기는가."""
    recorded: list[Any] = []

    class _Jgpc(_Client):
        def plan_joints_async(self, joint_values, *, velocity_scaling=None,
                              planning_time=5.0, tolerance=0.01, externally_spun=False):
            from rclpy.task import Future
            recorded.append(externally_spun)
            return Future()

        def _get_streamer(self):
            raise AssertionError('계획이 끝나기 전에는 스트리머를 만들지 않는다')

        move_to_joints_streamed_async = MoveGroupJgpcClient.move_to_joints_streamed_async

    _Jgpc().move_to_joints_streamed_async({'panda_joint1': 0.4}, externally_spun=True)
    assert recorded == [True]
