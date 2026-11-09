"""`follow_trajectory_async` 는 계획 비율(`fraction`)을 결과에 싣는다.

예전에는 계획 콜백에서 `fraction` 을 판정·로그에만 쓰고 버려, JTC 는 `None`, JGPC 는
발행 개수로 끝났다. 트윈은 결과의 `fraction` 을 `outputs` 에 담도록 짜여 있었지만
**한 번도 실리지 않았고**, 호출자는 도달 실패가 부분 경로(기하 문제) 때문인지 정착·처짐
(실행 문제) 때문인지 가를 수 없었다.

콜백 체인은 ROS 노드 없이 태운다 — 미리 끝내 둔 `Future` 는 executor 가 없으면
`add_done_callback` 이 즉시 부르므로 체인이 동기적으로 끝까지 돈다.
"""

from __future__ import annotations

from typing import Any

from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('moveit_msgs')

from geometry_msgs.msg import Pose  # noqa: E402
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory  # noqa: E402
from rclpy.task import Future  # noqa: E402

from robot_control.moveit.move_group_client import CartesianPathResult  # noqa: E402
from robot_control.moveit.move_group_jgpc_client import MoveGroupJgpcClient  # noqa: E402
from robot_control.moveit.move_group_jtc_client import MoveGroupJtcClient  # noqa: E402


class _Logger:
    def info(self, msg: str) -> None:
        pass

    def warning(self, msg: str) -> None:
        pass

    def error(self, msg: str) -> None:
        pass


def _done(value: Any) -> Future:
    future = Future()
    future.set_result(value)
    return future


def _plan_response(fraction: float) -> SimpleNamespace:
    return SimpleNamespace(error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
                           fraction=fraction, solution=RobotTrajectory())


def _base(cls: type, fraction: float) -> Any:
    """체인이 건드리는 속성만 채운 클라이언트. 계획 응답은 `fraction` 으로 고정한다."""
    client = object.__new__(cls)
    client._closed = False
    client._default_velocity_scaling = 1.0
    client._default_fraction_threshold = 0.6
    client._node = SimpleNamespace(get_logger=_Logger)
    client.plan_trajectory_async = lambda waypoints, **kwargs: _done(_plan_response(fraction))
    return client


def _jtc(fraction: float) -> MoveGroupJtcClient:
    client = _base(MoveGroupJtcClient, fraction)
    executed = SimpleNamespace(result=SimpleNamespace(
        error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS)))
    goal_handle = SimpleNamespace(accepted=True, get_result_async=lambda: _done(executed))
    client._trajectory_follower = SimpleNamespace(send_goal_async=lambda goal: _done(goal_handle))
    client._begin_motion = lambda *args, **kwargs: object()
    client._finish_motion = lambda *args, **kwargs: None
    client._track_goal = lambda goal_handle: None
    client._on_done = lambda goal_handle, callback: callback
    return client


def _jgpc(fraction: float, *, published: int) -> MoveGroupJgpcClient:
    client = _base(MoveGroupJgpcClient, fraction)
    client._tip_frame = 'panda_link8'
    client._rate = lambda publish_rate: publish_rate
    client._get_streamer = lambda: SimpleNamespace(
        stream_async=lambda trajectory, **kwargs: _done(published))
    return client


@pytest.mark.parametrize('fraction', [1.0, 0.75])
def test_jtc_result_carries_the_planned_fraction(fraction: float) -> None:
    future = _jtc(fraction).follow_trajectory_async([Pose()])

    assert future.result() == CartesianPathResult(fraction=fraction)


@pytest.mark.parametrize('fraction', [1.0, 0.75])
def test_jgpc_result_carries_fraction_and_published_count(fraction: float) -> None:
    """발행 개수는 예전 반환값이었다 — 버리지 않고 `published` 로 남긴다."""
    future = _jgpc(fraction, published=42).follow_trajectory_async([Pose()])

    assert future.result() == CartesianPathResult(fraction=fraction, published=42)
