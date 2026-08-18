"""컨트롤러 종류에 맞는 :class:`MoveGroupClient` 구현을 생성하는 팩토리."""

from __future__ import annotations

from typing import Optional

from rclpy.node import Node

from rdfp.moveit.move_group_client import MoveGroupClient
from rdfp.moveit.move_group_jgpc_client import MoveGroupJgpcClient
from rdfp.moveit.move_group_jtc_client import MoveGroupJtcClient
from rdfp.moveit.trajectory_streamer import DEFAULT_COMMAND_TOPIC

MODE_AUTO = 'auto'
MODE_JTC = 'jtc'
MODE_JGPC = 'jgpc'
CLIENT_MODES = (MODE_AUTO, MODE_JTC, MODE_JGPC)


def detect_controller_mode(node: Node, *,
                           arm_command_topic: str = DEFAULT_COMMAND_TOPIC) -> str:
    """실행 중인 스택의 arm 컨트롤러 종류를 판별한다.

    JGPC(forward command) 스택에서만 ``std_msgs/Float64MultiArray`` 명령
    토픽이 존재하므로 이를 신호로 쓴다. 토픽 그래프만 조회하므로 서비스
    호출이나 대기가 없다.

    Args:
        node: 토픽 그래프를 조회할 ROS2 Node.
        arm_command_topic: JGPC 판별에 쓸 명령 토픽 이름.

    Returns:
        ``'jgpc'`` 또는 ``'jtc'``.

    Note:
        DDS discovery 가 아직 끝나지 않았으면 JGPC 스택이라도 ``'jtc'`` 로
        판별될 수 있다. 노드를 막 생성한 직후에 호출하지 말고, 스택이 뜬 것을
        확인한 뒤 호출한다. 확실히 알고 있다면 ``mode`` 를 명시하는 편이 낫다.
    """
    publishers = node.get_publishers_info_by_topic(arm_command_topic)
    subscribers = node.get_subscriptions_info_by_topic(arm_command_topic)
    return MODE_JGPC if (publishers or subscribers) else MODE_JTC


def create_move_group_client(node: Node, *, mode: str = MODE_AUTO,
                             arm_command_topic: str = DEFAULT_COMMAND_TOPIC,
                             arm_command_joint_names: Optional[list[str]] = None,
                             **kwargs) -> MoveGroupClient:
    """컨트롤러에 맞는 :class:`MoveGroupClient` 구현을 생성한다.

    Args:
        node: 서비스/액션 클라이언트를 올릴 ROS2 Node.
        mode: ``'auto'`` (기본) 이면 :func:`detect_controller_mode` 로 판별한다.
            ``'jtc'`` / ``'jgpc'`` 로 강제할 수 있다.
        arm_command_topic: JGPC 명령 토픽. ``'auto'`` 판별에도 사용된다.
        arm_command_joint_names: JGPC 명령 배열의 joint 순서. ``None`` 이면
            첫 스트리밍 시 컨트롤러 파라미터에서 자동 조회한다.
        **kwargs: 각 클라이언트 생성자에 그대로 전달된다.

    Returns:
        :class:`~rdfp.moveit.move_group_jtc_client.MoveGroupJtcClient` 또는
        :class:`~rdfp.moveit.move_group_jgpc_client.MoveGroupJgpcClient`.

    Raises:
        ValueError: ``mode`` 가 유효하지 않을 때.
    """
    if mode not in CLIENT_MODES:
        raise ValueError(f'mode must be one of {CLIENT_MODES}, got {mode!r}')

    resolved = detect_controller_mode(node, arm_command_topic=arm_command_topic) \
        if mode == MODE_AUTO else mode

    if resolved == MODE_JGPC:
        node.get_logger().info('Using MoveGroupJgpcClient (forward command streaming)')
        return MoveGroupJgpcClient(node, arm_command_topic=arm_command_topic,
                                   arm_command_joint_names=arm_command_joint_names, **kwargs)

    node.get_logger().info('Using MoveGroupJtcClient (MoveGroup/ExecuteTrajectory action)')
    return MoveGroupJtcClient(node, **kwargs)
