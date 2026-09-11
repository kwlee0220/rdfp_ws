"""컨트롤러 종류에 맞는 :class:`MoveGroupClient` 구현을 생성하는 팩토리."""

from __future__ import annotations

from typing import Optional

from rclpy.node import Node

from robot_control.backend_profiles import (
    BACKEND_PROFILES, backends, resolve_backend_channel)
from robot_control.moveit.move_group_client import MoveGroupClient
from robot_control.moveit.move_group_jgpc_client import MoveGroupJgpcClient
from robot_control.moveit.move_group_jtc_client import MoveGroupJtcClient
from robot_control.moveit.trajectory_streamer import DEFAULT_COMMAND_TOPIC

MODE_AUTO = 'auto'
MODE_JTC = 'jtc'
MODE_JGPC = 'jgpc'
CLIENT_MODES = (MODE_AUTO, MODE_JTC, MODE_JGPC)

# 액션 서버 존재 확인에 쓰는 숨은 상태 토픽 (아래 `_jtc_server_present` 참조).
_JTC_ACTION_NAME = '/panda_arm_controller/follow_joint_trajectory'


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


def jtc_action_server_present(node: Node,
                              action_name: str = _JTC_ACTION_NAME) -> bool:
    """``FollowJointTrajectory`` 액션 **서버**가 떠 있는가.

    액션 서버는 내부적으로 ``<action>/_action/status`` 를 발행하므로 그 발행자 수로
    판단한다. **`ros2 action list` 에 이름이 보이는 것만으로는 부족하다** — 클라이언트만
    있어도 목록에 나오고, `moveit_simple_controller_manager` 가 정확히 그 경우다.
    숨은 토픽이라 `get_topic_names_and_types()` 에도 안 나온다.
    """
    try:
        return bool(node.get_publishers_info_by_topic(f'{action_name}/_action/status'))
    except Exception:       # noqa: BLE001 — 그래프 조회 실패는 판정 불가일 뿐이다
        return False


def create_move_group_client(node: Node, *, backend: Optional[str] = None,
                             mode: str = MODE_AUTO,
                             arm_command_topic: str = DEFAULT_COMMAND_TOPIC,
                             arm_command_format: Optional[str] = None,
                             arm_command_joint_names: Optional[list[str]] = None,
                             velocity_scaling: Optional[float] = None,
                             check_action_server: bool = True,
                             **kwargs) -> MoveGroupClient:
    """컨트롤러에 맞는 :class:`MoveGroupClient` 구현을 생성한다.

    **평소에는 `backend` 하나만 준다.** 나머지 넷은 :data:`BACKEND_PROFILES` 에 없는
    스택을 붙이거나 프로파일의 한 값만 덮어쓸 때 쓰는 탈출구다::

        client = create_move_group_client(node, backend='isaac')

    Args:
        node: 서비스/액션 클라이언트를 올릴 ROS2 Node.
        backend: :data:`BACKEND_PROFILES` 의 키 (``'isaac'`` / ``'functionbay'`` /
            ``'mock'`` / ``'mock_jgpc'``). 주면 아래 네 인자의 기본값을 채운다.
            **``'auto'`` 라는 백엔드는 없다** — 프로파일은 모두 확정 모드다. 붙어 있는
            스택에 맞추려면 ``backend`` 없이 ``mode='auto'`` 를 쓴다.
        mode: ``'auto'`` (기본) 이면 :func:`detect_controller_mode` 로 판별한다.
            ``'jtc'`` / ``'jgpc'`` 로 강제할 수 있다.
        arm_command_topic: JGPC 명령 토픽. ``'auto'`` 판별에도 사용된다.
        arm_command_format: **JGPC 전용.** JTC 생성자는 이 인자를 모르므로 JGPC 로
            결정됐을 때만 전달된다.
        arm_command_joint_names: JGPC 명령 배열의 joint 순서. ``None`` 이면
            첫 스트리밍 시 컨트롤러 파라미터에서 자동 조회한다.
        velocity_scaling: 팔 이동의 **전역 기본** 속도 배율. ``None`` 이면 프로파일의
            ``motion.velocity_scaling`` 을, 그것도 없으면 코드 기본값(1.0)을 쓴다.
            우선순위는 **인자 > 프로파일 > 코드 기본값** 이다. 각 이동 메서드의
            ``velocity_scaling`` 인자는 이것보다도 우선한다 — 이 값은 그 메서드들이
            아무것도 안 받았을 때의 바닥값이다.
        gravity_compensator: **JGPC 전용.** ``None`` (기본) 이면 프로파일의
            ``motion.gravity_compensation`` 으로 자동 구성한다. 그 키가 없으면 보상하지
            않는다. 명시로 주면 프로파일보다 우선한다.
        check_action_server: JTC 로 결정됐을 때 액션 서버 유무를 확인해 없으면
            ERROR 를 낸다. **스택보다 먼저 뜨는 호출자는 꺼야 한다** — 그때는 서버가
            없는 것이 정상이라 매 기동마다 오경보가 되고, 그런 로그는 사람을 길들여
            진짜 오류까지 흘려보게 만든다 (`robot_twin` 이 그 경우다).
        **kwargs: 각 클라이언트 생성자에 그대로 전달된다.

    Returns:
        :class:`~robot_control.moveit.move_group_jtc_client.MoveGroupJtcClient` 또는
        :class:`~robot_control.moveit.move_group_jgpc_client.MoveGroupJgpcClient`.

    Raises:
        ValueError: ``mode`` 가 유효하지 않을 때.
    """
    if backend is not None:
        channel = resolve_backend_channel(
            backend, arm_command_mode=(None if mode == MODE_AUTO else mode),
            arm_command_topic=(None if arm_command_topic == DEFAULT_COMMAND_TOPIC
                               else arm_command_topic),
            arm_command_format=arm_command_format,
            arm_command_joint_names=arm_command_joint_names)
        mode = channel['arm_command_mode']
        arm_command_topic = channel['arm_command_topic']
        arm_command_format = channel['arm_command_format']
        arm_command_joint_names = channel['arm_command_joint_names'] or None
        if velocity_scaling is None:
            # **명시 인자가 이긴다.** 프로파일 값은 아무것도 안 줬을 때만 쓴다.
            profile_motion = BACKEND_PROFILES[backend].get('motion') or {}
            profile_scaling = profile_motion.get('velocity_scaling')
            if profile_scaling is not None:
                velocity_scaling = float(profile_scaling)
        # **명령 스트리밍 주기도 프로파일에서 온다** (JGPC 전용). 안 채우면 데카르트
        # 궤적이 10 Hz 계단으로 나가고, 접촉을 동반하는 작업에서 그것이 실제로 실패를
        # 만든다 — 아래 JGPC 분기에서만 쓰인다.
        profile_rate = BACKEND_PROFILES[backend].get('arm_command_publish_rate')
        if profile_rate is not None and 'publish_rate' not in kwargs:
            kwargs['publish_rate'] = float(profile_rate)

    if velocity_scaling is not None:
        # `None` 은 넘기지 않는다 — 클라이언트 생성자 기본값(1.0)이 살아야 한다.
        kwargs['velocity_scaling'] = velocity_scaling

    if mode not in CLIENT_MODES:
        raise ValueError(f'mode must be one of {CLIENT_MODES}, got {mode!r}')

    resolved = detect_controller_mode(node, arm_command_topic=arm_command_topic) \
        if mode == MODE_AUTO else mode

    if resolved == MODE_JGPC:
        node.get_logger().info('Using MoveGroupJgpcClient (forward command streaming)')
        # `arm_command_format` 은 JGPC 전용이라 여기서만 넘긴다.
        if arm_command_format:
            kwargs['arm_command_format'] = arm_command_format
        # **중력 보상도 JGPC 전용이다** — 스트리밍 경로에만 얹을 수 있기 때문이다.
        # JTC 백엔드는 컨트롤러가 중력을 스스로 잡으므로 프로파일에 키가 없다.
        if backend and 'gravity_compensator' not in kwargs:
            from robot_control.gravity import load_gravity_compensator

            compensator = load_gravity_compensator(backend)
            if compensator is not None:
                node.get_logger().info(
                    f'Gravity compensation enabled: Kp={compensator.kp[0]:g}, '
                    f'{len(compensator.joint_names)} joints ({compensator.path})')
                kwargs['gravity_compensator'] = compensator
        return MoveGroupJgpcClient(node, arm_command_topic=arm_command_topic,
                                   arm_command_joint_names=arm_command_joint_names, **kwargs)

    # **JTC 인데 액션 서버가 없으면 기동 시점에 알린다.** 그 조합은 계획은 되고
    # 실행만 조용히 안 되어 "명령이 안 먹는다"로만 보인다 — 원인을 안 가리킨다.
    if check_action_server and not jtc_action_server_present(node):
        node.get_logger().error(
            f"arm command path resolved to JTC but no FollowJointTrajectory action server "
            f"is running on '{_JTC_ACTION_NAME}' — motions will plan but never execute "
            f"(MoveGroup returns CONTROL_FAILED, -4). Pass an explicit backend: "
            f"one of {list(backends())}.")

    # `publish_rate` 는 **JGPC 전용**이다 — JTC 는 컨트롤러가 보간하므로 개념이 없고,
    # 그 생성자는 이 인자를 몰라 `TypeError` 가 난다.
    kwargs.pop('publish_rate', None)
    node.get_logger().info('Using MoveGroupJtcClient (MoveGroup/ExecuteTrajectory action)')
    return MoveGroupJtcClient(node, **kwargs)
