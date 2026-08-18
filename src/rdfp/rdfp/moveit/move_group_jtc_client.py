from __future__ import annotations

from typing import Optional

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.client import Client
from rclpy.node import Node
from rclpy.task import Future
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory

from rdfp.moveit.move_group_client import (
    DEFAULT_JOINT_TOLERANCE,
    DEFAULT_PLANNING_TIME,
    GOAL_ACCEPT_TIMEOUT_SEC,
    MOVE_GROUP_TIMEOUT_SEC,
    PLAN_TIMEOUT_SEC,
    READY_TIMEOUT_SEC,
    TRAJECTORY_EXEC_TIMEOUT_SEC,
    MoveGroupClient,
    _EXECUTE_TRAJECTORY_ACTION,
    _JOINT_GOAL_LABEL,
    _build_move_group_goal,
    _validate_joint_values,
    _validate_trajectory,
)
from rdfp.ros2_utils import await_future_spin, check_timeout_and_get_remains


# 궤적을 실제로 실행하는 컨트롤러의 액션. cancel() 이 여기에 취소를 보낸다.
_CONTROLLER_ACTION = '/panda_arm_controller/follow_joint_trajectory'


class MoveGroupJtcClient(MoveGroupClient):
    """``JointTrajectoryController`` 환경용 :class:`MoveGroupClient` 구현.

    실행을 MoveIt 에 맡긴다.

    * named target 이동: ``MoveGroup`` 액션 한 번으로 계획 + 실행을 모두
      수행한다 (``plan_only=False``).
    * 카테시안 경로: ``GetCartesianPath`` 로 계획한 뒤 ``ExecuteTrajectory``
      액션으로 실행한다.

    두 경로 모두 최종적으로 ``moveit_simple_controller_manager`` 를 거쳐
    ``FollowJointTrajectory`` 액션을 쓴다. 그 액션을 제공하지 않는 컨트롤러
    (``position_controllers/JointGroupPositionController`` 등) 환경에서는
    동작하지 않으므로, 그때는
    :class:`~rdfp.moveit.move_group_jgpc_client.MoveGroupJgpcClient` 를 쓴다.

    실행 결과가 컨트롤러의 goal tolerance 검사를 거친 ``MoveItErrorCodes`` 로
    돌아오므로 **정상 반환은 목표 도달을 뜻한다**.
    """

    def __init__(self, node: Node, *,
                 execute_trajectory_action: str = _EXECUTE_TRAJECTORY_ACTION,
                 controller_action: str = _CONTROLLER_ACTION,
                 **kwargs) -> None:
        """JTC 클라이언트를 초기화한다.

        Args:
            node: 서비스/액션 클라이언트를 올릴 ROS2 Node.
            execute_trajectory_action: ``ExecuteTrajectory`` 액션 이름.
            controller_action: 궤적을 실제로 실행하는 컨트롤러의
                ``FollowJointTrajectory`` 액션 이름. :meth:`cancel` 이 여기에
                취소를 보낸다.
            **kwargs: :class:`MoveGroupClient` 생성자 인자.
        """
        super().__init__(node, **kwargs)
        self._execute_trajectory_action = execute_trajectory_action
        # 컨트롤러 액션의 CancelGoal 서비스. goal 핸들을 소유하지 않아도 취소를
        # 보낼 수 있는 유일한 경로다 (:meth:`cancel` 참조).
        self._controller_cancel_service = f'{controller_action}/_action/cancel_goal'
        self._controller_cancel_client: Client = node.create_client(
            CancelGoal, self._controller_cancel_service,
            callback_group=self._action_callback_group
        )
        # base 와 같은 ReentrantCallbackGroup 을 쓴다 — cancel 응답이 결과 대기에
        # 막히지 않도록 하기 위함이다 (base __init__ 주석 참조).
        self._trajectory_follower: ActionClient = ActionClient(
            node, ExecuteTrajectory, execute_trajectory_action,
            callback_group=self._action_callback_group
        )

    # ----- 중단 ------------------------------------------------------------

    def cancel(self) -> bool:
        """진행 중인 동작을 중단한다 (:meth:`MoveGroupClient.cancel` 구현).

        **컨트롤러의 ``FollowJointTrajectory`` 액션에 취소를 보낸다.** ``MoveGroup``
        액션 goal 취소만으로는 로봇이 멈추지 않기 때문이다 — ``move_group`` 이 goal
        을 실행 중일 때 ``CancelGoal`` 요청에 응답하지 않는 것으로 관측되었다
        (mock/Gazebo 무관, 실측).

        궤적을 실제로 실행하는 주체는 컨트롤러이므로 거기서 끊는 것이 확실하다.
        goal 핸들을 소유하지 않아도, ROS 2 action 규격상 ``goal_id`` 와 ``stamp`` 가
        모두 0 인 ``CancelGoal`` 요청은 **해당 서버의 모든 goal 취소**를 뜻한다.
        ``moveit_simple_controller_manager`` 가 소유한 goal 도 이 방식으로 취소된다.

        **주의**: 이 클라이언트가 시작하지 않은 궤적도 함께 취소된다. 한 팔을 여러
        주체가 동시에 지휘하지 않는다는 전제 위에서 안전한 동작이다.

        추적 중인 ``MoveGroup`` / ``ExecuteTrajectory`` goal 에도 취소를 보내
        상위 액션이 결과를 정리하게 한다.

        Returns:
            컨트롤러에 취소 요청을 보냈으면 ``True``. 서비스가 없으면 ``False``.
            **비동기다** — ``True`` 는 요청 전송을 뜻하며, 로봇은 감속 후 멈춘다.
        """
        # 상위 액션 goal 취소 (결과 정리용). 이것만으로는 로봇이 멈추지 않는다.
        super().cancel()

        if not self._controller_cancel_client.service_is_ready():
            self._node.get_logger().error(
                f'controller cancel service not available: {self._controller_cancel_service}'
            )
            return False

        # goal_id / stamp 를 채우지 않으면 "모든 goal 취소" 요청이 된다.
        self._controller_cancel_client.call_async(CancelGoal.Request())
        self._node.get_logger().info(
            f'cancel sent to controller action ({self._controller_cancel_service})'
        )
        return True

    # ----- Lifecycle ------------------------------------------------------

    def is_ready(self) -> bool:
        """계획용 서버에 더해 ``ExecuteTrajectory`` 액션 서버까지 확인한다."""
        return super().is_ready() and self._trajectory_follower.server_is_ready()

    def wait_until_ready(self, timeout_sec: float = READY_TIMEOUT_SEC) -> None:
        """계획용 서버 + ``ExecuteTrajectory`` 액션 서버를 순차 대기한다."""
        self._require_open()
        if timeout_sec <= 0:
            raise ValueError('timeout_sec must be positive')
        remains = self._wait_for_planning_servers(timeout_sec)
        if not self._trajectory_follower.wait_for_server(timeout_sec=remains):
            raise TimeoutError(
                f'Timed out waiting for {self._execute_trajectory_action} action server: '
                f'timeout={timeout_sec}s'
            )

    def _on_done(self, goal_handle, callback):
        """결과 콜백을 감싸 goal 추적을 해제한 뒤 원래 콜백을 호출한다.

        추적 해제가 콜백 예외에 가려지지 않도록 ``finally`` 로 보장한다.
        """
        def _wrapped(future) -> None:
            try:
                self._untrack_goal(goal_handle)
            finally:
                callback(future)

        return _wrapped

    def _close_resources(self) -> None:
        try:
            self._node.destroy_client(self._controller_cancel_client)
        except Exception:
            pass
        """공통 리소스에 더해 ``ExecuteTrajectory`` 액션 클라이언트를 정리한다."""
        try:
            self._trajectory_follower.destroy()
        except Exception:
            pass
        super()._close_resources()

    # ----- 실행 API 구현 ----------------------------------------------------

    def move_to_named_target(self, name: str, *,
                             velocity_scaling: Optional[float] = None,
                             planning_time: float = DEFAULT_PLANNING_TIME,
                             tolerance: float = DEFAULT_JOINT_TOLERANCE,
                             timeout: float = MOVE_GROUP_TIMEOUT_SEC,) -> None:
        """SRDF에 등록된 named target(group_state)으로 로봇을 이동시킨다.

        MoveGroup 액션을 사용해 joint-space 경로로 계획 및 실행한다. Cartesian
        경로가 아니라 관절 공간 경로이므로 :meth:`follow_trajectory` 와는
        다른 동작임에 주의. 첫 호출 시 ``move_group`` 노드에서 SRDF 를
        조회하여 group_state → joint 값 매핑을 캐시한다.

        Args:
            name: SRDF의 group_state 이름 (예: "ready", "extended", "transport").
            velocity_scaling: 속도 스케일링 계수. ``None`` 이면 생성자 기본값 사용.
            planning_time: MoveGroup 계획 허용 시간(초).
            tolerance: 각 관절 목표의 허용 오차(라디안).
            timeout: SRDF 조회 + 계획 + 실행 전체에 허용할 최대 시간(초).

        Raises:
            ValueError: ``name`` 이 유효하지 않거나 SRDF에 존재하지 않을 때.
            TimeoutError: SRDF 조회, goal 수락, 또는 실행 완료가 시간 초과될 때.
            RuntimeError: 계획/실행이 실패했거나 클라이언트가 close() 되었을 때.
        """
        self._require_open()
        if not isinstance(name, str) or not name:
            raise ValueError('name must be a non-empty string')

        vs = self._resolve_velocity_scaling(velocity_scaling)

        started_at = time.monotonic()

        # SRDF에서 대상 group_state의 joint 값 조회 (최초 1회만 실제 서비스 호출)
        remains = check_timeout_and_get_remains(started_at, timeout)
        joint_values = self._get_named_state_joint_values(name, timeout=remains)

        self._execute_joint_goal(
            joint_values, velocity_scaling=vs, planning_time=planning_time,
            tolerance=tolerance, started_at=started_at, timeout=timeout,
            what=f'named target "{name}"'
        )

    def _execute_joint_goal(self, joint_values: dict[str, float], *,
                            velocity_scaling: float, planning_time: float,
                            tolerance: float, started_at: float, timeout: float,
                            what: str) -> None:
        """joint 값 목표를 MoveGroup 액션으로 계획·실행한다(블로킹).

        ``move_to_named_target`` (SRDF 조회 후) 과 ``move_to_joints`` (값 직접 지정)
        가 공유하는 실행 체인이다. ``what`` 은 로그·오류 메시지에 쓰는 목표 설명이다.

        Args:
            started_at: 호출자가 ``time.monotonic()`` 으로 기록한 시작 시각. 앞선
                단계(SRDF 조회 등)에서 쓴 시간을 ``timeout`` 에 포함시키기 위해
                호출자가 넘긴다.
        """
        self._node.get_logger().info(
            f'Moving to {what} in group "{self._moveit_group_name}"...'
        )

        goal_msg = _build_move_group_goal(
            group_name=self._moveit_group_name, joint_values=joint_values,
            velocity_scaling=velocity_scaling, planning_time=planning_time,
            tolerance=tolerance,
        )

        # MoveGroup 액션 서버 준비 대기
        remains = check_timeout_and_get_remains(started_at, timeout)
        if not self._move_group_client.wait_for_server(timeout_sec=min(remains, 10.0)):
            raise TimeoutError(
                f'Timed out waiting for {self._move_group_action} action server'
            )

        # goal 전송 및 수락 대기
        send_future = self._move_group_client.send_goal_async(goal_msg)
        remains = check_timeout_and_get_remains(started_at, timeout)
        goal_handle = await_future_spin(
            self._node, send_future, remains, 'move_group goal response'
        )
        if not goal_handle.accepted:
            self._node.get_logger().error('Move group goal rejected')
            raise RuntimeError('Move group goal rejected')
        self._node.get_logger().info('Move group goal accepted!')
        # cancel() 이 취소를 보낼 수 있도록 추적한다.
        self._track_goal(goal_handle)

        # 결과 대기
        try:
            result_future = goal_handle.get_result_async()
            remains = check_timeout_and_get_remains(started_at, timeout)
            result_response = await_future_spin(
                self._node, result_future, remains, 'move_group result'
            )
        finally:
            self._untrack_goal(goal_handle)

        error_code = result_response.result.error_code.val
        self._node.get_logger().info(
            f'Move group execution result error code: {error_code}'
        )
        if error_code != MoveItErrorCodes.SUCCESS:
            self._node.get_logger().error(
                f'Move to {what} failed with code: {error_code}'
            )
            raise RuntimeError(
                f'Move to {what} failed with code: {error_code}'
            )
        self._node.get_logger().info(f'Reached {what}')

    def move_to_named_target_async(self, name: str, *,
                                   velocity_scaling: Optional[float] = None,
                                   planning_time: float = DEFAULT_PLANNING_TIME,
                                   tolerance: float = DEFAULT_JOINT_TOLERANCE,
                                   externally_spun: bool = False) -> Future:
        """SRDF의 named target으로 로봇을 비동기 이동시킨다.

        캐시가 비어 있으면 ``get_parameters`` 서비스 호출(SRDF 조회) →
        파싱/캐시 → MoveGroup ``send_goal_async`` → goal 수락 콜백 →
        ``get_result_async`` → 실행 완료 콜백 순으로 체인을 구성한다.

        Args:
            name: SRDF의 group_state 이름.
            velocity_scaling: 속도 스케일링 계수. ``None`` 이면 생성자 기본값 사용.
            planning_time: MoveGroup 계획 허용 시간(초).
            tolerance: 각 관절 목표의 허용 오차(라디안).

        Returns:
            최종 실행 결과를 담는 ``Future``. 성공 시 ``None`` 으로 resolve
            되고, 실패 시 적절한 예외가 설정된다.

        Raises:
            ValueError: ``name`` 이 유효하지 않을 때(즉시 발생).
            RuntimeError: 클라이언트가 이미 close() 되었을 때(즉시 발생).
        """
        self._require_open()
        if not isinstance(name, str) or not name:
            raise ValueError('name must be a non-empty string')
        vs = self._resolve_velocity_scaling(velocity_scaling)

        result_future = Future()
        what = f'named target "{name}"'

        def _start_move(joint_values: dict[str, float]) -> None:
            """joint 값 확보 후 MoveGroup goal 전송 단계로 진입."""
            self._execute_joint_goal_async(
                joint_values, velocity_scaling=vs, planning_time=planning_time,
                tolerance=tolerance, what=what, result_future=result_future
            )

        # SRDF 캐시가 있으면 바로 이동, 없으면 비동기 조회 후 이동
        if self._group_states is not None:
            try:
                joint_values = self._lookup_cached_named_state(name)
            except ValueError as exc:
                result_future.set_exception(exc)
                return result_future
            _start_move(joint_values)
            return result_future

        srdf_future, srdf_client = self._fetch_srdf_async()

        def _on_srdf_done(future: Future) -> None:
            """SRDF 조회 완료 콜백: 캐시 채우고 이동 단계로 진입."""
            try:
                exc = future.exception()
                if exc is not None:
                    result_future.set_exception(exc)
                    return
                response = future.result()
                if response is None or not response.values:
                    result_future.set_exception(RuntimeError(
                        'Failed to retrieve robot_description_semantic'
                    ))
                    return
                srdf_string = response.values[0].string_value
                if not srdf_string:
                    result_future.set_exception(RuntimeError(
                        'robot_description_semantic is empty'
                    ))
                    return
                self._store_srdf_cache(srdf_string)
                joint_values = self._lookup_cached_named_state(name)
                _start_move(joint_values)
            except Exception as exc:
                result_future.set_exception(exc)
            finally:
                # 임시 생성된 get_parameters 클라이언트 정리
                try:
                    self._node.destroy_client(srdf_client)
                except Exception:
                    pass

        srdf_future.add_done_callback(_on_srdf_done)
        return result_future

    def _execute_joint_goal_async(self, joint_values: dict[str, float], *,
                                  velocity_scaling: float, planning_time: float,
                                  tolerance: float, what: str,
                                  result_future: Optional[Future] = None) -> Future:
        """joint 값 목표를 MoveGroup 액션으로 계획·실행한다(비동기).

        ``move_to_named_target_async`` 와 ``move_to_joints_async`` 가 공유하는
        콜백 체인이다. Node 를 직접 spin 하지 않으므로 executor 가 다른 스레드에서
        spin 중인 환경에서 쓸 수 있다.

        Args:
            joint_values: ``{관절이름: 라디안}``.
            velocity_scaling: 이미 해석된 속도 스케일링 계수.
            planning_time: MoveGroup 계획 허용 시간(초).
            tolerance: 각 관절 목표의 허용 오차(라디안).
            what: 로그·오류 메시지에 쓰는 목표 설명.
            result_future: 채울 ``Future``. named target 은 SRDF 조회를 먼저 해야
                해서 호출자에게 Future 를 미리 돌려주므로 그것을 넘긴다.
                ``None`` 이면 새로 만든다.

        Returns:
            성공 시 ``None`` 으로 resolve 되는 ``Future``.
        """
        fut = Future() if result_future is None else result_future

        def _on_goal_response(future: Future) -> None:
            """MoveGroup goal 수락 콜백."""
            try:
                exc = future.exception()
                if exc is not None:
                    fut.set_exception(exc)
                    return
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self._node.get_logger().error('Move group goal rejected')
                    fut.set_exception(RuntimeError('Move group goal rejected'))
                    return
                self._node.get_logger().info('Move group goal accepted!')
                self._track_goal(goal_handle)
                get_result_future = goal_handle.get_result_async()
                get_result_future.add_done_callback(self._on_done(goal_handle, _on_execute_done))
            except Exception as exc:
                fut.set_exception(exc)

        def _on_execute_done(future: Future) -> None:
            """MoveGroup 실행 완료 콜백: 최종 결과를 Future 에 설정."""
            try:
                exc = future.exception()
                if exc is not None:
                    fut.set_exception(exc)
                    return
                result_response = future.result()
                error_code = result_response.result.error_code.val
                self._node.get_logger().info(
                    f'Move group execution result error code: {error_code}'
                )
                if error_code != MoveItErrorCodes.SUCCESS:
                    self._node.get_logger().error(
                        f'Move to {what} failed with code: {error_code}'
                    )
                    fut.set_exception(RuntimeError(
                        f'Move to {what} failed with code: {error_code}'
                    ))
                    return
                self._node.get_logger().info(f'Reached {what}')
                fut.set_result(None)
            except Exception as exc:
                fut.set_exception(exc)

        try:
            goal_msg = _build_move_group_goal(
                group_name=self._moveit_group_name, joint_values=joint_values,
                velocity_scaling=velocity_scaling, planning_time=planning_time,
                tolerance=tolerance,
            )
            self._node.get_logger().info(f'Moving to {what} (async)...')
            send_future = self._move_group_client.send_goal_async(goal_msg)
            send_future.add_done_callback(_on_goal_response)
        except Exception as exc:
            fut.set_exception(exc)
        return fut

    def move_to_joints(self, joint_values: dict[str, float], *,
                       velocity_scaling: Optional[float] = None,
                       planning_time: float = DEFAULT_PLANNING_TIME,
                       tolerance: float = DEFAULT_JOINT_TOLERANCE,
                       timeout: float = MOVE_GROUP_TIMEOUT_SEC) -> None:
        """관절 목표값으로 이동한다 — MoveGroup 액션으로 계획·실행(블로킹).

        SRDF 조회가 없다는 점만 빼면 :meth:`move_to_named_target` 과 같은 경로다.
        JTC 스택이므로 컨트롤러가 목표까지 추종하며, 정상 반환은 **도달을
        의미한다**.
        """
        self._require_open()
        values = _validate_joint_values(joint_values)
        vs = self._resolve_velocity_scaling(velocity_scaling)

        self._execute_joint_goal(
            values, velocity_scaling=vs, planning_time=planning_time,
            tolerance=tolerance, started_at=time.monotonic(), timeout=timeout,
            what=_JOINT_GOAL_LABEL
        )

    def move_to_joints_async(self, joint_values: dict[str, float], *,
                             velocity_scaling: Optional[float] = None,
                             planning_time: float = DEFAULT_PLANNING_TIME,
                             tolerance: float = DEFAULT_JOINT_TOLERANCE,
                             externally_spun: bool = False) -> Future:
        """:meth:`move_to_joints` 의 비동기 버전.

        ``externally_spun`` 은 이 구현에서 의미가 없다 — 콜백 체인만 쓰고 Node 를
        직접 spin 하지 않으므로 어느 쪽이든 동작이 같다.

        Returns:
            성공 시 ``None`` 으로 resolve 되는 ``Future``.
        """
        self._require_open()
        values = _validate_joint_values(joint_values)
        vs = self._resolve_velocity_scaling(velocity_scaling)

        return self._execute_joint_goal_async(
            values, velocity_scaling=vs, planning_time=planning_time,
            tolerance=tolerance, what=_JOINT_GOAL_LABEL
        )

    def follow_trajectory(self, waypoints: list[Pose], *,
                          velocity_scaling: Optional[float] = None,
                          fraction_threshold: Optional[float] = None,
                          max_step: Optional[float] = None,
                          jump_threshold: Optional[float] = None,
                          timeout: float = PLAN_TIMEOUT_SEC + TRAJECTORY_EXEC_TIMEOUT_SEC,) -> None:
        """waypoint 경로를 계획하고 실행한다(원스톱).

        Args:
            waypoints: Cartesian path 를 따라갈 Pose 리스트.
            velocity_scaling: Override. ``None`` 이면 생성자 기본값 사용.
            fraction_threshold: Override. ``None`` 이면 생성자 기본값 사용.
            max_step: Override. ``None`` 이면 생성자 기본값 사용.
            jump_threshold: Override. ``None`` 이면 생성자 기본값 사용.
            timeout: 계획 + 실행 전체에 허용할 최대 시간(초).

        Raises:
            ValueError, TimeoutError, RuntimeError: :meth:`plan_trajectory` 및
                :meth:`execute_trajectory` 와 동일.
        """
        self._require_open()
        self._node.get_logger().info(f'Moving through {len(waypoints)} cartesian waypoints...')

        started_at = time.monotonic()
        trajectory = self.plan_trajectory(
            waypoints,
            velocity_scaling=velocity_scaling,
            fraction_threshold=fraction_threshold,
            max_step=max_step,
            jump_threshold=jump_threshold,
            timeout=timeout,
        )
        remains = check_timeout_and_get_remains(started_at, timeout)
        self.execute_trajectory(trajectory, timeout=remains)

    def follow_trajectory_async(self, waypoints: list[Pose], *,
                                velocity_scaling: Optional[float] = None,
                                fraction_threshold: Optional[float] = None,
                                max_step: Optional[float] = None,
                                jump_threshold: Optional[float] = None,
                                externally_spun: bool = False) -> Future:
        """waypoint 경로를 비동기로 계획하고 실행한다.

        ``plan_trajectory_async`` → 계획 완료 콜백 → ``send_goal_async`` →
        goal 수락 콜백 → ``get_result_async`` → 실행 완료 콜백 순으로
        콜백 체인을 구성하며, 최종 결과를 반환된 ``Future`` 에 설정한다.

        Args:
            waypoints: Cartesian path 를 따라갈 Pose 리스트.
            velocity_scaling: Override. ``None`` 이면 생성자 기본값 사용.
            fraction_threshold: Override. ``None`` 이면 생성자 기본값 사용.
            max_step: Override. ``None`` 이면 생성자 기본값 사용.
            jump_threshold: Override. ``None`` 이면 생성자 기본값 사용.

        Returns:
            최종 실행 결과를 담는 ``Future``. 성공 시 ``None`` 으로 resolve
            되고, 실패 시 ``RuntimeError`` 가 설정된다.

        Raises:
            ValueError: 입력 매개변수가 유효하지 않을 때(즉시 발생).
            RuntimeError: 클라이언트가 이미 close() 되었을 때(즉시 발생).
        """
        self._require_open()
        vs = self._resolve_velocity_scaling(velocity_scaling)
        ft = self._resolve_fraction_threshold(fraction_threshold)

        self._node.get_logger().info(
            f'Moving through {len(waypoints)} cartesian waypoints (async)...')

        result_future = Future()
        plan_future = self.plan_trajectory_async(waypoints, max_step=max_step,
                                                 jump_threshold=jump_threshold)

        def _on_plan_done(future: Future) -> None:
            """계획 서비스 응답 콜백: 결과 검증 후 실행 goal 전송."""
            try:
                exc = future.exception()
                if exc is not None:
                    result_future.set_exception(exc)
                    return

                response = future.result()
                if response.error_code.val != MoveItErrorCodes.SUCCESS:
                    self._node.get_logger().error(
                        f'Path planning returned error code: {response.error_code.val}'
                    )
                    result_future.set_exception(RuntimeError(
                        f'Path planning failed with error code: {response.error_code.val}'
                    ))
                    return

                if response.fraction < ft:
                    self._node.get_logger().error(
                        f'Path planning failed (fraction: {response.fraction})')
                    result_future.set_exception(RuntimeError(
                        f'Path planning failed: only '
                        f'{response.fraction * 100:.1f}% of the path was planned'
                    ))
                    return

                self._node.get_logger().info(
                    f'Path planning successful (fraction: {response.fraction})')
                trajectory = response.solution
                if vs != 1.0:
                    trajectory = self.scale_trajectory_velocity(trajectory, vs)

                goal_msg = ExecuteTrajectory.Goal(trajectory=trajectory)
                goal_future = self._trajectory_follower.send_goal_async(goal_msg)
                goal_future.add_done_callback(_on_goal_response)
            except Exception as exc:
                result_future.set_exception(exc)

        def _on_goal_response(future: Future) -> None:
            """goal 수락 콜백: 수락 여부 확인 후 결과 대기."""
            try:
                exc = future.exception()
                if exc is not None:
                    result_future.set_exception(exc)
                    return

                goal_handle = future.result()
                if not goal_handle.accepted:
                    self._node.get_logger().error('Goal rejected')
                    result_future.set_exception(RuntimeError('Goal rejected'))
                    return

                self._node.get_logger().info('Goal accepted!')
                self._track_goal(goal_handle)
                get_result_future = goal_handle.get_result_async()
                get_result_future.add_done_callback(self._on_done(goal_handle, _on_execute_done))
            except Exception as exc:
                result_future.set_exception(exc)

        def _on_execute_done(future: Future) -> None:
            """실행 완료 콜백: 최종 결과를 result_future 에 설정."""
            try:
                exc = future.exception()
                if exc is not None:
                    result_future.set_exception(exc)
                    return

                result_response = future.result()
                error_code = result_response.result.error_code.val
                self._node.get_logger().info(
                    f'Trajectory execution result error code: {error_code}'
                )
                if error_code != MoveItErrorCodes.SUCCESS:
                    self._node.get_logger().error(
                        f'Trajectory execution failed with code: {error_code}'
                    )
                    result_future.set_exception(RuntimeError(
                        f'Trajectory execution failed with code: {error_code}'
                    ))
                    return

                self._node.get_logger().info('Trajectory execution completed!')
                result_future.set_result(None)
            except Exception as exc:
                result_future.set_exception(exc)

        plan_future.add_done_callback(_on_plan_done)
        return result_future

    def execute_trajectory(self, trajectory: RobotTrajectory, *,
                           timeout: float = TRAJECTORY_EXEC_TIMEOUT_SEC,) -> None:
        """주어진 trajectory 를 실행하고 완료될 때까지 대기한다.

        Args:
            trajectory: 실행할 ``RobotTrajectory`` 메시지.
            timeout: 실행 완료까지 허용할 최대 시간(초).

        Raises:
            ValueError: trajectory 가 유효하지 않을 때.
            TimeoutError: goal 수락 또는 결과 수신이 시간 초과될 때.
            RuntimeError: goal 이 거부되거나 실행이 실패했을 때.
        """
        started_at = time.monotonic()
        get_result_future = self.execute_trajectory_async(trajectory, timeout=timeout)
        remains = check_timeout_and_get_remains(started_at, timeout)
        result_response = await_future_spin(self._node, get_result_future, remains,
                                            'execute_trajectory result')

        result = result_response.result
        self._node.get_logger().info(
            f'Trajectory execution result error code: {result.error_code.val}')
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self._node.get_logger().error(
                f'Trajectory execution failed with code: {result.error_code.val}')
            raise RuntimeError(f'Trajectory execution failed with code: {result.error_code.val}')
        self._node.get_logger().info('Trajectory execution completed!')

    def execute_trajectory_async(self, trajectory: RobotTrajectory, *,
                                 timeout: float = GOAL_ACCEPT_TIMEOUT_SEC,) -> rclpy.Future:
        """trajectory 실행 goal 을 보내고 결과 Future 를 반환한다.

        Args:
            trajectory: 실행할 ``RobotTrajectory`` 메시지.
            timeout: goal 수락까지 허용할 최대 시간(초).

        Returns:
            실행 결과를 포함하는 Future 객체.

        Raises:
            ValueError: trajectory 가 유효하지 않을 때.
            TimeoutError: goal 이 시간 내에 수락되지 않았을 때.
            RuntimeError: goal 이 거부되었거나 클라이언트가 close() 되었을 때.
        """
        self._require_open()
        _validate_trajectory(trajectory)

        self._node.get_logger().info('Sending goal and waiting for completion...')

        goal_msg = ExecuteTrajectory.Goal(trajectory=trajectory)
        future = self._trajectory_follower.send_goal_async(goal_msg)
        goal_handle = await_future_spin(self._node, future, timeout,
                                        'execute_trajectory goal response')

        if not goal_handle.accepted:
            self._node.get_logger().error('Goal rejected')
            raise RuntimeError('Goal rejected')
        self._node.get_logger().info('Goal accepted!')
        self._track_goal(goal_handle)

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda _f, h=goal_handle: self._untrack_goal(h))
        return result_future
