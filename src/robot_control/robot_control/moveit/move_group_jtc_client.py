from __future__ import annotations

from typing import Optional, Sequence

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

from robot_control.moveit.motion_event_publisher import (
    OPERATION_CARTESIAN,
    OPERATION_JOINTS,
    OPERATION_NAMED_TARGET,
    RESULT_ABORTED,
    RESULT_FAILED,
    RESULT_SUCCEEDED,
    MotionContext,
    MotionEventPublisher,
)
from robot_control.moveit.motion_trajectory_publisher import (
    MotionTrajectoryPublisher,
    trajectory_duration_sec,
)
from robot_control.moveit.move_group_client import (
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
    _validate_joint_values,
    _validate_trajectory,
)
from robot_control.ros2_utils import await_future_spin, check_timeout_and_get_remains


# 궤적을 실제로 실행하는 컨트롤러의 액션. cancel() 이 여기에 취소를 보낸다.
_CONTROLLER_ACTION = '/panda_arm_controller/follow_joint_trajectory'


def _moveit_error_name(code: int) -> str:
    """``MoveItErrorCodes`` 값을 이름으로 바꾼다.

    연산 이벤트(R2)의 ``reason`` 은 **사람이 읽는 값**이라 숫자보다 이름이 낫다.
    레이블 계산에는 쓰이지 않으므로 모르는 값은 숫자 그대로 남긴다.
    """
    for name in dir(MoveItErrorCodes):
        if name.isupper() and getattr(MoveItErrorCodes, name, None) == code:
            return name
    return f'error_code={code}'


class MoveGroupJtcClient(MoveGroupClient):
    """``JointTrajectoryController`` 환경용 :class:`MoveGroupClient` 구현.

    **네 경로가 모두 "계획 → 실행" 두 단계다** (2026-09-18).

    * 관절 공간 이동(named target · 값 지정): ``plan_joints`` 로 계획만 받고
      ``ExecuteTrajectory`` 액션으로 실행한다.
    * 카테시안 경로: ``GetCartesianPath`` 로 계획한 뒤 같은 액션으로 실행한다.

    예전에는 관절 공간만 ``MoveGroup`` 액션 한 번(``plan_only=False``)으로 계획과
    실행을 함께 했는데, 그러면 **클라이언트가 실행 궤적을 보지 못한다.** 실행 궤적
    채널(R1)이 그 궤적을 실어야 해서 나눴다 —
    docs/learning/학습_샘플_사양.md §3.4.

    두 경로 모두 최종적으로 ``moveit_simple_controller_manager`` 를 거쳐
    ``FollowJointTrajectory`` 액션을 쓴다. 그 액션을 제공하지 않는 컨트롤러
    (``position_controllers/JointGroupPositionController`` 등) 환경에서는
    동작하지 않으므로, 그때는
    :class:`~robot_control.moveit.move_group_jgpc_client.MoveGroupJgpcClient` 를 쓴다.

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
        # R1 실행 궤적 채널. 처음 쓸 때 만든다 — 발행하지 않는 사용처에
        # 쓸데없는 발행자를 남기지 않기 위해서다.
        self._motion_pub: Optional[MotionTrajectoryPublisher] = None
        # R2 연산 이벤트 채널. 같은 이유로 지연 생성한다.
        self._event_pub: Optional[MotionEventPublisher] = None
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
            what=f'named target "{name}"', operation=OPERATION_NAMED_TARGET
        )

    def _execute_joint_goal(self, joint_values: dict[str, float], *,
                            velocity_scaling: float, planning_time: float,
                            tolerance: float, started_at: float, timeout: float,
                            what: str, operation: str) -> None:
        """joint 값 목표를 **계획한 뒤 실행한다**(블로킹).

        ``move_to_named_target`` (SRDF 조회 후) 과 ``move_to_joints`` (값 직접 지정)
        가 공유하는 실행 체인이다. ``what`` 은 로그·오류 메시지에 쓰는 목표 설명이다.

        **2026-09-18 에 계획과 실행을 나눴다.** 예전에는 ``MoveGroup`` 액션을
        ``plan_only=False`` 로 한 번 보내 move_group 이 계획과 실행을 함께 했는데,
        그러면 **클라이언트가 실행 궤적을 보지 못해** 실행 궤적 채널(R1)에 실을 것이
        없었다. 이제 ``plan_joints`` 로 계획만 받고 ``execute_trajectory`` 로 실행하며,
        그 안에서 goal 을 보내기 직전에 궤적을 발행한다. 데카르트 경로가 이미 같은
        구조라 **네 경로가 한 모양**이 된다.

        대가는 계획과 실행 사이에 왕복이 하나 늘어난다는 것이다. 그 사이에 팔이
        움직이면 MoveIt 의 ``allowed_start_tolerance``(0.01) 검사에 걸려
        ``CONTROL_FAILED`` 가 난다 — 접촉으로 팔이 눌린 뒤에 그런 일이 실측됐다
        (설계 §3.5). 접촉이 없으면 추종 오차가 0.0002 rad 이라 문제가 없어 여기서
        자동 재시도는 하지 않는다. 접촉을 다루는 호출자가 재시도를 갖는다.

        Args:
            started_at: 호출자가 ``time.monotonic()`` 으로 기록한 시작 시각. 앞선
                단계(SRDF 조회 등)에서 쓴 시간을 ``timeout`` 에 포함시키기 위해
                호출자가 넘긴다.
        """
        self._node.get_logger().info(
            f'Moving to {what} in group "{self._moveit_group_name}"...'
        )

        names = list(joint_values)
        values = [joint_values[n] for n in names]

        remains = check_timeout_and_get_remains(started_at, timeout)
        try:
            trajectory = self.plan_joints(
                joint_values, velocity_scaling=velocity_scaling,
                planning_time=planning_time, tolerance=tolerance, timeout=remains,
            )
        except Exception as exc:
            # 계획에서 끝났다 — 궤적을 넘기지도 못했으므로 `failed` 이고 짝이 없다.
            self._publish_failed(operation=operation,
                                 reason=f'{type(exc).__name__}: {exc}',
                                 target_joint_names=names, target_joint_values=values)
            raise

        remains = check_timeout_and_get_remains(started_at, timeout)
        try:
            # R1 실행 궤적과 R2 연산 이벤트는 이 안에서 발행된다.
            self.execute_trajectory(trajectory, timeout=remains,
                                    velocity_scaling=velocity_scaling,
                                    operation=operation, target_joint_names=names,
                                    target_joint_values=values)
        except RuntimeError as exc:
            # 목표 설명을 붙여 준다 — 예전 한 방 실행과 오류 문장을 맞춘다.
            raise RuntimeError(f'Move to {what} failed: {exc}') from exc
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
                tolerance=tolerance, what=what, operation=OPERATION_NAMED_TARGET,
                externally_spun=externally_spun, result_future=result_future
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
                                  tolerance: float, what: str, operation: str,
                                  externally_spun: bool = False,
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
        names = list(joint_values)
        values = [joint_values[n] for n in names]

        # R1 에서 만들어 결과 콜백까지 들고 가는 문맥. 계획이 끝나야 생긴다.
        ctx: Optional[MotionContext] = None

        def _fail(reason: str) -> None:
            """궤적을 넘기기 전에 끝났다 — `failed` 이고 짝 궤적이 없다 (R2 §3.1)."""
            self._publish_failed(operation=operation, reason=reason,
                                 target_joint_names=names, target_joint_values=values)

        def _on_plan_done(plan_future: Future) -> None:
            """계획 완료 콜백: 받은 궤적으로 실행 goal 을 보낸다.

            ⚠️ **`execute_trajectory_async` 를 부르지 않는다.** 그쪽은 goal 수락을
            `rclpy.spin_until_future_complete` 로 기다리는데, 이 콜백은 executor
            스레드에서 도므로 그 spin 이 성립하지 않아 **영영 안 풀린다** (2026-09-18
            실측: 45초 대기에도 미완료, 로그도 에러도 없다). 그래서 데카르트 비동기
            경로와 같은 **콜백 방식**으로 goal 을 보낸다.
            """
            nonlocal ctx
            try:
                exc = plan_future.exception()
                if exc is not None:
                    _fail(f'{type(exc).__name__}: {exc}')
                    fut.set_exception(exc)
                    return
                trajectory = plan_future.result()
                _validate_trajectory(trajectory)

                # R1: goal 을 보내기 **직전**에 실행 궤적을 발행한다 (t₀).
                ctx = self._begin_motion(trajectory, velocity_scaling, operation=operation,
                                         target_joint_names=names, target_joint_values=values)
                self._node.get_logger().info('Sending goal (async)...')
                goal_msg = ExecuteTrajectory.Goal(trajectory=trajectory)
                goal_future = self._trajectory_follower.send_goal_async(goal_msg)
                goal_future.add_done_callback(_on_goal_response)
            except Exception as exc:
                reason = f'{type(exc).__name__}: {exc}'
                if ctx is None:
                    _fail(reason)
                else:
                    self._finish_motion(ctx, result=RESULT_ABORTED, reason=reason,
                                        truncated_at_sec=0.0)
                fut.set_exception(exc)

        def _on_goal_response(goal_future: Future) -> None:
            """goal 수락 콜백: 수락 여부를 확인하고 결과를 기다린다."""
            try:
                exc = goal_future.exception()
                if exc is not None:
                    self._finish_motion(ctx, result=RESULT_ABORTED,
                                        reason=f'{type(exc).__name__}: {exc}',
                                        truncated_at_sec=0.0)
                    fut.set_exception(exc)
                    return

                goal_handle = goal_future.result()
                if not goal_handle.accepted:
                    self._node.get_logger().error('Goal rejected')
                    # 궤적은 이미 나갔으므로 `aborted` 다 — 한 점도 안 갔으니 0 이다.
                    self._finish_motion(ctx, result=RESULT_ABORTED,
                                        reason='goal rejected', truncated_at_sec=0.0)
                    fut.set_exception(RuntimeError('Goal rejected'))
                    return

                self._node.get_logger().info('Goal accepted!')
                self._track_goal(goal_handle)
                get_result_future = goal_handle.get_result_async()
                get_result_future.add_done_callback(self._on_done(goal_handle, _on_execute_done))
            except Exception as exc:
                self._finish_motion(ctx, result=RESULT_ABORTED,
                                    reason=f'{type(exc).__name__}: {exc}',
                                    truncated_at_sec=0.0)
                fut.set_exception(exc)

        def _on_execute_done(future: Future) -> None:
            """실행 완료 콜백: 최종 결과를 Future 에 설정하고 R2 를 남긴다."""
            try:
                exc = future.exception()
                if exc is not None:
                    self._finish_motion(
                        ctx, result=RESULT_ABORTED, reason=f'{type(exc).__name__}: {exc}',
                        truncated_at_sec=self._elapsed_since(ctx.stamp) if ctx else None)
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
                    self._finish_motion(
                        ctx, result=RESULT_ABORTED, reason=_moveit_error_name(error_code),
                        truncated_at_sec=self._elapsed_since(ctx.stamp) if ctx else None)
                    fut.set_exception(RuntimeError(
                        f'Move to {what} failed with code: {error_code}'
                    ))
                    return
                self._node.get_logger().info(f'Reached {what}')
                self._finish_motion(ctx, result=RESULT_SUCCEEDED)
                fut.set_result(None)
            except Exception as exc:
                fut.set_exception(exc)

        try:
            # 동기판(`_execute_joint_goal`)과 같은 (a) 구조다 — 계획만 받고 실행은
            # 위 콜백 체인이 한다. 그래야 클라이언트가 실행 궤적을 손에 쥐어 R1
            # 채널에 실을 수 있다 (R1 설계 §3.4). 다만 **동기판과 달리 goal 수락을
            # 기다리지 않는다** — 이 경로는 executor 콜백 안에서 도므로 spin 이
            # 성립하지 않는다 (`_on_plan_done` 주석).
            self._node.get_logger().info(f'Moving to {what} (async)...')
            # ⚠️ **`externally_spun` 을 반드시 넘긴다.** `plan_joints_async` 는 이 값을
            # `_complete_joint_values` 로 흘리는데, `False` 면 그쪽이 **노드를 직접
            # spin** 한다. executor 가 이미 다른 스레드에서 도는 환경(트윈)에서는 그
            # spin 이 성립하지 않아 **계획 future 가 영영 안 끝난다** — 에러도 로그도
            # 없이 "연산이 안 돌아온다"로만 나타난다 (2026-09-18 실측: 45초 대기에도
            # 미완료). 계획·실행 분리로 바꾸면서 생긴 구멍이다.
            plan_future = self.plan_joints_async(
                joint_values, velocity_scaling=velocity_scaling,
                planning_time=planning_time, tolerance=tolerance,
                externally_spun=externally_spun,
            )
            plan_future.add_done_callback(_on_plan_done)
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
        values = self._complete_joint_values(_validate_joint_values(joint_values))
        vs = self._resolve_velocity_scaling(velocity_scaling)

        self._execute_joint_goal(
            values, velocity_scaling=vs, planning_time=planning_time,
            tolerance=tolerance, started_at=time.monotonic(), timeout=timeout,
            what=_JOINT_GOAL_LABEL, operation=OPERATION_JOINTS
        )

    def move_to_joints_async(self, joint_values: dict[str, float], *,
                             velocity_scaling: Optional[float] = None,
                             planning_time: float = DEFAULT_PLANNING_TIME,
                             tolerance: float = DEFAULT_JOINT_TOLERANCE,
                             externally_spun: bool = False) -> Future:
        """:meth:`move_to_joints` 의 비동기 버전.

        ⚠️ **``externally_spun`` 을 정확히 줘야 한다.** 계획·실행을 나눈 뒤로
        (2026-09-18) 이 경로가 `plan_joints_async` → `_complete_joint_values` 를
        타는데, 그쪽은 `False` 면 **노드를 직접 spin** 한다. executor 가 이미 다른
        스레드에서 도는 환경에서 `False` 로 두면 **계획이 영영 안 끝난다.**
        (그 전에는 정말로 의미가 없었고, 옛 주석이 그렇게 적혀 있었다.)

        Returns:
            성공 시 ``None`` 으로 resolve 되는 ``Future``.
        """
        self._require_open()
        values = self._complete_joint_values(
            _validate_joint_values(joint_values), externally_spun=externally_spun)
        vs = self._resolve_velocity_scaling(velocity_scaling)

        return self._execute_joint_goal_async(
            values, velocity_scaling=vs, planning_time=planning_time,
            tolerance=tolerance, what=_JOINT_GOAL_LABEL, operation=OPERATION_JOINTS,
            externally_spun=externally_spun
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
        goal_pose = waypoints[-1] if waypoints else None
        try:
            trajectory = self.plan_trajectory(
                waypoints,
                velocity_scaling=velocity_scaling,
                fraction_threshold=fraction_threshold,
                max_step=max_step,
                jump_threshold=jump_threshold,
                timeout=timeout,
            )
        except Exception as exc:
            # 계획에서 끝났다 — 궤적을 넘기지도 못했으므로 `failed` 다 (R2 설계 §3.1).
            self._publish_failed(operation=OPERATION_CARTESIAN,
                                 reason=f'{type(exc).__name__}: {exc}',
                                 target_pose=goal_pose)
            raise
        remains = check_timeout_and_get_remains(started_at, timeout)
        # 궤적은 plan_trajectory 가 이미 스케일했다. 배율은 R1 기록용으로만 넘긴다.
        self.execute_trajectory(trajectory, timeout=remains,
                                velocity_scaling=self._resolve_velocity_scaling(velocity_scaling),
                                operation=OPERATION_CARTESIAN, target_pose=goal_pose)

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
        goal_pose = waypoints[-1] if waypoints else None
        # R1 에서 만들어 결과 콜백까지 들고 가는 문맥. 계획이 끝나야 생긴다.
        ctx: Optional[MotionContext] = None
        plan_future = self.plan_trajectory_async(waypoints, max_step=max_step,
                                                 jump_threshold=jump_threshold)

        def _plan_failed(reason: str) -> None:
            """계획에서 끝난 연산을 `failed` 로 남긴다 — 짝 궤적이 없는 것이 정상이다."""
            self._publish_failed(operation=OPERATION_CARTESIAN, reason=reason,
                                 target_pose=goal_pose)

        def _on_plan_done(future: Future) -> None:
            """계획 서비스 응답 콜백: 결과 검증 후 실행 goal 전송."""
            nonlocal ctx
            try:
                exc = future.exception()
                if exc is not None:
                    _plan_failed(f'{type(exc).__name__}: {exc}')
                    result_future.set_exception(exc)
                    return

                response = future.result()
                if response.error_code.val != MoveItErrorCodes.SUCCESS:
                    self._node.get_logger().error(
                        f'Path planning returned error code: {response.error_code.val}'
                    )
                    _plan_failed(_moveit_error_name(response.error_code.val))
                    result_future.set_exception(RuntimeError(
                        f'Path planning failed with error code: {response.error_code.val}'
                    ))
                    return

                if response.fraction < ft:
                    self._node.get_logger().error(
                        f'Path planning failed (fraction: {response.fraction})')
                    _plan_failed(f'fraction {response.fraction:.3f} < {ft:.3f}')
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

                # R1: goal 을 보내기 **직전**에 실행 궤적을 발행한다 (t₀).
                ctx = self._begin_motion(trajectory, vs, operation=OPERATION_CARTESIAN,
                                         target_pose=goal_pose)
                goal_msg = ExecuteTrajectory.Goal(trajectory=trajectory)
                goal_future = self._trajectory_follower.send_goal_async(goal_msg)
                goal_future.add_done_callback(_on_goal_response)
            except Exception as exc:
                # **R1 을 이미 냈으면 `failed` 가 아니다** — 짝 궤적이 있으므로
                # `aborted` 다. 이 구분이 적재 검증의 고아 판정을 지킨다.
                reason = f'{type(exc).__name__}: {exc}'
                if ctx is None:
                    _plan_failed(reason)
                else:
                    self._finish_motion(ctx, result=RESULT_ABORTED, reason=reason,
                                        truncated_at_sec=0.0)
                result_future.set_exception(exc)

        def _on_goal_response(future: Future) -> None:
            """goal 수락 콜백: 수락 여부 확인 후 결과 대기."""
            try:
                exc = future.exception()
                if exc is not None:
                    self._finish_motion(ctx, result=RESULT_ABORTED,
                                        reason=f'{type(exc).__name__}: {exc}',
                                        truncated_at_sec=0.0)
                    result_future.set_exception(exc)
                    return

                goal_handle = future.result()
                if not goal_handle.accepted:
                    self._node.get_logger().error('Goal rejected')
                    # 궤적은 이미 나갔으므로 `aborted` 다. 한 점도 안 갔으니 0 이다.
                    self._finish_motion(ctx, result=RESULT_ABORTED,
                                        reason='goal rejected', truncated_at_sec=0.0)
                    result_future.set_exception(RuntimeError('Goal rejected'))
                    return

                self._node.get_logger().info('Goal accepted!')
                self._track_goal(goal_handle)
                get_result_future = goal_handle.get_result_async()
                get_result_future.add_done_callback(self._on_done(goal_handle, _on_execute_done))
            except Exception as exc:
                self._finish_motion(ctx, result=RESULT_ABORTED,
                                    reason=f'{type(exc).__name__}: {exc}',
                                    truncated_at_sec=0.0)
                result_future.set_exception(exc)

        def _on_execute_done(future: Future) -> None:
            """실행 완료 콜백: 최종 결과를 result_future 에 설정하고 R2 를 남긴다."""
            try:
                exc = future.exception()
                if exc is not None:
                    self._finish_motion(
                        ctx, result=RESULT_ABORTED,
                        reason=f'{type(exc).__name__}: {exc}',
                        truncated_at_sec=self._elapsed_since(ctx.stamp) if ctx else None)
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
                    self._finish_motion(
                        ctx, result=RESULT_ABORTED, reason=_moveit_error_name(error_code),
                        truncated_at_sec=self._elapsed_since(ctx.stamp) if ctx else None)
                    result_future.set_exception(RuntimeError(
                        f'Trajectory execution failed with code: {error_code}'
                    ))
                    return

                self._node.get_logger().info('Trajectory execution completed!')
                self._finish_motion(ctx, result=RESULT_SUCCEEDED)
                result_future.set_result(None)
            except Exception as exc:
                result_future.set_exception(exc)

        plan_future.add_done_callback(_on_plan_done)
        return result_future

    # ----- 실행 궤적(R1) · 연산 이벤트(R2) ---------------------------------

    def _begin_motion(self, trajectory: RobotTrajectory, velocity_scaling: Optional[float], *,
                      operation: str,
                      target_joint_names: Sequence[str] = (),
                      target_joint_values: Sequence[float] = (),
                      target_pose: Optional[Pose] = None) -> MotionContext:
        """실행 궤적(L2)을 발행하고 **연산 문맥을 만든다** (R1 → R2).

        **컨트롤러로 goal 을 보내기 바로 앞에서 불러야 한다** — 그 순간이 t₀ 이기
        때문이다(R1 설계 §3.5).

        t₀ 를 **여기서 찍어 R1 에 넘긴다.** 발행자 안에서 찍게 두면 밖에서 그 값을 볼
        수 없는데, R2 의 ``truncated_at`` 이 ``끝난 시각 − t₀`` 라 반드시 필요하다
        (R2 설계 §3.3 의 (a) — `publish()` 의 반환 타입을 안 건드리는 쪽을 택했다).

        발행이 실패해도 예외를 내지 않는다. 그때는 ``operation_id`` 가 **빈 문자열**로
        남아, 적재 검증(미구현 목록 I5)이 짝 없는 이벤트로 드러낸다.
        """
        if self._motion_pub is None:
            self._motion_pub = MotionTrajectoryPublisher(self._node)
        stamp = self._node.get_clock().now().to_msg()
        operation_id = self._motion_pub.publish(
            trajectory, stamp=stamp,
            velocity_scaling=1.0 if velocity_scaling is None else velocity_scaling)
        jt = (trajectory.joint_trajectory if isinstance(trajectory, RobotTrajectory)
              else trajectory)
        return MotionContext(
            operation_id=operation_id or '', stamp=stamp, operation=operation,
            duration_sec=trajectory_duration_sec(jt),
            target_joint_names=tuple(target_joint_names),
            target_joint_values=tuple(target_joint_values),
            target_pose=target_pose,
            target_frame=self._tip_frame if target_pose is not None else '')

    def _finish_motion(self, ctx: Optional[MotionContext], *, result: str,
                       reason: str = '',
                       truncated_at_sec: Optional[float] = None) -> None:
        """연산 이벤트를 한 건 발행한다 (R2). 문맥이 없으면 아무것도 안 한다."""
        if ctx is None:
            return
        if self._event_pub is None:
            self._event_pub = MotionEventPublisher(self._node)
        self._event_pub.publish_context(ctx, result=result, reason=reason,
                                        truncated_at_sec=truncated_at_sec)

    def _publish_failed(self, *, operation: str, reason: str,
                        target_joint_names: Sequence[str] = (),
                        target_joint_values: Sequence[float] = (),
                        target_pose: Optional[Pose] = None) -> None:
        """궤적을 **넘기지도 못한** 연산을 `failed` 로 남긴다 (R2).

        계획 단계에서 끝났으므로 R1 이 없다 — **짝 궤적이 없는 것이 정상이고**,
        그것이 `failed` 의 정의다(R2 설계 §3.1). ``operation_id`` 는 여기서 만든다.
        """
        self._finish_motion(
            MotionContext(
                operation_id=MotionTrajectoryPublisher.new_operation_id(),
                stamp=self._node.get_clock().now().to_msg(), operation=operation,
                target_joint_names=tuple(target_joint_names),
                target_joint_values=tuple(target_joint_values),
                target_pose=target_pose,
                target_frame=self._tip_frame if target_pose is not None else ''),
            result=RESULT_FAILED, reason=reason)

    def _elapsed_since(self, stamp) -> float:
        """t₀ 로부터 지난 시간(초). 중단 시각을 궤적 축으로 옮기는 데 쓴다."""
        now = self._node.get_clock().now().to_msg()
        return (now.sec - stamp.sec) + (now.nanosec - stamp.nanosec) / 1_000_000_000

    def _on_execute_event(self, future: Future, ctx: MotionContext) -> None:
        """실행 결과를 R2 로 남긴다.

        ⚠️ **예외를 밖으로 내보내지 않는다.** done_callback 안에서 터지면 노드
        프로세스가 통째로 내려가고, 증상은 "그 노드만 조용히 무응답"이 된다.
        """
        try:
            exc = future.exception()
            if exc is not None:
                self._finish_motion(ctx, result=RESULT_ABORTED,
                                    reason=f'{type(exc).__name__}: {exc}',
                                    truncated_at_sec=self._elapsed_since(ctx.stamp))
                return
            code = future.result().result.error_code.val
            if code == MoveItErrorCodes.SUCCESS:
                self._finish_motion(ctx, result=RESULT_SUCCEEDED)
            else:
                self._finish_motion(ctx, result=RESULT_ABORTED,
                                    reason=_moveit_error_name(code),
                                    truncated_at_sec=self._elapsed_since(ctx.stamp))
        except Exception as exc:                       # noqa: BLE001 - 기록이 동작을 막지 않는다
            self._node.get_logger().warning(
                f'Failed to report motion event for {ctx.operation_id!r}: '
                f'{type(exc).__name__}: {exc}')

    def execute_trajectory(self, trajectory: RobotTrajectory, *,
                           timeout: float = TRAJECTORY_EXEC_TIMEOUT_SEC,
                           velocity_scaling: Optional[float] = None,
                           operation: str = OPERATION_CARTESIAN,
                           target_joint_names: Sequence[str] = (),
                           target_joint_values: Sequence[float] = (),
                           target_pose: Optional[Pose] = None) -> None:
        """주어진 trajectory 를 실행하고 완료될 때까지 대기한다.

        Args:
            trajectory: 실행할 ``RobotTrajectory`` 메시지.
            timeout: 실행 완료까지 허용할 최대 시간(초).
            velocity_scaling: **기록용**이다. 이 값으로 궤적을 다시 스케일하지
                않는다 — 궤적에는 이미 반영돼 있다. R1 실행 궤적 채널에 "어떤 배율로
                만든 궤적인가" 를 남기려는 것뿐이라, 이미 스케일한 궤적을 넘기는
                호출자는 그 값을 함께 준다. 안 주면 1.0 으로 기록한다.

        Raises:
            ValueError: trajectory 가 유효하지 않을 때.
            TimeoutError: goal 수락 또는 결과 수신이 시간 초과될 때.
            RuntimeError: goal 이 거부되거나 실행이 실패했을 때.
        """
        started_at = time.monotonic()
        get_result_future = self.execute_trajectory_async(
            trajectory, timeout=timeout, velocity_scaling=velocity_scaling,
            operation=operation, target_joint_names=target_joint_names,
            target_joint_values=target_joint_values, target_pose=target_pose)
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
                                 timeout: float = GOAL_ACCEPT_TIMEOUT_SEC,
                                 velocity_scaling: Optional[float] = None,
                                 operation: str = OPERATION_CARTESIAN,
                                 target_joint_names: Sequence[str] = (),
                                 target_joint_values: Sequence[float] = (),
                                 target_pose: Optional[Pose] = None) -> rclpy.Future:
        """trajectory 실행 goal 을 보내고 결과 Future 를 반환한다.

        Args:
            trajectory: 실행할 ``RobotTrajectory`` 메시지.
            timeout: goal 수락까지 허용할 최대 시간(초).
            velocity_scaling: **기록용**이다. 이 값으로 궤적을 다시 스케일하지
                않는다 — 궤적에는 이미 반영돼 있다. R1 실행 궤적 채널에 "어떤 배율로
                만든 궤적인가" 를 남기려는 것뿐이라, 이미 스케일한 궤적을 넘기는
                호출자는 그 값을 함께 준다. 안 주면 1.0 으로 기록한다.

        ⚠️ **이름과 달리 goal 수락까지는 블로킹이다** — `rclpy.spin_until_future_complete`
        로 기다린다. 그래서 **executor 콜백 안에서 부르면 안 된다**: 그 spin 이
        성립하지 않아 영영 안 풀린다(2026-09-18 실측 45초 미완). 비동기 체인에서는
        `send_goal_async` + `add_done_callback` 을 직접 쓴다 —
        `_execute_joint_goal_async` · `follow_trajectory_async` 가 그렇게 한다.

        Returns:
            실행 결과를 포함하는 Future 객체.

        Raises:
            ValueError: trajectory 가 유효하지 않을 때.
            TimeoutError: goal 이 시간 내에 수락되지 않았을 때.
            RuntimeError: goal 이 거부되었거나 클라이언트가 close() 되었을 때.
        """
        self._require_open()
        try:
            _validate_trajectory(trajectory)
        except Exception as exc:
            # 궤적을 컨트롤러로 **넘기지도 못했다** — `failed` 다 (R2 설계 §3.1).
            self._publish_failed(operation=operation,
                                 reason=f'{type(exc).__name__}: {exc}',
                                 target_joint_names=target_joint_names,
                                 target_joint_values=target_joint_values,
                                 target_pose=target_pose)
            raise

        self._node.get_logger().info('Sending goal and waiting for completion...')

        # R1: goal 을 보내기 **직전**에 실행 궤적을 발행한다 (t₀).
        ctx = self._begin_motion(trajectory, velocity_scaling, operation=operation,
                                 target_joint_names=target_joint_names,
                                 target_joint_values=target_joint_values,
                                 target_pose=target_pose)
        goal_msg = ExecuteTrajectory.Goal(trajectory=trajectory)
        future = self._trajectory_follower.send_goal_async(goal_msg)
        goal_handle = await_future_spin(self._node, future, timeout,
                                        'execute_trajectory goal response')

        if not goal_handle.accepted:
            self._node.get_logger().error('Goal rejected')
            # **`failed` 가 아니라 `aborted` 다** — 궤적은 이미 발행됐으므로 짝이 있다.
            # 한 점도 실행되지 않았으므로 중단 시각은 0 이다 (R2 설계 §3.1).
            self._finish_motion(ctx, result=RESULT_ABORTED, reason='goal rejected',
                                truncated_at_sec=0.0)
            raise RuntimeError('Goal rejected')
        self._node.get_logger().info('Goal accepted!')
        self._track_goal(goal_handle)

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda _f, h=goal_handle: self._untrack_goal(h))
        # R2: 실행 결과를 남긴다. untrack 과 별개의 콜백이라 순서에 의존하지 않는다.
        result_future.add_done_callback(lambda f, c=ctx: self._on_execute_event(f, c))
        return result_future
