from __future__ import annotations

from typing import Optional

import copy
import math
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.client import Client
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    RobotTrajectory,
)
from moveit_msgs.srv import GetCartesianPath
from rcl_interfaces.srv import GetParameters

from rdfp.ros2_utils import await_future_spin, check_timeout_and_get_remains


READY_TIMEOUT_SEC = 30.0
PLAN_TIMEOUT_SEC = 20.0
GOAL_ACCEPT_TIMEOUT_SEC = 10.0
TRAJECTORY_EXEC_TIMEOUT_SEC = 120.0
MOVE_GROUP_TIMEOUT_SEC = 120.0

DEFAULT_FRACTION_THRESHOLD = 0.60
DEFAULT_VELOCITY_SCALING = 1.0
DEFAULT_MAX_STEP = 0.01
DEFAULT_JUMP_THRESHOLD = 5.0
DEFAULT_PLANNING_TIME = 5.0
DEFAULT_JOINT_TOLERANCE = 1e-4

DEFAULT_FRAME_ID = 'panda_link0'
DEFAULT_MOVEIT_GROUP_NAME = 'panda_arm'

_CARTESIAN_PATH_SERVICE = '/compute_cartesian_path'
_EXECUTE_TRAJECTORY_ACTION = '/execute_trajectory'
_MOVE_GROUP_ACTION = '/move_action'
_MOVE_GROUP_NODE_NAME = '/move_group'


class MoveGroupClient:
    """MoveIt2 motion planning 클라이언트.

    MoveIt2의 다음 인터페이스들을 감싸 주입된 ``rclpy.node.Node`` 위에서
    로봇 이동을 계획하고 실행하는 통합 유틸리티 클래스이다.

    * ``GetCartesianPath`` 서비스 (``/compute_cartesian_path``) — 카테시안
      waypoint 경로 계획.
    * ``ExecuteTrajectory`` 액션 (``/execute_trajectory``) — 계획된
      ``RobotTrajectory`` 실행.
    * ``MoveGroup`` 액션 (``/move_action``) — SRDF의 ``group_state``
      (named target) 로의 joint-space 이동(plan + execute 통합).
    * ``get_parameters`` 서비스 (``/move_group/get_parameters``) — named
      target 조회를 위한 ``robot_description_semantic`` (SRDF) 획득.
      결과는 인스턴스에 캐시된다.

    지원 기능 요약:

    * **Cartesian 경로**: :meth:`follow_trajectory`,
      :meth:`plan_trajectory`, :meth:`execute_trajectory` 및 그
      ``_async`` 변형.
    * **Named target**: :meth:`move_to_named_target`,
      :meth:`move_to_named_target_async`, :meth:`get_named_targets`.
    * **Trajectory 유틸**: :meth:`scale_trajectory_velocity`.

    Lifecycle:
        생성자는 서비스/액션 **클라이언트 객체만** 생성하며, 실제 서버가
        준비될 때까지 대기하지 않는다. 서버 준비 여부는 :meth:`is_ready`
        로 즉시 확인하거나 :meth:`wait_until_ready` 로 블로킹 대기할 수
        있다. 준비 대기 없이 동기 메서드를 직접 호출해도 각 메서드의
        timeout 내에서 서버를 기다린다. 단, :meth:`is_ready` /
        :meth:`wait_until_ready` 는 카테시안 서비스/액션만 체크하며
        MoveGroup 액션은 :meth:`move_to_named_target` 내부에서 lazy 로
        대기한다.

    Threading:
        내부 동기 대기는 ``rclpy.spin_until_future_complete`` 를 사용하여
        호출자의 ``Node`` 를 **직접 spin** 한다. 이 때문에:

        * 호출자가 이미 다른 스레드에서 동일 ``Node`` 를 spin 중이면 이중
          spin 문제가 발생할 수 있다.
        * ``MultiThreadedExecutor`` 환경에서는 타이밍 경합이 일어날 수 있다.
        * 서비스/액션 콜백 내부에서 동기 메서드를 호출하면 데드락 가능성이
          있다.

        가능하면 **메인 스레드 또는 전용 스레드** 한 곳에서만 이 클래스의
        동기 메서드를 호출하라. 비동기 API(:meth:`follow_trajectory_async`,
        :meth:`move_to_named_target_async`, :meth:`plan_trajectory_async`,
        :meth:`execute_trajectory_async`)는 Future 를 반환하므로 호출자가
        자체 executor 에서 처리할 수 있다.
    """

    def __init__(
        self,
        node: Node,
        *,
        frame_id: str = DEFAULT_FRAME_ID,
        moveit_group_name: str = DEFAULT_MOVEIT_GROUP_NAME,
        fraction_threshold: float = DEFAULT_FRACTION_THRESHOLD,
        velocity_scaling: float = DEFAULT_VELOCITY_SCALING,
        max_step: float = DEFAULT_MAX_STEP,
        jump_threshold: float = DEFAULT_JUMP_THRESHOLD,
        cartesian_path_service: str = _CARTESIAN_PATH_SERVICE,
        execute_trajectory_action: str = _EXECUTE_TRAJECTORY_ACTION,
        move_group_action: str = _MOVE_GROUP_ACTION,
        move_group_node_name: str = _MOVE_GROUP_NODE_NAME,
    ) -> None:
        """클라이언트를 초기화하고 서비스/액션 클라이언트를 lazy 하게 생성한다.

        Args:
            node: 서비스/액션 클라이언트를 올릴 ROS2 Node. 호출자가 생성해야
                하며, 이 클래스는 Node 를 파괴하지 않는다.
            frame_id: 카테시안 경로 계획에 사용할 기준 프레임 ID.
            moveit_group_name: MoveIt 플래닝 그룹 이름.
            fraction_threshold: 계획된 카테시안 경로의 최소 허용 비율 기본값
                (0.0 ~ 1.0). 각 호출에서 override 가능.
            velocity_scaling: 계획된 trajectory 에 적용할 속도 스케일링 계수
                기본값 (1.0 = 원래 속도). 각 호출에서 override 가능.
            max_step: 카테시안 보간 최대 단계 간격 기본값(미터). 각 호출에서
                override 가능.
            jump_threshold: 관절 공간 점프 임계값 기본값(라디안). 0.0 을
                사용하면 점프 검사가 비활성화된다. 각 호출에서 override 가능.
            cartesian_path_service: ``GetCartesianPath`` 서비스 이름.
            execute_trajectory_action: ``ExecuteTrajectory`` 액션 이름.
            move_group_action: ``MoveGroup`` 액션 이름. named target 이동에
                사용된다 (기본값 ``/move_action``).
            move_group_node_name: SRDF 조회를 위해 ``get_parameters`` 서비스
                를 호출할 ``move_group`` 노드 이름 (기본값 ``/move_group``).

        Raises:
            ValueError: 입력 매개변수가 유효하지 않을 때.
        """
        if not isinstance(node, Node):
            raise ValueError('node must be a valid ROS2 Node instance')
        _validate_fraction_threshold(fraction_threshold)
        _validate_velocity_scaling(velocity_scaling)
        _validate_max_step(max_step)
        _validate_jump_threshold(jump_threshold)

        self._node = node
        self._frame_id = frame_id
        self._moveit_group_name = moveit_group_name
        self._default_fraction_threshold = fraction_threshold
        self._default_velocity_scaling = velocity_scaling
        self._default_max_step = max_step
        self._default_jump_threshold = jump_threshold
        self._closed = False

        # Lazy: 서비스/액션 클라이언트 객체만 생성하고 서버 준비는 대기하지 않는다.
        self._trajectory_planner: Client = node.create_client(
            GetCartesianPath, cartesian_path_service
        )
        self._trajectory_follower: ActionClient = ActionClient(
            node, ExecuteTrajectory, execute_trajectory_action
        )
        self._move_group_client: ActionClient = ActionClient(
            node, MoveGroup, move_group_action
        )
        self._move_group_action = move_group_action
        self._move_group_node_name = move_group_node_name
        # SRDF의 group_state 캐시: {group_name: {state_name: {joint_name: value}}}
        self._group_states: Optional[dict[str, dict[str, dict[str, float]]]] = None

    # ----- Lifecycle ------------------------------------------------------

    def is_ready(self) -> bool:
        """서비스와 액션 서버가 모두 준비되었는지 즉시 반환한다(non-blocking)."""
        self._require_open()
        return (
            self._trajectory_planner.service_is_ready()
            and self._trajectory_follower.server_is_ready()
        )

    def wait_until_ready(self, timeout_sec: float = READY_TIMEOUT_SEC) -> None:
        """서비스와 액션 서버가 모두 준비될 때까지 블로킹 대기한다.

        Args:
            timeout_sec: 총 대기 허용 시간(초).

        Raises:
            ValueError: ``timeout_sec`` 가 0 이하일 때.
            TimeoutError: 주어진 시간 내에 서비스나 액션 서버가 준비되지 못했을 때.
            RuntimeError: 클라이언트가 이미 close() 되었을 때.
        """
        self._require_open()
        if timeout_sec <= 0:
            raise ValueError('timeout_sec must be positive')

        started = time.monotonic()

        if not self._trajectory_planner.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError(
                f'Timed out waiting for {_CARTESIAN_PATH_SERVICE} service: '
                f'timeout={timeout_sec}s'
            )

        elapsed = time.monotonic() - started
        remains = max(0.001, timeout_sec - elapsed)
        if not self._trajectory_follower.wait_for_server(timeout_sec=remains):
            raise TimeoutError(
                f'Timed out waiting for {_EXECUTE_TRAJECTORY_ACTION} action server: '
                f'timeout={timeout_sec}s'
            )

    def close(self) -> None:
        """생성한 서비스/액션 클라이언트 리소스를 정리한다.

        멱등(idempotent): 두 번 호출해도 안전하다. 주입된 ``Node`` 자체는
        건드리지 않는다.
        """
        if self._closed:
            return
        try:
            self._trajectory_follower.destroy()
        except Exception:
            pass
        try:
            self._move_group_client.destroy()
        except Exception:
            pass
        try:
            self._node.destroy_client(self._trajectory_planner)
        except Exception:
            pass
        self._closed = True

    def destroy(self) -> None:
        """:meth:`close` 와 동일. ROS2 스타일 네이밍 별칭."""
        self.close()

    def __enter__(self) -> 'MoveGroupClient':
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # ----- High-level API -------------------------------------------------

    def get_named_targets(self, *, group: Optional[str] = None,
                          timeout: float = READY_TIMEOUT_SEC,) -> list[str]:
        """SRDF에 등록된 named target(group_state) 이름 목록을 반환한다.

        첫 호출 시 ``move_group`` 노드의 ``robot_description_semantic`` 파라미터를
        조회하여 파싱/캐시한다. 이후 호출은 캐시로부터 즉시 반환된다.

        Args:
            group: 조회할 planning 그룹 이름. ``None`` 이면 생성자에 지정된
                ``moveit_group_name`` 을 사용한다.
            timeout: SRDF 조회 허용 시간(초). 캐시가 이미 채워진 경우 무시된다.

        Returns:
            그룹에 등록된 group_state 이름을 사전순으로 정렬한 리스트. 해당
            그룹에 state 가 없으면 빈 리스트.

        Raises:
            TimeoutError: SRDF 조회가 시간 내에 완료되지 않을 때.
            RuntimeError: 클라이언트가 이미 close() 되었거나 SRDF 조회가 실패했을 때.
        """
        self._require_open()
        target_group = group if group is not None else self._moveit_group_name
        if self._group_states is None:
            self._group_states = self._fetch_and_parse_srdf(timeout=timeout)
        return sorted(self._group_states.get(target_group, {}).keys())

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

        self._node.get_logger().info(
            f'Moving to named target "{name}" in group "{self._moveit_group_name}"...'
        )

        goal_msg = _build_move_group_goal(
            group_name=self._moveit_group_name, joint_values=joint_values,
            velocity_scaling=vs, planning_time=planning_time, tolerance=tolerance,
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

        # 결과 대기
        result_future = goal_handle.get_result_async()
        remains = check_timeout_and_get_remains(started_at, timeout)
        result_response = await_future_spin(
            self._node, result_future, remains, 'move_group result'
        )

        error_code = result_response.result.error_code.val
        self._node.get_logger().info(
            f'Move group execution result error code: {error_code}'
        )
        if error_code != MoveItErrorCodes.SUCCESS:
            self._node.get_logger().error(
                f'Move to named target failed with code: {error_code}'
            )
            raise RuntimeError(
                f'Move to named target "{name}" failed with code: {error_code}'
            )
        self._node.get_logger().info(f'Reached named target "{name}"')

    def move_to_named_target_async(self, name: str, *,
                                   velocity_scaling: Optional[float] = None,
                                   planning_time: float = DEFAULT_PLANNING_TIME,
                                   tolerance: float = DEFAULT_JOINT_TOLERANCE,) -> Future:
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

        def _start_move(joint_values: dict[str, float]) -> None:
            """joint 값 확보 후 MoveGroup goal 전송 단계로 진입."""
            try:
                goal_msg = _build_move_group_goal(
                    group_name=self._moveit_group_name, joint_values=joint_values,
                    velocity_scaling=vs, planning_time=planning_time, tolerance=tolerance,
                )
                self._node.get_logger().info(
                    f'Moving to named target "{name}" (async)...'
                )
                send_future = self._move_group_client.send_goal_async(goal_msg)
                send_future.add_done_callback(_on_goal_response)
            except Exception as exc:
                result_future.set_exception(exc)

        def _on_goal_response(future: Future) -> None:
            """MoveGroup goal 수락 콜백."""
            try:
                exc = future.exception()
                if exc is not None:
                    result_future.set_exception(exc)
                    return
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self._node.get_logger().error('Move group goal rejected')
                    result_future.set_exception(RuntimeError('Move group goal rejected'))
                    return
                self._node.get_logger().info('Move group goal accepted!')
                get_result_future = goal_handle.get_result_async()
                get_result_future.add_done_callback(_on_execute_done)
            except Exception as exc:
                result_future.set_exception(exc)

        def _on_execute_done(future: Future) -> None:
            """MoveGroup 실행 완료 콜백: 최종 결과를 result_future에 설정."""
            try:
                exc = future.exception()
                if exc is not None:
                    result_future.set_exception(exc)
                    return
                result_response = future.result()
                error_code = result_response.result.error_code.val
                self._node.get_logger().info(
                    f'Move group execution result error code: {error_code}'
                )
                if error_code != MoveItErrorCodes.SUCCESS:
                    self._node.get_logger().error(
                        f'Move to named target failed with code: {error_code}'
                    )
                    result_future.set_exception(RuntimeError(
                        f'Move to named target "{name}" failed with code: {error_code}'
                    ))
                    return
                self._node.get_logger().info(f'Reached named target "{name}"')
                result_future.set_result(None)
            except Exception as exc:
                result_future.set_exception(exc)

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
                self._group_states = _parse_srdf_group_states(srdf_string)
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
                                jump_threshold: Optional[float] = None,) -> Future:
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

        self._node.get_logger().info(f'Moving through {len(waypoints)} cartesian waypoints (async)...')

        result_future = Future()
        plan_future = self.plan_trajectory_async(waypoints, max_step=max_step, jump_threshold=jump_threshold,)

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
                    self._node.get_logger().error(f'Path planning failed (fraction: {response.fraction})')
                    result_future.set_exception(RuntimeError(
                        f'Path planning failed: only '
                        f'{response.fraction * 100:.1f}% of the path was planned'
                    ))
                    return

                self._node.get_logger().info(f'Path planning successful (fraction: {response.fraction})')
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
                get_result_future = goal_handle.get_result_async()
                get_result_future.add_done_callback(_on_execute_done)
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

    def plan_trajectory(self, waypoints: list[Pose], *,
                        velocity_scaling: Optional[float] = None,
                        fraction_threshold: Optional[float] = None,
                        max_step: Optional[float] = None,
                        jump_threshold: Optional[float] = None,
                        timeout: float = PLAN_TIMEOUT_SEC,) -> RobotTrajectory:
        """주어진 waypoint들로 cartesian 경로를 계획하고 trajectory 를 반환한다.

        Args:
            waypoints: Cartesian path 를 따라갈 Pose 리스트.
            velocity_scaling: Override. ``None`` 이면 생성자 기본값 사용.
            fraction_threshold: Override. ``None`` 이면 생성자 기본값 사용.
            max_step: Override. ``None`` 이면 생성자 기본값 사용.
            jump_threshold: Override. ``None`` 이면 생성자 기본값 사용.
            timeout: 서비스 응답 대기 최대 시간(초).

        Returns:
            계획된 ``RobotTrajectory`` 메시지. ``velocity_scaling`` 이 1.0 이
            아니면 스케일된 trajectory 가 반환된다.

        Raises:
            ValueError: 입력 매개변수가 유효하지 않을 때.
            TimeoutError: 서비스 응답이 시간 초과될 때.
            RuntimeError: 경로 계획이 실패했거나 MoveIt 오류 코드를 반환했을 때.
        """
        vs = self._resolve_velocity_scaling(velocity_scaling)
        ft = self._resolve_fraction_threshold(fraction_threshold)

        future = self.plan_trajectory_async(waypoints, max_step=max_step, jump_threshold=jump_threshold,)
        response = await_future_spin(self._node, future, timeout, 'compute_cartesian_path service response')

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self._node.get_logger().error(f'Path planning returned error code: {response.error_code.val}')
            raise RuntimeError(f'Path planning failed with error code: {response.error_code.val}')

        if response.fraction < ft:
            self._node.get_logger().error(f'Path planning failed (fraction: {response.fraction})')
            raise RuntimeError(f'Path planning failed: only {response.fraction * 100:.1f}% of the path was planned')

        self._node.get_logger().info(f'Path planning successful (fraction: {response.fraction})')
        trajectory = response.solution
        if vs != 1.0:
            trajectory = self.scale_trajectory_velocity(trajectory, vs)
        return trajectory

    def plan_trajectory_async(self, waypoints: list[Pose], *, max_step: Optional[float] = None,
                              jump_threshold: Optional[float] = None,) -> rclpy.Future:
        """카테시안 경로 계획 서비스 요청을 보내고 Future 를 반환한다.

        Args:
            waypoints: Cartesian path 를 따라갈 Pose 리스트.
            max_step: Override. ``None`` 이면 생성자 기본값 사용.
            jump_threshold: Override. ``None`` 이면 생성자 기본값 사용.

        Returns:
            서비스 요청 결과를 포함하는 Future 객체.

        Raises:
            ValueError: 입력 매개변수가 유효하지 않을 때.
            RuntimeError: 클라이언트가 이미 close() 되었을 때.
        """
        self._require_open()
        _validate_waypoints(waypoints)
        ms = self._resolve_max_step(max_step)
        jt = self._resolve_jump_threshold(jump_threshold)

        self._node.get_logger().info(f'Planning cartesian path through {len(waypoints)} waypoints...')

        request = GetCartesianPath.Request()
        request.header.frame_id = self._frame_id
        request.group_name = self._moveit_group_name
        request.waypoints = waypoints
        request.max_step = ms
        request.jump_threshold = jt
        return self._trajectory_planner.call_async(request)


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
        result_response = await_future_spin(self._node, get_result_future, remains, 'execute_trajectory result')

        result = result_response.result
        self._node.get_logger().info(f'Trajectory execution result error code: {result.error_code.val}')
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self._node.get_logger().error(f'Trajectory execution failed with code: {result.error_code.val}')
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
        goal_handle = await_future_spin(self._node, future, timeout, 'execute_trajectory goal response')

        if not goal_handle.accepted:
            self._node.get_logger().error('Goal rejected')
            raise RuntimeError('Goal rejected')
        self._node.get_logger().info('Goal accepted!')
        return goal_handle.get_result_async()


    def scale_trajectory_velocity(
        self,
        trajectory: RobotTrajectory,
        velocity_scaling: float,
    ) -> RobotTrajectory:
        """Trajectory 의 속도를 스케일링한 새 trajectory 를 반환한다.

        Args:
            trajectory: 스케일링할 원본 trajectory.
            velocity_scaling: 양수 스케일링 계수(1.0 = 원래 속도).

        Returns:
            스케일된 ``RobotTrajectory`` 의 deep copy.

        Raises:
            ValueError: trajectory 또는 ``velocity_scaling`` 이 유효하지 않을 때.
        """
        _validate_trajectory(trajectory)
        _validate_velocity_scaling(velocity_scaling)

        scaled = copy.deepcopy(trajectory)
        for point in scaled.joint_trajectory.points:
            total_ns = (
                point.time_from_start.sec * 1_000_000_000
                + point.time_from_start.nanosec
            )
            scaled_ns = int(total_ns / velocity_scaling)
            point.time_from_start.sec = int(scaled_ns // 1_000_000_000)
            point.time_from_start.nanosec = int(scaled_ns % 1_000_000_000)
            if point.velocities:
                point.velocities = [v * velocity_scaling for v in point.velocities]
            if point.accelerations:
                point.accelerations = [
                    a * velocity_scaling * velocity_scaling
                    for a in point.accelerations
                ]
        return scaled

    # ----- Internals ------------------------------------------------------

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError('MoveGroupClient has been closed')

    def _get_named_state_joint_values(
        self, name: str, *, timeout: float,
    ) -> dict[str, float]:
        """SRDF에서 지정 group_state의 joint 값 매핑을 반환한다. 결과는 캐시된다."""
        if self._group_states is None:
            self._group_states = self._fetch_and_parse_srdf(timeout=timeout)
        return self._lookup_cached_named_state(name)

    def _lookup_cached_named_state(self, name: str) -> dict[str, float]:
        """캐시된 ``self._group_states`` 에서 group_state를 조회한다.

        Raises:
            RuntimeError: 캐시가 아직 로드되지 않았을 때.
            ValueError: 해당 이름의 group_state가 그룹에 존재하지 않을 때.
        """
        if self._group_states is None:
            raise RuntimeError('SRDF group_states cache is not loaded')
        group = self._group_states.get(self._moveit_group_name, {})
        if name not in group:
            available = sorted(group.keys())
            raise ValueError(
                f'Named target "{name}" not found in group '
                f'"{self._moveit_group_name}". Available: {available}'
            )
        return group[name]

    def _fetch_srdf_async(self) -> tuple[Future, Client]:
        """``move_group`` 노드의 ``get_parameters`` 서비스를 비동기 호출한다.

        Returns:
            (``GetParameters.Response`` 를 담을 Future, 임시 생성된 서비스 클라이언트).
            호출자는 Future 완료 후 클라이언트를 ``destroy_client`` 로 정리해야 한다.
        """
        srdf_service = f'{self._move_group_node_name}/get_parameters'
        client = self._node.create_client(GetParameters, srdf_service)
        req = GetParameters.Request()
        req.names = ['robot_description_semantic']
        return client.call_async(req), client

    def _fetch_and_parse_srdf(
        self, *, timeout: float,
    ) -> dict[str, dict[str, dict[str, float]]]:
        """``move_group`` 노드의 ``robot_description_semantic`` 파라미터를 조회하여
        파싱한 group_state 매핑을 반환한다.

        Returns:
            ``{group_name: {state_name: {joint_name: value}}}`` 형태의 dict.
        """
        srdf_service = f'{self._move_group_node_name}/get_parameters'
        client = self._node.create_client(GetParameters, srdf_service)
        try:
            started_at = time.monotonic()
            if not client.wait_for_service(timeout_sec=min(timeout, 10.0)):
                raise TimeoutError(f'Timed out waiting for {srdf_service}')

            req = GetParameters.Request()
            req.names = ['robot_description_semantic']
            future = client.call_async(req)
            remains = check_timeout_and_get_remains(started_at, timeout)
            response = await_future_spin(
                self._node, future, remains, f'{srdf_service} response'
            )
        finally:
            try:
                self._node.destroy_client(client)
            except Exception:
                pass

        if not response.values:
            raise RuntimeError('Failed to retrieve robot_description_semantic')
        srdf_string = response.values[0].string_value
        if not srdf_string:
            raise RuntimeError('robot_description_semantic is empty')

        return _parse_srdf_group_states(srdf_string)

    def _resolve_fraction_threshold(self, override: Optional[float]) -> float:
        if override is None:
            return self._default_fraction_threshold
        _validate_fraction_threshold(override)
        return override

    def _resolve_velocity_scaling(self, override: Optional[float]) -> float:
        if override is None:
            return self._default_velocity_scaling
        _validate_velocity_scaling(override)
        return override

    def _resolve_max_step(self, override: Optional[float]) -> float:
        if override is None:
            return self._default_max_step
        _validate_max_step(override)
        return override

    def _resolve_jump_threshold(self, override: Optional[float]) -> float:
        if override is None:
            return self._default_jump_threshold
        _validate_jump_threshold(override)
        return override


def _validate_fraction_threshold(value: float) -> None:
    if not (0.0 <= value <= 1.0):
        raise ValueError('fraction_threshold must be within [0.0, 1.0]')


def _validate_velocity_scaling(value: float) -> None:
    if value <= 0.0:
        raise ValueError('velocity_scaling must be positive')


def _validate_max_step(value: float) -> None:
    if value <= 0.0:
        raise ValueError('max_step must be positive')


def _validate_jump_threshold(value: float) -> None:
    if value < 0.0:
        raise ValueError('jump_threshold must be non-negative')


def _validate_trajectory(trajectory: RobotTrajectory) -> None:
    if (
        not isinstance(trajectory, RobotTrajectory)
        or not hasattr(trajectory, 'joint_trajectory')
        or not trajectory.joint_trajectory.points
    ):
        raise ValueError('trajectory must contain joint_trajectory with points')


def _parse_srdf_group_states(
    srdf_string: str,
) -> dict[str, dict[str, dict[str, float]]]:
    """SRDF XML 문자열을 파싱하여 ``group_state`` 매핑을 반환한다.

    Returns:
        ``{group_name: {state_name: {joint_name: value}}}`` 형태의 dict.
    """
    try:
        root = ET.fromstring(srdf_string)
    except ET.ParseError as e:
        raise RuntimeError(f'Failed to parse SRDF XML: {e}') from e

    result: dict[str, dict[str, dict[str, float]]] = {}
    for gs in root.findall('group_state'):
        group = gs.get('group')
        state_name = gs.get('name')
        if not group or not state_name:
            continue
        joint_values: dict[str, float] = {}
        for j in gs.findall('joint'):
            jname = j.get('name')
            jval_str = j.get('value')
            if jname is None or jval_str is None:
                continue
            try:
                joint_values[jname] = float(jval_str)
            except ValueError:
                continue
        result.setdefault(group, {})[state_name] = joint_values
    return result


def _build_move_group_goal(*, group_name: str, joint_values: dict[str, float],
                           velocity_scaling: float, planning_time: float,
                           tolerance: float,) -> MoveGroup.Goal:
    """MoveGroup 액션 goal 메시지를 구성한다 (plan + execute 모드)."""
    request = MotionPlanRequest()
    request.group_name = group_name
    request.num_planning_attempts = 1
    request.allowed_planning_time = planning_time
    request.max_velocity_scaling_factor = velocity_scaling
    request.max_acceleration_scaling_factor = 1.0

    joint_constraints: list[JointConstraint] = []
    for jname, jval in joint_values.items():
        jc = JointConstraint()
        jc.joint_name = jname
        jc.position = jval
        jc.tolerance_above = tolerance
        jc.tolerance_below = tolerance
        jc.weight = 1.0
        joint_constraints.append(jc)

    constraints = Constraints()
    constraints.joint_constraints = joint_constraints
    request.goal_constraints = [constraints]

    goal = MoveGroup.Goal()
    goal.request = request
    goal.planning_options.plan_only = False
    return goal


def _validate_waypoints(waypoints: list[Pose]) -> None:
    """Cartesian path 용 waypoint 리스트의 유효성을 검증한다."""
    if not waypoints or len(waypoints) < 1:
        raise ValueError('At least one waypoint is required')

    for i, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, Pose):
            raise ValueError(f'Waypoint {i} is not a valid Pose message')

        pos = waypoint.position
        if not (math.isfinite(pos.x) and math.isfinite(pos.y) and math.isfinite(pos.z)):
            raise ValueError(
                f'Waypoint {i} has invalid position: x={pos.x}, y={pos.y}, z={pos.z}'
            )

        quat = waypoint.orientation
        if not all(math.isfinite(v) for v in (quat.x, quat.y, quat.z, quat.w)):
            raise ValueError(
                f'Waypoint {i} has invalid quaternion: '
                f'x={quat.x}, y={quat.y}, z={quat.z}, w={quat.w}'
            )

        quat_norm = math.sqrt(quat.x ** 2 + quat.y ** 2 + quat.z ** 2 + quat.w ** 2)
        if quat_norm < 0.001:
            raise ValueError(f'Waypoint {i} has zero quaternion (invalid orientation)')
        if abs(quat_norm - 1.0) > 0.01:
            raise ValueError(
                f'Waypoint {i} has non-normalized quaternion (norm={quat_norm:.6f}). '
                f'Expected norm=1.0 ± 0.01'
            )
