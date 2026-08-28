from __future__ import annotations

from typing import Optional

import abc
import copy
import math
import threading
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    RobotTrajectory,
)
from moveit_msgs.srv import GetCartesianPath
from sensor_msgs.msg import JointState
from rcl_interfaces.srv import GetParameters

from robot_control.ros2_utils import (
    await_future_spin, await_future_spin_nospin, check_timeout_and_get_remains
)


READY_TIMEOUT_SEC = 30.0
PLAN_TIMEOUT_SEC = 20.0
GOAL_ACCEPT_TIMEOUT_SEC = 10.0
TRAJECTORY_EXEC_TIMEOUT_SEC = 120.0
MOVE_GROUP_TIMEOUT_SEC = 120.0

# 지정하지 않은 관절의 현재값을 읽어 오는 토픽.
_JOINT_STATES_TOPIC = '/joint_states'

DEFAULT_FRACTION_THRESHOLD = 0.60
DEFAULT_VELOCITY_SCALING = 1.0
DEFAULT_MAX_STEP = 0.01
DEFAULT_JUMP_THRESHOLD = 5.0
DEFAULT_PLANNING_TIME = 5.0
DEFAULT_JOINT_TOLERANCE = 1e-4

DEFAULT_FRAME_ID = 'panda_link0'
DEFAULT_MOVEIT_GROUP_NAME = 'panda_arm'

# joint 값 목표에는 SRDF 이름이 없다. 오류 메시지에서 named target 과 구분하기 위한
# 라벨이며, `_extract_planned_trajectory` 등 이름을 요구하는 헬퍼에 넘긴다.
_JOINT_GOAL_LABEL = 'joint goal'

_CARTESIAN_PATH_SERVICE = '/compute_cartesian_path'
_EXECUTE_TRAJECTORY_ACTION = '/execute_trajectory'
_MOVE_GROUP_ACTION = '/move_action'
_MOVE_GROUP_NODE_NAME = '/move_group'


class MoveGroupClient(abc.ABC):
    """MoveIt2 motion planning 클라이언트의 **공통 인터페이스** (추상 클래스).

    계획(planning)과 SRDF 조회는 컨트롤러 종류와 무관하므로 이 클래스가
    구현하고, **실행(execution)은 서브클래스가 담당**한다.

    * :class:`~robot_control.moveit.move_group_jtc_client.MoveGroupJtcClient` —
      ``JointTrajectoryController`` 환경. ``MoveGroup`` / ``ExecuteTrajectory``
      액션으로 실행한다.
    * :class:`~robot_control.moveit.move_group_jgpc_client.MoveGroupJgpcClient` —
      ``JointGroupPositionController`` 등 forward command 컨트롤러 환경.
      계획된 궤적을 ``std_msgs/Float64MultiArray`` 명령 토픽으로 직접
      스트리밍한다.

    이 클래스는 직접 생성할 수 없다. 컨트롤러를 자동 판별해 알맞은
    구현을 생성하려면
    :func:`~robot_control.moveit.move_group_factory.create_move_group_client` 를 쓴다::

        from robot_control.moveit import create_move_group_client

        with create_move_group_client(node) as client:
            client.wait_until_ready()
            client.move_to_named_target('ready')

    감싸는 MoveIt2 인터페이스:

    * ``GetCartesianPath`` 서비스 (``/compute_cartesian_path``) — 카테시안
      waypoint 경로 계획.
    * ``MoveGroup`` 액션 (``/move_action``) — named target 계획
      (``plan_only=True``). JTC 구현은 실행까지 이 액션으로 수행한다.
    * ``get_parameters`` 서비스 (``/move_group/get_parameters``) — named
      target 조회를 위한 ``robot_description_semantic`` (SRDF) 획득.
      결과는 인스턴스에 캐시된다.

    공통 기능 요약:

    * **계획**: :meth:`plan_trajectory`, :meth:`plan_named_target` 및 그
      ``_async`` 변형.
    * **SRDF 조회**: :meth:`get_named_targets`, :meth:`get_all_named_targets`,
      :meth:`get_planning_groups`.
    * **Trajectory 유틸**: :meth:`scale_trajectory_velocity`.

    서브클래스가 구현하는 추상 실행 API:

    * :meth:`move_to_named_target`, :meth:`move_to_named_target_async`
    * :meth:`follow_trajectory`, :meth:`follow_trajectory_async`

    .. warning::
        추상 실행 API 의 **보장 수준은 구현마다 다르다**. JTC 구현은 컨트롤러가
        goal tolerance 를 검사한 결과를 돌려주지만, JGPC 구현은 open loop 라
        정상 반환이 목표 도달을 뜻하지 않는다. 도달 확인이 필요한 코드는
        서브클래스 타입을 명시적으로 요구하라.

    Lifecycle:
        생성자는 서비스/액션 **클라이언트 객체만** 생성하며, 실제 서버가
        준비될 때까지 대기하지 않는다. 서버 준비 여부는 :meth:`is_ready`
        로 즉시 확인하거나 :meth:`wait_until_ready` 로 블로킹 대기할 수
        있다. 준비 대기 없이 동기 메서드를 직접 호출해도 각 메서드의
        timeout 내에서 서버를 기다린다. 서브클래스는 자기 실행 경로가
        요구하는 서버를 :meth:`is_ready` / :meth:`wait_until_ready` 에
        추가로 반영한다.

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
        :meth:`move_to_named_target_async`, :meth:`plan_trajectory_async`)는
        Future 를 반환하므로 호출자가 자체 executor 에서 처리할 수 있다.
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
            move_group_action: ``MoveGroup`` 액션 이름. named target 계획/이동에
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
        # ReentrantCallbackGroup 이 필요하다. 기본 콜백 그룹(MutuallyExclusive)에서는
        # 결과를 기다리는 동안 cancel 응답이 같은 그룹에 갇혀 처리되지 않아
        # `cancel_goal_async()` 의 Future 가 영원히 완료되지 않는다.
        self._action_callback_group = ReentrantCallbackGroup()
        self._move_group_client: ActionClient = ActionClient(
            node, MoveGroup, move_group_action, callback_group=self._action_callback_group
        )
        self._move_group_action = move_group_action
        self._move_group_node_name = move_group_node_name
        # SRDF의 group_state 캐시: {group_name: {state_name: {joint_name: value}}}
        self._group_states: Optional[dict[str, dict[str, dict[str, float]]]] = None
        # SRDF에 정의된 planning group 이름 캐시. group_state 가 하나도 없는 그룹
        # (예: panda_arm_hand)은 위 캐시에 나타나지 않으므로 따로 보관한다.
        self._planning_groups: Optional[list[str]] = None

        # 진행 중인 액션 goal 핸들. :meth:`cancel` 이 취소를 보낼 대상이다.
        # 콜백(executor 스레드)과 cancel 호출(임의 스레드)이 동시에 접근하므로
        # 락으로 보호한다.
        self._active_goals: list = []
        self._goal_lock = threading.Lock()

    # ----- 진행 중 goal 추적 ------------------------------------------------

    def _track_goal(self, goal_handle) -> None:
        """수락된 goal 핸들을 추적 목록에 넣는다.

        goal 을 보내는 모든 경로가 이 메서드를 호출해야 :meth:`cancel` 이 동작한다.
        """
        with self._goal_lock:
            self._active_goals.append(goal_handle)

    def _untrack_goal(self, goal_handle) -> None:
        """완료된 goal 핸들을 추적 목록에서 제거한다."""
        with self._goal_lock:
            try:
                self._active_goals.remove(goal_handle)
            except ValueError:
                pass

    def _take_active_goals(self) -> list:
        """추적 중인 goal 을 모두 꺼내 목록을 비운다."""
        with self._goal_lock:
            goals, self._active_goals = self._active_goals, []
            return goals

    def has_active_goal(self) -> bool:
        """취소할 수 있는 진행 중 goal 이 있는지 반환한다."""
        with self._goal_lock:
            return bool(self._active_goals)

    def cancel(self) -> bool:
        """진행 중인 동작을 중단한다.

        구현별로 중단 수단이 다르다.

        - JTC: 추적 중인 MoveGroup / ExecuteTrajectory goal 에 취소를 보낸다.
        - JGPC: 명령 스트리밍을 멈춘다 (:meth:`stop_streaming`).

        **비동기다.** ``True`` 는 취소 요청을 보냈다는 뜻이지 로봇이 이미 멈췄다는
        뜻이 아니다. 감속 정지에는 시간이 걸리며, 정지 후 로봇은 경로 중간의
        불확정 자세에 있다.

        Returns:
            취소를 보낼 대상이 있었으면 ``True``, 진행 중인 동작이 없으면 ``False``.
        """
        goals = self._take_active_goals()
        if not goals:
            return False

        for handle in goals:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self._node.get_logger().warning(f'cancel_goal_async failed: {exc}')
        self._node.get_logger().info(f'cancel requested for {len(goals)} active goal(s)')
        return True

    # ----- Lifecycle ------------------------------------------------------

    def is_ready(self) -> bool:
        """계획에 필요한 서버가 모두 준비되었는지 즉시 반환한다(non-blocking).

        서브클래스는 자기 실행 경로가 요구하는 서버를 추가로 확인하도록
        이 메서드를 확장한다.
        """
        self._require_open()
        return (
            self._trajectory_planner.service_is_ready()
            and self._move_group_client.server_is_ready()
        )

    def wait_until_ready(self, timeout_sec: float = READY_TIMEOUT_SEC) -> None:
        """계획에 필요한 서버가 준비될 때까지 블로킹 대기한다.

        기본 구현은 ``GetCartesianPath`` 서비스와 ``MoveGroup`` 액션만
        기다린다. 실행에 필요한 서버는 서브클래스가 덧붙인다.

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
        self._wait_for_planning_servers(timeout_sec)

    def _wait_for_planning_servers(self, timeout_sec: float) -> float:
        """계획용 서버를 순차 대기하고 남은 시간(초)을 반환한다.

        서브클래스가 실행용 서버를 추가로 기다릴 때 남은 예산을 이어받도록
        분리해 둔 헬퍼다.
        """
        started = time.monotonic()

        if not self._trajectory_planner.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError(
                f'Timed out waiting for {_CARTESIAN_PATH_SERVICE} service: '
                f'timeout={timeout_sec}s'
            )

        remains = max(0.001, timeout_sec - (time.monotonic() - started))
        if not self._move_group_client.wait_for_server(timeout_sec=remains):
            raise TimeoutError(
                f'Timed out waiting for {self._move_group_action} action server: '
                f'timeout={timeout_sec}s'
            )
        return max(0.001, timeout_sec - (time.monotonic() - started))

    def close(self) -> None:
        """생성한 서비스/액션 클라이언트 리소스를 정리한다.

        멱등(idempotent): 두 번 호출해도 안전하다. 주입된 ``Node`` 자체는
        건드리지 않는다. 서브클래스는 자기 리소스를 :meth:`_close_resources`
        에서 정리한다.
        """
        if self._closed:
            return
        self._close_resources()
        self._closed = True

    def _close_resources(self) -> None:
        """공통 리소스를 정리한다. 서브클래스는 super() 호출 후 자기 것을 정리한다."""
        try:
            self._move_group_client.destroy()
        except Exception:
            pass
        try:
            self._node.destroy_client(self._trajectory_planner)
        except Exception:
            pass

    def destroy(self) -> None:
        """:meth:`close` 와 동일. ROS2 스타일 네이밍 별칭."""
        self.close()

    def __enter__(self) -> 'MoveGroupClient':
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # ----- High-level API -------------------------------------------------

    def get_named_targets(self, *, group: Optional[str] = None,
                          timeout: float = READY_TIMEOUT_SEC,
                          externally_spun: bool = False) -> list[str]:
        """SRDF에 등록된 named target(group_state) 이름 목록을 반환한다.

        첫 호출 시 ``move_group`` 노드의 ``robot_description_semantic`` 파라미터를
        조회하여 파싱/캐시한다. 이후 호출은 캐시로부터 즉시 반환된다.

        Args:
            group: 조회할 planning 그룹 이름. ``None`` 이면 생성자에 지정된
                ``moveit_group_name`` 을 사용한다.
            timeout: SRDF 조회 허용 시간(초). 캐시가 이미 채워진 경우 무시된다.
            externally_spun: executor 가 다른 스레드에서 Node 를 spin 중이면
                ``True``. 자체 spin 대신 콜백 완료를 기다린다.

        Returns:
            그룹에 등록된 group_state 이름을 사전순으로 정렬한 리스트. 해당
            그룹에 state 가 없으면 빈 리스트.

        Raises:
            TimeoutError: SRDF 조회가 시간 내에 완료되지 않을 때.
            RuntimeError: 클라이언트가 이미 close() 되었거나 SRDF 조회가 실패했을 때.
        """
        self._require_open()
        target_group = group if group is not None else self._moveit_group_name
        group_states, _ = self._ensure_srdf_cache(timeout=timeout,
                                                  externally_spun=externally_spun)
        return sorted(group_states.get(target_group, {}).keys())

    def get_all_named_targets(self, *, timeout: float = READY_TIMEOUT_SEC,
                              externally_spun: bool = False) -> dict[str, list[str]]:
        """SRDF에 등록된 **모든** planning 그룹의 named target 목록을 반환한다.

        :meth:`get_named_targets` 는 인자 없이 호출하면 생성자에 지정된 그룹
        하나만 조회한다(기본 ``panda_arm``). 어떤 그룹이 있는지 모르는 상태에서
        전체를 훑거나 그룹 이름 자체를 알아낼 때 이 메서드를 사용한다.

        첫 호출 시 ``move_group`` 노드에서 SRDF 를 조회하여 캐시하며, 캐시는
        :meth:`get_named_targets` 와 공유한다.

        Args:
            timeout: SRDF 조회 허용 시간(초). 캐시가 이미 채워진 경우 무시된다.
            externally_spun: executor 가 다른 스레드에서 Node 를 spin 중이면
                ``True``. 자체 spin 대신 콜백 완료를 기다린다.

        Returns:
            ``{group_name: [state_name, ...]}`` 형태의 dict. 그룹 이름과 각 그룹의
            state 이름 모두 사전순으로 정렬된다. 등록된 group_state 가 하나도
            없으면 빈 dict.

        Raises:
            TimeoutError: SRDF 조회가 시간 내에 완료되지 않을 때.
            RuntimeError: 클라이언트가 이미 close() 되었거나 SRDF 조회가 실패했을 때.
        """
        self._require_open()
        group_states, _ = self._ensure_srdf_cache(timeout=timeout,
                                                  externally_spun=externally_spun)
        return {group: sorted(states.keys()) for group, states in sorted(group_states.items())}

    def get_planning_groups(self, *, timeout: float = READY_TIMEOUT_SEC,
                            externally_spun: bool = False) -> list[str]:
        """SRDF에 정의된 **모든** planning group 이름을 사전순으로 반환한다.

        :meth:`get_all_named_targets` 의 키는 ``group_state`` 를 하나 이상 가진
        그룹만 포함한다. 반면 이 메서드는 SRDF 의 ``<group>`` 정의 자체를 읽으므로,
        state 가 없는 그룹(예: ``panda_arm_hand``)도 함께 반환한다.

        첫 호출 시 ``move_group`` 노드에서 SRDF 를 조회하여 캐시하며, 캐시는
        :meth:`get_named_targets` / :meth:`get_all_named_targets` 와 공유한다.

        Args:
            timeout: SRDF 조회 허용 시간(초). 캐시가 이미 채워진 경우 무시된다.
            externally_spun: executor 가 다른 스레드에서 Node 를 spin 중이면
                ``True``. 자체 spin 대신 콜백 완료를 기다린다.

        Returns:
            planning group 이름을 사전순으로 정렬한 리스트.

        Raises:
            TimeoutError: SRDF 조회가 시간 내에 완료되지 않을 때.
            RuntimeError: 클라이언트가 이미 close() 되었거나 SRDF 조회가 실패했을 때.
        """
        self._require_open()
        _, planning_groups = self._ensure_srdf_cache(timeout=timeout,
                                                     externally_spun=externally_spun)
        return sorted(planning_groups)

    # ----- 추상 실행 API (서브클래스 구현) ----------------------------------

    @abc.abstractmethod
    def move_to_named_target(self, name: str, *,
                             velocity_scaling: Optional[float] = None,
                             planning_time: float = DEFAULT_PLANNING_TIME,
                             tolerance: float = DEFAULT_JOINT_TOLERANCE,
                             timeout: float = MOVE_GROUP_TIMEOUT_SEC) -> None:
        """SRDF에 등록된 named target 으로 로봇을 이동시킨다(블로킹).

        실행 메커니즘과 **보장 수준은 구현마다 다르다** — 클래스 docstring 의
        경고를 참고한다.

        Args:
            name: SRDF의 group_state 이름 (예: "ready").
            velocity_scaling: 속도 스케일링 계수. ``None`` 이면 생성자 기본값 사용.
            planning_time: MoveGroup 계획 허용 시간(초).
            tolerance: 각 관절 목표의 허용 오차(라디안).
            timeout: 전체 동작에 허용할 최대 시간(초).

        Raises:
            ValueError: ``name`` 이 유효하지 않거나 SRDF에 존재하지 않을 때.
            TimeoutError: 각 단계가 시간 초과될 때.
            RuntimeError: 계획/실행이 실패했거나 클라이언트가 close() 되었을 때.
        """

    @abc.abstractmethod
    def move_to_named_target_async(self, name: str, *,
                                   velocity_scaling: Optional[float] = None,
                                   planning_time: float = DEFAULT_PLANNING_TIME,
                                   tolerance: float = DEFAULT_JOINT_TOLERANCE,
                                   externally_spun: bool = False) -> Future:
        """:meth:`move_to_named_target` 의 비동기 버전.

        Args:
            name: SRDF의 group_state 이름.
            velocity_scaling: 속도 스케일링 계수. ``None`` 이면 생성자 기본값 사용.
            planning_time: MoveGroup 계획 허용 시간(초).
            tolerance: 각 관절 목표의 허용 오차(라디안).
            externally_spun: executor 가 다른 스레드에서 Node 를 spin 중이면
                ``True``. 스트리밍 구현에서만 의미가 있고, 액션 구현은 무시한다.

        Returns:
            최종 결과를 담는 ``Future``.
        """

    @abc.abstractmethod
    def move_to_joints(self, joint_values: dict[str, float], *,
                       velocity_scaling: Optional[float] = None,
                       planning_time: float = DEFAULT_PLANNING_TIME,
                       tolerance: float = DEFAULT_JOINT_TOLERANCE,
                       timeout: float = MOVE_GROUP_TIMEOUT_SEC) -> None:
        """관절 목표값으로 이동한다(블로킹).

        :meth:`move_to_named_target` 과 같은 joint-space 경로이며, 목표를 SRDF
        이름이 아니라 값으로 직접 준다는 점만 다르다. 실행 메커니즘과 **보장
        수준은 구현마다 다르다** — 클래스 docstring 의 경고를 참고한다.

        Args:
            joint_values: ``{관절이름: 라디안}``. **planning group 에 속한 관절만**
                넣는다 — 그룹 밖 관절(예: ``panda_arm`` 에 대한 finger joint)을
                섞으면 계획이 실패한다.
            velocity_scaling: 속도 스케일링 계수. ``None`` 이면 생성자 기본값 사용.
            planning_time: MoveGroup 계획 허용 시간(초).
            tolerance: 각 관절 목표의 허용 오차(라디안).
            timeout: 계획 + 실행 전체에 허용할 최대 시간(초).

        Raises:
            ValueError: ``joint_values`` 가 비었거나 값이 유한하지 않을 때.
            TimeoutError: goal 수락 또는 실행 완료가 시간 초과될 때.
            RuntimeError: 계획/실행이 실패했거나 클라이언트가 close() 되었을 때.
        """

    @abc.abstractmethod
    def move_to_joints_async(self, joint_values: dict[str, float], *,
                             velocity_scaling: Optional[float] = None,
                             planning_time: float = DEFAULT_PLANNING_TIME,
                             tolerance: float = DEFAULT_JOINT_TOLERANCE,
                             externally_spun: bool = False) -> Future:
        """:meth:`move_to_joints` 의 비동기 버전.

        Args:
            joint_values: ``{관절이름: 라디안}``.
            velocity_scaling: 속도 스케일링 계수. ``None`` 이면 생성자 기본값 사용.
            planning_time: MoveGroup 계획 허용 시간(초).
            tolerance: 각 관절 목표의 허용 오차(라디안).
            externally_spun: executor 가 다른 스레드에서 Node 를 spin 중이면
                ``True``. 스트리밍 구현에서만 의미가 있고, 액션 구현은 무시한다.

        Returns:
            최종 결과를 담는 ``Future``.
        """

    @abc.abstractmethod
    def follow_trajectory(self, waypoints: list[Pose], *,
                          velocity_scaling: Optional[float] = None,
                          fraction_threshold: Optional[float] = None,
                          max_step: Optional[float] = None,
                          jump_threshold: Optional[float] = None,
                          timeout: float = MOVE_GROUP_TIMEOUT_SEC) -> None:
        """카테시안 waypoint 경로를 계획하고 실행한다(블로킹).

        실행 메커니즘과 **보장 수준은 구현마다 다르다**.

        Args:
            waypoints: 통과할 카테시안 pose 목록.
            velocity_scaling: 속도 스케일링 계수. ``None`` 이면 생성자 기본값 사용.
            fraction_threshold: 계획된 경로의 최소 허용 비율. ``None`` 이면 기본값.
            max_step: 카테시안 보간 최대 단계 간격(미터). ``None`` 이면 기본값.
            jump_threshold: 관절 공간 점프 임계값. ``None`` 이면 기본값.
            timeout: 전체 동작에 허용할 최대 시간(초).

        Raises:
            ValueError: waypoint 가 유효하지 않을 때.
            TimeoutError: 각 단계가 시간 초과될 때.
            RuntimeError: 계획 비율 미달이거나 실행이 실패했을 때.
        """

    @abc.abstractmethod
    def follow_trajectory_async(self, waypoints: list[Pose], *,
                                velocity_scaling: Optional[float] = None,
                                fraction_threshold: Optional[float] = None,
                                max_step: Optional[float] = None,
                                jump_threshold: Optional[float] = None,
                                externally_spun: bool = False) -> Future:
        """:meth:`follow_trajectory` 의 비동기 버전.

        Returns:
            최종 결과를 담는 ``Future``.
        """

    # ----- Named target — 계획 전용 (컨트롤러 무관) --------------------------

    def plan_named_target(self, name: str, *,
                          velocity_scaling: Optional[float] = None,
                          planning_time: float = DEFAULT_PLANNING_TIME,
                          tolerance: float = DEFAULT_JOINT_TOLERANCE,
                          timeout: float = MOVE_GROUP_TIMEOUT_SEC,) -> RobotTrajectory:
        """named target 으로의 joint-space 궤적을 **계획만** 하여 반환한다.

        :meth:`move_to_named_target` 과 달리 ``plan_only=True`` 로 요청하므로
        ``move_group`` 이 실행을 시도하지 않는다. 따라서
        ``FollowJointTrajectory`` 액션이 없는 컨트롤러
        (``position_controllers/JointGroupPositionController`` 등) 환경에서도
        정상 동작한다. 반환된 궤적은 :meth:`stream_trajectory` 로 실행한다.

        Args:
            name: SRDF의 group_state 이름 (예: "ready", "extended", "transport").
            velocity_scaling: 속도 스케일링 계수. ``None`` 이면 생성자 기본값 사용.
            planning_time: MoveGroup 계획 허용 시간(초).
            tolerance: 각 관절 목표의 허용 오차(라디안). 플래너는 이 오차 안에서
                멈출 수 있으므로, 정밀도가 필요하면 작게 준다.
            timeout: SRDF 조회 + 계획 전체에 허용할 최대 시간(초).

        Returns:
            계획된 ``RobotTrajectory``.

        Raises:
            ValueError: ``name`` 이 유효하지 않거나 SRDF에 존재하지 않을 때.
            TimeoutError: SRDF 조회, goal 수락, 또는 계획 완료가 시간 초과될 때.
            RuntimeError: 계획이 실패했거나 클라이언트가 close() 되었을 때.
        """
        self._require_open()
        if not isinstance(name, str) or not name:
            raise ValueError('name must be a non-empty string')

        vs = self._resolve_velocity_scaling(velocity_scaling)
        started_at = time.monotonic()

        remains = check_timeout_and_get_remains(started_at, timeout)
        joint_values = self._get_named_state_joint_values(name, timeout=remains)

        self._node.get_logger().info(f'Planning joint-space path to named target "{name}"...')

        goal_msg = _build_move_group_goal(
            group_name=self._moveit_group_name, joint_values=joint_values,
            velocity_scaling=vs, planning_time=planning_time, tolerance=tolerance,
            plan_only=True,
        )

        remains = check_timeout_and_get_remains(started_at, timeout)
        if not self._move_group_client.wait_for_server(timeout_sec=min(remains, 10.0)):
            raise TimeoutError(f'Timed out waiting for {self._move_group_action} action server')

        send_future = self._move_group_client.send_goal_async(goal_msg)
        remains = check_timeout_and_get_remains(started_at, timeout)
        goal_handle = await_future_spin(self._node, send_future, remains,
                                        'move_group goal response')
        if not goal_handle.accepted:
            self._node.get_logger().error('Move group plan-only goal rejected')
            raise RuntimeError('Move group plan-only goal rejected')

        result_future = goal_handle.get_result_async()
        remains = check_timeout_and_get_remains(started_at, timeout)
        result_response = await_future_spin(self._node, result_future, remains, 'move_group result')

        return _extract_planned_trajectory(result_response.result, name)

    def plan_named_target_async(self, name: str, *,
                                velocity_scaling: Optional[float] = None,
                                planning_time: float = DEFAULT_PLANNING_TIME,
                                tolerance: float = DEFAULT_JOINT_TOLERANCE,) -> Future:
        """:meth:`plan_named_target` 의 비동기 버전.

        SRDF 조회 → MoveGroup ``send_goal_async`` (plan_only) → 결과 콜백
        순으로 체인을 구성한다. Node 를 직접 spin 하지 않으므로 executor 가
        다른 스레드에서 spin 중인 환경에서 사용할 수 있다.

        Args:
            name: SRDF의 group_state 이름.
            velocity_scaling: 속도 스케일링 계수. ``None`` 이면 생성자 기본값 사용.
            planning_time: MoveGroup 계획 허용 시간(초).
            tolerance: 각 관절 목표의 허용 오차(라디안).

        Returns:
            계획된 ``RobotTrajectory`` 로 resolve 되는 ``Future``.

        Raises:
            ValueError: ``name`` 이 유효하지 않을 때(즉시 발생).
            RuntimeError: 클라이언트가 이미 close() 되었을 때(즉시 발생).
        """
        self._require_open()
        if not isinstance(name, str) or not name:
            raise ValueError('name must be a non-empty string')
        vs = self._resolve_velocity_scaling(velocity_scaling)

        result_future = Future()

        def _start_plan(joint_values: dict[str, float]) -> None:
            """joint 값 확보 후 plan-only goal 전송 단계로 진입."""
            try:
                goal_msg = _build_move_group_goal(
                    group_name=self._moveit_group_name, joint_values=joint_values,
                    velocity_scaling=vs, planning_time=planning_time, tolerance=tolerance,
                    plan_only=True,
                )
                self._node.get_logger().info(f'Planning to named target "{name}" (async)...')
                send_future = self._move_group_client.send_goal_async(goal_msg)
                send_future.add_done_callback(_on_goal_response)
            except Exception as exc:
                result_future.set_exception(exc)

        def _on_goal_response(future: Future) -> None:
            """plan-only goal 수락 콜백."""
            try:
                goal_handle = future.result()
                if not goal_handle.accepted:
                    result_future.set_exception(RuntimeError('Move group plan-only goal rejected'))
                    return
                goal_handle.get_result_async().add_done_callback(_on_plan_done)
            except Exception as exc:
                result_future.set_exception(exc)

        def _on_plan_done(future: Future) -> None:
            """계획 완료 콜백: 궤적을 result_future 에 설정."""
            try:
                result_future.set_result(_extract_planned_trajectory(future.result().result, name))
            except Exception as exc:
                result_future.set_exception(exc)

        if self._group_states is not None:
            _start_plan(self._lookup_cached_named_state(name))
            return result_future

        srdf_future, srdf_client = self._fetch_srdf_async()

        def _on_srdf_done(future: Future) -> None:
            """SRDF 조회 완료 콜백: 캐시 채우고 계획 단계로 진입."""
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
                    result_future.set_exception(RuntimeError('robot_description_semantic is empty'))
                    return
                self._store_srdf_cache(srdf_string)
                _start_plan(self._lookup_cached_named_state(name))
            except Exception as exc:
                result_future.set_exception(exc)
            finally:
                try:
                    self._node.destroy_client(srdf_client)
                except Exception:
                    pass

        srdf_future.add_done_callback(_on_srdf_done)
        return result_future

    def plan_joints(self, joint_values: dict[str, float], *,
                    velocity_scaling: Optional[float] = None,
                    planning_time: float = DEFAULT_PLANNING_TIME,
                    tolerance: float = DEFAULT_JOINT_TOLERANCE,
                    timeout: float = MOVE_GROUP_TIMEOUT_SEC,) -> RobotTrajectory:
        """관절 목표값으로의 joint-space 궤적을 **계획만** 하여 반환한다.

        :meth:`plan_named_target` 의 값 지정 판이다. SRDF 조회가 없으므로 그만큼
        단계가 짧다. 반환된 궤적은 :meth:`stream_trajectory` 로 실행한다.

        Args:
            joint_values: ``{관절이름: 라디안}``. planning group 에 속한 관절만 넣는다.
            velocity_scaling: 속도 스케일링 계수. ``None`` 이면 생성자 기본값 사용.
            planning_time: MoveGroup 계획 허용 시간(초).
            tolerance: 각 관절 목표의 허용 오차(라디안).
            timeout: 계획 전체에 허용할 최대 시간(초).

        Returns:
            계획된 ``RobotTrajectory``.

        Raises:
            ValueError: ``joint_values`` 가 유효하지 않을 때.
            TimeoutError: goal 수락 또는 계획 완료가 시간 초과될 때.
            RuntimeError: 계획이 실패했거나 클라이언트가 close() 되었을 때.
        """
        self._require_open()
        values = self._complete_joint_values(_validate_joint_values(joint_values))
        vs = self._resolve_velocity_scaling(velocity_scaling)
        started_at = time.monotonic()

        self._node.get_logger().info(
            f'Planning joint-space path to {len(values)} joint value(s)...'
        )
        goal_msg = _build_move_group_goal(
            group_name=self._moveit_group_name, joint_values=values,
            velocity_scaling=vs, planning_time=planning_time, tolerance=tolerance,
            plan_only=True,
        )

        remains = check_timeout_and_get_remains(started_at, timeout)
        if not self._move_group_client.wait_for_server(timeout_sec=min(remains, 10.0)):
            raise TimeoutError(f'Timed out waiting for {self._move_group_action} action server')

        send_future = self._move_group_client.send_goal_async(goal_msg)
        remains = check_timeout_and_get_remains(started_at, timeout)
        goal_handle = await_future_spin(self._node, send_future, remains,
                                        'move_group goal response')
        if not goal_handle.accepted:
            self._node.get_logger().error('Move group plan-only goal rejected')
            raise RuntimeError('Move group plan-only goal rejected')

        result_future = goal_handle.get_result_async()
        remains = check_timeout_and_get_remains(started_at, timeout)
        result_response = await_future_spin(self._node, result_future, remains, 'move_group result')

        return _extract_planned_trajectory(result_response.result, _JOINT_GOAL_LABEL)

    def plan_joints_async(self, joint_values: dict[str, float], *,
                          velocity_scaling: Optional[float] = None,
                          planning_time: float = DEFAULT_PLANNING_TIME,
                          tolerance: float = DEFAULT_JOINT_TOLERANCE,) -> Future:
        """:meth:`plan_joints` 의 비동기 버전.

        Args:
            joint_values: ``{관절이름: 라디안}``.
            velocity_scaling: 속도 스케일링 계수. ``None`` 이면 생성자 기본값 사용.
            planning_time: MoveGroup 계획 허용 시간(초).
            tolerance: 각 관절 목표의 허용 오차(라디안).

        Returns:
            계획된 ``RobotTrajectory`` 로 resolve 되는 ``Future``.

        Raises:
            ValueError: ``joint_values`` 가 유효하지 않을 때(즉시 발생).
            RuntimeError: 클라이언트가 이미 close() 되었을 때(즉시 발생).
        """
        self._require_open()
        values = self._complete_joint_values(_validate_joint_values(joint_values))
        vs = self._resolve_velocity_scaling(velocity_scaling)

        result_future = Future()

        def _on_goal_response(future: Future) -> None:
            """plan-only goal 수락 콜백."""
            try:
                goal_handle = future.result()
                if not goal_handle.accepted:
                    result_future.set_exception(RuntimeError('Move group plan-only goal rejected'))
                    return
                goal_handle.get_result_async().add_done_callback(_on_plan_done)
            except Exception as exc:
                result_future.set_exception(exc)

        def _on_plan_done(future: Future) -> None:
            """계획 완료 콜백: 궤적을 result_future 에 설정."""
            try:
                result_future.set_result(
                    _extract_planned_trajectory(future.result().result, _JOINT_GOAL_LABEL)
                )
            except Exception as exc:
                result_future.set_exception(exc)

        try:
            goal_msg = _build_move_group_goal(
                group_name=self._moveit_group_name, joint_values=values,
                velocity_scaling=vs, planning_time=planning_time, tolerance=tolerance,
                plan_only=True,
            )
            self._node.get_logger().info(
                f'Planning to {len(values)} joint value(s) (async)...'
            )
            self._move_group_client.send_goal_async(goal_msg).add_done_callback(_on_goal_response)
        except Exception as exc:
            result_future.set_exception(exc)
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

        future = self.plan_trajectory_async(waypoints, max_step=max_step,
                                            jump_threshold=jump_threshold)
        response = await_future_spin(self._node, future, timeout,
                                     'compute_cartesian_path service response')

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self._node.get_logger().error(
                f'Path planning returned error code: {response.error_code.val}')
            raise RuntimeError(f'Path planning failed with error code: {response.error_code.val}')

        if response.fraction < ft:
            self._node.get_logger().error(f'Path planning failed (fraction: {response.fraction})')
            raise RuntimeError(f'Path planning failed: only '
                               f'{response.fraction * 100:.1f}% of the path was planned')

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

        self._node.get_logger().info(
            f'Planning cartesian path through {len(waypoints)} waypoints...')

        request = GetCartesianPath.Request()
        request.header.frame_id = self._frame_id
        request.group_name = self._moveit_group_name
        request.waypoints = waypoints
        request.max_step = ms
        request.jump_threshold = jt
        return self._trajectory_planner.call_async(request)

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
        self._ensure_srdf_cache(timeout=timeout)
        return self._lookup_cached_named_state(name)

    # ------------------------------------------------------------------
    # 관절 목표 보완 — 지정하지 않은 관절은 **현재값으로 고정**한다
    # ------------------------------------------------------------------

    def group_joint_names(self, *, timeout: float = READY_TIMEOUT_SEC,
                          externally_spun: bool = False) -> list[str]:
        """planning group 에 속한 관절 이름 목록.

        SRDF 의 ``<group_state>`` 가 그룹의 관절을 전부 나열하므로 그 키를 쓴다.
        어느 상태를 보든 관절 집합은 같다. 이미 캐시된 SRDF 를 재사용하므로 추가
        조회가 없다.

        ``<chain>`` 정의를 관절로 푸는 방법(URDF 파싱)이나 컨트롤러의 ``joints``
        파라미터 조회는 쓰지 않는다 — 전자는 무겁고, 후자는 planning group 과
        컨트롤러가 1:1 이라는 보장이 없다.

        Raises:
            RuntimeError: 그룹에 ``group_state`` 가 하나도 없어 관절을 알 수 없을 때.
        """
        group_states, _ = self._ensure_srdf_cache(timeout=timeout,
                                                  externally_spun=externally_spun)
        for joints in group_states.get(self._moveit_group_name, {}).values():
            return list(joints)
        raise RuntimeError(
            f"cannot determine joints of planning group '{self._moveit_group_name}': "
            'SRDF defines no <group_state> for it. Specify every joint explicitly.'
        )

    def _current_joint_positions(self, *, timeout: float,
                                 externally_spun: bool = False) -> dict[str, float]:
        """``/joint_states`` 한 건을 받아 ``{관절이름: 위치}`` 로 돌려준다.

        캐시를 두지 않고 매번 새로 받는다 — 목표를 만드는 기준값이므로 낡으면
        엉뚱한 자세로 간다.
        """
        future: Future = Future()

        def _on_msg(msg: JointState) -> None:
            if not future.done():
                future.set_result(msg)

        sub = self._node.create_subscription(JointState, _JOINT_STATES_TOPIC, _on_msg, 10)
        try:
            wait = await_future_spin_nospin if externally_spun else await_future_spin
            msg = wait(self._node, future, timeout, f"'{_JOINT_STATES_TOPIC}'")
        finally:
            self._node.destroy_subscription(sub)
        if msg is None:
            raise RuntimeError(f'no message on {_JOINT_STATES_TOPIC}')
        return dict(zip(msg.name, msg.position))

    def _complete_joint_values(self, values: dict[str, float], *,
                               timeout: float = READY_TIMEOUT_SEC,
                               externally_spun: bool = False) -> dict[str, float]:
        """지정하지 않은 그룹 관절을 **현재값으로 채운다**.

        채우지 않으면 목표가 자세 하나가 아니라 "지정한 관절만 만족하는 자세의
        **집합**"이 되고, 플래너가 그중 아무거나 고른다. 실측에서 ``panda_joint1``
        하나만 준 호출이 나머지 6축을 최대 3.5 rad 움직여 엔드이펙터가 로봇 뒤쪽
        위로 넘어갔다 — 호출 전에 결과를 알 수 없다는 뜻이라 위험하다.

        그룹 밖 관절(예: ``panda_finger_*``)은 채우지 않는다. 섞이면 계획이
        실패한다.
        """
        names = self.group_joint_names(timeout=timeout, externally_spun=externally_spun)
        missing = [n for n in names if n not in values]
        if not missing:
            return values

        current = self._current_joint_positions(timeout=timeout,
                                                externally_spun=externally_spun)
        unknown = [n for n in missing if n not in current]
        if unknown:
            raise RuntimeError(
                f'cannot hold joint(s) {unknown} — not present in '
                f'{_JOINT_STATES_TOPIC}. Specify them explicitly.'
            )
        completed = {n: current[n] for n in missing}
        completed.update(values)
        self._node.get_logger().info(
            f'Holding {len(missing)} unspecified joint(s) at their current value: '
            f'{", ".join(missing)}'
        )
        return completed

    def _ensure_srdf_cache(
        self, *, timeout: float, externally_spun: bool = False,
    ) -> tuple[dict[str, dict[str, dict[str, float]]], list[str]]:
        """SRDF 캐시가 비어 있으면 채우고 (group_state 매핑, planning group 목록)을 반환한다.

        두 캐시는 같은 SRDF 문자열에서 함께 파싱되므로 항상 동시에 채워진다.
        """
        if self._group_states is None or self._planning_groups is None:
            self._store_srdf_cache(
                self._fetch_srdf_string(timeout=timeout, externally_spun=externally_spun)
            )
        return self._group_states or {}, self._planning_groups or []

    def _store_srdf_cache(self, srdf_string: str) -> None:
        """SRDF 문자열을 파싱하여 group_state / planning group 캐시를 함께 채운다."""
        self._group_states = _parse_srdf_group_states(srdf_string)
        self._planning_groups = _parse_srdf_planning_groups(srdf_string)

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

    def _fetch_srdf_string(self, *, timeout: float, externally_spun: bool = False) -> str:
        """``move_group`` 노드의 ``robot_description_semantic`` 파라미터 값을 조회한다.

        Args:
            timeout: 조회 허용 시간(초).
            externally_spun: executor 가 다른 스레드에서 Node 를 spin 중이면
                ``True``. 이때 자체 spin 을 하면 같은 Node 를 두 스레드에서 동시에
                spin 하게 되므로 콜백 대기 방식으로 바꾼다.

        Returns:
            SRDF XML 문자열.
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
            wait = await_future_spin_nospin if externally_spun else await_future_spin
            response = wait(self._node, future, remains, f'{srdf_service} response')
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

        return srdf_string

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


def _extract_planned_trajectory(result, name: str) -> RobotTrajectory:
    """MoveGroup plan-only 결과에서 궤적을 꺼내고 유효성을 확인한다.

    Raises:
        RuntimeError: MoveIt 오류 코드를 반환했거나 궤적이 비어 있을 때.
    """
    error_code = result.error_code.val
    if error_code != MoveItErrorCodes.SUCCESS:
        raise RuntimeError(f'Planning to named target "{name}" failed with code: {error_code}')

    trajectory = result.planned_trajectory
    if not trajectory.joint_trajectory.points:
        raise RuntimeError(f'Planning to named target "{name}" returned an empty trajectory')
    return trajectory


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


def _parse_srdf_planning_groups(srdf_string: str) -> list[str]:
    """SRDF XML 문자열에서 planning group 이름 목록을 파싱한다.

    복합 그룹(예: ``panda_arm_hand``)은 하위 그룹을 ``<group name="..."/>`` 로
    중첩 참조하므로, 트리 전체를 훑는 ``iter()`` 를 쓰면 같은 이름이 중복 수집된다.
    최상위 자식만 보는 ``findall()`` 을 사용해야 한다.

    Returns:
        SRDF에 정의된 group 이름 리스트 (문서 등장 순서).
    """
    try:
        root = ET.fromstring(srdf_string)
    except ET.ParseError as e:
        raise RuntimeError(f'Failed to parse SRDF XML: {e}') from e

    names: list[str] = []
    for g in root.findall('group'):
        name = g.get('name')
        if name and name not in names:
            names.append(name)
    return names


def _build_move_group_goal(*, group_name: str, joint_values: dict[str, float],
                           velocity_scaling: float, planning_time: float,
                           tolerance: float, plan_only: bool = False) -> MoveGroup.Goal:
    """MoveGroup 액션 goal 메시지를 구성한다.

    ``plan_only`` 가 ``False`` 면 계획 후 실행까지 수행하고(기본), ``True`` 면
    계획만 하고 궤적을 결과로 돌려준다. ``FollowJointTrajectory`` 를 제공하지
    않는 컨트롤러(JGPC 등) 에서는 ``True`` 로 계획만 받아 별도로 실행해야 한다.
    """
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
    goal.planning_options.plan_only = plan_only
    return goal


def _validate_joint_values(joint_values: dict[str, float]) -> dict[str, float]:
    """관절 목표값 dict 를 검증하고 float 로 정규화한다.

    관절 **이름**은 여기서 검사하지 않는다 — 유효한 이름 집합은 planning group 에
    달려 있고 클라이언트는 그 목록을 갖고 있지 않다. 그룹 밖 관절이 섞이면 MoveIt
    계획 단계에서 실패한다.

    Returns:
        ``{관절이름: float}``.

    Raises:
        ValueError: dict 가 아니거나, 비었거나, 이름/값이 유효하지 않을 때.
    """
    if not isinstance(joint_values, dict) or not joint_values:
        raise ValueError('joint_values must be a non-empty {joint_name: radians} mapping')

    normalized: dict[str, float] = {}
    for name, value in joint_values.items():
        if not isinstance(name, str) or not name:
            raise ValueError(f'invalid joint name: {name!r}')
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"joint '{name}' value must be a number, got {value!r}")
        if not math.isfinite(float(value)):
            raise ValueError(f"joint '{name}' value is not finite: {value!r}")
        normalized[name] = float(value)
    return normalized


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
