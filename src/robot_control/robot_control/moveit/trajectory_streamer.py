"""계획된 trajectory 를 forward_command_controller 계열 컨트롤러로 스트리밍한다.

``position_controllers/JointGroupPositionController`` (JGPC) 같은
``forward_command_controller`` 기반 컨트롤러는 ``FollowJointTrajectory`` 액션을
제공하지 않는다. 대신 ``std_msgs/Float64MultiArray`` 를 ``~/commands`` 로 받아
보간 없이 그대로 hardware 에 write 한다.

따라서 ``JointTrajectoryController`` 가 컨트롤러 내부에서 수행하던 "궤적의
시간 보간" 을 클라이언트가 대신 해야 한다. 본 모듈은 그 역할을 담당한다 —
계획된 ``RobotTrajectory`` 의 각 point 를 ``time_from_start`` 시각에 맞춰
명령 토픽으로 발행한다.

주의: ``Float64MultiArray`` 에는 joint 이름이 없다. 배열 순서는 컨트롤러의
``joints`` 파라미터 순서와 정확히 일치해야 하며, 어긋나면 엉뚱한 관절이
움직인다. :meth:`TrajectoryStreamer.resolve_joint_names` 가 컨트롤러의
``get_parameters`` 서비스에서 그 순서를 직접 읽어오는 이유다.
"""

from __future__ import annotations

from typing import Optional

import threading
import time

from rclpy.node import Node
from rclpy.task import Future
from moveit_msgs.msg import RobotTrajectory
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

from robot_control.ros2_utils import await_future_spin, await_future_spin_nospin

DEFAULT_COMMAND_TOPIC = '/panda_arm_controller/commands'

# 명령 메시지 형식. 컨트롤러/시뮬레이터마다 기대하는 타입이 달라 선택 가능하다.
#   float64_multi_array : ros2_control JointGroupPositionController (이름 없음, 순서 계약)
#   joint_state         : 토픽 연동형 시뮬레이터 (예: 펑션베이 /input/panda_joint)
COMMAND_FORMAT_FLOAT64_MULTI_ARRAY = 'float64_multi_array'
COMMAND_FORMAT_JOINT_STATE = 'joint_state'
VALID_COMMAND_FORMATS = (COMMAND_FORMAT_FLOAT64_MULTI_ARRAY, COMMAND_FORMAT_JOINT_STATE)
DEFAULT_JOINT_NAMES_TIMEOUT_SEC = 10.0
DEFAULT_QOS_DEPTH = 10

# 스트리밍 루프의 sleep 분해능. stop_event 반응성과 CPU 사용량의 절충값이다.
_SLEEP_SLICE_SEC = 0.002


class TrajectoryStreamer:
    """``RobotTrajectory`` 를 Float64MultiArray 명령 토픽으로 시간 맞춰 발행한다.

    ``MoveGroupClient`` 와 마찬가지로 Node 를 주입받아 사용하며, Node 자체를
    파괴하지 않는다. 발행만 하므로 spin 이 필요 없다 — 호출자가 Node 를
    spin 중이든 아니든(예: ``MultiThreadedExecutor`` 백그라운드 spin) 동일하게
    동작한다. 단 :meth:`resolve_joint_names` 는 서비스 호출이므로 Node 가
    spin 되고 있어야 한다.
    """

    def __init__(self, node: Node, *, command_topic: str = DEFAULT_COMMAND_TOPIC,
                 joint_names: Optional[list[str]] = None,
                 controller_node_name: Optional[str] = None,
                 command_format: str = COMMAND_FORMAT_FLOAT64_MULTI_ARRAY,
                 qos_depth: int = DEFAULT_QOS_DEPTH,
                 gravity_compensator=None) -> None:
        """스트리머를 초기화하고 명령 퍼블리셔를 생성한다.

        Args:
            node: 퍼블리셔를 올릴 ROS2 Node. 호출자가 생성해야 하며 파괴하지 않는다.
            command_topic: 컨트롤러의 명령 토픽 (``std_msgs/Float64MultiArray``).
            joint_names: 명령 배열의 joint 순서. ``None`` 이면 첫 스트리밍 시
                컨트롤러의 ``joints`` 파라미터에서 자동 조회한다.
            controller_node_name: ``joints`` 파라미터를 조회할 컨트롤러 노드 이름.
                ``None`` 이면 ``command_topic`` 에서 유도한다
                (``/panda_arm_controller/commands`` → ``/panda_arm_controller``).
            command_format: ``'float64_multi_array'`` (기본, ros2_control JGPC) 또는
                ``'joint_state'`` (토픽 연동형 시뮬레이터). 후자는 관절 이름을 함께
                실어 보내므로 순서 오배치에 강하다. **컨트롤러 노드가 없는 백엔드에서는
                ``joint_names`` 를 명시해야 한다** — 자동 조회할 대상이 없기 때문이다.
            qos_depth: 명령 퍼블리셔의 QoS depth.
            gravity_compensator: :class:`robot_control.gravity.GravityCompensator` 또는
                ``None``. 주면 **발행 직전** 모든 명령점에 ``τ_g(q)/Kp`` 를 더한다 —
                중력 보상이 없는 백엔드(펑션베이)가 지령보다 처지는 것을 상쇄한다.
                ``None`` 이 기본이며 그 경우 명령을 그대로 내보낸다.

        Raises:
            ValueError: 입력 매개변수가 유효하지 않을 때.
        """
        if not isinstance(node, Node):
            raise ValueError('node must be a valid ROS2 Node instance')
        if not command_topic or not command_topic.strip():
            raise ValueError('command_topic must be a non-empty string')
        if joint_names is not None and not joint_names:
            raise ValueError('joint_names must be None or a non-empty list')
        if command_format not in VALID_COMMAND_FORMATS:
            raise ValueError(f'command_format must be one of {list(VALID_COMMAND_FORMATS)}, '
                             f'got {command_format!r}')

        self._node = node
        self._gravity = gravity_compensator
        self._command_topic = command_topic.strip()
        self._joint_names: Optional[list[str]] = list(joint_names) if joint_names else None
        self._controller_node_name = (controller_node_name
                                      or _derive_controller_node_name(self._command_topic))
        self._stop_event = threading.Event()
        # 스트리밍 진행 여부. stream() 진입/이탈 시 갱신하며 cancel 판정에 쓰인다.
        self._streaming = threading.Event()
        self._closed = False
        self._command_format = command_format
        msg_type = (JointState if command_format == COMMAND_FORMAT_JOINT_STATE
                    else Float64MultiArray)
        self._publisher = node.create_publisher(msg_type, self._command_topic, qos_depth)

    # ----- 속성 -----------------------------------------------------------

    @property
    def command_topic(self) -> str:
        """명령을 발행하는 토픽 이름."""
        return self._command_topic

    @property
    def command_format(self) -> str:
        """명령 메시지 형식 (``float64_multi_array`` | ``joint_state``)."""
        return self._command_format

    @property
    def joint_names(self) -> Optional[list[str]]:
        """확정된 명령 배열의 joint 순서. 아직 조회 전이면 ``None``."""
        return list(self._joint_names) if self._joint_names else None

    @property
    def is_streaming(self) -> bool:
        """현재 궤적을 발행 중인지."""
        return self._streaming.is_set()

    # ----- Lifecycle ------------------------------------------------------

    def close(self) -> None:
        """퍼블리셔를 정리한다. 멱등(idempotent)."""
        if self._closed:
            return
        self._stop_event.set()
        try:
            self._node.destroy_publisher(self._publisher)
        except Exception:
            pass
        self._closed = True

    def __enter__(self) -> 'TrajectoryStreamer':
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # ----- High-level API -------------------------------------------------

    def resolve_joint_names(self, *, timeout: float = DEFAULT_JOINT_NAMES_TIMEOUT_SEC,
                            externally_spun: bool = False) -> list[str]:
        """컨트롤러의 ``joints`` 파라미터를 조회하여 명령 배열 순서를 확정한다.

        생성자에서 ``joint_names`` 를 명시했으면 서비스를 호출하지 않고 그 값을
        반환한다. 조회 결과는 캐시되므로 두 번째 호출부터는 즉시 반환된다.

        Args:
            timeout: 서비스 준비 대기 + 응답 대기 전체에 허용할 시간(초).
            externally_spun: Node 가 이미 다른 스레드에서 spin 중이면 ``True``
                (예: ``MultiThreadedExecutor``). ``True`` 면 직접 spin 하지 않고
                콜백만 기다린다 — ``False`` 로 두면 "node already spinning" 오류가
                난다. 아무도 spin 하지 않는 일반적인 동기 환경에서는 ``False``.

        Returns:
            컨트롤러가 기대하는 joint 이름 순서.

        Raises:
            TimeoutError: 서비스가 준비되지 않거나 응답이 시간 초과될 때.
            RuntimeError: ``joints`` 파라미터를 읽지 못했거나 close() 되었을 때.
        """
        self._require_open()
        if self._joint_names:
            return list(self._joint_names)

        service_name = f'{self._controller_node_name}/get_parameters'
        client = self._node.create_client(GetParameters, service_name)
        try:
            started_at = time.monotonic()
            if not client.wait_for_service(timeout_sec=timeout):
                raise TimeoutError(f'Timed out waiting for {service_name}')

            request = GetParameters.Request()
            request.names = ['joints']
            future = client.call_async(request)

            remains = max(0.001, timeout - (time.monotonic() - started_at))
            waiter = await_future_spin_nospin if externally_spun else await_future_spin
            response = waiter(self._node, future, remains, f'{service_name} response')
        finally:
            try:
                self._node.destroy_client(client)
            except Exception:
                pass

        if response is None or not response.values:
            raise RuntimeError(
                f'Failed to read "joints" parameter from {self._controller_node_name}'
            )
        names = list(response.values[0].string_array_value)
        if not names:
            raise RuntimeError(f'"joints" parameter of {self._controller_node_name} is empty')

        self._node.get_logger().info(
            f'Resolved command joint order from {self._controller_node_name}: {names}'
        )
        self._joint_names = names
        return list(names)

    def stream(self, trajectory: RobotTrajectory, *, time_scaling: float = 1.0,
               publish_rate: Optional[float] = None,
               stop_event: Optional[threading.Event] = None,
               joint_names_timeout: float = DEFAULT_JOINT_NAMES_TIMEOUT_SEC,
               externally_spun: bool = False) -> int:
        """계획된 trajectory 를 ``time_from_start`` 시각에 맞춰 발행한다(블로킹).

        각 point 를 궤적이 지정한 상대 시각에 발행하므로, 호출은 궤적 길이만큼
        블로킹된다. 첫 point 는 계획 시점의 현재 자세이므로 즉시 발행해도
        급격한 이동이 발생하지 않는다.

        ``publish_rate`` 를 주면 궤적 point 시각 대신 균일한 시간 격자를 만들어
        그 사이를 선형 보간해 발행한다. MoveIt 은 시간 파라미터화 단계에서 궤적을
        0.1초 격자로 리샘플하므로 원본 궤적은 약 10Hz 다. ``JointTrajectoryController``
        는 이 사이를 컨트롤러 주기로 보간해 주지만 스트리밍에는 그 주체가 없어
        계단식 명령이 되므로, 실기에서는 이 인자로 밀도를 올린다.

        Args:
            trajectory: ``plan_named_target`` / ``plan_trajectory`` 등이 반환한 궤적.
            time_scaling: 재생 시간 배율. ``1.0`` 이 궤적 원래 속도이고,
                ``2.0`` 이면 두 배 느리게 재생한다(안전 확인용).
            publish_rate: 발행 주파수(Hz). ``None`` (기본)이면 보간 없이 궤적
                point 시각에만 발행한다(기존 동작). 값을 주면 ``1/publish_rate``
                간격 격자로 위치를 선형 보간해 발행한다. ``time_scaling`` 이
                적용된 뒤의 재생 시간축을 기준으로 한다.
            stop_event: 설정되면 스트리밍을 즉시 중단한다. ``None`` 이면
                :meth:`stop` 으로 제어되는 내부 이벤트만 사용한다.
            joint_names_timeout: joint 순서 자동 조회에 허용할 시간(초).
            externally_spun: joint 순서 자동 조회 시 Node 를 직접 spin 할지 여부.
                :meth:`resolve_joint_names` 참고. joint 순서가 이미 확정되어
                있으면 사용되지 않는다.

        Returns:
            실제로 발행한 명령 개수. ``publish_rate`` 를 주면 궤적 point 수와
            다르다.

        Raises:
            ValueError: 궤적이 비었거나, 컨트롤러가 요구하는 joint 이 궤적에
                없거나, ``publish_rate`` 가 양수가 아닐 때.
            TimeoutError: joint 순서 자동 조회가 시간 초과될 때.
            RuntimeError: 스트리머가 이미 close() 되었을 때.
        """
        self._require_open()
        if time_scaling <= 0.0:
            raise ValueError('time_scaling must be positive')
        if publish_rate is not None and publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive or None')

        joint_trajectory = _extract_joint_trajectory(trajectory)
        if not joint_trajectory.points:
            raise ValueError('trajectory has no points to stream')

        names = self._joint_names or self.resolve_joint_names(timeout=joint_names_timeout,
                                                              externally_spun=externally_spun)
        indices = _build_index_map(names, list(joint_trajectory.joint_names))

        # (발행 시각, 명령 배열) 목록으로 미리 펼친다. 보간 여부와 무관하게
        # 이후 루프가 동일해지므로 sleep/중단 처리가 한 벌로 끝난다.
        samples = _build_samples(joint_trajectory.points, indices, time_scaling, publish_rate)

        self._stop_event.clear()
        self._streaming.set()
        events = [self._stop_event] if stop_event is None else [self._stop_event, stop_event]

        rate_note = '' if publish_rate is None else f' at {publish_rate:g} Hz (interpolated)'
        self._node.get_logger().info(
            f'Streaming {len(samples)} command(s) from '
            f'{len(joint_trajectory.points)} trajectory point(s) '
            f'to {self._command_topic}{rate_note}'
        )

        published = 0
        started_at = time.monotonic()
        try:
            for target_elapsed, data in samples:
                if not _sleep_until(started_at + target_elapsed, events):
                    self._node.get_logger().warning(
                        f'Trajectory streaming stopped after {published} command(s)'
                    )
                    return published
                self._publisher.publish(self._build_command(data))
                published += 1
        finally:
            # 정상 종료·중단·예외 어느 경로로 빠져나가든 진행 표시를 내린다.
            self._streaming.clear()

        self._node.get_logger().info(f'Trajectory streaming done: {published} command(s)')
        return published

    def stream_async(self, trajectory: RobotTrajectory, *, time_scaling: float = 1.0,
                     publish_rate: Optional[float] = None,
                     stop_event: Optional[threading.Event] = None,
                     joint_names_timeout: float = DEFAULT_JOINT_NAMES_TIMEOUT_SEC,
                     externally_spun: bool = False) -> Future:
        """:meth:`stream` 을 별도 스레드에서 실행하고 Future 를 반환한다.

        Tk GUI 처럼 호출 스레드를 블로킹할 수 없는 환경에서 사용한다.

        Args:
            trajectory: 스트리밍할 궤적.
            time_scaling: 재생 시간 배율.
            publish_rate: 발행 주파수(Hz). ``None`` 이면 보간하지 않는다.
                :meth:`stream` 참고.
            stop_event: 외부 중단 이벤트.
            joint_names_timeout: joint 순서 자동 조회에 허용할 시간(초).
            externally_spun: :meth:`resolve_joint_names` 참고. executor 가 다른
                스레드에서 Node 를 spin 중이면 ``True``.

        Returns:
            발행한 명령 개수로 resolve 되는 ``Future``. 실패 시 예외가 설정된다.

        Raises:
            RuntimeError: 스트리머가 이미 close() 되었을 때(즉시 발생).
        """
        self._require_open()
        result_future = Future()

        def _run() -> None:
            try:
                published = self.stream(trajectory, time_scaling=time_scaling,
                                        publish_rate=publish_rate,
                                        stop_event=stop_event,
                                        joint_names_timeout=joint_names_timeout,
                                        externally_spun=externally_spun)
                result_future.set_result(published)
            except Exception as exc:   # noqa: BLE001
                result_future.set_exception(exc)

        threading.Thread(target=_run, name='trajectory-streamer', daemon=True).start()
        return result_future

    def stop(self) -> None:
        """진행 중인 스트리밍을 중단시킨다. 다음 :meth:`stream` 호출에는 영향이 없다."""
        self._stop_event.set()

    # ----- 내부 -----------------------------------------------------------

    def _apply_gravity(self, positions: list[float]) -> list[float]:
        """설정돼 있으면 명령점에 ``τ_g(q)/Kp`` 를 더한다.

        **모든 점에 적용한다** — 정착 오차만이 아니라 이동 중 처짐도 같은 식이라,
        마지막 점에만 얹으면 과도 구간의 손끝 말림이 그대로 남는다.

        보상기의 관절 수가 명령과 다르면 **조용히 건너뛰지 않고 예외를 낸다** — 어긋난
        채 일부만 보상하면 엉뚱한 관절에 오프셋이 실린다.
        """
        if self._gravity is None:
            return positions
        return self._gravity.compensate(positions)

    def _build_command(self, positions: list[float]):
        """설정된 형식으로 명령 메시지를 만든다.

        ``joint_state`` 형식은 관절 이름을 함께 싣는다. 수신 측이 이름을 무시하고
        순서만 보더라도, 이름이 있으면 `ros2 topic echo` 로 순서 오배치를 눈으로
        확인할 수 있다.
        """
        positions = self._apply_gravity(positions)
        if self._command_format == COMMAND_FORMAT_JOINT_STATE:
            msg = JointState()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.name = list(self._joint_names or [])
            msg.position = list(positions)
            return msg
        return Float64MultiArray(data=positions)

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError('TrajectoryStreamer has been closed')


def _derive_controller_node_name(command_topic: str) -> str:
    """``/foo/commands`` 형태의 명령 토픽에서 컨트롤러 노드 이름을 유도한다."""
    trimmed = command_topic.rstrip('/')
    if '/' not in trimmed.lstrip('/'):
        raise ValueError(f'Cannot derive controller node name from topic: {command_topic!r}')
    return trimmed.rsplit('/', 1)[0]


def _extract_joint_trajectory(trajectory: RobotTrajectory) -> JointTrajectory:
    """``RobotTrajectory`` / ``JointTrajectory`` 중 무엇을 받아도 관절 궤적을 꺼낸다."""
    if isinstance(trajectory, JointTrajectory):
        return trajectory
    joint_trajectory = getattr(trajectory, 'joint_trajectory', None)
    if joint_trajectory is None:
        raise ValueError('trajectory must be a RobotTrajectory or JointTrajectory message')
    return joint_trajectory


def _build_index_map(command_joint_names: list[str],
                     trajectory_joint_names: list[str]) -> list[int]:
    """컨트롤러 joint 순서 → 궤적 내 인덱스 매핑을 만든다.

    ``Float64MultiArray`` 에는 이름이 없으므로 이 재배열을 생략하면 엉뚱한
    관절이 움직인다.
    """
    missing = [name for name in command_joint_names if name not in trajectory_joint_names]
    if missing:
        raise ValueError(
            f'Trajectory is missing joints required by the controller: {missing}. '
            f'trajectory joints={trajectory_joint_names}'
        )
    return [trajectory_joint_names.index(name) for name in command_joint_names]


def _time_from_start_sec(point) -> float:
    """``JointTrajectoryPoint.time_from_start`` 를 초 단위 float 로 변환한다."""
    return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9


def _build_samples(points: list, indices: list[int], time_scaling: float,
                   publish_rate: Optional[float]) -> list[tuple[float, list[float]]]:
    """발행할 ``(상대 시각(초), 명령 배열)`` 목록을 만든다.

    ``publish_rate`` 가 ``None`` 이면 궤적 point 를 그대로 쓰고, 값이 있으면
    ``1/publish_rate`` 간격 격자에 위치를 선형 보간한다. 어느 쪽이든 시각에는
    ``time_scaling`` 이 이미 반영된 값이 담긴다.
    """
    times = [_time_from_start_sec(p) * time_scaling for p in points]
    if publish_rate is None:
        return [(t, [p.positions[i] for i in indices]) for t, p in zip(times, points)]

    dt = 1.0 / publish_rate
    duration = times[-1]
    # 마지막 격자점이 duration 에 못 미쳐도 종료 위치는 반드시 발행되도록,
    # 격자를 만든 뒤 duration 을 덧붙인다(중복이면 생략).
    grid = [k * dt for k in range(int(duration / dt) + 1)]
    if not grid or grid[-1] < duration:
        grid.append(duration)

    samples: list[tuple[float, list[float]]] = []
    seg = 0
    for t in grid:
        # times 는 단조 증가이므로 구간 포인터만 앞으로 밀면 된다 O(n+m).
        while seg + 1 < len(times) - 1 and times[seg + 1] < t:
            seg += 1
        t0, t1 = times[seg], times[seg + 1] if seg + 1 < len(times) else times[seg]
        p0, p1 = points[seg], points[min(seg + 1, len(points) - 1)]
        span = t1 - t0
        ratio = 0.0 if span <= 0.0 else min(1.0, max(0.0, (t - t0) / span))
        samples.append((t, [
            p0.positions[i] + (p1.positions[i] - p0.positions[i]) * ratio for i in indices
        ]))
    return samples


def _sleep_until(deadline: float, events: list[threading.Event]) -> bool:
    """``time.monotonic()`` 기준 deadline 까지 대기한다.

    Returns:
        정상적으로 대기를 마쳤으면 ``True``, 중단 이벤트로 깨어났으면 ``False``.
    """
    while True:
        if any(event.is_set() for event in events):
            return False
        remains = deadline - time.monotonic()
        if remains <= 0.0:
            return True
        time.sleep(min(remains, _SLEEP_SLICE_SEC))
