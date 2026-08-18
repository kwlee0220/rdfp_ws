from __future__ import annotations

from typing import Optional

from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory

from rdfp.moveit.move_group_client import (
    DEFAULT_JOINT_TOLERANCE,
    DEFAULT_PLANNING_TIME,
    MOVE_GROUP_TIMEOUT_SEC,
    PLAN_TIMEOUT_SEC,
    TRAJECTORY_EXEC_TIMEOUT_SEC,
    MoveGroupClient,
)
from rdfp.moveit.trajectory_streamer import DEFAULT_COMMAND_TOPIC, TrajectoryStreamer


class MoveGroupJgpcClient(MoveGroupClient):
    """forward command 컨트롤러 환경용 :class:`MoveGroupClient` 구현.

    계획은 MoveIt 에 맡기고 **실행만 직접** 한다. 계획된 궤적의 각 point 를
    ``time_from_start`` 시각에 맞춰 ``std_msgs/Float64MultiArray`` 명령 토픽
    (기본 ``/panda_arm_controller/commands``)으로 발행한다. 즉
    ``JointTrajectoryController`` 가 컨트롤러 내부에서 하던 시간 보간을
    클라이언트가 대신 수행하는 구조다.

    ``position_controllers/JointGroupPositionController`` 처럼
    ``FollowJointTrajectory`` 액션을 제공하지 않는 컨트롤러
    (``panda_jgpc_mock.launch.py``) 를 위한 구현이다.

    .. warning::
        **open loop 다.** 명령을 발행할 뿐 컨트롤러의 도달 여부를 확인하지
        않으므로, 정상 반환이 목표 도달을 뜻하지 않는다. 또한 명령 배열에는
        관절 이름이 없어 **순서가 어긋나면 조용히 엉뚱한 관절이 움직인다**
        (순서는 컨트롤러의 ``joints`` 파라미터에서 자동 조회한다). 궤적의
        velocity/acceleration 필드는 사용하지 않고 position 만 발행한다.

    도달 확인이나 컨트롤러 수준 보간이 필요하면
    :class:`~rdfp.moveit.move_group_jtc_client.MoveGroupJtcClient` 를 쓴다.
    """

    def __init__(self, node: Node, *,
                 arm_command_topic: str = DEFAULT_COMMAND_TOPIC,
                 arm_command_joint_names: Optional[list[str]] = None,
                 **kwargs) -> None:
        """JGPC 클라이언트를 초기화한다.

        Args:
            node: 서비스/액션 클라이언트를 올릴 ROS2 Node.
            arm_command_topic: ``std_msgs/Float64MultiArray`` 명령 토픽.
            arm_command_joint_names: 명령 배열의 joint 순서. ``None`` 이면 첫
                스트리밍 시 컨트롤러의 ``joints`` 파라미터에서 자동 조회한다.
            **kwargs: :class:`MoveGroupClient` 생성자 인자.
        """
        super().__init__(node, **kwargs)
        # 퍼블리셔를 미리 만들지 않도록 스트리머는 lazy 생성한다.
        self._arm_command_topic = arm_command_topic
        self._arm_command_joint_names = arm_command_joint_names
        self._streamer: Optional[TrajectoryStreamer] = None

    # ----- Lifecycle ------------------------------------------------------

    def _close_resources(self) -> None:
        """공통 리소스에 더해 스트리머(명령 퍼블리셔)를 정리한다."""
        if self._streamer is not None:
            try:
                self._streamer.close()
            except Exception:
                pass
        super()._close_resources()

    # ----- 실행 API 구현 (계획 → command 스트리밍) --------------------------

    def move_to_named_target(self, name: str, *,
                             velocity_scaling: Optional[float] = None,
                             planning_time: float = DEFAULT_PLANNING_TIME,
                             tolerance: float = DEFAULT_JOINT_TOLERANCE,
                             time_scaling: float = 1.0,
                             publish_rate: Optional[float] = None,
                             timeout: float = MOVE_GROUP_TIMEOUT_SEC) -> None:
        """named target 으로 이동한다 — 계획은 MoveIt, 실행은 command 스트리밍.

        발행한 point 개수가 필요하면 :meth:`move_to_named_target_streamed` 를
        직접 호출한다.
        """
        self.move_to_named_target_streamed(name, velocity_scaling=velocity_scaling,
                                           planning_time=planning_time, tolerance=tolerance,
                                           time_scaling=time_scaling, publish_rate=publish_rate,
                                           timeout=timeout)

    def move_to_named_target_async(self, name: str, *,
                                   velocity_scaling: Optional[float] = None,
                                   planning_time: float = DEFAULT_PLANNING_TIME,
                                   tolerance: float = DEFAULT_JOINT_TOLERANCE,
                                   time_scaling: float = 1.0,
                                   publish_rate: Optional[float] = None,
                                   externally_spun: bool = False) -> Future:
        """:meth:`move_to_named_target` 의 비동기 버전.

        Returns:
            발행한 point 개수로 resolve 되는 ``Future``.
        """
        return self.move_to_named_target_streamed_async(
            name, velocity_scaling=velocity_scaling, planning_time=planning_time,
            tolerance=tolerance, time_scaling=time_scaling, publish_rate=publish_rate,
            externally_spun=externally_spun
        )

    def move_to_joints(self, joint_values: dict[str, float], *,
                       velocity_scaling: Optional[float] = None,
                       planning_time: float = DEFAULT_PLANNING_TIME,
                       tolerance: float = DEFAULT_JOINT_TOLERANCE,
                       time_scaling: float = 1.0,
                       publish_rate: Optional[float] = None,
                       timeout: float = MOVE_GROUP_TIMEOUT_SEC) -> None:
        """관절 목표값으로 이동한다 — 계획은 MoveIt, 실행은 command 스트리밍.

        **open loop 다.** 정상 반환은 명령을 다 발행했다는 뜻이지 목표에 도달했다는
        뜻이 아니다. 발행한 point 개수가 필요하면
        :meth:`move_to_joints_streamed` 를 직접 호출한다.
        """
        self.move_to_joints_streamed(joint_values, velocity_scaling=velocity_scaling,
                                     planning_time=planning_time, tolerance=tolerance,
                                     time_scaling=time_scaling, publish_rate=publish_rate,
                                     timeout=timeout)

    def move_to_joints_async(self, joint_values: dict[str, float], *,
                             velocity_scaling: Optional[float] = None,
                             planning_time: float = DEFAULT_PLANNING_TIME,
                             tolerance: float = DEFAULT_JOINT_TOLERANCE,
                             time_scaling: float = 1.0,
                             publish_rate: Optional[float] = None,
                             externally_spun: bool = False) -> Future:
        """:meth:`move_to_joints` 의 비동기 버전.

        Returns:
            발행한 point 개수로 resolve 되는 ``Future``.
        """
        return self.move_to_joints_streamed_async(
            joint_values, velocity_scaling=velocity_scaling, planning_time=planning_time,
            tolerance=tolerance, time_scaling=time_scaling, publish_rate=publish_rate,
            externally_spun=externally_spun
        )

    def move_to_joints_streamed(self, joint_values: dict[str, float], *,
                                velocity_scaling: Optional[float] = None,
                                planning_time: float = DEFAULT_PLANNING_TIME,
                                tolerance: float = DEFAULT_JOINT_TOLERANCE,
                                time_scaling: float = 1.0,
                                publish_rate: Optional[float] = None,
                                timeout: float = MOVE_GROUP_TIMEOUT_SEC,) -> int:
        """관절 목표값으로 이동하고 **발행한 point 개수**를 반환한다.

        :meth:`move_to_named_target_streamed` 의 값 지정 판이다.

        Args:
            joint_values: ``{관절이름: 라디안}``. planning group 에 속한 관절만 넣는다.
            velocity_scaling: 속도 스케일링 계수. ``None`` 이면 생성자 기본값 사용.
            planning_time: MoveGroup 계획 허용 시간(초).
            tolerance: 각 관절 목표의 허용 오차(라디안).
            time_scaling: 재생 시간 배율. ``2.0`` 이면 두 배 느리게 재생한다.
            publish_rate: 명령 발행 주기(Hz). ``None`` 이면 궤적의 원래 point 시각을
                따른다.
            timeout: 계획에 허용할 최대 시간(초). 스트리밍 시간은 궤적 길이가
                정하므로 여기에 포함되지 않는다.

        Returns:
            실제로 발행한 point 개수.
        """
        trajectory = self.plan_joints(joint_values, velocity_scaling=velocity_scaling,
                                      planning_time=planning_time, tolerance=tolerance,
                                      timeout=timeout)
        published = self.stream_trajectory(trajectory, time_scaling=time_scaling,
                                           publish_rate=publish_rate)
        self._node.get_logger().info('Reached joint goal via command streaming')
        return published

    def move_to_joints_streamed_async(self, joint_values: dict[str, float], *,
                                      velocity_scaling: Optional[float] = None,
                                      planning_time: float = DEFAULT_PLANNING_TIME,
                                      tolerance: float = DEFAULT_JOINT_TOLERANCE,
                                      time_scaling: float = 1.0,
                                      publish_rate: Optional[float] = None,
                                      externally_spun: bool = False,) -> Future:
        """:meth:`move_to_joints_streamed` 의 비동기 버전.

        계획은 콜백 체인으로, 스트리밍은 별도 스레드에서 수행한다.

        Returns:
            발행한 point 개수로 resolve 되는 ``Future``.
        """
        self._require_open()
        result_future = Future()
        plan_future = self.plan_joints_async(joint_values, velocity_scaling=velocity_scaling,
                                             planning_time=planning_time, tolerance=tolerance)

        def _on_plan_done(future: Future) -> None:
            """계획 완료 콜백: 스트리밍 스레드로 넘긴다."""
            try:
                exc = future.exception()
                if exc is not None:
                    result_future.set_exception(exc)
                    return
                stream_future = self._get_streamer().stream_async(
                    future.result(), time_scaling=time_scaling, publish_rate=publish_rate,
                    externally_spun=externally_spun
                )
                stream_future.add_done_callback(_on_stream_done)
            except Exception as exc:
                result_future.set_exception(exc)

        def _on_stream_done(future: Future) -> None:
            """스트리밍 완료 콜백: 최종 결과를 result_future 에 설정."""
            try:
                exc = future.exception()
                if exc is not None:
                    result_future.set_exception(exc)
                    return
                self._node.get_logger().info('Reached joint goal via command streaming')
                result_future.set_result(future.result())
            except Exception as exc:
                result_future.set_exception(exc)

        plan_future.add_done_callback(_on_plan_done)
        return result_future

    def follow_trajectory(self, waypoints: list[Pose], *,
                          velocity_scaling: Optional[float] = None,
                          fraction_threshold: Optional[float] = None,
                          max_step: Optional[float] = None,
                          jump_threshold: Optional[float] = None,
                          time_scaling: float = 1.0,
                          publish_rate: Optional[float] = None,
                          timeout: float = PLAN_TIMEOUT_SEC + TRAJECTORY_EXEC_TIMEOUT_SEC) -> None:
        """카테시안 경로를 계획하고 command 스트리밍으로 실행한다.

        발행한 point 개수가 필요하면 :meth:`follow_trajectory_streamed` 를
        직접 호출한다.
        """
        self.follow_trajectory_streamed(waypoints, velocity_scaling=velocity_scaling,
                                        fraction_threshold=fraction_threshold, max_step=max_step,
                                        jump_threshold=jump_threshold, time_scaling=time_scaling,
                                        publish_rate=publish_rate, timeout=timeout)

    def follow_trajectory_streamed(self, waypoints: list[Pose], *,
                                   velocity_scaling: Optional[float] = None,
                                   fraction_threshold: Optional[float] = None,
                                   max_step: Optional[float] = None,
                                   jump_threshold: Optional[float] = None,
                                   time_scaling: float = 1.0,
                                   publish_rate: Optional[float] = None,
                                   timeout: float = PLAN_TIMEOUT_SEC + TRAJECTORY_EXEC_TIMEOUT_SEC
                                   ) -> int:
        """카테시안 경로를 계획하고 스트리밍 실행한 뒤 발행한 point 수를 반환한다.

        :meth:`MoveGroupJtcClient.follow_trajectory` 의 JGPC 판이다. 계획은
        ``GetCartesianPath`` 서비스로 동일하게 수행하고, 실행만
        ``ExecuteTrajectory`` 액션 대신 command 스트리밍으로 바꾼다.

        Args:
            waypoints: 통과할 카테시안 pose 목록.
            velocity_scaling: 속도 스케일링 계수. ``None`` 이면 생성자 기본값 사용.
            fraction_threshold: 계획된 경로의 최소 허용 비율. ``None`` 이면 기본값.
            max_step: 카테시안 보간 최대 단계 간격(미터). ``None`` 이면 기본값.
            jump_threshold: 관절 공간 점프 임계값. ``None`` 이면 기본값.
            time_scaling: 재생 시간 배율. ``2.0`` 이면 두 배 느리게 재생한다.
            timeout: 계획에 허용할 최대 시간(초). 스트리밍 시간은 궤적 길이가
                결정하므로 여기에 포함되지 않는다.

        Returns:
            실제로 발행한 point 개수.

        Raises:
            ValueError: waypoint 가 유효하지 않을 때.
            TimeoutError: 계획이 시간 초과될 때.
            RuntimeError: 계획 비율이 임계값에 못 미치거나 클라이언트가 close() 되었을 때.
        """
        self._node.get_logger().info(
            f'Moving through {len(waypoints)} cartesian waypoints via command streaming...'
        )
        trajectory = self.plan_trajectory(waypoints, velocity_scaling=velocity_scaling,
                                          fraction_threshold=fraction_threshold, max_step=max_step,
                                          jump_threshold=jump_threshold, timeout=timeout)
        published = self.stream_trajectory(trajectory, time_scaling=time_scaling,
                                           publish_rate=publish_rate)
        self._node.get_logger().info(
            f'Cartesian path streaming done: {published} command(s)'
        )
        return published

    def follow_trajectory_async(self, waypoints: list[Pose], *,
                                velocity_scaling: Optional[float] = None,
                                fraction_threshold: Optional[float] = None,
                                max_step: Optional[float] = None,
                                jump_threshold: Optional[float] = None,
                                time_scaling: float = 1.0,
                                publish_rate: Optional[float] = None,
                                externally_spun: bool = False) -> Future:
        """:meth:`follow_trajectory` 의 비동기 버전.

        ``plan_trajectory_async`` → 계획 완료 콜백(검증/스케일링) → 스트리밍
        스레드 순으로 체인을 구성한다.

        Returns:
            발행한 point 개수로 resolve 되는 ``Future``.
        """
        self._require_open()
        vs = self._resolve_velocity_scaling(velocity_scaling)
        ft = self._resolve_fraction_threshold(fraction_threshold)

        self._node.get_logger().info(
            f'Moving through {len(waypoints)} cartesian waypoints via streaming (async)...'
        )

        result_future = Future()
        plan_future = self.plan_trajectory_async(waypoints, max_step=max_step,
                                                 jump_threshold=jump_threshold)

        def _on_plan_done(future: Future) -> None:
            """계획 서비스 응답 콜백: 결과 검증 후 스트리밍 스레드로 넘긴다."""
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
                        f'Path planning failed (fraction: {response.fraction})'
                    )
                    result_future.set_exception(RuntimeError(
                        f'Path planning failed: only '
                        f'{response.fraction * 100:.1f}% of the path was planned'
                    ))
                    return

                self._node.get_logger().info(
                    f'Path planning successful (fraction: {response.fraction})'
                )
                trajectory = response.solution
                if vs != 1.0:
                    trajectory = self.scale_trajectory_velocity(trajectory, vs)

                stream_future = self._get_streamer().stream_async(
                    trajectory, time_scaling=time_scaling, publish_rate=publish_rate,
                    externally_spun=externally_spun
                )
                stream_future.add_done_callback(_on_stream_done)
            except Exception as exc:
                result_future.set_exception(exc)

        def _on_stream_done(future: Future) -> None:
            """스트리밍 완료 콜백: 최종 결과를 result_future 에 설정."""
            try:
                exc = future.exception()
                if exc is not None:
                    result_future.set_exception(exc)
                    return
                published = future.result()
                self._node.get_logger().info(
                    f'Cartesian path streaming done: {published} point(s)'
                )
                result_future.set_result(published)
            except Exception as exc:
                result_future.set_exception(exc)

        plan_future.add_done_callback(_on_plan_done)
        return result_future

    # ----- 스트리밍 기본기 --------------------------------------------------

    def stream_trajectory(self, trajectory: RobotTrajectory, *, time_scaling: float = 1.0,
                          publish_rate: Optional[float] = None,
                          externally_spun: bool = False) -> int:
        """계획된 궤적을 컨트롤러의 Float64MultiArray 명령 토픽으로 발행한다(블로킹).

        ``JointTrajectoryController`` 가 컨트롤러 내부에서 하던 궤적 시간 보간을
        클라이언트가 대신 수행한다. 각 point 를 ``time_from_start`` 시각에 맞춰
        발행하므로 호출은 궤적 길이만큼 블로킹된다.

        Args:
            trajectory: :meth:`plan_named_target` / :meth:`plan_trajectory` 가
                반환한 궤적.
            time_scaling: 재생 시간 배율. ``2.0`` 이면 두 배 느리게 재생한다.
            externally_spun: Node 가 이미 다른 스레드에서 spin 중이면 ``True``.
                명령 배열 joint 순서를 컨트롤러에서 자동 조회할 때만 영향을 준다.

        Returns:
            실제로 발행한 point 개수.

        Raises:
            ValueError: 궤적이 비었거나 컨트롤러가 요구하는 joint 이 궤적에 없을 때.
            TimeoutError: joint 순서 자동 조회가 시간 초과될 때.
            RuntimeError: 클라이언트가 이미 close() 되었을 때.
        """
        self._require_open()
        return self._get_streamer().stream(trajectory, time_scaling=time_scaling,
                                           publish_rate=publish_rate,
                                           externally_spun=externally_spun)

    def move_to_named_target_streamed(self, name: str, *,
                                      velocity_scaling: Optional[float] = None,
                                      planning_time: float = DEFAULT_PLANNING_TIME,
                                      tolerance: float = DEFAULT_JOINT_TOLERANCE,
                                      time_scaling: float = 1.0,
                                      publish_rate: Optional[float] = None,
                                      timeout: float = MOVE_GROUP_TIMEOUT_SEC,) -> int:
        """named target 으로 이동한다 — 계획은 MoveIt, 실행은 직접 스트리밍.

        :meth:`move_to_named_target` 의 JGPC 판이다. ``FollowJointTrajectory``
        액션을 쓰지 않으므로 ``position_controllers/JointGroupPositionController``
        같은 forward command 컨트롤러 환경에서 동작한다.

        Args:
            name: SRDF의 group_state 이름.
            velocity_scaling: 속도 스케일링 계수. ``None`` 이면 생성자 기본값 사용.
            planning_time: MoveGroup 계획 허용 시간(초).
            tolerance: 각 관절 목표의 허용 오차(라디안).
            time_scaling: 재생 시간 배율. ``2.0`` 이면 두 배 느리게 재생한다.
            timeout: SRDF 조회 + 계획에 허용할 최대 시간(초). 스트리밍 시간은
                궤적 길이에 의해 결정되므로 여기에 포함되지 않는다.

        Returns:
            실제로 발행한 point 개수.

        Raises:
            ValueError: ``name`` 이 유효하지 않거나 SRDF에 존재하지 않을 때.
            TimeoutError: SRDF 조회 또는 계획이 시간 초과될 때.
            RuntimeError: 계획이 실패했거나 클라이언트가 close() 되었을 때.
        """
        trajectory = self.plan_named_target(name, velocity_scaling=velocity_scaling,
                                            planning_time=planning_time, tolerance=tolerance,
                                            timeout=timeout)
        published = self.stream_trajectory(trajectory, time_scaling=time_scaling,
                                           publish_rate=publish_rate)
        self._node.get_logger().info(f'Reached named target "{name}" via command streaming')
        return published

    def move_to_named_target_streamed_async(self, name: str, *,
                                            velocity_scaling: Optional[float] = None,
                                            planning_time: float = DEFAULT_PLANNING_TIME,
                                            tolerance: float = DEFAULT_JOINT_TOLERANCE,
                                            time_scaling: float = 1.0,
                                            publish_rate: Optional[float] = None,
                                            externally_spun: bool = False,) -> Future:
        """:meth:`move_to_named_target_streamed` 의 비동기 버전.

        계획은 콜백 체인으로, 스트리밍은 별도 스레드에서 수행한다. Tk GUI 처럼
        호출 스레드를 블로킹할 수 없는 환경에서 사용한다.

        Args:
            name: SRDF의 group_state 이름.
            velocity_scaling: 속도 스케일링 계수. ``None`` 이면 생성자 기본값 사용.
            planning_time: MoveGroup 계획 허용 시간(초).
            tolerance: 각 관절 목표의 허용 오차(라디안).
            time_scaling: 재생 시간 배율.
            externally_spun: executor 가 다른 스레드에서 Node 를 spin 중이면 ``True``.

        Returns:
            발행한 point 개수로 resolve 되는 ``Future``.

        Raises:
            ValueError: ``name`` 이 유효하지 않을 때(즉시 발생).
            RuntimeError: 클라이언트가 이미 close() 되었을 때(즉시 발생).
        """
        self._require_open()
        result_future = Future()
        plan_future = self.plan_named_target_async(name, velocity_scaling=velocity_scaling,
                                                   planning_time=planning_time, tolerance=tolerance)

        def _on_plan_done(future: Future) -> None:
            """계획 완료 콜백: 스트리밍 스레드로 넘긴다."""
            try:
                exc = future.exception()
                if exc is not None:
                    result_future.set_exception(exc)
                    return
                stream_future = self._get_streamer().stream_async(
                    future.result(), time_scaling=time_scaling, publish_rate=publish_rate,
                    externally_spun=externally_spun
                )
                stream_future.add_done_callback(_on_stream_done)
            except Exception as exc:
                result_future.set_exception(exc)

        def _on_stream_done(future: Future) -> None:
            """스트리밍 완료 콜백: 최종 결과를 result_future 에 설정."""
            try:
                exc = future.exception()
                if exc is not None:
                    result_future.set_exception(exc)
                    return
                self._node.get_logger().info(f'Reached named target "{name}" via command streaming')
                result_future.set_result(future.result())
            except Exception as exc:
                result_future.set_exception(exc)

        plan_future.add_done_callback(_on_plan_done)
        return result_future

    def stop_streaming(self) -> None:
        """진행 중인 궤적 스트리밍을 중단시킨다. 스트리밍 중이 아니면 무시된다."""
        if self._streamer is not None:
            self._streamer.stop()

    def cancel(self) -> bool:
        """진행 중인 동작을 중단한다 (:meth:`MoveGroupClient.cancel` 구현).

        JGPC 는 명령 토픽 스트리밍으로 실행하므로 스트리밍을 멈추는 것이 곧 중단이다.
        계획 단계에서 MoveGroup 액션을 쓰는 경로도 있어 base 의 goal 취소도 함께
        수행한다.

        **주의**: 스트리밍을 멈추면 마지막으로 보낸 명령 위치에서 정지한다. 컨트롤러가
        그 목표를 유지하므로 감속 프로파일 없이 멈추는 셈이다.

        Returns:
            스트리밍이나 goal 중 하나라도 중단 대상이 있었으면 ``True``.
        """
        streaming = self._streamer is not None and self._streamer.is_streaming
        self.stop_streaming()
        # 계획 단계의 MoveGroup goal 도 취소한다.
        cancelled_goal = super().cancel()
        return streaming or cancelled_goal

    def _get_streamer(self) -> TrajectoryStreamer:
        """궤적 스트리머를 lazy 생성한다 (명령 퍼블리셔도 이때 만들어진다)."""
        if self._streamer is None:
            self._streamer = TrajectoryStreamer(self._node, command_topic=self._arm_command_topic,
                                                joint_names=self._arm_command_joint_names)
        return self._streamer
