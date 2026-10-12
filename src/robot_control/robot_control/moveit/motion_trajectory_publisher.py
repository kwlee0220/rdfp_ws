"""실행 궤적(L2) 채널 `motion/trajectory` 발행부 (R1).

연산 하나를 실행할 때 **컨트롤러로 보낸 궤적**을 한 건 발행한다. 학습 데이터의
관절-계획 레이블이 여기서 나온다.

**발행 지점이 세 곳인데 겹치지 않는다.** 궤적이 만들어지는 경로가 넷이지만
컨트롤러로 넘기는 자리는 셋으로 모인다:

* JTC (isaac) — `MoveGroupJtcClient` 의 `ExecuteTrajectory` goal 전송 두 자리
  (`execute_trajectory_async`, `follow_trajectory_async`)
* JGPC (펑션베이) — `TrajectoryStreamer.stream` 한 자리 (비동기판이 이것을 부른다)

`execute_trajectory` 는 JTC 클라이언트에만 있고 스트리밍은 JGPC 클라이언트에만
있으므로 한 연산이 두 번 발행되지 않는다.

설계: docs/learning/학습_샘플_사양.md §3.1~§3.3
"""
from __future__ import annotations

from typing import Optional, Union

from copy import deepcopy

import uuid

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from moveit_msgs.msg import RobotTrajectory
from rdfp_msgs.msg import MotionTrajectory
from trajectory_msgs.msg import JointTrajectory

#: 규약에 맞춘 **루트 상대** 이름. 앞에 `/` 를 붙이지 않는다 — 로봇별 네임스페이스가
#: 나중에 자동으로 붙어야 하기 때문이다 (docs/topic_naming_contract.md §2).
MOTION_TRAJECTORY_TOPIC = 'motion/trajectory'

#: 한 연산에 한 건이라 트래픽이 없다시피 하지만, **놓치면 그 연산의 레이블이 통째로
#: 사라지므로** RELIABLE 이다. VOLATILE 인 것은 늦게 붙은 구독자가 지난 연산의 궤적을
#: 받으면 안 되기 때문이다 — 이 채널은 상태가 아니라 사건이다.
DEFAULT_QOS_DEPTH = 10


def apply_time_scaling(jt: JointTrajectory, time_scaling: float) -> JointTrajectory:
    """`time_from_start` 에 스트리밍 재생 배율을 반영한 **새 궤적**을 만든다.

    **왜 필요한가** — `TrajectoryStreamer` 의 `time_scaling` 은 궤적을 고치지 않고
    **발행 일정**에만 곱한다(`_build_samples` 의 `time_from_start * time_scaling`).
    그대로 실으면 `time_from_start` 가 실제 실행 시간과 달라져, 레이블을 만드는 쪽이
    t₀ + time_from_start 로 계산한 시각이 틀리게 된다. 메시지 계약은 "time_from_start
    는 두 배율을 적용한 실제 실행 시간"이므로(설계 §3.1) 여기서 맞춘다.

    `velocity_scaling` 쪽은 이미 궤적에 반영돼 있어(`scale_trajectory_velocity` 또는
    MoveIt 시간 파라미터화) 손대지 않는다.

    ``time_scaling`` 이 2.0 이면 두 배 느리게 재생한다는 뜻이므로 시간은 2 배,
    속도는 1/2, 가속도는 1/4 이 된다.
    """
    if time_scaling == 1.0:
        return jt
    out = JointTrajectory()
    out.joint_names = list(jt.joint_names)
    out.points = []
    for p in jt.points:
        q = deepcopy(p)
        total_ns = p.time_from_start.sec * 1_000_000_000 + p.time_from_start.nanosec
        scaled_ns = int(total_ns * time_scaling)
        q.time_from_start.sec = int(scaled_ns // 1_000_000_000)
        q.time_from_start.nanosec = int(scaled_ns % 1_000_000_000)
        if p.velocities:
            q.velocities = [v / time_scaling for v in p.velocities]
        if p.accelerations:
            q.accelerations = [a / (time_scaling * time_scaling) for a in p.accelerations]
        out.points.append(q)
    return out


def trajectory_duration_sec(jt: JointTrajectory) -> float:
    """궤적 길이(초) — 마지막 경유점의 ``time_from_start``.

    **R2 가 중단 시각을 궤적 안으로 자를 때 쓴다**(`clamp_truncated_at`). 여기 두는
    것은 그 값이 R1 이 실은 시간축의 성질이기 때문이다 — 두 배율이 이미 반영된 뒤다.
    """
    if not jt.points:
        return 0.0
    last = jt.points[-1].time_from_start
    return last.sec + last.nanosec / 1_000_000_000


def _make_qos(depth: int = DEFAULT_QOS_DEPTH) -> QoSProfile:
    return QoSProfile(depth=depth,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST)


class MotionTrajectoryPublisher:
    """`motion/trajectory` 발행자. 생산자(클라이언트 · 스트리머)가 하나씩 갖는다."""

    def __init__(self, node: Node, *, topic: str = MOTION_TRAJECTORY_TOPIC,
                 qos_depth: int = DEFAULT_QOS_DEPTH) -> None:
        self._node = node
        self._topic = topic
        self._publisher = node.create_publisher(MotionTrajectory, topic, _make_qos(qos_depth))

    @staticmethod
    def new_operation_id() -> str:
        """연산 식별자를 만든다. R2 가 같은 값으로 짝짓는다."""
        return str(uuid.uuid4())

    def publish(self, trajectory: Union[RobotTrajectory, JointTrajectory], *,
                velocity_scaling: float = 1.0, time_scaling: float = 1.0,
                operation_id: Optional[str] = None,
                stamp=None) -> Optional[str]:
        """실행 궤적을 한 건 발행하고 연산 식별자를 돌려준다.

        **예외를 밖으로 내보내지 않는다.** 이 발행은 기록용이므로 실패해도 팔 동작은
        이어져야 한다(설계 §6). 실패하면 경고를 남기고 ``None`` 을 돌려주며, 그 연산은
        적재 검증(미구현 목록 I5)에서 빠진 것으로 드러난다.

        Args:
            trajectory: 컨트롤러로 보내는 바로 그 궤적(L2). ``RobotTrajectory`` 와
                ``JointTrajectory`` 를 모두 받는다.
            velocity_scaling: 계획에 적용한 속도 배율. 안 쓴 경로는 1.0.
            time_scaling: 스트리밍 재생 배율. 안 쓴 경로는 1.0.
            operation_id: 미리 만들어 둔 식별자. ``None`` 이면 여기서 만든다.
            stamp: **t₀** — 궤적의 0초가 시작된 시각. ``None`` 이면 노드 시계의
                현재 시각을 쓴다. 호출자는 **컨트롤러로 넘기기 직전**에 불러야
                그 값이 t₀ 가 된다 (설계 §3.5).

        Returns:
            연산 식별자. 발행에 실패하면 ``None``.
        """
        try:
            jt = trajectory.joint_trajectory if isinstance(trajectory, RobotTrajectory) \
                else trajectory
            msg = MotionTrajectory()
            msg.header.stamp = stamp if stamp is not None else self._node.get_clock().now().to_msg()
            msg.header.frame_id = ''
            msg.operation_id = operation_id or self.new_operation_id()
            msg.velocity_scaling = float(velocity_scaling)
            msg.time_scaling = float(time_scaling)
            # 시각은 바깥 header 하나에만 둔다 — 두 곳에 있으면 어느 쪽이 t₀ 인지
            # 모호해진다. ⚠️ **`msg.trajectory = jt` 뒤에 header 를 지우면 안 된다** —
            # ROS 2 파이썬 메시지는 참조 대입이라 호출자가 컨트롤러로 보내는 바로 그
            # 궤적의 header 까지 지워진다. 그래서 껍데기를 새로 만든다.
            out = apply_time_scaling(jt, float(time_scaling))
            if out is jt:                   # 배율이 1.0 이라 새 궤적을 안 만든 경우
                out = JointTrajectory()
                out.joint_names = list(jt.joint_names)
                out.points = jt.points      # 참조 — 내용을 바꾸지 않으므로 복사하지 않는다
            msg.trajectory = out
            self._publisher.publish(msg)
            return msg.operation_id
        except Exception as exc:                       # noqa: BLE001 - 기록이 동작을 막지 않는다
            self._node.get_logger().warning(
                f'Failed to publish motion trajectory on {self._topic}: '
                f'{type(exc).__name__}: {exc}')
            return None
