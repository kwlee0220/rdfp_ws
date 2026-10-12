"""연산 이벤트 채널 `motion/event` 발행부 (R2).

팔 동작 연산 **하나가 끝날 때마다** 한 건 발행한다. R1(`motion/trajectory`)이
"무엇을 보냈나"를 싣고 이쪽이 "어떤 연산이었고 어떻게 끝났나"를 싣는다. 둘은
``operation_id`` 로 짝짓는다.

**왜 있는가** — 실행 궤적만으로는 레이블을 만들 수 없다. 수집 설계 D5 의 규칙이
*"중단 · 실패 뒤에는 중단 시각의 궤적 값"* 인데, 궤적은 끝까지 갔을 때의 모습이라
멈췄는지도 언제 멈췄는지도 모른다. 그 정보가 없으면 중단된 연산 뒤의 레이블이
**팔이 가지 않은 자리**를 가리킨다.

**발행 지점은 셋이고 모두 클라이언트 쪽이다** — 끝을 아는 자리가 거기뿐이다.

* isaac — ``MoveGroupJtcClient`` 의 실행 결과 콜백 (관절 공간 · 데카르트가 합류한다)
* 펑션베이 실행 — ``TrajectoryStreamer.stream`` (**잘렸는지를 아는 유일한 자리**다)
* 펑션베이 계획 — ``MoveGroupJgpcClient`` 의 계획 콜백 (``failed`` 만 낸다)

트윈에서 내지 않는 이유는 트윈의 결과가 사실과 다를 수 있어서다 — 계획 실패와 실행
중단을 못 가르고, 잘린 스트리밍을 성공으로 보고하며, 타임아웃은 기다리기를 포기한
것이지 동작이 끝난 것이 아니다.

설계: docs/learning/학습_샘플_사양.md
"""

from __future__ import annotations

from typing import Optional, Sequence

from dataclasses import dataclass

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from rdfp_msgs.msg import MotionEvent

#: 규약에 맞춘 **루트 상대** 이름. 앞에 `/` 를 붙이지 않는다 — 로봇별 네임스페이스가
#: 나중에 자동으로 붙어야 하기 때문이다 (docs/topic_naming_contract.md §2).
MOTION_EVENT_TOPIC = 'motion/event'

#: R1 과 같다. 한 연산에 한 건이라 트래픽이 없다시피 하지만 **놓치면 그 연산의 레이블
#: 규칙을 못 고르므로** RELIABLE 이고, 늦게 붙은 구독자가 지난 연산의 결과를 받으면
#: 안 되므로 VOLATILE 이다 — 이 채널은 상태가 아니라 사건이다.
DEFAULT_QOS_DEPTH = 10

# ----- 고정 어휘 --------------------------------------------------------------
# **문자열 리터럴을 호출부에 흩지 않는다.** 오타는 에러가 아니라 "레이블이 이상하다"
# 로만 드러난다.

#: 연산 종류. 트윈 REST 이름이 아니라 **이동 방식**이다.
OPERATION_NAMED_TARGET = 'named_target'
OPERATION_JOINTS = 'joints'
OPERATION_CARTESIAN = 'cartesian'
OPERATIONS = frozenset({OPERATION_NAMED_TARGET, OPERATION_JOINTS, OPERATION_CARTESIAN})

#: 결과. 기준은 **궤적을 컨트롤러로 넘기려 했는가** 다.
RESULT_SUCCEEDED = 'succeeded'   # 넘겼고 끝까지 실행했다
RESULT_ABORTED = 'aborted'       # 넘겼는데 끝까지 못 갔다 (goal 거부 · 시작 오차 포함)
RESULT_FAILED = 'failed'         # 넘기지도 못했다 (계획 실패 · 궤적 검증 실패)
RESULTS = frozenset({RESULT_SUCCEEDED, RESULT_ABORTED, RESULT_FAILED})

#: 해석된 목표의 종류. 어느 필드를 봐야 하는지를 가른다.
TARGET_KIND_JOINTS = 'joints'
TARGET_KIND_POSE = 'pose'


def duration_msg(seconds: float) -> Duration:
    """초를 ``builtin_interfaces/Duration`` 으로 바꾼다. 음수는 0 으로 막는다.

    음수를 막는 것은 **중단 시각이 궤적의 0초보다 앞설 수 없기** 때문이다. goal 이
    거부되면 호출자가 0 을 주지만, 시계 지터로 아주 작은 음수가 나올 수 있다.
    """
    total_ns = max(0, int(round(float(seconds) * 1_000_000_000)))
    out = Duration()
    out.sec = int(total_ns // 1_000_000_000)
    out.nanosec = int(total_ns % 1_000_000_000)
    return out


def clamp_truncated_at(elapsed_sec: float, trajectory_duration_sec: float) -> float:
    """중단 시각을 궤적 길이 안으로 자른다.

    **JTC 에서 필요하다** — 컨트롤러가 goal tolerance 를 확인하느라 궤적 끝보다 늦게
    결과를 주므로 "끝난 시각 − t₀" 가 궤적 길이를 넘을 수 있다. 안 자르면
    ``L2(truncated_at)`` 가 궤적 밖을 가리켜 소비자가 외삽하게 된다.
    """
    return max(0.0, min(float(elapsed_sec), float(trajectory_duration_sec)))


@dataclass
class OperationTarget:
    """연산이 **무엇을 겨냥했는가** — 문맥에서 식별자·시각·궤적 길이를 뺀 나머지다.

    R1 을 발행하는 자리가 t₀ 와 ``operation_id`` 를 만드는데, 그 자리는 **무엇을
    겨냥했는지를 모른다.** 펑션베이가 그렇다 — `TrajectoryStreamer.stream` 은 궤적만
    받을 뿐 그것이 named target 이었는지 데카르트였는지 모른다. 그래서 호출자가 이
    객체로 넘긴다.

    Attributes:
        operation: 이동 방식 (:data:`OPERATIONS`).
        joint_names / joint_values: 해석된 관절 목표.
        pose: 데카르트 목표. **tip link 의 자세**다.
        frame: 그 tip link 이름.
    """

    operation: str
    joint_names: tuple = ()
    joint_values: tuple = ()
    pose: Optional[Pose] = None
    frame: str = ''


@dataclass
class MotionContext:
    """연산 하나를 R1 에서 R2 까지 들고 가는 문맥.

    **R1 을 발행하는 순간 만들어 결과 콜백까지 들고 간다.** 그러지 않으면 R2 가
    ``operation_id`` 도 t₀ 도 알 수 없다 — R1 의 ``publish()`` 는 식별자만 돌려주고
    ``stamp=None`` 이면 발행자 안에서 시각을 찍기 때문이다(설계 §2 의 3).

    Attributes:
        operation_id: 짝 ``MotionTrajectory`` 와 같은 값.
        stamp: **t₀** — 궤적의 0초가 시작된 시각. 호출자가 만들어 R1 에 넘긴 바로
            그 값이다(설계 §3.3 의 (a)).
        operation: 이동 방식 (:data:`OPERATIONS`).
        duration_sec: 궤적 길이. 중단 시각을 이 안으로 자른다.
        target_*: 해석된 목표. ``failed`` 여도 채운다.
    """

    operation_id: str
    stamp: object
    operation: str
    duration_sec: float = 0.0
    target_joint_names: tuple = ()
    target_joint_values: tuple = ()
    target_pose: Optional[Pose] = None
    target_frame: str = ''

    @classmethod
    def from_target(cls, target: Optional[OperationTarget], *, operation_id: str,
                    stamp, duration_sec: float = 0.0,
                    default_operation: str = OPERATION_CARTESIAN) -> 'MotionContext':
        """R1 을 발행한 자리에서 문맥을 만든다.

        ``target`` 이 없으면 무엇을 겨냥했는지 모르는 호출이다 — 목표 필드를 비우고
        ``default_operation`` 으로 둔다. 비워 두는 편이 **틀린 값을 적는 것보다 낫다.**
        """
        if target is None:
            return cls(operation_id=operation_id, stamp=stamp,
                       operation=default_operation, duration_sec=duration_sec)
        return cls(operation_id=operation_id, stamp=stamp, operation=target.operation,
                   duration_sec=duration_sec,
                   target_joint_names=tuple(target.joint_names),
                   target_joint_values=tuple(target.joint_values),
                   target_pose=target.pose, target_frame=target.frame)


def _make_qos(depth: int = DEFAULT_QOS_DEPTH) -> QoSProfile:
    return QoSProfile(depth=depth,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST)


class MotionEventPublisher:
    """`motion/event` 발행자. 생산자(클라이언트 · 스트리머)가 하나씩 갖는다."""

    def __init__(self, node: Node, *, topic: str = MOTION_EVENT_TOPIC,
                 qos_depth: int = DEFAULT_QOS_DEPTH) -> None:
        self._node = node
        self._topic = topic
        self._publisher = node.create_publisher(MotionEvent, topic, _make_qos(qos_depth))

    def publish(self, *, operation_id: str, operation: str, result: str,
                stamp=None, reason: str = '', truncated_at_sec: float = 0.0,
                target_joint_names: Optional[Sequence[str]] = None,
                target_joint_values: Optional[Sequence[float]] = None,
                target_pose: Optional[Pose] = None,
                target_frame: str = '') -> bool:
        """연산 이벤트를 한 건 발행한다.

        **예외를 밖으로 내보내지 않는다.** 이 발행은 기록용이므로 실패해도 팔 동작은
        이어져야 한다. 실패하면 경고를 남기고 ``False`` 를 돌려주며, 그 연산은 적재
        검증에서 짝 없는 궤적으로 드러난다.

        ``target_kind`` 는 **인자로 받지 않고 여기서 정한다.** 호출자가 종류와 값을
        따로 주면 어긋날 수 있고, 그 어긋남은 "어느 필드를 봐야 하는지 모른다" 로
        나타난다. ``target_pose`` 가 있으면 ``'pose'``, 관절값이 있으면
        ``'joints'``, 둘 다 없으면 빈 문자열이다.

        Args:
            operation_id: 짝 ``MotionTrajectory`` 와 **같은 값**. ``failed`` 라 짝이
                없으면 호출자가 새로 만들어 준다.
            operation: :data:`OPERATIONS` 중 하나 (이동 방식).
            result: :data:`RESULTS` 중 하나.
            stamp: **연산이 끝난 시각**. ``None`` 이면 노드 시계의 현재 시각을 쓴다.
                레이블의 구간 끝이 아니다 — 그것은 ``t₀ + truncated_at`` 이다.
            reason: 왜 그렇게 끝났는지. 사람이 읽는 값이고 레이블 계산에는 안 쓴다.
            truncated_at_sec: **궤적 기준** 중단 시각(초). 정상 종료면 궤적 길이,
                ``failed`` 나 goal 거부면 0. JTC 는 :func:`clamp_truncated_at` 을
                거쳐서 준다.
            target_joint_names: 해석된 관절 목표의 이름.
            target_joint_values: 같은 순서의 목표값(라디안).
            target_pose: 데카르트 목표. **tip link 기준이다.**
            target_frame: 그 tip link 이름 (예: ``panda_link8``).

        Returns:
            발행했으면 ``True``.
        """
        try:
            if operation not in OPERATIONS:
                raise ValueError(f'operation must be one of {sorted(OPERATIONS)}, '
                                 f'got {operation!r}')
            if result not in RESULTS:
                raise ValueError(f'result must be one of {sorted(RESULTS)}, got {result!r}')

            msg = MotionEvent()
            msg.header.stamp = stamp if stamp is not None else self._node.get_clock().now().to_msg()
            msg.header.frame_id = ''
            msg.operation_id = operation_id
            msg.operation = operation
            msg.result = result
            msg.reason = reason
            msg.truncated_at = duration_msg(truncated_at_sec)

            if target_pose is not None:
                msg.target_kind = TARGET_KIND_POSE
                msg.target_pose = target_pose
                msg.target_frame = target_frame
            elif target_joint_values:
                msg.target_kind = TARGET_KIND_JOINTS
                msg.target_joint_names = list(target_joint_names or [])
                msg.target_joint_values = [float(v) for v in target_joint_values]
            else:
                msg.target_kind = ''

            self._publisher.publish(msg)
            return True
        except Exception as exc:                       # noqa: BLE001 - 기록이 동작을 막지 않는다
            self._node.get_logger().warning(
                f'Failed to publish motion event on {self._topic}: '
                f'{type(exc).__name__}: {exc}')
            return False

    def publish_context(self, ctx: MotionContext, *, result: str, reason: str = '',
                        truncated_at_sec: Optional[float] = None, stamp=None) -> bool:
        """:class:`MotionContext` 로 이벤트를 낸다 — 발행 지점 셋이 공유하는 경로다.

        ``truncated_at`` 을 결과에 맞게 정한다. 호출자마다 이 규칙을 되풀이하면
        한 곳만 틀려도 **에러 없이 레이블만 어긋난다.**

        * ``None`` + ``succeeded`` → 궤적 길이 (끝까지 갔다)
        * ``None`` + ``aborted`` / ``failed`` → 0 (한 점도 못 갔다 — goal 거부 등)
        * 값을 주면 궤적 길이 안으로 **자른다** (:func:`clamp_truncated_at`)

        Args:
            ctx: R1 발행 때 만든 문맥.
            result: :data:`RESULTS` 중 하나.
            reason: 사람이 읽는 사유.
            truncated_at_sec: 궤적 기준 중단 시각(초). 위 규칙 참조.
            stamp: 연산이 끝난 시각. ``None`` 이면 노드 시계의 현재 시각이다.
        """
        if truncated_at_sec is None:
            resolved = ctx.duration_sec if result == RESULT_SUCCEEDED else 0.0
        else:
            resolved = clamp_truncated_at(truncated_at_sec, ctx.duration_sec)
        return self.publish(
            operation_id=ctx.operation_id, operation=ctx.operation, result=result,
            stamp=stamp, reason=reason, truncated_at_sec=resolved,
            target_joint_names=ctx.target_joint_names,
            target_joint_values=ctx.target_joint_values,
            target_pose=ctx.target_pose, target_frame=ctx.target_frame)


__all__ = ['MOTION_EVENT_TOPIC', 'MotionContext', 'MotionEventPublisher',
           'OperationTarget', 'OPERATIONS',
           'OPERATION_CARTESIAN', 'OPERATION_JOINTS', 'OPERATION_NAMED_TARGET',
           'RESULTS', 'RESULT_ABORTED', 'RESULT_FAILED', 'RESULT_SUCCEEDED',
           'TARGET_KIND_JOINTS', 'TARGET_KIND_POSE', 'clamp_truncated_at', 'duration_msg']
