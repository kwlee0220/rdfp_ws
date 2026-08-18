"""Leader EE pose 를 follower 작업공간으로 재매핑하는 teleop retargeting 노드.

leader 와 follower 의 관절 체계가 달라도 task-space(EE pose) 수준에서
동작을 미러링할 수 있도록, 클러치(clutch) 앵커 기반 상대 매핑을 수행한다.
설계 문서: docs/teleop/leader_follower_mirroring_design.md

- 입력: leader EE pose (PoseStamped, leader base frame 기준)
- 입력: follower EE pose (PoseStamped, follower base frame 기준; engage 앵커용)
- 출력: follower 목표 pose (PoseStamped, follower base frame 기준)
- 서비스: ``~/clutch`` (std_srvs/SetBool) — true 로 engage, false 로 disengage
- 출력: ``~/clutch_state`` (rdfp_msgs/ClutchState) — 상태가 **바뀔 때만** 발행한다.
  QoS 는 ``TRANSIENT_LOCAL`` 이라 늦게 구독해도 현재 상태를 즉시 받는다. 자동
  해제(watchdog / jump)를 폴링 없이 감지할 수 있다.
- 서비스: ``~/get_clutch_state`` (std_srvs/Trigger) — 동기 **조회**. ``success`` 가
  engaged 여부, ``message`` 는 engaged 면 ``'engaged'`` 아니면
  ``'disengaged: <마지막 사유>'``. ``~/clutch`` 는 SetBool 이라 호출 자체가 상태를
  바꾸므로 조회에 쓸 수 없다.

- 입력: ``~/pedal_heartbeat`` (std_msgs/Empty) — hold-to-engage 풋페달용.
  ``pedal_timeout`` 이 0 보다 클 때만 구독하며, engaged 상태에서 하트비트가
  끊기면 자동 disengage 한다 (**데드맨**). 페달 노드가 죽거나 USB 가 빠져
  아무도 disengage 를 못 보내는 상황을 잡기 위한 것이다. 기본값 0 이면
  비활성이므로 콘솔·GUI 로만 조작하는 구성에는 영향이 없다.

클러치가 engaged 상태일 때만 매핑을 수행하며, disengaged 상태에서는 마지막
목표 pose 를 새 stamp 로 재발행(hold)하여 하류의 twist 변환기가 zero-twist 를
만들도록 한다. leader 스트림이 끊기면 watchdog 이 자동으로 disengage 한다.
"""

from __future__ import annotations

from typing import Optional, Tuple

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped
from rdfp_msgs.msg import ClutchState
from std_msgs.msg import Empty
from std_srvs.srv import SetBool, Trigger

from rdfp.ros2_utils import get_parameter, parse_float, parse_str
from rdfp.teleop.retarget_math import (
    Quat, Vec3, clamp_to_box, compute_target, lpf_alpha, quat_from_rpy, quat_slerp
)

# 클러치 상태 토픽. 상태 변경 시에만 발행하므로 depth 는 1 이면 충분하고,
# TRANSIENT_LOCAL 이라 늦게 구독한 노드도 현재 상태를 즉시 받는다.
_CLUTCH_STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)

_DEFAULT_LEADER_POSE_TOPIC = 'leader/ee_pose'
_DEFAULT_FOLLOWER_POSE_TOPIC = 'ee_pose'
_DEFAULT_TARGET_POSE_TOPIC = 'follower/target_pose'

# (수신 시각[sec], 위치, 쿼터니언, 메시지 stamp) 형태의 pose 표본
_Sample = Tuple[float, Vec3, Quat, TimeMsg]


class TeleopRetargetNode(Node):
    """클러치 앵커 기반 상대 매핑으로 leader 동작을 follower 목표 pose 로 변환한다."""

    def __init__(self):
        super().__init__('teleop_retarget')

        self.declare_parameter('leader_pose_topic', _DEFAULT_LEADER_POSE_TOPIC)
        self.declare_parameter('follower_pose_topic', _DEFAULT_FOLLOWER_POSE_TOPIC)
        self.declare_parameter('target_pose_topic', _DEFAULT_TARGET_POSE_TOPIC)
        self.declare_parameter('follower_base_frame', 'panda_link0')
        self.declare_parameter('position_scale', 1.0)
        self.declare_parameter('align_roll', 0.0)
        self.declare_parameter('align_pitch', 0.0)
        self.declare_parameter('align_yaw', 0.0)
        self.declare_parameter('lpf_cutoff_hz', 3.0)
        self.declare_parameter('watchdog_timeout', 0.5)
        self.declare_parameter('max_sample_jump', 0.2)
        self.declare_parameter('pose_staleness', 1.0)
        # 페달 하트비트 감시. hold-to-engage 풋페달을 데드맨으로 쓰려면 켠다.
        # 0 이하면 비활성 — 콘솔/GUI 로만 조작하는 기존 구성에는 영향이 없다.
        self.declare_parameter('pedal_timeout', 0.0)
        # 축정렬 작업공간 박스 [x, y, z]. 둘 다 길이 3 으로 설정해야 clamp 가
        # 활성화된다. 빈 리스트 default 는 BYTE_ARRAY 로 잘못 추론되므로
        # (target_joint_states_executor 참고) 타입만 선언한다.
        self.declare_parameter('workspace_min', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('workspace_max', Parameter.Type.DOUBLE_ARRAY)

        self._follower_base_frame = get_parameter(self, 'follower_base_frame', parse_str)
        self._position_scale = get_parameter(self, 'position_scale', parse_float)
        self._lpf_cutoff_hz = get_parameter(self, 'lpf_cutoff_hz', parse_float)
        self._watchdog_timeout = get_parameter(self, 'watchdog_timeout', parse_float)
        self._max_sample_jump = get_parameter(self, 'max_sample_jump', parse_float)
        self._pose_staleness = get_parameter(self, 'pose_staleness', parse_float)
        self._pedal_timeout = get_parameter(self, 'pedal_timeout', parse_float)
        if self._watchdog_timeout <= 0.0:
            self.get_logger().error('watchdog_timeout must be > 0.')
            raise ValueError('watchdog_timeout must be > 0.')
        if self._pose_staleness <= 0.0:
            self.get_logger().error('pose_staleness must be > 0.')
            raise ValueError('pose_staleness must be > 0.')
        if self._position_scale <= 0.0:
            self.get_logger().error('position_scale must be > 0.')
            raise ValueError('position_scale must be > 0.')

        align_roll = get_parameter(self, 'align_roll', parse_float)
        align_pitch = get_parameter(self, 'align_pitch', parse_float)
        align_yaw = get_parameter(self, 'align_yaw', parse_float)
        self._align_quat = quat_from_rpy(align_roll, align_pitch, align_yaw)
        self._align_rpy = (align_roll, align_pitch, align_yaw)

        self._workspace_box = self._load_workspace_box()

        leader_pose_topic = get_parameter(self, 'leader_pose_topic', parse_str)
        follower_pose_topic = get_parameter(self, 'follower_pose_topic', parse_str)
        target_pose_topic = get_parameter(self, 'target_pose_topic', parse_str)

        self._pub = self.create_publisher(PoseStamped, target_pose_topic, 10)
        self._leader_sub = self.create_subscription(
            PoseStamped, leader_pose_topic, self._on_leader_pose, 10)
        self._follower_sub = self.create_subscription(
            PoseStamped, follower_pose_topic, self._on_follower_pose, 10)
        self._clutch_srv = self.create_service(SetBool, '~/clutch', self._on_clutch)
        # 상태 변경을 밀어주는 토픽(TRANSIENT_LOCAL). 자동 해제(watchdog/jump)를
        # 폴링 없이 즉시 감지할 수 있고, 늦게 구독해도 현재 상태를 바로 받는다.
        self._state_pub = self.create_publisher(
            ClutchState, '~/clutch_state', _CLUTCH_STATE_QOS)
        # 조회 전용(읽기) 서비스. `~/clutch` 는 SetBool 이라 호출 자체가 상태를
        # 바꾸므로 조회에 쓸 수 없다.
        self._clutch_state_srv = self.create_service(
            Trigger, '~/get_clutch_state', self._on_clutch_state)
        # 페달 하트비트. hold-to-engage 페달 노드가 밟고 있는 동안 주기적으로
        # 발행하며, 끊기면 engaged 상태를 자동 해제한다 (데드맨). pedal_timeout
        # 이 0 이하면 구독하지 않는다.
        self._pedal_sub = None
        self._last_pedal_sec: Optional[float] = None
        if self._pedal_timeout > 0.0:
            self._pedal_sub = self.create_subscription(
                Empty, '~/pedal_heartbeat', self._on_pedal_heartbeat, 10)

        # 클러치/매핑 상태
        self._engaged = False
        # 마지막 disengage 사유. 조회 서비스가 로그를 안 봐도 알 수 있게 돌려준다.
        self._last_disengage_reason = 'not engaged since startup'
        self._leader_latest: Optional[_Sample] = None
        self._follower_latest: Optional[_Sample] = None
        self._anchor_leader: Optional[Tuple[Vec3, Quat]] = None
        self._anchor_follower: Optional[Tuple[Vec3, Quat]] = None
        # 필터 상태 겸 마지막 발행 목표 (hold 재발행에 사용)
        self._filt: Optional[Tuple[Vec3, Quat]] = None
        self._prev_leader_stamp_sec: Optional[float] = None
        self._prev_leader_pos: Optional[Vec3] = None

        # watchdog: leader 스트림이 끊기면 자동 disengage 한다.
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_callback)

        # 초기 상태(disengaged)를 한 번 발행한다. TRANSIENT_LOCAL 이라 이후 구독자는
        # 상태 변경이 없어도 이 값을 즉시 받는다.
        self._publish_clutch_state()

        self.get_logger().info(
            f'TeleopRetargetNode started: {leader_pose_topic} -> {target_pose_topic} '
            f'(lpf={self._lpf_cutoff_hz}Hz, '
            f'clutch service=~/clutch, state topic=~/clutch_state, '
            f'state service=~/get_clutch_state, '
            f'workspace_clamp={self._workspace_box is not None}, '
            f'pedal_timeout={self._pedal_timeout})'
        )
        self._log_retarget_params()

    def _log_retarget_params(self) -> None:
        """실제로 적용된 retargeting 파라미터를 시작 시 한 번 출력한다.

        YAML 을 고쳐도 colcon build 전에는 share 의 구버전이 읽히므로, 어떤 값으로
        떴는지 로그에서 바로 확인할 수 있게 한다. align 각도는 rad 가 원값이지만
        감으로 읽기 어려우므로 deg 를 함께 적는다.
        """
        roll, pitch, yaw = self._align_rpy
        self.get_logger().info(
            f'Retarget params: position_scale={self._position_scale:g}, '
            f'align_roll={roll:g} rad ({math.degrees(roll):.1f} deg), '
            f'align_pitch={pitch:g} rad ({math.degrees(pitch):.1f} deg), '
            f'align_yaw={yaw:g} rad ({math.degrees(yaw):.1f} deg)'
        )

    def _load_workspace_box(self) -> Optional[Tuple[Vec3, Vec3]]:
        """workspace_min/max 파라미터를 읽어 clamp 박스를 구성한다.

        둘 다 길이 3 이면 활성화하고, 둘 다 미설정이면 비활성화한다.
        한쪽만 설정된 경우는 구성 오류로 본다. 타입만 선언된 파라미터는
        미설정 시 get_parameter 가 예외를 던지므로 get_parameter_or 로 읽는다.
        """
        pmin = self.get_parameter_or('workspace_min', None)
        pmax = self.get_parameter_or('workspace_max', None)
        wmin = list(pmin.get_parameter_value().double_array_value) if pmin is not None else []
        wmax = list(pmax.get_parameter_value().double_array_value) if pmax is not None else []
        if not wmin and not wmax:
            return None
        if len(wmin) != 3 or len(wmax) != 3:
            self.get_logger().error('workspace_min/max must both be 3-element arrays.')
            raise ValueError('workspace_min/max must both be 3-element arrays.')
        if any(lo > hi for lo, hi in zip(wmin, wmax)):
            self.get_logger().error('workspace_min must be <= workspace_max per axis.')
            raise ValueError('workspace_min must be <= workspace_max per axis.')
        return ((wmin[0], wmin[1], wmin[2]), (wmax[0], wmax[1], wmax[2]))

    # ------------------------------------------------------------------
    # 콜백
    # ------------------------------------------------------------------

    def _on_leader_pose(self, msg: PoseStamped) -> None:
        """leader pose 표본을 받아 매핑(engaged) 또는 hold 재발행(disengaged)한다."""
        now_sec = self._now_sec()
        p = msg.pose.position
        o = msg.pose.orientation
        pos: Vec3 = (p.x, p.y, p.z)
        quat: Quat = (o.x, o.y, o.z, o.w)
        self._leader_latest = (now_sec, pos, quat, msg.header.stamp)

        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        prev_stamp_sec = self._prev_leader_stamp_sec
        prev_pos = self._prev_leader_pos
        self._prev_leader_stamp_sec = stamp_sec
        self._prev_leader_pos = pos

        if not self._engaged:
            # disengaged: 마지막 목표를 새 stamp 로 재발행해 하류가 zero-twist 를
            # 만들도록 한다 (follower 는 현재 위치를 능동적으로 유지한다).
            if self._filt is not None:
                self._publish_target(self._filt[0], self._filt[1], msg.header.stamp)
            return

        # 중복/역행 stamp 표본은 무시한다.
        if prev_stamp_sec is None:
            return
        dt = stamp_sec - prev_stamp_sec
        if dt <= 0.0:
            return

        # leader 표본 간 위치 점프 감지 — TF 글리치/추적 이상 시 안전을 위해
        # 자동 disengage 한다 (운용자가 상황 확인 후 재 engage).
        if self._max_sample_jump > 0.0 and prev_pos is not None:
            jump = ((pos[0] - prev_pos[0]) ** 2 + (pos[1] - prev_pos[1]) ** 2
                    + (pos[2] - prev_pos[2]) ** 2) ** 0.5
            if jump > self._max_sample_jump:
                self._disengage(f'leader pose jumped {jump:.3f}m > {self._max_sample_jump:.3f}m')
                return

        raw_pos, raw_quat = compute_target(
            pos, quat,
            self._anchor_leader[0], self._anchor_leader[1],
            self._anchor_follower[0], self._anchor_follower[1],
            self._align_quat, self._position_scale)

        if self._workspace_box is not None:
            raw_pos = clamp_to_box(raw_pos, self._workspace_box[0], self._workspace_box[1])

        # 1차 저역통과 필터 (위치 lerp + 자세 slerp). 필터 상태는 engage 시점에
        # F₀ 로 초기화되므로 시작 순간의 목표 점프가 없다.
        alpha = lpf_alpha(dt, self._lpf_cutoff_hz)
        fpos, fquat = self._filt
        new_pos = (
            fpos[0] + alpha * (raw_pos[0] - fpos[0]),
            fpos[1] + alpha * (raw_pos[1] - fpos[1]),
            fpos[2] + alpha * (raw_pos[2] - fpos[2])
        )
        new_quat = quat_slerp(fquat, raw_quat, alpha)
        self._filt = (new_pos, new_quat)

        self._publish_target(new_pos, new_quat, msg.header.stamp)

    def _on_follower_pose(self, msg: PoseStamped) -> None:
        """follower 현재 pose 를 engage 앵커용으로 보관한다."""
        p = msg.pose.position
        o = msg.pose.orientation
        self._follower_latest = (
            self._now_sec(), (p.x, p.y, p.z), (o.x, o.y, o.z, o.w), msg.header.stamp)

    def _on_clutch(self, request: SetBool.Request,
                   response: SetBool.Response) -> SetBool.Response:
        """클러치 engage/disengage 서비스를 처리한다."""
        if request.data:
            error = self._try_engage()
            response.success = error is None
            response.message = 'engaged' if error is None else error
        else:
            self._disengage('clutch released by operator')
            response.success = True
            response.message = 'disengaged'
        return response

    def _on_clutch_state(self, request: Trigger.Request,
                         response: Trigger.Response) -> Trigger.Response:
        """현재 클러치 상태를 조회한다 (상태를 바꾸지 않는다).

        ``success`` 가 곧 engaged 여부이며, ``message`` 에는 engaged 면 앵커 포착
        여부를, disengaged 면 **마지막 해제 사유** 를 담는다.
        """
        del request
        response.success = self._engaged
        if self._engaged:
            response.message = 'engaged'
        else:
            response.message = f'disengaged: {self._last_disengage_reason}'
        return response

    def _on_pedal_heartbeat(self, msg: Empty) -> None:
        """페달이 눌려 있는 동안 오는 하트비트를 기록한다."""
        del msg
        self._last_pedal_sec = self._now_sec()

    def _watchdog_callback(self) -> None:
        """engaged 상태에서 입력이 끊기면 자동 disengage 한다.

        두 가지를 감시한다.

        - leader pose 스트림 (``watchdog_timeout``)
        - 페달 하트비트 (``pedal_timeout``, 0 이하면 비활성)

        페달 감시는 hold-to-engage 풋페달을 **데드맨** 으로 쓰기 위한 것이다.
        페달 노드가 죽거나 USB 가 빠지면 아무도 disengage 를 보내지 못하는데,
        하트비트가 끊기는 것으로 그 상황을 잡는다.
        """
        if not self._engaged:
            return

        now_sec = self._now_sec()
        if self._pedal_timeout > 0.0:
            if self._last_pedal_sec is None:
                # engage 는 되었는데 하트비트를 한 번도 못 받았다. 페달 없이
                # 서비스로만 잡은 경우이므로 즉시 해제한다.
                self._disengage('pedal heartbeat never received')
                return
            pedal_elapsed = now_sec - self._last_pedal_sec
            if pedal_elapsed > self._pedal_timeout:
                self._disengage(f'pedal heartbeat stale for {pedal_elapsed:.2f}s')
                return

        if self._leader_latest is None:
            return
        elapsed = now_sec - self._leader_latest[0]
        if elapsed > self._watchdog_timeout:
            self._disengage(f'leader pose stream stale for {elapsed:.2f}s')

    # ------------------------------------------------------------------
    # 내부 헬퍼
    # ------------------------------------------------------------------

    def _try_engage(self) -> Optional[str]:
        """양쪽 pose 신선도를 확인하고 앵커를 잡는다. 실패 시 사유를 반환한다."""
        if self._engaged:
            return None
        now_sec = self._now_sec()
        if self._leader_latest is None or now_sec - self._leader_latest[0] > self._pose_staleness:
            msg = 'cannot engage: leader pose is missing or stale'
            self.get_logger().warning(msg)
            return msg
        if (self._follower_latest is None
                or now_sec - self._follower_latest[0] > self._pose_staleness):
            msg = 'cannot engage: follower pose is missing or stale'
            self.get_logger().warning(msg)
            return msg

        self._anchor_leader = (self._leader_latest[1], self._leader_latest[2])
        self._anchor_follower = (self._follower_latest[1], self._follower_latest[2])
        # 필터 상태를 F₀ 로 초기화한다 — 목표가 follower 의 현재 pose 에서
        # 연속적으로 출발하므로 engage 순간의 점프가 없다.
        self._filt = self._anchor_follower
        self._engaged = True
        self.get_logger().info('Clutch engaged: anchors captured.')
        self._publish_clutch_state()
        return None

    def _disengage(self, reason: str) -> None:
        """클러치를 해제한다. 마지막 목표(_filt)는 hold 재발행을 위해 유지한다."""
        if not self._engaged:
            return
        self._engaged = False
        # 토픽·조회 서비스가 함께 쓰도록 사유를 보관한다.
        self._last_disengage_reason = reason
        self.get_logger().warning(f'Clutch disengaged: {reason}')
        self._publish_clutch_state()

    def _publish_clutch_state(self) -> None:
        """현재 클러치 상태를 발행한다. 상태가 바뀔 때만 호출한다."""
        msg = ClutchState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.engaged = self._engaged
        msg.reason = 'engaged' if self._engaged else self._last_disengage_reason
        self._state_pub.publish(msg)

    def _publish_target(self, pos: Vec3, quat: Quat, stamp: TimeMsg) -> None:
        """목표 pose 를 follower base frame 기준 PoseStamped 로 발행한다."""
        msg = PoseStamped()
        msg.header.frame_id = self._follower_base_frame
        msg.header.stamp = stamp
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = quat
        self._pub.publish(msg)

    def _now_sec(self) -> float:
        """노드 클럭의 현재 시각을 초 단위 float 으로 반환한다."""
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TeleopRetargetNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
