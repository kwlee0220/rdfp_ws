from __future__ import annotations

from typing import Optional, Tuple

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, TransformException

from robot_control.ros2_utils import get_parameter, parse_float, parse_str


_DEFAULT_TWIST_TOPIC_NAME = 'ee_twist'

_SOURCE_EE_POSE = 'ee_pose'
_SOURCE_JOINT_STATES = 'joint_states'
_VALID_SOURCES = (_SOURCE_EE_POSE, _SOURCE_JOINT_STATES)

# 위치(3) + 쿼터니언(4) 형태의 pose 표현
_Vec3 = Tuple[float, float, float]
_Quat = Tuple[float, float, float, float]


class EeTwistPublisher(Node):
    """연속적으로 들어오는 end-effector pose 를 시간 차분하여 TwistStamped 로 발행한다.

    twist 는 두 인접 pose 표본을 유한 차분(finite difference)하여 계산한다.

    - ``linear``: ``(p_curr - p_prev) / dt``
    - ``angular``: base_frame 기준 상대 회전 ``q_curr * q_prev^-1`` 를 axis-angle 로
      변환한 뒤 ``/ dt`` 한 값

    pose 표본의 출처는 ``source`` 파라미터로 선택한다.

    - ``ee_pose``: ``ee_pose_topic`` (PoseStamped) 을 그대로 구독하여 표본으로 쓴다.
    - ``joint_states``: ``joint_states_topic`` (JointState) 도착을 트리거로 TF 에서
      ``base_frame -> ee_frame`` 변환을 조회한다. 실제 FK 는 robot_state_publisher 가
      수행하며, 이 노드는 그 결과를 TF 에서 읽어 표본으로 쓴다.

    linear/angular velocity 는 모두 ``base_frame`` 좌표계로 표현한다.

    출력 ``header.stamp`` 는 입력 표본의 stamp 가 아니라 **발행 시각** 이다.
    ``moveit_servo`` 가 ``now - header.stamp >= incoming_command_timeout``
    (기본 0.1초) 인 명령을 stale 로 보고 경고 없이 폐기하기 때문이다. 과거에
    녹화된 pose 를 rosbag 등으로 재생할 때 표본 stamp 를 그대로 실으면 servo 가
    전 구간을 무시해 로봇이 움직이지 않는다. 차분 간격 ``dt`` 는 입력 stamp 로
    계산하므로 속도 값 자체는 원본 시간축을 그대로 반영한다.
    """

    def __init__(self):
        super().__init__('ee_twist_publisher')

        self.declare_parameter('source', _SOURCE_JOINT_STATES)
        self.declare_parameter('base_frame', 'panda_link0')
        self.declare_parameter('ee_frame', 'panda_hand')
        self.declare_parameter('ee_pose_topic', 'ee_pose')
        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('twist_topic', _DEFAULT_TWIST_TOPIC_NAME)
        self.declare_parameter('max_dt', 1.0)
        # 출력 스케일링 게인. MoveIt Servo 의 command_in_type=unitless 입력
        # (joystick 식 [-1,1] × scale) 으로 변환할 때 1/scale.linear,
        # 1/scale.rotational 를 지정한다. 기본 1.0 은 물리 단위(m/s, rad/s) 그대로.
        self.declare_parameter('linear_gain', 1.0)
        self.declare_parameter('angular_gain', 1.0)

        self._source = get_parameter(self, 'source', parse_str)
        if self._source not in _VALID_SOURCES:
            self.get_logger().error(f"Invalid source: {self._source!r}")
            raise ValueError(f"'source' must be one of {_VALID_SOURCES}, got {self._source!r}")

        self._base_frame = get_parameter(self, 'base_frame', parse_str)
        self._ee_frame = get_parameter(self, 'ee_frame', parse_str)
        self._max_dt = get_parameter(self, 'max_dt', parse_float)
        if self._max_dt <= 0.0:
            self.get_logger().error('max_dt must be > 0.')
            raise ValueError('max_dt must be > 0.')

        self._linear_gain = get_parameter(self, 'linear_gain', parse_float)
        self._angular_gain = get_parameter(self, 'angular_gain', parse_float)
        if self._linear_gain <= 0.0 or self._angular_gain <= 0.0:
            self.get_logger().error('linear_gain/angular_gain must be > 0.')
            raise ValueError('linear_gain/angular_gain must be > 0.')

        twist_topic = get_parameter(self, 'twist_topic', parse_str)
        self._pub = self.create_publisher(TwistStamped, twist_topic, 10)

        # 직전 pose 표본 (stamp[sec], position, quaternion)
        self._prev: Optional[Tuple[float, _Vec3, _Quat]] = None

        # TF 경고 스로틀링: 최대 5초에 한 번만 기록한다.
        self._last_warn_time = self.get_clock().now()
        self._warn_interval_sec = 5.0

        if self._source == _SOURCE_JOINT_STATES:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            joint_states_topic = get_parameter(self, 'joint_states_topic', parse_str)
            self._sub = self.create_subscription(
                JointState, joint_states_topic, self._on_joint_states, 10)
            source_desc = f'{joint_states_topic} (FK via TF {self._base_frame} -> {self._ee_frame})'
        else:
            ee_pose_topic = get_parameter(self, 'ee_pose_topic', parse_str)
            self._sub = self.create_subscription(
                PoseStamped, ee_pose_topic, self._on_ee_pose, 10)
            source_desc = ee_pose_topic

        self.get_logger().info(
            f'Publishing EE twist from {source_desc} on {twist_topic} '
            f'(frame_id={self._base_frame})'
        )

    def _on_ee_pose(self, msg: PoseStamped) -> None:
        """PoseStamped 표본을 차분하여 twist 를 발행한다."""
        p = msg.pose.position
        o = msg.pose.orientation
        self._process(msg.header.stamp, (p.x, p.y, p.z), (o.x, o.y, o.z, o.w))

    def _on_joint_states(self, msg: JointState) -> None:
        """joint_states 도착을 트리거로 TF 에서 ee pose 를 조회해 twist 를 발행한다."""
        try:
            t = self._tf_buffer.lookup_transform(self._base_frame, self._ee_frame, Time())
        except TransformException:
            self._warn_throttled(
                f'Could not get transform from {self._base_frame} to {self._ee_frame}')
            return

        tr = t.transform.translation
        rot = t.transform.rotation
        self._process(t.header.stamp, (tr.x, tr.y, tr.z), (rot.x, rot.y, rot.z, rot.w))

    def _process(self, stamp: TimeMsg, pos: _Vec3, quat: _Quat) -> None:
        """직전 표본과 현재 표본을 유한 차분하여 twist 를 발행한다."""
        t = self._stamp_to_sec(stamp)
        prev = self._prev
        # 다음 호출을 위해 현재 표본을 먼저 저장한다(공백 발생 시에도 최신 표본 유지).
        self._prev = (t, pos, quat)
        if prev is None:
            return

        dt = t - prev[0]
        if dt <= 0.0:
            # 중복 또는 역행 타임스탬프는 무시한다.
            return
        if dt > self._max_dt:
            # 큰 시간 공백 후에는 속도 스파이크를 피하기 위해 한 주기를 건너뛴다.
            self._warn_throttled(
                f'Sample gap {dt:.3f}s exceeds max_dt {self._max_dt:.3f}s; skipping')
            return

        prev_pos = prev[1]
        prev_quat = prev[2]
        kl = self._linear_gain
        ka = self._angular_gain
        linear = (kl * (pos[0] - prev_pos[0]) / dt,
                  kl * (pos[1] - prev_pos[1]) / dt,
                  kl * (pos[2] - prev_pos[2]) / dt)
        wx, wy, wz = self._angular_velocity(prev_quat, quat, dt)
        angular = (ka * wx, ka * wy, ka * wz)

        msg = TwistStamped()
        msg.header.frame_id = self._base_frame
        # 입력 표본의 stamp 가 아니라 **발행 시각** 을 싣는다. twist 는 "지금 이
        # 속도로 움직여라" 는 명령이지 과거 관측이 아니며, moveit_servo 는
        # `now - header.stamp >= incoming_command_timeout` (기본 0.1s) 인 명령을
        # stale 로 보고 **경고 없이 폐기** 한다. 표본 stamp 를 그대로 실으면
        # 과거에 녹화된 pose 를 재생할 때 servo 가 전 구간을 무시한다
        # (예: 3일 전 rosbag 재생 → stamp 가 26만 초 과거 → 로봇이 안 움직임).
        # 차분 간격 dt 는 위에서 입력 stamp 로 계산하므로 속도 값 자체는 원본
        # 시간축을 그대로 반영한다.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = linear
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = angular
        self._pub.publish(msg)

    @staticmethod
    def _stamp_to_sec(stamp: TimeMsg) -> float:
        """builtin_interfaces/Time 을 초 단위 float 으로 변환한다."""
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @classmethod
    def _angular_velocity(cls, q_prev: _Quat, q_curr: _Quat, dt: float) -> _Vec3:
        """base_frame 기준 각속도를 계산한다.

        base 좌표계 상대 회전은 ``q_delta = q_curr * q_prev^-1`` 이며, 이를
        axis-angle 로 변환한 뒤 ``dt`` 로 나눠 각속도(rad/s)를 얻는다.
        """
        qd = cls._quat_multiply(q_curr, cls._quat_conjugate(q_prev))
        x, y, z, w = qd
        # 최단 경로를 위해 w 가 음수면 부호를 뒤집는다(같은 회전을 나타낸다).
        if w < 0.0:
            x, y, z, w = -x, -y, -z, -w
        vnorm = math.sqrt(x * x + y * y + z * z)
        if vnorm < 1e-9:
            # 회전이 거의 없으면 각속도는 0 에 수렴한다.
            return (0.0, 0.0, 0.0)
        angle = 2.0 * math.atan2(vnorm, w)
        scale = angle / (vnorm * dt)
        return (x * scale, y * scale, z * scale)

    @staticmethod
    def _quat_conjugate(q: _Quat) -> _Quat:
        """단위 쿼터니언의 켤레(=역원)를 반환한다."""
        x, y, z, w = q
        return (-x, -y, -z, w)

    @staticmethod
    def _quat_multiply(q1: _Quat, q2: _Quat) -> _Quat:
        """Hamilton 곱 ``q1 * q2`` 를 계산한다."""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        )

    def _warn_throttled(self, message: str) -> None:
        """경고 로그를 최소 간격으로만 기록한다."""
        now = self.get_clock().now()
        elapsed = (now - self._last_warn_time).nanoseconds * 1e-9
        if elapsed >= self._warn_interval_sec:
            self.get_logger().warning(message)
            self._last_warn_time = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EeTwistPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
