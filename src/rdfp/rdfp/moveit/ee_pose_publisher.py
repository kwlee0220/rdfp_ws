
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from rdfp.ros2_utils import get_parameter, parse_float, parse_str
from tf2_ros import Buffer, TransformListener, TransformException


_DEFAULT_EE_POSE_TOPIC_NAME = 'ee_pose'


class EePosePublisher(Node):
    def __init__(self):
        super().__init__('ee_pose_publisher')

        self.declare_parameter('base_frame', 'panda_link0')
        self.declare_parameter('ee_frame', 'panda_hand')
        self.declare_parameter('publish_rate', 50.0)

        self._base_frame = get_parameter(self, 'base_frame', parse_str)
        self._ee_frame = get_parameter(self, 'ee_frame', parse_str)
        publish_rate = get_parameter(self, 'publish_rate', parse_float)
        if publish_rate is None or publish_rate <= 0.0:
            self.get_logger().warning('Publish rate must be > 0.')
            raise ValueError('Publish rate must be > 0.')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(PoseStamped, _DEFAULT_EE_POSE_TOPIC_NAME, 10)

        period = 1.0 / publish_rate
        self._timer = self.create_timer(period, self._timer_callback)

        # Throttle TF warnings: log at most once every 5 seconds
        self._last_warn_time = self.get_clock().now()
        self._warn_interval_sec = 5.0

        self.get_logger().info(
            f'Publishing EE pose: {self._base_frame} -> {self._ee_frame} '
            f'at {publish_rate} Hz on {_DEFAULT_EE_POSE_TOPIC_NAME}'
        )

    def _timer_callback(self):
        try:
            t = self._tf_buffer.lookup_transform(self._base_frame, self._ee_frame, Time())
        except TransformException:
            now = self.get_clock().now()
            elapsed = (now - self._last_warn_time).nanoseconds * 1e-9
            if elapsed >= self._warn_interval_sec:
                self.get_logger().warning(f'Could not get transform from {self._base_frame} to {self._ee_frame}')
                self._last_warn_time = now
            return

        msg = PoseStamped()
        msg.header.frame_id = self._base_frame
        msg.header.stamp = t.header.stamp

        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.position.z = t.transform.translation.z

        msg.pose.orientation.x = t.transform.rotation.x
        msg.pose.orientation.y = t.transform.rotation.y
        msg.pose.orientation.z = t.transform.rotation.z
        msg.pose.orientation.w = t.transform.rotation.w

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EePosePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
