from geometry_msgs.msg import Pose, PoseStamped
import tf_transformations


def pose(x, y, z, roll, pitch, yaw) -> Pose:
    """RPY를 쿼터니언으로 변환하여 Pose 메시지 생성"""
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose


def pose_stamped(pose: Pose, frame_id: str) -> PoseStamped:
    stamped = PoseStamped()
    stamped.header.frame_id = frame_id
    stamped.pose = pose
    return stamped