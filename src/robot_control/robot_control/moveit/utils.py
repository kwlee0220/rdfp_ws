import math

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


def downward_quaternion(yaw: float = 0.0):
    """**접근축이 정확히 연직 하향**인 자세의 쿼터니언 `(x, y, z, w)`.

    ``yaw`` 는 그 축 둘레의 회전이며, 2지 그리퍼에서는 **개폐축의 방향**이 된다.

    ⚠️ **왜 필요한가 — 현재 자세를 읽어 목표로 주면 오차가 누적된다.**
    처짐은 위치만이 아니라 자세도 틀어놓는다. 그런데 매번 *틀어진 현재 자세*를 읽어
    다음 목표로 주면 그 오차가 지워지지 않고 쌓인다. 2026-09-09 실측: 이동 16회 만에
    접근축이 연직에서 **22.3° 기울었다**(이동당 약 1.4°). 위치는 절대 목표로 주면서
    자세만 상대로 이어받은 탓이다 — 체크리스트가 위치에 대해 경고하는
    *"'현재 + 델타'로 쌓으면 밀린다"* 와 같은 실수다.

    **자세가 틀어지면 관절 구성이 달라지고, 처짐의 크기와 방향도 함께 달라진다.**
    그래서 반복 이동이나 캘리브레이션에서는 **반드시 절대 자세**를 쓴다.

    Args:
        yaw: 연직축 둘레 회전(rad). 0 이면 개폐축이 base 의 +x 를 향한다.

    Returns:
        `(x, y, z, w)` 튜플. `geometry_msgs/Quaternion` 에 그대로 대입한다.
    """
    # roll=π 로 z 축을 뒤집어 아래를 향하게 하고, 그 뒤 yaw 를 준다.
    return tuple(tf_transformations.quaternion_from_euler(math.pi, 0.0, yaw))


def downward_pose(x: float, y: float, z: float, yaw: float = 0.0) -> Pose:
    """위치와 yaw 만으로 **절대** 하향 자세 Pose 를 만든다.

    :func:`downward_quaternion` 의 이유를 그대로 따른다 — 현재 자세를 읽지 않는다.
    """
    out = Pose()
    out.position.x, out.position.y, out.position.z = float(x), float(y), float(z)
    (out.orientation.x, out.orientation.y,
     out.orientation.z, out.orientation.w) = downward_quaternion(yaw)
    return out


def pose_stamped(pose: Pose, frame_id: str) -> PoseStamped:
    stamped = PoseStamped()
    stamped.header.frame_id = frame_id
    stamped.pose = pose
    return stamped


def _quaternion_multiply(a, b) -> tuple:
    """`a ⊗ b` (xyzw 순서)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _quaternion_inverse(q) -> tuple:
    """단위 쿼터니언의 켤레 (xyzw)."""
    x, y, z, w = q
    return (-x, -y, -z, w)


def _rotate(q, v) -> tuple:
    """벡터 `v` 를 쿼터니언 `q` 로 돌린다."""
    x, y, z, w = q
    vx, vy, vz = v
    return (
        (1 - 2 * (y * y + z * z)) * vx + 2 * (x * y - z * w) * vy + 2 * (x * z + y * w) * vz,
        2 * (x * y + z * w) * vx + (1 - 2 * (x * x + z * z)) * vy + 2 * (y * z - x * w) * vz,
        2 * (x * z - y * w) * vx + 2 * (y * z + x * w) * vy + (1 - 2 * (x * x + y * y)) * vz)


def tip_pose_for_ee(ee_position, ee_orientation, tip_to_ee_translation,
                    tip_to_ee_rotation) -> tuple:
    """**EE 프레임 목표를 tip link 지령으로 옮긴다.** `(위치, 자세)` 를 돌려준다.

    데카르트 목표는 MoveIt 그룹의 **tip link** 기준으로 해석되는데, 사람이 다루는
    프레임은 보통 그것이 아니다 — 펑션베이는 `/ee_pose` 가 `grasp_center` 를 가리키고
    tip(`panda_link8`) 과 **149 mm** 떨어져 있다. 그 값을 그대로 목표로 넣으면 팔이
    그만큼 엉뚱한 곳으로 간다.

    Args:
        ee_position: EE 프레임이 서기를 바라는 위치 `(x, y, z)`.
        ee_orientation: 그 자세 `(x, y, z, w)`.
        tip_to_ee_translation: tip → EE 의 평행이동 `(x, y, z)`. **TF 에서 읽는다** —
            상수로 박으면 description 이 바뀔 때 조용히 어긋난다.
        tip_to_ee_rotation: tip → EE 의 회전 `(x, y, z, w)`.

    수식은 `T_base→tip = T_base→ee · (T_tip→ee)⁻¹` 이며, 위치는
    `p_tip = p_ee − R_tip · t_offset` 으로 정리된다.
    """
    inverse = _quaternion_inverse(tip_to_ee_rotation)
    tip_rotation = _quaternion_multiply(tuple(ee_orientation), inverse)
    shifted = _rotate(tip_rotation, tuple(tip_to_ee_translation))
    tip_position = tuple(p - s for p, s in zip(ee_position, shifted))
    return tip_position, tip_rotation


__all__ = ['downward_pose', 'downward_quaternion', 'pose', 'pose_stamped',
           'tip_pose_for_ee']
