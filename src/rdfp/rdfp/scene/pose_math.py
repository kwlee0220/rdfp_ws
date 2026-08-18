"""pose 합성용 최소 쿼터니언 연산.

`scene_state_node` 가 planning scene 의 pose 를 로봇 베이스 프레임으로 옮길 때 쓴다.
합성이 두 번 필요하기 때문이다.

    base ← 오브젝트 프레임 (TF)  ∘  오브젝트 ← primitive (CollisionObject)

`tf2_geometry_msgs.do_transform_pose` 를 쓰지 않는 이유는 두 가지다. 배포판마다
시그니처가 달라(Pose vs PoseStamped) 조용히 어긋날 수 있고, 여기 있는 함수들은
**ROS 없이 테스트할 수 있어야** 하기 때문이다 — 쿼터니언 순서 오류는 크래시가 아니라
'그럴듯하게 틀린 자세'로 나타나므로 단위 테스트로 고정해 두는 값이 크다.

모든 쿼터니언은 **ROS 규약 (x, y, z, w)** 이다.
"""

from __future__ import annotations

Quat = tuple[float, float, float, float]
Vec3 = tuple[float, float, float]

# 회전 없음. ROS 규약이라 w 가 마지막이다 (Isaac 의 wxyz 와 반대).
IDENTITY_QUAT: Quat = (0.0, 0.0, 0.0, 1.0)


def quat_multiply(lhs: Quat, rhs: Quat) -> Quat:
    """두 쿼터니언의 Hamilton 곱 ``lhs ⊗ rhs``.

    ``lhs`` 를 나중에 적용되는 회전(부모)으로 두는 순서다. 즉 부모 프레임의 회전이
    왼쪽, 자식의 상대 회전이 오른쪽이다.
    """
    x1, y1, z1, w1 = lhs
    x2, y2, z2, w2 = rhs
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quat_rotate(quat: Quat, vec: Vec3) -> Vec3:
    """벡터를 쿼터니언으로 회전한다 (``q ⊗ v ⊗ q⁻¹``).

    단위 쿼터니언을 전제하므로 켤레를 역원으로 쓴다.
    """
    x, y, z, w = quat
    vx, vy, vz = vec

    # t = 2 * (q_vec × v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)

    # v' = v + w * t + (q_vec × t)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def compose_pose(parent_pos: Vec3, parent_quat: Quat,
                 child_pos: Vec3, child_quat: Quat,) -> tuple[Vec3, Quat]:
    """부모 프레임 기준 자식 pose 를 부모의 부모 기준으로 옮긴다.

    Args:
        parent_pos: 부모 프레임의 원점 위치.
        parent_quat: 부모 프레임의 자세.
        child_pos: **부모 프레임 기준** 자식의 위치.
        child_quat: 부모 프레임 기준 자식의 자세.

    Returns:
        ``(위치, 자세)`` 튜플.
    """
    rotated = quat_rotate(parent_quat, child_pos)
    position = (parent_pos[0] + rotated[0],
                parent_pos[1] + rotated[1],
                parent_pos[2] + rotated[2])
    return position, quat_multiply(parent_quat, child_quat)


__all__ = ['IDENTITY_QUAT', 'Quat', 'Vec3', 'compose_pose', 'quat_multiply', 'quat_rotate']
