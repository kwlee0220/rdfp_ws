"""`/scene/objects` → MoveIt **planning scene** 반영 노드.

시뮬레이터가 알려 준 물체를 MoveIt 이 **장애물로 인식하게** 만든다. 이것이 없으면
MoveIt 은 빈 공간을 가정하고 계획하므로, 팔이 테이블을 뚫는 궤적이 나온다 — 실제로
Isaac 에서 드리프트로 `panda_link4` 와 `panda_link5` 가 상판을 파고든 것을 측정했다.

`mock_scene_state_node` 와 **방향이 반대**다.

    mock 백엔드   : planning scene → /scene/objects   (읽는다)
    이 노드        : /scene/objects → planning scene   (쓴다)

그래서 **mock 백엔드에서는 띄우면 안 된다** — 자기가 읽은 것을 자기가 되쓰는 고리가
된다. 시뮬레이터가 scene 의 원본인 백엔드(Isaac·펑션베이)에서만 쓴다.

`moveit_msgs/PlanningScene` 을 ``is_diff=True`` 로 발행한다. 같은 ``id`` 로 다시
보내면 MoveIt 이 갱신하므로, 물체가 움직여도 추가로 지울 필요가 없다.

무엇을 넣을 것인가 — ``fixture`` 인 것만
---------------------------------------

**조작 대상 물체는 넣지 않는 편이 낫다.** 넣으면 파지 자체가 충돌이 되어, 그것을
피하려고 `attach`/`detach` 로 물체를 손에 붙였다 뗐다 해야 한다. 그 방식에는 물리
시뮬레이터에서 무너지는 전제가 있다 — **물체는 미끄러져 떨어질 수 있는데 `attach` 는
명시적으로 뗄 때까지 유지된다.** 떨어지면 planner 의 믿음이 두 군데에서 동시에 틀린다.

    손       : 물체가 붙어 있다   ← 실제로는 비어 있다
    탁자 위  : 아무것도 없다      ← 실제로는 물체가 떨어져 있다

게다가 파지 동작 자체는 애초에 이 정보를 쓰지 않는다. 접근·하강·상승은 cartesian
경로이고, ``GetCartesianPath`` 는 ``avoid_collisions`` 기본값이 ``False`` 라 **충돌을
검사하지 않는다.** planning scene 을 실제로 쓰는 것은 관절공간 계획
(`move_to_named_target` / `move_to_joints`)뿐이며, 거기서 손가락이 대상 물체에 닿아
있으면 시작 자세 충돌로 `INVALID_MOTION_PLAN`(-2)이 되어 **계획 자체가 거부된다.**

그래서 이 노드는 **`SceneObject.fixture` 가 true 인 것만** 넣는다. 탁자·펜스는 들어가고
블록은 들어가지 않는다.

**분류는 발행 노드가 한다.** 여기에 이름 목록 파라미터를 두면 같은 사실이 두 곳에
적히고, 물체를 하나 더할 때 한쪽만 고쳐져 조용히 어긋난다 — 이름 목록을 쓰던
`collision_objects` 파라미터를 없앤 이유다. Isaac 은 `isaac_scene.json` 의 `dynamic` 을
뒤집어 채우고, mock 은 `/scene/reset` 요청에 실려 온 값을 기억한다.

잃는 것은 관절공간 이동 중 물체를 쓸고 갈 수 있다는 것인데, 시뮬레이터에서는 결과가
`/scene/objects` 에 그대로 기록되므로 데이터에서 보인다.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from moveit_msgs.msg import CollisionObject, PlanningScene
from rdfp_msgs.msg import SceneObject, SceneObjects
from shape_msgs.msg import SolidPrimitive

from robot_control.ros2_utils import get_parameter, parse_stripped_str

_DEFAULT_SCENE_TOPIC = '/scene/objects'
_DEFAULT_PLANNING_SCENE_TOPIC = '/planning_scene'
# `SceneObject.type` (소문자 문자열) → `SolidPrimitive` 상수.
# dimensions 순서는 두 쪽이 같으므로 **재배열하지 않는다** (SceneObject.msg 참고).
_PRIMITIVE_TYPES = {
    'box': SolidPrimitive.BOX,
    'sphere': SolidPrimitive.SPHERE,
    'cylinder': SolidPrimitive.CYLINDER,
    'cone': SolidPrimitive.CONE,
}

_SCENE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class PlanningSceneSyncNode(Node):
    """scene 물체를 planning scene 의 collision object 로 밀어 넣는다."""

    def __init__(self) -> None:
        super().__init__('planning_scene_sync')

        self.declare_parameter('scene_topic', _DEFAULT_SCENE_TOPIC)
        self.declare_parameter('planning_scene_topic', _DEFAULT_PLANNING_SCENE_TOPIC)
        self.declare_parameter('warn_interval_sec', 10.0)

        scene_topic = get_parameter(self, 'scene_topic', parse_stripped_str)
        planning_scene_topic = get_parameter(self, 'planning_scene_topic', parse_stripped_str)
        self._warn_interval = float(self.get_parameter('warn_interval_sec').value)

        self._publisher = self.create_publisher(PlanningScene, planning_scene_topic, 10)
        self.create_subscription(SceneObjects, scene_topic, self._on_scene, _SCENE_QOS)

        self._unsupported_logged: dict[str, float] = {}
        self._last_count: Optional[int] = None

        self.get_logger().info(
            f'PlanningSceneSyncNode started: {scene_topic} -> {planning_scene_topic} '
            f'(fixtures only)')

    def _to_collision_object(self, obj: SceneObject, frame_id: str,
                             stamp) -> Optional[CollisionObject]:
        """`SceneObject` 하나를 `CollisionObject` 로 바꾼다."""
        primitive_type = _PRIMITIVE_TYPES.get(obj.type.lower())
        if primitive_type is None:
            now = self.get_clock().now().nanoseconds * 1e-9
            # 기본값이 `-inf` 여야 **첫 번째는 반드시 남는다.** 0.0 으로 두면
            # `now < warn_interval` 인 동안(= sim time 이 막 0 에서 시작한 직후)
            # 첫 경고가 삼켜지는데, 하필 그때가 가장 보고 싶은 구간이다.
            if now - self._unsupported_logged.get(obj.type, float('-inf')) >= self._warn_interval:
                self._unsupported_logged[obj.type] = now
                self.get_logger().warning(
                    f"unsupported object type {obj.type!r} for '{obj.name}'; skipped. "
                    f'supported: {sorted(_PRIMITIVE_TYPES)}')
            return None

        primitive = SolidPrimitive()
        primitive.type = primitive_type
        primitive.dimensions = list(obj.dimensions)

        collision = CollisionObject()
        collision.header.stamp = stamp
        collision.header.frame_id = frame_id
        # id 를 물체 이름으로 둔다 — 같은 id 로 다시 보내면 MoveIt 이 **갱신**하므로
        # 움직이는 물체도 지웠다 다시 넣을 필요가 없다.
        collision.id = obj.name
        collision.primitives = [primitive]
        collision.primitive_poses = [obj.pose]
        collision.operation = CollisionObject.ADD
        return collision

    def _keep(self, obj: SceneObject) -> bool:
        """이 물체를 planning scene 에 넣을지 — 고정물만 넣는다.

        판단 근거를 메시지가 실어 온다. 여기서 이름으로 다시 분류하면 같은 사실이
        발행 노드와 이 노드 두 곳에 적히고, 물체를 하나 더할 때 한쪽만 고쳐진다.
        """
        return obj.fixture

    def _on_scene(self, msg: SceneObjects) -> None:
        scene = PlanningScene()
        # **반드시 diff 여야 한다.** 전체 교체로 보내면 로봇 상태·허용 충돌 행렬 같은
        # 나머지 필드가 빈 값으로 덮여 planning scene 이 망가진다.
        scene.is_diff = True
        scene.world.collision_objects = [
            obj for obj in (
                self._to_collision_object(o, msg.header.frame_id, msg.header.stamp)
                for o in msg.objects if self._keep(o))
            if obj is not None
        ]
        self._publisher.publish(scene)

        count = len(scene.world.collision_objects)
        if count != self._last_count:
            self._last_count = count
            self.get_logger().info(f'planning scene updated: {count} collision object(s)')


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = PlanningSceneSyncNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
