"""mock 백엔드용 scene 상태 발행 노드 (MoveIt planning scene → `/scene/objects`).

`/monitored_planning_scene` 을 구독해 world collision object 를 누적하고, 그 상태를
주기적으로 `rdfp_msgs/SceneObjects` 로 발행한다.

**`/get_planning_scene` 서비스 폴링에서 옮겨 왔다.** 서비스가 스냅샷을 그대로 준다는
점 때문에 처음에는 그쪽을 썼는데, 실측하니 **간헐적으로 빈 world 를 반환한다** —
물체가 있는 상태에서 1초 간격 10회 조회에 `[3,0,0,0,0,0,3,3,3,0]` 이 나왔다. 요청
컴포넌트 마스크를 바꿔도, 다른 조회자를 모두 없애도, RViz 를 종료해도 같았다.
move_group 의 planning scene monitor 가 diff scene 과 parent 를 오가는 것으로 보인다.

토픽은 **diff** 를 실어 나르므로(변경된 물체만, `is_diff=True`) 이 노드가 상태를
누적해야 한다. 그 대가로 서비스의 불안정과 무관해진다.

**구독은 이벤트 기반이지만 발행은 주기적으로 유지한다.** 두 가지 이유다.

1. 백엔드마다 성격이 다르다 — 물리 백엔드(Gazebo/Isaac)는 물체가 계속 움직여 주기
   발행이 자연스럽다. mock 만 이벤트성이면 트윈의 `staleness_ms` 를 백엔드별로
   달리 잡아야 한다.
2. **발행자가 죽은 것을 감지할 수 있다.** 이벤트 발행이면 scene 이 조용히 얼어붙은 것과
   정상인 것이 구분되지 않는다.

**이 노드가 내는 pose 는 ground truth 가 아니다.** planning scene 의 물체는 물리를
갖지 않아 누가 넣은 값 그대로 있으며, 파지에 실패해도 굴러떨어져도 변하지 않는다.
따라서 mock 에서 수집한 에피소드로는 성패를 관측할 수 없고, 이 노드의 값은
**변수·연산·기록 경로가 도는지 확인하는 배관 검증용**이다.
"""

from __future__ import annotations

from typing import Optional

import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener

from robot_control.ros2_utils import get_parameter, parse_float, parse_str
from robot_control.scene.pose_math import IDENTITY_QUAT, compose_pose
from rdfp_msgs.msg import SceneObject, SceneObjects
from rdfp_msgs.srv import ResetScene


_DEFAULT_SCENE_TOPIC = '/scene/objects'
_DEFAULT_RESET_SERVICE = '/scene/reset'
_MONITORED_SCENE_TOPIC = '/monitored_planning_scene'
_APPLY_SCENE_SERVICE = '/apply_planning_scene'
# 서비스 콜백 안에서 기다리는 한도.
_APPLY_TIMEOUT_SEC = 10.0

# 늦게 뜬 구독자(트윈·recorder)도 현재 scene 을 즉시 보게 한다. 구독 측도 durability 를
# 맞춰야 매칭된다 — volatile 로 구독하면 값이 영영 오지 않는다.
_SCENE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# move_group 의 `/monitored_planning_scene` 은 RELIABLE / VOLATILE 로 발행한다.
# TRANSIENT_LOCAL 로 구독하면 QoS 불일치로 **한 건도 받지 못한다** (경고만 뜬다).
_MONITORED_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# SolidPrimitive.type → 계약의 `type` 문자열. 값과 dimensions 순서를 모두 ROS 표준에
# 맞췄으므로 여기서는 이름만 바꾸고 배열은 그대로 옮긴다 (재배열은 조용히 틀릴 수
# 있는 지점을 새로 만든다).
_PRIMITIVE_NAMES = {
    SolidPrimitive.BOX: 'box',
    SolidPrimitive.SPHERE: 'sphere',
    SolidPrimitive.CYLINDER: 'cylinder',
    SolidPrimitive.CONE: 'cone',
    SolidPrimitive.PRISM: 'prism',
}


_PRIMITIVE_TYPES = {name: value for value, name in _PRIMITIVE_NAMES.items()}


class MockSceneStateNode(Node):
    """planning scene diff 를 누적해 `/scene/objects` 로 발행한다."""

    def __init__(self) -> None:
        super().__init__('mock_scene_state')

        self.declare_parameter('base_frame', 'panda_link0')
        self.declare_parameter('scene_topic', _DEFAULT_SCENE_TOPIC)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('warn_interval_sec', 5.0)

        self._base_frame = get_parameter(self, 'base_frame', parse_str)
        scene_topic = get_parameter(self, 'scene_topic', parse_str)
        publish_rate = get_parameter(self, 'publish_rate', parse_float)
        if publish_rate is None or publish_rate <= 0.0:
            raise ValueError(f"'publish_rate' must be > 0, got {publish_rate}")
        self._warn_interval_sec = get_parameter(self, 'warn_interval_sec', parse_float)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # 누적 상태. id → CollisionObject. 발행 시점에 SceneObject 로 변환한다.
        self._objects: dict[str, CollisionObject] = {}

        # 고정물 이름. `/scene/reset` 요청에 실려 온 `fixture` 를 기억한다.
        #
        # planning scene 을 왕복하면 이 분류가 사라지기 때문이다 — MoveIt 의
        # `CollisionObject` 에는 실을 자리가 없어서, 여기서 붙들지 않으면 발행하는
        # `SceneObject.fixture` 가 전부 false 가 된다.
        #
        # **첫 `/scene/reset` 이전에는 알 수 없다.** launch 시점에 planning scene 에
        # 이미 있던 물체는 전부 false 로 나간다. mock 스택은 `planning_scene_sync`
        # 를 쓰지 않으므로(방향이 반대다 — planning scene 이 원천이다) 계획에는
        # 영향이 없고, 영향받는 것은 데이터셋 라벨뿐이다.
        self._fixtures: set[str] = set()

        self._pub = self.create_publisher(SceneObjects, scene_topic, _SCENE_QOS)
        self._sub = self.create_subscription(
            PlanningScene, _MONITORED_SCENE_TOPIC, self._on_planning_scene, _MONITORED_QOS)

        # 쓰기 경로 — 서비스다. 요청/응답이 한 쌍이라 결과 토픽도, 결과를 기다리는
        # 폴링도 필요 없다.
        #
        # **콜백 그룹을 나누는 것이 필수다.** 이 서버 콜백 안에서 다시
        # `/apply_planning_scene` 을 호출하는데, 같은 그룹이면 응답을 기다리는 동안
        # executor 가 막혀 **영원히 끝나지 않는다.** 그래서 서버와 클라이언트를 다른
        # 그룹에 두고 `main` 이 MultiThreadedExecutor 를 쓴다.
        self._reset_srv = self.create_service(
            ResetScene, _DEFAULT_RESET_SERVICE, self._on_reset,
            callback_group=MutuallyExclusiveCallbackGroup())
        self._apply_cli = self.create_client(
            ApplyPlanningScene, _APPLY_SCENE_SERVICE,
            callback_group=MutuallyExclusiveCallbackGroup())

        self._timer = self.create_timer(1.0 / publish_rate, self._on_timer)
        self._last_warn = self.get_clock().now()

        self.get_logger().info(
            f'MockSceneStateNode started (subscribe: {_MONITORED_SCENE_TOPIC}, '
            f'publish: {scene_topic} @ {publish_rate} Hz, frame: {self._base_frame})'
        )

    # ── diff 누적 ───────────────────────────────────────────────

    def _on_planning_scene(self, msg: PlanningScene) -> None:
        """`/monitored_planning_scene` 을 누적 상태에 반영한다.

        `is_diff=False` 는 scene 전체이므로 통째로 교체하고, `is_diff=True` 는 변경된
        물체만 담고 있으므로 `operation` 에 따라 반영한다.

        **`is_diff=True` 에 물체가 없는 것은 'scene 을 비우라'는 뜻이 아니다** — 로봇
        상태만 바뀐 diff 도 같은 토픽으로 오기 때문이다. 그것을 비움으로 해석하면
        물체가 매 주기 사라진다.
        """
        if not msg.is_diff:
            self._objects = {obj.id: obj for obj in msg.world.collision_objects}
            return

        for obj in msg.world.collision_objects:
            self._apply_operation(obj)

    def _apply_operation(self, obj: CollisionObject) -> None:
        """diff 안의 `CollisionObject` 하나를 누적 상태에 반영한다."""
        if obj.operation == CollisionObject.REMOVE:
            if not obj.id:
                # MoveIt 규약: id 가 비면 전체 제거다.
                self._objects.clear()
            else:
                self._objects.pop(obj.id, None)
            return

        if obj.operation == CollisionObject.MOVE:
            # 기하는 그대로 두고 pose 만 바꾼다. 모르는 물체의 MOVE 는 버린다 —
            # 기하가 없어 발행할 수 없기 때문이다.
            existing = self._objects.get(obj.id)
            if existing is None:
                self._warn_throttled(f"MOVE for unknown object '{obj.id}'; ignored")
                return
            existing.pose = obj.pose
            if obj.primitive_poses:
                existing.primitive_poses = obj.primitive_poses
            if obj.mesh_poses:
                existing.mesh_poses = obj.mesh_poses
            return

        # ADD / APPEND. APPEND 는 기하를 덧붙이는 연산이지만 계약이 물체당 기하
        # 하나라 구분 없이 교체한다 (복합 물체는 발행 시점에 경고한다).
        self._objects[obj.id] = obj

    # ── 명령 처리 ───────────────────────────────────────────────

    def _on_reset(self, request: ResetScene.Request,
                  response: ResetScene.Response) -> ResetScene.Response:
        """`objects` 로 scene 을 **통째로 교체**한다.

        개별 추가/삭제를 열지 않는 이유는 그 조합 로직이 클라이언트로 새어 나가고,
        그러면 "어떤 배치였는지" 가 데이터셋 밖에 남기 때문이다.

        **빈 `objects` 는 유효한 요청이다** — 'scene 을 비운다'는 뜻이다.
        """
        if not self._apply_cli.service_is_ready():
            return self._fail(response, f"service '{_APPLY_SCENE_SERVICE}' not ready")

        scene = PlanningScene()
        scene.is_diff = True
        # 기존 물체 전체 제거 후 새로 넣는다. id 가 빈 REMOVE 가 '전부 제거'다.
        clear_all = CollisionObject()
        clear_all.operation = CollisionObject.REMOVE
        objects = [clear_all]
        for obj in request.objects:
            collision_object = self._to_collision_object(obj)
            if collision_object is None:
                return self._fail(response, f"unsupported type {obj.type!r} for '{obj.name}'")
            objects.append(collision_object)
        scene.world.collision_objects = objects

        # scene 을 통째로 교체하는 요청이므로 분류도 통째로 갈아 끼운다.
        fixtures = {obj.name for obj in request.objects if obj.fixture}

        apply_request = ApplyPlanningScene.Request()
        apply_request.scene = scene
        future = self._apply_cli.call_async(apply_request)
        # 다른 콜백 그룹이므로 여기서 기다려도 executor 가 막히지 않는다.
        applied = self._await(future)
        if applied is None:
            return self._fail(response, 'apply_planning_scene did not respond in time')
        if not applied.success:
            return self._fail(response, 'apply_planning_scene rejected the scene')

        # 적용에 성공한 뒤에 반영한다 — 거부된 요청의 분류를 남기면 실제 scene 과
        # 어긋난다.
        self._fixtures = fixtures

        response.success = True
        response.message = ''
        response.applied_count = len(request.objects)
        self.get_logger().info(
            f'scene reset applied: {response.applied_count} object(s), '
            f'{len(fixtures)} fixture(s) '
            f'(scene={request.scene!r}, seed={request.seed})')
        return response

    def _await(self, future, timeout_sec: float = _APPLY_TIMEOUT_SEC):
        """`call_async` 결과를 기다린다. 시간 초과면 ``None``.

        `spin_until_future_complete` 를 쓰지 않는다 — 이미 executor 가 다른 스레드에서
        노드를 돌리고 있으므로, 여기서 또 spin 하면 콜백이 그쪽으로 가서 영영 오지
        않는다.
        """
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                try:
                    return future.result()
                except Exception as exc:   # noqa: BLE001
                    self.get_logger().error(f'apply_planning_scene failed: {exc}')
                    return None
            time.sleep(0.01)
        return None

    def _fail(self, response: ResetScene.Response, message: str) -> ResetScene.Response:
        self.get_logger().warning(f'scene reset failed: {message}')
        response.success = False
        response.message = message
        response.applied_count = 0
        return response

    def _to_collision_object(self, obj: SceneObject) -> Optional[CollisionObject]:
        """계약의 `SceneObject` 를 planning scene 의 `CollisionObject` 로 되돌린다.

        `dimensions` 순서가 `shape_msgs/SolidPrimitive` 와 같으므로 **재배열 없이**
        그대로 옮긴다 — 재배열은 조용히 틀릴 수 있는 지점을 새로 만든다.
        """
        primitive_type = _PRIMITIVE_TYPES.get(obj.type)
        if primitive_type is None:
            return None

        collision_object = CollisionObject()
        collision_object.id = obj.name
        collision_object.header.frame_id = self._base_frame
        collision_object.operation = CollisionObject.ADD
        collision_object.pose = obj.pose

        primitive = SolidPrimitive()
        primitive.type = primitive_type
        primitive.dimensions = [float(d) for d in obj.dimensions]
        collision_object.primitives = [primitive]

        # 물체 pose 에 이미 자세가 실려 있으므로 primitive 는 원점에 둔다.
        identity = Pose()
        identity.orientation.w = 1.0
        collision_object.primitive_poses = [identity]
        return collision_object

    # ── 발행 ────────────────────────────────────────────────────

    def _on_timer(self) -> None:
        objects = []
        for collision_object in self._objects.values():
            converted = self._convert(collision_object)
            if converted is not None:
                objects.append(converted)

        msg = SceneObjects()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.objects = objects
        self._pub.publish(msg)

    # ── 변환 ────────────────────────────────────────────────────

    def _convert(self, obj: CollisionObject) -> Optional[SceneObject]:
        """`CollisionObject` 를 `SceneObject` 로 옮긴다.

        기하가 없거나 좌표 변환이 불가능하면 ``None`` 을 반환하고 경고만 남긴다 —
        물체 하나 때문에 나머지 scene 전체를 발행하지 못하는 편이 더 나쁘다.
        """
        primitive_count = len(obj.primitives)
        if primitive_count > 1:
            # 계약은 물체 하나에 기하 하나다. 복합 물체는 첫 기하만 옮기므로 크기가
            # 실제와 달라진다 — 조용히 넘기지 않고 알린다.
            self._warn_throttled(
                f"object '{obj.id}' has {primitive_count} primitives; only the first is published")

        if primitive_count > 0:
            primitive = obj.primitives[0]
            type_name = _PRIMITIVE_NAMES.get(primitive.type)
            if type_name is None:
                self._warn_throttled(
                    f"object '{obj.id}' has unsupported primitive type {primitive.type}; skipped")
                return None
            dimensions = [float(d) for d in primitive.dimensions]
            local_pose = obj.primitive_poses[0] if obj.primitive_poses else Pose()
        elif obj.meshes:
            type_name = 'mesh'
            dimensions = []
            local_pose = obj.mesh_poses[0] if obj.mesh_poses else Pose()
        else:
            self._warn_throttled(f"object '{obj.id}' has no geometry; skipped")
            return None

        pose = self._to_base_frame(obj, local_pose)
        if pose is None:
            return None

        scene_object = SceneObject()
        scene_object.name = obj.id
        scene_object.type = type_name
        scene_object.dimensions = dimensions
        scene_object.pose = pose
        scene_object.fixture = obj.id in self._fixtures
        return scene_object

    def _to_base_frame(self, obj: CollisionObject, local_pose: Pose) -> Optional[Pose]:
        """primitive pose 를 base_frame 기준으로 옮긴다.

        합성이 두 번 필요하다 — `base ← 오브젝트 프레임`(TF)과
        `오브젝트 ← primitive`(CollisionObject 내부)다. 후자를 빼먹으면 오프셋을 가진
        복합 물체의 위치가 조용히 틀린다.
        """
        # 1) 오브젝트 프레임 기준 primitive pose
        position, orientation = _as_tuples(local_pose)
        object_position, object_orientation = _as_tuples(obj.pose)
        position, orientation = compose_pose(
            object_position, object_orientation, position, orientation)

        # 2) base_frame ← 오브젝트 프레임
        source_frame = obj.header.frame_id or self._base_frame
        if source_frame != self._base_frame:
            try:
                tf = self._tf_buffer.lookup_transform(self._base_frame, source_frame, Time())
            except TransformException as exc:
                self._warn_throttled(
                    f"no transform {source_frame} -> {self._base_frame} for '{obj.id}': {exc}")
                return None
            translation = tf.transform.translation
            rotation = tf.transform.rotation
            position, orientation = compose_pose(
                (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w),
                position, orientation)

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = orientation
        return pose

    # ── 공통 ────────────────────────────────────────────────────

    def _warn_throttled(self, message: str) -> None:
        """발행 주기마다 같은 경고가 쏟아지는 것을 막는다."""
        now = self.get_clock().now()
        if (now - self._last_warn).nanoseconds * 1e-9 < self._warn_interval_sec:
            return
        self._last_warn = now
        self.get_logger().warning(message)


def _as_tuples(pose: Pose) -> tuple[tuple[float, float, float],
                                    tuple[float, float, float, float]]:
    """`Pose` 를 (위치, 자세) 튜플로 분해한다.

    자세가 전부 0 인 경우(기본 생성된 `Pose` 를 그대로 둔 경우)는 회전 없음으로
    본다 — norm 0 짜리 쿼터니언을 그대로 합성하면 모든 좌표가 0 이 되어 물체가
    원점으로 사라진다.
    """
    q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    if q == (0.0, 0.0, 0.0, 0.0):
        q = IDENTITY_QUAT
    return (pose.position.x, pose.position.y, pose.position.z), q


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockSceneStateNode()
    # 서비스 콜백 안에서 다른 서비스를 기다리므로 **단일 스레드로는 교착된다.**
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


__all__ = ['MockSceneStateNode', 'main']
