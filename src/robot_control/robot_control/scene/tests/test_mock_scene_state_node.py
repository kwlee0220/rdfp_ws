"""mock_scene_state_node 의 CollisionObject → SceneObject 변환 테스트.

노드를 spin 하지 않고 `_convert` / `_to_base_frame` 만 직접 부른다. ROS 메시지 타입이
필요하므로 sourcing 이 안 된 환경에서는 전체를 skip 한다.
"""

from __future__ import annotations

import math

import pytest

pytest.importorskip('moveit_msgs', reason='requires ROS 2 runtime (moveit_msgs)')

from geometry_msgs.msg import Pose, TransformStamped                  # noqa: E402
from moveit_msgs.msg import CollisionObject, PlanningScene            # noqa: E402
from shape_msgs.msg import Mesh, SolidPrimitive                       # noqa: E402

from robot_control.scene.mock_scene_state_node import MockSceneStateNode       # noqa: E402


BASE = 'panda_link0'
Z90 = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))


class _StubNode:
    """`_convert` 가 쓰는 부분만 갖춘 대역 (Node 초기화를 피한다)."""

    def __init__(self, transforms: dict = None, fixtures=None) -> None:
        self._base_frame = BASE
        self.warnings: list[str] = []
        self._transforms = transforms or {}
        self._fixtures = set(fixtures or ())

    # MockSceneStateNode 의 메서드를 그대로 빌려 쓴다.
    _convert = MockSceneStateNode._convert
    _to_base_frame = MockSceneStateNode._to_base_frame

    def _warn_throttled(self, message: str) -> None:
        self.warnings.append(message)

    @property
    def _tf_buffer(self):
        stub = self

        class _Buffer:
            def lookup_transform(self, target, source, _time):
                from tf2_ros import TransformException
                key = (target, source)
                if key not in stub._transforms:
                    raise TransformException(f'no transform {source} -> {target}')
                return stub._transforms[key]

        return _Buffer()


def _pose(x=0.0, y=0.0, z=0.0, quat=(0.0, 0.0, 0.0, 1.0)) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
    return p


def _box(name='cube_0', frame=BASE, obj_pose=None, primitive_pose=None) -> CollisionObject:
    obj = CollisionObject()
    obj.id = name
    obj.header.frame_id = frame
    obj.pose = obj_pose if obj_pose is not None else _pose()
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [0.05, 0.06, 0.07]
    obj.primitives = [primitive]
    obj.primitive_poses = [primitive_pose if primitive_pose is not None else _pose()]
    return obj


def _transform(x=0.0, y=0.0, z=0.0, quat=(0.0, 0.0, 0.0, 1.0)) -> TransformStamped:
    t = TransformStamped()
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
    (t.transform.rotation.x, t.transform.rotation.y,
     t.transform.rotation.z, t.transform.rotation.w) = quat
    return t


# ----- 기하 변환 -----

def test_box_keeps_solid_primitive_dimension_order() -> None:
    """계약이 SolidPrimitive 와 같은 순서라 재배열 없이 그대로 옮긴다."""
    out = _StubNode()._convert(_box(obj_pose=_pose(0.4, -0.1, 0.02)))

    assert out.name == 'cube_0'
    assert out.type == 'box'
    assert list(out.dimensions) == [0.05, 0.06, 0.07]
    assert (out.pose.position.x, out.pose.position.y, out.pose.position.z) == (0.4, -0.1, 0.02)


def test_cylinder_dimensions_are_height_then_radius() -> None:
    """SolidPrimitive 의 CYLINDER_HEIGHT=0, CYLINDER_RADIUS=1 순서를 따른다."""
    obj = _box()
    obj.primitives[0].type = SolidPrimitive.CYLINDER
    obj.primitives[0].dimensions = [0.10, 0.02]     # 높이 0.10, 반지름 0.02

    out = _StubNode()._convert(obj)

    assert out.type == 'cylinder'
    assert list(out.dimensions) == [0.10, 0.02]


def test_mesh_object_has_no_dimensions() -> None:
    obj = CollisionObject()
    obj.id = 'tray'
    obj.header.frame_id = BASE
    obj.pose = _pose(0.3, 0.0, 0.0)
    obj.meshes = [Mesh()]
    obj.mesh_poses = [_pose()]

    out = _StubNode()._convert(obj)

    assert out.type == 'mesh'
    assert list(out.dimensions) == []


# ----- pose 합성 -----

def test_primitive_offset_is_composed_with_the_object_pose() -> None:
    """이 합성을 빼먹으면 오프셋을 가진 물체 위치가 조용히 틀린다."""
    out = _StubNode()._convert(
        _box(obj_pose=_pose(0.4, 0.0, 0.0, Z90), primitive_pose=_pose(0.1, 0.0, 0.0)))

    # 부모가 z축 90° 돌아 있으므로 자식의 +x 오프셋은 +y 로 간다.
    assert out.pose.position.x == pytest.approx(0.4)
    assert out.pose.position.y == pytest.approx(0.1)


def test_pose_is_transformed_into_the_base_frame() -> None:
    node = _StubNode(transforms={(BASE, 'world'): _transform(x=-0.5)})

    out = node._convert(_box(frame='world', obj_pose=_pose(1.0, 0.0, 0.0)))

    assert out.pose.position.x == pytest.approx(0.5)


def test_object_is_skipped_when_the_transform_is_missing() -> None:
    """물체 하나 때문에 scene 전체 발행을 막지 않는다 — 경고 후 건너뛴다."""
    node = _StubNode()

    assert node._convert(_box(frame='world')) is None
    assert any('no transform' in w for w in node.warnings)


def test_zero_quaternion_is_treated_as_no_rotation() -> None:
    """기본 생성된 Pose 를 그대로 두면 자세가 (0,0,0,0)이라 norm 이 0 이다.

    그대로 합성하면 모든 좌표가 0 이 되어 **물체가 원점으로 사라진다.**
    """
    obj = _box(obj_pose=_pose(0.4, 0.1, 0.2, quat=(0.0, 0.0, 0.0, 0.0)))

    out = _StubNode()._convert(obj)

    assert (out.pose.position.x, out.pose.position.y) == (0.4, 0.1)
    assert out.pose.orientation.w == pytest.approx(1.0)


# ----- 방어 -----

def test_object_without_geometry_is_skipped() -> None:
    obj = CollisionObject()
    obj.id = 'ghost'
    obj.header.frame_id = BASE

    node = _StubNode()

    assert node._convert(obj) is None
    assert any('no geometry' in w for w in node.warnings)


def test_unsupported_primitive_type_is_skipped_with_a_warning() -> None:
    obj = _box()
    obj.primitives[0].type = 99

    node = _StubNode()

    assert node._convert(obj) is None
    assert any('unsupported primitive type' in w for w in node.warnings)


def test_multi_primitive_object_warns_but_still_publishes() -> None:
    """첫 기하만 나가므로 크기가 실제와 달라진다 — 조용히 넘기지 않는다."""
    obj = _box()
    second = SolidPrimitive()
    second.type = SolidPrimitive.SPHERE
    second.dimensions = [0.01]
    obj.primitives.append(second)
    obj.primitive_poses.append(_pose())

    node = _StubNode()
    out = node._convert(obj)

    assert out.type == 'box'
    assert any('only the first is published' in w for w in node.warnings)


# ----- diff 누적 (`/monitored_planning_scene`) -----

class _AccumNode(_StubNode):
    """`_on_planning_scene` 까지 빌려 쓰는 대역."""

    def __init__(self) -> None:
        super().__init__()
        self._objects = {}

    _on_planning_scene = MockSceneStateNode._on_planning_scene
    _apply_operation = MockSceneStateNode._apply_operation

    @property
    def names(self) -> list:
        return sorted(self._objects)


def _scene(objects, is_diff=True) -> PlanningScene:
    s = PlanningScene()
    s.is_diff = is_diff
    s.world.collision_objects = objects
    return s


def _with_op(obj: CollisionObject, operation) -> CollisionObject:
    obj.operation = operation
    return obj


def test_add_diff_accumulates_objects() -> None:
    node = _AccumNode()

    node._on_planning_scene(_scene([_with_op(_box('cube_0'), CollisionObject.ADD)]))
    node._on_planning_scene(_scene([_with_op(_box('ball_0'), CollisionObject.ADD)]))

    assert node.names == ['ball_0', 'cube_0']


def test_empty_diff_does_not_clear_the_scene() -> None:
    """로봇 상태만 바뀐 diff 도 같은 토픽으로 온다.

    이것을 '비우라'로 해석하면 물체가 매 주기 사라진다 — 서비스 폴링에서 겪은
    증상과 똑같아진다.
    """
    node = _AccumNode()
    node._on_planning_scene(_scene([_with_op(_box('cube_0'), CollisionObject.ADD)]))

    node._on_planning_scene(_scene([]))

    assert node.names == ['cube_0']


def test_full_scene_replaces_everything() -> None:
    node = _AccumNode()
    node._on_planning_scene(_scene([_with_op(_box('cube_0'), CollisionObject.ADD)]))

    node._on_planning_scene(_scene([_box('other')], is_diff=False))

    assert node.names == ['other']


def test_full_empty_scene_clears_everything() -> None:
    """`is_diff=False` 인 빈 scene 은 '전부 지웠다'가 맞다."""
    node = _AccumNode()
    node._on_planning_scene(_scene([_with_op(_box('cube_0'), CollisionObject.ADD)]))

    node._on_planning_scene(_scene([], is_diff=False))

    assert node.names == []


def test_remove_deletes_one_object() -> None:
    node = _AccumNode()
    node._on_planning_scene(_scene([_with_op(_box('cube_0'), CollisionObject.ADD),
                                    _with_op(_box('ball_0'), CollisionObject.ADD)]))

    node._on_planning_scene(_scene([_with_op(_box('cube_0'), CollisionObject.REMOVE)]))

    assert node.names == ['ball_0']


def test_remove_with_empty_id_clears_all() -> None:
    """MoveIt 규약 — id 가 비면 전체 제거다."""
    node = _AccumNode()
    node._on_planning_scene(_scene([_with_op(_box('cube_0'), CollisionObject.ADD)]))

    node._on_planning_scene(_scene([_with_op(_box(''), CollisionObject.REMOVE)]))

    assert node.names == []


def test_move_updates_pose_and_keeps_geometry() -> None:
    node = _AccumNode()
    node._on_planning_scene(_scene([_with_op(_box('cube_0'), CollisionObject.ADD)]))

    moved = CollisionObject()
    moved.id, moved.header.frame_id = 'cube_0', BASE
    moved.pose = _pose(0.7, 0.2, 0.05)
    node._on_planning_scene(_scene([_with_op(moved, CollisionObject.MOVE)]))

    out = node._convert(node._objects['cube_0'])
    assert out.type == 'box'                       # 기하가 살아 있다
    assert list(out.dimensions) == [0.05, 0.06, 0.07]
    assert out.pose.position.x == pytest.approx(0.7)


def test_move_for_unknown_object_is_ignored_with_a_warning() -> None:
    """기하를 모르는 물체는 발행할 수 없다."""
    node = _AccumNode()

    ghost = CollisionObject()
    ghost.id, ghost.header.frame_id = 'ghost', BASE
    node._on_planning_scene(_scene([_with_op(ghost, CollisionObject.MOVE)]))

    assert node.names == []
    assert any('MOVE for unknown object' in w for w in node.warnings)


def test_add_replaces_an_object_with_the_same_name() -> None:
    node = _AccumNode()
    node._on_planning_scene(_scene([_with_op(_box('cube_0'), CollisionObject.ADD)]))

    bigger = _box('cube_0')
    bigger.primitives[0].dimensions = [0.2, 0.2, 0.2]
    node._on_planning_scene(_scene([_with_op(bigger, CollisionObject.ADD)]))

    assert node.names == ['cube_0']
    assert list(node._convert(node._objects['cube_0']).dimensions) == [0.2, 0.2, 0.2]


# ---------- 고정물 분류 ----------

def test_fixture_flag_comes_from_the_remembered_reset():
    """`/scene/reset` 에서 기억한 이름만 `fixture=True` 로 나간다.

    planning scene 을 왕복하면 이 분류가 사라진다 — MoveIt 의 `CollisionObject` 에
    실을 자리가 없어서, 노드가 붙들지 않으면 전부 false 가 된다.
    """
    node = _StubNode(fixtures={'table'})
    table = node._convert(_box('table'))
    block = node._convert(_box('block_a'))
    assert table.fixture is True
    assert block.fixture is False


def test_fixture_is_false_before_the_first_reset():
    """첫 `/scene/reset` 이전에는 알 수 없으므로 전부 조작 대상으로 나간다."""
    node = _StubNode()
    assert node._convert(_box('table')).fixture is False
