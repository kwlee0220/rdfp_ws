"""planning_scene_sync_node 의 변환·attach/detach 테스트.

노드를 spin 하지 않고 `_to_collision_object` / `_on_scene` / `_on_command` 만 직접
부른다. ROS 메시지 타입이 필요하므로 sourcing 이 안 된 환경에서는 전체를 skip 한다.
"""

from __future__ import annotations

import pytest

pytest.importorskip('moveit_msgs', reason='requires ROS 2 runtime (moveit_msgs)')

from geometry_msgs.msg import Pose                                     # noqa: E402
from moveit_msgs.msg import CollisionObject                            # noqa: E402
from rdfp_msgs.msg import SceneObject, SceneObjects                   # noqa: E402
from shape_msgs.msg import SolidPrimitive                              # noqa: E402

from robot_control.scene.planning_scene_sync_node import PlanningSceneSyncNode  # noqa: E402

BASE = 'panda_link0'


class _StubClock:
    """`get_clock().now()` 만 흉내낸다. 경과 시간은 테스트가 직접 민다."""

    def __init__(self) -> None:
        self.nanoseconds = 0

    def now(self):
        return self

    def to_msg(self):
        from builtin_interfaces.msg import Time
        stamp = Time()
        stamp.sec = int(self.nanoseconds // 1_000_000_000)
        stamp.nanosec = int(self.nanoseconds % 1_000_000_000)
        return stamp


class _StubLogger:
    def __init__(self) -> None:
        self.warnings: list[str] = []
        self.infos: list[str] = []

    def warning(self, message: str) -> None:
        self.warnings.append(message)

    def info(self, message: str) -> None:
        self.infos.append(message)


class _StubPublisher:
    def __init__(self) -> None:
        self.published: list = []

    def publish(self, msg) -> None:
        self.published.append(msg)


class _StubNode:
    """Node 초기화를 피하고 대상 메서드만 빌려 쓴다."""

    def __init__(self) -> None:
        self._clock = _StubClock()
        self._logger = _StubLogger()
        self._publisher = _StubPublisher()
        self._warn_interval = 5.0
        self._unsupported_logged: dict[str, float] = {}
        self._last_count = -1

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._logger

    _to_collision_object = PlanningSceneSyncNode._to_collision_object
    _keep = PlanningSceneSyncNode._keep
    _on_scene = PlanningSceneSyncNode._on_scene


def _obj(name: str, obj_type: str = 'box', dims=(0.05, 0.05, 0.05),
         fixture: bool = False) -> SceneObject:
    obj = SceneObject()
    obj.name = name
    obj.type = obj_type
    obj.dimensions = list(dims)
    obj.fixture = fixture
    obj.pose = Pose()
    obj.pose.position.x = 0.5
    obj.pose.orientation.w = 1.0
    return obj


def _scene(*objects: SceneObject) -> SceneObjects:
    msg = SceneObjects()
    msg.header.frame_id = BASE
    msg.objects = list(objects)
    return msg


# ---------- 변환 ----------

def test_box_becomes_collision_object():
    node = _StubNode()
    collision = node._to_collision_object(_obj('block_a'), BASE, _StubClock().to_msg())
    assert collision.id == 'block_a'
    assert collision.header.frame_id == BASE
    assert collision.operation == CollisionObject.ADD
    assert collision.primitives[0].type == SolidPrimitive.BOX
    assert list(collision.primitives[0].dimensions) == [0.05, 0.05, 0.05]


def test_unsupported_type_is_skipped_not_raised():
    node = _StubNode()
    assert node._to_collision_object(_obj('blob', 'mesh'), BASE, _StubClock().to_msg()) is None
    assert any('unsupported object type' in w for w in node._logger.warnings)


def test_unsupported_type_warning_is_throttled():
    """같은 타입을 계속 받아도 경고는 `warn_interval` 마다 한 번이다."""
    node = _StubNode()
    stamp = _StubClock().to_msg()
    for _ in range(5):
        node._to_collision_object(_obj('blob', 'mesh'), BASE, stamp)
    assert len(node._logger.warnings) == 1

    # 시간을 밀면 다시 한 번 나온다.
    node._clock.nanoseconds = int(6 * 1e9)
    node._to_collision_object(_obj('blob', 'mesh'), BASE, stamp)
    assert len(node._logger.warnings) == 2


# ---------- world 동기화 ----------

def test_scene_is_published_as_diff():
    """전체 교체로 보내면 로봇 상태·ACM 이 빈 값으로 덮인다."""
    node = _StubNode()
    node._on_scene(_scene(_obj('table', dims=(0.6, 1.0, 0.4), fixture=True)))
    published = node._publisher.published[-1]
    assert published.is_diff is True
    assert [o.id for o in published.world.collision_objects] == ['table']


# ---------- 고정물 선별 ----------

def test_only_fixtures_are_kept():
    """**조작 대상은 planning scene 에 넣지 않는다.**

    넣으면 파지 자체가 충돌이 되어 관절공간 계획이 `INVALID_MOTION_PLAN`(-2)으로
    거부된다. 그것을 피하려고 물체를 손에 붙였다 떼면, 물체가 미끄러져 떨어졌을 때
    planner 의 믿음이 조용히 틀린다.
    """
    node = _StubNode()
    node._on_scene(_scene(_obj('block_a'),
                          _obj('table', dims=(0.6, 1.0, 0.4), fixture=True),
                          _obj('block_b')))
    assert [o.id for o in node._publisher.published[-1].world.collision_objects] == ['table']


def test_nothing_is_kept_when_no_object_is_a_fixture():
    """`fixture` 를 채우지 않는 발행자가 붙으면 planning scene 이 빈다.

    기본값 false 를 '조작 대상' 으로 둔 결과다. 팔이 탁자를 향해 그대로 내려가므로 눈에
    바로 보이고 원인도 빈 planning scene 을 가리킨다 — 반대로 기본값이 '고정물' 이면
    파지 계획이 INVALID_MOTION_PLAN 으로 거부되는데, 로그에 "계획 실패" 만 남아
    플래그를 안 채웠다는 단서가 없다.
    """
    node = _StubNode()
    node._on_scene(_scene(_obj('block_a'), _obj('table')))
    assert node._publisher.published[-1].world.collision_objects == []


def test_classification_follows_the_message_not_the_name():
    """이름이 아니라 메시지의 `fixture` 가 판단 근거다.

    이름으로 다시 분류하면 같은 사실이 발행 노드와 이 노드 두 곳에 적히고, 물체를
    하나 더할 때 한쪽만 고쳐진다.
    """
    node = _StubNode()
    node._on_scene(_scene(_obj('block_a', fixture=True), _obj('table')))
    assert [o.id for o in node._publisher.published[-1].world.collision_objects] == ['block_a']


def test_fixture_selection_survives_objects_appearing_later():
    """물체가 나중에 나타나도 선별은 그대로 적용된다."""
    node = _StubNode()
    node._on_scene(_scene(_obj('table', fixture=True)))
    node._on_scene(_scene(_obj('table', fixture=True), _obj('block_c')))
    assert [o.id for o in node._publisher.published[-1].world.collision_objects] == ['table']
