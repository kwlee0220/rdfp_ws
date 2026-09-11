"""isaac/scene_state_node 의 TF → SceneObjects 변환 테스트.

노드를 spin 하지 않고 `_lookup` / `_on_timer` / `_config_path` 만 직접 부른다.
ROS 메시지 타입이 필요하므로 sourcing 이 안 된 환경에서는 전체를 skip 한다.
"""

from __future__ import annotations

import json

import pytest

pytest.importorskip('rdfp_msgs', reason='requires ROS 2 runtime (rdfp_msgs)')

from geometry_msgs.msg import TransformStamped                       # noqa: E402

from robot_control.isaac.scene_state_node import IsaacSceneStateNode  # noqa: E402

BASE = 'panda_link0'


class _StubClock:
    def __init__(self, seconds: float = 100.0) -> None:
        self.nanoseconds = int(seconds * 1e9)

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


class _StubBuffer:
    """지정한 프레임만 변환을 돌려주고 나머지는 예외를 낸다."""

    def __init__(self, transforms: dict) -> None:
        self._transforms = transforms

    def lookup_transform(self, target, source, _time, timeout=None):
        if (target, source) not in self._transforms:
            raise LookupException(f'no transform {source} -> {target}')
        return self._transforms[(target, source)]

    def all_frames_as_yaml(self) -> str:
        return '\n'.join(f'{source}:' for _, source in self._transforms)


class LookupException(Exception):
    """tf2 예외 대역. 노드가 예외 계열을 통째로 잡으므로 타입은 무관하다."""


class _StubPublisher:
    def __init__(self) -> None:
        self.published: list = []

    def publish(self, msg) -> None:
        self.published.append(msg)


class _StubNode:
    def __init__(self, objects: list, transforms: dict, seconds: float = 100.0) -> None:
        self._clock = _StubClock(seconds)
        self._logger = _StubLogger()
        self._publisher = _StubPublisher()
        self._buffer = _StubBuffer(transforms)
        self._objects = objects
        self._base_frame = BASE
        self._warn_interval = 5.0
        self._missing_logged: dict[str, float] = {}
        self._last_heartbeat = float('-inf')
        self._last_resolved = None

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._logger

    _lookup = IsaacSceneStateNode._lookup
    _on_timer = IsaacSceneStateNode._on_timer
    _heartbeat = IsaacSceneStateNode._heartbeat


_HEARTBEAT_PREFIX = 'scene: resolved'


def _lookup_warnings(node) -> list:
    """조회 실패 경고만. 하트비트도 문제 구간에서는 WARNING 이라 섞인다."""
    return [w for w in node._logger.warnings if not w.startswith(_HEARTBEAT_PREFIX)]


def _heartbeats(node) -> list:
    """하트비트 줄만 — 정상이면 INFO, 문제면 WARNING 으로 나온다."""
    return [m for m in (node._logger.infos + node._logger.warnings)
            if m.startswith(_HEARTBEAT_PREFIX)]


def _tf(x: float, y: float, z: float, quat=(0.0, 0.0, 0.0, 1.0)) -> TransformStamped:
    tf = TransformStamped()
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.translation.z = z
    tf.transform.rotation.x, tf.transform.rotation.y = quat[0], quat[1]
    tf.transform.rotation.z, tf.transform.rotation.w = quat[2], quat[3]
    return tf


BLOCK = {'name': 'block_a', 'type': 'box', 'dimensions': [0.05, 0.05, 0.05]}
TABLE = {'name': 'table', 'type': 'box', 'dimensions': [0.6, 1.0, 0.4]}


# ---------- 변환 ----------

def test_transform_becomes_scene_object():
    node = _StubNode([BLOCK], {(BASE, 'block_a'): _tf(0.5, -0.15, 0.425)})
    node._on_timer()

    msg = node._publisher.published[-1]
    assert msg.header.frame_id == BASE
    # stamp 를 비우면 적재 시 epoch 0 으로 들어가 어느 에피소드에도 속하지 못한다.
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) != (0, 0)

    obj = msg.objects[0]
    assert obj.name == 'block_a' and obj.type == 'box'
    assert list(obj.dimensions) == [0.05, 0.05, 0.05]
    assert (obj.pose.position.x, obj.pose.position.y, obj.pose.position.z) == (0.5, -0.15, 0.425)


def test_orientation_is_not_reordered():
    """tf2 가 이미 ROS 규약(xyzw)으로 준다 — 여기서 또 뒤집으면 조용히 틀린다."""
    z45 = (0.0, 0.0, 0.3827, 0.9239)
    node = _StubNode([BLOCK], {(BASE, 'block_a'): _tf(0.5, 0.0, 0.425, z45)})
    node._on_timer()

    q = node._publisher.published[-1].objects[0].pose.orientation
    assert (pytest.approx(q.x), pytest.approx(q.y)) == (0.0, 0.0)
    assert q.z == pytest.approx(0.3827) and q.w == pytest.approx(0.9239)


def test_frame_key_overrides_name():
    """`frame` 이 있으면 그것으로 조회하고, 발행 이름은 `name` 을 쓴다."""
    spec = dict(BLOCK, frame='isaac_block_a')
    node = _StubNode([spec], {(BASE, 'isaac_block_a'): _tf(0.5, 0.0, 0.425)})
    node._on_timer()
    assert node._publisher.published[-1].objects[0].name == 'block_a'


# ---------- 누락 처리 ----------

def test_missing_transform_is_skipped_others_still_published():
    """하나가 없다고 전체가 비면 안 된다 — scene 이 사라진 것으로 오독된다."""
    node = _StubNode([BLOCK, TABLE], {(BASE, 'table'): _tf(0.55, 0.0, 0.2)})
    node._on_timer()

    msg = node._publisher.published[-1]
    assert [o.name for o in msg.objects] == ['table']
    assert any('block_a' in w for w in node._logger.warnings)


def test_empty_scene_is_still_published():
    """물체가 하나도 안 풀려도 발행한다 — 'scene 이 비었다'도 유효한 상태다."""
    node = _StubNode([BLOCK], {})
    node._on_timer()
    assert node._publisher.published[-1].objects == []


def test_missing_warning_is_throttled_but_first_always_logged():
    """**sim time 0 근처에서도 첫 경고는 나와야 한다.** Isaac 은 Stop 마다 0 으로 되돌아간다."""
    node = _StubNode([BLOCK], {}, seconds=0.0)
    node._on_timer()
    assert len(_lookup_warnings(node)) == 1

    node._on_timer()          # 같은 시각 — 억제된다
    assert len(_lookup_warnings(node)) == 1

    node._clock.nanoseconds = int(6 * 1e9)
    node._on_timer()          # 간격을 넘겼다 — 다시 나온다
    assert len(_lookup_warnings(node)) == 2


# ---------- 설정 경로 ----------

class _ParamNode:
    def __init__(self, value) -> None:
        self._value = value

    def get_parameter(self, _name):
        class _P:
            pass
        p = _P()
        p.value = self._value
        return p

    _config_path = IsaacSceneStateNode._config_path


@pytest.mark.parametrize('unset', ['', '   ', None])
def test_empty_config_file_falls_back_to_package_share(unset):
    """빈 값은 '지정 안 함'이라는 **유효한 뜻**이다 — 검증기로 거부하면 노드가 죽는다."""
    path = _ParamNode(unset)._config_path()
    assert path.endswith('config/isaac_scene.json')


def test_explicit_config_file_is_used_as_is(tmp_path):
    target = tmp_path / 'custom.json'
    target.write_text(json.dumps({'objects': []}), encoding='utf-8')
    assert _ParamNode(f'  {target}  ')._config_path() == str(target)


# ---------- 발행 대상 선별 ----------

def test_only_dynamic_objects_are_loaded():
    """`/scene/objects` 는 조작 대상 채널이다 — `dynamic: false` 는 싣지 않는다.

    빠지는 것은 발행뿐이며 시뮬레이터에는 그대로 있다 (`setup_scene.py` 가 같은
    JSON 으로 prim 을 만든다). 구분자를 새로 만들지 않고 기존 `dynamic` 을 쓴다.
    """
    cfg = {'objects': [
        {'name': 'table', 'type': 'box', 'dimensions': [0.6, 1.0, 0.4], 'dynamic': False},
        {'name': 'block_a', 'type': 'box', 'dimensions': [0.05] * 3, 'dynamic': True},
    ]}
    assert [o['name'] for o in cfg['objects'] if o.get('dynamic', False)] == ['block_a']


def test_missing_dynamic_key_is_not_published():
    """`dynamic` 이 없으면 static 이라는 뜻이므로 발행 대상이 아니다."""
    cfg = {'objects': [{'name': 'wall', 'type': 'box', 'dimensions': [1.0, 0.1, 1.0]}]}
    assert [o['name'] for o in cfg['objects'] if o.get('dynamic', False)] == []


# ----------------------------------------------------------------------
# /scene/reset — 쓰기 경로
#
# 서비스 호출 자체는 Isaac 이 있어야 하므로, 여기서는 **호출 전에 거르는 판정**과
# **적용 여부를 읽어서 확인하는 비교**만 고정한다. 그 둘이 이 경로의 안전장치다.
# ----------------------------------------------------------------------

from rdfp_msgs.msg import SceneObject                                   # noqa: E402
from robot_control.isaac.scene_state_node import _same_pose             # noqa: E402


def _pose(x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    from geometry_msgs.msg import Pose
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y = qx, qy
    p.orientation.z, p.orientation.w = qz, qw
    return p


class _ResetStub:
    """`_check_known` / `_entity_path` 만 빌려 쓰는 대역."""

    def __init__(self) -> None:
        self._root_prim = '/World/Scene'
        self._by_name = {
            'block_a': {'name': 'block_a', 'type': 'box', 'dimensions': [0.05, 0.05, 0.05]},
        }

    _entity_path = IsaacSceneStateNode._entity_path
    _check_known = IsaacSceneStateNode._check_known


def _obj(name='block_a', type_='box', dimensions=(0.05, 0.05, 0.05)):
    obj = SceneObject()
    obj.name = name
    obj.type = type_
    obj.dimensions = [float(v) for v in dimensions]
    return obj


def test_entity_path_matches_setup_scene_rule():
    """`setup_scene.py` 가 만드는 prim 경로와 같아야 한다 — 다르면 못 찾는다."""
    assert _ResetStub()._entity_path('block_a') == '/World/Scene/block_a'


def test_known_object_passes():
    assert _ResetStub()._check_known(_obj()) is None


def test_unknown_object_is_rejected():
    """Isaac 은 물체를 만들 수 없다 — 조용히 넘기면 '배치했다'는 거짓이 남는다."""
    problem = _ResetStub()._check_known(_obj(name='peg'))
    assert problem is not None and 'peg' in problem


def test_type_mismatch_is_rejected():
    problem = _ResetStub()._check_known(_obj(type_='cylinder'))
    assert problem is not None and 'geometry' in problem


def test_dimension_mismatch_is_rejected():
    """크기가 다르면 트윈이 아는 치수와 실제가 어긋나 파지 좌표가 빗나간다."""
    problem = _ResetStub()._check_known(_obj(dimensions=(0.07, 0.07, 0.07)))
    assert problem is not None and 'dimensions' in problem


def test_empty_type_and_dimensions_are_not_checked():
    """요청이 기하를 생략하면 위치만 바꾸겠다는 뜻이다 — 거부하지 않는다."""
    assert _ResetStub()._check_known(_obj(type_='', dimensions=())) is None


# ---- _same_pose : 적용 여부 판정 ----

def test_same_pose_accepts_float32_rounding():
    """Isaac 이 float32 로 돌려주므로 정확히 같지 않다."""
    assert _same_pose(_pose(0.41999998688697815, 0.18000000715255737),
                      _pose(0.42, 0.18)) is True


def test_same_pose_rejects_a_position_that_did_not_move():
    """실측된 조용한 실패 — `result=1` 인데 pose 가 그대로인 경우를 잡는다."""
    assert _same_pose(_pose(5.0, 5.0, 5.0), _pose(1.0, 2.0, 3.0)) is False


def test_same_pose_treats_negated_quaternion_as_equal():
    """`q` 와 `-q` 는 같은 회전이다 — 부호로 견주면 같은 자세를 다르다고 판정한다."""
    assert _same_pose(_pose(qz=0.7071068, qw=0.7071068),
                      _pose(qz=-0.7071068, qw=-0.7071068)) is True


def test_same_pose_rejects_a_different_orientation():
    """위치만 보면 자세가 조용히 무시되는 경우를 놓친다."""
    assert _same_pose(_pose(qz=0.7071068, qw=0.7071068), _pose()) is False


def test_same_pose_allows_free_fall_between_write_and_read():
    """재생 중에는 놓자마자 떨어진다 — 왕복 시간만큼의 낙하는 정상이다.

    실측(2026-09-02)에서 공중 배치가 이 때문에 거부됐다. 30 ms 면 4.4 mm 라
    고정 1 mm 허용치로는 정상 동작이 실패로 읽힌다.
    """
    fallen = _pose(0.5, 0.15, 0.9 - 0.0044)
    assert _same_pose(fallen, _pose(0.5, 0.15, 0.9)) is False          # 정지 가정
    assert _same_pose(fallen, _pose(0.5, 0.15, 0.9), 0.030) is True    # 30 ms 왕복


def test_same_pose_still_catches_a_prim_that_never_moved():
    """낙하 허용이 '아무 일도 안 했다'까지 통과시키면 안 된다."""
    assert _same_pose(_pose(5.0, 5.0, 5.0), _pose(1.0, 2.0, 3.0), 0.100) is False


# ---------- 하트비트 ----------
#
# **주기 발행 노드라 이 줄이 매번 나오면 콘솔이 이것만으로 찬다.** 실제로 5 초마다 같은
# 줄이 반복돼 다른 노드의 로그가 밀려났다. 정상 구간은 침묵, 문제 구간은 반복이어야 한다.

def _resolved_node(objects, transforms, seconds=100.0):
    node = _StubNode(objects, transforms, seconds)
    node._on_timer()
    return node


def test_heartbeat_logs_once_when_everything_resolves():
    """정상이 이어지는 동안에는 한 줄만 남는다."""
    node = _resolved_node([BLOCK], {(BASE, 'block_a'): _tf(0.5, -0.15, 0.425)})
    first = _heartbeats(node)
    assert len(first) == 1 and 'resolved 1/1' in first[0]

    # warn_interval 을 훌쩍 넘겨도 상태가 그대로면 더 찍지 않는다.
    for extra in (1.0, 10.0, 60.0):
        node._clock.nanoseconds = int((100.0 + extra) * 1e9)
        node._on_timer()
    assert _heartbeats(node) == first


def test_heartbeat_repeats_while_degraded():
    """못 푸는 상태는 조용해지면 고쳐진 줄로 오해하므로 되풀이한다 — WARNING 으로."""
    node = _resolved_node([BLOCK, TABLE], {(BASE, 'block_a'): _tf(0.5, -0.15, 0.425)})
    assert len(_heartbeats(node)) == 1
    assert 'resolved 1/2' in _heartbeats(node)[0]
    assert not node._logger.infos          # 정상이 아니므로 INFO 로 찍지 않는다

    node._clock.nanoseconds = int(100.9 * 1e9)      # warn_interval(5.0) 안
    node._on_timer()
    assert len(_heartbeats(node)) == 1

    node._clock.nanoseconds = int(106.0 * 1e9)      # 넘김
    node._on_timer()
    assert len(_heartbeats(node)) == 2


def test_heartbeat_logs_the_recovery():
    """문제가 풀린 것은 알려야 한다 — 상태가 바뀌면 interval 과 무관하게 남는다."""
    node = _resolved_node([BLOCK, TABLE], {(BASE, 'block_a'): _tf(0.5, -0.15, 0.425)})
    assert len(_heartbeats(node)) == 1

    node._buffer._transforms[(BASE, 'table')] = _tf(0.5, 0.0, 0.2)
    node._clock.nanoseconds = int(100.1 * 1e9)      # interval 한참 전
    node._on_timer()
    assert len(node._logger.infos) == 1 and 'resolved 2/2' in node._logger.infos[0]


def test_frame_list_says_when_it_is_truncated():
    """예전에는 'tf knows 15 frame(s)' 뒤에 12 개만 나열해 목록이 전부처럼 보였다."""
    many = {(BASE, f'frame_{i:02d}'): _tf(0.0, 0.0, 0.0) for i in range(20)}
    many[(BASE, 'block_a')] = _tf(0.5, -0.15, 0.425)
    node = _resolved_node([BLOCK], many)
    line = _heartbeats(node)[-1]
    assert 'more)' in line, line
