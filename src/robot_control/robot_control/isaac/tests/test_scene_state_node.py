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

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._logger

    _lookup = IsaacSceneStateNode._lookup
    _on_timer = IsaacSceneStateNode._on_timer
    _heartbeat = IsaacSceneStateNode._heartbeat


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
    assert len(node._logger.warnings) == 1

    node._on_timer()          # 같은 시각 — 억제된다
    assert len(node._logger.warnings) == 1

    node._clock.nanoseconds = int(6 * 1e9)
    node._on_timer()          # 간격을 넘겼다 — 다시 나온다
    assert len(node._logger.warnings) == 2


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


# ---------- 고정물 분류 ----------

def test_fixture_is_the_inverse_of_the_json_dynamic_flag():
    """분류의 출처는 `isaac_scene.json` 하나다.

    `dynamic` 은 시뮬레이터 쪽 `setup_scene.py` 가 rigid body 여부로 쓰는 물리
    플래그이며, 그것을 뒤집어 쓴다. `fixture` 키를 JSON 에 따로 두면 같은 사실이
    두 곳에 적혀 어긋난다.
    """
    node = _StubNode(
        [{'name': 'table', 'type': 'box', 'dimensions': [0.6, 1.0, 0.4], 'dynamic': False},
         {'name': 'block_a', 'type': 'box', 'dimensions': [0.05] * 3, 'dynamic': True}],
        {(BASE, 'table'): _tf(0.55, 0.0, 0.2),
         (BASE, 'block_a'): _tf(0.5, -0.15, 0.425)})
    node._on_timer()
    published = {o.name: o.fixture for o in node._publisher.published[-1].objects}
    assert published == {'table': True, 'block_a': False}


def test_missing_dynamic_key_is_treated_as_a_fixture():
    """`dynamic` 이 없으면 static 이라는 뜻이므로 고정물이다 (setup_scene.py 와 동일)."""
    node = _StubNode([{'name': 'wall', 'type': 'box', 'dimensions': [1.0, 0.1, 1.0]}],
                     {(BASE, 'wall'): _tf(0.0, 0.5, 0.5)})
    node._on_timer()
    assert node._publisher.published[-1].objects[0].fixture is True
