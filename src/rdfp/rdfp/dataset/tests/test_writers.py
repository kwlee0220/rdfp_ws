"""writers 단위 테스트.

DB 커넥션이 필요한 INSERT 실행 대신, writer 의 `row_values` 변환 로직과
batch/flush 상호작용을 검증한다.
"""

from __future__ import annotations

from types import SimpleNamespace

from rdfp.dataset.db.writers.joint_state import JointStateWriter
from rdfp.dataset.db.writers.pose_stamped import PoseStampedWriter
from rdfp.dataset.db.writers.twist_stamped import TwistStampedWriter


def _header(sec: int = 100, nanosec: int = 42) -> SimpleNamespace:
    return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec))


class _FakeConn:
    """executemany 호출만 기록하는 더미 커넥션."""

    def __init__(self) -> None:
        self.executed: list[tuple[str, list]] = []

    def cursor(self):
        this = self

        class _Cur:

            def __enter__(self):
                return self

            def __exit__(self, *a):
                return False

            def executemany(self, sql, params):
                this.executed.append((sql, list(params)))

        return _Cur()


def test_pose_stamped_row_values() -> None:
    w = PoseStampedWriter(conn=_FakeConn(), topic_id=42)
    msg = SimpleNamespace(
        header=_header(100, 500),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    row = w.row_values(episode_id=7, msg=msg)
    assert row == (7, 42, 100, 500, [1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])


def test_twist_stamped_row_values() -> None:
    w = TwistStampedWriter(conn=_FakeConn(), topic_id=9)
    msg = SimpleNamespace(
        header=_header(10, 0),
        twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.1, y=0.2, z=0.3),
            angular=SimpleNamespace(x=-0.1, y=-0.2, z=-0.3),
        ),
    )
    row = w.row_values(3, msg)
    assert row == (3, 9, 10, 0, [0.1, 0.2, 0.3, -0.1, -0.2, -0.3])


def test_joint_state_row_values_handles_empty_arrays() -> None:
    w = JointStateWriter(conn=_FakeConn(), topic_id=2)
    msg = SimpleNamespace(
        header=_header(1, 2),
        position=[0.5, 1.5],
        velocity=[],
        effort=[],
    )
    row = w.row_values(1, msg)
    assert row == (1, 2, 1, 2, [0.5, 1.5], [], [])


def test_batch_flush_at_threshold() -> None:
    conn = _FakeConn()
    w = PoseStampedWriter(conn=conn, batch_size=2)
    msg = SimpleNamespace(
        header=_header(),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=0, y=0, z=0),
            orientation=SimpleNamespace(x=0, y=0, z=0, w=1),
        ),
    )
    w.append(1, msg)
    assert conn.executed == []   # 아직 임계 미달
    w.append(1, msg)
    assert len(conn.executed) == 1   # 임계 도달 → 자동 flush
    sql, params = conn.executed[0]
    assert 'INSERT INTO pose_stampeds' in sql
    assert len(params) == 2


def test_consume_inserted_count_resets() -> None:
    conn = _FakeConn()
    w = PoseStampedWriter(conn=conn, batch_size=10)
    msg = SimpleNamespace(
        header=_header(),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=0, y=0, z=0),
            orientation=SimpleNamespace(x=0, y=0, z=0, w=1),
        ),
    )
    w.append(1, msg)
    w.append(1, msg)
    w.flush()
    assert w.consume_inserted_count() == 2
    assert w.consume_inserted_count() == 0


def test_drop_pending_clears_buffer() -> None:
    conn = _FakeConn()
    w = PoseStampedWriter(conn=conn, batch_size=100)
    msg = SimpleNamespace(
        header=_header(),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=0, y=0, z=0),
            orientation=SimpleNamespace(x=0, y=0, z=0, w=1),
        ),
    )
    w.append(1, msg)
    w.drop_pending()
    assert w._buffer == []
    w.flush()
    assert conn.executed == []


# --------------------------------------------------------------------------
# Table override (동일 writer 클래스를 다른 테이블로 라우팅)
# --------------------------------------------------------------------------

def test_writer_table_override_is_instance_scoped() -> None:
    """`table='...'` 인자는 인스턴스 단위로만 적용되고 클래스 기본값은 보존한다."""
    conn = _FakeConn()
    w = PoseStampedWriter(conn=conn, batch_size=1, table='pose_stampeds_alt')
    assert w.table == 'pose_stampeds_alt'
    # 클래스 기본값은 여전히 pose_stampeds (shared mutable 방지).
    assert PoseStampedWriter.table == 'pose_stampeds'
    msg = SimpleNamespace(
        header=_header(),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
            orientation=SimpleNamespace(x=0, y=0, z=0, w=1),
        ),
    )
    w.append(7, msg)
    assert len(conn.executed) == 1
    sql, _ = conn.executed[0]
    assert 'INSERT INTO pose_stampeds_alt' in sql


# --------------------------------------------------------------------------
# Gripper writers
# --------------------------------------------------------------------------

def test_gripper_command_row_values() -> None:
    from rdfp.dataset.db.writers.gripper_command import GripperCommandWriter

    conn = _FakeConn()
    w = GripperCommandWriter(conn=conn, batch_size=1, topic_id=11)
    msg = SimpleNamespace(header=_header(100, 500), position=0.04, max_effort=0.0,
                          label='open')
    w.append(3, msg)
    assert len(conn.executed) == 1
    sql, params = conn.executed[0]
    assert 'INSERT INTO gripper_cmds' in sql
    # 심볼이 아니라 숫자가 남아야 한다 — 데이터셋이 자기 완결적이어야 하기 때문이다.
    assert params[0] == (3, 11, 100, 500, 0.04, 0.0, 'open')


def test_gripper_command_label_is_optional() -> None:
    """`label` 은 제어에 쓰이지 않는 선택 필드다 — 없어도 적재가 멈추지 않는다."""
    from rdfp.dataset.db.writers.gripper_command import GripperCommandWriter

    conn = _FakeConn()
    w = GripperCommandWriter(conn=conn, batch_size=1, topic_id=11)
    w.append(3, SimpleNamespace(header=_header(100, 500), position=0.0, max_effort=30.0))

    assert conn.executed[0][1][0] == (3, 11, 100, 500, 0.0, 30.0, '')


def test_gripper_state_row_values() -> None:
    from rdfp.dataset.db.writers.gripper_action_state import GripperActionStateWriter

    conn = _FakeConn()
    w = GripperActionStateWriter(conn=conn, batch_size=1, topic_id=13)
    msg = SimpleNamespace(
        header=_header(10, 20),
        position=0.04, effort=12.5, stalled=False, reached_goal=True, status=4,
    )
    w.append(5, msg)
    assert len(conn.executed) == 1
    sql, params = conn.executed[0]
    assert 'INSERT INTO gripper_action_states' in sql
    assert params[0] == (5, 13, 10, 20, 0.04, 12.5, False, True, 4)


def test_gripper_state_row_values_without_status() -> None:
    """status 도입 이전에 녹화된 메시지는 UNKNOWN(0) 으로 적재한다."""
    from rdfp.dataset.db.writers.gripper_action_state import GripperActionStateWriter

    conn = _FakeConn()
    w = GripperActionStateWriter(conn=conn, batch_size=1, topic_id=13)
    msg = SimpleNamespace(
        header=_header(10, 20),
        position=0.04, effort=12.5, stalled=False, reached_goal=True,
    )
    w.append(5, msg)
    _, params = conn.executed[0]
    assert params[0] == (5, 13, 10, 20, 0.04, 12.5, False, True, 0)


# --------------------------------------------------------------------------
# Registry: resolve_message_type
# --------------------------------------------------------------------------

def test_resolve_message_type_maps_known_type() -> None:
    """등록된 메시지 타입은 해당 `TypeBinding` 으로 매핑된다."""
    from rdfp.dataset.db.registry import resolve_message_type

    binding = resolve_message_type('geometry_msgs/msg/PoseStamped')
    assert binding is not None
    assert binding.table == 'pose_stampeds'


def test_resolve_message_type_unknown_type_returns_none() -> None:
    from rdfp.dataset.db.registry import resolve_message_type

    assert resolve_message_type('foo/msg/Unknown') is None


def test_registry_contains_new_gripper_types() -> None:
    from rdfp.dataset.db.registry import MESSAGE_TYPE_REGISTRY

    assert MESSAGE_TYPE_REGISTRY['rdfp_msgs/msg/GripperCommand'].table == 'gripper_cmds'
    assert (MESSAGE_TYPE_REGISTRY['rdfp_msgs/msg/GripperActionState'].table
            == 'gripper_action_states')


def test_registry_table_matches_writer_class_default() -> None:
    """`binding.table` 이 writer 클래스의 기본 테이블과 일치한다.

    write 경로는 `binding.table` 을 writer 인스턴스의 `table` 로 주입하고,
    read 경로도 `binding.table` 을 직접 사용한다. writer 클래스의 기본값이
    `binding.table` 과 어긋나면 "인스턴스 오버라이드 없는 경우" 에만 조용히
    서로 다른 테이블을 가리키게 되므로 불변식으로 고정한다.
    """
    from rdfp.dataset.db.registry import MESSAGE_TYPE_REGISTRY

    for msg_type, binding in MESSAGE_TYPE_REGISTRY.items():
        assert binding.table == binding.writer_cls.table, (
            f'{msg_type}: binding.table={binding.table!r} != '
            f'writer_cls.table={binding.writer_cls.table!r}'
        )


# --------------------------------------------------------------------------
# TargetJointStatesWriter
# --------------------------------------------------------------------------

def _make_target_joint_states_msg(
    stamp_sec: int, stamp_nsec: int,
    tfs_sec: int, tfs_nsec: int,
    positions: list[float], velocities: list[float],
    accelerations: list[float], effort: list[float],
) -> SimpleNamespace:
    return SimpleNamespace(
        header=_header(stamp_sec, stamp_nsec),
        point=SimpleNamespace(
            positions=positions, velocities=velocities,
            accelerations=accelerations, effort=effort,
            time_from_start=SimpleNamespace(sec=tfs_sec, nanosec=tfs_nsec),
        ),
    )


def test_target_joint_states_row_values_flattens_point_and_tfs() -> None:
    """point 의 4개 배열 + time_from_start (sec/nanosec) 가 평탄화 적재된다."""
    from rdfp.dataset.db.writers.target_joint_states import TargetJointStatesWriter

    conn = _FakeConn()
    w = TargetJointStatesWriter(conn=conn, batch_size=1, topic_id=17)
    msg = _make_target_joint_states_msg(
        stamp_sec=200, stamp_nsec=300,
        tfs_sec=1, tfs_nsec=500_000_000,
        positions=[0.1, 0.2], velocities=[1.0, 1.0],
        accelerations=[0.0, 0.0], effort=[0.5, 0.6],
    )
    w.append(11, msg)

    assert len(conn.executed) == 1
    sql, params = conn.executed[0]
    assert 'INSERT INTO target_joint_states' in sql
    row = params[0]
    # (episode_id, topic_id, stamp_sec, stamp_nanosec,
    #  positions, velocities, accelerations, effort, tfs_sec, tfs_nanosec)
    assert row == (
        11, 17, 200, 300,
        [0.1, 0.2], [1.0, 1.0], [0.0, 0.0], [0.5, 0.6],
        1, 500_000_000,
    )


def test_target_joint_states_row_values_handles_empty_arrays() -> None:
    """배열 필드가 빈 리스트여도 그대로 빈 배열로 적재된다."""
    from rdfp.dataset.db.writers.target_joint_states import TargetJointStatesWriter

    conn = _FakeConn()
    w = TargetJointStatesWriter(conn=conn, batch_size=1, topic_id=19)
    msg = _make_target_joint_states_msg(
        stamp_sec=10, stamp_nsec=0,
        tfs_sec=0, tfs_nsec=0,
        positions=[], velocities=[], accelerations=[], effort=[],
    )
    w.append(7, msg)

    row = conn.executed[0][1][0]
    assert row == (7, 19, 10, 0, [], [], [], [], 0, 0)


def test_registry_target_joint_states_maps_to_target_joint_states() -> None:
    """TargetJointStates 의 기본 매핑이 target_joint_states + Writer 로 연결된다."""
    from rdfp.dataset.db.registry import MESSAGE_TYPE_REGISTRY
    from rdfp.dataset.db.writers.target_joint_states import TargetJointStatesWriter

    binding = MESSAGE_TYPE_REGISTRY['rdfp_msgs/msg/TargetJointStates']
    assert binding.table == 'target_joint_states'
    assert binding.writer_cls is TargetJointStatesWriter


# ----- SessionWriter (sessions 테이블) -----

class _FakeEpisodeConn:
    """execute 호출을 기록하고 고정 id 를 돌려주는 더미 커넥션."""

    def __init__(self, returned_id: int = 7) -> None:
        self.executed: list[tuple[str, tuple]] = []
        self._id = returned_id

    def cursor(self):
        this = self

        class _Cur:

            def __enter__(self):
                return self

            def __exit__(self, *a):
                return False

            def execute(self, sql, params=None):
                this.executed.append((sql, params))

            def fetchone(self):
                return (this._id,)

        return _Cur()


def _insert_params(conn: _FakeEpisodeConn) -> tuple:
    return conn.executed[0][1]


def test_session_writer_inserts_success_and_metadata() -> None:
    from psycopg.types.json import Jsonb

    from rdfp.dataset.db.writers.session_command import SessionWriter

    conn = _FakeEpisodeConn(returned_id=11)
    episode_id = SessionWriter(conn).insert_episode(
        1_500_000_000_123_000_000, 1_500_000_002_456_000_000, 'pick_red_cube',
        success=False, metadata={'seed': 42, 'scene': 'one_cube'}
    )

    assert episode_id == 11
    start_sec, start_ns, stop_sec, stop_ns, label, success, metadata = _insert_params(conn)
    assert (start_sec, start_ns) == (1_500_000_000, 123_000_000)
    assert (stop_sec, stop_ns) == (1_500_000_002, 456_000_000)
    assert label == 'pick_red_cube'
    assert success is False
    # dict 를 그대로 넘기면 psycopg 가 타입을 정하지 못하므로 jsonb 로 어댑트한다.
    assert isinstance(metadata, Jsonb)


def test_session_writer_defaults_leave_success_and_metadata_null() -> None:
    """판정 주체가 없는 수집(텔레오퍼레이션 등)에서는 둘 다 NULL 이어야 한다."""
    from rdfp.dataset.db.writers.session_command import SessionWriter

    conn = _FakeEpisodeConn()
    SessionWriter(conn).insert_episode(1_000_000_000, 2_000_000_000, None)

    *_, label, success, metadata = _insert_params(conn)
    assert label is None
    assert success is None
    assert metadata is None


def test_session_writer_keeps_empty_metadata_distinct_from_none() -> None:
    """빈 dict 는 '정보 없음'(NULL) 이 아니라 빈 jsonb 로 들어간다."""
    from psycopg.types.json import Jsonb

    from rdfp.dataset.db.writers.session_command import SessionWriter

    conn = _FakeEpisodeConn()
    SessionWriter(conn).insert_episode(1_000_000_000, 2_000_000_000, None, metadata={})

    assert isinstance(_insert_params(conn)[-1], Jsonb)


def test_schema_check_requires_the_new_session_columns() -> None:
    from rdfp.dataset.db.schema_check import REQUIRED_COLUMNS

    assert {'success', 'metadata'} <= set(REQUIRED_COLUMNS['sessions'])


# --------------------------------------------------------------------------
# scene_objects — /scene/objects (ground truth, 라벨·큐레이션용)
# --------------------------------------------------------------------------

def _scene_object(name: str, type_: str, dimensions: list, pos: tuple,
                  ori: tuple) -> SimpleNamespace:
    return SimpleNamespace(
        name=name, type=type_, dimensions=dimensions,
        pose=SimpleNamespace(
            position=SimpleNamespace(x=pos[0], y=pos[1], z=pos[2]),
            orientation=SimpleNamespace(x=ori[0], y=ori[1], z=ori[2], w=ori[3]),
        ),
    )


def _scene_header(sec: int = 100, nanosec: int = 42,
                  frame_id: str = 'panda_link0') -> SimpleNamespace:
    return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
                           frame_id=frame_id)


def test_scene_objects_row_values() -> None:
    from rdfp.dataset.db.writers.scene_objects import SceneObjectsWriter

    w = SceneObjectsWriter(conn=_FakeConn(), topic_id=11)
    msg = SimpleNamespace(
        header=_scene_header(30, 7),
        objects=[
            _scene_object('cube_0', 'box', [0.05, 0.05, 0.05],
                          (0.4, 0.1, 0.025), (0.0, 0.0, 0.0, 1.0)),
            _scene_object('can_0', 'cylinder', [0.12, 0.03],
                          (0.5, -0.2, 0.06), (0.0, 0.0, 0.7071, 0.7071)),
        ],
    )
    episode_id, topic_id, sec, nsec, frame_id, objects = w.row_values(5, msg)
    assert (episode_id, topic_id, sec, nsec, frame_id) == (5, 11, 30, 7, 'panda_link0')
    # jsonb 어댑터로 감싸 넘긴다 — 내용은 .obj 로 확인한다.
    assert objects.obj == [
        {'name': 'cube_0', 'type': 'box', 'dimensions': [0.05, 0.05, 0.05],
         'position': [0.4, 0.1, 0.025], 'orientation': [0.0, 0.0, 0.0, 1.0]},
        {'name': 'can_0', 'type': 'cylinder', 'dimensions': [0.12, 0.03],
         'position': [0.5, -0.2, 0.06], 'orientation': [0.0, 0.0, 0.7071, 0.7071]},
    ]


def test_scene_objects_row_values_keeps_empty_scene() -> None:
    """물체가 없는 것도 'scene 이 비었다'는 유효한 상태이므로 빈 배열로 적재된다."""
    from rdfp.dataset.db.writers.scene_objects import SceneObjectsWriter

    w = SceneObjectsWriter(conn=_FakeConn(), topic_id=11)
    msg = SimpleNamespace(header=_scene_header(), objects=[])
    assert w.row_values(1, msg)[-1].obj == []


def test_scene_objects_orientation_stays_xyzw() -> None:
    """orientation 순서가 xyzw 로 유지된다.

    wxyz 로 뒤집혀도 4개 float 에 unit norm 이라 타입 검사도 정규화 검사도
    통과하고 '그럴듯하게 틀린 자세'가 되므로, 비대칭 값으로 고정한다.
    """
    from rdfp.dataset.db.writers.scene_objects import SceneObjectsWriter

    w = SceneObjectsWriter(conn=_FakeConn(), topic_id=1)
    msg = SimpleNamespace(
        header=_scene_header(),
        objects=[_scene_object('o', 'box', [], (0.0, 0.0, 0.0), (0.1, 0.2, 0.3, 0.4))],
    )
    assert w.row_values(1, msg)[-1].obj[0]['orientation'] == [0.1, 0.2, 0.3, 0.4]


def test_registry_contains_scene_objects_type() -> None:
    from rdfp.dataset.db.registry import MESSAGE_TYPE_REGISTRY

    assert MESSAGE_TYPE_REGISTRY['rdfp_msgs/msg/SceneObjects'].table == 'scene_objects'


def test_schema_check_requires_scene_objects_columns() -> None:
    from rdfp.dataset.db.schema_check import REQUIRED_COLUMNS

    assert {'frame_id', 'objects'} <= set(REQUIRED_COLUMNS['scene_objects'])
