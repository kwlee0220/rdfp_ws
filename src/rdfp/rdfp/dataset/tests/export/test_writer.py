"""E7 저장 형식 writer 시험.

**실 PostgreSQL 을 쓴다** — 스키마의 GENERATED 컬럼과 CHECK 제약이 실제로 도는지가
이 계층의 위험이고, 스텁으로는 그것이 안 드러난다. `psql` 로 scratch DB 를 만들 수
없으면 모듈 전체를 건너뛴다.
"""

from __future__ import annotations

import itertools
import json
import os
import subprocess
import uuid

import numpy as np
import pytest

psycopg = pytest.importorskip('psycopg')

from rdfp.dataset.export import (  # noqa: E402
    ExportSpec, GRIPPER_SYMBOLS, LABEL_COMMAND, LABEL_PLAN, load_episode)
from rdfp.dataset.export.writer import (  # noqa: E402
    ExportError, _relative_mp4, export_dataset, export_episode)

SCHEMA = os.path.join(os.path.dirname(__file__), '..', '..', 'sql', 'schema.sql')
NS = 1_000_000_000
ARM = ['panda_joint1', 'panda_joint2']
#: 에피소드는 `(start_sec, start_nanosec)` 로 유일하다 — 시험마다 시각을 벌린다.
_EPOCH = itertools.count(1000, 1000)


def _psql(db: str, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run(['psql', '-d', db, '-v', 'ON_ERROR_STOP=1', '-q', *args],
                          capture_output=True, text=True)


@pytest.fixture(scope='module')
def dsn():
    """scratch DB 하나를 만들어 모듈이 끝나면 지운다."""
    if subprocess.run(['which', 'psql'], capture_output=True).returncode != 0:
        pytest.skip('psql not available')
    name = f'rdfp_e7_{uuid.uuid4().hex[:10]}'
    if subprocess.run(['createdb', name], capture_output=True).returncode != 0:
        pytest.skip('cannot create a scratch database (peer auth unavailable?)')
    try:
        out = _psql(name, '-f', os.path.abspath(SCHEMA))
        if out.returncode != 0:
            pytest.skip(f'schema load failed: {out.stderr[:200]}')
        yield f'postgresql:///{name}'
    finally:
        subprocess.run(['dropdb', '--if-exists', name], capture_output=True)


@pytest.fixture()
def conn(dsn):
    with psycopg.connect(dsn) as c:
        yield c
        c.rollback()


# ----- 합성 에피소드 -------------------------------------------------------

def _topic(conn, name: str, type_name: str = 'x/Msg') -> int:
    with conn.cursor() as cur:
        cur.execute('INSERT INTO topics (topic_name, topic_type) VALUES (%s, %s) '
                    'ON CONFLICT (topic_name) DO UPDATE SET topic_type = EXCLUDED.topic_type '
                    'RETURNING id', (name, type_name))
        return cur.fetchone()[0]


def _episode(conn, *, start: float, stop: float, metadata=None) -> int:
    with conn.cursor() as cur:
        cur.execute(
            'INSERT INTO sessions (start_sec, start_nanosec, stop_sec, stop_nanosec, '
            'task_label, metadata) VALUES (%s, %s, %s, %s, %s, %s) RETURNING id',
            (int(start), int(start % 1 * NS), int(stop), int(stop % 1 * NS),
             'pick', json.dumps(metadata or {})))
        return cur.fetchone()[0]


def _frames(conn, ep: int, *, t0: float, n: int, dt: float, topic='/camera/image_raw'):
    tid = _topic(conn, topic, 'sensor_msgs/msg/Image')
    with conn.cursor() as cur:
        cur.execute(
            'INSERT INTO image_streams (episode_id, topic_id, mp4_path, codec, '
            'pixel_format, container_fps, width, height, frame_count) '
            'VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)',
            (ep, tid, '/data/rdfp/recordings/cam_100_video.mp4', 'h264', 'yuv420p',
             int(round(1 / dt)), 640, 480, n))
        for i in range(n):
            t = t0 + i * dt
            cur.execute('INSERT INTO image_frames (episode_id, topic_id, frame_index, '
                        'stamp_sec, stamp_nanosec) VALUES (%s, %s, %s, %s, %s)',
                        (ep, tid, i, int(t), int(round(t % 1 * NS))))


def _joint_states(conn, ep: int, *, t0: float, n: int, dt: float,
                  topic='/joint_states', base=0.0):
    tid = _topic(conn, topic, 'sensor_msgs/msg/JointState')
    with conn.cursor() as cur:
        for i in range(n):
            t = t0 + i * dt
            cur.execute('INSERT INTO joint_states (episode_id, topic_id, stamp_sec, '
                        'stamp_nanosec, joint_names, position, velocity, effort) '
                        'VALUES (%s, %s, %s, %s, %s, %s, %s, %s)',
                        (ep, tid, int(t), int(round(t % 1 * NS)), ARM,
                         [base + i * 0.01, base + i * 0.02], [0.0, 0.0], [0.0, 0.0]))


def _trajectory(conn, ep: int, *, t0: float, op_id: str, points, duration: float):
    tid = _topic(conn, '/motion/trajectory', 'rdfp_msgs/msg/MotionTrajectory')
    with conn.cursor() as cur:
        cur.execute('INSERT INTO motion_trajectories (episode_id, topic_id, stamp_sec, '
                    'stamp_nanosec, operation_id, velocity_scaling, time_scaling, '
                    'joint_names, points) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)',
                    (ep, tid, int(t0), int(round(t0 % 1 * NS)), op_id, 1.0, 1.0, ARM,
                     json.dumps(points)))


def _event(conn, ep: int, *, t: float, op_id: str, result: str, truncated: float):
    tid = _topic(conn, '/motion/event', 'rdfp_msgs/msg/MotionEvent')
    with conn.cursor() as cur:
        cur.execute('INSERT INTO motion_events (episode_id, topic_id, stamp_sec, '
                    'stamp_nanosec, operation_id, operation, result, truncated_at_sec, '
                    'target_kind) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)',
                    (ep, tid, int(t), int(round(t % 1 * NS)), op_id, 'joints', result,
                     truncated, 'joints'))


def _gripper(conn, ep: int, *, t0: float, n: int, dt: float):
    st = _topic(conn, '/gripper_states', 'rdfp_msgs/msg/GripperState')
    cm = _topic(conn, '/gripper_cmds', 'rdfp_msgs/msg/GripperCommand')
    with conn.cursor() as cur:
        for i in range(n):
            t = t0 + i * dt
            cur.execute('INSERT INTO gripper_states (episode_id, topic_id, stamp_sec, '
                        'stamp_nanosec, width, at_goal, stalled, goal) '
                        'VALUES (%s, %s, %s, %s, %s, %s, %s, %s)',
                        (ep, st, int(t), int(round(t % 1 * NS)), 0.04, True, False, 'open'))
        # ⚠️ 초와 나노초를 나눠 넣는다 — `int(t0 + 0.5)` 로 적으면 소수부가 잘려
        # 명령이 t0 로 가고, "첫 명령 이전 구간" 시험이 통째로 무의미해진다.
        cur.execute('INSERT INTO gripper_cmds (episode_id, topic_id, stamp_sec, '
                    'stamp_nanosec, goal) VALUES (%s, %s, %s, %s, %s)',
                    (ep, cm, int(t0), int(0.5 * NS), 'close'))


def _make_episode(conn, *, truncate=False, with_event=True, n_frames=10, dt=0.1):
    """카메라 n 프레임 + 관절 + 그리퍼 + 연산 하나짜리 에피소드.

    Returns:
        `(episode_id, t0)`. `t0` 는 뒤에 명령 계열을 더 넣을 때 쓴다.
    """
    t0 = float(next(_EPOCH))
    ep = _episode(conn, start=t0, stop=t0 + n_frames * dt, metadata={'scene': 'r2'})
    _frames(conn, ep, t0=t0, n=n_frames, dt=dt)
    _joint_states(conn, ep, t0=t0, n=n_frames * 5, dt=dt / 5)
    _gripper(conn, ep, t0=t0, n=n_frames, dt=dt)
    # 2 초 궤적: 0 → 2 → 4 (관절1), 0 → 4 → 8 (관절2)
    points = [{'t': 0.0, 'positions': [0.0, 0.0], 'velocities': [0.0, 0.0]},
              {'t': 1.0, 'positions': [2.0, 4.0], 'velocities': [0.0, 0.0]},
              {'t': 2.0, 'positions': [4.0, 8.0], 'velocities': [0.0, 0.0]}]
    _trajectory(conn, ep, t0=t0, op_id='op-1', points=points, duration=2.0)
    if with_event:
        _event(conn, ep, t=t0 + 2.0, op_id='op-1',
               result='aborted' if truncate else 'succeeded',
               truncated=1.0 if truncate else 2.0)
    conn.commit()
    return ep, t0


def _spec(**kw) -> ExportSpec:
    base = dict(label=LABEL_PLAN, delta_sec=0.1, step_sec=0.1, chunk_len=4,
                arm_joints=tuple(ARM), gripper_symbols=GRIPPER_SYMBOLS,
                camera_topic='/camera/image_raw', camera_hz=10.0)
    base.update(kw)
    return ExportSpec(**base)


# ----- 모양 ---------------------------------------------------------------

def test_npz_has_the_shapes_the_design_declares(conn, tmp_path):
    ep, t0 = _make_episode(conn)
    line = export_episode(conn, ep, out_dir=str(tmp_path), spec=_spec())
    arrays = load_episode(str(tmp_path / 'episodes' / f'{ep}.npz'))
    n, k = len(ARM), 4
    assert arrays.frame_index.shape == (10,)
    assert arrays.t.shape == (10,)
    assert arrays.qpos.shape == (10, n + 1)
    assert arrays.action_arm.shape == (10, k, n)
    assert arrays.action_gripper.shape == (10, k, 3)
    assert arrays.is_pad.shape == (10, k)
    assert line.samples == 10


def test_dtypes_follow_the_design(conn, tmp_path):
    """`t` 만 float64 다 — 초 단위 절대 시각이라 float32 로는 밀리초를 잃는다."""
    ep, t0 = _make_episode(conn)
    export_episode(conn, ep, out_dir=str(tmp_path), spec=_spec())
    a = load_episode(str(tmp_path / 'episodes' / f'{ep}.npz'))
    assert a.frame_index.dtype == np.int32
    assert a.t.dtype == np.float64
    assert a.qpos.dtype == np.float32
    assert a.action_arm.dtype == np.float32
    assert a.is_pad.dtype == bool


def test_float32_would_have_lost_milliseconds(conn, tmp_path):
    """`t` 를 float32 로 두면 안 되는 이유를 수치로 박아 둔다."""
    ep, t0 = _make_episode(conn)
    export_episode(conn, ep, out_dir=str(tmp_path), spec=_spec())
    a = load_episode(str(tmp_path / 'episodes' / f'{ep}.npz'))
    lost = np.abs(a.t - a.t.astype(np.float32).astype(np.float64)).max()
    assert lost > 1e-5, '이 시각 범위에서는 float32 손실이 밀리초급이어야 한다'


def test_frame_index_matches_the_mp4_frames(conn, tmp_path):
    ep, t0 = _make_episode(conn)
    export_episode(conn, ep, out_dir=str(tmp_path), spec=_spec())
    a = load_episode(str(tmp_path / 'episodes' / f'{ep}.npz'))
    assert list(a.frame_index) == list(range(10))


# ----- 레이블이 제대로 옮겨졌는가 -------------------------------------------

def test_action_matches_the_label_function(conn, tmp_path):
    """저장이 계산을 그대로 옮겼는가 (설계 §7)."""
    from rdfp.dataset.export import PlanLabel, load_frame_clock, load_operations
    ep, t0 = _make_episode(conn)
    export_episode(conn, ep, out_dir=str(tmp_path), spec=_spec())
    a = load_episode(str(tmp_path / 'episodes' / f'{ep}.npz'))

    clock = load_frame_clock(conn, ep, topic='/camera/image_raw')
    ops, _ = load_operations(conn, ep)
    label = PlanLabel(ops, joint_names=ARM, initial_positions=(0.0, 0.0),
                      episode_end=clock.episode_end)
    for i, frame in enumerate(clock.frames):
        values, is_pad = label.chunk(frame.t, k=4, step=0.1, delta=0.1)
        assert np.allclose(a.action_arm[i], np.array(values, dtype=np.float32))
        assert list(a.is_pad[i]) == list(is_pad)


def test_truncated_operation_holds_the_stop_value(conn, tmp_path):
    """**R1/R2 가 존재하는 이유가 여기서 보인다.**

    2 초 궤적을 1 초에 중단하면 레이블은 **멈춘 자리(2.0)** 를 유지해야 하고
    궤적 끝점(4.0)으로 가면 안 된다 — 팔이 가지 않은 자리이기 때문이다.

    **에피소드가 궤적보다 길어야 뜻이 있다** — 짧으면 정상 종료라도 끝점에 닿지
    못해 두 경우가 구분되지 않는다(2.5 s 로 둔 이유다).
    """
    ep, t0 = _make_episode(conn, truncate=True, n_frames=25)
    export_episode(conn, ep, out_dir=str(tmp_path), spec=_spec())
    a = load_episode(str(tmp_path / 'episodes' / f'{ep}.npz'))
    tail = a.action_arm[-1, -1, 0]
    assert tail == pytest.approx(2.0, abs=1e-5)
    assert tail != pytest.approx(4.0, abs=0.5)


def test_untruncated_operation_reaches_the_endpoint(conn, tmp_path):
    """대조군 — 정상 종료면 끝점(4.0)까지 간다."""
    ep, t0 = _make_episode(conn, truncate=False, n_frames=25)
    export_episode(conn, ep, out_dir=str(tmp_path), spec=_spec())
    a = load_episode(str(tmp_path / 'episodes' / f'{ep}.npz'))
    assert a.action_arm[-1, -1, 0] == pytest.approx(4.0, abs=1e-5)


def test_gripper_action_is_one_hot_on_the_same_grid(conn, tmp_path):
    ep, t0 = _make_episode(conn)
    export_episode(conn, ep, out_dir=str(tmp_path), spec=_spec())
    a = load_episode(str(tmp_path / 'episodes' / f'{ep}.npz'))
    assert np.all(a.action_gripper.sum(axis=2) == 1.0)
    # 첫 명령은 t0+0.5 의 `close` 이고 그 전은 `open`(GRIPPER_PRIOR)이다.
    assert a.action_gripper[0, 0, GRIPPER_SYMBOLS.index('open')] == 1.0
    assert a.action_gripper[-1, -1, GRIPPER_SYMBOLS.index('close')] == 1.0


# ----- 통계와 metadata -----------------------------------------------------

def test_metadata_and_episodes_jsonl_are_written(conn, tmp_path):
    ep, t0 = _make_episode(conn)
    meta, lines = export_dataset(conn, [ep], out_dir=str(tmp_path),
                                 arm_joints=ARM, camera_topic='/camera/image_raw',
                                 chunk_len=4)
    written = json.loads((tmp_path / 'metadata.json').read_text(encoding='utf-8'))
    assert written['spec']['label'] == LABEL_PLAN
    assert written['spec']['arm_joints'] == ARM
    assert written['spec']['chunk_len'] == 4
    assert written['samples'] == 10
    rows = [json.loads(x) for x in
            (tmp_path / 'episodes.jsonl').read_text(encoding='utf-8').splitlines()]
    assert len(rows) == 1 and rows[0]['episode_id'] == ep
    assert rows[0]['samples'] == 10
    assert meta.spec.camera_hz == pytest.approx(10.0, rel=0.02)


def test_session_metadata_passes_through_whole(conn, tmp_path):
    """키를 골라 옮기지 않는다 — R4 가 자라도 여기를 안 고치려는 것이다."""
    ep, t0 = _make_episode(conn)
    _, lines = export_dataset(conn, [ep], out_dir=str(tmp_path), arm_joints=ARM,
                              camera_topic='/camera/image_raw', chunk_len=4)
    assert lines[0].metadata == {'scene': 'r2'}


def test_stats_are_raw_values_not_normalised(conn, tmp_path):
    """저장은 원값, 정규화는 학습 때 (설계 §6.4)."""
    ep, t0 = _make_episode(conn)
    meta, _ = export_dataset(conn, [ep], out_dir=str(tmp_path), arm_joints=ARM,
                             camera_topic='/camera/image_raw', chunk_len=4)
    a = load_episode(str(tmp_path / 'episodes' / f'{ep}.npz'))
    assert a.qpos[:, 0].mean() == pytest.approx(np.mean(a.qpos[:, 0]))
    # 통계가 원값 분포와 같다 — 저장본에 정규화를 걸지 않았다는 뜻이다.
    assert meta.qpos.mean[0] == pytest.approx(float(a.qpos[:, 0].mean()), rel=1e-4)


def test_action_stats_use_the_first_step_only(conn, tmp_path):
    """chunk 는 이웃끼리 겹치므로 전부 세면 겹치는 구간에 가중치가 실린다."""
    ep, t0 = _make_episode(conn)
    meta, _ = export_dataset(conn, [ep], out_dir=str(tmp_path), arm_joints=ARM,
                             camera_topic='/camera/image_raw', chunk_len=4)
    a = load_episode(str(tmp_path / 'episodes' / f'{ep}.npz'))
    assert meta.arm_action.mean[0] == pytest.approx(
        float(a.action_arm[:, 0, 0].mean()), rel=1e-4)


# ----- LabelStats 가 파일에 남는가 ------------------------------------------

def test_missing_event_is_counted_and_excluded(conn, tmp_path):
    """이벤트가 없으면 **정상 종료로 가정**하고, 그 사실을 파일에 남긴다.

    남기지 않으면 가정으로 만든 레이블이 정상 레이블과 구분 없이 학습에 들어간다.
    """
    ep, t0 = _make_episode(conn, with_event=False)
    meta, lines = export_dataset(conn, [ep], out_dir=str(tmp_path), arm_joints=ARM,
                                 camera_topic='/camera/image_raw', chunk_len=4)
    assert lines[0].assumed_full_duration == 1
    assert lines[0].trustworthy is False
    assert meta.episodes == ()          # 통계에서 빠졌다
    assert meta.samples == 0
    assert any('excluded' in n for n in meta.notes)
    # 그래도 파일은 남는다 — 왜 뺐는지 들여다볼 수 있어야 한다.
    assert (tmp_path / 'episodes' / f'{ep}.npz').exists()


def test_keeping_untrustworthy_is_opt_in(conn, tmp_path):
    ep, t0 = _make_episode(conn, with_event=False)
    meta, _ = export_dataset(conn, [ep], out_dir=str(tmp_path), arm_joints=ARM,
                             camera_topic='/camera/image_raw', chunk_len=4,
                             skip_untrustworthy=False)
    assert meta.episodes == (ep,)
    assert meta.samples == 10


# ----- k 를 잘라 쓰기 (설계 §5) ---------------------------------------------

def test_a_longer_chunk_can_be_truncated_to_a_shorter_one(conn, tmp_path):
    """**k 를 바꿔도 재export 가 필요 없다** — τ_j 가 k 와 무관하기 때문이다.

    이것이 성립해야 ACT 문서 §9 의 1(1 s 대 2 s 비교)을 한 벌로 할 수 있다.
    """
    ep, t0 = _make_episode(conn)
    export_episode(conn, ep, out_dir=str(tmp_path / 'k8'), spec=_spec(chunk_len=8))
    export_episode(conn, ep, out_dir=str(tmp_path / 'k4'), spec=_spec(chunk_len=4))
    long_ = load_episode(str(tmp_path / 'k8' / 'episodes' / f'{ep}.npz'))
    short = load_episode(str(tmp_path / 'k4' / 'episodes' / f'{ep}.npz'))
    assert np.array_equal(long_.action_arm[:, :4, :], short.action_arm)
    assert np.array_equal(long_.action_gripper[:, :4, :], short.action_gripper)
    assert np.array_equal(long_.is_pad[:, :4], short.is_pad)


# ----- 결정론 -------------------------------------------------------------

def test_exporting_twice_gives_identical_arrays(conn, tmp_path):
    """다시 뽑아도 같은 값이어야 한다 (설계 §7). 다르면 순서 의존이 있다는 뜻이다."""
    ep, t0 = _make_episode(conn)
    export_episode(conn, ep, out_dir=str(tmp_path / 'a'), spec=_spec())
    export_episode(conn, ep, out_dir=str(tmp_path / 'b'), spec=_spec())
    a = load_episode(str(tmp_path / 'a' / 'episodes' / f'{ep}.npz'))
    b = load_episode(str(tmp_path / 'b' / 'episodes' / f'{ep}.npz'))
    for name in ('frame_index', 't', 'qpos', 'action_arm', 'action_gripper', 'is_pad'):
        assert np.array_equal(getattr(a, name), getattr(b, name)), name


# ----- mp4 경로 -----------------------------------------------------------

def test_mp4_is_relative_to_recordings_dir(conn, tmp_path):
    """절대 경로로 적으면 학습 기계로 옮길 때 깨진다 (설계 §6.1)."""
    ep, t0 = _make_episode(conn)
    line = export_episode(conn, ep, out_dir=str(tmp_path), spec=_spec(),
                          recordings_dir='/data/rdfp/recordings')
    assert line.mp4 == 'cam_100_video.mp4'


def test_mp4_outside_recordings_dir_keeps_its_path(conn, tmp_path):
    ep, t0 = _make_episode(conn)
    line = export_episode(conn, ep, out_dir=str(tmp_path), spec=_spec(),
                          recordings_dir='/somewhere/else')
    assert line.mp4 == '/data/rdfp/recordings/cam_100_video.mp4'


def test_relative_mp4_helper_handles_no_recordings_dir():
    assert _relative_mp4('/a/b.mp4', None) == '/a/b.mp4'
    assert _relative_mp4('/a/b.mp4', '') == '/a/b.mp4'


# ----- 거부 ---------------------------------------------------------------

def test_unknown_label_is_rejected(conn, tmp_path):
    ep, t0 = _make_episode(conn)
    with pytest.raises(ExportError, match='unknown label'):
        export_episode(conn, ep, out_dir=str(tmp_path), spec=_spec(label='tcp_plan'))


def test_empty_episode_list_is_rejected(conn, tmp_path):
    with pytest.raises(ExportError, match='no episodes'):
        export_dataset(conn, [], out_dir=str(tmp_path))


# ----- 두 레이블 비교 (E5 의 목적) -------------------------------------------

def test_command_label_exports_with_the_same_shape(conn, tmp_path):
    """`CommandLabel` 은 `PlanLabel` 과 **같은 인터페이스**라 바꿔 끼워진다."""
    ep, t0 = _make_episode(conn)
    _joint_states(conn, ep, t0=t0, n=50, dt=0.02, topic='/target_joint_cmds',
                  base=0.5)
    conn.commit()
    plan = export_episode(conn, ep, out_dir=str(tmp_path / 'plan'), spec=_spec())
    cmd = export_episode(conn, ep, out_dir=str(tmp_path / 'cmd'),
                         spec=_spec(label=LABEL_COMMAND))
    a = load_episode(str(tmp_path / 'plan' / 'episodes' / f'{ep}.npz'))
    b = load_episode(str(tmp_path / 'cmd' / 'episodes' / f'{ep}.npz'))
    assert a.action_arm.shape == b.action_arm.shape
    assert plan.samples == cmd.samples
    # 값은 달라야 한다 — 그 차이를 보는 것이 E5 의 목적이다.
    assert not np.allclose(a.action_arm, b.action_arm)


def test_command_label_warns_about_the_gravity_offset_gap(conn, tmp_path):
    """오프셋 포함 여부가 spec 에 없다는 것을 note 로라도 남긴다 (E7 Q13′)."""
    ep, t0 = _make_episode(conn)
    _joint_states(conn, ep, t0=t0, n=50, dt=0.02, topic='/target_joint_cmds')
    conn.commit()
    meta, lines = export_dataset(conn, [ep], out_dir=str(tmp_path), arm_joints=ARM,
                                 camera_topic='/camera/image_raw', chunk_len=4,
                                 label=LABEL_COMMAND)
    assert any('gravity' in n for n in meta.notes)
    assert any('gravity-offset' in n for n in lines[0].notes)
