#!/usr/bin/env python3

"""관측 `qpos` 와 그리퍼 행동 (E2 · E4).

지키는 것:

1. **팔 관측은 선형 보간, 그리퍼 폭은 최근접.** 폭을 보간하면 개폐 중간의 없는
   값을 만든다.
2. **범위 밖 프레임을 버리지 않는다** (Q8) — 영상이 이미 있으므로 버리면 mp4
   프레임과 샘플 수가 어긋나 `frame_index` 가 밀린다. 양 끝 값을 유지한다.
3. **`width` 가 없으면 `NaN`** — 0 을 돌려주면 '닫혀 있다'는 거짓말이 된다.
4. **그리퍼 행동은 0차 유지 + one-hot**, 첫 명령 이전은 `open` (Q7).
5. **관절 이름으로 고른다** — 순서로 때우면 값 범위가 비슷해 뒤바뀌어도 안 드러난다.
   손가락 관절이 섞인 관측에서 팔 7축만 뽑는 것이 이 값의 쓸모다.

순수 계산부는 DB 없이, 로더는 실 PostgreSQL 로 건다.
"""

from __future__ import annotations

import math
import subprocess
from pathlib import Path

import pytest

from rdfp.dataset.export.observation import (
    GRIPPER_SYMBOLS, GripperCommandSeries, GripperWidthSeries, JointSeries,
    ObservationError, qpos)

NS = 1_000_000_000


def _joints(pairs, names=('j1', 'j2')) -> JointSeries:
    return JointSeries(joint_names=tuple(names),
                       stamps_ns=tuple(int(t * NS) for t, _ in pairs),
                       positions=tuple(tuple(p) for _, p in pairs))


# ── 1. 팔 관측: 선형 보간 ───────────────────────────────────────────────────

def test_joint_positions_are_linearly_interpolated() -> None:
    s = _joints([(100.0, (0.0, 0.0)), (100.2, (1.0, -1.0))])

    assert s.at(100.1) == pytest.approx((0.5, -0.5))
    assert s.at(100.05) == pytest.approx((0.25, -0.25))


def test_out_of_range_frames_hold_the_edge_values() -> None:
    """**버리지 않는다** — 영상이 이미 있어 샘플을 빼면 frame_index 가 밀린다."""
    s = _joints([(100.0, (0.0, 0.0)), (100.2, (1.0, -1.0))])

    assert s.at(99.0) == pytest.approx((0.0, 0.0))
    assert s.at(200.0) == pytest.approx((1.0, -1.0))


def test_an_empty_joint_series_is_refused() -> None:
    """**조용히 0 으로 채우지 않는다** — 0 도 그럴듯한 관절값이다."""
    with pytest.raises(ObservationError, match='empty'):
        JointSeries(joint_names=('j1',), stamps_ns=(), positions=()).at(1.0)


def test_duplicate_stamps_do_not_divide_by_zero() -> None:
    s = _joints([(100.0, (1.0, 2.0)), (100.0, (3.0, 4.0))])

    assert s.at(100.0) == pytest.approx((1.0, 2.0))


# ── 2. 그리퍼 폭: 최근접 ────────────────────────────────────────────────────

def test_width_takes_the_nearest_sample_not_an_interpolation() -> None:
    """개폐는 계단에 가깝다 — 보간하면 없는 중간값을 만든다."""
    w = GripperWidthSeries(stamps_ns=(100 * NS, 101 * NS), widths=(0.08, 0.0))

    assert w.at(100.4) == 0.08
    assert w.at(100.6) == 0.0
    assert w.at(100.5) == 0.08, '같은 거리면 앞쪽'


def test_width_is_nan_when_there_are_no_samples() -> None:
    """**0 은 '닫혀 있다'는 거짓말이다** — 펑션베이는 폭을 못 구해 NaN 을 낸다."""
    assert math.isnan(GripperWidthSeries(stamps_ns=(), widths=()).at(1.0))


def test_width_outside_the_range_holds_the_edge() -> None:
    w = GripperWidthSeries(stamps_ns=(100 * NS,), widths=(0.04,))

    assert w.at(0.0) == 0.04 and w.at(999.0) == 0.04


def test_qpos_is_arm_joints_plus_one_width() -> None:
    """ACT 의 관측 벡터다 — 팔 7축 + 그리퍼 1 (문서 §3.1)."""
    s = _joints([(100.0, (0.1, 0.2))])
    w = GripperWidthSeries(stamps_ns=(100 * NS,), widths=(0.08,))

    assert qpos(s, w, 100.0) == pytest.approx((0.1, 0.2, 0.08))


# ── 3. 그리퍼 행동: 0차 유지 + one-hot ──────────────────────────────────────

def _cmds(pairs) -> GripperCommandSeries:
    return GripperCommandSeries(stamps_ns=tuple(int(t * NS) for t, _ in pairs),
                                goals=tuple(g for _, g in pairs))


def test_the_last_command_is_held() -> None:
    """심볼은 경유점이 아니라 **유지되는 의도 상태**다."""
    c = _cmds([(100.0, 'open'), (102.0, 'grasp')])

    assert c.at(101.9) == 'open'
    assert c.at(102.0) == 'grasp'
    assert c.at(999.0) == 'grasp'


def test_before_the_first_command_is_open() -> None:
    """Q7 결정 — **닫힘을 기본값으로 두면 정책이 '시작하자마자 쥐라'를 배운다.**"""
    c = _cmds([(102.0, 'grasp')])

    assert c.at(100.0) == 'open'
    assert _cmds([]).at(100.0) == 'open'


def test_one_hot_order_is_the_contract() -> None:
    """배치가 argmax 로 심볼을 되돌리므로 순서가 계약이다."""
    assert GRIPPER_SYMBOLS == ('open', 'close', 'grasp')
    c = _cmds([(100.0, 'open'), (101.0, 'close'), (102.0, 'grasp')])

    assert c.one_hot(100.5) == (1.0, 0.0, 0.0)
    assert c.one_hot(101.5) == (0.0, 1.0, 0.0)
    assert c.one_hot(102.5) == (0.0, 0.0, 1.0)


def test_an_unknown_symbol_sets_nothing() -> None:
    """임의의 칸을 세우면 정책이 **없는 명령**을 배운다 — 데이터를 의심할 근거로 남긴다."""
    assert _cmds([(100.0, 'squeeze')]).one_hot(101.0) == (0.0, 0.0, 0.0)


# ── 4. DB 로더 ──────────────────────────────────────────────────────────────

psycopg = pytest.importorskip('psycopg', reason='needs psycopg')

from rdfp.dataset.export.observation import (                       # noqa: E402
    load_gripper_commands, load_gripper_width, load_joint_series)


@pytest.fixture
def db():
    name = 'rdfp_e2_test'
    schema = Path(__file__).resolve().parents[2] / 'sql' / 'schema.sql'
    try:
        subprocess.run(['psql', '-d', 'postgres', '-q', '-c',
                        f'DROP DATABASE IF EXISTS {name};', '-c',
                        f'CREATE DATABASE {name};'], check=True,
                       capture_output=True, timeout=30)
        subprocess.run(['psql', '-d', name, '-q', '-v', 'ON_ERROR_STOP=1',
                        '-f', str(schema)], check=True, capture_output=True, timeout=60)
    except Exception as exc:                       # noqa: BLE001
        pytest.skip(f'no local postgres: {exc}')
    conn = psycopg.connect(f'dbname={name}')
    try:
        yield conn
    finally:
        conn.close()
        subprocess.run(['psql', '-d', 'postgres', '-q', '-c',
                        f'DROP DATABASE IF EXISTS {name};'], capture_output=True)


def _setup(conn, rows, topic='/joint_states', episode_id=None):
    """행을 넣고 에피소드 id 를 돌려준다. `episode_id` 를 주면 그 에피소드에 더한다.

    에피소드 키 `(start_sec, start_nanosec)` 에 UNIQUE 가 걸려 있어 같은 키로 세션을
    두 번 만들 수 없다 — 한 에피소드에 토픽 여럿을 넣을 때는 id 를 물려준다.
    """
    with conn.cursor() as cur:
        if episode_id is None:
            cur.execute('INSERT INTO sessions (start_sec,start_nanosec,stop_sec,'
                        'stop_nanosec) VALUES (100,0,110,0) RETURNING id')
            ep = cur.fetchone()[0]
        else:
            ep = episode_id
        cur.execute('INSERT INTO topics (topic_name, topic_type) VALUES (%s, %s) '
                    'ON CONFLICT (topic_name) DO NOTHING', (topic, 'sensor_msgs/msg/JointState'))
        cur.execute('SELECT id FROM topics WHERE topic_name=%s', (topic,))
        tid = cur.fetchone()[0]
        for t, names, pos in rows:
            cur.execute('INSERT INTO joint_states (episode_id, topic_id, stamp_sec, '
                        'stamp_nanosec, joint_names, position, velocity, effort) '
                        'VALUES (%s,%s,%s,%s,%s,%s,%s,%s)',
                        (ep, tid, int(t), int(round((t % 1) * 1e9)), list(names),
                         list(pos), [], []))
    return ep


def test_loader_picks_joints_by_name(db) -> None:
    """**손가락 관절이 섞인 관측에서 팔만 뽑는 것이 이 값의 쓸모다.**"""
    ep = _setup(db, [(100.0, ['panda_joint1', 'panda_finger_joint1', 'panda_joint2'],
                      [0.1, 0.04, 0.2])])

    s = load_joint_series(db, ep, '/joint_states',
                          joint_names=['panda_joint1', 'panda_joint2'])

    assert s.joint_names == ('panda_joint1', 'panda_joint2')
    assert s.at(100.0) == pytest.approx((0.1, 0.2))


def test_loader_refuses_rows_without_joint_names(db) -> None:
    """I3 이전 데이터이거나 발행자가 `name` 을 안 채운 경우다 — 순서로 때우지 않는다."""
    ep = _setup(db, [(100.0, [], [0.1, 0.2])])

    with pytest.raises(ObservationError, match='no joint_names'):
        load_joint_series(db, ep, '/joint_states')


def test_loader_refuses_a_missing_joint(db) -> None:
    ep = _setup(db, [(100.0, ['panda_joint1'], [0.1])])

    with pytest.raises(ObservationError, match='missing joints'):
        load_joint_series(db, ep, '/joint_states', joint_names=['panda_joint1', 'nope'])


def test_loader_separates_topics_in_the_same_table(db) -> None:
    """`/joint_states`(관측)와 `/target_joint_cmds`(action)는 `topic_id` 로만 갈린다."""
    ep = _setup(db, [(100.0, ['j1'], [0.1])], topic='/joint_states')
    _setup(db, [(100.0, ['j1'], [9.9])], topic='/target_joint_cmds', episode_id=ep)

    assert load_joint_series(db, ep, '/joint_states').at(100.0) == pytest.approx((0.1,))
    assert load_joint_series(db, ep, '/target_joint_cmds').at(100.0) == pytest.approx((9.9,))


def test_gripper_loaders_return_empty_series_without_rows(db) -> None:
    """그리퍼 없는 스택도 유효하다 — 폭은 NaN, 명령은 기본값."""
    ep = _setup(db, [(100.0, ['j1'], [0.1])])

    assert math.isnan(load_gripper_width(db, ep).at(100.0))
    assert load_gripper_commands(db, ep).at(100.0) == 'open'
