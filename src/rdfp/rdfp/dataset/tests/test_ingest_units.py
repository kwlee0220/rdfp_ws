"""적재 단위 · 출처 · 동적 매핑 — `import` CLI 를 실 PostgreSQL 에 대고 끝까지 돌린다.

**ROS 없이 돈다** — bag 에 이미지 토픽이 없으면 `cmd_import` 가 `sensor_msgs` 를 끌어오지 않는다.
scratch DB 를 만들 수 없으면(`createdb` 권한) 모듈 전체를 건너뛴다 — `export/test_writer.py` 와
같은 방식이다.

시나리오는 설계안 §4.4 · design_gpt §14.2 의 인수 테스트에서 왔다.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import uuid
from pathlib import Path

import pytest
import yaml

psycopg = pytest.importorskip('psycopg')
pytest.importorskip('mcap_ros2')

from mcap_ros2.writer import Writer  # noqa: E402

import rdfp.dataset.import_cmd as import_cmd  # noqa: E402
import rdfp.dataset.mapping_cmd as mapping_cmd  # noqa: E402

SCHEMA = Path(__file__).resolve().parents[1] / 'sql' / 'schema.sql'
SEC = 1_000_000_000
S0 = 1_790_000_000 * SEC
DSN_ENV = 'RDFP_UNITS_TEST_DSN'

HEADER = """
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""
SESSION_DEF = 'std_msgs/Header header\nstring state\nstring task_label\n' + HEADER
JOINT_DEF = ('std_msgs/Header header\nstring[] name\nfloat64[] position\nfloat64[] velocity\n'
             'float64[] effort\n' + HEADER)
POSE_DEF = ('std_msgs/Header header\ngeometry_msgs/Pose pose\n'
            '================================================================================\n'
            'MSG: geometry_msgs/Pose\ngeometry_msgs/Point position\n'
            'geometry_msgs/Quaternion orientation\n'
            '================================================================================\n'
            'MSG: geometry_msgs/Point\nfloat64 x\nfloat64 y\nfloat64 z\n'
            '================================================================================\n'
            'MSG: geometry_msgs/Quaternion\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n'
            + HEADER)
STATUS_TYPE = 'demo_msgs/msg/Status'
STATUS_DEF = 'std_msgs/Header header\nfloat64 value\nstring note\nfloat64[] samples\n' + HEADER


# ----- fixture: scratch DB -------------------------------------------------

@pytest.fixture(scope='module')
def dsn():
    if shutil.which('createdb') is None or shutil.which('psql') is None:
        pytest.skip('PostgreSQL client tools not available')
    name = f'rdfp_units_{uuid.uuid4().hex[:10]}'
    if subprocess.run(['createdb', name], capture_output=True).returncode != 0:
        pytest.skip('cannot create a scratch database (peer auth unavailable?)')
    try:
        yield f'postgresql:///{name}'
    finally:
        subprocess.run(['dropdb', '--if-exists', name], capture_output=True)


@pytest.fixture()
def db(dsn, monkeypatch):
    """시험마다 빈 스키마 — 테이블을 다 지우고 schema.sql 을 다시 올린다."""
    with psycopg.connect(dsn, autocommit=True) as conn:
        conn.execute('DROP SCHEMA public CASCADE; CREATE SCHEMA public')
        conn.execute(SCHEMA.read_text(encoding='utf-8'))
    monkeypatch.setenv(DSN_ENV, dsn)
    with psycopg.connect(dsn) as conn:
        yield conn


def q(conn, query, *args):
    """읽고 **바로 트랜잭션을 닫는다** — 열어 두면 그 잠금 때문에 CLI 의 `DROP TABLE` 이 영영 기다린다."""
    conn.rollback()
    with conn.cursor() as cur:
        cur.execute(query, args)
        result = cur.fetchall()
    conn.rollback()
    return result


# ----- 합성 bag -------------------------------------------------------------

def _hdr(ns):
    return {'stamp': {'sec': ns // SEC, 'nanosec': ns % SEC}, 'frame_id': ''}


def write_bag(root: Path, session: str, *, status_def: str = STATUS_DEF,
              with_status: bool = True, offset_ns: int = 0) -> Path:
    """에피소드 둘(10~20 s, 30~40 s), 관절 · 자세 · 사용자 정의 타입 메시지."""
    date = session.split('_')[1]
    sdir = root / date / session
    sdir.mkdir(parents=True)
    rel = f'{session}_0.mcap'
    events: list = []
    counts: dict = {}
    with open(sdir / rel, 'wb') as fp, Writer(fp) as w:
        sch = {'s': w.register_msgdef('rdfp_msgs/msg/SessionCommand', SESSION_DEF),
               'j': w.register_msgdef('sensor_msgs/msg/JointState', JOINT_DEF),
               'p': w.register_msgdef('geometry_msgs/msg/PoseStamped', POSE_DEF),
               'c': w.register_msgdef(STATUS_TYPE, status_def)}

        def put(topic, key, ns, msg, type_name):
            ns += offset_ns
            msg['header'] = _hdr(ns)
            w.write_message(topic, sch[key], msg, log_time=ns)
            events.append(ns)
            counts[topic] = (type_name, counts.get(topic, (None, 0))[1] + 1)

        put('/session', 's', S0, {'state': 'IN_SESSION', 'task_label': 'pick'},
            'rdfp_msgs/msg/SessionCommand')
        for start in (10, 30):
            put('/session', 's', S0 + start * SEC, {'state': 'IN_EPISODE', 'task_label': 'pick'},
                'rdfp_msgs/msg/SessionCommand')
            for i in range(5):
                t = S0 + (start + 1 + i) * SEC
                put('/joint_states', 'j', t, {'name': ['j1'], 'position': [0.1 * i],
                                              'velocity': [0.0], 'effort': [0.0]},
                    'sensor_msgs/msg/JointState')
                put('/ee_pose', 'p', t + 1, {'pose': {
                    'position': {'x': 1.0, 'y': 2.0, 'z': float(i)},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}},
                    'geometry_msgs/msg/PoseStamped')
                if with_status:
                    put('/status', 'c', t + 2, {'value': float('nan') if i == 0 else float(i),
                                                'note': f'n{i}', 'samples': [1.0, 2.0]},
                        STATUS_TYPE)
            put('/session', 's', S0 + (start + 10) * SEC,
                {'state': 'IN_SESSION', 'task_label': 'pick'}, 'rdfp_msgs/msg/SessionCommand')
    start_ns, end_ns = min(events), max(events)
    meta = {'rosbag2_bagfile_information': {
        'version': 6, 'storage_identifier': 'mcap',
        'duration': {'nanoseconds': end_ns - start_ns},
        'starting_time': {'nanoseconds_since_epoch': start_ns},
        'message_count': len(events),
        'topics_with_message_count': [
            {'topic_metadata': {'name': t, 'type': ty, 'serialization_format': 'cdr'},
             'message_count': n} for t, (ty, n) in counts.items()],
        'files': [{'path': rel, 'starting_time': {'nanoseconds_since_epoch': start_ns},
                   'duration': {'nanoseconds': end_ns - start_ns},
                   'message_count': len(events)}]}}
    (sdir / 'metadata.yaml').write_text(yaml.safe_dump(meta), encoding='utf-8')
    return sdir


def write_recording(root: Path, episode_ns: int, topic: str = '/cam', frames: int = 3) -> None:
    """레코더 출력 한 벌(내용 없는 mp4 + sidecar + metadata) — 등록은 디코드하지 않는다."""
    root.mkdir(parents=True, exist_ok=True)
    stem = f'rec_{episode_ns}_{topic.strip("/").replace("/", "_")}'
    (root / f'{stem}.mp4').write_bytes(b'')
    with open(root / f'{stem}.jsonl', 'w') as fp:
        for i in range(frames):
            ns = episode_ns + (i + 1) * SEC
            fp.write(json.dumps({'frame_index': i,
                                 'stamp': {'sec': ns // SEC, 'nanosec': ns % SEC}}) + '\n')
    (root / f'{stem}_metadata.json').write_text(json.dumps({
        'episode_key': {'sec': episode_ns // SEC, 'nanosec': episode_ns % SEC},
        'mp4': f'{stem}.mp4', 'sidecar': f'{stem}.jsonl', 'topic': topic,
        'topic_type': 'sensor_msgs/msg/Image', 'codec': 'libx264', 'fps': 10,
        'pixel_format': 'rgb8', 'frame_id': 'cam', 'width': 4, 'height': 4,
        'nframe': frames}))


def config(tmp_path: Path, rosbag_dir: Path, *, recordings: bool = False,
           parallelism: int = 1) -> str:
    cfg = {'rosbag_dir': str(rosbag_dir), 'output_mp4_dir': str(tmp_path / 'videos'),
           'db': {'dsn_env': DSN_ENV}, 'parallelism': parallelism}
    if recordings:
        cfg['recordings_dir'] = str(tmp_path / 'recordings')
    path = tmp_path / f'dataset_{uuid.uuid4().hex[:6]}.yaml'
    path.write_text(yaml.safe_dump(cfg), encoding='utf-8')
    return str(path)


def run_import(cfg_path: str, *extra: str, capsys=None) -> tuple:
    rc = import_cmd.main(['--config', cfg_path, *extra])
    summary = None
    if capsys is not None:
        out = capsys.readouterr().out
        start = out.find('{')
        summary = json.loads(out[start:]) if start >= 0 else None
    return rc, summary


def rows(conn, table, topic):
    return q(conn, f'SELECT count(*) FROM {table} t JOIN topics p ON p.id = t.topic_id '
                   'WHERE p.topic_name = %s', topic)[0][0]


# ----- 시험 -----------------------------------------------------------------

SESSION_A = 'session_2026-11-23_10-00-00'


def test_import_records_source_and_units_and_reimport_skips(db, tmp_path, capsys) -> None:
    bags = tmp_path / 'bags'
    write_bag(bags, SESSION_A)
    cfg = config(tmp_path, bags)

    rc, summary = run_import(cfg, capsys=capsys)
    assert rc == 0
    assert summary['episodes'] == 2
    assert rows(db, 'joint_states', '/joint_states') == 10
    assert rows(db, 'pose_stampeds', '/ee_pose') == 10
    assert q(db, 'SELECT DISTINCT source_id, adopted FROM episode_sources') == [(SESSION_A, False)]
    units = q(db, 'SELECT p.topic_name, u.rows FROM ingest_units u JOIN topics p '
                  'ON p.id = u.topic_id ORDER BY 1, u.episode_id')
    assert units == [('/ee_pose', 5), ('/ee_pose', 5), ('/joint_states', 5), ('/joint_states', 5)]

    rc, summary = run_import(cfg, capsys=capsys)
    assert rc == 0
    assert summary['skipped'] == 2 and summary['episodes'] == 0
    assert rows(db, 'joint_states', '/joint_states') == 10      # 두 번 넣지 않는다


def test_partial_topics_then_fill_the_rest_keeps_episode_ids(db, tmp_path, capsys) -> None:
    """빠진 토픽을 나중에 채운다 — 예전에는 replace 로 에피소드를 통째로 다시 넣어야 했다."""
    bags = tmp_path / 'bags'
    write_bag(bags, SESSION_A)
    cfg = config(tmp_path, bags)

    assert run_import(cfg, '--topics', '/joint_states', capsys=capsys)[0] == 0
    ids = q(db, 'SELECT id FROM sessions ORDER BY id')
    assert rows(db, 'pose_stampeds', '/ee_pose') == 0
    joint_ids = q(db, 'SELECT id FROM joint_states ORDER BY id')

    rc, summary = run_import(cfg, capsys=capsys)
    assert rc == 0
    assert summary['episodes'] == 2                    # 새로 넣은 것이 있다
    assert rows(db, 'pose_stampeds', '/ee_pose') == 10
    assert q(db, 'SELECT id FROM sessions ORDER BY id') == ids
    assert q(db, 'SELECT id FROM joint_states ORDER BY id') == joint_ids   # 건드리지 않았다


def test_only_topics_replace_touches_only_the_chosen_topic(db, tmp_path, capsys) -> None:
    bags = tmp_path / 'bags'
    write_bag(bags, SESSION_A)
    cfg = config(tmp_path, bags)
    run_import(cfg, capsys=capsys)
    ids = q(db, 'SELECT id FROM sessions ORDER BY id')
    pose_ids = q(db, 'SELECT id FROM pose_stampeds ORDER BY id')
    joint_ids = q(db, 'SELECT id FROM joint_states ORDER BY id')

    rc, _ = run_import(cfg, '--topics', '/joint_states', '--only-topics-replace', capsys=capsys)
    assert rc == 0
    assert q(db, 'SELECT id FROM sessions ORDER BY id') == ids
    assert q(db, 'SELECT id FROM pose_stampeds ORDER BY id') == pose_ids
    new_joint_ids = q(db, 'SELECT id FROM joint_states ORDER BY id')
    assert len(new_joint_ids) == 10 and set(new_joint_ids).isdisjoint(joint_ids)


def test_only_topics_replace_requires_topics(db, tmp_path) -> None:
    bags = tmp_path / 'bags'
    write_bag(bags, SESSION_A)
    assert import_cmd.main(['--config', config(tmp_path, bags), '--only-topics-replace']) == 2


def test_legacy_episodes_are_adopted_not_refused(db, tmp_path, capsys) -> None:
    """이 구조 이전에 CLI 로 적재된 에피소드(출처 · 이력 없음)를 시각이 맞으면 연결한다.

    Codex Manager 는 이것을 '출처 미확인 충돌'로 거부해 기존 65 에피소드에 아무것도 못 넣었다.
    """
    bags = tmp_path / 'bags'
    write_bag(bags, SESSION_A)
    cfg = config(tmp_path, bags)
    run_import(cfg, '--topics', '/joint_states', capsys=capsys)
    db.rollback()
    db.execute('DELETE FROM ingest_units')
    db.execute('DELETE FROM episode_sources')
    db.commit()

    rc, summary = run_import(cfg, capsys=capsys)
    assert rc == 0
    assert summary['adopted'] == 2
    assert q(db, 'SELECT DISTINCT source_id, adopted FROM episode_sources') == [(SESSION_A, True)]
    assert rows(db, 'joint_states', '/joint_states') == 10          # 다시 넣지 않았다
    assert rows(db, 'pose_stampeds', '/ee_pose') == 10              # 빠진 것은 채웠다
    adopted_units = q(db, 'SELECT count(*) FROM ingest_units WHERE adopted')[0][0]
    assert adopted_units == 2                                       # 옛 관절 행에 이력을 채웠다


def test_same_key_from_another_recording_is_refused(db, tmp_path, capsys, caplog) -> None:
    """sim 시계가 되감긴 두 녹화본 — 예전에는 조용히 속이 빈 에피소드가 됐다."""
    first, second = tmp_path / 'a', tmp_path / 'b'
    write_bag(first, SESSION_A)
    write_bag(second, 'session_2026-11-23_11-00-00')
    assert run_import(config(tmp_path, first), capsys=capsys)[0] == 0
    before = q(db, 'SELECT count(*) FROM joint_states')

    with caplog.at_level('ERROR'):
        rc = import_cmd.main(['--config', config(tmp_path, second)])
    assert rc == 1
    assert 'episode key collision' in caplog.text
    assert SESSION_A in caplog.text
    assert q(db, 'SELECT count(*) FROM joint_states') == before


def test_dynamic_mapping_fills_existing_episodes_losslessly(db, tmp_path, capsys) -> None:
    bags = tmp_path / 'bags'
    write_bag(bags, SESSION_A)
    cfg = config(tmp_path, bags)
    run_import(cfg, capsys=capsys)
    assert q(db, "SELECT count(*) FROM topics WHERE topic_name = '/status'")[0][0] == 0

    assert mapping_cmd.main(['register', '--config', cfg, '--type', STATUS_TYPE]) == 0
    capsys.readouterr()
    rc, summary = run_import(cfg, capsys=capsys)
    assert rc == 0 and summary['episodes'] == 2
    assert rows(db, 'status', '/status') == 10
    payload = q(db, 'SELECT payload FROM status ORDER BY stamp_sec, stamp_nanosec LIMIT 1')[0][0]
    assert payload['value'] == {'$float': 'nan'}            # NaN 을 0 이나 null 로 바꾸지 않는다
    assert payload['note'] == 'n0' and payload['samples'] == [1.0, 2.0]
    assert rows(db, 'joint_states', '/joint_states') == 10  # 다른 토픽은 그대로

    # 해제하면 다음 적재부터 무시하고, 행은 남는다.
    assert mapping_cmd.main(['unregister', '--config', cfg, '--type', STATUS_TYPE,
                             '--yes']) == 0
    assert rows(db, 'status', '/status') == 10
    # 삭제는 테이블 이름을 다시 적어야 한다.
    assert mapping_cmd.main(['drop', '--config', cfg, '--type', STATUS_TYPE, '--yes']) == 2
    assert mapping_cmd.main(['drop', '--config', cfg, '--type', STATUS_TYPE, '--yes',
                             '--confirm', 'status']) == 0
    assert q(db, "SELECT to_regclass('status') IS NULL")[0][0]


def test_dynamic_mapping_refuses_a_changed_definition(db, tmp_path, capsys) -> None:
    bags = tmp_path / 'bags'
    write_bag(bags, SESSION_A)
    cfg = config(tmp_path, bags)
    assert mapping_cmd.main(['register', '--config', cfg, '--type', STATUS_TYPE]) == 0
    other = tmp_path / 'other'
    write_bag(other, 'session_2026-11-23_12-00-00', offset_ns=1000 * SEC,
              status_def=STATUS_DEF.replace('string note', 'string note\nint32 extra'))
    rc = import_cmd.main(['--config', config(tmp_path, other)])
    assert rc == 1
    assert q(db, 'SELECT count(*) FROM status')[0][0] == 0


def test_builtin_types_cannot_be_registered_or_dropped(db, tmp_path) -> None:
    bags = tmp_path / 'bags'
    write_bag(bags, SESSION_A)
    cfg = config(tmp_path, bags)
    assert mapping_cmd.main(['register', '--config', cfg,
                             '--type', 'sensor_msgs/msg/JointState']) == 1
    assert mapping_cmd.main(['drop', '--config', cfg, '--type', 'sensor_msgs/msg/JointState',
                             '--confirm', 'joint_states', '--yes']) == 1
    assert q(db, "SELECT to_regclass('joint_states') IS NOT NULL")[0][0]


def test_recordings_register_once_and_follow_topic_selection(db, tmp_path, capsys) -> None:
    bags = tmp_path / 'bags'
    write_bag(bags, SESSION_A)
    for start in (10, 30):
        write_recording(tmp_path / 'recordings', S0 + start * SEC)
    cfg = config(tmp_path, bags, recordings=True)

    assert run_import(cfg, '--topics', '/joint_states', capsys=capsys)[0] == 0
    assert q(db, 'SELECT count(*) FROM image_streams')[0][0] == 0     # 카메라를 안 골랐다

    rc, summary = run_import(cfg, capsys=capsys)
    assert rc == 0
    assert q(db, 'SELECT count(*) FROM image_streams')[0][0] == 2
    assert q(db, 'SELECT count(*) FROM image_frames')[0][0] == 6

    rc, summary = run_import(cfg, capsys=capsys)                     # UNIQUE 위반 없이 건너뛴다
    assert rc == 0 and summary['skipped'] == 2
    assert q(db, 'SELECT count(*) FROM image_streams')[0][0] == 2


def test_parallel_path_has_the_same_unit_rules(db, tmp_path, capsys) -> None:
    bags = tmp_path / 'bags'
    write_bag(bags, SESSION_A)
    cfg = config(tmp_path, bags, parallelism=2)
    assert run_import(cfg, '--topics', '/joint_states', capsys=capsys)[0] == 0
    rc, summary = run_import(cfg, capsys=capsys)
    assert rc == 0 and summary['episodes'] == 2
    assert rows(db, 'joint_states', '/joint_states') == 10
    assert rows(db, 'pose_stampeds', '/ee_pose') == 10
    rc, summary = run_import(cfg, capsys=capsys)
    assert rc == 0 and summary['skipped'] == 2


def test_import_on_a_pre_metadata_db_stops_with_init_db_hint(db, tmp_path, caplog) -> None:
    bags = tmp_path / 'bags'
    write_bag(bags, SESSION_A)
    db.rollback()
    db.execute('DROP TABLE ingest_units, episode_sources, ingest_sources, topic_tables')
    db.commit()
    with caplog.at_level('ERROR'):
        rc = import_cmd.main(['--config', config(tmp_path, bags)])
    assert rc == 1
    assert 'init-db' in caplog.text
    assert os.environ.get(DSN_ENV)
