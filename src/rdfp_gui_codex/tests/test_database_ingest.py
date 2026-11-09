import json

import pytest
from psycopg import sql

from rdfp_gui_codex.management import catalog
from rdfp_gui_codex.management.common import Control, Cancelled
from rdfp_gui_codex.management.ingest import plan_import, execute_import
from rdfp_gui_codex.management.common import dumps


def topic(bag, name='/custom'):
    return next(r for r in catalog.scan(str(bag)) if r['topic']==name)


def test_shared_table_selected_import_and_retry(db, bag):
    first = plan_import(db, str(bag), ['/joint_states'])
    assert execute_import(db, first)['rows'] == 2
    ids = [e['id'] for e in db.episodes()]
    assert execute_import(db, first)['skipped_units'] == 2
    second = plan_import(db, str(bag), ['/target_joint_cmds'])
    assert execute_import(db, second)['rows'] == 2
    assert [e['id'] for e in db.episodes()] == ids
    assert len(db.browse('joint_states')) == 4
    assert len(db.browse('joint_states', topic_name='/target_joint_cmds')) == 2
    execute_import(db, plan_import(db, str(bag), ['/joint_states'], replace=True))
    assert len(db.browse('joint_states')) == 4
    assert [e['id'] for e in db.episodes()] == ids


def test_register_generic_import_and_drop(db, bag):
    t = topic(bag)
    db.register(t, 'custom_readings')
    plan = plan_import(db, str(bag), ['/custom'])
    execute_import(db, plan)
    rows = db.browse('custom_readings')
    assert len(rows)==2 and rows[0]['payload']['reading']==7.5 and rows[0]['raw_cdr']
    db.set_mapping_active(t, False)
    assert db.mappings([t])[0]['status']=='비활성'
    assert len(db.browse('custom_readings'))==2
    with pytest.raises(ValueError):
        plan_import(db, str(bag), ['/custom'])
    db.set_mapping_active(t, True)
    assert db.drop_custom(t, confirmed_table='custom_readings')['deleted_rows']==2
    assert 'custom_readings' not in [r['table_name'] for r in db.tables()]


def test_shared_and_builtin_tables_cannot_be_dropped(db, bag):
    t = topic(bag)
    db.register(t, 'custom_shared')
    db.register({**t, 'topic': '/second'}, 'custom_shared', create=False)
    with pytest.raises(ValueError, match='공유'):
        db.drop_custom(t, confirmed_table='custom_shared')
    with pytest.raises(ValueError):
        db.drop_custom(topic(bag, '/joint_states'), confirmed_table='joint_states')


def test_name_injection_rejected(db, bag):
    with pytest.raises(ValueError):
        db.register(topic(bag), 'custom_x; DROP TABLE sessions')
    assert db.describe('sessions')['columns']


def test_schema_conflict_is_not_silently_accepted(db, bag):
    t = topic(bag)
    db.register(t, 'custom_x')
    assert db.mappings([{**t, 'schema_hash':'different'}])[0]['status']=='스키마 충돌'


def test_legacy_episode_collision_is_blocked(db, bag):
    with db.connection(write=True) as conn:
        conn.execute('INSERT INTO sessions(start_sec,start_nanosec,stop_sec,stop_nanosec) VALUES(10,0,12,0)')
    with pytest.raises(ValueError, match='충돌'):
        plan_import(db, str(bag), ['/joint_states'])
    assert len(db.episodes())==1


def test_altered_plan_is_rejected(db, bag):
    plan = plan_import(db, str(bag), ['/joint_states'])
    plan['replace'] = True
    with pytest.raises(ValueError, match='계획이 변경'):
        execute_import(db, plan)
    assert not db.episodes()


def test_mapping_changed_after_plan(db, bag):
    plan = plan_import(db, str(bag), ['/joint_states'])
    db.set_mapping_active(topic(bag, '/joint_states'), False)
    with pytest.raises(ValueError, match='매핑이 변경'):
        execute_import(db, plan)
    assert not db.episodes()


def test_failure_rolls_back_whole_episode(db, bag, monkeypatch):
    from rdfp.dataset.db.registry import MESSAGE_TYPE_REGISTRY
    cls = MESSAGE_TYPE_REGISTRY['sensor_msgs/msg/JointState'].writer_cls
    plan = plan_import(db, str(bag), ['/joint_states'])
    monkeypatch.setattr(cls, 'row_values', lambda *args: (_ for _ in ()).throw(ValueError('bad payload')))
    with pytest.raises(ValueError, match='bad payload'):
        execute_import(db, plan)
    assert not db.episodes()
    assert not db.browse('joint_states')


def test_cancel_between_episodes_keeps_only_committed_unit(db, bag):
    plan = plan_import(db, str(bag), ['/joint_states'])
    stopped = False
    def report(event):
        nonlocal stopped
        if 'commit 완료' in event['message']:
            stopped = True
    with pytest.raises(Cancelled):
        execute_import(db, plan, Control(cancelled=lambda: stopped, report=report))
    assert len(db.episodes())==1
    assert len(db.browse('joint_states'))==1
    assert execute_import(db, plan)['skipped_units']==1
    assert len(db.episodes())==2


def test_read_only_paths_do_not_initialize_manager(db):
    with db.connection(write=True) as conn:
        for t in ['dm_imports','dm_mappings','dm_owned_tables','dm_episode_sources','dm_sources']:
            conn.execute(sql.SQL('DROP TABLE {}').format(sql.Identifier(t)))
    assert db.tables()
    assert db.describe('joint_states')['columns']
    with db.connection() as conn:
        assert not db.ready(conn)


def test_import_history_must_match_actual_rows(db, bag):
    plan = plan_import(db, str(bag), ['/joint_states'])
    execute_import(db, plan)
    with db.connection(write=True) as conn:
        conn.execute('DELETE FROM joint_states WHERE id=(SELECT min(id) FROM joint_states)')
    with pytest.raises(ValueError, match='실제 행 수'):
        execute_import(db, plan)
    execute_import(db, plan_import(db, str(bag), ['/joint_states'], replace=True))
    assert len(db.browse('joint_states')) == 2


def test_saved_plan_is_executable(db, bag):
    plan = plan_import(db, str(bag), ['/joint_states'])
    restored = json.loads(dumps(plan))
    assert restored == plan
    assert execute_import(db, restored)['rows'] == 2


@pytest.fixture
def recording(tmp_path):
    import cv2
    import numpy as np
    from rdfp.dataset.tests.test_recorder_registry import _write_recording
    root = tmp_path/'recordings'
    metadata = _write_recording(root, episode_sec=10, nframe=2, frames=[
        {'frame_index': i, 'stamp': {'sec': 11, 'nanosec': i * 100_000_000}}
        for i in range(2)])
    fields = json.loads(metadata.read_text())
    path = root/fields['mp4']
    video = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'), 10, (32,32))
    assert video.isOpened()
    for _ in range(2):
        video.write(np.zeros((32,32,3), dtype=np.uint8))
    video.release()
    return root, root/fields['sidecar'], fields['topic']


def test_selected_recorder_import_is_idempotent(db, bag, recording):
    root, _, camera = recording
    plan = plan_import(db, str(bag), [], starts=[10*10**9],
                       recordings_dir=str(root), media_topics=[camera])
    execute_import(db, plan)
    assert len(db.browse('image_streams')) == 1
    assert len(db.browse('image_frames')) == 2
    assert execute_import(db, plan)['skipped_units'] == 1
    execute_import(db, plan_import(db, str(bag), [], starts=[10*10**9],
                   recordings_dir=str(root), media_topics=[camera], replace=True))
    assert len(db.browse('image_frames')) == 2


@pytest.mark.parametrize('frames', [
    [{'frame_index': 1, 'stamp': {'sec': 11, 'nanosec': 0}}],
    [{'frame_index': 0, 'stamp': {'sec': 11, 'nanosec': 1_000_000_000}}],
    [{'frame_index': 0, 'stamp': {'sec': 12, 'nanosec': 0}}],
    [{'frame_index': i, 'stamp': {'sec': 11, 'nanosec': 0}} for i in range(2)],
])
def test_invalid_recorder_time_axis_is_rejected(db, bag, recording, frames):
    root, sidecar, camera = recording
    sidecar.write_text(''.join(json.dumps(f)+'\n' for f in frames))
    with pytest.raises(ValueError, match='프레임 번호·시각'):
        plan_import(db, str(bag), [], starts=[10*10**9], recordings_dir=str(root), media_topics=[camera])
    assert not db.episodes()
