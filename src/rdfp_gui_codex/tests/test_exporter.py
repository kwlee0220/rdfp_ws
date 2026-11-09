import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from rdfp_gui_codex.management.common import Control, Cancelled
from rdfp_gui_codex.management.exporter import export_dataset, grid_frames, validate_options
from rdfp_gui_codex.management.ingest import plan_import, execute_import


@pytest.fixture
def recorded(db, bag, tmp_path):
    import cv2
    execute_import(db, plan_import(db, str(bag), ['/joint_states', '/target_joint_cmds'], starts=[10*10**9]))
    ep = db.episodes()[0]['id']
    video = tmp_path/'camera.mp4'
    out = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'mp4v'), 10, (32,24))
    assert out.isOpened()
    for i in range(5):
        out.write(np.full((24,32,3), 20*i, dtype=np.uint8))
    out.release()
    with db.connection(write=True) as conn:
        ids = {}
        for name, typ in [('/camera', 'sensor_msgs/msg/Image'), ('/gripper_states', 'rdfp_msgs/msg/GripperState'),
                          ('/gripper_cmds', 'rdfp_msgs/msg/GripperCommand')]:
            ids[name] = conn.execute('INSERT INTO topics(topic_name,topic_type) VALUES(%s,%s) RETURNING id', (name,typ)).fetchone()['id']
        conn.execute("INSERT INTO image_streams(episode_id,topic_id,mp4_path,codec,pixel_format,container_fps,width,height,frame_count) "
                     "VALUES(%s,%s,%s,'mpeg4','bgr8',10,32,24,5)", (ep,ids['/camera'],str(video)))
        for i in range(5):
            conn.execute('INSERT INTO image_frames(episode_id,topic_id,frame_index,stamp_sec,stamp_nanosec) VALUES(%s,%s,%s,11,%s)',
                         (ep,ids['/camera'],i,i*100_000_000))
            conn.execute("INSERT INTO gripper_states(episode_id,topic_id,stamp_sec,stamp_nanosec,goal,width,stalled,at_goal) "
                         "VALUES(%s,%s,11,%s,'open',0.08,false,true)", (ep,ids['/gripper_states'],i*100_000_000))
        conn.execute("INSERT INTO gripper_cmds(episode_id,topic_id,stamp_sec,stamp_nanosec,goal) VALUES(%s,%s,10,0,'open')",
                     (ep,ids['/gripper_cmds']))
    return ep, video


def test_npz_roundtrip_and_copied_video(db, recorded, tmp_path):
    ep, video = recorded
    output = tmp_path/'dataset'
    result = export_dataset(db,[ep],output,{'label':'joint_command'})
    assert result['episodes']==[ep]
    with np.load(output/'episodes'/f'{ep}.npz', allow_pickle=False) as arrays:
        assert arrays['qpos'].shape==(5,2)
        assert arrays['action_arm'].shape==(5,20,1)
        np.testing.assert_allclose(arrays['qpos'][:,1], .08)
    line = json.loads((output/'episodes.jsonl').read_text())
    assert (output/line['mp4']).read_bytes()==video.read_bytes()
    manifest = json.loads((output/'rdfp_manifest.json').read_text())
    assert manifest['accepted_episodes']==[ep] and manifest['files']
    assert db.episode_detail(ep)['counts']['image_frames']==5


def test_export_does_not_overwrite(db, recorded, tmp_path):
    output=tmp_path/'existing';output.mkdir()
    with pytest.raises(ValueError, match='이미 존재'):
        export_dataset(db,[recorded[0]],output,{'label':'joint_command'})
    assert not list(output.iterdir())


def test_cancelled_export_is_not_published(db, recorded, tmp_path):
    output=tmp_path/'cancelled'
    with pytest.raises(Cancelled):
        export_dataset(db,[recorded[0]],output,{'label':'joint_command'}, control=Control(cancelled=lambda:True))
    assert not output.exists()
    assert not list(tmp_path.glob('.cancelled-*'))


def test_missing_video_fails_without_completed_output(db, recorded, tmp_path):
    recorded[1].unlink()
    with pytest.raises(ValueError, match='영상 파일'):
        export_dataset(db,[recorded[0]],tmp_path/'missing',{'label':'joint_command'})
    assert not (tmp_path/'missing').exists()


def test_grid_is_causal_and_checks_gaps():
    clock=SimpleNamespace(episode_start_ns=0,episode_end_ns=1_000_000_001,
                          frames=[SimpleNamespace(stamp_ns=0,index=0), SimpleNamespace(stamp_ns=1_000_000_000,index=1)])
    values=list(grid_frames(clock,2,1.0))
    assert values==[(0,0,0.0),(500_000_000,0,.5),(1_000_000_000,1,0.0)]
    with pytest.raises(ValueError,match='시차'):
        list(grid_frames(clock,2,.2))


@pytest.mark.parametrize('options', [{'fps':0},{'chunk_len':2.5},{'delta_sec':float('nan')}, {'step_sec':0}])
def test_invalid_export_options(options):
    with pytest.raises(ValueError):
        validate_options(options)


@pytest.mark.skipif(importlib.util.find_spec('lerobot') is None, reason='optional LeRobot integration environment')
def test_real_lerobot_v3_roundtrip(db, recorded, tmp_path):
    ep, _ = recorded
    output=tmp_path/'lerobot'
    result=export_dataset(db,[ep],output,{'format':'lerobot_v3','label':'joint_command','fps':10})
    assert result['episodes']==[ep]
    info=json.loads((output/'meta/info.json').read_text())
    assert info['codebase_version']=='v3.0'
    assert list((output/'data').rglob('*.parquet'))
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    loaded=LeRobotDataset('local/rdfp',root=output,video_backend='pyav')
    assert len(loaded)==5
    assert tuple(loaded[0]['action'].shape)==(4,)
