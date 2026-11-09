import shutil
from pathlib import Path

import pytest
from mcap_ros2.writer import Writer

from rdfp_gui_codex.management import catalog
from rdfp_gui_codex.management.common import Cancelled, Control, json_value, parse_ids


def test_scan_schema_browse_and_episode_boundary(bag):
    rows = catalog.scan(str(bag))
    joint = next(r for r in rows if r['topic']=='/joint_states')
    assert joint['count'] == 2 and 'float64[] position' in joint['definition']
    assert [e['start_ns'] for e in catalog.episodes(str(bag))] == [10*10**9, 20*10**9]
    page = catalog.browse(str(bag), '/joint_states', offset=1, limit=1)
    assert len(page)==1 and page[0]['header_stamp_ns']==21*10**9
    assert page[0]['value']['position']==[.4]


def test_identity_is_location_independent(bag, tmp_path):
    copied = tmp_path/'renamed.mcap'
    shutil.copy2(bag, copied)
    assert catalog.source_identity(str(bag))[0] == catalog.source_identity(str(copied))[0]


def test_headerless_is_visible(tmp_path):
    p = tmp_path/'headerless.mcap'
    with p.open('wb') as f:
        w = Writer(f)
        schema = w.register_msgdef('std_msgs/msg/String', 'string data')
        w.write_message('/text', schema, {'data': 'hello'}, log_time=100)
        w.finish()
    rows = catalog.browse(str(p), '/text')
    assert rows[0]['header_stamp_ns'] is None
    assert rows[0]['value']['data'] == 'hello'


def test_metadata_escape_rejected(tmp_path):
    (tmp_path/'metadata.yaml').write_text('rosbag2_bagfile_information:\n  relative_file_paths: [../outside.mcap]\n')
    with pytest.raises(ValueError, match='밖'):
        catalog.source_files(str(tmp_path))


def test_cancellation_during_catalog(bag):
    with pytest.raises(Cancelled):
        catalog.scan(str(bag), Control(cancelled=lambda: True))


def test_loss_aware_values():
    assert json_value(float('nan')) == {'$float': 'nan'}
    assert json_value(2**63) == 2**63
    assert json_value([1,2,3], limit=1) == [1, {'$omitted': 2}]


@pytest.mark.parametrize('spec', ['', '0', '3-1', '1;DROP', '-1', '1-1000000'])
def test_bad_episode_selection(spec):
    with pytest.raises(ValueError):
        parse_ids(spec)


def test_episode_selection():
    assert parse_ids('1-3,2,8') == [1,2,3,8]
