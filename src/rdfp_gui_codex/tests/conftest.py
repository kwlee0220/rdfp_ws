"""Tests use a disposable PostgreSQL server, never the configured RDFP database."""
import getpass
from pathlib import Path
import shutil
import subprocess
import uuid

import pytest

from rdfp.dataset.db.config import DbConfig
from rdfp_gui_codex.management.database import Database


@pytest.fixture(scope='session')
def pg_server(tmp_path_factory):
    candidates = sorted(Path('/usr/lib/postgresql').glob('*/bin/initdb'))
    initdb = shutil.which('initdb') or (str(candidates[-1]) if candidates else None)
    if not initdb:
        pytest.skip('PostgreSQL server binaries are needed for integration tests')
    binary = Path(initdb).parent
    root = tmp_path_factory.mktemp('manager_pg')
    data, sock = root/'data', root/'socket'
    sock.mkdir()
    subprocess.run([str(binary/'initdb'), '-D', str(data), '-A', 'trust', '--no-locale', '-E', 'UTF8'],
                   check=True, capture_output=True)
    subprocess.run([str(binary/'pg_ctl'), '-D', str(data), '-l', str(root/'server.log'),
                    '-o', f"-F -k {sock} -h '' -p 55439", '-w', 'start'], check=True, capture_output=True)
    try:
        yield f'dbname=postgres user={getpass.getuser()} host={sock} port=55439'
    finally:
        subprocess.run([str(binary/'pg_ctl'), '-D', str(data), '-m', 'immediate', '-w', 'stop'],
                       check=True, capture_output=True)


@pytest.fixture
def db(pg_server, monkeypatch):
    monkeypatch.setenv('RDFP_MANAGER_TEST_DSN', pg_server)
    database = Database(DbConfig(dsn_env='RDFP_MANAGER_TEST_DSN', schema='test_'+uuid.uuid4().hex[:12]))
    database.initialize()
    return database


@pytest.fixture
def bag(tmp_path):
    from mcap_ros2.writer import Writer
    from rdfp.rosbag.tests.fixtures.make_synth_bag import SESSION_COMMAND_SCHEMA, JOINT_STATE_SCHEMA, HEADER_SUBSCHEMA
    path = tmp_path/'input.mcap'
    with path.open('wb') as stream:
        writer = Writer(stream)
        session = writer.register_msgdef('rdfp_msgs/msg/SessionCommand', SESSION_COMMAND_SCHEMA)
        joint = writer.register_msgdef('sensor_msgs/msg/JointState', JOINT_STATE_SCHEMA)
        custom = writer.register_msgdef('example_msgs/msg/Reading', 'std_msgs/Header header\nfloat64 reading\n'+HEADER_SUBSCHEMA)
        for sec, state in [(10, 'IN_EPISODE'), (12, 'IN_SESSION'), (20, 'IN_EPISODE'), (22, 'IN_SESSION')]:
            writer.write_message('/session', session, {'header': {'stamp': {'sec': sec, 'nanosec': 0}, 'frame_id': ''},
                'state': state, 'task_label': 'pick'}, log_time=sec*10**9)
        for sec in [11, 21]:
            header = {'stamp': {'sec': sec, 'nanosec': 0}, 'frame_id': 'panda_link0'}
            for topic in ['/joint_states', '/target_joint_cmds']:
                writer.write_message(topic, joint, {'header': header, 'name': ['panda_joint1'],
                    'position': [.4], 'velocity': [], 'effort': []}, log_time=sec*10**9)
            writer.write_message('/custom', custom, {'header': header, 'reading': 7.5}, log_time=sec*10**9)
        writer.finish()
    return path
