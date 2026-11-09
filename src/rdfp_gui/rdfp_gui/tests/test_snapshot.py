"""스냅샷 — 합성 bag 으로 토픽 · 매핑 · 에피소드를 묶는다 (DB 없음 · Qt 없음)."""

from __future__ import annotations

from rdfp.dataset.config import DatasetConfig
from rdfp.dataset.inspect.mapping import STATUS_MAPPED, STATUS_SPECIAL
from rdfp.rosbag.tests.fixtures.make_synth_bag import (JointStateEventSpec, PoseEventSpec,
                                                       SessionEventSpec, write_synth_bag)
from rdfp_gui.snapshot import (BagSnapshot, diagnose_topic, exported_flag, load_bag_snapshot,
                               load_db_snapshot, session_export_status, unmapped_topics)

S0 = 1_700_000_000_000_000_000
SEC = 1_000_000_000


def _cfg(tmp_path) -> DatasetConfig:
    return DatasetConfig.model_validate({'rosbag_dir': str(tmp_path), 'output_mp4_dir': '/tmp/v'})


def _bag(tmp_path):
    return write_synth_bag(
        tmp_path,
        session_events=[SessionEventSpec(S0, 'IN_SESSION', 'pick'),
                        SessionEventSpec(S0 + 10 * SEC, 'IN_EPISODE', 'pick'),
                        SessionEventSpec(S0 + 20 * SEC, 'IN_SESSION', 'pick')],
        joint_state_events=[JointStateEventSpec(S0 + s * SEC, position=[0.0]) for s in (11, 12)],
        pose_events=[PoseEventSpec(S0 + 15 * SEC)])


def test_bag_snapshot_pairs_topics_with_mappings(tmp_path) -> None:
    _bag(tmp_path)
    bag = load_bag_snapshot(_cfg(tmp_path))
    assert bag.error is None
    by_name = {t.name: t for t in bag.topics}
    assert by_name['/joint_states'].mapping.status == STATUS_MAPPED
    assert by_name['/joint_states'].mapping.table == 'joint_states'
    assert by_name['/joint_states'].checkable is True
    assert by_name['/session'].mapping.status == STATUS_SPECIAL
    assert by_name['/session'].checkable is False
    assert len(bag.episodes) == 1
    assert bag.windows == [(S0 + 10 * SEC, S0 + 20 * SEC)]
    assert unmapped_topics(bag) == []
    assert bag.topic_types()['/ee_pose_publisher/ee_pose'] == 'geometry_msgs/msg/PoseStamped'


def test_diagnose_topic_uses_the_episode_windows(tmp_path) -> None:
    _bag(tmp_path)
    bag = load_bag_snapshot(_cfg(tmp_path))
    diag = diagnose_topic(bag, '/joint_states')
    assert (diag.total, diag.in_episode) == (2, 2)
    assert bag.entry('/joint_states').glyph == '●'


def test_bag_snapshot_reports_missing_dir_and_config() -> None:
    assert load_bag_snapshot(None).error == 'config not loaded'
    missing = load_bag_snapshot(DatasetConfig.model_validate({'rosbag_dir': '/nope/x'}))
    assert 'not found' in missing.error


def test_db_snapshot_without_db_still_lists_builtin_tables(tmp_path, monkeypatch) -> None:
    """DB 에 못 붙어도 오른쪽 패널은 비지 않아야 선을 그릴 수 있다."""
    monkeypatch.delenv('RDFP_DB_DSN', raising=False)
    _bag(tmp_path)
    bag = load_bag_snapshot(_cfg(tmp_path))
    db = load_db_snapshot(_cfg(tmp_path), bag)
    assert db.error and 'RDFP_DB_DSN' in db.error
    names = [t.name for t in db.tables]
    assert 'joint_states' in names and 'sessions' in names
    joint = db.entry('joint_states')
    assert joint.rows is None
    assert joint.incoming == ['/joint_states']
    assert db.exported_keys is None


def test_exported_flag_keeps_unknown_distinct(tmp_path) -> None:
    _bag(tmp_path)
    bag = load_bag_snapshot(_cfg(tmp_path))
    be = bag.episodes[0]
    assert exported_flag(be, None) == '?'
    assert exported_flag(be, set()) == 'no'
    assert exported_flag(be, {be.start_ns}) == 'yes'
    session = bag.sessions[0]
    assert session_export_status(session, bag.episodes, None) == '? / 1'
    assert session_export_status(session, bag.episodes, {be.start_ns}) == '1 / 1'
    assert session_export_status(session, [], set()) == '-'


def test_empty_snapshot_helpers() -> None:
    bag = BagSnapshot(rosbag_dir=None)
    assert bag.entry('/x') is None
    assert bag.windows == []
    assert bag.mcap_paths('/x') == []
