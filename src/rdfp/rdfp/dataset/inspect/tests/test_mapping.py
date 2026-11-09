"""토픽 → 테이블 매핑 상태 (내장 registry 기준)."""

from __future__ import annotations

from rdfp.dataset.inspect.mapping import (
    SOURCE_BUILTIN, SOURCE_MEDIA, SOURCE_SYSTEM, STATUS_MAPPED, STATUS_SPECIAL, STATUS_SUSPECT,
    STATUS_UNMAPPED, GLYPH, builtin_tables, classify_topic, classify_topics, glyph_for,
    table_for_type)


def test_joint_state_is_mapped_to_joint_states() -> None:
    info = classify_topic('/target_joint_cmds', 'sensor_msgs/msg/JointState')
    assert info.status == STATUS_MAPPED
    assert info.table == 'joint_states'
    assert info.writer == 'JointStateWriter'
    assert info.reader == 'JointStateReader'
    assert info.ingested is True
    assert info.glyph == GLYPH[STATUS_MAPPED]


def test_two_topics_of_one_type_share_the_table() -> None:
    """매핑의 키는 타입이다 — 토픽이 아니다."""
    infos = classify_topics({'/joint_states': 'sensor_msgs/msg/JointState',
                             '/target_joint_cmds': 'sensor_msgs/msg/JointState'})
    assert {i.table for i in infos.values()} == {'joint_states'}


def test_session_and_image_are_special() -> None:
    session = classify_topic('/session', 'rdfp_msgs/msg/SessionCommand')
    assert session.status == STATUS_SPECIAL
    assert session.table == 'sessions'
    image = classify_topic('/camera/image_raw', 'sensor_msgs/msg/Image')
    assert image.status == STATUS_SPECIAL
    assert image.table == 'image_streams'
    assert image.is_image is True
    assert 'UNIQUE' in image.reason


def test_compressed_image_is_unmapped_with_the_recorder_explanation() -> None:
    info = classify_topic('/camera_image/compressed', 'sensor_msgs/msg/CompressedImage')
    assert info.status == STATUS_UNMAPPED
    assert info.table is None
    assert info.ingested is False
    assert '레코더 mp4' in info.reason


def test_unknown_type_is_unmapped_and_names_the_registry() -> None:
    info = classify_topic('/foo', 'some_pkg/msg/Thing')
    assert info.status == STATUS_UNMAPPED
    assert 'registry' in info.reason
    assert glyph_for(info, suspect=True) == GLYPH[STATUS_UNMAPPED]   # 매핑 없음이 더 중요하다


def test_suspect_glyph_overlays_mapped_only() -> None:
    mapped = classify_topic('/ee_pose', 'geometry_msgs/msg/PoseStamped')
    assert glyph_for(mapped, suspect=True) == GLYPH[STATUS_SUSPECT]
    assert glyph_for(mapped, suspect=False) == GLYPH[STATUS_MAPPED]


def test_builtin_tables_cover_every_required_table_with_a_source() -> None:
    tables = {t.name: t for t in builtin_tables()}
    assert tables['sessions'].source == SOURCE_SYSTEM
    assert tables['topics'].source == SOURCE_SYSTEM
    assert tables['image_streams'].source == SOURCE_MEDIA
    assert tables['image_frames'].source == SOURCE_MEDIA
    assert tables['joint_states'].source == SOURCE_BUILTIN
    assert tables['joint_states'].type_names == ('sensor_msgs/msg/JointState',)
    assert tables['motion_events'].type_names == ('rdfp_msgs/msg/MotionEvent',)


def test_table_for_type() -> None:
    assert table_for_type('sensor_msgs/msg/Image') == 'image_streams'
    assert table_for_type('rdfp_msgs/msg/GripperCommand') == 'gripper_cmds'
    assert table_for_type('nope/msg/Nope') is None
