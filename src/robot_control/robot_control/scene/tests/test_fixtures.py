#!/usr/bin/env python3

"""고정물 정의 — **값과 그 합성이 맞는가.**

이 값들은 시뮬레이터 XML 과 메시에서 손으로 옮겨 온 것이라, 틀려도 에러가 아니라
"팔이 엉뚱한 데로 간다"로 나타난다. 특히 **`geometry` 는 부품 원점 기준 상대값**이고
절대 좌표는 `pose` 와 합성해서 얻는데, 그 비대칭이 조용히 틀리기 쉽다.
"""

from __future__ import annotations

import json

import pytest

from robot_control.scene.fixtures import (
    Fixture, as_payload, describe, fixtures_path, load_fixtures, load_fixtures_file)

PEG_DIAMETER = 0.016        # functionbay_scene.json 의 peg (반지름 0.008)


@pytest.fixture(scope='module')
def fixtures():
    return load_fixtures('functionbay')


def test_the_file_is_reachable():
    """프로파일의 `scene.fixtures_file` 이 실재하는 파일을 가리키는가."""
    import os

    path = fixtures_path('functionbay')
    assert path and os.path.isfile(path), path


def test_frame_is_the_robot_base(fixtures):
    """시뮬레이터 XML 의 좌표계가 곧 `panda_link0` 다 (`t1__Fixed_1_vm.xml`)."""
    assert fixtures.frame == 'panda_link0'


def test_hole_pose_matches_the_simulator_xml(fixtures):
    """`t1__floating_1_vm.xml` 의 `<origin xyz="0.5 0 0.003" rpy="0 0 0"/>`."""
    hole = fixtures['peg_hole']

    assert hole.xyz == (0.5, 0.0, 0.003)
    assert hole.rpy == (0.0, 0.0, 0.0)
    assert hole.source_body == 'peg_hole_round_16'


def test_geometry_is_relative_not_absolute(fixtures):
    """**`geometry` 는 원점 기준이다.** 절대값을 적어 두면 XML 이 바뀌었을 때 어느 쪽이
    낡았는지 알 수 없다 — 그 구분이 이 파일의 전제다."""
    hole = fixtures['peg_hole']

    assert hole.value('entry_z') == 0.025, '원점 기준 상대값이어야 한다'
    assert hole.entry_point()[2] == pytest.approx(0.028), 'pose.z 0.003 과 합성된다'


def test_part_bottom_sits_on_the_table(fixtures):
    """부품 바닥(-0.003) + pose.z(+0.003) = 0 — 탁자면과 맞는다.

    이 교차 검증이 pose 와 메시가 같은 부품을 가리킨다는 증거다.
    """
    hole = fixtures['peg_hole']
    bottom_z = hole.xyz[2] + hole.value('bounds_z')[0]

    assert bottom_z == pytest.approx(0.0, abs=1e-9)


def test_hole_depth_and_clearance(fixtures):
    """깊이 25 mm, 지름 여유 1.0 mm (한쪽 0.5 mm) — 메시 실측."""
    hole = fixtures['peg_hole']

    assert hole.depth() == pytest.approx(0.025)
    assert hole.value('inner_diameter') == pytest.approx(0.017)
    assert hole.radial_clearance(PEG_DIAMETER) == pytest.approx(0.0005)


def test_a_fully_inserted_peg_reaches_the_entry_height(fixtures):
    """peg(50 mm)를 바닥까지 넣으면 중심이 입구 높이와 같다 — 깊이가 반높이와 같아서다.

    이 관계가 깨지면 깊이나 peg 치수 중 하나가 틀린 것이다.
    """
    hole = fixtures['peg_hole']
    peg_center = hole.floor_point()[2] + 0.050 / 2.0

    assert peg_center == pytest.approx(hole.entry_point()[2])


def test_unknown_fixture_is_rejected(fixtures):
    """조용히 `None` 을 돌려주면 좌표가 0 이 되어 팔이 원점으로 간다."""
    with pytest.raises(KeyError, match='없다'):
        fixtures['peg_hole_v2']


def test_missing_geometry_key_is_rejected(fixtures):
    """없는 값을 0 으로 떨어뜨리면 그것이 바닥 높이가 되어 팔이 탁자로 내려간다."""
    with pytest.raises(KeyError, match='geometry.rim_z'):
        fixtures['peg_hole'].absolute_z('rim_z')


def test_tilted_fixture_refuses_z_composition():
    """지금 고정물은 전부 `rpy=0` 이다. 기울어진 것이 생기면 z 만 더하는 것이 틀리므로,
    조용히 틀리는 대신 막는다."""
    tilted = Fixture({'name': 'x', 'pose': {'xyz': [0, 0, 0.1], 'rpy': [0, 0.3, 0]},
                      'geometry': {'entry_z': 0.02}}, 'panda_link0')

    with pytest.raises(NotImplementedError, match='기울어진'):
        tilted.entry_point()


def test_duplicate_names_are_rejected(tmp_path):
    """이름이 겹치면 뒤엣것이 조용히 이긴다."""
    path = tmp_path / 'dup.json'
    path.write_text(json.dumps({'frame': 'panda_link0', 'fixtures': [
        {'name': 'a', 'pose': {'xyz': [0, 0, 0]}},
        {'name': 'a', 'pose': {'xyz': [1, 0, 0]}}]}), encoding='utf-8')

    with pytest.raises(ValueError, match='중복'):
        load_fixtures_file(str(path))


def test_describe_mentions_every_fixture(fixtures):
    """`fb_fixtures.py` 가 이 출력을 쓴다 — 빠지면 사람이 못 본다."""
    text = describe('functionbay')

    for name in fixtures:
        assert name in text


# ----- 실효 바닥 — 틀리면 손끝이 고정물을 파고든다 ---------------------------

def test_top_z_is_the_entry_face(fixtures):
    """손끝이 닿는 면은 고정물 **입구면**이다 — 원점도 구멍 바닥도 아니다."""
    hole = fixtures['peg_hole']

    assert hole.top_z() == pytest.approx(hole.entry_point()[2])
    assert hole.top_z() > hole.floor_point()[2]


def test_covers_xy_uses_the_footprint(fixtures):
    hole = fixtures['peg_hole']
    x, y, _ = hole.entry_point()

    assert hole.covers_xy(x, y) is True
    assert hole.covers_xy(x + 0.5, y) is False


def test_unknown_footprint_is_none_not_false():
    """**"안 덮는다"와 "모른다"는 다르다.** 뭉개면 모르는 고정물 위로 손끝을 내린다.

    실제 파일 대신 **발자국 없는 고정물을 만들어** 본다 — 실 데이터로 시험하면 나중에
    그 값을 채우는 순간 이 시험이 거동이 아니라 데이터 상태 때문에 깨진다 (실제로
    2026-09-10 에 `peg_tray` 를 채우자 깨졌다).
    """
    bare = Fixture({'name': 'bare', 'pose': {'xyz': [1.0, 2.0, 0.0]},
                    'geometry': {'entry_z': 0.01}}, 'panda_link0')

    assert bare.value('outer_extent_xy') is None
    assert bare.covers_xy(1.0, 2.0) is None


def test_floor_at_returns_the_fixture_top_not_the_table(fixtures):
    """**핵심 회귀 시험.** 구멍에 꽂힌 peg 은 자기 바닥이 구멍 속(z=+0.003)이지만
    손끝이 실제로 닿는 면은 구멍 상면(z=+0.028)이다. 탁자면(0)을 쓰면 15 mm 파고든다.
    """
    x, y, entry_z = fixtures['peg_hole'].entry_point()
    floor_z, source, _ = fixtures.floor_at(x, y)

    assert floor_z == pytest.approx(entry_z)
    assert source == 'peg_hole'
    assert floor_z > 0.0, '탁자면으로 떨어지면 안 된다'


def test_floor_at_falls_back_to_the_table_away_from_fixtures(fixtures):
    floor_z, source, _ = fixtures.floor_at(0.0, 0.0)

    assert floor_z == pytest.approx(0.0)
    assert source is None


def test_floor_at_reports_fixtures_it_could_not_judge(tmp_path):
    """발자국을 모르는 고정물은 **이름으로 돌려준다** — 조용히 빠지면 바닥이 낙관적이 된다."""
    path = tmp_path / 'mixed.json'
    path.write_text(json.dumps({'fixtures': [
        {'name': 'known', 'pose': {'xyz': [0.0, 0.0, 0.0]},
         'geometry': {'entry_z': 0.02, 'outer_extent_xy': [0.04, 0.04]}},
        {'name': 'unmeasured', 'pose': {'xyz': [1.0, 0.0, 0.0]},
         'geometry': {'entry_z': 0.05}}]}), encoding='utf-8')
    mixed = load_fixtures_file(str(path))

    floor_z, source, unknown = mixed.floor_at(0.0, 0.0)

    assert floor_z == pytest.approx(0.02) and source == 'known'
    assert unknown == ['unmeasured']


def test_every_shipped_fixture_declares_its_footprint(fixtures):
    """**실 데이터는 전부 발자국을 갖고 있어야 한다.**

    하나라도 비면 그 위에서 파지할 때 `floor_at` 이 바닥을 탁자면으로 잡아 손끝이
    고정물을 파고든다 — 구멍 쪽에서 실제로 15 mm 파고들었다.
    """
    missing = [name for name in fixtures if fixtures[name].value('outer_extent_xy') is None]

    assert not missing, f'발자국이 없는 고정물: {missing} (메시 바운딩박스에서 잰다)'


# --- as_payload — 트윈 `fixtures` 변수가 싣는 값 (2026-09-11) ------------------


def test_payload_carries_absolute_points_not_relative_geometry(fixtures):
    """싣는 것은 **절대 좌표**다 — 부품 원점 기준값이 새어 나가면 안 된다."""
    payload = as_payload(fixtures)
    assert payload['frame'] == fixtures.frame
    hole = payload['fixtures']['peg_hole']
    assert hole['entry_point'] == pytest.approx({'x': 0.5, 'y': 0.0, 'z': 0.028})
    assert hole['floor_point'] == pytest.approx({'x': 0.5, 'y': 0.0, 'z': 0.003})
    assert hole['depth'] == pytest.approx(0.025)
    assert hole['inner_diameter'] == pytest.approx(0.017)


def test_payload_omits_what_is_not_known(tmp_path):
    """**모르는 값은 키 자체가 없다 — 0 으로 채우지 않는다.**

    바닥을 모르는데 0 을 넣으면 "바닥이 탁자면"이라는 거짓말이 되어 손끝이 그대로
    내려간다. 실제로 구멍 쪽에서 15 mm 파고든 적이 있다.

    ⚠️ **합성 고정물로 시험한다** — 실제 파일을 쓰면 나중에 그 값을 재서 채웠을 때
    시험이 깨진다. 그것은 동작이 아니라 **데이터 상태**를 시험하는 것이다
    (`peg_tray` 의 안쪽 형상이 2026-09-11 에 채워지며 실제로 그렇게 깨졌다).
    """
    path = tmp_path / 'partial.json'
    path.write_text(json.dumps({'frame': 'panda_link0', 'fixtures': [
        {'name': 'unmeasured', 'type': 'tray',
         'pose': {'xyz': [0.5, 0.0, 0.003]},
         'geometry': {'entry_z': 0.025, 'outer_extent_xy': [0.04, 0.04]}}]}),
        encoding='utf-8')

    item = as_payload(load_fixtures_file(str(path)))['fixtures']['unmeasured']

    assert 'entry_point' in item
    for key in ('floor_point', 'depth', 'inner_diameter'):
        assert key not in item, f'{key} 를 모르는데 실었다'


def test_every_shipped_fixture_that_can_be_inserted_into_declares_a_floor(fixtures):
    """꽂을 수 있다고 주장하는 자리는 **바닥을 알아야 한다.**

    입구만 알고 바닥을 모르면 얼마나 깊은지 모르는 채 내려가게 된다.
    """
    for name in fixtures:
        f = fixtures[name]
        if 'entry_z' in f.geometry and f.value('inner_diameter') is not None:
            assert 'floor_z' in f.geometry, f'{name}: 안지름은 아는데 바닥을 모른다'


def test_payload_is_json_serializable(fixtures):
    """REST 응답으로 나가므로 평범한 값만 담겨야 한다."""
    text = json.dumps(as_payload(fixtures))
    assert 'peg_hole' in text and 'peg_tray' in text


def test_payload_lists_every_fixture(fixtures):
    """목록이 곧 *"다른 hole"* 을 고를 후보다 — 하나라도 빠지면 안 보인다."""
    assert set(as_payload(fixtures)['fixtures']) == set(fixtures)
