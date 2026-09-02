"""Isaac 트윈 설정의 `reset_scene` 레시피가 `isaac_scene.json` 과 어긋나지 않는가.

**두 곳에 적힌 값이라 조용히 갈라진다.** 트윈은 레시피대로 배치를 뽑아 `/scene/reset`
으로 보내고, `isaac_scene_state_node` 는 `isaac_scene.json` 과 대조해 **거부**한다 —
Isaac 은 물체를 만들거나 기하를 바꿀 수 없기 때문이다
(`docs/scene/isaac_scene_reset.md` §3).

거부는 런타임에만 드러나고, 그때는 수집 루프가 이미 돌고 있다. 그래서 여기서 묶는다.

`isaac_scene.json` 은 `robot_control` 패키지에 있으므로 소스 트리에서만 이 검사가
의미가 있다 — 없으면 건너뛴다.
"""

from __future__ import annotations

import json
import pathlib

import pytest

# 테이블 상단 높이. 중심 z + 두께/2 로 나오며, 블록은 그 위에 반높이만큼 얹힌다.
TABLE_TOP_Z = 0.4
BLOCK_REST_Z = 0.425


def _twin_config_dir() -> pathlib.Path:
    # robot_twin/robot_twin/tests/<this> -> src/robot_twin/config
    return pathlib.Path(__file__).resolve().parents[2] / 'config'


def _isaac_scene_json() -> pathlib.Path:
    # src/robot_twin/robot_twin/tests/<this> -> src/robot_control/config/isaac_scene.json
    src = pathlib.Path(__file__).resolve().parents[3]
    return src / 'robot_control' / 'config' / 'isaac_scene.json'


def _load() -> tuple:
    from robot_twin.config import load_config

    twin_path = _twin_config_dir() / 'robot_twin_panda_isaac.yaml'
    scene_path = _isaac_scene_json()
    if not twin_path.is_file() or not scene_path.is_file():
        pytest.skip('배포 설정이 없는 트리 (installed)')

    config = load_config(str(twin_path))
    with open(scene_path, encoding='utf-8') as handle:
        scene = json.load(handle)
    # 발행·리셋 대상은 `dynamic: true` 인 것뿐이다.
    stage = {o['name']: o for o in scene['objects'] if o.get('dynamic', False)}
    return config, stage


def _recipes(config) -> dict:
    from robot_twin.backends import scene_recipes

    return scene_recipes(config.operation('reset_scene'))


def test_isaac_config_exposes_reset_scene():
    """노드가 `/scene/reset` 을 여는데 연산이 없으면 수집 루프가 배치를 못 바꾼다."""
    config, _ = _load()
    op = config.operation('reset_scene')
    assert op is not None
    assert op.backend.get('service') == '/scene/reset'
    assert op.backend.get('service_type') == 'rdfp_msgs/srv/ResetScene'


def test_every_recipe_names_only_stage_objects():
    """Isaac 은 물체를 **만들 수 없다** — 스테이지에 없는 이름은 거부된다."""
    config, stage = _load()
    for scene_name, recipe in _recipes(config).items():
        for spec in recipe['objects']:
            assert spec['name'] in stage, (
                f"scene {scene_name!r}: {spec['name']!r} is not in isaac_scene.json "
                f'(dynamic objects: {sorted(stage)})')


def test_every_recipe_places_all_stage_objects():
    """빠뜨린 물체는 **이전 에피소드가 남긴 자리**에 그대로 있어 배치가 새어 나간다."""
    config, stage = _load()
    for scene_name, recipe in _recipes(config).items():
        named = {spec['name'] for spec in recipe['objects']}
        assert named == set(stage), (
            f'scene {scene_name!r} places {sorted(named)}, '
            f'stage has {sorted(stage)}')


def test_recipe_geometry_matches_the_stage():
    """기하가 다르면 노드가 거부한다 — Isaac 은 크기를 바꿀 수 없다."""
    config, stage = _load()
    for scene_name, recipe in _recipes(config).items():
        for spec in recipe['objects']:
            want = stage[spec['name']]
            assert spec['type'] == want['type'], f'{scene_name}/{spec["name"]}: type'
            size = [float(v) for v in spec['size']]
            dims = [float(v) for v in want['dimensions']]
            assert size == pytest.approx(dims), f'{scene_name}/{spec["name"]}: size'


def test_recipe_z_keeps_blocks_on_the_table():
    """z 를 흔들면 블록이 공중에서 떨어지거나 테이블에 박힌다.

    5 cm 블록은 테이블 상단(0.4)에 반높이만큼 얹혀 0.425 다. 범위가 아니라 **고정값**
    이어야 하는 유일한 축이다.
    """
    config, _ = _load()
    for scene_name, recipe in _recipes(config).items():
        for spec in recipe['objects']:
            z = spec['z']
            assert isinstance(z, (int, float)), (
                f'{scene_name}/{spec["name"]}: z must be fixed, got {z!r}')
            assert float(z) == pytest.approx(BLOCK_REST_Z), (
                f'{scene_name}/{spec["name"]}: z={z} but the table top is {TABLE_TOP_Z}')


def test_sampled_blocks_do_not_overlap():
    """5 cm 블록끼리 파고들면 물리가 밀어내 배치가 레시피와 달라진다.

    각 물체의 x·y 범위를 **최악으로** 잡아(서로 마주보는 끝) 겹치는지 본다. 한 번
    뽑아 보는 것으로는 운 좋게 통과할 수 있다.
    """
    config, _ = _load()

    def _span(raw):
        if isinstance(raw, (list, tuple)):
            return float(raw[0]), float(raw[1])
        return float(raw), float(raw)

    for scene_name, recipe in _recipes(config).items():
        specs = recipe['objects']
        for i, a in enumerate(specs):
            for b in specs[i + 1:]:
                half = (float(a['size'][0]) + float(b['size'][0])) / 2.0
                gaps = []
                for axis in ('x', 'y'):
                    a_lo, a_hi = _span(a[axis])
                    b_lo, b_hi = _span(b[axis])
                    # 두 구간이 가장 가까워지는 거리. 겹치면 0 이다.
                    gaps.append(max(0.0, max(a_lo - b_hi, b_lo - a_hi)))
                assert max(gaps) >= half, (
                    f'{scene_name}: {a["name"]} and {b["name"]} can overlap '
                    f'(worst-case gap {max(gaps):.3f} m < {half:.3f} m)')
