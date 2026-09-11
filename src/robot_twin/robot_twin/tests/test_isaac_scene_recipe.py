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

# 테이블 상단 높이. 중심 z + 두께/2 로 나오며, 물체는 그 위에 반높이만큼 얹힌다.
TABLE_TOP_Z = 0.4


def _half_height(spec) -> float:
    """안착 z 를 정하는 반높이. **`size` 순서가 타입마다 다르다.**"""
    kind = spec.get('type', 'box')
    if kind == 'box':
        return float(spec['size'][2]) / 2.0          # [x, y, z]
    if kind == 'cylinder':
        return float(spec['size'][0]) / 2.0          # [높이, 반지름]
    raise AssertionError(f'{spec["name"]}: unsupported type {kind!r}')


def _footprint_half(spec) -> tuple:
    """탁자 위에서 차지하는 x·y 반폭.

    실린더는 **반지름 하나가 x·y 양쪽 반폭**이다 — `size[1]` 을 y 전용으로 쓰면
    폭을 절반으로 잘못 보고, 그러면 겹침·이탈 검사가 통과해 버린다.
    """
    kind = spec.get('type', 'box')
    if kind == 'box':
        return float(spec['size'][0]) / 2.0, float(spec['size'][1]) / 2.0
    if kind == 'cylinder':
        r = float(spec['size'][1])
        return r, r
    raise AssertionError(f'{spec["name"]}: unsupported type {kind!r}')


def _span(raw) -> tuple:
    """축 값이 숫자(고정)면 폭 0 인 구간으로, [최소, 최대] 면 그대로."""
    if isinstance(raw, (list, tuple)):
        return float(raw[0]), float(raw[1])
    return float(raw), float(raw)


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


def test_recipe_z_keeps_objects_on_the_table():
    """z 를 흔들면 물체가 공중에서 떨어지거나 테이블에 박힌다.

    테이블 상단(0.4)에 **반높이만큼** 얹힌다 — 5 cm 블록은 0.425, 높이 6 cm 실린더는
    0.43 이다. 범위가 아니라 **고정값**이어야 하는 유일한 축이다.
    """
    config, _ = _load()
    for scene_name, recipe in _recipes(config).items():
        for spec in recipe['objects']:
            z = spec['z']
            assert isinstance(z, (int, float)), (
                f'{scene_name}/{spec["name"]}: z must be fixed, got {z!r}')
            want = TABLE_TOP_Z + _half_height(spec)
            assert float(z) == pytest.approx(want), (
                f'{scene_name}/{spec["name"]}: z={z} but resting on the table '
                f'({TABLE_TOP_Z}) would be {want}')


def test_sampled_blocks_do_not_overlap():
    """5 cm 블록끼리 파고들면 물리가 밀어내 배치가 레시피와 달라진다.

    각 물체의 x·y 범위를 **최악으로** 잡아(서로 마주보는 끝) 겹치는지 본다. 한 번
    뽑아 보는 것으로는 운 좋게 통과할 수 있다.
    """
    config, _ = _load()
    for scene_name, recipe in _recipes(config).items():
        specs = recipe['objects']
        for i, a in enumerate(specs):
            for b in specs[i + 1:]:
                a_half = _footprint_half(a)
                b_half = _footprint_half(b)
                for axis, k in (('x', 0), ('y', 1)):
                    need = a_half[k] + b_half[k]
                    a_lo, a_hi = _span(a[axis])
                    b_lo, b_hi = _span(b[axis])
                    # 두 구간이 가장 가까워지는 거리. 겹치면 0 이다.
                    gap = max(0.0, max(a_lo - b_hi, b_lo - a_hi))
                    if gap >= need:
                        break       # 이 축만으로 떨어져 있으면 충분하다
                else:
                    raise AssertionError(
                        f'{scene_name}: {a["name"]} and {b["name"]} can overlap '
                        f'on every axis')


def test_sampled_blocks_stay_on_the_table_footprint():
    """x·y 를 흔들면 블록이 테이블 밖으로 나가 바닥으로 떨어진다.

    z 는 위에서 고정값으로 묶었지만 **x·y 는 범위 추출이라 최악값이 테이블을 넘을 수
    있다.** 넘으면 물리가 블록을 떨어뜨리고, `/scene/objects` 는 레시피와 다른 자리를
    말한다 — 실패가 배치 시점이 아니라 파지 시점에 드러난다.

    테이블 자체는 `dynamic: false` 라 `_load()` 의 stage 에 없다. 여기서만 필요하므로
    JSON 에서 직접 읽는다.
    """
    config, _ = _load()
    with open(_isaac_scene_json(), encoding='utf-8') as handle:
        scene = json.load(handle)
    table = next(o for o in scene['objects'] if o['name'] == 'table')
    tx, ty, _tz = table['position']
    tdx, tdy, _tdz = table['dimensions']
    bounds = {'x': (tx - tdx / 2.0, tx + tdx / 2.0), 'y': (ty - tdy / 2.0, ty + tdy / 2.0)}

    for scene_name, recipe in _recipes(config).items():
        for spec in recipe['objects']:
            for axis, k in (('x', 0), ('y', 1)):
                half = _footprint_half(spec)[k]
                lo, hi = _span(spec[axis])
                low, high = bounds[axis]
                assert lo - half >= low and hi + half <= high, (
                    f'{scene_name}/{spec["name"]}: {axis} range '
                    f'[{lo - half:.3f}, {hi + half:.3f}] leaves the table '
                    f'[{low:.3f}, {high:.3f}]')
