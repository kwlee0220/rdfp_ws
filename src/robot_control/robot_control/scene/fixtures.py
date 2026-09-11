#!/usr/bin/env python3

"""고정물(fixture) — **TF 에 없는, 안 움직이는 작업 대상물**의 위치와 형상.

    from robot_control.scene.fixtures import load_fixtures

    fixtures = load_fixtures('functionbay')
    hole = fixtures['peg_hole']
    x, y, z = hole.entry_point()        # 구멍 입구 (panda_link0 기준)

**왜 `/scene/objects` 가 아닌가.** 그 채널은 **조작 대상 전용**이고 `SceneObject` 에는
고정물을 구분할 필드가 없다 (`fixture` 는 2026-09-01 에 삭제됐다). 여기 것을 실으면
모든 소비자에게 peg 과 똑같이 보인다. 게다가 값이 상수라 에피소드마다 기록할 이유가
없다 — 매 프레임 같은 숫자를 흘릴 뿐이다.

**왜 TF 가 아닌가.** 고정 프레임이므로 `/tf_static` 이 제자리처럼 보이지만, 그 토픽은
녹화 목록에 없어 **데이터셋에 안 남는다.** 넣으려면 로봇 링크 13개가 딸려 온다.

**이 모듈은 `rclpy` 를 import 하지 않는다** — 값만 읽는 도구와 launch 에서도 쓸 수 있어야
한다. 설계 근거: `docs/scene/scene_objects_guide.md`, 측정 출처는 JSON 의 `_comment`.
"""

from __future__ import annotations

from typing import Optional

import json
import math
import os

from robot_control.backend_profiles import BACKEND_PROFILES


class Fixture:
    """고정물 하나. **`geometry` 값은 부품 원점 기준 상대값**이라 여기서 합성한다."""

    def __init__(self, entry: dict, frame: str) -> None:
        self.name: str = entry['name']
        self.type: str = entry.get('type', '')
        self.source_body: Optional[str] = entry.get('source_body')
        self.frame = frame
        pose = entry.get('pose') or {}
        self.xyz: tuple = tuple(float(v) for v in pose.get('xyz', (0.0, 0.0, 0.0)))
        self.rpy: tuple = tuple(float(v) for v in pose.get('rpy', (0.0, 0.0, 0.0)))
        self._geometry: dict = {k: v for k, v in (entry.get('geometry') or {}).items()
                                if not k.startswith('_')}

    def __repr__(self) -> str:
        return f'Fixture({self.name!r}, xyz={self.xyz})'

    @property
    def geometry(self) -> dict:
        """부품 원점 기준 형상값. **절대 좌표가 아니다** — `absolute_z()` 로 옮긴다."""
        return dict(self._geometry)

    def value(self, key: str, default=None):
        return self._geometry.get(key, default)

    def absolute_z(self, key: str) -> float:
        """`geometry` 의 z 값을 `frame` 기준 절대 높이로.

        Raises:
            KeyError: 그 키가 없을 때. **0 으로 떨어뜨리지 않는다** — 없는 값을 바닥
                높이로 쓰면 팔이 탁자로 내려간다.
        """
        if key not in self._geometry:
            raise KeyError(f'{self.name}: geometry.{key} 가 없다 '
                           f'(있는 것: {sorted(self._geometry)})')
        if any(abs(angle) > 1e-9 for angle in self.rpy):
            # 지금 고정물은 전부 rpy=0 이다. 기울어진 것이 생기면 z 만 더하는 것이
            # 틀리므로, 조용히 틀리는 대신 여기서 막는다.
            raise NotImplementedError(
                f'{self.name}: rpy={self.rpy} — 기울어진 고정물의 z 합성은 미구현이다')
        return self.xyz[2] + float(self._geometry[key])

    def entry_point(self) -> tuple:
        """구멍/트레이 **입구의 중심** `(x, y, z)` — `frame` 기준."""
        return (self.xyz[0], self.xyz[1], self.absolute_z('entry_z'))

    def floor_point(self) -> tuple:
        """구멍 **바닥의 중심** `(x, y, z)` — `frame` 기준."""
        return (self.xyz[0], self.xyz[1], self.absolute_z('floor_z'))

    def depth(self) -> float:
        """입구에서 바닥까지 (m)."""
        return self.absolute_z('entry_z') - self.absolute_z('floor_z')

    def radial_clearance(self, peg_diameter: float) -> float:
        """peg 지름을 주면 **한쪽 반경 여유** (m). 음수면 안 들어간다."""
        inner = self.value('inner_diameter')
        if inner is None:
            raise KeyError(f'{self.name}: geometry.inner_diameter 가 없다')
        return (float(inner) - float(peg_diameter)) / 2.0

    def top_z(self) -> float:
        """**손끝이 닿는 윗면**의 높이 — 구멍·트레이는 입구면이다.

        물체가 이 고정물에 꽂혀 있으면 물체의 바닥은 고정물 **속**에 있으므로, 손끝이
        내려갈 수 있는 한계는 물체 바닥이 아니라 이 면이다.
        """
        return self.absolute_z('entry_z')

    def covers_xy(self, x: float, y: float) -> Optional[bool]:
        """이 고정물이 `(x, y)` 를 덮는가. **발자국을 모르면 `None`** 이다.

        `None` 을 `False` 로 뭉개면 안 된다 — "안 덮는다"와 "모른다"는 다르고, 뒤를
        모르는 채로 손끝을 내리면 그대로 부딪힌다. 호출자가 경고할 수 있게 구분한다.
        """
        extent = self.value('outer_extent_xy')
        if extent is None:
            return None
        half_x, half_y = float(extent[0]) / 2.0, float(extent[1]) / 2.0
        return abs(x - self.xyz[0]) <= half_x and abs(y - self.xyz[1]) <= half_y


class FixtureSet(dict):
    """이름 → :class:`Fixture`. 모르는 이름은 **거부한다.**"""

    def __init__(self, fixtures: dict, frame: str, path: str) -> None:
        super().__init__(fixtures)
        self.frame = frame
        self.path = path

    def __missing__(self, key):
        raise KeyError(f'{key!r} 라는 고정물이 없다 — {self.path} 에 있는 것: '
                       f'{sorted(self)}')

    def floor_at(self, x: float, y: float, table_z: float = 0.0) -> tuple:
        """`(x, y)` 에서 **손끝이 닿는 실효 바닥** — `(높이, 근거 이름, 판정불가 목록)`.

        탁자면(`table_z`)에서 시작해 그 점을 덮는 고정물의 윗면 중 가장 높은 것을 쓴다.
        **물체 자체의 바닥으로 판단하면 안 된다** — 구멍에 꽂힌 peg 은 바닥이 구멍 속
        (z=+0.003)이지만 손끝이 실제로 닿는 면은 구멍 상면(z=+0.028)이라, 그대로 내리면
        칼라를 15 mm 파고든다 (2026-09-10 실측).

        세 번째 값은 **발자국을 몰라 판정하지 못한 고정물** 이름들이다. 비어 있지 않으면
        반환된 바닥이 낙관적일 수 있으므로 호출자가 경고한다.
        """
        floor_z, source, unknown = float(table_z), None, []
        for fixture in self.values():
            covered = fixture.covers_xy(x, y)
            if covered is None:
                unknown.append(fixture.name)
                continue
            if covered and fixture.top_z() > floor_z:
                floor_z, source = fixture.top_z(), fixture.name
        return floor_z, source, unknown


def as_payload(fixtures: 'FixtureSet') -> dict:
    """`FixtureSet` 을 **JSON 으로 나갈 수 있는 평범한 값**으로 편다.

    트윈의 `fixtures` 상태 변수가 이것을 그대로 싣는다. 여기 두는 것은 좌표 합성을
    아는 쪽이 이 모듈이기 때문이다 — 트윈이 `geometry` 를 직접 읽으면 부품 원점
    기준값을 절대 좌표로 착각하게 된다.

    **모르는 값은 키를 뺀다 — `null` 이나 0 을 넣지 않는다.** `peg_tray` 는 안쪽을
    아직 안 재서 `floor_point` · `depth` · `inner_diameter` 가 없는데, 0 으로 채우면
    "바닥이 탁자면"이라는 **거짓말**이 되어 손끝이 그대로 내려간다. 없는 키를 만난
    호출자는 모른다는 것을 알게 된다.
    """
    entries: dict = {}
    for name in sorted(fixtures):
        f = fixtures[name]
        item: dict = {'name': f.name, 'type': f.type,
                      'position': dict(zip('xyz', f.xyz))}
        if f.source_body:
            item['source_body'] = f.source_body
        if 'entry_z' in f.geometry:
            item['entry_point'] = dict(zip('xyz', f.entry_point()))
        if 'floor_z' in f.geometry:
            item['floor_point'] = dict(zip('xyz', f.floor_point()))
            if 'entry_z' in f.geometry:
                item['depth'] = f.depth()
        if f.value('inner_diameter') is not None:
            item['inner_diameter'] = float(f.value('inner_diameter'))
        if f.value('outer_extent_xy') is not None:
            item['outer_extent_xy'] = [float(v) for v in f.value('outer_extent_xy')]
        entries[name] = item
    return {'frame': fixtures.frame, 'fixtures': entries}


def fixtures_path(backend: str) -> Optional[str]:
    """백엔드 프로파일의 `scene.fixtures_file` 을 절대 경로로. 없으면 `None`."""
    from robot_control.backends import get_backend

    return get_backend(backend).share_path(
        (BACKEND_PROFILES[backend].get('scene') or {}).get('fixtures_file'))


def load_fixtures(backend: str) -> FixtureSet:
    """백엔드의 고정물 정의를 읽는다.

    Raises:
        FileNotFoundError: 프로파일에 `scene.fixtures_file` 이 없거나 파일이 없을 때.
            **빈 집합을 돌려주지 않는다** — 그러면 "고정물이 없는 백엔드"와 "설정이
            빠진 백엔드"가 구별되지 않고, 증상이 `KeyError` 하나로 뭉개진다.
    """
    path = fixtures_path(backend)
    if not path:
        raise FileNotFoundError(
            f'{backend}: 프로파일에 scene.fixtures_file 이 없다 — 고정물 정의가 없는 '
            '백엔드이거나 아직 안 적은 것이다')
    if not os.path.isfile(path):
        raise FileNotFoundError(f'{backend}: {path} 가 없다 (colcon build 를 다시 돌렸나?)')
    return load_fixtures_file(path)


def load_fixtures_file(path: str) -> FixtureSet:
    """정의 파일 하나를 읽는다. 프로파일을 거치지 않으므로 테스트·도구가 쓴다."""
    with open(path, encoding='utf-8') as handle:
        raw = json.load(handle)
    frame = raw.get('frame', 'panda_link0')
    entries: dict = {}
    for entry in raw.get('fixtures', []):
        fixture = Fixture(entry, frame)
        if fixture.name in entries:
            raise ValueError(f'{path}: 고정물 이름이 중복이다 — {fixture.name!r}')
        entries[fixture.name] = fixture
    return FixtureSet(entries, frame, path)


def describe(backend: str) -> str:
    """사람이 읽을 요약. `fb_fixtures.py` 와 테스트가 쓴다."""
    fixtures = load_fixtures(backend)
    lines = [f'{backend} 고정물 ({fixtures.frame} 기준) — {fixtures.path}']
    for name in sorted(fixtures):
        f = fixtures[name]
        lines.append(f'  {name} ({f.type}, body={f.source_body})')
        lines.append(f'    원점  ({f.xyz[0]:+.4f}, {f.xyz[1]:+.4f}, {f.xyz[2]:+.4f})'
                     f'  rpy {tuple(round(math.degrees(a), 1) for a in f.rpy)}°')
        if 'entry_z' in f.geometry:
            x, y, z = f.entry_point()
            lines.append(f'    입구  ({x:+.4f}, {y:+.4f}, {z:+.4f})')
        if 'floor_z' in f.geometry:
            x, y, z = f.floor_point()
            lines.append(f'    바닥  ({x:+.4f}, {y:+.4f}, {z:+.4f})   깊이 '
                         f'{f.depth() * 1000:.1f} mm')
        if f.value('inner_diameter') is not None:
            lines.append(f'    안지름 {f.value("inner_diameter") * 1000:.2f} mm')
    return '\n'.join(lines)


__all__ = ['Fixture', 'FixtureSet', 'as_payload', 'describe', 'fixtures_path',
           'load_fixtures', 'load_fixtures_file']
