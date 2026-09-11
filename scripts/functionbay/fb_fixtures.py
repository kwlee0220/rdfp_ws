#!/usr/bin/env python3
"""고정물(구멍·트레이)의 위치와 형상을 찍는다 — **ROS 도 시뮬레이터도 필요 없다.**

    ./fb_fixtures.py            # 사람이 읽는 요약
    ./fb_fixtures.py --json     # 절대 좌표까지 계산해 JSON 으로

`/scene/objects` 에는 **움직이는 물체만** 실린다. 구멍·트레이는 시뮬레이터가
`static="true"` 라 TF 로도 안 내보내므로, 값은 백엔드 프로파일이 가리키는 고정물
정의 파일에서 온다 (`docs/gripper/…` 아님 — `scene.fixtures_file`).
"""
from __future__ import annotations

import json
import sys

from robot_control.scene.fixtures import describe, load_fixtures

BACKEND = 'functionbay'


def main() -> None:
    if '--json' in sys.argv:
        fixtures = load_fixtures(BACKEND)
        out = {'frame': fixtures.frame, 'source': fixtures.path, 'fixtures': {}}
        for name in sorted(fixtures):
            f = fixtures[name]
            item = {'type': f.type, 'source_body': f.source_body,
                    'origin': list(f.xyz), 'rpy': list(f.rpy),
                    'geometry': f.geometry}
            if 'entry_z' in f.geometry:
                item['entry_point'] = list(f.entry_point())
            if 'floor_z' in f.geometry:
                item['floor_point'] = list(f.floor_point())
                item['depth'] = f.depth()
            out['fixtures'][name] = item
        print(json.dumps(out, indent=2, ensure_ascii=False))
        return
    print(describe(BACKEND))


if __name__ == '__main__':
    main()
