#!/usr/bin/env python3
"""적재(`import`) 한 번의 결과를 점검한다 — 운영 가이드 §3.1 의 ①~⑤.

`import` 가 성공했다는 것과 학습에 쓸 데이터가 생겼다는 것은 다르다. 이 스크립트는
`import` 직전 · 직후 시각과 종료 코드 · 요약 JSON 을 받아 다음을 차례로 본다.

    ① 종료 코드와 요약(새로 적재 · 건너뜀 · 실패)
    ② 실행 로그에서 [시작, 끝] 구간의 episode_done / episode_skipped / episode_failed
    ③ 이번에 적재한 에피소드별 팔 관절 관측 행 수(`/joint_states` 토픽만)와 궤적 수
    ④ 에피소드별 영상 프레임 수
    ⑤ 등록된 mp4 파일이 실제로 있는가

종료 코드는 `import` 의 종료 코드를 그대로 돌려준다. 점검 자체가 실패하면(설정 · 실행
로그를 읽지 못함) `import` 가 성공했어도 0 이 아닌 값으로 끝난다. 같은 실행 로그에
적재 작업을 한 번에 하나씩 기록하는 경우를 기준으로 한다 — 시간 구간만으로는 동시에
실행한 두 적재를 구분하지 못한다.

사용법 (``.rdfprc`` 를 소싱한 셸 — ``rdfp`` 패키지를 import 한다)::

    SUMMARY=$(mktemp)
    START=$(date +%Y-%m-%dT%H:%M:%S.%6N%:z)
    ros2 run rdfp import --config "$CFG" > "$SUMMARY"; RC=$?
    END=$(date +%Y-%m-%dT%H:%M:%S.%6N%:z)
    ./scripts/check_import.py --config "$CFG" --since "$START" --until "$END" \\
        --rc "$RC" --summary "$SUMMARY"
"""
from __future__ import annotations

from typing import Optional

import argparse
import json
import os
import sys
from datetime import datetime


def _read_summary(path: str) -> dict:
    """요약 JSON 을 읽는다. 없거나 깨졌으면 빈 dict.

    순차 모드 실패 · 사전 점검 실패는 요약 없이 끝난다. 실패하면 `ros2 run` 이 같은 표준
    출력에 ``[ros2run]: Process exited with failure 1`` 을 덧붙이므로 앞의 JSON 객체만 읽는다.
    """
    try:
        with open(path, encoding='utf-8') as fp:
            obj, _ = json.JSONDecoder().raw_decode(fp.read().lstrip())
    except (OSError, ValueError):
        return {}
    return obj if isinstance(obj, dict) else {}


def _give_up(reason: str, rc: int) -> int:
    print(f'cannot start the episode check ({reason}) - see the stderr of import')
    return rc or 1


def check(cfg_path: str, since: datetime, until: datetime, rc: int, summary_path: str) -> int:
    """①~⑤ 를 출력하고 돌려줄 종료 코드를 정한다."""
    summary = _read_summary(summary_path)
    if summary:
        done_n, skip_n, fail_n = (summary.get(k, 0) for k in ('episodes', 'skipped', 'failed'))
        print(f'[1] exit code {rc} - imported {done_n}, already imported (skipped) {skip_n}, '
              f'failed {fail_n}')
        # 대상이 없으면 실행 로그도 생기지 않는다 (가이드 §9.2).
        if rc == 0 and done_n == skip_n == fail_n == 0:
            print('[1] nothing to import in this run - see the "0 episodes" trap in guide §10.2')
            return 0
    else:
        # 건수는 ② 의 로그 집계로 본다 — 0 으로 채우지 않는다.
        print(f'[1] exit code {rc} - no summary')

    try:
        from rdfp.dataset.config import load_dataset_config
        from rdfp.dataset.db.connection import open_connection
        cfg = load_dataset_config(cfg_path)
    except Exception as exc:     # noqa: BLE001 — 설정 · topics_file 오류는 import 도 같은 이유로 멈췄다
        return _give_up(f'cannot read config: {exc}', rc)
    if not cfg.output_mp4_dir:
        return _give_up('output_mp4_dir is not set in the config', rc)
    log = os.path.join(cfg.output_mp4_dir, '_logs', 'postproc_run.jsonl')
    if not os.path.isfile(log):
        return _give_up(f'run log not found: {log}', rc)

    with open(log, encoding='utf-8') as fp:
        recent = [r for r in map(json.loads, fp)
                  if since <= datetime.fromisoformat(r['ts']) <= until]
    failed = [r for r in recent if r['event'] == 'episode_failed']
    ids = [r['episode_id'] for r in recent if r['event'] == 'episode_done']
    skipped = sum(r['event'] == 'episode_skipped' for r in recent)
    print(f'[2] this run - imported {len(ids)}, skipped {skipped}, failed {len(failed)}')
    for r in failed:
        print(f'    failed {r["start_ts"]} {r["task_label"]}: {r["error"]}')
    if not ids:
        return rc                # 완료된 에피소드가 없다 — import 의 종료 코드를 그대로

    with open_connection(cfg.db) as conn:        # db.schema 를 search_path 로 건다
        rows = conn.execute(
            'select s.id, s.task_label,'
            ' (select count(*) from joint_states j join topics t on t.id = j.topic_id'
            "   where j.episode_id = s.id and t.topic_name = '/joint_states'),"
            ' (select count(*) from motion_trajectories m where m.episode_id = s.id),'
            ' (select count(*) from image_frames        f where f.episode_id = s.id)'
            ' from sessions s where s.id = any(%s) order by s.id', (ids,)).fetchall()
        streams = conn.execute('select episode_id, mp4_path from image_streams'
                               ' where episode_id = any(%s)', (ids,)).fetchall()
    for row in rows:
        print('[3][4] id={} task={} joints={} trajs={} frames={}'.format(*row))
    roots = cfg.mp4_roots()                      # recordings_dir → output_mp4_dir
    missing = [s for s in streams
               if not any(os.path.isfile(os.path.join(r, s[1])) for r in roots)]
    print(f'[5] video streams {len(streams)}, missing files {len(missing)}')
    for m in missing[:10]:
        print(f'    missing {m}')
    return rc                    # 점검 결과와 별개로 import 의 종료 코드를 유지한다


def main(argv: Optional[list] = None) -> int:
    p = argparse.ArgumentParser(description='Check the result of one dataset import run.')
    p.add_argument('--config', required=True, help='import 에 쓴 데이터셋 설정 YAML')
    p.add_argument('--since', required=True, help='import 직전 시각 (ISO 8601, 마이크로초까지)')
    p.add_argument('--until', required=True, help='import 직후 시각 (ISO 8601, 마이크로초까지)')
    p.add_argument('--rc', required=True, type=int, help='import 의 종료 코드')
    p.add_argument('--summary', required=True, help='import 의 표준 출력을 담은 파일')
    args = p.parse_args(argv)
    return check(args.config, datetime.fromisoformat(args.since),
                 datetime.fromisoformat(args.until), args.rc, args.summary)


if __name__ == '__main__':
    sys.exit(main())
