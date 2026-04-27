"""`list` 독립 CLI — DB 의 sessions 테이블에 적재된 에피소드 목록 출력.

이전에는 ``dataset list`` 서브커맨드였으나, 별도 entry_point 로 분리되어
독립 console script (``list``) 로 동작한다.

`start_ts` 오름차순으로 정렬하며, 각 에피소드의 DB id / 시작 시각 /
지속 시간 / task_label 을 표시한다. text 포맷은 IDX/ID/START_TS/DUR_SEC/
TASK_LABEL 구성.

공개 엔트리:
    * :func:`main` — argparse 진입점 (setup.py 의 ``list`` console_script).
    * :func:`cmd_list` — 파싱된 ``args`` 와 확정된 ``config_path`` 로 실행.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from datetime import datetime

from .cli_common import (
    DEFAULT_CONFIG_FILENAME, add_common_args, add_config_arg,
    configure_logging, load_dataset_or_fail, resolve_config_path,
)
from .db.connection import open_connection


# text 포맷의 START_TS 컬럼 폭 (YYYY-MM-DD HH:MM:SS = 19 자).
_START_TS_WIDTH: int = 19


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='list',
        description='List episodes stored in the sessions table.',
    )
    add_common_args(p)
    add_config_arg(p, required=False)
    p.add_argument('--format', choices=['text', 'json'], default='text')
    return p


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    configure_logging(args.log_level)

    config_path = resolve_config_path(args)
    if config_path is None:
        logging.error(
            'no --config given and %s not found in the current working directory',
            DEFAULT_CONFIG_FILENAME)
        return 2
    return cmd_list(args, config_path)


def cmd_list(args: argparse.Namespace, config_path: str) -> int:
    """`list` 명령: DB 의 `sessions` 테이블에 적재된 에피소드 목록을 출력한다."""
    cfg = load_dataset_or_fail(config_path)
    if cfg is None:
        return 2

    episodes: list[dict] = []
    try:
        with open_connection(cfg.db) as conn:
            with conn.cursor() as cur:
                cur.execute(
                    'SELECT id, start_ts, stop_ts, task_label '
                    'FROM sessions ORDER BY start_ts ASC'
                )
                for row in cur.fetchall():
                    ep_id, start_ts, stop_ts, task_label = row
                    duration = (stop_ts - start_ts).total_seconds()
                    episodes.append({
                        'id': int(ep_id),
                        'start_ts': start_ts.isoformat(),
                        'stop_ts': stop_ts.isoformat(),
                        'duration_sec': round(duration, 3),
                        'task_label': task_label,
                    })
            conn.rollback()
    except Exception:
        logging.exception('failed to list episodes')
        return 1

    if args.format == 'json':
        print(json.dumps(
            {'episode_count': len(episodes), 'episodes': episodes},
            indent=2, ensure_ascii=False,
        ))
        return 0

    print(f'total {len(episodes)} episode(s)')
    if not episodes:
        return 0

    # 컬럼 폭 동적 계산 (ID 는 실제 값 폭, TASK_LABEL 은 헤더 폭을 최소로).
    id_w = max(len('ID'), max(len(str(e['id'])) for e in episodes))
    header = (
        f'{"IDX":>4}  {"ID":>{id_w}}  {"START_TS":<{_START_TS_WIDTH}}  '
        f'{"DUR_SEC":>10}  TASK_LABEL'
    )
    print(header)
    for idx, e in enumerate(episodes):
        # ISO 문자열의 소수점 이하를 절사해 "YYYY-MM-DD HH:MM:SS" 로 정렬.
        start_ts_str = datetime.fromisoformat(e['start_ts']).strftime('%Y-%m-%d %H:%M:%S')
        print(
            f'{idx:>4}  {e["id"]:>{id_w}}  {start_ts_str:<{_START_TS_WIDTH}}  '
            f'{e["duration_sec"]:>10.3f}  {e["task_label"] or ""}'
        )
    return 0


__all__ = ['cmd_list', 'main']


if __name__ == '__main__':
    sys.exit(main())
