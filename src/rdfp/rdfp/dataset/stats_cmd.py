"""`stats` 독립 CLI — DB 에 적재된 테이블별 행 수 출력.

이전에는 ``dataset stats`` 서브커맨드였으나, 별도 entry_point 로 분리되어
독립 console script (``stats``) 로 동작한다.

`REQUIRED_COLUMNS` 의 키를 정렬한 순서대로 ``SELECT COUNT(*)`` 를 수행하며,
조회에 실패한 테이블은 결과에서 제외한다 (DEBUG 로 사유 기록).

공개 엔트리:
    * :func:`main` — argparse 진입점 (setup.py 의 ``stats`` console_script).
    * :func:`cmd_stats` — 파싱된 ``args`` 와 확정된 ``config_path`` 로 실행.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys

from .cli_common import (
    DEFAULT_CONFIG_FILENAME, add_common_args, add_config_arg,
    configure_logging, load_dataset_or_fail, resolve_config_path,
)
from .db.connection import open_connection
from .db.schema_check import REQUIRED_COLUMNS


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='stats',
        description='Print DB row counts per table for the post-processed dataset.',
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
    return cmd_stats(args, config_path)


def cmd_stats(args: argparse.Namespace, config_path: str) -> int:
    """`stats` 명령: DB 에 적재된 테이블별 행 수를 출력한다."""
    cfg = load_dataset_or_fail(config_path)
    if cfg is None:
        return 2

    tables = sorted(REQUIRED_COLUMNS)
    stats: dict[str, int] = {}
    try:
        with open_connection(cfg.db) as conn:
            with conn.cursor() as cur:
                for t in tables:
                    try:
                        cur.execute(f'SELECT COUNT(*) FROM {t}')
                        stats[t] = int(cur.fetchone()[0])
                    except Exception as e:   # noqa: BLE001
                        logging.debug('skip %s: %s', t, e)
            conn.rollback()
    except Exception:
        logging.exception('failed to read db stats')
        return 1

    if args.format == 'json':
        print(json.dumps(stats, indent=2, ensure_ascii=False))
        return 0

    for t in tables:
        if t in stats:
            print(f'{t:<20} {stats[t]}')
    return 0


__all__ = ['cmd_stats', 'main']


if __name__ == '__main__':
    sys.exit(main())
