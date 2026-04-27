"""`init-db` 독립 CLI — DB 에 필요한 테이블/인덱스/FK 를 생성.

이전에는 ``dataset init-db`` 서브커맨드였으나, 별도 entry_point 로 분리되어
독립 console script (``init-db``) 로 동작한다.

DB 접속 정보 확정 순서:
    1. ``--config`` 이 명시되거나 cwd 의 ``dataset_config.yaml`` 이 존재
       하면 그 ``db`` 섹션을 사용.
    2. 그렇지 않으면 ``--dsn-env`` (기본 ``RDFP_DB_DSN``) + ``--schema``
       (기본 ``public``) 로 fallback.

``--drop`` 은 destructive 작업이므로 ``--yes`` 가 함께 있거나 대화형 셸
에서 ``YES`` 를 입력해야만 진행된다 (non-interactive 환경 + ``--yes`` 부재
시 거부).

공개 엔트리:
    * :func:`main` — argparse 진입점 (setup.py 의 ``init-db`` console_script).
    * :func:`cmd_init_db` — 파싱된 ``args`` 로 실행.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys

from .cli_common import (
    add_common_args, configure_logging, load_dataset_or_fail,
    resolve_config_path,
)
from .db.config import DbConfig, resolve_dsn
from .db.initialize import initialize_schema


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='init-db',
        description='Create required tables/indexes/FKs in the target database.',
    )
    add_common_args(p)
    p.add_argument('--config',
                   help='optional path to dataset_config.yaml (uses db.dsn_env + db.schema). '
                        '미지정 시 cwd 의 dataset_config.yaml 을 자동 사용한다.')
    p.add_argument('--dsn-env', default='RDFP_DB_DSN',
                   help='environment variable name holding the DSN '
                        '(used when --config is not given)')
    p.add_argument('--schema', default='public',
                   help='target schema name (used when --config is not given)')
    p.add_argument('--drop', action='store_true',
                   help='drop existing postproc tables before recreating '
                        '(destructive — requires --yes)')
    p.add_argument('--yes', action='store_true',
                   help='non-interactive confirmation for --drop')
    return p


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    configure_logging(args.log_level)
    return cmd_init_db(args)


def cmd_init_db(args: argparse.Namespace) -> int:
    """`init-db` 명령: schema.sql 을 대상 DB 에 적용한다."""
    config_path = resolve_config_path(args)
    if config_path is not None:
        cfg = load_dataset_or_fail(config_path)
        if cfg is None:
            return 2
        db_cfg = cfg.db
    else:
        db_cfg = DbConfig.model_validate({
            'dsn_env': args.dsn_env,
            'schema': args.schema,
        })

    try:
        dsn = resolve_dsn(db_cfg)
    except RuntimeError as e:
        logging.error('%s', e)
        return 2

    if args.drop and not args.yes:
        if not _confirm_destructive(db_cfg.schema_):
            logging.error('aborted by user')
            return 2

    try:
        initialize_schema(dsn, schema=db_cfg.schema_, drop_first=args.drop)
    except FileNotFoundError as e:
        logging.error('%s', e)
        return 1
    except Exception:
        logging.exception('failed to initialize database schema')
        return 1

    print(f'schema initialized (schema={db_cfg.schema_!r}, drop_first={args.drop})')
    return 0


def _confirm_destructive(schema: str) -> bool:
    """대화형으로 --drop 의도를 재확인한다."""
    if not sys.stdin.isatty() or os.environ.get('CI'):
        # non-interactive 환경에서는 --yes 없으면 거부한다.
        return False
    prompt = (
        f"This will DROP existing postproc tables in schema {schema!r}. "
        f"Type 'YES' to proceed: "
    )
    try:
        answer = input(prompt).strip()
    except EOFError:
        return False
    return answer == 'YES'


__all__ = ['cmd_init_db', 'main']


if __name__ == '__main__':
    sys.exit(main())
