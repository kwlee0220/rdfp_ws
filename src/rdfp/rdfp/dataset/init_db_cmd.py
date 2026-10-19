"""`init-db` 독립 CLI — DB 에 필요한 테이블/인덱스/FK 를 생성.

이전에는 ``dataset init-db`` 서브커맨드였으나, 별도 entry_point 로 분리되어
독립 console script (``init-db``) 로 동작한다.

DB 접속 정보 확정 순서 (``dsn_env`` · ``schema`` 를 항목마다 따로 정한다):
    1. 명시한 ``--dsn-env`` · ``--schema``.
    2. 설정 파일의 ``db`` 섹션 — ``--config`` 또는 기본 설정 파일
       (``$RDFP_HOME/config/dataset_config.yaml``, ``RDFP_HOME`` 이 없으면
       ``./config/dataset_config.yaml``).
    3. 내장 기본값 (``RDFP_DB_DSN`` / ``public``).

**명시한 인자가 설정을 이긴다.** 기본 설정 파일은 사람이 고른 것이 아니라 *찾아진*
것이다. 예전에는 그것이 있으면 ``--dsn-env`` · ``--schema`` 를 조용히 무시해,
워크스페이스 루트에서 ``init-db --schema rdfp_v1`` 이 ``public`` 에 테이블을
만들었다. 어느 대상에 적용하는지는 INFO 로그로 남긴다.

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
from .db.config import DEFAULT_DSN_ENV, DbConfig, resolve_dsn
from .db.initialize import initialize_schema


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='init-db',
        description='Create required tables/indexes/FKs in the target database.',
    )
    add_common_args(p)
    p.add_argument('--config',
                   help='path to dataset_config.yaml (uses db.dsn_env + db.schema). '
                        'If omitted, $RDFP_HOME/config/dataset_config.yaml '
                        '(./config/dataset_config.yaml without RDFP_HOME) is used when present.')
    # 기본값을 두지 않는다 — None 이어야 "명시하지 않았다"를 알고 설정 값을 쓸 수 있다.
    p.add_argument('--dsn-env', default=None,
                   help='environment variable name holding the DSN; overrides db.dsn_env '
                        f'(default: {DEFAULT_DSN_ENV} when no config file is found)')
    p.add_argument('--schema', default=None,
                   help='target schema name; overrides db.schema '
                        '(default: public when no config file is found)')
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
        db_cfg = DbConfig()
    # 명시한 인자는 어느 설정보다 앞선다 (모듈 docstring).
    overrides: dict[str, str] = {}
    if args.dsn_env is not None:
        overrides['dsn_env'] = args.dsn_env
    if args.schema is not None:
        overrides['schema_'] = args.schema
    if overrides:
        db_cfg = db_cfg.model_copy(update=overrides)
    logging.info('target: schema=%r, DSN from $%s (config: %s)',
                 db_cfg.schema_, db_cfg.dsn_env, config_path or 'none')

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
