"""`mapping` 콘솔 스크립트 — 동적 매핑(JSONB 테이블) 조회 · 등록 · 해제 · 삭제.

```bash
ros2 run rdfp mapping list
ros2 run rdfp mapping register --type rdfp_msgs/msg/ClutchState --dry-run   # DDL 만 본다
ros2 run rdfp mapping register --type rdfp_msgs/msg/ClutchState [--table clutch_states]
ros2 run rdfp mapping unregister --type rdfp_msgs/msg/ClutchState --yes     # 데이터는 남는다
ros2 run rdfp mapping drop --type rdfp_msgs/msg/ClutchState --confirm clutch_states --yes
```

설계: `docs/collect/데이터셋_관리_GUI_설계안.md` §4.3. 데이터셋 GUI 의 [테이블 생성…] · [등록 해제] ·
[테이블 삭제] 가 이 명령을 서브프로세스로 부른다 — GUI 조작이 그대로 CLI 명령이 된다.

**등록 해제와 테이블 삭제는 따로다** (§4.3.2) — 합치면 "매핑을 끄려다 데이터를 지우는" 사고가 난다.
삭제는 테이블 이름을 `--confirm` 으로 다시 적어야 하고, 내장 테이블은 여기 없으므로 지울 수 없다.

메시지 정의는 bag 에서 읽는다(`rosbag_dir` 의 MCAP 에 박힌 `.msg` 텍스트) — ROS 를 요구하지 않는다.
다른 dataset CLI 와 같은 `*_cmd.py` 형제 모듈이며 `dataset <sub>` 디스패처를 되살리지 않는다.
"""

from __future__ import annotations

from typing import Optional, Sequence

import argparse
import json
import logging
import sys

from .cli_common import (DEFAULT_CONFIG_FILE_PATH, add_common_args, add_config_arg,
                         configure_logging, load_dataset_or_fail, resolve_config_path)

_logger = logging.getLogger(__name__)


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog='mapping',
                                description='List / register / unregister / drop dynamic '
                                            '(JSONB) topic tables.')
    add_common_args(p)
    sub = p.add_subparsers(dest='command')

    ls = sub.add_parser('list', help='dynamic mappings and their row counts')
    add_config_arg(ls, required=False)
    ls.add_argument('--format', choices=('text', 'json'), default='text')

    rg = sub.add_parser('register', help='create a JSONB table for a message type')
    add_config_arg(rg, required=False)
    rg.add_argument('--type', required=True, dest='type_name', metavar='TYPE')
    rg.add_argument('--table', help='table name (default: snake_case of the type, plural)')
    rg.add_argument('--rosbag-dir', help='where to read the .msg definition (default: config)')
    rg.add_argument('--dry-run', action='store_true', help='print the DDL and exit')

    ur = sub.add_parser('unregister', help='stop importing a type (table and rows are kept)')
    add_config_arg(ur, required=False)
    ur.add_argument('--type', required=True, dest='type_name', metavar='TYPE')
    ur.add_argument('--yes', action='store_true', help='non-interactive confirmation')

    dr = sub.add_parser('drop', help='DROP the dynamic table of a type (destructive)')
    add_config_arg(dr, required=False)
    dr.add_argument('--type', required=True, dest='type_name', metavar='TYPE')
    dr.add_argument('--confirm', metavar='TABLE', help='repeat the table name to confirm')
    dr.add_argument('--yes', action='store_true', help='non-interactive confirmation')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    configure_logging(args.log_level)
    if args.command is None:
        logging.error('no subcommand given (expected: list | register | unregister | drop)')
        return 2
    config_path = resolve_config_path(args)
    if config_path is None:
        logging.error('no --config given and default config not found: %s',
                      DEFAULT_CONFIG_FILE_PATH)
        return 2
    cfg = load_dataset_or_fail(config_path)
    if cfg is None:
        return 2
    handler = {'list': _cmd_list, 'register': _cmd_register, 'unregister': _cmd_unregister,
               'drop': _cmd_drop}[args.command]
    try:
        return handler(args, cfg)
    except Exception as exc:  # noqa: BLE001 — 드라이버 예외 종류가 많다; 메시지로 끝낸다
        from rdfp.dataset.db.dynamic import DynamicMappingError

        if isinstance(exc, DynamicMappingError):
            logging.error('%s', exc)
            return 1
        logging.exception('mapping %s failed', args.command)
        return 1


def _cmd_list(args: argparse.Namespace, cfg) -> int:
    from rdfp.dataset.db.connection import open_connection
    from rdfp.dataset.db.dynamic import load_mappings, row_count

    rows = []
    with open_connection(cfg.db) as conn:
        for m in load_mappings(conn, active_only=False).values():
            try:
                n: Optional[int] = row_count(conn, m.table_name)
            except Exception:  # noqa: BLE001 — 테이블이 없어졌으면 행 수만 모른다
                conn.rollback()
                n = None
            rows.append({'type': m.type_name, 'table': m.table_name, 'active': m.active,
                         'rows': n})
        conn.rollback()
    if args.format == 'json':
        print(json.dumps(rows, ensure_ascii=False, indent=2))
        return 0
    if not rows:
        print('(no dynamic mappings)')
        return 0
    w_type = max(len(r['type']) for r in rows)
    w_table = max(len(r['table']) for r in rows)
    for r in rows:
        state = 'active' if r['active'] else 'unregistered'
        rows_text = '?' if r['rows'] is None else f'{r["rows"]:,}'
        print(f'{r["type"]:<{w_type}}  {r["table"]:<{w_table}}  {state:<12}  {rows_text} rows')
    return 0


def find_schema_text(rosbag_dir: str, type_name: str) -> Optional[str]:
    """bag 에서 그 타입의 `.msg` 정의 텍스트를 찾는다 — 최근 세션부터 본다."""
    from .inspect.bag_inspect import read_schema_text, scan_sessions

    for session in reversed(scan_sessions(rosbag_dir)):
        for topic in session.topics:
            if topic.type_name != type_name:
                continue
            for path in session.mcap_paths:
                schema = read_schema_text(path, topic.name)
                if schema is not None:
                    return schema.text
    return None


def _cmd_register(args: argparse.Namespace, cfg) -> int:
    from rdfp.dataset.db.connection import open_connection
    from rdfp.dataset.db.dynamic import (create_table_sql, default_table_name, register,
                                         registrable, validate_table_name)

    reason = registrable(args.type_name)
    if reason:
        logging.error('%s', reason)
        return 1
    table = validate_table_name(args.table or default_table_name(args.type_name))
    rosbag_dir = args.rosbag_dir or cfg.rosbag_dir
    schema = find_schema_text(rosbag_dir, args.type_name)
    if schema is None:
        logging.error('no message definition for %s under %s — record it into a bag first',
                      args.type_name, rosbag_dir)
        return 1
    if args.dry_run:
        with open_connection(cfg.db) as conn:
            print(create_table_sql(table).as_string(conn))
            conn.rollback()
        return 0
    with open_connection(cfg.db) as conn:
        result = register(conn, args.type_name, table, schema)
        conn.commit()
    if result['created']:
        print(f'registered {args.type_name} -> {table} (table created)')
    elif result['reactivated']:
        print(f're-registered {args.type_name} -> {table} (existing rows kept)')
    else:
        print(f'{args.type_name} -> {table} is already registered')
    print('existing episodes get this topic on the next `import` (missing units are filled)')
    return 0


def _confirm(prompt: str, yes: bool) -> bool:
    if yes:
        return True
    if not sys.stdin.isatty():
        logging.error('refusing without --yes in a non-interactive shell')
        return False
    return input(prompt).strip().lower() in ('y', 'yes')


def _cmd_unregister(args: argparse.Namespace, cfg) -> int:
    from rdfp.dataset.db.connection import open_connection
    from rdfp.dataset.db.dynamic import unregister

    if not _confirm(f'unregister {args.type_name}? (table and rows are kept) [y/N] ', args.yes):
        return 1
    with open_connection(cfg.db) as conn:
        result = unregister(conn, args.type_name)
        conn.commit()
    print(f'unregistered {args.type_name}; table {result["table_name"]} and its rows are kept')
    return 0


def _cmd_drop(args: argparse.Namespace, cfg) -> int:
    from rdfp.dataset.db.connection import open_connection
    from rdfp.dataset.db.dynamic import DynamicMappingError, drop, load_mappings, row_count

    with open_connection(cfg.db) as conn:
        mapping = load_mappings(conn, active_only=False).get(args.type_name)
        if mapping is None:
            raise DynamicMappingError(f'{args.type_name} is not a dynamic mapping')
        rows = row_count(conn, mapping.table_name)
        print(f'{mapping.table_name}: {rows:,} row(s) will be deleted')
        # 테이블 이름을 다시 적게 한다 — 설계안 §6.6 의 '이름을 타이핑' 규칙.
        if args.confirm != mapping.table_name:
            logging.error('pass --confirm %s to drop it', mapping.table_name)
            return 2
        if not _confirm(f'DROP TABLE {mapping.table_name}? [y/N] ', args.yes):
            return 1
        result = drop(conn, args.type_name)
        conn.commit()
    print(f'dropped {result["table_name"]} ({result["deleted_rows"]:,} rows)')
    return 0


__all__ = ['find_schema_text', 'main']


if __name__ == '__main__':
    sys.exit(main())
