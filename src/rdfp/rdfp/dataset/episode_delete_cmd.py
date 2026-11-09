"""`episode-delete` 콘솔 스크립트 — DB 의 에피소드를 지운다 (운영 가이드 §8.6 의 SQL 을 명령으로).

```bash
ros2 run rdfp episode-delete --config config/dataset_config.yaml --ids 11,12-13 --dry-run
ros2 run rdfp episode-delete --ids 42 --delete-files --yes
```

지우기 **전에** 영상 경로를 뽑아 보인다 — 행을 지운 뒤에는 어느 파일이 그 에피소드 것이었는지 DB 로
알 수 없다. `--delete-files` 를 주면 그 파일(레코더 mp4 · `.jsonl` sidecar · `_metadata.json`,
bag 인코딩 `episode_<id>/` 디렉터리)도 함께 지운다. bag 에서 에피소드를 빼는 것은 `rosbag delete`
의 몫이다.

확인 규칙은 다른 파괴적 명령과 같다 — 비대화형에서는 `--yes` 가 필수고, 대화형에서는 `delete` 를
타이핑해야 진행한다 (설계안 §6.6). 다른 dataset CLI 와 같은 `*_cmd.py` 형제 모듈이며 `dataset <sub>`
디스패처를 되살리지 않는다.
"""

from __future__ import annotations

from typing import Optional, Sequence

import argparse
import logging
import shutil
import sys
from datetime import datetime
from pathlib import Path

from .cli_common import (DEFAULT_CONFIG_FILE_PATH, add_common_args, add_config_arg,
                         configure_logging, load_dataset_or_fail, resolve_config_path)
from .db.connection import open_connection
from .inspect.db_inspect import (EpisodeSummary, delete_episodes, episode_summaries,
                                 recording_files_for)

_logger = logging.getLogger(__name__)

#: 대화형 확인에서 타이핑해야 하는 단어.
CONFIRM_WORD = 'delete'


def parse_id_spec(spec: str) -> list[int]:
    """`11,12-13` → `[11, 12, 13]`. 중복은 하나로, 순서는 오름차순."""
    out: set = set()
    for piece in spec.split(','):
        piece = piece.strip()
        if not piece:
            continue
        if '-' in piece:
            lo_s, _, hi_s = piece.partition('-')
            lo, hi = int(lo_s), int(hi_s)
            if lo > hi:
                raise ValueError(f'invalid range: {piece}')
            out.update(range(lo, hi + 1))
        else:
            out.add(int(piece))
    return sorted(out)


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='episode-delete',
        description='Delete episodes (sessions rows + cascaded children) from the database.')
    add_common_args(p)
    add_config_arg(p, required=False)
    p.add_argument('--ids', required=True, metavar='SPEC',
                   help="에피소드 id (예: '11,12-13'). `ros2 run rdfp list` 의 ID 열이다")
    p.add_argument('--delete-files', action='store_true',
                   help='DB 행과 함께 영상 파일(레코더 mp4 · sidecar · metadata, bag 인코딩 디렉터리)도 지운다')
    p.add_argument('--yes', action='store_true', help='비대화형 확인 (non-TTY 에서는 필수)')
    p.add_argument('--dry-run', action='store_true', help='지울 것만 보이고 끝낸다')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    configure_logging(args.log_level)
    config_path = resolve_config_path(args)
    if config_path is None:
        logging.error('no --config given and default config not found: %s',
                      DEFAULT_CONFIG_FILE_PATH)
        return 2
    return cmd_episode_delete(args, config_path)


def _fmt_time(ns: int) -> str:
    return datetime.fromtimestamp(ns / 1e9).strftime('%Y-%m-%d %H:%M:%S')


def _print_plan(targets: Sequence[EpisodeSummary], files: Sequence[Path]) -> None:
    print(f'{"id":>6}  {"start":<19}  {"dur":>7}  {"rows":>8}  {"frames":>7}  task')
    for s in targets:
        print(f'{s.id:>6}  {_fmt_time(s.start_ns):<19}  {s.duration_sec:>6.1f}s  '
              f'{s.message_rows:>8,}  {s.frames:>7,}  {s.task_label or "-"}')
    print(f'\n{len(targets)} episode(s); child rows go with them (ON DELETE CASCADE).')
    if files:
        print(f'{len(files)} file(s) on disk belong to these episodes:')
        for f in files:
            print(f'  {f}')
    else:
        print('no video files found on disk for these episodes.')


def _confirm(count: int, yes: bool) -> bool:
    if yes:
        return True
    if not sys.stdin.isatty():
        logging.error('refusing to delete without --yes in a non-interactive shell')
        return False
    reply = input(f"type '{CONFIRM_WORD}' to remove {count} episode(s) from the database: ")
    return reply.strip() == CONFIRM_WORD


def _remove(path: Path) -> None:
    if path.is_dir():
        shutil.rmtree(path)
    else:
        path.unlink()


def cmd_episode_delete(args: argparse.Namespace, config_path: str) -> int:
    """`episode-delete` 명령: 고른 에피소드를 DB 에서 지우고, 요청하면 파일도 지운다."""
    cfg = load_dataset_or_fail(config_path)
    if cfg is None:
        return 2
    try:
        ids = parse_id_spec(args.ids)
    except ValueError as exc:
        logging.error('bad --ids: %s', exc)
        return 2
    if not ids:
        logging.error('no episode id given')
        return 2

    roots = cfg.mp4_roots()
    try:
        with open_connection(cfg.db) as conn:
            summaries = {s.id: s for s in episode_summaries(conn, mp4_roots=roots)}
            missing = [i for i in ids if i not in summaries]
            if missing:
                logging.error('episode id(s) not found in the database: %s', missing)
                return 2
            targets = [summaries[i] for i in ids]
            files: list[Path] = []
            for s in targets:
                for st in s.streams:
                    files.extend(recording_files_for(st.mp4_path, roots))
            _print_plan(targets, files)
            if args.dry_run:
                logging.info('dry-run: nothing was deleted')
                return 0
            if not _confirm(len(targets), args.yes):
                logging.info('aborted')
                return 1
            deleted = delete_episodes(conn, ids)
            conn.commit()
    except Exception:
        logging.exception('episode delete failed')
        return 1

    print(f'deleted {deleted} episode(s) from the database')
    if args.delete_files:
        removed = 0
        for f in files:
            try:
                _remove(f)
                removed += 1
                _logger.info('removed %s', f)
            except OSError as exc:
                logging.error('failed to remove %s: %s', f, exc)
        print(f'removed {removed}/{len(files)} file(s)')
        if removed != len(files):
            return 1
    elif files:
        print('video files were kept (pass --delete-files to remove them)')
    return 0


__all__ = ['CONFIRM_WORD', 'cmd_episode_delete', 'main', 'parse_id_spec']


if __name__ == '__main__':
    sys.exit(main())
