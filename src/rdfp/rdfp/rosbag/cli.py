"""`rosbag` CLI 엔트리포인트.

rosbag MCAP 아카이브를 **읽기 전용** 으로 검사하는 서브커맨드와, 보관 중인
rosbag 데이터를 일괄 삭제하는 관리용 서브커맨드를 모은다. DB 접속이 필요한
명령은 `dataset` CLI 쪽에 둔다.

서브커맨드:
    list-topics   — rosbag 에 기록된 모든 토픽 요약 출력 (ls -l 유사).
    clear         — rosbag 루트 아래 모든 세션 데이터 삭제 (destructive).

`--rosbag-dir` 인자가 미지정일 경우 ``RDFP_ROSBAG_DIR`` 환경변수가 fallback
으로 사용된다. 둘 다 없으면 exit 2.

사용 예:
    ros2 run rdfp rosbag list-topics --rosbag-dir /data/rdfp/rosbag
    ros2 run rdfp rosbag clear --rosbag-dir /data/rdfp/rosbag
    ros2 run rdfp rosbag clear --rosbag-dir /data/rdfp/rosbag --yes

    # 환경변수로 지정 (반복 호출에 편리).
    export RDFP_ROSBAG_DIR=/data/rdfp/rosbag
    ros2 run rdfp rosbag list-topics
    ros2 run rdfp rosbag clear
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import shutil
import sys
from datetime import datetime
from pathlib import Path


from rdfp.dataset.cli_common import configure_logging

from .catalog import DATE_DIR_RE, SESSION_DIR_RE, discover_splits


# text 포맷의 START_TS 컬럼 폭 (YYYY-MM-DD HH:MM:SS = 19 자).
_START_TS_WIDTH: int = 19

# `--rosbag-dir` 미지정 시 fallback 으로 읽는 환경변수 이름.
_ROSBAG_DIR_ENV: str = 'RDFP_ROSBAG_DIR'


def _add_rosbag_dir_arg(parser: argparse.ArgumentParser) -> None:
    """서브커맨드 파서에 `--rosbag-dir` 옵션을 추가한다.

    명시되지 않으면 ``RDFP_ROSBAG_DIR`` 환경변수에서 fallback 으로 읽는다.
    """
    parser.add_argument(
        '--rosbag-dir',
        help='rosbag2 MCAP root directory (e.g. /data/rdfp/rosbag). '
             f'미지정 시 {_ROSBAG_DIR_ENV} 환경변수를 사용한다.',
    )


def _resolve_rosbag_dir(args: argparse.Namespace) -> Path | None:
    """`--rosbag-dir` 인자 또는 ``RDFP_ROSBAG_DIR`` 환경변수에서 경로를 확정한다.

    Returns:
        확정된 ``Path`` 또는 둘 다 비어 있으면 ``None``. None 인 경우 호출
        측은 사용자 오류로 간주하고 exit 2 로 귀결시킨다 (에러 로그는 본
        함수에서 이미 남긴다).
    """
    raw = args.rosbag_dir or os.environ.get(_ROSBAG_DIR_ENV, '').strip()
    if not raw:
        logging.error(
            'rosbag directory not specified: pass --rosbag-dir or set %s',
            _ROSBAG_DIR_ENV,
        )
        return None
    return Path(raw)


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='rosbag',
        description='Inspect rosbag2 MCAP archives (read-only).',
    )
    add_common_args(p)
    sub = p.add_subparsers(dest='command')

    lt = sub.add_parser('list-topics', help='list all recorded topics (ls -l style summary)')
    _add_rosbag_dir_arg(lt)
    lt.add_argument('--format', choices=['text', 'json'], default='text')
    lt.add_argument('--sort', choices=['name', 'count', 'rate', 'type'], default='name',
                    help='sort order for text output (default: name)')

    cl = sub.add_parser('clear', help='delete all session data under rosbag_dir (destructive)')
    _add_rosbag_dir_arg(cl)
    cl.add_argument('--yes', action='store_true',
                    help='non-interactive confirmation (required in non-TTY)')

    le = sub.add_parser('list-episodes',
                        help='list episodes detected in the bags, oldest first')
    _add_rosbag_dir_arg(le)
    _add_selection_args(le)
    le.add_argument('--format', choices=['text', 'json'], default='text')

    ex = sub.add_parser('export', help='ingest selected episodes into the database')
    _add_rosbag_dir_arg(ex)
    _add_selection_args(ex)
    ex.add_argument('--only-new', action='store_true',
                    help='skip episodes already present in the database')
    ex.add_argument('--dry-run', action='store_true',
                    help='print the selection and exit without writing')

    dl = sub.add_parser('delete', help='remove selected episodes from the bags (destructive)')
    _add_rosbag_dir_arg(dl)
    _add_selection_args(dl)
    dl.add_argument('--exported', action='store_true',
                    help='restrict to episodes already present in the database')
    dl.add_argument('--yes', action='store_true',
                    help='non-interactive confirmation (required in non-TTY)')
    dl.add_argument('--dry-run', action='store_true',
                    help='print the selection and exit without deleting')

    return p


def _add_selection_args(parser: argparse.ArgumentParser) -> None:
    """에피소드 선택 인자 — 세 서브커맨드가 같은 문법을 쓴다.

    시간은 **rosbag 이 기록한 시각(log time)** 기준이다. `header.stamp` 는 발행자
    시계라 경과시간이 섞이는 회차가 있어 사람이 아는 시각과 어긋날 수 있다.
    """
    parser.add_argument('--dates', nargs='*', metavar='YYYY-MM-DD',
                        help='날짜 폴더 선행 필터')
    parser.add_argument('--task', help='이 task_label 만')
    parser.add_argument('--since', metavar='TIME',
                        help="시작 시각 (로컬, 'YYYY-MM-DD HH:MM[:SS]')")
    parser.add_argument('--until', metavar='TIME', help='종료 시각 (같은 형식)')
    parser.add_argument('--episodes', metavar='SPEC',
                        help="목록 순번 (예: '3-7,9'). **순번은 안정적이지 않다** — "
                             '스크립트에는 --start-ns 를 쓴다')
    parser.add_argument('--start-ns', type=int, action='append', metavar='NS',
                        help='에피소드 자연키(start_ns). 여러 번 줄 수 있다')
    parser.add_argument('--config', help='dataset 설정 YAML — DB 적재 여부 확인·적재에 쓴다')


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    configure_logging(args.log_level)

    command = args.command
    if command == 'list-topics':
        return _cmd_list_topics(args)
    if command == 'clear':
        return _cmd_clear(args)
    if command == 'list-episodes':
        return _cmd_list_episodes(args)
    if command == 'export':
        return _cmd_export(args)
    if command == 'delete':
        return _cmd_delete(args)
    if command is None:
        logging.error('no subcommand given (expected: list-topics | list-episodes | '
                      'export | delete | clear)')
        return 2
    logging.error('unknown command: %r', command)
    return 2


def _cmd_list_topics(args: argparse.Namespace) -> int:
    """`list-topics` 서브커맨드: rosbag 에 기록된 모든 토픽 요약을 출력한다.

    split 단위로 mcap `Summary` 를 열어 토픽별 message_count 를 누적하고,
    `iter_messages()` 를 한 번 순회하여 토픽별 첫/마지막 `log_time` 을
    수집한다. text 포맷은 `ls -l` 스타일로 동적 컬럼 폭을 적용한다.
    """
    rosbag_dir = _resolve_rosbag_dir(args)
    if rosbag_dir is None:
        return 2

    try:
        splits = discover_splits(rosbag_dir)
    except FileNotFoundError as e:
        logging.error('%s', e)
        return 1

    if not splits:
        logging.warning('no splits found under %s', rosbag_dir)

    # topic name -> {'count': int, 'types': set[str], 'first_ns': int | None,
    #                'last_ns': int | None}
    per_topic: dict[str, dict] = {}
    sessions: set[str] = set()

    for sp in splits:
        try:
            _inspect_split_all_topics(sp.path, per_topic)
        except Exception as e:   # noqa: BLE001
            logging.warning('failed to read %s: %s', sp.path, e)
            continue
        sessions.add(sp.session_name)

    topics_list: list[dict] = []
    for name in sorted(per_topic.keys()):
        d = per_topic[name]
        duration = (
            (d['last_ns'] - d['first_ns']) / 1e9
            if d['first_ns'] is not None and d['last_ns'] is not None
            else 0.0
        )
        rate = d['count'] / duration if duration > 0 else 0.0
        topics_list.append({
            'name': name,
            'types': sorted(d['types']),
            'message_count': d['count'],
            'first_log_ns': d['first_ns'],
            'last_log_ns': d['last_ns'],
            'duration_sec': round(duration, 3),
            'rate_hz': round(rate, 3),
        })

    result: dict = {
        'session_count': len(sessions),
        'split_count': len(splits),
        'topic_count': len(topics_list),
        'topics': topics_list,
    }

    if args.format == 'json':
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return 0

    _print_list_topics_text(result, sort_by=args.sort)
    return 0


def _inspect_split_all_topics(split_path: Path, per_topic: dict) -> None:
    """단일 split 의 모든 토픽 통계를 `per_topic` 에 누적한다.

    `message_count` 는 summary 의 `channel_message_counts` 에서 즉시 얻고,
    첫/마지막 `log_time` 은 `iter_messages()` 를 log_time 오름차순으로
    순회하며 토픽별로 한 번만 세팅 (first) 또는 덮어쓴다 (last).
    `discover_splits` 가 split 을 시간순으로 정렬해 주므로 여러 split 에
    걸친 전역 first/last 가 자연스럽게 맞춰진다.
    """
    from mcap.reader import make_reader

    with open(split_path, 'rb') as f:
        reader = make_reader(f)
        summary = reader.get_summary()
        if summary is None:
            return

        counts = (
            summary.statistics.channel_message_counts if summary.statistics else {}
        )
        # channel_id → (topic, type_name).
        ch_meta: dict[int, tuple[str, str | None]] = {}
        for ch in summary.channels.values():
            schema = summary.schemas.get(ch.schema_id)
            ch_meta[ch.id] = (ch.topic, schema.name if schema is not None else None)

        for ch_id, (topic, type_name) in ch_meta.items():
            d = per_topic.setdefault(topic, {
                'count': 0, 'types': set(), 'first_ns': None, 'last_ns': None,
            })
            d['count'] += counts.get(ch_id, 0)
            if type_name:
                d['types'].add(type_name)

        # log_time 범위는 토픽별로 iter_messages 순회를 통해 확정.
        for _schema, channel, msg in reader.iter_messages():
            topic = channel.topic
            d = per_topic.setdefault(topic, {
                'count': 0, 'types': set(), 'first_ns': None, 'last_ns': None,
            })
            if d['first_ns'] is None:
                d['first_ns'] = msg.log_time
            d['last_ns'] = msg.log_time


def _print_list_topics_text(result: dict, *, sort_by: str = 'name') -> None:
    """`list-topics` 의 `ls -l` 스타일 text 출력.

    각 컬럼은 실제 값의 최대 폭에 맞춰 동적으로 정렬한다. TYPE 컬럼에
    여러 타입이 섞인 드문 경우 쉼표로 연결한다.
    """
    topics = list(result['topics'])
    print(
        f'total {result["topic_count"]}  '
        f'({result["session_count"]} session(s), {result["split_count"]} split(s))'
    )
    if not topics:
        return

    # 정렬 기준 적용.
    if sort_by == 'count':
        topics.sort(key=lambda t: -t['message_count'])
    elif sort_by == 'rate':
        topics.sort(key=lambda t: -t['rate_hz'])
    elif sort_by == 'type':
        topics.sort(key=lambda t: (','.join(t['types']), t['name']))
    # 'name' 은 이미 오름차순.

    def _type_str(t: dict) -> str:
        types = t['types']
        if not types:
            return '-'
        if len(types) == 1:
            return types[0]
        return ','.join(types)

    def _first_ts(t: dict) -> str:
        first_ns = t['first_log_ns']
        if first_ns is None:
            return '-'
        return datetime.fromtimestamp(first_ns // 1_000_000_000).strftime('%Y-%m-%d %H:%M:%S')

    # 컬럼 폭 동적 계산.
    count_w = max(len('COUNT'), max(len(str(t['message_count'])) for t in topics))
    rate_strings = [f'{t["rate_hz"]:.3f}' for t in topics]
    rate_w = max(len('RATE'), max(len(r) for r in rate_strings))
    type_w = max(len('TYPE'), max(len(_type_str(t)) for t in topics))

    header = (
        f'{"COUNT":>{count_w}}  {"RATE":>{rate_w}} Hz  '
        f'{"FIRST_TS":<{_START_TS_WIDTH}}  {"TYPE":<{type_w}}  TOPIC'
    )
    print(header)
    for t, rate_s in zip(topics, rate_strings):
        print(
            f'{t["message_count"]:>{count_w}}  {rate_s:>{rate_w}} Hz  '
            f'{_first_ts(t):<{_START_TS_WIDTH}}  {_type_str(t):<{type_w}}  '
            f'{t["name"]}'
        )


def _cmd_clear(args: argparse.Namespace) -> int:
    """`clear` 서브커맨드: rosbag 루트 아래의 세션 데이터를 일괄 삭제한다.

    `<rosbag_dir>/YYYY-MM-DD/session_*/` 에 해당하는 디렉터리 전체가 대상이다
    (metadata.yaml 유무 무관 — 비정상 종료 세션도 포함). 삭제 후 비어진 날짜
    디렉터리는 함께 제거한다.

    안전장치:
        - 항상 대상 목록을 먼저 출력한다.
        - 대화형: ``'y'`` (대소문자 무관) 를 타이핑해야 진행.
        - 비대화형 (non-TTY 또는 CI): ``--yes`` 가 없으면 거부.
    """
    rosbag_dir = _resolve_rosbag_dir(args)
    if rosbag_dir is None:
        return 2
    if not rosbag_dir.is_dir():
        logging.error('rosbag_dir not found: %s', rosbag_dir)
        return 1

    targets = _enumerate_session_dirs(rosbag_dir)

    if not targets:
        print(f'(no session directories found under {rosbag_dir})')
        return 0

    total_bytes = 0
    for t in targets:
        try:
            total_bytes += _dir_size(t)
        except OSError:
            pass

    print(
        f'Found {len(targets)} session director{"y" if len(targets) == 1 else "ies"} '
        f'(~{_fmt_bytes(total_bytes)}) under {rosbag_dir}:'
    )
    for t in targets:
        print(f'  {t.relative_to(rosbag_dir)}')

    if not args.yes:
        if not _confirm_clear(rosbag_dir):
            logging.error('aborted by user')
            return 2

    deleted = 0
    for t in targets:
        try:
            shutil.rmtree(t)
            deleted += 1
        except Exception as e:   # noqa: BLE001
            logging.error('failed to delete %s: %s', t, e)

    # 삭제로 비어진 날짜 디렉터리를 정리한다 (rmdir 은 비어있지 않으면 실패하므로 안전).
    for date_dir in sorted(rosbag_dir.iterdir()):
        if date_dir.is_dir() and DATE_DIR_RE.match(date_dir.name):
            try:
                date_dir.rmdir()
            except OSError:
                pass

    print(
        f'deleted {deleted}/{len(targets)} session director'
        f'{"y" if len(targets) == 1 else "ies"}'
    )
    return 0 if deleted == len(targets) else 1


# --- 에피소드 서브커맨드 ------------------------------------------------------


def _parse_time(text: str) -> int:
    """`'YYYY-MM-DD HH:MM[:SS]'` 를 epoch ns 로 바꾼다 (로컬 시간대)."""
    for fmt in ('%Y-%m-%d %H:%M:%S', '%Y-%m-%d %H:%M', '%Y-%m-%d'):
        try:
            return int(datetime.strptime(text, fmt).timestamp() * 1_000_000_000)
        except ValueError:
            continue
    raise ValueError(f"cannot parse time: {text!r} (expected 'YYYY-MM-DD HH:MM[:SS]')")


def _fmt_time(ns: int) -> str:
    return datetime.fromtimestamp(ns / 1e9).strftime('%Y-%m-%d %H:%M:%S')


def _dataset_config(config_path: str | None):
    """설정 YAML 을 읽는다. 없거나 실패하면 `None`.

    실패를 오류로 올리지 않는 이유는, 적재 여부 확인이 **선택 기능**이기 때문이다 —
    설정을 못 읽으면 그 열이 '확인 못 함'(`?`) 이 되고 목록 자체는 계속 나온다.
    """
    if not config_path:
        return None
    try:
        from rdfp.dataset.cli_common import load_dataset_or_fail

        return load_dataset_or_fail(config_path)
    except Exception as e:          # noqa: BLE001 — 설정/환경 문제는 '확인 못 함' 이다
        logging.warning('cannot read dataset settings from %s: %s', config_path, e)
        return None


def _resolve_bag_dir(args: argparse.Namespace, cfg) -> Path | None:
    """bag 루트를 정한다 — **인자 > 설정 > 환경변수** 순이다.

    설정이 환경변수를 이기는 이유: `--config` 는 이 실행을 위해 사람이 명시한 파일이고,
    `RDFP_ROSBAG_DIR` 은 셸에 남아 있는 주변값이다. 환경변수가 이기면 **설정이 가리키는
    DB 에 다른 bag 을 적재**할 수 있다.
    """
    if args.rosbag_dir:
        return Path(args.rosbag_dir)
    if cfg is not None and getattr(cfg, 'rosbag_dir', None):
        return Path(cfg.rosbag_dir)
    return _resolve_rosbag_dir(args)


def _gather(args: argparse.Namespace) -> tuple[list, list] | None:
    """`(전체 목록, 선택된 목록)`. 사용자 오류면 `None` 을 돌려준다."""
    from .episode_ops import build_views, load_episodes, lookup_exported, parse_index_spec, select

    cfg = _dataset_config(getattr(args, 'config', None))
    rosbag_dir = _resolve_bag_dir(args, cfg)
    if rosbag_dir is None:
        return None
    try:
        episodes = load_episodes(rosbag_dir, dates=getattr(args, 'dates', None) or None)
    except FileNotFoundError as e:
        logging.error('%s', e)
        return None
    if not episodes:
        logging.warning('no episodes found under %s', rosbag_dir)

    exported = None
    if cfg is not None and episodes:
        try:
            exported = lookup_exported(episodes, cfg.db)
        except Exception as e:      # noqa: BLE001 — DB 접속 실패는 '확인 못 함' 이다
            logging.warning('cannot check export status: %s', e)
    views = build_views(episodes, exported)

    try:
        since = _parse_time(args.since) if getattr(args, 'since', None) else None
        until = _parse_time(args.until) if getattr(args, 'until', None) else None
        picked = select(views, since_ns=since, until_ns=until,
                        task=getattr(args, 'task', None),
                        only_exported=bool(getattr(args, 'exported', False)),
                        only_new=bool(getattr(args, 'only_new', False)))
        if getattr(args, 'episodes', None):
            chosen = {v.index for v in parse_index_spec(args.episodes, views)}
            picked = [v for v in picked if v.index in chosen]
        if getattr(args, 'start_ns', None):
            wanted = set(args.start_ns)
            picked = [v for v in picked if v.start_ns in wanted]
    except ValueError as e:
        logging.error('%s', e)
        return None
    return views, picked


def _print_episodes(views: list, *, fmt: str = 'text') -> None:
    from .episode_ops import EXPORT_UNKNOWN, EXPORT_YES

    if fmt == 'json':
        import json

        print(json.dumps([{
            'index': v.index,
            'start_ns': v.start_ns,
            'log_start_ns': v.bag.log_start_ns,
            'log_stop_ns': v.bag.log_stop_ns,
            'duration_sec': round(v.bag.duration_ns / 1e9, 3),
            'task_label': v.bag.task_label,
            'success': v.bag.episode.success,
            'exported': v.exported,
            'session': v.bag.session_name,
        } for v in views], ensure_ascii=False, indent=2))
        return

    if not views:
        print('(no episodes)')
        return
    mark = {EXPORT_YES: 'yes', EXPORT_UNKNOWN: '?'}
    w_task = max([len(v.bag.task_label) for v in views] + [4])
    w_sess = max([len(v.bag.session_name) for v in views] + [7])
    print(f"{'#':>4}  {'recorded (log time)':<19}  {'dur':>7}  {'task':<{w_task}}  "
          f"{'result':<7}  {'export':<6}  {'session':<{w_sess}}  start_ns")
    for v in views:
        success = v.bag.episode.success
        result = 'success' if success is True else ('failure' if success is False else '-')
        print(f'{v.index:>4}  {_fmt_time(v.bag.log_start_ns):<19}  '
              f'{v.bag.duration_ns / 1e9:>6.1f}s  {v.bag.task_label:<{w_task}}  '
              f'{result:<7}  {mark.get(v.exported, "-"):<6}  '
              f'{v.bag.session_name:<{w_sess}}  {v.start_ns}')
    print(f'\n{len(views)} episode(s)')


def _cmd_list_episodes(args: argparse.Namespace) -> int:
    """`list-episodes`: bag 의 에피소드를 기록 시각 순으로 낸다."""
    got = _gather(args)
    if got is None:
        return 2
    _all, picked = got
    _print_episodes(picked, fmt=args.format)
    return 0


def _cmd_export(args: argparse.Namespace) -> int:
    """`export`: 선택된 에피소드만 DB·MP4 로 적재한다.

    적재 자체는 `import` 와 **같은 파이프라인**을 탄다 — 선택 결과를 자연키 목록으로
    바꿔 `episode_filter.start_ns_in` 에 실어 넘긴다.
    """
    if not getattr(args, 'config', None):
        logging.error('--config is required for export (database and mp4 output)')
        return 2
    got = _gather(args)
    if got is None:
        return 2
    _all, picked = got
    if not picked:
        logging.warning('no episode selected; nothing to export')
        return 0

    _print_episodes(picked)
    if args.dry_run:
        logging.info('dry-run: nothing was written')
        return 0

    from rdfp.dataset.import_cmd import cmd_import

    ns = argparse.Namespace(start_ns_in=[v.start_ns for v in picked])
    return cmd_import(ns, args.config)


def _cmd_delete(args: argparse.Namespace) -> int:
    """`delete`: 선택된 에피소드를 bag 에서 제거한다 (split 재작성)."""
    got = _gather(args)
    if got is None:
        return 2
    _all, picked = got
    if not picked:
        logging.warning('no episode selected; nothing to delete')
        return 0

    _print_episodes(picked)
    if args.dry_run:
        logging.info('dry-run: nothing was deleted')
        return 0
    if not _confirm_delete(len(picked), args.yes):
        logging.info('aborted')
        return 1

    from .episode_ops import delete_episodes

    rosbag_dir = _resolve_bag_dir(args, _dataset_config(getattr(args, 'config', None)))
    try:
        reports, gone = delete_episodes(rosbag_dir, picked)
    except RuntimeError as e:
        logging.error('delete verification failed: %s', e)
        return 1
    for r in reports:
        logging.info('%s: removed %d message(s), kept %d', r.split_path, r.removed, r.kept)
    logging.info('deleted %d episode(s)', len(gone))
    return 0


def _confirm_delete(count: int, yes: bool) -> bool:
    """삭제 확인. 비대화형에서는 `--yes` 가 필수다 (`clear` 와 같은 규칙)."""
    if yes:
        return True
    if not sys.stdin.isatty():
        logging.error('refusing to delete without --yes in a non-interactive shell')
        return False
    reply = input(f'delete {count} episode(s) from the bags? [y/N] ').strip().lower()
    return reply in ('y', 'yes')


def add_common_args(parser):
    """모든 CLI 의 최상위 파서에 공통 옵션을 추가한다.

    현재는 `--log-level` 한 개만 제공한다.
    """
    parser.add_argument('--log-level', default='info',
                        choices=('debug', 'info', 'warning', 'error'))


def _enumerate_session_dirs(rosbag_dir: Path) -> list[Path]:
    """`<rosbag_dir>/YYYY-MM-DD/session_*/` 디렉터리를 수집한다.

    `discover_splits` 는 metadata.yaml 이 없는 세션을 건너뛰지만, 삭제
    대상 선정에서는 비정상 종료 세션도 포함해야 하므로 별도 스캔을 수행한다.
    """
    results: list[Path] = []
    for date_dir in sorted(rosbag_dir.iterdir()):
        if not date_dir.is_dir() or not DATE_DIR_RE.match(date_dir.name):
            continue
        for session_dir in sorted(date_dir.iterdir()):
            if session_dir.is_dir() and SESSION_DIR_RE.match(session_dir.name):
                results.append(session_dir)
    return results


def _dir_size(path: Path) -> int:
    """디렉터리 하위 모든 일반 파일의 총 byte 크기."""
    total = 0
    for p in path.rglob('*'):
        if p.is_file():
            try:
                total += p.stat().st_size
            except OSError:
                pass
    return total


def _fmt_bytes(n: int) -> str:
    """사람이 읽기 좋은 크기 문자열 (KB/MB/GB)."""
    if n < 1024:
        return f'{n} B'
    units = ['KB', 'MB', 'GB', 'TB']
    size = float(n)
    for u in units:
        size /= 1024.0
        if size < 1024.0:
            return f'{size:.1f} {u}'
    return f'{size:.1f} PB'


def _confirm_clear(rosbag_dir: Path) -> bool:
    """대화형으로 clear 의도를 재확인한다 (`y` / `Y` 입력 시 진행)."""
    if not sys.stdin.isatty() or os.environ.get('CI'):
        # non-interactive 환경에서는 --yes 없으면 거부한다.
        return False
    prompt = (
        f"This will permanently DELETE all session data under {rosbag_dir}. "
        f"Type 'y' to proceed: "
    )
    try:
        answer = input(prompt).strip().lower()
    except EOFError:
        return False
    return answer == 'y'


if __name__ == '__main__':
    sys.exit(main())
