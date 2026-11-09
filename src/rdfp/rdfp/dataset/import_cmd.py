"""`import` 독립 CLI — rosbag → DB / MP4 적재 파이프라인.

이전에는 ``dataset import`` 서브커맨드였으나, ROS sourcing 이 필요한 무거운
경로를 별도 entry_point 로 분리하여 ``dataset`` (init-db / stats / list /
replay) 와 독립적으로 동작하도록 했다.

스캔 → 에피소드 감지 → 에피소드 단위 트랜잭션 삽입 + MP4 인코딩까지
수행한다. 결과 summary 는 JSON 으로 출력하며, 적재 도중 ``PostProcError``
는 사용자 친화 메시지로, 그 외 예외는 traceback 과 함께 로그에 남긴다.

에피소드 단위 트랜잭션:
    1. ``enter`` → ``sessions`` INSERT (혹은 skip / replace 정책 적용).
    2. 메시지 dispatch → DB writer 또는 mp4 sink 로 라우팅.
    3. ``exit`` → writer flush → mp4 sink finalize → DB COMMIT.
       어느 단계에서 실패해도 DB ROLLBACK + mp4 abort 로 원자성을 확보한다.

공개 엔트리:
    * :func:`main` — argparse 진입점 (setup.py 의 ``import`` console_script).
    * :func:`cmd_import` — 파싱된 ``args`` 와 확정된 ``config_path`` 로 실행.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path

from rdfp.rosbag.catalog import discover_splits

from .cli_common import (
    DEFAULT_CONFIG_FILE_PATH, add_common_args, add_config_arg,
    configure_logging, load_dataset_or_fail, resolve_config_path,
)


_logger = logging.getLogger(__name__)


def _split_deletion_blocker(failed: int, excluded: int) -> str | None:
    """`delete_splits_after_import` 를 막을 사유. 지워도 되면 ``None``.

    지우는 단위는 **에피소드가 아니라 split 파일**이고, 대상은 이번 실행이 찾은 split
    전부다(날짜 필터만 반영된다). 그래서 그 split 의 에피소드가 전부 DB 에 들어갔다고
    말할 수 없는 실행에서는 지우지 않는다.

    Args:
        failed: 실패한 에피소드 수. 병렬 모드는 실패를 예외로 올리지 않으므로, 이것을
            보지 않으면 실패한 에피소드의 원본까지 "성공"한 실행 뒤에 지워진다.
        excluded: 감지됐지만 에피소드 필터가 뺀 수 — `start_ns_in`(`rosbag export` 의
            선택) · task_label 포함/제외 · 최소 길이. 같은 split 에 있으면 함께 지워진다.
    """
    if failed:
        return f'{failed} episode(s) failed'
    if excluded:
        return (f'episode filters excluded {excluded} episode(s) that may share '
                f'the same splits')
    return None


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='import',
        description='Ingest rosbag splits into the post-processed dataset (DB + MP4).',
    )
    add_common_args(p)
    add_config_arg(p, required=False)
    p.add_argument('--start-ns', type=int, action='append', dest='start_ns_in', metavar='NS',
                   help='이 자연키(start_ns)의 에피소드만 적재한다. 여러 번 줄 수 있다 — '
                        '`rosbag export` 와 데이터셋 GUI 가 선택을 넘기는 길이다')
    p.add_argument('--topics', nargs='+', metavar='TOPIC',
                   help='설정의 토픽 목록 대신 이 토픽만 적재한다 (/session 은 자동 포함). '
                        '⚠️ 적재는 에피소드 단위라 빠진 토픽은 그 에피소드에 빈 채로 커밋된다')
    p.add_argument('--on-existing', choices=('skip', 'replace', 'error'), dest='on_existing',
                   help='설정의 on_existing_episode 를 이번 실행만 바꾼다')
    return p


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    configure_logging(args.log_level)

    config_path = resolve_config_path(args)
    if config_path is None:
        logging.error('no --config given and default config not found: %s',
                      DEFAULT_CONFIG_FILE_PATH)
        return 2
    return cmd_import(args, config_path)


def cmd_import(args: argparse.Namespace, config_path: str) -> int:
    """`import` 서브커맨드: rosbag 을 DB / MP4 로 적재.

    스캔 → 에피소드 감지 → 에피소드 단위 트랜잭션 삽입 + MP4 인코딩까지
    수행한다.
    """
    cfg = load_dataset_or_fail(config_path)
    if cfg is None:
        return 2

    # `rosbag export` 가 선택 결과를 자연키로 실어 보낸다. 설정의 `start_ns_in` 을
    # **대신한다**(교집합이 아니다). 작업 포함·제외와 최소 길이 같은 나머지 필터는 그대로
    # 함께 적용된다. 적재 경로를 둘로 만들지 않기 위해 이 함수 하나만 쓴다.
    start_ns_in = list(getattr(args, 'start_ns_in', None) or [])
    if start_ns_in:
        cfg = cfg.model_copy(update={
            'episode_filter': cfg.episode_filter.model_copy(
                update={'start_ns_in': start_ns_in}),
        })
        _logger.info('episode selection: %d key(s) from the caller', len(start_ns_in))

    # `rosbag export --rosbag-dir` 가 에피소드를 고른 루트도 함께 보낸다. 고른 bag 과 적재하는
    # bag 이 같아야 하므로 설정의 `rosbag_dir` 보다 앞선다 — 그러지 않으면 자연키만 들고 설정의
    # 루트를 뒤져, 같은 키를 가진 다른 에피소드를 적재할 수 있다. `import` CLI 에는 이 인자가 없다.
    rosbag_dir = getattr(args, 'rosbag_dir', None)
    if rosbag_dir:
        if str(rosbag_dir) != cfg.rosbag_dir:
            _logger.info('bag root from the caller: %s (config rosbag_dir: %s)',
                         rosbag_dir, cfg.rosbag_dir)
        cfg = cfg.model_copy(update={'rosbag_dir': str(rosbag_dir)})

    # GUI 가 토픽을 골라 넘기는 길. 설정의 `topics` 를 **대신한다** (`effective_topics` 가 /session 을
    # 더한다). 파이프라인은 그대로 `cfg.effective_topics()` 만 보므로 적재 경로가 둘이 되지 않는다.
    topics = list(getattr(args, 'topics', None) or [])
    if topics:
        cfg = cfg.model_copy(update={'topics': topics})
        _logger.info('topic selection: %d topic(s) from the caller', len(topics))
    on_existing = getattr(args, 'on_existing', None)
    if on_existing:
        cfg = cfg.model_copy(update={'on_existing_episode': on_existing})
        _logger.info('on_existing_episode: %s (from the caller)', on_existing)

    # ROS / 무거운 ingest 의존은 본 함수 진입 시점에만 로드한다 (cli.py 의
    # ROS-free 서브커맨드들이 sensor_msgs 없이도 동작하도록).
    # ``discover_splits`` 는 가벼워 module top-level 에 둔다 (테스트 patch).
    from rdfp.dataset.db.connection import open_connection
    from rdfp.dataset.db.schema_check import SchemaCheckError, ensure_schema
    from rdfp.dataset.db.topics import upsert_topic_ids
    from .ingest.exceptions import PostProcError
    from .ingest.media.frame_router import FrameRouter
    from .ingest.pipeline import (
        _apply_episode_filters,
        _delete_used_splits,
        _detect_all_episodes,
        _determine_topic_classification,
        _empty_summary,
        _run_ingestion,
        _run_parallel,
    )
    from .ingest.run_log import RunLog
    from .inspect.findings import LEVEL_ERROR, LEVEL_WARNING
    from .inspect.preflight import check_import

    try:
        splits = discover_splits(cfg.rosbag_dir,
                                 dates=cfg.session_filter.dates or None,
                                 topics=cfg.effective_topics() or None)
        _logger.info('found %d finalized split(s)', len(splits))
        if not splits:
            summary = _empty_summary()
        else:
            episodes = _detect_all_episodes(splits)
            detected = len(episodes)
            _logger.info('detected %d episode(s)', detected)

            episodes = _apply_episode_filters(episodes, cfg.episode_filter)
            _logger.info('%d episode(s) remain after filters', len(episodes))

            if not episodes:
                summary = _empty_summary(splits=len(splits))
            else:
                required_tables, topic_entries, image_topic_types = (
                    _determine_topic_classification(splits, cfg))
                image_topics = set(image_topic_types)

                # 사전 점검 — 데이터셋 GUI 와 **같은 함수**다 (`inspect/preflight.py`). 죽는 조합
                # (이미지 토픽 + 레코더 등록 → image_streams UNIQUE 위반)과 `output_mp4_dir` 누락은
                # 여기서 멈추고, 겹치는 에피소드 창 같은 징후는 경고로 남긴다.
                known_types = {t: ty for t, (_, ty) in topic_entries.items()}
                known_types.update(image_topic_types)
                windows = [(ep.start_ns, ep.stop_ns) for ep in episodes
                           if hasattr(ep, 'start_ns')]
                findings = check_import(cfg, topic_types=known_types,
                                        episode_windows=windows or None)
                for f in findings:
                    if f.level == LEVEL_WARNING:
                        _logger.warning('preflight %s: %s', f.code, f.message)
                blocking = [f.message for f in findings if f.level == LEVEL_ERROR]
                if blocking:
                    raise PostProcError('; '.join(blocking))
                _logger.info('required tables: %s', sorted(required_tables))
                if image_topics:
                    _logger.info('camera topics routed to mp4: %s',
                                 sorted(image_topics))
                # 이미지 토픽도 image_streams / image_frames FK 로 사용하므로
                # 해당 테이블을 추가한다 (required_tables 는 message-writer 기준
                # 이라 빠져 있음).
                if image_topics:
                    required_tables = required_tables | {'image_streams', 'image_frames'}

                run_log_path = Path(cfg.output_mp4_dir) / '_logs' / 'postproc_run.jsonl'
                run_log = RunLog(run_log_path)

                summary = {
                    'episodes': 0, 'splits': len(splits),
                    'inserted': {t: 0 for t in required_tables},
                    'skipped': 0, 'replaced': 0, 'failed': 0, 'mp4_files': 0,
                }

                # 스키마 검증 + topics 룩업 동기화를 단일 커넥션으로 선행한다
                # (병렬 모드든 아니든 필요). topic_ids 는 이후 writer 의 FK 값
                # 으로 쓰이므로 여기서 commit 하여 워커 프로세스에서도 보이게 한다.
                with open_connection(cfg.db) as conn:
                    try:
                        ensure_schema(conn, required_tables)
                    except SchemaCheckError as e:
                        raise PostProcError(str(e)) from e
                    topics_to_register: dict[str, str] = {
                        topic: type_name
                        for topic, (_, type_name) in topic_entries.items()
                    }
                    topics_to_register.update(image_topic_types)
                    topic_ids = upsert_topic_ids(conn, topics_to_register)
                    conn.commit()

                if cfg.parallelism > 1:
                    _run_parallel(
                        cfg, splits, episodes, topic_entries, topic_ids,
                        image_topics, run_log_path, summary)
                else:
                    with open_connection(cfg.db) as conn:
                        router = FrameRouter(
                            Path(cfg.output_mp4_dir),
                            conn=conn,
                            topic_ids={t: topic_ids[t]
                                       for t in image_topics if t in topic_ids},
                            fps=cfg.mp4.nominal_fps,
                            codec=cfg.mp4.codec)
                        _run_ingestion(
                            conn, splits, episodes,
                            topic_entries=topic_entries,
                            topic_ids=topic_ids,
                            image_topics=image_topics,
                            router=router,
                            run_log=run_log,
                            cfg=cfg,
                            summary=summary)

                # 사용한 split 은 그 안의 에피소드가 전부 들어갔다고 말할 수 있을 때만 지운다.
                if cfg.delete_splits_after_import:
                    blocker = _split_deletion_blocker(summary['failed'],
                                                      detected - len(episodes))
                    if blocker is None:
                        summary['deleted_splits'] = _delete_used_splits(splits)
                    else:
                        _logger.warning('delete_splits_after_import skipped: %s', blocker)
                        summary['deleted_splits'] = 0
                        summary['delete_skipped'] = blocker
    except PostProcError as e:
        logging.error('post-processing failed: %s', e)
        return 1
    except Exception:
        logging.exception('post-processing crashed')
        return 1

    print(json.dumps(summary, indent=2, ensure_ascii=False))
    # **병렬 모드는 에피소드 실패를 예외로 올리지 않는다** — 세지 않으면 실패가 있어도
    # 0 으로 끝나, 종료 코드로 성공을 판정하는 호출자(cron 등)가 놓친다.
    if summary.get('failed'):
        logging.error('%d episode(s) failed; see episode_failed in %s',
                      summary['failed'],
                      Path(cfg.output_mp4_dir or '.') / '_logs' / 'postproc_run.jsonl')
        return 1
    return 0


__all__ = ['cmd_import', 'main']


if __name__ == '__main__':
    sys.exit(main())
