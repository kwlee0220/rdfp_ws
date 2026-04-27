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
    DEFAULT_CONFIG_FILENAME, add_common_args, add_config_arg,
    configure_logging, load_dataset_or_fail, resolve_config_path,
)


_logger = logging.getLogger(__name__)


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='import',
        description='Ingest rosbag splits into the post-processed dataset (DB + MP4).',
    )
    add_common_args(p)
    add_config_arg(p, required=False)
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
    return cmd_import(args, config_path)


def cmd_import(args: argparse.Namespace, config_path: str) -> int:
    """`import` 서브커맨드: rosbag 을 DB / MP4 로 적재.

    스캔 → 에피소드 감지 → 에피소드 단위 트랜잭션 삽입 + MP4 인코딩까지
    수행한다.
    """
    del args   # 현재 import 서브커맨드는 추가 인자가 없다.
    cfg = load_dataset_or_fail(config_path)
    if cfg is None:
        return 2

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

    try:
        splits = discover_splits(cfg.rosbag_dir,
                                 dates=cfg.session_filter.dates or None,
                                 topics=cfg.effective_topics() or None)
        _logger.info('found %d finalized split(s)', len(splits))
        if not splits:
            summary = _empty_summary()
        else:
            episodes = _detect_all_episodes(splits)
            _logger.info('detected %d episode(s)', len(episodes))

            episodes = _apply_episode_filters(episodes, cfg.episode_filter)
            _logger.info('%d episode(s) remain after filters', len(episodes))

            if not episodes:
                summary = _empty_summary(splits=len(splits))
            else:
                # 여기서부터는 실제 import 경로. ``output_mp4_dir`` 이 반드시 필요하다.
                if cfg.output_mp4_dir is None:
                    raise PostProcError(
                        'output_mp4_dir is required for import; '
                        'add it to the dataset config')

                required_tables, topic_entries, image_topic_types = (
                    _determine_topic_classification(splits, cfg))
                image_topics = set(image_topic_types)
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
                    'skipped': 0, 'replaced': 0, 'mp4_files': 0,
                    'warnings': 0,
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

                # import 가 예외 없이 끝났을 때만 사용된 split 파일을 제거한다.
                if cfg.delete_splits_after_import:
                    summary['deleted_splits'] = _delete_used_splits(splits)
    except PostProcError as e:
        logging.error('post-processing failed: %s', e)
        return 1
    except Exception:
        logging.exception('post-processing crashed')
        return 1

    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


__all__ = ['cmd_import', 'main']


if __name__ == '__main__':
    sys.exit(main())
