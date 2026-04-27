"""dataset import 파이프라인 헬퍼 모음.

상위 오케스트레이션은 :func:`rdfp.dataset.import_cmd.cmd_import` 가 직접
수행한다. 본 모듈은 그 과정에서 호출되는 단계별 헬퍼 (split discovery
보조, 에피소드 감지, 토픽 분류, 병렬/직렬 ingestion 루프 등) 와 공용
타입 (PostProcError) 만 보유한다.

에피소드 단위 트랜잭션:
  1. `enter` → `sessions` INSERT (혹은 skip / replace 정책 적용).
  2. 메시지 dispatch → DB writer 또는 mp4 sink 로 라우팅.
  3. `exit` → writer flush → mp4 sink finalize → DB COMMIT.
     어느 단계에서 실패해도 DB ROLLBACK + mp4 abort 로 원자성을 확보한다.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

import psycopg

from ..config import SESSION_TOPIC, DatasetConfig, EpisodeFilter
from rdfp.dataset.db.registry import (
    SESSION_COMMAND_TYPE, is_image_type, resolve_message_type,
)
from rdfp.dataset.db.writers.base import WriterBase
from rdfp.dataset.db.writers.session_command import SessionWriter
from rdfp.rosbag.merged_stream import merge_split_streams
from .episode.detector import Episode, SessionEvent, detect_episodes
from .episode.dispatcher import AssignedMessage, DispatchEvent, assign_and_dispatch
from .exceptions import PostProcError
from .quality_gate import QualityGate
from .run_log import RunLog
from .media.frame_router import FrameRouter


_logger = logging.getLogger(__name__)


def _run_parallel(cfg: DatasetConfig, splits: list, episodes: list[Episode],
                  topic_entries: dict[str, tuple[str, str]], topic_ids: dict[str, int],
                  image_topics: set[str], run_log_path: Path,
                  summary: dict) -> None:
    """`multiprocessing.Pool` 로 에피소드별 처리를 분산한다."""
    from multiprocessing import get_context

    from .episode_worker import serialize_splits

    split_paths = serialize_splits(splits)
    workers = min(cfg.parallelism, max(1, len(episodes)))
    _logger.info(
        'parallel mode: %d episode(s) across %d worker(s)', len(episodes), workers,
    )

    ctx = get_context('spawn')
    args_iter = [
        (cfg, ep, split_paths, dict(topic_entries), dict(topic_ids),
         set(image_topics), run_log_path)
        for ep in episodes
    ]
    with ctx.Pool(processes=workers) as pool:
        for result in pool.imap_unordered(_episode_worker_star, args_iter):
            _apply_result_to_summary(result, summary, cfg.on_existing_episode)


def _episode_worker_star(args: tuple) -> dict:
    """Pool.imap_unordered 용 unpack 함수 (top-level 필요)."""
    from .episode_worker import process_episode

    return process_episode(*args)


def _apply_result_to_summary(
    result: dict, summary: dict, policy: str,
) -> None:
    status = result.get('status')
    if status == 'done':
        summary['episodes'] += 1
        summary['mp4_files'] += len(result.get('mp4_files', []))
        if policy == 'replace' and result.get('replaced'):
            summary['replaced'] += 1
        for table, n in result.get('inserted', {}).items():
            summary['inserted'][table] = summary['inserted'].get(table, 0) + n
    elif status == 'skipped':
        summary['skipped'] += 1


def _empty_summary(splits: int = 0) -> dict:
    return {
        'episodes': 0, 'splits': splits,
        'inserted': {}, 'skipped': 0, 'replaced': 0, 'mp4_files': 0,
        'warnings': 0,
    }


def _delete_used_splits(splits: list) -> int:
    """import 에 사용된 split 파일을 제거하고, 비게 된 세션 디렉터리도 정리한다.

    개별 파일 삭제 실패는 warning 로깅 후 건너뛴다 (import 자체는 이미 성공한
    상태이므로 전체 실패로 전환하지 않는다).

    Returns:
        삭제에 성공한 .mcap 파일 수.
    """
    deleted = 0
    session_dirs: set[Path] = set()
    for sp in splits:
        try:
            sp.path.unlink()
            deleted += 1
            session_dirs.add(Path(sp.session_path))
            _logger.info('deleted split: %s', sp.path)
        except FileNotFoundError:
            # 이미 없는 경우는 deleted 로 치지 않지만 경고도 하지 않는다.
            session_dirs.add(Path(sp.session_path))
        except OSError as e:
            _logger.warning('failed to delete split %s: %s', sp.path, e)

    # 세션 디렉터리에 metadata.yaml 만 남았다면 세션 전체를 제거한다. 사용자가
    # 의도치 않게 넣어둔 다른 파일이 있다면 디렉터리를 보존한다.
    for sd in session_dirs:
        try:
            remaining = list(sd.iterdir())
        except OSError as e:
            _logger.warning('failed to inspect session dir %s: %s', sd, e)
            continue
        if remaining and all(p.name == 'metadata.yaml' and p.is_file() for p in remaining):
            for p in remaining:
                try:
                    p.unlink()
                except OSError as e:
                    _logger.warning('failed to delete %s: %s', p, e)
                    break
            else:
                try:
                    sd.rmdir()
                    _logger.info('removed empty session dir: %s', sd)
                except OSError as e:
                    _logger.warning('failed to rmdir %s: %s', sd, e)
    return deleted


def _detect_all_episodes(splits: list) -> list[Episode]:
    """splits 전체에서 `/session` 메시지만 뽑아 에피소드를 감지한다."""
    session_events: list[SessionEvent] = []
    for m in merge_split_streams(
        (str(s.path) for s in splits),
        topics=[SESSION_TOPIC],
    ):
        if m.type_name != SESSION_COMMAND_TYPE:
            continue
        session_events.append(SessionEvent(
            stamp_ns=m.stamp_ns,
            state=str(m.msg.state),
            task_label=str(getattr(m.msg, 'task_label', '') or ''),
        ))
    if not session_events:
        raise PostProcError(
            'no /session messages found within rosbag_dir; cannot detect episodes'
        )
    return detect_episodes(session_events)


def _apply_episode_filters(
    episodes: list[Episode], filt: EpisodeFilter,
) -> list[Episode]:
    """task_label / min_duration 필터를 적용한다."""
    include = set(filt.task_labels_include)
    exclude = set(filt.task_labels_exclude)
    min_dur_ns = int(filt.min_duration_sec * 1_000_000_000)

    out: list[Episode] = []
    for ep in episodes:
        if ep.stop_ns - ep.start_ns < min_dur_ns:
            continue
        if include and (ep.task_label or '') not in include:
            continue
        if (ep.task_label or '') in exclude:
            continue
        out.append(ep)
    return out


def _determine_topic_classification(splits: list,
                                    cfg: DatasetConfig) -> tuple[set[str], dict[str, tuple[str, str]], dict[str, str]]:
    """토픽을 DB / mp4 / 무시 로 분류한다.

    Returns:
        (required_tables, topic_entries, image_topic_types).

        `topic_entries` 는 **토픽 이름 → (table_name, type_name)** 매핑 (둘
        다 str). writer 클래스는 pickle 우려가 없도록 여기선 담지 않고, 실제
        writer 생성 시점에 `resolve_message_type(type_name)` 으로 다시
        조회한다.

        `image_topic_types` 는 mp4 로 라우팅될 이미지 토픽의 **토픽 → 메시지
        타입** 매핑. 호출자가 ``topics`` 테이블에 upsert 할 수 있도록 type_name
        을 함께 돌려준다. 이미지 토픽은 mp4 라우팅 대상이므로
        `required_tables` 에 포함되지 않는다 (호출자가 image_streams /
        image_frames 를 별도로 추가).
    """
    from rdfp.rosbag.mcap_reader import iter_split_messages

    want_topics = set(cfg.effective_topics()) if cfg.effective_topics() else None
    topic_types: dict[str, str] = {}
    for sp in splits:
        if want_topics is not None and len(topic_types) == len(want_topics):
            break
        for m in iter_split_messages(
            sp.path, topics=list(want_topics) if want_topics else None,
        ):
            if m.topic not in topic_types:
                topic_types[m.topic] = m.type_name
            if want_topics is not None and len(topic_types) == len(want_topics):
                break

    topic_entries: dict[str, tuple[str, str]] = {}
    required: set[str] = set()
    image_topic_types: dict[str, str] = {}
    for topic, type_name in topic_types.items():
        if topic == SESSION_TOPIC:
            continue
        if is_image_type(type_name):
            image_topic_types[topic] = type_name
            continue
        binding = resolve_message_type(type_name)
        if binding is None:
            _logger.warning(
                'no writer registered for topic %s (type=%s); messages will be ignored',
                topic, type_name,
            )
            continue
        topic_entries[topic] = (binding.table, type_name)
        required.add(binding.table)
    return required, topic_entries, image_topic_types


def _run_ingestion(conn: psycopg.Connection, splits: list, episodes: list[Episode], *,
                   topic_entries: dict[str, tuple[str, str]],
                   topic_ids: dict[str, int],
                   image_topics: set[str],
                   router: FrameRouter,
                   run_log: RunLog,
                   cfg: DatasetConfig,
                   summary: dict,) -> None:
    """에피소드 단위 트랜잭션으로 실제 INSERT 와 mp4 인코딩을 수행한다."""
    sess_writer = SessionWriter(conn)

    # topic → writer instance. 각 writer 가 자신의 table 속성을 갖는다.
    writers: dict[str, WriterBase] = {}
    for topic, (table_name, type_name) in topic_entries.items():
        binding = resolve_message_type(type_name)
        if binding is None:
            continue
        writers[topic] = binding.writer_cls(
            conn, table=table_name, topic_id=topic_ids[topic],
        )

    quality = QualityGate(
        check_stamp_regression=cfg.quality_gate.stamp_regression,
        idle_gap_ns=int(cfg.quality_gate.idle_gap_sec * 1_000_000_000),
    )

    # 현재 에피소드 컨텍스트.
    cur_episode_id: Optional[int] = None
    cur_episode_ep: Episode | None = None
    cur_ignored = False

    all_topics = cfg.effective_topics() or None
    raw_stream = merge_split_streams(
        (str(s.path) for s in splits),
        topics=all_topics,
    )

    for kind, payload in assign_and_dispatch(episodes, raw_stream):
        if kind == 'enter':
            ev: DispatchEvent = payload
            ep = episodes[ev.episode_index]
            cur_episode_ep = ep
            try:
                cur_episode_id = _open_episode(
                    sess_writer, ep, cfg.on_existing_episode, router, summary,
                )
            except Exception as e:
                _rollback_all(writers, router, conn)
                _logger.exception('failed to open episode %d', ev.episode_index)
                run_log.episode_failed(
                    start_ns=ep.start_ns, stop_ns=ep.stop_ns,
                    task_label=ep.task_label, error=str(e),
                )
                raise
            cur_ignored = cur_episode_id is None
            if cur_ignored:
                conn.rollback()
                run_log.episode_skipped(
                    episode_id=-1,
                    start_ns=ep.start_ns, stop_ns=ep.stop_ns,
                    task_label=ep.task_label,
                )
                continue
            router.open_episode(cur_episode_id)
            quality.reset_episode(cur_episode_id)
        elif kind == 'message':
            if cur_ignored:
                continue
            am: AssignedMessage = payload
            if am.topic == SESSION_TOPIC:
                continue
            quality.on_message(am.topic, am.stamp_ns)
            if am.topic in image_topics:
                try:
                    router.on_image(am.topic, am.type_name, am.msg)
                except Exception as e:
                    _rollback_all(writers, router, conn)
                    _logger.exception('image encoding failed')
                    assert cur_episode_ep is not None
                    run_log.episode_failed(
                        start_ns=cur_episode_ep.start_ns,
                        stop_ns=cur_episode_ep.stop_ns,
                        task_label=cur_episode_ep.task_label,
                        error=str(e),
                    )
                    raise
                continue
            writer = writers.get(am.topic)
            if writer is None:
                continue
            try:
                writer.append(cur_episode_id, am.msg)
            except Exception as e:
                _rollback_all(writers, router, conn)
                _logger.exception(
                    'insert failed for topic=%s (type=%s) in episode %d',
                    am.topic, am.type_name, am.episode_index,
                )
                assert cur_episode_ep is not None
                run_log.episode_failed(
                    start_ns=cur_episode_ep.start_ns,
                    stop_ns=cur_episode_ep.stop_ns,
                    task_label=cur_episode_ep.task_label,
                    error=str(e),
                )
                raise
        elif kind == 'exit':
            if cur_ignored:
                cur_episode_id = None
                cur_episode_ep = None
                cur_ignored = False
                continue
            assert cur_episode_ep is not None
            try:
                for w in writers.values():
                    w.flush()
                mp4_files = router.finalize_episode()
                conn.commit()
                row_counts: dict[str, int] = {}
                for w in writers.values():
                    n = w.consume_inserted_count()
                    row_counts[w.table] = n
                    summary['inserted'][w.table] = (
                        summary['inserted'].get(w.table, 0) + n
                    )
                summary['episodes'] += 1
                summary['mp4_files'] += len(mp4_files)
                _logger.info(
                    'episode committed: id=%d rows=%s mp4=%d',
                    cur_episode_id, row_counts, len(mp4_files),
                )
                run_log.episode_done(
                    episode_id=cur_episode_id,
                    start_ns=cur_episode_ep.start_ns,
                    stop_ns=cur_episode_ep.stop_ns,
                    task_label=cur_episode_ep.task_label,
                    row_counts=row_counts,
                    mp4_files=mp4_files,
                )
                for qw in quality.pop_warnings():
                    run_log.quality_warning(
                        episode_id=qw.episode_id,
                        kind=qw.kind, topic=qw.topic,
                        stamp_ns=qw.stamp_ns, prev_stamp_ns=qw.prev_stamp_ns,
                    )
                    summary['warnings'] += 1
            except Exception as e:
                _rollback_all(writers, router, conn)
                _logger.exception('commit failed for episode_id=%s', cur_episode_id)
                run_log.episode_failed(
                    start_ns=cur_episode_ep.start_ns,
                    stop_ns=cur_episode_ep.stop_ns,
                    task_label=cur_episode_ep.task_label,
                    error=str(e),
                )
                raise
            cur_episode_id = None
            cur_episode_ep = None


def _rollback_all(
    writers: dict[str, WriterBase],
    router: FrameRouter,
    conn: psycopg.Connection,
) -> None:
    """에피소드 트랜잭션 실패 시 모든 상태를 되돌린다."""
    for w in writers.values():
        w.drop_pending()
    router.abort_episode()
    conn.rollback()


def _open_episode(sess_writer: SessionWriter, ep: Episode, policy: str, router: FrameRouter,
                  summary: dict,) -> Optional[int]:
    """에피소드에 해당하는 sessions 행을 확보한다.

    단일 에피소드 트랜잭션의 첫 단계. 커밋은 `exit` 단계에서 수행되며,
    본 함수의 모든 DB 변경은 아직 미커밋 상태다. `replace` 정책은 FK cascade
    로 자식 row 를 제거하고, mp4 디렉터리도 함께 삭제한다.

    Returns:
        새/기존 에피소드 id. `skip` 정책으로 건너뛴 경우 None.
    """
    existing = sess_writer.find_existing(ep.start_ns)
    if existing is None:
        return sess_writer.insert_episode(ep.start_ns, ep.stop_ns, ep.task_label)

    if policy == 'skip':
        summary['skipped'] += 1
        _logger.info(
            'episode already present (start_ns=%d, id=%d); skipped',
            ep.start_ns, existing,
        )
        return None
    if policy == 'error':
        raise PostProcError(
            f'episode already present (start_ns={ep.start_ns}, id={existing})'
        )
    if policy == 'replace':
        sess_writer.delete_by_id(existing)
        router.remove_existing_episode_dir(existing)
        summary['replaced'] += 1
        return sess_writer.insert_episode(ep.start_ns, ep.stop_ns, ep.task_label)
    raise PostProcError(f'unknown on_existing_episode policy: {policy!r}')


__all__ = ['PostProcError']
