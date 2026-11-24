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

from typing import TYPE_CHECKING, Optional

import logging
from pathlib import Path

import psycopg

from ..config import SESSION_TOPIC, DatasetConfig, EpisodeFilter
from rdfp.dataset.db.registry import is_image_type, resolve_message_type
from rdfp.dataset.db.writers.base import WriterBase
from rdfp.dataset.db.writers.session_command import SessionWriter
from rdfp.rosbag.merged_stream import merge_split_streams
from .episode.detector import Episode
from .episode.dispatcher import AssignedMessage, DispatchEvent, assign_and_dispatch
from .episode.from_bag import episodes_from_splits
from .exceptions import PostProcError
from .run_log import RunLog
from .media.recorder_registry import RecorderRegistrar
from .units import Opening, Origin, finish_units, make_writer, open_episode, register_recordings

if TYPE_CHECKING:
    # FrameRouter 는 `sensor_msgs` 를 끌어온다 — bag 에 이미지 토픽이 있을 때만 만든다. 그래야
    # 영상을 레코더 mp4 로 받는 평소 적재(I4)가 ROS 없이 돈다 (2026-11-23).
    from .media.frame_router import FrameRouter


_logger = logging.getLogger(__name__)


def _run_parallel(cfg: DatasetConfig, splits: list, episodes: list[Episode],
                  topic_entries: dict[str, tuple[str, str]], topic_ids: dict[str, int],
                  image_topics: set[str], run_log_path: Path,
                  summary: dict, origins: Optional[dict] = None) -> None:
    """`multiprocessing.Pool` 로 에피소드별 처리를 분산한다."""
    from multiprocessing import get_context

    from .episode_worker import serialize_splits

    split_paths = serialize_splits(splits)
    workers = min(cfg.parallelism, max(1, len(episodes)))
    _logger.info(
        'parallel mode: %d episode(s) across %d worker(s)', len(episodes), workers,
    )

    ctx = get_context('spawn')
    origins = origins or {}
    args_iter = [
        (cfg, ep, split_paths, dict(topic_entries), dict(topic_ids),
         set(image_topics), run_log_path, origins.get(ep.start_ns))
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
    if result.get('adopted'):
        summary['adopted'] = summary.get('adopted', 0) + 1
    if status == 'done':
        summary['episodes'] += 1
        summary['mp4_files'] += len(result.get('mp4_files', []))
        if policy == 'replace' and result.get('replaced'):
            summary['replaced'] += 1
        for table, n in result.get('inserted', {}).items():
            summary['inserted'][table] = summary['inserted'].get(table, 0) + n
    elif status == 'skipped':
        summary['skipped'] += 1
    else:
        # 워커는 실패를 예외가 아니라 `status='failed'` 로 돌려준다. 여기서 세지 않으면
        # 요약에도 종료 코드에도 안 남고 JSONL 의 `episode_failed` 에만 남는다.
        summary['failed'] = summary.get('failed', 0) + 1


def _empty_summary(splits: int = 0) -> dict:
    return {
        'episodes': 0, 'splits': splits,
        'inserted': {}, 'skipped': 0, 'replaced': 0, 'failed': 0, 'mp4_files': 0,
        'adopted': 0,
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
    """splits 전체에서 `/session` 메시지만 뽑아 에피소드를 감지한다.

    실제 감지는 `episode.from_bag.episodes_from_splits` 가 한다 — `rosbag` CLI 도 같은
    함수를 쓴다. 여기서는 적재 경로가 기대하는 `Episode` 목록으로 벗겨 낸다.
    """
    bag_episodes = episodes_from_splits(splits)
    if not bag_episodes:
        raise PostProcError(
            'no /session messages found within rosbag_dir; cannot detect episodes'
        )
    return [be.episode for be in bag_episodes]


def _detect_episodes_with_origin(splits: list) -> tuple:
    """`_detect_all_episodes` + 에피소드별 출처(녹화본).

    출처는 시작 `/session` 메시지가 있던 세션 디렉터리다 (`BagEpisode.session_name`). 적재가 그
    값으로 **다른 녹화본이 같은 에피소드 키를 낸 경우**(sim 시계 되감김)를 가려낸다.

    Returns:
        `(episodes, origins)` — origins 는 `start_ns → Origin`.
    """
    bag_episodes = episodes_from_splits(splits)
    if not bag_episodes:
        raise PostProcError(
            'no /session messages found within rosbag_dir; cannot detect episodes'
        )
    origins = {be.start_ns: Origin(be.session_name, be.session_path)
               for be in bag_episodes if be.session_name}
    return [be.episode for be in bag_episodes], origins


def _apply_episode_filters(
    episodes: list[Episode], filt: EpisodeFilter,
) -> list[Episode]:
    """task_label / min_duration / start_ns 필터를 적용한다."""
    include = set(filt.task_labels_include)
    exclude = set(filt.task_labels_exclude)
    min_dur_ns = int(filt.min_duration_sec * 1_000_000_000)
    # 자연키로 못박은 경우 그것이 가장 강한 조건이다 — 나머지 필터도 함께 적용된다.
    keys = set(filt.start_ns_in)

    out: list[Episode] = []
    for ep in episodes:
        if keys and ep.start_ns not in keys:
            continue
        if ep.stop_ns - ep.start_ns < min_dur_ns:
            continue
        if include and (ep.task_label or '') not in include:
            continue
        if (ep.task_label or '') in exclude:
            continue
        out.append(ep)
    return out


def _determine_topic_classification(
    splits: list, cfg: DatasetConfig, dynamic: Optional[dict] = None
) -> tuple[set[str], dict[str, tuple[str, str]], dict[str, str]]:
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
    # 타입은 MCAP summary 에서 먼저 읽는다 — 메시지를 풀지 않는다 (2026-11-23). 예전에는 타입을
    # 알려고 bag 을 거의 한 번 통째로 디코드했다 (실측 37만 건 bag 에서 ~29 초).
    topic_types: dict[str, str] = _summary_topic_types(splits, want_topics)
    for sp in splits:
        if topic_types:
            break
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
            # registry 에 없으면 동적 매핑(`topic_tables`)을 본다 — **내장이 이긴다**.
            # 동적 테이블은 `REQUIRED_COLUMNS` 에 없으므로 `required` 에 넣지 않고 호출자가
            # 따로 존재를 확인한다.
            mapping = (dynamic or {}).get(type_name)
            if mapping is not None:
                topic_entries[topic] = (mapping.table_name, type_name)
                continue
            _logger.warning(
                'no writer registered for topic %s (type=%s); messages will be ignored',
                topic, type_name,
            )
            continue
        topic_entries[topic] = (binding.table, type_name)
        required.add(binding.table)
    return required, topic_entries, image_topic_types


def _units_for(topic_entries: dict[str, tuple[str, str]], topic_ids: dict[str, int],
               image_topics: set[str]) -> dict[str, tuple]:
    """이번 실행이 적재할 수 있는 단위 — 토픽 → (테이블, topic_id)."""
    units = {t: (table, topic_ids[t]) for t, (table, _) in topic_entries.items()}
    units.update({t: ('image_streams', topic_ids[t]) for t in image_topics if t in topic_ids})
    return units


def _replace_topics(cfg: DatasetConfig, image_topics: set[str]) -> Optional[frozenset]:
    """`--only-topics-replace` 대상. 꺼져 있으면 ``None``."""
    if not cfg.only_topics_replace:
        return None
    selection = frozenset(cfg.topic_selection or ()) - {SESSION_TOPIC}
    if not selection:
        raise PostProcError('only_topics_replace needs an explicit topic selection (--topics)')
    bag_images = selection & image_topics
    if bag_images:
        # bag 이미지 인코딩 mp4 는 에피소드 디렉터리 단위로 쓰여 토픽 하나만 갈아 끼울 수 없다.
        raise PostProcError(f'cannot replace bag image topics one by one: {sorted(bag_images)}; '
                            f'use on_existing_episode=replace')
    return selection


def _media_selection(cfg: DatasetConfig) -> Optional[frozenset]:
    """레코더 mp4 를 어느 카메라까지 등록하나. 토픽을 고르지 않았으면 전부(``None``)."""
    if cfg.topic_selection is None:
        return None
    return frozenset(cfg.topic_selection)


def _summary_topic_types(splits: list, want_topics: Optional[set]) -> dict[str, str]:
    """split 들의 MCAP summary 에서 토픽 → 타입. summary 가 없는 split 이 있으면 빈 dict —
    호출자가 메시지를 풀어 알아낸다 (쓰다 끊긴 파일)."""
    from mcap.reader import make_reader

    found: dict[str, str] = {}
    for sp in splits:
        try:
            with open(sp.path, 'rb') as fp:
                summary = make_reader(fp).get_summary()
        except Exception:  # noqa: BLE001 — 읽을 수 없으면 느린 길로 간다
            return {}
        if summary is None:
            return {}
        for ch in summary.channels.values():
            if want_topics is not None and ch.topic not in want_topics:
                continue
            schema = summary.schemas.get(ch.schema_id)
            if schema is not None:
                found.setdefault(ch.topic, schema.name)
    return found


def _settle_finished(conn: psycopg.Connection, episodes: list[Episode], *, cfg: DatasetConfig,
                     units: dict, registrar, origins: dict,
                     replace_topics: Optional[frozenset], media_only: Optional[frozenset],
                     summary: dict, run_log: RunLog) -> list[Episode]:
    """이미 **전부** 적재된 에피소드를 bag 을 읽기 전에 정리하고, 일이 남은 것만 돌려준다.

    다시 돌린 `import` 가 아무것도 안 넣으면서 bag 을 통째로 읽던 것을 막는다(실측 79 초 → 몇 초).
    출처 연결(adopt)과 옛 단위의 이력 채우기는 여기서 커밋된다.
    """
    if replace_topics is not None or cfg.on_existing_episode != 'skip':
        return episodes
    sess_writer = SessionWriter(conn)
    remaining: list[Episode] = []
    for ep in episodes:
        if sess_writer.find_existing(ep.start_ns) is None:
            remaining.append(ep)
            continue
        opening = open_episode(conn, sess_writer, ep, origin=origins.get(ep.start_ns),
                               policy='skip', units=units)
        missing = set(units) - set(opening.skip_topics)
        if not missing and registrar is not None:
            missing = set(registrar.pending(opening.episode_id, ep.start_ns,
                                            only_topics=media_only))
        if missing:
            conn.rollback()
            remaining.append(ep)
            continue
        conn.commit()
        summary['skipped'] += 1
        if opening.adopted:
            summary['adopted'] = summary.get('adopted', 0) + 1
        run_log.episode_skipped(episode_id=opening.episode_id, start_ns=ep.start_ns,
                                stop_ns=ep.stop_ns, task_label=ep.task_label)
    settled = len(episodes) - len(remaining)
    if settled:
        _logger.info('%d episode(s) already fully loaded; %d left to stream',
                     settled, len(remaining))
    return remaining


def _run_ingestion(conn: psycopg.Connection, splits: list, episodes: list[Episode], *,
                   topic_entries: dict[str, tuple[str, str]],
                   topic_ids: dict[str, int],
                   image_topics: set[str],
                   router: Optional['FrameRouter'],
                   run_log: RunLog,
                   cfg: DatasetConfig,
                   summary: dict,
                   origins: Optional[dict] = None) -> None:
    """에피소드 단위 트랜잭션으로 실제 INSERT 와 mp4 인코딩을 수행한다.

    무엇을 넣고 무엇을 건너뛸지는 `units.open_episode` 가 정한다 (병렬 경로와 같은 함수).
    `router` 는 bag 에 이미지 토픽이 있을 때만 있다.
    """
    sess_writer = SessionWriter(conn)
    # 레코더 출력은 **루프 밖에서 한 번만 훑는다** — 에피소드마다 디렉터리를 다시
    # 읽으면 파일 수에 비례해 느려진다. 설정에 경로가 없으면 등록하지 않는다(I4).
    registrar = (RecorderRegistrar(conn, cfg.recordings_dir)
                 if cfg.recordings_dir else None)
    origins = origins or {}
    units = _units_for(topic_entries, topic_ids, image_topics)
    replace_topics = _replace_topics(cfg, image_topics)
    media_only = _media_selection(cfg)

    episodes = _settle_finished(conn, episodes, cfg=cfg, units=units, registrar=registrar,
                                origins=origins, replace_topics=replace_topics,
                                media_only=media_only, summary=summary, run_log=run_log)
    if not episodes:
        return

    # topic → writer instance. 각 writer 가 자신의 table 속성을 갖는다.
    writers: dict[str, WriterBase] = {
        topic: make_writer(conn, table_name, type_name, topic_ids[topic])
        for topic, (table_name, type_name) in topic_entries.items()
    }

    # 현재 에피소드 컨텍스트.
    cur_episode_id: Optional[int] = None
    cur_episode_ep: Episode | None = None
    cur_opening: Optional[Opening] = None
    cur_skip: frozenset = frozenset()

    def _fail(ep: Episode, error: Exception, what: str) -> None:
        _rollback_all(writers, router, conn)
        _logger.exception(what)
        run_log.episode_failed(start_ns=ep.start_ns, stop_ns=ep.stop_ns,
                               task_label=ep.task_label, error=str(error))

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
                cur_opening = open_episode(
                    conn, sess_writer, ep, origin=origins.get(ep.start_ns),
                    policy=cfg.on_existing_episode, units=units,
                    replace_topics=replace_topics,
                    remove_episode_dir=(router.remove_existing_episode_dir
                                        if router is not None else None))
            except Exception as e:
                _fail(ep, e, f'failed to open episode {ev.episode_index}')
                raise
            cur_episode_id = cur_opening.episode_id
            cur_skip = cur_opening.skip_topics
            if cur_opening.replaced:
                summary['replaced'] += 1
            if cur_opening.adopted:
                summary['adopted'] = summary.get('adopted', 0) + 1
            if router is not None:
                router.open_episode(cur_episode_id)
        elif kind == 'message':
            am: AssignedMessage = payload
            if am.topic == SESSION_TOPIC or am.topic in cur_skip:
                continue
            if am.topic in image_topics:
                if router is None:
                    continue
                try:
                    router.on_image(am.topic, am.type_name, am.msg)
                except Exception as e:
                    assert cur_episode_ep is not None
                    _fail(cur_episode_ep, e, 'image encoding failed')
                    raise
                continue
            writer = writers.get(am.topic)
            if writer is None:
                continue
            try:
                writer.append(cur_episode_id, am.msg)
            except Exception as e:
                assert cur_episode_ep is not None
                _fail(cur_episode_ep, e,
                      f'insert failed for topic={am.topic} (type={am.type_name}) '
                      f'in episode {am.episode_index}')
                raise
        elif kind == 'exit':
            assert cur_episode_ep is not None and cur_opening is not None
            try:
                for w in writers.values():
                    w.flush()
                mp4_files = router.finalize_episode() if router is not None else []
                row_counts: dict[str, int] = {}
                written: dict[str, tuple] = {}
                for topic, w in writers.items():
                    n = w.consume_inserted_count()
                    if topic not in cur_skip:
                        written[topic] = (w.table, topic_ids[topic], n)
                        row_counts[w.table] = row_counts.get(w.table, 0) + n
                finish_units(conn, cur_opening, written)
                # FrameRouter 가 finalize 단계에서 INSERT 한 image_streams / image_frames 행도
                # 합산한다 (message-writer 가 아닌 경로라 위 루프에서 누락된다).
                if router is not None:
                    for table, n in router.consume_inserted_count().items():
                        row_counts[table] = row_counts.get(table, 0) + n
                # **레코더 mp4 등록은 커밋 직전, 같은 트랜잭션 안에서** (I4). 이미 등록된
                # 카메라는 건너뛴다 — 두 번 넣으면 UNIQUE 위반으로 에피소드가 통째로 실패한다.
                if registrar is not None:
                    register_recordings(conn, registrar, cur_opening,
                                        cur_episode_ep.start_ns, only_topics=media_only)
                    # 레코더 등록분도 합산한다 — 빼먹으면 DB 에는 행이 있는데 요약은 0 이다.
                    for table, n in registrar.consume_inserted_count().items():
                        row_counts[table] = row_counts.get(table, 0) + n
                conn.commit()
            except Exception as e:
                _fail(cur_episode_ep, e, f'commit failed for episode_id={cur_episode_id}')
                raise
            for table, n in row_counts.items():
                summary['inserted'][table] = summary['inserted'].get(table, 0) + n
            nothing_new = cur_opening.existed and not cur_opening.replaced and not any(
                row_counts.values())
            if nothing_new:
                # 이미 전부 적재된 에피소드다 — 예전 `skip` 과 같은 결과. 연결(adopt)은 커밋됐다.
                summary['skipped'] += 1
                _logger.info('episode already present (start_ns=%d, id=%d); nothing new',
                             cur_episode_ep.start_ns, cur_episode_id)
                run_log.episode_skipped(episode_id=cur_episode_id,
                                        start_ns=cur_episode_ep.start_ns,
                                        stop_ns=cur_episode_ep.stop_ns,
                                        task_label=cur_episode_ep.task_label)
            else:
                summary['episodes'] += 1
                summary['mp4_files'] += len(mp4_files)
                _logger.info('episode committed: id=%d rows=%s mp4=%d',
                             cur_episode_id, row_counts, len(mp4_files))
                run_log.episode_done(episode_id=cur_episode_id,
                                     start_ns=cur_episode_ep.start_ns,
                                     stop_ns=cur_episode_ep.stop_ns,
                                     task_label=cur_episode_ep.task_label,
                                     row_counts=row_counts, mp4_files=mp4_files)
            cur_episode_id = None
            cur_episode_ep = None
            cur_opening = None
            cur_skip = frozenset()


def _rollback_all(
    writers: dict[str, WriterBase],
    router: Optional['FrameRouter'],
    conn: psycopg.Connection,
) -> None:
    """에피소드 트랜잭션 실패 시 모든 상태를 되돌린다."""
    for w in writers.values():
        w.drop_pending()
    if router is not None:
        router.abort_episode()
    conn.rollback()


__all__ = ['PostProcError']
