"""에피소드 단위 독립 워커.

`multiprocessing.Pool` 로 분산 실행이 가능하도록 **module-level** 함수로
분리되어 있다. 각 워커는 자신의 DB 커넥션과 FrameRouter 를 새로 열고,
단일 에피소드의 [start_ns, stop_ns) 구간만 처리한 뒤 요약을 반환한다.

장점:
  * 에피소드 간 DB 트랜잭션이 독립이므로 한 에피소드 실패가 다른 에피소드에
    영향을 주지 않는다.
  * mcap-ros2 의 time-range 필터를 활용해 각 워커가 자기 몫만 읽는다.
  * pickle 호환: `DatasetConfig` 와 `Episode` 모두 plain data 이므로 Pool 에서
    직렬화가 가능하다.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

import logging
from pathlib import Path

from ..config import SESSION_TOPIC, DatasetConfig
from rdfp.dataset.db.connection import open_connection
from rdfp.dataset.db.writers.base import WriterBase
from rdfp.dataset.db.writers.session_command import SessionWriter
from rdfp.rosbag.catalog import Split
from rdfp.rosbag.merged_stream import merge_split_streams
from .episode.detector import Episode
from .exceptions import PostProcError
from .run_log import RunLog
from .media.recorder_registry import RecorderRegistrar
from .units import Origin, finish_units, make_writer, open_episode, register_recordings

if TYPE_CHECKING:
    from .media.frame_router import FrameRouter


_logger = logging.getLogger(__name__)


class EpisodeResult(dict):
    """에피소드 처리 결과 요약 (기본 dict + 이름)."""


def process_episode(cfg: DatasetConfig, episode: Episode, split_paths: list[str],
                    topic_entries: dict[str, tuple[str, str]],
                    topic_ids: dict[str, int],
                    image_topics: set[str], run_log_path: Path,
                    origin: Optional[Origin] = None) -> dict:
    """단일 에피소드를 처리해 DB / mp4 에 적재한다.

    본 함수는 multiprocessing 워커로 호출될 수 있도록 모든 입력이 pickle
    가능하도록 설계되어 있다. `topic_entries` 는 **토픽 → (table, type_name)**
    매핑으로, writer 클래스 객체가 아닌 문자열만 담아 직렬화 부담이 없다.
    `topic_ids` 는 **토픽 → topics.id** 매핑으로, 각 writer 인스턴스의
    `topic_id` 로 주입된다. 예외는 **반환값의 `status='failed'`** 로 변환해
    호출자(Pool.imap_unordered) 가 다른 에피소드를 계속 처리할 수 있게 한다.

    Args:
        cfg: postproc 설정. Pool 워커가 직렬화할 수 있도록 pickle-friendly 하게 설계된 객체.
        episode: 처리할 에피소드 정보. `start_ns`, `stop_ns`, `task_label` 을 담은 객체.
        split_paths: 에피소드 구간을 포함하는 rosbag split 파일 경로 목록.
            `merge_split_streams()` 의 입력으로 사용된다.
        topic_entries: **토픽 이름 → (table_name, type_name)** 매핑.
            각 topic_name 이 DB 의 어느 table 에 어떤 type_name 으로 저장되는지 명시한다.
            writer 클래스는 pickle 우려가 없도록 여기선 담지 않고, 실제 writer 인스턴스에서
            사용된다.
        topic_ids: **토픽 이름 → topics.id** 매핑.
            각 writer 인스턴스에 주입되어 INSERT 시 어느 토픽의 row 인지 기록하는 데 사용된다.
        image_topics: mp4 라우팅 대상 토픽 이름 집합.
            이 토픽의 메시지는 writer 가 아닌 FrameRouter 로 전달된다.
        run_log_path: 에피소드 처리 결과를 기록할 run log 파일 경로.
            워커는 이 경로에 처리 결과를 기록한다.
    Returns:
        dict: 에피소드 처리 결과 요약. `status` 필드는 'done', 'skipped', 'failed' 중 하나이며,
            'done' 인 경우 `episode_id`, `inserted`, `mp4_files` 필드가 포함된다.
    Raises:
        PostProcError: 처리 실패 사유가 명확할 때. 호출자에게 예외가 전달되면 Pool 전체가
            중단되므로, 예외는 내부에서 잡아서 'failed' 상태로 변환해 반환한다.
        Exception: 그 외 처리 중 예외 발생 시. 마찬가지로 호출자에게 예외가 전달되면
            Pool 전체가 중단되므로, 예외는 내부에서 잡아서 'failed' 상태로 변환해 반환한다
    """
    run_log = RunLog(run_log_path)
    try:
        result = _process_episode_inner(cfg, episode, split_paths, topic_entries, topic_ids,
                                        image_topics, origin)
    except PostProcError as e:
        _logger.error('episode failed: %s', e)
        run_log.episode_failed(start_ns=episode.start_ns, stop_ns=episode.stop_ns,
                               task_label=episode.task_label, error=str(e))
        return {
            'status': 'failed', 'error': str(e),
            'start_ns': episode.start_ns, 'stop_ns': episode.stop_ns,
        }
    except Exception as e:   # noqa: BLE001
        _logger.exception('episode processing crashed')
        run_log.episode_failed(
            start_ns=episode.start_ns, stop_ns=episode.stop_ns,
            task_label=episode.task_label, error=str(e),
        )
        return {
            'status': 'failed', 'error': str(e),
            'start_ns': episode.start_ns, 'stop_ns': episode.stop_ns,
        }

    status = result['status']
    if status == 'done':
        run_log.episode_done(
            episode_id=result['episode_id'],
            start_ns=episode.start_ns, stop_ns=episode.stop_ns,
            task_label=episode.task_label,
            row_counts=result['inserted'],
            mp4_files=result['mp4_files'],
        )
    elif status == 'skipped':
        run_log.episode_skipped(
            episode_id=result.get('episode_id', -1),
            start_ns=episode.start_ns, stop_ns=episode.stop_ns,
            task_label=episode.task_label,
        )
    return result


def _process_episode_inner(cfg: DatasetConfig, episode: Episode, split_paths: list[str],
                           topic_entries: dict[str, tuple[str, str]],
                           topic_ids: dict[str, int],
                           image_topics: set[str],
                           origin: Optional[Origin] = None) -> dict:
    """실제 처리 로직. 예외는 호출자가 잡는다.

    무엇을 넣고 건너뛸지는 직렬 경로와 같은 `units.open_episode` 가 정한다.

    Returns:
        dict: `status` 는 'done' 또는 'skipped'('이미 전부 적재돼 새로 넣은 것이 없다').
    """
    from .pipeline import _media_selection, _replace_topics, _units_for

    # 에피소드 시간 구간만 스트리밍 (mcap_reader 의 time_range_ns 필터 사용).
    ep_range = (episode.start_ns, episode.stop_ns)
    want_topics = set(cfg.effective_topics()) if cfg.effective_topics() else set()
    want_topics.discard(SESSION_TOPIC)
    units = _units_for(topic_entries, topic_ids, image_topics)

    with open_connection(cfg.db) as conn:
        router: Optional['FrameRouter'] = None
        if image_topics:
            from .media.frame_router import FrameRouter

            router = FrameRouter(
                Path(cfg.output_mp4_dir), conn=conn,
                topic_ids={t: topic_ids[t] for t in image_topics if t in topic_ids},
                fps=cfg.mp4.nominal_fps, codec=cfg.mp4.codec)
        sess_writer = SessionWriter(conn)
        try:
            opening = open_episode(
                conn, sess_writer, episode, origin=origin, policy=cfg.on_existing_episode,
                units=units, replace_topics=_replace_topics(cfg, image_topics),
                remove_episode_dir=(router.remove_existing_episode_dir
                                    if router is not None else None))
        except Exception:
            conn.rollback()
            raise
        episode_id = opening.episode_id

        writers: dict[str, WriterBase] = {
            topic: make_writer(conn, table_name, type_name, topic_ids[topic])
            for topic, (table_name, type_name) in topic_entries.items()
            if topic not in opening.skip_topics
        }
        registrar = (RecorderRegistrar(conn, cfg.recordings_dir)
                     if cfg.recordings_dir else None)

        if router is not None:
            router.open_episode(episode_id)
        try:
            # `want_topics` 가 비었으면 '전 토픽'이다 (`effective_topics` 규약). 그때는 걸러
            # 읽지 않고 건너뛸 토픽만 버린다. 골라 읽는데 남은 것이 없으면 읽지 않는다.
            pending = {t for t in want_topics if t not in opening.skip_topics}
            if pending or not want_topics:
                _ingest_messages(split_paths, ep_range, pending, image_topics,
                                 writers, router, episode_id, skip=opening.skip_topics)
            for w in writers.values():
                w.flush()
            mp4_files = router.finalize_episode() if router is not None else []
            inserted: dict[str, int] = {}
            written: dict[str, tuple] = {}
            for topic, w in writers.items():
                n = w.consume_inserted_count()
                written[topic] = (w.table, topic_ids[topic], n)
                inserted[w.table] = inserted.get(w.table, 0) + n
            finish_units(conn, opening, written)
            if router is not None:
                for table, n in router.consume_inserted_count().items():
                    inserted[table] = inserted.get(table, 0) + n
            # **레코더 mp4 등록은 커밋 직전에, 같은 트랜잭션 안에서 한다** (I4).
            if registrar is not None:
                register_recordings(conn, registrar, opening, episode.start_ns,
                                    only_topics=_media_selection(cfg))
                for table, n in registrar.consume_inserted_count().items():
                    inserted[table] = inserted.get(table, 0) + n
            conn.commit()
        except Exception:
            for w in writers.values():
                w.drop_pending()
            if router is not None:
                router.abort_episode()
            conn.rollback()
            raise

    nothing_new = opening.existed and not opening.replaced and not any(inserted.values())
    return {
        'status': 'skipped' if nothing_new else 'done',
        'episode_id': int(episode_id),
        'inserted': inserted,
        'mp4_files': list(mp4_files),
        'replaced': opening.replaced,
        'adopted': opening.adopted,
        'start_ns': episode.start_ns,
        'stop_ns': episode.stop_ns,
    }


def _ingest_messages(split_paths: list[str], ep_range: tuple[int, int], want_topics: set[str],
                     image_topics: set[str], writers: dict[str, WriterBase],
                     router: Optional['FrameRouter'], episode_id: int,
                     skip: frozenset = frozenset()) -> None:
    """에피소드 구간 내 메시지를 writer / router 로 분배한다.

    Args:
        split_paths: 에피소드 구간을 포함하는 rosbag split 파일 경로 목록.
        ep_range: 에피소드 구간 (start_ns, stop_ns).
        want_topics: 이 워커가 처리해야 하는 토픽 이름 집합.
            None 이면 모든 토픽이 대상이다.
        image_topics: mp4 라우팅 대상 토픽 이름 집합.
            이 토픽의 메시지는 writer 가 아닌 FrameRouter 로 전달된다.
        writers: topic → writer instance 매핑.
            각 writer 인스턴스는 자신의 table 속성을 갖는다.
        router: FrameRouter 인스턴스.
            이미지 토픽 메시지를 처리하는 데 사용된다.
        episode_id: 현재 에피소드 ID.
            writer 인스턴스스에 주입되어 INSERT 시 어느 에피소드의 row 인지
            기록하는 데 사용된다.
     Raises:
        PostProcError: 처리 실패 사유가 명확할 때.
        Exception: 그 외 처리 중 예외 발생 시.
    """
    topic_filter = sorted(want_topics) if want_topics else None
    for m in merge_split_streams(split_paths, topics=topic_filter, time_range_ns=ep_range,):
        if m.topic == SESSION_TOPIC or m.topic in skip:
            continue
        if m.topic in image_topics:
            if router is not None:
                router.on_image(m.topic, m.type_name, m.msg)
            continue
        writer = writers.get(m.topic)
        if writer is None:
            continue
        writer.append(episode_id, m.msg)


def serialize_splits(splits: list[Split]) -> list[str]:
    """Pool 워커에 넘기기 위한 split 경로 목록만 추출한다."""
    return [str(s.path) for s in splits]


__all__ = ['EpisodeResult', 'process_episode', 'serialize_splits']
