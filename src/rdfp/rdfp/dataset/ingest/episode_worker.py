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

import logging
from pathlib import Path
from typing import Optional

from ..config import SESSION_TOPIC, DatasetConfig
from rdfp.dataset.db.connection import open_connection
from rdfp.dataset.db.registry import resolve_message_type
from rdfp.dataset.db.writers.base import WriterBase
from rdfp.dataset.db.writers.session_command import SessionWriter
from rdfp.rosbag.catalog import Split
from rdfp.rosbag.merged_stream import merge_split_streams
from .episode.detector import Episode
from .exceptions import PostProcError
from .run_log import RunLog
from .media.frame_router import FrameRouter


_logger = logging.getLogger(__name__)


class EpisodeResult(dict):
    """에피소드 처리 결과 요약 (기본 dict + 이름)."""


def process_episode(cfg: DatasetConfig, episode: Episode, split_paths: list[str],
                    topic_entries: dict[str, tuple[str, str]],
                    topic_ids: dict[str, int],
                    image_topics: set[str], run_log_path: Path,) -> dict:
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
                                        image_topics,)
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
                           image_topics: set[str],) -> dict:
    """실제 처리 로직. 예외는 호출자가 잡는다.

        DB 트랜잭션과 FrameRouter 컨텍스트를 열고, 에피소드 구간 메시지를 읽어
        writer / router 로 분배한다. 에피소드가 이미 존재할 때의 정책은
        `cfg.on_existing_episode` 에 따라 분기한다.

    Args:
        cfg: postproc 설정.
        episode: 처리할 에피소드 정보.
        split_paths: 에피소드 구간을 포함하는 rosbag split 파일 경로 목록.
        topic_entries: **토픽 이름 → (table_name, type_name)** 매핑.
        topic_ids: **토픽 이름 → topics.id** 매핑.
        image_topics: mp4 라우팅 대상 토픽 이름 집합.
    Returns:
        dict: 에피소드 처리 결과 요약. `status` 필드는 'done' 또는 'skipped' 이며,
            'done' 인 경우 `episode_id`, `inserted`, `mp4_files` 필드가 포함된다.
    Raises:
        PostProcError: 처리 실패 사유가 명확할 때.
        Exception: 그 외 처리 중 예외 발생 시.
    """
    # 에피소드 시간 구간만 스트리밍 (mcap_reader 의 time_range_ns 필터 사용).
    ep_range = (episode.start_ns, episode.stop_ns)

    # 이 워커가 읽어야 하는 토픽 (세션 토픽은 필요 없음). cfg.effective_topics()
    # 로 전역 대상 토픽을 얻되, 세션 토픽은 에피소드 경계가 이미 확정된 상태이
    # 므로 재수신하지 않는다.
    want_topics = set(cfg.effective_topics()) if cfg.effective_topics() else set()
    if SESSION_TOPIC in want_topics:
        want_topics.discard(SESSION_TOPIC)

    with open_connection(cfg.db) as conn:
        # FrameRouter 가 sink 에 conn / topic_id 를 주입해야 하므로 conn 보다
        # 늦게 생성한다.
        router = FrameRouter(
            Path(cfg.output_mp4_dir),
            conn=conn,
            topic_ids={t: topic_ids[t] for t in image_topics if t in topic_ids},
            fps=cfg.mp4.nominal_fps, codec=cfg.mp4.codec,
        )
        sess_writer = SessionWriter(conn)
        try:
            episode_id = _open_episode(sess_writer, episode, cfg.on_existing_episode, router)
        except Exception:
            conn.rollback()
            raise

        if episode_id is None:
            conn.rollback()
            return {
                'status': 'skipped',
                'start_ns': episode.start_ns, 'stop_ns': episode.stop_ns,
            }

        # topic → writer instance. 토픽별 테이블이 다를 수 있으므로 table 을 인스턴스에 바인딩한다.
        # topic_id 도 함께 주입하여 INSERT 시 어느 토픽의 row 인지 기록한다.
        writers: dict[str, WriterBase] = {}
        for topic, (table_name, type_name) in topic_entries.items():
            binding = resolve_message_type(type_name)
            if binding is None:
                continue
            writers[topic] = binding.writer_cls(
                conn, table=table_name, topic_id=topic_ids[topic],
            )

        router.open_episode(episode_id)
        try:
            _ingest_messages(
                split_paths, ep_range, want_topics, image_topics,
                writers, router, episode_id,
            )
            for w in writers.values():
                w.flush()
            mp4_files = router.finalize_episode()
            conn.commit()
        except Exception:
            for w in writers.values():
                w.drop_pending()
            router.abort_episode()
            conn.rollback()
            raise

        inserted: dict[str, int] = {}
        for w in writers.values():
            inserted[w.table] = w.consume_inserted_count()

    return {
        'status': 'done',
        'episode_id': int(episode_id),
        'inserted': inserted,
        'mp4_files': list(mp4_files),
        'start_ns': episode.start_ns,
        'stop_ns': episode.stop_ns,
    }


def _ingest_messages(split_paths: list[str], ep_range: tuple[int, int], want_topics: set[str],
                     image_topics: set[str], writers: dict[str, WriterBase],
                     router: FrameRouter, episode_id: int) -> None:
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
        if m.topic == SESSION_TOPIC:
            continue
        if m.topic in image_topics:
            router.on_image(m.topic, m.type_name, m.msg)
            continue
        writer = writers.get(m.topic)
        if writer is None:
            continue
        writer.append(episode_id, m.msg)


def _open_episode(sess_writer: SessionWriter, ep: Episode, policy: str,
                  router: FrameRouter,) -> Optional[int]:
    """sessions INSERT + 정책 분기. 상세는 pipeline._open_episode 와 동일.

        단일 에피소드 트랜잭션의 첫 단계. 커밋은 `exit` 단계에서 수행되며,
        호출자(Pool.imap_unordered) 가 다른 에피소드를 계속 처리할 수 있게 한다.

    Args:
        sess_writer: SessionWriter 인스턴스. DB 커넥션이 열려있어야 한다.
        ep: 처리할 에피소드 정보.
        policy: 에피소드가 이미 존재할 때의 정책. 'skip', 'error', 'replace' 중 하나여야 한다.
        router: FrameRouter 인스턴스. 'replace' 정책에서 기존 에피소드의 mp4 파일을
            삭제하는 데 사용된다.

    Returns:
        int: 확보된 에피소드 ID. 새로 INSERT 했든 기존 ID 를 재사용했든 ID 는 반환된다.
        None: 'skip' 정책으로 기존 에피소드가 존재할 때. 이 경우 호출자에게 None 이 반환되며,
            호출자는 이를 처리하지 않고 다음 에피소드를 계속 처리할 수 있다.
    Raises:
        PostProcError: 'error' 정책으로 기존 에피소드가 존재할 때,
            또는 알 수 없는 정책이 주어졌을 때.
    """
    existing = sess_writer.find_existing(ep.start_ns)
    if existing is None:
        return sess_writer.insert_episode(ep.start_ns, ep.stop_ns, ep.task_label)
    if policy == 'skip':
        return None
    if policy == 'error':
        raise PostProcError(
            f'episode already present (start_ns={ep.start_ns}, id={existing})'
        )
    if policy == 'replace':
        sess_writer.delete_by_id(existing)
        router.remove_existing_episode_dir(existing)
        return sess_writer.insert_episode(ep.start_ns, ep.stop_ns, ep.task_label)
    raise PostProcError(f'unknown on_existing_episode policy: {policy!r}')


def serialize_splits(splits: list[Split]) -> list[str]:
    """Pool 워커에 넘기기 위한 split 경로 목록만 추출한다."""
    return [str(s.path) for s in splits]


__all__ = ['EpisodeResult', 'process_episode', 'serialize_splits']
