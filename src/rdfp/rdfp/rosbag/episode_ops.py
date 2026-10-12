"""rosbag 에 기록된 에피소드의 선택 · 적재 여부 판정 · 삭제.

`rosbag` CLI 의 세 서브커맨드(`list-episodes` / `export` / `delete`)가 공유하는 로직이다.
ROS 를 요구하지 않고, DB 는 **선택적**으로만 쓴다.

## 시간 기준이 둘인 이유

| 기준 | 값 | 쓰는 곳 |
|---|---|---|
| **log time** | rosbag 이 메시지를 **기록한** 시각. 언제나 녹화기의 벽시계 | 목록 정렬 · `--since`/`--until` |
| **stamp** | 발행자가 찍은 `header.stamp` | DB 자연키 (`episodes.start_sec/start_nanosec`) |

사용자가 지정하는 시간 범위는 log time 이다 — `header.stamp` 는 경과시간이 섞이는 회차가
있어 사람이 아는 시각과 어긋날 수 있다. 반면 DB 조회는 stamp 로만 가능하다. 그래서 선택은
log time 으로 하고, 그 결과를 stamp 키로 바꿔 DB·적재 경로에 넘긴다.

## 삭제는 split 을 다시 쓴다

split 은 기본 1시간 단위라 **에피소드 하나보다 훨씬 크다.** 통째로 지우면 같은 split 의
다른 에피소드가 함께 사라지므로, MCAP 를 읽어 **지울 구간의 메시지만 빼고 새로 쓴다.**
메시지 본문은 열지 않고 바이트째 옮기므로 스키마·인코딩이 보존된다.
"""

from __future__ import annotations

from typing import NamedTuple, Optional, Sequence

import logging
import os
import tempfile
from pathlib import Path

import yaml
from mcap.reader import make_reader
from mcap.writer import Writer

from rdfp.dataset.episode_key import fetch_exported_keys, key_range
from rdfp.dataset.ingest.episode.from_bag import BagEpisode, episodes_from_splits

_logger = logging.getLogger(__name__)

# `list-episodes` 의 export 열. DB 를 보지 않았을 때를 '아니오' 와 반드시 구분한다 —
# 뭉개면 "확인 못 함" 이 "적재 안 됨" 으로 읽혀 재적재를 유발한다.
EXPORT_YES = 'yes'
EXPORT_NO = 'no'
EXPORT_UNKNOWN = 'unknown'


class EpisodeView(NamedTuple):
    """목록 한 줄. `bag` 의 값에 적재 여부를 덧붙인 것이다."""

    index: int                 # 목록 내 순번 (1-base). **안정적이지 않다** — 아래 주 참조
    bag: BagEpisode
    exported: str              # EXPORT_YES / EXPORT_NO / EXPORT_UNKNOWN

    @property
    def start_ns(self) -> int:
        return self.bag.start_ns


def build_views(episodes: Sequence[BagEpisode],
                exported_keys: Optional[set[int]]) -> list[EpisodeView]:
    """에피소드 목록에 적재 여부를 붙인다.

    Args:
        episodes: `episodes_from_splits` 결과 (log time 오름차순).
        exported_keys: DB 에 있는 `start_ns` 집합. **`None` 이면 확인하지 않았다는
            뜻**이며 모든 행이 `EXPORT_UNKNOWN` 이 된다.
    """
    views: list[EpisodeView] = []
    for i, be in enumerate(episodes, start=1):
        if exported_keys is None:
            flag = EXPORT_UNKNOWN
        else:
            flag = EXPORT_YES if be.start_ns in exported_keys else EXPORT_NO
        views.append(EpisodeView(index=i, bag=be, exported=flag))
    return views


def load_episodes(rosbag_dir: str | Path, *, dates: Optional[list[str]] = None,
                  ) -> list[BagEpisode]:
    """`rosbag_dir` 을 스캔해 에피소드를 log time 오름차순으로 돌려준다."""
    from .catalog import discover_splits

    splits = discover_splits(rosbag_dir, dates=dates)
    if not splits:
        return []
    return episodes_from_splits(splits)


def lookup_exported(episodes: Sequence[BagEpisode], db_cfg) -> set[int]:
    """DB 에서 적재된 `start_ns` 집합을 가져온다 (질의 한 번).

    Args:
        episodes: 조회 구간을 좁히는 데 쓴다 (최소/최대 `start_ns`).
        db_cfg: `rdfp.dataset.db.config.DbConfig`. DSN 은 그 안의 환경변수 이름으로
            해석된다 — 평문 DSN 을 주고받지 않는다.
    """
    from rdfp.dataset.db.connection import open_connection

    lo, hi = key_range(be.start_ns for be in episodes)
    with open_connection(db_cfg) as conn:
        return fetch_exported_keys(conn, lo, hi)


# --- 선택 -------------------------------------------------------------------


def select(views: Sequence[EpisodeView], *,
           since_ns: Optional[int] = None,
           until_ns: Optional[int] = None,
           task: Optional[str] = None,
           only_exported: bool = False,
           only_new: bool = False) -> list[EpisodeView]:
    """조건을 만족하는 에피소드만 고른다.

    시간 범위는 **log time** 기준이고 `[since_ns, until_ns]` 양끝 포함이다. 에피소드가
    구간에 **걸치기만 해도** 포함된다 — 경계에 걸린 에피소드를 반쪽만 다루면 데이터가
    조용히 잘리기 때문이다.

    Raises:
        ValueError: `only_exported` / `only_new` 를 요구했는데 적재 여부가
            `EXPORT_UNKNOWN` 인 행이 있을 때. 모르는 상태로 삭제·적재를 진행하면
            안 된다.
    """
    if only_exported and only_new:
        raise ValueError('only_exported and only_new are mutually exclusive')

    out: list[EpisodeView] = []
    for v in views:
        be = v.bag
        if since_ns is not None and be.log_stop_ns < since_ns:
            continue
        if until_ns is not None and be.log_start_ns > until_ns:
            continue
        if task is not None and be.task_label != task:
            continue
        if only_exported or only_new:
            if v.exported == EXPORT_UNKNOWN:
                raise ValueError(
                    'export status is unknown; pass --config so the database can be '
                    'checked before selecting by export state')
            if only_exported and v.exported != EXPORT_YES:
                continue
            if only_new and v.exported != EXPORT_NO:
                continue
        out.append(v)
    return out


def parse_index_spec(spec: str, views: Sequence[EpisodeView]) -> list[EpisodeView]:
    """`3-7,9` 같은 순번 표기를 실제 행으로 바꾼다.

    ⚠️ **순번은 안정적이지 않다** — `--dates` 를 바꾸거나 새 세션이 추가되면 같은 번호가
    다른 에피소드를 가리킨다. 사람이 목록을 보고 바로 고르는 경우에만 쓰고, 스크립트에는
    `--start-ns` 를 쓴다.
    """
    by_index = {v.index: v for v in views}
    picked: dict[int, EpisodeView] = {}
    for part in spec.split(','):
        part = part.strip()
        if not part:
            continue
        if '-' in part:
            lo_s, hi_s = part.split('-', 1)
            lo, hi = int(lo_s), int(hi_s)
            if lo > hi:
                raise ValueError(f'invalid range: {part}')
            rng = range(lo, hi + 1)
        else:
            rng = range(int(part), int(part) + 1)
        for i in rng:
            if i not in by_index:
                raise ValueError(f'no episode with index {i} (1..{len(views)})')
            picked[i] = by_index[i]
    return [picked[i] for i in sorted(picked)]


# --- 삭제 (MCAP 재작성) ------------------------------------------------------


class DeleteReport(NamedTuple):
    """split 하나를 다시 쓴 결과."""

    split_path: str
    removed: int               # 제거한 메시지 수
    kept: int                  # 남긴 메시지 수
    removed_episodes: int      # 이 split 에서 사라진 에피소드 수


def _windows_for(views: Sequence[EpisodeView]) -> list[tuple[int, int]]:
    """log time 기준 삭제 구간 목록 (양끝 포함).

    경계의 `/session` 메시지까지 함께 지운다 — 시작 이벤트가 남으면 목록에 껍데기
    에피소드가 계속 보이고, 종료 이벤트가 남으면 다음 에피소드의 경계 판정이 틀어진다.
    """
    return sorted((v.bag.log_start_ns, v.bag.log_stop_ns) for v in views)


def _in_windows(log_ns: int, windows: Sequence[tuple[int, int]]) -> bool:
    for lo, hi in windows:
        if lo <= log_ns <= hi:
            return True
        if log_ns < lo:
            break
    return False


def rewrite_split(split_path: str | Path, windows: Sequence[tuple[int, int]],
                  ) -> tuple[int, int, int, int, dict[str, int]]:
    """`windows` 에 드는 메시지를 뺀 새 MCAP 을 만들어 원본을 대체한다.

    메시지 본문은 **역직렬화하지 않고** 바이트째 옮긴다. 스키마·채널도 원본 레코드를
    그대로 재등록하므로 인코딩과 메타데이터가 보존된다.

    Returns:
        `(removed, kept, new_start_ns, new_end_ns, per_topic_kept)`.
        남은 메시지가 없으면 `new_start_ns`/`new_end_ns` 는 `0` 이다.
    """
    src = Path(split_path)
    removed = kept = 0
    new_start = new_end = 0
    per_topic: dict[str, int] = {}

    fd, tmp_name = tempfile.mkstemp(prefix=src.name + '.', suffix='.tmp', dir=str(src.parent))
    os.close(fd)
    tmp = Path(tmp_name)
    try:
        with src.open('rb') as fin, tmp.open('wb') as fout:
            reader = make_reader(fin)
            header = reader.get_header()
            writer = Writer(fout)
            writer.start(profile=header.profile, library=header.library)

            schema_ids: dict[int, int] = {}
            channel_ids: dict[int, int] = {}
            for schema, channel, message in reader.iter_messages():
                if _in_windows(message.log_time, windows):
                    removed += 1
                    continue
                if schema is not None and schema.id not in schema_ids:
                    schema_ids[schema.id] = writer.register_schema(
                        name=schema.name, encoding=schema.encoding, data=schema.data)
                if channel.id not in channel_ids:
                    channel_ids[channel.id] = writer.register_channel(
                        topic=channel.topic,
                        message_encoding=channel.message_encoding,
                        schema_id=schema_ids.get(schema.id if schema else -1, 0),
                        metadata=dict(channel.metadata or {}),
                    )
                writer.add_message(
                    channel_id=channel_ids[channel.id],
                    log_time=message.log_time,
                    data=message.data,
                    publish_time=message.publish_time,
                    sequence=message.sequence,
                )
                kept += 1
                per_topic[channel.topic] = per_topic.get(channel.topic, 0) + 1
                if new_start == 0:
                    new_start = message.log_time
                else:
                    new_start = min(new_start, message.log_time)
                new_end = max(new_end, message.log_time)
            writer.finish()
        os.replace(tmp, src)
    except BaseException:
        tmp.unlink(missing_ok=True)
        raise
    return removed, kept, new_start, new_end, per_topic


def update_session_metadata(session_path: str | Path) -> None:
    """재작성된 split 에 맞춰 `metadata.yaml` 의 개수·시각을 다시 계산한다.

    `discover_splits` 가 이 파일을 읽으므로 고치지 않으면 카탈로그가 낡은 값을 보고한다.
    아는 키만 갱신하고 나머지는 그대로 둔다 — rosbag2 가 쓰는 다른 필드를 지우지 않기
    위해서다.
    """
    session = Path(session_path)
    meta_path = session / 'metadata.yaml'
    with meta_path.open('r', encoding='utf-8') as f:
        raw = yaml.safe_load(f) or {}
    info = raw.get('rosbag2_bagfile_information')
    if not isinstance(info, dict):
        return

    files = info.get('files') or []
    total = 0
    per_topic: dict[str, int] = {}
    starts: list[int] = []
    ends: list[int] = []
    for entry in files:
        if not isinstance(entry, dict):
            continue
        rel = entry.get('path')
        if not isinstance(rel, str):
            continue
        path = session / rel
        if not path.exists():
            entry['message_count'] = 0
            continue
        count = 0
        lo = hi = 0
        with path.open('rb') as f:
            reader = make_reader(f)
            for _schema, channel, message in reader.iter_messages():
                count += 1
                per_topic[channel.topic] = per_topic.get(channel.topic, 0) + 1
                lo = message.log_time if lo == 0 else min(lo, message.log_time)
                hi = max(hi, message.log_time)
        entry['message_count'] = count
        if count:
            entry['starting_time'] = {'nanoseconds_since_epoch': lo}
            entry['duration'] = {'nanoseconds': max(hi - lo, 1)}
            starts.append(lo)
            ends.append(hi)
        total += count

    info['message_count'] = total
    if starts:
        info['starting_time'] = {'nanoseconds_since_epoch': min(starts)}
        info['duration'] = {'nanoseconds': max(max(ends) - min(starts), 1)}
    for entry in info.get('topics_with_message_count') or []:
        if not isinstance(entry, dict):
            continue
        name = (entry.get('topic_metadata') or {}).get('name')
        if isinstance(name, str):
            entry['message_count'] = per_topic.get(name, 0)

    with meta_path.open('w', encoding='utf-8') as f:
        yaml.safe_dump(raw, f, sort_keys=False, allow_unicode=True)


def delete_episodes(rosbag_dir: str | Path, views: Sequence[EpisodeView],
                    ) -> tuple[list[DeleteReport], list[int]]:
    """선택된 에피소드를 bag 에서 제거한다.

    split 을 다시 쓴 뒤 **에피소드를 다시 감지해**, 지우려던 것만 사라지고 나머지가
    그대로인지 확인한다. 그 확인이 이 함수의 안전장치다.

    Returns:
        `(split 별 보고, 실제로 사라진 start_ns 목록)`.

    Raises:
        RuntimeError: 재감지 결과가 기대와 다를 때 (남아야 할 에피소드가 사라졌거나
            지워야 할 것이 남았을 때).
    """
    if not views:
        return [], []

    targets = {v.bag.start_ns for v in views}
    before = {be.start_ns for be in load_episodes(rosbag_dir)}
    expected_after = before - targets

    windows = _windows_for(views)
    affected: dict[str, list[tuple[int, int]]] = {}
    for v in views:
        for sp in v.bag.split_paths:
            affected.setdefault(sp, [])
    for sp in affected:
        affected[sp] = windows

    reports: list[DeleteReport] = []
    sessions: set[str] = set()
    for sp, wins in sorted(affected.items()):
        removed, kept, _s, _e, _pt = rewrite_split(sp, wins)
        reports.append(DeleteReport(split_path=sp, removed=removed, kept=kept,
                                    removed_episodes=0))
        sessions.add(str(Path(sp).parent))
        _logger.info('rewrote %s: removed %d, kept %d', sp, removed, kept)

    for sess in sorted(sessions):
        update_session_metadata(sess)

    after = {be.start_ns for be in load_episodes(rosbag_dir)}
    if after != expected_after:
        lost = expected_after - after
        left = after & targets
        raise RuntimeError(
            f'episode set after delete does not match expectation; '
            f'unexpectedly lost={sorted(lost)}, still present={sorted(left)}')
    return reports, sorted(targets)


__all__ = [
    'EXPORT_YES', 'EXPORT_NO', 'EXPORT_UNKNOWN',
    'EpisodeView', 'DeleteReport',
    'build_views', 'load_episodes', 'lookup_exported', 'select', 'parse_index_spec',
    'rewrite_split', 'update_session_metadata', 'delete_episodes',
]
