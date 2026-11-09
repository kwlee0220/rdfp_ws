"""bag 쪽 진단 — 세션 트리 · 토픽 요약 · 메시지 정의 · stamp 진단 · 메시지 열람.

적재 경로에서 데이터가 **조용히** 빠지는 자리를 숫자로 보이게 한다 (설계안 §2 · §4.1). 그래서
세는 기준이 적재와 같다:

* `metadata.yaml` 이 없는 세션은 `discover_splits` 가 건너뛴다 → 여기서는 **보이되 '미완료'로
  표시**한다 (`SessionInfo.finalized`). 지금 CLI 는 이것을 조용히 빼서 "왜 0 개인가" 를 알 수 없다.
* `header.stamp` 가 없는 메시지는 `iter_split_messages` 가 버린다 → `StampDiagnosis.without_stamp`.
  판정 함수를 **그대로 가져다 쓴다** — 따로 쓰면 두 벌이 되어 한쪽만 뒤처진다.
* 에피소드 창 `[start_ns, stop_ns)` 밖은 적재가 버린다 → `StampDiagnosis.in_episode`.

토픽별 타입 · 메시지 수는 `metadata.yaml` 에서 읽는다(빠르다). 메시지 정의와 stamp 진단은 MCAP 을
열어야 하므로 호출자가 **토픽을 골랐을 때** 백그라운드에서 부른다.

ROS 를 요구하지 않는다 — `mcap` · `mcap_ros2` · `yaml` 만 쓴다.
"""

from __future__ import annotations

from typing import Any, Iterable, Optional, Sequence

import bisect
import logging
from dataclasses import dataclass
from pathlib import Path

import yaml
from mcap.reader import make_reader
from mcap_ros2.reader import read_ros2_messages

from rdfp.rosbag.catalog import (DATE_DIR_RE, SESSION_DIR_RE, _MetadataError,
                                 _parse_session_metadata)
from rdfp.rosbag.mcap_reader import _extract_header_stamp_ns
from rdfp.rosbag.merged_stream import merge_split_streams

_logger = logging.getLogger(__name__)

#: 미완료 세션에 붙는 설명. 복구 명령까지 적어 사용자가 다음 할 일을 바로 알게 한다.
NOTE_UNFINALIZED = ('metadata.yaml 이 없다 — 녹화 중이거나 비정상 종료된 세션이라 적재 대상이 '
                    '아니다. 녹화가 끝난 것이 확실하면 `ros2 bag reindex -s mcap <세션 디렉터리>` '
                    '로 복구한다.')


# ----- 세션 트리 -------------------------------------------------------------

@dataclass(frozen=True)
class TopicCount:
    """세션 하나 안의 토픽 한 줄."""

    name: str
    type_name: str
    message_count: int


@dataclass(frozen=True)
class SessionInfo:
    """세션 디렉터리 하나. **미완료 세션도 이 모양으로 돌아온다.**

    Attributes:
        finalized: `metadata.yaml` 이 있고 읽혔는가. `False` 면 적재 대상이 아니다.
        splits: `rdfp.rosbag.catalog.Split` 들. 미완료 세션은 빈 튜플이다.
        mcap_paths: 디렉터리의 `.mcap` 전부 (미완료 세션은 여기서만 내용을 안다).
        note: 왜 적재 대상이 아닌가. 완료 세션은 빈 문자열이다.
    """

    path: Path
    name: str
    date_dir: str
    finalized: bool
    splits: tuple
    mcap_paths: tuple
    message_count: int
    start_ns: Optional[int]
    end_ns: Optional[int]
    topics: tuple
    note: str = ''

    @property
    def duration_ns(self) -> int:
        if self.start_ns is None or self.end_ns is None:
            return 0
        return max(self.end_ns - self.start_ns, 0)

    def topic_types(self) -> dict[str, str]:
        return {t.name: t.type_name for t in self.topics}


def scan_sessions(rosbag_dir: str | Path, *,
                  dates: Optional[Sequence[str]] = None) -> list[SessionInfo]:
    """`<rosbag_dir>/YYYY-MM-DD/session_*/` 를 전부 훑는다 — **미완료 세션을 포함한다.**

    `discover_splits` 와 같은 디렉터리 규칙을 쓰되 `metadata.yaml` 이 없는 세션을 건너뛰지 않고
    `finalized=False` 로 돌려준다. 그 세션의 토픽 · 수는 MCAP summary 에서 읽는다.

    Raises:
        FileNotFoundError: `rosbag_dir` 이 없을 때.
    """
    root = Path(rosbag_dir)
    if not root.is_dir():
        raise FileNotFoundError(f'rosbag_dir not found: {rosbag_dir}')
    allowed = set(dates) if dates else None
    out: list[SessionInfo] = []
    for date_dir in sorted(root.iterdir()):
        if not date_dir.is_dir() or not DATE_DIR_RE.match(date_dir.name):
            continue
        if allowed is not None and date_dir.name not in allowed:
            continue
        for session_dir in sorted(date_dir.iterdir()):
            if not session_dir.is_dir() or not SESSION_DIR_RE.match(session_dir.name):
                continue
            out.append(_inspect_session(session_dir, date_dir.name))
    out.sort(key=lambda s: (s.start_ns if s.start_ns is not None else 0, s.name))
    return out


def _inspect_session(session_dir: Path, date_dir: str) -> SessionInfo:
    mcaps = tuple(sorted(session_dir.glob('*.mcap')))
    meta_path = session_dir / 'metadata.yaml'
    if meta_path.is_file():
        try:
            splits = tuple(_parse_session_metadata(session_dir, date_dir, meta_path))
            topics = _topics_from_metadata(meta_path)
        except (_MetadataError, yaml.YAMLError, OSError) as exc:
            summary = _summarize_mcaps(mcaps)
            return SessionInfo(
                path=session_dir, name=session_dir.name, date_dir=date_dir, finalized=False,
                splits=(), mcap_paths=mcaps, message_count=summary.message_count,
                start_ns=summary.start_ns, end_ns=summary.end_ns, topics=summary.topics,
                note=f'metadata.yaml 을 읽을 수 없다 ({exc}). {summary.note}'.strip())
        starts = [s.start_ns for s in splits]
        ends = [s.end_ns for s in splits]
        return SessionInfo(
            path=session_dir, name=session_dir.name, date_dir=date_dir, finalized=True,
            splits=splits, mcap_paths=mcaps,
            message_count=sum(s.message_count for s in splits),
            start_ns=min(starts) if starts else None, end_ns=max(ends) if ends else None,
            topics=topics)

    summary = _summarize_mcaps(mcaps)
    return SessionInfo(
        path=session_dir, name=session_dir.name, date_dir=date_dir, finalized=False,
        splits=(), mcap_paths=mcaps, message_count=summary.message_count,
        start_ns=summary.start_ns, end_ns=summary.end_ns, topics=summary.topics,
        note=f'{NOTE_UNFINALIZED} {summary.note}'.strip())


def _topics_from_metadata(meta_path: Path) -> tuple:
    """`metadata.yaml` 의 `topics_with_message_count` 를 `TopicCount` 로 바꾼다."""
    with meta_path.open('r', encoding='utf-8') as f:
        raw = yaml.safe_load(f) or {}
    info = raw.get('rosbag2_bagfile_information') or {}
    out: list[TopicCount] = []
    for entry in info.get('topics_with_message_count') or []:
        if not isinstance(entry, dict):
            continue
        meta = entry.get('topic_metadata') or {}
        name = meta.get('name')
        if not isinstance(name, str) or not name:
            continue
        count = entry.get('message_count')
        out.append(TopicCount(name=name, type_name=str(meta.get('type') or ''),
                              message_count=int(count) if isinstance(count, int) else 0))
    out.sort(key=lambda t: t.name)
    return tuple(out)


@dataclass(frozen=True)
class _McapSummary:
    topics: tuple
    start_ns: Optional[int]
    end_ns: Optional[int]
    message_count: int
    note: str = ''


def _summarize_mcaps(paths: Sequence[Path]) -> _McapSummary:
    """MCAP 파일들의 summary 섹션만 읽어 토픽 · 수 · 시각을 합친다 (메시지는 안 푼다).

    summary 가 없는 파일(쓰다 끊긴 파일)은 세지 못하므로 `note` 에 적는다 — 0 이 '비었다'로
    읽히면 안 된다.
    """
    merged: dict[str, tuple[str, int]] = {}
    starts: list[int] = []
    ends: list[int] = []
    total = 0
    unreadable: list[str] = []
    for path in paths:
        try:
            with path.open('rb') as f:
                summary = make_reader(f).get_summary()
        except Exception as exc:  # noqa: BLE001 — 잘린 파일은 무엇으로든 실패할 수 있다
            _logger.warning('cannot read mcap summary %s: %s', path, exc)
            unreadable.append(path.name)
            continue
        if summary is None:
            unreadable.append(path.name)
            continue
        stats = summary.statistics
        counts = stats.channel_message_counts if stats else {}
        for ch in summary.channels.values():
            schema = summary.schemas.get(ch.schema_id)
            type_name = schema.name if schema is not None else ''
            prev_type, prev_count = merged.get(ch.topic, ('', 0))
            merged[ch.topic] = (type_name or prev_type, prev_count + counts.get(ch.id, 0))
        if stats and stats.message_count:
            starts.append(stats.message_start_time)
            ends.append(stats.message_end_time)
            total += stats.message_count
    topics = tuple(TopicCount(name=n, type_name=t, message_count=c)
                   for n, (t, c) in sorted(merged.items()))
    note = ''
    if not paths:
        note = '.mcap 파일이 없다.'
    elif unreadable:
        note = f'MCAP summary 를 읽을 수 없는 파일: {", ".join(unreadable)} (잘린 파일?).'
    return _McapSummary(topics=topics, start_ns=min(starts) if starts else None,
                        end_ns=max(ends) if ends else None, message_count=total, note=note)


# ----- 토픽 요약 -------------------------------------------------------------

@dataclass(frozen=True)
class TopicSummary:
    """여러 세션을 합친 토픽 한 줄."""

    name: str
    types: tuple
    message_count: int
    session_count: int

    @property
    def type_name(self) -> str:
        """타입 하나면 그 이름, 섞였으면 쉼표로 이어 보인다 (섞임 자체가 이상 신호다)."""
        return ','.join(self.types)


def summarize_topics(sessions: Iterable[SessionInfo], *,
                     include_unfinalized: bool = False) -> list[TopicSummary]:
    """세션들의 토픽을 합친다. 기본은 적재가 보는 것과 같은 **완료 세션만**이다."""
    counts: dict[str, int] = {}
    types: dict[str, set] = {}
    sessions_of: dict[str, set] = {}
    for s in sessions:
        if not s.finalized and not include_unfinalized:
            continue
        for t in s.topics:
            counts[t.name] = counts.get(t.name, 0) + t.message_count
            types.setdefault(t.name, set())
            if t.type_name:
                types[t.name].add(t.type_name)
            sessions_of.setdefault(t.name, set()).add(s.name)
    return [TopicSummary(name=n, types=tuple(sorted(types[n])), message_count=counts[n],
                         session_count=len(sessions_of[n]))
            for n in sorted(counts)]


def finalized_splits(sessions: Iterable[SessionInfo]) -> list:
    """적재가 보는 split 목록 — `discover_splits` 와 같은 결과를 세션 트리에서 꺼낸다."""
    splits = [sp for s in sessions if s.finalized for sp in s.splits]
    splits.sort(key=lambda s: (s.start_ns, s.path.name))
    return splits


def mcap_paths_for_topic(sessions: Iterable[SessionInfo], topic: str, *,
                         include_unfinalized: bool = False) -> list[Path]:
    """그 토픽이 들어 있는 세션들의 `.mcap` 경로 (시간순)."""
    out: list[Path] = []
    for s in sessions:
        if not s.finalized and not include_unfinalized:
            continue
        if topic in s.topic_types():
            out.extend(s.mcap_paths)
    return out


# ----- 메시지 정의 -----------------------------------------------------------

@dataclass(frozen=True)
class SchemaText:
    """MCAP 에 박힌 메시지 정의. ROS 가 없어도 읽힌다."""

    type_name: str
    encoding: str
    text: str


def read_schema_text(mcap_path: str | Path, topic: str) -> Optional[SchemaText]:
    """한 split 에서 `topic` 의 스키마 텍스트(`.msg` 정의)를 꺼낸다. 토픽이 없으면 ``None``."""
    with Path(mcap_path).open('rb') as f:
        summary = make_reader(f).get_summary()
        if summary is None:
            return None
        for ch in summary.channels.values():
            if ch.topic != topic:
                continue
            schema = summary.schemas.get(ch.schema_id)
            if schema is None:
                return None
            return SchemaText(type_name=schema.name, encoding=schema.encoding,
                              text=schema.data.decode('utf-8', errors='replace'))
    return None


# ----- stamp 진단 ------------------------------------------------------------

@dataclass
class StampDiagnosis:
    """토픽 하나의 stamp 진단. **적재가 거르는 기준과 같은 기준으로 센다.**

    Attributes:
        total: bag 의 메시지 수 (log 기준 — 전부).
        without_stamp: `header.stamp` 가 없어 적재가 버리는 수.
        zero_stamp: `header.stamp == 0` (발행자가 안 채웠다) — 창 밖으로 떨어진다.
        in_episode: 어느 에피소드 창 `[start, stop)` 안에 드는 수. 창을 안 줬으면 0 이고
            `episodes_checked` 가 ``None`` 이다.
    """

    topic: str
    type_name: str = ''
    total: int = 0
    without_stamp: int = 0
    zero_stamp: int = 0
    in_episode: int = 0
    first_stamp_ns: Optional[int] = None
    last_stamp_ns: Optional[int] = None
    first_log_ns: Optional[int] = None
    last_log_ns: Optional[int] = None
    episodes_checked: Optional[int] = None

    @property
    def stamped(self) -> int:
        """적재가 정렬할 수 있는 메시지 수."""
        return self.total - self.without_stamp

    @property
    def mean_rate_hz(self) -> Optional[float]:
        """stamp 기준 평균 주기. 두 건 미만이면 ``None``."""
        if (self.first_stamp_ns is None or self.last_stamp_ns is None
                or self.stamped < 2 or self.last_stamp_ns <= self.first_stamp_ns):
            return None
        return (self.stamped - 1) / ((self.last_stamp_ns - self.first_stamp_ns) / 1e9)

    def ratio(self, n: int) -> float:
        return n / self.total if self.total else 0.0

    @property
    def suspect(self) -> bool:
        """적재에서 조용히 빠질 징후가 있는가.

        stamp 가 없거나 0 인 메시지가 하나라도 있으면 그렇다. 창을 봤는데 **한 건도 안 들면**
        벽시계 · sim 시계 불일치(펑션베이 회차 함정, Isaac 재기동 함정)의 전형이다.
        """
        if self.without_stamp or self.zero_stamp:
            return True
        return bool(self.episodes_checked) and self.stamped > 0 and self.in_episode == 0

    def summary_lines(self) -> list[str]:
        """사람이 읽는 줄 — 상세 패널의 '통계' 블록."""
        lines = [f'메시지 수 {self.total:,}']
        rate = self.mean_rate_hz
        if rate is not None:
            lines.append(f'평균 주기 {rate:.2f} Hz (stamp 기준)')
        lines.append(f'header.stamp 없음: {self.without_stamp:,} '
                     f'({self.ratio(self.without_stamp):.1%})')
        lines.append(f'header.stamp == 0: {self.zero_stamp:,} '
                     f'({self.ratio(self.zero_stamp):.1%})')
        if self.episodes_checked is not None:
            lines.append(f'에피소드 창 안: {self.in_episode:,} ({self.ratio(self.in_episode):.1%}) '
                         f'— 창 {self.episodes_checked}개 기준')
        if self.first_stamp_ns is not None and self.last_stamp_ns is not None:
            lines.append(f'stamp 범위: {self.first_stamp_ns} ~ {self.last_stamp_ns} ns')
        return lines


def episode_windows(episodes: Iterable[Any]) -> list[tuple[int, int]]:
    """에피소드 목록을 `[(start_ns, stop_ns), …]` 로 (정렬). `BagEpisode` · `Episode` 모두 받는다."""
    out: list[tuple[int, int]] = []
    for e in episodes:
        ep = getattr(e, 'episode', e)
        out.append((int(ep.start_ns), int(ep.stop_ns)))
    out.sort()
    return out


class _WindowIndex:
    """정렬된 창 목록에 대한 포함 판정. 창이 겹칠 수 있어(sim 시계 되감김) 앞으로도 훑는다."""

    def __init__(self, windows: Sequence[tuple[int, int]]) -> None:
        self._windows = sorted(windows)
        self._starts = [w[0] for w in self._windows]

    def __len__(self) -> int:
        return len(self._windows)

    def contains(self, stamp_ns: int) -> bool:
        i = bisect.bisect_right(self._starts, stamp_ns) - 1
        while i >= 0:
            start, stop = self._windows[i]
            if start <= stamp_ns < stop:
                return True
            i -= 1
        return False


def diagnose_stamps(mcap_paths: Iterable[str | Path], topic: str, *,
                    windows: Optional[Sequence[tuple[int, int]]] = None,
                    limit: Optional[int] = None) -> StampDiagnosis:
    """토픽 하나의 메시지를 전부 풀어 stamp 를 센다.

    Args:
        mcap_paths: 읽을 split 들.
        windows: 에피소드 창 `[start, stop)` 목록. ``None`` 이면 창 판정을 하지 않는다.
        limit: 큰 토픽을 미리 보기 위한 상한. ``None`` 이면 전부 센다.
    """
    diag = StampDiagnosis(topic=topic)
    index = _WindowIndex(windows) if windows is not None else None
    if index is not None:
        diag.episodes_checked = len(index)
    seen = 0
    for path in mcap_paths:
        for rec in read_ros2_messages(str(path), topics=[topic]):
            seen += 1
            diag.total += 1
            if not diag.type_name:
                diag.type_name = rec.schema.name
            log_ns = rec.log_time_ns
            diag.first_log_ns = (log_ns if diag.first_log_ns is None
                                 else min(diag.first_log_ns, log_ns))
            diag.last_log_ns = (log_ns if diag.last_log_ns is None
                                else max(diag.last_log_ns, log_ns))
            # 적재가 쓰는 판정 그대로다 — 다른 함수로 세면 기준이 둘이 된다.
            stamp_ns = _extract_header_stamp_ns(rec)
            if stamp_ns is None:
                diag.without_stamp += 1
            else:
                if stamp_ns == 0:
                    diag.zero_stamp += 1
                diag.first_stamp_ns = (stamp_ns if diag.first_stamp_ns is None
                                       else min(diag.first_stamp_ns, stamp_ns))
                diag.last_stamp_ns = (stamp_ns if diag.last_stamp_ns is None
                                      else max(diag.last_stamp_ns, stamp_ns))
                if index is not None and index.contains(stamp_ns):
                    diag.in_episode += 1
            if limit is not None and seen >= limit:
                return diag
    return diag


# ----- 메시지 열람 -----------------------------------------------------------

@dataclass(frozen=True)
class MessageRow:
    """데이터 브라우저의 한 행 — 메시지를 평탄화한 것."""

    stamp_ns: int
    log_time_ns: int
    fields: dict


def _is_message(value: Any) -> bool:
    """`mcap_ros2` 가 동적으로 만든 메시지 인스턴스인가 (`__slots__` 가 필드 목록이다)."""
    if isinstance(value, (bool, int, float, str, bytes, bytearray, list, tuple, dict)):
        return False
    return bool(getattr(type(value), '__slots__', None))


def flatten_message(msg: Any, *, max_items: int = 12) -> dict:
    """중첩 메시지를 `a.b.c` 키의 평면 dict 로 만든다.

    기본형 배열은 그대로(길면 잘라 `… (+N)` 표시), 메시지 배열은 원소마다 평탄화한 dict 목록,
    `bytes` 는 길이만 적는다 — 이미지 데이터를 표에 그대로 넣으면 화면이 멎는다.
    """
    out: dict = {}
    _flatten_into(out, '', msg, max_items)
    return out


def _flatten_into(out: dict, key: str, value: Any, max_items: int) -> None:
    if _is_message(value):
        for name in type(value).__slots__:
            _flatten_into(out, f'{key}.{name}' if key else name, getattr(value, name), max_items)
    elif isinstance(value, (bytes, bytearray)):
        out[key] = f'<{len(value)} bytes>'
    elif isinstance(value, (list, tuple)):
        out[key] = _render_sequence(value, max_items)
    else:
        out[key] = value


def _render_sequence(seq: Sequence[Any], max_items: int) -> list:
    head = list(seq[:max_items])
    if head and _is_message(head[0]):
        items: list = [flatten_message(x, max_items=max_items) for x in head]
    else:
        items = head
    if len(seq) > max_items:
        items.append(f'… (+{len(seq) - max_items})')
    return items


def read_topic_rows(mcap_paths: Iterable[str | Path], topic: str, *,
                    time_range_ns: Optional[tuple[int, int]] = None, offset: int = 0,
                    limit: int = 1000, max_items: int = 12) -> tuple[list[MessageRow], bool]:
    """토픽 메시지를 `header.stamp` 순으로 한 페이지 읽는다.

    적재가 읽는 경로(`merge_split_streams`)를 그대로 타므로 **stamp 가 없는 메시지는 여기에도
    안 나온다** — 그 수는 :func:`diagnose_stamps` 가 센다.

    Returns:
        `(rows, has_more)`. `has_more` 가 참이면 다음 페이지가 있다.
    """
    stream = merge_split_streams([str(p) for p in mcap_paths], topics=[topic],
                                 time_range_ns=time_range_ns)
    rows: list[MessageRow] = []
    skipped = 0
    for m in stream:
        if skipped < offset:
            skipped += 1
            continue
        if len(rows) >= limit:
            return rows, True
        rows.append(MessageRow(stamp_ns=m.stamp_ns, log_time_ns=m.log_time_ns,
                               fields=flatten_message(m.msg, max_items=max_items)))
    return rows, False


def row_columns(rows: Iterable[MessageRow]) -> list[str]:
    """행들의 키를 처음 나온 순서로 합친다 (표의 열 머리)."""
    seen: dict[str, None] = {}
    for r in rows:
        for k in r.fields:
            seen.setdefault(k, None)
    return list(seen)


__all__ = [
    'NOTE_UNFINALIZED', 'MessageRow', 'SchemaText', 'SessionInfo', 'StampDiagnosis',
    'TopicCount', 'TopicSummary', 'diagnose_stamps', 'episode_windows', 'finalized_splits',
    'flatten_message', 'mcap_paths_for_topic', 'read_schema_text', 'read_topic_rows',
    'row_columns', 'scan_sessions', 'summarize_topics',
]
