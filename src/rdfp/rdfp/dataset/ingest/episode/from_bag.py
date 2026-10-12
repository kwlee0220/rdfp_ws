"""rosbag split 목록 → 에피소드 목록.

`/session` 메시지만 뽑아 :func:`detect_episodes` 에 넘기는 15줄짜리 루프인데, **두 곳에서
필요하다** — 적재 파이프라인과 `rosbag` CLI 다. 복사하면 안 되는 이유가 있다: `outcome` /
`metadata` 를 `getattr` 폴백으로 읽는 것은 그 필드가 나중에 추가되어 **옛 bag 에는 없기**
때문이고, 두 벌이 되면 한쪽만 뒤처진다.

`detect_episodes` 가 돌려주는 :class:`Episode` 는 `header.stamp` 만 갖는다. 그런데 사람이
보는 목록은 **rosbag 에 기록된 시각(log time)** 순이어야 한다 — `header.stamp` 는 발행자
시계라 경과시간이 섞이는 회차가 있지만 log time 은 언제나 녹화기의 벽시계다. 그래서
:class:`BagEpisode` 가 그 값과 세션 출처를 함께 들고 다닌다.

이 모듈은 ROS 도 DB 도 요구하지 않는다 (`mcap_ros2` 만 쓴다).
"""

from __future__ import annotations

from typing import Iterable, NamedTuple

import logging

from rdfp.dataset.db.registry import SESSION_COMMAND_TYPE
from rdfp.rosbag.mcap_reader import iter_split_messages

from .detector import Episode, SessionEvent, detect_episodes

_logger = logging.getLogger(__name__)

SESSION_TOPIC = '/session'


class BagEpisode(NamedTuple):
    """bag 에서 감지한 에피소드 + 출처 정보.

    `episode.start_ns` 가 DB 의 자연키다 (`rdfp.dataset.episode_key`).
    """

    episode: Episode
    log_start_ns: int          # 시작 `/session` 메시지가 rosbag 에 기록된 시각
    log_stop_ns: int           # 종료 `/session` 메시지가 기록된 시각
    session_name: str          # 예: session_2026-09-11_21-13-40
    session_path: str
    split_paths: tuple[str, ...]   # 이 구간이 걸쳐 있는 split (log time 기준)

    @property
    def start_ns(self) -> int:
        """DB 자연키 (`header.stamp` 기준)."""
        return self.episode.start_ns

    @property
    def task_label(self) -> str:
        return self.episode.task_label or ''

    @property
    def duration_ns(self) -> int:
        return self.episode.stop_ns - self.episode.start_ns


class _Event(NamedTuple):
    """`SessionEvent` + 그 메시지가 어디서 왔는지."""

    event: SessionEvent
    log_ns: int
    session_name: str
    session_path: str
    split_path: str


def _collect_session_events(splits: Iterable) -> list[_Event]:
    """split 마다 `/session` 메시지를 뽑아 출처와 함께 모으고 stamp 순으로 정렬한다.

    split 단위로 읽는 이유는 **출처를 알아야** 하기 때문이다 —
    `merge_split_streams` 는 어느 split 에서 온 메시지인지 알려주지 않는다.
    `/session` 만 읽으므로 비용은 split 수에 선형이다.
    """
    out: list[_Event] = []
    for s in splits:
        for m in iter_split_messages(s.path, topics=[SESSION_TOPIC]):
            if m.type_name != SESSION_COMMAND_TYPE:
                continue
            # `outcome` / `metadata` 는 나중에 추가된 필드다. 옛 bag 에는 없으므로
            # getattr 기본값으로 받아 과거 데이터도 그대로 읽힌다.
            event = SessionEvent(
                stamp_ns=m.stamp_ns,
                state=str(m.msg.state),
                task_label=str(getattr(m.msg, 'task_label', '') or ''),
                outcome=str(getattr(m.msg, 'outcome', '') or ''),
                metadata=str(getattr(m.msg, 'metadata', '') or ''),
            )
            out.append(_Event(event, m.log_time_ns, s.session_name,
                              str(s.session_path), str(s.path)))
    out.sort(key=lambda e: (e.event.stamp_ns, e.log_ns))
    return out


def episodes_from_splits(splits: Iterable) -> list[BagEpisode]:
    """split 목록에서 에피소드를 감지해 **log time 오름차순**으로 돌려준다.

    Args:
        splits: :class:`rdfp.rosbag.catalog.Split` 목록.

    Returns:
        :class:`BagEpisode` 목록. `/session` 메시지가 없으면 빈 목록이다 —
        예외를 던지지 않는다. 호출자가 "에피소드가 없다" 와 "적재할 수 없다" 를
        따로 판단해야 하기 때문이다.
    """
    split_list = list(splits)
    events = _collect_session_events(split_list)
    if not events:
        return []

    episodes = detect_episodes([e.event for e in events])
    # stamp → 출처. 같은 stamp 가 여럿이면 앞선 것을 쓴다 (경과시간 회차의 충돌).
    by_stamp: dict[int, _Event] = {}
    for e in events:
        by_stamp.setdefault(e.event.stamp_ns, e)

    out: list[BagEpisode] = []
    for ep in episodes:
        start_ev = by_stamp.get(ep.start_ns)
        stop_ev = by_stamp.get(ep.stop_ns)
        log_start = start_ev.log_ns if start_ev else ep.start_ns
        log_stop = stop_ev.log_ns if stop_ev else ep.stop_ns
        sname = start_ev.session_name if start_ev else ''
        spath = start_ev.session_path if start_ev else ''
        paths = tuple(
            str(s.path) for s in split_list
            if s.start_ns <= log_stop and log_start < s.end_ns
        )
        out.append(BagEpisode(episode=ep, log_start_ns=log_start, log_stop_ns=log_stop,
                              session_name=sname, session_path=spath, split_paths=paths))

    out.sort(key=lambda be: (be.log_start_ns, be.start_ns))
    return out


__all__ = ['BagEpisode', 'SESSION_TOPIC', 'episodes_from_splits']
