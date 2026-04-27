"""rosbag 루트 스캔 및 split 후보 선정.

`<rosbag_dir>/YYYY-MM-DD/session_YYYY-MM-DD_HH-MM-SS/metadata.yaml` 구조
(운영 방안 4장) 를 전제로 각 세션의 `metadata.yaml` 을 파싱하고, 해당 세션의
모든 split (.mcap) 을 반환한다. `metadata.yaml` 이 없는 세션은 아직 녹화가
끝나지 않았거나 비정상 종료된 것으로 간주하여 대상에서 제외한다.
"""

from __future__ import annotations

from typing import Iterator

import re
from dataclasses import dataclass
from pathlib import Path

import yaml


# 세션 디렉터리 이름 패턴 (운영 방안 4장 규칙).
SESSION_DIR_RE = re.compile(r'^session_\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}Z?$')
# 날짜 폴더 이름 패턴.
DATE_DIR_RE = re.compile(r'^\d{4}-\d{2}-\d{2}$')


@dataclass(frozen=True)
class Split:
    """하나의 MCAP split 파일 정보."""

    path: Path                 # .mcap 절대 경로
    session_path: Path         # 세션 디렉터리 절대 경로
    session_name: str          # 예: session_2026-04-11_09-00-03
    date_dir: str              # YYYY-MM-DD (세션 디렉터리 상위)
    split_index: int           # 0, 1, 2, ...
    start_ns: int              # split 시작 시각 (epoch ns)
    end_ns: int                # split 종료 시각 (epoch ns, exclusive)
    message_count: int         # 토픽 전체 메시지 수
    topics: frozenset[str]     # split 에 포함된 토픽 이름 집합


def discover_splits(
    rosbag_dir: str | Path,
    *,
    dates: list[str] | None = None,
    topics: list[str] | None = None,
) -> list[Split]:
    """`<rosbag_dir>` 하위를 스캔하여 split 목록을 반환한다.

    `metadata.yaml` 이 없는 세션 디렉터리는 아직 녹화가 끝나지 않았거나
    비정상 종료된 것으로 간주하여 제외한다. 즉 "활성 split" 은 자연스럽게
    필터된다.

    Args:
        rosbag_dir: rosbag 루트 디렉터리.
        dates: 선행 필터할 날짜(YYYY-MM-DD) 목록. None 이면 모든 날짜.
        topics: 선행 필터할 토픽 목록. None 이면 모든 토픽. 지정 시 해당
            토픽이 하나라도 포함된 split 만 반환한다.

    Returns:
        stamp 오름차순으로 정렬된 split 리스트.

    Raises:
        FileNotFoundError: `rosbag_dir` 이 존재하지 않는 경우.
    """
    root = Path(rosbag_dir)
    if not root.is_dir():
        raise FileNotFoundError(f"rosbag_dir not found: {rosbag_dir}")

    allowed_dates = set(dates) if dates else None
    allowed_topics = set(topics) if topics else None

    splits: list[Split] = []
    for date_dir in sorted(root.iterdir()):
        if not date_dir.is_dir() or not DATE_DIR_RE.match(date_dir.name):
            continue
        if allowed_dates is not None and date_dir.name not in allowed_dates:
            continue

        for session_dir in sorted(date_dir.iterdir()):
            if not session_dir.is_dir() or not SESSION_DIR_RE.match(session_dir.name):
                continue
            meta_path = session_dir / 'metadata.yaml'
            if not meta_path.is_file():
                # metadata.yaml 이 없으면 이 세션을 건너뛴다 (녹화 중이거나 비정상
                # 종료된 세션).
                continue
            try:
                session_splits = _parse_session_metadata(
                    session_dir, date_dir.name, meta_path,
                )
            except _MetadataError:
                # metadata 포맷이 기대와 다르면 스킵한다 (상위에서 warning 로깅).
                continue

            for sp in session_splits:
                if allowed_topics is not None and sp.topics.isdisjoint(allowed_topics):
                    continue
                splits.append(sp)

    splits.sort(key=lambda s: (s.start_ns, s.path.name))
    return splits


def _parse_session_metadata(session_dir: Path, date_dir: str, meta_path: Path) -> Iterator[Split]:
    """rosbag2 가 생성한 `metadata.yaml` 을 파싱해 split 목록을 생성한다.

    rosbag2 의 metadata 구조는 다음과 같다:
        rosbag2_bagfile_information:
          storage_identifier: mcap
          topics_with_message_count:
            - topic_metadata: { name: ..., type: ... }
              message_count: N
          files:
            - path: <bag>_0.mcap
              starting_time: { nanoseconds_since_epoch: ... }
              duration: { nanoseconds: ... }
              message_count: N
    """
    with meta_path.open('r', encoding='utf-8') as f:
        raw = yaml.safe_load(f) or {}
    info = raw.get('rosbag2_bagfile_information')
    if not isinstance(info, dict):
        raise _MetadataError('missing rosbag2_bagfile_information')

    topics_list = info.get('topics_with_message_count') or []
    topics: set[str] = set()
    for entry in topics_list:
        if not isinstance(entry, dict):
            continue
        meta = entry.get('topic_metadata') or {}
        name = meta.get('name')
        if isinstance(name, str) and name:
            topics.add(name)
    frozen_topics = frozenset(topics)

    files = info.get('files') or []
    if not isinstance(files, list):
        raise _MetadataError('invalid files section')

    for idx, entry in enumerate(files):
        if not isinstance(entry, dict):
            continue
        rel = entry.get('path')
        if not isinstance(rel, str) or not rel:
            continue
        split_path = (session_dir / rel).resolve()
        start = _extract_ns(entry.get('starting_time'), 'nanoseconds_since_epoch')
        duration = _extract_ns(entry.get('duration'), 'nanoseconds')
        if start is None or duration is None:
            continue
        mcount = entry.get('message_count')
        mcount = int(mcount) if isinstance(mcount, int) else 0
        yield Split(
            path=split_path,
            session_path=session_dir.resolve(),
            session_name=session_dir.name,
            date_dir=date_dir,
            split_index=idx,
            start_ns=int(start),
            end_ns=int(start) + int(duration),
            message_count=mcount,
            topics=frozen_topics,
        )


def _extract_ns(obj: object, key: str) -> int | None:
    """`{key: <int>}` 형태 dict 에서 정수를 꺼낸다."""
    if isinstance(obj, dict):
        v = obj.get(key)
        if isinstance(v, int):
            return v
    return None


class _MetadataError(Exception):
    """metadata.yaml 파싱 실패를 표시하는 내부 예외."""


__all__ = ['Split', 'discover_splits']
