"""실행 요약 JSONL 로그.

각 에피소드 처리가 끝날 때마다 한 줄을 append 한다. 실패한 에피소드는
`event='episode_failed'` 로 에러 메시지와 함께 남긴다.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path


class RunLog:
    """에피소드 단위 JSONL 로거."""

    def __init__(self, path: Path) -> None:
        self._path = Path(path)
        self._path.parent.mkdir(parents=True, exist_ok=True)

    def episode_done(
        self,
        *,
        episode_id: int,
        start_ns: int,
        stop_ns: int,
        task_label: str | None,
        row_counts: dict[str, int],
        mp4_files: list[str],
    ) -> None:
        self._write({
            'ts': _now_iso(),
            'event': 'episode_done',
            'episode_id': int(episode_id),
            'start_ts': _ns_to_iso(start_ns),
            'stop_ts': _ns_to_iso(stop_ns),
            'task_label': task_label,
            'row_counts': dict(row_counts),
            'mp4_files': list(mp4_files),
        })

    def episode_failed(
        self,
        *,
        start_ns: int,
        stop_ns: int,
        task_label: str | None,
        error: str,
    ) -> None:
        self._write({
            'ts': _now_iso(),
            'event': 'episode_failed',
            'start_ts': _ns_to_iso(start_ns),
            'stop_ts': _ns_to_iso(stop_ns),
            'task_label': task_label,
            'error': error,
        })

    def episode_skipped(
        self,
        *,
        episode_id: int,
        start_ns: int,
        stop_ns: int,
        task_label: str | None,
    ) -> None:
        self._write({
            'ts': _now_iso(),
            'event': 'episode_skipped',
            'episode_id': int(episode_id),
            'start_ts': _ns_to_iso(start_ns),
            'stop_ts': _ns_to_iso(stop_ns),
            'task_label': task_label,
        })

    def quality_warning(
        self,
        *,
        episode_id: int | None,
        kind: str,
        topic: str,
        stamp_ns: int,
        prev_stamp_ns: int,
    ) -> None:
        self._write({
            'ts': _now_iso(),
            'event': 'quality_warning',
            'episode_id': episode_id,
            'kind': kind,
            'topic': topic,
            'stamp_ns': int(stamp_ns),
            'prev_stamp_ns': int(prev_stamp_ns),
        })

    def _write(self, record: dict) -> None:
        with self._path.open('a', encoding='utf-8') as f:
            f.write(json.dumps(record, ensure_ascii=False) + '\n')


def _now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat()


def _ns_to_iso(ns: int) -> str:
    sec = ns // 1_000_000_000
    nsec = ns % 1_000_000_000
    base = datetime.fromtimestamp(sec, tz=timezone.utc).astimezone().isoformat()
    # ISO-8601 뒤에 나노초 보조 표기를 붙인다 (표준 datetime 은 마이크로초까지).
    return f'{base}[ns={nsec}]'


__all__ = ['RunLog']
