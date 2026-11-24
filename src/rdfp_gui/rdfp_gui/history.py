"""작업 이력 — 실행한 CLI 명령 · 결과를 로컬 SQLite 에 남긴다. **Qt 를 모른다.**

Codex Manager 의 작업 이력(Journal) 아이디어를 가져왔다 (2026-11-23 통합). 하단 패널의 로그는 다음
작업이 시작되면 지워지지만, 이력은 "언제 무엇을 어떤 명령으로 돌렸고 어떻게 끝났나"를 남긴다.
요약 JSON(`import` 의 마지막 출력)도 함께 담는다.

위치는 `$XDG_STATE_HOME/rdfp-dataset-gui/jobs.sqlite3` (없으면 `~/.local/state/…`)다 — 데이터 DB 가
아니라 이 기계의 사용자 기록이다.
"""

from __future__ import annotations

from typing import Optional

import json
import os
import sqlite3
import time
from dataclasses import dataclass
from pathlib import Path


def default_path() -> Path:
    base = os.environ.get('XDG_STATE_HOME') or str(Path.home() / '.local' / 'state')
    return Path(base) / 'rdfp-dataset-gui' / 'jobs.sqlite3'


@dataclass(frozen=True)
class JobRecord:
    id: int
    title: str
    command: str
    started: float
    finished: Optional[float]
    exit_code: Optional[int]
    summary: Optional[dict]

    @property
    def state(self) -> str:
        if self.finished is None:
            return '실행 중 또는 중단됨'
        return '성공' if self.exit_code == 0 else f'실패 (exit {self.exit_code})'


class JobHistory:
    LIMIT = 200

    def __init__(self, path: Optional[Path] = None) -> None:
        self.path = Path(path) if path else default_path()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self._connect() as conn:
            conn.execute('CREATE TABLE IF NOT EXISTS jobs (id INTEGER PRIMARY KEY AUTOINCREMENT, '
                         'title TEXT NOT NULL, command TEXT NOT NULL, started REAL NOT NULL, '
                         'finished REAL, exit_code INTEGER, summary TEXT)')

    def _connect(self) -> sqlite3.Connection:
        return sqlite3.connect(str(self.path), timeout=5)

    def start(self, title: str, command: str) -> int:
        with self._connect() as conn:
            cur = conn.execute('INSERT INTO jobs (title, command, started) VALUES (?, ?, ?)',
                               (title, command, time.time()))
            return int(cur.lastrowid)

    def finish(self, job_id: int, exit_code: int, summary: Optional[dict] = None) -> None:
        with self._connect() as conn:
            conn.execute('UPDATE jobs SET finished = ?, exit_code = ?, summary = ? WHERE id = ?',
                         (time.time(), int(exit_code),
                          json.dumps(summary, ensure_ascii=False) if summary else None, job_id))

    def recent(self, limit: int = LIMIT) -> list[JobRecord]:
        with self._connect() as conn:
            rows = conn.execute('SELECT id, title, command, started, finished, exit_code, summary '
                                'FROM jobs ORDER BY id DESC LIMIT ?', (limit,)).fetchall()
        return [JobRecord(int(r[0]), r[1], r[2], r[3], r[4], r[5],
                          json.loads(r[6]) if r[6] else None) for r in rows]


__all__ = ['JobHistory', 'JobRecord', 'default_path']
