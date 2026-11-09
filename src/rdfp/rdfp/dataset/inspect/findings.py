"""점검 결과 한 줄 — 사전 점검 · 에피소드 점검이 같은 모양으로 돌려준다.

세 단계만 둔다. `error` 는 실행을 막고, `warning` 은 보여 주되 막지 않고, `info` 는 사실
확인이다. GUI 는 색으로, CLI 는 접두어로 보인다.
"""

from __future__ import annotations

from typing import Iterable

from dataclasses import dataclass

LEVEL_ERROR = 'error'
LEVEL_WARNING = 'warning'
LEVEL_INFO = 'info'

LEVELS = (LEVEL_ERROR, LEVEL_WARNING, LEVEL_INFO)


@dataclass(frozen=True)
class Finding:
    """점검 한 건.

    Attributes:
        level: `error` · `warning` · `info`.
        code: 기계가 읽는 짧은 식별자 (예: ``image_conflict``). 테스트와 GUI 가 이 값으로
            분기하므로 문구를 바꿔도 코드는 유지한다.
        message: 사람이 읽는 설명. 원인과 할 일을 함께 적는다.
    """

    level: str
    code: str
    message: str

    def __str__(self) -> str:
        return f'[{self.level}] {self.code}: {self.message}'


def has_errors(findings: Iterable[Finding]) -> bool:
    """실행을 막는 항목이 하나라도 있는가."""
    return any(f.level == LEVEL_ERROR for f in findings)


def only(findings: Iterable[Finding], level: str) -> list[Finding]:
    """한 단계만 골라낸다."""
    return [f for f in findings if f.level == level]


__all__ = ['Finding', 'LEVELS', 'LEVEL_ERROR', 'LEVEL_INFO', 'LEVEL_WARNING', 'has_errors',
           'only']
