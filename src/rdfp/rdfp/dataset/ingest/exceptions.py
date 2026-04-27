"""후처리기 공용 예외."""

from __future__ import annotations


class PostProcError(RuntimeError):
    """후처리기 실행 중 복구 불가능한 오류."""


__all__ = ['PostProcError']
