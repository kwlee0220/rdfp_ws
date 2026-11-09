"""백그라운드 읽기 — MCAP · DB 는 느리므로 `QThreadPool` 에서 읽고 결과는 **GUI 스레드에서** 받는다.

콜백이 GUI 스레드에서 불리는 것이 핵심이다: 워커가 `Background` (GUI 스레드에 사는 QObject) 의
시그널을 emit 하면 Qt 가 큐로 넘겨 슬롯이 GUI 스레드에서 돈다. 콜백을 워커에서 직접 부르면 위젯을
다른 스레드에서 만지게 되어 간헐적으로 죽는다.
"""

from __future__ import annotations

from typing import Any, Callable, Optional

import itertools
import traceback

from PySide6.QtCore import QObject, QRunnable, QThreadPool, Signal, Slot


class _Task(QRunnable):
    def __init__(self, owner: 'Background', token: int, fn: Callable, args: tuple) -> None:
        super().__init__()
        self._owner = owner
        self._token = token
        self._fn = fn
        self._args = args
        self.setAutoDelete(True)

    def run(self) -> None:
        try:
            result = self._fn(*self._args)
        except Exception:  # noqa: BLE001 — 어떤 예외든 화면에 보여야 한다
            self._owner._failed.emit(self._token, traceback.format_exc())
            return
        self._owner._done.emit(self._token, result)


class Background(QObject):
    """`run(fn, on_done, on_error, *args)` — fn 은 워커에서, 콜백은 GUI 스레드에서."""

    _done = Signal(int, object)
    _failed = Signal(int, str)

    def __init__(self, parent: Optional[QObject] = None, *, max_threads: int = 4) -> None:
        super().__init__(parent)
        self._pool = QThreadPool(self)
        self._pool.setMaxThreadCount(max_threads)
        self._pending: dict = {}
        self._tokens = itertools.count(1)
        self._done.connect(self._dispatch_done)
        self._failed.connect(self._dispatch_failed)

    def run(self, fn: Callable, on_done: Callable[[Any], None],
            on_error: Optional[Callable[[str], None]] = None, *args: Any) -> int:
        token = next(self._tokens)
        self._pending[token] = (on_done, on_error)
        self._pool.start(_Task(self, token, fn, args))
        return token

    def cancel(self, token: int) -> None:
        """결과를 버린다 — 이미 돌고 있는 작업은 멈추지 못하지만 콜백은 안 불린다."""
        self._pending.pop(token, None)

    @property
    def active(self) -> int:
        return len(self._pending)

    def wait(self, timeout_ms: int = 30000) -> bool:
        return self._pool.waitForDone(timeout_ms)

    @Slot(int, object)
    def _dispatch_done(self, token: int, result: Any) -> None:
        callbacks = self._pending.pop(token, None)
        if callbacks is not None:
            callbacks[0](result)

    @Slot(int, str)
    def _dispatch_failed(self, token: int, text: str) -> None:
        callbacks = self._pending.pop(token, None)
        if callbacks is None:
            return
        if callbacks[1] is not None:
            callbacks[1](text)


__all__ = ['Background']
