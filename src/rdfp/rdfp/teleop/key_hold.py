#!/usr/bin/env python3

"""키를 **누르고 있는 동안만** 명령이 나가게 하는 장치.

터미널은 "키를 뗐다"를 알려 주지 않는다. 자동반복(typematic)으로 같은 문자가 반복해
들어올 뿐이고, 떼면 그냥 **안 온다** — '뗐다'와 '잠깐 쉰다'가 구분되지 않는다. 그래서
문자가 끊기는 것을 뗀 것으로 읽고 TTL 이 만료되면 멈춘다. 저장소 다른 곳에서
"deadman TTL" 이라 부르는 것이 이것이다.

세 조각이 함께여야 동작한다. 하나라도 빠지면 **에러 없이** 팔이 흘러간다.

1. **매 틱 입력 버퍼를 완전히 비운다** (:class:`TerminalKeyReader`). 한 틱에 한 글자만
   소비하면 자동반복으로 쌓인 잔여 문자가 키를 뗀 뒤에도 TTL 을 계속 갱신한다.
2. **TTL 로 뗌을 추정한다** (:class:`HoldKeyTracker`).
3. **만료 시 0 을 한 번 보낸다** (:attr:`HoldState.just_released`). 침묵하면 `moveit_servo`
   가 `incoming_command_timeout`(0.1s)을 다 기다린 뒤에야 정지로 판정하고, 그동안
   Butterworth 평활 필터가 마지막 속도를 이어 내보낸다. **매 틱 보내도 안 된다** —
   명령이 안 끊겨 servo 가 자체 정지 경로로 못 들어간다.

남는 체감 지연은 **자동반복 초기 지연**(터미널 기본 ~0.5s)이라 홀드 초반이 한 번
끊긴다. `xset r rate 30 30` 으로 없앤다. 근본 해결은 press/release 를 직접 주는
evdev 이며 `clutch_pedal` 이 그 선례다 — 입력 장치 권한이 필요하고 SSH 세션에서는
못 쓴다.

이 모듈은 **ROS 를 모른다.** 무엇을 발행할지는 호출자가 정한다.
"""

from __future__ import annotations

from typing import Callable, Iterable, Optional, Sequence

import select
import sys
import termios
import tty
from dataclasses import dataclass, field


# ESC 로 시작하는 escape sequence(`\x1b[X`, 화살표 등)의 뒤따르는 바이트를 기다리는 시간.
# 너무 짧으면 OS 스케줄링·터미널 버퍼 타이밍 때문에 3 바이트가 갈라져 들어와 '[' 가
# 별개 키로, 'D' 가 unknown key 로 오인식된다.
_ESCAPE_SEQUENCE_TIMEOUT_SEC = 0.05


class TerminalRawMode:
    """터미널 cbreak 모드 컨텍스트 매니저.

    컨텍스트를 벗어날 때(예외로 벗어나는 경우 포함) 설정을 복원한다. 복원에 실패해도
    조용히 넘긴다 — 이미 터미널이 없어진 상황이라 여기서 예외를 올려 봐야 원인을
    가리기만 한다.
    """

    def __enter__(self) -> 'TerminalRawMode':
        self._stdin_fd = sys.stdin.fileno()
        self._old_term = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        try:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_term)
        except Exception:
            pass


class TerminalKeyReader:
    """stdin 에 쌓인 키를 **한 번에 모두** 읽는다.

    :class:`TerminalRawMode` 안에서 쓴다. 논블로킹이라 버퍼가 비면 빈 리스트다.

    호출 가능 객체라 :class:`HoldKeyTracker` 에 그대로 넘길 수 있고, 테스트나 다른
    입력원(evdev 등)은 같은 모양의 함수로 바꿔 끼우면 된다.
    """

    def read_one(self) -> Optional[str]:
        """키 하나. 버퍼가 비어 있으면 ``None``. escape sequence 는 합쳐 돌려준다."""
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return None
        ch = sys.stdin.read(1)
        if ch != '\x1b':
            return ch

        ready, _, _ = select.select([sys.stdin], [], [], _ESCAPE_SEQUENCE_TIMEOUT_SEC)
        if not ready:
            return ch
        ch2 = sys.stdin.read(1)
        if ch2 != '[':
            return ch + ch2
        ready, _, _ = select.select([sys.stdin], [], [], _ESCAPE_SEQUENCE_TIMEOUT_SEC)
        if not ready:
            return ch + ch2
        return ch + ch2 + sys.stdin.read(1)

    def __call__(self) -> list:
        """이번 틱에 쌓인 키 전부를 입력 순서대로.

        **반드시 다 비운다.** 남기면 그 잔여 문자가 다음 틱의 TTL 을 갱신해, 키를
        뗀 뒤에도 팔이 계속 움직인다.
        """
        keys = []
        while True:
            key = self.read_one()
            if key is None:
                return keys
            keys.append(key)


@dataclass(frozen=True)
class HoldState:
    """한 틱의 결과.

    Attributes:
        key: 지금 유효한 **모션 키**. 홀드 키(SPACE)만 눌린 상태이거나 홀드가
            끊긴 상태면 ``None`` 이다.
        active: 홀드가 살아 있는가. ``True`` 면 호출자는 명령을 발행한다.
        just_released: **이번 틱에** 만료됐는가. 여기서 0 을 한 번 발행한다.
            다음 틱부터는 ``False`` 이므로 자연히 한 번만 나간다.
        other_keys: 모션·홀드 키가 아닌 입력. 순서를 유지하며, 처리는 호출자 몫이다
            (one-shot 명령·정지 키·오타 안내 등).
    """

    key: Optional[str] = None
    active: bool = False
    just_released: bool = False
    other_keys: tuple = field(default_factory=tuple)


class HoldKeyTracker:
    """키를 누르고 있는 동안만 유효한 '현재 모션 키'를 관리한다.

    한 틱에 :meth:`tick` 을 한 번 부르고 결과대로 발행한다::

        state = tracker.tick(dt)
        for key in state.other_keys:
            ...                      # one-shot / 정지 / 알 수 없는 키
        if state.just_released:
            publish_zero()           # 정확히 한 번
        elif state.active:
            publish(state.key)       # key 가 None 이면 0 (홀드 키만 눌린 상태)

    Args:
        motion_keys: 눌린 동안 움직이는 키들.
        ttl_sec: 마지막 입력 뒤 이만큼 지나면 뗀 것으로 본다. 자동반복 간격보다
            **넉넉히 길어야** 반복 사이에 끊기지 않고, 사람이 뗀 것을 알아채기에는
            **짧아야** 한다.
        hold_keys: TTL 만 갱신하고 움직이지는 않는 키들 (SPACE 같은 데드맨 키).
        read_keys: 이번 틱의 입력을 돌려주는 호출 가능 객체. 기본은 터미널이다.
    """

    def __init__(self, motion_keys: Iterable[str], *, ttl_sec: float,
                 hold_keys: Sequence[str] = (),
                 read_keys: Optional[Callable[[], Sequence[str]]] = None) -> None:
        if ttl_sec <= 0:
            raise ValueError(f'ttl_sec must be positive, got {ttl_sec}')
        self._motion_keys = frozenset(motion_keys)
        self._hold_keys = frozenset(hold_keys)
        self._ttl_sec = float(ttl_sec)
        self._read_keys = read_keys if read_keys is not None else TerminalKeyReader()
        self._remaining = 0.0
        self._key: Optional[str] = None
        # 만료됐을 때 0 을 아직 안 보냈는가. 시작 상태는 '보낼 것 없음' 이다 —
        # 노드가 뜨자마자 아무도 안 눌렀는데 0 을 쏘지 않게.
        self._zero_pending = False

    # -- 조회 --------------------------------------------------------------

    @property
    def key(self) -> Optional[str]:
        """지금 유효한 모션 키 (없으면 ``None``)."""
        return self._key

    @property
    def active(self) -> bool:
        return self._remaining > 0.0

    @property
    def remaining_sec(self) -> float:
        return self._remaining

    # -- 갱신 --------------------------------------------------------------

    def tick(self, dt: float) -> HoldState:
        """``dt`` 초가 지났다고 보고 한 틱 진행한다."""
        self._remaining = max(0.0, self._remaining - max(0.0, dt))

        others = []
        latest: Optional[str] = None
        seen_hold = False
        for key in self._read_keys():
            if key in self._motion_keys:
                # **배치 안에서는 마지막 것만 유효하다.** 자동반복으로 여러 글자가
                # 쌓이므로 앞의 것을 반영하면 이미 지난 입력을 따라가게 된다.
                latest, seen_hold = key, True
            elif key in self._hold_keys:
                latest, seen_hold = None, True
            else:
                others.append(key)

        if seen_hold:
            self._key = latest
            self._remaining = self._ttl_sec
            self._zero_pending = True

        just_released = False
        if self._remaining <= 0.0:
            self._key = None
            if self._zero_pending:
                self._zero_pending = False
                just_released = True

        return HoldState(key=self._key, active=self._remaining > 0.0,
                         just_released=just_released, other_keys=tuple(others))

    def cancel(self) -> None:
        """홀드를 즉시 끊는다 — 정지 키처럼 **호출자가 직접 0 을 보낼 때** 쓴다.

        :attr:`HoldState.just_released` 가 뒤이어 뜨지 않는다. 0 이 두 번 나가면
        두 번째가 servo 의 정지 판정을 깨뜨려 오히려 늦어진다.
        """
        self._remaining = 0.0
        self._key = None
        self._zero_pending = False


__all__ = ['HoldKeyTracker', 'HoldState', 'TerminalKeyReader', 'TerminalRawMode']
