#!/usr/bin/env python3

"""`HoldKeyTracker` — 키를 누른 동안만 명령이 나가는 판정.

**터미널은 '키를 뗐다'를 알려 주지 않는다.** 자동반복 문자가 끊기는 것을 뗀 것으로
읽고 TTL 이 만료되면 멈춘다. 여기서 지키는 성질은 셋이다.

1. 자동반복 간격 동안 끊기지 않는다 (TTL 이 메운다).
2. 만료 시 **정확히 한 번** 0 을 보내라고 알린다 — 침묵하면 servo 가
   `incoming_command_timeout`(0.1s)을 다 기다리고 평활 필터가 마지막 속도를 이어
   내보내며, 매 틱 보내면 servo 가 자체 정지 경로로 못 들어간다.
3. 한 틱에 쌓인 문자 중 **마지막 것만** 유효하다.

`key_hold` 는 ROS 를 모르므로 이 파일도 ROS 없이 돈다.
"""

from __future__ import annotations

import pytest

from rdfp.teleop import key_hold
from rdfp.teleop.key_hold import HoldKeyTracker, HoldState

TICK = 0.01
TTL = 0.06
MOTION = ('q', 'a', 'j', 'l', 'i', 'k')
SPACE = ' '


class _Keys:
    """대역 입력원. `feed()` 한 것을 다음 `tick()` 이 한 번에 가져간다."""

    def __init__(self) -> None:
        self.pending: list = []
        self.reads = 0

    def feed(self, *keys: str) -> None:
        self.pending.extend(keys)

    def __call__(self) -> list:
        self.reads += 1
        keys, self.pending = self.pending, []
        return keys


@pytest.fixture
def keys() -> _Keys:
    return _Keys()


@pytest.fixture
def tracker(keys) -> HoldKeyTracker:
    return HoldKeyTracker(MOTION, ttl_sec=TTL, hold_keys=(SPACE,), read_keys=keys)


def run(tracker: HoldKeyTracker, n: int = 1) -> HoldState:
    """`n` 틱 진행하고 마지막 상태를 돌려준다."""
    state = HoldState()
    for _ in range(n):
        state = tracker.tick(TICK)
    return state


def zeros(tracker: HoldKeyTracker, n: int) -> int:
    """`n` 틱 도는 동안 `just_released` 가 뜬 횟수."""
    return sum(1 for _ in range(n) if tracker.tick(TICK).just_released)


# ----- 누르는 동안 -----------------------------------------------------------

def test_pressed_key_becomes_active(keys, tracker):
    keys.feed('q')
    state = tracker.tick(TICK)

    assert state.key == 'q'
    assert state.active is True
    assert state.just_released is False


def test_hold_survives_the_gap_between_autorepeat_chars(keys, tracker):
    """자동반복 간격(~33 ms)은 TTL(60 ms)이 메운다 — 홀드 중 끊기면 안 된다."""
    keys.feed('q')
    tracker.tick(TICK)

    state = run(tracker, 3)          # 30 ms 무입력

    assert state.active is True
    assert state.key == 'q'


def test_last_key_in_a_batch_wins(keys, tracker):
    """한 틱에 여러 문자가 쌓인다 — 앞의 것을 반영하면 지난 입력을 따라간다."""
    keys.feed('q', 'q', 'a')

    assert tracker.tick(TICK).key == 'a'


def test_switching_keys_switches_direction(keys, tracker):
    keys.feed('q')
    tracker.tick(TICK)
    keys.feed('l')

    assert tracker.tick(TICK).key == 'l'


# ----- 뗐을 때 ---------------------------------------------------------------

def test_release_reports_just_released_exactly_once(keys, tracker):
    """**핵심.** 만료 시 한 번만 알린다."""
    keys.feed('q')
    tracker.tick(TICK)

    assert zeros(tracker, 50) == 1


def test_key_is_cleared_after_release(keys, tracker):
    keys.feed('q')
    tracker.tick(TICK)
    state = run(tracker, 20)

    assert state.active is False
    assert state.key is None


def test_release_happens_within_the_ttl(keys, tracker):
    """TTL + 한 틱 안에 알려야 servo 의 0.1 s 타임아웃을 건너뛴다."""
    keys.feed('q')
    tracker.tick(TICK)

    ticks = 0
    while not tracker.tick(TICK).just_released:
        ticks += 1
        assert ticks < 30, 'just_released 가 영영 안 떴다'
    assert ticks <= int(TTL / TICK) + 1


def test_press_after_release_works_again(keys, tracker):
    """한 번 멈춘 뒤 다시 누르면 움직여야 한다 — 표시가 안 풀리는 실수를 잡는다."""
    keys.feed('q')
    tracker.tick(TICK)
    run(tracker, 20)

    keys.feed('q')
    state = tracker.tick(TICK)

    assert state.active is True and state.key == 'q'


def test_second_release_reports_again(keys, tracker):
    total = 0
    for _ in range(2):
        keys.feed('q')
        tracker.tick(TICK)
        total += zeros(tracker, 20)

    assert total == 2


def test_idle_tracker_never_reports_a_release(tracker):
    """아무도 안 눌렀는데 기동 직후 0 을 쏘면 안 된다."""
    assert zeros(tracker, 50) == 0


# ----- 홀드 키(SPACE) --------------------------------------------------------

def test_hold_key_refreshes_without_moving(keys, tracker):
    """SPACE 는 TTL 만 갱신한다 — 살아 있지만 움직이지는 않는다."""
    keys.feed(SPACE)
    state = tracker.tick(TICK)

    assert state.active is True
    assert state.key is None


def test_hold_key_after_motion_key_stops_the_motion(keys, tracker):
    """같은 배치에서 SPACE 가 뒤에 오면 그것이 이긴다 — 마지막 것만 유효하다."""
    keys.feed('q', SPACE)

    assert tracker.tick(TICK).key is None


# ----- 그 밖의 키 ------------------------------------------------------------

def test_other_keys_pass_through_in_order(keys, tracker):
    """모션·홀드 키가 아닌 것은 그대로 넘겨 호출자가 처리한다."""
    keys.feed('=', 'q', '[')

    state = tracker.tick(TICK)

    assert state.other_keys == ('=', '[')
    assert state.key == 'q', '같은 배치의 모션 키는 그대로 유효하다'


def test_other_keys_do_not_refresh_the_hold(keys, tracker):
    """one-shot 키를 연타한다고 팔이 계속 움직이면 안 된다."""
    keys.feed('q')
    tracker.tick(TICK)

    released = False
    for _ in range(20):
        keys.feed('=')
        released = released or tracker.tick(TICK).just_released

    assert released is True


# ----- cancel ---------------------------------------------------------------

def test_cancel_stops_immediately(keys, tracker):
    keys.feed('q')
    tracker.tick(TICK)

    tracker.cancel()
    state = tracker.tick(TICK)

    assert state.active is False and state.key is None


def test_cancel_suppresses_the_release_report(keys, tracker):
    """정지 키는 호출자가 직접 0 을 보낸다 — 여기서 또 알리면 0 이 두 번 나간다.

    두 번째 0 이 servo 의 정지 판정을 다시 깨뜨려 오히려 늦어진다.
    """
    keys.feed('q')
    tracker.tick(TICK)
    tracker.cancel()

    assert zeros(tracker, 30) == 0


# ----- 생성 인자 -------------------------------------------------------------

@pytest.mark.parametrize('bad', [0.0, -0.1])
def test_nonpositive_ttl_is_rejected(bad, keys):
    """TTL 이 0 이면 홀드가 성립하지 않는다 — 조용히 안 움직이는 것보다 낫다."""
    with pytest.raises(ValueError, match='ttl_sec'):
        HoldKeyTracker(MOTION, ttl_sec=bad, read_keys=keys)


def test_reader_is_consulted_once_per_tick(keys, tracker):
    """틱마다 버퍼를 **완전히** 비운다 — 남기면 잔여 문자가 TTL 을 계속 갱신한다."""
    run(tracker, 5)

    assert keys.reads == 5
    assert keys.pending == []


def test_state_is_immutable(keys, tracker):
    """호출자가 상태를 바꿔 다음 틱에 영향을 주면 안 된다."""
    keys.feed('q')
    state = tracker.tick(TICK)

    with pytest.raises(Exception):
        state.key = 'a'         # type: ignore[misc]


def test_defaults_are_a_terminal_reader():
    """`read_keys` 를 안 주면 터미널을 읽는다 — 생성만으로 stdin 을 건드리지 않는다."""
    tracker = HoldKeyTracker(MOTION, ttl_sec=TTL)

    assert tracker.active is False
    assert tracker.key is None


# ----- TerminalKeyReader ----------------------------------------------------

class _FakeStdin:
    """`select` 와 `read(1)` 만 흉내내는 stdin."""

    def __init__(self, chars: str) -> None:
        self.chars = list(chars)

    def fileno(self) -> int:
        return 0

    def read(self, n: int) -> str:
        return self.chars.pop(0) if self.chars else ''


@pytest.fixture
def fake_stdin(monkeypatch):
    def _install(chars: str):
        stdin = _FakeStdin(chars)
        monkeypatch.setattr(key_hold.sys, 'stdin', stdin)
        monkeypatch.setattr(key_hold.select, 'select',
                            lambda r, w, x, t: ((r if stdin.chars else []), [], []))
        return stdin
    return _install


def test_reader_drains_the_whole_buffer(fake_stdin):
    """**한 틱에 다 비워야 한다.**

    한 글자만 소비하면 자동반복으로 쌓인 잔여 문자가 다음 틱들의 TTL 을 계속
    갱신해, 키를 뗀 뒤에도 팔이 움직인다 — 이 프로젝트가 실제로 겪은 증상이다.
    """
    fake_stdin('qqqq')

    assert key_hold.TerminalKeyReader()() == ['q', 'q', 'q', 'q']


def test_reader_returns_empty_when_nothing_is_buffered(fake_stdin):
    fake_stdin('')

    assert key_hold.TerminalKeyReader()() == []


def test_reader_assembles_an_escape_sequence(fake_stdin):
    """화살표는 `\\x1b[A` 세 바이트다.

    합치지 않으면 '[' 가 episode_start 키로, 'A' 가 알 수 없는 키로 오인식된다.
    """
    fake_stdin('\x1b[A')

    assert key_hold.TerminalKeyReader()() == ['\x1b[A']


def test_reader_keeps_a_bare_escape_separate(fake_stdin):
    """ESC 뒤에 '[' 가 아닌 것이 오면 escape sequence 가 아니다."""
    fake_stdin('\x1bq')

    assert key_hold.TerminalKeyReader()() == ['\x1bq']
