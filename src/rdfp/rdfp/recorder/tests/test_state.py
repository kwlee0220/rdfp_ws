#!/usr/bin/env python3

from __future__ import annotations

import logging

import pytest

from rdfp.recorder.exceptions import RecorderStateError
from rdfp.recorder.state import RecorderState, RecorderStateMachine


@pytest.fixture
def sm() -> RecorderStateMachine:
    return RecorderStateMachine(logging.getLogger("test.state"))


# ---------- 초기 상태 ---------------------------------------------------------


def test_initial_state_is_idle(sm: RecorderStateMachine) -> None:
    assert sm.state is RecorderState.IDLE
    assert not sm.is_shutdown()


# ---------- require / require_not_shutdown -----------------------------------


def test_require_passes_when_state_matches(sm: RecorderStateMachine) -> None:
    sm.require(RecorderState.IDLE)
    sm.require(RecorderState.IDLE, RecorderState.RECORDING)


def test_require_raises_when_state_mismatch(sm: RecorderStateMachine) -> None:
    with pytest.raises(RecorderStateError, match="not allowed in state IDLE"):
        sm.require(RecorderState.RECORDING)


def test_require_not_shutdown_passes_until_terminal(
    sm: RecorderStateMachine,
) -> None:
    sm.require_not_shutdown()  # IDLE
    sm.transition(RecorderState.RECORDING)
    sm.require_not_shutdown()
    sm.transition(RecorderState.SHUTDOWN)
    with pytest.raises(RecorderStateError, match="SHUTDOWN"):
        sm.require_not_shutdown()


# ---------- 허용 전이 ---------------------------------------------------------


@pytest.mark.parametrize(
    ("path"),
    [
        [RecorderState.RECORDING, RecorderState.STOPPING, RecorderState.IDLE],
        [RecorderState.RECORDING, RecorderState.FAILED, RecorderState.RECORDING],
        [RecorderState.RECORDING, RecorderState.STOPPING, RecorderState.FAILED],
        [RecorderState.SHUTDOWN],
        [RecorderState.RECORDING, RecorderState.SHUTDOWN],
    ],
)
def test_allowed_transition_paths(
    sm: RecorderStateMachine, path: list[RecorderState]
) -> None:
    for target in path:
        sm.transition(target)
        assert sm.state is target


def test_transition_same_state_is_noop(sm: RecorderStateMachine) -> None:
    before = sm.state
    result = sm.transition(RecorderState.IDLE)
    assert result is before
    assert sm.state is before


# ---------- 금지 전이 ---------------------------------------------------------


@pytest.mark.parametrize(
    ("initial", "target"),
    [
        (RecorderState.IDLE, RecorderState.STOPPING),
        (RecorderState.IDLE, RecorderState.FAILED),
        (RecorderState.STOPPING, RecorderState.RECORDING),  # STOPPING→RECORDING 금지
        (RecorderState.FAILED, RecorderState.STOPPING),
        (RecorderState.SHUTDOWN, RecorderState.IDLE),
        (RecorderState.SHUTDOWN, RecorderState.RECORDING),
    ],
)
def test_disallowed_transitions_raise(
    sm: RecorderStateMachine,
    initial: RecorderState,
    target: RecorderState,
) -> None:
    # IDLE 에서 출발하는 경우에 맞는 경로로 initial 까지 도달
    _force_to(sm, initial)
    with pytest.raises(RecorderStateError, match="invalid state transition"):
        sm.transition(target)


def _force_to(sm: RecorderStateMachine, target: RecorderState) -> None:
    """테스트용: 허용된 경로를 따라 target 까지 이동."""
    if target is RecorderState.IDLE:
        return
    if target is RecorderState.RECORDING:
        sm.transition(RecorderState.RECORDING)
        return
    if target is RecorderState.STOPPING:
        sm.transition(RecorderState.RECORDING)
        sm.transition(RecorderState.STOPPING)
        return
    if target is RecorderState.FAILED:
        sm.transition(RecorderState.RECORDING)
        sm.transition(RecorderState.FAILED)
        return
    if target is RecorderState.SHUTDOWN:
        sm.transition(RecorderState.SHUTDOWN)
        return


# ---------- try_transition ---------------------------------------------------


def test_try_transition_returns_true_on_success(
    sm: RecorderStateMachine,
) -> None:
    assert sm.try_transition(RecorderState.RECORDING) is True
    assert sm.state is RecorderState.RECORDING


def test_try_transition_returns_false_on_invalid(
    sm: RecorderStateMachine,
) -> None:
    # IDLE → STOPPING 은 금지
    assert sm.try_transition(RecorderState.STOPPING) is False
    assert sm.state is RecorderState.IDLE


def test_try_transition_from_states_filter(sm: RecorderStateMachine) -> None:
    # RECORDING 으로 이동
    sm.transition(RecorderState.RECORDING)
    # from_states 에 RECORDING 이 포함되면 성공
    assert sm.try_transition(
        RecorderState.FAILED,
        from_states=[RecorderState.RECORDING],
    ) is True
    assert sm.state is RecorderState.FAILED


def test_try_transition_from_states_filter_blocks(
    sm: RecorderStateMachine,
) -> None:
    sm.transition(RecorderState.RECORDING)
    # from_states 가 STOPPING 만 허용하므로 RECORDING 에서는 실패해야 한다
    assert sm.try_transition(
        RecorderState.FAILED,
        from_states=[RecorderState.STOPPING],
    ) is False
    assert sm.state is RecorderState.RECORDING


# ---------- 스레드 안전성 (기본 확인) ---------------------------------------


def test_lock_is_reentrant(sm: RecorderStateMachine) -> None:
    """state_machine.lock 은 RLock 이어야 중첩 호출 시 데드락이 없다."""
    with sm.lock:
        with sm.lock:  # RLock 이면 성공
            sm.require(RecorderState.IDLE)
