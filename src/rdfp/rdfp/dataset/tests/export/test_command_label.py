#!/usr/bin/env python3

"""관절-명령 레이블 (E5) — 비교 기준.

**틀려도 에러가 안 난다.** 그럴듯한 관절값이라 규칙이 어긋나도 학습 뒤에야 드러난다.

1. **공백은 보간하지 않고 유지한다** — 명령은 팔이 움직이는 동안에만 나오므로,
   앞 동작의 끝과 다음 동작의 시작을 이으면 **없던 움직임**이 생긴다. 컨트롤러는
   실제로 마지막 설정값을 붙들고 있다.
2. **공백 임계는 명령 주기에서 추정한다** — isaac 100 Hz, 펑션베이 50 Hz 로 달라
   호출자가 백엔드를 알지 않아도 되게 한다. 주기는 **중앙값**이다(평균은 긴 공백
   몇 개에 끌려간다).
3. **첫 명령 이전은 에피소드 시작 관측** — 관절-계획의 "첫 연산 이전" 과 같다.
4. **`PlanLabel` 과 같은 인터페이스** — `arm_action_chunks()` 에 바꿔 끼워 같은
   격자에서 두 레이블을 비교하는 것이 이 항목의 목적이다.

ROS 도 DB 도 없이 돈다.
"""

from __future__ import annotations

import pytest

from rdfp.dataset.export.command_label import CommandLabel
from rdfp.dataset.export.frame_clock import FrameClock, FrameRef, ImageStream
from rdfp.dataset.export.observation import JointSeries

NS = 1_000_000_000
NAMES = ('j1', 'j2')


def _series(pairs, names=NAMES) -> JointSeries:
    return JointSeries(joint_names=tuple(names),
                       stamps_ns=tuple(int(round(t * NS)) for t, _ in pairs),
                       positions=tuple(tuple(p) for _, p in pairs))


def _dense(t0, n, step=0.02, start=0.0, rate=1.0) -> list:
    """`step` 간격의 조밀한 명령열."""
    return [(t0 + i * step, (start + i * rate * step, -(start + i * rate * step)))
            for i in range(n)]


def _label(pairs, *, initial=(9.0, 9.0), gap=None, end=None) -> CommandLabel:
    return CommandLabel(_series(pairs), initial_positions=initial,
                        gap_sec=gap, episode_end=end)


# ── 1. 보간 ─────────────────────────────────────────────────────────────────

def test_commands_are_linearly_interpolated():
    lab = _label([(100.0, (0.0, 0.0)), (100.02, (1.0, -1.0))], gap=0.1)

    assert lab.at(100.01) == pytest.approx((0.5, -0.5))


def test_before_the_first_command_is_the_initial_observation():
    lab = _label(_dense(100.0, 5), initial=(9.0, 9.0))

    assert lab.at(99.0) == (9.0, 9.0)


def test_after_the_last_command_holds():
    """컨트롤러가 마지막 설정값을 붙들고 있다."""
    lab = _label(_dense(100.0, 5))
    last = lab.at(100.08)

    assert lab.at(200.0) == pytest.approx(last)


# ── 2. 공백 ─────────────────────────────────────────────────────────────────

def test_a_gap_holds_instead_of_interpolating():
    """**핵심.** 보간하면 앞 동작과 다음 동작 사이에 없던 움직임이 생긴다."""
    lab = _label([(100.0, (1.0, -1.0)), (105.0, (5.0, -5.0))], gap=0.04)

    assert lab.at(102.5) == pytest.approx((1.0, -1.0)), '직전 명령 유지'
    assert lab.at(102.5) != pytest.approx((3.0, -3.0)), '선형 보간이 아니다'


def test_a_short_interval_still_interpolates():
    lab = _label([(100.0, (0.0, 0.0)), (100.03, (3.0, -3.0))], gap=0.04)

    assert lab.at(100.01) == pytest.approx((1.0, -1.0))


def test_gap_threshold_is_inferred_from_the_median_period():
    """isaac 100 Hz · 펑션베이 50 Hz — 호출자가 백엔드를 몰라도 되게 한다."""
    lab = _label(_dense(100.0, 10, step=0.02))

    assert lab.period_sec == pytest.approx(0.02)
    assert lab.gap_sec == pytest.approx(0.04)


def test_the_period_is_a_median_not_a_mean():
    """**긴 공백 몇 개가 평균을 통째로 끌어올린다.**"""
    pairs = _dense(100.0, 10, step=0.02) + [(200.0, (9.0, -9.0))]
    lab = _label(pairs)

    assert lab.period_sec == pytest.approx(0.02), '중앙값이라 100초 공백에 안 흔들린다'
    assert lab.gap_sec == pytest.approx(0.04)


def test_gap_intervals_expose_the_holes():
    """연산 사이의 공백이 여기 드러난다 — 개수가 연산 수와 다르면 의심할 근거다."""
    pairs = _dense(100.0, 5, step=0.02) + _dense(105.0, 5, step=0.02, start=1.0)
    lab = _label(pairs)
    gaps = lab.gap_intervals()

    assert len(gaps) == 1
    assert gaps[0][0] == pytest.approx(100.08) and gaps[0][1] == pytest.approx(105.0)


def test_an_explicit_threshold_wins_over_the_estimate():
    lab = _label(_dense(100.0, 10, step=0.02), gap=1.0)

    assert lab.gap_sec == 1.0


# ── 3. 생성 인자 ────────────────────────────────────────────────────────────

def test_an_empty_series_is_refused():
    with pytest.raises(ValueError, match='empty'):
        CommandLabel(_series([]), initial_positions=(0.0, 0.0))


def test_initial_positions_must_match_the_joint_count():
    with pytest.raises(ValueError, match='initial_positions'):
        CommandLabel(_series(_dense(100.0, 3)), initial_positions=(0.0,))


def test_a_single_command_holds_everywhere():
    lab = _label([(100.0, (0.3, 0.4))])

    assert lab.at(100.0) == pytest.approx((0.3, 0.4))
    assert lab.at(999.0) == pytest.approx((0.3, 0.4))
    assert lab.at(50.0) == (9.0, 9.0), '그 전은 초기 관측'


# ── 4. chunk — PlanLabel 과 같은 모양 ───────────────────────────────────────

def test_chunk_samples_at_t_plus_delta_then_steps():
    lab = _label(_dense(100.0, 20, step=0.02), gap=0.05)
    values, is_pad = lab.chunk(100.0, k=3, step=0.02, delta=0.02)

    assert [round(v[0], 4) for v in values] == pytest.approx([0.02, 0.04, 0.06])
    assert is_pad == (False, False, False)


def test_is_pad_marks_steps_past_the_episode_end():
    lab = _label(_dense(100.0, 20, step=0.02), end=100.1)
    _, is_pad = lab.chunk(100.06, k=3, step=0.02, delta=0.02)

    assert is_pad == (False, False, True), 'τ = 100.08 / 100.10 / 100.12'


@pytest.mark.parametrize('kw', [{'k': 0, 'step': 0.02}, {'k': 3, 'step': 0.0}])
def test_chunk_rejects_degenerate_shapes(kw):
    with pytest.raises(ValueError):
        _label(_dense(100.0, 5)).chunk(100.0, delta=0.0, **kw)


def test_it_drops_into_arm_action_chunks_like_the_plan_label():
    """**두 레이블을 같은 격자에서 비교하는 것이 이 항목의 목적이다.**"""
    from rdfp.dataset.export.frame_clock import arm_action_chunks

    clock = FrameClock(
        episode_id=1,
        stream=ImageStream(topic='/c', mp4_path='a.mp4', codec='libx264',
                           pixel_format='bgr8', fps=10, width=32, height=32,
                           frame_count=2),
        frames=(FrameRef(0, int(100.0 * NS)), FrameRef(1, int(100.1 * NS))),
        episode_start_ns=int(100.0 * NS), episode_end_ns=int(101.0 * NS))
    lab = _label(_dense(100.0, 30, step=0.02), gap=0.05, end=clock.episode_end)

    rows = arm_action_chunks(clock, lab, k=2, step=0.02, delta=0.02)

    assert len(rows) == 2
    assert [r[0] for r in rows] == [0, 1]
    assert [round(v[0], 4) for v in rows[0][2]] == pytest.approx([0.02, 0.04])
