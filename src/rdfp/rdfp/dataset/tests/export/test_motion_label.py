#!/usr/bin/env python3

"""관절-계획 레이블 함수 `plan(τ)` 와 chunk (E3).

**틀려도 에러가 안 난다** — 레이블은 그럴듯한 관절값이라, 규칙이 어긋나도 학습이
끝난 뒤 "정책이 안 움직인다"로만 드러난다. 그래서 규칙마다 따로 건다.

1. **구간 셋** — 첫 연산 이전은 초기 관측, 실행 중은 궤적 보간, 끝난 뒤는 끝점.
2. **중단은 중단 시각에서 자른다** — 그 뒤 값이 "팔이 가지 않은 자리"면 안 된다.
3. **연산 사이는 직전 끝점 유지** (컨트롤러가 실제로 붙들고 있다). 다만 **움직이는
   동안 경유점을 반복하는 것**은 금지다 — 그러면 정책이 "현재 위치"를 배운다.
4. **관절 순서를 정본에 맞춘다** — 어긋나면 값 범위가 비슷해 데이터로도 안 드러난다.
5. **chunk 는 경유점 번호가 아니라 시각으로 뽑고**, 연산 경계를 넘어도 자르지 않는다.

ROS 도 DB 도 없이 돈다.
"""

from __future__ import annotations

import pytest

from rdfp.dataset.export.motion_label import Operation, PlanLabel, Waypoint

JOINTS = ('j1', 'j2')


def _op(t0, pts, *, end=None, result='succeeded', names=JOINTS, op_id='op'):
    wps = tuple(Waypoint(t=t, positions=tuple(p), velocities=tuple(v))
                for t, p, v in pts)
    return Operation(t0=t0, joint_names=tuple(names), waypoints=wps,
                     end=t0 + (wps[-1].t if end is None else end),
                     result=result, operation_id=op_id)


def _label(ops, *, initial=(9.0, 9.0), episode_end=None):
    return PlanLabel(ops, joint_names=JOINTS, initial_positions=initial,
                     episode_end=episode_end)


#: 0 → 1 로 2초에 걸쳐 가는 궤적 (속도 없음 = 선형)
LINEAR = [(0.0, (0.0, 0.0), ()), (1.0, (0.5, -0.5), ()), (2.0, (1.0, -1.0), ())]


# ── 1. 구간 셋 ──────────────────────────────────────────────────────────────

def test_before_the_first_operation_uses_the_initial_observation() -> None:
    """첫 연산 이전에는 계획이 없다 — 에피소드 시작 관측을 쓴다."""
    lab = _label([_op(10.0, LINEAR)], initial=(9.0, 9.0))

    assert lab.at(0.0) == (9.0, 9.0)
    assert lab.at(9.999) == (9.0, 9.0)


def test_during_an_operation_interpolates_the_trajectory() -> None:
    lab = _label([_op(10.0, LINEAR)])

    assert lab.at(10.0) == pytest.approx((0.0, 0.0))
    assert lab.at(11.0) == pytest.approx((0.5, -0.5))
    assert lab.at(11.5) == pytest.approx((0.75, -0.75))


def test_after_a_normal_end_holds_the_endpoint() -> None:
    """연산이 끝나면 컨트롤러가 마지막 설정값을 붙들고 있다 — 그것이 사실이다."""
    lab = _label([_op(10.0, LINEAR)])

    assert lab.at(12.0) == pytest.approx((1.0, -1.0))
    assert lab.at(100.0) == pytest.approx((1.0, -1.0))


def test_no_operations_at_all_is_the_initial_observation() -> None:
    lab = _label([], initial=(0.1, 0.2))

    assert lab.at(0.0) == (0.1, 0.2) and lab.at(1e6) == (0.1, 0.2)


# ── 2. 중단 ─────────────────────────────────────────────────────────────────

def test_an_aborted_operation_is_cut_at_the_truncation_time() -> None:
    """**핵심.** 중단 뒤의 레이블이 팔이 **가지 않은 자리**를 가리키면 안 된다."""
    lab = _label([_op(10.0, LINEAR, end=1.0, result='aborted')])

    assert lab.at(11.0) == pytest.approx((0.5, -0.5))     # 멈춘 자리
    assert lab.at(50.0) == pytest.approx((0.5, -0.5))     # 그대로 유지
    assert lab.at(50.0) != pytest.approx((1.0, -1.0))     # 궤적 끝점이 아니다


def test_a_goal_rejected_operation_holds_the_trajectory_start() -> None:
    """goal 거부는 `aborted` + `truncated_at = 0` 이다 — 한 점도 안 갔다."""
    lab = _label([_op(10.0, LINEAR, end=0.0, result='aborted')])

    assert lab.at(20.0) == pytest.approx((0.0, 0.0))


def test_a_failed_operation_leaves_no_trace() -> None:
    """궤적을 넘기지도 못했으므로 시간축에 자리가 없다 — 앞 값이 그대로 이어진다."""
    lab = _label([_op(10.0, LINEAR), _op(20.0, LINEAR, result='failed', op_id='bad')])

    assert lab.at(25.0) == pytest.approx((1.0, -1.0))
    assert lab.stats.skipped_failed == 1


# ── 3. 연산 사이 ────────────────────────────────────────────────────────────

def test_the_gap_between_operations_holds_the_previous_endpoint() -> None:
    first = _op(10.0, LINEAR, op_id='a')
    second = _op(20.0, [(0.0, (1.0, -1.0), ()), (1.0, (2.0, -2.0), ())], op_id='b')
    lab = _label([first, second])

    assert lab.at(15.0) == pytest.approx((1.0, -1.0))    # 공백 — 끝점 유지
    assert lab.at(20.5) == pytest.approx((1.5, -1.5))    # 다음 연산이 시작됐다


def test_operations_are_sorted_by_start_time() -> None:
    """입력 순서에 기대지 않는다 — DB 정렬이 바뀌어도 같은 레이블이어야 한다."""
    a = _op(10.0, LINEAR, op_id='a')
    b = _op(20.0, [(0.0, (5.0, 5.0), ())], op_id='b')

    assert _label([b, a]).at(15.0) == pytest.approx((1.0, -1.0))


# ── 4. 관절 순서 ────────────────────────────────────────────────────────────

def test_joint_order_is_normalised_to_the_canonical_names() -> None:
    """어긋나면 값 범위가 비슷해 **데이터를 열어봐도 안 드러난다.**"""
    swapped = _op(10.0, [(0.0, (7.0, 3.0), ())], names=('j2', 'j1'))
    lab = _label([swapped])

    assert lab.at(10.0) == pytest.approx((3.0, 7.0))


def test_a_missing_joint_is_refused_at_construction() -> None:
    bad = _op(10.0, [(0.0, (1.0, 2.0), ())], names=('j1', 'jX'))

    with pytest.raises(ValueError, match='missing joints'):
        _label([bad])


def test_initial_positions_must_match_the_joint_count() -> None:
    with pytest.raises(ValueError, match='initial_positions'):
        PlanLabel([], joint_names=JOINTS, initial_positions=(1.0,))


# ── 5. 보간 방식 ────────────────────────────────────────────────────────────

def test_velocities_make_it_cubic_hermite() -> None:
    """속도가 있으면 Hermite 다 — 양 끝에서 위치와 속도를 둘 다 맞춘다."""
    pts = [(0.0, (0.0,), (0.0,)), (1.0, (1.0,), (0.0,))]
    lab = PlanLabel([_op(0.0, pts, names=('j1',))], joint_names=('j1',),
                    initial_positions=(0.0,))

    # 양 끝 속도가 0 이므로 중점은 선형(0.5)과 같지만, 1/4 지점은 더 완만하다.
    assert lab.at(0.5)[0] == pytest.approx(0.5)
    assert lab.at(0.25)[0] == pytest.approx(0.15625)
    assert lab.at(0.25)[0] < 0.25


def test_missing_velocities_fall_back_to_linear() -> None:
    lab = _label([_op(0.0, LINEAR)])

    assert lab.at(0.25)[0] == pytest.approx(0.125)


def test_a_single_point_trajectory_is_that_point() -> None:
    lab = _label([_op(10.0, [(0.0, (0.3, 0.4), ())])])

    assert lab.at(10.0) == pytest.approx((0.3, 0.4))
    assert lab.at(99.0) == pytest.approx((0.3, 0.4))


# ── 6. chunk ────────────────────────────────────────────────────────────────

def test_chunk_samples_at_t_plus_delta_then_steps() -> None:
    lab = _label([_op(0.0, LINEAR)])
    values, is_pad = lab.chunk(0.0, k=3, step=0.5, delta=0.5)

    assert [v[0] for v in values] == pytest.approx([0.25, 0.5, 0.75])
    assert is_pad == (False, False, False)


def test_chunk_crosses_the_operation_boundary_without_cutting() -> None:
    """시연이 이어서 한 동작이므로 **다음 연산의 계획이 들어오는 것이 맞다** (ACT §5.4)."""
    first = _op(0.0, [(0.0, (0.0, 0.0), ()), (1.0, (1.0, 1.0), ())], op_id='a')
    second = _op(2.0, [(0.0, (1.0, 1.0), ()), (1.0, (5.0, 5.0), ())], op_id='b')
    lab = _label([first, second])
    values, _ = lab.chunk(0.5, k=4, step=1.0, delta=0.5)

    # τ = 1.0(끝) / 2.0(다음 시작) / 3.0(다음 끝) / 4.0(유지)
    assert [v[0] for v in values] == pytest.approx([1.0, 1.0, 5.0, 5.0])


def test_is_pad_marks_steps_past_the_episode_end() -> None:
    lab = _label([_op(0.0, LINEAR)], episode_end=2.0)
    values, is_pad = lab.chunk(1.0, k=3, step=1.0, delta=0.0)

    assert is_pad == (False, False, True)
    # 패딩 구간의 값은 마지막 유효 자세가 이어진다 — 따로 채우지 않는다.
    assert values[2] == pytest.approx((1.0, -1.0))


def test_chunk_without_an_episode_end_pads_nothing() -> None:
    _, is_pad = _label([_op(0.0, LINEAR)]).chunk(0.0, k=3, step=1.0, delta=0.0)

    assert is_pad == (False, False, False)


@pytest.mark.parametrize('kw', [{'k': 0, 'step': 0.1}, {'k': 3, 'step': 0.0}])
def test_chunk_rejects_degenerate_shapes(kw) -> None:
    with pytest.raises(ValueError):
        _label([_op(0.0, LINEAR)]).chunk(0.0, delta=0.0, **kw)


# ── 7. 통계 ─────────────────────────────────────────────────────────────────

def test_overlapping_operations_are_counted() -> None:
    """팔 자원 락이 있으면 0 이어야 한다 — 0 이 아니면 데이터를 의심할 근거다."""
    a = _op(0.0, LINEAR, op_id='a')          # 0.0 ~ 2.0
    b = _op(1.0, LINEAR, op_id='b')          # 1.0 부터 — 겹친다
    lab = _label([a, b])

    assert lab.stats.overlapping == 1
    assert lab.at(1.5) == pytest.approx((0.25, -0.25)), '나중 연산이 이긴다'
