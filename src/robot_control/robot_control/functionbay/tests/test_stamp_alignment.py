"""`stamp_alignment` 의 오프셋 추적과 sim stamp 짝 찾기 테스트 (ROS 불필요).

합성 스트림은 2026-09-17 실측 조건을 흉내 낸다 — 관절 50 Hz 가 약 48 ms 마다 묶여 도착하고
(체크리스트 A-21), 주기적 정지 동안 벽시계는 1 s 흐르는데 sim 시각은 0.1 s 만 간다(A-17).
"""

from __future__ import annotations

import pytest

from robot_control.functionbay.stamp_alignment import MonotonicStamps, OffsetHistory, StampAligner

MS = 1_000_000
SEC = 1_000_000_000
PERIOD = 20 * MS            # 관절 50 Hz
BATCH = 48 * MS             # 묶음 도착 간격
LATENCY = 2 * MS            # 묶음 발송 뒤 수신까지
ELAPSED_EPOCH = 1_789_000_000 * SEC


def _stream(duration_ns, epoch_ns, stall_at_ns=None, stall_wall_ns=SEC, stall_sim_ns=100 * MS):
    """(수신 ns, sim stamp ns) 목록. 표본은 만들어진 뒤 다음 묶음 발송 때 한꺼번에 도착한다."""
    produced = []                                     # (생산 벽시계, sim stamp)
    wall, sim = 0, 0
    stalled = False
    while wall < duration_ns:
        if stall_at_ns is not None and not stalled and wall >= stall_at_ns:
            wall += stall_wall_ns                     # 벽시계는 흐르고
            sim += stall_sim_ns                       # sim 은 조금만 간다
            stalled = True
        produced.append((wall, sim))
        wall += PERIOD
        sim += PERIOD
    rows, send = [], BATCH
    for wall, sim in produced:
        while send < wall:
            send += BATCH
        rows.append((send + LATENCY + epoch_ns, sim))
    return rows


def _align_all(rows):
    aligner = StampAligner()
    out = []
    for recv, sim in rows:
        aligner.observe(recv, sim)
        out.append(aligner.align(sim))
    return aligner, out


def test_offset_absorbs_elapsed_epoch_and_keeps_spacing():
    """경과시간 stamp 도 벽시계 축으로 옮기고, 창이 찬 뒤에는 간격을 20 ms 그대로 둔다."""
    rows = _stream(5 * SEC, ELAPSED_EPOCH)
    _, aligned = _align_all(rows)

    settled = rows[0][0] + 500 * MS               # 창(0.5 s)이 찬 뒤
    steady = [a for (recv, _), a in zip(rows, aligned) if recv > settled]
    assert {b - a for a, b in zip(steady, steady[1:])} == {PERIOD}
    # 수신 시각을 stamp 로 쓰면 묶음 안의 표본이 같은 시각에 몰린다 — 그와 대조한다.
    assert len({recv for recv, _ in rows}) < len(rows) / 2


def test_spacing_never_goes_backwards_while_the_window_fills():
    """창이 차는 동안에는 간격이 좁아질 수 있지만(같은 묶음이 한 시각에 모인다) 역순은 없다."""
    rows = _stream(6 * SEC, ELAPSED_EPOCH, stall_at_ns=3 * SEC)
    _, aligned = _align_all(rows)
    assert min(b - a for a, b in zip(aligned, aligned[1:])) >= 0


def test_aligned_stamp_never_runs_ahead_of_reception():
    """정렬 시각은 수신 시각보다 늦을 수 없고, 묶음 대기 + 지연 이내로 앞선다."""
    rows = _stream(5 * SEC, ELAPSED_EPOCH)
    _, aligned = _align_all(rows)
    lags = [recv - stamp for (recv, _), stamp in zip(rows, aligned)]
    assert min(lags) >= 0
    assert max(lags) <= BATCH + LATENCY


def test_wall_clock_stamps_get_a_small_offset():
    """시뮬레이터가 이미 벽시계면 오프셋은 전달 지연 수준이다."""
    aligner, _ = _align_all(_stream(3 * SEC, epoch_ns=0))
    assert 0 <= aligner.offset_ns <= BATCH + LATENCY


def test_stall_resets_the_window_instead_of_pulling_samples_into_the_gap():
    """정지 뒤 첫 표본은 공백 뒤에 놓여야 한다 — 창을 안 비우면 약 0.9 s 당겨진다."""
    rows = _stream(6 * SEC, ELAPSED_EPOCH, stall_at_ns=3 * SEC)
    aligner, aligned = _align_all(rows)

    assert aligner.resets == 1
    recv = [r for r, _ in rows]
    gap = max(range(1, len(recv)), key=lambda i: recv[i] - recv[i - 1])
    assert aligned[gap] - aligned[gap - 1] > 800 * MS
    assert all(r >= a for r, a in zip(recv, aligned))


def test_align_before_any_sample_is_none():
    assert StampAligner().align(123) is None


@pytest.mark.parametrize('window, gap', [(0, 1), (1, 0)])
def test_rejects_non_positive_windows(window, gap):
    with pytest.raises(ValueError):
        StampAligner(window_ns=window, gap_ns=gap)


def test_monotonic_stamps_never_go_backwards_and_count_the_clamps():
    """정지 직후 오프셋이 내려가 역순이 될 표본은 직전 출력 바로 뒤로 누른다."""
    clamp = MonotonicStamps()
    assert [clamp(v) for v in (100, 200, 194, 195, 300)] == [100, 200, 201, 202, 300]
    assert clamp.clamped == 2


def test_history_waits_for_a_later_pair():
    """sim stamp 가 뒤쪽 기록보다 새로우면 아직 모른다 (``None``)."""
    history = OffsetHistory()
    history.add(100, 7)
    assert history.lookup(150) is None
    assert history.nearest(150) == 7


def test_history_picks_the_nearer_side_and_tolerates_out_of_order_adds():
    history = OffsetHistory()
    history.add(300, 30)
    history.add(100, 10)             # 늦게 도착한 앞쪽 기록
    assert history.lookup(140) == 10
    assert history.lookup(260) == 30
    assert history.lookup(100) == 10


def test_history_prunes_old_entries():
    history = OffsetHistory(keep_ns=1 * SEC)
    for k in range(5):
        history.add(k * SEC, k)
    assert len(history) == 2          # 3 s, 4 s 만 남는다


def test_shared_offset_preserves_the_camera_joint_relation_across_a_stall():
    """카메라에 관절 짝의 오프셋을 쓰면 sim stamp 관계가 정지 경계에서도 보존된다."""
    joints = _stream(6 * SEC, ELAPSED_EPOCH, stall_at_ns=3 * SEC)
    aligner = StampAligner()
    history = OffsetHistory()
    for recv, sim in joints:
        history.add(sim, aligner.observe(recv, sim) + 0)

    joint_sims = [sim for _, sim in joints]
    for cam_sim in range(7 * MS, joint_sims[-1], 100 * MS):     # 카메라 10 Hz, 관절보다 7 ms 늦게
        offset = history.lookup(cam_sim)
        nearest = min(joint_sims, key=lambda s: abs(s - cam_sim))
        joint_offset = history.lookup(nearest)
        assert (cam_sim + offset) - (nearest + joint_offset) == cam_sim - nearest
