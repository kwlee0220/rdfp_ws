"""quality_gate 단위 테스트."""

from __future__ import annotations

from rdfp.dataset.ingest.quality_gate import QualityGate


def test_no_warnings_on_monotonic_stream() -> None:
    q = QualityGate()
    q.reset_episode(1)
    for t in range(100, 200, 10):
        q.on_message('/a', t)
    assert q.pop_warnings() == []


def test_detects_stamp_regression() -> None:
    q = QualityGate()
    q.reset_episode(1)
    q.on_message('/a', 100)
    q.on_message('/a', 90)      # 역행!
    warnings = q.pop_warnings()
    assert len(warnings) == 1
    assert warnings[0].kind == 'stamp_regression'
    assert warnings[0].topic == '/a'
    assert warnings[0].prev_stamp_ns == 100
    assert warnings[0].stamp_ns == 90


def test_regression_per_topic_independent() -> None:
    q = QualityGate()
    q.reset_episode(1)
    q.on_message('/a', 100)
    q.on_message('/b', 50)      # 다른 토픽이므로 정상
    q.on_message('/a', 110)     # 정상
    assert q.pop_warnings() == []


def test_idle_gap_detection() -> None:
    q = QualityGate(idle_gap_ns=1_000_000_000)   # 1초 이상 공백이면 경고
    q.reset_episode(1)
    q.on_message('/a', 100_000_000)
    q.on_message('/a', 500_000_000)             # 0.4초 갭 → 정상
    q.on_message('/a', 2_500_000_000)           # 2초 갭 → 경고
    warnings = q.pop_warnings()
    assert len(warnings) == 1
    assert warnings[0].kind == 'idle_gap'
    assert warnings[0].prev_stamp_ns == 500_000_000
    assert warnings[0].stamp_ns == 2_500_000_000


def test_idle_gap_disabled_by_default() -> None:
    q = QualityGate()   # idle_gap_ns=0
    q.reset_episode(1)
    q.on_message('/a', 0)
    q.on_message('/a', 10_000_000_000_000)
    assert q.pop_warnings() == []


def test_reset_clears_state() -> None:
    q = QualityGate()
    q.reset_episode(1)
    q.on_message('/a', 100)
    q.reset_episode(2)
    q.on_message('/a', 50)   # 새 에피소드이므로 이전 stamp 와 비교하지 않음
    assert q.pop_warnings() == []


def test_episode_id_attached_to_warning() -> None:
    q = QualityGate()
    q.reset_episode(42)
    q.on_message('/a', 100)
    q.on_message('/a', 50)
    [w] = q.pop_warnings()
    assert w.episode_id == 42


def test_pop_warnings_drains() -> None:
    q = QualityGate()
    q.reset_episode(1)
    q.on_message('/a', 100)
    q.on_message('/a', 50)
    assert len(q.pop_warnings()) == 1
    assert q.pop_warnings() == []
