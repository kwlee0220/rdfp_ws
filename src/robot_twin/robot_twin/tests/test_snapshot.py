"""`robot_twin.snapshot` 단위 테스트 — ROS 없이 동작한다."""

from __future__ import annotations

from robot_twin.snapshot import (
    GenerationCounter, Quality, Reason, Snapshot, judge
)


def _snap(received_at: float = 100.0, error: str = None) -> Snapshot:
    return Snapshot(msg=object(), received_at=received_at, gen=1, error=error)


def test_generation_counter_is_monotonic() -> None:
    gen = GenerationCounter()

    assert [gen.next() for _ in range(3)] == [1, 2, 3]


def test_no_data_when_snapshot_missing() -> None:
    st = judge(None, staleness_ms=500)

    assert st.quality is Quality.NO_DATA
    assert st.age_ms is None


def test_source_unavailable_takes_precedence_over_age() -> None:
    """퍼블리셔 없음과 퍼블리셔 멈춤은 age 만으로 구분되지 않는다 (설계서 5.1)."""
    st = judge(_snap(), staleness_ms=500, source_available=False,
               reason=Reason.NO_PUBLISHER, now=100.0)

    assert st.quality is Quality.SOURCE_UNAVAILABLE
    assert st.reason is Reason.NO_PUBLISHER


def test_source_unavailable_even_without_snapshot() -> None:
    st = judge(None, staleness_ms=None, source_available=False,
               reason=Reason.SOURCE_NODE_DOWN)

    assert st.quality is Quality.SOURCE_UNAVAILABLE
    assert st.reason is Reason.SOURCE_NODE_DOWN


def test_ok_within_threshold() -> None:
    st = judge(_snap(received_at=100.0), staleness_ms=500, now=100.2)

    assert st.quality is Quality.OK
    assert st.age_ms == 200


def test_stale_beyond_threshold() -> None:
    st = judge(_snap(received_at=100.0), staleness_ms=500, now=100.6)

    assert st.quality is Quality.STALE
    assert st.age_ms == 600


def test_null_staleness_never_goes_stale() -> None:
    """이벤트성·정적 소스는 값이 오래된 것이 정상이다 (설계서 5.1)."""
    st = judge(_snap(received_at=0.0), staleness_ms=None, now=100000.0)

    assert st.quality is Quality.OK
    assert st.age_ms == 100000000


def test_error_snapshot_reports_error() -> None:
    st = judge(_snap(error='empty name'), staleness_ms=500, now=100.0)

    assert st.quality is Quality.ERROR
    assert st.error == 'empty name'


def test_age_is_never_negative() -> None:
    """시계가 뒤로 갈 수 있다 (NTP 보정 등)."""
    st = judge(_snap(received_at=200.0), staleness_ms=500, now=100.0)

    assert st.age_ms == 0
    assert st.quality is Quality.OK


def test_snapshot_is_immutable() -> None:
    snap = _snap()

    try:
        snap.received_at = 1.0
    except Exception as exc:  # frozen dataclass 는 FrozenInstanceError 를 던진다
        assert 'frozen' in type(exc).__name__.lower() or 'frozen' in str(exc).lower()
    else:
        raise AssertionError('Snapshot must be immutable')
