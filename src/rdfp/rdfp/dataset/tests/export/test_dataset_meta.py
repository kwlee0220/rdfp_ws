#!/usr/bin/env python3

"""정규화와 데이터셋 metadata (E6).

지키는 것:

1. **`NaN` 이 통계를 오염시키지 않는다** — 펑션베이는 그리퍼 폭을 못 구해 `NaN` 을
   낸다(0 은 '닫혀 있다'는 거짓말이라 그렇게 둔 것이다). 한 샘플이 섞이면 평균과
   표준편차가 통째로 `NaN` 이 된다.
2. **움직이지 않은 차원을 0 으로 나누지 않는다** — `inf` 가 되어 학습이 조용히
   망가진다. 정규화를 건너뛰고 **경고로 남긴다**(데이터가 치우쳤다는 신호다).
3. **배치 규약 불일치를 검사로 잡는다** — ACT §7 이 *"카메라 주기 · δ · T_a · k 가
   같아야 한다"* 고 못 박지만 **어긋나도 에러가 안 난다.** 정책이 조금씩 틀린 곳을
   겨냥할 뿐이라 "학습은 됐는데 잘 안 된다"로만 나타난다.
4. **왕복한다** — 저장했다 읽어도 같은 값이어야 배치가 쓸 수 있다.

ROS 도 DB 도 없이 돈다(마지막 로더만 예외).
"""

from __future__ import annotations

import math

import pytest

from rdfp.dataset.export.dataset_meta import (
    DatasetMetadata, ExportSpec, MetadataError, Normalizer, RunningStats,
    build_metadata)

ARM = ('j1', 'j2')
SYMS = ('open', 'close', 'grasp')


def _spec(**kw) -> ExportSpec:
    base = dict(label='joint_plan', delta_sec=0.1, step_sec=0.1, chunk_len=10,
                arm_joints=ARM, gripper_symbols=SYMS,
                camera_topic='/camera/image_raw', camera_hz=10.0)
    base.update(kw)
    return ExportSpec(**base)


# ── 1. 통계 ─────────────────────────────────────────────────────────────────

def test_mean_and_std_are_per_dimension() -> None:
    st = RunningStats(dim=2)
    st.update_many([(0.0, 10.0), (2.0, 10.0), (4.0, 10.0)])

    assert st.mean == pytest.approx((2.0, 10.0))
    assert st.std[0] == pytest.approx(math.sqrt(8 / 3))
    assert st.std[1] == 0.0


def test_nan_is_skipped_per_dimension_and_counted() -> None:
    """**핵심.** 한 샘플이 통계를 통째로 `NaN` 으로 만들면 안 된다."""
    st = RunningStats(dim=2)
    st.update_many([(1.0, math.nan), (3.0, 5.0)])

    assert st.mean[0] == pytest.approx(2.0)
    assert st.mean[1] == pytest.approx(5.0), '다른 차원은 멀쩡하다'
    assert st.nan_count == [0, 1]
    assert not any(math.isnan(v) for v in st.mean + st.std)


def test_a_dimension_with_only_nan_has_no_samples() -> None:
    st = RunningStats(dim=1)
    st.update_many([(math.nan,), (math.nan,)])

    assert st.count == [0] and st.mean == (0.0,) and st.std == (0.0,)


def test_wrong_width_is_refused() -> None:
    with pytest.raises(ValueError, match='expected 2 values'):
        RunningStats(dim=2).update((1.0,))


def test_streaming_matches_a_direct_computation() -> None:
    """스트리밍 누적이 정확값과 같아야 한다 (`statistics` 는 내부에서 유리수로 센다).

    입력은 일부러 고약하게 잡았다 — 값이 10⁶ 인데 퍼짐이 0.06 이라 **자릿수 상쇄**가
    일어나는 조건이다. 관절값은 이 정도로 나쁘지 않지만, 여기서 견디면 실제
    데이터에서는 문제가 없다.
    """
    import statistics
    xs = [1e6 + i * 0.001 for i in range(200)]
    st = RunningStats(dim=1)
    st.update_many([(x,) for x in xs])

    assert st.mean[0] == pytest.approx(statistics.fmean(xs), rel=1e-12)
    assert st.std[0] == pytest.approx(statistics.pstdev(xs), rel=1e-6)


def test_welford_beats_the_naive_sum_of_squares() -> None:
    """**Welford 를 쓰는 이유다.** 단순 누적합은 같은 입력에서 무너진다.

    `E[x²] − E[x]²` 는 두 큰 수의 차라서, 값이 크고 퍼짐이 작으면 유효숫자가
    통째로 날아간다 — 음수가 나와 `sqrt` 가 터지기도 한다.
    """
    import math
    import statistics
    xs = [1e6 + i * 0.001 for i in range(200)]
    exact = statistics.pstdev(xs)

    st = RunningStats(dim=1)
    st.update_many([(x,) for x in xs])
    welford_err = abs(st.std[0] - exact) / exact

    n = len(xs)
    naive_var = sum(x * x for x in xs) / n - (sum(xs) / n) ** 2
    naive = math.sqrt(naive_var) if naive_var > 0 else float('nan')
    naive_err = float('inf') if math.isnan(naive) else abs(naive - exact) / exact

    assert welford_err < naive_err, f'welford={welford_err:.2e} naive={naive_err:.2e}'
    assert welford_err < 1e-6


# ── 2. 정규화 ───────────────────────────────────────────────────────────────

def test_normalizer_round_trips() -> None:
    st = RunningStats(dim=2)
    st.update_many([(0.0, 1.0), (2.0, 3.0), (4.0, 5.0)])
    n = Normalizer.from_stats(st)

    assert n.invert(n.apply((1.5, 2.5))) == pytest.approx((1.5, 2.5))


def test_a_constant_dimension_is_left_alone() -> None:
    """**0 으로 나누면 `inf` 가 되어 학습이 조용히 망가진다.**"""
    st = RunningStats(dim=2)
    st.update_many([(1.0, 7.0), (3.0, 7.0)])
    n = Normalizer.from_stats(st)

    assert n.degenerate == (1,)
    assert n.std[1] == 1.0
    assert n.apply((2.0, 7.0))[1] == 0.0
    assert not any(math.isinf(v) for v in n.apply((2.0, 7.0)))


def test_normalizer_dict_round_trip() -> None:
    n = Normalizer(mean=(1.0, 2.0), std=(0.5, 1.0), degenerate=(1,))

    assert Normalizer.from_dict(n.to_dict()) == n


# ── 3. 배치 규약 검사 ───────────────────────────────────────────────────────

def test_identical_specs_are_compatible() -> None:
    _spec().assert_compatible(_spec())


@pytest.mark.parametrize('kw', [
    {'delta_sec': 0.2}, {'step_sec': 0.02}, {'chunk_len': 20},
    {'label': 'joint_command'}, {'arm_joints': ('j2', 'j1')},
    {'gripper_symbols': ('close', 'open', 'grasp')},
])
def test_a_mismatch_is_raised(kw) -> None:
    """**어긋나도 에러가 안 나는 것이 문제라서** 검사를 명시적으로 둔다."""
    with pytest.raises(MetadataError, match='does not match training'):
        _spec().assert_compatible(_spec(**kw))


def test_all_mismatches_are_reported_together() -> None:
    """하나씩 고치며 다시 돌리게 하지 않는다."""
    with pytest.raises(MetadataError) as e:
        _spec().assert_compatible(_spec(chunk_len=20, step_sec=0.02))

    assert 'chunk_len' in str(e.value) and 'step_sec' in str(e.value)


def test_camera_hz_tolerates_real_world_jitter() -> None:
    """실측값이라 회차마다 흔들린다 — 펑션베이 9.54~9.60 Hz."""
    _spec(camera_hz=9.60).assert_compatible(_spec(camera_hz=9.54))


def test_a_large_camera_hz_change_is_raised() -> None:
    """주기가 크게 달라지면 chunk 가 덮는 시간 범위가 달라진다."""
    with pytest.raises(MetadataError, match='camera_hz'):
        _spec(camera_hz=10.0).assert_compatible(_spec(camera_hz=50.0))


def test_backend_mismatch_is_raised_when_both_are_known() -> None:
    """관절-명령 레이블은 **백엔드를 섞으면 안 된다.**"""
    with pytest.raises(MetadataError, match='backend'):
        _spec(backend='isaac').assert_compatible(_spec(backend='functionbay'))


def test_an_unknown_backend_does_not_block() -> None:
    """아직 기록하지 않는 데이터가 있다 (R4 미구현)."""
    _spec(backend='isaac').assert_compatible(_spec(backend=None))


# ── 4. metadata 조립 ────────────────────────────────────────────────────────

def _built(**kw) -> DatasetMetadata:
    q = RunningStats(dim=3)
    q.update_many([(0.0, 1.0, 0.08), (2.0, 1.0, math.nan)])
    a = RunningStats(dim=2)
    a.update_many([(0.0, 0.0), (1.0, 2.0)])
    return build_metadata(_spec(**kw), q, a, episodes=[1, 2], samples=2)


def test_notes_name_the_joint_that_never_moved() -> None:
    """숫자만 남기면 아무도 안 본다 — 사람이 읽을 문장으로 남긴다."""
    notes = ' '.join(_built().notes)

    assert 'j2' in notes and 'never varied' in notes


def test_notes_count_nan_samples() -> None:
    assert any('NaN' in n for n in _built().notes)


def test_notes_flag_the_missing_backend_and_model() -> None:
    """**모르는 것을 모른다고 적는다** — 채워지면 사라진다."""
    notes = ' '.join(_built().notes)

    assert 'backend is unknown' in notes
    assert 'kinematic model fingerprint is unknown' in notes


def test_a_known_backend_leaves_no_note() -> None:
    notes = ' '.join(_built(backend='isaac').notes)

    assert 'backend is unknown' not in notes


def test_metadata_dict_round_trip() -> None:
    """저장했다 읽어도 같아야 배치가 쓸 수 있다."""
    m = _built(backend='isaac')

    assert DatasetMetadata.from_dict(m.to_dict()) == m


def test_metadata_survives_json() -> None:
    import json
    m = _built(backend='isaac')

    assert DatasetMetadata.from_dict(json.loads(json.dumps(m.to_dict()))) == m
