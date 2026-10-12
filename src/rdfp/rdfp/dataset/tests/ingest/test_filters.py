"""에피소드 필터 로직 단위 테스트 (pipeline._apply_episode_filters).

필터 자체는 순수 Python 이지만 `pipeline` 이 `sensor_msgs` 를 전이 의존하므로,
ROS sourcing 이 안 된 환경에서는 전체 테스트를 skip 한다.
"""

from __future__ import annotations

import pytest

pytest.importorskip('sensor_msgs', reason='requires ROS 2 runtime (transitive dependency)')

from rdfp.dataset.config import EpisodeFilter                        # noqa: E402
from rdfp.dataset.ingest.episode.detector import Episode             # noqa: E402
from rdfp.dataset.ingest.pipeline import _apply_episode_filters      # noqa: E402


def _ep(start: int, stop: int, label: str | None = None) -> Episode:
    return Episode(start_ns=start, stop_ns=stop, task_label=label)


def test_no_filter_keeps_all() -> None:
    eps = [_ep(0, 10, 'a'), _ep(20, 30, 'b')]
    assert _apply_episode_filters(eps, EpisodeFilter()) == eps


def test_min_duration_filters_short_episodes() -> None:
    eps = [_ep(0, 1_000_000_000, 'x'), _ep(2_000_000_000, 4_000_000_000, 'y')]
    out = _apply_episode_filters(eps, EpisodeFilter(min_duration_sec=1.5))
    assert [(e.start_ns, e.task_label) for e in out] == [(2_000_000_000, 'y')]


def test_include_filter_whitelists_labels() -> None:
    eps = [_ep(0, 10, 'pick'), _ep(20, 30, 'pour'), _ep(40, 50, 'other')]
    out = _apply_episode_filters(eps, EpisodeFilter(task_labels_include=['pick', 'pour']))
    assert [e.task_label for e in out] == ['pick', 'pour']


def test_exclude_filter_removes_labels() -> None:
    eps = [_ep(0, 10, 'pick'), _ep(20, 30, 'outlier'), _ep(40, 50, 'pour')]
    out = _apply_episode_filters(eps, EpisodeFilter(task_labels_exclude=['outlier']))
    assert [e.task_label for e in out] == ['pick', 'pour']


def test_include_and_exclude_combined() -> None:
    # include 가 먼저 적용되고, include 안의 항목에서 exclude 가 제거한다 (AND).
    eps = [_ep(0, 10, 'a'), _ep(20, 30, 'b'), _ep(40, 50, 'c')]
    out = _apply_episode_filters(
        eps, EpisodeFilter(task_labels_include=['a', 'b'], task_labels_exclude=['b']),
    )
    assert [e.task_label for e in out] == ['a']


def test_empty_task_label_treated_as_empty_string() -> None:
    # task_label 이 None 인 에피소드는 빈 문자열로 취급한다.
    eps = [_ep(0, 10, None)]
    assert _apply_episode_filters(eps, EpisodeFilter(task_labels_include=[''])) == eps


# ----- start_ns_in — `rosbag export` 가 선택 결과를 넘기는 경로 --------------


def test_start_ns_in_selects_only_listed_keys() -> None:
    """자연키로 못박으면 그 에피소드만 남는다."""
    eps = [_ep(100, 200), _ep(300, 400), _ep(500, 600)]
    filt = EpisodeFilter(start_ns_in=[300, 500])

    out = _apply_episode_filters(eps, filt)

    assert [e.start_ns for e in out] == [300, 500]


def test_start_ns_in_empty_means_no_restriction() -> None:
    """비어 있으면 제한하지 않는다 — 기존 `import` 동작이 그대로 유지된다."""
    eps = [_ep(100, 200), _ep(300, 400)]

    out = _apply_episode_filters(eps, EpisodeFilter())

    assert len(out) == 2


def test_start_ns_in_combines_with_other_filters() -> None:
    """자연키가 가장 강한 조건이지만 나머지 필터도 함께 적용된다.

    키로 골라도 `min_duration_sec` 에 걸리면 빠진다 — 그러지 않으면 품질 기준을
    우회하는 경로가 생긴다.
    """
    eps = [_ep(0, 1_000_000_000, 'peg'), _ep(10_000_000_000, 10_100_000_000, 'peg')]
    filt = EpisodeFilter(start_ns_in=[0, 10_000_000_000], min_duration_sec=0.5)

    out = _apply_episode_filters(eps, filt)

    assert [e.start_ns for e in out] == [0]
