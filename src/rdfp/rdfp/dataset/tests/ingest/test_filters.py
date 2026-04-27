"""에피소드 필터 로직 단위 테스트 (pipeline._apply_episode_filters)."""

from __future__ import annotations

from rdfp.dataset.config import EpisodeFilter
from rdfp.dataset.ingest.episode.detector import Episode
from rdfp.dataset.ingest.pipeline import _apply_episode_filters


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
