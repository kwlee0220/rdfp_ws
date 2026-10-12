"""에피소드 선택 · 적재 여부 판정 · 삭제 단위 테스트.

합성 MCAP 으로 실제 파일을 만들어 돌린다 — 삭제는 split 을 다시 쓰는 동작이라 모킹으로는
검증되지 않는다. ROS 는 필요 없다 (`mcap_ros2` 만 쓴다).
"""

from __future__ import annotations

from pathlib import Path

import pytest

from rdfp.rosbag.catalog import discover_splits
from rdfp.rosbag.episode_ops import (
    EXPORT_NO, EXPORT_UNKNOWN, EXPORT_YES,
    build_views, delete_episodes, load_episodes, parse_index_spec, select,
)
from rdfp.rosbag.tests.fixtures.make_synth_bag import (
    PoseEventSpec, SessionEventSpec, write_synth_bag,
)

BASE_NS = 1_757_000_000_000_000_000
SEC = 1_000_000_000


def _bag(root: Path, labels: tuple[str, ...] = ('a', 'b', 'c')) -> Path:
    """에피소드 하나당 12초, 200초 간격으로 `labels` 개수만큼 만든다."""
    session, poses = [], []
    for i, label in enumerate(labels):
        t0 = BASE_NS + i * 200 * SEC
        session += [SessionEventSpec(t0, 'IN_EPISODE', label),
                    SessionEventSpec(t0 + 12 * SEC, 'IN_SESSION', label)]
        poses += [PoseEventSpec(t0 + k * SEC) for k in range(1, 12)]
    return write_synth_bag(root, session_events=session, pose_events=poses)


def _start(i: int) -> int:
    """i 번째(0-base) 에피소드의 자연키."""
    return BASE_NS + i * 200 * SEC


# ----- 목록 ------------------------------------------------------------------


def test_episodes_are_ordered_and_carry_origin(tmp_path) -> None:
    _bag(tmp_path)

    eps = load_episodes(tmp_path)

    assert [e.task_label for e in eps] == ['a', 'b', 'c']
    assert [e.start_ns for e in eps] == [_start(0), _start(1), _start(2)]
    # log time 이 채워져야 목록 정렬과 `--since/--until` 이 성립한다.
    assert all(e.log_start_ns > 0 for e in eps)
    assert all(e.session_name.startswith('session_') for e in eps)
    assert all(e.split_paths for e in eps)


def test_export_status_is_three_valued(tmp_path) -> None:
    """DB 를 보지 않은 것과 '적재 안 됨' 은 반드시 구분된다."""
    _bag(tmp_path)
    eps = load_episodes(tmp_path)

    unknown = build_views(eps, exported_keys=None)
    known = build_views(eps, exported_keys={_start(1)})

    assert {v.exported for v in unknown} == {EXPORT_UNKNOWN}
    assert [v.exported for v in known] == [EXPORT_NO, EXPORT_YES, EXPORT_NO]


# ----- 선택 ------------------------------------------------------------------


def test_select_by_task(tmp_path) -> None:
    _bag(tmp_path, labels=('peg', 'tray', 'peg'))
    views = build_views(load_episodes(tmp_path), exported_keys=None)

    out = select(views, task='peg')

    assert [v.index for v in out] == [1, 3]


def test_select_time_range_includes_straddling_episode(tmp_path) -> None:
    """구간에 걸치기만 해도 포함된다 — 반쪽만 다루면 데이터가 조용히 잘린다."""
    _bag(tmp_path)
    views = build_views(load_episodes(tmp_path), exported_keys=None)

    # 2번 에피소드의 중간 한 점만 지정한다.
    mid = _start(1) + 6 * SEC
    out = select(views, since_ns=mid, until_ns=mid)

    assert [v.index for v in out] == [2]


def test_select_only_new_and_only_exported(tmp_path) -> None:
    _bag(tmp_path)
    views = build_views(load_episodes(tmp_path), exported_keys={_start(0), _start(2)})

    assert [v.index for v in select(views, only_new=True)] == [2]
    assert [v.index for v in select(views, only_exported=True)] == [1, 3]


def test_select_refuses_export_state_when_unknown(tmp_path) -> None:
    """적재 여부를 모르는 채로 상태 기반 선택을 허용하면 오판으로 지우거나 재적재한다."""
    _bag(tmp_path)
    views = build_views(load_episodes(tmp_path), exported_keys=None)

    with pytest.raises(ValueError, match='unknown'):
        select(views, only_exported=True)


def test_select_rejects_contradictory_flags(tmp_path) -> None:
    _bag(tmp_path)
    views = build_views(load_episodes(tmp_path), exported_keys=set())

    with pytest.raises(ValueError, match='mutually exclusive'):
        select(views, only_new=True, only_exported=True)


def test_parse_index_spec(tmp_path) -> None:
    _bag(tmp_path)
    views = build_views(load_episodes(tmp_path), exported_keys=None)

    assert [v.index for v in parse_index_spec('1,3', views)] == [1, 3]
    assert [v.index for v in parse_index_spec('2-3', views)] == [2, 3]
    # 중복은 한 번만, 순서는 항상 오름차순이다.
    assert [v.index for v in parse_index_spec('3,1,1', views)] == [1, 3]
    with pytest.raises(ValueError, match='no episode with index'):
        parse_index_spec('9', views)
    with pytest.raises(ValueError, match='invalid range'):
        parse_index_spec('3-1', views)


# ----- 삭제 ------------------------------------------------------------------


def test_delete_removes_only_the_selected_episode(tmp_path) -> None:
    _bag(tmp_path)
    views = build_views(load_episodes(tmp_path), exported_keys=None)
    target = [v for v in views if v.index == 2]

    reports, gone = delete_episodes(tmp_path, target)

    assert gone == [_start(1)]
    assert sum(r.removed for r in reports) == 13   # /session 2 + pose 11
    assert [e.task_label for e in load_episodes(tmp_path)] == ['a', 'c']


def test_delete_updates_session_metadata(tmp_path) -> None:
    """`discover_splits` 가 `metadata.yaml` 을 읽으므로 개수를 다시 계산해야 한다."""
    _bag(tmp_path)
    before = sum(s.message_count for s in discover_splits(tmp_path))
    views = build_views(load_episodes(tmp_path), exported_keys=None)

    delete_episodes(tmp_path, [v for v in views if v.index == 1])

    after = sum(s.message_count for s in discover_splits(tmp_path))
    assert after == before - 13


def test_delete_preserves_other_topics_of_remaining_episodes(tmp_path) -> None:
    """구간 밖 메시지는 한 건도 잃지 않는다."""
    from rdfp.rosbag.mcap_reader import iter_split_messages

    _bag(tmp_path)
    views = build_views(load_episodes(tmp_path), exported_keys=None)
    kept_range = (_start(2), _start(2) + 12 * SEC)

    delete_episodes(tmp_path, [v for v in views if v.index == 1])

    split = discover_splits(tmp_path)[0]
    poses = list(iter_split_messages(split.path,
                                     topics=['/ee_pose_publisher/ee_pose'],
                                     time_range_ns=kept_range))
    assert len(poses) == 11


def test_delete_nothing_is_a_noop(tmp_path) -> None:
    _bag(tmp_path)

    reports, gone = delete_episodes(tmp_path, [])

    assert reports == [] and gone == []
    assert len(load_episodes(tmp_path)) == 3
