"""catalog 모듈 테스트."""

from __future__ import annotations

from rdfp.rosbag.catalog import discover_splits
from rdfp.rosbag.tests.fixtures.make_synth_bag import (
    SessionEventSpec, write_synth_bag,
)


S0 = 1_700_000_000_000_000_000     # 2023-11-14 기준 epoch ns


def _min_events(base_ns: int) -> list[SessionEventSpec]:
    # 최소한의 메시지 (bag 의 start/end 결정 목적).
    return [
        SessionEventSpec(base_ns + 0,              'IDLE', ''),
        SessionEventSpec(base_ns + 1_000_000_000,  'IDLE', ''),
    ]


def test_scan_empty_dir(tmp_path) -> None:
    (tmp_path / 'empty').mkdir()
    assert discover_splits(tmp_path / 'empty') == []


def test_scan_finds_single_split(tmp_path) -> None:
    write_synth_bag(tmp_path, session_events=_min_events(S0))
    splits = discover_splits(tmp_path)
    assert len(splits) == 1
    assert splits[0].session_name.startswith('session_')
    assert '/session' in splits[0].topics


def test_date_filter(tmp_path) -> None:
    write_synth_bag(
        tmp_path, date_dir='2026-04-11',
        session_name='session_2026-04-11_09-00-00',
        session_events=_min_events(S0),
    )
    write_synth_bag(
        tmp_path, date_dir='2026-04-12',
        session_name='session_2026-04-12_09-00-00',
        session_events=_min_events(S0 + 10_000_000_000),
    )
    # 모든 날짜
    all_splits = discover_splits(tmp_path)
    assert len(all_splits) == 2
    # 한 날짜만
    filt = discover_splits(tmp_path, dates=['2026-04-11'])
    assert len(filt) == 1
    assert filt[0].date_dir == '2026-04-11'


def test_missing_metadata_is_skipped(tmp_path) -> None:
    # 세션 디렉터리는 있지만 metadata.yaml 이 없는 경우 스킵된다 (녹화 중이거나
    # 비정상 종료된 세션으로 간주).
    (tmp_path / '2026-04-11' / 'session_2026-04-11_09-00-00').mkdir(parents=True)
    assert discover_splits(tmp_path) == []
