"""`_delete_used_splits` 단위 테스트."""

from __future__ import annotations

from pathlib import Path

from rdfp.rosbag.catalog import Split
from rdfp.dataset.ingest.pipeline import _delete_used_splits


def _make_split_tree(tmp_path: Path, session: str, split_files: list[str]) -> list[Split]:
    session_dir = tmp_path / '2026-04-11' / session
    session_dir.mkdir(parents=True)
    (session_dir / 'metadata.yaml').write_text('rosbag2_bagfile_information: {}\n')

    splits: list[Split] = []
    for i, name in enumerate(split_files):
        p = session_dir / name
        p.write_bytes(b'fake mcap bytes')
        splits.append(Split(
            path=p,
            session_path=session_dir,
            session_name=session_dir.name,
            date_dir='2026-04-11',
            split_index=i,
            start_ns=0, end_ns=1, message_count=0,
            topics=frozenset(),
        ))
    return splits


def test_deletes_splits_and_empty_session_dir(tmp_path) -> None:
    splits = _make_split_tree(
        tmp_path, 'session_2026-04-11_09-00-00', ['bag_0.mcap', 'bag_1.mcap'],
    )
    session_dir = Path(splits[0].session_path)
    deleted = _delete_used_splits(splits)

    assert deleted == 2
    # metadata.yaml 만 남았으므로 세션 디렉터리가 통째로 제거되어야 한다.
    assert not session_dir.exists()


def test_preserves_session_dir_when_extra_files_remain(tmp_path) -> None:
    splits = _make_split_tree(
        tmp_path, 'session_2026-04-11_10-00-00', ['bag_0.mcap'],
    )
    session_dir = Path(splits[0].session_path)
    # 의도치 않은 파일이 남아 있다면 삭제하지 않아야 한다.
    (session_dir / 'notes.txt').write_text('keep me')

    deleted = _delete_used_splits(splits)

    assert deleted == 1
    assert session_dir.exists()
    assert (session_dir / 'notes.txt').is_file()
    assert (session_dir / 'metadata.yaml').is_file()


def test_missing_split_file_does_not_fail(tmp_path) -> None:
    splits = _make_split_tree(
        tmp_path, 'session_2026-04-11_11-00-00', ['bag_0.mcap'],
    )
    # 사전 삭제: 이미 없는 파일이어도 전체 동작이 실패하지 않아야 한다.
    splits[0].path.unlink()
    session_dir = Path(splits[0].session_path)

    deleted = _delete_used_splits(splits)

    assert deleted == 0
    # metadata.yaml 만 남았으므로 디렉터리는 정리된다.
    assert not session_dir.exists()
