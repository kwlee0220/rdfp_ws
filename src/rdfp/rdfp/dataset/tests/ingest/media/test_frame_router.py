"""media.frame_router 단위 테스트.

ffmpeg 인코딩 / DB 적재는 통합 테스트에서 다룬다. 여기서는 라우팅 동작과
fail-fast 정책, 에피소드 수명 주기만 검증한다.
"""

from __future__ import annotations

import numpy as np
import pytest

from rdfp.dataset.ingest.media.frame_router import FrameRouter, UnsupportedImageError


class _StubConn:
    """psycopg.Connection 의 자리만 채우는 stub (sink 가 직접 사용하지 않는 경로용)."""


def test_sanitize_topic_in_paths(tmp_path) -> None:
    router = FrameRouter(tmp_path, conn=_StubConn(), topic_ids={})
    router.open_episode(7)
    # sink 생성은 on_image 에서 이루어지므로, 간접 검증: finalize_episode()
    # 호출 후 디렉터리만 존재하는지 확인.
    router.finalize_episode()
    assert (tmp_path / 'episode_00000007').is_dir()


def test_remove_existing_episode_dir(tmp_path) -> None:
    target = tmp_path / 'episode_00000042'
    target.mkdir()
    (target / 'dummy.mp4').write_bytes(b'x')
    router = FrameRouter(tmp_path, conn=_StubConn(), topic_ids={})
    router.remove_existing_episode_dir(42)
    assert not target.exists()


def test_on_image_routes_to_ffmpeg_sink(tmp_path, monkeypatch) -> None:
    """FfmpegSink 를 stub 으로 교체해 라우팅 동작만 검증한다."""
    from rdfp.dataset.ingest.media import frame_router as fr_mod

    created: list[dict] = []
    written: list[object] = []
    finalized: list[str] = []

    class _StubSink:
        def __init__(self, mp4_path, **kwargs) -> None:
            created.append({'mp4': mp4_path, **kwargs})
            self._mp4 = mp4_path
            self.frame_count = 0

        def write(self, image) -> None:
            written.append(image)
            self.frame_count += 1

        def finalize(self) -> None:
            finalized.append(str(self._mp4))

        def abort(self) -> None:
            pass

    monkeypatch.setattr(fr_mod, 'FfmpegSink', _StubSink)

    router = FrameRouter(
        tmp_path, conn=_StubConn(),
        topic_ids={'/cam/raw': 42}, fps=30,
    )
    router.open_episode(1)

    msg = _make_image_msg(4, 4, (10, 20, 30))
    router.on_image('/cam/raw', 'sensor_msgs/msg/Image', msg)
    router.on_image('/cam/raw', 'sensor_msgs/msg/Image', msg)

    produced = router.finalize_episode()
    assert len(created) == 1
    assert created[0]['topic_id'] == 42
    assert created[0]['episode_id'] == 1
    assert len(written) == 2
    assert len(finalized) == 1
    # 프레임이 기록되었으므로 produced 목록에 포함된다.
    assert produced != []
    assert produced[0].endswith('cam_raw.mp4')


def _make_image_msg(width: int, height: int, bgr: tuple[int, int, int]):
    from types import SimpleNamespace

    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, :] = bgr
    return SimpleNamespace(
        height=height, width=width, encoding='bgr8',
        is_bigendian=0, step=width * 3, data=frame.tobytes(),
        header=SimpleNamespace(frame_id='cam_link'),
    )


def test_image_outside_episode_is_ignored(tmp_path, monkeypatch) -> None:
    from rdfp.dataset.ingest.media import frame_router as fr_mod
    calls = []
    monkeypatch.setattr(fr_mod, 'FfmpegSink', lambda *a, **k: calls.append((a, k)))

    router = FrameRouter(tmp_path, conn=_StubConn(), topic_ids={'/cam/raw': 1})
    # open_episode 호출 없이 on_image → sink 생성되지 않아야 한다.
    msg = _make_image_msg(2, 2, (0, 0, 0))
    router.on_image('/cam/raw', 'sensor_msgs/msg/Image', msg)
    assert calls == []


def test_compressed_image_raises(tmp_path) -> None:
    router = FrameRouter(tmp_path, conn=_StubConn(), topic_ids={'/cam/jpg': 1})
    router.open_episode(1)
    from types import SimpleNamespace
    msg = SimpleNamespace(data=b'\x00' * 8, format='jpeg',
                          header=SimpleNamespace(frame_id=''))
    with pytest.raises(UnsupportedImageError, match='unsupported image message type'):
        router.on_image('/cam/jpg', 'sensor_msgs/msg/CompressedImage', msg)


def test_unsupported_encoding_raises(tmp_path) -> None:
    router = FrameRouter(tmp_path, conn=_StubConn(), topic_ids={'/cam/depth': 1})
    router.open_episode(1)
    msg = _make_image_msg(2, 2, (0, 0, 0))
    msg.encoding = '16uc1'
    with pytest.raises(UnsupportedImageError, match='unsupported image encoding'):
        router.on_image('/cam/depth', 'sensor_msgs/msg/Image', msg)
