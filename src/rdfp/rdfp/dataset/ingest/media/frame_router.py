"""이미지 메시지 → 현재 에피소드의 카메라 sink 로 라우팅한다.

에피소드 `enter` 이벤트에서 해당 에피소드용 디렉터리를 준비하고, 카메라별
sink 는 첫 프레임이 도착할 때 지연 생성된다. `exit` 에서는 프레임이 한 장
이상 기록된 모든 sink 를 finalize 한다.

지원 메시지 타입은 ``sensor_msgs/msg/Image`` 의 8-bit raw 인코딩
(``bgr8`` / ``rgb8`` / ``bgra8`` / ``rgba8`` / ``mono8``) 만이며, 그 외
(``CompressedImage``, ``16UC1``, ``mono16``, ``32FC1`` 등) 는 fail-fast 로
``UnsupportedImageError`` 를 발생시킨다.
"""

from __future__ import annotations

from typing import Any

import logging
import shutil
from pathlib import Path

import psycopg

from .ffmpeg_sink import FfmpegSink


_logger = logging.getLogger(__name__)

_IMAGE_TYPE: str = 'sensor_msgs/msg/Image'

# 8-bit raw 인코딩만 직통. recorder 의 SUPPORTED_ENCODINGS 와 동일.
_SUPPORTED_INPUT_ENCODINGS: frozenset[str] = frozenset({
    'bgr8', 'rgb8', 'bgra8', 'rgba8', 'mono8',
})


class UnsupportedImageError(RuntimeError):
    """지원하지 않는 이미지 메시지 형식 (fail-fast)."""


class FrameRouter:
    """에피소드×카메라 단위 mp4 sink 의 수명 주기를 관리한다."""

    def __init__(self, output_root: Path, *, conn: psycopg.Connection,
                 topic_ids: dict[str, int], fps: int = 30, codec: str = 'h264',
                 bitrate: str = '4M') -> None:
        """
        Args:
            output_root: mp4 파일이 적재될 출력 루트. 에피소드별 하위 디렉터리가
                생성된다.
            conn: 외부 주입 PostgreSQL 커넥션. sink 에 그대로 전달된다.
            topic_ids: ``{topic_name: topics.id}`` 매핑. ``on_image`` 에서
                토픽 이름으로 FK 를 lookup 하기 위해 필요.
            fps: mp4 컨테이너 CFR.
            codec: 메타에 기록할 비디오 코덱 라벨.
            bitrate: ffmpeg ``-b:v`` 값.
        """
        self._root = Path(output_root)
        self._conn = conn
        self._topic_ids = dict(topic_ids)
        self._fps = int(fps)
        self._codec = codec
        self._bitrate = bitrate
        self._episode_id: int | None = None
        self._episode_dir: Path | None = None
        self._sinks: dict[str, FfmpegSink] = {}

    # --- 에피소드 수명 주기 ---

    def open_episode(self, episode_id: int) -> None:
        """새 에피소드 진입. 대응되는 출력 디렉터리를 준비한다."""
        assert self._episode_id is None, 'open_episode called while another is active'
        self._episode_id = int(episode_id)
        self._episode_dir = self._root / f'episode_{episode_id:08d}'
        self._episode_dir.mkdir(parents=True, exist_ok=True)
        self._sinks = {}

    def finalize_episode(self) -> list[str]:
        """에피소드 sink 를 모두 닫고 생성된 mp4 상대 경로 목록을 반환한다.

        프레임이 한 장도 기록되지 않은 sink 는 finalize 되지만 파일을
        만들지 않는다 (설계서 9.5).
        """
        if self._episode_id is None:
            return []
        produced: list[str] = []
        for topic, sink in self._sinks.items():
            try:
                sink.finalize()
            except Exception:
                _logger.exception('ffmpeg finalize failed for topic=%s', topic)
                raise
            if sink.frame_count > 0:
                produced.append(str(
                    self._mp4_path(topic).relative_to(self._root)
                ))
        self._episode_id = None
        self._episode_dir = None
        self._sinks = {}
        return produced

    def abort_episode(self) -> None:
        """에피소드 실패 시: 열린 sink 를 abort 하고 에피소드 디렉터리를 제거한다."""
        if self._episode_id is None:
            return
        for sink in self._sinks.values():
            try:
                sink.abort()
            except Exception:
                _logger.exception('sink abort raised; continuing')
        if self._episode_dir is not None and self._episode_dir.exists():
            try:
                shutil.rmtree(self._episode_dir)
            except OSError:
                _logger.warning(
                    'could not remove episode video dir: %s', self._episode_dir,
                )
        self._episode_id = None
        self._episode_dir = None
        self._sinks = {}

    # --- 프레임 라우팅 ---

    def on_image(self, topic: str, type_name: str, msg: Any) -> None:
        """이미지 메시지를 현재 에피소드의 해당 카메라 sink 로 전달한다.

        Raises:
            UnsupportedImageError: 메시지 타입이 ``sensor_msgs/msg/Image`` 가
                아니거나, ``encoding`` 이 8-bit raw 가 아닌 경우.
            KeyError: ``topic`` 에 대한 ``topics.id`` 가 ``topic_ids`` 에
                등록되지 않은 경우.
        """
        if self._episode_id is None:
            return   # 에피소드 밖. 스킵.

        if type_name != _IMAGE_TYPE:
            raise UnsupportedImageError(
                f'unsupported image message type on topic {topic!r}: {type_name!r}; '
                f'only {_IMAGE_TYPE!r} is supported (CompressedImage / depth not supported)'
            )
        encoding = str(getattr(msg, 'encoding', '')).lower()
        if encoding not in _SUPPORTED_INPUT_ENCODINGS:
            raise UnsupportedImageError(
                f'unsupported image encoding on topic {topic!r}: {encoding!r}; '
                f'expected one of {sorted(_SUPPORTED_INPUT_ENCODINGS)} '
                f'(16UC1/mono16/32FC1 not supported)'
            )

        sink = self._sinks.get(topic)
        if sink is None:
            sink = FfmpegSink(
                mp4_path=self._mp4_path(topic),
                conn=self._conn,
                episode_id=self._episode_id,
                topic_id=self._topic_ids[topic],
                mp4_root=self._root,
                fps=self._fps,
                codec=self._codec,
                bitrate=self._bitrate,
            )
            self._sinks[topic] = sink
        sink.write(msg)

    # --- 경로 헬퍼 ---

    def remove_existing_episode_dir(self, episode_id: int) -> None:
        """`replace` 정책에서 기존 에피소드 mp4 디렉터리를 제거한다."""
        target = self._root / f'episode_{episode_id:08d}'
        if target.exists():
            shutil.rmtree(target)

    def _mp4_path(self, topic: str) -> Path:
        assert self._episode_dir is not None
        return self._episode_dir / (_sanitize_topic(topic) + '.mp4')


def _sanitize_topic(topic: str) -> str:
    """토픽명의 `/` 를 `_` 로 치환하고 선행 `_` 를 제거한다."""
    sanitized = topic.replace('/', '_')
    return sanitized.lstrip('_') or 'unnamed_topic'


__all__ = ['FrameRouter', 'UnsupportedImageError']
