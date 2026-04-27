"""`(episode_id, camera_topic)` 단위 mp4 + DBMS sidecar 작성기.

`FFMpegMp4Recorder` 는 해상도 / pixel_format 을 생성자에서 고정하므로, 본
sink 는 첫 프레임이 도착하는 시점에 두 값을 결정해 지연 `start()` 한다.
첫 프레임의 ``encoding`` 으로 recorder 를 구성하기 때문에 동일 인코딩이
유지되는 한 ``Mp4ImageRecorder`` 의 fast path (raw bytes 직통) 가 매번
발동한다.

저장 위치:
    * 영상 → mp4 파일 (디스크).
    * 프레임별 stamp → ``image_frames`` 테이블 (Mp4ImageRecorder).
    * 글로벌 메타 (mp4_path, codec, fps, frame_id, 해상도, frame_count) →
      ``image_streams`` 테이블 (finalize 시 한 행 INSERT).

트랜잭션:
    호출자가 conn 을 주입하고 commit / rollback 을 관리한다. 본 sink 와
    그 내부 ``Mp4ImageRecorder`` 는 어느 쪽도 commit 을 하지 않는다.
"""

from __future__ import annotations

import logging
from pathlib import Path

import psycopg
from sensor_msgs.msg import Image

from rdfp.dataset.db.writers.image_stream import ImageStreamWriter
from rdfp.recorder.ffmpeg_mp4_recorder import FFMpegMp4Recorder
from .mp4_image_recorder import Mp4ImageRecorder
from rdfp.types import Resolution


_logger = logging.getLogger(__name__)
_DEFAULT_STOP_TIMEOUT_SEC: float = 5.0


class FfmpegSink:
    """에피소드×카메라 하나에 대응되는 mp4 + DBMS sidecar 작성기."""

    def __init__(self, mp4_path: Path, *, conn: psycopg.Connection, episode_id: int,
                 topic_id: int, mp4_root: Path, fps: int, codec: str = 'h264',
                 bitrate: str = '4M') -> None:
        """
        Args:
            mp4_path: mp4 파일을 쓸 절대/상대 경로.
            conn: 외부 주입 PostgreSQL 커넥션. 본 sink 는 commit / rollback 을
                수행하지 않는다.
            episode_id: ``sessions.id`` FK.
            topic_id: ``topics.id`` FK.
            mp4_root: 출력 루트 디렉터리. ``image_streams.mp4_path`` 를 이 루트
                기준 상대경로로 저장하기 위해 사용한다.
            fps: mp4 컨테이너 CFR.
            codec: 메타에 기록할 비디오 코덱 라벨 (예: ``'h264'``).
                현재 ffmpeg 는 ``encoder_mode='cpu'`` (libx264) 로 고정.
            bitrate: ffmpeg ``-b:v`` 값.
        """
        self._mp4_path = Path(mp4_path)
        self._mp4_root = Path(mp4_root)
        self._conn = conn
        self._episode_id = int(episode_id)
        self._topic_id = int(topic_id)
        self._fps = int(fps)
        self._codec = codec
        self._bitrate = bitrate
        # 첫 프레임 시점에 결정되는 값들.
        self._ffmpeg: FFMpegMp4Recorder | None = None
        self._recorder: Mp4ImageRecorder | None = None
        self._pixel_format: str | None = None
        self._frame_id: str = ''
        self._resolution: Resolution | None = None
        self._frame_count: int = 0
        self._closed: bool = False

    @property
    def frame_count(self) -> int:
        return self._frame_count

    def write(self, image: Image) -> None:
        """``sensor_msgs/Image`` 한 장을 인코더에 넣고 sidecar 행을 적재한다.

        첫 호출에서 lazy-start 하여 ffmpeg 와 ``Mp4ImageRecorder`` 를
        구성한다. 이후 호출은 단순 위임.
        """
        if self._closed:
            raise RuntimeError('sink already closed')

        if self._recorder is None:
            self._lazy_start(image)
        assert self._recorder is not None

        self._recorder.write(image)
        self._frame_count += 1

    def finalize(self) -> None:
        """mp4 를 닫고 image_streams 행을 INSERT 한다.

        프레임이 한 장도 들어오지 않았으면 mp4 / DB 행 모두 만들지 않고 조용히
        종료한다 (sink 자체가 lazy-start 라 ``self._recorder`` 가 None).
        """
        if self._closed:
            return
        self._closed = True
        if self._recorder is None:
            return
        try:
            self._recorder.stop(_DEFAULT_STOP_TIMEOUT_SEC)
        finally:
            assert self._ffmpeg is not None
            self._ffmpeg.shutdown()

        # image_streams 글로벌 메타 INSERT. mp4_path 는 출력 루트 기준 상대경로.
        try:
            mp4_rel = str(self._mp4_path.relative_to(self._mp4_root))
        except ValueError:
            mp4_rel = str(self._mp4_path)
        assert self._resolution is not None and self._pixel_format is not None
        ImageStreamWriter(self._conn).insert(
            episode_id=self._episode_id, topic_id=self._topic_id,
            mp4_path=mp4_rel, codec=self._codec,
            pixel_format=self._pixel_format, container_fps=self._fps,
            frame_id=self._frame_id,
            width=self._resolution.width, height=self._resolution.height,
            frame_count=self._frame_count,
        )

    def abort(self) -> None:
        """트랜잭션 실패 시: 인코더 종료 + 미flush 행 폐기 + mp4 파일 제거.

        DB 트랜잭션 자체의 rollback 은 호출자(pipeline) 가 수행한다. 본 메서드는
        sink 에 누적된 미flush 버퍼만 폐기한다.
        """
        if self._closed:
            return
        self._closed = True
        if self._recorder is not None:
            self._recorder.abort()
            try:
                # stop 은 mp4 finalize 시도지만 실패할 수 있으므로 예외 무시.
                self._recorder.stop(_DEFAULT_STOP_TIMEOUT_SEC)
            except Exception:
                _logger.exception('ffmpeg stop failed during abort')
            try:
                assert self._ffmpeg is not None
                self._ffmpeg.shutdown()
            except Exception:
                pass
            self._recorder = None
            self._ffmpeg = None
        if self._mp4_path.exists():
            try:
                self._mp4_path.unlink()
            except OSError:
                _logger.warning('could not delete partial mp4: %s', self._mp4_path)

    # ---------- 내부 helper -------------------------------------------------

    def _lazy_start(self, image: Image) -> None:
        """첫 프레임의 해상도 / encoding / frame_id 로 recorder 를 구성한다.

        ffmpeg 의 ``pixel_format`` 을 첫 프레임 ``image.encoding`` 으로 잡아
        fast path 가 항상 발동하도록 한다. 동일 토픽에서 인코딩이 바뀌지
        않는다는 가정. 바뀌더라도 cv_bridge slow path 가 흡수한다 (해상도가
        같다는 가정 하에).
        """
        width = int(image.width)
        height = int(image.height)
        if width <= 0 or height <= 0:
            raise ValueError(
                f'invalid image resolution from first frame: {width}x{height}'
            )
        self._resolution = Resolution(width, height)
        self._pixel_format = str(image.encoding)
        self._frame_id = str(getattr(getattr(image, 'header', None), 'frame_id', '') or '')

        self._mp4_path.parent.mkdir(parents=True, exist_ok=True)
        # encoder_mode='cpu' 로 고정해 개발 환경 의존성을 최소화한다.
        self._ffmpeg = FFMpegMp4Recorder(
            fps=self._fps, resolution=self._resolution,
            pixel_format=self._pixel_format, encoder_mode='cpu',
            bitrate=self._bitrate,
        )
        self._recorder = Mp4ImageRecorder(
            self._ffmpeg, conn=self._conn,
            episode_id=self._episode_id, topic_id=self._topic_id,
        )
        self._recorder.start(str(self._mp4_path))


__all__ = ['FfmpegSink']
