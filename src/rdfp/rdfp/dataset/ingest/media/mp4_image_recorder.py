from __future__ import annotations

from typing import Optional

import logging

import psycopg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from rdfp.dataset.db.writers.image_frame import ImageFrameWriter
from rdfp.recorder.ffmpeg_mp4_recorder import FFMpegMp4Recorder
from rdfp.types import InvalidFrameError


_DEFAULT_BATCH_SIZE: int = 100


class Mp4ImageRecorder:
    """``sensor_msgs/Image`` 를 받아 MP4 와 DBMS sidecar 테이블로 저장하는 녹화기.

    내부적으로 ``FFMpegMp4Recorder`` 로 영상을 인코딩하며, 매 프레임마다
    ``image_frames`` 테이블에 ``(episode_id, topic_id, frame_index, stamp)``
    한 행을 적재한다.

    트랜잭션 정책:
        본 클래스는 커넥션을 ``__init__`` 에서 외부로부터 주입받으며 commit /
        rollback 을 수행하지 않는다. 호출자가 트랜잭션을 관리해야 한다 (보통
        episode 경계에서 commit). ``WriterBase`` 의 배치 INSERT 를 사용하므로
        ``stop()`` 시점에 누적된 미flush 행을 일괄 INSERT 한다.

    1:1 매치 불변식:
        mp4 프레임 ↔ DB 행의 1:1 매치를 유지하기 위해, ``FFMpegMp4Recorder.write*()``
        가 성공한 경우에만 writer 에 행을 추가하고 ``frame_index`` 를 증가시킨다.
        DB INSERT 오류는 그대로 전파한다 (PostgreSQL 은 오류 발생 트랜잭션
        에서 후속 statement 를 모두 거부하므로 swallow 가 위험하다).
    """

    def __init__(self, recorder: FFMpegMp4Recorder, *,
                 conn: psycopg.Connection, episode_id: int, topic_id: int,
                 table: Optional[str] = None,
                 batch_size: int = _DEFAULT_BATCH_SIZE,
                 logger: Optional[logging.Logger] = None) -> None:
        """
        Args:
            recorder: 영상 인코딩을 담당할 ``FFMpegMp4Recorder`` 인스턴스.
            conn: 외부에서 주입된 PostgreSQL 커넥션. 본 클래스는 commit /
                rollback 을 수행하지 않는다.
            episode_id: 현재 episode (sessions.id) 의 FK 값.
            topic_id: 녹화 대상 토픽 (topics.id) 의 FK 값.
            table: 적재 대상 테이블 이름. 생략 시 ``image_frames`` 사용.
            batch_size: writer 의 배치 INSERT 임계 (기본 100). mp4 프레임율을
                고려하여 너무 크게 잡으면 오류 시 손실 구간이 길어진다.
            logger: 변환 / sidecar 관련 로그를 출력할 logger. 생략 시 모듈 로거.
        """
        self._recorder = recorder
        self._bridge = CvBridge()
        # FFMpegMp4Recorder 가 사용한 pixel_format 으로 cv_bridge 변환을 수행한다
        # (동일 패키지 내부 결합).
        self._pixel_format: str = recorder.pixel_format
        self._logger = logger or logging.getLogger(f"{__name__}.{self.__class__.__name__}")

        self._episode_id: int = episode_id
        self._writer: ImageFrameWriter = ImageFrameWriter(
            conn=conn, batch_size=batch_size, table=table, topic_id=topic_id
        )
        self._frame_index: int = 0

    def start(self, output_path: str) -> None:
        """녹화를 시작한다.

        ``FFMpegMp4Recorder.start()`` 가 성공한 직후에 frame_index 를 0 으로
        리셋하고 writer 의 미flush 버퍼를 비운다 (이전 stop() 에서 flush 되지
        않은 잔존 행 방어). recorder 가 예외를 발생시키면 writer 상태는
        변경되지 않는다.

        Args:
            output_path: 출력 MP4 파일 경로.
        """
        self._recorder.start(output_path)
        self._frame_index = 0
        # 이전 episode 의 미flush 행이 남아 있을 수 있으므로 안전하게 폐기한다.
        # 호출자가 이전 트랜잭션을 commit / rollback 한 시점이라고 가정한다.
        self._writer.drop_pending()

    def write(self, image: Image) -> None:
        """``sensor_msgs/Image`` 한 장을 녹화하고 DBMS sidecar 에 한 행을 추가한다.

        ``image.header.stamp`` 를 그대로 stamp 로 사용한다.

        Fast path: ``image.encoding`` 이 recorder 의 설정 인코딩과 동일하면
        cv_bridge / ndarray 라운드트립을 건너뛰고 ``image.data`` 를 그대로
        ``recorder.write_bytes`` 로 전달한다. 풀프레임 복사 한 번을 절약한다.

        Slow path: 인코딩이 다르면 cv_bridge 로 색공간 변환을 수행한다.
        cv_bridge 변환 실패 시 ``InvalidFrameError`` 를 발생시키며, recorder
        측 오류는 그대로 전파한다.

        DB 측 오류 (``psycopg.Error``) 도 그대로 전파되며, 호출자는 트랜잭션
        rollback 후 적절히 대응해야 한다 (PostgreSQL 은 오류 발생 트랜잭션
        에서 후속 statement 를 모두 거부하므로 swallow 가 위험하다).
        """
        if image.encoding == self._pixel_format:
            # Fast path — 변환 없이 raw bytes 직접 enqueue
            self._recorder.write_bytes(image.data)
        else:
            # Slow path — 색공간 변환 필요
            try:
                frame = self._bridge.imgmsg_to_cv2(image, desired_encoding=self._pixel_format)
            except Exception as exc:
                raise InvalidFrameError(
                    f"failed to convert sensor_msgs/Image (encoding={image.encoding!r}) to numpy "
                    f"with desired_encoding={self._pixel_format!r}: {exc}"
                ) from exc
            self._recorder.write(frame)

        # recorder.write* 가 성공한 경우에만 sidecar 적재 + frame_index 증가 (1:1 매치)
        self._writer.append_frame(self._episode_id, image, self._frame_index)
        self._frame_index += 1

    def stop(self, timeout: float) -> str:
        """녹화를 종료하고 미flush 행을 INSERT 한 뒤 MP4 출력 경로를 반환한다.

        ``FFMpegMp4Recorder.stop()`` 의 성공 여부와 무관하게 writer flush 를
        시도한다. flush 오류는 로그 후 전파한다 (호출자가 트랜잭션을 정리해야
        한다).
        """
        try:
            return self._recorder.stop(timeout)
        finally:
            try:
                self._writer.flush()
            except Exception as exc:
                self._logger.error(f'image_frames flush failed: {exc}')
                raise

    def abort(self) -> None:
        """미flush 행을 폐기한다 (호출자가 트랜잭션 rollback 시 사용).

        recorder 자체는 종료하지 않는다. 호출자가 ``recorder.shutdown()`` 등을
        별도로 처리해야 한다.
        """
        self._writer.drop_pending()

    @property
    def frame_index(self) -> int:
        """다음에 부여될 frame_index (= 지금까지 적재된 행 수)."""
        return self._frame_index
