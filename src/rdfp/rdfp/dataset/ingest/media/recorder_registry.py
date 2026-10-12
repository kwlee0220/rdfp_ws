"""레코더가 남긴 mp4 를 `image_streams` · `image_frames` 에 등록한다 (I4).

**영상의 원본은 레코더 mp4 다** — rosbag 에는 영상을 담지 않는다(좌표 공간 문서
권고 12). 적재는 rosbag 에서 이미지 토픽을 인코딩하는 대신(`FrameRouter`), 이미
만들어져 있는 mp4 를 **그 에피소드 행에 붙이기만** 한다.

**등록을 레코더에 맡기지 않는 이유**는 녹화를 DB 에서 떼어 두려고 rosbag 을 둔
취지에 맞추기 위해서다 — 레코더는 파일만 남기고, DB 는 적재가 만진다.

붙이는 열쇠는 **`episode_key`** 다. 레코더가 metadata 에 적은 `/session` IN_EPISODE
의 `header.stamp` 이고, 적재가 에피소드를 식별하는 값과 같은 것이다.

⚠️ **파일 이름으로 맞추면 안 된다.** 파일 이름의 시각은 벽시계(`datetime.now()`)이고
`episode_key` 는 스택 시계다 — `use_sim_time` 스택에서는 둘이 아예 무관하다.

**같은 트랜잭션 안에서 부른다.** 에피소드 행을 만든 커넥션을 그대로 받아 쓰므로,
메시지 적재가 롤백되면 영상 등록도 함께 되돌아간다.
"""

from __future__ import annotations

from typing import Iterator, Optional

import json
import os
from dataclasses import dataclass
from pathlib import Path

import psycopg

from ...db.topics import upsert_topic_ids
from ...db.writers.image_frame import ImageFrameWriter
from ...db.writers.image_stream import ImageStreamWriter

#: metadata 파일을 알아보는 접미사. mp4·sidecar 이름은 metadata 안에 적혀 있으므로
#: 이름 규칙을 여기서 다시 조립하지 않는다.
METADATA_SUFFIX = '_metadata.json'

#: 타입을 못 읽었을 때 쓰는 값. `topics` 는 NOT NULL 이라 빈 문자열을 넣을 수 없다.
_FALLBACK_TOPIC_TYPE = 'sensor_msgs/msg/Image'


class RecordingError(RuntimeError):
    """레코더 출력이 스스로 모순될 때. **조용히 넘기지 않는다.**"""


@dataclass(frozen=True)
class Recording:
    """레코더가 남긴 한 벌(mp4 + sidecar + metadata)."""

    episode_ns: int
    topic: str
    topic_type: str
    mp4_path: Path
    sidecar_path: Path
    metadata_path: Path
    codec: str
    pixel_format: str
    fps: int
    frame_id: str
    width: int
    height: int
    nframe: int


def _read_recording(metadata_path: Path) -> Optional[Recording]:
    """metadata 한 개를 읽는다. 읽을 수 없으면 ``None``.

    **키가 없으면 건너뛴다** — 2026-09-18 이전 형식의 출력에는 `episode_key` 가
    없고, 그것을 억지로 붙이면 엉뚱한 에피소드에 영상이 달린다.
    """
    try:
        with open(metadata_path, encoding='utf-8') as fp:
            meta = json.load(fp)
    except (OSError, json.JSONDecodeError):
        return None
    key = meta.get('episode_key')
    if not isinstance(key, dict) or 'sec' not in key:
        return None
    root = metadata_path.parent
    mp4 = meta.get('mp4')
    sidecar = meta.get('sidecar')
    if not mp4 or not sidecar:
        return None
    return Recording(
        episode_ns=int(key['sec']) * 1_000_000_000 + int(key.get('nanosec', 0)),
        topic=str(meta.get('topic') or ''),
        topic_type=str(meta.get('topic_type') or _FALLBACK_TOPIC_TYPE),
        mp4_path=root / mp4, sidecar_path=root / sidecar, metadata_path=metadata_path,
        codec=str(meta.get('codec') or ''),
        pixel_format=str(meta.get('pixel_format') or meta.get('encoding') or ''),
        fps=int(meta.get('fps') or 0), frame_id=str(meta.get('frame_id') or ''),
        width=int(meta.get('width') or 0), height=int(meta.get('height') or 0),
        nframe=int(meta.get('nframe') or 0))


def scan_recordings(root: os.PathLike) -> dict:
    """레코더 출력 루트를 훑어 **에피소드 키 → 녹화 목록** 으로 모은다.

    한 에피소드에 카메라가 여럿이면(`file_prefix` 로 가른다) 목록이 여럿이다.
    """
    index: dict = {}
    base = Path(root)
    if not base.is_dir():
        return index
    for path in sorted(base.rglob(f'*{METADATA_SUFFIX}')):
        rec = _read_recording(path)
        if rec is not None:
            index.setdefault(rec.episode_ns, []).append(rec)
    return index


def _iter_sidecar(path: Path) -> Iterator[tuple]:
    """sidecar(jsonl)에서 ``(frame_index, sec, nanosec)`` 를 뽑는다.

    깨진 줄은 **건너뛰지 않고 올린다** — mp4 의 N번째 프레임과 sidecar 의 N번째
    줄이 1:1 이라는 것이 이 데이터의 전부이고, 한 줄만 빠져도 그 뒤가 전부 밀린다.
    """
    with open(path, encoding='utf-8') as fp:
        for lineno, line in enumerate(fp, 1):
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
                stamp = rec['stamp']
                yield int(rec['frame_index']), int(stamp['sec']), int(stamp['nanosec'])
            except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
                raise RecordingError(
                    f'{path}: malformed sidecar line {lineno}: {exc}') from exc


class RecorderRegistrar:
    """레코더 출력을 에피소드에 붙인다.

    Args:
        conn: **에피소드 행을 만든 그 커넥션.** 같은 트랜잭션이어야 메시지 적재와
            함께 커밋·롤백된다.
        root: 레코더 출력 루트 (`image_pipeline.yaml` 의 `output_dir`).
        index: 미리 훑어 둔 결과. 여러 에피소드를 처리할 때 재사용한다.
    """

    def __init__(self, conn: psycopg.Connection, root: os.PathLike,
                 *, index: Optional[dict] = None) -> None:
        self._conn = conn
        self._root = Path(root)
        self._index = scan_recordings(root) if index is None else index
        self._inserted: dict = {}

    @property
    def index(self) -> dict:
        """에피소드 키 → 녹화 목록. 다음 에피소드에 물려주면 재훑기를 피한다."""
        return self._index

    def register(self, episode_id: int, episode_ns: int) -> dict:
        """한 에피소드의 녹화를 등록하고 테이블별 INSERT 수를 돌려준다.

        해당 에피소드의 녹화가 없으면 빈 dict 다 — **오류가 아니다.** 영상 없이
        수집한 에피소드가 정상적으로 있을 수 있다(헤드리스 실행 등).

        Raises:
            RecordingError: mp4 가 없거나 sidecar 줄 수가 `nframe` 과 다를 때.
                조용히 넘기면 **프레임 인덱스가 밀린 채로** 적재된다.
        """
        recordings = self._index.get(episode_ns) or []
        if not recordings:
            return {}

        counts: dict = {}
        for rec in recordings:
            if not rec.mp4_path.exists():
                raise RecordingError(
                    f'{rec.metadata_path}: mp4 not found ({rec.mp4_path})')
            if not rec.sidecar_path.exists():
                raise RecordingError(
                    f'{rec.metadata_path}: sidecar not found ({rec.sidecar_path})')

            topic_id = self._topic_id(rec)
            frames = list(_iter_sidecar(rec.sidecar_path))
            if len(frames) != rec.nframe:
                raise RecordingError(
                    f'{rec.metadata_path}: sidecar has {len(frames)} line(s) but '
                    f'metadata says nframe={rec.nframe}')

            # mp4_path 는 **출력 루트 기준 상대경로**다 — ffmpeg_sink 와 같은 규약이라
            # 재생 쪽이 루트만 바꿔 끼우면 된다 (docker 의 RDFP_DATA_DIR).
            try:
                mp4_rel = str(rec.mp4_path.relative_to(self._root))
            except ValueError:
                mp4_rel = str(rec.mp4_path)

            ImageStreamWriter(self._conn).insert(
                episode_id=episode_id, topic_id=topic_id, mp4_path=mp4_rel,
                codec=rec.codec, pixel_format=rec.pixel_format,
                container_fps=rec.fps, frame_id=rec.frame_id,
                width=rec.width, height=rec.height, frame_count=rec.nframe)
            counts['image_streams'] = counts.get('image_streams', 0) + 1

            writer = ImageFrameWriter(self._conn, topic_id=topic_id)
            for frame_index, sec, nsec in frames:
                writer.append_row(episode_id, frame_index, sec, nsec)
            writer.flush()
            counts['image_frames'] = (counts.get('image_frames', 0)
                                      + writer.consume_inserted_count())

        for table, n in counts.items():
            self._inserted[table] = self._inserted.get(table, 0) + n
        return counts

    def consume_inserted_count(self) -> dict:
        """마지막 호출 이후 INSERT 한 행 수를 돌려주고 0 으로 되돌린다.

        `WriterBase` 와 같은 모양이라 파이프라인이 요약에 그대로 더할 수 있다 —
        이 경로를 빼먹으면 DB 에는 행이 있는데 요약은 0 으로 보고한다.
        """
        out, self._inserted = self._inserted, {}
        return out

    def _topic_id(self, rec: Recording) -> int:
        """토픽 행을 확보한다.

        **영상은 rosbag 에 담지 않으므로 `topics` 행이 아직 없을 수 있다** — 메시지
        적재 경로에서 만들어지지 않는다. 그래서 여기서 upsert 한다.
        """
        if not rec.topic:
            raise RecordingError(f'{rec.metadata_path}: metadata has no topic name')
        return upsert_topic_ids(self._conn, {rec.topic: rec.topic_type})[rec.topic]


__all__ = ['METADATA_SUFFIX', 'RecorderRegistrar', 'Recording', 'RecordingError',
           'scan_recordings']
