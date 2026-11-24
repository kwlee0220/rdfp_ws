"""토픽 → 테이블 매핑 상태 — "어느 토픽이 어느 테이블로 가고, 왜 이 토픽은 안 가나".

정본은 **코드**다 (`rdfp/dataset/db/registry.py` 의 `MESSAGE_TYPE_REGISTRY`). 매핑의 키는
**타입**이고 토픽이 아니라서, 같은 타입의 토픽 둘(`/joint_states` · `/target_joint_cmds`)이 한
테이블로 가고 `topic_id` 로만 갈린다. 이 모듈은 그 사실을 토픽 하나마다 한 줄(`MappingInfo`)로
풀어 GUI 와 CLI 가 같은 말을 하게 한다.

상태는 넷이다 (설계안 §3.1):

| 상태 | 뜻 |
|---|---|
| `mapped`   | registry 에 타입이 있다 — 적재된다 |
| `special`  | `/session`(에피소드 경계) · `sensor_msgs/Image`(mp4 로 간다) |
| `unmapped` | writer 가 없다 — 적재가 **조용히** 무시한다. 가장 먼저 눈에 띄게 할 것 |
| `suspect`  | 매핑은 있는데 stamp 진단에 걸렸다 — 진단 결과를 덧씌워 쓴다 (`glyph_for`) |

동적 매핑(`topic_tables`, 설계안 §4.3 — 2026-11-23 구현)은 `dynamic` 인자로 받는다. registry 에
없는 타입이 거기 있으면 `mapped` 이고 출처가 `dynamic` 이다. **내장이 이긴다.**
"""

from __future__ import annotations

from typing import Mapping, Optional

from dataclasses import dataclass

from rdfp.dataset.config import SESSION_TOPIC
from rdfp.dataset.db.registry import (MESSAGE_TYPE_REGISTRY, SESSION_COMMAND_TYPE, is_image_type,
                                      resolve_message_type)
from rdfp.dataset.db.schema_check import META_TABLES, REQUIRED_COLUMNS

STATUS_MAPPED = 'mapped'
STATUS_SPECIAL = 'special'
STATUS_UNMAPPED = 'unmapped'
STATUS_SUSPECT = 'suspect'

#: 화면 · CLI 가 같은 기호를 쓴다 (설계안 §3.1).
GLYPH = {STATUS_MAPPED: '●', STATUS_SPECIAL: '◇', STATUS_UNMAPPED: '✖', STATUS_SUSPECT: '△'}

SOURCE_BUILTIN = 'builtin'    # registry 의 writer 가 넣는 메시지 테이블
SOURCE_SYSTEM = 'system'      # sessions · topics — 적재 골격
SOURCE_MEDIA = 'media'        # image_streams · image_frames — mp4 와 짝
SOURCE_DYNAMIC = 'dynamic'    # topic_tables 로 등록한 JSONB 테이블
SOURCE_META = 'meta'          # ingest_sources · episode_sources · ingest_units · topic_tables

SYSTEM_TABLES = ('sessions', 'topics')
MEDIA_TABLES = ('image_streams', 'image_frames')

COMPRESSED_IMAGE_TYPE = 'sensor_msgs/msg/CompressedImage'

_REASON_SESSION = ('에피소드 경계. IN_EPISODE → IN_SESSION 전이 하나가 에피소드 하나이고 '
                   '`sessions` 행이 된다. 메시지 자체는 테이블에 남지 않는다.')
_REASON_IMAGE = ('mp4 로 인코딩된다 (FrameRouter → image_streams · image_frames). ⚠️ 설정에 '
                 '`recordings_dir` 이 있으면 레코더 등록(I4)과 같은 (episode_id, topic_id) 로 '
                 'image_streams 에 넣어 UNIQUE 위반으로 import 가 죽는다 — 둘 중 하나만 켠다.')
_REASON_COMPRESSED = ('적재는 sensor_msgs/Image 만 인코딩한다 — 압축 이미지는 registry 에도 없어 '
                      '`no writer registered … ignored` 경고만 남기고 무시된다. 영상의 원본은 '
                      '레코더 mp4(image_streams) 이므로 이 토픽은 녹화 목록에서 빼는 것이 맞다.')
_REASON_UNKNOWN = ('registry(`rdfp/dataset/db/registry.py`)에 이 타입의 writer 가 없다. 적재는 '
                   '`no writer registered` 경고 한 줄을 남기고 메시지를 무시한다 — rosbag 에는 '
                   '남고 DB 에만 없다. [테이블 생성…](`rdfp mapping register`)으로 JSONB 테이블에 '
                   '담을 수 있다. 자주 쓰게 되면 writer·reader 를 써서 내장으로 올린다.')
_REASON_DYNAMIC = ('동적 매핑 (`topic_tables`) — 메시지를 JSONB `payload` 한 열에 담는다. 같은 '
                   '타입의 다른 토픽도 이 테이블로 가고 `topic_id` 로 갈린다. 등록할 때 본 `.msg` '
                   '정의와 다른 bag 은 적재가 거부한다.')


@dataclass(frozen=True)
class MappingInfo:
    """토픽 하나의 매핑 한 줄."""

    topic: str
    type_name: str
    status: str
    table: Optional[str]
    writer: Optional[str]
    reader: Optional[str]
    reason: str
    source: str = SOURCE_BUILTIN

    @property
    def ingested(self) -> bool:
        """적재가 이 토픽의 메시지를 DB(또는 mp4)에 남기는가."""
        return self.status in (STATUS_MAPPED, STATUS_SPECIAL) and self.table is not None

    @property
    def is_image(self) -> bool:
        return is_image_type(self.type_name)

    @property
    def glyph(self) -> str:
        return GLYPH[self.status]


def glyph_for(info: MappingInfo, *, suspect: bool = False) -> str:
    """stamp 진단을 덧씌운 기호 — 매핑된 토픽이 진단에 걸리면 `△` 로 바꿔 보인다."""
    if suspect and info.status == STATUS_MAPPED:
        return GLYPH[STATUS_SUSPECT]
    return info.glyph


def classify_topic(topic: str, type_name: str,
                   dynamic: Optional[Mapping[str, object]] = None) -> MappingInfo:
    """토픽 하나를 `mapped` · `special` · `unmapped` 로 가른다 — 적재의 분류와 같은 순서다.

    `_determine_topic_classification` 이 `/session` → 이미지 → registry → 동적 매핑 순으로 보므로
    여기도 같다. `dynamic` 은 타입 → `DynamicMapping`(또는 `table_name` 속성이 있는 것) 이다.
    """
    if topic == SESSION_TOPIC or type_name == SESSION_COMMAND_TYPE:
        return MappingInfo(topic, type_name, STATUS_SPECIAL, 'sessions', 'SessionWriter', None,
                           _REASON_SESSION)
    if is_image_type(type_name):
        return MappingInfo(topic, type_name, STATUS_SPECIAL, 'image_streams', 'FrameRouter',
                           'Mp4ImageReplayer', _REASON_IMAGE)
    binding = resolve_message_type(type_name)
    if binding is None:
        mapping = (dynamic or {}).get(type_name)
        if mapping is not None:
            return MappingInfo(topic, type_name, STATUS_MAPPED, getattr(mapping, 'table_name'),
                               'GenericJsonbWriter', None, _REASON_DYNAMIC, SOURCE_DYNAMIC)
        reason = _REASON_COMPRESSED if type_name == COMPRESSED_IMAGE_TYPE else _REASON_UNKNOWN
        return MappingInfo(topic, type_name, STATUS_UNMAPPED, None, None, None, reason)
    return MappingInfo(topic, type_name, STATUS_MAPPED, binding.table,
                       binding.writer_cls.__name__, binding.reader_cls.__name__,
                       f'`{type_name}` → `{binding.table}` (내장 registry). 같은 타입의 다른 '
                       f'토픽도 이 테이블로 가고 `topic_id` 로 갈린다.')


def classify_topics(topic_types: Mapping[str, str],
                    dynamic: Optional[Mapping[str, object]] = None) -> dict[str, MappingInfo]:
    """토픽 → 타입 매핑 전체를 한 번에 가른다 (토픽 이름순)."""
    return {t: classify_topic(t, topic_types[t], dynamic) for t in sorted(topic_types)}


@dataclass(frozen=True)
class TableInfo:
    """DB 테이블 한 줄 — 출처와 어느 타입이 들어오는가."""

    name: str
    source: str
    type_names: tuple
    description: str


def builtin_tables(dynamic: Optional[Mapping[str, object]] = None) -> list[TableInfo]:
    """적재가 아는 테이블 전부 (`schema_check.REQUIRED_COLUMNS` + 동적 매핑) 와 그 출처."""
    types_by_table: dict[str, list[str]] = {}
    for type_name, binding in MESSAGE_TYPE_REGISTRY.items():
        types_by_table.setdefault(binding.table, []).append(type_name)
    out: list[TableInfo] = []
    for name in sorted(REQUIRED_COLUMNS):
        if name in META_TABLES:
            out.append(TableInfo(name, SOURCE_META, (), _META_DESC.get(name, '적재 메타')))
        elif name in SYSTEM_TABLES:
            desc = ('에피소드 한 행 (`/session` 의 IN_EPISODE → IN_SESSION 전이)' if name == 'sessions'
                    else '토픽 이름 ↔ id. 메시지 테이블의 `topic_id` 가 가리킨다')
            out.append(TableInfo(name, SOURCE_SYSTEM, (), desc))
        elif name in MEDIA_TABLES:
            desc = ('에피소드 × 카메라 토픽당 mp4 한 벌 (레코더 등록 I4 또는 bag 인코딩)'
                    if name == 'image_streams' else 'mp4 의 N번째 프레임 ↔ 원본 stamp (frame clock 의 근거)')
            out.append(TableInfo(name, SOURCE_MEDIA, ('sensor_msgs/msg/Image',), desc))
        else:
            types = tuple(sorted(types_by_table.get(name, ())))
            out.append(TableInfo(name, SOURCE_BUILTIN, types,
                                 '내장 registry — writer 가 뜻을 담아 설계한 열'))
    for type_name, mapping in sorted((dynamic or {}).items()):
        out.append(TableInfo(getattr(mapping, 'table_name'), SOURCE_DYNAMIC, (type_name,),
                             '동적 매핑 — JSONB payload (`rdfp mapping` 으로 등록 · 해제 · 삭제)'))
    out.sort(key=lambda t: t.name)
    return out


_META_DESC = {
    'ingest_sources': '녹화본(세션 디렉터리) — 에피소드 출처의 대상',
    'episode_sources': '에피소드 ↔ 녹화본. 같은 키를 낸 다른 녹화본을 가려낸다',
    'ingest_units': '적재 단위 (에피소드 × 토픽) — 빠진 토픽 채우기 · 토픽 교체의 근거',
    'topic_tables': '동적 매핑 등록부 — 타입 → JSONB 테이블',
}


def table_for_type(type_name: str,
                   dynamic: Optional[Mapping[str, object]] = None) -> Optional[str]:
    """타입이 가는 테이블 이름. 이미지는 `image_streams`, 없으면 ``None``."""
    if is_image_type(type_name):
        return 'image_streams'
    binding = resolve_message_type(type_name)
    if binding is not None:
        return binding.table
    mapping = (dynamic or {}).get(type_name)
    return getattr(mapping, 'table_name') if mapping is not None else None


__all__ = [
    'COMPRESSED_IMAGE_TYPE', 'GLYPH', 'MEDIA_TABLES', 'MappingInfo', 'SOURCE_BUILTIN',
    'SOURCE_DYNAMIC', 'SOURCE_MEDIA', 'SOURCE_META', 'SOURCE_SYSTEM', 'STATUS_MAPPED',
    'STATUS_SPECIAL', 'STATUS_SUSPECT',
    'STATUS_UNMAPPED', 'SYSTEM_TABLES', 'TableInfo', 'builtin_tables', 'classify_topic',
    'classify_topics', 'glyph_for', 'table_for_type',
]
