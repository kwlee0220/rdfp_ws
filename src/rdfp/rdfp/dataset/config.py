"""dataset CLI 전용 설정 모델 및 YAML 로더.

`dataset` 서브커맨드 (init-db / import / stats / list / replay) 가 사용한다.

`init-db` / `stats` / `list` / `replay` 는 `db` 섹션만 필요하지만, `import`
는 `output_mp4_dir` 이 반드시 지정되어야 한다 (런타임 검증).

DB 접속 설정(`DbConfig`) 및 `resolve_dsn` 은 DB 계층의 관심사이므로
`rdfp.dataset.db.config` 에 정의되어 있으며 여기서 재노출한다.
"""

from __future__ import annotations

from typing import Literal

import os
import re
from pathlib import Path

import yaml
from pydantic import BaseModel, Field, field_validator

from rdfp.dataset.db.config import DbConfig


# 세션 이벤트(녹화 시작/종료) 를 방송하는 ROS 2 토픽 이름. session_control_node
# 가 `rdfp_msgs/msg/SessionCommand` 로 발행하며, 데이터셋 후처리기가 에피소드
# 경계를 감지하는 단일 신호로 사용한다.
SESSION_TOPIC: str = '/session'


class EpisodeFilter(BaseModel):
    """에피소드 단위 필터 (dataset import 전용)."""

    task_labels_include: list[str] = Field(default_factory=list)
    task_labels_exclude: list[str] = Field(default_factory=list)
    min_duration_sec: float = 0.0


class SessionFilter(BaseModel):
    """세션(날짜) 단위 선행 필터 (dataset import 전용)."""

    dates: list[str] = Field(default_factory=list)

    @field_validator('dates')
    @classmethod
    def _check_date_format(cls, v: list[str]) -> list[str]:
        pat = re.compile(r'^\d{4}-\d{2}-\d{2}$')
        for d in v:
            if not pat.match(d):
                raise ValueError(f"date must be YYYY-MM-DD: {d!r}")
        return v


class Mp4Config(BaseModel):
    """MP4 인코딩 기본값 (Phase 2 에서 사용)."""

    codec: str = 'h264'
    nominal_fps: int = 30


class QualityGateConfig(BaseModel):
    """품질 게이트 설정 (Phase 3).

    감지된 이상은 JSONL 로그에 `event='quality_warning'` 로 기록되지만,
    에피소드 자체를 제외하거나 실패시키지 않는다.
    """

    stamp_regression: bool = True
    idle_gap_sec: float = 0.0     # 0 = 비활성. 양수 지정 시 토픽 간 유휴 감지.


_FALLBACK_PATTERN = re.compile(r'\$\{([A-Za-z_][A-Za-z0-9_]*):-([^}]*)\}')


def _expand_path(value: str) -> str:
    """`${VAR}` / `${VAR:-default}` 환경변수 치환과 `~` 홈 디렉터리 확장을 적용한다.

    `${VAR:-default}` 는 셸 파라미터 확장과 같은 의미로, 변수가 unset 이거나
    빈 문자열일 때 `default` 를 사용한다. fallback 값 자체도 `~` 및 다른
    `${VAR}` 치환의 대상이 된다 (한 단계 재확장).

    경로 필드에서만 사용한다 (다른 문자열 필드에는 무차별 적용하지 않는다).
    """
    def _sub(m: re.Match[str]) -> str:
        return os.environ.get(m.group(1)) or m.group(2)

    expanded = _FALLBACK_PATTERN.sub(_sub, value)
    return os.path.expanduser(os.path.expandvars(expanded))


class DatasetConfig(BaseModel):
    """`dataset` CLI 전용 설정."""

    rosbag_dir: str
    topics: list[str] = Field(default_factory=list)
    episode_filter: EpisodeFilter = Field(default_factory=EpisodeFilter)
    session_filter: SessionFilter = Field(default_factory=SessionFilter)

    # dataset 전용 필드.
    # import 전용 필드. init-db/stats/list/replay 에서는 미지정 허용.
    output_mp4_dir: str | None = None

    @field_validator('rosbag_dir')
    @classmethod
    def _expand_rosbag_dir(cls, v: str) -> str:
        return _expand_path(v)

    @field_validator('output_mp4_dir')
    @classmethod
    def _expand_output_mp4_dir(cls, v: str | None) -> str | None:
        return _expand_path(v) if v is not None else None
    db: DbConfig = Field(default_factory=lambda: DbConfig())
    on_existing_episode: Literal['skip', 'replace', 'error'] = 'skip'
    mp4: Mp4Config = Field(default_factory=Mp4Config)
    # Phase 3: 에피소드 단위 병렬 처리 워커 수. 1 이면 기존 단일 프로세스 스트리밍.
    parallelism: int = 1
    quality_gate: QualityGateConfig = Field(default_factory=QualityGateConfig)
    # True 이면 import 가 예외 없이 끝난 뒤 사용된 split (.mcap) 과, 그 결과
    # 비게 된 세션 디렉터리(metadata.yaml 만 남은 경우) 를 제거한다.
    delete_splits_after_import: bool = False

    def effective_topics(self) -> list[str]:
        """`/session` 자동 포함을 적용한 대상 토픽 목록을 반환한다.

        원 `topics` 가 비어 있으면 (전 토픽 대상) 빈 목록 그대로 반환한다.
        호출 측은 빈 목록을 "rosbag 의 모든 토픽" 으로 해석한다.
        """
        if not self.topics:
            return []
        if SESSION_TOPIC not in self.topics:
            return [SESSION_TOPIC, *self.topics]
        return list(self.topics)


def _read_topics_file(path: Path) -> list[str]:
    """텍스트 파일에서 녹화 토픽 목록을 읽는다.

    규칙:
      - 한 줄에 토픽 하나.
      - `#` 으로 시작하는 줄과 빈 줄은 무시.
      - 줄 끝 공백은 strip 한다 (인라인 `#` 주석은 지원하지 않는다).
    """
    if not path.is_file():
        raise FileNotFoundError(f"topics_file not found: {path}")
    topics: list[str] = []
    with path.open('r', encoding='utf-8') as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue
            topics.append(stripped)
    return topics


def load_dataset_config(path: str | os.PathLike) -> DatasetConfig:
    """YAML 파일을 읽어 `DatasetConfig` 로 로드한다.

    `topics_file:` 디렉티브가 있으면 동일 파일을 파싱하여 `topics:` 와 병합한다
    (파일 항목 먼저, 이후 YAML 의 `topics:`, 중복은 순서 유지하며 제거).
    `topics_file` 의 상대 경로는 YAML 파일의 디렉터리 기준으로 해석한다.

    Raises:
        FileNotFoundError: 설정 파일 또는 `topics_file` 이 없는 경우.
        ValueError: YAML 구문이 잘못되었거나 스키마 검증에 실패한 경우.
    """
    p = Path(path)
    if not p.is_file():
        raise FileNotFoundError(f"config file not found: {path}")
    with p.open('r', encoding='utf-8') as f:
        raw = yaml.safe_load(f) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"config root must be a mapping: {path}")

    # topics_file 디렉티브는 YAML 로더 전용. 모델 필드가 아니므로 raw 에서 제거한다.
    topics_file = raw.pop('topics_file', None)
    if topics_file is not None:
        tf = Path(_expand_path(str(topics_file)))
        if not tf.is_absolute():
            tf = p.parent / tf
        file_topics = _read_topics_file(tf)
        existing = list(raw.get('topics') or [])
        seen: set[str] = set()
        merged: list[str] = []
        for t in (*file_topics, *existing):
            if t not in seen:
                merged.append(t)
                seen.add(t)
        raw['topics'] = merged

    return DatasetConfig.model_validate(raw)


__all__ = [
    'SESSION_TOPIC',
    'DbConfig',
    'EpisodeFilter',
    'SessionFilter',
    'Mp4Config',
    'QualityGateConfig',
    'DatasetConfig',
    'load_dataset_config',
]
