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


class DatasetConfig(BaseModel):
    """`dataset` CLI 전용 설정."""

    rosbag_dir: str
    topics: list[str] = Field(default_factory=list)
    episode_filter: EpisodeFilter = Field(default_factory=EpisodeFilter)
    session_filter: SessionFilter = Field(default_factory=SessionFilter)

    # dataset 전용 필드.
    # import 전용 필드. init-db/stats/list/replay 에서는 미지정 허용.
    output_mp4_dir: str | None = None
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


def load_dataset_config(path: str | os.PathLike) -> DatasetConfig:
    """YAML 파일을 읽어 `DatasetConfig` 로 로드한다.

    Raises:
        FileNotFoundError: 파일이 없는 경우.
        ValueError: YAML 구문이 잘못되었거나 스키마 검증에 실패한 경우.
    """
    p = Path(path)
    if not p.is_file():
        raise FileNotFoundError(f"config file not found: {path}")
    with p.open('r', encoding='utf-8') as f:
        raw = yaml.safe_load(f) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"config root must be a mapping: {path}")
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
