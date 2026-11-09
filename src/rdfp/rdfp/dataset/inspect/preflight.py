"""import · export 사전 점검 — 실행 전에 보여 주고, 죽을 조합은 막는다 (설계안 §4.4 · §4.6).

`cmd_import` 안에 흩어져 있던 검사를 여기로 꺼내 **CLI 와 GUI 가 같은 함수를 부른다.** GUI 만
검사하면 CLI 로 돌릴 때 같은 함정에 다시 빠진다.

:func:`check_import` 는 **DB 를 열지 않는다** — 설정 · 토픽 분류 · 에피소드 창만 보므로 ROS 도
DB 도 없이 시험할 수 있다. 접속이 필요한 것은 :func:`check_db` · :func:`check_export` 로 나눴다.
"""

from __future__ import annotations

from typing import Any, Iterable, Mapping, Optional, Sequence

import os
from pathlib import Path

import yaml

from rdfp.dataset.config import DatasetConfig
from rdfp.dataset.db.registry import is_image_type
from rdfp.dataset.db.schema_check import REQUIRED_COLUMNS, SchemaCheckError, ensure_schema

from .findings import LEVEL_ERROR, LEVEL_INFO, LEVEL_WARNING, Finding, has_errors
from .mapping import STATUS_UNMAPPED, classify_topics

#: 참조 녹화 목록. `dataset_config.yaml` 의 `topics_file` 이 이것과 다르면 녹화와 적재가 어긋난다.
REFERENCE_TOPICS_LIST = Path('config') / 'recording_topics.list'


def overlapping_windows(windows: Sequence[tuple[int, int]]) -> list[tuple[int, int]]:
    """구간이 겹치는 창의 (i, j) 쌍 — 시뮬레이터 재기동으로 sim 시계가 되감긴 bag 의 징후다."""
    order = sorted(range(len(windows)), key=lambda i: windows[i])
    out: list[tuple[int, int]] = []
    for a in range(len(order)):
        i = order[a]
        for b in range(a + 1, len(order)):
            j = order[b]
            if windows[j][0] >= windows[i][1]:
                break
            out.append((i, j))
    return out


def check_import(cfg: DatasetConfig, *, topic_types: Mapping[str, str],
                 selected_topics: Optional[Iterable[str]] = None,
                 episode_windows: Optional[Sequence[tuple[int, int]]] = (),
                 existing_keys: Optional[Iterable[int]] = None,
                 policy: Optional[str] = None) -> list[Finding]:
    """import 를 돌리기 전에 설정과 선택을 점검한다 (DB 접속 없음).

    Args:
        cfg: 적재 설정.
        topic_types: bag 의 토픽 → 타입 (선택 가능한 전부).
        selected_topics: 적재할 토픽. ``None`` 이면 전부다.
        episode_windows: 적재할 에피소드 창 `[start, stop)` 목록. ``None`` 이면 창을 보지 않는다
            (빈 목록과 다르다 — 빈 목록은 '하나도 없다' 라 막는다).
        existing_keys: DB 에 이미 있는 에피소드의 `start_ns`. ``None`` 이면 확인하지 않았다.
        policy: `on_existing_episode` 재정의. ``None`` 이면 설정값.
    """
    findings: list[Finding] = []
    policy = policy or cfg.on_existing_episode

    if cfg.output_mp4_dir is None:
        findings.append(Finding(
            LEVEL_ERROR, 'output_mp4_dir',
            'output_mp4_dir is required for import; add it to the dataset config'))

    chosen = set(selected_topics) if selected_topics is not None else set(topic_types)
    infos = classify_topics({t: ty for t, ty in topic_types.items() if t in chosen})
    images = sorted(t for t, ty in topic_types.items() if t in chosen and is_image_type(ty))
    if images and cfg.recordings_dir:
        findings.append(Finding(
            LEVEL_ERROR, 'image_conflict',
            f'이미지 토픽 {images} 을 적재하면서 `recordings_dir` 도 켜져 있다 — bag 쪽(FrameRouter)과 '
            f'레코더 등록(RecorderRegistrar)이 같은 (episode_id, topic_id) 로 image_streams 에 넣어 '
            f'UNIQUE 위반으로 import 가 죽는다. 녹화 목록에서 이미지 토픽을 빼거나 '
            f'`recordings_dir` 을 비운다 (둘 중 하나만).'))

    unmapped = sorted(t for t, i in infos.items() if i.status == STATUS_UNMAPPED)
    if unmapped:
        findings.append(Finding(LEVEL_WARNING, 'unmapped_topics',
                                f'writer 가 없어 무시되는 토픽: {unmapped} — rosbag 에는 남고 DB 에는 '
                                f'들어가지 않는다.'))

    if selected_topics is not None and chosen < set(topic_types):
        left_out = sorted(set(topic_types) - chosen)
        findings.append(Finding(LEVEL_WARNING, 'partial_topics',
                                f'토픽 일부만 적재한다 (제외: {left_out}). 적재는 에피소드 단위라 '
                                f'제외한 토픽은 그 에피소드에 **빈 채로** 커밋되고, 뒤에 채우려면 '
                                f'`replace` 로 에피소드를 통째로 다시 넣어야 한다.'))

    # ``None`` 은 "창을 안 봤다" 다 — 호출자가 에피소드 유무를 따로 안다는 뜻이라 여기서 판정하지
    # 않는다. 빈 목록은 "하나도 없다" 라 막는다.
    if episode_windows is None:
        pass
    elif not episode_windows:
        findings.append(Finding(LEVEL_ERROR, 'no_episodes',
                                '적재할 에피소드가 없다 — `/session` 의 IN_EPISODE → IN_SESSION 전이가 '
                                'bag 에 없거나 선택이 비었다.'))
    else:
        pairs = overlapping_windows(list(episode_windows))
        if pairs:
            findings.append(Finding(
                LEVEL_WARNING, 'overlapping_episodes',
                f'구간이 겹치는 에피소드 {len(pairs)} 쌍 — 시뮬레이터 재기동으로 sim 시계가 0 으로 '
                f'되감긴 bag 을 같은 rosbag_dir 에 둔 징후다. 함께 적재하면 관절 데이터가 조용히 '
                f'0 건이 된다. 적재가 끝난 bag 은 rosbag_dir 밖으로 뺀다.'))

    if existing_keys is not None and episode_windows:
        dup = sorted({w[0] for w in episode_windows} & set(existing_keys))
        if dup:
            if policy == 'replace':
                findings.append(Finding(LEVEL_WARNING, 'existing_replace',
                                        f'이미 적재된 에피소드 {len(dup)}개를 **덮어쓴다** (replace): '
                                        f'기존 행과 mp4 디렉터리가 지워지고 id 가 바뀐다.'))
            elif policy == 'error':
                findings.append(Finding(LEVEL_ERROR, 'existing_error',
                                        f'이미 적재된 에피소드 {len(dup)}개가 있어 정책 `error` 로는 '
                                        f'첫 중복에서 멈춘다. `skip` 또는 `replace` 를 고른다.'))
            else:
                findings.append(Finding(LEVEL_INFO, 'existing_skip',
                                        f'이미 적재된 에피소드 {len(dup)}개는 건너뛴다 (skip).'))
    return findings


def check_db(cfg: DatasetConfig, *,
             required_tables: Optional[Iterable[str]] = None) -> list[Finding]:
    """DSN · 접속 · 스키마를 확인한다. 비밀번호는 어디에도 적지 않는다."""
    from rdfp.dataset.db.config import resolve_dsn
    from rdfp.dataset.db.connection import open_connection

    try:
        resolve_dsn(cfg.db)
    except RuntimeError as exc:
        return [Finding(LEVEL_ERROR, 'dsn', str(exc))]
    tables = list(required_tables) if required_tables is not None else list(REQUIRED_COLUMNS)
    try:
        with open_connection(cfg.db) as conn:
            ensure_schema(conn, tables)
    except SchemaCheckError as exc:
        return [Finding(LEVEL_ERROR, 'schema', f'{exc} — `ros2 run rdfp init-db` 로 만든다')]
    except Exception as exc:  # noqa: BLE001 — 드라이버 예외 종류가 많다; 메시지만 보인다
        return [Finding(LEVEL_ERROR, 'db_connect', f'cannot connect to the database: {exc}')]
    return [Finding(LEVEL_INFO, 'db_ok', f'DB 접속 · 스키마 확인 ({len(tables)} tables)')]


def check_export(conn: Any, episode_ids: Sequence[int], *, camera_topic: str = '',
                 label: Optional[str] = None,
                 command_topic: Optional[str] = None) -> list[Finding]:
    """export 가 예외로 멈추는 조건을 미리 본다 — frame clock · 카메라 선택 · 팔 관절."""
    from rdfp.dataset.export.frame_clock import available_clock_topics
    from rdfp.dataset.export.writer import (DEFAULT_COMMAND_TOPIC, LABEL_PLAN, ExportError,
                                            default_arm_joints)

    label = label or LABEL_PLAN
    command_topic = command_topic or DEFAULT_COMMAND_TOPIC
    findings: list[Finding] = []
    if not episode_ids:
        return [Finding(LEVEL_ERROR, 'no_episodes', 'export 할 에피소드가 없다.')]
    cameras: set = set()
    for eid in episode_ids:
        topics = available_clock_topics(conn, eid)
        cameras.update(topics)
        if not topics:
            findings.append(Finding(LEVEL_ERROR, 'no_video',
                                    f'에피소드 {eid}: 영상(image_streams)이 없어 frame clock 이 서지 '
                                    f'않는다 — 레코더 출력이 등록됐는지 본다.'))
        elif len(topics) > 1 and not camera_topic:
            findings.append(Finding(LEVEL_ERROR, 'camera_choice',
                                    f'에피소드 {eid}: 카메라가 {len(topics)}개 {topics} — 시계로 쓸 '
                                    f'토픽을 골라야 한다 (아무거나 고르면 영상과 레이블의 위상이 어긋난다).'))
        elif camera_topic and camera_topic not in topics:
            findings.append(Finding(LEVEL_ERROR, 'camera_missing',
                                    f'에피소드 {eid}: {camera_topic!r} 영상이 없다 (있는 것: {topics}).'))
        try:
            default_arm_joints(conn, eid, label=label, command_topic=command_topic)
        except ExportError as exc:
            findings.append(Finding(LEVEL_ERROR, 'arm_joints', f'에피소드 {eid}: {exc}'))
        except Exception as exc:  # noqa: BLE001 — 점검이 GUI 를 죽이면 안 된다
            findings.append(Finding(LEVEL_ERROR, 'arm_joints',
                                    f'에피소드 {eid}: 팔 관절 순서를 정할 수 없다 ({type(exc).__name__}: {exc})'))
    if len(cameras) > 1 and not camera_topic:
        findings.append(Finding(LEVEL_WARNING, 'cameras_vary',
                                f'에피소드마다 카메라 토픽이 다르다 {sorted(cameras)} — 한 데이터셋에 '
                                f'섞이면 배치 규약(camera_topic)이 어긋난다.'))
    if not has_errors(findings):
        findings.append(Finding(LEVEL_INFO, 'export_ok',
                                f'{len(episode_ids)} 에피소드 — frame clock · 팔 관절 확인'))
    return findings


def check_topics_file(config_path: str | Path, *,
                      reference: Optional[str | Path] = None) -> list[Finding]:
    """`dataset_config.yaml` 의 `topics_file` 이 녹화 목록과 **같은 파일**인지 본다 (설계안 §4.7).

    기준 파일은 인자 > `$RDFP_HOME/config/recording_topics.list` 다. 기준을 모르면 존재만 본다.
    """
    p = Path(config_path)
    try:
        raw = yaml.safe_load(p.read_text(encoding='utf-8')) or {}
    except (OSError, yaml.YAMLError) as exc:
        return [Finding(LEVEL_ERROR, 'config_unreadable', f'cannot read {p}: {exc}')]
    topics_file = raw.get('topics_file') if isinstance(raw, dict) else None
    if not topics_file:
        return [Finding(LEVEL_INFO, 'topics_all',
                        '`topics_file` 이 없다 — bag 의 모든 토픽을 적재 대상으로 본다.')]
    tf = Path(os.path.expanduser(os.path.expandvars(str(topics_file))))
    if not tf.is_absolute():
        tf = p.parent / tf
    if not tf.is_file():
        return [Finding(LEVEL_ERROR, 'topics_file_missing', f'topics_file not found: {tf}')]
    if reference is None:
        home = os.environ.get('RDFP_HOME')
        reference = Path(home) / REFERENCE_TOPICS_LIST if home else None
    if reference is None:
        return [Finding(LEVEL_INFO, 'topics_file_ok', f'topics_file: {tf}')]
    ref = Path(reference)
    if ref.is_file() and os.path.realpath(ref) == os.path.realpath(tf):
        return [Finding(LEVEL_INFO, 'topics_file_same',
                        f'적재 토픽 목록이 녹화 목록과 같은 파일이다: {tf}')]
    return [Finding(LEVEL_WARNING, 'topics_file_differs',
                    f'적재 토픽 목록({tf})이 녹화 목록({ref})과 다른 파일이다 — 갈리면 녹화한 것과 '
                    f'적재하는 것이 어긋난다.')]


__all__ = ['REFERENCE_TOPICS_LIST', 'check_db', 'check_export', 'check_import',
           'check_topics_file', 'overlapping_windows']
