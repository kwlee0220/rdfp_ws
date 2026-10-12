# 데이터셋 후처리기 설계서

**참조 문서:**
- [dataset_ingest_design.md](dataset_ingest_design.md)
- [session_control_guide.md](session_control_guide.md)

---

## 0. rosbag 저장 구조 — 운영 규칙

수집이 만드는 rosbag 의 **디렉터리 · 분할 · 재실행 규칙**이다. `scripts/record_rosbag.sh` 가 이 규칙대로 기록하고
§6 의 split 후보 선정이 이 구조를 전제한다. (옛 「rosbag2 운영 방안」 보고서에서 **구현된 부분만** 옮겼다 —
systemd 연계 · SQLite 인덱스 · retention 자동화는 구현하지 않았고 원문은
[../archive/rosbag2_operation_report.md](../archive/rosbag2_operation_report.md) 에 있다.)

루트는 환경변수 `RDFP_ROSBAG_DIR` (`.rdfprc` 기본 `/data/rdfp/rosbag`) 이고, `dataset_config.yaml` 의 `rosbag_dir` 이
같은 곳을 가리켜야 한다. ⚠️ **적재가 끝난 세션 디렉터리는 이 루트 밖으로 뺀다** — 시뮬레이터를 재기동하면 sim 시계가
0 으로 되감겨 에피소드 키가 회차 간에 겹치고, 함께 적재하면 새 에피소드의 관절 데이터가 **조용히 0 건**이 된다(§10.3).

### 0.1 권장 디렉터리 구조

상위 디렉터리는 날짜별로 구성하고, 그 아래에 실행 시각 기준 세션 디렉터리를 둔다.

```text
$RDFP_ROSBAG_DIR/   (기본 /data/rdfp/rosbag/)
├── 2026-04-11/
│   ├── session_2026-04-11_09-00-03/
│   │   ├── metadata.yaml
│   │   ├── session_2026-04-11_09-00-03_0.mcap
│   │   ├── session_2026-04-11_09-00-03_1.mcap
│   │   └── ...
│   └── session_2026-04-11_14-22-10/
│       ├── metadata.yaml
│       ├── session_2026-04-11_14-22-10_0.mcap
│       └── ...
├── 2026-04-12/
│   └── session_2026-04-12_00-05-41/
```

#### 0.1.1 split 파일명 규칙

rosbag2가 생성하는 split 파일의 이름은 **`-o` 옵션으로 지정한 경로의 basename**을 접두로 사용하고, 뒤에 split 인덱스(`_0`, `_1`, …)와 저장 포맷 확장자(`.mcap`)가 붙는다.

예를 들어 `-o $RDFP_ROSBAG_DIR/   (기본 /data/rdfp/rosbag/)2026-04-11/session_2026-04-11_09-00-03` 으로 기록하면, 생성되는 파일은 다음과 같다.

```text
session_2026-04-11_09-00-03_0.mcap
session_2026-04-11_09-00-03_1.mcap
...
```

즉, 본 운영 규칙에서는 **세션 디렉터리명과 bag basename이 동일**하다는 전제를 둔다. 이는 세션 디렉터리만 보아도 내부 split 파일을 식별할 수 있도록 하기 위함이다.

#### 0.1.2 구조의 장점

* 날짜 기준 보관/백업/삭제가 용이함
* 프로세스 재시작 시에도 세션 단위로 기록이 분리됨
* 세션 내부 파일은 1시간 단위로 관리 가능
* 추후 특정 시간 범위 조회 시 후보 세션을 빠르게 좁힐 수 있음

---

### 0.2 파일 분할 기준

세션 내부 파일은 **시간 기준(1시간)** 과 **크기 기준 상한** 을 함께 적용하여 split한다.

* `-d 3600` — 하나의 split 파일의 최대 지속 시간을 3600초(1시간)로 제한한다.
* `--max-bag-size <bytes>` — 하나의 split 파일의 최대 크기를 바이트 단위로 제한한다.

두 옵션을 함께 지정하면 **둘 중 먼저 도달한 조건**에서 split이 발생한다. 영상 토픽이 포함된 환경에서는 1시간 이내에도 파일이 과도하게 커질 수 있으므로, 크기 상한을 안전망으로 함께 거는 것을 권장한다.

크기 상한을 함께 두면 다음 효과가 있다.

* 손상·크래시 발생 시 영향 범위를 마지막 split 파일로 국한
* 파일시스템 및 전송 한계(네트워크 업로드 제한, NAS quota 등) 회피
* MCAP 인덱스 로딩·재생 시 메모리 부담 완화

권장값은 다음과 같다.

* 압축 영상 중심: 2~5 GB
* raw 영상(1080p 이상) 중심: 1~2 GB
* 저대역(`/delta_twist_stamp`, `/ee_pose_publisher/ee_pose` 등 상태 토픽 위주): 크기 상한 생략 가능

> 옵션 값은 바이트 정수만 지원되며 `5G` 와 같은 접미사는 허용되지 않는다. 스크립트에서는 `$((5 * 1024**3))` 형태로 계산하여 전달한다. 또한 값을 너무 작게 잡으면 split 파일 수가 폭증하여 메타데이터 오버헤드와 조회 비용이 늘어나므로 주의한다.

### 0.3 중간 종료 · 재실행

#### 0.3.1 기본 방침

`rosbag2`가 종료된 뒤 다시 실행될 경우, **기존 bag에 이어쓰기하지 않는다.**

#### 0.3.2 운영 규칙

재실행 시에는 항상 새로운 세션 디렉터리를 생성한다.

예:

```text
2026-04-11/session_2026-04-11_09-00-03/
2026-04-11/session_2026-04-11_14-22-10/
2026-04-11/session_2026-04-11_18-05-44/
```

#### 0.3.3 이유

이 방식은 다음과 같은 장점이 있다.

* bag 무결성 유지
* 세션 간 경계 명확
* 장애 발생 시 영향 범위 축소
* 조회 시 세션별 필터링 용이

---

## 1. 개요

### 1.1 목적

로봇이 동작하면서 발생하는 데이터는 rosbag2 (MCAP) 로 일단 기록된다. 이 rosbag 은 최종 저장소가 아니라 **정제 전 중간 저장소** 이며, 저장 용량 확보 등의 이유로 언제든 삭제될 수 있다. 본 후처리기는 rosbag 이 유효한 동안 오프라인으로 실행되어, 사용자가 지정한 범위의 메시지를 꺼내 **에피소드 단위로 분할**하고 다음 두 저장소에 적재한다.

- **PostgreSQL 데이터베이스** — 세션 메타데이터 및 메시지 타입별 시계열 테이블
- **에피소드별 MP4 파일** — 카메라 이미지 토픽

### 1.2 실행 형태

- **CLI 배치 프로그램** 이다. ROS 2 노드가 아니라 일반 Python 프로세스로 실행된다.
- `rclpy` 는 메시지 역직렬화와 메시지 타입 import 에만 사용하며, 노드를 스핀하지 않는다.
- 엔트리포인트: `ros2 run rdfp rosbag` (조회 계열) · `ros2 run rdfp dataset` (적재/DB 계열).

### 1.3 입력·출력

| 구분 | 대상 |
|---|---|
| 입력 | `<rosbag_dir>` 하위의 MCAP rosbag 아카이브 ([§0.1](#01-권장-디렉터리-구조) 구조) |
| 출력 (DB) | `sessions`, `pose_stampeds`, `twist_stampeds`, `joint_states`, `joint_trajectories`, `gripper_cmds`, `gripper_states` 테이블 |
| 출력 (MP4) | `<output_mp4_dir>/<episode_id>/<camera_topic>.mp4` + `<camera_topic>.metadata.json` + `<camera_topic>.frames.jsonl` |

### 1.4 전제

- Ubuntu 22.04, ROS 2 Humble.
- rosbag 내 모든 메시지의 `header.stamp` 는 유효하다.
- `/session` 토픽 (`rdfp_msgs/msg/SessionCommand`) 은 반드시 존재한다.
- 본 프로그램은 오프라인 실행이므로 실시간 제약은 없다.

### 1.5 기술 스택

| 영역 | 선택 |
|---|---|
| 언어 | Python 3.10+ |
| MCAP 리더 | `mcap`, `mcap-ros2-support` |
| 메시지 역직렬화 | `mcap_ros2` 가 MCAP 내장 IDL 스키마로 동적 역직렬화 (별도 `rdfp_msgs` import 불필요) |
| DB 드라이버 | `psycopg` (v3) — `executemany` 배치 삽입 |
| 설정 | `pydantic` (v2) + `PyYAML` |
| CLI | `argparse` (표준 라이브러리) |
| 영상 인코딩 | 기존 `rdfp.recorder.FFMpegMp4Recorder` 재사용 (ffmpeg subprocess) |
| 이미지 디코딩 | `opencv-python` (`cv2.imdecode`, 색상 변환) |
| 테스트 | `pytest`, 합성 MCAP fixture + ffmpeg 통합 테스트 |

### 1.6 의존성 설치

apt (`package.xml` 에 선언):

```bash
sudo apt install python3-yaml python3-opencv ffmpeg
```

pip (apt 버전이 없거나 구버전이어서 pip 로 설치):

```bash
pip install --user 'mcap' 'mcap-ros2-support' 'pydantic>=2' 'psycopg[binary]>=3'
```

apt 저장소의 `python3-pydantic` 은 v1, `python3-psycopg2` 는 v2 라서 본 프로젝트
요구사항과 맞지 않는다. `mcap` / `mcap-ros2-support` 는 apt 에 존재하지 않는다.
시스템 바이너리 `ffmpeg` / `ffprobe` 가 `PATH` 에 있어야 한다.

---

## 2. 용어

| 용어 | 정의 |
|---|---|
| 세션 메시지 | `/session` 토픽(`rdfp_msgs/msg/SessionCommand`) 에 포함된 각 메시지 |
| start_msg | `state == 'IN_EPISODE'` 인 세션 메시지 |
| start_ts | `start_msg.header.stamp` (에피소드 시작 시각) |
| stop_msg | `state == 'IN_SESSION'` 인 세션 메시지 중, **직전 세션 메시지의 `state == 'IN_EPISODE'`** 인 경우에 한정 |
| stop_ts | `stop_msg.header.stamp` (에피소드 종료 시각) |
| 에피소드 구간 | `[start_ts, stop_ts)` — **반열림 구간** |
| episode_id | 후처리기가 부여한 에피소드 식별자. `sessions` 테이블의 `BIGSERIAL id` 를 그대로 사용 |

세션 메시지 중 `state == 'IDLE'` 또는 "`IDLE → IN_SESSION`" 전이에서 나오는 `IN_SESSION` 메시지는 **stop_msg 가 아니다**.

---

## 3. 시스템 구성

### 3.1 데이터 흐름

```text
[<rosbag_dir>의 MCAP splits]
   │
   │  (1) catalog 스캔 (인덱스 우선, metadata.yaml fallback) → split 후보
   │      · metadata.yaml 이 있는 (녹화 종료된) 세션만 대상
   ▼
[선정된 split 후보]
   │  (2) mcap reader: (topic, stamp, payload_bytes)
   ▼
[지정 토픽 ∪ {/session} 메시지 스트림]
   │  (3) 역직렬화 + header.stamp 기준 스트리밍 머지
   ▼
[stamp 오름차순 메시지 스트림]
   │  (4) /session 상태 머신 → episodes[]
   │  (5) 에피소드 필터 (task_label / min_duration / dates)
   │  (6) Episode dispatcher: 각 메시지에 episode_id 할당
   │       └─ 품질 게이트: stamp 역행 / idle gap 감지 → JSONL 경고
   ▼
[(episode_id, msg_type, msg) 스트림]
   │  (7) 메시지 타입별 writer / 카메라 토픽별 mp4 sink
   ▼
[PostgreSQL tables]  +  [<output_mp4_dir>/<episode_id>/<camera>.mp4]
```

**실행 모드**: `parallelism=1` (기본) 이면 위 흐름을 단일 프로세스에서 스트리밍
으로 수행한다. `parallelism>1` 이면 에피소드 리스트까지만 공통으로 계산한 뒤,
각 에피소드를 독립 워커 프로세스에 분배해 병렬 적재한다 ([3.4](#34-병렬-처리-모드) 참조).

### 3.2 패키지 구성

후처리기는 `rdfp` 패키지의 `rdfp/dataset/`(적재 · 조회 · export) 와 `rdfp/rosbag/`(bag 카탈로그 · `rosbag` CLI) 에 있다.
아래는 2026-09-21 의 실제 트리다 (`tests/` 생략).

```text
src/rdfp/rdfp/dataset/
├── cli_common.py                # 공통 argparse (--config · --log-level · DSN 해석)
├── config.py                    # pydantic v2 DatasetConfig + YAML 로드 (환경변수 치환)
├── types.py · episode_key.py    # 공용 타입 · 에피소드 키 (start_sec, start_nanosec)
├── import_cmd.py                # `import` — 적재 전 과정을 인라인으로 (run_import 없음)
├── export_cmd.py                # `export` — 학습 샘플 트리 (E7)
├── stats_cmd.py · list_cmd.py · init_db_cmd.py · replay_cmd.py · replay_gui_cmd.py
├── ingest/
│   ├── pipeline.py              # 직렬 경로 (_run_ingestion) · 병렬 분기 (_run_parallel)
│   ├── episode_worker.py        # 병렬 모드의 에피소드 워커 (에피소드 = 트랜잭션)
│   ├── episode/detector.py · dispatcher.py · from_bag.py   # /session → 에피소드 창, 메시지 할당
│   ├── media/frame_router.py · ffmpeg_sink.py · mp4_image_recorder.py   # rosbag 이미지 → mp4 (FrameRouter 경로)
│   ├── media/recorder_registry.py                          # 레코더 mp4 · sidecar 등록 (I4, episode_key 로)
│   ├── quality_gate.py · run_log.py · exceptions.py
├── db/
│   ├── registry.py              # msg_type → (table, writer, reader) 바인딩
│   ├── connection.py · initialize.py · schema_check.py · topics.py · config.py
│   ├── writers/                 # 테이블별 INSERT (session_command · joint_state · motion_* · gripper_* · scene_objects · image_* …)
│   ├── readers/                 # 테이블별 SELECT (재생 · export 가 쓴다)
│   ├── topic_message_replayer.py · mp4_image_replayer.py · message_reader.py · image_message_reader.py
├── export/                      # E1~E7 — frame_clock · observation · motion_label · command_label · dataset_meta · writer
└── sql/schema.sql · drop.sql    # 스키마 정본 (§5.2)

src/rdfp/rdfp/rosbag/            # catalog.discover_splits · merged_stream · mcap_reader · cli (`rosbag` 명령)
```

엔트리포인트는 `setup.py` 의 `console_scripts` 에 **독립 명령**으로 등록한다 — `rosbag` · `import` · `replay` ·
`stats` · `export` · `list` · `init-db` · `replay_gui`. 새 기능은 `dataset <sub>` 서브커맨드가 아니라 형제
`*_cmd.py` 로 더한다.

### 3.3 CLI — 독립 명령 여덟

옛 `dataset <subcommand>` 단일 CLI 는 **독립 최상위 명령**으로 갈라졌다 (`ros2 run rdfp <명령>`).

| 명령 | 용도 | DB |
|---|---|:-:|
| `rosbag list-topics` · `list-episodes` · `export` · `delete` · `clear` | bag 카탈로그 조회 · 범위 적재 · 삭제 — [rosbag_cli_user_guide.md](rosbag_cli_user_guide.md) | 조회는 불필요 |
| `init-db` | 테이블 · 인덱스 · FK 생성 (`--drop --yes` 로 재생성) | ○ |
| `import` | 실제 적재 (`--dry-run` 은 에피소드 목록만) | ○ |
| `stats` · `list` | 테이블별 행 수 · 적재된 에피소드 목록 | ○ |
| `export` | DB → 학습 샘플 트리 (`.npz` + `metadata.json`) — [../learning/저장_형식.md](../learning/저장_형식.md) | ○ |
| `replay` · `replay_gui` | 적재된 에피소드를 라이브 토픽으로 재생 (ROS 필요) — [replay_mock_stack_guide.md](replay_mock_stack_guide.md) | ○ |

공통 옵션은 `cli_common.py` 가 공유한다 — `--config <dataset_config.yaml>` (경로 필드에 `${VAR:-default}` 치환) ·
`--log-level` · `init-db` 의 `--dsn-env`/`--schema`. 운영 절차는 [dataset_postproc_user_guide.md](dataset_postproc_user_guide.md).

### 3.4 병렬 처리 모드

설정의 `parallelism` 값이 2 이상이면 `multiprocessing.Pool` (spawn 컨텍스트) 로
에피소드를 병렬 처리한다.

- 선행 단계 (split 스캔, 세션 메시지 기반 에피소드 감지, 스키마 검증) 는 부모
  프로세스에서 한 번만 실행된다.
- 각 워커는 `episode_worker.process_episode()` 를 호출해 **자기 에피소드만**
  전담한다. 워커는 자신의 DB 커넥션과 `FrameRouter` 를 새로 열며,
  `mcap_reader` 의 time-range 필터로 해당 에피소드 구간만 읽는다.
- 에피소드 간 상태 공유가 없으므로 락·동기화가 필요하지 않다. 한 에피소드의
  실패는 다른 에피소드 처리에 영향을 주지 않는다 (`imap_unordered` 기반).
- pickle 경계를 넘기 위해 워커 인자는 모두 plain data (`PostProcConfig`,
  `Episode`, `list[str]`) 로 제한한다.

단일 프로세스 모드(`parallelism=1`) 는 `orchestrator._run_ingestion` 의 스트리밍
경로를 유지하며, 전환 비용 없이 기존 동작과 동일하다.

---

## 4. 범위 지정

### 4.1 기본 지정 방식

rosbag 은 import 의 임시 스테이징으로 취급된다. **시간 범위 필터는 제공하지 않으며**, 녹화가 종료된 (즉 `metadata.yaml` 이 쓰인) 세션의 모든 split 이 대상이 된다. 아직 녹화 중이거나 비정상 종료된 세션은 `metadata.yaml` 부재로 자동 제외된다.

| 항목 | 설명 |
|---|---|
| **topics** | 후처리 대상 토픽 화이트리스트. 빈 목록이면 rosbag 에 존재하는 모든 토픽이 대상. |

- **`/session` 자동 포함**: 사용자가 `topics` 에 `/session` 을 누락해도 파이프라인이 자동으로 추가한다. 누락되면 에피소드 감지가 불가능하기 때문이다.

### 4.2 부가 필터

| 필터 | 설명 |
|---|---|
| **task_label include / exclude** | 에피소드 시작 시점의 `task_label` 을 기준으로 포함·제외한다. |
| **min_duration_sec** | `stop_ts - start_ts < min_duration_sec` 인 에피소드는 건너뛴다. |
| **session_filter.dates** | `YYYY-MM-DD` 목록에 해당하는 날짜 디렉터리만 스캔한다. |

조합은 **AND** 로 동작한다.

### 4.3 설정 파일 (`dataset_config.yaml`)

> (2026-09-21) 별도의 `rosbag_config.yaml` 은 더 이상 없다 — `rosbag` CLI 도 `--config dataset_config.yaml` 또는 `--rosbag-dir` 를 받는다. 아래 `rosbag_config` 서술은 옛 구조다.

```yaml
rosbag_dir: /data/rosbag
output_mp4_dir: /data/postproc/videos

db:
  dsn_env: RDFP_DB_DSN             # 환경변수 이름. 값은 DSN 문자열.
  schema: public

topics:
  - /session
  - /ee_pose_publisher/ee_pose
  - /delta_twist_stamp
  - /joint_states
  - /delta_joint_cmds
  - /camera/image_raw

episode_filter:
  task_labels_include: [pick_and_place, pour_water]
  task_labels_exclude: [outlier_demo]
  min_duration_sec: 2.0

session_filter:
  dates: [2026-04-11]              # 선택

on_existing_episode: skip          # skip | replace | error

mp4:
  codec: h264
  nominal_fps: 30                  # CFR 로 선언할 프레임 레이트

# 운영 통합 옵션 (모두 선택)
catalog_index_path: /data/rosbag/_index.sqlite   # 운영 방안 10.5 SQLite 인덱스
parallelism: 4                     # 1 이면 단일 프로세스. >1 이면 에피소드 병렬 처리
quality_gate:
  stamp_regression: true           # header.stamp 역행 감지
  idle_gap_sec: 0.0                # >0 시 토픽별 유휴 갭 감지 (초)

# rosbag 을 임시 스테이징으로 사용할 때: import 완료 후 사용된 split 파일과
# 비게 된 세션 디렉터리를 제거한다. dry-run 에서는 무시된다.
delete_splits_after_import: false
```

### 4.4 설정 모델 (pydantic)

설정 파일은 `config.py` 의 pydantic 모델로 로드·검증된다. `db.dsn_env` 는 DSN
값이 아닌 **환경변수 이름** 을 가리키므로 YAML 에 자격증명이 남지 않는다.

```python
from __future__ import annotations
from typing import Literal
from pydantic import BaseModel, Field


class DbConfig(BaseModel):
    dsn_env: str = 'RDFP_DB_DSN'
    schema_: str = Field('public', alias='schema')


class EpisodeFilter(BaseModel):
    task_labels_include: list[str] = []
    task_labels_exclude: list[str] = []
    min_duration_sec: float = 0.0


class SessionFilter(BaseModel):
    dates: list[str] = []                 # YYYY-MM-DD


class Mp4Config(BaseModel):
    codec: str = 'h264'
    nominal_fps: int = 30


class QualityGateConfig(BaseModel):
    stamp_regression: bool = True
    idle_gap_sec: float = 0.0             # 0 = 비활성


class PostProcConfig(BaseModel):
    rosbag_dir: str
    output_mp4_dir: str
    db: DbConfig = DbConfig()
    topics: list[str] = []
    episode_filter: EpisodeFilter = EpisodeFilter()
    session_filter: SessionFilter = SessionFilter()
    on_existing_episode: Literal['skip', 'replace', 'error'] = 'skip'
    mp4: Mp4Config = Mp4Config()
    catalog_index_path: str | None = None
    parallelism: int = 1
    quality_gate: QualityGateConfig = QualityGateConfig()
    delete_splits_after_import: bool = False
```

- `topics` 가 빈 목록이면 "rosbag 에 존재하는 모든 토픽" 으로 확장되며,
  `/session` 은 자동 선행 포함된다.
- `catalog_index_path` 가 지정되고 파일이 존재하면 SQLite 인덱스가 우선 사용되고,
  실패 시 `metadata.yaml` 스캔으로 fallback 한다.

### 4.5 인증 정보

DSN 은 **설정 파일에 평문으로 기록하지 않는다**. `db.dsn_env` 에 환경변수 이름을 지정하고, 실제 값은 쉘/systemd 환경변수로 주입한다.

```bash
export RDFP_DB_DSN="postgresql://rdfp@localhost:5432/rdfp"
ros2 run rdfp import --config dataset_config.yaml
```

### 4.6 환경변수

| 이름 | 설명 | 필수 |
|---|---|---|
| `RDFP_DB_DSN` | PostgreSQL DSN. 예: `postgresql://user:pass@host:5432/dbname` | 필수 |
| `RDFP_POSTPROC_LOG_DIR` | 실행 요약 JSONL 로그 저장 경로. 기본값: `<output_mp4_dir>/_logs/`. | 선택 |
| `RDFP_POSTPROC_BATCH_SIZE` | DB 배치 삽입 버퍼 크기. 기본값: `1000`. | 선택 |

환경변수 이름은 `config.py` 의 상수로 선언하고 코드 내에서 직접 문자열을 참조하지 않는다.

---

## 5. 데이터베이스 모델

### 5.1 토픽 ↔ 테이블 매핑

`db/registry.py` 의 `MESSAGE_TYPE_REGISTRY` 가 **메시지 타입**으로 writer · reader · 테이블을 고른다 (2026-09-21 기준).

| 메시지 타입 | 대표 토픽 | 테이블 |
|---|---|---|
| `rdfp_msgs/SessionCommand` | `/session` | `sessions` (에피소드 = `IN_EPISODE → IN_SESSION` 전이) |
| `sensor_msgs/JointState` | `/joint_states` · `/target_joint_cmds` | `joint_states` — **관측과 명령이 같은 테이블, `topic_id` 로 갈린다**. `joint_names TEXT[]` (2026-09-18) |
| `rdfp_msgs/MotionTrajectory` | `/motion/trajectory` | `motion_trajectories` (R1 — 경유점 jsonb, `duration_sec` GENERATED) |
| `rdfp_msgs/MotionEvent` | `/motion/event` | `motion_events` (R2 — `(episode_id, operation_id)` 로 R1 과 짝) |
| `rdfp_msgs/GripperCommand` | `/gripper_cmds` | `gripper_cmds` (`goal` 심볼) |
| `rdfp_msgs/GripperState` | `/gripper_states` | `gripper_states` |
| `rdfp_msgs/SceneObjects` | `/scene/objects` | `scene_objects` (jsonb 배열) |
| `geometry_msgs/PoseStamped` | `/ee_pose` | `pose_stampeds` |
| `geometry_msgs/TwistStamped` | `/servo_node/delta_twist_cmds` | `twist_stampeds` |
| `control_msgs/JointJog` | `/servo_node/delta_joint_cmds` | `joint_jogs` |
| `rdfp_msgs/TargetJointStates` | (구형) | `target_joint_states` |
| `sensor_msgs/Image` | 녹화 목록의 이미지 토픽 (있을 때만) | DB 행 없음 → `FrameRouter` 가 mp4 로 만들고 `image_streams` · `image_frames` 를 넣는다 |
| — (레코더 mp4) | `rdfp_image_recorder` 의 출력 | `image_streams` · `image_frames` — `recordings_dir` 이 있으면 `RecorderRegistrar` 가 **`episode_key` 로** 등록 (I4) |

⚠️ **이미지 토픽이 녹화 목록에 있고 `recordings_dir` 도 켜져 있으면 `import` 가 죽는다** — 두 경로가 같은
`(episode_id, topic_id)` 로 `image_streams` 를 넣어 UNIQUE 위반이다. 영상은 rosbag 에 담지 않는 것이 현행이라
`config/recording_topics*.list` 에서 카메라 토픽을 뺐다 (2026-09-19). 모든 테이블에 `topics` 등록부가 붙고
`schema_check` 가 기동 때 존재를 검증한다. 신규 타입은 registry 에 한 줄 + writer/reader 로 붙는다 (§5.5).

### 5.2 스키마

**정본은 `src/rdfp/rdfp/dataset/sql/schema.sql` 이다** (`init-db` 가 그대로 적용한다; `drop.sql` 이 짝). 이 문서에
SQL 을 복사하지 않는다 — 2026-09-18 의 `joint_names` · `motion_*` 추가처럼 스키마가 바뀔 때 두 곳이 어긋났다.
2026-09-21 기준 테이블 14개:

| 테이블 | 무엇 | 비고 |
|---|---|---|
| `sessions` | 에피소드 한 건 — `(start_sec, start_nanosec)` **UNIQUE**, `id BIGSERIAL`, `task_label` · `success` · `metadata jsonb` | `success` 는 `stop_episode` 의 `outcome` (NULL = 판정 없음) |
| `topics` | 토픽 이름 · 타입 등록부 — 모든 시계열 행이 `topic_id` 로 가리킨다 | 영상은 rosbag 에 없어 `RecorderRegistrar` 가 upsert |
| `joint_states` | 관측 `/joint_states` 와 명령 `/target_joint_cmds` — `topic_id` 로 갈림, `joint_names TEXT[]` | 이름이 빈 행은 export 가 거부 |
| `motion_trajectories` | R1 실행 궤적 — 연산마다 한 행, 경유점 jsonb, `stamp` = t₀ | `duration_sec` GENERATED |
| `motion_events` | R2 연산 이벤트 — 결과 · 끝난 시각 · `truncated_at_sec` | `failed` 만 짝 궤적이 없다 (불변식) |
| `gripper_cmds` · `gripper_states` | 심볼 명령 · 연속 상태(`width` · `stalled` · `at_goal`) | |
| `scene_objects` | `/scene/objects` 스냅샷 (물체 배열 jsonb) | 관측 컬럼에는 넣지 않는다 — [scene_objects_observation_decision](../learning/scene_objects_observation_decision.md) |
| `pose_stampeds` · `twist_stampeds` · `joint_jogs` · `target_joint_states` | `/ee_pose` · servo 입력 둘 · 구형 목표 | |
| `image_streams` · `image_frames` | 에피소드 × 카메라 토픽의 mp4 한 벌과 프레임별 stamp — `(episode_id, topic_id, frame_index)` UNIQUE | 레코더 등록(I4) 또는 `FrameRouter` |

공통 규칙: 시각은 `stamp_sec` · `stamp_nanosec` + GENERATED `stamp_ts`; 자식 테이블은 `episode_id … ON DELETE CASCADE`
(그래서 `on_existing_episode: replace` 가 에피소드째 지우고 다시 넣는다); 기존 DB 에는 `ALTER TABLE … ADD COLUMN IF NOT EXISTS`
로 무손실 추가한다.

### 5.3 스키마 사전 검증

후처리 시작 시점에 다음을 확인하고, 하나라도 실패하면 **즉시 종료**한다.

1. `sessions` 테이블이 존재하고 필수 컬럼이 모두 있는가.
2. 범위 지정에 포함된 모든 토픽의 메시지 타입에 대해, `registry` 에 매핑된 테이블이 DB 에 실재하는가 (카메라 토픽 제외).
3. 각 비-session 테이블에 `episode_id` 컬럼이 존재하고 `sessions(id)` FK 가 정의되어 있는가.

검증은 `information_schema.columns`, `information_schema.table_constraints` 조회로 구현한다.

### 5.4 이유: `session_path` 컬럼 제외

rosbag 은 정제 후 삭제될 수 있는 **중간 저장소** 이므로, DB 레코드에 원본 세션 디렉터리 경로를 남길 필요가 없다. 재실행 멱등성은 `sessions.UNIQUE(start_sec, start_nanosec)` 로 확보한다 (에피소드 시작 시각은 ns 정밀도의 `header.stamp` 이므로 실질적으로 충돌하지 않는다).

### 5.5 Registry 설계

`registry.py` 는 메시지 타입 → `(테이블명, WriterBase 서브클래스)` 쌍을 중앙에서 관리한다. 행 변환 로직은 각 writer 의 `row_values()` 메서드에 포함되므로 별도 extractor 함수를 두지 않는다. 추가로 `TOPIC_OVERRIDE` 는 **토픽 이름 기준** 으로 타입 매핑을 재정의하며, `resolve_topic_entry(topic, type_name)` 가 override → 타입 기반 순으로 조회한다.

```python
from .db.writers.base import WriterBase
from .db.writers.gripper_command import GripperCommandWriter
from .db.writers.gripper_state import GripperStateWriter
from .db.writers.joint_state import JointStateWriter
from .db.writers.joint_trajectories import JointTrajectoriesWriter
from .db.writers.pose_stamped import PoseStampedWriter
from .db.writers.twist_stamped import TwistStampedWriter


SESSION_COMMAND_TYPE = 'rdfp_msgs/msg/SessionCommand'


MESSAGE_TYPE_REGISTRY: dict[str, tuple[str, type[WriterBase]]] = {
    'geometry_msgs/msg/PoseStamped':        ('pose_stampeds',      PoseStampedWriter),
    'geometry_msgs/msg/TwistStamped':       ('twist_stampeds',     TwistStampedWriter),
    'sensor_msgs/msg/JointState':           ('joint_states',       JointStateWriter),
    'trajectory_msgs/msg/JointTrajectory':  ('joint_trajectories', JointTrajectoriesWriter),
    'rdfp_msgs/msg/GripperCommand':         ('gripper_cmds',       GripperCommandWriter),
    'rdfp_msgs/msg/GripperState':           ('gripper_states',     GripperStateWriter),
}


TOPIC_OVERRIDE: dict[str, tuple[str, type[WriterBase]]] = {
    # 기본 비어 있음. 같은 메시지 타입을 토픽에 따라 서로 다른 테이블로 보내야 할 때
    # '/topic_name': ('table_name', SomeWriter) 형태로 항목을 추가한다.
}


IMAGE_MESSAGE_TYPES: frozenset[str] = frozenset({
    'sensor_msgs/msg/Image',
    'sensor_msgs/msg/CompressedImage',
})
```

- `SessionCommand` 는 `SessionWriter` 가 별도 API (`insert_episode`, `find_existing`, `delete_by_id`) 로 처리하므로 위 딕셔너리에 포함되지 않는다.
- 이미지 타입은 DB 가 아니라 `FrameRouter` → `FfmpegSink` 로 라우팅된다.
- `TOPIC_OVERRIDE` 의 value 는 `(table_name, writer_cls)` 이며, 동일 writer 클래스를 다른 테이블에 쓸 수 있도록 `WriterBase.__init__(..., table=...)` 로 테이블명을 인스턴스 단위로 바인딩한다 (클래스 속성은 건드리지 않음).

### 5.6 Writer 인터페이스

`db/writers/base.py` 는 에피소드 경계에서 flush 되는 배치 INSERT 의 공통 베이스를 정의한다.

```python
class WriterBase:
    """에피소드 경계에서 flush 되는 배치 INSERT writer."""

    table: str = ''
    columns: tuple[str, ...] = ()

    def __init__(self, conn, batch_size: int = 1000) -> None:
        self._conn = conn
        self._buffer: list[tuple] = []
        self._batch_size = batch_size
        self._inserted_since_reset: int = 0

    def append(self, episode_id: int, msg) -> None:
        """메시지를 버퍼에 추가한다. 임계 초과 시 flush."""
        ...

    def flush(self) -> None:
        """버퍼를 executemany 로 일괄 INSERT 하고 카운터를 증가시킨다."""
        ...

    def consume_inserted_count(self) -> int:
        """직전 reset 이후 INSERT 된 행 수를 꺼내고 카운터를 0 으로 리셋."""
        ...

    def drop_pending(self) -> None:
        """ROLLBACK 시 버퍼에 남아있던 행을 폐기."""
        ...

    def row_values(self, episode_id: int, msg) -> tuple:
        """서브클래스에서 컬럼 순서대로 튜플 생성."""
        raise NotImplementedError
```

- 에피소드 커밋 직전 순서: 모든 writer 에 대해 `flush()` → `conn.commit()` → `consume_inserted_count()` 로 summary 집계.
- 롤백 시 순서: `drop_pending()` → `conn.rollback()`.
- `SessionWriter` 는 메시지 단위 append 대신 에피소드 레코드 자체를 INSERT/DELETE 하는 별도 API 를 제공한다.

---

## 6. rosbag 읽기 전략

### 6.1 Split 후보 선정

1. `catalog.discover_splits()` 가 디스패처 역할을 한다.
   - 설정 `catalog_index_path` 가 지정되고 파일이 존재하면 `catalog_index.scan_via_index()`
     를 먼저 호출한다. 운영 방안 [10.5 SQLite 인덱스](dataset_ingest_design.md#105-인덱스-저장-형식)
     스키마 (`sessions`, `splits`, `split_topics`) 를 그대로 사용하며, 날짜 + 토픽 조건
     쿼리로 split 을 선정한다.
   - 인덱스가 없거나 스키마 불일치면 `catalog.scan_rosbag_dir()` 로 fallback —
     `<rosbag_dir>` 하위를 날짜 폴더 → 세션 디렉터리 순으로 순회하며 각 세션의
     `metadata.yaml` 을 파싱한다. **`metadata.yaml` 이 없는 세션** (녹화 중 또는
     비정상 종료) **은 자동으로 제외된다**.
2. `session_filter.dates` 가 지정된 경우 날짜 단계에서 선행 필터링한다 (인덱스·
   fallback 양쪽 동일 규약).
3. 사용자 시간 범위 필터는 존재하지 않는다 — 선정된 세션의 모든 split 이 대상이다
   (에피소드는 `/session` 상태 머신으로 자동 분절된다).

### 6.2 Split 내부 읽기

- 리더: `mcap` + `mcap-ros2-support`.
- 각 split 을 토픽 필터로 1차 필터링. 에피소드 단위 병렬 처리 시 내부적으로
  에피소드 경계 `[start_ns, stop_ns)` 를 사용해 `header.stamp` 기반 범위 필터도
  적용한다.
- 역직렬화는 `mcap_ros2` 가 MCAP 에 내장된 ROS 2 IDL 스키마로부터 동적으로 수행
  하므로 `rdfp_msgs` 등 소스 메시지 패키지를 파이썬 런타임에 설치할 필요가 없다.
- 타입이 `registry` 에 없는 토픽은 경고 로그 후 무시.
- 정렬 기준은 반드시 `msg.header.stamp`. bag 의 `log_time` (수신 시각) 과 다를 수 있으므로,
  split 내부에서 `header.stamp` 로 안정 정렬 후 yield 한다.

### 6.3 여러 Split 스트리밍 머지

- `heapq.merge` 로 split 별 iterator 를 stamp 키로 머지한다.
- 동일 stamp 가 여러 토픽에서 발생할 경우, 토픽 이름 오름차순으로 tie-break 한다.
- 세션 경계(다른 세션 디렉터리) 를 넘을 때는 `/session` 의 TRANSIENT_LOCAL 특성상 새 세션의 초기 상태 메시지가 먼저 도착한다는 가정으로 동작한다. 가정이 깨지면 [11장](#11-오류-처리) 에 따라 경고·폐기로 방어한다.

---

## 7. 에피소드 감지

### 7.1 알고리즘

세션 메시지만 stamp 오름차순으로 훑으며 직전 `state` 를 유지한다.

```
prev_state = None
current_start = None
current_task  = None
episodes = []                       # [(start_ns, stop_ns, task_label), ...]

for msg in session_messages_in_stamp_order:
    state = msg.state
    stamp = to_ns(msg.header.stamp)

    if state == 'IN_EPISODE' and prev_state != 'IN_EPISODE':
        # start_msg
        current_start = stamp
        current_task  = msg.task_label or None

    elif state == 'IN_SESSION' and prev_state == 'IN_EPISODE':
        # stop_msg — 요구사항 정의에 정확히 부합
        episodes.append((current_start, stamp, current_task))
        current_start = None
        current_task  = None

    # 그 외 전이 (IDLE → IN_SESSION, IDLE 유지 등) 는 무시

    prev_state = state
```

`state == 'IN_SESSION'` 이면서 직전 state 가 `IN_EPISODE` 인 경우에만 stop_msg 이다. `IDLE → IN_SESSION` 전이의 `IN_SESSION` 메시지는 stop_msg 가 아니다.

### 7.2 경계 포함 규칙

- 구간은 **`[start_ts, stop_ts)`** 반열림으로 처리한다.
- `stop_ts` 와 동일 stamp 를 가진 `/session` 외 메시지는 해당 에피소드에 **포함하지 않는다**.

### 7.3 `task_label` 고정

에피소드 시작 시점의 `task_label` 을 해당 에피소드의 `task_label` 로 고정한다. 노드 규약 ([session_control_guide 2.3](session_control_guide.md)) 에 따라 `IN_EPISODE` 중에는 변경되지 않으므로 중간 재샘플링은 불필요하다. 빈 문자열(`""`) 이면 `task_label = NULL`.

### 7.4 비정상 상황 처리

| 케이스 | 처리 |
|---|---|
| bag 시작 시 `/session` 이 한 번도 등장하지 않음 | **즉시 실패** (에피소드 분할 불가) |
| 마지막 세션 메시지가 `IN_EPISODE` 로 끝나고 종료 경계 없음 | 경고. 해당 에피소드는 **폐기** (불완전). |
| `IN_EPISODE → IDLE` (중간 `IN_SESSION` 없이) | 방어적으로, 마지막 `IN_EPISODE` 메시지의 stamp 를 stop 으로 간주하고 경고. |
| 동일 `(start_sec, start_nanosec)` 가 이미 DB 에 존재 | `on_existing_episode` 설정에 따름. |

---

## 8. 메시지 → 에피소드 할당

### 8.1 정책

에피소드 리스트(stamp 오름차순) 와 메시지 스트림을 동시에 훑는 **sweep-line** 방식으로 할당한다.

```python
def assign_episodes(msg_stream, episodes):
    """
    episodes : [(episode_id, start_ns, stop_ns), ...]  (stamp 오름차순)
    msg_stream: (stamp_ns, topic, msg) generator       (stamp 오름차순)
    """
    i = 0
    for stamp_ns, topic, msg in msg_stream:
        while i < len(episodes) and stamp_ns >= episodes[i][2]:
            i += 1
        if i >= len(episodes):
            break
        ep_id, start_ns, stop_ns = episodes[i]
        if start_ns <= stamp_ns < stop_ns:
            yield ep_id, topic, msg
```

시간 복잡도 O(N + E), 메모리 O(E).

### 8.2 배치 삽입 · 에피소드 이벤트

- 토픽별 writer 는 내부에 버퍼(기본 1000건) 를 두고 `executemany` 로 flush 한다.
- Dispatcher 는 에피소드가 바뀌는 순간 **에피소드 전환 이벤트** 를 전체 writer 와 video sink 에 브로드캐스트한다. writer 는 이전 에피소드의 버퍼를 flush, video sink 는 이전 mp4 를 닫고 새 mp4 를 연다.

### 8.3 트랜잭션 경계

- **에피소드 단위로 트랜잭션** 을 분리한다.
  1. `sessions` 에 에피소드 행 INSERT → `RETURNING id` 로 `episode_id` 확보.
  2. 하위 테이블에 메시지 INSERT (batch).
  3. 카메라 mp4 인코딩 완료 · metadata.json + frames.jsonl 기록.
  4. COMMIT.
- 중간 실패 시 해당 에피소드의 DB 행 / mp4 디렉터리를 **함께 롤백** 한다. mp4 는 `tempfile` → `os.rename` 으로 원자적 공개를 구현한다.

---

## 9. 카메라 이미지 → MP4

> **현행 경로는 레코더 등록(I4)이다 (2026-09-18).** 영상은 rosbag 에 담지 않고 `rdfp_image_recorder` 가 에피소드마다
> mp4 + sidecar(`frame_index` · `header.stamp`) + metadata(`episode_key`) 를 남기며, 적재가 `recordings_dir` 을 훑어
> **`episode_key` 로**(파일 이름의 벽시계가 아니라) 에피소드에 붙인다 — `ingest/media/recorder_registry.py`.
> sidecar 줄 수 ≠ `nframe` 이면 예외다. 아래 §9.1~§9.5 는 rosbag 에 이미지 토픽이 있을 때만 도는 `FrameRouter` 경로다
> — 두 경로를 함께 켜면 UNIQUE 위반으로 죽는다(§5.1).

### 9.1 출력 구조

```text
<output_mp4_dir>/
├── <episode_id>/
│   ├── <camera_topic_sanitized>.mp4
│   ├── <camera_topic_sanitized>.metadata.json
│   ├── <camera_topic_sanitized>.frames.jsonl
│   └── ...
```

- `<episode_id>` 는 DB 의 `sessions.id` 와 동일. 폴더명은 `episode_{id:08d}` 사용.
- 토픽명에서 `/` 는 `_` 로 치환한다 (예: `/camera/image_raw` → `camera_image_raw`).
- 에피소드×카메라 조합마다 mp4 + metadata.json + frames.jsonl 세 파일을 생성한다.

### 9.2 지원 입력 타입

| 입력 타입 | 처리 |
|---|---|
| `sensor_msgs/msg/Image` | 픽셀 포맷에 따라 BGR/RGB ndarray 로 변환 후 인코더에 전달. |
| `sensor_msgs/msg/CompressedImage` | `cv2.imdecode` 등으로 **압축 해제** 후 ndarray 로 변환, 이후 `Image` 와 동일 경로로 인코딩. |

카메라 토픽 지정은 **와일드카드를 지원하지 않는다**. 사용자가 `dataset_config.yaml` 의 `topics` 에 정확한 토픽명을 명시해야 한다.

### 9.3 CFR-passthrough 인코딩

MP4 는 다음 규칙으로 기록한다.

- **프레임 1:1 전달**: 수신된 프레임을 **순서대로, 드롭·중복 없이** 인코더에 전달한다.
- **CFR 선언**: 출력 mp4 의 컨테이너 프레임 레이트는 `mp4.nominal_fps` (기본 30) 로 **고정 프레임 레이트로 선언** 한다. 실제 수신 간격이 불규칙해도 플레이어는 일정 속도로 재생한다.
- **리타이밍 없음**: 원본 `header.stamp` 에 맞추기 위한 프레임 중복 삽입 · 드롭을 **수행하지 않는다**.
- 코덱은 H.264 (libx264) 를 기본값으로 한다. 기존 `rdfp.recorder.ffmpeg_mp4_recorder` 를 재사용한다.

### 9.4 metadata.json + frames.jsonl

mp4 의 CFR 선언은 편의적인 재생용이고, **정확한 프레임 시각은 frames.jsonl 이 유일한 진실원천** 이다. 글로벌 메타와 프레임별 stamp 는 두 파일로 분리된다.

#### 9.4.1 `<base>.metadata.json` (글로벌 메타)

```json
{
  "episode_id": 42,
  "topic": "/camera/image_raw",
  "encoding": "h264",
  "container_fps": 30,
  "frame_id": "camera_optical_frame",
  "nominal_frame_count": 1234,
  "width": 1920,
  "height": 1080,
  "created_at": "2026-04-19T10:15:03+09:00"
}
```

- `frame_id` 는 첫 유효 메시지의 `header.frame_id` (카메라당 불변 가정).
- `nominal_frame_count` 는 `frames.jsonl` 의 라인 수와 일치해야 한다 (1:1 불변식).
- `created_at` / `width` / `height` 는 `finalize()` 시점에 채워진다.

#### 9.4.2 `<base>.frames.jsonl` (프레임별 stamp)

라인 단위 JSON. 라이브 녹화의 `Mp4ImageRecorder` 사이드카와 **동일 키 포맷**.

```jsonl
{"frame_index": 0, "stamp": {"sec": 1712345678, "nanosec": 123456789}}
{"frame_index": 1, "stamp": {"sec": 1712345678, "nanosec": 157201234}}
{"frame_index": 2, "stamp": {"sec": 1712345678, "nanosec": 190876543}}
```

- `frame_index` 는 mp4 내 0-기반 프레임 인덱스.
- `stamp.sec` / `stamp.nanosec` 은 원본 `header.stamp`.
- 매 프레임 즉시 flush 되므로 `finalize()` 가 호출되지 못해도 부분 데이터가 보존된다.
- 소비자는 프레임 N 의 실제 시각을 이 파일의 N-번째 라인에서 조회한다.

### 9.5 에피소드·카메라별 sink 수명

- 에피소드 진입 시 dispatcher 가 해당 에피소드의 카메라별 ffmpeg sink 를 **지연 생성** (첫 프레임 도착 시).
- 에피소드 종료 시 dispatcher 가 모든 카메라 sink 를 닫고 metadata.json 을 기록한다 (frames.jsonl 은 매 프레임 단위로 이미 flush 됨).
- 카메라 토픽이 에피소드 중에 한 번도 발행되지 않으면 해당 조합의 mp4 / metadata / frames 모두 생성되지 않는다.
- 트랜잭션 실패 시 sink 의 `abort()` 가 mp4 / frames.jsonl / metadata.json 세 파일을 모두 정리한다.

---

## 10. 재실행 · 멱등성

### 10.1 중복 처리 정책 (`on_existing_episode`)

| 값 | 동작 |
|---|---|
| `skip` (기본) | 동일 `(start_sec, start_nanosec)` 가 `sessions` 에 이미 있으면 해당 에피소드 전체(메시지 + mp4)를 건너뜀 |
| `replace` | 기존 `sessions` 행을 삭제 (FK `ON DELETE CASCADE` 로 자식 테이블도 제거) 후 mp4 디렉터리를 삭제하고 재삽입 |
| `error` | 즉시 실패 |

### 10.3 ⚠️ sim 시계 겹침 — 회차가 다른 bag 을 같은 `rosbag_dir` 에 두지 않는다

에피소드 키가 `(start_sec, start_nanosec)` 이므로 **시뮬레이터를 재기동해 sim 시계가 0 으로 되감기면 회차 간에 키가
겹칠 수 있다.** 2026-09-20 실측 — 옛 bag(sim 31.9~896.6 s)과 새 bag(701.1~779.4 s)을 함께 적재하니 새 에피소드 둘이
`joint_states` · `motion_*` · `gripper_*` 전부 **0 건**으로 커밋됐고, 이미지만 sidecar 의 `episode_key` 로 정상 등록돼
겉보기에 멀쩡했다. 같은 bag 을 혼자 적재하면 정상이다. 그래서 **적재가 끝난 세션 디렉터리는 `rosbag_dir` 밖으로 뺀다**
(`/data/rdfp/rosbag_imported/` 로 옮기거나 `ros2 run rdfp rosbag delete`). 확인은 적재 후 `joint_states` 가 0 인
에피소드가 있는지 세는 것이다.

### 10.2 장애 복구

- 에피소드 단위 트랜잭션 덕분에, 중간 장애로 부분 적재된 에피소드는 `replace` 로 재실행하면 정상화된다.
- 진행 상황 로그(JSONL) 에 `(episode_id, row_counts, mp4_files)` 를 남겨 재현성을 확보한다.

---

## 11. 오류 처리

### 11.1 실패·스킵 규칙

| 상황 | 동작 |
|---|---|
| `<rosbag_dir>` 이 없거나 읽기 권한 없음 | 즉시 실패 |
| `<rosbag_dir>` 하위에 `/session` 메시지가 한 번도 없음 | 즉시 실패 |
| 범위에 포함된 토픽의 테이블이 DB 에 없음 (카메라 제외) | 즉시 실패 |
| 메시지 타입이 `registry` 에 없음 | 경고 후 해당 토픽 무시 |
| 단일 메시지 역직렬화 실패 | 경고 후 해당 메시지만 스킵 |
| `JointTrajectory.points` 가 빈 배열 | 경고 후 해당 메시지 스킵 |
| DB 커넥션 단절 | 최대 N회 재시도 후 실패 (에피소드 트랜잭션 롤백) |
| 이미지 디코딩 실패 | 경고 후 해당 프레임만 스킵 (에피소드는 유지) |
| mp4 인코딩 실패 | 해당 에피소드 트랜잭션 롤백, 다음 에피소드로 계속 |

### 11.2 로깅 규약

- 로거 이름: `rdfp.postproc` 루트 아래 모듈별 서브로거.
- 레벨: 기본 `INFO`. CLI 공통 옵션 `--log-level debug` 로 상향 가능.
- 포맷: `%(asctime)s [%(levelname)s] %(name)s: %(message)s`
- 메시지는 **영어** 로 작성한다 (프로젝트 규약).

### 11.3 실행 요약 JSONL

각 에피소드 처리가 끝날 때마다 한 줄씩 append 한다. 기본 경로는
`<output_mp4_dir>/_logs/postproc_run.jsonl` 이며, `RDFP_POSTPROC_LOG_DIR`
환경변수로 재정의할 수 있다.

**`episode_done`** — 정상 커밋:
```json
{"ts":"2026-04-19T10:15:03+09:00","event":"episode_done","episode_id":42,
 "start_ts":"2026-04-11T13:10:00+09:00[ns=123456789]",
 "stop_ts":"2026-04-11T13:12:30+09:00[ns=987654321]",
 "task_label":"pick_and_place",
 "row_counts":{"pose_stampeds":12345,"twist_stampeds":6789,"joint_states":45678},
 "mp4_files":["episode_00000042/camera_image_raw.mp4"]}
```

**`episode_failed`** — 트랜잭션 롤백된 실패:
```json
{"ts":"...","event":"episode_failed",
 "start_ts":"...","stop_ts":"...","task_label":"x","error":"..."}
```

**`episode_skipped`** — `on_existing_episode=skip` 으로 건너뜀:
```json
{"ts":"...","event":"episode_skipped","episode_id":7,
 "start_ts":"...","stop_ts":"...","task_label":"..."}
```

**`quality_warning`** — 품질 게이트 감지 ([12장](#12-품질-게이트) 참조):
```json
{"ts":"...","event":"quality_warning","episode_id":42,
 "kind":"stamp_regression","topic":"/ee_pose_publisher/ee_pose",
 "stamp_ns":1712345678123456789,"prev_stamp_ns":1712345678500000000}
```

`stamp_*` 필드의 `[ns=...]` 보조 표기는 표준 datetime 이 마이크로초까지만 표현하는 한계를 보완하여 나노초 정밀도를 유지하기 위한 것이다.

---

## 12. 품질 게이트

`quality_gate.QualityGate` 는 각 메시지의 `header.stamp` 시퀀스를 관찰해
품질 문제를 감지한다. **에피소드 자체를 제외하거나 실패시키지 않는다** —
감지 결과는 JSONL 실행 로그에 `event='quality_warning'` 레코드로만 남긴다.

### 12.1 감지 항목

| 종류 | 조건 | 설정 키 |
|---|---|---|
| `stamp_regression` | 같은 토픽에서 직전 메시지보다 `stamp` 가 작아짐 | `quality_gate.stamp_regression` (bool) |
| `idle_gap` | 같은 토픽의 인접 메시지 간 `stamp` 차이가 임계 초과 | `quality_gate.idle_gap_sec` (>0 시 활성) |

### 12.2 JSONL 경고 레코드

```json
{"ts":"2026-04-19T10:15:03+09:00","event":"quality_warning",
 "episode_id":42,"kind":"stamp_regression","topic":"/ee_pose_publisher/ee_pose",
 "stamp_ns":1712345678123456789,"prev_stamp_ns":1712345678500000000}
```

실행 요약 summary 의 `warnings` 카운터도 함께 증가한다. 대량 경고가 관찰되면
해당 bag 의 QoS 설정·시계 동기화 상태를 점검한다.

### 12.3 범위

품질 게이트는 에피소드 단위로 상태를 초기화한다 (토픽별 마지막 stamp 를 비움).
병렬 모드에서는 각 워커가 자체 `QualityGate` 를 가지며, 서로 간섭하지 않는다.

---

## 13. 성능 참고

| 항목 | 기준값 | 비고 |
|---|---|---|
| DB 배치 삽입 버퍼 | 1000행 | 레이턴시 vs. 메모리 트레이드오프 |
| MCAP 스트리밍 머지 heap 크기 | split 개수 만큼 | 수십 개 이하면 무시 가능 |
| 이미지 디코딩 | 에피소드 단위 프로세스 풀 (`parallelism>1`) | `cv2.imdecode` CPU-bound |
| mp4 인코딩 | libx264 preset `medium` (기본) | 용량·CPU 균형 |

단일 워크스테이션에서 30 분 분량 bag 처리 시간은 이미지 포함 기준 bag 재생 시간의 0.3 ~ 0.6 배가 목표이다. 병렬 모드는 대체로 **에피소드 수 × 워커 비율** 에 비례해 단축되며, I/O 경합이 심한 SSD 환경에서는 CPU 코어 수보다 2 ~ 4 워커 정도가 적정하다.

---

## 14. 알려진 제약

- `use_sim_time` 환경에서 기록된 bag 의 `header.stamp` 는 sim time 일 수 있다. 본 후처리기는 이를 구분하지 않고 그대로 stamp 로 취급한다. 필요 시 사용자가 별도 관리한다.
- 시계 점프·역행 메시지는 `quality_gate` 가 경고만 남기고 그대로 저장한다 (에피소드 제외 없음).
- 매우 큰 단일 에피소드 (>1GB mp4) 는 ffmpeg 버퍼링 이슈가 있을 수 있다. 필요 시 mp4 split 옵션을 향후 기능으로 검토한다.
- 병렬 모드에서는 각 워커가 split 범위를 재스캔하므로 에피소드 수가 적고 split 수가 많을수록 상대적 이득이 줄어든다 (아주 짧은 rosbag 은 `parallelism=1` 이 유리).

---

## 15. 테스트 전략

### 15.1 단위 테스트

- `episode.detector` — 합성 `/session` 메시지 시퀀스 각 케이스에 대해 경계 추출 검증.
  - 정상: `IDLE → IN_SESSION → IN_EPISODE → IN_SESSION → IDLE`
  - 2단 전이 (stop_session): `IN_EPISODE → IN_SESSION → IDLE`
  - 무효 전이: `IDLE → IN_SESSION → IDLE` (stop_msg 아님)
  - 세션 중 여러 에피소드 연속 생성
  - bag 끝에 미완결 에피소드 (폐기 여부)
- `episode.dispatcher` — 다양한 stamp 분포에서 sweep-line 이 정확히 할당, 빈 에피소드도 `enter`/`exit` 를 발행하는지.
- 각 writer — 메시지 → `row_values` 튜플 변환, 배치 임계, `consume_inserted_count` / `drop_pending`.
- `mcap_reader` — 토픽 필터·시간 필터 동작.
- `quality_gate` — stamp 역행 / idle gap / 토픽별 독립 / 에피소드 reset.
- `catalog_index` — SQLite 인덱스 읽기, 누락·스키마 불일치 시 `IndexUnavailable`.
- `cli` — argparse 파싱, `list-episodes` text/json 출력 포맷.

### 15.2 통합 테스트

- 소형 합성 MCAP 을 fixture 로 포함 (수 초 길이, 2개 에피소드, 여러 토픽).
- 합성 이미지를 포함한 bag 에서 `FrameRouter` → 실제 ffmpeg 호출까지 수행해 mp4 생성 및 `ffprobe` 로 프레임 수·해상도 검증 (테스트는 ffmpeg/ffprobe 가 없으면 자동 skip).
- DB 쪽 통합은 Docker `postgres:15` 를 pytest fixture 로 기동하여 `schema.sql` 실행 → 실제 INSERT / FK cascade / `replace` 정책 검증 (운영 환경에서 수행).

### 15.3 수동 스모크

- 실 운영 bag 중 30 분 이하 샘플로 end-to-end 실행.
- DB 에피소드 행 수 · mp4 duration 이 에피소드 경계와 일치하는지 확인.
- 동일 bag 으로 2회 실행해 `skip` / `replace` 정책이 기대대로 동작하는지 확인.
- `parallelism=1` vs `>1` 결과 (에피소드 수, row 수, mp4 수) 가 동일한지 비교.

### 15.4 회귀

- 합성 MCAP + 기대 DB snapshot (pg_dump 일부) 을 gold 로 저장하여 CI 에서 비교.
