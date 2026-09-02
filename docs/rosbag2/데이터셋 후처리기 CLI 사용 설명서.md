# 데이터셋 후처리기 CLI 사용 설명서

rosbag2 MCAP 아카이브를 **에피소드 단위** 로 분할하여 PostgreSQL 에 적재하고, 카메라 토픽은 MP4 + `metadata.json` (글로벌 메타) + `frames.jsonl` (프레임별 stamp) 로 내보내는 배치 프로그램의 사용 방법을 설명한다. CLI 는 관심사에 따라 **`rosbag`** (rosbag 조회) 과 **`dataset`** (DB 적재/조회) 두 개의 실행 파일로 분리되어 있다.

동작 원리·내부 구조는 [데이터셋 후처리기 설계서](./데이터셋%20후처리기%20설계서.md) 를 참고한다. 본 문서는 **실행 명령과 옵션** 에 집중한다.

---

## 1. 개요

두 개의 ROS 2 실행파일이 있다.

**`rosbag`** — rosbag 을 조회 및 관리.

| 서브커맨드 | 용도 |
|---|---|
| [`list-episodes`](#43-list-episodes) | 실제 쓰기 없이 감지된 에피소드 목록만 출력 |
| [`list-topics`](#46-list-topics) | rosbag 에 기록된 모든 토픽 요약 출력 (ls -l 스타일) |
| [`topic-info`](#45-topic-info) | 지정 토픽의 split 별 메시지 수·타입·시각 범위 출력 |
| [`clear`](#47-clear) | rosbag 루트 아래의 모든 세션 데이터 삭제 (**파괴적**) |

**`dataset`** — DB 로 적재된 데이터셋을 만들거나 조회.

| 서브커맨드 | 용도 |
|---|---|
| [`init-db`](#41-init-db) | 필요한 DB 테이블·인덱스·FK 생성 |
| [`import`](#42-import) | rosbag → DB / MP4 실제 적재 |
| [`stats`](#44-stats) | 현재 DB 에 적재된 테이블별 행 수 출력 |

공통 구조:

```
ros2 run rdfp rosbag  [--log-level LEVEL] <서브커맨드> [옵션...]
ros2 run rdfp dataset [--log-level LEVEL] <서브커맨드> [옵션...]
```

---

## 2. 사전 준비

### 2.1 시스템 요구사항

- Ubuntu 22.04, ROS 2 Humble
- PostgreSQL 12 이상 (로컬 또는 원격)
- `ffmpeg`, `ffprobe` (시스템 PATH)

### 2.2 Python 의존성 설치

apt:
```bash
sudo apt install python3-yaml python3-opencv ffmpeg
```

pip (apt 버전이 맞지 않거나 부재):
```bash
pip install --user 'mcap' 'mcap-ros2-support' 'pydantic>=2' 'psycopg[binary]>=3'
```

### 2.3 빌드 및 환경 로드

```bash
cd ~/development/ros/rdfp_ws
colcon build --packages-select rdfp
source install/setup.bash
```

### 2.4 DB 연결 정보 설정

DSN 을 **환경변수** 로 주입한다 (YAML 에 평문으로 적지 않는다).

```bash
export RDFP_DB_DSN="postgresql://rdfp:rdfp@localhost:5432/rdfp"
```

### 2.5 설정 파일 준비

샘플 설정을 복사 후 편집한다.

```bash
# rosbag CLI 용
cp docs/rosbag2/rosbag_config.sample.yaml /etc/rdfp/rosbag_config.yaml
vi /etc/rdfp/rosbag_config.yaml

# dataset CLI 용
cp docs/rosbag2/dataset_config.sample.yaml /etc/rdfp/dataset_config.yaml
vi /etc/rdfp/dataset_config.yaml
```

최소 편집 대상:

- `rosbag_dir` — rosbag 루트 (두 설정 모두 필수)
- `output_mp4_dir` — MP4 출력 루트 (dataset 설정 전용, import 에서 사용)
- `db.dsn_env` — DSN 을 담을 환경변수 이름 (기본 `RDFP_DB_DSN`)

rosbag 은 import 의 임시 스테이징으로 취급된다. 시간 범위 필터는 제공하지 않으며, `metadata.yaml` 이 있는 (녹화가 종료된) 세션의 모든 split 이 대상이 된다.

---

## 3. 5 분 퀵스타트

```bash
# 1) 환경변수
export RDFP_DB_DSN="postgresql://rdfp:rdfp@localhost:5432/rdfp"
source install/setup.bash

# 2) 빈 DB 에 스키마 생성
ros2 run rdfp dataset init-db --dsn-env RDFP_DB_DSN

# 3) 설정 파일 편집 후 — 감지된 에피소드 목록 확인 (쓰기 없음)
ros2 run rdfp rosbag list-episodes --config /etc/rdfp/rosbag_config.yaml

# 4) 실제 적재
ros2 run rdfp dataset import --config /etc/rdfp/dataset_config.yaml

# 5) 적재 결과 확인
ros2 run rdfp dataset stats --config /etc/rdfp/dataset_config.yaml
```

실행 요약 JSONL 로그는 `<output_mp4_dir>/_logs/postproc_run.jsonl` 에 기록된다.

---

## 4. 서브커맨드 상세

### 4.1 `init-db`

DB 에 후처리기가 요구하는 테이블·인덱스·FK 를 생성한다. `CREATE TABLE IF NOT EXISTS` 기반이라 반복 실행해도 안전하다.

#### 옵션

| 옵션 | 필수 | 설명 |
|---|---|---|
| `--config <path>` | 선택 | `dataset_config.yaml` 을 지정하면 `db.dsn_env` + `db.schema` 를 사용. **미지정 시 현재 작업 디렉터리의 `dataset_config.yaml` 을 자동 탐색**하며, 그것도 없으면 `--dsn-env` + `--schema` 로 fallback 된다. |
| `--dsn-env <ENV>` | 선택 | DSN 이 담긴 환경변수 이름 (기본 `RDFP_DB_DSN`). `--config` 와 cwd 기본 파일이 모두 없을 때 사용 |
| `--schema <name>` | 선택 | 대상 스키마 (기본 `public`). 미존재 스키마는 `CREATE SCHEMA IF NOT EXISTS` 로 선행 생성 |
| `--drop` | 선택 | 기존 후처리기 테이블을 `DROP CASCADE` 로 제거 후 재생성 (**파괴적**) |
| `--yes` | 선택 | `--drop` 의 대화형 확인을 건너뜀 (CI·스크립트 용) |

#### 사용 예

```bash
# (a) 환경변수 DSN + public 스키마 (가장 간단)
ros2 run rdfp dataset init-db --dsn-env RDFP_DB_DSN

# (b) 커스텀 스키마
ros2 run rdfp dataset init-db \
    --dsn-env RDFP_DB_DSN --schema rdfp_v1

# (c) dataset_config.yaml 의 db 섹션 재사용
ros2 run rdfp dataset init-db --config /etc/rdfp/dataset_config.yaml

# (d) 기존 스키마 완전 재생성
ros2 run rdfp dataset init-db --dsn-env RDFP_DB_DSN --drop --yes
```

`--drop` 을 TTY 에서 실행하면 `YES` 입력을 요구한다. 스크립트/CI 에서는 반드시 `--yes` 를 함께 지정해야 하며, 그렇지 않으면 거부된다.

---

### 4.2 `import`

설정 파일의 범위에 해당하는 rosbag 메시지를 에피소드 단위로 적재한다.

#### 옵션

| 옵션 | 필수 | 설명 |
|---|---|---|
| `--config <path>` | 선택 | 설정 YAML 경로. 미지정 시 현재 작업 디렉터리의 `dataset_config.yaml` 을 자동 사용한다. cwd 에도 파일이 없으면 exit 2. |
| `--dry-run` | 선택 | DB·MP4 쓰기 없이 에피소드 감지까지만 수행 (JSON 출력) |

#### 출력

성공 시 stdout 으로 실행 요약 JSON 을 출력한다.

```json
{
  "episodes": 3,
  "splits": 1,
  "inserted": {
    "pose_stampeds": 540,
    "twist_stampeds": 270,
    "joint_states": 900,
    "joint_trajectories": 12
  },
  "skipped": 0,
  "replaced": 0,
  "mp4_files": 3,
  "warnings": 0
}
```

#### 사용 예

```bash
# (a) 기본 실행
ros2 run rdfp dataset import --config /etc/rdfp/dataset_config.yaml

# (b) 에피소드만 검증 (DB 에 쓰지 않음)
ros2 run rdfp dataset import --config /etc/rdfp/dataset_config.yaml --dry-run

# (c) 디버그 로그 포함
ros2 run rdfp dataset import --config /etc/rdfp/dataset_config.yaml --log-level debug
```

#### 동일 에피소드 재실행 정책

동일 `(start_sec, start_nanosec)` 에피소드가 이미 DB 에 있으면 설정의 `on_existing_episode` 값에 따라 처리된다.

- `skip` (기본) — 건너뜀
- `replace` — 기존 DB 행(FK cascade) + MP4 디렉터리를 삭제 후 재적재
- `error` — 즉시 실패

```yaml
# /etc/rdfp/dataset_config.yaml
on_existing_episode: replace      # 재처리 시 교체
```

---

### 4.3 `list-episodes`

실제 쓰기 없이 스캔·에피소드 감지까지만 수행한다. `import --dry-run` 보다 사람이 읽기 쉬운 표 형식을 제공한다.

#### 옵션

| 옵션 | 필수 | 설명 |
|---|---|---|
| `--config <path>` | 선택 | 설정 YAML 경로. 미지정 시 현재 작업 디렉터리의 `rosbag_config.yaml` 을 자동 사용한다. cwd 에도 파일이 없으면 exit 2. |
| `--format {text,json}` | 선택 | 출력 포맷 (기본 `text`) |

#### 출력 — text

`START_TS` 는 로컬 타임존 기준의 `YYYY-MM-DD HH:MM:SS` 형식이며, 소수점 이하는
절사된다. 원본 나노초 값이 필요하면 `--format json` 을 쓴다.

```
 IDX  START_TS                DUR_SEC  TASK_LABEL
   0  2024-04-11 09:00:02       3.000  pick_and_place
   1  2024-04-11 09:00:06       4.000  pick_and_place
   2  2024-04-11 09:00:20       2.500  pour_water
```

#### 출력 — json

```json
{
  "episodes": 3,
  "splits": 1,
  "dry_run": true,
  "episode_list": [
    {"start_ns": 1712790002000000000, "stop_ns": 1712790005000000000,
     "task_label": "pick_and_place", "duration_sec": 3.0},
    ...
  ]
}
```

#### 사용 예

```bash
# 사람이 보기용
ros2 run rdfp rosbag list-episodes --config /etc/rdfp/rosbag_config.yaml

# 다른 도구에 파이프
ros2 run rdfp rosbag list-episodes \
    --config /etc/rdfp/rosbag_config.yaml --format json | jq '.episodes'
```

---

### 4.4 `stats`

현재 DB 의 후처리기 관련 테이블별 `COUNT(*)` 를 출력한다.

#### 옵션

| 옵션 | 필수 | 설명 |
|---|---|---|
| `--config <path>` | 선택 | 설정 YAML 경로 (`db.dsn_env`, `db.schema` 사용). 미지정 시 현재 작업 디렉터리의 `dataset_config.yaml` 을 자동 사용한다. |
| `--format {text,json}` | 선택 | 출력 포맷 (기본 `text`) |

#### 출력 — text

```
gripper_cmds         24
gripper_states       180
joint_states         900
joint_trajectories   12
pose_stampeds        540
sessions             3
twist_stampeds       270
```

#### 출력 — json

```json
{
  "gripper_cmds": 24,
  "gripper_states": 180,
  "joint_states": 900,
  "joint_trajectories": 12,
  "pose_stampeds": 540,
  "sessions": 3,
  "twist_stampeds": 270
}
```

#### 사용 예

```bash
# 적재 후 검증
ros2 run rdfp dataset stats --config /etc/rdfp/dataset_config.yaml

# 모니터링 스크립트에서 세션 수 추출
ros2 run rdfp dataset stats \
    --config /etc/rdfp/dataset_config.yaml --format json | jq '.sessions'
```

---

### 4.5 `topic-info`

지정 토픽이 rosbag 에 어떻게 기록되어 있는지 조회한다 (DB 접속 없음). split 별로 메시지 수 · 타입 · 첫/마지막 `log_time` 을 집계한다.

#### 옵션

| 인자/옵션 | 필수 | 설명 |
|---|---|---|
| `<topic>` (positional) | 필수 | 조회할 토픽 이름 (예: `/camera/image_raw`) |
| `--config <path>` | 선택 | 설정 YAML 경로 (`rosbag_dir` / `session_filter.dates` 사용). 미지정 시 cwd 의 `rosbag_config.yaml` 을 자동 사용한다. |
| `--format {text,json}` | 선택 | 출력 포맷 (기본 `text`) |

`session_filter.dates` 가 지정되어 있으면 해당 날짜의 split 만 스캔한다. `topics` 필터는 적용되지 않으므로 설정 파일에 없는 토픽도 조회 가능하다.

#### 출력 — text

```
Topic:         /camera/image_raw
Type:          sensor_msgs/msg/Image
Session count: 1
Split count:   3
Message count: 4500
First log_ts:  2024-04-11 09:00:02
Last log_ts:   2024-04-11 09:02:30
Duration:      148.240 s
Avg rate:      30.356 Hz

Splits:
  IDX  SESSION                           START_TS                COUNT     DUR_SEC
    0  session_2024-04-11_09-00-02       2024-04-11 09:00:02      1500      50.000
    1  session_2024-04-11_09-00-02       2024-04-11 09:00:52      1500      50.000
    2  session_2024-04-11_09-00-02       2024-04-11 09:01:42      1500      48.240
```

시각 컬럼은 로컬 타임존 기준 `YYYY-MM-DD HH:MM:SS` 로 절사된다. 원본 나노초 값은 `--format json` 을 사용한다.

#### 출력 — json

```json
{
  "topic": "/camera/image_raw",
  "types": ["sensor_msgs/msg/Image"],
  "session_count": 1,
  "split_count": 3,
  "message_count": 4500,
  "first_log_ns": 1712790002000000000,
  "last_log_ns":  1712790150240000000,
  "duration_sec": 148.24,
  "avg_rate_hz":  30.356,
  "splits": [
    {"session_name": "session_2024-04-11_09-00-02", "split_index": 0,
     "path": "/data/rosbag/2024-04-11/session_2024-04-11_09-00-02/bag_0.mcap",
     "start_ns": 1712790002000000000, "end_ns": 1712790052000000000,
     "count": 1500, "duration_sec": 50.0}
  ]
}
```

#### 사용 예

```bash
# 카메라 토픽 요약
ros2 run rdfp rosbag topic-info /camera/image_raw --config /etc/rdfp/rosbag_config.yaml

# JSON 으로 받아 평균 Hz 추출
ros2 run rdfp rosbag topic-info /servo_node/delta_twist_cmds \
    --config /etc/rdfp/rosbag_config.yaml --format json | jq '.avg_rate_hz'

# 존재하지 않는 토픽 → message_count=0 로 정상 종료 (exit 0)
ros2 run rdfp rosbag topic-info /does_not_exist --config /etc/rdfp/rosbag_config.yaml
```

#### 주의

- 시각 필드는 rosbag 의 **`log_time`** (rosbag 에 기록된 시각) 이며 메시지의 `header.stamp` 와는 약간 다를 수 있다. 정확한 stamp 가 필요하면 `list-episodes` 의 결과와 교차 확인한다.
- 같은 토픽 이름이 서로 다른 타입으로 기록된 경우 `types` 배열에 복수로 나타나며, text 출력의 "Type" 라벨이 "Types" 로 바뀐다 (정상 rosbag 이라면 단일 타입).

---

### 4.6 `list-topics`

rosbag 에 기록된 **모든 토픽**의 요약을 `ls -l` 스타일로 출력한다 (DB 접속 없음). 각 토픽의 메시지 수·타입·평균 Hz·첫 `log_time` 을 한 줄에 보여준다.

#### 옵션

| 옵션 | 필수 | 설명 |
|---|---|---|
| `--config <path>` | 선택 | 설정 YAML 경로 (`rosbag_dir` / `session_filter.dates` 사용). 미지정 시 cwd 의 `rosbag_config.yaml` 을 자동 사용한다. |
| `--format {text,json}` | 선택 | 출력 포맷 (기본 `text`) |
| `--sort {name,count,rate,type}` | 선택 | text 정렬 기준 (기본 `name`). JSON 에는 영향 없음 |

`topics` 필터는 적용되지 않으므로 설정 파일에 없는 토픽도 모두 나타난다. `session_filter.dates` 는 그대로 적용된다.

#### 출력 — text

`total <N>` 요약 라인에 이어 토픽당 한 줄이 출력된다. 컬럼 폭은 실제 값의 최대 폭에 맞춰 자동으로 정렬된다.

```
total 3  (1 session(s), 3 split(s))
COUNT     RATE Hz  FIRST_TS             TYPE                            TOPIC
 4500   30.000 Hz  2024-04-11 09:00:02  sensor_msgs/msg/Image           /camera/image_raw
  450    9.000 Hz  2024-04-11 09:00:02  geometry_msgs/msg/TwistStamped  /servo_node/delta_twist_cmds
   45    0.900 Hz  2024-04-11 09:00:02  rdfp_msgs/msg/SessionCommand    /session
```

시각은 로컬 타임존 기준 `YYYY-MM-DD HH:MM:SS` 로 절사된다. 원본 나노초 값은 `--format json` 을 사용한다.

#### 출력 — json

```json
{
  "session_count": 1,
  "split_count": 3,
  "topic_count": 3,
  "topics": [
    {"name": "/camera/image_raw",
     "types": ["sensor_msgs/msg/Image"],
     "message_count": 4500,
     "first_log_ns": 1712790002000000000,
     "last_log_ns":  1712790150240000000,
     "duration_sec": 148.24,
     "rate_hz":      30.356},
    ...
  ]
}
```

#### 사용 예

```bash
# 기본 (이름순)
ros2 run rdfp rosbag list-topics --config /etc/rdfp/rosbag_config.yaml

# 메시지 수 많은 순
ros2 run rdfp rosbag list-topics --config /etc/rdfp/rosbag_config.yaml --sort count

# JSON 으로 받아 토픽 이름만 추출
ros2 run rdfp rosbag list-topics \
    --config /etc/rdfp/rosbag_config.yaml --format json | jq '.topics[].name'
```

#### 주의

- `4.5 topic-info` 와 동일하게 시각 필드는 `log_time` 기준이다.
- rosbag 이 하나도 없으면 `total 0` 만 출력하고 exit 0 으로 정상 종료한다.

---

### 4.7 `clear`

`rosbag_dir` 아래의 **모든 세션 데이터를 삭제**한다. 녹화가 완료된 세션뿐 아니라 `metadata.yaml` 이 없는 비정상 종료 세션도 포함된다. 안전장치를 반드시 거치므로 의도치 않은 실행으로부터 보호된다.

#### 삭제 대상

- `<rosbag_dir>/YYYY-MM-DD/session_YYYY-MM-DD_HH-MM-SS/` 형식을 가진 세션 디렉터리 전체.
- 위 디렉터리 제거 후 비어진 날짜 폴더(`YYYY-MM-DD/`)도 함께 제거.
- `rosbag_dir` 루트 자체, 그리고 네이밍 규칙에 부합하지 않는 폴더/파일은 **보존** 한다.
- `session_filter.dates` 가 설정되어 있으면 해당 날짜만 대상으로 한다.

#### 옵션

| 옵션 | 필수 | 설명 |
|---|---|---|
| `--config <path>` | 선택 | 설정 YAML 경로 (`rosbag_dir` / `session_filter.dates` 사용). 미지정 시 cwd 의 `rosbag_config.yaml` 을 자동 사용한다. |
| `--dry-run` | 선택 | 대상 목록만 출력하고 실제 삭제는 수행하지 않음 |
| `--yes` | 선택 | 대화형 `'YES'` 확인을 건너뜀. **non-TTY / CI 환경에서는 필수** |

#### 안전장치

- 삭제 전 항상 대상 세션 목록과 예상 용량을 출력한다.
- TTY 에서 실행 시 `'YES'` 정확 입력을 요구한다 (다른 값은 거부).
- non-interactive 환경에서 `--yes` 없이 호출하면 exit 2 로 거부된다.
- `catalog_index_path` 가 설정되어 있고 파일이 존재하면 삭제 완료 후 "stale" 경고를 남긴다 (인덱스는 자동 재생성되지 않음).

#### 사용 예

```bash
# (a) 미리보기 — 삭제하지 않음
ros2 run rdfp rosbag clear --dry-run

# (b) 대화형 확인 후 전체 삭제
ros2 run rdfp rosbag clear

# (c) 자동화 스크립트 (--yes 필수)
ros2 run rdfp rosbag clear --yes

# (d) 특정 날짜만 삭제 (session_filter.dates 활용)
#   rosbag_config.yaml 에
#     session_filter: {dates: [2026-04-11]}
#   을 설정한 뒤
ros2 run rdfp rosbag clear --yes
```

#### 출력 예

```
Found 3 session directories (~1.8 GB) under /data/rdfp/rosbag:
  2026-04-20/session_2026-04-20_10-00-00
  2026-04-21/session_2026-04-21_09-30-00
  2026-04-21/session_2026-04-21_11-22-33
This will permanently DELETE all session data under /data/rdfp/rosbag. Type 'YES' to proceed: YES
deleted 3/3 session directories
```

#### 종료 코드

| 코드 | 의미 |
|---|---|
| `0` | 정상 종료 (대상이 0 개여도 0 을 반환) |
| `1` | `rosbag_dir` 부재, 또는 일부 세션의 `rmtree` 실패 |
| `2` | 설정 누락, 또는 non-TTY 에서 `--yes` 미지정 |

---

## 5. 공통 옵션

모든 서브커맨드에 적용된다. 서브커맨드 **앞** 에 위치해야 한다.

| 옵션 | 설명 |
|---|---|
| `--log-level {debug,info,warning,error}` | 표준 로그 레벨 (기본 `info`) |
| `-h`, `--help` | 도움말 출력 |

예:
```bash
ros2 run rdfp dataset --log-level debug import --config dataset_config.yaml
```

---

## 6. 환경변수

| 변수 | 용도 | 필수 |
|---|---|---|
| `RDFP_DB_DSN` | PostgreSQL DSN (기본 변수명). 설정 파일의 `db.dsn_env` 로 다른 이름 지정 가능 | 필수 |

JSONL 실행 로그 경로는 `<output_mp4_dir>/_logs/postproc_run.jsonl` 로 고정되며,
DB 배치 INSERT 버퍼 크기도 내부 기본값(1000)이 그대로 사용된다. 이 둘은 현재
환경변수로 오버라이드할 수 없다.

**DSN 형식**:
```
postgresql://<user>[:<password>]@<host>:<port>/<dbname>
```

예: `postgresql://rdfp:secret@db.internal:5432/rdfp`

---

## 7. 설정 파일 주요 필드

CLI 별로 설정 파일이 분리되어 있다.

- **`rosbag_config.yaml`** — `rosbag` CLI 전용. 읽기 전용이므로 `db` 섹션과 `output_mp4_dir` 은 필요 없다. 전체 예시는 [rosbag_config.sample.yaml](./rosbag_config.sample.yaml) 참조.
- **`dataset_config.yaml`** — `dataset` CLI 전용. DB 접속 정보와 `output_mp4_dir` 을 반드시 포함한다. 전체 예시는 [dataset_config.sample.yaml](./dataset_config.sample.yaml) 참조.

전체 스키마는 [설계서 4 장](./데이터셋%20후처리기%20설계서.md#4-범위-지정) 에 정의되어 있으며 두 설정 파일은 동일한 스키마 (`PostProcConfig`) 를 공유한다. 차이는 필수 필드 여부뿐이다.

### 7.1 필수 필드

```yaml
rosbag_dir:     /data/rosbag                 # rosbag 루트 (import 의 임시 스테이징)
output_mp4_dir: /data/postproc/videos        # MP4 출력 루트

db:
  dsn_env: RDFP_DB_DSN                       # DSN 담은 환경변수 이름
  schema:  public
```

시간 범위 필터는 없다. `metadata.yaml` 이 있는 (녹화 종료된) 세션의 모든 split 이 대상이 되며, 녹화 중이거나 비정상 종료된 세션 (`metadata.yaml` 부재) 은 자동으로 스킵된다.

### 7.2 자주 쓰는 필터

```yaml
topics:                                      # 빈 목록이면 모든 토픽 (/session 자동 포함)
  - /session
  - /ee_pose_publisher/ee_pose
  - /delta_twist_stamp
  - /camera/image_raw

episode_filter:
  task_labels_include: [pick_and_place]      # 지정 task_label 만
  task_labels_exclude: [outlier_demo]
  min_duration_sec: 2.0                      # 이보다 짧은 에피소드 제외

session_filter:
  dates: [2026-04-11]                        # YYYY-MM-DD 목록만 처리
```

### 7.3 재실행 / 영상 / 운영 옵션

```yaml
on_existing_episode: skip                    # skip | replace | error

mp4:
  codec: h264
  nominal_fps: 30

catalog_index_path: /data/rosbag/_index.sqlite   # 운영 인덱스(선택)
parallelism: 4                                   # 병렬 워커 (기본 1)

quality_gate:
  stamp_regression: true
  idle_gap_sec: 0.0                              # >0 시 유휴 갭 경고

# rosbag 을 임시 스테이징으로 운용할 때: import 성공 후 사용된 split
# (.mcap) 을 제거하고, metadata.yaml 만 남은 세션 디렉터리도 정리한다.
# dry-run 모드에서는 무시된다.
delete_splits_after_import: false
```

---

## 8. 자주 쓰는 시나리오

### 8.1 신규 DB 세팅부터 첫 적재까지

```bash
export RDFP_DB_DSN="postgresql://rdfp@localhost:5432/rdfp"
source install/setup.bash

ros2 run rdfp dataset init-db --dsn-env RDFP_DB_DSN
cp docs/rosbag2/rosbag_config.sample.yaml  /etc/rdfp/rosbag_config.yaml
cp docs/rosbag2/dataset_config.sample.yaml /etc/rdfp/dataset_config.yaml
vi /etc/rdfp/rosbag_config.yaml /etc/rdfp/dataset_config.yaml
ros2 run rdfp rosbag list-episodes --config /etc/rdfp/rosbag_config.yaml
ros2 run rdfp dataset import --config /etc/rdfp/dataset_config.yaml
```

### 8.2 특정 날짜 / task 만 재적재

```bash
# /etc/rdfp/dataset_config.yaml 를 아래로 편집
#   session_filter:
#     dates: [2026-04-11]
#   episode_filter:
#     task_labels_include: [pick_and_place]
#   on_existing_episode: replace

ros2 run rdfp dataset import --config /etc/rdfp/dataset_config.yaml
```

### 8.3 장시간 rosbag 을 병렬 처리

```bash
# /etc/rdfp/dataset_config.yaml 에 추가
#   parallelism: 4
ros2 run rdfp dataset import --config /etc/rdfp/dataset_config.yaml
```

병렬도는 보통 CPU 코어 수의 절반 ~ 전체가 적정. 에피소드 수보다 크게 잡아도 실제 워커는 에피소드 수로 상한이 걸린다.

### 8.4 기존 DB 를 완전히 초기화 후 재적재

```bash
ros2 run rdfp dataset init-db --dsn-env RDFP_DB_DSN --drop --yes
ros2 run rdfp dataset import --config /etc/rdfp/dataset_config.yaml
```

### 8.5 rosbag 전체 초기화 (DB + 파일)

```bash
# 1) 삭제 전 규모 확인
ros2 run rdfp rosbag clear --dry-run

# 2) rosbag 파일 전부 삭제
ros2 run rdfp rosbag clear --yes

# 3) DB 도 초기화 (선택)
ros2 run rdfp dataset init-db --dsn-env RDFP_DB_DSN --drop --yes
```

rosbag 은 임시 스테이징 성격이므로 적재가 끝난 뒤 주기적으로 삭제해도 문제없다. `delete_splits_after_import: true` 옵션으로 import 중 자동 정리하는 방법도 있다 ([설정 7.3 절](#73-재실행--영상--운영-옵션) 참고).

### 8.6 cron / systemd 에서 주기 실행

```bash
#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
source /home/ros/rdfp_ws/install/setup.bash
export RDFP_DB_DSN="postgresql://rdfp:secret@db.internal:5432/rdfp"
ros2 run rdfp dataset import \
    --config /etc/rdfp/dataset_config.yaml \
    --log-level info
```

non-interactive 환경에서 `--drop` 사용 시 반드시 `--yes` 를 함께 지정한다.

---

## 9. 로그와 종료 코드

### 9.1 표준 로그

- stderr 로 `%(asctime)s [%(levelname)s] %(name)s: %(message)s` 포맷.
- 레벨은 `--log-level` 로 조절.

### 9.2 실행 요약 JSONL

에피소드 처리마다 한 줄씩 `<output_mp4_dir>/_logs/postproc_run.jsonl` 에 append.

| `event` | 의미 |
|---|---|
| `episode_done` | 커밋 완료. `episode_id`, `row_counts`, `mp4_files` 포함 |
| `episode_skipped` | `on_existing_episode=skip` 로 건너뜀 |
| `episode_failed` | 트랜잭션 롤백된 실패. `error` 메시지 포함 |
| `quality_warning` | 품질 게이트 감지 (`stamp_regression` / `idle_gap`) |

예:
```bash
tail -f /data/postproc/videos/_logs/postproc_run.jsonl | jq .
```

### 9.3 종료 코드

| 코드 | 의미 |
|---|---|
| `0` | 정상 종료 |
| `1` | 실행 실패 (PostProcError, 적재 실패, crash 등) |
| `2` | 설정/인자 오류 (파일 없음, 환경변수 누락, 잘못된 옵션) |

---

## 10. 문제 해결

| 증상 | 원인 / 조치 |
|---|---|
| `environment variable 'RDFP_DB_DSN' is not set` | `export RDFP_DB_DSN=...` 누락. 또는 `db.dsn_env` 가 실제로 설정한 변수명과 다름 |
| `required tables not found in schema: ['sessions', ...]` | `init-db` 를 실행하지 않았거나, `db.schema` 가 실제와 다름 |
| `no /session messages found within rosbag_dir` | `rosbag_dir` 하위에 `metadata.yaml` 을 가진 세션이 없음. 녹화가 종료된 세션인지, 경로가 올바른지 확인 |
| `no writer registered for topic ... (type=...)` | 해당 타입의 writer 가 registry 에 없음 → 해당 토픽은 **무시** 되고 실행 계속. 필요하면 writer 추가 |
| `tables missing episode_id FK to sessions(id)` | 스키마가 구버전. `init-db --drop --yes` 로 재생성 |
| `catalog index unavailable (...); falling back to metadata.yaml scan` | 인덱스 파일 경로 잘못됨 또는 스키마 불일치. 경고일 뿐 실행은 계속됨 |
| `ffmpeg not found` | `sudo apt install ffmpeg` |
| `mcap 모듈 없음` | `pip install --user mcap mcap-ros2-support` |

자세한 로그가 필요하면:
```bash
ros2 run rdfp dataset --log-level debug import --config /etc/rdfp/dataset_config.yaml
```

---

## 11. 관련 문서

- [데이터셋 후처리기 설계서](./데이터셋%20후처리기%20설계서.md) — 내부 구조·알고리즘·DB 스키마 전체
- [데이터셋 후처리기 실환경 검증 절차](./데이터셋%20후처리기%20실환경%20검증%20절차.md) — 실 인프라에서의 검증 runbook
- [rosbag_config.sample.yaml](./rosbag_config.sample.yaml) — rosbag CLI 샘플 설정
- [dataset_config.sample.yaml](./dataset_config.sample.yaml) — dataset CLI 샘플 설정
- [rosbag2 운영 방안](./rosbag2%20운영%20방안.md) — 입력 rosbag 디렉터리 구조·인덱스 포맷
- [session_control_guide.md](../session/session_control_guide.md) — `/session` 토픽 규격
