# 데이터셋 후처리기 운영 가이드 — DB·MP4 적재

rosbag2 MCAP 아카이브를 **에피소드 단위**로 분할해 PostgreSQL 에 적재하고, 카메라 토픽은 MP4 + `metadata.json`(글로벌 메타) + `frames.jsonl`(프레임별 stamp) 로 내보내는 배치 프로그램의 **실행과 운영**을 다룬다.

동작 원리·내부 구조는 [데이터셋 후처리기 설계서](dataset_ingest_design.md) 를, bag 아카이브 쪽 조작(조회·범위 export·삭제)은 [rosbag_cli_user_guide.md](rosbag_cli_user_guide.md) 를 본다.

---

> **전 과정에서 이 문서의 자리** — **③ 적재**의 정본이다(§2 는 ① 준비도 겸한다).
> 여섯 단계의 지도와 단계 사이 함정은 [../collect/학습_데이터_전_과정_지도.md](학습_데이터_전_과정_지도.md) 에 있다.

## 1. 개요

`rosbag` → **PostgreSQL + MP4** 적재와 그 운영을 다룬다. 명령은 **독립 실행파일**이다.

| 명령 | 용도 |
|---|---|
| [`init-db`](#41-init-db) | 필요한 DB 테이블·인덱스·FK 생성 |
| [`import`](#42-import) | rosbag → DB / MP4 실제 적재 |
| [`stats`](#43-stats) | 현재 DB 에 적재된 테이블별 행 수 출력 |
| `list` | 적재된 에피소드 목록 (DB 조회) |
| `replay` | 적재된 에피소드를 라이브 토픽으로 재생 (ROS 필요) |

```
ros2 run rdfp <명령> [--log-level LEVEL] [옵션...]
```

> **bag 쪽 조작은 이 문서가 아니다.** rosbag 아카이브를 조회하거나 에피소드를 범위로
> 내보내거나 지우는 것은 `rosbag` 명령이며
> [rosbag_cli_user_guide.md](rosbag_cli_user_guide.md) 가 갖는다 — `list-topics` ·
> `list-episodes` · `export` · `delete` · `clear`.
>
> **`export` 와 `import` 의 관계**: `export` 는 자체 적재 경로 없이 선택 결과를
> `episode_filter.start_ns_in` 에 실어 `import` 와 같은 파이프라인에 넘긴다. 즉 이 문서의
> `import` 설명·설정 필드·문제 해결이 `export` 에도 그대로 적용된다.

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
ros2 run rdfp init-db --dsn-env RDFP_DB_DSN

# 3) 설정 파일 편집 후 — 감지된 에피소드 목록 확인 (쓰기 없음)
ros2 run rdfp rosbag list-episodes --config /etc/rdfp/dataset_config.yaml

# 4) 실제 적재
ros2 run rdfp import --config /etc/rdfp/dataset_config.yaml

# 5) 적재 결과 확인
ros2 run rdfp stats --config /etc/rdfp/dataset_config.yaml
```

실행 요약 JSONL 로그는 `<output_mp4_dir>/_logs/postproc_run.jsonl` 에 기록된다.

---

## 4. 명령 상세

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
ros2 run rdfp init-db --dsn-env RDFP_DB_DSN

# (b) 커스텀 스키마
ros2 run rdfp init-db \
    --dsn-env RDFP_DB_DSN --schema rdfp_v1

# (c) dataset_config.yaml 의 db 섹션 재사용
ros2 run rdfp init-db --config /etc/rdfp/dataset_config.yaml

# (d) 기존 스키마 완전 재생성
ros2 run rdfp init-db --dsn-env RDFP_DB_DSN --drop --yes
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
ros2 run rdfp import --config /etc/rdfp/dataset_config.yaml

# (b) 에피소드만 검증 (DB 에 쓰지 않음)
ros2 run rdfp import --config /etc/rdfp/dataset_config.yaml --dry-run

# (c) 디버그 로그 포함
ros2 run rdfp import --config /etc/rdfp/dataset_config.yaml --log-level debug
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

### 4.3 `stats`

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
ros2 run rdfp stats --config /etc/rdfp/dataset_config.yaml

# 모니터링 스크립트에서 세션 수 추출
ros2 run rdfp stats \
    --config /etc/rdfp/dataset_config.yaml --format json | jq '.sessions'
```

---

## 5. 공통 옵션

다섯 명령이 모두 받는다. 명령마다 자기 파서가 선언하므로 **명령 뒤** 에 위치한다.

| 옵션 | 설명 |
|---|---|
| `--log-level {debug,info,warning,error}` | 표준 로그 레벨 (기본 `info`) |
| `-h`, `--help` | 도움말 출력 |

예:
```bash
ros2 run rdfp import --log-level debug --config dataset_config.yaml
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

- **`rosbag_config.yaml`** — `rosbag` CLI 전용. 읽기 전용이므로 `db` 섹션과 `output_mp4_dir` 은 필요 없다. 전체 예시는 [dataset_config.yaml](../../config/dataset_config.yaml) 참조.
- **`dataset_config.yaml`** — `dataset` CLI 전용. DB 접속 정보와 `output_mp4_dir` 을 반드시 포함한다. 전체 예시는 [dataset_config.yaml](../../config/dataset_config.yaml) 참조.

전체 스키마는 [설계서 4 장](dataset_ingest_design.md#4-범위-지정) 에 정의되어 있으며 두 설정 파일은 동일한 스키마 (`PostProcConfig`) 를 공유한다. 차이는 필수 필드 여부뿐이다.

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

ros2 run rdfp init-db --dsn-env RDFP_DB_DSN
cp docs/rosbag2/rosbag_config.sample.yaml  /etc/rdfp/rosbag_config.yaml
cp docs/rosbag2/dataset_config.sample.yaml /etc/rdfp/dataset_config.yaml
vi /etc/rdfp/rosbag_config.yaml /etc/rdfp/dataset_config.yaml
ros2 run rdfp rosbag list-episodes --config /etc/rdfp/dataset_config.yaml
ros2 run rdfp import --config /etc/rdfp/dataset_config.yaml
```

### 8.2 특정 날짜 / task 만 재적재

```bash
# /etc/rdfp/dataset_config.yaml 를 아래로 편집
#   session_filter:
#     dates: [2026-04-11]
#   episode_filter:
#     task_labels_include: [pick_and_place]
#   on_existing_episode: replace

ros2 run rdfp import --config /etc/rdfp/dataset_config.yaml
```

### 8.3 장시간 rosbag 을 병렬 처리

```bash
# /etc/rdfp/dataset_config.yaml 에 추가
#   parallelism: 4
ros2 run rdfp import --config /etc/rdfp/dataset_config.yaml
```

병렬도는 보통 CPU 코어 수의 절반 ~ 전체가 적정. 에피소드 수보다 크게 잡아도 실제 워커는 에피소드 수로 상한이 걸린다.

### 8.4 기존 DB 를 완전히 초기화 후 재적재

```bash
ros2 run rdfp init-db --dsn-env RDFP_DB_DSN --drop --yes
ros2 run rdfp import --config /etc/rdfp/dataset_config.yaml
```

### 8.5 rosbag 초기화 (DB + bag) — **mp4 는 남는다**

```bash
# 1) 삭제 전 규모 확인 (clear 에는 --dry-run 이 없다)
ros2 run rdfp rosbag list-topics

# 2) rosbag 파일 전부 삭제
ros2 run rdfp rosbag clear --yes

# 3) DB 도 초기화 (선택)
ros2 run rdfp init-db --dsn-env RDFP_DB_DSN --drop --yes
```

rosbag 은 임시 스테이징 성격이므로 적재가 끝난 뒤 주기적으로 삭제해도 문제없다. **전부 지우는 것이 부담스러우면 `rosbag delete --exported` 로 적재된 에피소드만 지운다** ([rosbag_cli_user_guide.md](rosbag_cli_user_guide.md) §5). `delete_splits_after_import: true` 옵션으로 import 중 자동 정리하는 방법도 있다 ([설정 7.3 절](#73-재실행--영상--운영-옵션) 참고).

⚠️ **이 절은 판을 비우지 않는다.** 세션 레코더가 남긴 **mp4 와 sidecar**(보통 가장 큰
덩어리)와 export 산출물이 그대로 남는다. 수집을 처음부터 다시 하려면 **§8.6** 을 쓴다.

### 8.6 수집 데이터를 **전부** 지운다 (DB + mp4 + bag + 학습 샘플)

위 8.5 는 **bag 과 DB 만** 지운다. 수집을 다시 시작하려고 판을 비우는 것이라면
**덩어리가 다섯**이고, 그중 가장 큰 `recordings/`(레코더 mp4)를 어떤 CLI 도 모른다.

⚠️ **`scripts/clear_dataset.sh` 를 믿고 끝내면 안 된다.** 이름과 달리 `init-db --drop`
과 `videos/` 삭제뿐이라 **mp4 가 그대로 남는다** — `videos/` 는 bag 에서 mp4 를 뽑는
`FrameRouter` 의 자리이고, 수집 경로에서는 보통 비어 있다.

```bash
cd ~/development/ros/rdfp_ws
source ./.rdfprc                 # RDFP_DATA_ROOT · RDFP_DB_DSN · RDFP_ROSBAG_DIR

# ① 먼저 내린다 — 수집 노드가 살아 있으면 레코더가 지우는 족족 다시 쓴다
./scripts/kill_stack.sh

# ② 규모 확인 (지우기 전에 한 번 본다)
python3 - <<'EOF'
import os, psycopg
with psycopg.connect(os.environ['RDFP_DB_DSN']) as c:
    print('에피소드', c.execute('select count(*) from sessions').fetchone()[0], '건')
EOF
du -sh "$RDFP_DATA_ROOT"/{recordings,rosbag,rosbag_imported,datasets} 2>/dev/null

# ③ 삭제
ros2 run rdfp init-db --dsn-env RDFP_DB_DSN --drop --yes   # DB (지우고 바로 재생성한다)
ros2 run rdfp rosbag clear --yes                           # 적재 대기 bag
rm -rf "$RDFP_DATA_ROOT/rosbag_imported"                   # 적재 끝나 빼 둔 bag
rm -rf "${RDFP_DATA_ROOT:?}/recordings"/*                  # 레코더 mp4 + sidecar
rm -rf "${RDFP_DATA_ROOT:?}/datasets"/*                    # export 산출물 (.npz 등)

# ④ 확인 — 둘 다 0 이어야 한다
python3 -c "
import os, psycopg
with psycopg.connect(os.environ['RDFP_DB_DSN']) as c:
    print('에피소드', c.execute('select count(*) from sessions').fetchone()[0])"
find "$RDFP_DATA_ROOT" \( -name '*.mp4' -o -name '*.mcap' -o -name '*.npz' \) | wc -l
```

| 덩어리 | 어디 | 지우는 수단 |
|---|---|---|
| DB 에피소드 | `sessions` 외 13 테이블 | `init-db --drop --yes` (재생성까지 한다) |
| 적재 대기 bag | `$RDFP_DATA_ROOT/rosbag` | `rosbag clear --yes` |
| 보관 bag | `$RDFP_DATA_ROOT/rosbag_imported` | **CLI 없음** — `rm -rf` |
| 레코더 mp4 · sidecar | `$RDFP_DATA_ROOT/recordings` | **CLI 없음** — `rm -rf` |
| 학습 샘플 | `$RDFP_DATA_ROOT/datasets` | **CLI 없음** — `rm -rf` |

**`${RDFP_DATA_ROOT:?}` 의 `:?` 는 오타가 아니다.** 변수가 비어 있으면 `rm -rf /recordings/*`
가 되므로, 비었을 때 **실행 대신 오류로 멈추게** 한다. `.rdfprc` 를 안 소싱한 셸에서
그대로 붙여 넣는 것이 흔한 사고다.

**골라서 지우기.** 전부가 아니라 일부만이면 bag 쪽은
`ros2 run rdfp rosbag list-episodes` 로 목록을 보고 `rosbag delete` 로 고른다
([rosbag_cli_user_guide.md](rosbag_cli_user_guide.md) §5). **DB 쪽은 그런 CLI 가 없어**
직접 지운다 — 외래키가 `ON DELETE CASCADE` 라 자식 행은 함께 사라진다.

```sql
delete from sessions where id in (11, 12, 13);
```

⚠️ **mp4 는 따로 지운다.** DB 행을 지워도 파일은 남아 고아가 된다. 어떤 파일이
고아인지는 `image_streams.mp4_path`(파일 이름만 담긴다) 와 디렉터리를 맞춰 본다.

### 8.7 cron / systemd 에서 주기 실행

```bash
#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
source /home/ros/rdfp_ws/install/setup.bash
export RDFP_DB_DSN="postgresql://rdfp:secret@db.internal:5432/rdfp"
ros2 run rdfp import \
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
ros2 run rdfp import --log-level debug --config /etc/rdfp/dataset_config.yaml
```

---

## 11. 관련 문서

- [데이터셋 후처리기 설계서](dataset_ingest_design.md) — 내부 구조·알고리즘·DB 스키마 전체
- [../archive/dataset_postproc_verification_runbook.md](../archive/dataset_postproc_verification_runbook.md) (보관 — 구 CLI · 구 경로, 갱신하지 않는다) — 실 인프라에서의 검증 runbook
- [rosbag_cli_user_guide.md](rosbag_cli_user_guide.md) — `rosbag` 명령 (조회 · 범위 export · 삭제)
- [dataset_config.yaml](../../config/dataset_config.yaml) — bag 쪽 샘플 설정
- [dataset_config.yaml](../../config/dataset_config.yaml) — 적재 샘플 설정
- [dataset_ingest_design.md §0 rosbag 저장 구조](dataset_ingest_design.md#0-rosbag-저장-구조--운영-규칙) — 입력 rosbag 디렉터리 구조·인덱스 포맷
- [session_control_guide.md](session_control_guide.md) — `/session` 토픽 규격
