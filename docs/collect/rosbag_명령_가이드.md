# `scripts/rosbag.sh` 사용 설명서 — rosbag2 녹화 · 조회 · 적재 · 삭제

[scripts/rosbag.sh](../../scripts/rosbag.sh) 는 rosbag2 와 관련된 **녹화 · 조회 · 적재 · 삭제를 한 이름 아래에서** 실행한다.
처리 로직은 새로 쓰지 않고 기존 구현에 넘긴다 — 녹화는 [scripts/record_rosbag.sh](../../scripts/record_rosbag.sh),
적재는 `ros2 run rdfp import`, 나머지는 `ros2 run rdfp rosbag` 이다. 셸 스크립트를 거치지 않고 `ros2 run rdfp rosbag <명령>` 을
직접 불러도 옵션과 동작은 같다(`record` · `import` 는 제외). 적재한 뒤의 조회 · 재생은 DB 기반 CLI(`stats` · `list` · `replay`)가
맡는다 — [데이터셋_적재_운영_가이드.md](데이터셋_적재_운영_가이드.md).

영상은 rosbag 경로가 아니다 — Isaac 수집에서는 카메라 영상을 `rdfp_image_recorder` 가 mp4 로 따로 남긴다
([rdfp_image_recorder_node_guide.md](rdfp_image_recorder_node_guide.md)). 기본 토픽 목록(mock)에는 이미지 토픽도 포함되어
있으므로, 선택한 목록에 따라 bag 에 이미지가 기록될 수 있다(§3.5).

## 목차

1. [한눈에](#1-한눈에)
2. [준비 — 실행 환경](#2-준비--실행-환경)
3. [`record` — 녹화](#3-record--녹화)
4. [에피소드 고르기 — 공통 선택 인자](#4-에피소드-고르기--공통-선택-인자)
5. [`list-topics` · `list-episodes` — 조회](#5-list-topics--list-episodes--조회)
6. [`import` · `export` — DB 적재](#6-import--export--db-적재)
7. [`delete` · `clear` — 삭제](#7-delete--clear--삭제)
8. [활용 절차](#8-활용-절차)
9. [주의 사항](#9-주의-사항)
10. [옛 스크립트에서 옮기기](#10-옛-스크립트에서-옮기기)
11. [문제 해결](#11-문제-해결)
12. [개발자용 — Python API · 관련 파일](#12-개발자용--python-api--관련-파일)

---

## 1. 한눈에

```bash
./scripts/rosbag.sh help              # 명령 목록
./scripts/rosbag.sh help record       # 명령별 옵션 — `record --help` 와 같다
./scripts/rosbag.sh <명령> [옵션]
```

| 명령 | 하는 일 | 넘겨받는 구현 |
|---|---|---|
| `record` | 목록 파일에 적힌 토픽을 MCAP 으로 녹화한다. Ctrl+C 로 끝낸다 | `scripts/record_rosbag.sh` |
| `list-topics` | 루트 아래 녹화된 토픽을 요약한다 (읽기 전용) | `ros2 run rdfp rosbag list-topics` |
| `list-episodes` | bag 에 담긴 에피소드를 나열한다 (DB 적재 여부 포함) | `ros2 run rdfp rosbag list-episodes` |
| `import` | 적재 설정의 **범위 전체**를 DB 에 넣는다 | `ros2 run rdfp import` |
| `export` | **고른 에피소드만** DB 에 넣는다 | `ros2 run rdfp rosbag export` |
| `delete` | 고른 에피소드를 bag 에서 지운다 (파괴적 — 확인을 받는다) | `ros2 run rdfp rosbag delete` |
| `clear` | 루트 아래 세션을 **전부** 지운다 (파괴적 — 확인을 받는다) | `ros2 run rdfp rosbag clear` |

**공통 규칙**

- **bag 루트에 숨은 기본값이 없다.** 명령마다 다음 순서로 정하고, 정해지지 않으면 시작하지 않는다.

  | 명령 | bag 루트를 정하는 순서 |
  |---|---|
  | `record` | `--rosbag-dir` > `RDFP_ROSBAG_DIR` |
  | `list-episodes` · `delete` | `--rosbag-dir` > `--config` 의 `rosbag_dir` > `RDFP_ROSBAG_DIR` |
  | `export` | `--rosbag-dir` > `--config` 의 `rosbag_dir` — **고른 루트에서 그대로 적재한다.** `--config` 가 필수라 환경변수까지 내려가지 않는다 |
  | `list-topics` · `clear` | `--rosbag-dir` > `RDFP_ROSBAG_DIR` |
  | `import` | 설정 파일의 `rosbag_dir` (§6) |

  설정이 환경변수를 이기는 것이 중요하다 — 환경변수가 이기면 **설정이 가리키는 DB 에 다른 bag 을 적재**할 수 있다.
  같은 이유로 조회 · 미리 보기 · 적재 · 삭제를 이어서 할 때는 **같은 루트 지정(`--config` / `--rosbag-dir`)을 유지한다**(§7).
- `record` 를 뺀 명령의 옵션은 **검사하지 않고 그대로 넘긴다.** 옵션은 `<명령> --help` 로 본다 — 받는 쪽의 도움말이 나온다.
- 알 수 없는 명령과 `record` 의 알 수 없는 옵션은 종료 코드 **2** 로 거부한다. 넘겨받는 쪽의 종료 코드는 성공 `0`,
  사용자 오류(경로 · 인자 · 설정) `2`, 실행 실패(검증 실패 등) `1` 이다. 삭제 확인을 거부하면 `delete` 는 `1`, `clear` 는 `2` 로 끝난다.
- 모든 명령을 `exec` 로 넘긴다 — `rosbag.sh` 를 띄운 PID 가 곧 실제 프로세스의 PID 다(§3.7).
- Bash 로 실행한다(`sh scripts/rosbag.sh` 는 쓰지 않는다).

## 2. 준비 — 실행 환경

```bash
rdfp_env                     # ROS + 워크스페이스 overlay + RDFP_ROSBAG_DIR · RDFP_HOME · RDFP_DB_DSN
cd ~/development/ros/rdfp_ws
```

`rdfp_env` 는 이 개발 환경의 편의 명령이다(워크스페이스 루트의 `.rdfprc`). 해당 명령이 없는 셸에서는 직접 준비한다.

```bash
cd ~/development/ros/rdfp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export RDFP_ROSBAG_DIR="$HOME/rdfp_data/rosbag"   # 녹화 · 조회 · 적재가 모두 이 값을 본다
export RDFP_HOME="$PWD"                           # 기본 적재 설정(config/dataset_config.yaml)의 위치
```

- `install/setup.bash` 는 워크스페이스를 빌드한 뒤 생긴다. 없으면 `record` 는 녹화 전에 멈추고(§3.3), 나머지 명령은
  `Package 'rdfp' not found` 로 실패한다.
- `RDFP_HOME` 이 없으면 기본 적재 설정을 **현재 디렉터리의** `config/dataset_config.yaml` 에서 찾는다 — 워크스페이스 루트에서 실행한다.
- DB 적재(`import` · `export`)와 적재 여부 확인(`list-episodes --config`)은 DB 준비와 `RDFP_DB_DSN` 이 따로 필요하다
  ([데이터셋_적재_운영_가이드.md](데이터셋_적재_운영_가이드.md)).

## 3. `record` — 녹화

### 3.1 빠른 시작

```bash
./scripts/rosbag.sh record      # 기본 목록 config/recording_topics.list — 모든 백엔드 공용

# 끝낼 때는 실행한 터미널에서 Ctrl+C 를 누르고 종료를 기다린다 (§3.7)
```

목록은 백엔드와 무관하다(§3.5). 필요한 로봇 스택과 수집 계층을 별도 터미널에서 먼저 기동한다(§3.8).
Isaac · 펑션베이 수집은 [scripts/run_collect_isaac.sh](../../scripts/run_collect_isaac.sh) ·
[scripts/run_collect_functionbay.sh](../../scripts/run_collect_functionbay.sh) 가 녹화를 대신 띄우고 끄므로(내부에서
`rosbag.sh record` 를 부른다) 그것을 쓰면 직접 실행할 일이 없다.

녹화 전에 `ros2 topic list` 로 대상 토픽을, `ros2 interface show rdfp_msgs/msg/SessionCommand` 로 사용자 정의 메시지 타입을
확인할 수 있다.

### 3.2 옵션

| 옵션 | 환경변수 | 기본값 | 뜻 |
|---|---|---|---|
| `--topics-file FILE` | `RECORDING_TOPICS_FILE` | `config/recording_topics.list` | 녹화할 토픽 목록. 기본값은 스크립트 위치 기준이라 어디서 실행해도 되지만, **직접 준 상대 경로는 현재 디렉터리 기준**이다 |
| `--rosbag-dir DIR` | `RDFP_ROSBAG_DIR` | **없음 — 필수** | 저장 루트 |
| `--split-duration SEC` | `SPLIT_DURATION_SEC` | `3600` | 분할 간격(초). 양의 정수만 받는다 |
| `--config FILE` | — | `$RDFP_HOME/config/dataset_config.yaml` | 나중에 `import` 할 적재 설정. **대조에만** 쓴다(§3.4) |
| — | `STORAGE_ID` | `mcap` | `ros2 bag record -s` 로 넘길 저장 플러그인 ID. RDFP 적재는 `.mcap` 만 읽으므로 기본값을 유지한다 |

우선순위는 **옵션 > 환경변수 > 기본값**이다. 환경변수로 쓰던 옛 사용법도 그대로 통한다. `--option=값` 형식도 받는다.

```bash
# 저장 위치를 바꾸고 사용자 목록(§3.5)을 10분 단위로 나눈다 (워크스페이스 루트에서)
./scripts/rosbag.sh record \
  --topics-file config/recording_topics_custom.list \
  --rosbag-dir /data/rdfp/rosbag \
  --split-duration 600
```

`--split-duration` 은 **파일 분할 간격이며 전체 녹화의 제한 시간이 아니다.** 파일을 나눈 뒤에도 녹화를 계속한다. 적재는 split
하나를 통째로 메모리에 올리므로 이 값이 적재의 메모리 요구량도 정한다(§9.4).
크기 기준 분할(`ros2 bag record -b`)은 걸지 않는다 — [데이터셋_적재_시스템_설계서.md §0.2](데이터셋_적재_시스템_설계서.md#02-파일-분할-기준)
는 시간 · 크기 상한을 함께 걸라고 권하지만 구현하지 않았다. 기본 목록처럼 이미지 토픽을 포함하면 파일 크기가 빠르게 늘 수 있다.
rosbag2 자체 옵션은 넘기지 않는다 — rosbag2 도움말은 `ros2 bag record --help` 로 본다.

### 3.3 시작 전 검사

| 검사 | 하는 곳 | 어긋나면 |
|---|---|---|
| `record` 옵션 · 루트 지정 · 분할 간격 형식 | `rosbag.sh` | 종료 코드 2 |
| 토픽 목록 파일이 있고 토픽이 한 줄 이상 | `record_rosbag.sh` | `Recording topics file not found` / `No topics declared` 로 멈춘다 (이하 종료 코드 1) |
| ROS 2 환경 (`ros2` 명령) | `record_rosbag.sh` | `ros2 command not found` |
| 워크스페이스 overlay (`rdfp_msgs`) | `record_rosbag.sh` | `Package 'rdfp_msgs' not found` 로 **녹화 전에** 멈춘다 — overlay 없이 녹화하면 `/session` 등 `rdfp_msgs` 토픽이 0 건으로 남는데, rosbag2 는 멈추지 않고 경고만 반복하기 때문이다(2026-10-17 실측) |
| MCAP 저장 플러그인 `ros-humble-rosbag2-storage-mcap` | `record_rosbag.sh` (`STORAGE_ID=mcap` 일 때) | 설치 명령을 출력하고 멈춘다 |
| 저장 위치 여유 공간 10 GiB 이상 | `record_rosbag.sh` (`df` 가 있으면, **시작할 때 한 번만**) | `df -Pk` 의 가용 공간이 10,485,760 KiB 미만이면 `Less than 10 GB free space`. 녹화 중에는 다시 보지 않는다 |
| 적재 설정과의 대조 | `rosbag.sh` | **경고만** 하고 녹화한다 (§3.4) |
| 로봇 스택과 수집 계층이 떠 있다 | **하지 않는다** | 없는 토픽은 오류 없이 비어 있게 된다. 순서는 §3.8 |

MCAP 패키지 검사는 `dpkg -s ros-humble-rosbag2-storage-mcap` 으로 고정되어 Debian/Ubuntu 의 ROS 2 Humble 패키지 설치 환경을
전제로 한다. 다른 배포판이나 소스 설치 환경에서는 플러그인이 있어도 이 검사를 통과하지 못할 수 있다.

### 3.4 적재 설정과의 대조

녹화를 시작하기 전에, 나중에 적재할 설정(`--config`, 기본 `$RDFP_HOME/config/dataset_config.yaml`)이 이 녹화를 제대로 읽을지 본다.
어긋나면 오류가 아니라 **"조용히 빠짐"** 으로 나타나는 두 가지를 경고한다.

```text
[WARN] Recorded but NOT imported by /data/rdfp/dataset.yaml:
[WARN]   - /camera/image_raw
[WARN]   - /camera/image_compressed
[WARN] /…/config/dataset_config.yaml reads bags from /home/<user>/rosbag, but this run records to /data/rdfp/rosbag.
[WARN] Importing with that config will not find this recording.
```

- **`Recorded but NOT imported`** — 녹화할 토픽 중 적재 설정의 토픽 목록(`topics_file` + `topics`, `/session` 은 자동 포함)에 없는 것.
  녹화는 되지만 DB 에 들어가지 않는다. 적재 설정의 토픽 목록이 비어 있으면(전 토픽 적재) 보지 않는다.
- **`reads bags from …`** — 적재 설정의 `rosbag_dir` 이 녹화 루트와 다르다. 그 설정으로 적재하면 이 녹화를 찾지 못한다.

설정은 적재와 같은 코드(`rdfp.dataset.config`)로, **지금 셸의 환경변수 그대로** 해석한다. `--rosbag-dir` 로 루트를 바꿔도
`RDFP_ROSBAG_DIR` 이 그대로면 기본 설정은 여전히 그 환경변수의 루트를 본다는 것을 알려 주기 위해서다.
설정 파일이 없거나 읽을 수 없으면 경고(`skipping the import check` / `Import check skipped`)만 남기고 녹화한다.
단, `--config` 로 **명시한** 파일이 없으면 오류로 멈춘다.

### 3.5 토픽 목록 파일

목록 파일은 패키지 `config/` 가 아니라 **워크스페이스 루트 [config/](../../config/)** 에 있다 — `colcon build` 로 설치되지 않는다.

목록은 [config/recording_topics.list](../../config/recording_topics.list) **하나이고 모든 백엔드(mock · Isaac · 펑션베이)가
쓴다** — `record` 의 기본값이다. 2026-10-18 까지 Isaac 판(`recording_topics_isaac.list`)이 따로 있었는데, 갈리는 것이 카메라
토픽뿐이었고 카메라를 어느 백엔드에서도 bag 에 담지 않게 되어(아래 ⚠️) 합쳤다.

담는 것은 다음 11개다 — `/session`, `/servo_node/delta_twist_cmds`, `/ee_pose`, `/servo_node/delta_joint_cmds`,
`/target_joint_cmds`, `/joint_states`, `/gripper_cmds`, `/gripper_states`, `/scene/objects`, `/motion/trajectory`, `/motion/event`.
모두 상위 계층의 정규 이름이라 백엔드와 무관하다. 각 토픽을 왜 담는지, 카메라를 되살릴 때 백엔드별 이름이 무엇인지는 파일 안의
주석에 있다.

**형식 규칙** — 녹화와 적재가 같은 규칙으로 읽는다.

- 한 줄에 토픽 하나.
- `#` 으로 시작하는 줄과 빈 줄은 무시한다. 공백만 있는 줄과 앞에 공백이 있는 주석 줄도 무시한다.
- 토픽 앞뒤의 공백은 지운다.
- **줄 중간의 `#` 주석은 지원하지 않는다** — `/foo  # 설명` 은 그 문자열 전체가 토픽 이름이 된다.
- 목록은 시작할 때 한 번 읽는다. 녹화 도중 파일을 수정했다면 종료 후 다시 실행해야 반영된다.

사용자 목록은 다음처럼 만든다. RDFP 에피소드 적재에 필요한 `/session` 도 포함한다 — 에피소드 경계를 `/session` 의 전이로
만들기 때문이다.

```bash
cat > config/recording_topics_custom.list <<'EOF'
# 세션과 관절 상태만 기록
/session
/joint_states
EOF

./scripts/rosbag.sh record --topics-file config/recording_topics_custom.list
```

**적재 설정과 같은 목록이어야 한다.** [config/dataset_config.yaml](../../config/dataset_config.yaml) 의 `topics_file:` 이 목록을
다시 읽어 DB 에 넣을 토픽을 거른다(`DatasetConfig.effective_topics()`). 갈리면 녹화한 것과 적재하는 것이 오류 없이 어긋난다 —
§3.4 의 경고가 이것을 잡는다. `dataset_config.yaml` 의 기본값이 공용 목록이고, 수집 래퍼(`run_collect_*.sh`)가 따로 써 두는 적재
설정도 같은 목록을 가리킨다.
`--topics-file` 의 상대 경로는 **현재 작업 디렉터리 기준**, 적재 YAML 의 `topics_file` 상대 경로는 **그 YAML 파일의 디렉터리
기준**이다. 예를 들어 `config/dataset_config.yaml` 에서는 `topics_file: recording_topics_custom.list` 로 지정한다.

⚠️ **카메라 토픽을 rosbag 에 담으면서 레코더 mp4 등록도 켜 두면 `import` 가 죽는다.** 두 경로가 같은 `(episode_id, topic_id)` 로
`image_streams` 행을 넣어 UNIQUE 제약에 걸린다. `dataset_config.yaml` 은 `recordings_dir`(레코더 mp4 등록)을 켜 둔 상태이므로
공용 목록에서 카메라를 뺐다. **사용자 목록에 카메라 토픽을 넣고 레코더도 돌렸다면** 적재 전에 둘 중 하나를 끈다 — 적재의
`topics_file` 에서 카메라 토픽을 제외하거나 `recordings_dir` 을 뺀다. 이미 녹화한 bag 은 목록을 수정해도 바뀌지 않는다.
펑션베이 카메라는 압축(`CompressedImage`)이라 bag 경로로는 아예 적재할 수 없다 — 적재는 `sensor_msgs/Image` 만 인코딩하고
`CompressedImage` 는 경고(`no writer registered … ignored`)만 남기고 무시한다.

⚠️ **목록에 있는데 발행되지 않는 토픽은 조용히 비어 있다.** rosbag2 는 오류를 내지 않는다. 공용 목록의 토픽은 백엔드와 무관한
이름이지만, 카메라처럼 백엔드마다 이름이 다른 토픽을 사용자 목록에 넣을 때는 그 백엔드의 이름을 쓴다.

### 3.6 출력 — 디렉터리 구조

```text
$RDFP_ROSBAG_DIR/
└── 2026-10-17/                                   # 시작한 날 (로컬 시각)
    └── session_2026-10-17_09-00-03/              # 실행할 때마다 새로 만든다 — 이어 쓰지 않는다
        ├── metadata.yaml                         # 정상 종료 후 확인한다 (§3.7)
        ├── session_2026-10-17_09-00-03_0.mcap
        └── session_2026-10-17_09-00-03_1.mcap    # --split-duration 마다 다음 파일
```

- 세션 디렉터리 이름과 분할 파일 접두어가 같다 — `-o` 로 준 경로의 basename 을 rosbag2 가 접두어로 쓴다.
- 자정을 넘겨도 날짜 디렉터리는 **시작한 날**에 남는다.
- 이름은 언제나 `session_YYYY-MM-DD_HH-MM-SS` 형식이다. 같은 초에 다시 시작해 이름이 겹치면 **다음 초까지 기다려** 새 이름을
  만든다 — 조회 · 적재 쪽 세션 탐색(`rdfp.rosbag.catalog`)이 이 형식만 찾기 때문이다. **녹화를 동시에 둘 띄우는 것은 지원하지 않는다**
  (디렉터리는 rosbag2 가 만들므로 이름을 미리 잡아 둘 수 없다).
- 규칙의 배경은 [데이터셋_적재_시스템_설계서.md §0](데이터셋_적재_시스템_설계서.md#0-rosbag-저장-구조--운영-규칙).

시작하면 설정을 한 번 출력한다. 로그를 남겨 두면 어떤 목록으로 녹화했는지 나중에 확인할 수 있다.

```text
[INFO] Starting rosbag2 recording
[INFO] Base dir           : /data/rdfp/rosbag
[INFO] Day dir            : /data/rdfp/rosbag/2026-10-17
[INFO] Session dir        : /data/rdfp/rosbag/2026-10-17/session_2026-10-17_09-00-03
[INFO] Storage            : mcap
[INFO] Split duration     : 3600 sec
[INFO] Topics file        : /home/<user>/development/ros/rdfp_ws/scripts/../config/recording_topics.list
[INFO] Topic count        : 13
[INFO] Topics             :
[INFO]   - /session
...
```

이 문서의 날짜와 세션 이름은 예시다. 실제 명령에는 시작 로그의 `[INFO] Session dir` 값을 사용한다.

### 3.7 끝내기

**그 터미널에서 Ctrl+C(SIGINT)로 끝낸다.** rosbag2 가 마지막 분할 파일을 닫고 `metadata.yaml` 을 쓴다.

`rosbag.sh` 는 `record_rosbag.sh` 로, 그것은 다시 `ros2 bag record` 로 `exec` 하므로 녹화 프로세스가 처음 띄운 PID 를 그대로
이어받는다. 백그라운드로 띄웠다면 그 PID 에 SIGINT 를 보낸다(`run_collect_isaac.sh` 가 이렇게 한다).

```bash
./scripts/rosbag.sh record > rosbag.log 2>&1 &
RB=$!
# … 수집 …
kill -INT "$RB"; wait "$RB"
```

⚠️ **SIGKILL(`kill -9`) · 터미널 강제 종료 · 크래시로 끝나면 `metadata.yaml` 이 없거나 기록이 불완전할 수 있다.** 메타데이터가 없으면
조회 · 적재 쪽 세션 탐색이 그 세션을 "아직 녹화 중이거나 비정상 종료"로 보고 **조용히 건너뛴다** — `list-episodes` 는 빈 목록을,
`import` · `export` 는 빈 요약으로 성공을 낸다. 오류가 없다. `.mcap` 이 있는데도 아무것도 안 보이면 거의 이 경우다.
녹화 프로세스가 종료된 상태에서 다음 명령으로 메타데이터 재생성을 시도할 수 있다.

```bash
ros2 bag reindex -s mcap "$RDFP_ROSBAG_DIR/2026-10-17/session_2026-10-17_09-00-03"
```

재색인은 기록되지 않았거나 손상된 메시지까지 복원하지는 않는다. 완료 후 `ros2 bag info` 로 읽을 수 있는지 확인한다.

끝낸 뒤 확인한다.

```bash
ls "$RDFP_ROSBAG_DIR/2026-10-17/session_2026-10-17_09-00-03/metadata.yaml"
ros2 bag info "$RDFP_ROSBAG_DIR/2026-10-17/session_2026-10-17_09-00-03"    # 담긴 토픽과 메시지 수
```

### 3.8 수집 절차 안에서의 순서

1. 로봇 스택을 띄운다 — 예: `ros2 launch robot_control panda_isaac.launch.py`
2. 수집 계층을 띄우고 `session_control` 서비스가 설 때까지 기다린다 — `ros2 launch rdfp rdfp_collect.launch.py backend:=isaac`
3. **`./scripts/rosbag.sh record` 를 띄운다.**
4. 세션 · 에피소드를 진행한다 (텔레오퍼레이션, 트윈 연산, `robot-twin-collect` 등).
5. 마지막 에피소드와 세션을 닫는다.
6. **녹화를 Ctrl+C 로 끝낸다.**
7. 수집 계층 → 로봇 스택 순서로 내린다.

**녹화를 끄기 전에 수집 계층이나 스택을 먼저 내리면 마지막 에피소드의 `/session` 전이와 관절 상태가 bag 에 남지 않는다.**
적재는 `/session` 의 `IN_EPISODE → IN_SESSION` 전이로 에피소드를 자르므로, 끝 전이가 없으면 마지막 에피소드가 서지 않는다.
수집 래퍼(`run_collect_isaac.sh` · `run_collect_functionbay.sh`)는 2 → 3, 6 → 7 의 순서를 자동으로 지킨다. 백엔드별 기동 명령은
[수집_계층_얹기_절차서.md](수집_계층_얹기_절차서.md) 에 있다.

## 4. 에피소드 고르기 — 공통 선택 인자

`list-episodes` · `export` · `delete` 는 같은 선택 문법을 쓴다.

> ### 시간 기준이 둘이다 — 헷갈리면 잘못된 범위를 지운다
>
> | 기준 | 값 | 쓰이는 곳 |
> |---|---|---|
> | **log time** | rosbag 이 메시지를 **기록한** 시각. 언제나 녹화기의 벽시계 | 목록 정렬 · `--since` / `--until` |
> | **stamp** | 발행자가 찍은 `header.stamp` | 에피소드 자연키 · DB 조회 · `--start-ns` |
>
> 사용자가 지정하는 시간 범위는 **log time** 이다. `header.stamp` 는 발행자 시계라 시뮬레이터에 따라 경과시간이 섞이는 회차가 있어
> 사람이 아는 시각과 어긋날 수 있다.

| 인자 | 뜻 |
|---|---|
| `--rosbag-dir PATH` | bag 루트. 미지정 시 `--config` 의 `rosbag_dir` → `RDFP_ROSBAG_DIR`. `export` 는 이 루트에서 고르고 이 루트에서 적재한다 |
| `--dates YYYY-MM-DD …` | 날짜 폴더 선행 필터. 파일을 열기 전에 걸러 빠르다 |
| `--task LABEL` | 그 `task_label` 만 |
| `--since TIME` / `--until TIME` | log time 범위. `'YYYY-MM-DD HH:MM[:SS]'` 또는 `'YYYY-MM-DD'` (로컬 시간대) |
| `--episodes SPEC` | 목록 순번. `3` · `3-7` · `3-7,9` |
| `--start-ns NS` | 에피소드 자연키(§9.2). 여러 번 지정할 수 있다 |
| `--config PATH` | dataset 설정 YAML — **적재 여부 확인과 적재에 쓴다** |

- 여러 인자를 주면 **AND** 다.
- **시간 범위는 걸치기만 해도 포함된다.** 에피소드가 `[since, until]` 에 일부라도 겹치면 선택된다 — 경계에 걸린 것을 반쪽만 다루면
  데이터가 조용히 잘리기 때문이다.

⚠️ **`--episodes` 의 순번은 안정적이지 않다.** 순번은 그 실행의 목록 안에서만 유효하다. `--dates` 를 바꾸거나 새 세션이 녹화되면
같은 번호가 다른 에피소드를 가리킨다. **사람이 목록을 보고 바로 고를 때만 쓰고, 스크립트 · 재현에는 `--start-ns` 를 쓴다**
(목록 마지막 열에 그 값이 있다).

## 5. `list-topics` · `list-episodes` — 조회

### 5.1 `list-topics` — 토픽 요약

```bash
./scripts/rosbag.sh list-topics
./scripts/rosbag.sh list-topics --format json --sort count       # --sort name|count|rate|type
```

split 마다 MCAP `Summary` 를 열어 **토픽별 메시지 수 · 주기 · 타입 · 첫/마지막 시각**을 모아 `ls -l` 스타일로 낸다.
에피소드 경계는 보지 않는다. `metadata.yaml` 이 없는 세션은 여기서도 빠지므로(§3.7), 녹화 직후에는 `ros2 bag info <세션>` 으로 확인한다.

### 5.2 `list-episodes` — 무엇이 담겼나

```bash
./scripts/rosbag.sh list-episodes
./scripts/rosbag.sh list-episodes --config config/dataset_config.yaml --task peg_in_hole
./scripts/rosbag.sh list-episodes --config config/dataset_config.yaml --format json
```

```
   #  recorded (log time)      dur  task         result   export  session                      start_ns
   1  2026-09-11 21:14:03    12.4s  peg_in_hole  success  yes     session_2026-09-11_21-13-40  1789...
   2  2026-09-11 21:15:58    15.1s  peg_in_hole  -        -       session_2026-09-11_21-13-40  1789...
   3  2026-09-11 21:17:22     9.8s  tray_place   failure  yes     session_2026-09-11_21-13-40  1789...

3 episode(s)
```

**기록 시각 오름차순**이다. `--format json` 은 같은 내용에 `log_start_ns` · `log_stop_ns` 까지 실어 기계가 읽을 형태로 낸다.

`result` 열은 `SessionCommand.outcome` 이다. **`-` 는 실패가 아니라 판정 없음**이다 — 성패를 기록하지 않은 에피소드가 실패로
집계되면 학습셋에서 조용히 빠진다.

`export` 열은 세 값이다.

| 표시 | 뜻 |
|---|---|
| `yes` | DB 에 그 자연키의 행이 있다 (테이블 이름은 `sessions` 다) |
| `-` | 없다 |
| `?` | **확인하지 않았다** — `--config` 미지정, 또는 DB 접속 · 설정 읽기 실패 |

`?` 를 '없다' 로 읽으면 이미 적재된 것을 다시 넣게 된다. 그래서 두 값으로 뭉개지 않는다. **적재 상태로 고르는 선택
(`--only-new` · `--exported`)은 `?` 가 하나라도 있으면 거부된다.** 적재 여부는 **질의 한 번**으로 확인한다(목록의 최소~최대
`start_sec` 구간). 에피소드마다 묻지 않으므로 목록이 길어도 비용이 늘지 않는다.

## 6. `import` · `export` — DB 적재

|  | `import` | `export` |
|---|---|---|
| 넣는 범위 | 적재 설정의 범위 **전체** (설정의 필터 적용) | 선택 인자(§4)로 **고른 에피소드만** — 설정의 필터와 AND (§6.2) |
| 적재 설정 `--config` | 선택 — 없으면 `$RDFP_HOME/config/dataset_config.yaml` | **필수** — DB 접속과 mp4 출력 경로가 거기 있다 |
| bag 위치 | 설정의 `rosbag_dir` | `--rosbag-dir` > 설정의 `rosbag_dir` — 고르기와 적재가 **같은 루트**를 쓴다 |
| 이미 적재된 에피소드 | 설정의 `on_existing_episode` 정책 | 같은 정책 + `--only-new` 로 미리 건너뛸 수 있다 |
| 미리 보기 | — | `--dry-run` — 선택 결과만 출력하고 아무것도 쓰지 않는다 |

```bash
./scripts/rosbag.sh import                                         # 기본 설정 — RDFP_ROSBAG_DIR 이 필요하다
./scripts/rosbag.sh import --config /data/rdfp/dataset.yaml        # run_collect_isaac.sh 가 만든 설정

# 시간 범위
./scripts/rosbag.sh export --config config/dataset_config.yaml \
    --since '2026-09-11 21:00' --until '2026-09-11 22:00'
# 아직 적재되지 않은 것만
./scripts/rosbag.sh export --config config/dataset_config.yaml --only-new
# 목록에서 고른 뒤 먼저 확인
./scripts/rosbag.sh export --config config/dataset_config.yaml --episodes 2-3 --dry-run
```

**`export` 는 자체 적재 경로가 없다.** 선택 결과를 자연키 목록으로 바꿔 `episode_filter.start_ns_in` 에 실어 **`import` 와 같은
파이프라인**에 넘긴다. 덕분에 `on_existing_episode` 정책, 요약 집계, mp4 라우팅, 에피소드 단위 트랜잭션이 한 벌로 유지된다.
즉 `export` 는 "`import` 에 선택을 더한 것" 이다. `--rosbag-dir` 로 고른 루트도 함께 넘겨 설정의 `rosbag_dir` 대신 쓰게 한다 —
예전에는 넘기지 않아, 자연키만 들고 설정의 루트를 다시 뒤져 아무것도 안 들어가거나 같은 키를 가진 **다른 에피소드**가 들어갈 수
있었다(2026-10-17 수정).

⚠️ **`--config` 없이 `import` 하면 `RDFP_ROSBAG_DIR` 을 요구한다.** 기본 설정의 `rosbag_dir` 은 `${RDFP_ROSBAG_DIR:-$HOME/rosbag}`
이라, 환경변수가 없으면 녹화와 다른 곳(`$HOME/rosbag`)을 조용히 보기 때문이다. `--config` 를 주면 그 설정을 고른 것으로 보고
묻지 않는다.

⚠️ **적재가 끝난 세션은 `RDFP_ROSBAG_DIR` 밖으로 옮긴다**(또는 `delete --exported`, §7). 시뮬레이터를 재기동하면 sim 시계가
0 으로 되감겨 에피소드 키가 회차 사이에 겹치고, 옛 bag 과 함께 적재하면 새 에피소드의 관절 데이터가 **조용히 0 건**이 된다(§9.2).

### 6.1 `--only-new` 와 `on_existing_episode: skip` 은 다르다

|  | 판정 시점 | 결과 |
|---|---|---|
| `on_existing_episode: skip` | 에피소드 구간을 **읽은 뒤** | 중복은 건너뛰지만 **읽는 비용을 치른다** |
| `--only-new` | 읽기 **전** | 중복은 아예 스트리밍하지 않는다 |

둘은 겹치지 않는다. 대량 재적재에서는 `--only-new` 가 시간을 실제로 줄인다.

### 6.2 설정 파일로 범위를 고정한다

`dataset_config.yaml` 의 `episode_filter` 로도 적재 범위를 정할 수 있다. `export` 와 함께 쓰면 두 가지가 다르게 합쳐진다.

- **자연키는 대신한다.** `export` 가 고른 에피소드의 자연키 목록이 설정의 `start_ns_in` 을 **대체**한다(교집합이 아니다).
- **나머지 필터는 함께 적용된다(AND).** 작업 포함 · 제외(`task_labels_include` · `task_labels_exclude`)와 최소 길이(`min_duration_sec`)는
  그대로 걸린다. 예를 들어 `--task B` 로 골라도 설정이 `task_labels_include: [A]` 면 **아무것도 적재되지 않는다** — 선택으로 품질
  기준을 우회하는 경로를 만들지 않기 위해서다.

```yaml
episode_filter:
  task_labels_include: [peg_in_hole]
  min_duration_sec: 3.0
  start_ns_in: [1789131071582800000]   # 자연키로 못박는다
```

설정 키 전체와 적재 운영은 [데이터셋_적재_운영_가이드.md](데이터셋_적재_운영_가이드.md).

## 7. `delete` · `clear` — 삭제

```bash
# 반드시 먼저 dry-run 으로 대상을 확인한다
./scripts/rosbag.sh delete --since '2026-09-11 21:00' --until '2026-09-11 22:00' --dry-run
# 확인 후 실행 — 터미널에서는 묻고, --yes 를 주면 묻지 않는다
./scripts/rosbag.sh delete --since '2026-09-11 21:00' --until '2026-09-11 22:00' --yes
# 이미 DB 에 들어간 것만 정리 (디스크 회수)
./scripts/rosbag.sh delete --exported --config config/dataset_config.yaml --yes

# 루트 아래 세션 전부 — 터미널에서 y 를 입력해야 지운다
./scripts/rosbag.sh clear
```

- 터미널에서는 확인을 받고, 터미널이 아닌 곳(스크립트 · 파이프)에서는 **`--yes` 없이 거부한다.** 통합 명령은 `--yes` 를 대신 붙이지
  않는다 — 묻지 않고 지우려면 사용자가 명시한다.
- `--exported` 는 **DB 에 이미 적재된 에피소드만** 고른다 — 적재를 마친 bag 을 비우는 용도다. `--config` 가 함께 있어야 한다
  (없으면 적재 여부가 `?` 라 거부된다, §5.2).
- 조회(`list-episodes`) · 미리 보기(`--dry-run`) · 삭제에는 **같은 루트 지정(`--config` 또는 `--rosbag-dir`)**을 쓴다. 조회만 `--config` 로
  하고 삭제에서 빼면 삭제는 `RDFP_ROSBAG_DIR` 루트에서 대상을 찾는다 — 두 루트가 다르고 자연키까지 겹치면(§9.2) **다른 에피소드를 지운다.**
- `clear` 는 날짜 폴더(`YYYY-MM-DD`) 아래 **이름 규칙에 맞는** 세션 디렉터리(`session_YYYY-MM-DD_HH-MM-SS`)를 `metadata.yaml` 유무와
  관계없이 지우고, 비게 된 날짜 디렉터리도 지운다. 에피소드 단위로 지우려면 `delete` 를 쓴다. **아직 적재하지 않은 bag 도 함께 지워진다.**
- 이름 규칙에 맞지 않는 디렉터리 — 옛 녹화 스크립트가 같은 초에 다시 시작하며 만든 `session_…_1` 등 — 는 **조회 · 적재 · `clear` 모두에서
  보이지 않는다.** 남아 있으면 손으로 지운다.

### 7.1 `delete` 는 어떻게 지우는가 — split 을 다시 쓴다

split 은 기본 **1시간** 단위라 에피소드 하나보다 훨씬 크다. 통째로 지우면 같은 split 의 다른 에피소드가 함께 사라진다. 그래서
이렇게 한다.

1. MCAP 를 읽어 **지울 구간의 메시지만 빼고** 임시 파일에 새로 쓴다. 메시지 본문은 역직렬화하지 않고 **바이트째 복사**하며,
   스키마 · 채널도 원본 레코드를 그대로 재등록한다 — 인코딩과 메타데이터가 보존된다.
2. 원본을 그 파일로 **원자적으로 교체**한다 (`os.replace`).
3. `metadata.yaml` 의 메시지 수 · 시작 시각 · 구간 · 토픽별 개수를 다시 계산한다. 세션 탐색(`discover_splits`)이 이 파일을 읽으므로,
   고치지 않으면 카탈로그가 낡은 값을 보고한다.
4. **에피소드를 다시 감지해 검증한다** — 지우려던 것만 사라지고 나머지가 그대로인지 본다. 어긋나면 `RuntimeError` 로 멈추고
   무엇이 어긋났는지 보고한다.

경계의 `/session` 메시지까지 함께 지운다. 시작 이벤트가 남으면 목록에 껍데기 에피소드가 계속 보이고, 종료 이벤트가 남으면
**다음 에피소드의 경계 판정이 틀어진다.**

### 7.2 삭제는 되돌릴 수 없다

`delete` 는 split 을 새로 써서 원본을 교체한다. **백업이 없으면 복구 수단이 없다.**

- 반드시 `--dry-run` 으로 대상을 먼저 확인한다.
- `--exported` 는 "DB 에 있다" 만 본다 — **mp4 는 확인하지 않는다** (§9.3).
- 검증(4단계)이 실패하면 그 시점에 멈추지만, **이미 교체된 split 은 되돌아가지 않는다.** 중요한 데이터라면 세션 디렉터리를
  복사해 두고 실행한다.

## 8. 활용 절차

### 8.1 녹화 → 적재 → 정리

```bash
# ⓪ 녹화한다 — 끝낼 때 Ctrl+C (§3)
./scripts/rosbag.sh record

# ① 무엇이 담겼는지 본다 (적재 여부까지)
./scripts/rosbag.sh list-episodes --config config/dataset_config.yaml

# ② 아직 안 들어간 것만 적재
./scripts/rosbag.sh export --config config/dataset_config.yaml --only-new

# ③ 들어간 것을 확인하고
./scripts/rosbag.sh list-episodes --config config/dataset_config.yaml

# ④ 적재된 것만 bag 에서 지워 디스크를 회수
./scripts/rosbag.sh delete --exported --config config/dataset_config.yaml --dry-run
./scripts/rosbag.sh delete --exported --config config/dataset_config.yaml --yes
```

③ 을 건너뛰지 않는다 — ②가 일부만 성공했을 수 있고, ④는 파괴적이다. 예시의 적재 설정은 기본 설정이다 — 수집 래퍼
(`run_collect_isaac.sh` · `run_collect_functionbay.sh`)로 모았다면 그것이 데이터 루트에 써 둔 `dataset.yaml` 을 쓴다
(`rosbag_dir` · `recordings_dir` 이 그 루트를 가리킨다).

### 8.2 특정 작업만 데이터셋에 넣기

```bash
./scripts/rosbag.sh list-episodes --config config/dataset_config.yaml --task peg_in_hole
./scripts/rosbag.sh export --config config/dataset_config.yaml --task peg_in_hole --only-new
```

### 8.3 실패한 시도를 버리기

성패는 목록의 `result` 열에 있다. **성패 필터는 없으므로** 순번이나 자연키로 고른다.

```bash
./scripts/rosbag.sh list-episodes --config config/dataset_config.yaml     # result 열 확인
./scripts/rosbag.sh delete --config config/dataset_config.yaml --start-ns 1789131071582800000 --dry-run
./scripts/rosbag.sh delete --config config/dataset_config.yaml --start-ns 1789131071582800000 --yes
```

세 명령에 같은 `--config` 를 준다 — 조회한 루트와 삭제하는 루트가 같아야 한다(§7).

### 8.4 "녹화는 됐는데 DB 에 없다" 진단

```bash
./scripts/rosbag.sh list-topics                                          # 토픽 · 건수는 있는가
./scripts/rosbag.sh list-episodes --config config/dataset_config.yaml    # 에피소드로 잡히는가 · 적재됐는가
```

- `list-topics` 에도 안 보인다 → `metadata.yaml` 누락 (§3.7)
- 토픽은 보이는데 에피소드가 0 → `/session` 이 녹화 목록에 없거나, 종료 stamp 가 시작보다 크지 않다 (§9.1)
- 에피소드는 보이는데 `export` 가 `-` → 아직 적재 안 됨. `./scripts/rosbag.sh export --config config/dataset_config.yaml --only-new`.
  `?` 이면 `--config` 와 DB 접속을 확인한다 (§5.2)
- 에피소드 · 적재는 됐는데 특정 토픽만 없다 → 적재 설정의 `topics_file` 이 녹화 목록과 다르거나(§3.4), 그 토픽의 stamp 기준이
  `/session` 과 다르다(§9.1)

### 8.5 특정 구간만 다시 적재

재적재는 설정의 `on_existing_episode` 가 정한다 (`skip` 기본 / `replace` / `error`). 기존 행을 지우고 새로 넣으려면
`on_existing_episode: replace` 로 둔 설정으로 `export` 한다 — **DB 의 `id` 가 바뀐다** (§9.2).

```bash
./scripts/rosbag.sh export --config config/dataset_config_replace.yaml --episodes 5 --dry-run   # 대상 확인
./scripts/rosbag.sh export --config config/dataset_config_replace.yaml --episodes 5
```

`dataset_config_replace.yaml` 은 예시 이름이다 — 기본 설정을 복사해 `on_existing_episode` 만 바꾼 파일이다.

## 9. 주의 사항

`metadata.yaml` 누락은 §3.7, 백엔드마다 다른 토픽 목록은 §3.5, 삭제의 비가역성은 §7.2 에 있다.

### 9.1 `header.stamp` 가 어긋나면 조용히 빠진다

적재는 모든 메시지를 `header.stamp` 로 정렬하고, `/session` 의 시작 · 종료 stamp 로 에피소드 창을 만든다. 어긋나는 방식에 따라
빠지는 것이 다르다.

| 경우 | 결과 |
|---|---|
| 메시지에 `header` 필드 자체가 없다 | 읽기 단계에서 건너뛴다 — bag 에는 있지만 적재되지 않는다 |
| `/session` 의 종료 stamp 가 시작 stamp 보다 크지 않다 (예: 둘 다 0) | 경고(`episode stop_ts (…) not after start_ts (…); skipping`)를 남기고 **그 에피소드를 버린다** |
| `/session` 과 데이터 토픽의 시계 기준이 다르다(sim time ↔ 벽시계), 또는 데이터 메시지의 stamp 가 0 이다 | 에피소드는 잡히지만 그 토픽의 메시지가 **에피소드 창 밖으로 떨어져** 0 건으로 적재된다 |

stamp 0 을 일괄로 버리지는 않는다 — 시작만 0 이고 종료가 그보다 크면 에피소드는 만들어진다(창이 0 부터 시작한다).
**벽시계를 쓰는 것 자체는 문제가 아니다 — 모든 발행자가 같은 시계 기준이면 된다.** 대표적인 사고는 sim time 스택에서
`use_sim_time=False` 로 도는 발행자가 자기 시계로 찍는 경우다 — 그 값은 벽시계라 sim time 창 밖으로 떨어진다.
`ros2 bag info` 로는 메시지가 보이므로 원인을 찾기 어렵다.

### 9.2 에피소드 식별자는 유도된 자연키다

rosbag2 안에는 **에피소드라는 개체도, 그 식별자도 없다.** `SessionCommand` 에 id 필드가 없고 세션 노드도 만들지 않는다.
식별자로 쓰는 것은 시작 `/session` 메시지의 `header.stamp`(= `start_ns`)이고, DB 의 `sessions.UNIQUE (start_sec, start_nanosec)` 이
같은 값이라 bag ↔ DB 를 잇는다. DB 의 `id` 는 `BIGSERIAL` 대리키라 bag 만 보고는 알 수 없고, `on_existing_episode: replace` 로
재적재하면 **바뀐다.**

⚠️ **`start_ns` 가 언제나 유일하지는 않다.** `header.stamp` 가 경과시간인 회차에서는 시뮬레이터를 재시작할 때 0 부터 다시
시작하므로 **서로 다른 세션의 에피소드가 같은 키**를 갖는다. 그러면 `export` 열의 `yes` 가 다른 세션의 에피소드를 가리킬 수
있다. 목록에 세션 이름이 함께 나오는 이유이며, 의심되면 수집 전에 stamp 기준을 확인한다. 적재가 끝난 세션을 루트 밖으로
옮기라는 이유(§6)도 이것이다.

### 9.3 "적재됨" 의 기준은 `sessions` 행 하나다

mp4 나 `image_streams` 행은 보지 않는다. `image_streams` 까지 보면 더 엄격해지지만 **이미지 토픽이 없는 에피소드와 구분할 수
없다.** 영상까지 확인해야 하면 `stats` 로 DB 쪽 집계를 본다.

### 9.4 split 하나는 메모리에 올려 정렬한다

읽기 계층이 split 전체를 적재한 뒤 `header.stamp` 로 정렬한다. split 크기가 곧 메모리 요구량이므로, 녹화의 분할 간격
(`record --split-duration`, 기본 3600초)이 이 비용을 정한다.

### 9.5 없는 기능

- **메시지 내용을 보는 명령이 없다.** 개별 값은 `ros2 bag play` / `ros2 bag info` 같은 표준 도구로 보거나, 적재 후 DB 를 조회한다.
- **성패(`result`) 필터가 없다.** 목록을 보고 `--start-ns` 로 고른다.
- **bag 을 직접 재생하는 경로가 없다.** 재생은 DB 를 거친다.
- **에피소드 단위 이동 · 복사가 없다.** 지우는 것만 된다.
- **크기 기준 분할이 없다.** 녹화는 시간 기준으로만 나눈다(§3.2).

## 10. 옛 스크립트에서 옮기기

| 옛 명령 | 새 명령 | 달라진 점 |
|---|---|---|
| `./scripts/record_rosbag.sh` | `./scripts/rosbag.sh record` | 옛 스크립트는 녹화 구현으로 남는다. 직접 실행하면 이제 **명령줄 인자를 거부**하고(예전엔 무시), **`RDFP_ROSBAG_DIR` 이 필수**이며(예전 기본 `~/rdfp_data/rosbag`), overlay 를 검사한다 |
| `RECORDING_TOPICS_FILE=… SPLIT_DURATION_SEC=… ./scripts/record_rosbag.sh` | `./scripts/rosbag.sh record --topics-file … --split-duration …` | 환경변수 방식도 `rosbag.sh record` 에서 그대로 통한다 |
| `./scripts/list_rosbag_topics.sh` | `./scripts/rosbag.sh list-topics` | 옛 스크립트는 **지웠다**(2026-10-17) |
| `./scripts/import_rosbag.sh` | `./scripts/rosbag.sh import` | 옛 스크립트는 **지웠다**(2026-10-17). `--config` 없이 실행하면 `RDFP_ROSBAG_DIR` 이 필요하다 |
| `./scripts/clear_rosbag.sh` | `./scripts/rosbag.sh clear` | 옛 스크립트는 **지웠다**(2026-10-17). ⚠️ **묻지 않고 지우지 않는다** — 옛 스크립트는 `--yes` 를 박아 두었다. 터미널 밖에서 쓰던 자동화는 `--yes` 를 명시해야 한다 |
| `ros2 run rdfp rosbag <명령> …` | `./scripts/rosbag.sh <명령> …` | 직접 불러도 된다 — 통합 진입점은 같은 옵션을 그대로 넘긴다 |
| `export --rosbag-dir DIR --config …` | (같음) | 이제 **고른 루트(`DIR`)에서 적재한다.** 예전에는 설정의 `rosbag_dir` 에서 다시 찾아, 두 루트가 다르면 엉뚱한 에피소드가 적재될 수 있었다 |

세션 디렉터리 이름 규칙도 바뀌었다 — 같은 초에 다시 시작하면 예전에는 `session_…_1` 처럼 접미사를 붙였다. 이제는 다음 초 이름을
쓴다. 남아 있는 옛 디렉터리는 §7 을 본다.

## 11. 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| `unknown command` / `record: unknown option` (종료 코드 2) | 명령 이름 오타, `record` 에 없는 옵션 | `./scripts/rosbag.sh help` / `help record` |
| `record: bag root is not set` · `rosbag directory not specified` | 저장 루트를 정하지 않았다 | `rdfp_env` 또는 `--rosbag-dir` |
| `import: RDFP_ROSBAG_DIR is not set …` | 기본 적재 설정으로 적재하는데 환경변수가 없다 | `rdfp_env`, 또는 `--config` 로 설정을 고른다 |
| `--config is required for export` | `export` 에 적재 설정을 주지 않았다 | `--config config/dataset_config.yaml` |
| `ros2 command not found` | ROS 환경을 소싱하지 않았다 | `rdfp_env` |
| `Package 'rdfp_msgs' not found` (record) · `Package 'rdfp' not found` (나머지) | 워크스페이스 overlay 를 소싱하지 않았다 | `rdfp_env` 또는 `source install/setup.bash` |
| `STORAGE_ID=mcap requires 'ros-humble-rosbag2-storage-mcap'` | MCAP 플러그인이 없다 | `sudo apt install ros-humble-rosbag2-storage-mcap` |
| `Less than 10 GB free space` | 저장 위치 공간 부족 | 적재가 끝난 세션을 정리하거나 `--rosbag-dir` 을 다른 디스크로 |
| `Recording topics file not found` | 목록 경로가 틀렸다 | 상대 경로는 현재 디렉터리 기준이다 — 워크스페이스 루트에서 실행하거나 절대 경로로 준다 |
| `No topics declared` | 목록이 비었거나 주석 · 빈 줄만 있다 | 한 줄에 하나씩 토픽 이름을 적는다 |
| 저장 디렉터리 생성 시 `Permission denied` | 저장 위치에 쓰기 권한이 없다 | 쓰기 가능한 경로를 `--rosbag-dir` 로 지정한다 |
| `record_rosbag.sh takes no arguments` | 녹화 구현을 인자와 함께 직접 실행했다 | `./scripts/rosbag.sh record` 의 옵션으로 준다 |
| 녹화 시작 때 `Recorded but NOT imported` | 녹화 목록에 적재 설정이 받지 않는 토픽이 있다 | 의도가 아니면 목록이나 적재 설정의 `topics_file` 을 맞춘다 (§3.4) |
| 녹화 시작 때 `reads bags from …` | 적재 설정의 `rosbag_dir` 과 녹화 루트가 다르다 | 같은 루트로 녹화하거나, 적재 때 그 루트를 가리키는 설정을 쓴다 (§3.4) |
| `list-episodes` 가 빈 목록 · `import` 가 `found 0 finalized split(s)` 로 빈 요약 | `metadata.yaml` 이 없다(§3.7), 녹화와 적재의 루트가 다르다(§3.4), 세션 디렉터리 이름이 규칙에 맞지 않는다(§7) | `ros2 bag reindex -s mcap <세션>` / 루트를 맞춘다 |
| 토픽은 보이는데 에피소드가 0 | `/session` 이 녹화되지 않았거나, `/session` 의 종료 stamp 가 시작보다 크지 않다 | 목록에 `/session` 을 넣는다 / 발행자를 고친다 (§9.1) |
| `export` 가 고른 에피소드를 일부만 적재하거나 아무것도 적재하지 않는다 | 설정의 `episode_filter`(작업 포함 · 제외, 최소 길이)가 걸렀다 — 선택과 AND 다 | 설정의 필터를 확인한다 (§6.2) |
| `--only-new` · `--exported` 가 거부된다 | `export` 열에 `?` 가 있다 — `--config` 가 없거나 DB 접속 · 설정 읽기에 실패했다 | `--config` 를 주고 `RDFP_DB_DSN` · DB 상태를 확인한다 (§5.2) |
| `import` 가 `image_streams` UNIQUE 위반으로 죽는다 | 목록에 카메라 토픽이 있고 `recordings_dir` 도 켜져 있다 | 둘 중 하나만 켠다 (§3.5) |
| `clear` · `delete` 가 대상만 출력하고 끝난다 | 터미널이 아닌 곳에서 `--yes` 없이 실행했다 | 대상을 확인한 뒤 `--yes` 를 명시한다 |
| bag 에는 있는데 DB 에 없다 | 적재 설정의 `topics_file` 이 녹화 목록과 다르다 / 그 토픽의 stamp 가 0 이거나 `/session` 과 시계 기준이 다르다 / 재기동 전 bag 과 함께 적재했다 | 목록을 맞춘다 / 발행자를 고친다 (§9.1) / 적재한 bag 을 루트 밖으로 뺀다 (§6 · §8.4) |
| Isaac bag 에 이미지가 없다 | 정상이다 — Isaac 목록은 카메라를 일부러 뺐다 | 영상은 레코더 mp4 에서 본다 |

## 12. 개발자용 — Python API · 관련 파일

명령으로 안 되는 조회는 라이브러리로 한다. ROS 없이 import 된다 (`mcap_ros2` 만 필요).

```python
from rdfp.rosbag.episode_ops import build_views, load_episodes, select

episodes = load_episodes('/data/rdfp/rosbag', dates=['2026-09-11'])
views = build_views(episodes, exported_keys=None)          # None = 확인 안 함
picked = select(views, task='peg_in_hole', since_ns=t0, until_ns=t1)
for v in picked:
    print(v.index, v.start_ns, v.bag.session_name, v.bag.episode.success)
```

메시지를 직접 읽으려면 세 함수를 쓴다.

| 함수 | 하는 일 |
|---|---|
| `catalog.discover_splits(dir, dates=, topics=)` | `Split` 목록 (경로 · 세션 · 시각 · 건수 · 토픽) |
| `mcap_reader.iter_split_messages(path, topics=, time_range_ns=)` | split 하나를 `header.stamp` 순으로. `TimedMessage`(`stamp_ns` · `topic` · `type_name` · `msg` · `log_time_ns`) |
| `merged_stream.merge_split_streams(paths, topics=, time_range_ns=)` | 여러 split 을 `heapq.merge` 로 stamp 순 단일 스트림 |

> `mcap_ros2` 의 `start_time`/`end_time` 은 **`log_time` 기준**이라 `iter_split_messages` 는 그 인자를 쓰지 않고 `header.stamp` 로
> 다시 필터한다. 두 시각은 같지 않다.

적재 파이프라인도 같은 API 를 쓴다 — split 수집(`discover_splits`), 에피소드 경계 검출(`episode.from_bag.episodes_from_splits`,
**명령과 같은 함수**), 토픽 분류, 에피소드 구간 스트리밍 적재.

| 경로 | 내용 |
|---|---|
| `scripts/rosbag.sh` | 통합 진입점 — 명령 분기, `record` 옵션 해석, 적재 설정 대조 |
| `scripts/record_rosbag.sh` | 녹화 구현 — 사전 검사, 세션 디렉터리 이름, `ros2 bag record` |
| `src/rdfp/rdfp/rosbag/cli.py` | 서브커맨드 · 출력 포맷 |
| `src/rdfp/rdfp/rosbag/episode_ops.py` | 선택 · 적재 여부 · 삭제(MCAP 재작성 · metadata 재계산 · 검증) |
| `src/rdfp/rdfp/rosbag/catalog.py` | split 탐색 · `Split` · 세션 디렉터리 이름 규칙(`SESSION_DIR_RE`) |
| `src/rdfp/rdfp/rosbag/mcap_reader.py` | split 하나 읽기 · `TimedMessage` |
| `src/rdfp/rdfp/rosbag/merged_stream.py` | k-way 시각 머지 |
| `src/rdfp/rdfp/dataset/episode_key.py` | 자연키 (`split_ns` / `join_ns` / `fetch_exported_keys`) |
| `src/rdfp/rdfp/dataset/ingest/episode/from_bag.py` | split → 에피소드 (`BagEpisode`) |
| `src/rdfp/rdfp/dataset/ingest/episode/detector.py` | `/session` 전이 → `Episode` |
| `src/rdfp/rdfp/rosbag/tests/` | `test_catalog.py` · `test_rosbag_cli.py` · `test_episode_ops.py` + fixtures |

관련 문서:

- [데이터셋_적재_시스템_설계서.md](데이터셋_적재_시스템_설계서.md) — rosbag 저장 · 분할 규칙(§0), 에피소드 분할 규칙과 적재 설계
- [데이터셋_적재_운영_가이드.md](데이터셋_적재_운영_가이드.md) — `import` · `stats` · `list` · `replay`, 적재 설정
