# rdfp_ws

Franka Emika Panda 로봇용 ROS 2 (Humble) 워크스페이스. 두 계층으로 나뉜다.

- **제어 계층** — MoveIt2 계획 · 실행(JTC / JGPC), 그리퍼 · 카메라 · scene 상태, 백엔드
  (mock · Isaac Sim · 펑션베이)별 스택, 그리고 제어 계층을 REST 로 노출하는 로봇 트윈.
  학습 데이터 수집 없이 **단독으로** 쓸 수 있다.
- **수집 계층** — 위 스택 위에 얹어 모방 학습 데이터를 만든다. 세션 · 에피소드 생명주기,
  세션 구동 녹화(mp4), rosbag2 → PostgreSQL 적재, 학습 샘플 export, 재생 CLI · GUI, 텔레오퍼레이션.

> **처음 읽는다면** [docs/rdfp_프레임워크_전체_구조와_설계.md](docs/rdfp_프레임워크_전체_구조와_설계.md) — 무엇을 왜 이렇게
> 나눴는지. **학습 데이터를 만드는 흐름**(생성 → 수집 → 적재 → export → 학습)은
> [docs/collect/학습_데이터_파이프라인_개요.md](docs/collect/학습_데이터_파이프라인_개요.md) 가 한 화면에 잇는다.

## 패키지

| 패키지 | 계층 | 역할 |
|---|---|---|
| [`rdfp_msgs`](src/rdfp_msgs/) | — | 메시지 / 서비스 정의 (IDL). 먼저 빌드한다 |
| [`panda_ftsensor_robotiq`](src/panda_ftsensor_robotiq/) | — | 대상 로봇 description (벤더 URDF + Robotiq 메시). 자산만 있고 코드는 없다 |
| [`robot_control`](src/robot_control/) | **제어** | MoveIt2 클라이언트, 그리퍼 · 카메라 · scene 노드, 백엔드 프로파일(`config/backends/`), launch helper, `panda_*` launch |
| [`robot_twin`](src/robot_twin/) | 제어 | 로봇 트윈 — 제어 계층을 변수 / 연산 REST API 로 노출한다 |
| [`rdfp`](src/rdfp/) | **수집** | 세션 · 에피소드, 세션 구동 레코더, rosbag · 데이터셋 CLI(적재 · export · 재생), 텔레오퍼레이션, `rdfp_collect` 등 launch |

의존 방향은 **수집 → 제어 단방향**이다. `robot_control` / `robot_twin` 은 `rdfp` 를
import 하지 않으며, 각 패키지의 `test_layer_boundary.py` 가 빌드에서 강제한다. 트윈이
세션 연산을 쓰는 것은 import 가 아니라 entry point(`robot_twin.backends`)를 통해서다.
근거는 [설계서](docs/rdfp_프레임워크_전체_구조와_설계.md) §7.6.

### 함께 쓰는 저장소

| 저장소 | 역할 |
|---|---|
| `~/development/mdtpy/robot-twin-client` | 트윈 REST 클라이언트(표준 라이브러리만) · MCP 서버 · 작업 CLI — `robot-twin-collect`(수집 에피소드 구동) · `robot-twin-pick` · `robot-twin-stack` · `robot-twin-peg` |
| `~/development/rdfp/rdfp-train` | export 산출물로 ACT 정책을 학습한다. **ROS 밖** 별도 venv 에서 돈다 |

## 사전 요구사항

- Ubuntu 22.04 + ROS 2 Humble (MoveIt2 · `moveit_servo` · `moveit_resources_panda*`)
- PostgreSQL ≥ 14 (데이터셋 적재)
- ffmpeg (mp4 인코딩)

```bash
sudo apt install ros-humble-tf-transformations ros-humble-topic-based-ros2-control \
                 python3-yaml python3-opencv libturbojpeg ffmpeg

# 적재(mcap · pydantic · psycopg) · 트윈(fastapi · uvicorn) · 카메라(PyTurboJPEG)
pip install --user 'mcap' 'mcap-ros2-support' 'pydantic>=2' 'psycopg[binary]>=3' \
                   'fastapi' 'uvicorn[standard]' 'PyTurboJPEG'
```

`topic-based-ros2-control` 은 Isaac 백엔드용이다. USB 풋페달 클러치(`clutch_pedal`)를 쓰면
`python3-evdev` 가 더 필요하다. **ROS 워크스페이스에 venv 를 쓰지 않는다** — 이유와 pip
패키지가 apt 패키지를 가리는 함정은 [docs/python_env_guide.md](docs/python_env_guide.md).

## 환경 — `.rdfprc`

셸은 ROS 를 자동으로 source 하지 않는다. **`rdfp_env` 로 켠다** — 워크스페이스 루트의
`.rdfprc` 를 읽어 ROS 배포판 환경 위에 `install/setup.bash` 를 얹고 아래 변수를 export 한다.
구성은 [python_env_guide.md](docs/python_env_guide.md) §2.3.

| 변수 | 용도 |
|---|---|
| `RDFP_HOME` | 워크스페이스 루트. 데이터셋 CLI 가 `--config` 없을 때 `$RDFP_HOME/config/dataset_config.yaml` 을 찾는다 |
| `RDFP_DATA_ROOT` | 학습 데이터 루트(`/data/rdfp`). 수집 스크립트가 이 아래에 `rosbag/` · `recordings/` · `videos/` · `logs/` · `dataset.yaml` 을 만든다. **비어 있으면 `~/rdfp_data` 로 떨어진다** |
| `RDFP_ROSBAG_DIR` · `RDFP_VIDEOS_DIR` · `RDFP_RECORDINGS_DIR` | 위 루트에서 파생한다. rosbag CLI · `config/dataset_config.yaml` 이 읽는다 |
| `RDFP_DB_DSN` | PostgreSQL DSN. YAML 에 평문으로 두지 않는다(`db.dsn_env` 가 변수 이름을 가리킨다) |
| `RDFP_TWIN_CONFIG` | (선택) `run_robot_twin.sh` 에 임의의 트윈 설정 파일을 줄 때 |

`.rdfprc` 는 DSN 비밀번호를 평문으로 담으므로 **저장소에 넣지 않는다.** 새로 클론했다면
직접 만든다.

## 빌드

워크스페이스 루트에서 실행한다 (`src/<pkg>/` 안에서 빌드하면 그 안의 stale 산출물이
PYTHONPATH 를 가린다).

```bash
cd ~/development/ros/rdfp_ws
colcon build --packages-select rdfp_msgs robot_control robot_twin rdfp
source install/setup.bash        # 또는 rdfp_env
```

**설정 파일을 고쳐도 `colcon build` 를 해야 반영된다** — launch · 노드 · 트윈이 모두
설치본(`install/<pkg>/share/<pkg>/config/`)을 읽는다.

## 실행

### launch

제어 스택과 수집 계층은 **launch 둘로 나눠** 띄운다. `backend:=<이름>` 이 arm 명령 채널 ·
카메라 토픽 · 해상도 · `use_sim_time` 을 `config/backends/<이름>.yaml` 에서 가져온다.

```bash
# 제어 스택 — 백엔드마다 하나 (robot_control)
ros2 launch robot_control panda_mock.launch.py         # mock hardware (JTC)
ros2 launch robot_control panda_mock_jgpc.launch.py    # 위의 JGPC 판
ros2 launch robot_control panda_isaac.launch.py        # Isaac Sim (topic_based_ros2_control)
ros2 launch robot_control panda_functionbay.launch.py  # 펑션베이
ros2 launch robot_control panda_gazebo.launch.py       # Gazebo (미검증 · 보류)

# 수집 계층 — 제어 스택 **위에** 다른 터미널에서 얹는다 (rdfp)
ros2 launch rdfp rdfp_collect.launch.py backend:=isaac        # 또는 functionbay

ros2 launch rdfp replay_panda_mock.launch.py           # 재생 모드 스택
```

배선과 인자는 [제어 계열](src/robot_control/launch/README.md) · [수집 계열](src/rdfp/launch/README.md)
launch README, 얹는 절차는 [수집_계층_얹기_절차서.md](docs/collect/수집_계층_얹기_절차서.md).

### 실행 스크립트 (`scripts/`)

실제 작업은 대개 이 래퍼로 한다. 백엔드별 기동 순서와 선행 조건을 스크립트가 확인한다.

| 스크립트 | 하는 일 |
|---|---|
| `run_panda_mock.sh` · `run_panda_isaac.sh` · `run_panda_functionbay.sh` | 백엔드별 제어 스택 |
| `run_isaac_sim.sh` | Isaac Sim 을 ROS 브리지가 살아 있는 환경으로 띄운다 |
| `run_robot_twin.sh [--isaac\|--functionbay]` | 로봇 트윈 (mock :8801 · Isaac :8802 · 펑션베이 :8803) |
| `run_collect_isaac.sh` · `run_collect_functionbay.sh` | 수집 계층 + rosbag 녹화 (`--with-stack` 이면 제어 스택 · 트윈까지). 에피소드 구동은 `robot-twin-collect` 가 따로 한다 |
| `run_teleop_keyboard.sh` | 키보드 텔레오퍼레이션 |
| `rosbag.sh <record\|list-episodes\|import\|export\|delete\|clear …>` | rosbag 녹화 · 조회 · 적재 · 삭제 통합 진입점 ([가이드](docs/collect/rosbag_명령_가이드.md)) |
| `kill_stack.sh` | 로봇 스택 · 시뮬레이터 잔여 프로세스 정리 |
| `clear_dataset.sh` | DB 스키마를 비우고 적재가 만든 mp4(`videos/`)를 지운다 — 레코더 mp4 는 남는다 |
| `check_doc_links.sh` | 문서 링크 검사 (출력 없음 = 전부 유효) |

`scripts/functionbay/`(`fb_*.py`) · `scripts/isaac/`(`is_*.py`)는 백엔드별 점검 · 실험 스크립트다.
Isaac 전 과정 명령은 [isaac_bringup_runbook.md](docs/backends/isaac/isaac_bringup_runbook.md) §2,
펑션베이 기동 순서는 [펑션베이_시뮬레이터_사용법.md](docs/backends/functionbay/펑션베이_시뮬레이터_사용법.md).

### 데이터셋 CLI (각각 독립 명령)

```bash
ros2 run rdfp init-db  --dsn-env RDFP_DB_DSN [--drop --yes]
ros2 run rdfp import   --config /data/rdfp/dataset.yaml
ros2 run rdfp stats    --config /data/rdfp/dataset.yaml
ros2 run rdfp list     --config /data/rdfp/dataset.yaml
ros2 run rdfp export   --config /data/rdfp/dataset.yaml --out /data/rdfp/datasets/v1   # 학습 샘플 트리
ros2 run rdfp replay   42 --config /data/rdfp/dataset.yaml
ros2 run rdfp replay_gui --config /data/rdfp/dataset.yaml
ros2 run rdfp rosbag   list-episodes --config /data/rdfp/dataset.yaml   # bag 루트는 RDFP_ROSBAG_DIR
```

`/data/rdfp/dataset.yaml` 은 수집 스크립트가 데이터 루트에 써 두는 설정이다 — 레코더 출력 ·
rosbag 경로와 맞춰져 있어 **수집한 데이터는 이것으로 적재한다.** `--config` 를 생략하면
`$RDFP_HOME/config/dataset_config.yaml` 을 쓴다.

**ROS 가 필요한 것은 적재까지다.** `import` · `replay` · `replay_gui` · `rosbag export` 는 ROS
메시지 패키지를 import 하므로 소싱한 셸에서 돌린다. `export` 는 ROS 를 쓰지 않아 소싱하지 않은
셸에서도 `python3 -m rdfp.dataset.export_cmd`(`src/rdfp` 에서)로 돈다 — 학습 환경에서 다시
export 할 때 쓴다.

## 설정 파일

| 파일 | 위치 | 내용 |
|---|---|---|
| `config/dataset_config.yaml` | 워크스페이스 루트 | 적재 설정의 기본 · 예시. 경로 필드는 `${RDFP_*:-기본값}` 을 치환한다 |
| `config/recording_topics.list` | 워크스페이스 루트 | rosbag 녹화 목록 — **모든 백엔드 공용**이고 `dataset_config.yaml` 의 `topics_file` 도 이것을 가리킨다 |
| `config/backends/<이름>.yaml` | `robot_control` | 백엔드 프로파일 — arm 명령 채널 · servo · 카메라 · scene · `use_sim_time` |
| `config/image_pipeline.yaml` | `robot_control` | 카메라 · 뷰어 · 레코더 설정의 단일 출처 (해상도 · fps · 인코딩) |
| `config/robot_twin_*.yaml` | `robot_twin` | 트윈이 노출할 변수 · 연산 선언 (백엔드마다 하나) |

워크스페이스 루트의 `config/` 는 패키지 share 에 **설치되지 않는다.** `dataset_config.yaml` 을
다른 디렉터리로 복사하면 상대 경로인 `topics_file` 이 깨지므로 절대 경로로 바꾼다. 전체 대응표는
[CLAUDE.md](CLAUDE.md) "Build & Run".

## 운영 시 자주 만나는 함정

- **`import` 가 아무것도 적재하지 않는다** — 에러 없이 빈 summary 만 나오면 대부분 rosbag2
  세션 디렉터리에 `metadata.yaml` 이 없다(녹화가 SIGKILL · 충돌로 끝났다).
  `ros2 bag reindex -s mcap <session_dir>` 로 되살린다.
- **`required tables not found in schema`** — DB 스키마가 없다. `ros2 run rdfp init-db` 를 먼저 실행한다.
- **수집한 데이터가 `~/rdfp_data` 에 있다** — `rdfp_env`(`.rdfprc`)를 켜지 않은 셸에서 수집
  스크립트를 돌렸다. 단계마다 데이터 루트가 같아야 한다.
- **에피소드는 있는데 관절 데이터가 0 건이다** — 시뮬레이터를 재기동한 뒤의 bag 을 이전 bag 과
  같은 `rosbag_dir` 에서 함께 적재했다(sim 시계가 0 으로 되감겨 에피소드 키가 겹친다).
  적재를 마친 bag 은 `rosbag_dir` 밖으로 뺀다.
- **launch 가 YAML 을 못 찾는다** — 각 패키지는 `src/<pkg>/config/` 만 share 로 설치한다.
  워크스페이스 루트의 YAML 은 `config_file:=<절대 경로>` 로 넘긴다.
- **`ros2 run rdfp <script>` / `ros2 launch rdfp panda_*` 가 안 된다** — 제어 계층 스크립트와
  `panda_*` launch 는 `robot_control` 소속이다. 반대로 세션을 아는 `rdfp_camera_node` ·
  `rdfp_image_viewer_node` 는 `rdfp` 소속이다.
- **재빌드했는데 코드 변경이 반영되지 않는다** — `src/<pkg>/` 안의 stale `build/` · `install/`
  이 PYTHONPATH 를 가린다. `find src -maxdepth 2 \( -name install -o -name build \)` 로 찾아 지운다.

단계 *사이*에서만 생기는 함정은 [학습_데이터_파이프라인_개요.md](docs/collect/학습_데이터_파이프라인_개요.md) §6,
백엔드별 함정은 [docs/backends/README.md](docs/backends/README.md).

## 디렉터리 구조

```
rdfp_ws/
├── src/
│   ├── rdfp_msgs/               # 메시지 / 서비스 정의
│   ├── panda_ftsensor_robotiq/  # 대상 로봇 description (자산)
│   ├── robot_control/           # 제어 계층 (moveit / gripper / camera / scene / backends / launch_helpers)
│   ├── robot_twin/              # 로봇 트윈 REST 게이트웨이
│   └── rdfp/                    # 수집 계층 (session / recorder / camera / rosbag / dataset / teleop)
├── config/                      # 워크스페이스 공용 설정 — dataset_config.yaml · recording_topics.list
├── scripts/                     # 실행 래퍼 · 정리 · 점검 (functionbay/ · isaac/)
├── docs/                        # 문서 (한국어) — INDEX.md 가 색인
│   ├── rdfp_프레임워크_전체_구조와_설계.md  # 최상위 아키텍처 + 설계서
│   ├── control/  collect/  learning/  twin/  teleop/  backends/
│   └── archive/                     # 이력 (갱신하지 않는다)
├── checklist_results/           # 펑션베이 인수 체크리스트 회차별 결과
├── docker/                      # Gazebo · 재생 mock · 재생 GUI 컨테이너
├── data/                        # 검증 · 재생용 데이터 (설치되지 않는다)
├── build/  install/  log/       # colcon 산출물 (gitignored)
└── CLAUDE.md                    # Claude Code 용 작업 지침
```

## 문서

> **[docs/rdfp_프레임워크_전체_구조와_설계.md](docs/rdfp_프레임워크_전체_구조와_설계.md) — 최상위 아키텍처 + 설계서.**
> 이 워크스페이스가 무엇을 만드는지(모방 학습 데이터 생성 · 저장 · 관리), 세 서브시스템의 경계와
> 인터페이스 계약, 설계 결정의 근거. **처음 읽는다면 여기부터.**

> **[docs/collect/학습_데이터_파이프라인_개요.md](docs/collect/학습_데이터_파이프라인_개요.md) — 학습 데이터 파이프라인.**
> 여섯 단계의 순서와 데이터 흐름, 다음 단계로 넘어가도 되는 조건, 따라 치는 명령 한 화면,
> 단계 사이의 함정.

> **[docs/INDEX.md](docs/INDEX.md) — 전체 문서 색인.** 폴더별 표와 "하고 싶은 것 → 문서".
> 원하는 내용이 어느 문서에 있는지는 여기서 먼저 찾는다.

패키지 README:

- [src/robot_control/README.md](src/robot_control/README.md) — 제어 계층 구성 · 실행 · 계층 규칙.
  launch 와 helper 인벤토리는 [src/robot_control/launch/README.md](src/robot_control/launch/README.md).
- [src/robot_twin/README.md](src/robot_twin/README.md) — 백엔드별 설정 파일, 선택적인 세션 연산.
- [src/rdfp/README.md](src/rdfp/README.md) — 수집 계층 walkthrough (세션 · 레코더 · 데이터셋 CLI · 재생 GUI).
  launch 는 [src/rdfp/launch/README.md](src/rdfp/launch/README.md), 레코더 코어는
  [src/rdfp/rdfp/recorder/README.md](src/rdfp/rdfp/recorder/README.md).
- [src/rdfp_msgs/README.md](src/rdfp_msgs/README.md) — 인터페이스 목록과 변경 시 주의사항.
- [CLAUDE.md](CLAUDE.md) — Claude Code 가 읽는 빌드 · 아키텍처 · 주의사항 요약.

주제별 문서 (한국어) — 폴더는 코드 계층을 따른다:

- [docs/control/](docs/control/) — 제어 계층: MoveIt 클라이언트 · 그리퍼 · 카메라 노드 · scene 계약 (진입점 `README.md`).
- [docs/collect/](docs/collect/) — 수집 계층: `수집_계층_얹기_절차서.md`, `session_control_guide.md`, 레코더,
  `rosbag_명령_가이드.md` · `데이터셋_적재_운영_가이드.md` · `데이터셋_적재_시스템_설계서.md`, 재생,
  `학습_데이터_파이프라인_개요.md`.
- [docs/learning/](docs/learning/) — 학습 데이터: `결정_기록.md`(왜) · `학습_샘플_사양.md`(무엇) · `저장_형식.md` ·
  `미구현_목록.md` · `열린_결정.md`.
- [docs/backends/](docs/backends/) — Isaac(`isaac_bringup_runbook.md` 에 전 과정 명령) · 펑션베이 · Gazebo · 실기.
- [docs/twin/](docs/twin/) — `로봇_트윈_사용자_가이드.md` · `로봇_트윈_설계서.md` · MCP 서버.
- [docs/teleop/](docs/teleop/) — 텔레오퍼레이션 (진입점 `README.md`).
- [docs/topic_naming_contract.md](docs/topic_naming_contract.md) — 토픽 이름 규약.
- [docs/archive/](docs/archive/) — 이력 (갱신하지 않는다).

## 라이선스

Apache-2.0 (패키지 단위는 각 `package.xml` 참조).

## 관리자

kwlee — kwlee@etri.re.kr
