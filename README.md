# rdfp_ws

Franka Emika Panda 로봇용 ROS 2 (Humble) 통합 워크스페이스. MoveIt2 기반
카테시안 경로 계획·실행, 카메라 캡처, MP4 녹화, 세션·에피소드 생명주기
제어, rosbag2 → PostgreSQL/MP4 데이터셋 적재, 적재된 에피소드 재생
(Tk GUI 포함) 까지를 네 패키지로 묶었다.

## 패키지

| 패키지 | 위치 | 역할 |
|---|---|---|
| `rdfp_msgs` | [src/rdfp_msgs/](src/rdfp_msgs/) | 서비스 / 메시지 인터페이스 정의 |
| `robot_control` | [src/robot_control/](src/robot_control/) | **로봇 제어 계층** — MoveIt2 클라이언트, 카메라, scene 상태, launch helper, `panda_*` launch. 학습 데이터 수집 없이 단독 사용 가능 |
| `robot_twin` | [src/robot_twin/](src/robot_twin/) | 로봇 트윈 — 제어 계층을 변수/연산 REST API 로 노출 |
| `rdfp` | [src/rdfp/](src/rdfp/) | **학습 데이터 수집 계층** — 세션/에피소드 생명주기, 녹화, 데이터셋 ingestion / replay CLI · GUI, teleop, `rdfp_*` launch |

의존 방향은 위에서 아래로 **단방향**이다. `robot_control` / `robot_twin` 은 `rdfp` 를
import 하지 않으며, 이는 각 패키지의 `test_layer_boundary.py` 가 강제한다. 근거와
경위는 [docs/rdfp_framework_design.md](docs/rdfp_framework_design.md) §7.6 참고.

자세한 사용법과 API 는 [src/rdfp/README.md](src/rdfp/README.md) 참고.

## 사전 요구사항

- Ubuntu 22.04 + ROS 2 Humble
- PostgreSQL ≥ 14 (데이터셋 적재용)
- ffmpeg (MP4 인코딩)

```bash
sudo apt install ros-humble-tf-transformations \
                 python3-yaml python3-opencv ffmpeg

pip install --user 'mcap' 'mcap-ros2-support' 'pydantic>=2' 'psycopg[binary]>=3'
```

## 빌드 & 실행

워크스페이스 root 에서 실행한다 (`src/<pkg>/` 안에서 빌드하지 말 것 — stale
산출물이 PYTHONPATH 를 섀도잉할 수 있음).

```bash
cd ~/development/ros/rdfp_ws
colcon build --packages-select rdfp_msgs robot_control robot_twin rdfp
source install/setup.bash
```

대표 launch:

```bash
ros2 launch robot_control panda_mock.launch.py            # MoveIt2 mock 단독
ros2 launch rdfp rdfp_panda_mock.launch.py       # + camera / recorder / ee_pose
ros2 launch rdfp replay_panda_mock.launch.py     # replay 모드 스택
```

대표 console_script (각각 독립 명령):

```bash
ros2 run rdfp init-db --dsn-env RDFP_DB_DSN [--drop --yes]
ros2 run rdfp import   --config dataset_config.yaml
ros2 run rdfp stats    --config dataset_config.yaml
ros2 run rdfp list     --config dataset_config.yaml
ros2 run rdfp replay   42 --config dataset_config.yaml
ros2 run rdfp replay_gui --config dataset_config.yaml
ros2 run rdfp rosbag   list-episodes --config rosbag_config.yaml
```

`init-db` / `stats` / `list` / `rosbag` 은 ROS sourcing 없이도 동작한다.
`import` / `replay` / `replay_gui` 는 ROS msgs 의존이 있어 sourcing 이 필요.

## 환경 변수

| 변수 | 용도 |
|---|---|
| `RDFP_DB_DSN` | PostgreSQL DSN (필수). yaml 에 평문으로 두지 않음. 변수명 자체는 `dataset_config.yaml` 의 `db.dsn_env` 로 변경 가능. |

## 운영 시 자주 만나는 함정

- **`ros2 run rdfp import` 가 아무것도 적재하지 않음** — 빈 summary 만
  출력되고 에러는 없는 경우, 대부분 rosbag2 세션 디렉터리에
  `metadata.yaml` 이 없다 (`rosbag2` 가 SIGKILL/충돌로 graceful shutdown
  없이 끝났을 때). `ros2 bag reindex -s mcap <session_dir>` 로 `.mcap` 에서
  메타데이터를 재구성하면 됨.
- **DB 스키마 미생성으로 import 실패 (`required tables not found`)** —
  `ros2 run rdfp init-db --dsn-env RDFP_DB_DSN` 을 먼저 실행.
- **launch 가 YAML 을 못 찾음 (`panda_robot.yaml not found`)** —
  setup.py 의 `data_files` 글로브는 `src/rdfp/config/*` 만 share 로
  설치한다. 워크스페이스 root 의 YAML 은 `config_file:=<absolute-path>` 로
  넘기거나 `src/rdfp/config/` 로 옮길 것.
- **`ros2 run rdfp <script>` / `ros2 launch rdfp panda_*` 가 안 됨** — 제어 계층
  스크립트와 `panda_*` launch 는 `robot_control` 로 옮겨졌다.
  `ros2 launch robot_control panda_mock.launch.py` 처럼 패키지를 바꿔 부른다.
- **rebuild 했는데 코드 변경이 반영 안 됨** — `src/<pkg>/` 내부의 stale
  `build/` · `install/` 가 PYTHONPATH 를 섀도잉할 수 있음.
  `find src/rdfp -maxdepth 2 -name install -o -name build` 로 확인 후 제거.
  `colcon build` 는 **항상 워크스페이스 root** 에서 실행.

## 설정 파일

샘플은 [docs/rosbag2/](docs/rosbag2/) 에 있다. 실제 인스턴스는 사용자별 경로
와 DSN 정보가 들어가므로 `.gitignore` 로 제외되어 있다.

```bash
cp docs/rosbag2/dataset_config.sample.yaml dataset_config.yaml
vi dataset_config.yaml   # rosbag_dir / output_mp4_dir / db.dsn_env 등 편집
```

## 디렉터리 구조

```
rdfp_ws/
├── src/
│   ├── rdfp_msgs/            # 메시지 / 서비스 정의
│   ├── robot_control/         # 제어 계층 (moveit / camera / scene / launch_helpers)
│   ├── robot_twin/            # 로봇 트윈 REST 게이트웨이
│   └── rdfp/                 # 수집 계층 (session / recorder / dataset / teleop)
├── docs/                      # 설계서 / 사용 설명서 / 검증 절차 (한국어)
│   ├── rdfp_framework_design.md   # 최상위 아키텍처 + 설계서
│   ├── INDEX.md                   # 전체 문서 인덱스
│   ├── rosbag2/               # 데이터셋 후처리기 사용/설계 문서
│   └── moveit/  recorder/  session/  replay/  camera/  teleop/
│       robot_twin/  simulation/  environment/
├── build/  install/  log/     # colcon 산출물 (gitignored)
└── CLAUDE.md                  # Claude Code 용 작업 지침
```

## 문서

> **[docs/rdfp_framework_design.md](docs/rdfp_framework_design.md) — 최상위 아키텍처 +
> 설계서.** 이 워크스페이스가 무엇을 만드는지(imitation learning 학습 데이터 생성·저장·
> 관리), 세 서브시스템의 경계와 인터페이스 계약, 설계 결정의 근거를 다룬다. **처음
> 읽는다면 여기부터.**

> **[docs/INDEX.md](docs/INDEX.md) — 전체 문서 인덱스.** 저장소의 모든 markdown
> 문서를 주제별로 분류하고, 각 문서의 기술 내용·위치·현행 여부를 표로 정리했다.
> 원하는 내용이 어느 문서에 있는지는 여기서 먼저 찾는다.

패키지 walkthrough · 워크스페이스 hint:

- [src/rdfp/README.md](src/rdfp/README.md) — 애플리케이션 패키지 사용
  walkthrough (`MoveGroupClient`, `camera_node`, `image_recorder_node`,
  `rdfp_image_recorder`, `session_control_node`, dataset CLI, replay GUI).
- [src/robot_control/launch/README.md](src/robot_control/launch/README.md) — 제어 계열 launch
  + helper 인벤토리 (자체 완결).
- [src/rdfp/launch/README.md](src/rdfp/launch/README.md) — 수집 계열(`rdfp_*`) launch
  + YAML ↔ 인자 대응표.
- [src/rdfp/rdfp/recorder/README.md](src/rdfp/rdfp/recorder/README.md) — ROS 비의존
  `FFMpegMp4Recorder` 코어 (두 ROS 어댑터 노드는 패키지 README 에서 다룸).
- [CLAUDE.md](CLAUDE.md) — Claude Code 가 읽는 빌드/아키텍처/주의사항 요약.

서브시스템별 사용 설명서 (한국어):

- [docs/recorder/](docs/recorder/) — `ffmpeg_mp4_recorder_guide.md`,
  `image_recorder_node_guide.md`, `rdfp_image_recorder_node_guide.md`.
- [docs/session/](docs/session/) — `session_control_guide.md`,
  `session_control_client_guide.md`.
- [docs/moveit/](docs/moveit/), [docs/camera/](docs/camera/),
  [docs/replay/](docs/replay/), [docs/teleop/](docs/teleop/) — 각 서브시스템 가이드.
- [docs/rosbag2/](docs/rosbag2/) — 데이터셋 후처리기 CLI 설명서, 설계서, 검증
  절차, 운영 방안, 샘플 yaml.

## 라이선스

Apache-2.0 (패키지 단위는 각 `package.xml` 참조).

## 관리자

kwlee — kwlee@etri.re.kr
