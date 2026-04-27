# rdfp_ws

Franka Emika Panda 로봇용 ROS 2 (Humble) 통합 워크스페이스. MoveIt2 기반
카테시안 경로 계획·실행, 카메라 캡처, MP4 녹화, 세션·에피소드 생명주기
제어, rosbag2 → PostgreSQL/MP4 데이터셋 적재, 적재된 에피소드 재생
(Tk GUI 포함) 까지를 두 패키지로 묶었다.

## 패키지

| 패키지 | 위치 | 역할 |
|---|---|---|
| `rdfp` | [src/rdfp/](src/rdfp/) | 애플리케이션 — MoveIt2 launch 스택, 카메라/녹화 노드, 데이터셋 ingestion / replay CLI · GUI |
| `rdfp_msgs` | [src/rdfp_msgs/](src/rdfp_msgs/) | 서비스 / 메시지 인터페이스 정의 |

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
colcon build --packages-select rdfp_msgs rdfp
source install/setup.bash
```

대표 launch:

```bash
ros2 launch rdfp panda_mock.launch.py            # MoveIt2 mock 단독
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
| `RDFP_DB_DSN` | PostgreSQL DSN (필수). yaml 에 평문으로 두지 않음. |
| `RDFP_POSTPROC_LOG_DIR` | JSONL 로그 저장 경로 (선택, 기본 `<output_mp4_dir>/_logs/`) |
| `RDFP_POSTPROC_BATCH_SIZE` | DB 배치 INSERT 버퍼 크기 (선택, 기본 1000) |

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
│   ├── rdfp/                  # 애플리케이션 패키지 (자세한 README 별도)
│   └── rdfp_msgs/             # 메시지 / 서비스 정의
├── docs/                      # 설계서 / 사용 설명서 / 검증 절차 (한국어)
│   ├── rosbag2/               # 데이터셋 후처리기 사용/설계 문서
│   ├── moveit/  recorder/  session/  replay/  camera/
│   └── plan.md
├── build/  install/  log/     # colcon 산출물 (gitignored)
└── CLAUDE.md                  # Claude Code 용 작업 지침
```

## 문서

- [src/rdfp/README.md](src/rdfp/README.md) — 애플리케이션 패키지 사용 walkthrough.
- [src/rdfp/launch/README.md](src/rdfp/launch/README.md) — launch 파일 / helper 인벤토리.
- [src/rdfp/rdfp/recorder/README.md](src/rdfp/rdfp/recorder/README.md) — `FFMpegMp4Recorder` 와 `image_recorder_node`.
- [docs/rosbag2/](docs/rosbag2/) — 데이터셋 후처리기 CLI 설명서, 설계서, 검증 절차.
- [CLAUDE.md](CLAUDE.md) — Claude Code 가 읽는 빌드/아키텍처/주의사항 요약.

## 라이선스

Apache-2.0 (패키지 단위는 각 `package.xml` 참조).

## 관리자

kwlee — kwlee@etri.re.kr
