# rdfp

**학습 데이터 수집 계층** 패키지다. 제어 스택(`robot_control`) 위에 얹어 모방 학습
데이터를 만든다 — 세션 · 에피소드 생명주기, 세션 구동 녹화(mp4), rosbag2 → PostgreSQL
적재, 학습 샘플 export, 적재된 에피소드 재생(CLI · Tk GUI), 텔레오퍼레이션.

MoveIt2 클라이언트 · 그리퍼 · 카메라 · scene 상태는 하위 패키지
[`robot_control`](../robot_control/README.md), 로봇 트윈은 [`robot_twin`](../robot_twin/README.md) 이다.
의존은 이 패키지 → `robot_control` 한 방향이며, 분리 근거는 설계서
[§7.6](../../docs/rdfp_프레임워크_전체_구조와_설계.md) 참고. 서비스 · 메시지 인터페이스는
`rdfp_msgs` 에 있으므로 함께 빌드한다.

> 워크스페이스 전체는 [../../README.md](../../README.md), 데이터를 만드는 흐름(생성 → 수집 → 적재 →
> export → 학습)은 [학습_데이터_파이프라인_개요.md](../../docs/collect/학습_데이터_파이프라인_개요.md) 가 잇는다.

## 1. 패키지 구조

```
rdfp/
├── package.xml                # ROS 2 패키지 메타데이터
├── setup.py                   # console_scripts + robot_twin.backends entry point
├── setup.cfg                  # 설치 경로 설정 (addopts = -p no:anyio — 지우지 않는다)
├── config/                    # replay_panda_mock / teleop_mirror YAML
├── launch/                    # rdfp_collect, replay_panda_mock, teleop_mirror
├── scripts/                   # 노드 단독 실행 예 (run_camera_node.sh 등)
├── resource/rdfp              # ament 리소스 마커
└── rdfp/                      # Python 소스 root
    ├── session/               # session_control_node — IDLE / IN_SESSION / IN_EPISODE 상태 머신
    │                          # twin_backend.py — robot_twin 에 세션 연산을 제공하는 entry point
    ├── camera/                # rdfp_camera_node · rdfp_image_viewer_node — /session 을 아는 카메라 · 뷰어
    ├── recorder/              # FFMpegMp4Recorder + 두 ROS 어댑터 (rdfp_image_recorder · image_recorder_node)
    ├── teleop/                # teleop_keyboard, session_teleop, teleop_retarget,
    │                          # clutch_pedal(USB 풋페달), ClutchClient
    ├── rosbag/                # MCAP catalog/discovery + `rosbag` CLI
    ├── dataset/               # DB 스키마(sql/) + 적재 파이프라인 + export(학습 샘플) + 재생
    │                          # CLI: import / stats / list / init-db / export / replay / replay_gui
    └── samples/               # 수동 샘플/데모 스크립트 (entry_point 아님)
```

## 2. Launch 구조

launch 진입점은 **계층별로 두 패키지에 나뉘어 있고**, 공유 helper 는 하위 패키지의
설치 모듈이다.

| 위치 | 내용 |
|---|---|
| `src/rdfp/launch/` | `rdfp_collect`, `replay_panda_mock`, `teleop_mirror` |
| `src/robot_control/launch/` | `panda_mock`, `panda_mock_jgpc`, `panda_isaac`, `panda_functionbay`, `panda_gazebo` |
| `robot_control/launch_helpers/` | 공유 argument 선언 · `Node` 팩토리 · startup orchestration |

```python
from robot_control.launch_helpers.camera import create_camera_node
```

**수집 = 제어 스택 launch + `rdfp_collect`.** `rdfp_collect` 는 이미 떠 있는 제어 스택 위에
수집 노드 넷(`session_control` · `rdfp_image_viewer` · `rdfp_image_recorder` ·
`target_joint_cmds_publisher`)만 띄운다. `backend:=<이름>` 이 arm 명령 채널 · 카메라 토픽 ·
해상도 · `use_sim_time` 을 `config/backends/<이름>.yaml` 에서 가져오므로 **두 launch 에 같은
백엔드를 준다.** 인자와 YAML 대응은 [launch/README.md](launch/README.md).

## 3. 활용 절차

실제 수집은 Isaac · 펑션베이에서 하고, 대개 래퍼 스크립트(`scripts/run_collect_isaac.sh` ·
`run_collect_functionbay.sh`)가 아래 단계를 묶어 처리한다. 전 과정 명령은
[isaac_bringup_runbook.md](../../docs/backends/isaac/isaac_bringup_runbook.md) §2.
여기서는 각 단계를 손으로 밟는 순서를 적는다.

### 3.1 사전 준비

#### 3.1.1 워크스페이스 빌드 및 환경 설정

```bash
cd ~/development/ros/rdfp_ws
colcon build --packages-select rdfp_msgs robot_control robot_twin rdfp
rdfp_env        # .rdfprc — install/setup.bash + RDFP_* 변수 (워크스페이스 README "환경")
```

#### 3.1.2 rosbag 저장 디렉토리 확인

* rosbag2 저장 디렉토리는 `RDFP_ROSBAG_DIR` 환경변수(또는 `scripts/rosbag.sh record --rosbag-dir`)로
  정한다. **기본값이 없다** — `rdfp_env` 가 `/data/rdfp/rosbag` 으로 설정한다.
* 저장 디렉토리의 여유 공간을 확인한다 (최소 10 GB 권장).
* 적재 설정의 `rosbag_dir` 이 녹화 루트와 같아야 한다 (`scripts/rosbag.sh record` 가 시작할 때
  대조해 다르면 경고한다).
* 녹화 대상 토픽 목록은 워크스페이스 루트의 `config/recording_topics.list` 다 — **모든 백엔드 공용**이다.

#### 3.1.3 rosbag 저장소 초기화 (필요에 따라)

설정된 rosbag 저장소에 저장된 모든 데이터를 삭제한다.

```bash
ros2 run rdfp rosbag clear
```

#### 3.1.4 PostgreSQL 데이터베이스 준비

DSN 은 환경변수 `RDFP_DB_DSN` 으로 준다 (`.rdfprc` 가 export 한다. 예:
`postgresql://user:pass@localhost:5432/rdfp`). 데이터베이스가 준비되면 스키마를 만든다.

```bash
ros2 run rdfp init-db                # 필요한 테이블이 없으면 만든다
ros2 run rdfp init-db --drop --yes   # 주의: 기존 스키마와 모든 데이터를 지운다
```

### 3.2 학습 데이터 수집

#### 3.2.1 수집 환경 확인 (옵션, 별도 터미널)

```bash
cd ~/development/ros/rdfp_ws
watch ./scripts/list_rdfp_nodes.sh     # 주요 노드 (터미널 #1)
watch ./scripts/list_rdfp_topics.sh    # 주요 토픽 (터미널 #2)
```

#### 3.2.2 제어 스택과 수집 계층 실행 (별도 터미널)

```bash
ros2 launch robot_control panda_isaac.launch.py          # 또는 panda_functionbay — 백엔드별 선행 조건은 런북
ros2 launch rdfp rdfp_collect.launch.py backend:=panda_isaac   # 다른 터미널, 같은 백엔드
```

#### 3.2.3 rosbag2 녹화 시작 (별도 터미널)

```bash
./scripts/rosbag.sh record      # 워크스페이스 루트에서. 끝낼 때는 Ctrl+C
```

녹화가 시작되면 `RDFP_ROSBAG_DIR/YYYY-MM-DD/session_YYYY-MM-DD_HH-MM-SS/` 디렉토리가
생기고, 그 안에 rosbag2 MCAP 파일이 생성된다. 옵션 · 끝내는 법은
[docs/collect/rosbag_명령_가이드.md](../../docs/collect/rosbag_명령_가이드.md).

#### 3.2.4 에피소드 구동

두 가지 길이 있다.

* **트윈으로** — `robot-twin-collect`(`~/development/mdtpy/robot-twin-client`)가 세션 · 에피소드
  경계를 찍으며 작업을 반복한다. 로봇 트윈(`scripts/run_robot_twin.sh`)이 떠 있어야 한다.
* **키보드 텔레오퍼레이션으로** — `teleop_keyboard` 는 **`backend` 파라미터가 필수**다(기본값이
  없어 안 주면 노드가 뜨지 않는다). 래퍼는 `TELEOP_BACKEND`(기본 `panda_isaac`)를 넘기고 터미널
  자동반복 속도도 맞춘다.

  ```bash
  ./scripts/run_teleop_keyboard.sh                                 # backend:=panda_isaac
  ros2 run rdfp teleop_keyboard --ros-args -p backend:=panda_functionbay
  ```

  ⚠️ `teleop_keyboard` 는 기동하면서 servo 를 시작하므로, **트윈으로 작업할 때는 띄워 두지
  않는다** — 명령이 없어도 팔이 흐르고 트윈의 MoveIt 실행과 컨트롤러를 두고 싸운다
  ([servo_vs_planned_motion.md](../../docs/teleop/servo_vs_planned_motion.md) §5).

#### 3.2.5 키보드 명령 키

`?` 키로 노드 안에서도 같은 도움말을 볼 수 있다.

| 카테고리 | 키 | 동작 |
|---|---|---|
| 이동 (Cartesian linear) | `j` / `l` | +x / -x |
|                         | `i` / `k` | +y / -y |
|                         | `q` / `a` | +z / -z |
| 이동 (Cartesian angular) | `r` / `f` | roll +x / -x |
|                          | `t` / `g` | pitch +y / -y |
|                          | `y` / `h` | yaw +z / -z |
| 이동 (Joint)            | `'` / `;` | `panda_joint1` + / - |
| 제어                    | `SPACE`   | deadman |
|                         | `x`       | twist / joint_jog 을 0 으로 즉시 발행 (stop) |
|                         | `/`       | MoveIt SRDF 의 `ready` named target 으로 이동 |
|                         | `?`       | 명령 키 도움말 출력 |
|                         | `Ctrl-C`  | 종료 |
| 그리퍼                  | `=`       | open — `gripper_cmds` 에 `goal='open'` 발행 |
|                         | `-`       | close — 같은 토픽에 `goal='close'` 발행 |
|                         | `\`       | grasp — 물체를 집을 때 (`goal='grasp'`) |
| 세션                    | `<` 또는 `,` | `start_session` |
|                         | `>` 또는 `.` | `stop_session` |
| 에피소드                | `[`       | `start_episode` |
|                         | `]`       | `stop_episode` |
| Task                    | `1`–`9`   | `tasks` 파라미터의 라벨 선택 (현재 매핑은 노드 시작 로그 참조) |
|                         | `0`       | task 라벨 해제 |

> 참고
> - **Deadman 동작**: motion 키(linear/angular/joint)는 `SPACE` 또는 motion 키를 누르는 동안만
>   발행된다. 마지막 입력으로부터 `deadman_ttl_sec`(기본 0.06초)이 지나면 0 을 한 번 내고
>   멈춘다. 터미널은 '키를 뗐다'를 알려 주지 않으므로 자동반복으로 판정한다 —
>   [key_hold_user_guide.md](../../docs/teleop/key_hold_user_guide.md).
> - **속도**는 servo 의 `scale.linear` 가 아니라 `linear_step` / `angular_step`(기본 0.25, 무차원
>   비율)으로 조절한다.
> - **One-shot 명령**: 그리퍼/세션/에피소드/task 키는 deadman 과 무관하게 누른 즉시 한 번만
>   호출된다. 수집 계층(`session_control`)이 없으면 세션 · 에피소드 · task 키는
>   `ignored: session_control not available` 로그만 남긴다.
> - **Home 이동 중**: `/` 키 처리 동안에는 servo 명령 발행이 일시 중단되어 `move_group` 의
>   trajectory 와 충돌하지 않는다.
> - **키가 안 먹으면** 노드가 찍는 servo status 경고를 본다 — 관절 한계 · 특이점에서 servo 가
>   스스로 멈춘다. `/` 로 빠져나온다.

#### 3.2.6 rosbag 에 수집된 토픽 · 에피소드 확인

```bash
ros2 run rdfp rosbag list-topics
ros2 run rdfp rosbag list-episodes --config /data/rdfp/dataset.yaml   # DB 적재 여부(export 열)까지
```

### 3.3 적재 · export

```bash
ros2 run rdfp import --config /data/rdfp/dataset.yaml
ros2 run rdfp export --config /data/rdfp/dataset.yaml --out /data/rdfp/datasets/v1
```

`/data/rdfp/dataset.yaml` 은 수집 래퍼가 데이터 루트에 써 두는 설정이다 — 레코더 출력 · rosbag
경로와 맞춰져 있다. `--config` 를 생략하면 `$RDFP_HOME/config/dataset_config.yaml` 을 쓴다.
단계를 넘어가도 되는지의 확인표와 단계 사이 함정은
[학습_데이터_파이프라인_개요.md](../../docs/collect/학습_데이터_파이프라인_개요.md) §2 · §6.

### 3.4 수집된 데이터 확인 및 재생

#### 3.4.1 재생용 mock 스택 실행 (별도 터미널)

```bash
ros2 launch rdfp replay_panda_mock.launch.py
```

arm 을 구동하는 경로는 `replay_arm_path` 로 고른다 — `ee_twist`(기본) / `target_joint_cmds` /
`none`. 자세한 내용은 [docs/collect/replay_mock_stack_guide.md](../../docs/collect/replay_mock_stack_guide.md).

#### 3.4.2 적재된 에피소드 재생 도구 실행 (Tk GUI)

```bash
ros2 run rdfp replay_gui --config /data/rdfp/dataset.yaml --ros-args -p backend:=panda_mock
```

`backend` 는 **필수**다 (기본값 없음). 붙을 스택에 맞춰 `panda_mock`(JTC) / `panda_mock_jgpc`(JGPC) 등을 준다.

## 4. 의존성

apt (`package.xml` 에 선언 — `rosdep` 이 잡는다): ROS 메시지 패키지, `python3-yaml`,
`python3-opencv`, `ffmpeg`.

pip (apt 버전이 없거나 구버전이라 `package.xml` 에 넣지 않았다 — `rosdep` 이 **못 잡는다**):

```bash
pip install --user 'mcap' 'mcap-ros2-support' 'pydantic>=2' 'psycopg[binary]>=3'
```

USB 풋페달 클러치(`clutch_pedal`)를 쓰면 `python3-evdev` 가 더 필요하다. `rdfp/dataset/` 이나
`rdfp/rosbag/` 에 pip 전용 의존을 더하면 이 절에도 적는다.

## 5. 제어 계층 API — `robot_control`

이 패키지의 노드가 쓰는 MoveIt2 클라이언트와 카메라 노드는 `robot_control` 소속이고, 사용법도
그쪽 문서에 있다.

| 주제 | 문서 |
|---|---|
| `create_move_group_client(node, backend=...)` · `MoveGroupJtcClient` / `MoveGroupJgpcClient` | [move_group_client_user_guide.md](../../docs/control/move_group_client_user_guide.md) · [move_group_jgpc_client_user_guide.md](../../docs/control/move_group_jgpc_client_user_guide.md) |
| `camera_node` · `image_capture_node` · `image_viewer_node` | [camera_nodes_guide.md](../../docs/control/camera_nodes_guide.md) |
| 그리퍼 계약(`gripper_cmds` / `gripper_states`) | [gripper_node_design.md](../../docs/control/gripper_node_design.md) |
| 제어 계층 진입점 | [docs/control/README.md](../../docs/control/README.md) |

## 6. 세션 인지 카메라 노드 (`rdfp_camera_node` · `rdfp_image_viewer_node`)

`/session` 을 구독하는 카메라 어댑터다. `rdfp_camera_node` 는 `IN_EPISODE` 일 때만 이미지를
발행하고 끊기면 `reconnect_interval_sec` 주기로 재연결하며, `rdfp_image_viewer_node` 는 프레임에
세션 상태를 겹쳐 그린다. 하드웨어 추상화는 `robot_control.camera.opencv_camera` 를 그대로 쓴다.
세션을 알기 때문에 제어 계층이 아니라 이 패키지에 있다 — `ros2 run rdfp <노드>` 로 부른다.
상세는 [session_camera_nodes_guide.md](../../docs/collect/session_camera_nodes_guide.md).

## 7. 이미지 레코더

`sensor_msgs/Image` 를 MP4 로 녹화하는 ROS 어댑터가 둘 있고, 둘 다 녹화 엔진
`rdfp.recorder.FFMpegMp4Recorder` 를 쓴다(엔진 가이드:
[ffmpeg_mp4_recorder_guide.md](../../docs/collect/ffmpeg_mp4_recorder_guide.md)).
**수집에 쓰는 것은 `rdfp_image_recorder` 하나다.**

| 노드 | 녹화 시점 | 쓰는 곳 |
|---|---|---|
| `rdfp_image_recorder` | `/session` 이 `IN_EPISODE` 인 구간을 자동으로 | `rdfp_collect` 의 유일한 레코더 |
| `image_recorder_node` | 서비스 호출(start / stop)로 | 보조 — launch 가 띄우지 않으며 손으로 실행한다 |

사전 요구사항: `ffmpeg` 바이너리가 PATH 에 있고, `rdfp_msgs` 가 함께 빌드되어 있어야 한다.

### 7.1 세션 구동 레코더 (`rdfp_image_recorder`)

`rdfp_image_recorder` (= `RdfpImageRecorderNode`) 는 `/session` 토픽
(`rdfp_msgs/msg/SessionCommand`) 의 상태 전이를 직접 구독하여 `IN_EPISODE` 구간에만 녹화한다.
녹화 경계는 메시지 도착 순서가 아닌 **타임스탬프 기반** 으로 판정하며, `pending_image_queue` 로
도착 순서 차이를 보상한다.

녹화 1회 당 3개 파일이 생성된다 (`<prefix>=file_prefix`, `<start_ts>=YYYYMMDD-HHMMSS.SSS`):

* `<output_dir>/<prefix>_<start_ts>.mp4` — 영상
* `<output_dir>/<prefix>_<start_ts>.jsonl` — 프레임별 sidecar
  (`{"frame_index": N, "stamp": {"sec": ..., "nanosec": ...}}`)
* `<output_dir>/<prefix>_<start_ts>_metadata.json` — recording metadata
  (에피소드 키 · topic · codec · fps · 해상도 · 프레임 수 — **에피소드마다 한 벌**이다)

적재는 이 mp4 를 rosbag 이미지와 별도로 metadata 의 **에피소드 키**로 에피소드에 붙인다
(설정 `recordings_dir`). 압축 이미지(`CompressedImage`)를 직접 받는 것도 이 노드뿐이다
(`input_format`). 인코딩 설정(`bitrate` · `gop_size` · `preset` · `encoder_mode`)은
`image_pipeline.yaml` 의 `image_recorder` 블록이 갖는다.

⚠️ 선언한 `pixel_format` · 해상도 · fps 와 다른 프레임은 **경고만 남기고 버린다** — mp4 는
생기고 오류도 없어서, 적재에서 `image_frames: 0` 으로야 드러난다. Isaac 카메라는 `rgb8` 이고
`backend:=panda_isaac` 이 그 값을 넣는다.

상세는 [rdfp_image_recorder_node_guide.md](../../docs/collect/rdfp_image_recorder_node_guide.md).

### 7.2 서비스 구동 레코더 (`image_recorder_node`)

외부 서비스 호출로 start/stop 을 받는다. `sensor_msgs/Image` 만 구독한다(압축 이미지 불가).
상세는 [image_recorder_node_guide.md](../../docs/collect/image_recorder_node_guide.md).

```bash
ros2 run rdfp image_recorder_node \
  --ros-args \
  -r image:=/camera/color/image_raw \
  -p output_dir:=/data/rdfp/recordings \
  -p session_prefix:=demo \
  -p fps:=30 \
  -p resolution:=1280x720 \
  -p pixel_format:=rgb8 \
  -p encoder_mode:=auto
```

기본 토픽명은 `image` 이며, ROS2 remap 관례 (`-r image:=...`) 로 다른 토픽에 연결한다.
녹화는 두 서비스로 제어한다 (요청 필드는 모두 비어 있다).

```bash
# 세션 시작 — 응답 mp4_path 에 생성될 파일 경로가 담긴다.
ros2 service call /image_recorder/start_session rdfp_msgs/srv/StartSession

# 세션 종료 — 응답 mp4_path 에 finalize 된 파일 경로가 담긴다.
ros2 service call /image_recorder/stop_session  rdfp_msgs/srv/StopSession
```

파일명은 `<output_dir>/<session_prefix>_YYYYMMDD-HHMMSS.SSS.mp4` 형식이다 (`SSS` 는 밀리초
3자리). 한 노드에서 start/stop 을 반복하여 여러 세션을 순차적으로 녹화할 수 있다.

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `output_dir` | `/data/rdfp/recordings` | MP4 저장 디렉터리 (없으면 자동 생성) |
| `session_prefix` | `session` | 파일명 접두사 (`[A-Za-z0-9_-]` 권장) |
| `fps` | `30` | CFR 프레임 레이트 (실제 토픽 rate 와 일치해야 함) |
| `resolution` | (필수) | `"WIDTHxHEIGHT"` 문자열 |
| `pixel_format` | `bgr8` | `bgr8` / `rgb8` / `mono8` 중 하나 |
| `encoder_mode` | `auto` | `auto` / `cpu` / `gpu` |
| `queue_size` | `120` | 프레임 큐 최대 크기 |
| `auto_start` | `false` | 기동하자마자 녹화를 시작한다 |

`bitrate`, `gop_size`, `preset`, `preferred_hw_codec`, `ffmpeg_binary`, `vaapi_device` 등의
recorder 세부 설정은 이 노드에서 노출하지 않으며 `FFMpegMp4Recorder` 의 기본값을 쓴다.

동작 특성:

- **단일 세션**: 동시에 하나의 녹화만 허용한다. 이미 RECORDING 중에 호출된 `start_session` 은
  실패 응답을 반환한다.
- **encoding/해상도 검증**: 수신 프레임이 파라미터와 다르면 ERROR 로그 후 drop 한다 (로그는
  최초 1회 + 100건마다 1회로 억제).
- **자동 종료**: 불일치 프레임이 **연속 5장** 이상 누적되면 노드가 스스로 `recorder.stop()` 을
  호출해 세션을 종료한다. 설정을 수정한 뒤 `start_session` 으로 새 세션을 시작할 수 있다.
- **CFR 책임**: recorder 는 CFR + passthrough 라 토픽 publish rate 와 파라미터 `fps` 가 다르면
  재생 속도가 왜곡된다. `fps` 를 실제 rate 에 맞춰야 한다.
- **SIGINT 안전**: Ctrl+C 로 종료해도 진행 중인 세션의 MP4 가 finalize 된다
  (`destroy_node()` 에서 `recorder.stop()` → `recorder.shutdown()`).

## 8. 세션 제어 노드 (`session_control_node`)

세션(session)과 에피소드(episode) 생명주기를 제어하는 ROS2 노드다. 외부 클라이언트는 서비스로
제어 명령을 전달하고, 본 노드는 내부 상태 머신을 갱신한 뒤 변경된 상태(`state`)와
`task_label` 을 `/session` 토픽으로 발행하여 다른 노드가 수신·반응할 수 있게 한다.

상세 사용 설명과 Python 클라이언트 (`SessionControlClient`) 사용법은
[docs/collect/session_control_guide.md](../../docs/collect/session_control_guide.md) 한 문서에 있다.

### 8.1 실행 예

```bash
# 단독 실행 — 서비스는 /session_control/<name>, 토픽은 /session (루트) 에 노출됨
ros2 run rdfp session_control_node

# launch 로 실행 — rdfp_collect 가 이 노드를 띄운다 (재생 스택 replay_panda_mock 은 띄우지 않는다)
ros2 launch rdfp rdfp_collect.launch.py backend:=panda_isaac
```

### 8.2 상태 머신

토픽 발행 내용은 `(state, task_label)` 쌍이며, `<L>` 은 현재 `task_label`, `<NEW>` 는
`set_task_label(task_label=<NEW>)` 로 지정된 새 라벨이다.

| 현재 상태 | 서비스 | 발행 (순서) | 다음 상태 |
|---|---|---|---|
| IDLE | `start_session` | `(IN_SESSION, <L>)` | IN_SESSION |
| IDLE | `set_task_label(task_label=<NEW>)` | `(IDLE, <NEW>)` | IDLE |
| IN_SESSION | `stop_session` | `(IDLE, <L>)` | IDLE |
| IN_SESSION | `set_task_label(task_label=<NEW>)` | `(IN_SESSION, <NEW>)` | IN_SESSION |
| IN_SESSION | `start_episode` | `(IN_EPISODE, <L>)` | IN_EPISODE |
| IN_EPISODE | `stop_episode` | `(IN_SESSION, <L>)` | IN_SESSION |
| IN_EPISODE | `stop_session` | `(IN_SESSION, <L>)`, `(IDLE, <L>)` | IDLE |

- 초기 상태는 `IDLE`, `task_label` 초기값은 `''`.
- 허용되지 않은 상태의 서비스 호출(예: `IDLE` 에서 `stop_session`, `IN_EPISODE` 에서
  `set_task_label`)은 `success=false`, `message='invalid command'` 로 거부되며 상태와 토픽은
  변하지 않는다. **라벨은 에피소드를 시작하기 전에 정한다.**
- `IN_EPISODE` 에서 `stop_session` 을 받으면 `(IN_SESSION, <L>)` → `(IDLE, <L>)` 순서로 두
  메시지를 연속 발행하여 `IN_EPISODE → IN_SESSION → IDLE` 의 논리적 2 단계 전이를 구독자에게
  노출한다. 이 원자적 동작은 `stop_session` 핸들러 내부에서 처리되므로 클라이언트는 한 번만
  호출하면 된다. 다만 **depth 1 로 구독하면 앞의 `IN_SESSION` 이 덮여 `IDLE` 만 도착**하므로,
  구독자는 `IDLE` 도 에피소드 종료로 처리해야 한다.

### 8.3 서비스 호출

세션 제어 명령은 5 개의 분할된 서비스로 제공된다. `start_session` · `stop_session` ·
`start_episode` 는 `std_srvs/srv/Trigger`, `stop_episode` 는 `rdfp_msgs/srv/StopEpisode`
(종료 시점의 성패 `outcome` 과 부가정보 `metadata` 를 받는다), `set_task_label` 은
`rdfp_msgs/srv/SetString` 을 쓴다. 조회용 `get_session_state` 까지 합치면 6 개다.

```bash
# 세션 시작/종료
ros2 service call /session_control/start_session std_srvs/srv/Trigger "{}"
ros2 service call /session_control/stop_session std_srvs/srv/Trigger "{}"

# 에피소드 시작/종료 — stop_episode 는 성패와 부가정보를 실을 수 있다 (둘 다 생략 가능)
ros2 service call /session_control/start_episode std_srvs/srv/Trigger "{}"
ros2 service call /session_control/stop_episode rdfp_msgs/srv/StopEpisode \
  "{outcome: 'success', metadata: '{\"seed\": 7}'}"

# task label 설정 (빈 문자열이면 task clear)
ros2 service call /session_control/set_task_label rdfp_msgs/srv/SetString \
  "{task_label: 'pick_and_place'}"
ros2 service call /session_control/set_task_label rdfp_msgs/srv/SetString \
  "{task_label: ''}"

# 현재 상태 조회 — 응답은 state / task_label 필드로 분리됨
ros2 service call /session_control/get_session_state \
  rdfp_msgs/srv/GetSessionState "{}"
# 예: state='IN_SESSION', task_label='pick_and_place'
```

### 8.4 토픽 QoS

`/session` publisher 는 다음 QoS 로 설정된다. 늦게 붙은 구독자(예: 나중에 시작된 recorder)도
**직전 상태를 즉시 수신**하여 현재 세션 상태를 복원할 수 있도록 하기 위함이다. 이름은 절대
경로(`/session`)다 — 세션은 시스템에 하나이고 로봇별 네임스페이스를 타면 안 되기 때문이다.

| 항목 | 값 |
|---|---|
| `reliability` | `RELIABLE` |
| `durability` | `TRANSIENT_LOCAL` |
| `history depth` | `1` |

서비스 응답에는 `state`/`task_label` 이 포함되지 않으므로, 상태 변화를 따라가야 하는
클라이언트는 이 토픽을 구독해야 한다. 구독자는 호환되는 QoS 로 연결해야 하며,
`ros2 topic echo` 로 직접 확인할 때는 다음 플래그가 필요하다.

```bash
ros2 topic echo /session rdfp_msgs/msg/SessionCommand \
  --qos-durability transient_local \
  --qos-reliability reliable
```

동작 특성:

- **단일 스레드 executor**: `SingleThreadedExecutor` 를 쓰므로 서비스 콜백 간 race condition 이
  없으며 상태 보호용 lock 을 두지 않는다.
- **로깅**: 상태 전이와 `task_label` 설정은 `info` 레벨, 유효하지 않은 명령은 `warning` 레벨로
  출력된다.
- **SIGINT 안전**: Ctrl+C 수신 시 `destroy_node()` → `rclpy.try_shutdown()` 경로로 깨끗하게
  종료된다.

## 9. 데이터셋 CLI (`rosbag` / `import` / `export` / `stats` / `list` / `init-db` / `replay`)

rosbag2 MCAP 아카이브를 에피소드 단위로 분할해 PostgreSQL 에 적재하고, 적재된 에피소드를
학습 샘플로 export 하거나 라이브 토픽으로 재생한다. CLI 는 관심사별로 **여러 독립
console_script** 로 나뉘어 있다.

| 명령 | 모듈 | 역할 | ROS 의존 |
|---|---|---|---|
| `rosbag list-topics` · `list-episodes` | `rdfp.rosbag.cli` | rosbag2 조회. bag 루트는 `--rosbag-dir` 또는 `RDFP_ROSBAG_DIR` (`list-episodes --config` 는 DB 적재 여부까지 본다) | 없음 |
| `rosbag export` | `rdfp.rosbag.cli` | 고른 에피소드만 적재 (`import` 재사용) | 있음 |
| `rosbag delete` · `clear` | `rdfp.rosbag.cli` | bag 삭제 | 없음 |
| `init-db` | `rdfp.dataset.init_db_cmd` | DB 스키마 생성/재생성 | 없음 |
| `import` | `rdfp.dataset.import_cmd` | rosbag → DB 적재 + 레코더 mp4 등록 | 있음 (sensor_msgs 등) |
| `export` | `rdfp.dataset.export_cmd` | 적재된 에피소드 → 학습 샘플 트리(`.npz` + `metadata.json`) | 없음 |
| `stats` | `rdfp.dataset.stats_cmd` | 적재된 테이블별 행 수 출력 | 없음 |
| `list` | `rdfp.dataset.list_cmd` | `sessions` 에피소드 목록 출력 | 없음 |
| `replay` | `rdfp.dataset.replay_cmd` | 지정 에피소드를 라이브 토픽으로 재생 | 있음 (rclpy) |

"ROS 의존 없음"은 소싱하지 않은 셸에서 `python3 -m <모듈>`(`src/rdfp` 에서)로 돈다는 뜻이다 —
학습 환경에서 다시 export 할 때 쓴다. 이전에는 `dataset <sub>` 형태의 서브커맨드였지만 ROS 의존
분리와 단독 실행 편의를 위해 모두 top-level 명령으로 나뉘었다. 새 명령도 `dataset` 의
서브커맨드가 아니라 형제 `*_cmd.py` 모듈로 추가한다.

상세 문서:

- [rosbag_명령_가이드.md](../../docs/collect/rosbag_명령_가이드.md) — rosbag 녹화 · 조회 · 적재 · 삭제
- [데이터셋 적재 운영 가이드](../../docs/collect/데이터셋_적재_운영_가이드.md) — 적재 명령 레퍼런스
- [데이터셋 적재 시스템 설계서](../../docs/collect/데이터셋_적재_시스템_설계서.md) — 내부 구조 · 알고리즘
- [저장_형식.md](../../docs/learning/저장_형식.md) — export 파일 형식
- 설정 예시: [config/dataset_config.yaml](../../config/dataset_config.yaml)

### 9.1 DB 스키마 준비

```bash
# (A) CLI 로 (권장) — DSN 은 RDFP_DB_DSN
ros2 run rdfp init-db --dsn-env RDFP_DB_DSN
#   기존 테이블을 제거하고 재생성하려면 (주의):
ros2 run rdfp init-db --dsn-env RDFP_DB_DSN --drop --yes
#   설정 파일의 db 섹션(스키마 포함)을 그대로 쓰려면:
ros2 run rdfp init-db --config /data/rdfp/dataset.yaml

# (B) psql 로 직접 (대체 수단)
psql "${RDFP_DB_DSN}" -f src/rdfp/rdfp/dataset/sql/schema.sql
```

### 9.2 실행

```bash
rdfp_env      # install/setup.bash + RDFP_DB_DSN 등

# 적재 · export
ros2 run rdfp import --config /data/rdfp/dataset.yaml
ros2 run rdfp export --config /data/rdfp/dataset.yaml --out /data/rdfp/datasets/v1

# rosbag 조회 (DB 미사용)
ros2 run rdfp rosbag list-topics --rosbag-dir /data/rdfp/rosbag

# 적재된 DB 조회
ros2 run rdfp stats --config /data/rdfp/dataset.yaml
ros2 run rdfp list  --config /data/rdfp/dataset.yaml

# 적재된 에피소드를 라이브 토픽으로 재생 (별도 MoveIt 스택 필요).
#   --topic 기본값(/servo_node/delta_twist_cmds)은 재생 스택의 어느 경로와도 맞지 않는다 —
#   replay_arm_path 에 맞춰 준다: ee_twist(기본) → /ee_pose, target_joint_cmds → /target_joint_cmds
ros2 run rdfp replay 42 --config /data/rdfp/dataset.yaml --topic /ee_pose
```

공통 옵션: `--log-level {debug,info,warning,error}` (기본 `info`). 설정 파일을 다른
디렉터리로 복사하면 상대 경로인 `topics_file` 이 깨지므로 절대 경로로 바꾼다.

⚠️ `export` 의 `--episodes` 는 **DB 의 에피소드 ID** 다 — `rosbag list-episodes` 의 순번과 다르다.
같은 `--out` 으로 다시 export 하면 이전 실행의 `.npz` 가 남는다.

### 9.3 주요 동작

- **에피소드 감지**: `/session` 토픽(`rdfp_msgs/msg/SessionCommand`) 의 상태 전이
  (`IN_EPISODE → IN_SESSION`) 를 기준으로 자동 분할한다.
- **DB 적재**: 에피소드 단위 트랜잭션으로 `sessions` 와 메시지 타입별 테이블(`joint_states` ·
  `pose_stampeds` · `twist_stampeds` · `gripper_cmds` · `gripper_states` · `motion_trajectories` ·
  `motion_events` · `scene_objects` …)에 INSERT 한다. 정본은 `rdfp/dataset/sql/schema.sql`.
- **영상**: 현행 수집은 영상을 rosbag 에 담지 않는다. 레코더 mp4 를 `recordings_dir` 에서 찾아
  `image_streams`(경로 · 코덱 · 해상도 · 프레임 수) · `image_frames`(프레임별 stamp)에 등록하며,
  mp4 자체는 옮기지 않는다. (bag 에 `sensor_msgs/Image` 가 있으면 에피소드 × 카메라마다 mp4 로
  인코딩하는 경로도 남아 있지만, 둘을 함께 켜면 `image_streams` UNIQUE 제약에 걸린다.
  `CompressedImage` 는 writer 가 없어 경고 후 무시된다.)
- **멱등성**: 동일 에피소드(`start_sec, start_nanosec` 기준) 중복 시 `on_existing_episode`
  정책 (`skip` 기본 / `replace` / `error`) 이 적용된다.
- **병렬 처리**: 설정의 `parallelism > 1` 이면 `multiprocessing.Pool` 로 에피소드 단위 분산 처리.
- **품질 게이트는 없다** (2026-10-18 제거): 정렬된 스트림을 봐서 stamp 역행이 잡힐 수 없었고
  병렬 모드에서는 돌지도 않았다. 옛 설정의 `quality_gate` 블록은 무시된다 — 근거는
  `docs/collect/데이터셋_적재_시스템_설계서.md` §12.
- **세션 finalization 필수**: `discover_splits` 는 `metadata.yaml` 이 없는 세션을 "활성/비정상
  종료" 로 간주해 건너뛴다. `rosbag2` 프로세스가 SIGKILL/충돌 등으로 graceful shutdown 없이
  끝난 경우, `import` 가 빈 summary 만 반환한다. 이때는 `ros2 bag reindex -s mcap <session_dir>`
  로 `.mcap` 에서 메타데이터를 재구성하면 된다.

### 9.4 환경변수

| 변수 | 용도 |
|---|---|
| `RDFP_DB_DSN` | PostgreSQL DSN (필수). 설정 파일에 평문으로 두지 않는다. 변수명 자체는 설정의 `db.dsn_env` 로 바꿀 수 있다 |
| `RDFP_ROSBAG_DIR` · `RDFP_VIDEOS_DIR` · `RDFP_RECORDINGS_DIR` | `config/dataset_config.yaml` 의 경로 필드가 `${VAR:-기본값}` 으로 읽는다. `rosbag` CLI 는 `--rosbag-dir` 가 없을 때 `RDFP_ROSBAG_DIR` 를 쓴다 |
| `RDFP_HOME` | `--config` 를 생략했을 때 `$RDFP_HOME/config/dataset_config.yaml` 을 찾는다 |

> JSONL 로그 경로는 `<output_mp4_dir>/_logs/postproc_run.jsonl` 로 고정이며, DB 배치 INSERT
> 버퍼 크기는 코드 상 `WriterBase.batch_size=1000` 기본값을 그대로 쓴다 (환경변수로는 노출되지
> 않는다).

## 10. 데이터셋 재생 GUI (`replay_gui`)

`rdfp.dataset.replay_gui_cmd` 는 적재된 에피소드를 라이브 토픽으로 재생하는 Tk GUI 다. 일반
토픽은 단일 워커의 `TopicMessageReplayer` (stamp 기준 k-way merge), 이미지 토픽은 토픽별
`Mp4ImageReplayer` (decoder 스레드 + publisher 스레드, mp4 lazy 디코딩) 로 나눠 띄우며, 공통
`start_time` / `first_history_time` anchor 로 cadence 를 보존한다.

```bash
# MoveIt 스택과 함께 띄우는 launch 진입점
ros2 launch rdfp replay_panda_mock.launch.py
```

이 launch 는 arm 구동 어댑터를 `replay_arm_path` 로 택일한다 — `ee_twist`(기본, pose 차분 →
servo, 개루프) / `target_joint_cmds`(관절 위치 폐루프, 재현 충실도 최고) / `none`(외부
클라이언트로 구동). 구조와 주의점은
[docs/collect/replay_mock_stack_guide.md](../../docs/collect/replay_mock_stack_guide.md).

GUI 동작 특성:

- "Topics to replay" 다중 선택은 **기본 모두 미선택** 상태로 시작한다 (의도하지 않은 토픽 재생
  방지). `Select all` / `Clear all` 버튼으로 일괄 토글한다.
- "위치 초기화" 버튼은 `move_to_named_target_async("ready")` 를 호출한다. JTC/JGPC 선택은
  **노드 생성 시 `backend` 파라미터 하나로** 정해지며 호출마다 갈리지 않는다. `backend` 는
  기본값이 없어 **안 주면 노드가 뜨지 않는다** — 짐작하게 두면 틀린 짐작이 조용히 통과해
  "'위치 초기화' 가 안 먹는다" 로만 보이기 때문이다.
- replayer 들은 **one-shot lifecycle** — `start()` 두 번 호출 시
  `RuntimeError('… already started')`. 재생을 다시 하려면 새 인스턴스를 만든다.
- 종료 시 use-after-destroy 방지 가드: 워커 스레드가 2초 안에 종료되지 못하면 publisher destroy
  와 `cv2.VideoCapture.release()` 를 건너뛰고 warning 으로 leak 을 가시화한다.

## 11. 데이터셋 관리 GUI (`dataset_manager`)

별도 패키지 [rdfp_gui_codex](../rdfp_gui_codex/README.md)에서 제공한다. 저장된 MCAP 토픽과 PostgreSQL 테이블을 한 화면에서 비교하고, 선택 토픽 Import와 에피소드 단위 Export를 수행하는 데스크톱 도구다. 메시지 정의·테이블 스키마·실제 데이터 조회, JSONB/CDR 범용 테이블 등록, 매핑 해제, 영상 프레임 미리보기를 지원한다. 학습 데이터는 NPZ 또는 LeRobot v3로 내보낸다.

```bash
# rdfp_ws 루트에서 실행한다. 설치·DB 설정은 아래 가이드를 참고한다.
./scripts/run_dataset_manager.sh
```

실행 환경과 사용 방법은 [Dataset Manager 사용자 가이드](../../docs/collect/Dataset_Manager_사용자_가이드.md)에 정리했다. 기존 적재 CLI와는 중복 처리 정책이 다르며, Manager의 교체는 선택한 에피소드·토픽에만 적용한다. 같은 schema에 대한 CLI 적재와 GUI의 쓰기·Export 작업은 동시에 실행하지 않는다.

## 12. 라이선스

Apache-2.0 (`package.xml`).

## 13. 관리자

- kwlee (kwlee@etri.re.kr)
