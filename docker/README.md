# rdfp Docker (panda mock / replay mock / replay GUI)

`run_panda_mock.sh` / `run_replay_mock.sh` / `replay_gui` 를 컨테이너로 실행하기
위한 구성이다. **apt ROS 2 Humble 만 사용**하므로, 호스트에서 겪던 소스 빌드
moveit(ws_moveit2)과 apt base 라이브러리 간 ABI 충돌이 발생하지 않는다.

## 이미지 구성

| 이미지 | 역할 | 실행 명령 |
|---|---|---|
| `rdfp-base` | ROS+moveit+rdfp 빌드 (공유 베이스) | — |
| `rdfp-panda-mock` | 풀 앱 mock 스택 (+camera/recorder) | `ros2 launch rdfp rdfp_panda_mock.launch.py` |
| `rdfp-panda-jgpc-mock` | JGPC(forward command) mock 스택 (+camera) | `ros2 launch robot_control panda_jgpc_mock.launch.py` |
| `rdfp-gazebo` | Gazebo(Fortress) 백엔드 풀 앱 스택 | `ros2 launch rdfp rdfp_panda_gazebo.launch.py` |
| `rdfp-replay-mock` | replay 스택 | `ros2 launch rdfp replay_panda_mock.launch.py` |
| `rdfp-replay-gui` | Tk 재생 GUI (+DB/MP4 재생) | `ros2 run rdfp replay_gui` |

`rdfp-gazebo` 만 base 위에 `ros_gz` + `ign_ros2_control`(+ `ign` CLI)을 추가한다
(다른 이미지는 gazebo 의존성 없이 가볍게 유지).

`replay-mock` 과 `replay-gui` 는 **별도 컨테이너**로 뜨며, `--network host` +
동일 `ROS_DOMAIN_ID` 로 서로 ROS 통신한다(GUI 가 mock 스택의 MoveIt 을 구동).

## 이미지 아키텍처 (공유 베이스)

무거운 작업(ROS+moveit apt 설치, `colcon build`)은 `rdfp-base` 에서 **한 번만**
수행하고, 각 앱 이미지는 base 를 상속해 실행 명령(`CMD`)만 바꾼다.

```
rdfp-base                  ← ROS/moveit/colcon build/entrypoint (무거운 것 전부)
├── rdfp-panda-mock        → CMD: ros2 launch rdfp rdfp_panda_mock.launch.py
├── rdfp-panda-jgpc-mock   → CMD: ros2 launch robot_control panda_jgpc_mock.launch.py
├── rdfp-gazebo            → +ros_gz/ign_ros2_control, CMD: ros2 launch rdfp rdfp_panda_gazebo.launch.py
├── rdfp-replay-mock       → CMD: ros2 launch rdfp replay_panda_mock.launch.py
└── rdfp-replay-gui        → CMD: ros2 run rdfp replay_gui   (+Tk/pip 만 추가)
```

`panda-mock` / `replay-mock` 의 Dockerfile 은 사실상 `FROM rdfp-base` + `CMD`
두 줄뿐이다. 이렇게 나눈 이유:

- **빌드 비용 1회** — apt 설치·colcon 빌드를 이미지마다 반복하지 않는다.
- **디스크 공유** — 앱 이미지들은 base 레이어(~5GB)를 공유하고 델타만 추가한다.
  `docker images` 의 5.15GB 는 공유 레이어 포함 총량이지 이미지마다 3배를 쓰는 게
  아니다(`replay-gui` 만 pip 패키지로 +0.1GB).
- **일관성** — 세 이미지가 동일한 moveit/rdfp 빌드를 쓴다(상호 통신 시 중요).
- **빠른 재빌드** — rdfp 코드 변경 시 base 만 다시 빌드하면 앱 이미지는 거의 즉시.
- **확장 용이** — 새 변형은 `FROM rdfp-base` + `CMD` 두 줄이면 된다.

단, 앱 이미지는 `rdfp-base:latest` 태그를 참조하므로 base 가 먼저 빌드돼 있어야
한다(`build.sh` 가 base→panda-mock→panda-jgpc-mock→gazebo→replay-mock→replay-gui
순서를 강제).

## 빌드

```bash
cd ~/development/ros/rdfp_ws
./docker/build.sh    # base → panda-mock → panda-jgpc-mock → gazebo → replay-mock → replay-gui
```

- 빌드 컨텍스트는 워크스페이스 루트이며, `.dockerignore` 로 `src/` 만 포함된다
  (259MB 샘플 mp4 등은 제외).
- 첫 빌드는 base 이미지(`osrf/ros:humble-desktop`, ~3GB)를 받으므로 수 분
  걸린다. 이후 rdfp 코드 변경 시 재빌드는 base 레이어 재사용으로 수십 초.

## 실행 (X11 forwarding)

각 스크립트가 `xhost +local:docker`, `DISPLAY`, `/tmp/.X11-unix` 마운트를
자동 처리한다.

### panda mock 풀 앱 스택

```bash
./docker/run_panda_mock.sh
# 웹캠 없이:            ./docker/run_panda_mock.sh enable_camera_node:=false
# 녹화 출력 경로 지정:  RDFP_REC_DIR=/data/rec ./docker/run_panda_mock.sh
```

camera 노드(웹캠)와 image_recorder 를 포함한다. 스크립트가 호스트의 `/dev/video*`
를 자동 패스스루하고, 녹화 출력 디렉터리(기본 `/tmp/recordings`)를 볼륨 마운트한다.
웹캠이 없으면 `enable_camera_node:=false` 로 카메라를 끈다.

### panda JGPC mock 스택

```bash
./docker/run_panda_jgpc_mock.sh
# 웹캠 없이: ./docker/run_panda_jgpc_mock.sh enable_camera_node:=false
```

`panda_arm_controller` 의 **타입만** `position_controllers/JointGroupPositionController`
로 바뀐 변형이다(이름은 동일). 결과적으로:

- `/panda_arm_controller/joint_trajectory` 가 없어지고
  `/panda_arm_controller/commands` (`std_msgs/Float64MultiArray`, **관절 이름 없이
  배열 순서가 컨트롤러 `joints` 파라미터 순서와 일치해야 함**) 를 받는다.
- `moveit_simple_controller_manager` 가 `FollowJointTrajectory`/`GripperCommand` 만
  다루므로 **move_group 의 arm plan & execute 가 동작하지 않는다.** 계획 자체는 정상이다.
  다만 `create_move_group_client(node)` 가 이 스택을 감지해 `MoveGroupJgpcClient` 를
  돌려주므로, 호출부는 평소대로 `move_to_named_target()` / `follow_trajectory()` 를
  쓰면 된다 (내부에서 계획 후 command 스트리밍으로 실행). 단 **open loop** 라
  정상 반환이 목표 도달을 뜻하지 않는다.
- servo 의 `command_out_type` 이 `std_msgs/Float64MultiArray` 로 바뀐다.

`session_control` / `image_recorder` 는 포함되지 않으므로 녹화 볼륨(`RDFP_REC_DIR`)
설정이 없다. camera 노드는 포함되어 `/dev/video*` 를 자동 패스스루한다.

### Gazebo 스택

```bash
./docker/run_gazebo.sh
# launch 인자 추가:  ./docker/run_gazebo.sh enable_rviz:=true simulate_camera:=true
# 백엔드만(앱 노드 없이): ./docker/run_gazebo.sh ros2 launch robot_control panda_gazebo.launch.py
```

Gazebo(gz-sim) 창이 뜬다. GL 렌더는 Intel/AMD 면 `/dev/dri` 자동 패스스루로
하드웨어 가속, NVIDIA 면 `GPU=1`, 둘 다 없으면 소프트웨어 렌더(느림).

### replay 스택

```bash
./docker/run_replay_mock.sh
```

RViz2 와 image_viewer 창이 호스트 화면에 뜬다.

### replay GUI (다른 터미널)

```bash
export RDFP_DB_DSN='postgresql://user:pw@localhost:5432/rdfp'
export RDFP_DATA_DIR=/data/rdfp     # DB 에 저장된 MP4 경로와 동일하게 마운트
./docker/run_replay_gui.sh
```

## ROS 통신 (도메인 / 네트워킹) — 중요

`replay-gui`(호스트든 컨테이너든)가 `replay-mock` 컨테이너의 로봇을 구동하려면
아래가 **모두** 일치해야 한다.

- `--network host` (스크립트 기본).
- **`ROS_DOMAIN_ID` 일치** — 두 run 스크립트 기본값은 `31`. 호스트에서 GUI 를
  직접 띄우는 경우 호스트 셸의 `ROS_DOMAIN_ID` 도 `31` 이어야 한다.
- **동일 RMW** — run 스크립트는 컨테이너에 `rmw_fastrtps_cpp` 를 주입하고,
  호스트 셸의 `.ros2rc` 기본값도 같다. 별도 조치 없이 맞는다.

### 도메인 ID 바꾸기

세 run 스크립트는 모두 `ROS_DOMAIN_ID` 환경변수를 받아 컨테이너에 주입한다
(`${ROS_DOMAIN_ID:-31}` — 없으면 기본 31). 코드 수정 없이 값을 바꿀 수 있다.

```bash
# 실행 시 한 번만 지정
ROS_DOMAIN_ID=42 ./docker/run_replay_mock.sh
ROS_DOMAIN_ID=42 ./docker/run_replay_gui.sh

# 셸 전체에 적용(이후 실행되는 모든 컨테이너에 반영)
export ROS_DOMAIN_ID=42
./docker/run_panda_mock.sh
```

통신하는 **모든** 주체(두 컨테이너 + 호스트에서 띄우는 GUI/노드)가 같은 값이어야
한다. 하나라도 다르면 서로 발견되지 않아 로봇이 움직이지 않는다. 호스트 셸에
`ROS_DOMAIN_ID` 가 이미 설정돼 있으면 스크립트가 그 값을 그대로 상속한다. 기본값
`31` 자체를 바꾸려면 각 스크립트의 `:-31` 폴백을 수정한다.

> **흔한 함정**: ROS 2 는 `ROS_DOMAIN_ID` 만 인식한다(`ROS2_DOMAIN_ID` 는 무시).
> 호스트에서 실수로 `ROS2_DOMAIN_ID=31` 로 두면 호스트는 도메인 **0**, 컨테이너는
> **31** 이 되어 서로 발견되지 않고 — GUI 재생을 눌러도 **로봇이 움직이지 않는다**.
> 확인: `echo $ROS_DOMAIN_ID` (비어 있으면 0). 필요 시 GUI 쪽을
> `ROS_DOMAIN_ID=31 ./scripts/run_replay_gui.sh` 로 맞추거나 컨테이너를
> `ROS_DOMAIN_ID=0 ./docker/run_replay_mock.sh` 로 맞춘다.

통신 확인:

```bash
ros2 topic list | grep -E "target_joint_states|joint_trajectory|joint_states"
ros2 topic echo /target_joint_states                     # GUI 가 발행 중인지
ros2 topic hz  /panda_arm_controller/joint_trajectory    # executor 변환 발행 중인지
```

## GPU 가속 (선택)

기본은 **소프트웨어 렌더(llvmpipe)** 로 동작한다. NVIDIA GPU 가속을 쓰려면
`nvidia-container-toolkit`(CDI) 설정 후 `GPU=1` 을 명시한다.

```bash
GPU=1 ./docker/run_replay_mock.sh
```

> `GPU=1` 없이는 `--gpus` 를 붙이지 않는다. GPU 미구성 호스트에서 `--gpus all`
> 은 `failed to discover GPU vendor from CDI` 로 실패하므로 자동 감지는 하지
> 않는다(opt-in).

## 전제 / 주의

- **X11**: 스크립트가 `xhost +local:docker` 를 수행한다. 원격/Wayland 환경에서는
  추가 설정이 필요할 수 있다.
- **DB/데이터**: `replay_gui` 는 PostgreSQL 과 MP4 파일이 필요하다. `--network
  host` 라 `localhost` DB 에 접속되며, MP4 는 DB 에 기록된 경로와 **같은 경로**로
  마운트되어야 한다(`RDFP_DATA_DIR`).
- **재생 소스**: `replay_panda_mock.launch.py` 는 토픽이 재생되어 들어온다고
  가정하는 실행/시각화 측이다. 실제 재생은 `replay_gui`(또는 `ros2 bag play`,
  `ros2 run rdfp replay`)가 담당한다.
- **moveit_py**: rdfp `package.xml` 에 선언돼 있으나 실제로는 import 하지 않으며
  apt 에도 없다. Dockerfile 은 `rosdep` 대신 필요한 패키지를 명시 설치하므로
  문제되지 않는다.

## 트러블슈팅

| 증상 | 원인 / 해결 |
|---|---|
| `failed to discover GPU vendor from CDI` | GPU 미구성인데 `--gpus` 사용. `GPU=1` 을 빼거나 nvidia-container-toolkit 설치. |
| `cannot open display` / GUI 안 뜸 | `xhost +local:docker` 미적용 또는 `DISPLAY` 미전달. 스크립트로 실행했는지 확인. Wayland 는 추가 설정 필요. |
| GUI 재생해도 **로봇이 안 움직임** | 호스트↔컨테이너 `ROS_DOMAIN_ID` 불일치(위 "ROS 통신" 참조). |
| `replay_gui` DB 접속 실패 | `RDFP_DB_DSN` 미설정/오류. `--network host` 라 DSN 의 host 는 보통 `localhost`. |
| replay 시 이미지/포즈는 되는데 관절만 안 움직임 | `target_joint_states_executor` 로의 `/target_joint_states` 미도달 — 도메인/네트워크 확인. |

## 파일

| 파일 | 역할 |
|---|---|
| `Dockerfile.base` | 공유 베이스(ROS+moveit+rdfp 빌드) |
| `Dockerfile.panda_mock` / `.panda_jgpc_mock` / `.replay_mock` / `.replay_gui` | 각 스택 이미지(base 상속, CMD 만 교체) |
| `Dockerfile.gazebo` | base + `ros_gz`/`ign_ros2_control`, gazebo 스택 |
| `entrypoint.sh` | ROS + rdfp 오버레이 소싱(ws_moveit2 미사용 = apt-only) |
| `build.sh` | 이미지 6개 순차 빌드 |
| `run_panda_mock.sh` / `run_panda_jgpc_mock.sh` / `run_gazebo.sh` / `run_replay_mock.sh` / `run_replay_gui.sh` | X11 + `--network host` + 도메인/RMW 설정 실행 |

> run 스크립트는 인자 없이 실행하면 기본 launch/명령을, launch 인자만 주면
> (`enable_rviz:=true` 등) 기본 명령 뒤에 붙여서, `ros2 …` 로 시작하는 전체
> 명령을 주면 그대로 실행한다.
