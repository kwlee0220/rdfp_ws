# rdfp

Franka Emika Panda 로봇 암의 ROS 2 (Humble) 통합 패키지입니다. MoveIt2 기반
카테시안 경로 계획/실행 외에도 카메라 캡처, MP4 녹화, 세션·에피소드 생명주기
제어, rosbag2 → PostgreSQL/MP4 데이터셋 적재, 적재된 에피소드 재생 (Tk
GUI 포함) 까지 한 패키지로 묶여 있습니다.

서비스/메시지 인터페이스는 별도 패키지 `rdfp_msgs` 에서 제공되므로 함께
빌드해야 합니다.

## 1. 패키지 구조

> **`rdfp` 는 학습 데이터 수집 계층이다.** MoveIt2 클라이언트 · 카메라 · scene 상태는
> 하위 패키지 [`robot_control`](../robot_control/README.md) 로 분리되어 있고, 로봇 트윈은
> [`robot_twin`](../robot_twin/README.md) 이다. 분리 근거는 설계서
> [§7.6](../../docs/rdfp_framework_design.md) 참고.

```
rdfp/
├── package.xml                # ROS 2 패키지 메타데이터
├── setup.py                   # console_scripts + robot_twin.backends entry point
├── setup.cfg                  # 설치 경로 설정
├── config/                    # panda_robot / replay_panda_mock / teleop_mirror YAML
├── launch/                    # rdfp_*.launch.py, replay_panda_mock, teleop_mirror
├── resource/rdfp              # ament 리소스 마커
└── rdfp/                      # Python 소스 root
    ├── session/               # session_control_node — IDLE / IN_SESSION / IN_EPISODE 상태 머신
    │                          # twin_backend.py — robot_twin 에 세션 연산을 제공하는 entry point
    ├── recorder/              # FFMpegMp4Recorder + image_recorder_node (ROS adapter)
    ├── teleop/                # teleop_keyboard, session_teleop, teleop_retarget,
    │                          # clutch_pedal(USB 풋페달), ClutchClient
    ├── rosbag/                # MCAP catalog/discovery + `rosbag` CLI
    ├── dataset/               # DB 스키마 + ingestion 파이프라인 + `import`/`stats`/`list`/`init-db`/`replay` CLI
    │                          # (replay_gui_cmd — Tk GUI 도 여기 포함)
    └── samples/               # 수동 샘플/데모 스크립트 (entry_point 아님)
```

## 2. Launch 구조

launch 진입점은 **계층별로 두 패키지에 나뉘어 있고**, 공유 helper 는 하위 패키지의
설치 모듈이다.

| 위치 | 내용 |
|---|---|
| `src/rdfp/launch/` | `rdfp_panda_mock`, `rdfp_panda_jgpc_mock`, `rdfp_panda_gazebo`, `rdfp`, `rdfp_advanced`, `replay_panda_mock`, `teleop_mirror` |
| `src/robot_control/launch/` | `panda_mock`, `panda_jgpc_mock`, `panda_gazebo` |
| `robot_control/launch_helpers/` | 공유 argument 선언 · `Node` 팩토리 · startup orchestration |

```python
from robot_control.launch_helpers.camera import create_camera_node
```

Panda mock 전체 스택의 메인 진입점은 `ros2 launch rdfp rdfp_panda_mock.launch.py`
이고, 수집 노드 없이 제어 스택만 필요하면
`ros2 launch robot_control panda_mock.launch.py` 입니다.
launch 파일 간 역할 분리와 의존 관계의 상세 설명은 `launch/README.md` 를 참고하세요.

## 3. 활용 절차

### 3.1 사전 준비
#### 3.1.1 워크스페이스 빌드 및 환경 설정
```bash
cd ~/development/ros/rdfp_ws
colcon build --packages-select rdfp_msgs robot_control robot_twin rdfp
source install/setup.bash
```

#### 3.1.2 rosbag 저장 디렉토리 확인
* rosbag2 의 기본 저장 디렉토리는 `~/rosbags` 입니다. 다른 위치에 저장하려면
  `ROS2_BAG_DIR` 환경변수를 설정하세요.
* rosbag2 저장 디렉토리가 비어 있는지 확인하고, 충분한 여유 공간이 있는지 확인하세요 (최소 10 GB 권장).
* 관련 설정 파일
  - `record_rosbag.sh` 의 `ROS2_BAG_DIR` 기본값
  - `dataset_config.yaml` 의 `rosbag_dir` 필드
* 수집 대상 토픽 목록은 config/recording_topics.list 에 정의되어 있습니다.
필요에 따라 수정하세요.

#### 3.1.3 rosbag 저장소 초기화 (필요에 따라)
설정된 rosbag 저장소에 저장된 모든 데이터를 삭제.
```bash
ros2 run rdfp rosbag clear
```

#### 3.1.4 PostgreSQL 데이터베이스 준비
데이터셋 후처리기에서 사용할 PostgreSQL 데이터베이스를 준비합니다. DSN은
환경변수 `RDFP_DB_DSN` 을 통해 설정함. (예: `postgresql://user:pass@localhost:5432/rdfp`).
데이터베이스가 준비되면 다음 명령으로 스키마를 초기화합니다.
```bash
ros2 run rdfp init-db   # 필요한 테이블들이 없는 경우에 생성.
ros2 run rdfp init-db --drop --yes  # 주의: 기존 데이터베이스 스키마와 모든 데이터가 삭제됩니다
```

### 3.2 학습 데이터 수집

#### 3.2.1 수집 환경 확인 (옵션, 별도 터미널)
```bash

# 주요 ROS2 노드 모니터링 (터미널 #1)
cd ~/development/ros/rdfp_ws/src/rdfp/scripts
watch rdfp_list_nodes.sh

# 주요 토픽 모니터링 (터미널 #2)
cd ~/development/ros/rdfp_ws/src/rdfp/scripts
watch rdfp_list_topics.sh
```

#### 3.2.2 Panda mock 환경 실행 (별도 터미널)
```bash
ros2 launch rdfp rdfp_panda_mock.launch.py
```
실행 후 RViz, ros2_control, move_group, servo, camera, ee_pose_publisher, gripper,
mock_scene_state 노드가 모두 실행되어야 합니다 (scene 노드는 `enable_scene:=false` 로 끌 수 있습니다).

#### 3.2.3 키보드기반 teleop 으로 학습 데이터 생성 (별도 터미널)
```bash
ros2 run rdfp teleop_keyboard
```

#### 3.2.4 rosbag2 녹화 시작 (별도 터미널)
```bash
ros2 run rdfp record_rosbag
```
녹화가 시작되면 `record_rosbag` 노드가 `ROS2_BAG_DIR` (기본 `~/rosbags`) 아래에
`YYYYMMDD-HHMMSS` 형식의 새 디렉토리를 만들고, 그 안에 rosbag2 MCAP 파일이
생성됩니다.

#### 3.2.5 키보드 조작을 통한 학습 데이터 생성

주요 명령 키 목록 (`?` 키로 노드 내에서도 동일한 도움말을 출력할 수 있음):

| 카테고리 | 키 | 동작 |
|---|---|---|
| 이동 (Cartesian linear) | `j` / `l` | +x / -x |
|                         | `i` / `k` | +y / -y |
|                         | `q` / `a` | +z / -z |
| 이동 (Cartesian angular) | `r` / `f` | roll +x / -x |
|                          | `t` / `g` | pitch +y / -y |
|                          | `y` / `h` | yaw +z / -z |
| 이동 (Joint)            | `'` / `;` | `panda_joint1` + / - |
| 제어                    | `SPACE`   | deadman (눌러야 motion 명령이 발행됨, 기본 TTL 0.1s) |
|                         | `x`       | twist / joint_jog 을 0 으로 즉시 발행 (stop) |
|                         | `/`       | MoveIt SRDF 의 `ready` named target 으로 이동 |
|                         | `?`       | 명령 키 도움말 출력 |
|                         | `Ctrl-C`  | 종료 |
| 그리퍼                  | `=`       | open — `/gripper_cmds` 에 `goal='open'` 발행 |
|                         | `-`       | close — 같은 토픽에 `goal='close'` 발행 |
| 세션                    | `<` 또는 `,` | `start_session` |
|                         | `>` 또는 `.` | `stop_session` |
| 에피소드                | `[`       | `start_episode` |
|                         | `]`       | `stop_episode` |
| Task                    | `1`–`9`   | `tasks` 파라미터의 라벨 선택 (현재 매핑은 노드 시작 로그 참조) |
|                         | `0`       | task 라벨 해제 |

> 참고
> - **Deadman 동작**: motion 키(linear/angular/joint)는 `SPACE` 또는 motion 키를
>   누르는 동안만 발행된다. 마지막 입력으로부터 `deadman_ttl_sec`(기본 0.1초)이
>   지나면 자동으로 멈춘다.
> - **One-shot 명령**: 그리퍼/세션/에피소드/task 키는 deadman 과 무관하게
>   누른 즉시 한 번만 서비스가 호출된다.
> - **Home 이동 중**: `/` 키 처리 동안에는 servo 명령 발행이 일시 중단되어
>   `move_group` 의 trajectory 와 충돌하지 않는다.

#### 3.2.6 rosbag에 수집된 토픽 목록 확인
```bash
ros2 run rdfp rosbag list-topics
```

### 3.3 수집 데이터를 데이터베이스에 적재
```bash
ros2 run rdfp import
```

### 3.4 수집된 데이터 확인 및 재생

#### 3.4.1 동작 재생을 위한 panda mock 환경 실행 (별도 터미널)
```bash
ros2 launch rdfp replay_panda_mock.launch.py
```

arm 을 구동하는 경로는 `replay_arm_path` 로 고릅니다 (기본 `target_joint_states`).
자세한 내용은 [docs/replay/replay_mock_stack_guide.md](../../docs/replay/replay_mock_stack_guide.md).

#### 3.4.2 적재된 에피소드 재생 도구 실행 (Tk GUI 포함)
```bash
ros2 run rdfp replay_gui
```

## 4. 주요 기능

### 4.1 rdfp.moveit 모듈

MoveIt2 서비스/액션 인터페이스를 사용하여 카테시안 경로를 계획하고 실행하는 통합 모듈입니다.
외부에서 생성한 `rclpy.node.Node` 를 주입받아 서비스/액션 클라이언트를 그 위에 올립니다.

**계획은 컨트롤러와 무관하지만 실행은 그렇지 않으므로**, 클라이언트가 세 개로 나뉘어 있습니다.

| 클래스 | 역할 |
|---|---|
| [`MoveGroupClient`](../robot_control/robot_control/moveit/move_group_client.py) | 계획 + SRDF 조회. **추상 클래스라 직접 생성 불가** |
| [`MoveGroupJtcClient`](../robot_control/robot_control/moveit/move_group_jtc_client.py) | `JointTrajectoryController` 환경 — MoveGroup/ExecuteTrajectory 액션으로 실행 |
| [`MoveGroupJgpcClient`](../robot_control/robot_control/moveit/move_group_jgpc_client.py) | forward command 컨트롤러 환경 — `Float64MultiArray` 명령 스트리밍으로 실행. **open loop** |

핵심 진입점은 [`create_move_group_client(node)`](../robot_control/robot_control/moveit/move_group_factory.py) 팩토리입니다.
`/panda_arm_controller/commands` 토픽 존재 여부로 컨트롤러를 판별해 알맞은 구현을 돌려주므로,
호출부는 어느 스택인지 몰라도 됩니다. 확실히 알고 있다면 `mode='jtc'` / `mode='jgpc'` 로 강제합니다.

#### 4.1.1 주요 메서드

| 메서드 | 설명 |
|------|------|
| `create_move_group_client(node, mode='auto')` | 컨트롤러(JTC/JGPC)를 판별해 알맞은 구현 생성 — **권장 진입점** |
| `MoveGroupClient` | 공통 인터페이스(추상). 직접 생성 불가 |
| `MoveGroupJtcClient` / `MoveGroupJgpcClient` | 각각 액션 실행 / command 스트리밍 실행 구현 |
| `wait_until_ready(timeout_sec=30.0)` | 서비스와 액션 서버가 준비될 때까지 블로킹 대기 |
| `is_ready()` | 서비스/액션 준비 여부를 즉시 반환 (non-blocking) |
| `get_named_targets(group=None)` | SRDF `group_state` 에 정의된 named target 목록 조회 (**한 그룹만**. `None` 은 생성자의 기본 그룹) |
| `get_all_named_targets()` | 모든 planning 그룹의 named target 을 `{group: [name, ...]}` 로 조회 |
| `get_planning_groups()` | SRDF 에 정의된 planning group 이름 전체 조회 (named target 이 없는 그룹 포함) |
| `move_to_named_target(name, ...)` | SRDF 의 named target (예: `"ready"`) 으로 이동 |
| `move_to_named_target_async(name, ...)` | 위 호출의 비동기 버전 (실행 Future 반환) |
| `follow_trajectory(waypoints, ...)` | 카테시안 경로 계획 및 실행 (원스톱) |
| `follow_trajectory_async(waypoints, ...)` | 위 호출의 비동기 버전 (계획 + 실행 Future 반환) |
| `plan_trajectory(waypoints, ...)` | 카테시안 경로 계획만 수행 |
| `plan_trajectory_async(waypoints, ...)` | 경로 계획 요청을 보내고 Future 반환 |
| `execute_trajectory(trajectory, ...)` | 사전 계획된 trajectory 실행 — **JTC 전용** |
| `execute_trajectory_async(trajectory, ...)` | 실행 goal 을 보내고 결과 Future 반환 — **JTC 전용** |
| `stream_trajectory(trajectory, ..., publish_rate=None)` | 궤적을 명령 토픽으로 발행 — **JGPC 전용**. `publish_rate` 로 보간 발행 |
| `follow_trajectory_streamed(waypoints, ...)` | 카테시안 계획 + 스트리밍 — **JGPC 전용** |
| `stop_streaming()` | 진행 중인 스트리밍 중단 — **JGPC 전용** |
| `scale_trajectory_velocity(trajectory, factor)` | trajectory 의 속도 스케일링 |
| `close()` / `destroy()` | 생성한 클라이언트 리소스 정리 (컨텍스트 매니저도 지원) |
| `pose(x, y, z, roll, pitch, yaw)` | RPY 를 쿼터니언으로 변환해 Pose 생성 (유틸) |

#### 4.1.2 주요 특징

- **컨트롤러 추상화**: `move_to_named_target()` / `follow_trajectory()` 는 두 환경에서
  모두 동작하되 실행 메커니즘이 다릅니다. **JGPC 구현은 open loop 라 정상 반환이
  목표 도달을 뜻하지 않습니다** — 도달 확인이 필요하면 JTC 구현을 명시적으로 요구하세요
- **기본값 주입**: `fraction_threshold`, `velocity_scaling`, `max_step`, `jump_threshold`
  는 생성자에 기본값으로 설정하고 호출 시 `None` 이 아닌 값으로 override 가능
- **Lazy 초기화**: 생성자는 클라이언트 객체만 만들고 서버 대기는 하지 않음
- **컨텍스트 매니저**: `with create_move_group_client(node) as client:` 로 리소스 자동 정리
- **비동기 API**: `*_async()` 메서드로 Future 기반 호출 가능
- **안전성**: waypoint 유효성 검사, 쿼터니언 정규화 확인
- **타임아웃**: 모든 작업에 타임아웃 설정 가능
- **예외 기반 오류 처리**: `ValueError`, `TimeoutError`, `RuntimeError` 로 실패 신호

## 5. 의존성

| 패키지 | 용도 |
|--------|------|
| `rclpy` | ROS2 Python 클라이언트 라이브러리 |
| `geometry_msgs` | Pose 등 기하 메시지 타입 |
| `moveit_msgs` | GetCartesianPath 서비스, ExecuteTrajectory 액션 |
| `tf_transformations` | 오일러 각도 ↔ 쿼터니언 변환 (`sudo apt install ros-humble-tf-transformations`) |
| `builtin_interfaces` | Duration 등 기본 메시지 타입 |

## 6. MoveIt2 인터페이스

이 패키지는 MoveItPy가 아닌 MoveIt2의 서비스/액션 인터페이스를 직접 사용합니다.

| 항목 | 값 |
|------|------|
| 경로 계획 서비스 | `/compute_cartesian_path` (`GetCartesianPath`) |
| 경로 실행 액션 | `/execute_trajectory` (`ExecuteTrajectory`) |
| 플래닝 그룹 | `panda_arm` |
| 기준 프레임 | `panda_link0` |
| 보간 간격 | 0.01 m (1 cm) |
| Jump Threshold | 0.0 (비활성화) |
| 성공 기준 | fraction > 0.9 (경로의 90% 이상 계산 성공 시 실행) |

## 7. 활용 방법

### 7.1 빌드

워크스페이스 루트에서 패키지를 빌드합니다.

```bash
cd ~/development/ros/rdfp_ws
colcon build --packages-select rdfp
source install/setup.bash
```

### 7.2 데모 실행

먼저 별도의 터미널에서 Panda MoveIt 환경(RViz, ros2_control, move_group, servo, camera, ee_pose_publisher, gripper, mock_scene_state)을 실행합니다.

```bash
ros2 launch robot_control panda_mock.launch.py
# 또는 YAML 기반 풀 스택 (camera / recorder / ee_pose 까지)
ros2 launch rdfp rdfp_panda_mock.launch.py
```

환경이 완전히 로드된 후 새 터미널에서 카테시안 데모 스크립트를 실행할 수 있습니다.
(별도 console_script 로 등록되지는 않았으므로 모듈로 직접 실행합니다.)

```bash
python3 -m rdfp.moveit.test_move_cartesian
```

### 7.3 커스텀 경유점 정의

`rdfp.moveit` 모듈을 사용하여 자신만의 경유점을 정의할 수 있습니다.

```python
import rclpy
from rclpy.node import Node
from rdfp.moveit import create_move_group_client, pose

rclpy.init()
node = Node('custom_cartesian_planner')

try:
    # 컨텍스트 매니저로 클라이언트를 생성하면 리소스가 자동 정리됩니다.
    with create_move_group_client(node) as client:
        client.wait_until_ready()

        # 커스텀 경유점 정의 (x, y, z, roll, pitch, yaw)
        # roll=3.14 (π) → 엔드이펙터가 아래를 향하는 자세
        waypoints = [
            pose(0.4,  0.2, 0.5, 3.14, 0.0, 0.0),
            pose(0.4, -0.2, 0.5, 3.14, 0.0, 0.0),
            pose(0.6, -0.2, 0.5, 3.14, 0.0, 0.0),
            pose(0.6,  0.2, 0.5, 3.14, 0.0, 0.0),
        ]

        # 경로 실행 (생성자 기본값: 원래 속도)
        client.follow_trajectory(waypoints)

finally:
    node.destroy_node()
    rclpy.shutdown()
```

### 7.4 속도 스케일링 조정

`velocity_scaling` 은 생성자 기본값으로 설정하거나 각 호출에서 override 할 수 있습니다.

```python
# 방법 A: 클라이언트 전체 기본값을 낮춤 (안전 모드)
with create_move_group_client(node, velocity_scaling=0.2) as client:
    client.wait_until_ready()
    client.follow_trajectory(waypoints)  # 20% 속도

# 방법 B: 호출 단위로 override
client.follow_trajectory(waypoints, velocity_scaling=0.5)  # 이 호출만 50%
```

## 8. 카메라 노드 (`camera_node`)

`camera_node` 는 토픽 이름을 파라미터로 받지 않고, 다음 **private 기본 토픽**을
사용합니다 (`~/` prefix 는 노드 이름으로 자동 치환되어 기본 노드 이름
`camera_node` 기준 `/camera_node/...` 로 resolve 됩니다).

- 이미지(raw): `~/image_raw` → `/camera_node/image_raw`
- 이미지(압축): `~/image_compressed` → `/camera_node/image_compressed`
- CameraInfo: `~/camera_info` → `/camera_node/camera_info`
- 상태: `~/camera_status` → `/camera_node/camera_status`

실제 토픽 연결은 ROS2 remap 으로 지정합니다.

### 8.1 실행 예 (`ros2 run`)

```bash
ros2 run robot_control camera_node \
  --ros-args \
  -p camera_id:=0 \
  -p fps:=30 \
  -p resolution:=640x480 \
  -r ~/image_raw:=/camera/image_raw \
  -r ~/camera_info:=/camera/camera_info \
  -r ~/camera_status:=/camera/image_raw/status
```

### 8.2 launch 인자 매핑 (`launch/camera_launch_helper.py`)

- `camera_image_topic` -> remap `~/image_raw`
- `camera_info_topic` -> remap `~/camera_info`
- `camera_status_topic` -> remap `~/camera_status`

즉, launch 에서는 토픽을 파라미터로 넘기지 않고 remap 값으로만 제어합니다.

## 9. 이미지 녹화 노드 (`image_recorder_node`)

`sensor_msgs/Image` 토픽을 수신해 MP4 파일로 녹화하는 ROS2 노드입니다.
녹화 엔진은 `rdfp.recorder.FFMpegMp4Recorder` 를 재사용하며, 본 노드는
ROS2 인터페이스 ↔ recorder 간 얇은 어댑터 역할을 수행합니다.

상세 사용 설명은 [docs/recorder/image_recorder_node_guide.md](../../docs/recorder/image_recorder_node_guide.md)
를, 녹화 엔진 자체의 가이드는 [docs/recorder/ffmpeg_mp4_recorder_guide.md](../../docs/recorder/ffmpeg_mp4_recorder_guide.md)
를 참고하세요.

### 9.1 사전 요구사항

- `ffmpeg` 바이너리가 시스템 PATH 에 있어야 합니다.
- 서비스 인터페이스가 정의된 별도 패키지 `rdfp_msgs` 가 함께 빌드되어야 합니다.

```bash
colcon build --packages-select rdfp_msgs robot_control robot_twin rdfp
source install/setup.bash
```

### 9.2 실행 예

```bash
ros2 run rdfp image_recorder_node \
  --ros-args \
  -r image:=/camera/color/image_raw \
  -p output_dir:=/tmp/recordings \
  -p session_prefix:=demo \
  -p fps:=30 \
  -p resolution:=1280x720 \
  -p pixel_format:=rgb8 \
  -p encoder_mode:=auto
```

기본 토픽명은 `image` 이며, ROS2 remap 관례 (`-r image:=...`) 로 다른 토픽에
연결할 수 있습니다.

### 9.3 서비스 호출

녹화는 두 개의 서비스로 제어합니다 (요청 필드는 모두 비어있음).

```bash
# 세션 시작 — 응답 mp4_path 에 생성될 파일 경로가 담깁니다.
ros2 service call /image_recorder/start_session rdfp_msgs/srv/StartSession

# 세션 종료 — 응답 mp4_path 에 finalize 된 파일 경로가 담깁니다.
ros2 service call /image_recorder/stop_session  rdfp_msgs/srv/StopSession
```

파일명은 `<output_dir>/<session_prefix>_YYYYMMDD-HHMMSS.SSS.mp4` 형식으로
생성됩니다 (`SSS` 는 밀리초 3자리). 한 노드에서 start/stop 을 반복하여 여러
세션을 순차적으로 녹화할 수 있습니다.

### 9.4 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `output_dir` | `/tmp/recordings` | MP4 저장 디렉터리 (없으면 자동 생성) |
| `session_prefix` | `session` | 파일명 접두사 (`[A-Za-z0-9_-]` 권장) |
| `fps` | `30` | CFR 프레임 레이트 (실제 토픽 rate 와 일치해야 함) |
| `resolution` | (필수) | `"WIDTHxHEIGHT"` 문자열 |
| `pixel_format` | `bgr8` | `bgr8` / `rgb8` / `mono8` 중 하나 |
| `encoder_mode` | `auto` | `auto` / `cpu` / `gpu` |
| `queue_size` | `120` | 프레임 큐 최대 크기 |

`bitrate`, `gop_size`, `preset`, `preferred_hw_codec`, `ffmpeg_binary`,
`vaapi_device` 등의 recorder 세부 설정은 노드에서 노출하지 않으며
`FFMpegMp4Recorder` 의 기본값을 그대로 사용합니다.

### 동작 특성

- **단일 세션**: 동시에 하나의 녹화만 허용. 이미 RECORDING 중에 호출된
  `start_session` 은 실패 응답을 반환합니다.
- **encoding/해상도 검증**: 수신 프레임이 파라미터와 다르면 ERROR 로그 후
  drop 합니다 (로그는 최초 1회 + 100건마다 1회로 억제).
- **자동 종료**: 불일치 프레임이 **연속 5장** 이상 누적되면 노드가 스스로
  `recorder.stop()` 을 호출해 세션을 종료합니다. 사용자는 설정을 수정한 뒤
  `start_session` 으로 새 세션을 시작할 수 있습니다.
- **CFR 책임**: recorder 는 CFR + passthrough 라 토픽 publish rate 와
  파라미터 `fps` 가 다르면 재생 속도가 왜곡됩니다. 사용자가 `fps` 를 실제
  rate 에 맞춰야 합니다.
- **SIGINT 안전**: Ctrl+C 로 종료해도 진행 중인 세션의 MP4 가 finalize
  됩니다 (`destroy_node()` 에서 `recorder.stop()` → `recorder.shutdown()`).

### 9.5 세션 기반 자동 녹화 노드 (`rdfp_image_recorder`)

`image_recorder_node` 가 외부 서비스 호출로 start/stop 을 받는 반면,
`rdfp_image_recorder` (= `RdfpImageRecorderNode`) 는 `/session` 토픽
(`rdfp_msgs/msg/SessionCommand`) 의 상태 전이를 직접 구독하여
`IN_EPISODE` 구간에만 자동으로 녹화합니다. 녹화 경계는 메시지 도착 순서가
아닌 **타임스탬프 기반** 으로 판정하며, `pending_image_queue` 로 도착
순서 차이를 보상합니다.

녹화 1회 당 3개 파일이 생성됩니다 (`<prefix>=session_prefix`,
`<start_ts>=YYYYMMDD-HHMMSS.SSS`):

* `<output_dir>/<prefix>_<start_ts>.mp4` — 영상
* `<output_dir>/<prefix>_<start_ts>.jsonl` — 프레임별 sidecar
  (`{"frame_index": N, "stamp": {"sec": ..., "nanosec": ...}}`)
* `<output_dir>/<prefix>_metadata.json` — recording metadata

상세 사용 설명은 [docs/recorder/rdfp_image_recorder_node_guide.md](../../docs/recorder/rdfp_image_recorder_node_guide.md)
를 참고하세요. launch 통합 예는 `rdfp_advanced.launch.py` 에 있습니다.

## 10. 세션 제어 노드 (`session_control_node`)

세션(session)과 에피소드(episode) 생명주기를 제어하는 ROS2 노드입니다. 외부
클라이언트는 서비스로 제어 명령을 전달하고, 본 노드는 내부 상태 머신을 갱신한
뒤 변경된 상태(`state`)와 `task_label` 을 `session` 토픽으로 발행하여 다른
노드가 수신·반응할 수 있게 합니다.

상세 사용 설명은 [docs/session/session_control_guide.md](../../docs/session/session_control_guide.md)
를, Python 클라이언트 (`SessionControlClient`) 사용 가이드는
[docs/session/session_control_client_guide.md](../../docs/session/session_control_client_guide.md)
를 참고하세요.

### 10.1 사전 요구사항

서비스/메시지 인터페이스가 정의된 `rdfp_msgs` 패키지가 함께 빌드되어야 합니다.

```bash
colcon build --packages-select rdfp_msgs robot_control robot_twin rdfp
source install/setup.bash
```

### 10.2 실행 예

```bash
# 단독 실행 — 서비스/토픽은 /session_control 네임스페이스 하위에 노출됨
ros2 run rdfp session_control_node

# launch 파일로 실행
ros2 launch rdfp rdfp_advanced.launch.py
```

### 10.3 상태 머신

토픽 발행 내용은 `(state, task_label)` 쌍이며, `<L>` 은 현재 `task_label`,
`<NEW>` 는 `set_task_label(task_label=<NEW>)` 로 지정된 새 라벨입니다.

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
- 허용되지 않은 상태의 서비스 호출(예: `IDLE` 에서 `stop_session`, `IN_EPISODE`
  에서 `set_task_label`)은 `success=false`, `message='invalid command'` 로
  거부되며 상태와 토픽은 변하지 않습니다.
- `IN_EPISODE` 에서 `stop_session` 을 받으면 `(IN_SESSION, <L>)` →
  `(IDLE, <L>)` 순서로 두 메시지를 연속 발행하여 `IN_EPISODE → IN_SESSION →
  IDLE` 의 논리적 2 단계 전이를 구독자에게 노출합니다. 이 원자적 동작은
  `stop_session` 핸들러 내부에서 처리되므로 클라이언트는 한 번만 호출하면 됩니다.

### 10.4 서비스 호출

세션 제어 명령은 5 개의 분할된 서비스로 제공됩니다. 4 개는 `std_srvs/srv/Trigger`,
`set_task_label` 만 `rdfp_msgs/srv/SetString` 를 사용합니다.

```bash
# 세션 시작/종료
ros2 service call /session_control/start_session std_srvs/srv/Trigger "{}"
ros2 service call /session_control/stop_session std_srvs/srv/Trigger "{}"

# 에피소드 시작/종료
ros2 service call /session_control/start_episode std_srvs/srv/Trigger "{}"
ros2 service call /session_control/stop_episode std_srvs/srv/Trigger "{}"

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

### 10.5 토픽 QoS

`session` publisher 는 다음 QoS 로 설정됩니다. 늦게 붙은 구독자(예: 나중에
시작된 recorder)도 **직전 상태를 즉시 수신**하여 현재 세션 상태를 복원할 수
있도록 하기 위함입니다.

| 항목 | 값 |
|---|---|
| `reliability` | `RELIABLE` |
| `durability` | `TRANSIENT_LOCAL` |
| `history depth` | `1` |

서비스 응답에는 `state`/`task_label` 이 포함되지 않으므로, 상태 변화를 따라가야
하는 클라이언트는 반드시 이 토픽을 구독해야 합니다. 구독자는 호환되는 QoS 로
연결해야 하며, `ros2 topic echo` 로 직접 확인할 때는 다음 플래그가 필요합니다.

```bash
ros2 topic echo /session_control/session rdfp_msgs/msg/SessionCommand \
  --qos-durability transient_local \
  --qos-reliability reliable
```

### 동작 특성

- **단일 스레드 executor**: `SingleThreadedExecutor` 를 사용하므로 서비스
  콜백 간 race condition 이 없으며 상태 보호용 lock 을 두지 않습니다.
- **로깅**: 상태 전이와 `task_label` 설정은 `info` 레벨, 유효하지 않은 명령은
  `warning` 레벨로 출력됩니다.
- **SIGINT 안전**: Ctrl+C 수신 시 `destroy_node()` → `rclpy.try_shutdown()`
  경로로 깨끗하게 종료됩니다.

## 11. 데이터셋 후처리기 (`rosbag` / `import` / `stats` / `list` / `init-db` / `replay`)

rosbag2 MCAP 아카이브를 에피소드 단위로 분할하여 PostgreSQL 에 적재하고,
카메라 토픽은 에피소드별 MP4 와 함께 글로벌 메타 (`image_streams`) /
프레임별 stamp (`image_frames`) 를 DB 테이블에 적재하는 배치 프로그램입니다.
CLI 는 관심사별로 **여러 독립 console_script** 로 분리되어 있습니다.

| 명령 | 모듈 | 역할 | ROS 의존 |
|---|---|---|---|
| `rosbag list-topics` / `rosbag clear` | `rdfp.rosbag.cli` | rosbag2 split 조회 (DB 미사용). **`--config` 가 아니라 `--rosbag-dir`** | 없음 |
| `init-db` | `rdfp.dataset.init_db_cmd` | DB 스키마 생성/재생성 | 없음 |
| `import` | `rdfp.dataset.import_cmd` | rosbag → DB/MP4 적재 | 있음 (sensor_msgs 등) |
| `stats` | `rdfp.dataset.stats_cmd` | 적재된 테이블별 행 수 출력 | 없음 |
| `list` | `rdfp.dataset.list_cmd` | `sessions` 에피소드 목록 출력 | 없음 |
| `replay` | `rdfp.dataset.replay_cmd` | 지정 에피소드를 라이브 토픽으로 재생 | 있음 (rclpy) |

이전에는 `dataset <sub>` 형태의 서브커맨드였지만 ROS 의존 분리와 단독 실행
편의성을 위해 모두 top-level 명령으로 분리되었습니다. 새 명령을 추가할 때도
`dataset` 의 서브커맨드가 아니라 형제 `*_cmd.py` 모듈로 추가합니다.

상세 문서는 다음을 참고하세요:

- [데이터셋 후처리기 CLI 사용 설명서](../../docs/rosbag2/데이터셋%20후처리기%20CLI%20사용%20설명서.md) — 사용자용 명령 레퍼런스
- [데이터셋 후처리기 설계서](../../docs/rosbag2/데이터셋%20후처리기%20설계서.md) — 내부 구조·알고리즘
- [데이터셋 후처리기 실환경 검증 절차](../../docs/rosbag2/데이터셋%20후처리기%20실환경%20검증%20절차.md) — 실 인프라 검증 runbook
- 샘플 설정: [rosbag_config.sample.yaml](../../docs/rosbag2/rosbag_config.sample.yaml) · [dataset_config.sample.yaml](../../docs/rosbag2/dataset_config.sample.yaml)

### 11.1 Post-processor 의존성

apt (`package.xml` 에 선언):

```bash
sudo apt install python3-yaml python3-opencv ffmpeg
```

pip (apt 버전이 없거나 구버전이어서 pip 로 설치 필요):

```bash
pip install --user 'mcap' 'mcap-ros2-support' 'pydantic>=2' 'psycopg[binary]>=3'
```

### 11.2 DB 스키마 준비

```bash
export RDFP_DB_DSN="postgresql://rdfp@localhost:5432/rdfp"

# (A) CLI 로 (권장)
ros2 run rdfp init-db --dsn-env RDFP_DB_DSN
#   기존 테이블을 제거하고 재생성하려면 (주의):
ros2 run rdfp init-db --dsn-env RDFP_DB_DSN --drop --yes
#   dataset_config.yaml 의 db 섹션(스키마 포함) 을 그대로 사용하려면:
ros2 run rdfp init-db --config dataset_config.yaml

# (B) psql 로 직접 (대체 수단)
psql "${RDFP_DB_DSN}" -f src/rdfp/rdfp/dataset/sql/schema.sql
```

### 11.3 실행

```bash
source install/setup.bash
export RDFP_DB_DSN="postgresql://rdfp@localhost:5432/rdfp"

# 설정 파일 복사 후 편집
cp docs/rosbag2/rosbag_config.sample.yaml  /etc/rdfp/rosbag_config.yaml
cp docs/rosbag2/dataset_config.sample.yaml /etc/rdfp/dataset_config.yaml
vi /etc/rdfp/rosbag_config.yaml  /etc/rdfp/dataset_config.yaml

# 적재 실행
ros2 run rdfp import --config /etc/rdfp/dataset_config.yaml

# rosbag2 split 단위 조회 (DB 미사용)
ros2 run rdfp rosbag list-topics --rosbag-dir /data/rdfp/rosbag

# 적재된 DB 조회
ros2 run rdfp stats --config /etc/rdfp/dataset_config.yaml
ros2 run rdfp list  --config /etc/rdfp/dataset_config.yaml

# 적재된 에피소드를 라이브 토픽으로 재생 (별도 MoveIt 스택 필요)
ros2 run rdfp replay 42 --config /etc/rdfp/dataset_config.yaml
```

공통 옵션: `--log-level {debug,info,warning,error}` (기본 `info`).

### 11.4 주요 동작

- **에피소드 감지**: `/session` 토픽(`rdfp_msgs/msg/SessionCommand`) 의 상태 전이
  (`IN_EPISODE → IN_SESSION`) 를 기준으로 자동 분할합니다.
- **DB 적재**: 에피소드 단위 트랜잭션으로 `sessions` + 토픽별 테이블 (`pose_stampeds`,
  `twist_stampeds`, `joint_states`, `target_joint_states`, `gripper_cmds`,
  `gripper_states`) 에 INSERT 합니다.
- **MP4 생성**: 카메라 토픽(`sensor_msgs/Image` 의 8-bit raw 인코딩) 은 에피소드
  × 카메라마다 별도 mp4 로 인코딩되고 (CFR-passthrough), 글로벌 메타
  (mp4_path / 코덱 / 해상도 / fps / frame_id / frame_count) 는 `image_streams`
  테이블에, 프레임별 원본 stamp 는 `image_frames` 테이블에 적재됩니다.
  `CompressedImage` 와 16-bit (16UC1 / mono16 / 32FC1) 는 fail-fast 처리됩니다.
- **멱등성**: 동일 에피소드(`start_sec, start_nanosec` 기준) 중복 시
  `on_existing_episode` 정책 (`skip` / `replace` / `error`) 이 적용됩니다.
- **병렬 처리**: 설정의 `parallelism > 1` 이면 `multiprocessing.Pool` 로 에피소드
  단위 분산 처리.
- **품질 게이트**: `quality_gate.stamp_regression` / `idle_gap_sec` 로 stamp 역행·
  유휴 갭을 감지해 JSONL 로그 (`_logs/postproc_run.jsonl`) 에 `quality_warning`
  레코드로 남깁니다.
- **세션 finalization 필수**: `discover_splits` 는 `metadata.yaml` 이 없는
  세션을 "활성/비정상 종료" 로 간주해 건너뜁니다. `rosbag2` 프로세스가
  SIGKILL/충돌 등으로 graceful shutdown 없이 끝난 경우, `import` 가 빈
  summary 만 반환합니다. 이때는
  `ros2 bag reindex -s mcap <session_dir>` 로 `.mcap` 에서 메타데이터를
  재구성하면 됩니다.

### 11.5 환경변수

| 변수 | 용도 |
|---|---|
| `RDFP_DB_DSN` | PostgreSQL DSN (필수). 설정 파일에 평문으로 두지 않음. 변수명 자체는 `dataset_config.yaml` 의 `db.dsn_env` 로 변경 가능. |

> JSONL 로그 경로는 `<output_mp4_dir>/_logs/postproc_run.jsonl` 로 고정이며,
> DB 배치 INSERT 버퍼 크기는 코드 상 `WriterBase.batch_size=1000` 기본값을
> 그대로 사용한다 (현재 환경변수로는 노출되지 않음).

## 12. 데이터셋 재생 GUI (`replay_gui`)

`rdfp.dataset.replay_gui_cmd` 는 적재된 에피소드를 라이브 토픽으로
재생하는 Tk GUI 입니다. 일반 토픽은 단일 워커의 `TopicMessageReplayer`
(stamp 기준 k-way merge), 이미지 토픽은 토픽별 `Mp4ImageReplayer`
(decoder 스레드 + publisher 스레드, mp4 lazy 디코딩) 로 분리해 띄우며,
공통 `start_time` / `first_history_time` anchor 로 cadence 를 보존합니다.

```bash
# MoveIt 스택과 함께 띄우는 launch 진입점
ros2 launch rdfp replay_panda_mock.launch.py
```

이 launch 는 arm 구동 어댑터를 `replay_arm_path` 로 택일합니다 —
`target_joint_states`(기본, 위치 폐루프) / `ee_twist`(pose 차분 → servo, 개루프) /
`none`(외부 클라이언트로 구동). 구조와 주의점은
[docs/replay/replay_mock_stack_guide.md](../../docs/replay/replay_mock_stack_guide.md)
를 참고합니다.

GUI 동작 특성:

- "Topics to replay" 다중 선택은 **기본 모두 미선택** 상태로 시작 (의도하지
  않은 토픽 재생 방지). `Select all` / `Clear all` 버튼으로 일괄 토글.
- "위치 초기화" 버튼은 `move_to_named_target_async("ready")`
  호출.
- replayer 들은 **one-shot lifecycle** — `start()` 두 번 호출 시
  `RuntimeError('… already started')`. 재생을 다시 하려면 새 인스턴스를
  생성합니다.
- 종료 시 use-after-destroy 방지 가드: 워커 스레드가 2초 안에 종료되지
  못하면 publisher destroy 와 `cv2.VideoCapture.release()` 를 건너뛰고
  warning 으로 leak 을 가시화합니다.

## 13. 기술 참고 사항

- **서비스/액션 기반**: MoveItPy 대신 `GetCartesianPath` 서비스와 `ExecuteTrajectory` 액션을 직접 사용하므로, move_group 노드가 실행 중이어야 합니다.
- **블로킹 실행**: `rclpy.spin_until_future_complete()`를 사용하여 각 경로의 계획과 실행이 완료될 때까지 대기합니다.
- **속도 스케일링 원리**: trajectory의 각 포인트에 대해 `time_from_start`를 `1/scaling_factor`만큼 늘리고, `velocities`에 `scaling_factor`를, `accelerations`에 `scaling_factor²`를 곱하여 물리적으로 일관된 감속을 구현합니다.
- **Jump Threshold**: 0.0으로 설정하여 관절 공간에서의 급격한 점프 검사를 비활성화합니다. 필요 시 양수 값으로 설정하여 안전성을 높일 수 있습니다.

## 14. 라이선스

TODO

## 15. 관리자

- kwlee (kwlee@etri.re.kr)
