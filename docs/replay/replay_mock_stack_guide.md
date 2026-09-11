# replay mock 스택 가이드 — `replay_panda_mock.launch.py`

데이터셋에 적재된 에피소드를 mock 하드웨어 위에서 다시 재생하기 위한 런치
스택이다. 이 문서는 **구조 / 사용법 / 주의할 점** 세 부분으로 나뉜다.

관련 문서:

- [replay_approaches.md](replay_approaches.md) —
  왜 어떤 명령 타입을 골라야 하는지에 대한 원리 (본 문서의 이론적 배경)
- [../rosbag2/데이터셋 후처리기 CLI 사용 설명서.md](../rosbag2/데이터셋%20후처리기%20CLI%20사용%20설명서.md) —
  에피소드 적재(`import`) 및 조회(`list` / `stats`)
- [../../src/robot_control/launch/README.md](../../src/robot_control/launch/README.md) — 런치 헬퍼 인벤토리 (제어 계열)
- [../../src/rdfp/launch/README.md](../../src/rdfp/launch/README.md) — `replay_panda_mock` 인자 표

---

## 1. 구조

### 1-1. 설계 원칙 — "데이터셋이 주는 것은 만들지 않는다"

`rdfp_panda_mock.launch.py` 와 같은 스택을 띄우되, **재생 시 데이터셋에서 이미
제공되는 정보를 생성하는 노드는 제외한다.** 실시간 발행자와 재생 데이터가 같은
토픽에서 충돌하는 것을 막기 위해서다.

| 노드 | rdfp_panda_mock | replay_panda_mock | 제외 사유 |
|---|:---:|:---:|---|
| `camera_node` | O | **X** | 이미지가 MP4 에서 재생됨 |
| `ee_pose_publisher` | O | **X** | `/ee_pose` 가 DB 에서 재생됨 |
| `gripper_action_node` | O | **X** | 그리퍼 명령이 재생 쪽에서 발행됨 |
| `session_control_node` | O | **X** | 재생 중에는 세션 상태머신이 불필요 |
| `image_recorder_node` | O | **X** | 재생을 다시 녹화하지 않음 |
| `target_joint_cmds_publisher` | O | **X** | 재생 쪽이 발행자 |
| `mock_scene_state_node` | O | **X** | 물체 상태도 데이터셋에서 나와야 한다. 라이브 scene 노드를 띄우면 `/scene/objects` 에 **두 번째 출처**가 생긴다 |
| `joint_state_broadcaster` | O | **O** | 실시간 관측값이므로 **유지** |

`joint_state_broadcaster` 를 남기는 것이 핵심이다. `/joint_states` 는 재생 대상이
아니라 **재생 결과로 로봇이 실제 어디에 있는지** 를 알려주는 관측값이다. 이 값이
`robot_state_publisher` 를 거쳐 `/tf` 가 되고 RViz 가 그걸 그린다.

### 1-2. 기동되는 노드

```
ros2_control    /controller_manager  /joint_state_broadcaster
                /panda_arm_controller  /panda_hand_controller
                /static_transform_publisher  /robot_state_publisher
moveit          /move_group  /servo_node
rviz            /rviz2
robot_control   /rdfp_image_viewer_node       (enable_image_viewer)
                /gripper_control
                + replay_arm_path 에 따른 arm 구동 노드 (1-4 참고)
```

**이 스택은 `rdfp` 패키지의 노드를 하나도 띄우지 않는다.** launch 파일만 `rdfp` 에
있고(`ros2 launch rdfp replay_panda_mock.launch.py`), 기동되는 노드는 전부
`robot_control` 소유다 — 수집 계층이 재생에 필요 없기 때문이다(1-1).

기동은 **순차적**이다. `panda_hand_controller` spawner 가 정상 종료되어야 상위
노드가 일괄 spawn 된다 (`RegisterEventHandler(OnProcessExit)` 체인).

### 1-3. 워크스페이스 구현 노드

다섯 모두 **`robot_control` 패키지**다 — `ros2 run` 으로 개별 기동할 때 패키지
이름을 틀리지 않도록 주의한다.

| 노드 | 입력 | 출력 | 역할 |
|---|---|---|---|
| `rdfp_image_viewer_node` | `/camera/image_raw` | (화면) | 재생 프레임에 상태 오버레이 표시 |
| `gripper_action_node` | `/gripper_cmds` | `gripper_cmd` action | 재생된 그리퍼 명령을 action goal 로 중계 (수집 때와 같은 노드) |
| `target_joint_cmds_executor` | `/target_joint_cmds` | `/panda_arm_controller/joint_trajectory` | 길이 1 `JointTrajectory` 로 래핑 |
| `ee_twist_publisher` | `/ee_pose` | `/servo_node/delta_twist_cmds` | pose 유한 차분 → twist |
| `servo_auto_start` | — | `start_servo` 서비스 | servo 를 한 번 기동시키고 종료 |

`gripper_action_node` 는 녹화 당시와 **같은 노드**이며, 다른 점은 명령 토픽의 퍼블리셔가
그대로 구독한다 (`~/gripper_cmds` → `/gripper_cmds` remap). 재생
시에는 발행자만 재생 도구로 바뀌고 토픽 이름은 동일하다.

### 1-4. arm 재생 경로 — `replay_arm_path`

arm 을 구동하는 노드 조합을 인자 하나로 고른다. 두 경로가 동시에
`/panda_arm_controller/joint_trajectory` 를 쓰면 명령이 충돌하므로 **배타적으로만
기동한다.**

```
[ee_twist]  (기본값)
  DB → /ee_pose → ee_twist_publisher → /servo_node/delta_twist_cmds
                → servo_node → /panda_arm_controller/joint_trajectory → arm
       (servo_auto_start 가 start_servo 를 호출해 줘야 동작)

[target_joint_cmds]
  DB → /target_joint_cmds → target_joint_cmds_executor
                          → /panda_arm_controller/joint_trajectory → arm

[none]
  arm 구동 노드 없음. MoveGroupClient.follow_trajectory 등 외부 클라이언트로 구동.
```

| | `ee_twist` (기본) | `target_joint_cmds` | `none` |
|---|:---:|:---:|:---:|
| 제어 루프 | 속도 개루프 | **위치 폐루프** | (외부 결정) |
| 드리프트 | **누적됨** | 없음 | — |
| 여유자유도(팔꿈치) 재현 | X (EE 6-DOF 만) | **O** | X |
| 시작 pose 의존성 | **있음** | 없음 | — |
| 추가 서비스 호출 | `start_servo` 필요 | 불필요 | — |

기본값은 `ee_twist` 다. **재현 충실도가 목적이면 `target_joint_cmds` 를 명시한다** —
관절 절대 위치를 그대로 명령하므로 드리프트가 없고 팔꿈치 형상까지 원본과 일치한다.

---

## 2. 사용 가이드

### 2-1. 사전 준비

에피소드가 DB 에 적재되어 있어야 한다.

```bash
ros2 run rdfp import --config dataset_config.yaml   # rosbag → DB + MP4
ros2 run rdfp list   --config dataset_config.yaml   # episode id 확인
```

DSN 은 `dataset_config.yaml` 의 `db.dsn_env` 가 가리키는 환경변수(기본
`RDFP_DB_DSN`)에서 읽는다.

### 2-2. 기본 재생 (`ee_twist`)

```bash
# 터미널 1 — 스택 기동 (replay_arm_path 기본값 = ee_twist)
ros2 launch rdfp replay_panda_mock.launch.py

# 터미널 2 — 반드시 /ee_pose 를 재생해야 한다
ros2 run rdfp replay <episode_id> --topic /ee_pose --config dataset_config.yaml
```

`--topic` 은 여러 개 지정할 수 있고, 모든 토픽의 메시지를 stamp 순으로 merge 하여
원래 cadence 로 재생한다.

### 2-3. GUI 로 재생

```bash
ros2 run rdfp replay_gui --config dataset_config.yaml
```

`Mp4ImageReplayer`(이미지)와 `TopicMessageReplayer`(그 외)를 lock-step 으로 구동해
카메라 영상과 관절 데이터의 시간 정렬을 유지한다. "위치 초기화" 버튼은
`move_to_named_target_async("ready")` 를 호출한다.

### 2-4. `target_joint_cmds` 경로로 재생 (재현 충실도 우선)

```bash
# 터미널 1
ros2 launch rdfp replay_panda_mock.launch.py replay_arm_path:=target_joint_cmds

# 터미널 2
ros2 run rdfp replay <episode_id> --topic /target_joint_cmds \
    --config dataset_config.yaml
```

servo 를 `speed_units` 로 운용한다면 `ee_twist` 경로의 게인을 함께 조정한다.

```bash
ros2 launch rdfp replay_panda_mock.launch.py replay_arm_path:=ee_twist \
    ee_twist_linear_gain:=1.0 ee_twist_angular_gain:=1.0
```

### 2-5. 외부 클라이언트로 구동

```bash
ros2 launch rdfp replay_panda_mock.launch.py replay_arm_path:=none
```

`MoveGroupClient.follow_trajectory()` / `plan_trajectory()` 로 직접 구동할 때
사용한다. arm 구동 노드가 없으므로 명령 충돌이 발생하지 않는다.

### 2-6. Docker

```bash
./docker/run_replay_mock.sh                                      # 기본(ee_twist)
./docker/run_replay_mock.sh replay_arm_path:=target_joint_cmds   # launch 인자 전달
```

컨테이너는 `ROS_DOMAIN_ID=31` / `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 로 뜬다.
호스트에서 `replay` CLI 나 `ros2 topic` 을 쓰려면 **동일하게 맞춰야 한다** (3-1 참고).

### 2-7. 주요 launch 인자

기본값은 모두 **전용** `<rdfp share>/config/replay_panda_mock.yaml` 에서 온다.
CLI `arg:=value` 가 YAML 보다 우선하고, `config_file:=<path>` 로 YAML 자체를
갈아끼울 수 있다.

| 인자 | YAML 키 | 기본값 | 설명 |
|---|---|---|---|
| `replay_arm_path` | `replay.arm_path` | `ee_twist` | arm 구동 경로 (`ee_twist` / `target_joint_cmds` / `none`) |
| `enable_image_viewer` | `image_viewer.enabled` | `true` | 이미지 뷰어 기동 여부 |
| `camera_image_topic` | `image_viewer.image_topic` | `/camera/image_raw` | 뷰어가 구독할 이미지 토픽 |
| `ee_twist_source_topic` | `replay.ee_twist.source_topic` | `/ee_pose` | 차분 대상 PoseStamped 토픽 |
| `ee_twist_output_topic` | `replay.ee_twist.output_topic` | `/servo_node/delta_twist_cmds` | twist 출력 토픽 |
| `ee_twist_linear_gain` | `replay.ee_twist.linear_gain` | `2.5` | `1 / scale.linear` (unitless 보정) |
| `ee_twist_angular_gain` | `replay.ee_twist.angular_gain` | `1.25` | `1 / scale.rotational` (unitless 보정) |
| `ee_twist_max_dt` | `replay.ee_twist.max_dt` | `1.0` | 이 값을 넘는 샘플 공백은 건너뜀 |
| `servo_start_timeout` | `replay.servo_start_timeout` | `30.0` | `start_servo` 서비스 대기 한도(초) |
| `base_frame` / `ee_frame` | `ee_pose.base_frame` / `.ee_frame` | `panda_link0` / `panda_hand` | `ee_twist_publisher` 프레임 |
| `ros2_control_hardware_type` | `ros2_control.hardware_type` | `mock_components` | URDF 생성 |
| `log_level` | `log_level` | `info` | `move_group` / `servo` |
| `config_file` | — | `<rdfp share>/config/replay_panda_mock.yaml` | YAML 설정 경로 |

> ⚠️ **`--show-args` 는 `config_file` 하나만 보여준다.** 나머지는 `config_file` 이
> resolve 된 뒤에 선언되기 때문이다. 위 표가 전체 목록이고, 정본은
> [launch/README.md](../../src/rdfp/launch/README.md) 의 §3 "YAML ↔ 인자 대응표" 다.

> 📌 **설정 파일이 `panda_robot.yaml` 에서 분리되었다.** 이 런치는 camera /
> ee_pose_publisher / image_recorder / session_control 을 띄우지 않아, 공유 YAML
> 시절에는 `camera.id` 같은 키를 고쳐도 아무 일이 일어나지 않으면서 경고도
> 없었다. 지금은 그런 키가 파일에 아예 없다. 예전 스크립트가
> `config_file:=...panda_robot.yaml` 을 넘기고 있다면 아래처럼 즉시 실패하므로
> (조용히 잘못된 값으로 뜨지는 않는다), 새 파일을 복사해 쓴다.
>
> ```
> [ERROR] [launch]: Caught exception in launch (...): 'replay'
> ```

---

## 3. 주의할 점

### 3-1. ROS 환경(domain / RMW) 불일치

가장 자주 겪는 문제다. **`ROS_DOMAIN_ID` 와 `RMW_IMPLEMENTATION` 중 하나만
달라도 DDS 디스커버리가 실패한다.**

본 워크스페이스의 표준 환경은 `~/development/ros/.ros2rc` 에 있고, 터미널마다
`rdfp_env` 로 켠다.

```bash
export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp    # .ros2rc 기본값
```

**셸을 여는 것만으로는 켜지지 않는다.** `~/.bashrc` 는 `~/.devrc` 를 읽어
`ros2_env` / `rdfp_env` 를 **정의만** 하는 옵트인 구조다 — 자동 source 는 모든 셸의
`PYTHONPATH` 를 오염시키기 때문이다([환경 가이드 §2.1](../environment/python_env_guide.md)).
`docker/run_*.sh` 도 컨테이너에 같은 값(`31` / `rmw_fastrtps_cpp`)을 주므로,
스택을 호스트에서 띄우든 컨테이너에서 띄우든 **양쪽이 자연히 통한다.**

문제는 **한쪽만 환경을 켜지 않았을 때** 생긴다. 특히 스크립트·IDE 터미널·cron·
에이전트 셸에서 `source install/setup.bash` 만 하고 `ros2 launch` 를 띄우는 경우다 —
이 파일이 `/opt/ros/humble` 을 chain 하므로 `ros2` 명령은 멀쩡히 돌지만
기본값(domain 0 / `rmw_fastrtps_cpp`)으로 떠서, 정상 터미널에서는 그 스택이 전혀
보이지 않는다.

증상은 이렇다. **daemon 캐시 문제처럼 보이지만 아니다.**

```
WARNING: topic [/joint_states] does not appear to be published yet
Could not determine the type for the passed topic
```

진단은 `ros2 daemon` 프로세스를 세어 보는 것이 가장 빠르다. 두 개가 뜨면 환경이
갈린 것이다.

```bash
$ ps -eo args | grep ros2-daemon | grep -v grep
... --ros-domain-id  0 --rmw-implementation rmw_fastrtps_cpp      ← 잘못 띄운 스택
... --ros-domain-id 31 --rmw-implementation rmw_fastrtps_cpp      ← 정상 터미널
```

스택 쪽 프로세스의 실제 환경도 직접 확인할 수 있다.

```bash
tr '\0' '\n' < /proc/<launch_pid>/environ | grep -E "ROS_DOMAIN_ID|RMW_IMPL"
```

**해결은 터미널의 변수를 지우는 것이 아니라 스택을 31 로 다시 띄우는 것이다.**

```bash
rdfp_env
ros2 launch rdfp replay_panda_mock.launch.py
```

### 3-2. YAML / launch 수정은 재빌드해야 반영된다

`setup.py` 의 `data_files` 가 `config/*` 와 `launch/*.py` 를 `share/rdfp/` 로
**복사**한다. 런치는 `get_package_share_directory("rdfp")` 로 설치본을 읽으므로,
`src/rdfp/config/*.yaml` 이나 `src/rdfp/launch/*.py` 를 고쳐도 재빌드 전까지는
반영되지 않는다.

```bash
colcon build --packages-select rdfp && source install/setup.bash
```

**launch helper 는 다른 패키지에 있다.** 이 런치는
`robot_control.launch_helpers.*` 를 **설치된 파이썬 모듈**로 import 하므로,
helper 나 노드 구현을 고쳤다면 위 명령으로는 반영되지 않는다.

```bash
colcon build --packages-select robot_control rdfp && source install/setup.bash
```

재빌드 없이 확인하려면 소스 경로를 직접 지정한다.

```bash
ros2 launch src/rdfp/launch/replay_panda_mock.launch.py
ros2 launch rdfp replay_panda_mock.launch.py config_file:=$PWD/src/rdfp/config/replay_panda_mock.yaml
```

> 파일 이름에 주의한다 — 본 런치의 설정은 `replay_panda_mock.yaml` 이다.
> `panda_robot.yaml` 을 넘기면 `'replay'` 키가 없어 즉시 실패한다 (§2-7).

### 3-3. `replay` CLI 의 `--topic` 기본값

`--topic` 기본값은 `/servo_node/delta_twist_cmds` 다. **어떤 경로를 쓰든 명시적으로
지정하는 것을 권한다.** 기본값 그대로 두면 녹화된 twist 가 servo 로 직접 발행되어,
`replay_arm_path` 설정과 무관하게 예상 밖의 경로로 로봇이 움직인다.

| `replay_arm_path` | 재생할 토픽 |
|---|---|
| `ee_twist` (기본) | `--topic /ee_pose` |
| `target_joint_cmds` | `--topic /target_joint_cmds` |

### 3-4. `ee_twist` 경로의 구조적 한계 (기본 경로다)

구현되어 있다고 해서 재현 정확도가 보장되지는 않는다. 속도 명령 기반이므로:

1. **적분 드리프트** — 위치 오차를 되돌릴 기준이 없어 시간에 따라 누적된다.
2. **시작 pose 의존** — 재생 시작 위치가 녹화 때와 다르면 전체가 오프셋된다.
3. **여유자유도 미재현** — EE twist 는 6-DOF 라 7-DOF 팔꿈치 형상이 달라진다.
4. **특이점 / 한계 스케일링** — servo 가 속도를 줄이면 그 감쇠분이 위치 오차로 남는다.

또한 `EeTwistPublisher` 자체의 손실이 있다.

- **첫 샘플은 항상 버려진다** (차분 대상이 없음).
- `dt > max_dt`(기본 1.0초) 구간은 통째로 건너뛴다.
- `dt <= 0` 인 중복 / 역행 타임스탬프도 버린다.

긴 에피소드일수록 오차가 커진다. 짧은 구간으로 먼저 확인하는 것이 좋다.

### 3-4-1. `ros2 bag play -r` 배속은 이동량을 **반비례로** 바꾼다

직관과 반대다. `ee_twist` 경로에서 **배속을 올리면 로봇이 덜 움직인다.**

`EeTwistPublisher` 는 속도를 **메시지 stamp 기준**으로 계산한다.

```python
dt = t - prev[0]                       # 원본 stamp 차이 — 재생 배속과 무관
linear = kl * (pos - prev_pos) / dt
```

배속을 바꿔도 stamp 간격은 원본 그대로이므로 **속도 명령의 크기가 변하지
않는다.** 그런데 servo 는 그 속도를 **벽시계 시간**으로 적분한다.

```
이동량 = 속도(불변) × 재생 지속시간(배속에 반비례)
```

실측(`ee_pose_bag`, 시작 자세 대비 최대 관절 편차):

| 재생 | 지속시간 | 최대 편차 |
|---|---|---|
| `-r 1` | 51.6s | **0.412 rad** (23.6°) |
| `-r 3` | 17.2s | 0.153 rad (8.8°) |

즉 동작을 크게 보려고 `-r 3` 을 주면 오히려 1/3 로 줄어든다.

| 조작 | 속도 크기 | 지속시간 | 총 이동량 |
|---|:-:|:-:|:-:|
| `-r 3` | 그대로 | 1/3 | **1/3 ↓** |
| `-r 0.5` | 그대로 | 2배 | **2배 ↑** (stale 주의, 3-4-3 참고) |
| 게인 10배 | **10배** | 그대로 | **10배 ↑** |

**이동량을 실제로 키우는 유일한 방법은 게인이다.**

```bash
ros2 launch rdfp replay_panda_mock.launch.py \
    ee_twist_linear_gain:=25.0 ee_twist_angular_gain:=12.0
```

기본값 2.5 / 1.25 는 servo 의 `unitless` 스케일을 정확히 상쇄해 **원본 속도
1.0배** 를 만드는 값이다. 올리면 그만큼 원본과 달라지므로 시연·확인 용도로만
쓴다.

`target_joint_cmds` 경로는 관절 절대 위치를 명령하므로 배속·게인 어느 것에도
이동량이 좌우되지 않는다.

### 3-4-2. 과거에 녹화된 stamp 는 servo 가 폐기한다

`moveit_servo` 는 매 주기 이렇게 판정한다.

```
now - command.header.stamp >= incoming_command_timeout (기본 0.1s)  →  stale, 무시
```

**에러도 경고도 없이 조용히 버린다.** rosbag 은 `header.stamp` 를 원본 그대로
내보내므로, 며칠 전 녹화한 bag 을 재생하면 stamp 가 수십만 초 과거가 되어 servo
가 전 구간을 무시한다. 증상은 "`/servo_node/delta_twist_cmds` 는 나가는데
`/panda_arm_controller/joint_trajectory` 가 0건" 이다.

이 때문에 `EeTwistPublisher` 는 출력 `header.stamp` 에 입력 표본의 stamp 가
아니라 **발행 시각** 을 싣는다. 차분 간격 `dt` 는 입력 stamp 로 계산하므로 속도
값 자체는 원본 시간축을 그대로 반영한다.

직접 servo 에 twist 를 넣는 다른 경로를 만들 때도 같은 규칙을 지켜야 한다.
확인은 이렇게 한다.

```bash
ros2 topic echo /servo_node/delta_twist_cmds --field header.stamp --once
date +%s        # 두 값의 차이가 0.1초 이내여야 한다
```

### 3-4-3. 발행 주기가 아니라 **실효 주기** 가 기준이다

`ee_twist_publisher` 는 `dt <= 0` 인 메시지를 버린다. 소스가 pose 를 낮은 주기로
갱신하면서 높은 주기로 반복 발행하면, 발행 주기는 높아 보여도 실효 주기는 훨씬
낮다.

`ee_pose_bag` 실측:

```
발행 10328건 / 51.6s = 200 Hz
직전과 stamp 동일        : 9333 (90.4%)
stamp·pose 둘 다 동일    : 9333          → 완전 중복(정보 손실 없음)
stamp 같은데 pose 다름   :    0
고유 stamp 995개         → 실효 19.3 Hz
```

이 경우 버려지는 것이 전부 완전 중복이라 **정보 손실은 없다.** 다만 servo 가
보는 명령 주기는 19 Hz 이지 200 Hz 가 아니다.

실효 주기 기준 동작 구간:

| 실효 twist 주기 | 상태 |
|---|---|
| < 10 Hz | ❌ servo stale → 끊김. 개루프라 감속분이 **영구 오차** 로 남음 |
| 10 ~ 29 Hz | ⚠️ 동작하지만 servo 출력 주기(29.4 Hz)를 못 채워 계단식 |
| ≈ 30 Hz 이상 | ✅ servo 와 정합 (초과분은 버려지나 무해) |

`-r 0.5` 로 재생하면 위 bag 은 9.7 Hz 가 되어 임계 아래로 떨어진다. 이때는
타임아웃을 늘린다.

```bash
ros2 param set /servo_node incoming_command_timeout 0.5
```

실효 주기 확인은 두 토픽의 `hz` 를 비교하면 된다.

```bash
ros2 topic hz /ee_pose                        # 발행 주기
ros2 topic hz /servo_node/delta_twist_cmds    # 실효 주기
```

### 3-5. servo 는 `start_servo` 없이는 동작하지 않는다

`moveit_servo` 는 `start_servo` (`std_srvs/Trigger`) 호출 전까지 입력을 **조용히
무시한다.** 에러도 경고도 없다. `replay_arm_path:=ee_twist` 는
`servo_auto_start_node` 를 함께 띄워 이 문제를 해결하지만, 직접 servo 를 쓰는
경우에는 수동 호출이 필요하다.

```bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger
```

`servo_auto_start` 노드는 일회성이라 요청 후 종료된다. 로그에
`start_servo requested; servo now accepts twist commands` 가 찍혔는지 확인한다.

### 3-6. `/session` 토픽이 없다

`session_control_node` 를 기동하지 않으므로 `/session` 토픽과
`start_session` / `stop_session` 서비스가 **존재하지 않는다.** 의도된 설계다.

결과적으로:

- `teleop_keyboard` 는 replay 스택에서 실행할 수 없다
  (`SessionControlClient: service 'start_session' not available` 로 실패).
- `rdfp_image_viewer_node` 의 오버레이에서 세션 상태 필드는 기본값으로 고정된다.

### 3-7. DB 는 joint 이름을 저장하지 않는다

`joint_states` 테이블의 컬럼은 `position` / `velocity` / `effort` 뿐이다.
**`sensor_msgs/JointState` 의 `name` 은 적재 단계에서 버려진다.**

```sql
CREATE TABLE IF NOT EXISTS joint_states (
    ..., position DOUBLE PRECISION[], velocity DOUBLE PRECISION[], effort DOUBLE PRECISION[]
);   -- name 컬럼 없음
```

따라서 DB 재생으로 들어온 `/target_joint_cmds` 메시지는 `name` 이 **빈 배열**이다.
`target_joint_cmds_executor` 는 이를 위해 `joint_names` 파라미터 폴백을 갖는다.

| 우선순위 | 출처 | 해당 상황 |
|---|---|---|
| 1 | 메시지의 `name` | 라이브 토픽을 직접 중계 |
| 2 | `joint_names` 파라미터 | **DB 재생** (이름 유실) |

런치는 panda_joint1~7 을 넘기므로 정상 동작한다. `ros2 run` 으로 직접 띄울 때
둘 다 비면 빈 `joint_names` 로 발행되고 컨트롤러가 거부할 수 있다 (경고 로그 출력).

```bash
ros2 run robot_control target_joint_cmds_executor --ros-args \
    -p joint_names:="[panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7]"
```

`velocity` / `effort` 는 길이가 `position` 과 일치할 때만 실린다. JTC 는 길이가
어긋난 궤적을 통째로 거부하기 때문이다.

### 3-8. 재생기(replayer) 는 일회성이다

`TopicMessageReplayer` / `Mp4ImageReplayer` 모두 `start()` 를 두 번 호출하면
`RuntimeError('… already started')` 가 발생한다. 워커가 정상 종료된 뒤에도
마찬가지다 — 이터레이터가 소진되고 메시지 stamp 가 in-place 로 변형되어 재사용하면
조용히 0건만 발행하기 때문이다. **다시 재생하려면 새 인스턴스를 만들어야 한다.**

### 3-9. 카메라 영상이 안 나오는 경우

replay 스택은 `camera_node` 를 띄우지 않는다. 이미지는 DB 에 적재된 MP4 에서
재생되므로, 이미지 토픽을 **명시적으로 재생 대상에 포함해야** 뷰어에 프레임이
표시된다.

```bash
ros2 run rdfp replay <episode_id> \
    --topic /target_joint_cmds /camera/image_raw --config dataset_config.yaml
```

이때 `dataset_config.yaml` 의 **`output_mp4_dir` 이 설정되어 있어야 한다.**
비어 있으면 다음 로그와 함께 해당 토픽만 조용히 건너뛴다.

```
output_mp4_dir is required to replay image topic /camera/image_raw;
set it in the dataset config
```

`replay_gui` 는 이미지(`Mp4ImageReplayer`)와 그 외 토픽(`TopicMessageReplayer`)을
분리해 lock-step 으로 구동하므로 시간 정렬이 더 정확하다. 영상까지 함께 볼 때는
GUI 쪽이 낫다.
