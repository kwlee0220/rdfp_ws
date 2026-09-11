# Gazebo (Fortress) 백엔드 브링업 가이드

> 대상: ROS 2 Humble + Gazebo Fortress (gz-sim 6) + MoveIt2
> 관련 설계: [multi_simulator_backend_design.md](multi_simulator_backend_design.md)
> launch: [panda_gazebo.launch.py](../../src/robot_control/launch/panda_gazebo.launch.py)

기존 `mock_components` 기반 스택을 Gazebo 물리 시뮬레이션으로 교체해 실행하는
백엔드다. MoveIt / RViz / ee_pose / gripper 등 상위 노드는 mock 과 **동일한
helper 를 그대로 재사용**하며, 달라지는 것은 로봇 백엔드(물리 + controller_manager
호스팅)뿐이다.

---

## 1. 사전 설치 (필수)

ros2_control 통합 플러그인 `ign_ros2_control` 은 **별도 설치**가 필요하다
(rdfp 빌드 자체에는 불필요하지만 런타임에 반드시 필요).

```bash
sudo apt install ros-humble-ign-ros2-control
# ros_gz 브리지 스택이 없다면:
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-bridge ros-humble-ros-gz-image
```

설치 확인:

```bash
ros2 pkg prefix ign_ros2_control      # 경로가 나오면 OK
ign gazebo --version                   # Gazebo Sim, version 6.x (Fortress)
```

> `package.xml` 에 `ign_ros2_control` / `ros_gz_*` 를 `exec_depend` 로 선언해
> 두었으므로 `rosdep install --from-paths src --ignore-src` 로도 설치된다.

---

## 2. 빌드 & 실행

```bash
cd ~/development/ros/rdfp_ws
colcon build --packages-select rdfp_msgs robot_control robot_twin rdfp
source install/setup.bash

# 기본 (빈 월드, 카메라 없음)
ros2 launch robot_control panda_gazebo.launch.py

# 월드 지정
ros2 launch robot_control panda_gazebo.launch.py world:=empty.sdf

# 손목 카메라 센서까지 시뮬레이션 (/camera/image_raw 로 브리지)
ros2 launch robot_control panda_gazebo.launch.py simulate_camera:=true
```

실행되면 Gazebo 창에 Panda 가 `ready` 자세로 스폰되고, RViz2 의 MoveIt
MotionPlanning 패널에서 플래닝/실행이 가능하다.

---

## 3. 아키텍처 — mock 과 무엇이 다른가

| 항목 | mock (`panda_mock.launch.py`) | gazebo (`panda_gazebo.launch.py`) |
|---|---|---|
| 물리 | `mock_components` (무물리) | Gazebo Fortress 물리엔진 |
| controller_manager | 독립 `ros2_control_node` | gz 프로세스 내 `ign_ros2_control` 플러그인 |
| robot_description | moveit_resources xacro | **rdfp description** (inertial 보강 + gazebo 분기) |
| 베이스 고정 | `static_transform_publisher` (TF만) | URDF `world` 고정 조인트 (물리 앵커) |
| 클럭 | system time | Gazebo `/clock` → `use_sim_time:=true` 전파 |
| 기동 트리거 | `ros2_control_node` 시작 | `create`(스폰) 종료 |
| 카메라 | OpenCV `camera_node` (웹캠) | (옵션) gz 카메라 센서 + ros_gz 브리지 |

controller(`panda_arm_controller`, `panda_hand_controller`,
`joint_state_broadcaster`)·관절 이름(`panda_*`)·액션 인터페이스는 **양쪽이
동일**하다. 따라서 MoveGroup 클라이언트, recorder, session, dataset 등 상위 계층은
백엔드 교체와 무관하게 그대로 동작한다(설계 문서의 "백엔드 계약").

### 기동 시퀀스

```
robot_state_publisher  +  gz_sim(서버/클라이언트)  +  /clock 브리지
        └─ create (robot_description → 월드에 스폰)
              └─[OnProcessExit]─ joint_state_broadcaster
                    └─ panda_arm_controller
                          └─ panda_hand_controller
                                └─ move_group · servo · rviz · ee_pose ·
                                   gripper · (카메라 브리지)
```

---

## 4. 신규/변경 파일

| 경로 | 역할 |
|---|---|
| `src/robot_control/description/panda_links.urdf.xacro` | 베이스 panda.urdf 포크 + 12개 링크 inertial 보강 |
| `src/robot_control/description/panda.ros2_control.xacro` | arm ros2_control + `gazebo` 분기 추가 |
| `src/robot_control/description/panda_hand.ros2_control.xacro` | hand ros2_control + `gazebo` 분기 (mimic 처리) |
| `src/robot_control/description/panda.gazebo.xacro` | ign_ros2_control 플러그인 · world 앵커 · 카메라 센서 매크로 |
| `src/robot_control/description/panda.urdf.xacro` | 최상위 — 위 요소 결합, `ros2_control_hardware_type` 로 분기 |
| `src/robot_control/description/initial_positions.yaml` | 초기 관절값(ready) |
| `src/robot_control/robot_control/launch_helpers/gazebo.py` | gazebo moveit config 빌더 · gz_sim include · 스폰 · 브리지 · 기동 핸들러 |
| `src/robot_control/launch/panda_gazebo.launch.py` | Gazebo 풀 스택 브링업 |
| `setup.py` | `description/*` 를 share 로 설치 |
| `package.xml` | ros_gz / ign_ros2_control 등 런타임 의존성 |

> 단일 description 으로 mock / gazebo / isaac 을 모두 커버한다.
> `ros2_control_hardware_type:=mock_components` 로 빌드하면 추가된 inertial 만
> 더해진 동일 모델이 나오며(무해), 기존 mock 동작과 호환된다.

---

## 5. 검증 상태

- ✅ xacro → URDF 생성: mock / gazebo / gazebo+camera 3개 분기 모두
  `check_urdf` 통과.
- ✅ `colcon build` 성공, `description/*` share 설치 확인,
  설치본에서 `$(find rdfp)` 해석 정상.
- ✅ `ros2 launch robot_control panda_gazebo.launch.py --show-args` 정상
  (launch description 빌드 성공). robot_description 은 `simulate_camera` 를
  런타임에 resolve 하는 deferred `Command` 로 구성됨.
- ⚠️ **런타임(실제 Gazebo 물리/제어) 검증은 미완료** — `ign_ros2_control`
  설치 + 디스플레이 환경이 필요하다. 아래 알려진 이슈를 함께 확인할 것.

---

## 6. 알려진 이슈 / 트러블슈팅

### 6.1 메시(model://) 로딩 실패
ign 의 URDF 파서는 `package://moveit_resources_panda_description/...` 를
`model://moveit_resources_panda_description/...` 로 변환한다. gz 가 이 경로를
못 찾으면 `Unable to find file with URI [model://...link0.dae]` 류 에러가 나며
로봇이 **보이지 않는다**(컨트롤러는 정상 로드됨).

→ launch 의 `gz_resource_path_actions()` 가 `IGN_GAZEBO_RESOURCE_PATH` /
`GZ_SIM_RESOURCE_PATH` 에 `moveit_resources_panda_description` share 의 부모
경로를 자동 추가하므로 **별도 설정 없이 해결**된다. 여전히 안 보이면 수동 확인:

```bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:\
$(dirname $(ros2 pkg prefix moveit_resources_panda_description)/share/moveit_resources_panda_description)
```

### 6.2 그리퍼 mimic 조인트
`panda_finger_joint2` 는 `panda_finger_joint1` 을 mimic 한다. ign_ros2_control
(Fortress)의 mimic 지원이 제한적이라, description 의 gazebo 분기에서는
finger_joint2 의 `command_interface` 를 빼고 mimic 파라미터만 둔다
([panda_hand.ros2_control.xacro](../../src/robot_control/description/panda_hand.ros2_control.xacro)).
그리퍼 거동이 비정상이면 이 부분을 우선 점검한다.

### 6.3 inertial 근사값
링크 inertial 은 Franka 공개 식별 파라미터(Gaz et al., 2019) 기반의 **근사값**
이다. 위치 제어 시뮬레이션에는 충분하나, 동역학 정확도가 중요한 용도라면
실제 로봇 파라미터로 보정한다.

### 6.4 컨트롤러가 안 붙음 (spawner timeout)
`create` 가 끝나기 전 controller_manager 가 아직 안 떠 있으면 spawner 가
재시도하다 timeout 날 수 있다. spawner timeout 은 120s 로 잡혀 있으나, 느린
머신에서 문제가 되면 `controller_launch_helper.py` 의 timeout 을 늘린다.

### 6.5 use_sim_time
모든 노드에 `use_sim_time:=true` 가 전파된다(`SetParameter`). 외부에서
별도 노드를 띄워 이 스택과 통신할 때도 반드시 `use_sim_time:=true` 로 맞춰야
타임스탬프/TF 가 어긋나지 않는다. recorder/dataset 연동 시 특히 주의.

---

## 7. 풀 앱 launch (teleop / recorder 포함)

`panda_gazebo.launch.py` 는 MoveIt 백엔드까지만 띄운다(=`panda_mock` 대응).
teleop / recorder 처럼 rdfp 애플리케이션 노드를 함께 쓰려면
**`rdfp_panda_gazebo.launch.py`** 를 사용한다(=`rdfp_panda_mock` 대응).

```bash
# Gazebo + MoveIt + session_control + (옵션) viewer/recorder
ros2 launch rdfp rdfp_panda_gazebo.launch.py

# 다른 터미널에서 teleop (use_sim_time 을 맞춘다)
ros2 run rdfp teleop_keyboard --ros-args -p use_sim_time:=true
```

추가 기동 노드: `session_control`(teleop 필수),
`target_joint_states_publisher`, 그리고 `simulate_camera:=true` +
`enable_image_viewer:=true` / `enable_image_recorder_node:=true` 시
카메라 의존 노드. 모든 노드에 `use_sim_time:=true` 가 전파된다.

> `teleop_keyboard` 는 `session_control_node` 의 `start_session` 서비스가 없으면
> 기동 시 예외로 죽는다. `panda_gazebo.launch.py` 만 띄운 상태에서 teleop 을
> 실행하면 이 에러가 난다 → `rdfp_panda_gazebo.launch.py` 를 쓰거나
> `session_control_node` 를 별도로 띄울 것.

### 6.6 키를 한 번 눌렀는데 로봇이 멈추지 않고 계속 드리프트 / 또는 trajectory 거부
원인은 teleop 가 아니라 servo→JTC 인터페이스다. MoveIt Servo 는
`publish_joint_velocities: true` 일 때 모든 trajectory point(끝점 포함)에 jogging
속도를 넣는다(`use_gazebo:true` 의 redundant point 30개에도 동일 속도 복제).
position 인터페이스 JTC 는 이 끝점 속도를 두 가지로 잘못 처리한다.
- `allow_nonzero_velocity_at_trajectory_end: true`(기본) → 끝나도 그 속도로 계속
  진행 → **한 번 입력으로 무한 드리프트**.
- `false` → "Velocity of last trajectory point is not zero" 로 **거부** → 저깅 불가.

→ 해결: Gazebo 백엔드 servo 파라미터에서 **`publish_joint_velocities: false`**
([build_gazebo_servo_params](../../src/robot_control/robot_control/launch_helpers/gazebo.py)).
위치만 발행하면 끝점 속도 자체가 없어 거부도 드리프트도 사라지고, 입력이 끊기면
마지막 위치에서 정지한다. `use_gazebo: true`(타이밍용 redundant point)와
`config/gazebo_ros2_controllers.yaml` 의 `allow_nonzero_velocity_at_trajectory_end:
false`(명시적 정지 보장; 속도가 없으므로 이제 거부도 없음)도 함께 적용되어 있다.

여전히 드리프트하면 teleop / servo / 컨트롤러를 모두 재기동했는지 확인한다
(rebuild 후 떠 있던 프로세스에는 반영되지 않음).

## 8. 후속 과제 (이번 범위 밖)

- 설계 문서의 Phase 1(app/backend launch 분리)로 mock/gazebo launch 중복 통합.
- 그리퍼 mimic 조인트 거동 검증(§6.2).
- Isaac Sim 백엔드(`backend_isaac`) — description 의 `isaac` 분기는 이미 존재.
