# robot_control launch 안내

**로봇 제어 계층의 launch 진입점과 helper 모듈.**
학습 데이터 수집(세션·녹화·데이터셋) 없이 Panda + MoveIt2 스택만 띄우는 데 필요한 모든 것이 이 문서에 있다.

```bash
ros2 launch robot_control panda_mock.launch.py        # 가장 많이 쓰는 진입점
```

> 수집 계층까지 띄우려면 `rdfp` 패키지의 `rdfp_*` launch 를 쓴다 — [src/rdfp/launch/README.md](../../rdfp/launch/README.md). 그쪽 launch 들도 helper 는 **이 패키지 것을 쓴다** (§5). 패키지를 나눈 근거는 설계서 [§7.6](../../../docs/rdfp_framework_design.md).

---

## 목차

- [1. Launch 파일](#1-launch-파일)
- [2. Launch 인자](#2-launch-인자)
- [3. 설정 파일](#3-설정-파일)
- [4. 실행 구조와 순차 기동](#4-실행-구조와-순차-기동)
- [5. Helper 모듈](#5-helper-모듈)
- [6. 어느 launch 를 쓸까](#6-어느-launch-를-쓸까)
- [7. 유지보수 메모](#7-유지보수-메모)

---

## 1. Launch 파일

| launch | 백엔드 | 요약 |
|---|---|---|
| `panda_mock.launch.py` | mock | **기본 진입점.** ros2_control(mock) + MoveIt + RViz + 카메라 + ee_pose + 그리퍼 + scene |
| `panda_jgpc_mock.launch.py` | mock | 위와 노드 구성 동일, arm 컨트롤러 **타입만** JGPC 로 교체 |
| `panda_gazebo.launch.py` | Gazebo (Fortress) | `panda_mock` 의 Gazebo 대응본 |
| `panda_functionbay.launch.py` | 펑션베이+Crisp | **ros2_control 없음** — 토픽 브리지로 연동 |
| `panda_isaac.launch.py` | Isaac Sim | **ros2_control 없음** — 토픽 브리지. 팔·그리퍼·scene·파지 (Phase 0~6). 카메라는 노드가 아니라 Isaac 이 직접 발행한다 |

### `panda_mock.launch.py`

`static_tf` + `robot_state_publisher` + `ros2_control_node` + controller spawner 3종을 순차 기동한 뒤 `move_group`, `servo_node`, `rviz2`, `camera_node`, `ee_pose_node`, `gripper_action_node`, `mock_scene_state_node` 를 일괄 spawn 한다.
순서와 근거는 §4.

### `panda_jgpc_mock.launch.py`

arm 컨트롤러 타입만 바꾼 variant 다 — 컨트롤러 *이름* 은 `panda_arm_controller`로 동일해서 spawner helper 를 그대로 재사용한다.

- 설정: `config/panda_jgpc_ros2_controllers.yaml`
  (`joint_trajectory_controller/JointTrajectoryController` →
  `position_controllers/JointGroupPositionController`)
- arm 명령 인터페이스가 `/panda_arm_controller/joint_trajectory`
  (JointTrajectory) → `/panda_arm_controller/commands`
  (`std_msgs/Float64MultiArray`, **관절 이름 없음 — 배열 순서가 컨트롤러의 `joints` 파라미터와 일치해야 한다**) 로 바뀐다.
- servo 파라미터도 launch 안에서 `command_out_type` /`command_out_topic` / `publish_joint_velocities` 를 덮어쓴다.

> ⚠️ JGPC 는 `FollowJointTrajectory` 액션을 제공하지 않으므로 **arm 의 `move_group` plan & execute 가 동작하지 않는다** (planning / IK / RViz 표시는 정상). arm 제어는 servo, commands 토픽 직접 발행, 또는 `create_move_group_client()` 가 돌려주는 `MoveGroupJgpcClient` 로 한다 —
> [docs/moveit/MoveGroupJgpcClient_UserGuide.md](../../../docs/moveit/MoveGroupJgpcClient_UserGuide.md).

### `panda_functionbay.launch.py`

펑션베이는 ros2_control 하드웨어 플러그인을 제공하지 않고 **ROS 2 토픽으로만**
연동한다. 그래서 `controller_manager` · `joint_state_broadcaster` · spawner 3종이
없고, `robot_control/functionbay/` 의 브리지 노드가 그 자리를 대신한다.

- `joint_state_fusion` — `/output/panda_joint`(이름 없는 배열) → `/joint_states`
- `readiness_gate` — 첫 관절 보고를 기동 완료 신호로 삼고 종료 (spawner 대체)

시뮬레이터가 내부 보간을 하지 않으므로 명령 스트리밍에 `publish_rate=50.0` 이
**필수**다 (미지정 시 MoveIt 의 ~10 Hz 리샘플 그대로 계단 동작). 그리퍼는 실물이
Robotiq 2F-85 라 Panda Hand 전제와 어긋나 **아직 연동하지 않는다**.

상세: [docs/simulation/functionbay_backend_design.md](../../../docs/simulation/functionbay_backend_design.md)

### `panda_isaac.launch.py`

`panda_functionbay` 와 같은 구조다 — ros2_control 없이 토픽으로만 연동하므로
`controller_manager` · spawner 가 없고, `readiness_gate`(펑션베이 것 그대로)의 **종료
코드**를 기동 신호로 쓴다. `joint_state_fusion` 은 **쓰지 않는다** — Isaac 이 관절
이름과 finger 2개를 모두 채워 보내므로 중계할 것이 없다.

펑션베이와 갈리는 지점은 **`use_sim_time`** 이다. Isaac 은 `/clock` 을 발행하므로
기본값이 `true` 이고 모든 노드에 전파한다. 이 값이 한 노드라도 어긋나면 `move_group`
이 `Failed to fetch current robot state` 로 **조용히** 무력해진다.

**Phase 0~9 가 구현·검증됐고, servo 경로(Phase 10)는 실기 확인까지 마쳤다.** 인자로
필요한 부분만 켠다.

| 인자 | 기본 | 켜면 |
|---|---|---|
| — | — | Phase 0·1: `/clock`·`/joint_states`·MoveIt·팔 명령 |
| `enable_gripper` | `false` | `gripper_action_bridge` + `gripper_action_node` (Phase 2) |
| `enable_scene` | `false` | `isaac_scene_state_node` — `/scene/objects` 발행 (Phase 3) |
| `enable_servo` | `false` | `servo_node` + `servo_auto_start` + `isaac_servo_bridge`. **teleop 두 경로가 모두 여기로 수렴**한다 |
| `enable_rviz` | `true` | VRAM 이 빠듯한 호스트에서는 false 를 권한다 |

**MoveIt 의 planning scene 에는 아무것도 들어가지 않는다** (2026-09-01 결정).
`/scene/objects` 는 조작 대상 채널이라 `isaac_scene_state_node` 가 `dynamic: true` 인
물체만 싣고, 조작 대상을 planning scene 에 넣으면 파지 자세가 시작 자세 충돌이 되어
관절공간 계획이 `INVALID_MOTION_PLAN`(-2)으로 거부된다 — 넣을 것이 남지 않아
`planning_scene_sync` 노드를 삭제했다.

> ⚠️ **감수한 위험 — 관절공간 이동 중 팔이 탁자를 통과한다.** 자기충돌 검사는 그대로
> 동작하고, 파지 동작은 전부 cartesian 이라(`avoid_collisions` 기본값 `False`) 영향이
> 없다. 실제로 잃는 것은 `move_to_named_target` / `move_to_joints` 중의 탁자 회피뿐이다.
>
> 근거와 되살릴 때의 대안은
> [scene_objects_guide.md](../../../docs/scene/scene_objects_guide.md) §7 에 있다.

**Isaac 쪽 준비가 선행되어야 한다** — `setup_graph.py` 의 `PHASE` 가 쓰려는 단계
이상이어야 하고, scene 을 열 때마다 그 스크립트를 다시 돌려야 한다(ROS 2 bridge 확장
활성화가 USD 에 저장되지 않는다).

**카메라 인자는 없다.** 이미지를 카메라 노드가 아니라 **Isaac 그래프가 직접**
발행하므로(`PHASE>=4`), 이 launch 가 켜고 끌 대상이 아니다. 보기·녹화는 수집 계층의
몫이며 해상도·주파수·토픽은 `config/isaac_scene.json` 의 `camera` 블록이 단일 출처다.

수용 기준은 `scripts/isaac/is_check_phase0.py` ~ `is_check_phase6.py` 가 검사한다
(`is_check_phase5.py` 는 attach/detach 철회와 함께 삭제됐다).
배관이 의심되면 `scripts/isaac/is_topics.py` 를 **먼저** 돌린다. 시작 자세가 scene 과
충돌해 계획이 `-2`(INVALID_MOTION_PLAN)로 거부되면 `scripts/isaac/is_recover.py` 로
계획 없이 빠져나온다.

수집 계층까지 함께 띄우려면 `ros2 launch rdfp rdfp_panda_isaac.launch.py` 를 쓴다.

상세: [docs/simulation/isaac_backend_skeleton.md](../../../docs/simulation/isaac_backend_skeleton.md)

### `panda_gazebo.launch.py`

mock 백엔드와의 차이는 **로봇 백엔드뿐**이고 MoveIt / RViz / ee_pose / 그리퍼는 동일하게 재사용한다.

- 별도 `ros2_control_node` 를 띄우지 않는다 — `ign_ros2_control` 플러그인이 controller_manager 를 호스팅한다.
- `static_transform_publisher` 대신 URDF 의 `world` 고정 조인트로 베이스를 앵커링.
- 모든 노드에 `use_sim_time:=true` 를 전파한다.
- 카메라는 옵션이다 (`simulate_camera:=true` → gz 카메라 센서 + ros_gz 브리지).

상세: [docs/simulation/gazebo_bringup_guide.md](../../../docs/simulation/gazebo_bringup_guide.md)

---

## 2. Launch 인자

이 패키지의 launch 는 모두 **`--show-args` 가 그대로 동작한다.** 인자를 선언 시점에 전부 노출하기 때문이다.

```bash
ros2 launch robot_control panda_mock.launch.py --show-args
```

> 수집 계층(`rdfp`)의 `rdfp_*` launch 는 `OpaqueFunction` 안에서 인자를 선언해 `--show-args` 가 1~2개만 출력한다. 그쪽은 [해당 README](../../rdfp/launch/README.md) §3의 표를 봐야 한다. **이 차이가 두 계열의 가장 큰 사용성 차이다.**

### 2.1 `panda_mock` / `panda_jgpc_mock` — 16개

두 launch 의 인자는 **완전히 동일**하다 (컨트롤러 타입만 다르고 인자 체계는 공유).

**로봇 공통 (7개)**

| 인자 | 기본값 | 선언 위치 | 설명 |
|---|---|---|---|
| `ros2_control_hardware_type` | `mock_components` | `launch_helpers/common.py` | ros2_control 하드웨어 인터페이스 |
| `log_level` | `info` | `launch_helpers/common.py` | `move_group` / `servo` 로그 레벨. `debug`\|`info`\|`warn`\|`error`\|`fatal` |
| `base_frame` | `panda_link0` | `launch_helpers/ee_pose.py` | EE pose TF lookup 기준 프레임 |
| `ee_frame` | `panda_hand` | `launch_helpers/ee_pose.py` | EE pose TF lookup 대상 프레임 |
| `publish_rate` | `50.0` | `launch_helpers/ee_pose.py` | `/ee_pose` 발행 Hz |
| `enable_scene` | `true` | `launch_helpers/scene.py` | 백엔드 scene 상태 노드 기동 여부 |
| `scene_publish_rate` | `2.0` | `launch_helpers/scene.py` | `/scene/objects` 발행 Hz |

`scene_publish_rate` 가 `publish_rate` 라는 이름을 쓰지 않는 이유는 EE pose 인자와 이름이 충돌하기 때문이다.

**카메라 (9개)** — 기본값은 `config/image_pipeline.yaml` 에서 온다.

| 인자 | YAML 키 | 현재 값 | 설명 |
|---|---|---|---|
| `enable_camera_node` | `camera.enabled` | `true` | |
| `camera_id` | `camera.id` | `…/data/shibuya_7_8.mp4` | 장치 인덱스 또는 파일/URI |
| `camera_image_topic` | `camera.image_topic` | `/camera/image_raw` | |
| `camera_info_topic` | `camera.info_topic` | `/camera/camera_info` | |
| `camera_status_topic` | `camera.status_topic` | `/camera/camera_status` | |
| `camera_fps` | `camera.fps` | `10` | |
| `camera_resolution` | `camera.resolution` | `1280x720` | |
| `camera_frame_id` | `camera.frame_id` | `camera_link` | |
| `camera_compress_image` | `camera.compress_image` | `false` | `rdfp_camera_node`(rdfp 패키지) 는 미지원 |

> **시뮬레이터 계열은 이 9개를 쓰지 않는다.** `panda_functionbay` 는
> `declare_simulator_camera_arguments()` 로 **`enable_camera_node` 와
> `camera_image_topic` 둘만** 선언한다 (`panda_isaac` 은 카메라 인자가 아예 없다).
>
> 카메라 주기·해상도·`camera_id` 는 **우리가 정하는 값이 아니라 시뮬레이터가 주는
> 대로 쓰는 값**이라 인자로 열지 않는다. 열어 두면 실제와 다른 값을 넣을 수 있고,
> 특히 레코더가 CFR 이라 fps 가 어긋나면 **영상의 시간축이 통째로 밀린다.**
> `camera_info` 도 OpenCV 경로 전용이다 — 시뮬레이터는 제공하지 않는다.
>
> `enable_camera_node` 는 남긴다(기본 **`false`**) — USB 카메라를 함께 붙이는 경우가
> 있어서이며, 그때 쓰는 설정은 `image_pipeline.yaml` 에서 직접 온다.

> **이 두 launch 는 설정 파일을 갈아끼울 argument 가 없다.** `image_pipeline.yaml`을 기본값 원천으로만 쓴다. 값을 바꾸려면 `arg:=value` 로 개별 지정하거나 YAML 자체를 수정한다. 파일 교체가 필요하면 `rdfp_panda_mock` 계열을 쓴다 (`image_pipeline_config_file:=`).

### 2.2 `panda_isaac` — 15개

`--show-args` 로 확인한 실제 목록이다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `ros2_control_hardware_type` | `mock_components` | **쓰이지 않는다** — 공통 helper 가 선언할 뿐 이 스택엔 ros2_control 이 없다 |
| `log_level` | `info` | |
| `base_frame` / `ee_frame` / `publish_rate` | `panda_link0` / `panda_hand` / `50.0` | `/ee_pose` 공통 인자 |
| `use_sim_time` | **`true`** | Isaac 이 `/clock` 을 발행한다. 한 노드라도 어긋나면 `move_group` 이 **조용히** 무력해진다 |
| `joint_state_topic` | `/joint_states` | Isaac 이 관절 상태를 내보내는 토픽 |
| `isaac_ready_timeout` | `60.0` | 첫 관절 보고 대기 한도(초). 넘기면 launch 가 실패한다 |
| `enable_rviz` | `true` | |
| `enable_gripper` | `false` | Phase 2 |
| `gripper_command_topic` | `/isaac/gripper_command` | |
| `enable_scene` | `false` | Phase 3 — `/scene/objects` 발행 |
| `scene_publish_rate` | `2.0` | |
| `sync_planning_scene` | `true` | 정적 물체를 MoveIt 장애물로 |
| `enable_servo` | `false` | servo(twist) 경로 |

**카메라 인자가 없다.** 이미지는 Isaac 그래프가 직접 발행하며, 해상도·주파수·토픽은
`config/isaac_scene.json` 의 `camera` 블록이 단일 출처다.

### 2.3 `panda_functionbay` — 11개

| 인자 | 기본값 | 설명 |
|---|---|---|
| `ros2_control_hardware_type` · `log_level` · `base_frame` · `ee_frame` · `publish_rate` | — | 공통 |
| `servo_linear_scale` | **`0.8`** | MoveIt 기본 `0.4` 가 아니다. **런타임 변경이 안 먹으므로** 여기서 준다 |
| `enable_camera_node` · `camera_image_topic` | `false` · — | 시뮬레이터 계열은 이 둘만 연다 |
| `fb_joint_report_topic` · `fb_finger_position` · `fb_ready_timeout` | — | 펑션베이 전용 |

### 2.4 `panda_gazebo`

| 인자 | 설명 |
|---|---|
| `log_level`, `base_frame`, `ee_frame`, `publish_rate` | 위 공통 인자와 동일 |
| `world` | gz world 파일 |
| `enable_rviz` | RViz 기동 여부 |
| `simulate_camera` | gz 카메라 센서 + ros_gz 브리지 기동 |
| `camera_image_topic`, `camera_info_topic` | 브리지 출력 토픽 |
| `gz_args`, `gz_version`, `ign_args`, `ign_version` | gz-sim 실행 인자/버전 |
| `debugger`, `debug_env`, `on_exit_shutdown` | 디버깅·종료 정책 |

`ros2_control_hardware_type` / `enable_camera_node` / scene 인자가 **없다** — 백엔드가 Gazebo 라 하드웨어 타입이 무의미하고, 카메라는 `simulate_camera` 로 갈음한다.

> 이 표만 `--show-args` 로 확인하지 못했다. 이 환경에는 `ros_gz_sim` 이 없어
> launch 로드 자체가 실패한다(`PackageNotFoundError`). 나머지 넷은 실제 출력을 옮겼다.

---

## 3. 설정 파일

`setup.py` 가 `config/*` 와 `description/*` 를 `share/robot_control/` 로 설치한다.

| 파일 | 읽는 launch | 교체 argument |
|---|---|---|
| `config/image_pipeline.yaml` | `panda_mock`, `panda_jgpc_mock` | — (기본값 원천) |
| `config/panda_jgpc_ros2_controllers.yaml` | `panda_jgpc_mock` | — |
| `config/gazebo_ros2_controllers.yaml` | `panda_gazebo` | — |
| `config/panda.rviz` | RViz 를 띄우는 전부 | — |
| `description/*.xacro` | `panda_gazebo` | — |

교체 argument 가 비어 있는 것은 경로가 코드 상수이기 때문이다.

### 3.1 파일이 세 층위로 나뉜다

| 층위 | 읽는 주체 | 형식 | 파일 |
|---|---|---|---|
| ① argument 기본값 | launch (`yaml.safe_load`) | 자유 | `image_pipeline.yaml` |
| ② 노드에 경로째 전달 | 해당 노드 프로세스 | **ROS 파라미터 규격** (`<노드>: ros__parameters:`) | `panda_jgpc_ros2_controllers.yaml`, `gazebo_ros2_controllers.yaml`, `panda.rviz` |
| ③ 외부 패키지 | MoveIt / ros2_control | 각 패키지 규약 | `moveit_resources_panda_moveit_config` 의 6종 |

①만 `config_file:=` 류로 교체할 수 있다(이 패키지에는 그 argument 가 없지만
`rdfp_*` 계열에는 있다). ②는 사용자가 튜닝할 값이 아니라 **코드와 짝이 맞아야
하는 정의**라 상수로 둔다 — `panda_jgpc_ros2_controllers.yaml` 의 `joints` 순서는
`/panda_arm_controller/commands` 배열 순서를 정의하므로, 바꾸면 에러 없이 엉뚱한
관절이 움직인다.

③은 `build_moveit_config()` 가 `moveit_resources_panda_moveit_config` 에서 가져온다 — `panda.urdf.xacro`, `panda.srdf`, `kinematics.yaml`, `joint_limits.yaml`, `gripper_moveit_controllers.yaml`, `panda_simulated_config.yaml`(servo).
마지막 것의 `command_in_type: unitless`(scale.linear=0.4 / rotational=0.8)가 teleop twist 게인 기본값 `2.5 / 1.25` 의 근거다.

### 3.2 `panda_jgpc_ros2_controllers.yaml` 은 왜 별도 파일인가

JGPC 계열 두 launch 만 쓰지만 `panda_robot.yaml`(수집 계층) 과 합치지 않는다.

- `panda_robot.yaml` 의 키는 **JTC 판과 완전히 동일**하다. JGPC 전용 파일로 합치면 복제되어, 한쪽만 고쳤을 때 스택에 따라 다르게 동작한다.
- 층위 ①과 ②는 스키마가 다르다. 한 파일에 `ros__parameters` 블록과 자유 형식 블록이 섞이면 어느 쪽이 어느 규격인지 구분되지 않는다.
- 합치면 `config_file:=` 이 컨트롤러 정의까지 갈아끼우게 된다.
- `panda_jgpc_mock` 은 애초에 `panda_robot.yaml` 을 읽지 않는다.

---

## 4. 실행 구조와 순차 기동

`panda_mock.launch.py` 는 다른 launch 를 `include` 하지 않는다. **helper 를 조합해**
최종 orchestration 만 수행하며, 기능 단위는 helper 모듈이 나눈다.

```text
panda_mock.launch.py
 ├─ ros2_control     (launch_helpers/controller.py, controller_startup.py)
 │   ├─ static_tf
 │   ├─ robot_state_publisher
 │   ├─ ros2_control_node
 │   └─ controller spawner chain
 ├─ MoveIt           (launch_helpers/common.py)
 │   ├─ move_group
 │   ├─ servo_node
 │   └─ ee_pose_node  (launch_helpers/ee_pose.py)
 ├─ RViz             (launch_helpers/common.py)
 │   └─ rviz2
 ├─ 카메라           (launch_helpers/camera.py)
 │   └─ camera_node
 ├─ 그리퍼           (launch_helpers/gripper.py)
 │   └─ gripper_action_node
 └─ scene            (launch_helpers/scene.py)
     └─ mock_scene_state_node
```

### 순차 기동

controller 기동 정책은 `launch_helpers/controller_startup.py` 에 모여 있다.
`RegisterEventHandler(OnProcessExit)` 로 **의도적으로 직렬화**한다.

```text
ros2_control_node start
  -> joint_state_broadcaster spawner
     -> panda_arm_controller spawner
        -> panda_hand_controller spawner
           -> post_hand_actions 실행
```

각 launch 의 `post_hand_actions`:

| launch | post_hand_actions |
|---|---|
| `panda_mock.launch.py` | `move_group`, `servo_node`, `rviz2`, `camera_node`, `ee_pose_node`, `gripper_action_node`, `mock_scene_state_node` |
| `panda_jgpc_mock.launch.py` | 위와 동일 (컨트롤러 타입만 다름) |

### scene 노드는 기본 on 이다

`enable_scene:=false` 로 끌 수 있지만 **기본값 `true` 는 의도적**이다. 노드는 2 Hz 타이머 하나를 쓰고 `/scene/reset` 요청이 오기 전까지 아무것도 바꾸지 않는 반면, 꺼 두면 robot twin 의 `reset_scene` 이 **서비스를 찾지 못해** 실패하면서 로그에 원인이 남지 않는다.

MoveIt 계획 파이프라인은 셋이 로드된다: OMPL, PILZ, CHOMP.

---

## 5. Helper 모듈

**helper 는 `robot_control/launch_helpers/` 에 설치되는 정규 파이썬 모듈이다.**
이 패키지의 launch 와 `rdfp` 패키지의 `rdfp_*` launch 가 **함께** 쓴다.

```python
from robot_control.launch_helpers.camera import create_camera_node
from robot_control.launch_helpers.common import build_moveit_config
```

| 모듈 | 내용 |
|---|---|
| `common.py` | Panda + MoveIt2 공통. `declare_ros2_control_hardware_type_argument`, `declare_log_level_argument`, `build_moveit_config`, `build_servo_params`, `create_static_tf_node`, `create_robot_state_publisher`, `create_move_group_node`, `create_servo_node`, `create_rviz_node` |
| `controller.py` | `ros2_control_node` + spawner 3종 (`joint_state_broadcaster`, `panda_arm_controller`, `panda_hand_controller`) |
| `controller_startup.py` | 순차 기동 event handler (`create_controller_startup_handlers`) |
| `camera.py` | 카메라 argument 선언 + `camera_node` 생성 |
| `image_pipeline.py` | **`config/image_pipeline.yaml` 로더 + camera / image_viewer / image_recorder argument 선언.** 카메라를 띄우는 launch 가 여섯이라 기본값을 한 곳에 모은 모듈 |
| `ee_pose.py` | EE pose argument 선언 + `ee_pose_node` 생성 |
| `gripper.py` | `create_gripper_node()` → `gripper_action_node` (액션 서버가 있는 스택), `create_robotiq_2f_gripper_node()` → `robotiq_2f_gripper_node` (펑션베이 — 관절 목표각 토픽). **갈리는 축은 백엔드가 아니라 실행 수단이다.** **노드가 하나다** — `GripperState.goal` 에 마지막 명령을 실으려면 상태 발행자가 명령을 알아야 해서 구 `gripper_control_node`·`gripper_state_publisher` 를 합쳤다. `/joint_states` 로 대신할 수 없는 이유는 [토픽 규약](../../../docs/topic_naming_contract.md) §2.2, 설계는 [GripperNode_Design.md](../../../docs/moveit/GripperNode_Design.md) |
| `scene.py` | `enable_scene` / `scene_publish_rate` 선언 + `create_mock_scene_node()`. **팩토리는 아직 mock 용 하나뿐이다** — Isaac 은 launch 안에서 직접 만든다 |
| `gazebo.py` | Gazebo 백엔드 전용 — gz-sim 기동, 스폰, 브리지 |

### 두 개의 의도적인 argument 재사용

- `camera.declare_camera_arguments()` 는 `image_pipeline.load_config()` 를 기본 YAML 로 호출하는 **얇은 래퍼**다. 그래서 helper 만 쓰는 launch 도 코드 변경 없이 YAML 값을 따라간다. 설정 파일을 바꿔 끼워야 하면 이 함수 대신 `image_pipeline.declare_camera_arguments(load_config(path))` 를 쓴다 —
  `config_file` 이 resolve 된 뒤여야 하므로 `OpaqueFunction` 이 필요하다.
- `scene.py` 는 `base_frame` 을 **선언하지 않고** `ee_pose.py` 의 선언을 재사용한다. scene 물체 pose 와 EE pose 의 기준 프레임이 갈라지면 에러 없이 틀린 좌표가 나가기 때문이다.

### Launch × Helper 의존 관계

`image_pipeline` 열의 ⭕ 는 `camera` 를 통한 **간접** 의존이다.

| launch | common | controller | controller_startup | camera | image_pipeline | ee_pose | gripper | scene | gazebo |
|---|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| `panda_mock` | ✅ | ✅ | ✅ | ✅ | ⭕ | ✅ | ✅ | ✅ | |
| `panda_jgpc_mock` | ✅ | ✅ | ✅ | ✅ | ⭕ | ✅ | ✅ | ✅ | |
| `panda_gazebo` | ✅ | ✅ | | | | ✅ | ✅ | | ✅ |
| `panda_functionbay` | ✅ | | ✅ | ✅ | ✅ | ✅ | | | |
| `panda_isaac` | ✅ | | ✅ | | | ✅ | | | |

**토픽 브리지 계열(`panda_functionbay` · `panda_isaac`)은 `controller` 를 쓰지 않는다** —
ros2_control 이 없어 spawner 가 없기 때문이다. 대신 `controller_startup` 의
`_chain_or_shutdown` 만 빌려 `readiness_gate` 종료 뒤 노드를 띄운다.

`panda_isaac` 이 `scene` helper 도 안 쓰는 이유는 백엔드 노드가 다르기 때문이다 —
`scene.py` 의 팩토리는 `create_mock_scene_node()` 하나뿐이고, Isaac 은
`isaac_scene_state_node` 를 launch 안에서 직접 만든다.
`camera` 를 안 쓰는 이유는 §1 에 있다(Isaac 이 이미지를 직접 발행한다).

`rdfp` 패키지 launch 의 helper 사용은 [그쪽 README](../../rdfp/launch/README.md) §4 에 있다.

---

## 6. 어느 launch 를 쓸까

| 하려는 것 | launch |
|---|---|
| Panda mock 전체 스택 | `panda_mock.launch.py` |
| arm 을 JGPC 로 제어 (move_group plan&execute 불가) | `panda_jgpc_mock.launch.py` |
| Gazebo 물리 시뮬레이션 | `panda_gazebo.launch.py` |
| **Isaac Sim** 백엔드 | `panda_isaac.launch.py` (Isaac 쪽 준비 선행 — §1) |
| 펑션베이 시뮬레이터 | `panda_functionbay.launch.py` |
| **세션·녹화·데이터셋까지** | `rdfp` 패키지의 `rdfp_panda_mock.launch.py` 등 |
| 이 스택을 띄운 뒤 수집 계층만 얹기 | `ros2 launch rdfp rdfp_collect.launch.py` (아래) |

### 수집 계층을 나중에 얹기

이 패키지의 launch 로 로봇을 띄워 둔 상태에서 `rdfp` 패키지의
`rdfp_collect.launch.py` 를 따로 띄우면 **묶음 launch 와 같은 노드 그래프**가 된다.
제어 스택을 재시작하지 않고 수집 계층만 껐다 켤 수 있다.

```bash
ros2 launch robot_control panda_mock.launch.py                              # 터미널 1
ros2 launch rdfp rdfp_collect.launch.py                                    # 터미널 2
# JGPC 스택이면
ros2 launch rdfp rdfp_collect.launch.py arm_cmd_source:=float64_multi_array
```

동등성은 실측으로 검증되어 있고, Gazebo 조합은 인자를 맞춰야 한다 —
[rdfp launch README §6.1](../../rdfp/launch/README.md) 참고.

Isaac 스택에 수집 계층까지 얹으려면 **묶음 launch** 를 쓴다 —
`ros2 launch rdfp rdfp_panda_isaac.launch.py`. 카메라 설정을 `isaac_scene.json` 에서
읽어 넘기므로 인자를 손으로 맞출 필요가 없다.

robot twin(REST) 을 함께 쓰려면 이 launch 위에 `ros2 run robot_twin robot_twin --config <파일>` 을 띄운다.
백엔드마다 설정 파일이 다르다 — mock 은 `robot_twin_panda01.yaml`, Isaac 은
`robot_twin_panda_isaac.yaml`(명령 채널이 다르다). 세션/에피소드 연산은
`rdfp` 패키지가 설치돼 있어야 동작한다 — [robot_twin/README.md](../../robot_twin/README.md).

---

## 7. 유지보수 메모

- **helper 는 `sys.path` 조작 없이 정규 import 로 가져온다.** 예전에는 launch 파일들이 `sys.path.insert(0, os.path.dirname(__file__))` 로 같은 디렉터리의 sibling helper 를 끌어다 썼는데, 그 방식은 한 디렉터리 안에서만 동작해서 launch 가 두 패키지로 갈라지면 상위 패키지가 하위 helper 를 가져올 수 없다. helper 를 설치 모듈로 승격해 해결했다 — **이 패턴을 되살리지 않는다.**
- **이 패키지는 `rdfp` 를 import 하지 않는다.** `Node(package=...)` 에 `rdfp` 를 적는 것도 안 된다 — 제어 계층만 설치한 환경에서 실행파일을 못 찾는다. 경계는 `robot_control/tests/test_layer_boundary.py` 가 강제한다.
- Panda 공통 설정(MoveIt config / static_tf / rsp / move_group / servo / rviz)을 수정할 때는 먼저 `launch_helpers/common.py` 를 검토한다.
- controller 기동 순서를 바꿀 때는 `launch_helpers/controller_startup.py` 를 먼저 수정한다.
- **카메라 argument 를 고칠 때는 `launch_helpers/image_pipeline.py` 하나만 본다.**
  여섯 launch 가 이 모듈을 (직접 또는 `camera.py` 경유로) 쓰므로 한 곳을 고치면 전부 반영된다. 어느 한 launch 에서만 기본값을 바꾸려 하면 값이 다시 갈라지므로, 그 launch 에 `config_file` 을 주는 쪽을 택한다.
- **`--show-args` 는 설치본을 읽는다.** 인자를 고친 뒤 `colcon build` 를 빠뜨리면
  낡은 이름이 그대로 나오고, 그 출력을 믿고 문서를 쓰면 문서까지 틀린다. 실제로 이
  문서를 검증하다 `enable_scene` 이 `enable_scene_node` 로 보인 적이 있다 — 설치본이
  반나절 낡아 있었다. 인자 목록을 옮길 때는 **빌드 직후에** 뽑는다.
- **토픽 이름을 새로 정하거나 바꿀 때는 [토픽 이름 규약](../../../docs/topic_naming_contract.md)
  을 먼저 본다.** 논리 채널마다 정규 이름이 하나씩 정해져 있고, 노드 코드에는 **절대
  경로를 쓰지 않는다** — 상대로 두어야 remap 도 (나중의) 네임스페이스도 통한다.
- launch 를 추가하면 §1 표와 §5 의존 관계 표를 함께 갱신한다.
