# launch 디렉터리 안내

이 디렉터리는 크게 두 종류의 파일로 구성된다.

- **launch 파일**: 실제 `ros2 launch ...` 진입점 (`*.launch.py`)
- **helper 파일**: 여러 launch 파일에서 재사용하는 argument 선언, 노드 생성, startup orchestration 유틸리티 (`*.py`)

현재 구조는 크게 두 계열로 나뉜다.

- **Panda + MoveIt2 계열**
  - `panda_mock.launch.py`
  - `panda_jgpc_mock.launch.py`
  - `rdfp_panda_mock.launch.py`
  - `rdfp_panda_jgpc_mock.launch.py`
  - `replay_panda_mock.launch.py`
  - `ros2_control.launch.py`
  - `moveit.launch.py`
  - `rviz2.launch.py`
- **세션/카메라 앱 계열**
  - `rdfp.launch.py`
  - `rdfp_advanced.launch.py`


## 1. Naming 규칙

- `*.launch.py`
  - 실제 `ros2 launch ...` 진입점 — 사용자가 직접 실행하는 파일.
- `launch_helper.py` (**패키지 기반 공통**)
  - MoveIt config / static_tf / robot_state_publisher / move_group / servo / rviz 등
    Panda + MoveIt2 계열이 공통으로 쓰는 설정·노드 생성 helper.
- `<기능>_launch_helper.py` (**기능별 helper**)
  - launch argument 선언과 관련 `Node` 생성을 묶은 얇은 helper 모듈.
  - 예: `camera_launch_helper.py`, `ee_pose_launch_helper.py`, `gripper_launch_helper.py`.
- `controller_launch_helper.py` / `controller_startup_launch_helper.py`
  - ros2_control 런타임과 controller 순차 기동 event handler 전용 helper.


## 2. 파일 목록

### 2.1 Launch 파일

- `panda_mock.launch.py`
  - Panda mock 환경용 **기본** 메인 launch.
  - `static_tf` + `robot_state_publisher` + `ros2_control` + controller spawner 3종 기동 후,
    `move_group`, `servo_node`, `rviz2`, `camera_node`, `ee_pose_node`,
    `gripper_control_node` 를 순차 spawn 한다.
- `panda_jgpc_mock.launch.py`
  - `panda_mock.launch.py` 와 노드 구성은 같고, arm 컨트롤러 **타입만**
    `joint_trajectory_controller/JointTrajectoryController` →
    `position_controllers/JointGroupPositionController` (JGPC) 로 교체한 variant.
  - controller 설정은 `config/panda_jgpc_ros2_controllers.yaml` 을 사용한다
    (controller *이름* 은 `panda_arm_controller` 로 동일 → spawner helper 재사용).
  - arm 명령 인터페이스가 `/panda_arm_controller/joint_trajectory`
    (JointTrajectory) → `/panda_arm_controller/commands`
    (std_msgs/Float64MultiArray) 로 바뀌며, servo 파라미터도 launch 안에서
    그에 맞게 override 한다.
  - **주의**: JGPC 는 `FollowJointTrajectory` 액션을 제공하지 않으므로 arm 의
    `move_group` plan & execute 는 동작하지 않는다 (planning / IK / RViz 표시는
    정상). arm 제어는 servo, commands 토픽 직접 발행, 또는
    `create_move_group_client()` 가 돌려주는 `MoveGroupJgpcClient` 의
    `move_to_named_target()` (= `move_to_named_target_streamed()`) 로 한다 —
    `docs/moveit/MoveGroupJgpcClient_UserGuide.md` 참고.
- `rdfp_panda_mock.launch.py`
  - `panda_mock.launch.py` 의 모든 스택에 더해, rdfp 애플리케이션 노드
    (`session_control_node`, `rdfp_image_viewer_node`, `image_recorder_node`,
    `target_joint_cmds_publisher`) 를 함께 기동한다.
  - YAML 설정 (`config/panda_robot.yaml`) 으로 각 노드의 인자를 일괄
    override 할 수 있다. CLI `config_file:=<path>` 로 다른 YAML 을 지정 가능.
- `rdfp_panda_jgpc_mock.launch.py`
  - `rdfp_panda_mock.launch.py` 의 **JGPC 판**. arm 컨트롤러 타입만
    `position_controllers/JointGroupPositionController` 로 교체하고 rdfp
    애플리케이션 노드는 그대로 띄운다.
  - `panda_jgpc_mock.launch.py` + rdfp 애플리케이션 = 본 파일 이라고 보면 된다.
  - YAML 은 `config/panda_robot.yaml` 을 **그대로 공유**한다.
  - arm plan & execute 제약은 `panda_jgpc_mock.launch.py` 와 동일하다.
- `replay_panda_mock.launch.py`
  - 데이터셋 **재생 전용** variant. 재생 도구가 공급하는 토픽을 그대로 쓰기
    위해 `camera`, `ee_pose`, `session_control`, `image_recorder`,
    `target_joint_cmds_publisher` 는 기동하지 않는다.
    (`joint_state_broadcaster` 는 재생 결과 관측값이므로 **유지**한다.)
  - 설정은 **전용** `config/replay_panda_mock.yaml` (13키). 기동하지 않는 노드의
    키는 아예 없다 — 공유 YAML 을 쓰던 시절 조용히 무시되던 13개 argument 를
    정리했다.
  - `gripper_control_node` 는 유지되어 재생된 gripper 명령을 action
    으로 전달한다.
  - arm 구동 노드는 `replay_arm_path` argument 로 **배타 선택**한다. 두 경로가
    동시에 `/panda_arm_controller/joint_trajectory` 를 쓰면 명령이 충돌한다.

    | 값 | 기동 노드 | 배선 |
    |---|---|---|
    | `target_joint_cmds` | `target_joint_cmds_executor` | `/target_joint_cmds` (JointState) → JointTrajectory(길이 1) → `/panda_arm_controller/joint_trajectory` |
    | `ee_twist` (기본) | `ee_twist_publisher` + `servo_auto_start` | `/ee_pose` → 유한 차분 → `/servo_node/delta_twist_cmds` → servo → JTC |
    | `none` | — | `MoveGroupClient.follow_trajectory` 등 외부 클라이언트로 구동 |

  - `target_joint_cmds` 는 위치 폐루프라 드리프트가 없고 여유자유도까지
    재현되지만, `ee_twist` 는 속도 개루프라 오차가 누적된다. 관련 인자:
    `ee_twist_source_topic`, `ee_twist_output_topic`, `ee_twist_linear_gain`
    (기본 2.5 = 1/scale.linear), `ee_twist_angular_gain` (기본 1.25),
    `ee_twist_max_dt`, `servo_start_timeout`.
  - 전체 사용법·주의점은
    [docs/replay/replay_mock_stack_guide.md](../../../docs/replay/replay_mock_stack_guide.md)
    참고.
- `ros2_control.launch.py`
  - Panda 제어 런타임만 실행 (static_tf, robot_state_publisher,
    ros2_control_node, controller spawner 3종).
- `moveit.launch.py`
  - MoveIt 계층만 실행 (`move_group`, `servo_node`, `ee_pose_node`).
    ros2_control 이 별도로 올라와 있다는 가정.
- `rviz2.launch.py`
  - RViz2 만 실행.
- `rdfp.launch.py`
  - `session_control_node`, `image_viewer_node`, `image_recorder_node` 를 함께
    기동. 카메라 자체는 별도로 띄운다 (argument 만 `camera_launch_helper`
    에서 재사용).
- `rdfp_advanced.launch.py`
  - `session_control_node`, `rdfp_camera_node`, `rdfp_image_viewer_node` 를
    기동. `RdfpCameraNode` 는 세션 상태(`/session`) 가 ``IN_EPISODE`` 인 구간
    에서만 이미지를 발행한다.
- `teleop_mirror.launch.py`
  - leader-follower 동작 미러링 체인 2종 기동: `teleop_retarget` (클러치 기반
    retargeting), `ee_twist_node` (목표 pose → `/servo_node/delta_twist_cmds`).
  - 모든 argument 기본값은 **`config/teleop_mirror.yaml`** 에서 온다
    (`rdfp_panda_mock` 과 같은 방식). CLI `arg:=value` 로 개별 덮어쓰기,
    `config_file:=<path>` 로 YAML 교체.
  - **leader pose (`leader_pose_topic`, 기본 `/leader/ee_pose`) 는 외부 어댑터가
    공급한다는 전제** — 본 런치는 그 토픽의 발행자를 만들지 않는다. OMY-L100 은
    별도 저장소 `omy_leader_bridge` 가 담당한다. 계약:
    `docs/teleop/external_input_adapters.md`.
    leader `/tf` 를 직접 조회해야 하면 `ros2 run rdfp ee_pose_node` 를 따로 띄운다.
  - follower Panda + MoveIt2 스택이 **이미 실행 중** 이라는 전제 (servo 포함).
  - **USB 풋페달**로 클러치를 잡으려면 `enable_clutch_pedal:=true` 를 준다
    (`pedal_device_name` / `pedal_key_code` / `pedal_mode` / `pedal_grab` /
    `pedal_device_path` 로 장치를 지정). 기본 `false` 인 이유는 `python3-evdev`
    와 실제 장치가 없으면 노드가 기동에 실패하기 때문이다.
  - 데드맨은 **페달 노드 + `pedal_timeout`(하트비트 감시)** 두 조각이 함께 있어야
    성립하므로, 런치가 정합을 자동으로 맞춘다 — `hold` 모드인데 `pedal_timeout`
    이 0 이면 0.3 을 채우고, `toggle` 모드면 0 으로 되돌린다(하트비트를 안 보내
    므로 켜면 engage 즉시 풀린다). `docs/teleop/clutch_pedal_guide.md` 참고.
  - 설계: `docs/teleop/leader_follower_mirroring_design.md`,
    운영 절차: `docs/teleop/omy_leader_teleop_guide.md`,
    노드 상세: `docs/teleop/teleop_retarget_node_guide.md`,
    풋페달: `docs/teleop/clutch_pedal_guide.md`. servo 가
    `command_in_type: unitless` 이므로 twist 게인 기본값이 2.5 / 1.25 다.

### 2.2 Helper 파일

- `launch_helper.py`
  - Panda + MoveIt2 계열 launch 의 공통 helper 모듈.
  - `declare_ros2_control_hardware_type_argument`, `declare_log_level_argument`,
    `build_moveit_config`, `build_servo_params`, `create_static_tf_node`,
    `create_robot_state_publisher`, `create_move_group_node`,
    `create_servo_node`, `create_rviz_node` 제공.
- `controller_launch_helper.py`
  - `ros2_control_node` 및 controller spawner 3종
    (`joint_state_broadcaster`, `panda_arm_controller`, `panda_hand_controller`)
    생성 helper.
- `controller_startup_launch_helper.py`
  - controller 순차 기동 event handler (`create_controller_startup_handlers`)
    helper.
- `camera_launch_helper.py`
  - 카메라 관련 launch argument 선언 및 `camera_node` 생성 helper.
- `image_pipeline_launch_helper.py`
  - **`config/image_pipeline.yaml` 로더 + camera / image_viewer / image_recorder
    argument 선언 helper.** 카메라를 띄우는 launch 가 여섯이라 기본값을 한 곳에
    모은 모듈이다.
  - `load_config(path=None)` / `declare_camera_arguments(config)` /
    `declare_image_viewer_arguments(config)` /
    `declare_image_recorder_arguments(config, include_auto_start=)` /
    `declare_image_pipeline_arguments(config, ...)` /
    `declare_config_file_argument(name)`.
  - `camera_launch_helper.declare_camera_arguments()` 는 이 모듈을 기본 YAML 로
    호출하는 얇은 래퍼다 — 그래서 helper 만 쓰는 launch 도 코드 변경 없이 YAML
    값을 따라간다.
- `ee_pose_launch_helper.py`
  - EE pose publisher 관련 launch argument 선언 및 `ee_pose_node` 생성 helper.
- `gripper_launch_helper.py`
  - `gripper_control_node` 생성 helper (argument 없이 노드 생성만 제공).
- `scene_launch_helper.py`
  - 씬 상태 노드 argument 선언(`declare_scene_arguments`) + **백엔드별** 노드
    생성 helper. 현재는 `create_mock_scene_node()` 하나이며, Gazebo/Isaac
    백엔드는 형제 팩토리를 추가한다 (executable 만 다르고 argument 계약은 공유).
  - `base_frame` 은 이 helper 가 선언하지 않고 `ee_pose_launch_helper` 쪽
    선언을 **재사용**한다 — 씬 물체 pose 와 EE pose 의 기준 프레임이 갈라지면
    조용히 틀린 좌표가 나가기 때문. `camera_launch_helper` 가
    `image_pipeline_launch_helper` 의 argument 를 참조하는 것과 같은 구조다.


## 3. Launch × 설정 파일 한눈에

각 launch 가 읽는 설정 파일 전부. **교체 가능** 열이 비어 있으면 경로가 코드에
상수로 박혀 있어 `arg:=value` 로 바꿀 수 없다.

| launch | rdfp 패키지 설정 파일 | 교체 argument |
|---|---|---|
| `panda_mock` | `image_pipeline.yaml` | — (기본값 원천) |
| | `panda.rviz` | — |
| `panda_jgpc_mock` | `image_pipeline.yaml` | — (기본값 원천) |
| | **`panda_jgpc_ros2_controllers.yaml`** | — |
| | `panda.rviz` | — |
| `rdfp_panda_mock` | `panda_robot.yaml` | `config_file` |
| | `image_pipeline.yaml` | `image_pipeline_config_file` |
| | `panda.rviz` | — |
| `rdfp_panda_jgpc_mock` | `panda_robot.yaml` | `config_file` |
| | `image_pipeline.yaml` | `image_pipeline_config_file` |
| | **`panda_jgpc_ros2_controllers.yaml`** | — |
| | `panda.rviz` | — |
| `replay_panda_mock` | `replay_panda_mock.yaml` | `config_file` |
| | `panda.rviz` | — |
| `rdfp` / `rdfp_advanced` | `image_pipeline.yaml` | `config_file` |
| `teleop_mirror` | `teleop_mirror.yaml` | `config_file` |
| `panda_gazebo` / `rdfp_panda_gazebo` | `gazebo_ros2_controllers.yaml` | — |
| | `panda.rviz` | — |
| `ros2_control` / `moveit` / `rviz2` | (rdfp 설정 파일 없음) | — |

launch 이름이 비어 있는 행은 **바로 위 launch 에 속한다**.

### 3.1 파일이 세 층위로 나뉜다

| 층위 | 읽는 주체 | 형식 | 파일 |
|---|---|---|---|
| ① argument 기본값 | launch (`yaml.safe_load`) | 자유 | `rdfp_panda_mock` / `image_pipeline` / `replay_panda_mock` / `teleop_mirror` `.yaml` |
| ② 노드에 경로째 전달 | 해당 노드 프로세스 | **ROS 파라미터 규격** (`<노드>: ros__parameters:`) | `panda_jgpc_ros2_controllers.yaml`, `gazebo_ros2_controllers.yaml`, `panda.rviz` |
| ③ 외부 패키지 | MoveIt / ros2_control | 각 패키지 규약 | `moveit_resources_panda_moveit_config` 의 6종 (아래) |

①만 `config_file:=` 류로 교체할 수 있다. ②는 사용자가 튜닝할 값이 아니라
**코드와 짝이 맞아야 하는 정의**라서 상수로 둔다 — 예컨대
`panda_jgpc_ros2_controllers.yaml` 의 `joints` 순서는
`/panda_arm_controller/commands` 배열 순서를 정의하므로, 바꾸면 에러 없이 엉뚱한
관절이 움직인다.

③은 `build_moveit_config()` 가 `moveit_resources_panda_moveit_config` 에서
가져온다 — `panda.urdf.xacro`, `panda.srdf`, `kinematics.yaml`,
`joint_limits.yaml`, `gripper_moveit_controllers.yaml`,
`panda_simulated_config.yaml`(servo). 마지막 것의
`command_in_type: unitless`(scale.linear=0.4 / rotational=0.8)가 twist 게인
기본값 `2.5 / 1.25` 의 근거다. JGPC 계열은 이 servo 파라미터를 로드한 뒤 코드에서
`command_out_type` / `command_out_topic` / `publish_joint_velocities` 를 덮어쓴다.

### 3.2 `panda_jgpc_ros2_controllers.yaml` 은 왜 별도 파일인가

JGPC 계열 두 launch 만 쓰지만 `panda_robot.yaml` 과 합치지 않는다.

- `panda_robot.yaml` 의 6키는 **JTC 판과 완전히 동일**하다. JGPC 전용 파일로
  합치면 그 6키가 복제되어, 한쪽만 고쳤을 때 스택에 따라 다르게 동작한다.
- 층위 ①과 ②는 스키마가 다르다. 한 파일에 `ros__parameters` 블록과 자유 형식
  블록이 섞이면 어느 쪽이 어느 규격인지 구분되지 않는다.
- 합치면 `config_file:=` 이 컨트롤러 정의까지 갈아끼우게 된다. `log_level` 하나
  바꾸려 파일을 복사한 사용자가 컨트롤러 설정까지 떠안고, 업스트림 수정을
  따라가지 못한다.
- `panda_jgpc_mock` 은 애초에 `panda_robot.yaml` 을 읽지 않는다 (`config_file`
  argument 자체가 없다).

---

## 4. Launch 인자

> ⚠️ **`--show-args` 가 YAML 계열에서는 쓸모없다.** `rdfp_panda_mock`,
> `rdfp_panda_jgpc_mock`, `replay_panda_mock`, `teleop_mirror` 는 `config_file` 이
> `OpaqueFunction` 안에서 resolve 된 **뒤에야** 나머지 argument 를 선언하므로,
> 출력에 20여 개 중 1~2 개만 나온다. 이 절의 표가 그 대체다.

`--show-args` 로 실제 노출되는 인자 수 (측정값):

| launch | 노출 | 실제 |
|---|:-:|:-:|
| `panda_mock` / `panda_jgpc_mock` | 14 | 14 ✅ |
| `rdfp` | **1** (`config_file`) | 15 ❌ |
| `rdfp_advanced` | **1** (`config_file`) | 14 ❌ |
| `rdfp_panda_mock` / `rdfp_panda_jgpc_mock` | **2** (`config_file`, `image_pipeline_config_file`) | 20~21 ❌ |
| `teleop_mirror` | **1** (`config_file`) | 23 ❌ |
| `replay_panda_mock` | **1** (`config_file`) | 13 ❌ |

### 4.1 YAML 설정 파일을 쓰는 launch

설정 파일은 **관심사별**로 나뉜다. 로봇 설정과 이미지 파이프라인 설정은 바뀌는
주기도, 쓰는 launch 도 다르다.

| 설정 파일 | 키 수 | 읽는 launch | 지정 argument |
|---|:-:|---|---|
| `config/image_pipeline.yaml` | 14 | `rdfp_panda_mock`, `rdfp_panda_jgpc_mock` | `image_pipeline_config_file` |
| | | `rdfp`, `rdfp_advanced` | `config_file` |
| | | `panda_mock`, `panda_jgpc_mock` | (지정 불가 — 기본값 원천으로만) |
| `config/panda_robot.yaml` | 6 | `rdfp_panda_mock`, `rdfp_panda_jgpc_mock` | `config_file` |
| `config/replay_panda_mock.yaml` | 13 | `replay_panda_mock` | `config_file` |
| `config/teleop_mirror.yaml` | 23 | `teleop_mirror` | `config_file` |

> **`image_pipeline.yaml` 이 카메라 설정의 단일 출처다.** 예전에는 앱 계열
> (`rdfp`, `rdfp_advanced`) 이 `camera_launch_helper` 의 하드코딩 기본값
> (`640x480`, `id: 4`) 을, panda 계열이 YAML (`1280x720`, mp4 경로) 을 써서 같은
> 카메라인데 값이 갈라져 있었다. 지금은 여섯 launch 가 같은 파일을 본다.

> `replay_panda_mock` 은 원래 `panda_robot.yaml` 을 공유했다. 그런데 이 런치는
> camera / ee_pose_publisher / image_recorder / session_control 을 기동하지 않아
> **선언된 27개 argument 중 13개에 소비자가 없었다** — `camera.id` 나
> `image_recorder.output_dir` 을 고쳐도 아무 일이 일어나지 않으면서 경고도 없었다.
> 전용 파일로 갈라 실제로 동작하는 키만 남겼고, 이제 **없는 블록이 곧 "그 노드를
> 안 띄운다"** 는 뜻이 된다.

우선순위는 세 계층 모두 동일하다.

```
CLI arg:=value  >  config_file 의 값  >  (없음 — YAML 키는 필수)
```

`config_file:=<path>` 로 YAML 자체를 갈아끼울 수 있다. YAML 키가 빠지면 launch 가
`KeyError` 로 즉시 실패한다 — 조용히 기본값으로 떨어지지 않는다.

### 4.2 `image_pipeline.yaml` — YAML ↔ 인자 대응

camera → image_viewer / image_recorder 파이프라인 설정. **여섯 launch 의 공통
출처**다. 세 블록을 한 파일에 둔 이유는 값을 서로 공유하기 때문이다.

```
camera.image_topic  ─▶ image_viewer / image_recorder 의 `image` remap 대상
camera.resolution   ─▶ image_recorder 의 resolution (별도 키가 없다)
camera.fps          ◀▶ image_recorder.fps  (어긋나면 프레임 drop)
```

| YAML 블록 | 키 | launch argument | 현재 YAML 값 | 비고 |
|---|---|---|---|---|
| `camera` | `enabled` | `enable_camera_node` | `true` | |
| | `id` | `camera_id` | `…/data/shibuya_7_8.mp4` | 장치 인덱스 또는 파일/URI |
| | `image_topic` | `camera_image_topic` | `/camera/image_raw` | 뷰어·레코더도 같이 remap 된다 |
| | `info_topic` | `camera_info_topic` | `/camera/camera_info` | |
| | `status_topic` | `camera_status_topic` | `/camera/camera_status` | |
| | `fps` | `camera_fps` | `10` | |
| | `resolution` | `camera_resolution` | `1280x720` | **`image_recorder` 가 그대로 쓴다** |
| | `frame_id` | `camera_frame_id` | `camera_link` | |
| | `compress_image` | `camera_compress_image` | `false` | `rdfp_camera_node` 는 미지원 |
| `image_viewer` | `enabled` | `enable_image_viewer_node` | `true` | 헤드리스면 `false` |
| `image_recorder` | `enabled` | `enable_image_recorder_node` | `true` | |
| | `fps` | `image_recorder_fps` | `10` | **`camera.fps` 와 맞춘다** |
| | `output_dir` | `image_recorder_output_dir` | `/tmp/recordings` | |
| | `auto_start` | `image_recorder_auto_start` | `true` | **`rdfp_advanced` 는 선언하지 않는다** |

`rdfp_advanced` 의 `RdfpImageRecorderNode` 는 `/session` 상태로 녹화를 시작하므로
`auto_start` 파라미터를 받지 않는다. 그래서 그 launch 만
`declare_image_pipeline_arguments(config, include_auto_start=False)` 로 선언에서
뺀다 — 소비자 없는 argument 를 만들지 않기 위해서다.

### 4.3 로봇 계열 공통 인자 (helper 선언)

| 인자 | helper 기본값 | 선언 위치 | 설명 |
|---|---|---|---|
| `ros2_control_hardware_type` | `mock_components` | `launch_helper` | ros2_control 하드웨어 인터페이스 |
| `log_level` | `info` | `launch_helper` | `move_group` / `servo` / `session_control` 로그 레벨. `debug`\|`info`\|`warn`\|`error`\|`fatal` |
| `base_frame` | `panda_link0` | `ee_pose_launch_helper` | EE pose TF lookup 기준 프레임 |
| `ee_frame` | `panda_hand` | `ee_pose_launch_helper` | EE pose TF lookup 대상 프레임 |
| `publish_rate` | `50.0` | `ee_pose_launch_helper` | `/ee_pose` 발행 Hz |
| `enable_scene_node` | `true` | `scene_launch_helper` | 백엔드 씬 상태 노드 기동 여부 |
| `scene_publish_rate` | `2.0` | `scene_launch_helper` | `/scene/objects` 발행 Hz. 이름이 `publish_rate` 가 아닌 이유는 EE pose 인자와 충돌하기 때문 |

`panda_mock` / `panda_jgpc_mock` 은 위 7개 + `camera_launch_helper` 의 카메라 9개
= 16개를 쓴다. 카메라 쪽 기본값은 helper 가 하드코딩하지 않고
`image_pipeline.yaml` 을 읽어 온다 — 그래서 `--show-args` 가 그대로 동작하면서도
값은 다른 launch 와 일치한다. 다만 이 두 launch 에는 설정 파일을 바꿔 끼울
argument 가 없다 (기본 YAML 고정).

씬 인자 둘은 `rdfp_panda_mock` / `rdfp_panda_jgpc_mock` **에서도 그대로 쓴다** —
같은 helper 를 부르기 때문이다. 4.4 의 표에도 다시 실어 두었으니, 그 launch 를
쓸 때는 4.4 만 봐도 된다.

### 4.4 `rdfp_panda_mock` / `rdfp_panda_jgpc_mock` — YAML ↔ 인자 대응

두 launch 는 **YAML 을 둘** 읽는다 — 로봇 설정과 이미지 파이프라인 설정.
YAML 값이 argument 의 기본값이 되고, CLI `arg:=value` 가 그보다 우선한다.
여기에 **YAML 을 거치지 않는 씬 인자 둘**이 더 있다 (아래 세 번째 표).

**이 절이 두 launch 의 인자 전체 목록이다.** 두 launch 는 `config_file` 을
resolve 한 뒤 `OpaqueFunction` 안에서 나머지를 선언하므로, `--show-args` 는
`config_file` 과 `image_pipeline_config_file` **둘만** 출력한다.

```bash
ros2 launch rdfp rdfp_panda_mock.launch.py \
    config_file:=$HOME/my_robot.yaml \
    image_pipeline_config_file:=$HOME/my_camera.yaml
```

**로봇 설정** — `config/panda_robot.yaml` (두 launch 공유)

| YAML 블록 | 키 | launch argument | 현재 YAML 값 | 비고 |
|---|---|---|---|---|
| `ros2_control` | `hardware_type` | `ros2_control_hardware_type` | `mock_components` | |
| *(최상위)* | `log_level` | `log_level` | `info` | 블록이 아닌 스칼라 |
| `ee_pose` | `base_frame` | `base_frame` | `panda_link0` | |
| | `ee_frame` | `ee_frame` | `panda_hand` | |
| | `publish_rate` | `publish_rate` | `50.0` | teleop 이 쓰는 `/ee_pose` 주기 |
| `target_joint_cmds` | `input_topic` | `target_joint_cmds_input_topic` | `/panda_arm_controller/joint_trajectory` | **`rdfp_panda_mock` 전용** (아래) |
| *(YAML 밖)* | — | `config_file` | `<rdfp share>/config/panda_robot.yaml` | 다른 YAML 지정 |

블록 이름이 비어 있는 행은 **바로 위 블록에 속한다** (markdown 은 셀 병합이 없다).

**이미지 파이프라인** — `config/image_pipeline.yaml` 의 14개 (4.2 참고)

| | launch argument | 비고 |
|---|---|---|
| *(YAML 밖)* | `image_pipeline_config_file` | 기본 `<rdfp share>/config/image_pipeline.yaml` |

**씬** — YAML 없음. `scene_launch_helper` 의 하드코딩 기본값을 쓴다.

| launch argument | 기본값 | 비고 |
|---|---|---|
| `enable_scene_node` | `true` | 백엔드 씬 상태 노드(mock 은 `mock_scene_state`) 기동 여부. 끄면 `/scene/objects` 가 없고 트윈의 `reset_scene` 이 결과 대기 timeout 으로 실패한다 |
| `scene_publish_rate` | `2.0` | `/scene/objects` 발행 Hz. `publish_rate` 라는 이름을 못 쓰는 이유는 EE pose 인자와 충돌해서다 |

씬만 YAML 밖에 있는 이유는 노브가 둘뿐이고 스택마다 달라질 값이 아니어서다.
`panda_robot.yaml` 에 블록을 새로 넣으면 이 YAML 을 복사해 쓰는 **외부 설정
파일들이 `KeyError` 로 깨진다** (launch 는 없는 키를 조용히 넘기지 않는다).

명명 규칙이 블록마다 다르다 — `ee_pose` 는 키 이름이 그대로 argument 가 되고
(`base_frame` → `base_frame`), `camera` / `image_recorder` 는 블록 이름이 접두사로
붙으며 (`camera.fps` → `camera_fps`), `enabled` 만은 `enable_<노드>_node` 로 바뀐다.

**두 launch 의 유일한 인자 차이가 `target_joint_cmds_input_topic` 이다.** JGPC 판은
`target_joint_cmds_publisher` 를 `source=float64_multi_array` 로 고정하고 입력을
`/panda_arm_controller/commands` 로 하드코딩하므로 이 인자를 선언하지 않는다.

teleop 만 시험할 때 로그를 줄이는 조합:

```bash
ros2 launch rdfp rdfp_panda_jgpc_mock.launch.py \
    enable_camera_node:=false \
    enable_image_viewer_node:=false \
    enable_image_recorder_node:=false
```

### 4.5 `replay_panda_mock` — YAML ↔ 인자 대응

설정 파일은 **전용** `config/replay_panda_mock.yaml` 이다. 13개가 전부다.

| YAML 블록 | 키 | launch argument | 현재 YAML 값 | 비고 |
|---|---|---|---|---|
| `ros2_control` | `hardware_type` | `ros2_control_hardware_type` | `mock_components` | URDF 생성에 쓰인다 |
| *(최상위)* | `log_level` | `log_level` | `info` | `move_group` / `servo` |
| `ee_pose` | `base_frame` | `base_frame` | `panda_link0` | `ee_pose_node` 는 안 띄우지만 `ee_twist` 가 프레임을 쓴다 |
| | `ee_frame` | `ee_frame` | `panda_hand` | |
| `image_viewer` | `enabled` | `enable_image_viewer_node` | `true` | |
| | `image_topic` | `camera_image_topic` | `/camera/image_raw` | 재생된 이미지 토픽. argument 이름은 호환 위해 유지 |
| `replay` | `arm_path` | `replay_arm_path` | `ee_twist` | arm 구동 경로 **배타 선택**. `ee_twist` \| `target_joint_cmds` \| `none` |
| `replay.ee_twist` | `source_topic` | `ee_twist_source_topic` | `/ee_pose` | 미분 대상 PoseStamped 입력 |
| | `output_topic` | `ee_twist_output_topic` | `/servo_node/delta_twist_cmds` | servo 가 소비하는 출력 |
| | `linear_gain` | `ee_twist_linear_gain` | `2.5` | `1/scale.linear`. servo 가 `command_in_type: unitless` 라서 필요 |
| | `angular_gain` | `ee_twist_angular_gain` | `1.25` | `1/scale.rotational` |
| | `max_dt` | `ee_twist_max_dt` | `1.0` | 이 값을 넘는 샘플 공백 뒤에는 속도 스파이크를 피해 한 주기 건너뛴다 |
| `replay` | `servo_start_timeout` | `servo_start_timeout` | `30.0` | `/servo_node/start_servo` 대기 시간 |

블록 이름이 비어 있는 행은 **바로 위 블록에 속한다** (markdown 은 셀 병합이 없다).

`replay.ee_twist.*` / `servo_start_timeout` 은 `arm_path: ee_twist` 일 때만 의미가
있다. 경로별 특성 비교와 함정은
[docs/replay/replay_mock_stack_guide.md](../../../docs/replay/replay_mock_stack_guide.md).

**없는 블록이 정보다.** `camera` / `image_recorder` / `session_control` 블록이 없는
것은 본 런치가 그 노드를 띄우지 않는다는 뜻이다 (재생 도구가 해당 토픽을 공급).
공유 YAML 시절에는 그 키들이 존재하면서 조용히 무시되었다.

`replay_arm_path` 는 `--show-args` 에서 사라졌지만, 값이 잘못되면 launch 가
`RuntimeError` 로 즉시 거부하므로 오타가 조용히 넘어가지는 않는다.

```
'replay_arm_path' must be one of ['target_joint_cmds', 'ee_twist', 'none'], got 'bogus'
```

### 4.6 `teleop_mirror` — YAML ↔ 인자 대응

설정 파일은 **전용** `config/teleop_mirror.yaml` 이다.
leader pose 발행자는 **본 launch 가 만들지 않는다** (외부 어댑터 담당).

| YAML 블록 | 키 | launch argument | 현재 YAML 값 | 비고 |
|---|---|---|---|---|
| `leader` | `pose_topic` | `leader_pose_topic` | `/leader/ee_pose` | 외부 어댑터가 발행한다 |
| `follower` | `pose_topic` | `follower_pose_topic` | `/ee_pose` | 팔로워 스택의 `ee_pose_publisher` |
| | `target_pose_topic` | `target_pose_topic` | `/follower/target_pose` | retarget 결과 |
| | `base_frame` | `follower_base_frame` | `panda_link0` | target pose / twist 의 `frame_id` |
| `retarget` | `position_scale` | `position_scale` | `1.0` | leader→follower 변위 배율 |
| | `align_roll` | `align_roll` | `0.0` | `R_align` (rad). leader base → follower base 자세 정렬 |
| | `align_pitch` | `align_pitch` | `0.0` | |
| | `align_yaw` | `align_yaw` | `0.0` | `pi` 면 x/y 가 뒤집혀 거울 매핑 |
| | `lpf_cutoff_hz` | `lpf_cutoff_hz` | `3.0` | 저역통과 차단주파수. `<=0` 이면 비활성 |
| | `watchdog_timeout` | `watchdog_timeout` | `0.5` | leader 스트림이 이만큼 끊기면 자동 disengage (초) |
| | `max_sample_jump` | `max_sample_jump` | `0.2` | 샘플 간 이동이 이보다 크면 자동 disengage (m) |
| | `workspace_min` | `workspace_min` | `[]` | 워크스페이스 박스 `[x, y, z]`. **min/max 둘 다** 채워야 적용 |
| | `workspace_max` | `workspace_max` | `[]` | |
| `clutch_pedal` | `enabled` | `enable_clutch_pedal` | `false` | `python3-evdev` + 실제 장치가 없으면 노드가 기동 실패 |
| | `device_path` | `pedal_device_path` | `''` | 비우면 이름 탐색. `/dev/input/by-id/...` 권장 |
| | `device_name` | `pedal_device_name` | `pedal` | 장치 이름 **부분 문자열** (대소문자 무시) |
| | `key_code` | `pedal_key_code` | `''` | 비우면 그 장치의 **아무 키나** 페달로 취급 |
| | `mode` | `pedal_mode` | `hold` | `hold` = 데드맨(권장) / `toggle` = 전환 |
| | `grab` | `pedal_grab` | `false` | 장치 독점(`EVIOCGRAB`) |
| | `timeout` | `pedal_timeout` | `0.0` | 하트비트 감시. **런치가 자동 보정한다** — 아래 |
| `twist` | `topic` | `twist_topic` | `/servo_node/delta_twist_cmds` | MoveIt Servo 입력 |
| | `linear_gain` | `twist_linear_gain` | `2.5` | `1/scale.linear` (servo `command_in_type: unitless`) |
| | `angular_gain` | `twist_angular_gain` | `1.25` | `1/scale.rotational` |
| *(YAML 밖)* | — | `config_file` | `<rdfp share>/config/teleop_mirror.yaml` | 다른 YAML 지정 |

블록 이름이 비어 있는 행은 **바로 위 블록에 속한다** (markdown 은 셀 병합이 없다).

argument 이름은 YAML 키와 1:1 이 아니다 — `clutch_pedal.*` 는 `pedal_*` 접두사가
붙고 (`enabled` 만 `enable_clutch_pedal`), `follower.base_frame` 은
`follower_base_frame`, `twist.topic` 은 `twist_topic` 이 된다. 접두사가 생기는
이유는 `base_frame` / `topic` 같은 이름이 다른 launch 의 공통 인자와 충돌하기
때문이다.

`pedal_timeout` 은 launch 가 `enable_clutch_pedal` / `pedal_mode` 를 보고 값을
**자동으로 맞춘다.** 데드맨은 페달 노드와 하트비트 감시 **두 조각**이 함께 있어야
성립하는데, 한쪽만 켠 구성은 조용히 잘못 동작하기 때문이다.

| 입력 | 적용값 | 이유 |
|---|---|---|
| `enable_clutch_pedal:=true`, `hold`, 미지정 | **0.3** | 데드맨 아닌 채로 뜨는 것을 방지 |
| `enable_clutch_pedal:=true`, `hold`, `0.5` | 0.5 | 명시값 존중 |
| `pedal_mode:=toggle`, `0.3` | **0.0** | toggle 은 하트비트를 안 보내 engage 즉시 풀린다 |
| `enable_clutch_pedal:=false`, `0.3` | 0.3 | 페달 노드를 따로 띄우는 구성 |

실제 적용값은 `ros2 param get /teleop_retarget pedal_timeout` 으로 확인한다
(위 4행 모두 실측 확인됨). `pedal_mode` 에 `hold`/`toggle` 외의 값을 주면 정합
로직이 조용히 흘러 데드맨이 사라지므로, launch 가 `RuntimeError` 로 거부한다.

### 4.7 `rdfp` / `rdfp_advanced`

이미지 파이프라인 설정만 쓰므로 `config_file` 이 곧 `image_pipeline.yaml` 이다.

```bash
ros2 launch rdfp rdfp_advanced.launch.py config_file:=$HOME/my_camera.yaml
```

| 인자 | 출처 | `rdfp` | `rdfp_advanced` |
|---|---|:-:|:-:|
| `log_level` | 하드코딩 `info` | ✅ | ✅ |
| `camera_*` 9종 | `image_pipeline.yaml` | 토픽 remap 용 | 노드 파라미터로도 사용 |
| `enable_image_viewer_node` | `image_pipeline.yaml` | ✅ | ✅ |
| `enable_image_recorder_node` / `_fps` / `_output_dir` | `image_pipeline.yaml` | ✅ | ✅ |
| `image_recorder_auto_start` | `image_pipeline.yaml` | ✅ | **미선언** |
| `config_file` | — | ✅ | ✅ |

`rdfp.launch.py` 는 카메라 노드를 띄우지 않고 **argument 만** 재사용해 토픽을
remap 한다. `rdfp_advanced.launch.py` 는 `rdfp_camera_node` 를 직접 띄우므로
`camera_id` / `camera_fps` 등이 실제 노드 파라미터로 들어간다.

> ⚠️ `rdfp_camera_node` 는 현재 **엔트리포인트가 깨져 있다** —
> `setup.py` 에 등록돼 있으나 `rdfp/camera/rdfp_camera_node.py` 가 존재하지 않아
> `ModuleNotFoundError` 로 죽는다. 본 문서의 설정 변경과는 무관한 기존 문제이며,
> `rdfp_advanced` 의 나머지 노드는 정상 기동한다.

### 4.8 부분 스택 launch

helper 공통 인자의 부분집합만 노출한다. 값의 의미는 위 공통 인자 표와 같다.

| launch | 노출 인자 |
|---|---|
| `ros2_control.launch.py` | `ros2_control_hardware_type` |
| `moveit.launch.py` | `ros2_control_hardware_type`, `log_level`, `base_frame`, `ee_frame`, `publish_rate` |
| `rviz2.launch.py` | `ros2_control_hardware_type` |

`rviz2` / `ros2_control` 의 `ros2_control_hardware_type` 은 URDF 를 만드는 데 쓰이므로,
같이 띄우는 다른 스택과 **값을 맞춰야** 로봇 모델이 어긋나지 않는다.


## 5. Launch × Helper 의존 관계

`image_pipeline` 열의 ⭕ 는 `camera_launch_helper` 를 통한 **간접** 의존이다
(그 helper 가 YAML 로더를 감싼다).

| launch | launch_helper | controller_launch_helper | controller_startup_launch_helper | camera_launch_helper | image_pipeline_launch_helper | ee_pose_launch_helper | gripper_launch_helper | scene_launch_helper |
|---|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| `panda_mock.launch.py`            | ✅ | ✅ | ✅ | ✅ | ⭕ | ✅ | ✅ | ✅ |
| `panda_jgpc_mock.launch.py`       | ✅ | ✅ | ✅ | ✅ | ⭕ | ✅ | ✅ | ✅ |
| `rdfp_panda_mock.launch.py`       | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| `rdfp_panda_jgpc_mock.launch.py`  | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| `replay_panda_mock.launch.py`     | ✅ | ✅ | ✅ |    |    |    |    |    |
| `ros2_control.launch.py`          | ✅ | ✅ | ✅ |    |    |    |    |    |
| `moveit.launch.py`                | ✅ |    |    |    |    | ✅ |    |    |
| `rviz2.launch.py`                 | ✅ |    |    |    |    |    |    |    |
| `rdfp.launch.py`                  |    |    |    |    | ✅ |    |    |    |
| `rdfp_advanced.launch.py`         |    |    |    |    | ✅ |    |    |    |


## 6. Helper 중심으로 본 역참조

- `launch_helper.py`
  - `panda_mock.launch.py`, `panda_jgpc_mock.launch.py`,
    `rdfp_panda_mock.launch.py`, `rdfp_panda_jgpc_mock.launch.py`,
    `replay_panda_mock.launch.py`, `ros2_control.launch.py`,
    `moveit.launch.py`, `rviz2.launch.py`
- `controller_launch_helper.py`
  - `panda_mock.launch.py`, `panda_jgpc_mock.launch.py`,
    `rdfp_panda_mock.launch.py`, `rdfp_panda_jgpc_mock.launch.py`,
    `replay_panda_mock.launch.py`, `ros2_control.launch.py`
- `controller_startup_launch_helper.py`
  - `panda_mock.launch.py`, `panda_jgpc_mock.launch.py`,
    `rdfp_panda_mock.launch.py`, `rdfp_panda_jgpc_mock.launch.py`,
    `replay_panda_mock.launch.py`, `ros2_control.launch.py`
- `camera_launch_helper.py`
  - `panda_mock.launch.py`, `panda_jgpc_mock.launch.py`,
    `rdfp_panda_mock.launch.py`, `rdfp_panda_jgpc_mock.launch.py`,
    `rdfp.launch.py`, `rdfp_advanced.launch.py`
- `ee_pose_launch_helper.py`
  - `panda_mock.launch.py`, `panda_jgpc_mock.launch.py`,
    `rdfp_panda_mock.launch.py`, `rdfp_panda_jgpc_mock.launch.py`, `moveit.launch.py`
- `gripper_launch_helper.py`
  - `panda_mock.launch.py`, `panda_jgpc_mock.launch.py`,
    `rdfp_panda_mock.launch.py`, `rdfp_panda_jgpc_mock.launch.py`
- `scene_launch_helper.py`
  - `panda_mock.launch.py`, `panda_jgpc_mock.launch.py`,
    `rdfp_panda_mock.launch.py`, `rdfp_panda_jgpc_mock.launch.py`
  - `replay_panda_mock.launch.py` 는 **의도적으로 제외**한다 — 리플레이 중의
    물체 상태는 데이터셋에서 나와야 하고, 씬 노드를 띄우면 라이브 planning
    scene 을 별도로 발행해 두 출처가 섞인다.


## 7. Panda 계열 실행 구조

`panda_mock.launch.py` 는 논리적으로 분리된 하위 단위를 `include` 하지 않고
동일한 helper 를 재사용하여 **최종 orchestration** 을 수행한다.

```text
panda_mock.launch.py
 ├─ ros2_control.launch.py 와 같은 책임
 │   ├─ static_tf
 │   ├─ robot_state_publisher
 │   ├─ ros2_control_node
 │   └─ controller spawner chain
 ├─ moveit.launch.py 와 같은 책임
 │   ├─ move_group
 │   ├─ servo_node
 │   └─ ee_pose_node
 ├─ rviz2.launch.py 와 같은 책임
 │   └─ rviz2
 ├─ camera_launch_helper.py 와 같은 책임
 │   └─ camera_node
 └─ gripper_launch_helper.py 와 같은 책임
     └─ gripper_control_node
```

`rdfp_panda_mock.launch.py` 는 위 구성에 **rdfp 애플리케이션 묶음** 을 더한다.

```text
rdfp_panda_mock.launch.py
 ├─ panda_mock.launch.py 와 같은 스택
 └─ rdfp 애플리케이션 묶음
     ├─ session_control_node
     ├─ rdfp_image_viewer_node
     ├─ image_recorder_node
     └─ target_joint_cmds_publisher
```

`replay_panda_mock.launch.py` 는 재생 시 데이터셋이 공급하는 값들의 소스 노드
(camera / ee_pose / session_control / image_recorder / target_joint_cmds_publisher)
를 빼고, 재생된 토픽을 실제 컨트롤러/액션으로 흘려보내는 **어댑터 노드** 를 대신
포함한다. arm 어댑터는 `replay_arm_path` 로 하나만 고른다.

```text
replay_panda_mock.launch.py
 ├─ ros2_control 스택 (joint_state_broadcaster 포함 — 재생 결과 관측용)
 ├─ move_group / servo_node / rviz2
 ├─ gripper_control_node             (항상)
 ├─ rdfp_image_viewer_node            (enable_image_viewer_node)
 └─ arm 어댑터 — replay_arm_path 로 택일
     ├─ ee_twist_publisher + servo_auto_start (ee_twist, 기본)
     ├─ target_joint_cmds_executor            (target_joint_cmds)
     └─ (없음)                                 (none)
```


## 8. Startup orchestration

Panda 계열에서 controller 순차 기동 정책은
`controller_startup_launch_helper.py` 에 모여 있다. 순서는 다음과 같다.

```text
ros2_control_node start
  -> joint_state_broadcaster spawner
     -> panda_arm_controller spawner
        -> panda_hand_controller spawner
           -> post_hand_actions 실행
```

각 launch 의 `post_hand_actions` 구성:

- `ros2_control.launch.py`
  - (빈 리스트)
- `panda_mock.launch.py`
  - `move_group`, `servo_node`, `rviz2`, `camera_node`, `ee_pose_node`,
    `gripper_control_node`
- `panda_jgpc_mock.launch.py`
  - `panda_mock.launch.py` 와 동일 (컨트롤러 타입만 다름)
- `rdfp_panda_mock.launch.py`
  - 위 6 노드 + `session_control_node`, `rdfp_image_viewer_node`,
    `image_recorder_node`, `target_joint_cmds_publisher`
- `rdfp_panda_jgpc_mock.launch.py`
  - 위 6 노드 + `session_control_node`, `rdfp_image_viewer_node`,
    `image_recorder_node`
- `replay_panda_mock.launch.py`
  - `move_group`, `servo_node`, `rviz2`,
    `rdfp_image_viewer_node`, `gripper_control_node`,
    `target_joint_cmds_executor`


## 9. 파일 사용 가이드

- Panda mock 전체 스택만 띄우고 싶으면:
  - `panda_mock.launch.py`
- arm 을 JTC 대신 JointGroupPositionController 로 제어하고 싶으면:
  - `panda_jgpc_mock.launch.py` (단, move_group 의 arm plan & execute 는 불가)
- JGPC + rdfp 앱 노드까지 전부 띄우고 싶으면 (teleop / 세션 / 녹화 포함):
  - `rdfp_panda_jgpc_mock.launch.py`
- Panda mock + rdfp 앱 노드까지 전부 띄우고 싶으면:
  - `rdfp_panda_mock.launch.py`
- 데이터셋 재생 모드로 띄우고 싶으면:
  - `replay_panda_mock.launch.py`
- 제어 계층만 띄우고 싶으면:
  - `ros2_control.launch.py`
- MoveIt 계층만 띄우고 싶으면:
  - `moveit.launch.py`
- RViz 만 띄우고 싶으면:
  - `rviz2.launch.py`
- 세션 기반 카메라 / 뷰어 / 녹화 앱만 띄우고 싶으면:
  - `rdfp.launch.py` (기본 camera 와 함께 사용)
  - `rdfp_advanced.launch.py` (`RdfpCameraNode` 포함)


## 10. 유지보수 메모

- 이 디렉터리의 launch 파일들은 ROS2 launch runner 제약 때문에 공통적으로
  `sys.path.insert(0, os.path.dirname(__file__))` 패턴을 사용한다.
  같은 디렉터리의 helper 파일을 top-level 모듈처럼 import 하기 위한 것이다.
- Panda 계열 공통 설정 (MoveIt config / static_tf / robot_state_publisher
  /move_group/servo/rviz) 을 수정할 때는 먼저 `launch_helper.py` 를 검토한다.
- controller 기동 순서를 바꿀 때는 `controller_startup_launch_helper.py` 를
  먼저 수정한다.
- 카메라 argument 체계를 바꾸면 `camera_launch_helper.py` 를 사용하는 모든
  launch (`panda_mock.launch.py`, `rdfp_panda_mock.launch.py`,
  `rdfp.launch.py`, `rdfp_advanced.launch.py`) 에 영향이 간다.
- YAML 계열은 argument 기본값을 손대려면 YAML 을 먼저 확인한다. CLI 에서
  `config_file:=<path>` 또는 개별 `arg:=value` 로 덮어쓸 수 있다.

  | launch | YAML |
  |---|---|
  | `rdfp_panda_mock` / `rdfp_panda_jgpc_mock` | `config/panda_robot.yaml` + `config/image_pipeline.yaml` |
  | `rdfp` / `rdfp_advanced` | `config/image_pipeline.yaml` |
  | `panda_mock` / `panda_jgpc_mock` | `config/image_pipeline.yaml` (기본값 원천, 교체 불가) |
  | `replay_panda_mock` | `config/replay_panda_mock.yaml` |
  | `teleop_mirror` | `config/teleop_mirror.yaml` |

- **카메라 관련 argument 를 고칠 때는 `image_pipeline_launch_helper.py` 하나만
  본다.** 여섯 launch 가 이 모듈을 (직접 또는 `camera_launch_helper` 경유로)
  쓰므로 한 곳을 고치면 전부 반영된다. 반대로 어느 한 launch 에서만 기본값을
  바꾸려 하면 다시 값이 갈라지므로, 그 launch 에 `config_file` 을 주는 쪽을
  택한다.

- **argument 를 추가할 때 소비자가 있는지 먼저 확인한다.** `replay_panda_mock` 은
  `rdfp_panda_mock` 에서 선언을 통째로 복사해 온 탓에 13개가 아무도 읽지 않는
  상태로 오래 남아 있었다. 값을 고쳐도 아무 일이 없고 경고도 없어 발견이 늦는다.
  점검: 선언 이름을 `LaunchConfiguration("<이름>")` 으로 grep 해 launch 본문과
  해당 launch 가 호출하는 helper 양쪽에서 참조가 있는지 본다.
- **`teleop_mirror` / `replay_panda_mock` 은 값 조회 방식이 다르다.** 두 launch 는
  `OpaqueFunction` 안에서 값을 **즉시** 읽어야 한다 (전자는 double array 파라미터와
  페달 정합 로직, 후자는 `replay_arm_path` 로 기동 노드를 고르기 때문). 이때
  `LaunchConfiguration(...).perform(context)` 은 쓸 수 없다 — 같은 함수가 반환할
  선언이 아직 실행되지 않아 "launch configuration does not exist" 로 실패한다.
  대신 `_resolve()` / `_resolve_arm_path()` 가 `context.launch_configurations`
  (CLI 가 준 값) 를 조회하고 없으면 YAML 기본값을 쓴다. 새 argument 를 이 경로에
  추가할 때 `perform()` 을 쓰면 조용히 실패하므로 같은 헬퍼를 거쳐야 한다.
- **argument 를 추가·삭제·개명하면 "Launch 인자" 절의 표를 함께 고친다.** YAML
  계열은 `--show-args` 가 `config_file` 만 출력하므로 그 표가 사실상 유일한
  인자 목록이다. 기본값을 YAML 에서 바꿨을 때도 표의 "현재 YAML 값" 열을
  갱신한다.
