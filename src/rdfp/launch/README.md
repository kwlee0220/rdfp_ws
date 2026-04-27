# launch 디렉터리 안내

이 디렉터리는 크게 두 종류의 파일로 구성된다.

- **launch 파일**: 실제 `ros2 launch ...` 진입점 (`*.launch.py`)
- **helper 파일**: 여러 launch 파일에서 재사용하는 argument 선언, 노드 생성, startup orchestration 유틸리티 (`*.py`)

현재 구조는 크게 두 계열로 나뉜다.

- **Panda + MoveIt2 계열**
  - `panda_mock.launch.py`
  - `rdfp_panda_mock.launch.py`
  - `replay_panda_mock.launch.py`
  - `ros2_control.launch.py`
  - `moveit.launch.py`
  - `rviz2.launch.py`
- **세션/카메라 앱 계열**
  - `rdfp.launch.py`
  - `rdfp_advanced.launch.py`


## Naming 규칙

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


## 파일 목록

### Launch 파일

- `panda_mock.launch.py`
  - Panda mock 환경용 **기본** 메인 launch.
  - `static_tf` + `robot_state_publisher` + `ros2_control` + controller spawner 3종 기동 후,
    `move_group`, `servo_node`, `rviz2`, `camera_node`, `ee_pose_node`,
    `gripper_control_node` 를 순차 spawn 한다.
- `rdfp_panda_mock.launch.py`
  - `panda_mock.launch.py` 의 모든 스택에 더해, rdfp 애플리케이션 노드
    (`session_control_node`, `rdfp_image_viewer_node`, `image_recorder_node`,
    `target_joint_states_publisher`) 를 함께 기동한다.
  - YAML 설정 (`config/rdfp_panda_mock.yaml`) 으로 각 노드의 인자를 일괄
    override 할 수 있다. CLI `config_file:=<path>` 로 다른 YAML 을 지정 가능.
- `replay_panda_mock.launch.py`
  - 데이터셋 **재생 전용** variant. 재생 도구가 공급하는 토픽을 그대로 쓰기
    위해 `camera`, `ee_pose`, `session_control`, `image_recorder`,
    `target_joint_states_publisher` 는 기동하지 않는다.
  - 대신 `target_joint_states_executor` 를 띄워 재생 도구의
    `/target_joint_states` 를 `/panda_arm_controller/joint_trajectory` 로
    변환(JointTrajectory 길이 1 래핑) 하여 실제 arm 제어에 흘려 보낸다.
  - `gripper_command_subscriber` 는 유지되어 재생된 gripper 명령을 action
    으로 전달한다.
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

### Helper 파일

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
- `ee_pose_launch_helper.py`
  - EE pose publisher 관련 launch argument 선언 및 `ee_pose_node` 생성 helper.
- `gripper_launch_helper.py`
  - `gripper_control_node` 생성 helper (argument 없이 노드 생성만 제공).


## Launch × Helper 의존 관계

| launch | launch_helper | controller_launch_helper | controller_startup_launch_helper | camera_launch_helper | ee_pose_launch_helper | gripper_launch_helper |
|---|:-:|:-:|:-:|:-:|:-:|:-:|
| `panda_mock.launch.py`            | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| `rdfp_panda_mock.launch.py`       | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| `replay_panda_mock.launch.py`     | ✅ | ✅ | ✅ |    |    |    |
| `ros2_control.launch.py`          | ✅ | ✅ | ✅ |    |    |    |
| `moveit.launch.py`                | ✅ |    |    |    | ✅ |    |
| `rviz2.launch.py`                 | ✅ |    |    |    |    |    |
| `rdfp.launch.py`                  |    |    |    | ✅ |    |    |
| `rdfp_advanced.launch.py`         |    |    |    | ✅ |    |    |


## Helper 중심으로 본 역참조

- `launch_helper.py`
  - `panda_mock.launch.py`, `rdfp_panda_mock.launch.py`,
    `replay_panda_mock.launch.py`, `ros2_control.launch.py`,
    `moveit.launch.py`, `rviz2.launch.py`
- `controller_launch_helper.py`
  - `panda_mock.launch.py`, `rdfp_panda_mock.launch.py`,
    `replay_panda_mock.launch.py`, `ros2_control.launch.py`
- `controller_startup_launch_helper.py`
  - `panda_mock.launch.py`, `rdfp_panda_mock.launch.py`,
    `replay_panda_mock.launch.py`, `ros2_control.launch.py`
- `camera_launch_helper.py`
  - `panda_mock.launch.py`, `rdfp_panda_mock.launch.py`,
    `rdfp.launch.py`, `rdfp_advanced.launch.py`
- `ee_pose_launch_helper.py`
  - `panda_mock.launch.py`, `rdfp_panda_mock.launch.py`, `moveit.launch.py`
- `gripper_launch_helper.py`
  - `panda_mock.launch.py`, `rdfp_panda_mock.launch.py`


## Panda 계열 실행 구조

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
     └─ target_joint_states_publisher
```

`replay_panda_mock.launch.py` 는 재생 시 데이터셋이 공급하는 값들의 소스 노드
(camera / ee_pose / session_control / image_recorder / target_joint_states_publisher)
를 빼고, 재생된 토픽을 실제 컨트롤러/액션으로 흘려보내는 어댑터 노드
(`target_joint_states_executor`, `gripper_command_subscriber`) 를 대신 포함한다.

```text
replay_panda_mock.launch.py
 ├─ ros2_control 스택 (panda_mock 와 동일)
 ├─ move_group + servo_node + rviz2
 └─ rdfp 애플리케이션 (재생 어댑터)
     ├─ rdfp_image_viewer_node
     ├─ gripper_command_subscriber
     └─ target_joint_states_executor
```


## Startup orchestration

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
- `rdfp_panda_mock.launch.py`
  - 위 6 노드 + `session_control_node`, `rdfp_image_viewer_node`,
    `image_recorder_node`, `target_joint_states_publisher`
- `replay_panda_mock.launch.py`
  - `move_group`, `servo_node`, `rviz2`,
    `rdfp_image_viewer_node`, `gripper_command_subscriber`,
    `target_joint_states_executor`


## 파일 사용 가이드

- Panda mock 전체 스택만 띄우고 싶으면:
  - `panda_mock.launch.py`
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


## 유지보수 메모

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
- `rdfp_panda_mock.launch.py` / `replay_panda_mock.launch.py` 는 YAML 기본값
  (`config/rdfp_panda_mock.yaml`) 을 사용하므로 argument 기본값을 손대려면
  YAML 을 먼저 확인한다. CLI 에서 `config_file:=<path>` 또는 개별
  `arg:=value` 로 덮어쓸 수 있다.
