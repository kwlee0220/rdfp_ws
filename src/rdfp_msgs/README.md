# rdfp_msgs

`rdfp` 패키지가 사용하는 ROS 2 (Humble) 메시지/서비스 인터페이스 정의 패키지다.
`ament_cmake` + `rosidl_default_generators` 기반의 순수 IDL 패키지이며 런타임
코드는 포함하지 않는다.

`rdfp` 와 `robot_control` 의 노드 (`session_control_node`, `image_recorder_node`,
`GripperActionNode`, `target_joint_cmds_publisher` 등) 와 데이터셋 파이프라인이 본
패키지를 참조하므로 함께 빌드해야 한다.

## 빌드

워크스페이스 루트에서 `rdfp` 보다 먼저 빌드되도록 함께 지정한다.

```bash
cd ~/development/ros/rdfp_ws
colcon build --packages-select rdfp_msgs robot_control robot_twin rdfp
source install/setup.bash
```

`rdfp_msgs` 만 단독으로 다시 빌드하려면 다음과 같이 실행한다.

```bash
colcon build --packages-select rdfp_msgs
source install/setup.bash
```

## 인터페이스 목록

### 메시지 (`msg/`)

| 타입 | 사용처 | 요약 |
|---|---|---|
| `SessionCommand` | `session_control_node` 의 `session` 토픽 | 세션 상태 머신 (`IDLE` / `IN_SESSION` / `IN_EPISODE`) 와 `task_label` 을 발행. `header.stamp` 는 발행 시각. `outcome` / `metadata` 는 **에피소드 종료 전이에서만** 채워지며 (`stop_episode` 가 준 값), 그 외 전이에서는 `''` 다. 후처리 시 `sessions` 테이블의 `success` / `metadata` 가 된다. |
| `GripperCommand` | 호출자(teleop / robot twin / 재생) → `GripperNode` 의 `gripper_cmds` | **심볼만 싣는다** — `goal` 이 `'open'`/`'close'`/`'grasp'` 중 하나다. 숫자(목표 폭·파지력)는 그리퍼에 종속이라 `GripperNode` 의 `targets` 파라미터가 갖는다 (2026-09-01 결정). `header.stamp` 필수 — 비우면 epoch 0 에 적재된다. |
| `GripperState` | `GripperNode` → `gripper_states` (주기 발행) | 그리퍼의 **연속 상태**. `goal`(마지막 명령), `width`(**개구 폭 m — 관절값이 아니다**, 못 구하면 NaN), `stalled`(관측), `at_goal`(**판정** — 목표 위치 도달이 아니라 "시킨 일을 이뤘는가"). 소비자는 `at_goal` 하나만 보면 된다. |
| `TargetJointStates` | `target_joint_states_publisher` / `target_joint_states_executor` | 단일 `trajectory_msgs/JointTrajectoryPoint` 에 `Header` 를 부여한 timestamped joint setpoint. |
| `ClutchState` | `teleop_retarget` | teleop 클러치 상태(`engaged`) 와 해제 사유(`reason`). 상태 변경 시에만 발행하며 QoS 는 `TRANSIENT_LOCAL`. |
| `SceneObject` | `SceneObjects` 의 원소 | 물체 하나 — `name`, `type`(`'box'`/`'sphere'`/`'cylinder'`/`'mesh'` **문자열**), `dimensions`(종류마다 길이가 다르다), `pose`. `orientation` 은 **ROS xyzw** 이며 Isaac(wxyz)은 발행 노드가 변환해야 한다. |
| `SceneObjects` | 백엔드별 `scene_state_node` → `/scene/objects` | 한 시점의 scene 물체 전체. mock / Gazebo / Isaac 이 각자 이 타입으로 변환해 발행하므로 트윈·후처리기가 환경 구현을 모른다. `frame_id` 는 로봇 베이스(`panda_link0`) 고정, QoS 는 `TRANSIENT_LOCAL`. |

> 그리퍼 계열(`GripperCommand` / `GripperState`)의 필드 판정식·백엔드별 실현
> 가능성·데이터셋 사용법은 [docs/moveit/GripperNode_Design.md](../../docs/moveit/GripperNode_Design.md)
> 에 있다. **명령은 심볼, 관측은 물리량**이라는 비대칭이 의도적이다.

> scene 계열(`SceneObject` / `SceneObjects` 와 서비스 `ResetScene`)의
> 필드 계약·발행 노드·트윈 연산·함정은 [docs/scene/scene_objects_guide.md](../../docs/scene/scene_objects_guide.md)
> 에 모아 두었다.

### 서비스 (`srv/`)

| 타입 | 사용처 | 요약 |
|---|---|---|
| `StartSession` | `image_recorder_node` 의 `/image_recorder/start_session` | 요청 비어 있음 / 응답 `success`, `mp4_path`. 녹화 세션을 시작하고 생성된 MP4 절대 경로를 반환. |
| `StopSession` | `image_recorder_node` 의 `/image_recorder/stop_session` | 요청 비어 있음 / 응답 `success`, `mp4_path`. finalize 된 MP4 경로를 반환 (노드 내부 finalize timeout 5.0 초). |
| `GetSessionState` | `session_control_node` 의 `/session_control/get_session_state` | 요청 비어 있음 / 응답 `state`, `task_label`. 현재 상태와 라벨을 단발성 조회. |
| `SetString` | `session_control_node` 의 `/session_control/set_task_label` | 요청 `task_label` / 응답 `success`, `message`. 빈 문자열은 task clear, 유효하지 않은 상태 전이는 `success=false`, `message='invalid command'`. |
| `StopEpisode` | `session_control_node` 의 `/session_control/stop_episode` | 요청 `outcome`(`''`/`'success'`/`'failure'`), `metadata`(JSON object 문자열) / 응답 `success`, `message`. **응답의 `success` 는 명령 수용 여부이지 작업의 성패가 아니다** — 성패는 요청의 `outcome` 이고 `''` 는 실패가 아니라 판정 없음이다. 잘못된 값은 노드가 거부한다. |

세션 시작/종료와 에피소드 시작 (`/session_control/start_session`,
`/session_control/stop_session`, `/session_control/start_episode`) 은 본 패키지의
서비스가 아니라 `std_srvs/srv/Trigger` 를 그대로 사용한다. `/session_control/stop_episode`
만 예외로 위의 `StopEpisode` 를 쓴다 — 종료 시점에만 알 수 있는 성패·부가정보를 받아
`SessionCommand` 로 흘려야 rosbag2 에 기록되기 때문이다.

## 인터페이스 변경 시 주의사항

- 메시지/서비스 파일을 추가하거나 삭제할 때는 `CMakeLists.txt` 의
  `rosidl_generate_interfaces(...)` 목록과 `package.xml` 의 `<depend>` 양쪽을
  모두 갱신한다. 파일만 추가하고 CMakeLists 에 등록하지 않으면 generator 가
  타입을 만들지 않는다.
- 새 메시지가 외부 패키지 타입을 참조하면 `package.xml` 에 `<depend>` 를
  추가하고 `CMakeLists.txt` 의 `find_package(...)` + `DEPENDENCIES` 인자에도
  반영한다 (현재는 `std_msgs`, `trajectory_msgs` 를 의존).
- 본 패키지를 변경한 뒤에는 반드시 `colcon build --packages-select rdfp_msgs
  rdfp` 로 두 패키지를 함께 재빌드해야 `rdfp` 측 import (`from rdfp_msgs.srv
  import ...`) 가 새 정의를 사용한다.

## 라이선스

Apache-2.0

## 관리자

- kwlee (kwlee@etri.re.kr)
