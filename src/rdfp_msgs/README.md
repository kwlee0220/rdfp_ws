# rdfp_msgs

`rdfp` 패키지가 사용하는 ROS 2 (Humble) 메시지/서비스 인터페이스 정의 패키지다.
`ament_cmake` + `rosidl_default_generators` 기반의 순수 IDL 패키지이며 런타임
코드는 포함하지 않는다.

`rdfp` 의 노드 (`session_control_node`, `image_recorder_node`,
`gripper_control_node`, `target_joint_states_publisher` 등) 와 데이터셋
파이프라인이 본 패키지를 참조하므로 함께 빌드해야 한다.

## 빌드

워크스페이스 루트에서 `rdfp` 보다 먼저 빌드되도록 함께 지정한다.

```bash
cd ~/development/ros/rdfp_ws
colcon build --packages-select rdfp_msgs rdfp
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
| `SessionCommand` | `session_control_node` 의 `session` 토픽 | 세션 상태 머신 (`IDLE` / `IN_SESSION` / `IN_EPISODE`) 와 `task_label` 을 발행. `header.stamp` 는 발행 시각. |
| `GripperCommand` | `teleop_keyboard` / `session_teleop` → gripper 토픽 | `'open'` / `'close'` 명령을 텍스트 필드로 전송. |
| `GripperState` | `gripper_control_node` 가 액션 피드백을 토픽으로 재발행 | `position`, `effort`, `stalled`, `reached_goal` (`control_msgs/action/GripperCommand` Feedback 과 동일한 필드 구성). |
| `TargetJointStates` | `target_joint_states_publisher` / `target_joint_states_executor` | 단일 `trajectory_msgs/JointTrajectoryPoint` 에 `Header` 를 부여한 timestamped joint setpoint. |

### 서비스 (`srv/`)

| 타입 | 사용처 | 요약 |
|---|---|---|
| `StartSession` | `image_recorder_node` 의 `/image_recorder/start_session` | 요청 비어 있음 / 응답 `success`, `mp4_path`. 녹화 세션을 시작하고 생성된 MP4 절대 경로를 반환. |
| `StopSession` | `image_recorder_node` 의 `/image_recorder/stop_session` | 요청 비어 있음 / 응답 `success`, `mp4_path`. finalize 된 MP4 경로를 반환 (노드 내부 finalize timeout 5.0 초). |
| `GetSessionState` | `session_control_node` 의 `/session_control/get_session_state` | 요청 비어 있음 / 응답 `state`, `task_label`. 현재 상태와 라벨을 단발성 조회. |
| `SetString` | `session_control_node` 의 `/session_control/set_task_label` | 요청 `task_label` / 응답 `success`, `message`. 빈 문자열은 task clear, 유효하지 않은 상태 전이는 `success=false`, `message='invalid command'`. |

세션 시작/종료/에피소드 시작/종료 (`/session_control/start_session`,
`/session_control/stop_session`, `/session_control/start_episode`,
`/session_control/stop_episode`) 는 본 패키지의 서비스가 아니라
`std_srvs/srv/Trigger` 를 그대로 사용한다.

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
