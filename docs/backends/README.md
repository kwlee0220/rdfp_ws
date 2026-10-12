# 백엔드 문서 안내 — 시뮬레이터 · 실기

로봇 스택이 붙는 **실/가상 로봇 환경**마다 폴더 하나다. 백엔드 하나는 `config/backends/<이름>.yaml` 프로파일 하나 +
(값으로 표현되지 않는 것이 있을 때만) `robot_control/backends/<이름>.py` 로 정의되고, 제어 launch
`panda_<이름>.launch.py` 와 수집 launch `rdfp_collect backend:=<이름>` 이 그 값을 읽는다.

| 백엔드 | 문서 | 상태 |
|---|---|---|
| **Isaac Sim** (1차 학습의 줄) | [isaac/isaac_bringup_runbook.md](isaac/isaac_bringup_runbook.md) 절차 · [isaac/isaac_backend.md](isaac/isaac_backend.md) 설계·함정 · [isaac/isaac_windows_setup.md](isaac/isaac_windows_setup.md) · [isaac/isaac_scene_reset.md](isaac/isaac_scene_reset.md) | 현행 — 수집 · 적재 · export 전 과정 실측 |
| **펑션베이** (Panda + Robotiq 2F-85, peg-in-hole) | [functionbay/펑션베이_시뮬레이터_사용법.md](functionbay/펑션베이_시뮬레이터_사용법.md) (띄우는 순서) · [functionbay/펑션베이_백엔드_설계.md](functionbay/펑션베이_백엔드_설계.md) · [펑션베이_인수_체크리스트.md](functionbay/펑션베이_인수_체크리스트.md) (인수 절차 — 결과는 루트 `checklist_results/`) · [펑션베이_요청_목록.md](functionbay/펑션베이_요청_목록.md) · [펑션베이_남은_작업.md](functionbay/펑션베이_남은_작업.md) · [펑션베이_중력_보상.md](functionbay/펑션베이_중력_보상.md) · [grasp_center_frame.md](functionbay/grasp_center_frame.md) · [peg_in_hole_via_mcp_plan.md](functionbay/peg_in_hole_via_mcp_plan.md) | 현행 — 벤더 빌드에 따라 회차마다 다르다 |
| mock (`mock` · `mock_jgpc`) | [../../src/robot_control/launch/README.md](../../src/robot_control/launch/README.md) — 별도 문서 없음 | 현행 — 배관 검증용, 물체를 파지할 수 없다 |
| Gazebo Fortress | [gazebo_bringup_guide.md](gazebo_bringup_guide.md) | **미검증 · 보류** — `ros_gz_sim` 미설치, 프로파일 없음 |
| 실기 Franka Panda | [franka_panda_bringup_prep.md](franka_panda_bringup_prep.md) | 설계안 — 착수 조건 정리 |

프로파일의 블록 스키마와 함정은 [../../src/robot_control/config/backends/README.md](../../src/robot_control/config/backends/README.md) 가 정본이다.

## 백엔드 비교

| | mock / mock_jgpc | Isaac | 펑션베이 | Gazebo |
|---|---|---|---|---|
| 팔 제어 | ros2_control `GenericSystem` — JTC / **JGPC** | ros2_control `TopicBasedSystem` + JTC (`open_loop_control: true`) | **ros2_control 없음** — `MoveGroupJgpcClient` 가 `/input/panda_joint` 로 50 Hz 스트리밍 + 중력 보상 | `ign_ros2_control` + JTC |
| 관절 상태 | `joint_state_broadcaster` 100 Hz | `joint_state_broadcaster` 100 Hz (Isaac 은 `/isaac_joint_states` 로 비킨다) | `joint_state_fusion` ← `/output/panda_joint` 50 Hz, 벽시계로 정렬 | broadcaster |
| 시각 | 벽시계 | **`use_sim_time: true`** (`/clock`) | 벽시계 (`/clock` 은 쓰지 않는다) | `/clock` |
| `/scene/objects` · `/scene/reset` | `mock_scene_state_node` — 둘 다 | `isaac_scene_state_node` — 둘 다 (`simulation_interfaces`) | `functionbay_scene_state_node` — **`/scene/reset` 없음** | 미구현 |
| 카메라 | OpenCV `camera_node` (장치) `bgr8` | Isaac 그래프 `/isaac/camera/image_raw` **`rgb8`** 640×480 10 fps (실측 프레임 누락 있음) | `/camera_image/compressed` (JPEG) → `fb_camera_restamp` | gz 센서 640×480 |
| 그리퍼 | `GripperActionNode` (mock 은 effort 없어 `grasp` 판정 불가) | `GripperActionNode` (`stall_effort` 1.0, 속도 조건 끔) | `Robotiq2FGripperNode` — 씬마다 축 수 · 단위 다름 | `GripperActionNode` |
| 기본 속도 배율 | 1.0 | 1.0 | 0.4 | — |
| 학습 데이터 | ✗ (파지 불가) | **○ — 1차 학습** | ○ (peg-in-hole 환경 대기) | ✗ |

`backend:=<이름>` 이 arm 명령 채널 · 카메라 토픽 · 해상도 · fps · `pixel_format` · `use_sim_time` 을 프로파일에서
가져오므로 소비자가 백엔드를 각자 알 필요가 없다 — 어긋남은 에러가 아니라 "명령이 안 먹는다" · "0 프레임" 으로만 나타난다.

## 왜 launch 를 하나로 합치지 않았나 (2026-09-08 결정)

옛 설계안([../archive/isaac/multi_simulator_backend_design.md](../archive/isaac/multi_simulator_backend_design.md) §4)은
`backends/backend_*.launch.py` 로 launch 를 재배치해 인자 하나로 백엔드를 고르자는 것이었다. **채택하지 않았다.**

- **문제는 launch 파일의 배치가 아니라 값의 중복이었다.** 같은 사실("Isaac 은 JTC 다", "펑션베이 명령 토픽은
  `/input/panda_joint`")이 launch · 노드 · 트윈 설정에 각각 적혀 있었고, 백엔드가 바뀌었을 때 한쪽만 뒤처졌다 —
  Isaac 이 bridge → ros2_control 로 바뀔 때 실제로 그랬다. 그래서 값을 `config/backends/*.yaml` 한 곳에 모았다.
- launch 를 하나로 합치면 인자가 백엔드 수만큼 곱해지고, 백엔드마다 **다른 노드 집합**(펑션베이의
  `servo_command_bridge` · `joint_state_fusion`, Isaac 의 `commanded_joint_state_node`)을 조건문으로 감당해야 한다.
  지금 구조는 그 차이를 파일 경계로 남긴다.
- "인자 하나로 백엔드를 고른다"는 문제의식은 **수집 계층**(`rdfp_collect backend:=`)에서 실현됐고(2026-09-13),
  같은 날 백엔드별 묶음 launch 다섯(`rdfp_panda_*`)을 전부 삭제했다 — 제어 launch + `rdfp_collect` 가 같은 노드
  그래프를 만든다는 것을 실측으로 확인한 뒤다.
- 옛 설계안의 §5 "백엔드 계약"(무엇을 발행해야 하는가)과 §7 URDF 전략은 지금도 참고가 된다 — 프로파일은 그 계약의
  **값 부분**을 적은 것이다.

## 공통 함정

- **`auto` 라는 백엔드는 없다.** 프로파일 넷(`mock` · `mock_jgpc` · `isaac` · `functionbay`)이 모두 확정 모드다.
  런타임 판별은 `/panda_arm_controller/commands` 토픽 유무만 보므로 펑션베이를 JTC 로 오판한다.
- **시뮬레이터를 프로세스째 재시작하면 ROS 스택도 다시 띄운다.** sim 시계가 0 으로 되감겨 TF 버퍼가 옛 stamp 를 들고
  있고(`TF_OLD_DATA`), `/ee_pose` 가 굳으며, 컨트롤러는 `active` 라고 거짓 보고한다. Isaac 의 타임라인 Stop/Play 는
  다르다(되감지 않는다).
- **같은 `ROS_DOMAIN_ID` 에 시뮬레이터 인스턴스가 둘이면** `/clock` · `/tf` 발행자가 겹쳐 작은 시각에서
  `TF_OLD_DATA` 가 쏟아지고 배치가 seed 와 무관하게 같아진다 — `kill_stack.sh` 로 전부 내리고 하나만 띄운다.
- 파일을 지웠을 때는 재빌드로 부족하다 — colcon 이 없어진 프로파일을 설치본에서 안 치운다.
