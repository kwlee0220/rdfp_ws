# 문서 인덱스

`rdfp_ws` 워크스페이스의 모든 markdown 문서 목록이다. 찾는 내용이 어느 문서에
있는지 여기서 먼저 확인한다.

> **전체 구조를 먼저 알고 싶다면** [rdfp_framework_design.md](rdfp_framework_design.md) 를 읽는다 —
> 세 서브시스템과 그 사이의 인터페이스 계약을 다루는 최상위 문서다.

- **문서 총 55개** (본 인덱스 제외)
- 경로는 저장소 루트(`rdfp_ws/`) 기준이다.
- **상태** 컬럼: `현행` = 현재 코드와 일치 / `설계안` = 코드 미반영 / `이력` = 초기
  요구사항·프롬프트 등 참고용 / `구식` = 현재 코드와 어긋난 내용 있음(주의)

---

## 목차

- [1. 시작하기 — 워크스페이스 전반](#1-시작하기--워크스페이스-전반)
- [2. Launch / 실행 환경](#2-launch--실행-환경)
- [3. MoveIt2 — 계획·실행·서보](#3-moveit2--계획실행서보)
- [4. 그리퍼](#4-그리퍼)
- [5. 카메라](#5-카메라)
- [6. 녹화 (MP4 Recorder)](#6-녹화-mp4-recorder)
- [7. 세션 / 에피소드 생명주기](#7-세션--에피소드-생명주기)
- [8. rosbag2 → 데이터셋 후처리](#8-rosbag2--데이터셋-후처리)
- [9. 재생 (Replay)](#9-재생-replay)
- [10. Teleoperation](#10-teleoperation)
- [11. 시뮬레이터 백엔드 / Scene](#11-시뮬레이터-백엔드--scene)
- [12. 외부 로봇 (OMY-L100)](#12-외부-로봇-omy-l100)
- [13. 로봇 트윈 (REST 게이트웨이)](#13-로봇-트윈-rest-게이트웨이)
- [14. AI 어시스턴트용 지침 (CLAUDE.md)](#14-ai-어시스턴트용-지침-claudemd)
- [주제별 빠른 찾기](#주제별-빠른-찾기)
- [문서 정비가 필요한 항목](#문서-정비가-필요한-항목)

---

## 1. 시작하기 — 워크스페이스 전반

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [rdfp_framework_design.md](rdfp_framework_design.md) | **⭐ 최상위 아키텍처 + 설계서**. imitation learning 데이터 프레임워크의 세 서브시스템(실/가상 로봇 환경 · 에피소드 생성기 · 학습 데이터 저장/관리기)과 그 사이의 **표준 인터페이스 계약**, action/observation 구분, 2단 저장(MCAP → DBMS)의 근거, 에피소드 생명주기 시퀀스, **패키징 이슈**(§7 — 절단선 두 축. §7.1~7.5 세로축: 저장/관리기를 ROS 밖으로 뺄 것인가(보류, 결합도 실측·분리 트리거). **§7.6 가로축: 제어 계층 패키지 분리(완료)** — 워크스페이스가 아니라 패키지가 단위인 이유, launch helper `sys.path` 트릭 해소, twin↔session entry point seam, 경계 테스트), 현재 상태/미결 항목 | 현행 |
| [README.md](../README.md) | 워크스페이스 최상위 소개. 네 패키지(`rdfp_msgs`/`robot_control`/`robot_twin`/`rdfp`) 구성과 계층 의존 방향, 빌드 순서, 주요 launch·console script 개요 | 현행 |
| [src/robot_control/README.md](../src/robot_control/README.md) | **`robot_control` — 로봇 제어 계층**. `moveit`/`camera`/`scene`/`launch_helpers` 구성, `panda_*` launch 실행법, 계층 규칙(수집 계층 import 금지와 그 강제 방법) | 현행 |
| [src/robot_twin/README.md](../src/robot_twin/README.md) | **`robot_twin` — 로봇 트윈 패키지**. 실행법, 세션/에피소드 연산이 entry point 로 선택적으로 붙는 구조와 미설치 시 동작 | 현행 |
| [src/rdfp/README.md](../src/rdfp/README.md) | **`rdfp` 패키지 전체 walkthrough (가장 포괄적)**. `MoveGroupClient` API, 두 recorder 노드 파라미터표, `session_control_node` 상태머신·QoS, 데이터셋 CLI 레퍼런스, replay GUI 개요 | 현행 |
| [src/rdfp_msgs/README.md](../src/rdfp_msgs/README.md) | `rdfp_msgs` 인터페이스 패키지. `msg/`·`srv/` IDL 목록과 각 타입의 용도, 빌드 방법 | 현행 |
| [docker/README.md](../docker/README.md) | Docker 실행 구성. `run_panda_mock.sh` / `run_replay_mock.sh` / `replay_gui` 컨테이너화. apt ROS 2 Humble 만 사용해 소스빌드 moveit 과의 ABI 충돌 회피 | 현행 |
| [environment/python_env_guide.md](environment/python_env_guide.md) | **ROS 2 Python 환경 구성**. venv(uv)가 ROS 2 에서 깨지는 세 가지 구조적 원인(C 확장 ABI / `PYTHONPATH` 우선순위 / console script shebang), ROS·비-ROS 프로젝트 공존 방법(`~/.bashrc` 자동 source 제거), 프로젝트 생성 절차(venv 없는 기본형 + `--system-site-packages` 예외형), 트러블슈팅 표, Ubuntu 24.04/Jazzy 마이그레이션 전략 | 현행 |

## 2. Launch / 실행 환경

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [src/robot_control/launch/README.md](../src/robot_control/launch/README.md) | **제어 계열 launch + helper 전체 인벤토리 (자체 완결)**. `panda_mock`/`panda_jgpc_mock`/`panda_gazebo`/`moveit`/`ros2_control`/`rviz2` 의 인자 전체(§2 — 이 계열은 `--show-args` 가 그대로 동작한다), 설정 파일 3층위 구조, 순차 기동 정책과 `post_hand_actions`, helper 9개 모듈 목록과 두 가지 의도적 argument 재사용(§5) | 현행 |
| [src/rdfp/launch/README.md](../src/rdfp/launch/README.md) | **수집 계열(`rdfp_*`) launch**. `rdfp_panda_mock`/`rdfp_panda_jgpc_mock`/`replay_panda_mock`/`teleop_mirror`/`rdfp`/`rdfp_advanced`. **§3 "YAML ↔ 인자 대응표"** — 설정 YAML 4종의 키 ↔ argument 대응 (이 계열은 `OpaqueFunction` 때문에 `--show-args` 가 1~2개만 출력하므로 이 표가 유일한 목록), `replay_arm_path` 배타 선택, `pedal_timeout` 자동 보정 표. **§6.1 묶음 launch ↔ 분리 조합 대응** — `panda_mock` + `rdfp_collect` 가 `rdfp_panda_mock` 과 동등함(실측 검증), Gazebo 쌍만 인자를 맞춰야 하는 이유와 명령. helper 는 `robot_control` 문서로 위임 | 현행 |
| [robot/franka_panda_bringup_prep.md](robot/franka_panda_bringup_prep.md) | **실기 Franka Panda bringup — 준비 사항 (미착수)**. `panda_mock`/`panda_gazebo` 처럼 실기용 로봇 스택 launch 를 만들기 위한 착수 조건. 사용자가 제공해야 할 정보(모델·**시스템 버전**이 libfranka→franka_ros2 버전을 연쇄 결정·FCI 라이선스·robot_ip·엔드이펙터 load·전용 NIC·충돌 임계·안전 절차), 설치 필요 SW(현재 `franka_description` 1.0.1 만 있고 **libfranka/franka_ros2 부재** — 다만 `~/ansible/roles/franka_ros2` role 이 준비되어 있고 **실행 전 수정 3건**(`.bashrc` 자동 source 가 옵트인 정책과 충돌 · `version:` 미고정으로 FER 지원 여부 불확실 · deb 빌더 경로 불일치)), **PREEMPT_RT 실시간 커널**(FCI 1 kHz 루프 · 최악 지연 상한)과 이 PC 실측 상태(`PREEMPT_DYNAMIC` 일반 커널 · rtprio/memlock 미설정) — 단 **RT 는 팔 실행에만 걸린다**: libfranka 의 비실시간 채널(상태 읽기 · 그리퍼 · 설정)과 실시간 채널(팔 제어)을 구분해 **배선 검증 · 녹화 · 적재는 RT 없이 가능**하고 가이딩 모드로 실기 데이터까지 받을 수 있음을 3단계로 정리, 코드 변경 4건(xacro `franka` 분기 + `robot_ip` · Franka Hand 는 액션 서버라 그리퍼 어댑터 필요 · servo 설정 교체 · 안전 기동 체인), scene 은 별도 과제(읽기는 인식 노드 필요 · `reset_scene` 은 원리적 불가) | 설계안 |

## 3. MoveIt2 — 계획·실행·서보

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [moveit/README.md](moveit/README.md) | **moveit 문서 진입점**. "하려는 일 → 문서" 표, 두 진입점(`MoveGroupClient` / `ServoClient`)의 계층 그림, 읽는 순서, `robot_control/moveit/` 모듈 중 다른 폴더에 문서가 있는 것들의 위치 | 현행 |
| [moveit/MoveGroupClient_UserGuide.md](moveit/MoveGroupClient_UserGuide.md) | **공통 인터페이스 + JTC 구현**. 추상 base + `create_move_group_client()` 팩토리 + 구현별 API 가용성 대조표, Cartesian waypoint 경로 계획·실행, SRDF named target 이동, joint 목표값 이동(`move_to_joints`), 계획 전용 API(`plan_named_target`/`plan_joints`), 동작 중단(`cancel`), 동기/비동기 API, 파라미터 튜닝, Threading 주의사항, 실전 예제. JGPC 상세는 아래 문서로 위임 | 현행 |
| [moveit/MoveGroupJgpcClient_UserGuide.md](moveit/MoveGroupJgpcClient_UserGuide.md) | **JGPC 구현 전담**. `FollowJointTrajectory` 가 없는 컨트롤러에서 계획(plan_only)과 실행(명령 토픽 스트리밍)을 분리하는 방법. `MoveGroupJgpcClient` 전체 API(`stream_trajectory`/`*_streamed`/JGPC `cancel`), `TrajectoryStreamer`, joint 순서 자동 조회, `publish_rate` 보간 실측치, `tolerance` 가 최종 자세를 결정하는 이유, `replay_gui` 연동 | 현행 |
| [moveit/servo_client_programmers_guide.md](moveit/servo_client_programmers_guide.md) | `ServoClient` — `/servo_node` 시작/정지/상태 확인 유틸리티. **Node 가 아님**에 따른 사용 제약, `ServoStatus` 상태 전이, 무인 스택용 `servo_auto_start_node`, 실제 사용처 4곳 | 현행 |

## 4. 그리퍼

> **2026-09-02 에 `moveit/` 에서 분리했다.** MoveIt 이 아니라 ros2_control·시뮬레이터 계층이고, 구현 둘 중 하나(`Robotiq2FGripperNode`)는 MoveIt 을 아예 쓰지 않는다.

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [gripper/README.md](gripper/README.md) | **그리퍼 문서 진입점**. "하려는 일 → 문서" 표, **계약 하나 · 구현 둘**의 계층 그림(액션 서버 유무가 구현을 가른다), 읽는 순서, 그리고 세 가지 요점 — 명령은 심볼/관측은 물리량, 성공 판정은 `at_goal` 하나, `/joint_states` 의 손가락으로 폭을 읽지 않는다 | 현행 |
| [gripper/GripperNode_Design.md](gripper/GripperNode_Design.md) | **⭐ 그리퍼 인터페이스 설계 (2026-09-02)**. `GripperCommand`(심볼)/`GripperState`(물리량) 계약과 `GripperNode` 노드 계약. **명령은 심볼, 관측은 물리량**이라는 비대칭이 의도적인 이유 — 숫자는 그리퍼에 종속이라 기구를 바꾸면 틀린 값이 된다. `width` 는 관절값이 아니라 **개구 폭(m)**(Panda 는 2배), 못 구하면 NaN. **`at_goal` 은 위치 도달이 아니라 "시킨 일을 이뤘는가"** — `open`/`close` 는 목표 자세 도달 AND NOT `stalled`, `grasp` 는 `stalled` 다(목표 자세에 닿으면 오히려 헛닫힘). 소비자는 `at_goal` 하나만 본다. **도달을 무엇으로 재는지는 구현이 정한다** — `width` 에 묶이지 않아 `NaN` 인 스택도 판정할 수 있고, 관절 잔차로 재는 쪽이 정확한 백엔드가 있다. `effort` 를 넣지 않은 근거, 백엔드별 실현 가능성(펑션베이 `width` 매핑 미결 · Isaac `stalled` 미구현 · **mock 은 `grasp` 를 영영 성공으로 기록하지 못한다**), 반영 범위와 미결 | 현행 |
| [gripper/GripperActionNode_Guide.md](gripper/GripperActionNode_Guide.md) | **`GripperNode` 계약의 액션 기반 구현** (`robot_control`, 실행 파일 `gripper_action_node`, 노드 이름 `gripper`). 채널·QoS(`gripper_states` 는 **`TRANSIENT_LOCAL` 이 아니다**), 파라미터 7종과 기동 시 `ValueError` 로 죽는 조건, 심볼→액션 goal 변환과 **모르는 심볼 거부**(거부한 명령은 `goal` 에 싣지 않는다), 액션 결과를 상태로 쓰지 않는 이유(명령당 1건 · `at_goal` 은 매 주기 재평가), `targets`(관절값)와 `width`(개구 폭)의 **단위 변환**과 그것을 빠뜨려 `open` 판정이 뒤집혔던 회귀. **mock 은 `effort` state interface 자체가 없어 `grasp` 가 영영 성공으로 기록되지 않는다** — 트윈 타임아웃·데이터셋 공백으로 나타난다. Isaac 이 같은 노드를 쓰는 이유, 증상별 트러블슈팅 표 | 현행 |
| [gripper/Robotiq2FGripperNode_Guide.md](gripper/Robotiq2FGripperNode_Guide.md) | **Robotiq 2F 계열(2F-85) 그리퍼 구동** (`robot_control`, 실행 파일 `robotiq_2f_gripper_node`) — 액션 서버가 없어 기구를 직접 구동하는 펑션베이용 `GripperNode` 구현. **이름이 인터페이스가 아니라 기구를 가리키는 이유**(가릴 표준 인터페이스가 없다)와 모델이 아니라 계열인 이유(2F-140 은 `targets` 로 흡수). 스칼라 하나를 `axis_signs` 로 **6축에 펼쳐** 보내는 이유(시뮬레이터가 mimic 을 강제하지 않아 한 축만 보내면 링키지가 어긋난 자세가 되고 거부되지도 않는다), **`at_goal` 을 개구 폭이 아니라 관절 잔차로 판정**(지령·보고가 같은 관절 공간이라 실측 잔차 0.0000), `stalled` 을 토크와 속도 **두 신호**로 보는 이유와 실측 임계(빈손 0.004 / 자유이동 0.845 / 파지 17.19 N·m → 임계 1.0), **`width` 는 NaN** 이고 근사를 넣지 않는 이유. 계약 5케이스 검증 표와 증상별 트러블슈팅 | 현행 |
| [gripper/gripper_action_server_notes.md](gripper/gripper_action_server_notes.md) | **`/panda_hand_controller/gripper_cmd` 액션 서버 계층**. `GripperActionController` 파라미터, `position` 이 gap 이 아닌 이유(mimic 관절), 관절 한계 미검사, mock 환경에서 feedback 이 오지 않는 문제, 진단 명령. ⚠️ `status`(선점 vs 실패 구분)를 싣던 `GripperActionState` 토픽은 삭제됐다 | 현행 |

## 5. 카메라

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [camera/camera_node_guide.md](camera/camera_node_guide.md) | `CameraNode` — OpenCV `VideoCapture` 로 카메라/비디오/파일을 열어 `Image`(또는 `CompressedImage`) + `CameraInfo` + 상태 토픽 발행 | 현행 |
| [camera/image_capture_node_guide.md](camera/image_capture_node_guide.md) | `ImageCaptureNode` — `ReconnectingCamera` 로 캡처해 JPEG `CompressedImage` 만 발행하는 경량 노드. `max_attempts=0` 의 비대칭 동작(기동 시 무한 재시도 / 사용 중 끊기면 종료) 포함 | 현행 |
| [camera/rdfp_camera_node_guide.md](camera/rdfp_camera_node_guide.md) | `RdfpCameraNode` — 세션 상태에 따라 카메라를 열고 닫으며 `IN_EPISODE` 구간에서만 이미지 발행 | 현행 |
| [camera/image_viewer_node_guide.md](camera/image_viewer_node_guide.md) | `ImageViewerNode` — 이미지 토픽을 OpenCV 윈도우에 표시하는 단순 뷰어 | 현행 |
| [camera/rdfp_image_viewer_node_guide.md](camera/rdfp_image_viewer_node_guide.md) | `RdfpImageViewerNode` — 프레임 좌상단에 세션 상태를 오버레이하는 뷰어 | 현행 |
| [camera/opencv_camera_guide.md](camera/opencv_camera_guide.md) | `OpenCvCamera` 클래스 — 웹캠·비디오 파일·네트워크 스트림을 통합 인터페이스로 다루는 `VideoCapture` 래퍼 | 현행 |
| [camera/initial_requirements.md](camera/initial_requirements.md) | MP4 Recorder 노드 초기 요구사항 명세서. ⚠️ **내용은 카메라가 아니라 recorder** — 디렉터리가 어긋나 있다 | 이력 |

## 6. 녹화 (MP4 Recorder)

> 이름이 비슷한 **두 개의 recorder 노드**가 있다. 서비스로 제어하는
> `image_recorder_node` 와 `/session` 을 구독해 자동 녹화하는
> `rdfp_image_recorder` 를 혼동하지 않도록 주의한다.

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [recorder/image_recorder_node_guide.md](recorder/image_recorder_node_guide.md) | `ImageRecorderNode` — **서비스 기반** start/stop 제어. 호출당 1개 녹화. `rdfp_panda_mock.launch.py` 가 사용 | 현행 |
| [recorder/rdfp_image_recorder_node_guide.md](recorder/rdfp_image_recorder_node_guide.md) | `RdfpImageRecorderNode` — **`/session` 구독 자동 녹화**. `IN_EPISODE` 구간을 타임스탬프 기반으로 분할, `.jsonl` 사이드카 + `metadata.json` 생성. `rdfp_advanced.launch.py` 가 사용 | 현행 |
| [recorder/ffmpeg_mp4_recorder_guide.md](recorder/ffmpeg_mp4_recorder_guide.md) | `FFMpegMp4Recorder` 사용 가이드. ffmpeg subprocess 로 `numpy.ndarray` → MP4. ROS 비의존 코어 | 현행 |
| [src/rdfp/rdfp/recorder/README.md](../src/rdfp/rdfp/recorder/README.md) | `FFMpegMp4Recorder` 모듈 요약. CFR passthrough, CPU/GPU 인코더(`libx264`/`h264_nvenc`/`h264_qsv`/`h264_vaapi`), 생성자 1회 GPU probe | 현행 |

## 7. 세션 / 에피소드 생명주기

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [session/session_control_guide.md](session/session_control_guide.md) | `SessionControlNode` — IDLE → IN_SESSION → IN_EPISODE 상태머신, 서비스/토픽 인터페이스, `TRANSIENT_LOCAL` QoS 규약 | 현행 |
| [session/session_control_client_guide.md](session/session_control_client_guide.md) | `SessionControlClient` — 세션 노드를 호출하는 클라이언트 측 사용법과 제약 | 현행 |

## 8. rosbag2 → 데이터셋 후처리

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [rosbag2/데이터셋 후처리기 설계서.md](rosbag2/데이터셋%20후처리기%20설계서.md) | **후처리 파이프라인 설계 원본**. 에피소드 분할 규칙, DB 스키마, 적재 트랜잭션, 이미지 MP4 분기 구조 | 현행 |
| [rosbag2/데이터셋 후처리기 CLI 사용 설명서.md](rosbag2/데이터셋%20후처리기%20CLI%20사용%20설명서.md) | 후처리 CLI 실행 명령·옵션 레퍼런스, `dataset_config.yaml` 구조 | ⚠️ 구식 |
| [rosbag2/데이터셋 후처리기 실환경 검증 절차.md](rosbag2/데이터셋%20후처리기%20실환경%20검증%20절차.md) | 실 인프라(Docker PostgreSQL·실 rosbag·성능 측정) 검증 runbook | 현행 |
| [rosbag2/scene_objects_observation_decision.md](rosbag2/scene_objects_observation_decision.md) | **`scene_objects` 를 학습 입력(observation)에 넣을 것인가 — 선택지 비교**. '적재할 것인가'(확정·구현 완료)와 '관측에 넣을 것인가'(미결)를 분리하는 이유(적재는 비가역, 관측은 export 컬럼 매핑이라 번복 가능), 세 선택지(A 카메라만 / B GT 직접 / C 보조 손실·teacher-student·별도 estimator) 비교, **A 가 실패하는 조건**(물체가 어떤 채널에도 안 담기면 부분 관측 → 행동 다중성 → mode averaging), 현재 스택의 한시적 제약 3가지(scene 카메라 부재·mock 물리 없음·단일 시점), 권고와 재검토 트리거, 넣기로 할 경우의 실무 쟁점(가변 길이 인코딩·좌표계·xyzw·dimensions 순서) | 설계안 |
| [rosbag2/rosbag2 운영 방안.md](rosbag2/rosbag2%20운영%20방안.md) | MCAP 기반 장기 저장 운영 설계. 날짜-시간/토픽 기준 조회를 고려한 분할·보관 정책 | 현행 |

> ⚠️ **CLI 사용 설명서 주의**: 문서는 `ros2 run rdfp dataset <subcommand>` 형태로
> 안내하지만, 현재 `setup.py` 의 entry point 는 **독립 최상위 명령**으로 분리되어
> 있다 — `import` / `replay` / `stats` / `list` / `init-db` / `rosbag`.
> 예: `ros2 run rdfp dataset import` → **`ros2 run rdfp import`**.
> 정확한 현행 명령은 [src/rdfp/README.md](../src/rdfp/README.md) 또는
> [CLAUDE.md](../CLAUDE.md) 의 Console Scripts 절을 참고한다.

## 9. 재생 (Replay)

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [replay/replay_mock_stack_guide.md](replay/replay_mock_stack_guide.md) | **`replay_panda_mock.launch.py` 구조·사용법·주의점**. 제외 노드와 그 사유, `replay_arm_path` 3경로(`ee_twist`(기본)/`target_joint_cmds`/`none`) 비교, launch 인자표, ROS 환경 불일치·`start_servo` 누락·배속 역효과·stale stamp 등 함정 12가지 | 현행 |
| [replay/replay_approaches.md](replay/replay_approaches.md) | **twist vs pose vs joint 명령 비교**. "위치 명령은 폐루프, 속도 명령은 개루프" 원리, 방법 A(Servo pose tracking) / B(Cartesian path) / **C(joint replay)** 를 각각 절로 다루고 선택 기준을 표로 정리. **C 가 팔꿈치(여유자유도)까지 맞는 유일한 방법**이지만 scene 이 바뀌면 대응하지 못하고, A·B 는 그 반대라는 대비가 핵심. **고주기(50 Hz 이상) 녹화를 B 에 그대로 넣으면 계산량이 아니라 시간축이 문제** — `GetCartesianPath` 요청에 시각 필드가 없어 속도 프로파일이 버려지고 출력은 TOTG 가 ~10 Hz 로 리샘플하므로, waypoint 를 촘촘히 줘도 얻는 것이 없다. **부록에 데시메이션 설명** — 균일 간격이 아니라 거리·각도 기반이어야 코너가 살아남는 이유, RDP, 자세까지 함께 판정해야 하는 이유. **Servo pose tracking(방법 A)은 Humble 의 `servo_node` 로는 쓸 수 없다** — `delta_twist_cmds`/`delta_joint_cmds` 만 받으며 pose tracking 은 C++ 라이브러리 클래스라 노드를 새로 작성해야 한다(Jazzy 부터 `servo_node` 1급 명령 타입) | 현행 |

> replay **GUI**(`ros2 run rdfp replay_gui`) 전용 문서는 없다. GUI 사용법은
> [src/rdfp/README.md](../src/rdfp/README.md), "위치 초기화" 버튼의 컨트롤러별
> 동작은 [moveit/MoveGroupJgpcClient_UserGuide.md](moveit/MoveGroupJgpcClient_UserGuide.md) 를 본다.

## 10. Teleoperation

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [teleop/README.md](teleop/README.md) | **teleop 문서 진입점**. "하려는 일 → 읽을 문서" 표, 입력 어댑터부터 arm 까지의 전체 그림, 문서별 역할·독자·갱신 빈도, 조용히 실패하는 함정 4종의 참조 위치 | 현행 |
| [teleop/clutch_pedal_guide.md](teleop/clutch_pedal_guide.md) | **USB 풋페달 클러치 가이드**. hold(데드맨) vs toggle 비교, 데드맨을 완성하는 두 조각(페달 하트비트 + `pedal_timeout`), evdev 설치·권한·장치 식별, 파라미터 8종, 안전 동작표, `ClutchClient` API, 문제 해결 5종 | 현행 |
| [teleop/teleop_retarget_node_guide.md](teleop/teleop_retarget_node_guide.md) | **`teleop_retarget` 노드 가이드**. retargeting 이 필요한 이유(관절체계·원점·방향·작업공간 불일치), 클러치 앵커 상대 매핑 수식, 상태 기계(engage 조건·hold 재발행·자동해제 2종), 파라미터 13개 레퍼런스, 튜닝(좌우반전·스케일·손떨림), 문제 해결 5종 | 현행 |
| [teleop/omy_leader_teleop_guide.md](teleop/omy_leader_teleop_guide.md) | **OMY-L100 리더로 팔로워 제어 — 실행 절차서**. 터미널 4개 기동 순서, 체인 단계별 유량 실측표, 끊긴 지점별 원인 진단표, 함정 4가지(클러치 자동해제 / `start_servo` 누락 / 환경 불일치 / 실효 주기), 리더 없이 bag 으로 시험하는 법 | 현행 |
| [teleop/external_input_adapters.md](teleop/external_input_adapters.md) | **외부 입력 어댑터 계약**. 키보드·게임패드·OMY-L100·Isaac Sim 등 어떤 입력이든 로봇을 구동하기 위해 지켜야 할 토픽·타입·QoS·stamp·주기 규약. 두 진입 경로(twist 직접 / pose→retarget), 어겼을 때 **조용히 실패하는** 4가지 함정, 현재 어댑터 목록 | 현행 |
| [teleop/joystick_setup.md](teleop/joystick_setup.md) | 게임패드를 `/joy` 토픽까지 연결하는 절차. Bluetooth 페어링, 권한, 검증 방법 (Ubuntu 22.04 + Humble 기준 검증됨) | 현행 |
| [teleop/leader_follower_mirroring_design.md](teleop/leader_follower_mirroring_design.md) | **이기종** leader-follower 동작 미러링 설계. 관절 체계가 다른 두 로봇 간 EE pose 기반 retargeting. `teleop_mirror.launch.py` 의 근거 문서 | 설계안 |
| [teleop/leader_gripper_mapping_design.md](teleop/leader_gripper_mapping_design.md) | **리더 트리거 → 그리퍼 명령 매핑 설계**. `position` 은 트리거에서 뽑고 `max_effort` 는 고정하는 근거, 50 Hz 입력에 대한 전송 정책(deadband·최소 간격·선점 result), 액션 유지 vs 스트리밍 컨트롤러 교환표, 입력 끊김 시 **유지**(팔과 반대) 근거, 이진 → 연속 단계적 도입, 부록 A(deadband·hysteresis 개념). ⚠️ **전제가 둘 다 흔들린다** — 트리거 관절 존재가 미확인이고, `GripperCommand` 가 숫자를 싣지 않게 되어(2026-09-01) **연속 `position` 매핑은 현행 인터페이스로 보낼 수 없다.** 이진(open/close) 단계만 유효 | 설계안 |

> **외부 저장소** — OMY-L100 리더 암 어댑터는 rdfp 밖의 `omy_leader_bridge` 저장소에
> 있다 (ROS 2 Jazzy + `rmw_zenoh` 라 rdfp 와 런타임을 공유할 수 없다). 연결
> 규약은 위 `external_input_adapters.md` 가 정의한다.

## 11. 시뮬레이터 백엔드 / Scene

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [scene/scene_objects_guide.md](scene/scene_objects_guide.md) | **⭐ scene 물체 상태 계약 통합 문서**. 백엔드(mock/Gazebo/Isaac) 차이를 발행 노드가 흡수하고 `/scene/objects` 하나만 내보내는 경계, `SceneObject` 필드별 계약(`type` 이 문자열인 이유, `dimensions` 가 `SolidPrimitive` 순서라 **cylinder 는 [높이, 반지름]**, 배열-of-구조체인 이유, orientation **xyzw** 와 Isaac wxyz 함정), `SceneObjects` 의 stamp 필수·frame_id·빈 배열 의미, 쓰기 경로(`/scene/reset` 서비스 · 무작위 추출이 트윈인 이유 · 개별 추가/삭제를 두지 않는 이유), **백엔드별 발행 노드 3종** — `mock_scene_state_node`(서비스 폴링→diff 누적 실측 근거, QoS 두 개, 좌표 이중 합성, **mesh 는 읽기만 되고 배치는 불가**, mock 은 물리 없음) · `isaac_scene_state_node`(**TF 경유로 좌표 변환과 wxyz 규약을 동시에 해결** — 변환 코드를 아예 없앤 판단, 이름·크기는 `isaac_scene.json`) · `gazebo_scene_state_node`(미구현), 그리고 **무엇이 실리는가**(조작 대상뿐 — Isaac 은 `dynamic: true` 만 발행, 환경 물체는 시뮬레이터에 남고 발행만 빠진다. **MoveIt planning scene 에는 아무것도 넣지 않는다** — 2026-09-01 결정으로 `planning_scene_sync` 와 `SceneObject.fixture` 를 삭제했고, **팔이 탁자를 통과하는 것은 단순성을 위해 감수한 위험**이다. 자기충돌 검사와 cartesian 파지는 영향 없음), 트윈 변수·`reset_scene`(resource `[scene,arm]`)·레시피 문법, launch 인자(기본 on 인 이유·이름 충돌·replay 제외), DB 적재, 함정 11가지 | 현행 |
| [simulation/gazebo_bringup_guide.md](simulation/gazebo_bringup_guide.md) | Gazebo Fortress(gz-sim 6) 백엔드 브링업. `mock_components` 를 물리 시뮬레이션으로 교체해 실행하는 절차. `panda_gazebo.launch.py` 사용 | 현행 |
| [simulation/multi_simulator_backend_design.md](simulation/multi_simulator_backend_design.md) | mock / Gazebo / Isaac Sim 을 교체 가능한 백엔드로 꽂는 구조 설계. 문서 자체가 "코드 미반영" 명시 | 설계안 |
| [simulation/functionbay_backend_design.md](simulation/functionbay_backend_design.md) | **펑션베이 백엔드 (토픽 연동형) 설계**. ros2_control 없이 MoveIt 스택을 올리는 구성 — `joint_state_fusion`(이름 없는 배열에 이름 부여) · `readiness_gate`(spawner 대체, 종료코드 게이팅). 보간 없는 시뮬레이터라 `publish_rate=50` 이 필수인 근거(10 Hz vs 50 Hz 실측), 그리퍼가 Robotiq 2F-85 라 Panda Hand 전제와 어긋나 보류한 경위, 수집 계층 조합 실측 | 현행 |
| [simulation/functionbay_open_work.md](simulation/functionbay_open_work.md) | **펑션베이 백엔드 인수인계 — 남은 작업**. 3회 실측(08-18 / 08-28 / **08-31 — 솔버가 `펑션베이` → `서울대` 로 교체됨**). §0.1 에 **두 솔버 비교표**, §0.2 에 **수집용 솔버 권장(서울대) 과 근거** — 정확도는 10배 개선(처짐 0.0983 → 0.0097 rad), 응답은 1.8배 악화(τ 133 → 242 ms), 보고 주기 50 → 10 → 30 Hz(카메라는 9.6 Hz 로 무관), `/output/endeffector` 복구. servo 단순 브리지는 **발산이 사라진 대신 추종 실패**(명령 방향과 무관하게 EE 가 아래로 흐름). 간헐적 고착은 후보가 **Unity 수신 경로 하나로 축소**(엔드포인트 재시작·EE 명령 모두 무효 확인). 그 밖에 자기충돌, 제어기 강성 모델(오차 = 중력토크/K_p), 재현 절차, 진단 스크립트 6종 | 현행 |
| [simulation/functionbay_vendor_requests.md](simulation/functionbay_vendor_requests.md) | **펑션베이 벤더 요청 항목**. `functionbay_open_work.md` 실측에서 "시뮬레이터 쪽에서만 고칠 수 있는" 것만 뽑아 요청 단위로 재구성 — 각 항목이 *무엇을 · 왜 우리가 못 고치나 · 수용 기준* 세 가지를 갖춘다. 필수 4건(**중력 보상**·제어기 사양 공개·카메라 `stamp`·명령 무반응 고착 문의)과 후속 7건(`K_p` 상향·게인 노출·관절 한계 클램프·`/output/endeffector`·`rgb8`·그리퍼 단위·NTP). 검증 스크립트 절차와 **요청하지 않는 것**(이미 정상인 8항목 — 특히 `/clock` 은 불필요) 포함 | 현행 |
| [simulation/isaac_backend_skeleton.md](simulation/isaac_backend_skeleton.md) | **Isaac Sim 백엔드 — 설계·구현·실측 (Phase 0~9 완료, 2026-08-30)**. 골격 → 팔(τ 195.6→34.9 ms) → 그리퍼 → scene(쿼터니언 45° 검증) → 카메라(640×480 @ 5 Hz) → **물리 파지**(블록 +0.0999 m 들어올림) → **수집 계층 연동**(에피소드 2개 분절) → **테스트·토폴로지 계약**(robot_control 146→231) → **robot_twin 연동**(REST 로 팔·그리퍼·세션 실제 제어). **DB 적재까지 확인** — init-db 12테이블, 에피소드 3개, 무결성·재적재·MP4 검증. **배포 구성 네 가지**(Windows+WSL2 / Ubuntu 단독 / Ubuntu 2대 / Windows+Ubuntu)별 구동법 — A 만 실측, B~D 미검증. 결정 10건 중 **7건 종결**. **함정 15건**을 증상 색인표와 함께 기록 — `DEFAULT_ROS_DISTRO=jazzy`, MTU 초과 소실, `initialPeersList` 포트 누락, Stop/Play 시간 역행, cp949 가림, **로봇이 월드 원점에 없어 35 cm 어긋남**, **에셋 기본 자세가 탁자 안**, **마찰 재질 미바인딩**, **`externally_spun` 미전달로 트윈이 영원히 RUNNING** | 현행 |
| [simulation/isaac_windows_setup.md](simulation/isaac_windows_setup.md) | **새 Windows 11 머신 구축 절차 (구성 A: Windows Isaac + WSL2 스택)**. Isaac Sim 만 설치된 상태에서 처음부터 돌리기까지 8단계 — WSL2/Ubuntu-22.04, `.wslconfig`(메모리 캡·mirrored networking), ROS 2 Humble 의존 목록, 워크스페이스 빌드, **Isaac 쪽은 파일 2개 복사**(`run_isaac_humble.bat` + `fastdds_wsl_bridge.xml`), 환경변수 4개, **sim_side 스크립트 6개의 실행 순서와 각각이 없으면 생기는 증상**, 검증 순서. 자주 막히는 곳 7가지를 증상 → 원인 → 확인법 표로 정리 — **셋이 모두 '에러 없이 토픽이 안 보이는' 증상이라 눈으로 구별되지 않는다**. 근거·설계는 isaac_backend_skeleton.md 에 있고 이 문서는 재현 절차만 담는다 | 현행 |

| [topic_naming_contract.md](topic_naming_contract.md) | **⭐ 토픽 이름 규약 (1단계 결정, 2026-09-01)**. 논리 채널마다 이름을 하나로 고정한다 — 기준은 `panda_mock` 이 쓰는 이름이며 `config/recording_topics.list` 11종이 그대로 정규 이름이다. **정규 이름은 상대 경로**로 적어 2단계(로봇별 네임스페이스)가 `PushRosNamespace` 한 줄로 끝나게 한다. 단수/복수는 **메시지 모양을 따른다**(`ee_pose` 단수 · `scene/objects` 복수). **`gripper_states` 는 `/joint_states` 로 대신할 수 없다** — 펑션베이가 TF 성립용 고정값을 주입해 거짓말을 하기 때문이며, 값은 관절값이 아니라 **개구 폭(m)** 이다(필드 의미는 `GripperNode_Design.md` 가 정본). 그리퍼 두 채널은 `~/` 가 아니라 **루트 상대**다 — `~/` 는 네임스페이스가 아니라 **노드 이름**을 붙여 채널이 구현에 묶이고 2단계 `PushRosNamespace` 도 타지 못한다. 폐기된 `gripper_action_states` 의 판단 기록과 그것이 남긴 Isaac `stalled` 부채도 여기 있다. **arm 명령은 규약이 아니다** — 스택마다 컨트롤러 유무와 타입이 달라 통일하지 않고, 그 위의 `target_joint_cmds` 를 정규 채널로 둔다. remap 은 **백엔드 경계에서만**. 지금 어긋난 곳(절대 기본값·검사 스크립트 11종·트윈 설정 8종)과 적용 순서 5단계, `/clock`·`/tf` 를 전역으로 두는 판단과 **다중 로봇에는 프레임 접두사가 필수**라는 점 | 현행 |
## 12. 외부 로봇 (OMY-L100)

> `rdfp` 패키지와 직접 관련 없는 별도 로봇(ROBOTIS OMY-L100) 자료다.
> 컨테이너 내부는 **Ubuntu 24.04 + ROS 2 Jazzy** 기준이라 이 워크스페이스의
> Humble 환경과 다르다.

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [OMY-L100_Docker_Setup_and_Usage.md](OMY-L100_Docker_Setup_and_Usage.md) | OMY-L100 소프트웨어 설치·활용. ROBOTIS Docker 컨테이너 기준 | 현행 |
| [OMY-L100_Docker_Setup_and_Usage_with_Volumes.md](OMY-L100_Docker_Setup_and_Usage_with_Volumes.md) | 위 문서의 **볼륨 마운트 버전**. 호스트 작업물을 컨테이너에 연결하는 구성 추가 | 현행 |

## 13. 로봇 트윈 (REST 게이트웨이)

> ROS 2 를 직접 사용할 수 없는 외부 시스템(MDT Platform 등)이 REST 로 로봇 상태를
> 조회하고 명령을 실행하기 위한 게이트웨이다. 트윈 자체는 Python + rclpy 로
> 구현하며 rdfp 의 `MoveGroupClient` / 그리퍼 서비스를 재사용한다.

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [robot_twin/robot_twin_user_guide.md](robot_twin/robot_twin_user_guide.md) | **로봇 트윈 사용 설명서**. 시작/종료 절차(`--config` 절대경로·PGID 종료), REST 인터페이스 전체와 응답 형식(`quality`·ETag/304·배치 조회 보장 수준·오류 코드표), 제공 상태 변수·연산 목록과 미구현 항목, 자원 락, curl/Python 예제 프로그램(재시도·취소·모니터링), 로봇 연동 설정(`move_group_mode` 필수·다른 로봇 붙이기), **자동 에피소드 수집 경로**(`reset_scene` 로 물체 랜덤 배치 + `start_session`/`start_episode`/`stop_episode`/`stop_session` 경계 + 수집 루프 예제, scene 노드는 mock 계열 launch 가 기본 기동), 새 변수/연산 추가 방법, 운영 주의(인증 없음·E-stop 한계·워치독), 트러블슈팅 표 | 현행 |
| [robot_twin/robot_twin_design.md](robot_twin/robot_twin_design.md) | **로봇 트윈 설계서 (1,589줄)**. `rdfp` 내 `twin/` 서브패키지로 구현하는 단일 Python 프로세스 게이트웨이. 스레드 모델(executor 전용 스레드 + 불변 스냅샷 원자 교체, `externally_spun=True`, `mode='auto'` 금지), 선언적 트윈 정의 YAML, 상태 변수의 품질(`OK`/`STALE`/`NO_DATA`/`SOURCE_UNAVAILABLE`/`ERROR`)·조회 API(envelope·ETag·배치·부분실패)·JSON 직렬화 규약, extern_op 기반 연산(다중 세션·`kind` 고정·자원 락 admission control·`phase: CANCELING`·E-stop·오류 코드), 미채택 요소(`IDLE`/`Idempotency-Key`/인증/TLS)와 그 근거, rdfp 구현 격차. **부록 B(extern_op 확장 목록)는 확장 명세 초안으로 사용 가능** | 설계안 |
| [robot_twin/mcp_server_design.md](robot_twin/mcp_server_design.md) | **로봇 트윈 MCP 서버 설계서**. LLM 에이전트가 대화만으로 로봇 상태를 읽고 연산을 조합하게 하는 경로(`~/development/mdtpy/robot-twin` 의 `mcp_server.py`). **트윈이 자기 능력을 알리기 위한 변경**(`config.py`/`api.py` 에 `description` 필드 + 카탈로그 노출, 설정 YAML 19건 작성)과 그 필요성(`extra='forbid'` 라 YAML 만으로는 기동 실패), 도구 18개의 생성 규칙(카탈로그 자동 생성 5 + 검증 래퍼 교체 2 + 추가 11, 감춤 4), 결정 13건(상태 변수를 resource 가 아닌 tool 로 노출한 이유, 여는 연산만 감춘 비대칭, 작업 상태를 서버가 소유하고 metadata 자동 병합, SDK 2.0 에서 low-level API 를 쓴 경위), 이슈(인증 없는 경로 위임·카탈로그 캐시·동시 호출 무보호·타임아웃 2층) | 현행 |
| [robot_twin/auto_episode_collection_draft.md](robot_twin/auto_episode_collection_draft.md) | **자동 에피소드 수집을 위한 트윈 확장 (임시 초안)**. 물체 위치를 랜덤화하며 스크립트 pick-and-place 를 반복해 학습 데이터를 양산하는 경로. mock/Gazebo/Isaac 은 인터페이스는 통일 가능하나 **mock 은 물체가 움직이지 않아 자동 라벨링이 불가능**하다는 구분, 백엔드별 발행 노드 → `/scene/objects` 단일 토픽 → 트윈(코드 변경 0) 배선, `rdfp_msgs/SceneObjects` 제안(stamp 필수·frame `panda_link0` 고정), 저수준 spawn API 대신 `reset_scene(scene, seed)` + 레시피 config(그리퍼 `targets` 패턴 재사용)와 **실제 배치 pose 를 outputs 로 남겨야 재현된다**는 근거, 최대 구멍인 **트윈의 에피소드 경계 부재**(`start_episode`/`stop_episode` 우선)와 `sessions` 스키마 확장 필요성, 작업 순서·미결정 체크리스트. **§6 작업 1~5b 완료(2026-08-17) — 남은 것은 Gazebo(6)·Isaac(7) 연동뿐이며, 완료 표시가 붙은 절은 이미 코드에 반영됨.** 작업 후 정식 문서로 옮기고 삭제 예정 | 설계안 |

## 14. AI 어시스턴트용 지침 (CLAUDE.md)

사람이 읽는 문서가 아니라 Claude Code 에 주입되는 지침이지만, **비자명한 동작
(non-obvious behaviors)** 이 정리되어 있어 디버깅 시 유용하다.

| 문서 | 내용 |
|---|---|
| [CLAUDE.md](../CLAUDE.md) | 워크스페이스 최상위 지침 — 빌드 명령, 아키텍처 개요, 코딩 컨벤션, **"Important non-obvious behaviors"**(stale build artifact 문제, `metadata.yaml` 없는 rosbag 무시, JGPC launch 주의점 등) |
| [src/rdfp/CLAUDE.md](../src/rdfp/CLAUDE.md) | `rdfp` 패키지 지침 — console script 추가 절차, `setup.py data_files` 설치 규칙, apt/pip 의존성 분리, 두 recorder 노드 구분, replay 서브시스템 결합 관계 |
| [src/rdfp_msgs/CLAUDE.md](../src/rdfp_msgs/CLAUDE.md) | `rdfp_msgs` 지침 — 순수 IDL 패키지 특성과 인터페이스 추가 절차 |

---

## 주제별 빠른 찾기

| 하고 싶은 것 | 볼 문서 |
|---|---|
| 처음 빌드하고 실행해 본다 | [README.md](../README.md) → [src/rdfp/README.md](../src/rdfp/README.md) |
| 학습 데이터 없이 로봇 제어 스택만 쓴다 | [src/robot_control/README.md](../src/robot_control/README.md) |
| 패키지가 왜 넷으로 나뉘어 있는지 알고 싶다 | [rdfp_framework_design.md](rdfp_framework_design.md) §7.6 |
| Python 가상환경(uv)과 ROS 2 가 충돌한다 / 새 ROS 2 프로젝트를 만든다 | [environment/python_env_guide.md](environment/python_env_guide.md) |
| Ubuntu 24.04 / ROS 2 Jazzy 로 옮긴다 | [environment/python_env_guide.md](environment/python_env_guide.md) 의 "마이그레이션" 절 |
| 제어 스택은 두고 수집만 재시작하고 싶다 | [src/rdfp/launch/README.md](../src/rdfp/launch/README.md) §6.1 (`rdfp_collect`) |
| 어떤 launch 를 써야 할지 모르겠다 | 제어만 → [src/robot_control/launch/README.md](../src/robot_control/launch/README.md) §6 / 수집까지 → [src/rdfp/launch/README.md](../src/rdfp/launch/README.md) §6 |
| `panda_mock` 의 인자를 알고 싶다 | [src/robot_control/launch/README.md](../src/robot_control/launch/README.md) §2.1 |
| MoveIt 문서 중 뭘 볼지 모르겠다 | [moveit/README.md](moveit/README.md) |
| 로봇을 특정 pose 로 움직인다 | [moveit/MoveGroupClient_UserGuide.md](moveit/MoveGroupClient_UserGuide.md) |
| 실기 Franka Panda 를 붙인다 | [robot/franka_panda_bringup_prep.md](robot/franka_panda_bringup_prep.md) |
| scene 에 물체를 놓거나 물체 좌표를 읽는다 | [scene/scene_objects_guide.md](scene/scene_objects_guide.md) |
| JGPC(비-JTC) 환경에서 움직인다 | [moveit/MoveGroupJgpcClient_UserGuide.md](moveit/MoveGroupJgpcClient_UserGuide.md) |
| servo 로 실시간 제어한다 | [moveit/servo_client_programmers_guide.md](moveit/servo_client_programmers_guide.md) |
| 그리퍼를 코드에서 쓴다 (명령 발행·상태 구독·성공 판정) | [gripper/GripperNode_Design.md](gripper/GripperNode_Design.md) §5 |
| 그리퍼가 안 움직인다 / 상태가 안 온다 | [gripper/gripper_action_server_notes.md](gripper/gripper_action_server_notes.md) |
| 그리퍼 메시지 필드의 뜻을 안다 (`width`/`stalled`/`at_goal`) | [gripper/GripperNode_Design.md](gripper/GripperNode_Design.md) |
| 그리퍼 노드를 띄우고 파라미터를 조정한다 | [gripper/GripperActionNode_Guide.md](gripper/GripperActionNode_Guide.md) (mock·Isaac) / [gripper/Robotiq2FGripperNode_Guide.md](gripper/Robotiq2FGripperNode_Guide.md) (펑션베이) |
| 카메라 영상을 토픽으로 낸다 | [camera/camera_node_guide.md](camera/camera_node_guide.md) |
| 카메라 영상을 JPEG 압축으로만 낸다 | [camera/image_capture_node_guide.md](camera/image_capture_node_guide.md) |
| 영상을 MP4 로 녹화한다 | 서비스 제어 → [recorder/image_recorder_node_guide.md](recorder/image_recorder_node_guide.md) / 세션 자동 → [recorder/rdfp_image_recorder_node_guide.md](recorder/rdfp_image_recorder_node_guide.md) |
| 세션·에피소드를 시작/종료한다 | [session/session_control_guide.md](session/session_control_guide.md) |
| rosbag 을 DB 에 적재한다 | [rosbag2/데이터셋 후처리기 CLI 사용 설명서.md](rosbag2/데이터셋%20후처리기%20CLI%20사용%20설명서.md) (⚠️ 명령 형태는 구식) |
| 적재 파이프라인 내부를 이해한다 | [rosbag2/데이터셋 후처리기 설계서.md](rosbag2/데이터셋%20후처리기%20설계서.md) |
| 적재된 에피소드를 재생한다 | [replay/replay_mock_stack_guide.md](replay/replay_mock_stack_guide.md) |
| 조이스틱을 연결한다 | [teleop/joystick_setup.md](teleop/joystick_setup.md) |
| OMY-L100 리더로 로봇을 움직인다 | [teleop/omy_leader_teleop_guide.md](teleop/omy_leader_teleop_guide.md) |
| 새 입력 장치(게임패드·시뮬레이터 등)를 붙인다 | [teleop/external_input_adapters.md](teleop/external_input_adapters.md) |
| 리더-팔로워 매핑을 튜닝한다 (스케일·정렬·필터) | [teleop/teleop_retarget_node_guide.md](teleop/teleop_retarget_node_guide.md) |
| 펑션베이 시뮬레이터로 돌린다 | [simulation/functionbay_backend_design.md](simulation/functionbay_backend_design.md) |
| 펑션베이 작업을 이어서 한다 (남은 작업·블로커) | [simulation/functionbay_open_work.md](simulation/functionbay_open_work.md) |
| 펑션베이 시뮬레이터 제작사에 수정을 요청한다 | [simulation/functionbay_vendor_requests.md](simulation/functionbay_vendor_requests.md) |
| 토픽 이름을 정하거나 바꾼다 | [topic_naming_contract.md](topic_naming_contract.md) |
| 그리퍼 명령·상태 채널을 다룬다 | [gripper/GripperNode_Design.md](gripper/GripperNode_Design.md) |
| Isaac Sim 백엔드를 개발한다 | [simulation/isaac_backend_skeleton.md](simulation/isaac_backend_skeleton.md) |
| 새 Windows 머신에 Isaac 환경을 구축한다 | [simulation/isaac_windows_setup.md](simulation/isaac_windows_setup.md) |
| Gazebo 로 돌린다 | [simulation/gazebo_bringup_guide.md](simulation/gazebo_bringup_guide.md) |
| Docker 로 실행한다 | [docker/README.md](../docker/README.md) |
| 외부 시스템에 REST 로 로봇을 노출한다 | [robot_twin/robot_twin_user_guide.md](robot_twin/robot_twin_user_guide.md) (사용) → [robot_twin_design.md](robot_twin/robot_twin_design.md) (설계 근거) |
| 이상 동작을 디버깅한다 | [CLAUDE.md](../CLAUDE.md) 의 "Important non-obvious behaviors" 절 |

---

## 문서 정비가 필요한 항목

인덱스 작성 중 확인된 불일치다. 문서를 읽을 때 참고한다.

| 항목 | 위치 | 내용 |
|---|---|---|
| 구식 CLI 안내 | [rosbag2/데이터셋 후처리기 CLI 사용 설명서.md](rosbag2/데이터셋%20후처리기%20CLI%20사용%20설명서.md) | `ros2 run rdfp dataset <sub>` 형태로 안내하나, 실제로는 `import`/`replay`/`stats`/`list`/`init-db` 독립 명령으로 분리됨 |
| 디렉터리 불일치 | [camera/initial_requirements.md](camera/initial_requirements.md) | 제목·내용이 "MP4 Recorder 노드 요구사항" 인데 `docs/camera/` 에 위치 |
| 문서 없음 | — | `replay_gui` (Tk GUI) 전용 가이드가 없다. `src/rdfp/README.md` 에 개요만 있음 |
| 미등재 문서 | [data/README.md](../data/README.md) | 검증용 rosbag(`ee_pose_bag`) 설명이 인덱스 표에 없다. `replay_panda_mock` 의 `ee_twist` 경로 검증에 쓰인다 |
