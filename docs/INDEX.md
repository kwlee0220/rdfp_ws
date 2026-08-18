# 문서 인덱스

`rdfp_ws` 워크스페이스의 모든 markdown 문서 목록이다. 찾는 내용이 어느 문서에
있는지 여기서 먼저 확인한다.

> **전체 구조를 먼저 알고 싶다면** [rdfp_framework_design.md](rdfp_framework_design.md) 를 읽는다 —
> 세 서브시스템과 그 사이의 인터페이스 계약을 다루는 최상위 문서다.

- **문서 총 53개** (본 인덱스 제외)
- 경로는 저장소 루트(`rdfp_ws/`) 기준이다.
- **상태** 컬럼: `현행` = 현재 코드와 일치 / `설계안` = 코드 미반영 / `이력` = 초기
  요구사항·프롬프트 등 참고용 / `구식` = 현재 코드와 어긋난 내용 있음(주의)

---

## 목차

- [1. 시작하기 — 워크스페이스 전반](#1-시작하기--워크스페이스-전반)
- [2. Launch / 실행 환경](#2-launch--실행-환경)
- [3. MoveIt2 — 계획·실행·그리퍼](#3-moveit2--계획실행그리퍼)
- [4. 카메라](#4-카메라)
- [5. 녹화 (MP4 Recorder)](#5-녹화-mp4-recorder)
- [6. 세션 / 에피소드 생명주기](#6-세션--에피소드-생명주기)
- [7. rosbag2 → 데이터셋 후처리](#7-rosbag2--데이터셋-후처리)
- [8. 재생 (Replay)](#8-재생-replay)
- [9. Teleoperation](#9-teleoperation)
- [10. 시뮬레이터 백엔드](#10-시뮬레이터-백엔드)
- [11. 외부 로봇 (OMY-L100)](#11-외부-로봇-omy-l100)
- [12. 로봇 트윈 (REST 게이트웨이)](#12-로봇-트윈-rest-게이트웨이)
- [13. AI 어시스턴트용 지침 (CLAUDE.md)](#13-ai-어시스턴트용-지침-claudemd)
- [주제별 빠른 찾기](#주제별-빠른-찾기)
- [문서 정비가 필요한 항목](#문서-정비가-필요한-항목)

---

## 1. 시작하기 — 워크스페이스 전반

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [rdfp_framework_design.md](rdfp_framework_design.md) | **⭐ 최상위 아키텍처 + 설계서**. imitation learning 데이터 프레임워크의 세 서브시스템(실/가상 로봇 환경 · 에피소드 생성기 · 학습 데이터 저장/관리기)과 그 사이의 **표준 인터페이스 계약**, action/observation 구분, 2단 저장(MCAP → DBMS)의 근거, 에피소드 생명주기 시퀀스, **패키징 이슈**(ROS 워크스페이스에 저장/관리기를 두는 것이 맞는가 — 결합도 실측·선택지 비교·분리 트리거), 현재 상태/미결 항목 | 현행 |
| [README.md](../README.md) | 워크스페이스 최상위 소개. 두 패키지(`rdfp`/`rdfp_msgs`) 구성, 빌드 순서, 주요 launch·console script 개요 | 현행 |
| [src/rdfp/README.md](../src/rdfp/README.md) | **`rdfp` 패키지 전체 walkthrough (720줄, 가장 포괄적)**. `MoveGroupClient` API, 두 recorder 노드 파라미터표, `session_control_node` 상태머신·QoS, 데이터셋 CLI 레퍼런스, replay GUI 개요 | 현행 |
| [src/rdfp_msgs/README.md](../src/rdfp_msgs/README.md) | `rdfp_msgs` 인터페이스 패키지. `msg/`·`srv/` IDL 목록과 각 타입의 용도, 빌드 방법 | 현행 |
| [docker/README.md](../docker/README.md) | Docker 실행 구성. `run_panda_mock.sh` / `run_replay_mock.sh` / `replay_gui` 컨테이너화. apt ROS 2 Humble 만 사용해 소스빌드 moveit 과의 ABI 충돌 회피 | 현행 |
| [environment/python_env_guide.md](environment/python_env_guide.md) | **ROS 2 Python 환경 구성**. venv(uv)가 ROS 2 에서 깨지는 세 가지 구조적 원인(C 확장 ABI / `PYTHONPATH` 우선순위 / console script shebang), ROS·비-ROS 프로젝트 공존 방법(`~/.bashrc` 자동 source 제거), 프로젝트 생성 절차(venv 없는 기본형 + `--system-site-packages` 예외형), 트러블슈팅 표, Ubuntu 24.04/Jazzy 마이그레이션 전략 | 현행 |

## 2. Launch / 실행 환경

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [src/rdfp/launch/README.md](../src/rdfp/launch/README.md) | **launch 파일·helper 전체 인벤토리**. 계열별 분류, launch × helper 의존관계 표, Panda 컨트롤러 순차 기동 순서와 그 근거, 각 launch 의 `post_hand_actions` 구성. **§4 "Launch 인자"** — launch 별 argument 표 + 설정 YAML 4종(`image_pipeline`=§4.2 / `rdfp_panda_mock`=§4.4 / `replay_panda_mock`=§4.5 / `teleop_mirror`=§4.6) 의 키 ↔ argument 대응 (YAML 계열은 `--show-args` 가 `config_file` 만 출력하므로 이 표가 유일한 목록). 절 번호 있음 | 현행 |

## 3. MoveIt2 — 계획·실행·그리퍼

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [moveit/README.md](moveit/README.md) | **moveit 문서 진입점**. "하려는 일 → 문서" 표, 세 진입점(`MoveGroupClient` / `ServoClient` / `GripperControlNode`)의 계층 그림, 읽는 순서, `rdfp/moveit/` 모듈 중 다른 폴더에 문서가 있는 것들의 위치 | 현행 |
| [moveit/MoveGroupClient_UserGuide.md](moveit/MoveGroupClient_UserGuide.md) | **공통 인터페이스 + JTC 구현**. 추상 base + `create_move_group_client()` 팩토리 + 구현별 API 가용성 대조표, Cartesian waypoint 경로 계획·실행, SRDF named target 이동, joint 목표값 이동(`move_to_joints`), 계획 전용 API(`plan_named_target`/`plan_joints`), 동작 중단(`cancel`), 동기/비동기 API, 파라미터 튜닝, Threading 주의사항, 실전 예제. JGPC 상세는 아래 문서로 위임 | 현행 |
| [moveit/MoveGroupJgpcClient_UserGuide.md](moveit/MoveGroupJgpcClient_UserGuide.md) | **JGPC 구현 전담**. `FollowJointTrajectory` 가 없는 컨트롤러에서 계획(plan_only)과 실행(명령 토픽 스트리밍)을 분리하는 방법. `MoveGroupJgpcClient` 전체 API(`stream_trajectory`/`*_streamed`/JGPC `cancel`), `TrajectoryStreamer`, joint 순서 자동 조회, `publish_rate` 보간 실측치, `tolerance` 가 최종 자세를 결정하는 이유, `replay_gui` 연동 | 현행 |
| [moveit/servo_client_programmers_guide.md](moveit/servo_client_programmers_guide.md) | `ServoClient` — `/servo_node` 시작/정지/상태 확인 유틸리티. **Node 가 아님**에 따른 사용 제약, `ServoStatus` 상태 전이, 무인 스택용 `servo_auto_start_node`, 실제 사용처 4곳 | 현행 |
| [moveit/GripperControlNode_Guide.md](moveit/GripperControlNode_Guide.md) | `GripperControlNode` — `control_msgs/GripperCommand` 액션을 `std_srvs/Trigger` 서비스로 감싼 그리퍼 제어 노드. 서비스/토픽 인터페이스, 예제, 에러 처리, 트러블슈팅 | 현행 |
| [moveit/gripper_action_server_notes.md](moveit/gripper_action_server_notes.md) | **`/panda_hand_controller/gripper_cmd` 액션 서버 계층**. `GripperActionController` 파라미터, `position` 이 gap 이 아닌 이유(mimic 관절), 관절 한계 미검사, mock 환경에서 feedback 이 오지 않는 문제, 진단 명령 | 현행 |

## 4. 카메라

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [camera/camera_node_guide.md](camera/camera_node_guide.md) | `CameraNode` — OpenCV `VideoCapture` 로 카메라/비디오/파일을 열어 `Image`(또는 `CompressedImage`) + `CameraInfo` + 상태 토픽 발행 | 현행 |
| [camera/rdfp_camera_node_guide.md](camera/rdfp_camera_node_guide.md) | `RdfpCameraNode` — 세션 상태에 따라 카메라를 열고 닫으며 `IN_EPISODE` 구간에서만 이미지 발행 | 현행 |
| [camera/image_viewer_node_guide.md](camera/image_viewer_node_guide.md) | `ImageViewerNode` — 이미지 토픽을 OpenCV 윈도우에 표시하는 단순 뷰어 | 현행 |
| [camera/rdfp_image_viewer_node_guide.md](camera/rdfp_image_viewer_node_guide.md) | `RdfpImageViewerNode` — 프레임 좌상단에 세션 상태를 오버레이하는 뷰어 | 현행 |
| [camera/opencv_camera_guide.md](camera/opencv_camera_guide.md) | `OpenCvCamera` 클래스 — 웹캠·비디오 파일·네트워크 스트림을 통합 인터페이스로 다루는 `VideoCapture` 래퍼 | 현행 |
| [camera/initial_requirements.md](camera/initial_requirements.md) | MP4 Recorder 노드 초기 요구사항 명세서. ⚠️ **내용은 카메라가 아니라 recorder** — 디렉터리가 어긋나 있다 | 이력 |

## 5. 녹화 (MP4 Recorder)

> 이름이 비슷한 **두 개의 recorder 노드**가 있다. 서비스로 제어하는
> `image_recorder_node` 와 `/session` 을 구독해 자동 녹화하는
> `rdfp_image_recorder` 를 혼동하지 않도록 주의한다.

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [recorder/image_recorder_node_guide.md](recorder/image_recorder_node_guide.md) | `ImageRecorderNode` — **서비스 기반** start/stop 제어. 호출당 1개 녹화. `rdfp_panda_mock.launch.py` 가 사용 | 현행 |
| [recorder/rdfp_image_recorder_node_guide.md](recorder/rdfp_image_recorder_node_guide.md) | `RdfpImageRecorderNode` — **`/session` 구독 자동 녹화**. `IN_EPISODE` 구간을 타임스탬프 기반으로 분할, `.jsonl` 사이드카 + `metadata.json` 생성. `rdfp_advanced.launch.py` 가 사용 | 현행 |
| [recorder/ffmpeg_mp4_recorder_guide.md](recorder/ffmpeg_mp4_recorder_guide.md) | `FFMpegMp4Recorder` 사용 가이드. ffmpeg subprocess 로 `numpy.ndarray` → MP4. ROS 비의존 코어 | 현행 |
| [src/rdfp/rdfp/recorder/README.md](../src/rdfp/rdfp/recorder/README.md) | `FFMpegMp4Recorder` 모듈 요약. CFR passthrough, CPU/GPU 인코더(`libx264`/`h264_nvenc`/`h264_qsv`/`h264_vaapi`), 생성자 1회 GPU probe | 현행 |

## 6. 세션 / 에피소드 생명주기

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [session/session_control_guide.md](session/session_control_guide.md) | `SessionControlNode` — IDLE → IN_SESSION → IN_EPISODE 상태머신, 서비스/토픽 인터페이스, `TRANSIENT_LOCAL` QoS 규약 | 현행 |
| [session/session_control_client_guide.md](session/session_control_client_guide.md) | `SessionControlClient` — 세션 노드를 호출하는 클라이언트 측 사용법과 제약 | 현행 |

## 7. rosbag2 → 데이터셋 후처리

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [rosbag2/데이터셋 후처리기 설계서.md](rosbag2/데이터셋%20후처리기%20설계서.md) | **후처리 파이프라인 설계 원본**. 에피소드 분할 규칙, DB 스키마, 적재 트랜잭션, 이미지 MP4 분기 구조 | 현행 |
| [rosbag2/데이터셋 후처리기 CLI 사용 설명서.md](rosbag2/데이터셋%20후처리기%20CLI%20사용%20설명서.md) | 후처리 CLI 실행 명령·옵션 레퍼런스, `dataset_config.yaml` 구조 | ⚠️ 구식 |
| [rosbag2/데이터셋 후처리기 실환경 검증 절차.md](rosbag2/데이터셋%20후처리기%20실환경%20검증%20절차.md) | 실 인프라(Docker PostgreSQL·실 rosbag·성능 측정) 검증 runbook | 현행 |
| [rosbag2/rosbag2 운영 방안.md](rosbag2/rosbag2%20운영%20방안.md) | MCAP 기반 장기 저장 운영 설계. 날짜-시간/토픽 기준 조회를 고려한 분할·보관 정책 | 현행 |

> ⚠️ **CLI 사용 설명서 주의**: 문서는 `ros2 run rdfp dataset <subcommand>` 형태로
> 안내하지만, 현재 `setup.py` 의 entry point 는 **독립 최상위 명령**으로 분리되어
> 있다 — `import` / `replay` / `stats` / `list` / `init-db` / `rosbag`.
> 예: `ros2 run rdfp dataset import` → **`ros2 run rdfp import`**.
> 정확한 현행 명령은 [src/rdfp/README.md](../src/rdfp/README.md) 또는
> [CLAUDE.md](../CLAUDE.md) 의 Console Scripts 절을 참고한다.

## 8. 재생 (Replay)

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [replay/replay_mock_stack_guide.md](replay/replay_mock_stack_guide.md) | **`replay_panda_mock.launch.py` 구조·사용법·주의점**. 제외 노드와 그 사유, `replay_arm_path` 3경로(`ee_twist`(기본)/`target_joint_cmds`/`none`) 비교, launch 인자표, ROS 환경 불일치·`start_servo` 누락·배속 역효과·stale stamp 등 함정 12가지 | 현행 |
| [replay/cartesian_path_replay_approaches.md](replay/cartesian_path_replay_approaches.md) | **twist vs pose vs joint 명령 비교**. "위치 명령은 폐루프, 속도 명령은 개루프" 원리와 드리프트 없는 EE 경로 재생 방법 | 현행 |
| [replay/design_chatgpt.md](replay/design_chatgpt.md) | Panda mock joint replay 설계 문서. demo playback(1차) / teaching 동작 재현(2차) 목적 | 설계안 |

> replay **GUI**(`ros2 run rdfp replay_gui`) 전용 문서는 없다. GUI 사용법은
> [src/rdfp/README.md](../src/rdfp/README.md), "위치 초기화" 버튼의 컨트롤러별
> 동작은 [moveit/MoveGroupJgpcClient_UserGuide.md](moveit/MoveGroupJgpcClient_UserGuide.md) 를 본다.

## 9. Teleoperation

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [teleop/README.md](teleop/README.md) | **teleop 문서 진입점**. "하려는 일 → 읽을 문서" 표, 입력 어댑터부터 arm 까지의 전체 그림, 문서별 역할·독자·갱신 빈도, 조용히 실패하는 함정 4종의 참조 위치 | 현행 |
| [teleop/clutch_pedal_guide.md](teleop/clutch_pedal_guide.md) | **USB 풋페달 클러치 가이드**. hold(데드맨) vs toggle 비교, 데드맨을 완성하는 두 조각(페달 하트비트 + `pedal_timeout`), evdev 설치·권한·장치 식별, 파라미터 8종, 안전 동작표, `ClutchClient` API, 문제 해결 5종 | 현행 |
| [teleop/teleop_retarget_node_guide.md](teleop/teleop_retarget_node_guide.md) | **`teleop_retarget` 노드 가이드**. retargeting 이 필요한 이유(관절체계·원점·방향·작업공간 불일치), 클러치 앵커 상대 매핑 수식, 상태 기계(engage 조건·hold 재발행·자동해제 2종), 파라미터 13개 레퍼런스, 튜닝(좌우반전·스케일·손떨림), 문제 해결 5종 | 현행 |
| [teleop/omy_leader_teleop_guide.md](teleop/omy_leader_teleop_guide.md) | **OMY-L100 리더로 팔로워 제어 — 실행 절차서**. 터미널 4개 기동 순서, 체인 단계별 유량 실측표, 끊긴 지점별 원인 진단표, 함정 4가지(클러치 자동해제 / `start_servo` 누락 / 환경 불일치 / 실효 주기), 리더 없이 bag 으로 시험하는 법 | 현행 |
| [teleop/external_input_adapters.md](teleop/external_input_adapters.md) | **외부 입력 어댑터 계약**. 키보드·게임패드·OMY-L100·Isaac Sim 등 어떤 입력이든 로봇을 구동하기 위해 지켜야 할 토픽·타입·QoS·stamp·주기 규약. 두 진입 경로(twist 직접 / pose→retarget), 어겼을 때 **조용히 실패하는** 4가지 함정, 현재 어댑터 목록 | 현행 |
| [teleop/joystick_setup.md](teleop/joystick_setup.md) | 게임패드를 `/joy` 토픽까지 연결하는 절차. Bluetooth 페어링, 권한, 검증 방법 (Ubuntu 22.04 + Humble 기준 검증됨) | 현행 |
| [teleop/leader_follower_mirroring_design.md](teleop/leader_follower_mirroring_design.md) | **이기종** leader-follower 동작 미러링 설계. 관절 체계가 다른 두 로봇 간 EE pose 기반 retargeting. `teleop_mirror.launch.py` 의 근거 문서 | 설계안 |
| [teleop/leader_gripper_mapping_design.md](teleop/leader_gripper_mapping_design.md) | **리더 트리거 → 그리퍼 명령 매핑 설계**. `position` 은 트리거에서 뽑고 `max_effort` 는 고정하는 근거, 50 Hz 입력에 대한 전송 정책(deadband·최소 간격·선점 result), 액션 유지 vs 스트리밍 컨트롤러 교환표, 입력 끊김 시 **유지**(팔과 반대) 근거, 이진 → 연속 단계적 도입, 부록 A(deadband·hysteresis 개념). **전제(트리거 관절 존재)가 미확인** | 설계안 |

> **외부 저장소** — OMY-L100 리더 암 어댑터는 rdfp 밖의 `omy_leader_bridge` 저장소에
> 있다 (ROS 2 Jazzy + `rmw_zenoh` 라 rdfp 와 런타임을 공유할 수 없다). 연결
> 규약은 위 `external_input_adapters.md` 가 정의한다.

## 10. 시뮬레이터 백엔드

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [simulation/gazebo_bringup_guide.md](simulation/gazebo_bringup_guide.md) | Gazebo Fortress(gz-sim 6) 백엔드 브링업. `mock_components` 를 물리 시뮬레이션으로 교체해 실행하는 절차. `panda_gazebo.launch.py` 사용 | 현행 |
| [simulation/multi_simulator_backend_design.md](simulation/multi_simulator_backend_design.md) | mock / Gazebo / Isaac Sim 을 교체 가능한 백엔드로 꽂는 구조 설계. 문서 자체가 "코드 미반영" 명시 | 설계안 |

## 11. 외부 로봇 (OMY-L100)

> `rdfp` 패키지와 직접 관련 없는 별도 로봇(ROBOTIS OMY-L100) 자료다.
> 컨테이너 내부는 **Ubuntu 24.04 + ROS 2 Jazzy** 기준이라 이 워크스페이스의
> Humble 환경과 다르다.

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [OMY-L100_Docker_Setup_and_Usage.md](OMY-L100_Docker_Setup_and_Usage.md) | OMY-L100 소프트웨어 설치·활용. ROBOTIS Docker 컨테이너 기준 | 현행 |
| [OMY-L100_Docker_Setup_and_Usage_with_Volumes.md](OMY-L100_Docker_Setup_and_Usage_with_Volumes.md) | 위 문서의 **볼륨 마운트 버전**. 호스트 작업물을 컨테이너에 연결하는 구성 추가 | 현행 |

## 12. 로봇 트윈 (REST 게이트웨이)

> ROS 2 를 직접 사용할 수 없는 외부 시스템(MDT Platform 등)이 REST 로 로봇 상태를
> 조회하고 명령을 실행하기 위한 게이트웨이다. 트윈 자체는 Python + rclpy 로
> 구현하며 rdfp 의 `MoveGroupClient` / 그리퍼 서비스를 재사용한다.

| 문서 | 내용 | 상태 |
|---|---|:-:|
| [robot_twin/robot_twin_user_guide.md](robot_twin/robot_twin_user_guide.md) | **로봇 트윈 사용 설명서**. 시작/종료 절차(`--config` 절대경로·PGID 종료), REST 인터페이스 전체와 응답 형식(`quality`·ETag/304·배치 조회 보장 수준·오류 코드표), curl/Python 예제 프로그램(재시도·취소·모니터링), 로봇 연동 설정(`move_group_mode` 필수·다른 로봇 붙이기), 제공 상태 변수·연산 목록과 미구현 항목, **자동 에피소드 수집 경로**(`reset_scene` 로 물체 랜덤 배치 + `start_session`/`start_episode`/`stop_episode`/`stop_session` 경계 + 수집 루프 예제, 씬 노드는 mock 계열 launch 가 기본 기동), 자원 락, 새 변수/연산 추가 방법, 운영 주의(인증 없음·E-stop 한계·워치독), 트러블슈팅 표 | 현행 |
| [robot_twin/robot_twin_design.md](robot_twin/robot_twin_design.md) | **로봇 트윈 설계서 (1,589줄)**. `rdfp` 내 `twin/` 서브패키지로 구현하는 단일 Python 프로세스 게이트웨이. 스레드 모델(executor 전용 스레드 + 불변 스냅샷 원자 교체, `externally_spun=True`, `mode='auto'` 금지), 선언적 트윈 정의 YAML, 상태 변수의 품질(`OK`/`STALE`/`NO_DATA`/`SOURCE_UNAVAILABLE`/`ERROR`)·조회 API(envelope·ETag·배치·부분실패)·JSON 직렬화 규약, extern_op 기반 연산(다중 세션·`kind` 고정·자원 락 admission control·`phase: CANCELING`·E-stop·오류 코드), 미채택 요소(`IDLE`/`Idempotency-Key`/인증/TLS)와 그 근거, rdfp 구현 격차. **부록 B(extern_op 확장 목록)는 확장 명세 초안으로 사용 가능** | 설계안 |
| [robot_twin/mcp_server_design.md](robot_twin/mcp_server_design.md) | **로봇 트윈 MCP 서버 설계서**. LLM 에이전트가 대화만으로 로봇 상태를 읽고 연산을 조합하게 하는 경로(`~/development/mdtpy/robot-twin` 의 `mcp_server.py`). **트윈이 자기 능력을 알리기 위한 변경**(`config.py`/`api.py` 에 `description` 필드 + 카탈로그 노출, 설정 YAML 19건 작성)과 그 필요성(`extra='forbid'` 라 YAML 만으로는 기동 실패), 도구 18개의 생성 규칙(카탈로그 자동 생성 5 + 검증 래퍼 교체 2 + 추가 11, 감춤 4), 결정 13건(상태 변수를 resource 가 아닌 tool 로 노출한 이유, 여는 연산만 감춘 비대칭, 작업 상태를 서버가 소유하고 metadata 자동 병합, SDK 2.0 에서 low-level API 를 쓴 경위), 이슈(인증 없는 경로 위임·카탈로그 캐시·동시 호출 무보호·타임아웃 2층) | 현행 |
| [robot_twin/auto_episode_collection_draft.md](robot_twin/auto_episode_collection_draft.md) | **자동 에피소드 수집을 위한 트윈 확장 (임시 초안)**. 물체 위치를 랜덤화하며 스크립트 pick-and-place 를 반복해 학습 데이터를 양산하는 경로. mock/Gazebo/Isaac 은 인터페이스는 통일 가능하나 **mock 은 물체가 움직이지 않아 자동 라벨링이 불가능**하다는 구분, 백엔드별 발행 노드 → `/scene/objects` 단일 토픽 → 트윈(코드 변경 0) 배선, `rdfp_msgs/SceneObjects` 제안(stamp 필수·frame `panda_link0` 고정), 저수준 spawn API 대신 `reset_scene(scene, seed)` + 레시피 config(그리퍼 `targets` 패턴 재사용)와 **실제 배치 pose 를 outputs 로 남겨야 재현된다**는 근거, 최대 구멍인 **트윈의 에피소드 경계 부재**(`start_episode`/`stop_episode` 우선)와 `sessions` 스키마 확장 필요성, 작업 순서·미결정 체크리스트. **작업 후 정식 문서로 옮기고 삭제 예정** | 설계안 |

## 13. AI 어시스턴트용 지침 (CLAUDE.md)

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
| Python 가상환경(uv)과 ROS 2 가 충돌한다 / 새 ROS 2 프로젝트를 만든다 | [environment/python_env_guide.md](environment/python_env_guide.md) |
| Ubuntu 24.04 / ROS 2 Jazzy 로 옮긴다 | [environment/python_env_guide.md](environment/python_env_guide.md) 의 "마이그레이션" 절 |
| 어떤 launch 를 써야 할지 모르겠다 | [src/rdfp/launch/README.md](../src/rdfp/launch/README.md) §9 "파일 사용 가이드" |
| MoveIt 문서 중 뭘 볼지 모르겠다 | [moveit/README.md](moveit/README.md) |
| 로봇을 특정 pose 로 움직인다 | [moveit/MoveGroupClient_UserGuide.md](moveit/MoveGroupClient_UserGuide.md) |
| JGPC(비-JTC) 환경에서 움직인다 | [moveit/MoveGroupJgpcClient_UserGuide.md](moveit/MoveGroupJgpcClient_UserGuide.md) |
| servo 로 실시간 제어한다 | [moveit/servo_client_programmers_guide.md](moveit/servo_client_programmers_guide.md) |
| 그리퍼를 연다/닫는다 | [moveit/GripperControlNode_Guide.md](moveit/GripperControlNode_Guide.md) |
| 그리퍼가 안 움직인다 / 상태가 안 온다 | [moveit/gripper_action_server_notes.md](moveit/gripper_action_server_notes.md) |
| 카메라 영상을 토픽으로 낸다 | [camera/camera_node_guide.md](camera/camera_node_guide.md) |
| 영상을 MP4 로 녹화한다 | 서비스 제어 → [recorder/image_recorder_node_guide.md](recorder/image_recorder_node_guide.md) / 세션 자동 → [recorder/rdfp_image_recorder_node_guide.md](recorder/rdfp_image_recorder_node_guide.md) |
| 세션·에피소드를 시작/종료한다 | [session/session_control_guide.md](session/session_control_guide.md) |
| rosbag 을 DB 에 적재한다 | [rosbag2/데이터셋 후처리기 CLI 사용 설명서.md](rosbag2/데이터셋%20후처리기%20CLI%20사용%20설명서.md) (⚠️ 명령 형태는 구식) |
| 적재 파이프라인 내부를 이해한다 | [rosbag2/데이터셋 후처리기 설계서.md](rosbag2/데이터셋%20후처리기%20설계서.md) |
| 적재된 에피소드를 재생한다 | [replay/replay_mock_stack_guide.md](replay/replay_mock_stack_guide.md) |
| 조이스틱을 연결한다 | [teleop/joystick_setup.md](teleop/joystick_setup.md) |
| OMY-L100 리더로 로봇을 움직인다 | [teleop/omy_leader_teleop_guide.md](teleop/omy_leader_teleop_guide.md) |
| 새 입력 장치(게임패드·시뮬레이터 등)를 붙인다 | [teleop/external_input_adapters.md](teleop/external_input_adapters.md) |
| 리더-팔로워 매핑을 튜닝한다 (스케일·정렬·필터) | [teleop/teleop_retarget_node_guide.md](teleop/teleop_retarget_node_guide.md) |
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
