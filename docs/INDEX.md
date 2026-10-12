# 문서 인덱스

`rdfp_ws` 의 markdown 문서 색인이다. 찾는 내용이 어느 문서에 있는지 여기서 먼저 확인한다.
**요약은 한 줄이다** — 무엇을 담았는지는 각 문서 머리의 상태 · 요약 블록이 정본이다(색인의 긴 요약이
두 번째 정본이 되어 낡았던 것이 2026-09-21 재편의 이유다).

> **전체 구조를 먼저 알고 싶다면** [rdfp_framework_design.md](rdfp_framework_design.md) — 세 서브시스템과 그 사이의
> 인터페이스 계약을 다루는 최상위 문서다. **학습 데이터를 만드는 전 과정(생성 → 수집 → 적재 → export → 학습)** 은
> [collect/학습_데이터_전_과정_지도.md](collect/학습_데이터_전_과정_지도.md) 가 한 화면에 잇는다.

- **현행 문서 62개** (이 색인 제외) + `archive/` 보관 21개. 경로는 `docs/` 기준이다.
- **종류** — `절차`(사람이 실행하며 읽는다) · `사양`(코드와 1:1 인 계약 · 레퍼런스) · `설계`(왜 그렇게 했나) ·
  `현황`(무엇이 남았나 · 정하지 않았나) · `설계안`(코드 미반영) · `기록`(작업 기록 · 교훈). `archive/` 는 전부 `이력`(갱신하지 않는다).
- 폴더는 코드 계층을 따른다 — `control/`(robot_control) · `collect/`(rdfp) · `learning/`(학습 데이터) · `twin/`(robot_twin) ·
  `teleop/` · `backends/`(시뮬레이터 · 실기). 링크 검사는 `scripts/check_doc_links.sh`.

---

## 목차

- [루트](#루트)
- [control/ — 제어 계층](#control--제어-계층)
- [collect/ — 수집 계층](#collect--수집-계층)
- [learning/ — 학습 데이터](#learning--학습-데이터)
- [twin/ — REST 게이트웨이 · MCP](#twin--rest-게이트웨이--mcp)
- [teleop/](#teleop)
- [backends/ — 시뮬레이터 · 실기](#backends--시뮬레이터--실기)
- [archive/ — 보관](#archive--보관)
- [코드 옆 문서](#코드-옆-문서)
- [하고 싶은 것 → 문서](#하고-싶은-것--문서)

---

## 루트

| 문서 | 종류 | 한 줄 |
|---|:-:|---|
| [rdfp_framework_design.md](rdfp_framework_design.md) | 설계 | ⭐ 최상위 구조 — 세 서브시스템 · 표준 인터페이스 계약 · 2단 저장 · 에피소드 생명주기 · 패키지 분리(§7) · 현재 상태(§8) |
| [topic_naming_contract.md](topic_naming_contract.md) | 사양 | ⭐ 토픽 이름 규약 — 논리 채널별 정규 이름(루트 상대), arm 명령이 규약 밖인 이유, remap 은 백엔드 경계에서만, 2단계 네임스페이스(미구현) |
| [python_env_guide.md](python_env_guide.md) | 절차 | ROS 2 와 uv venv 가 한 머신에서 공존하는 법 — venv 가 깨지는 세 원인, 옵트인 셸, 24.04/Jazzy 이전 |

## control/ — 제어 계층

| 문서 | 종류 | 한 줄 |
|---|:-:|---|
| [control/README.md](control/README.md) | — | 진입점 — MoveIt · 그리퍼 · 카메라 · scene 의 "하려는 일 → 문서" 표와 전체 그림 |
| [control/move_group_client_user_guide.md](control/move_group_client_user_guide.md) | 사양 | `MoveGroupClient` 공통 인터페이스 + JTC 구현 — 팩토리 · Cartesian · named target · `move_to_joints` · 비동기 · `cancel` · threading (JGPC 는 §1.2 대조표뿐) |
| [control/move_group_jgpc_client_user_guide.md](control/move_group_jgpc_client_user_guide.md) | 사양 | JGPC 구현 — 계획 · 스트리밍 분리, `TrajectoryStreamer`, `publish_rate` 보간, `tolerance`, 증상표(§9.2) — **JGPC 전용 내용은 전부 여기** |
| [control/servo_client_user_guide.md](control/servo_client_user_guide.md) | 사양 | `ServoClient` — `/servo_node` 시작 · 정지 · 상태, `servo_auto_start_node` |
| [control/gripper_node_design.md](control/gripper_node_design.md) | 사양 | ⭐ 그리퍼 계약 — 명령은 심볼 · 관측은 물리량, `at_goal` 판정식, 백엔드별 실현 가능성 |
| [control/gripper_action_node_guide.md](control/gripper_action_node_guide.md) | 사양 | `GripperActionNode`(mock · Isaac) — 파라미터 · `stalled` · 단위 변환 · 트러블슈팅, §9 에 `/panda_hand_controller/gripper_cmd` 액션 서버 계층 노트 |
| [control/robotiq_2f_gripper_node_guide.md](control/robotiq_2f_gripper_node_guide.md) | 사양 | `Robotiq2FGripperNode`(펑션베이) — 6축 펼치기, 잔차 · 토크 판정, `width` 는 NaN |
| [control/camera_nodes_guide.md](control/camera_nodes_guide.md) | 사양 | 카메라 노드 셋 중 고르기(§0) · `camera_node`(§A) · `image_capture_node`(§B) |
| [control/image_viewer_node_guide.md](control/image_viewer_node_guide.md) | 사양 | `ImageViewerNode` — 이미지 토픽을 창에 |
| [control/opencv_camera_guide.md](control/opencv_camera_guide.md) | 사양 | `OpenCvCamera` — 장치 · 파일 · 스트림 통합 래퍼 |
| [control/compressed_image_pipeline_design.md](control/compressed_image_pipeline_design.md) | 설계 | 시뮬레이터가 JPEG 으로 발행할 때 뷰어 · 레코더를 잇는 법 — `camera.source` · `enable_camera` · 36배 대역폭 |
| [control/scene_objects_guide.md](control/scene_objects_guide.md) | 사양 | ⭐ `/scene/objects` · `/scene/reset` 계약 — 필드 · xyzw · 백엔드별 발행 노드 · 조작 대상만 싣는 이유 · planning scene 에 안 넣는 이유 |

## collect/ — 수집 계층

| 문서 | 종류 | 한 줄 |
|---|:-:|---|
| [collect/학습_데이터_전_과정_지도.md](collect/학습_데이터_전_과정_지도.md) | 절차 | ⭐ 생성 → 수집 → 적재 → export → 학습 여섯 단계 — 명령 한 화면, 단계별 정본, 단계 *사이*의 함정 |
| [collect/수집_계층_얹기_절차서.md](collect/수집_계층_얹기_절차서.md) | 절차 | ⭐ `rdfp_collect` — 백엔드별 기동, 에피소드 한 번 수집, rosbag · 적재로 잇기, 함정, 인자 22개 |
| [collect/session_control_guide.md](collect/session_control_guide.md) | 사양 | ⭐ `session_control_node` + CLI + `SessionControlClient` — 상태 머신 · 서비스 6종 · TRANSIENT_LOCAL |
| [collect/session_camera_nodes_guide.md](collect/session_camera_nodes_guide.md) | 사양 | `rdfp_camera_node`(§A) · `rdfp_image_viewer_node`(§B) — `/session` 을 따르는 카메라 · 뷰어 |
| [collect/rdfp_image_recorder_node_guide.md](collect/rdfp_image_recorder_node_guide.md) | 사양 | `rdfp_image_recorder` — `/session` 자동 녹화, 타임스탬프 분절, sidecar · metadata(`episode_key`), `Image` · `CompressedImage` |
| [collect/image_recorder_node_guide.md](collect/image_recorder_node_guide.md) | 사양 | `image_recorder_node` — 서비스 구동 레코더 (**보조** — launch 없음, 압축 미지원) |
| [collect/ffmpeg_mp4_recorder_guide.md](collect/ffmpeg_mp4_recorder_guide.md) | 사양 | `FFMpegMp4Recorder` — ffmpeg subprocess MP4 코어 (ROS 비의존) |
| [collect/rosbag_cli_user_guide.md](collect/rosbag_cli_user_guide.md) | 절차 | ⭐ `rosbag` CLI — `list-topics` · `list-episodes` · `export` · `delete` · `clear`, 시간 범위는 log time 기준 |
| [collect/dataset_postproc_user_guide.md](collect/dataset_postproc_user_guide.md) | 절차 | `init-db` · `import` · `stats` · `list` · `replay` 운영, `dataset_config.yaml`, 삭제 절차(§8.6) |
| [collect/dataset_ingest_design.md](collect/dataset_ingest_design.md) | 설계 | 적재 파이프라인 — rosbag 저장 규칙(§0), 에피소드 분할, 스키마 14 테이블(§5), 트랜잭션, 레코더 mp4 등록, sim 시계 겹침 함정 |
| [collect/replay_mock_stack_guide.md](collect/replay_mock_stack_guide.md) | 절차 | `replay_panda_mock` — 왜 어떤 명령 타입인가(§0), 구조, `replay_arm_path`, 함정 12가지 |

## learning/ — 학습 데이터

| 문서 | 종류 | 한 줄 |
|---|:-:|---|
| [learning/결정_기록.md](learning/결정_기록.md) | 설계 | ⭐ 결정 38건을 한 표로 — 목표 · action 채널 · 샘플 · 형식 · 운영, 각각의 **이유**와 보관본 근거, 폐기한 안 |
| [learning/학습_샘플_사양.md](learning/학습_샘플_사양.md) | 사양 | ⭐ 채널(R1 · R2 메시지) · 명령 체인 L0~L4 · 시간 정렬 · 샘플 구조 · `plan(τ)` · 비교 기준 · 배치 규약 · 실측 상수 |
| [learning/저장_형식.md](learning/저장_형식.md) | 사양 | export 트리 — `metadata.json` · `episodes.jsonl` · `.npz`, chunk 미리 계산, LeRobot 형식을 안 쓰는 이유(§5.3), 인코딩 측정(부록 C) |
| [learning/추론기_배치_목표와_설계_이슈.md](learning/추론기_배치_목표와_설계_이슈.md) | 설계 | ⭐ 목표 — 두 줄(isaac / 펑션베이) 분리, 1차 과제(isaac 쌓기) 정의와 알려진 위험, 2단계(실기) 설계 이슈 여섯 |
| [learning/추론기_배치_스택_구성.md](learning/추론기_배치_스택_구성.md) | 설계안 | isaac 1단계 추론기를 돌릴 스택 — 관측 · 어댑터(P2) · 추론 노드(P1)의 걸림돌 셋(후처리기 누락 · 앙상블 시각 문제 · 실행 환경) · 조건 대조 · 평가 루프 |
| [learning/미구현_목록.md](learning/미구현_목록.md) | 현황 | **1차 과제 진척 A~E(§0)**, 남은 항목 R · I · O · P · C 와 코드 확인 근거 (구현된 것은 각 절의 「구현됨」 표) |
| [learning/열린_결정.md](learning/열린_결정.md) | 현황 | 아직 정하지 않은 것 — 통합표 O-1~O-22, 보류(데카르트) C-1~C-7, R4 · Q13′ · R8 검토 |
| [learning/scene_objects_observation_decision.md](learning/scene_objects_observation_decision.md) | 설계 | `scene_objects` 를 학습 관측에 넣을 것인가 — 적재 O · 관측 X 와 재검토 트리거 |

## twin/ — REST 게이트웨이 · MCP

| 문서 | 종류 | 한 줄 |
|---|:-:|---|
| [twin/robot_twin_user_guide.md](twin/robot_twin_user_guide.md) | 절차 · 사양 | 트윈 사용 설명서 — 기동, REST 전체, 변수 · 연산, 자동 에피소드 수집, 확장, 운영 주의 |
| [twin/robot_twin_design.md](twin/robot_twin_design.md) | 설계 | 트윈 설계 근거 — 스레드 모델 · 스냅샷 · 세션 · 자원 락 · 오류 코드 · 결정 이력 |
| [twin/mcp_server_design.md](twin/mcp_server_design.md) | 설계 | MCP 서버(`robot-twin-client`) — 카탈로그 노출, 도구 구성, 결정 13건 |

## teleop/

| 문서 | 종류 | 한 줄 |
|---|:-:|---|
| [teleop/README.md](teleop/README.md) | — | 진입점 — 입력 어댑터부터 arm 까지의 그림, 문서별 역할 |
| [teleop/external_input_adapters.md](teleop/external_input_adapters.md) | 사양 | 외부 입력 어댑터 계약 — 토픽 · 타입 · QoS · stamp · 주기 |
| [teleop/omy_leader_teleop_guide.md](teleop/omy_leader_teleop_guide.md) | 절차 | OMY-L100 리더로 팔로워 제어 — 컨테이너(터미널 0)부터 미러링까지, 진단표 |
| [teleop/omy_l100_docker_setup.md](teleop/omy_l100_docker_setup.md) | 절차 | OMY-L100 ROBOTIS Docker 설치 · 볼륨 · 장치 매핑 (Jazzy 컨테이너) |
| [teleop/teleop_retarget_node_guide.md](teleop/teleop_retarget_node_guide.md) | 사양 | `teleop_retarget` — 클러치 앵커 상대 매핑, 상태 기계, 파라미터 13개, 튜닝 |
| [teleop/clutch_pedal_guide.md](teleop/clutch_pedal_guide.md) | 절차 · 사양 | USB 풋페달 클러치 — hold 데드맨, `pedal_timeout`, evdev, `ClutchClient` |
| [teleop/key_hold_user_guide.md](teleop/key_hold_user_guide.md) | 사양 | `HoldKeyTracker` — 터미널이 '뗐다' 를 안 알려 주는 문제, TTL, `xset r rate` |
| [teleop/joystick_setup.md](teleop/joystick_setup.md) | 절차 | 게임패드를 `/joy` 까지 |
| [teleop/leader_follower_mirroring_design.md](teleop/leader_follower_mirroring_design.md) | 설계 | 이기종 leader-follower EE retargeting — `teleop_mirror` 의 근거 (로드맵 1~4 구현) |
| [teleop/servo_vs_planned_motion.md](teleop/servo_vs_planned_motion.md) | 설계 · 기록 | 물체를 든 채 servo 로 올리면 옆으로 샌다 — 되읽기(래칫) 원인과 `commanded` + `open_loop_control` 해결, 재현 절차 |

## backends/ — 시뮬레이터 · 실기

| 문서 | 종류 | 한 줄 |
|---|:-:|---|
| [backends/README.md](backends/README.md) | — | 진입점 — 백엔드 비교표, 프로파일, launch 를 하나로 합치지 않은 이유, 공통 함정 |
| [backends/isaac/isaac_bringup_runbook.md](backends/isaac/isaac_bringup_runbook.md) | 절차 | ⭐ Isaac + 스택 + 트윈 + 수집 + 작업 수행 전 과정 명령(§2), Script Editor 목록, 함정 |
| [backends/isaac/isaac_backend.md](backends/isaac/isaac_backend.md) | 설계 | Isaac 백엔드 — 결정(D1″ `TopicBasedSystem`), 배포, 골격 · 토픽 계약, 운용 근거, 함정 사전(§7), 열린 결정 |
| [backends/isaac/isaac_windows_setup.md](backends/isaac/isaac_windows_setup.md) | 절차 | 새 Windows 11 + WSL2 머신 구축 8단계, DDS 프로파일(부록) |
| [backends/isaac/isaac_scene_reset.md](backends/isaac/isaac_scene_reset.md) | 사양 | Isaac 의 `/scene/reset` — `simulation_interfaces` 경유, `result=1` 함정, 선행 조건 |
| [backends/functionbay/펑션베이_백엔드_설계.md](backends/functionbay/펑션베이_백엔드_설계.md) | 설계 | 토픽 연동형 백엔드 — `joint_state_fusion` · `readiness_gate` · `publish_rate` · 그리퍼 채널(씬별 규약) |
| [backends/functionbay/펑션베이_시뮬레이터_사용법.md](backends/functionbay/펑션베이_시뮬레이터_사용법.md) | 절차 | 띄우는 순서 — 솔버 → 시뮬레이터(`-Scene LAB_ETRI -Model <씬>_ms`) → `ros_tcp_endpoint` → **`/scene/start`**(저절로 시작하지 않는다) → 로봇 스택 → **차트 종료 대기**(`fb_wait_chart`), 정지 토픽, 씬 바꾸기 |
| [backends/functionbay/펑션베이_인수_체크리스트.md](backends/functionbay/펑션베이_인수_체크리스트.md) | 절차 | ⭐ 새 빌드마다 무엇이 바뀌었는지 가리는 절차 — A · B · C 항목의 확인 방법 · 판정 기준, 측정 함정 17가지. **결과는 저장소 루트 `checklist_results/`** |
| [backends/functionbay/펑션베이_요청_목록.md](backends/functionbay/펑션베이_요청_목록.md) | 현황 | 벤더 요청 상태 — **2026-09-30: A-1 · A-5 가 `r2` 에서 해결, B-11 종결**(§0.4). 살아 있는 7건 앞에, 닫힌 것은 부록 E |
| [backends/functionbay/펑션베이_남은_작업.md](backends/functionbay/펑션베이_남은_작업.md) | 현황 | ⭐ **맨 앞 "지금 남은 작업" 7건**(15 수집용 씬 결정 · 16 `/data` 권한 · 4 링크 메시 …) · 원장(§8) · 재현 절차(§9) · 함정(§10) |
| [backends/functionbay/펑션베이_중력_보상.md](backends/functionbay/펑션베이_중력_보상.md) | 설계 · 기록 | 처짐을 `−τ_g(q)/Kp` 로 상쇄 — 증상 · 원인 · 실측 · 벤더 반영 시 되돌리는 법 |
| [backends/functionbay/grasp_center_frame.md](backends/functionbay/grasp_center_frame.md) | 사양 · 기록 | `grasp_center` 는 손끝이 아니다(−40 mm) — 파지 높이 산정, 데드밴드, 정착 대기 |
| [backends/functionbay/peg_in_hole_via_mcp_plan.md](backends/functionbay/peg_in_hole_via_mcp_plan.md) | 기록 | peg-in-hole 을 트윈 · MCP 로 — 7단계 완료, 실패 기록(지령 주기 · 파지 판정 지터) |
| [backends/gazebo_bringup_guide.md](backends/gazebo_bringup_guide.md) | 절차 | Gazebo Fortress 브링업 — **미검증 · 보류** (`ros_gz_sim` 미설치, 프로파일 없음) |
| [backends/franka_panda_bringup_prep.md](backends/franka_panda_bringup_prep.md) | 설계안 | 실기 Franka Panda 착수 조건 — 로봇 정보 · franka_ros2 · PREEMPT_RT · 코드 변경 4건 |

## archive/ — 보관

갱신하지 않는 이력 문서다. 목록과 규칙은 [archive/README.md](archive/README.md). 재편 직전 원본 전체(PDF 포함)는
저장소 밖 `~/development/ros/rdfp_docs_backup_20260921/` 에 있다.

| 무리 | 문서 |
|---|---|
| 학습 데이터 설계 원문 (9) | 수집 설계 · MoveIt 명령 체인 · 좌표 공간 비교 · ACT 샘플 구성 · R1 · R2 · R4 · Q13 · R8 |
| 펑션베이 (5) | 미해결 작업 원문 · 인수 체크리스트 이력 20260917 · 20261001 · 벤더 메일 2 |
| Isaac (2) | Phase 0~11 단계 계획 · 멀티 시뮬레이터 backend 설계안 |
| 1차 과제 진척 기록 (1) | isaac 쌓기 과제의 A0~D4 실측 · 경위와 옛 결정 D1~D8 (2026-09-19~20) |
| 그 밖 (4) | recorder 최초 요구사항 · 후처리기 실환경 검증 runbook · rosbag2 운영 방안 · 리더 트리거 그리퍼 매핑 |
| 재편 계획 | [archive/문서_재편_계획.md](archive/문서_재편_계획.md) |

## 코드 옆 문서

| 문서 | 한 줄 |
|---|---|
| [../README.md](../README.md) | 워크스페이스 소개 — 네 패키지, 빌드, launch · console script 개요 |
| [../CLAUDE.md](../CLAUDE.md) | Claude Code 지침 — 빌드, 아키텍처, **비자명한 동작** 목록(디버깅 때 유용) |
| [../src/robot_control/README.md](../src/robot_control/README.md) · [launch/README.md](../src/robot_control/launch/README.md) · [config/backends/README.md](../src/robot_control/config/backends/README.md) | 제어 계층 패키지, 제어 launch + helper 인벤토리, 백엔드 프로파일 스키마 |
| [../src/rdfp/README.md](../src/rdfp/README.md) · [launch/README.md](../src/rdfp/launch/README.md) · [rdfp/recorder/README.md](../src/rdfp/rdfp/recorder/README.md) · [CLAUDE.md](../src/rdfp/CLAUDE.md) | 수집 패키지 walkthrough, 수집 launch + YAML↔인자 표, 레코더 코어 요약, 패키지 지침 |
| [../src/robot_twin/README.md](../src/robot_twin/README.md) · [../src/rdfp_msgs/README.md](../src/rdfp_msgs/README.md) · [../src/rdfp_msgs/CLAUDE.md](../src/rdfp_msgs/CLAUDE.md) | 트윈 · IDL 패키지 |
| [../docker/README.md](../docker/README.md) · [../data/README.md](../data/README.md) | Docker 실행 구성 · 검증용 rosbag(`ee_pose_bag`) |
| `~/development/rdfp/rdfp-train/README.md` · `docs/학습_실행_절차.md` | 학습 저장소(ROS 밖) — 환경, D4 절차, 다른 머신으로 옮기는 법 |
| `~/development/mdtpy/robot-twin-client/` | 트윈 클라이언트 · MCP 서버 · `robot-twin-collect` |

---

## 하고 싶은 것 → 문서

| 하고 싶은 것 | 볼 문서 |
|---|---|
| 처음 빌드하고 실행해 본다 | [../README.md](../README.md) → [../src/rdfp/README.md](../src/rdfp/README.md) |
| 학습 데이터 없이 로봇 제어 스택만 쓴다 | [../src/robot_control/README.md](../src/robot_control/README.md) |
| 패키지가 왜 넷으로 나뉘어 있는지 | [rdfp_framework_design.md](rdfp_framework_design.md) §7.6 |
| Python venv 와 ROS 2 가 충돌한다 / 24.04 로 옮긴다 | [python_env_guide.md](python_env_guide.md) |
| 어떤 launch 를 써야 할지 모르겠다 | 제어 → [../src/robot_control/launch/README.md](../src/robot_control/launch/README.md) / 수집 → [collect/수집_계층_얹기_절차서.md](collect/수집_계층_얹기_절차서.md) |
| 로봇을 특정 pose 로 움직인다 / JGPC 에서 / servo 로 | [control/README.md](control/README.md) |
| 그리퍼를 코드에서 쓴다 · 상태가 안 온다 · 노드를 띄운다 | [control/gripper_node_design.md](control/gripper_node_design.md) §5 · [control/gripper_action_node_guide.md](control/gripper_action_node_guide.md) |
| 카메라 노드 셋 중 무엇을 쓸지 | [control/camera_nodes_guide.md](control/camera_nodes_guide.md) §0 |
| scene 에 물체를 놓거나 좌표를 읽는다 | [control/scene_objects_guide.md](control/scene_objects_guide.md) |
| **Isaac 을 띄우고 수집까지 한 번에** | [backends/isaac/isaac_bringup_runbook.md](backends/isaac/isaac_bringup_runbook.md) §2 |
| **학습 데이터를 만드는 전 과정을 한눈에** | [collect/학습_데이터_전_과정_지도.md](collect/학습_데이터_전_과정_지도.md) |
| 세션 · 에피소드를 시작 · 종료한다 | [collect/session_control_guide.md](collect/session_control_guide.md) |
| 영상을 MP4 로 녹화한다 | [collect/rdfp_image_recorder_node_guide.md](collect/rdfp_image_recorder_node_guide.md) |
| rosbag 의 에피소드를 보거나 내보내거나 지운다 | [collect/rosbag_cli_user_guide.md](collect/rosbag_cli_user_guide.md) |
| rosbag 을 DB 에 적재한다 / 수집 데이터를 지운다 | [collect/dataset_postproc_user_guide.md](collect/dataset_postproc_user_guide.md) |
| 적재 파이프라인 내부 · 스키마 | [collect/dataset_ingest_design.md](collect/dataset_ingest_design.md) |
| 적재된 에피소드를 재생한다 | [collect/replay_mock_stack_guide.md](collect/replay_mock_stack_guide.md) |
| **왜 관절-계획 · ACT · `.npz` 인가** | [learning/결정_기록.md](learning/결정_기록.md) |
| 학습 샘플이 정확히 무엇인가 / R1 · R2 필드 / 배치 규약 | [learning/학습_샘플_사양.md](learning/학습_샘플_사양.md) |
| export 파일의 모양 | [learning/저장_형식.md](learning/저장_형식.md) §6 |
| 무엇이 남았나 · 무엇을 정해야 하나 | [learning/미구현_목록.md](learning/미구현_목록.md) · [learning/열린_결정.md](learning/열린_결정.md) |
| 학습을 돌린다 | `rdfp-train/docs/학습_실행_절차.md` |
| 학습한 추론기를 isaac 에서 돌린다 (스택 구성 · P1 · P2 · P3) | [learning/추론기_배치_스택_구성.md](learning/추론기_배치_스택_구성.md) |
| 외부 시스템에 REST 로 로봇을 노출한다 | [twin/robot_twin_user_guide.md](twin/robot_twin_user_guide.md) |
| OMY-L100 리더로 로봇을 움직인다 / 새 입력 장치를 붙인다 | [teleop/omy_leader_teleop_guide.md](teleop/omy_leader_teleop_guide.md) · [teleop/external_input_adapters.md](teleop/external_input_adapters.md) |
| 펑션베이로 돌린다 / 새 빌드를 받았다 / 벤더에 요청한다 | [backends/functionbay/펑션베이_시뮬레이터_사용법.md](backends/functionbay/펑션베이_시뮬레이터_사용법.md) · [backends/functionbay/펑션베이_백엔드_설계.md](backends/functionbay/펑션베이_백엔드_설계.md) · [펑션베이_인수_체크리스트.md](backends/functionbay/펑션베이_인수_체크리스트.md) · [펑션베이_요청_목록.md](backends/functionbay/펑션베이_요청_목록.md) |
| 토픽 이름을 정하거나 바꾼다 | [topic_naming_contract.md](topic_naming_contract.md) |
| 이상 동작을 디버깅한다 | [../CLAUDE.md](../CLAUDE.md) 「Important non-obvious behaviors」 |
| 옛 설계 · 실측 원문을 찾는다 | [archive/README.md](archive/README.md) |
