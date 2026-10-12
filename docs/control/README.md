# 제어 계층 문서 안내 — MoveIt · 그리퍼 · 카메라 · scene

`robot_control` 패키지(제어 계층)의 문서 진입점이다. **하려는 일부터 고른다.**
수집 계층(세션 · 레코더 · 적재)은 [../collect/](../collect/), 시뮬레이터·실기 연동은
[../backends/](../backends/), teleop 은 [../teleop/](../teleop/) 에 있다.

| 하려는 일 | 문서 |
|---|---|
| **MoveIt** — 로봇을 계획해서 움직인다 (Cartesian / named target / joint 목표값) — 공통 인터페이스 + JTC | [move_group_client_user_guide.md](move_group_client_user_guide.md) |
| JGPC 스택(`panda_mock_jgpc` · 펑션베이)에서 움직인다 / 명령을 직접 스트리밍한다 — **JGPC 전용 내용은 전부 여기** | [move_group_jgpc_client_user_guide.md](move_group_jgpc_client_user_guide.md) |
| 텔레옵·replay 에서 servo 를 켜고 상태를 확인한다 | [servo_client_user_guide.md](servo_client_user_guide.md) |
| **그리퍼** — 코드에서 쓴다 (명령 발행 · 상태 구독 · 성공 판정) / 필드의 뜻 (`width` / `stalled` / `at_goal`) | [gripper_node_design.md](gripper_node_design.md) **§5** / §2 |
| mock · Gazebo · Isaac 에서 그리퍼 노드를 띄우고 파라미터를 맞춘다 / 액션 서버 계층을 파고든다 | [gripper_action_node_guide.md](gripper_action_node_guide.md) (§9 가 액션 서버 노트) |
| 펑션베이(Robotiq 2F)에서 같은 일을 한다 | [robotiq_2f_gripper_node_guide.md](robotiq_2f_gripper_node_guide.md) |
| **카메라** — 노드 셋 중 무엇을 쓸지 고른다 / `camera_node` · `image_capture_node` 를 띄운다 | [camera_nodes_guide.md](camera_nodes_guide.md) §0 / §A · §B |
| 이미지 토픽을 창에 띄운다 | [image_viewer_node_guide.md](image_viewer_node_guide.md) |
| `OpenCvCamera` 로 장치·파일·스트림을 연다 | [opencv_camera_guide.md](opencv_camera_guide.md) |
| 시뮬레이터가 JPEG 으로 발행할 때 뷰어·레코더를 잇는다 | [compressed_image_pipeline_design.md](compressed_image_pipeline_design.md) |
| **scene** — 물체를 놓거나 물체 좌표를 읽는다 (`/scene/objects` · `/scene/reset` 계약) | [scene_objects_guide.md](scene_objects_guide.md) |

## MoveIt — 전체 그림

두 진입점이 병렬로 존재한다. 서로 다른 MoveIt 인터페이스를 쓰므로 목적에 따라 고른다.

```text
[계획 + 실행]                            [실시간 서보]

 MoveGroupClient                          ServoClient
  ├ plan_* / move_to_* / follow_*          ├ start/stop/pause
  └ create_move_group_client()             └ check_status()
        │                                        │
        │  ← move_group_client_user_guide           │  ← servo_client_user_guide
        ▼                                        ▼
  /move_action                              /servo_node/*
  /compute_cartesian_path                    start_servo 등
        │                                        │
        │  JTC: /panda_arm_controller/joint_trajectory
        │  JGPC: /panda_arm_controller/commands   ← move_group_jgpc_client_user_guide
        ▼                                        ▼
                    ros2_control 컨트롤러 → hardware
```

- **처음이라면** `move_group_client_user_guide` 의 「개요」·「Quick Start」만 본다.
  나머지는 필요할 때 찾아 읽는 레퍼런스다.
- **JGPC 스택을 쓴다면** 위 문서의 §1.2 「구현별 API 가용성」 표로 무엇이 되고 안 되는지 확인한 뒤
  `move_group_jgpc_client_user_guide` 로 넘어간다. **경계는 그 표 하나다** — 공통 가이드에는 JGPC 전용
  API · 스트리밍 · `publish_rate` 설명이 없고(포인터뿐), JGPC 가이드는 base API 를 되풀이하지 않는다.

## 그리퍼 — 계약 하나, 구현 둘

상위 계층(teleop · 트윈 · 재생 · 데이터셋)은 계약만 알면 되고, 구현은 **액션 서버가
있느냐**로 갈린다. 2026-09-02 에 MoveIt 문서에서 분리했다 — MoveIt 이 아니라 ros2_control·
시뮬레이터 계층이고, 구현 둘 중 하나(`Robotiq2FGripperNode`)는 MoveIt 을 아예 쓰지 않는다.

```text
              gripper_cmds  (rdfp_msgs/GripperCommand — 심볼 open/close/grasp)
              gripper_states (rdfp_msgs/GripperState  — width · stalled · at_goal)
                                   ▲
                    GripperNode 계약  ← gripper_node_design
                                   │
              ┌────────────────────┴────────────────────┐
     GripperActionNode                         Robotiq2FGripperNode
     ← gripper_action_node_guide                 ← robotiq_2f_gripper_node_guide
              │                                          │
              ▼                                          ▼
   /panda_hand_controller/gripper_cmd          /input/gripper_joint  (규약은 씬마다 — 아래 주)
   (control_msgs/GripperCommand 액션)          /output/gripper_joint (축 수는 씬마다)
              │  ← gripper_action_node_guide §9            │
              ▼                                            ▼
     ros2_control 컨트롤러                        시뮬레이터 (컨트롤러 없음)
     mock · Gazebo · Isaac                        펑션베이
```

**세 가지만 기억한다.**

1. **명령은 심볼, 관측은 물리량.** 숫자는 그리퍼에 종속이라 명령에 싣지 않는다
   (`gripper_node_design` §1.1).
2. **성공 판정은 `at_goal` 하나.** `goal` 마다 다른 판정식은 노드가 이미 적용했다 (§2.3).
3. **`/joint_states` 의 손가락 관절로 폭을 읽지 않는다.** 펑션베이는 TF 성립용 고정값을
   주입해 "항상 열려 있다"고 거짓말한다 (§1.2, [토픽 규약](../topic_naming_contract.md) §2.2).

> ⚠️ **펑션베이의 그리퍼 명령 규약은 씬마다 다르다** — `t1`(Crisp)은 6축·라디안·
> 위치 나열, `r2`(RecurDyn)은 2축·**도**·관절당 `(p, v, f)` 다. 틀리면 에러가 아니라
> "거의 안 움직인다"(1.745%)로 나타난다. 규약은 `config/functionbay_gripper_*.json`
> 이 갖고 `config/backends/functionbay.yaml` 의 `gripper.profile_file` 이 가리킨다.
> 근거: [../backends/functionbay/펑션베이_백엔드_설계.md](../backends/functionbay/펑션베이_백엔드_설계.md) §6.1a.
> **펑션베이에서 물체를 잡으러 간다면** [../backends/functionbay/grasp_center_frame.md](../backends/functionbay/grasp_center_frame.md) 를
> 먼저 본다 — `/ee_pose` 의 `grasp_center` 는 손끝이 아니라 파지 중심이고 실제 손끝은 약 40 mm 아래다.

## 카메라 — 노드 셋

| | `camera_node` | `image_capture_node` | `rdfp_camera_node` |
|---|---|---|---|
| 패키지 | `robot_control` | `robot_control` | **`rdfp`** (수집 계층) |
| 세션에 맞춰 켜고 끄는가 | ❌ | ❌ | ✅ |
| 내보내는 것 | raw `Image` (+ 선택적 JPEG) | JPEG `CompressedImage` 전용 | raw `Image` |
| 문서 | [camera_nodes_guide.md](camera_nodes_guide.md) §A | 같은 문서 §B | [../collect/session_camera_nodes_guide.md](../collect/session_camera_nodes_guide.md) |

고르는 기준과 `ros2 run` 패키지 함정은 [camera_nodes_guide.md](camera_nodes_guide.md) §0 에 있다.

## 여기 없는 것

MoveIt 스택과 함께 뜨지만 **쓰이는 문맥이 다른** 모듈은 다른 폴더에 문서가 있다.

| 모듈 · 주제 | 문서 |
|---|---|
| `ee_pose_publisher`, `ee_twist_publisher`, `target_joint_cmds_publisher` / `_executor`, `servo_auto_start_node` (launch 연동) | [../collect/replay_mock_stack_guide.md](../collect/replay_mock_stack_guide.md), [../teleop/](../teleop/) |
| `TrajectoryStreamer` | [move_group_jgpc_client_user_guide.md](move_group_jgpc_client_user_guide.md) |
| launch 파일 / 헬퍼 인벤토리 | [../../src/robot_control/launch/README.md](../../src/robot_control/launch/README.md) |
| 백엔드 프로파일 (`config/backends/*.yaml`) | [../../src/robot_control/config/backends/README.md](../../src/robot_control/config/backends/README.md) · [../backends/README.md](../backends/README.md) |
| 토픽 이름 규약 (`~/` 가 아니라 루트 상대인 이유) | [../topic_naming_contract.md](../topic_naming_contract.md) |
| 세션 인지 카메라·뷰어 / 레코더 | [../collect/session_camera_nodes_guide.md](../collect/session_camera_nodes_guide.md) · [../collect/rdfp_image_recorder_node_guide.md](../collect/rdfp_image_recorder_node_guide.md) |
| REST 로 팔·그리퍼를 조작한다 | [../twin/robot_twin_user_guide.md](../twin/robot_twin_user_guide.md) |
| 리더 트리거 → 그리퍼 매핑 (설계안, 전제 무효) | [../archive/leader_gripper_mapping_design.md](../archive/leader_gripper_mapping_design.md) |

전체 문서 색인은 [../INDEX.md](../INDEX.md) 에 있다.
