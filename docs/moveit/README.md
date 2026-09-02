# moveit 문서 안내

MoveIt2 스택(계획·실행·서보·그리퍼) 관련 문서 7개의 진입점이다. **하려는 일부터 고른다.**

| 하려는 일 | 문서 |
|---|---|
| 로봇을 계획해서 움직인다 (Cartesian / named target / joint 목표값) | [MoveGroupClient_UserGuide.md](MoveGroupClient_UserGuide.md) |
| JGPC 스택(`panda_jgpc_mock`)에서 움직인다 / 명령을 직접 스트리밍한다 | [MoveGroupJgpcClient_UserGuide.md](MoveGroupJgpcClient_UserGuide.md) |
| 텔레옵·replay 에서 servo 를 켜고 상태를 확인한다 | [servo_client_programmers_guide.md](servo_client_programmers_guide.md) |
| 그리퍼를 코드에서 쓴다 (명령·상태·판정) | [GripperNode_Design.md](GripperNode_Design.md) §5 |
| 그리퍼 노드를 띄우고 파라미터를 맞춘다 | [GripperActionNode_Guide.md](GripperActionNode_Guide.md) (mock·Isaac) / [Robotiq2FGripperNode_Guide.md](Robotiq2FGripperNode_Guide.md) (펑션베이) |
| 그리퍼가 이상하다 — 액션 서버 계층을 파고든다 | [gripper_action_server_notes.md](gripper_action_server_notes.md) |

## 전체 그림

세 진입점이 병렬로 존재한다. 서로 다른 MoveIt 인터페이스를 쓰므로 목적에 따라 고른다.

```text
[계획 + 실행]                    [실시간 서보]              [그리퍼]

 MoveGroupClient                  ServoClient           GripperNode (계약)
  ├ plan_* / move_to_* / follow_*  ├ start/stop/pause    ├ gripper_cmds  (구독, 심볼)
  └ create_move_group_client()     └ check_status()      └ gripper_states (발행, 주기)
        │                                │                       │
        │  ← MoveGroupClient_UserGuide   │  ← servo_client_      │  ← GripperNode_Design
        │                                │     programmers_guide │
        ▼                                ▼                       ▼
  /move_action                    /servo_node/*          구현이 둘 — 실행 수단이 다르다
  /compute_cartesian_path          start_servo 등        ┌──────────────┴──────────────┐
        │                                          GripperActionNode      Robotiq2FGripperNode
        │  JTC: /panda_arm_controller/…                  │                      │
        │  JGPC: /panda_arm_controller/commands          ▼                      ▼
        │        ← MoveGroupJgpcClient_UserGuide   /panda_hand_controller  /input/gripper_joint
        │                                            /gripper_cmd          (6축 목표각)
        │                                                │  ← gripper_action_server_notes
        ▼                                                ▼
                        ros2_control 컨트롤러 → hardware        (컨트롤러 없음)
```

**그리퍼는 계약과 구현이 나뉜다.** 상위 계층은 `GripperNode` 계약(두 토픽 + `at_goal`)만
알면 되고, 액션 서버가 있느냐 없느냐가 구현을 가른다.

## 읽는 순서

- **처음이라면** `MoveGroupClient_UserGuide` 의 「개요」·「Quick Start」만 본다.
  나머지는 필요할 때 찾아 읽는 레퍼런스다.
- **JGPC 스택을 쓴다면** 위 문서의 「구현별 API 가용성」 표로 무엇이 되고 안 되는지 확인한 뒤
  `MoveGroupJgpcClient_UserGuide` 로 넘어간다. 두 문서는 역할을 나눠 가진다 —
  공통 인터페이스와 JTC 는 앞 문서, JGPC 전용 API 와 스트리밍 상세는 뒤 문서다.
- **그리퍼는** `GripperNode_Design`(계약 — 필드의 뜻과 §5 사용법) → 쓰는 스택의 구현
  가이드 순서다. `gripper_action_server_notes` 는 그 아래 ros2_control 계층의 실측 기반
  함정 모음이라, 예상과 다르게 동작할 때 먼저 본다 (액션 스택 한정).

## 여기 없는 것

`robot_control/moveit/` 의 모듈이지만 **쓰이는 문맥이 다른 서브시스템**이라 다른 폴더에 문서가 있다.

| 모듈 | 문서 |
|---|---|
| `ee_pose_publisher`, `ee_twist_publisher` | [../replay/replay_mock_stack_guide.md](../replay/replay_mock_stack_guide.md), [../teleop/](../teleop/) |
| `target_joint_cmds_publisher` / `_executor` | [../replay/replay_mock_stack_guide.md](../replay/replay_mock_stack_guide.md) |
| `servo_auto_start_node` | [servo_client_programmers_guide.md](servo_client_programmers_guide.md) (개요) + [../replay/replay_mock_stack_guide.md](../replay/replay_mock_stack_guide.md) (launch 연동) |
| `TrajectoryStreamer` | [MoveGroupJgpcClient_UserGuide.md](MoveGroupJgpcClient_UserGuide.md) |
| launch 파일 / 헬퍼 인벤토리 | [../../src/robot_control/launch/README.md](../../src/robot_control/launch/README.md) (제어 계열 + helper 전체) |

전체 문서 색인은 [../INDEX.md](../INDEX.md) 에 있다.
