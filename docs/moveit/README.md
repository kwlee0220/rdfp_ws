# moveit 문서 안내

MoveIt2 스택(계획·실행·서보) 관련 문서 3개의 진입점이다. **하려는 일부터 고른다.**

> **그리퍼는 여기 없다 (2026-09-02 이동).** MoveIt 이 아니라 ros2_control·시뮬레이터
> 계층이라 [../gripper/](../gripper/) 로 옮겼다.

| 하려는 일 | 문서 |
|---|---|
| 로봇을 계획해서 움직인다 (Cartesian / named target / joint 목표값) | [MoveGroupClient_UserGuide.md](MoveGroupClient_UserGuide.md) |
| JGPC 스택(`panda_jgpc_mock`)에서 움직인다 / 명령을 직접 스트리밍한다 | [MoveGroupJgpcClient_UserGuide.md](MoveGroupJgpcClient_UserGuide.md) |
| 텔레옵·replay 에서 servo 를 켜고 상태를 확인한다 | [servo_client_programmers_guide.md](servo_client_programmers_guide.md) |
| 그리퍼를 다룬다 | [../gripper/README.md](../gripper/README.md) |

## 전체 그림

두 진입점이 병렬로 존재한다. 서로 다른 MoveIt 인터페이스를 쓰므로 목적에 따라 고른다.

```text
[계획 + 실행]                            [실시간 서보]

 MoveGroupClient                          ServoClient
  ├ plan_* / move_to_* / follow_*          ├ start/stop/pause
  └ create_move_group_client()             └ check_status()
        │                                        │
        │  ← MoveGroupClient_UserGuide           │  ← servo_client_programmers_guide
        ▼                                        ▼
  /move_action                              /servo_node/*
  /compute_cartesian_path                    start_servo 등
        │                                        │
        │  JTC: /panda_arm_controller/joint_trajectory
        │  JGPC: /panda_arm_controller/commands   ← MoveGroupJgpcClient_UserGuide
        ▼                                        ▼
                    ros2_control 컨트롤러 → hardware
```

## 읽는 순서

- **처음이라면** `MoveGroupClient_UserGuide` 의 「개요」·「Quick Start」만 본다.
  나머지는 필요할 때 찾아 읽는 레퍼런스다.
- **JGPC 스택을 쓴다면** 위 문서의 「구현별 API 가용성」 표로 무엇이 되고 안 되는지 확인한 뒤
  `MoveGroupJgpcClient_UserGuide` 로 넘어간다. 두 문서는 역할을 나눠 가진다 —
  공통 인터페이스와 JTC 는 앞 문서, JGPC 전용 API 와 스트리밍 상세는 뒤 문서다.
- **그리퍼는** [../gripper/README.md](../gripper/README.md) 가 진입점이다.

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
