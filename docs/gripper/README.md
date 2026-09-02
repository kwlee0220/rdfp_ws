# gripper 문서 안내

그리퍼 관련 문서 4개의 진입점이다. **하려는 일부터 고른다.**

| 하려는 일 | 문서 |
|---|---|
| 코드에서 그리퍼를 쓴다 (명령 발행 · 상태 구독 · 성공 판정) | [GripperNode_Design.md](GripperNode_Design.md) **§5** |
| 필드의 뜻을 안다 (`width` / `stalled` / `at_goal`) | [GripperNode_Design.md](GripperNode_Design.md) §2 |
| mock · Gazebo · Isaac 에서 노드를 띄우고 파라미터를 맞춘다 | [GripperActionNode_Guide.md](GripperActionNode_Guide.md) |
| 펑션베이(Robotiq 2F)에서 같은 일을 한다 | [Robotiq2FGripperNode_Guide.md](Robotiq2FGripperNode_Guide.md) |
| 그리퍼가 이상하다 — ros2_control 액션 서버 계층을 파고든다 | [gripper_action_server_notes.md](gripper_action_server_notes.md) |

## 전체 그림

**계약 하나, 구현 둘.** 상위 계층(teleop · 트윈 · 재생 · 데이터셋)은 계약만 알면 되고,
구현은 **액션 서버가 있느냐**로 갈린다.

```text
              gripper_cmds  (rdfp_msgs/GripperCommand — 심볼 open/close/grasp)
              gripper_states (rdfp_msgs/GripperState  — width · stalled · at_goal)
                                   ▲
                    GripperNode 계약  ← GripperNode_Design
                                   │
              ┌────────────────────┴────────────────────┐
     GripperActionNode                         Robotiq2FGripperNode
     ← GripperActionNode_Guide                 ← Robotiq2FGripperNode_Guide
              │                                          │
              ▼                                          ▼
   /panda_hand_controller/gripper_cmd          /input/gripper_joint  (6축 목표각)
   (control_msgs/GripperCommand 액션)          /output/gripper_joint (6축 q/v/f)
              │  ← gripper_action_server_notes             │
              ▼                                            ▼
     ros2_control 컨트롤러                        시뮬레이터 (컨트롤러 없음)
     mock · Gazebo · Isaac(브리지)                펑션베이
```

## 읽는 순서

- **처음이라면** `GripperNode_Design` 의 §1(왜 별도 인터페이스인가)과 §5(쓰는 법)만
  본다. §2~§4 는 필요할 때 찾아 읽는 레퍼런스다.
- **노드를 띄우거나 튜닝한다면** 쓰는 스택의 구현 가이드로 간다. 두 가이드는 계약을
  중복 설명하지 않고 **그 구현이 계약을 어떻게 만족시키는지**만 다룬다.
- **예상과 다르게 동작하면** `gripper_action_server_notes` 를 먼저 본다. 실측 기반
  함정 모음이다 — 단, **액션 스택 한정**이고 펑션베이에는 해당하지 않는다.

## 세 가지만 기억한다

1. **명령은 심볼, 관측은 물리량.** 숫자는 그리퍼에 종속이라 명령에 싣지 않는다
   (`GripperNode_Design` §1.1).
2. **성공 판정은 `at_goal` 하나.** `goal` 마다 다른 판정식은 노드가 이미 적용했다 (§2.3).
3. **`/joint_states` 의 손가락 관절로 폭을 읽지 않는다.** 펑션베이는 TF 성립용 고정값을
   주입해 "항상 열려 있다"고 거짓말한다 (§1.2, [토픽 규약](../topic_naming_contract.md) §2.2).

## 여기 없는 것

| 주제 | 문서 |
|---|---|
| 팔 계획·실행·서보 | [../moveit/README.md](../moveit/README.md) |
| 토픽 이름 규약 (`~/` 가 아니라 루트 상대인 이유) | [../topic_naming_contract.md](../topic_naming_contract.md) §2.2 |
| 펑션베이 그리퍼 채널 실측 사양 | [../simulation/functionbay_backend_design.md](../simulation/functionbay_backend_design.md) §6.1 |
| 리더 트리거 → 그리퍼 매핑 (설계안) | [../teleop/leader_gripper_mapping_design.md](../teleop/leader_gripper_mapping_design.md) |
| REST 로 그리퍼를 조작한다 | [../robot_twin/robot_twin_user_guide.md](../robot_twin/robot_twin_user_guide.md) |

전체 문서 색인은 [../INDEX.md](../INDEX.md) 에 있다.
