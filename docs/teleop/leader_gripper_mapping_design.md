# 리더 트리거 → 그리퍼 명령 매핑 설계

> **상태: 설계안 — 코드 미반영.**
>
> 전제인 "OMY-L100 에 그리퍼 역할의 관절이 있다"가 **아직 확인되지 않았다** (1.2).
> 확인 결과에 따라 3장 이후가 그대로 쓰이거나 폐기된다.

리더 장치의 트리거 관절 위치값을 팔로워 그리퍼의 `control_msgs/GripperCommand`
goal 로 바꾸는 방법을 정한다. 팔 미러링은
[leader_follower_mirroring_design.md](leader_follower_mirroring_design.md) 가 다루며,
이 문서는 **그리퍼만** 다룬다.

---

## 1. 목적과 전제

### 1.1 지금은 그리퍼 경로가 없다

OMY-L100 은 **리더(입력) 장치**다. bringup 이 `leader` 네임스페이스에 띄우는 것은
`ros2_control_node` / `robot_state_publisher` / `gravity_compensation_controller` /
`spring_actuator_controller` / `joint_state_broadcaster` /
`joint_trajectory_command_broadcaster` 이며 **그리퍼 컨트롤러가 없다**
([OMY-L100_Docker_Setup_and_Usage.md](../OMY-L100_Docker_Setup_and_Usage.md) 5.1).

브리지가 넘기는 토픽도 `bridge_topics.json` 기준 `/leader/ee_pose` 하나뿐이다.
그래서 현재 체인은 이렇게 갈려 있다.

| 대상 | 명령원 |
|---|---|
| 팔 | OMY-L100 → `/leader/ee_pose` → `teleop_retarget` → 팔로워 |
| **그리퍼** | **별도 경로** — `teleop_keyboard` 의 `=`/`-`, 또는 트윈의 `move_gripper_to_target` |

리더로 자세를 만들면서 그리퍼는 손이나 API 로 따로 쳐야 한다. 이 문서는 그 분리를
없애는 것을 목표로 한다.

### 1.2 먼저 확인할 것 ⚠️

공식 문서는 OMY-L100 을 **6-DOF** 로만 기술한다. 그리퍼용 트리거가 별도 관절로
노출되는지는 확인되지 않았다. 실물이 연결된 상태에서 다음으로 판정한다.

```bash
# 컨테이너 내부
ros2 topic echo /leader/joint_states --once
```

| `name[]` | 판정 |
|---|---|
| 관절 7개 (팔 6 + 1) | 트리거 존재 → 이 설계를 적용한다 |
| 관절 6개 | 하드웨어 입력이 없다 → 이 설계는 폐기하고 키보드·페달 등 별도 입력을 쓴다 |

관절 이름과 **실제 가동 범위**를 함께 기록한다. 3.1 의 캘리브레이션이 그 값을 쓴다.

### 1.3 데이터 흐름 (설계안)

```
OMY-L100 트리거 관절
    │  /leader/joint_states                    (컨테이너 — Jazzy + rmw_zenoh)
    ▼
omy_leader_bridge  (UDP relay)                 ← bridge_topics.json 에 토픽 1줄 추가
    │  /leader/joint_states                    (호스트 — Humble + fastdds)
    ▼
[신규] 그리퍼 매핑 노드                          ← 캘리브레이션·deadband·전송률·게이트
    │  open/close 서비스 또는 gripper_cmd goal
    ▼
gripper_control_node → panda_hand_controller
```

---

## 2. 결론 요약

| 항목 | 결정 |
|---|---|
| `position` | 트리거 위치를 정규화해 매핑한다 |
| `max_effort` | **트리거에서 유도하지 않는다.** 고정 파라미터 (예: 30 N) |
| 전송 주기 | 메시지마다 보내지 않는다. deadband + 최소 간격 (10~20 Hz) |
| 인터페이스 | `GripperCommand` **액션 유지**. 스트리밍 컨트롤러로 바꾸지 않는다 |
| 입력 끊김 | 마지막 명령을 **유지**한다. 열지 않는다 |
| 매핑 위치 | **호스트(rdfp)**. 브리지는 원시 릴레이만 한다 |
| 도입 순서 | 1단계 이진(임계값) → 필요할 때 2단계 연속 |

---

## 3. 매핑

### 3.1 `position` — 트리거에서 뽑는다

```
u        = clamp((q_trigger - q_min) / (q_max - q_min), 0, 1)
position = (1 - u) × POSITION_OPEN
```

**기호**

| 기호 | 뜻 | 어디서 오나 |
|---|---|---|
| `q_trigger` | **지금 읽은 트리거 관절의 위치값** | `/leader/joint_states` 의 해당 관절 `position` |
| `q_min` | 트리거를 **완전히 놓았을 때**의 값 | 캘리브레이션 실측 (1.2 에서 함께 기록) |
| `q_max` | 트리거를 **끝까지 당겼을 때**의 값 | 캘리브레이션 실측 |
| `u` | 정규화한 당김 정도. `0` = 놓음, `1` = 끝까지 | 계산값 |
| `POSITION_OPEN` | 팔로워 그리퍼의 **완전 개방값** (panda 는 `0.04`) | 팔로워 설정 |

`q_*` 의 단위는 트리거가 회전 관절이면 rad, 직동 관절이면 m 이다. **정규화 과정에서
단위가 상쇄되므로 어느 쪽이든 수식은 그대로다.**

`clamp(x, lo, hi)` 는 `x` 를 `[lo, hi]` 범위로 잘라내는 함수다 — `lo` 보다 작으면 `lo`,
`hi` 보다 크면 `hi`, 사이면 `x` 그대로. 여기서는 캘리브레이션 범위를 벗어난 입력(더 세게
당기거나 반대로 밀어낸 경우)이 `u` 를 `0~1` 밖으로 내보내지 못하게 막는다. 이것이 없으면
`position` 이 음수가 되거나 관절 한계를 넘는 goal 이 나간다.

`position` 은 `u` 를 뒤집어 쓴다 — **당길수록(`u` ↑) 그리퍼가 닫혀야(`position` ↓)** 하기
때문이다.

**세 가지를 놓치기 쉽다.**

1. **`q_min` / `q_max` 는 실측 캘리브레이션 값이어야 한다.** 기구 공차가 있어 사양값을
   쓰면 완전 개방·완전 폐쇄에 도달하지 못한다. 양 끝에 **끝단 deadband**(예: 5%)를 둬서 (부록 A.3)
   "끝까지 당기면 확실히 `0.0`" 이 되게 한다.
2. **팔로워 목표는 half-width 다.** `panda_finger_joint1` 의 범위는 `0 ~ 0.04` 이며
   **손가락 사이 거리의 절반**이다. `0.08` 로 매핑하면 두 배로 벌리라는 명령이 된다.
   실기 franka 의 `move`/`grasp` 가 쓰는 `width` 와도 2배 차이가 난다.
3. **부호를 따로 뒤집는 파라미터는 필요 없다.** 당길수록 관절값이 감소하는 하드웨어면
   `q_max < q_min` 이 되는데, 분자와 분모가 함께 부호를 바꾸므로 `u` 는 그대로 `0~1` 이
   나온다. `q_min`/`q_max` 를 **"놓았을 때 / 당겼을 때"로 정의**해 두면 부호 문제가
   캘리브레이션에 흡수된다.

### 3.2 `max_effort` — 트리거에서 뽑지 않는다

`GripperCommand` goal 은 `position` 과 `max_effort` 둘뿐인데 성격이 다르다.

트리거를 더 당긴다고 손가락이 더 세게 쥐는 것이 아니다. **명령 위치가 물체 두께보다
작아지는 순간 컨트롤러가 stall 하면서 `max_effort` 만큼 버틴다.** 즉 파지력을 정하는
것은 `max_effort` 이고, 트리거 깊이는 "얼마나 파고들라고 명령했는가"일 뿐이다.

트리거 깊이를 힘으로 환산하면 **사람이 손끝 감각 없이 힘을 조절하는 셈**이라 위험하고
재현성도 없다. 고정 파라미터로 두고 작업별로 조정한다.

`max_effort = 0` 은 피한다 — 드라이버 기본값이라 파지력이 미정이고, 실기 franka
계열에서는 파지 없는 이동으로 해석될 수 있다. 트윈 설정의 `grasp` 목표가 쓰는
`30 N` 이 출발점으로 무난하다 (Franka Hand 연속 파지력 약 70 N 안에서 보수적인 값).

---

## 4. 전송 정책 — 진짜 함정은 여기다

`/leader/joint_states` 는 50 Hz 정도로 온다. **메시지마다 goal 을 보내면 안 된다.**
**액션은 새 goal 이 이전 goal 을 선점하는 구조**라, 초당 50번 선점하면 컨트롤러가 실행보다 선점 처리에 시간을 쓰고 result 는 `CANCELED` / `ABORTED` 로 쏟아진다.

세 가지를 함께 건다.

| 장치 | 값(출발점) | 막는 것 (해결사항) |
|---|---|---|
| **deadband** (부록 A.2) | 직전 **전송값** 대비 1~2 mm | 사람 손 떨림이 그대로 goal 이 되는 것 |
| **최소 간격** | 10~20 Hz 상한 | 기구 속도가 못 따라가는 무의미한 명령 |
| **선점 result 무시** | — | 정상 동작 중에 에러 로그가 계속 찍히는 것 |

세 번째는 "마지막 goal 만 유효"로 두고 취소된 이전 goal 의 result 는 로그도 남기지 않는다는 뜻이다. `gripper_control_node` 가 result 를 `~/gripper_action_states` 로 재발행하는 구조이므로, 그 경로를 쓸 때 특히 주의한다.

---

## 5. 인터페이스 선택 — 액션을 유지한다

스트리밍 제어에 액션은 원래 맞지 않는 인터페이스다. 대안은 팔로워 그리퍼 컨트롤러를 `JointGroupPositionController` 로 바꿔 `/commands`(`std_msgs/Float64MultiArray`)에 원하는 주기로 흘리는 것이다.

| | `GripperCommand` 액션 (유지) | `JointGroupPositionController` |
|---|---|---|
| 고주기 스트리밍 | 선점 처리 필요 (4장) | 자연스럽다 |
| 파지 성공 판정 | `stalled` / `reached_goal` | **불가** |
| 파지력 제어 | `max_effort` | **불가** |
| MoveIt·트윈·`gripper_control_node` | 그대로 동작 | **전부 깨진다** |
| 실기 franka | 액션만 노출된다 | 적용 불가 |

**텔레오퍼레이션 하나 때문에 나머지를 잃는 교환이다.** 액션을 유지하고 4장의 전송률 제어로 대응한다.

---

## 6. 안전

### 6.1 입력이 끊기면 마지막 명령을 유지한다

브리지가 UDP 릴레이라 값이 끊길 수 있다. 트리거 메시지가 일정 시간(예: 300 ms) 이상 없으면 **새 goal 을 보내지 않고 마지막 명령을 유지**한다.

"안전하게 열자"는 직관은 위험하다 — **쥐고 있던 물체를 떨어뜨린다.** 대신 그 상태를 로그와 상태 토픽으로 드러내 운용자가 알게 한다.

팔 쪽 데드맨(`teleop_retarget` 의 `pedal_timeout`)과 방향이 반대인 점에 주의한다. 팔은 끊기면 멈추는 것이 안전하지만, 그리퍼는 **유지**가 안전하다.

### 6.2 클러치를 게이트로 쓴다

클러치가 풀린 동안에는 팔이 움직이지 않는데 그리퍼만 트리거를 따라가면 어색하고 위험하다. `~/clutch_state`(`rdfp_msgs/ClutchState`, TRANSIENT_LOCAL) 를 구독하거나 `ClutchClient` 를 써서 **engage 상태에서만 명령을 내보낸다.**

`~/clutch` 는 `SetBool` 이라 호출 자체가 상태를 바꾸므로 조회에 쓰지 않는다.

---

## 7. 매핑 로직을 어디에 둘 것인가 — 호스트(rdfp)

브리지는 **원시 토픽 릴레이만** 하고, 캘리브레이션·임계값·전송률 같은 정책은 rdfp 에 둔다. 이유는 셋이다.

- 컨테이너(Jazzy)에 정책이 들어가면 파라미터를 바꿀 때마다 컨테이너를 건드려야 한다.
- 테스트도 컨테이너 쪽에서 해야 해서 CI·단위 테스트에 올리기 어렵다.
- [external_input_adapters.md](external_input_adapters.md) 의 어댑터 계약이 이미 그 방향이다 — 외부는 표준 토픽만 내보내고 정책은 rdfp 가 갖는다.

브리지에서 할 일은 `bridge_topics.json` 의 `topics` 배열에 `/leader/joint_states` 를 한 줄 추가하고 relay·host 를 재시작하는 것뿐이다. 토픽 이름의 단일 출처가 그 파일이라 다른 곳은 손대지 않는다.

---

## 8. 단계적 도입

### 1단계 — 이진 (권장 시작점)

임계값 + **hysteresis**(부록 A.4)로 open/close 만 판정한다. 예를 들어 `u > 0.7` 에서 close, `u < 0.3` 에서 open 으로 벌려 잡아 채터링을 막는다.

- `gripper_control_node` 가 구독하는 명령 토픽에 `position` 두 값 중 하나를 발행하면 되므로 새로 만들 것이 거의 없다.
- 상태가 둘뿐이라 전송 빈도 문제가 사실상 사라진다 (상태 전이에서만 보낸다).
- 파지 성공/실패 판정도 명확하다.

### 2단계 — 연속

얇은 물체나 부분 개방이 실제로 필요하다고 확인된 뒤에 3장의 연속 매핑으로 넘어간다. 처음부터 연속으로 가면 캘리브레이션·전송률·선점을 한꺼번에 상대해야 한다.

---

## 9. 검증

**리더 없이** — `/leader/joint_states` 를 bag 으로 녹화해 두고 재생하면 매핑·deadband·전송률을 실물 없이 시험할 수 있다. teleop 절차서가 `ee_pose` 에 대해 이미 쓰는 방법이다 ([omy_leader_teleop_guide.md](omy_leader_teleop_guide.md)).

**실물로** — 두 가지를 본다.

| 확인 | 방법 | 기대 |
|---|---|---|
| deadband·전송률이 먹는가 | 트리거를 천천히 끝에서 끝까지 움직이며 goal 전송 횟수를 센다 | 50 Hz 가 아니라 10~20 Hz 상한 |
| 파지가 성립하는가 | 물체를 쥔 상태에서 액션 result 확인 | `stalled: true` / `reached_goal: false` |

두 번째의 판정 규칙은 그리퍼 연산과 동일하다 — **`reached_goal: false` 는 실패가 아니다.** 물체를 물면 목표까지 갈 수 없으므로 당연한 결과이며, 파지 판정의 축은 `stalled` 다.

---

## 10. 미결 항목

| # | 항목 | 결정에 필요한 것 |
|---|---|---|
| 1 | 트리거 관절 존재 여부 | 실물 `/leader/joint_states` 확인 (1.2) |
| 2 | `q_min` / `q_max` | 실측 캘리브레이션. 절차를 노드에 넣을지 파라미터로 둘지 |
| 3 | `max_effort` 기본값 | 대상물이 정해진 뒤 실측. 30 N 은 출발점일 뿐이다 |
| 4 | 트리거 응답 곡선 | 선형으로 충분한지, 끝단에서 완만하게 할지 (사용성 문제) |
| 5 | 상태 노출 | 매핑 노드가 현재 목표·게이트 상태를 토픽으로 낼지 |

---

## 부록 A. deadband 와 hysteresis

4장·8장이 전제하는 개념이다. 둘 다 "작은 변화를 의도로 보지 않는다"는 점은 같지만 **기준이 다르고 쓰는 자리도 다르다.**

### A.1 deadband — 입력이 변해도 출력이 반응하지 않는 구간

우리말로는 불감대(不感帶)다. 임계값보다 작은 변화는 **의도가 아니라 잡음으로 보고 무시**한다. 이 문서에서는 두 가지 용도로 쓰며, 성격이 다르므로 구분해 부른다.

| | 쓰이는 곳 | 기준 |
|---|---|---|
| **전송 deadband** (A.2) | 4장 — goal 을 보낼지 말지 | 직전 **전송값** |
| **끝단 deadband** (A.3) | 3.1 — 정규화값 양 끝 포화 | 입력 범위의 양 끝 |

### A.2 전송 deadband — "얼마나 변해야 새로 보내나"

직전에 **보낸 값**과 비교해 차이가 임계값(1~2 mm)보다 작으면 goal 을 보내지 않는다.

```
직전 전송 = 0.0200
0.0203  → 차이 0.3 mm → 무시
0.0208  → 차이 0.8 mm → 무시
0.0215  → 차이 1.5 mm → 전송, 기준값이 0.0215 로 갱신
```

없으면 이렇게 된다 — 사람 손은 가만히 있어도 미세하게 떨리고 트리거 센서에도 잡음이 있다. 그 값이 50 Hz 로 들어오면 **손을 멈추고 있어도 초당 50개의 goal 이 나간다.**
액션은 새 goal 이 이전 것을 선점하므로 컨트롤러가 실행보다 선점 처리에 시간을 쓰고, 그리퍼는 미세하게 떨린다.

> **기준이 "직전 센서값"이 아니라 "직전 전송값"인 것이 중요하다 ⚠️**
> 센서값 기준으로 하면 임계값보다 작은 변화가 계속 누적되어, 실제로는 크게 움직였는데도 영영 보내지 않는 드리프트가 생긴다.

### A.3 끝단 deadband — "끝까지 갔다고 인정하는 구간"

정규화값 `u` 의 양 끝을 포화시켜, 90%만 당겨도 완전 폐쇄로 친다.

```
u < 0.05  →  u = 0   (완전 개방)
u > 0.95  →  u = 1   (완전 폐쇄)
```

캘리브레이션한 `q_min` / `q_max` 가 기구 공차·온도로 조금씩 어긋나면 끝까지 당겨도 `u = 0.98` 에 머물러 **완전히 닫히지 않는다.** 그러면 "다 당겼는데 왜 안 잡히지"가 된다. 끝단을 뭉개서 그것을 막는다.

### A.4 hysteresis — 방향에 따라 임계값이 다르다

8장의 `u > 0.7 → close`, `u < 0.3 → open` 이 이것이다. deadband 가 아니다.

```
u:  0 ─────── 0.3 ─────── 0.7 ─────── 1
        open   │           │   close
               └── 이 구간에서는 직전 상태를 유지한다 ──┘
```

닫히는 중에는 `0.7` 을 넘어야 close 로 바뀌고, 열리는 중에는 `0.3` 아래로 내려가야 open 으로 바뀐다. 사이 구간에서는 **아무 일도 일어나지 않는다.**

임계값이 하나뿐이면(예: `u > 0.5`) `u` 가 0.5 근처에서 오르내릴 때 open/close 가 초당 수십 번 뒤집힌다 — 채터링이다. 두 임계값의 간격이 그 진동폭보다 넓어야 한다.

| | deadband | hysteresis |
|---|---|---|
| 기준 | 직전 값 하나 | **방향에 따라 다른 두 임계값** |
| 목적 | 잡음·떨림 억제 | 경계에서의 채터링 방지 |
| 쓰는 곳 | 연속값 전송 (4장) | 이진 판정 (8장) |

### A.5 대가와 값 정하는 법

**deadband 는 지연이 없는 대신 분해능을 버린다.** 1 mm 로 잡으면 1 mm 미만의 미세 조작이 불가능해진다. 반대로 저역통과 필터(LPF)는 분해능을 유지하지만 **지연이 생겨** 텔레오퍼레이션에서 손과 로봇이 어긋나는 느낌을 준다.

그래서 텔레오퍼레이션에서는 대개 deadband 를 먼저 쓰고, 그래도 거칠면 아주 약한 LPF 를 얹는다. `teleop_retarget` 의 손떨림 튜닝도 같은 문제를 다룬다 ([teleop_retarget_node_guide.md](teleop_retarget_node_guide.md)).

값은 **실측으로 정한다.** 트리거를 일부러 가만히 잡고 있을 때의 값 변동폭을 재서 그보다 조금 크게 잡으면 된다. hysteresis 의 두 임계값 간격도 같은 방식으로 정한다.

---

## 참고 문서

- [external_input_adapters.md](external_input_adapters.md) — 외부 어댑터 경계 계약
- [teleop_retarget_node_guide.md](teleop_retarget_node_guide.md) — 클러치 상태·자동 해제
- [clutch_pedal_guide.md](clutch_pedal_guide.md) — 클러치 게이트(6.2)의 상대편
- [omy_leader_teleop_guide.md](omy_leader_teleop_guide.md) — 전체 체인 기동 절차
- [../moveit/GripperControlNode_Guide.md](../moveit/GripperControlNode_Guide.md) — open/close 서비스
- [../moveit/gripper_action_server_notes.md](../moveit/gripper_action_server_notes.md) — 액션 서버의 feedback/result 성격
- [../robot_twin/robot_twin_user_guide.md](../robot_twin/robot_twin_user_guide.md) 4.2 — 트윈의 그리퍼 연산 (`open`/`close`/`grasp` 목표)
- 외부 저장소 `omy_leader_bridge` — `bridge_topics.json` 이 릴레이 토픽의 단일 출처
