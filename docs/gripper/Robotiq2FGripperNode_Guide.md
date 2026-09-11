# Robotiq2FGripperNode — Robotiq 2F 계열 그리퍼 구동

[GripperNode 설계](GripperNode_Design.md)가 정한 계약을 **Robotiq 2F 그리퍼의 관절
목표각을 토픽으로 직접 씀으로써** 실현한 노드다. 현재 쓰이는 곳은 **펑션베이 백엔드**
(2F-85)이며, [`GripperActionNode`](GripperActionNode_Guide.md)와 **계약은 같고 실행
수단이 다르다.**

**이름이 인터페이스가 아니라 기구를 가리키는 이유** — `GripperActionNode` 는
`control_msgs/GripperCommand` 액션이라는 표준 인터페이스 뒤에 있어 어떤 그리퍼든
가릴 수 있다. 여기는 그런 인터페이스가 없어 기구를 직접 구동하므로, 이름이 가리킬
불변항이 기구밖에 없다 ([설계서](GripperNode_Design.md) §3.2).

**계열 안에서는 파라미터로 흡수된다.** 기본값은 2F-85 실측치이고, 2F-140 은 링키지
구조(부호 벡터)가 같고 스트로크만 다르므로 `targets` 만 바꾸면 된다.
**단, 펑션베이는 씬마다 축 수·단위·packing 이 달라** `gripper_profile_file`
파라미터로 규약 파일을 받는다 (backend_design §6.1a).

| | |
|---|---|
| 패키지 | `robot_control` |
| 실행 파일 | `robotiq_2f_gripper_node` |
| 노드 이름 | `gripper` (launch 가 지정) |
| 소스 | [robotiq_2f_gripper_node.py](../../src/robot_control/robot_control/gripper/robotiq_2f_gripper_node.py) |

| | `GripperActionNode` | **`Robotiq2FGripperNode`** |
|---|---|---|
| 실행 | `control_msgs/GripperCommand` 액션 | 관절 목표각 토픽 |
| 쓰는 스택 | mock · Gazebo · Isaac(브리지) | 펑션베이 |
| `at_goal` 근거 | 개구 폭 | **관절 잔차** |
| `width` | 손가락 관절 × 2 | **NaN** (§5) |
| `stalled` | ⬜ 미판정 | ✅ 토크 + 속도 |

---

## 1. 채널

```text
  gripper_cmds  (rdfp_msgs/GripperCommand)      심볼 'open' / 'close' / 'grasp'
      │
      ▼
  ┌─────────────────────── Robotiq2FGripperNode ────────────────────────┐
  │  targets 의 스칼라 s 를 axis_signs 로 펼쳐 6축 목표각            │
  │                                                                 │
  │  관절 잔차 → 자세 도달 판정                                      │
  │  토크 + 속도 → stalled                                           │
  └─────────────────────────────────────────────────────────────────┘
      │                                         │
      ▼                                         ▼
  joint_command                             gripper_states
  (std_msgs/Float64MultiArray)              (rdfp_msgs/GripperState, 10 Hz)
```

| 방향 | 이름 | 타입 | 비고 |
|---|---|---|---|
| 구독 | `gripper_cmds` | `rdfp_msgs/GripperCommand` | 계약 채널 (루트 상대) |
| 발행 | `gripper_states` | `rdfp_msgs/GripperState` | 계약 채널. 명령이 없어도 주기 발행 |
| 발행 | `joint_command` | `std_msgs/Float64MultiArray` | 백엔드 채널 — **remap 대상** |
| 구독 | `joint_report` | `sensor_msgs/JointState` | 백엔드 채널. position/velocity/**effort** 전부 필요 |

**백엔드 두 채널은 상대 이름이고 결합은 remap 이 만든다.** 펑션베이 launch 가
`joint_command → /input/gripper_joint`, `joint_report → /output/gripper_joint` 로
잇는다 ([토픽 규약](../topic_naming_contract.md) §4).

`gripper_states` 는 `GripperActionNode` 와 마찬가지로 **`TRANSIENT_LOCAL` 이 아니다** —
늦게 붙은 구독자는 다음 발행까지 최대 `1/publish_rate` 초 기다린다.

---

## 2. 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `axis_signs` | `[1, 1, -1, -1, -1, 1]` | 스칼라 목표를 축별로 펼치는 부호 |
| `targets.open` | `0.0` | 심볼 → 구동 스칼라 `s` [rad] |
| `targets.close` | `0.725` | 완전 폐쇄 |
| `targets.grasp` | `0.725` | `close` 와 **목표가 같다** — 다른 것은 판정식이다 |
| `position_tolerance` | `0.005` | 자세 도달 판정의 잔차 한계 [rad] |
| `stall_effort` | `1.0` | `stalled` 판정 토크 임계 [N·m] |
| `stall_velocity` | `0.001` | `stalled` 판정 속도 상한 [rad/s] |
| `publish_rate` | `10.0` | `gripper_states` 발행 Hz |

기본값은 전부 **2F-85 실측치**다 ([펑션베이 설계서](../simulation/functionbay_backend_design.md) §6.1).
다른 그리퍼에 붙이면 이 표를 다시 채운다 — 그것이 이 노드에서 유일하게 기구를 아는 자리다.

### 기동 시 검증

| 조건 | 동작 |
|---|---|
| `axis_signs` 가 비었다 | `ValueError` — **노드가 뜨지 않는다** |
| `targets.<goal>` 이 유한하지 않다 | `ValueError` |
| `publish_rate <= 0` | `ValueError` |

---

## 3. 명령 — 스칼라 하나를 6축으로 펼친다

```python
return [scalar * sign for sign in self._axis_signs]
```

`close` → `[+0.725, +0.725, −0.725, −0.725, −0.725, +0.725]`.

**6축을 한꺼번에 채우는 것이 이 노드의 핵심 책임이다.** 2F-85 는 링키지로 묶여 있는데
**시뮬레이터가 mimic 을 강제하지 않는다** — 한 축만 보내면 나머지 5축은 그대로 남아
링키지가 어긋난 자세가 되고, 시뮬레이터는 그것을 거부하지 않는다. 그 오류는 TF·파지
판정·데이터셋에 에러 없이 스며든다.

일관되게 채우면 **전 구간 추종 오차 0.0000** 이다. 한때 "인덱스 1·4 는 25% 오차"로
기록됐던 것은 축 특성이 아니라 **한 축만 바꾼 모순된 지령에 링키지가 저항한 결과**였다.

### 거부와 경고

| 상황 | 동작 |
|---|---|
| 모르는 심볼 | `ERROR` 로그 + **거부.** `goal` 을 갱신하지 않는다 |
| `joint_command` 구독자 0 | `WARNING` 후 **그래도 발행한다** |

두 번째가 `GripperActionNode` 와 다르다. 저쪽은 액션 서버가 없으면 goal 전송이 아예
불가능해 포기하지만, **토픽 발행은 실패하는 연산이 아니고** 시뮬레이터가 곧 붙을 수
있다. 명령이 정말 유실됐다면 잔차가 줄지 않아 `at_goal` 이 `false` 로 남는다 —
거짓말이 아니라 정직한 결과다.

---

## 4. 판정 — 잔차와 토크

### 4.1 `at_goal` 은 **관절 잔차**로 판정한다

```python
residual = max(abs(p - t) for p, t in zip(self._positions, self._target))
```

계약이 요구하는 것은 "목표 **자세** 도달"이고 무엇으로 재는지는 구현이 정한다
([설계서](GripperNode_Design.md) §2.3). 이 노드가 잔차를 쓰는 이유는 **지령과 보고가
같은 관절 공간**이기 때문이다 — 실측 잔차가 0.0000 이다. 개구 폭으로 환산하면 2F-85
링키지 근사 오차만 더해진다.

**최악 축을 본다.** 한 축만 어긋나도 도달이 아니다 — 어긋난 자세를 성공으로 읽지 않기
위해서다.

| `goal` | 조건 |
|---|---|
| `''` (명령 이전) | `false` |
| 보고 길이가 축 수와 다르다 | `false` (잔차 `NaN`) |
| `open` / `close` | `잔차 ≤ position_tolerance` **AND NOT** `stalled` |
| `grasp` | `stalled` |

### 4.2 `stalled` 은 신호 **둘**이 함께 서야 한다

```python
max|effort| > stall_effort  AND  max|velocity| < stall_velocity
```

| 상황 | 토크 max | 판정 |
|---|---|---|
| 빈손 폐쇄 (정지) | 0.004 N·m | `false` |
| 자유 이동 중 (가속) | 0.845 N·m | `false` — 임계 아래이고 속도로도 갈린다 |
| **파지** | **17.19 N·m** | **`true`** |

임계 `1.0` 은 자유 이동 최대(0.845)보다 위, 파지(17.19)보다 한참 아래라 양쪽에서 17배
이상 여유가 있고, 빈손·파지·파지유지·높이이탈 네 상황에서 오분류 0건이었다.

> ⚠️ **두 신호는 독립이 아니다 — 실제 시뮬레이터에서는 같은 조건이다 (2026-09-04).**
> 펑션베이의 `effort` 는 측정된 토크가 아니라 **드라이브의 비례 오차항**이다:
> `effort = 200.0 × (지령 − 실제)`, 18점 대조 오차 0.037 이하
> ([백엔드 설계서](../simulation/functionbay_backend_design.md) §6.1). 따라서
>
> ```
> |effort| > stall_effort 1.0  ⟺  잔차 > 0.005  =  position_tolerance
> ```
>
> 두 임계가 수치까지 같다(`1.0 ÷ 200 = 0.005`). 결과로 **§4.1 판정식의
> `AND NOT stalled` 항은 결과를 바꿀 수 없다** — 잔차가 허용치 이내면 effort 는 반드시
> 임계 이하이므로 `stalled` 이 참일 수 없다.
>
> **`grasp` 판정은 그대로 성립한다.** `stalled` = "목표까지 못 갔고 멈췄다"이고 그것이
> 파지의 옳은 정의다. 무너지는 것은 *서로 검증한다*는 근거뿐이며, **힘을 재는 관측은
> 이 채널로 얻을 수 없다.**
>
> 위 표의 `0.845` / `17.19` 는 잔차 `0.0042` / `0.0859` 의 환산값이다. 대역 시뮬레이터가
> 아니라 실제 모델 실측이지만, **접촉력이 아니라 위치 오차를 재고 있었다.**

`effort` 가 비어 오면 `false` — 계약대로 **"모른다"** 는 뜻이다.

> ⚠️ **`stall_velocity` 0.001 은 잡음 문턱에 걸려 있다.** 막혀 멈춘 상태의 실측 속도가
> 0.0001 · 0.0004 · **0.0015** 로 임계를 넘나들어 `stalled` 이 깜빡인다. Isaac 은 같은
> 이유로 `stall_velocity: 0.0` 으로 끈다 (`panda_isaac.launch.py`).

### 4.3 계약 케이스 검증 (**대역 시뮬레이터**, 2026-09-02)

> **아래 표는 대역 시뮬레이터 결과지만, 2026-09-04 에 실제 펑션베이에서 노드를 띄워
> 세 행을 재현했다.** `open` → `at_goal: true`(잔차 0.0001), 물체를 문 채 `close` →
> `at_goal: false` · `stalled: true`("뭔가 끼었다"), 같은 상태에서 `grasp` →
> `at_goal: true` · `stalled: true`(파지 성공). **판정식이 실물에서 계약대로 동작한다.**

| `goal` | 물체 | `stalled` | `at_goal` | 뜻 |
|---|:-:|:-:|:-:|---|
| `close` | 없음 | false | **true** (0.5 s) | 빈손 폐쇄 성공 |
| `open` | — | false | **true** (0.5 s) | 개방 성공 |
| `grasp` | **있음** | **true** | **true** (0.5 s) | **파지 성공** |
| `grasp` | 없음 | false | **false** (영영) | **헛닫힘** — 목표 자세까지 닫혔다 |
| `close` | **있음** | **true** | **false** (영영) | **뭔가 끼었다** — 시킨 일이 아니다 |

명령 직후 3틱이 모두 `false` 이고 도달 시점에만 `true` 로 바뀌는 것도 확인했다 — 명령을
보냈다는 이유로 서지 않는다.

---

## 5. `width` 는 NaN 이다

개구 폭은 6축 각도에서 바로 나오지 않는다. 2F-85 링키지 기하가 필요하고 **선형이
아니며** 제원이 아직 없다.

**근사값을 넣지 않는다.** 넣으면 "폭을 안다"는 신호가 데이터셋에 남고, 나중에 제원을
받아 고칠 때 **과거 에피소드의 의미가 조용히 바뀐다.** 계약이 `NaN` 을 "모른다"로
정의해 두었으므로 그대로 둔다.

**`at_goal` 은 영향받지 않는다** — 잔차로 판정하기 때문이다. 잃는 것은 `grasp` 성공과
헛닫힘을 폭으로 **교차 검증**하는 것뿐이고, 판정 자체는 `stalled` 이 가른다.

> ⚠️ **`/joint_states` 의 `panda_finger_joint1` 을 폭으로 읽지 않는다.** 그 값은
> 펑션베이 launch 가 TF 성립용으로 주입하는 **고정값 0.04** 다 — URDF 는 Panda Hand 인데
> 시뮬레이터는 2F-85 라 관절이 대응하지 않는다. 이 노드가 떠 있어도 그 값은 여전히
> 고정이며, "항상 열려 있다"고 거짓말한다
> ([토픽 규약](../topic_naming_contract.md) §2.2).

---

## 6. 실행

펑션베이 launch 가 띄운다. `enable_gripper:=false` 로 끌 수 있다.

```bash
ros2 launch robot_control panda_functionbay.launch.py
```

시뮬레이터 준비 후(`readiness_gate` 통과) 기동한다 — 관절 보고를 받아야 판정이 서기
때문이다.

직접 띄울 때는 remap 이 필요하다.

```bash
ros2 run robot_control robotiq_2f_gripper_node --ros-args \
    -r joint_command:=/input/gripper_joint \
    -r joint_report:=/output/gripper_joint
```

```bash
ros2 topic pub --once /gripper_cmds rdfp_msgs/msg/GripperCommand \
    "{header: {stamp: {sec: 0}}, goal: 'grasp'}"
ros2 topic echo /gripper_states --once
```

---

## 7. 트러블슈팅

| 증상 | 원인 | 확인 |
|---|---|---|
| 명령에 반응이 없다 | 모르는 심볼 | 로그의 `unknown goal ...; known: [...]` |
| 〃 | 시뮬레이터 미연결 | 로그의 `nobody subscribes 'joint_command'` |
| `at_goal` 이 영영 `false` | 명령이 유실됐다 (잔차가 안 줄어든다) | `ros2 topic echo /input/gripper_joint` |
| 〃 (`close`) | 뭔가 끼었다 — **정상 판정** | `stalled` 이 `true` 인지 |
| 〃 (`grasp`) | 헛닫힘 — **정상 판정** | `stalled` 이 `false` 인지 |
| 상태가 갱신되지 않는다 | 보고 길이가 축 수와 다르다 | 로그의 `has N positions, expected M` (5 초 스로틀) |
| `stalled` 이 안 선다 | 보고에 `effort` 가 없다 | `ros2 topic echo /output/gripper_joint --once` |
| 한 축만 어긋난 자세로 간다 | 이 노드를 거치지 않고 직접 발행했다 | `/input/gripper_joint` 의 퍼블리셔 수 |
| `width` 가 `NaN` | **의도된 동작** (§5) | 기동 시 WARN 로그 |

---

## 8. 관련 문서

- [GripperNode_Design.md](GripperNode_Design.md) — **계약 정본.** 메시지 필드의 뜻,
  `at_goal` 판정식과 "무엇으로 재는지는 구현이 정한다"(§2.3)
- [GripperActionNode_Guide.md](GripperActionNode_Guide.md) — 액션 기반 자매 구현
- [../simulation/functionbay_backend_design.md](../simulation/functionbay_backend_design.md)
  §6.1 — 채널 사양과 임계값의 실측 근거
- [../topic_naming_contract.md](../topic_naming_contract.md) §2.2 — `/joint_states` 의
  손가락을 믿으면 안 되는 이유
