# GripperNode 설계

작성 2026-09-02 · **반영 완료**

그리퍼를 **백엔드와 무관하게 같은 방식으로** 활용하기 위한 인터페이스와, 그것을 구현하는 노드(`GripperNode`)의 계약을 정의한다. **소비자가 코드에서 쓰는 법은 §5** 에 있고, 실행 방법·파라미터·트러블슈팅은 구현별 가이드가 갖는다 — [GripperActionNode](GripperActionNode_Guide.md) / [Robotiq2FGripperNode](Robotiq2FGripperNode_Guide.md).

> **설계안으로 시작해 2026-09-02 에 전부 반영됐다.** 무엇이 달라졌고 왜 그렇게 정했는지가 이 문서의 내용이며, 이전 구현과의 차이는 §7, 반영 범위는 §8 에 있다.

---

## 1. 왜 별도 인터페이스가 필요한가

그리퍼는 **로봇마다 기구가 다르다.** 같은 "쥐어라"가 Panda Hand 에서는 평행 조(parallel jaw)의 prismatic 관절 0.04 m 이고, Robotiq 2F-85 에서는 링키지의 revolute 관절 0.725 rad 다. 단위조차 다르다.

그래서 상위 계층(트윈 · 텔레오퍼레이션 · 데이터셋)이 기구를 알지 못하게 하는 추상화가 필요하다. **그 추상화가 `GripperCommand` / `GripperState` 두 메시지이고, 이를 구현하는 쪽이 `GripperNode` 다.**

### 1.1 비대칭이 의도적이다 — 명령은 심볼, 관측은 물리량

| | 형식 | 이유 |
|---|---|---|
| 명령 `GripperCommand` | **심볼** (`open`/`close`/`grasp`) | 부르는 쪽이 기구를 몰라도 된다 |
| 관측 `GripperState` | **물리량** (m) | 재는 쪽은 자기 기구를 이미 안다 |

명령에 숫자를 실으면 **부르는 쪽이 그리퍼를 알아야 한다.** `position: 0.035` 는 Panda Hand 에서만 뜻이 있고 2F-85 로 옮기면 틀린 값이 된다. 심볼은 "그 그리퍼에 맞게 쥐어라"로 남아 그대로 이식된다 — 실제 조작 데이터셋들이 그리퍼를 이진/삼진 신호로 인코딩하는 이유가 이것이다.

반대로 관측은 물리량이어야 한다. 관절값으로 두면 같은 필드가 Panda 에서는 m, 2F-85에서는 rad 를 뜻해 **비교가 불가능**해진다. 폭(m)으로 고정하면 같은 데이터셋의 `scene/objects` 치수와 직접 맞댈 수 있다 — 5 cm 블록을 4.5 cm 로 쥐었다는 것이 그대로 읽힌다.

**숫자는 어디 있나.** 심볼 → 실제 목표값 변환표(`targets.<goal>`)를 `GripperNode` 가 파라미터로 갖는다. 기구를 아는 유일한 자리다.

### 1.2 `/joint_states` 로 대신할 수 없다

**펑션베이는 손가락 관절에 TF 성립용 고정값(`0.04`)을 주입한다.** mock 과 Isaac 은 실제 값을 싣지만, 펑션베이 스택에서 `/joint_states` 를 읽으면 **"항상 열려 있다"는 거짓말**을 받는다. 값이 비어 있으면 알아차리지만 그럴듯한 상수는 데이터를 열어봐도 드러나지 않는다.

`GripperState` 를 별도 채널로 두는 첫 번째 이유가 이것이다.

---

## 2. 메시지

### 2.1 `GripperCommand` — 명령

```
std_msgs/Header header
string goal
```

`goal` 은 **수행할 동작의 심볼**이다. 표준 값 셋은 `open` / `close` / `grasp` 이며 백엔드가 더 정의할 수 있으나 **이 셋은 어디에나 있어야 한다.**

| `goal` | 뜻 |
|---|---|
| `open` | 손을 편다 |
| `close` | 빈손으로 닫는다 (힘을 주지 않으므로 파지 용도가 아니다) |
| `grasp` | 물체를 쥔다 (`close` 와 자세는 같고 **힘이 다르다**) |

**모르는 심볼은 거부한다.** 조용히 무시하면 팔은 움직이는데 그리퍼만 안 움직이는 상태가 되어 원인이 보이지 않는다.

**중간 개구는 표현할 수 없다.** 목표가 셋뿐이라 감수한 제약이며, 필요해지면 심볼을 더한다(`grasp_light` 등). 숫자 필드를 되살리는 것은 마지막 수단이다.

> `header.stamp` 를 비우면 안 된다. 적재 코드가 이 값을 그대로 쓰므로 (`dataset/db/writers/base.py` 의 `extract_stamp`), 비면 **epoch 0 에 적재되어 어느 에피소드에도 속하지 못한다.** 로봇은 정상 동작하므로 데이터를 열어보기 전까지 드러나지 않는다.

### 2.2 `GripperState` — 연속 상태

```
std_msgs/Header header
string  goal        # 마지막 명령의 goal. 명령 이전에는 ""
float64 width       # 개구 폭 (m). 못 구하면 NaN
bool    stalled     # 힘을 내는데 움직이지 않는다
bool    at_goal     # 시킨 일을 이뤘는가 (위치 도달이 아니다. 매 주기 재평가)
```

주기 발행이며 **학습 데이터의 observation 채널**이다.

#### `goal` — 상태가 자기 목표를 들고 있다

이 필드가 없으면 관측만 봐서는 **어떤 명령에 대한 상태인지 알 수 없어** 명령 토픽과 시각으로 짝지어야 한다. 데이터셋에서 그 짝짓기는 에피소드 경계와 지터에 취약하다. 목표를 함께 실으면 상태가 **자기 완결적**이 된다.

명령 수신 시점에 갱신한다. 따라서 전이 중에는 `goal` 이 새 목표를 가리키면서 `width`는 아직 옛 자세인 구간이 있다 — 그것이 "가는 중"의 정직한 표현이다.

#### `width` — 관절값이 아니라 개구 폭

**손가락 사이 거리(m)** 다. `control_msgs/GripperCommand` 의 "gripper gap size" 및 실기 franka 의 `width` 와 같은 정의다. Panda Hand 는 대칭 평행 조(parallel jaw)라 `panda_finger_joint1` 값의 **2배**다.

**못 구하면 NaN 이다.** `0` 을 넣으면 "닫혀 있다"는 거짓말이 된다.

#### `stalled` — 힘을 내는데 안 움직인다 = 물었다

판정은 **로봇 스택이 자기 방식으로** 한다. 위치 변화와 힘의 임계값이 그리퍼마다 다르기 때문이며, 판정 수단이 없는 스택은 `false` 로 둔다.

> ⚠️ **`false` 가 "물지 않았다"를 보장하지 않는다.** 판정을 구현하지 않은 스택에서도 `false` 다. 이 값만으로 파지 실패를 결론짓지 않는다.

#### `at_goal` — 위치가 아니라 **의도**의 달성 여부다

**"시킨 일을 이뤘는가"** 다. "목표 위치에 갔는가"가 아니다.

이 구분이 중요한 이유는 `grasp` 다. 물체를 쥐면 **목표 자세에 닿기 전에 물체가 막아서** 멈춘다 — 위치 기준으로는 "도달 못 함"이지만 시킨 일은 이룬 것이다. 그래서 `goal` 마다 판정식이 다르며, **그 판정을 노드가 한다** (§2.3).

> ⚠️ **`control_msgs/GripperCommand.reached_goal` 과 뜻이 다르다.** 그쪽은 위치 도달을 뜻한다. 이름을 `reached_goal` 이 아니라 `at_goal` 로 둔 것이 그 구분의 표시이며, ROS 배경지식을 가진 사람이 반대로 읽지 않도록 여기 못박는다.

**판정을 소비자가 아니라 노드가 하는 이유**는 이 문서의 원칙(§1.1)과 같다 — `goal` 심볼과 `targets` 표를 함께 가진 유일한 자리가 노드다. 소비자마다 판정식을 들고 있으면 한 곳이 빠뜨렸을 때 **파지 성공이 실패로 집계**된다.

그래서 두 필드의 역할이 갈린다.

| | 성격 |
|---|---|
| `stalled` | **관측** — 힘을 내는데 안 움직인다 (물리적 사실) |
| `at_goal` | **판정** — 시킨 일을 이뤘다 (의도 대비 결과) |

**매 주기 재평가한다.** "한 번 이뤘었나"가 아니라 "지금 그런가"이므로, 물체를 놓쳐 손이
벌어지면 `at_goal` 이 다시 `false` 가 된다. 이벤트형으로는 표현할 수 없던 상태다. 비용은
무시할 수준이다 — 이미 매 주기 계산하는 값들의 비교 한둘이다.

**`goal` 이 `""` 이면 `at_goal` 은 무의미하다.** 첫 명령 이전에는 물을 대상이 없다.
소비자는 `goal` 이 비었는지 **먼저** 확인한다.

> ⚠️ **`at_goal=false` 는 "실패"와 "진행 중"을 구분하지 못한다.** 둘 다 `false` 다. 가르려면 `width` 를 함께 본다 — `grasp` 인데 **최소 폭에서 정지**해 있으면 헛닫힘 (물체가 없었다)이고, 폭이 변하는 중이면 아직 가는 중이다. 즉 **"성공했는가"에는 `at_goal` 하나로 답하지만 "왜 아닌가"에는 세 필드를 본다.**

### 2.3 `at_goal` 판정식 — 구현자용 명세

**이 표는 소비자가 아니라 `GripperNode` 구현자가 읽는다.** 소비자는 `at_goal` 하나만 보면 된다 — 그것이 이 설계의 요점이다.

| `goal` | `at_goal = true` 조건 |
|---|---|
| `open` | **목표 자세 도달** AND NOT `stalled` |
| `close` | **목표 자세 도달** AND NOT `stalled` |
| **`grasp`** | **`stalled`** (목표 자세에 못 간 채 힘으로 멈췄다 = 물체를 쥐었다) |
| `""` | 무의미 (판정하지 않는다) |

`open`/`close` 에 **`NOT stalled`** 를 넣은 이유는 **막혀서 멈춘 것을 성공으로 읽지 않기 위해서**다. "빈손으로 닫아라"라고 했는데 뭔가 끼어 멈췄으면 시킨 일을 이룬 것이 아니다.

`grasp` 만 자세 조건이 없다. 파지는 **목표 자세에 닿으면 오히려 실패**(헛닫힘)이기 때문이다.

#### "목표 자세 도달"을 무엇으로 재는지는 구현이 정한다

**`at_goal` 은 `width` 에 묶여 있지 않다.** 둘은 독립적으로 얻는 값이고, `width` 가
`NaN` 이어도 `at_goal` 은 판정될 수 있다 — 오히려 그쪽이 정확한 백엔드가 있다.

| 구현 | 무엇으로 재나 | 왜 |
|---|---|---|
| [`GripperActionNode`](GripperActionNode_Guide.md) (mock·Isaac) | `\|width − 목표 폭\| ≤ width_tolerance` | `/joint_states` 의 손가락 관절 하나가 관측의 전부다 |
| 펑션베이 (미구현) | 6축 **관절 잔차** `< 0.005 rad` | 지령과 보고가 같은 관절 공간이라 **실측 잔차가 0.0000** 이다. `width` 로 환산하면 2F-85 링키지 근사 오차가 더해질 뿐이다 |

**폭으로 재는 것을 계약으로 못박지 않는다.** 그렇게 하면 `width` 를 못 구하는 스택이
`at_goal` 까지 잃는데, 그 둘은 서로 다른 질문이다 — "얼마나 벌어졌나"와 "시킨 일을
이뤘나"다. 실제로 펑션베이는 후자를 mock 보다 정확히 답할 수 있으면서 전자는 근사밖에
못 한다.

구현이 자유롭게 고르되 **의미는 하나여야 한다** — "시킨 자세에 도달했는가". 도달
여부를 눈속임으로 만드는 대체 지표(예: 명령을 보냈으니 `true`)는 계약 위반이다.

관측값으로 풀어 보면 이렇게 나타난다.

| `goal` | 상황 | `at_goal` | `stalled` | `width` |
|---|---|---|---|---|
| `open` | 성공 | true | false | 최대 |
| `close` | 성공 (빈손) | true | false | 최소 |
| `close` | 뭔가 끼었다 | false | **true** | 중간 |
| **`grasp`** | **성공 (쥐었다)** | **true** | **true** | 중간 (물체 두께) |
| `grasp` | 실패 (헛닫힘) | false | false | **최소에서 정지** |
| (any) | 이동 중 | false | false | 변하는 중 |
| `""` | 명령 이전 | 무의미 | — | — |

**`grasp` 성공과 헛닫힘은 `at_goal` 이 이미 가른다** (성공 `true` / 헛닫힘 `false`).
`width` 는 그것을 **눈으로 확인하는 교차 검증**이다 — 물체 두께에서 멈췄는가, 최소
폭까지 닫혔는가. `width` 가 `NaN` 인 스택에서는 이 교차 검증만 없고 판정은 그대로다.

### 2.4 `effort` 를 넣지 않은 이유

손끝 파지력(N)을 실을지 검토했고 **넣지 않기로 했다.**

**오늘 기준으로 제대로 채울 수 있는 백엔드가 하나도 없다.** mock 은 `/joint_states.effort` 를 채우지 않아 항상 NaN, Isaac 은 미구현, 펑션베이는 관절 토크(N·m)만 있고 손끝 힘으로 바꾸려면 **개구 폭에 따라 변하는 링키지 모멘트 암**이 필요한데 기구 제원이 없다.

**항상 NaN 인 필드는 없는 것보다 나쁘다.** 스키마에 컬럼이 있으니 데이터처럼 보이는데 실제로는 비어 있다. `GripperCommand` 에서 `position`/`max_effort` 를 뺀 것과 같은 판단이다.

그리고 **"힘이 있다/없다"는 이미 `stalled` 가 답한다.** 단위가 정의되지 않은 숫자를 더하면 임계값이 이식되지 않는다 — `effort > 5` 가 어느 백엔드에서는 N·m 이고 다른 곳에서는 N 인데 필드만 봐서는 구분할 수 없다.

> **관절 토크를 그대로 싣거나 여러 관절의 평균을 쓰는 안은 채택하지 않았다.** 평균은 실제로 동작하지 않는다 — 그리퍼가 좌우 대칭이라 토크 부호가 반대여서 **상쇄된다.** 펑션베이 실측: 파지 시 관절 토크 `[0.0, +17.14, +5.10, 0.004, -17.19, -5.26]` N·m 의 평균은 **−0.035** 로, 빈손일 때의 −0.000 과 구분되지 않는다. 평균절대값(7.45 vs 0.002)을 써야 갈리는데 그것은 "평균"이 아니라 별도로 정의해야 하는 집계다.

**`stalled` 판정의 사후 검증이 필요하면** 백엔드 원시 채널을 기록한다 — 펑션베이는 `/output/gripper_joint`(6관절 q/v/f)를 `config/recording_topics.list` 에 넣으면 토크가 데이터셋에 남는다. 지금은 기록 대상이 아니다.

**나중에 다시 넣는 것은 공짜가 아니다.** ROS 메시지는 스키마 진화가 없어 필드를 더하면 타입이 바뀌고 옛 bag·DB 와 어긋난다. 필요해지면 그때 breaking change 를 한 번 더 치르는 것이며, 그 판단을 지금 미리 하지 않는다.

> **접촉력이 필요해지면 그리퍼가 아니라 FT 센서가 답한다.** 그리퍼 힘은 "얼마나 세게 쥐고 있나"(미끄러짐 감지)에 가깝고, Peg-in-Hole 의 삽입력은 다른 신호다. 현재 FT 는 **어디에도 없다** — 로봇 이름이 `panda_ftsensor_robotiq` 인데 URDF 에 FT 링크가 없고 (`robotiq2panda` 의 0.130 m 오프셋에 흡수), 시뮬레이터 출력도 `panda_joint` / `gripper_joint` / `endeffector` 셋뿐이다.

---

## 3. `GripperNode` — 노드 계약

**명령을 받는 노드가 상태도 낸다.** 두 일을 한 노드에 두는 이유는 `GripperState.goal` 때문이다 — 명령을 아는 쪽이 상태를 내면 목표가 **자연히 손에 있고 경합도 없다.** 나누면 상태 발행자가 명령 토픽을 따로 구독해야 하고, 명령 직후·발행 직전 구간에서 옛 목표가 실리는 창이 생긴다.

```
             ┌──────────────── GripperNode ────────────────┐
  ~/gripper_cmds  ─────▶ │ goal 해석 · targets 조회 · 실행 │
  (GripperCommand)       │                                  │
                         │  백엔드별 실행 · 상태 취득       │ ◀── 백엔드 채널
                         │                                  │
             ~/gripper_states ◀──── width · stalled · at_goal│
             (GripperState)        └────────────────────────┘
```

### 3.1 어느 구현이든 지켜야 하는 것

1. **`~/gripper_cmds` 를 구독**하고 `open`/`close`/`grasp` 를 처리한다. 모르는 심볼은 **거부하고 로그를 남긴다** — 조용히 무시하지 않는다.
2. **`~/gripper_states` 를 주기 발행**한다. 명령이 없어도 발행한다 — 연속 상태 채널이다.
3. `goal` 은 **마지막으로 받은 명령**, 없으면 `""`.
4. `width` 는 **개구 폭(m)**. 못 구하면 **NaN** (0 이 아니다).
5. `at_goal` 은 **의도의 달성 여부**이지 위치 도달이 아니다. 판정식은 §2.3 을 따르며 **매 주기 재평가**한다. `goal` 이 `""` 면 값의 의미가 없다. **`grasp` 의 판정은 `stalled` 에 딸리므로**, `stalled` 를 구현하지 못하면 파지 성공을 보고할 수 없다 — 그 사실을 백엔드 문서에 적는다.
6. `stalled` 판정 수단이 없으면 **`false`**.
7. 심볼 → 목표값 표는 **`targets.<goal>` 파라미터**로 노출한다. 기구를 아는 유일한 자리다.

### 3.2 구현체

**이름은 그 구현에서 변하지 않는 것을 가리킨다.** 그것이 무엇인지는 표준 인터페이스가
있느냐에 달렸다.

| | 이름이 가리키는 것 | 왜 |
|---|---|---|
| 표준 인터페이스가 **있다** | **인터페이스** | 그 뒤의 기구가 무엇이든 같은 코드다 |
| **없다** | **기구 계열** | 기구를 직접 구동하므로 불변항이 기구뿐이다 |

**백엔드로 가르지 않는다.** scene 계열(`MockSceneStateNode` / `IsaacSceneStateNode`)은
물체 목록을 얻는 방법이 백엔드마다 근본적으로 달라 그 관례가 맞지만, 그리퍼는 그렇지
않다 — Isaac 은 `isaac_gripper_bridge` 로 액션 서버를 열어 **mock 과 같은 노드**를 쓴다.
처음에 `MockGripperNode` 로 이름 붙였다가 Isaac 이 쓰는 순간 거짓이 되어
**2026-09-02 에 `GripperActionNode` 로 고쳤다.**

같은 날 `Robotiq2FGripperNode` 는 반대 방향으로 정리됐다. `GripperJointNode` 로
시작했으나 **"관절로 지령한다"는 변별력이 없다** — 그리퍼는 대체로 관절로 구현된다.
이 노드가 가릴 인터페이스가 없다는 것이 요점이므로 기구 계열을 이름에 실었다.
**모델(2F-85)이 아니라 계열(2F)** 인 것은 2F-140 이 링키지 구조가 같고 스트로크만 달라
`targets` 파라미터로 흡수되기 때문이다.

| 구현 | 실행 경로 | 쓰는 백엔드 | 상태 |
|---|---|---|:-:|
| [`GripperActionNode`](GripperActionNode_Guide.md) | `control_msgs/GripperCommand` **액션** (`/panda_hand_controller/gripper_cmd`) | mock · Gazebo · Isaac(브리지 경유) | ✅ |
| [`Robotiq2FGripperNode`](Robotiq2FGripperNode_Guide.md) | **관절 목표각 토픽** (6축 `Float64MultiArray` ← `JointState`) | 펑션베이 | ✅ |

**`stalled` 판정은 구현을 가르는 축이 아니다.** 임계 effort 하나가 다를 뿐이라
서브클래스로 나누면 액션 경로가 같은 코드가 두 벌이 된다 — **파라미터로 다룬다.**

기구 종속 수치도 마찬가지로 **전부 파라미터**다. `GripperActionNode` 의 기본값이
Panda Hand 전용(`panda_finger_joint1`, `width_scale 2.0`)이고 `Robotiq2FGripperNode` 의
기본값이 2F-85 전용(부호 벡터, `close: 0.725`)이지만, **코드에는 기구 종속 분기가
없다.** 이름의 차이는 코드가 얼마나 특수한가가 아니라 **가릴 인터페이스가 있느냐**에서
온다.

**`GripperActionNode` 가 액션 결과로 상태를 만들지 않는 것에 주의한다.** 결과는 명령당
1 건이라 "지금 어떤 상태인가"에 답할 수 없고, `at_goal` 은 매 주기 재평가여야 하기
때문이다 — 구현 상세는 [GripperActionNode_Guide.md](GripperActionNode_Guide.md).

---

## 4. 백엔드별 실현 가능성

같은 계약이라도 **채울 수 있는 정도가 다르다.** 무엇이 되고 무엇이 안 되는지를 미리
적어 둔다 — `false`/`NaN` 이 "판정 못 함"인지 "판정 결과"인지 구분해야 하기 때문이다.

| | `width` | `stalled` | `at_goal` (`open`/`close`) | `at_goal` (`grasp`) |
|---|---|---|---|---|
| **mock** | ✅ `panda_finger_joint1 × 2` | ❌ **항상 false** (판정 수단 없음) | ✅ | ❌ **항상 false** (§4.3) |
| **Isaac** | ✅ 관절 보고에서 환산 | ⚠️ 미구현 — effort 실측 후 임계값 결정 필요 | ✅ | ⚠️ `stalled` 에 딸림 |
| **펑션베이** | ❌ **NaN** — 2F-85 기구 매핑 미정 (§4.1) | ✅ **매우 명확** (§4.2) | ✅ **관절 잔차로 판정** | ✅ |

**`grasp` 의 `at_goal` 은 `stalled` 에 그대로 딸린다** (§2.3). 따라서 `stalled` 를
구현하지 못한 백엔드는 **파지 성공을 보고할 수 없다.**

### 4.1 펑션베이 — `width` 는 NaN 이다

시뮬레이터는 6개 관절 각도를 준다. 개구 폭으로 바꾸려면 2F-85 링키지 기하가 필요하며
**선형이 아니다.** 구동축(`finger_joint`)의 실측 범위는 `0 ~ 0.725 rad` 이고 스트로크
사양은 85 mm 이므로 대략적인 매핑은 세울 수 있으나, 정확도가 필요하면 제원을 받아야
한다.

**구현은 근사 대신 `NaN` 을 골랐다 (2026-09-02).** 근사값을 실으면 "폭을 안다"는 신호가
데이터셋에 남고, 나중에 제원을 받아 고칠 때 **과거 에피소드의 의미가 조용히 바뀐다.**

**`at_goal` 은 이 결정에 영향받지 않는다** — 관절 잔차로 판정하기 때문이다(§2.3).
`width` 가 없어 잃는 것은 `grasp` 성공과 헛닫힘의 **교차 검증**뿐이고, 판정 자체는
`stalled` 이 가른다. 이것이 §2.3 을 폭에서 떼어낸 이유이기도 하다.

### 4.2 펑션베이 — `stalled` 는 오히려 쉽다

실측한 분리도가 압도적이다.

| | 관절 토크 최대 | 위치 잔차 최대 |
|---|---|---|
| 빈손 폐쇄 | **0.004 N·m** | **0.0000 rad** |
| **파지** | **17.19 N·m** | **0.0859 rad** |

**토크 비가 4300배**이고, 신호가 **둘(토크·잔차)이라 서로 검증된다.** 임계값
`토크 > 1.0 N·m` 는 자유 이동 중 최대(0.845)보다 위, 파지(17.19)보다 한참 아래라 양쪽
모두 17배 이상 여유가 있다. 빈손·파지·파지유지·높이이탈 네 상황에서 오분류 0건을
확인했다.

파지 시 축이 두 그룹으로 갈리는 것도 근거가 된다 — **구동축은 목표에 정확히 도달하고,
물체에 닿는 축에만 잔차와 토크가 남는다.** 모터는 명령대로 돌았는데 손가락이 막힌
것이므로 물리적으로 옳은 그림이다.

계단 응답은 dead time 88 ms, τ 283 ms, 99% 도달 1051 ms 이므로 **판정 타임아웃은
1.5 초**면 충분하다.

### 4.3 mock — `grasp` 는 영영 성공으로 기록되지 않는다

mock 은 `stalled` 판정 수단이 없어 항상 `false` 이므로, §2.3 의 판정식에 따라 **`grasp`
의 `at_goal` 도 항상 `false`** 다.

**이것이 옳다.** mock 의 물체는 물리를 갖지 않아 파지에 실패해도 굴러떨어져도 pose 가
그대로이므로, **애초에 성패를 관측할 수 없는 백엔드**다 (`SceneObject.msg` 의 경고와
같은 사실이다). 여기서 `at_goal=true` 를 내면 "성공했다"는 거짓말이 데이터에 쌓인다.

> 위치 기준 정의였다면 mock 의 `grasp` 는 목표 폭에 도달해 **`at_goal=true`** 가 됐을
> 것이다. 의도 기준으로 바꾸면서 그 거짓말이 사라졌다 — 정의 변경의 부수 이득이다.

**mock 에서 수집한 에피소드로는 파지 성패를 라벨링할 수 없다.** 그 용도의 데이터는
물리가 있는 백엔드에서 모은다.

---

## 5. 소비자가 쓰는 법

**구현을 몰라도 된다.** 두 채널과 `at_goal` 하나면 된다 — 액션 스택이든 관절 지령
스택이든 같은 코드가 돈다.

### 5.1 명령을 보낸다

```python
from rdfp_msgs.msg import GripperCommand

# ⚠️ 퍼블리셔는 **기동 시점에 만들어 둔다.** 만들자마자 publish 하면 DDS 매칭 전이라
#    구독자가 없어 **첫 명령이 조용히 버려진다.** 로그에도 남지 않는다.
self._pub = node.create_publisher(GripperCommand, '/gripper_cmds', 10)

msg = GripperCommand()
# ⚠️ **stamp 를 반드시 채운다.** 비우면 epoch 0 에 적재되어 어느 에피소드에도
#    속하지 못한다. 로봇은 정상 동작하므로 데이터를 열어보기 전까지 드러나지 않는다.
msg.header.stamp = node.get_clock().now().to_msg()
msg.goal = 'grasp'          # 숫자는 싣지 않는다 — GripperNode 의 targets 가 푼다
self._pub.publish(msg)
```

모르는 심볼은 노드가 **거부한다**(§3.1-1). 보낼 수 있는 심볼은 그 노드의 `targets`
파라미터가 정하므로, 늘리려면 노드 쪽을 함께 고친다.

### 5.2 결과를 본다

```python
from rdfp_msgs.msg import GripperState

def on_state(msg: GripperState) -> None:
    if msg.goal == 'grasp' and msg.at_goal:
        print('파지 성공')

# ⚠️ 구독은 **명령을 보내기 전에** 만들어 둔다. 명령 직후에 만들면 DDS 매칭 동안
#    상태가 지나가 버린다.
node.create_subscription(GripperState, '/gripper_states', on_state, 10)
```

**`at_goal` 하나만 본다.** `goal` 별 판정식은 노드가 이미 적용했다(§2.3).

`/gripper_states` 는 **주기 발행**이므로 "갱신됐다"가 "명령이 끝났다"를 뜻하지 않는다.
완료를 기다리는 코드는 `goal` 이 자기 명령과 같고 `at_goal` 이 참인 스냅샷을 기다린다
— 세대만 보면 그리퍼가 움직이기도 전에 성공을 돌려준다(트윈이 실제로 그랬다).

`TRANSIENT_LOCAL` 이 아니므로 늦게 붙은 구독자는 다음 발행까지 기다린다.

### 5.3 하지 말 것

| | 왜 |
|---|---|
| `/joint_states` 의 손가락 관절로 폭을 읽는다 | 백엔드마다 믿을 수 없다 — 펑션베이는 TF 성립용 고정값을 주입해 "항상 열려 있다"고 거짓말한다(§1.2) |
| `stalled` 만 보고 파지 성공을 판정한다 | **관측이지 판정이 아니다.** 판정 수단이 없는 스택도 `false` 다(§2.2) |
| `at_goal=false` 를 곧바로 실패로 읽는다 | "진행 중"과 구분되지 않는다 — `width` 를 함께 본다 |
| 명령에 숫자를 싣는다 | 필드가 없다. 그리고 그것이 §1.1 의 요점이다 |

---

## 6. 데이터셋에서 어떻게 읽히나

두 채널이 학습 데이터의 **action / observation** 짝을 이룬다.

| 역할 | 토픽 | 내용 |
|---|---|---|
| action | `~/gripper_cmds` | 무엇을 시켰나 (심볼) |
| observation | `~/gripper_states` | 무엇이 되었나 (폭 · 파지 여부) |

**자동 라벨링의 파지 성공 판정은 `at_goal` 하나로 한다** — `goal == 'grasp'` 인 구간에서 `at_goal` 이 참이면 성공이다. `goal` 별 판정식은 노드가 이미 적용했으므로 소비자가 표를 들고 있을 필요가 없다(§2.3).

**단 `at_goal=false` 를 곧바로 실패로 집계하지 않는다** — 진행 중일 수도 있다. 실패로 확정하려면 `width` 가 최소에서 정지했는지 함께 본다. 그리고 `stalled` 를 구현하지 못한 백엔드(mock)에서는 `grasp` 가 **구조적으로 성공하지 않으므로**, 그 데이터로는 파지 성패를 라벨링하지 않는다(§4.3).

`GripperState` 는 `config/recording_topics.list` 에 있어야 한다. **`/joint_states` 로 대신할 수 없는 이유는 §1.2** 에 있다.

---

## 7. 현행 구현과 무엇이 다른가

| | 현행 | 이 설계 |
|---|---|---|
| 명령 필드 | `label` | **`goal`** |
| 상태 폭 | `position` | **`width`** (이름이 관절값처럼 보이던 문제) |
| 상태 목표 | 없음 | **`goal` 추가** — 상태가 자기 완결적이 된다 |
| 성공 여부 | 없음 (이벤트 메시지에만, 위치 기준) | **`at_goal`** — **의도** 기준, 매 주기 재평가 |
| 힘 | `GripperState.effort` | **제거** (§2.4) |
| 이벤트 메시지 | `GripperActionState` (표준) | **표준에서 제거.** 필요하면 구현 내부 사정으로 |
| 상태 발행자 | `gripper_state_publisher` (`/joint_states` 파생) | **`GripperNode` 가 직접** |

`GripperActionState` 를 표준에서 빼는 이유는 `GripperState` 와 **`status` 를 뺀 전부가 겹치기 때문**이다. 같은 사실을 다른 이름(`position`/`width`)으로 두 채널이 실으면 소비자가 어느 쪽을 믿을지 모르고, 하나는 이벤트·하나는 주기라 **값이 어긋나는 구간이 반드시 생긴다.**

---

## 8. 반영 범위

**코드·설정은 반영됐다 (2026-09-02).** 문서 일부가 남는다.

| 대상 | 할 일 |
|---|---|
| 메시지 | ✅ `GripperCommand.goal`, `GripperState` 재정의. `GripperActionState` 는 **삭제** |
| 노드 | ✅ `GripperActionNode` (`gripper_action_node`) — mock·Gazebo·Isaac. ✅ `Robotiq2FGripperNode` (`robotiq_2f_gripper_node`) — 펑션베이. 구 `gripper_control_node`·`gripper_state_publisher` 제거 |
| DB | ✅ `gripper_cmds.goal`, `gripper_states` 를 `goal`/`width`/`stalled`/`at_goal` 로 재정의. `gripper_action_states` 테이블 삭제 (`drop.sql` 은 구 DB 정리용으로 남겼다) |
| writer/reader | ✅ `gripper_command`·`gripper_state` 갱신, `gripper_action_state` 삭제 |
| 트윈 | ✅ `gripper_state`(`/gripper_states`) 로 이전. **완료 판정을 세대→`at_goal` 로 바꿨다** — 주기 발행 채널에서 "갱신됨"은 "끝남"이 아니다 |
| 설정 | ✅ `config/recording_topics.list` (`/gripper_cmds`·`/gripper_states`), 트윈 YAML 2개 |
| 문서 | ✅ 전 범위 반영. 구 `GripperControlNode_Guide.md` 는 **삭제**했다 — 설명하던 노드·메시지가 전부 없어졌고, 남은 소비자 관점은 §5 로 옮겼다 |

**전 구간이 breaking change 다.** 옛 bag·DB 는 이전 필드명으로 남으므로 reader 에 호환 경로를 둘지 결정해야 한다.

### 미결

- `GripperActionState` 의 `status` 가 갖던 **`CANCELED`(후속 명령에 의한 선점) vs `ABORTED`(실패) 구분**이 사라졌다. `at_goal` 은 둘 다 `false` 로 본다. 선점을 실패로 오독하지 않으려면 어딘가에 남아야 하는지 판단이 필요하다.
- **트윈의 `move_gripper_to_target grasp` 는 mock 에서 타임아웃한다.** mock 은 `stalled` 판정 수단이 없어 `at_goal` 이 서지 않는다(§4.3). 완료를 정직하게 판정한 대가이며, 대안은 (a) 그대로 두고 mock 에서 grasp 를 쓰지 않기, (b) `sync_timeout_sec` 경과 시 `at_goal=false` 로 성공 반환, (c) mock 에 가짜 stall 을 넣기 — 셋 다 각각의 거짓말이 있어 결정이 필요하다.
- **`stalled` 임계값을 파라미터로 여는 일이 남았다.** Isaac 실측이 선행이며, 기본값은 '판정 안 함'이어야 mock 동작이 그대로다 (§3.2).
- 펑션베이 `width` — 지금은 `NaN` 이다. 제원을 받아 채울지, 계속 비워 둘지 결정이 남았다 (§4.1).
- Isaac `stalled` 임계값 — effort 실측이 선행되어야 한다.

---

## 9. 관련 문서

- [GripperActionNode_Guide.md](GripperActionNode_Guide.md) — 액션 기반 구현 (mock · Gazebo · Isaac)
- [Robotiq2FGripperNode_Guide.md](Robotiq2FGripperNode_Guide.md) — Robotiq 2F 직접 구동 (펑션베이)
- [gripper_action_server_notes.md](gripper_action_server_notes.md) — mock 액션 서버 쪽 사정
- [../simulation/functionbay_backend_design.md](../simulation/functionbay_backend_design.md)
  §6 — 펑션베이 그리퍼 채널 실측 사양(입출력 형식 · 부호 · mimic 미강제 · 파지 판정)
- [../scene/scene_objects_guide.md](../scene/scene_objects_guide.md) — `width` 를 물리량으로
  둔 덕에 직접 비교되는 대상(`scene/objects` 치수)
