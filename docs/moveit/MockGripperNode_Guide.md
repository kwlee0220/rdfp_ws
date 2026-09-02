# MockGripperNode — mock 스택의 `GripperNode` 구현

[GripperNode 설계](GripperNode_Design.md)가 정한 계약을 **`control_msgs/GripperCommand`
액션 서버가 있는 스택**에서 실현한 노드다. 계약이 무엇을 요구하는지는 설계서가, 이
문서는 **그 요구를 어떻게 만족시켰고 무엇을 만족시키지 못하는지**를 다룬다.

| | |
|---|---|
| 패키지 | `robot_control` |
| 실행 파일 | `mock_gripper_node` |
| 노드 이름 | `gripper` (launch 가 지정) |
| 소스 | [mock_gripper_node.py](../../src/robot_control/robot_control/gripper/mock_gripper_node.py) |

> **이름이 `Mock` 이지만 mock 전용이 아니다.** 액션 서버만 있으면 되므로 Isaac 도 이
> 노드를 쓴다 (§7). 반대로 액션 서버가 없는 펑션베이에는 쓸 수 없다.

---

## 1. 채널

```text
  /gripper_cmds  (rdfp_msgs/GripperCommand)      심볼 'open' / 'close' / 'grasp'
      │
      ▼
  ┌──────────────────────── MockGripperNode ────────────────────────┐
  │  targets 파라미터로 심볼 → (관절값, 힘)                          │
  │                                                                 │
  │  /joint_states 의 finger joint × width_scale → 개구 폭          │
  │  폭 + goal → at_goal 판정 (매 주기)                              │
  └─────────────────────────────────────────────────────────────────┘
      │                                         │
      ▼                                         ▼
  /panda_hand_controller/gripper_cmd        /gripper_states
  (control_msgs/GripperCommand 액션)        (rdfp_msgs/GripperState, 10 Hz)
```

| 방향 | 이름 | 타입 | 비고 |
|---|---|---|---|
| 구독 | `gripper_cmds` | `rdfp_msgs/GripperCommand` | **루트 상대** (`~/` 가 아니다 — [토픽 규약](../topic_naming_contract.md) §2.2) |
| 발행 | `gripper_states` | `rdfp_msgs/GripperState` | 루트 상대. 명령이 없어도 주기 발행 |
| 구독 | `/joint_states` | `sensor_msgs/JointState` | **절대 경로** — 스택 공용 채널이라 remap 대상이 아니다 |
| 액션 클라이언트 | `/panda_hand_controller/gripper_cmd` | `control_msgs/GripperCommand` | 절대 경로 (컨트롤러가 여는 이름) |

### QoS — `gripper_states` 는 `TRANSIENT_LOCAL` 이 아니다

depth 10 의 기본 QoS(reliable · volatile)다. **늦게 붙은 구독자는 다음 발행까지
기다린다** — `publish_rate` 기본값에서 최대 100 ms 다. `/session` 처럼 상태를 즉시
받아야 하는 채널이 아니라 연속 스트림이므로 지연을 감수한다.

---

## 2. 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `targets.open` | `[0.035, 10.0]` | 심볼 → `[position(m), max_effort(N)]` |
| `targets.close` | `[0.0, 10.0]` | 빈손으로 닫는다 |
| `targets.grasp` | `[0.0, 30.0]` | `close` 와 자세가 같고 **힘이 다르다** |
| `finger_joint` | `panda_finger_joint1` | 폭을 계산할 손가락 관절 |
| `width_scale` | `2.0` | 관절값 → 개구 폭 배수 |
| `width_tolerance` | `0.005` | `at_goal` 판정의 폭 허용오차 (m) |
| `publish_rate` | `10.0` | `gripper_states` 발행 Hz |

**`targets` 의 `position` 은 관절값이지 개구 폭이 아니다.** 액션 goal 이 그 단위를
받기 때문이다. `open` 의 `0.035` 는 SRDF `<group_state group="hand" name="open">` 의
값과 같다 — 즉 RViz 의 `hand` 그룹 `open` 과 같은 자세로 간다.

`targets` 의 숫자가 **이 노드에 있는 이유**는 그리퍼에 종속이기 때문이다. 명령에는
의도(심볼)만 실려 오고, 그 로봇의 수치는 기구를 아는 여기에 남는다. 그리퍼를 바꾸면
이 파라미터만 고치면 되고 트윈 설정·데이터셋·teleop 은 그대로다.

### 기동 시 검증

| 조건 | 동작 |
|---|---|
| `targets.<goal>` 의 길이가 2 가 아니다 | `ValueError` — **노드가 뜨지 않는다** |
| `publish_rate <= 0` | `ValueError` — 뜨지 않는다 |

둘 다 조용히 넘기면 "그리퍼만 안 움직인다"로 나타나 원인을 찾기 어렵다.

---

## 3. 명령 경로 — 심볼을 액션 goal 로

```python
goal = str(msg.goal or '').strip()
target = self._targets.get(goal)
if target is None:
    self.get_logger().error(f'[gripper] unknown goal {goal!r}; known: {sorted(self._targets)}')
    return                                   # ← goal 을 갱신하지 않는다
```

**모르는 심볼은 거부한다.** 조용히 무시하면 팔은 움직이는데 그리퍼만 안 움직이는
상태가 되고, 로그에 아무것도 없어 원인이 보이지 않는다.

거부한 명령은 `GripperState.goal` 에 **싣지 않는다.** 실행되지 않은 목표를 상태에
실으면 `at_goal` 이 있지도 않은 목표를 판정하게 된다. 액션 서버가 아직 준비되지
않았을 때도 같다 — 경고만 남기고 `goal` 은 그대로 둔다.

### 결과는 로그로만 남는다

`_on_result` 는 액션 status 를 로그로 남길 뿐 상태를 만들지 않는다.

**결과로 상태를 만들지 않는 이유가 두 가지다.**

1. 결과는 **명령당 1 건**이라 "지금 어떤 상태인가"에 답할 수 없다.
2. `at_goal` 은 **매 주기 재평가**여야 한다 — 물체를 놓쳐 손이 벌어지면 다시 `false`
   가 되어야 하는데, 결과 기반이면 마지막 결과에 얼어붙는다.

그래서 상태는 전적으로 `/joint_states` 에서 만든다.

> `status` 가 `CANCELED` 면 실패가 아니라 **후속 명령에 의한 선점**이다. 다만 그
> 구분은 이제 로그에만 있다 — 싣고 다니던 `GripperActionState` 토픽은 2026-09-02 에
> 삭제됐다 ([설계서](GripperNode_Design.md) §7 미결).

---

## 4. 상태 경로 — 관절값에서 폭으로, 폭에서 판정으로

### 4.1 `width` — 관절값 × `width_scale`

```python
idx = list(msg.name).index(self._finger_joint)
self._width = float(msg.position[idx]) * self._width_scale
```

관절이 없거나 인덱스가 범위를 벗어나면 **갱신하지 않는다.** 초기값은 `NaN` 이므로
`/joint_states` 가 오기 전이나 손가락 관절이 실리지 않는 스택에서는 계속 `NaN` 이다.

**`0` 을 넣지 않는 것이 핵심이다.** 0 은 "닫혀 있다"는 거짓말이 되고, 펑션베이가
`/joint_states` 에 고정값을 주입해 만드는 오염과 같은 종류다. `NaN` 은 "모른다"가
사실대로 남는다.

### 4.2 `at_goal` — 단위를 맞춰 비교한다

```python
def _target_width(self, goal):
    target = self._targets.get(goal)
    return math.nan if target is None else target[0] * self._width_scale
```

**`targets` 는 관절값, `width` 는 개구 폭이라 그대로 비교하면 안 된다.** 실제로
초기 구현이 그대로 비교했고, `open` 은 폭 0.070 을 목표 0.035 와 견주어 **영영
`at_goal` 이 서지 않았다.** `close` 는 양쪽 다 0 이라 우연히 맞아 증상이 반쪽만
드러났다. 지금은 `_target_width()` 가 변환을 맡고, 회귀 테스트가 "관절값을 그대로
견주면 `False`" 를 고정한다.

판정식은 [설계서](GripperNode_Design.md) §2.3 그대로다.

| `goal` | 조건 |
|---|---|
| `''` (명령 이전) | `false` — 판정 대상이 없다 |
| `width` 가 `NaN` | `false` — 모르면 도달을 주장하지 않는다 |
| `open` / `close` | `\|width − 목표 폭\| ≤ width_tolerance` **AND NOT** `stalled` |
| `grasp` | `stalled` |
| `targets` 에 없는 심볼 | `false` — 애초에 명령이 거부됐어야 한다 |

---

## 5. mock 의 한계 — `grasp` 는 성공으로 기록되지 않는다

`_stalled()` 는 **항상 `False`** 를 돌려준다. 판정 수단이 없기 때문이다.

근거는 하드웨어 기술에 있다 —
[panda_hand.ros2_control.xacro](../../src/robot_control/description/panda_hand.ros2_control.xacro)
가 손가락 관절에 선언하는 state interface 는 `position` 과 `velocity` 뿐이고
**`effort` 가 없다.** 따라서 `joint_state_broadcaster` 가 내는 `/joint_states` 의
`effort` 배열은 비어 있고, "힘을 내는데 안 움직인다"를 관측할 방법이 없다.

`grasp` 의 판정식이 곧 `stalled` 이므로 **`grasp` 의 `at_goal` 도 항상 `False`** 다.

**이것이 옳다.** mock 의 물체는 물리를 갖지 않아 파지에 실패해도, 굴러떨어져도 pose 가
그대로다 — **애초에 성패를 관측할 수 없는 백엔드**다. 위치 기준으로 정의했다면 목표
폭 0.0 에 도달해 `True` 가 됐겠지만, 그것은 "성공했다"는 거짓말이 데이터에 쌓이는
길이다. 기동 시 이 사실을 경고로 남긴다.

```text
[WARN] stalled is always False on mock (no force sensing) — therefore
       'grasp' never reports at_goal=True. mock cannot observe grasp success.
```

### 파급 — 두 곳에서 증상으로 나타난다

| 어디서 | 무슨 일이 | 대응 |
|---|---|---|
| 트윈 `move_gripper_to_target grasp` | `at_goal` 을 기다리다 `sync_timeout_sec`(기본 5 초) 타임아웃 | mock 에서는 `open`/`close` 만 쓴다 |
| 데이터셋 `gripper_states.at_goal` | `grasp` 구간이 전부 `false` | mock 수집분으로 파지 성패를 학습 신호로 쓰지 않는다 |

`stalled` 을 실제로 판정하려면 백엔드가 힘을 실을 수 있어야 한다. 펑션베이는 관절
토크로 가능하고(파지 17 N·m vs 빈손 0.004 N·m), Isaac 은 가능하지만 임계값을 아직
실측하지 않았다 ([설계서](GripperNode_Design.md) §4).

---

## 6. 실행과 확인

### 기동

launch 가 띄우므로 직접 실행할 일은 드물다.

```bash
ros2 run robot_control mock_gripper_node --ros-args -r __node:=gripper
```

**띄우는 launch** — `panda_mock`, `panda_jgpc_mock`, `panda_gazebo`,
`rdfp_panda_mock`, `rdfp_panda_jgpc_mock`, `replay_panda_mock` 은 항상,
`panda_isaac` / `rdfp_panda_isaac` 은 `enable_gripper:=true` 일 때
(`isaac_gripper_bridge` 와 함께).

`panda_functionbay` 는 **띄우지 않는다** — 액션 서버가 없어 이 구현을 쓸 수 없다.

### 명령과 확인

```bash
# 명령 — 심볼만 보낸다
ros2 topic pub --once /gripper_cmds rdfp_msgs/msg/GripperCommand \
    "{header: {stamp: {sec: 0}}, goal: 'open'}"

# 상태 — 이것 하나로 성패를 본다
ros2 topic echo /gripper_states --once
```

```text
goal: 'open'
width: 0.07        ← 개구 폭(m). 관절값 0.035 의 2배다
stalled: false
at_goal: true
```

**`header.stamp` 를 비우지 않는다.** 적재 코드가 이 값을 그대로 쓰므로 비면 epoch 0
에 적재되어 어느 에피소드에도 속하지 못한다. 로봇은 정상 동작하므로 데이터를 열어보기
전까지 드러나지 않는다.

### 파라미터 조정

```bash
ros2 param get /gripper targets.grasp          # [0.0, 30.0]
```

목표를 늘리려면 launch 에서 파라미터를 넘긴다.

```python
create_gripper_node({'targets.pinch': [0.015, 10.0]})
```

⚠️ 새 심볼은 **양쪽**에 있어야 한다 — 트윈에서 부르려면 `backend.labels` 에도 추가해야
하고, 없으면 이 노드가 명령을 거부한다.

---

## 7. Isaac 도 이 노드를 쓴다

Isaac 에는 ros2_control 이 없어 `panda_hand_controller` 액션 서버가 없다. 대신
`isaac_gripper_bridge` 가 **같은 이름의 액션 서버**를 열고 받은 목표를 관절 위치
토픽으로 바꾼다. 명령 경로가 mock 과 동일해지므로 이 노드를 그대로 쓴다.

**`IsaacGripperNode` 는 미구현이다.** 달라야 하는 것은 `stalled` 판정 하나뿐이고
(Isaac 은 effort 를 실을 수 있다), 임계값 실측이 선행 작업이다. 그때까지 Isaac 의
파지 여부는 데이터에 남지 않는다.

---

## 8. 트러블슈팅

| 증상 | 원인 | 확인 |
|---|---|---|
| 명령을 보내도 아무 반응이 없다 | 모르는 심볼 | 노드 로그의 `unknown goal ...; known: [...]` |
| 〃 | 액션 서버 미기동 | 로그의 `action server ... not ready`, `ros2 action list \| grep gripper_cmd` |
| `at_goal` 이 `grasp` 에서 계속 `false` | **mock 의 정상 동작** (§5) | 기동 시 WARN 로그 |
| `width` 가 `NaN` | `/joint_states` 미수신 또는 `finger_joint` 이름 불일치 | `ros2 topic echo /joint_states --once \| grep -A2 finger` |
| `at_goal` 이 `open` 에서 서지 않는다 | `targets` 와 `width_scale` 이 어긋났다 | `width` 가 `targets.open[0] × width_scale` 근처인지 |
| 노드가 뜨지 않는다 | `targets.<goal>` 길이 ≠ 2, 또는 `publish_rate <= 0` | 기동 로그의 `ValueError` |
| 상태 조회가 잠깐 비어 있다 | `TRANSIENT_LOCAL` 이 아니다 (§1) | 다음 발행까지 최대 `1/publish_rate` 초 |

**그리퍼가 안 움직일 때 계층부터 가른다.** 명령 토픽(`/gripper_cmds`) → 이 노드 →
액션(`/panda_hand_controller/gripper_cmd`) → 컨트롤러 순이며, 액션 계층 자체의 함정은
[gripper_action_server_notes.md](gripper_action_server_notes.md) 가 다룬다.

---

## 9. 관련 문서

- [GripperNode_Design.md](GripperNode_Design.md) — **계약 정본.** 메시지 필드의 뜻,
  `at_goal` 판정식, 백엔드별 실현 가능성, 데이터셋에서 읽는 법
- [gripper_action_server_notes.md](gripper_action_server_notes.md) — 그 아래 액션 서버
  계층. 컨트롤러 파라미터, feedback 이 오지 않는 이유, 선점 동작
- [../topic_naming_contract.md](../topic_naming_contract.md) §2.2 — 채널 이름이 `~/`
  가 아니라 루트 상대인 이유
- [../../src/robot_control/launch/README.md](../../src/robot_control/launch/README.md)
  — `launch_helpers/gripper.py` 와 기동 순서
