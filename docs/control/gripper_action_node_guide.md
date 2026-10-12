# GripperActionNode — 액션 기반 `GripperNode` 구현

[GripperNode 설계](gripper_node_design.md)가 정한 계약을 **`control_msgs/GripperCommand` 액션**으로 실현한 노드다. 계약이 무엇을 요구하는지는 설계서가, 이 문서는 **그 요구를 어떻게 만족시켰고 무엇을 만족시키지 못하는지**를 다룬다.

| | |
|---|---|
| 패키지 | `robot_control` |
| 실행 파일 | `gripper_action_node` |
| 노드 이름 | `gripper` (launch 가 지정) |
| 소스 | [gripper_action_node.py](../../src/robot_control/robot_control/gripper/gripper_action_node.py) |

**백엔드가 아니라 실행 수단으로 이름 붙였다.** 액션 서버만 있으면 되므로 mock 과 Isaac 이 같은 노드를 쓰고(§7), 액션 서버가 없는 펑션베이는 쓸 수 없다.

> **예전 이름은 `MockGripperNode` 였다 (2026-09-02 개명).** `MockSceneStateNode` / `IsaacSceneStateNode` 처럼 백엔드 접두사를 붙이는 관례를 따랐는데, 그리퍼는 **갈리는 축이 백엔드가 아니라 실행 수단**이라 Isaac 이 이 노드를 쓰는 순간 이름이 거짓이 됐다. scene 은 백엔드마다 물체를 얻는 방법이 근본적으로 달라 그 관례가 맞지만, 그리퍼는 액션 서버만 있으면 백엔드가 무엇이든 같은 코드다.

---

## 1. 채널

```text
  /gripper_cmds  (rdfp_msgs/GripperCommand)      심볼 'open' / 'close' / 'grasp'
      │
      ▼
  ┌─────────────────────── GripperActionNode ───────────────────────┐
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

depth 10 의 기본 QoS(reliable · volatile)다. **늦게 붙은 구독자는 다음 발행까지 기다린다** — `publish_rate` 기본값에서 최대 100 ms 다. `/session` 처럼 상태를 즉시 받아야 하는 채널이 아니라 연속 스트림이므로 지연을 감수한다.

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
| `stall_effort` | `0.0` | `stalled` 판정 토크 임계 (N·m). **0 이면 판정하지 않는다** |
| `stall_velocity` | `0.0` | 속도 상한 (rad/s). **0 이하면 속도 조건을 쓰지 않는다** |
| `publish_rate` | `10.0` | `gripper_states` 발행 Hz |

**`targets` 의 `position` 은 관절값이지 개구 폭이 아니다.** 액션 goal 이 그 단위를 받기 때문이다. `open` 의 `0.035` 는 SRDF `<group_state group="hand" name="open">` 의 값과 같다 — 즉 RViz 의 `hand` 그룹 `open` 과 같은 자세로 간다.

`targets` 의 숫자가 **이 노드에 있는 이유**는 그리퍼에 종속이기 때문이다. 명령에는 의도(심볼)만 실려 오고, 그 로봇의 수치는 기구를 아는 여기에 남는다. 그리퍼를 바꾸면 이 파라미터만 고치면 되고 트윈 설정·데이터셋·teleop 은 그대로다.

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

**모르는 심볼은 거부한다.** 조용히 무시하면 팔은 움직이는데 그리퍼만 안 움직이는 상태가 되고, 로그에 아무것도 없어 원인이 보이지 않는다.

거부한 명령은 `GripperState.goal` 에 **싣지 않는다.** 실행되지 않은 목표를 상태에 실으면 `at_goal` 이 있지도 않은 목표를 판정하게 된다. 액션 서버가 아직 준비되지 않았을 때도 같다 — 경고만 남기고 `goal` 은 그대로 둔다.

### 결과는 로그로만 남는다

`_on_result` 는 액션 status 를 로그로 남길 뿐 상태를 만들지 않는다.

**결과로 상태를 만들지 않는 이유가 두 가지다.**

1. 결과는 **명령당 1 건**이라 "지금 어떤 상태인가"에 답할 수 없다.
2. `at_goal` 은 **매 주기 재평가**여야 한다 — 물체를 놓쳐 손이 벌어지면 다시 `false`가 되어야 하는데, 결과 기반이면 마지막 결과에 얼어붙는다.

그래서 상태는 전적으로 `/joint_states` 에서 만든다.

> `status` 가 `CANCELED` 면 실패가 아니라 **후속 명령에 의한 선점**이다. 다만 그 구분은 이제 로그에만 있다 — 싣고 다니던 `GripperActionState` 토픽은 2026-09-02 에 삭제됐다 ([설계서](gripper_node_design.md) §7 미결).

---

## 4. 상태 경로 — 관절값에서 폭으로, 폭에서 판정으로

### 4.1 `width` — 관절값 × `width_scale`

```python
idx = list(msg.name).index(self._finger_joint)
self._width = float(msg.position[idx]) * self._width_scale
```

관절이 없거나 인덱스가 범위를 벗어나면 **갱신하지 않는다.** 초기값은 `NaN` 이므로 `/joint_states` 가 오기 전이나 손가락 관절이 실리지 않는 스택에서는 계속 `NaN` 이다.

**`0` 을 넣지 않는 것이 핵심이다.** 0 은 "닫혀 있다"는 거짓말이 되고, 펑션베이가 `/joint_states` 에 고정값을 주입해 만드는 오염과 같은 종류다. `NaN` 은 "모른다"가 사실대로 남는다.

### 4.2 `at_goal` — 단위를 맞춰 비교한다

```python
def _target_width(self, goal):
    target = self._targets.get(goal)
    return math.nan if target is None else target[0] * self._width_scale
```

**`targets` 는 관절값, `width` 는 개구 폭이라 그대로 비교하면 안 된다.** 실제로 초기 구현이 그대로 비교했고, `open` 은 폭 0.070 을 목표 0.035 와 견주어 **영영 `at_goal` 이 서지 않았다.** `close` 는 양쪽 다 0 이라 우연히 맞아 증상이 반쪽만 드러났다. 지금은 `_target_width()` 가 변환을 맡고, 회귀 테스트가 "관절값을 그대로 견주면 `False`" 를 고정한다.

| `goal` | 조건 |
|---|---|
| `''` (명령 이전) | `false` — 판정 대상이 없다 |
| `width` 가 `NaN` | `false` — 모르면 도달을 주장하지 않는다 |
| `open` / `close` | `\|width − 목표 폭\| ≤ width_tolerance` **AND NOT** `stalled` |
| `grasp` | `stalled` |
| `targets` 에 없는 심볼 | `false` — 애초에 명령이 거부됐어야 한다 |

**폭으로 재는 것은 계약이 아니라 이 구현의 선택이다.** [설계서](gripper_node_design.md)
§2.3 이 요구하는 것은 "목표 **자세** 도달"이고, 무엇으로 재는지는 구현이 정한다. 이
노드가 폭을 쓰는 이유는 관측이 `/joint_states` 의 손가락 관절 하나뿐이기 때문이다.

그래서 위 표의 둘째 줄 — **`width` 가 `NaN` 이면 `at_goal` 도 `false`** — 은 **이 구현
한정**이다. 지령과 보고가 같은 관절 공간인 백엔드(펑션베이)는 관절 잔차로 재므로
`width=NaN` 이어도 `at_goal` 을 정확히 판정한다.

---

## 5. `stalled` — 파라미터로 켠다

`stall_effort` 가 **0 이면 판정하지 않고** 항상 `False` 다. 그때의 `False` 는
**"물지 않았다"가 아니라 "모른다"** 는 뜻이며, `grasp` 의 판정식이 곧 `stalled` 이므로
**`grasp` 의 `at_goal` 도 항상 `False`** 가 된다.

| 백엔드 | 설정 | 왜 |
|---|---|---|
| **Isaac** | `stall_effort: 1.0` | ✅ 실측으로 켰다 (아래) |
| **mock** | (끔) | ❌ [panda_hand.ros2_control.xacro](../../src/robot_control/description/panda_hand.ros2_control.xacro) 가 손가락에 선언하는 state interface 가 `position`/`velocity` 뿐이라 `/joint_states` 에 effort 가 없다. 게다가 planning scene 물체는 물리를 갖지 않아 **애초에 성패를 관측할 수 없다** |

### 5.1 Isaac 실측 (2026-09-02)

| 상황 | \|effort\| 최대 |
|---|---|
| 열림·빈손 폐쇄 (정지) | **0.001** N·m |
| 개폐 이동 중 | 0.175 |
| 빈손 폐쇄 + **팔을 흔듦** | **0.126** (정착 과도 0.503) |
| **블록 파지 유지** | **22.4** |

임계 `1.0` 은 거짓 양성 최악값(0.503)의 **2배**, 파지의 **1/22** 지점이다. 펑션베이가
쓰는 값과 같다.

### 5.2 ⚠️ 속도 조건은 Isaac 에서 끈다

**파지 중에도 손가락이 계속 움직인다** — PhysX 접촉에서 떨리기 때문이다.

```
파지 유지 중 |velocity| : 평균 0.008,  최대 0.258 rad/s
  0.002 rad/s 를 넘는 샘플이 96.5%
```

펑션베이는 링키지가 멎어 속도가 0 이 되므로 "힘을 내는데 안 움직인다"가 그대로
성립하지만, **Isaac 에서 같은 조건을 걸면 파지의 96.5% 를 놓친다.** 그래서
`stall_velocity` 는 기본 0(미사용)이고 필요한 스택만 켠다.

### 5.3 계약 케이스 검증 (실기, 2026-09-02)

| 명령 | 물체 | `stalled` | `at_goal` | |
|---|:-:|:-:|:-:|---|
| (명령 이전) | 문 채 | true | **false** | `goal` 이 없으면 무의미 |
| `grasp` | **있음** | **true** | **true** | **파지 성공** |
| `open` | — | false | true | 개방 성공 |
| `close` | 없음 | false | true | 빈손 폐쇄 성공 |
| `grasp` | 없음 | false | **false** | **헛닫힘** |

### 5.4 판정을 끈 스택(mock)의 파급

| 어디서 | 무슨 일이 | 대응 |
|---|---|---|
| 트윈 `move_gripper_to_target grasp` | `at_goal` 을 기다리다 `sync_timeout_sec`(기본 5 초) 타임아웃 | mock 에서는 `open`/`close` 만 쓴다 |
| 데이터셋 `gripper_states.at_goal` | `grasp` 구간이 전부 `false` | mock 수집분으로 파지 성패를 라벨링하지 않는다 |

**Isaac 에서는 둘 다 해소됐다** — `stall_effort` 를 켜면 `grasp` 가 정상 완료된다.

참고로 펑션베이는 관절 토크로 판정 가능하다 (파지 17 N·m vs 빈손 0.004 N·m) — 다만 액션 서버가 없어 이 노드를 쓸 수 없다 ([설계서](gripper_node_design.md) §4).

---

## 6. 실행과 확인

### 기동

launch 가 띄우므로 직접 실행할 일은 드물다.

```bash
ros2 run robot_control gripper_action_node --ros-args -r __node:=gripper
```

**띄우는 launch** — `panda_mock`, `panda_mock_jgpc`, `panda_gazebo`, `replay_panda_mock` 은 항상, `panda_isaac` 은 `enable_gripper:=true` 일 때 (`panda_hand_controller` 와 함께).

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

**`header.stamp` 를 비우지 않는다.** 적재 코드가 이 값을 그대로 쓰므로 비면 epoch 0에 적재되어 어느 에피소드에도 속하지 못한다. 로봇은 정상 동작하므로 데이터를 열어보기 전까지 드러나지 않는다.

### 파라미터 조정

```bash
ros2 param get /gripper targets.grasp          # [0.0, 30.0]
```

목표를 늘리려면 launch 에서 파라미터를 넘긴다.

```python
create_gripper_node({'targets.pinch': [0.015, 10.0]})
```

⚠️ 새 심볼은 **양쪽**에 있어야 한다 — 트윈에서 부르려면 `backend.labels` 에도 추가해야 하고, 없으면 이 노드가 명령을 거부한다.

---

## 7. Isaac 도 이 노드를 쓴다 — 이름이 그래서 `Action` 이다

Isaac 에는 ros2_control 이 없어 `panda_hand_controller` 액션 서버가 없다. 대신 `panda_hand_controller` 가 **같은 이름의 액션 서버**를 열고 받은 목표를 관절 위치 토픽으로 바꾼다. 명령 경로가 mock 과 동일해지므로 이 노드를 그대로 쓴다.

```text
mock    /gripper_cmds → GripperActionNode → gripper_cmd 액션 → panda_hand_controller
Isaac   /gripper_cmds → GripperActionNode → gripper_cmd 액션 → panda_hand_controller
                                                                → /isaac/gripper_command
```

**갈리는 축이 백엔드가 아니라 실행 수단**이라는 것이 이 그림이다. 두 스택은 액션 서버를 누가 제공하느냐만 다르고, 그 위는 같은 코드다. 백엔드별로 노드를 나누는 `MockSceneStateNode`/`IsaacSceneStateNode` 관례가 그리퍼에는 맞지 않는 이유다.

남은 차이는 `stalled` 판정 하나이며, 그것도 서브클래스가 아니라 파라미터로 다룰 일이다 (§5).

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

**그리퍼가 안 움직일 때 계층부터 가른다.** 명령 토픽(`/gripper_cmds`) → 이 노드 → 액션(`/panda_hand_controller/gripper_cmd`) → 컨트롤러 순이며, 액션 계층 자체의 함정은 [gripper_action_node_guide.md](gripper_action_node_guide.md) 가 다룬다.

---

## 9. 액션 서버 계층 — `/panda_hand_controller/gripper_cmd` 동작 노트

Panda gripper 를 실제로 구동하는 **ros2_control 액션 서버** 계층의 동작 정리. 그 위에 얹힌 인터페이스(`GripperCommand`/`GripperState`)와 노드 계약은 [gripper_node_design.md](gripper_node_design.md) 를 본다.

> 본 문서의 실측값은 `panda_mock.launch.py` (hardware_type = `mock_components`)
> 환경에서 확인한 것이다. 실물 하드웨어에서 달라질 수 있는 항목은 그때그때
> 표시했다 — 다만 [feedback 미발행](#2-feedback-은-발행되지-않는다) 은 컨트롤러
> 구현에서 오는 것이므로 하드웨어와 무관하다.

### 계층 구조

그리퍼 명령은 세 계층을 거친다. 문제가 생겼을 때 어느 계층인지부터 가른다.

```text
[응용]  명령 토픽      /gripper_cmds (rdfp_msgs/GripperCommand — 심볼)
          │                          ← GripperActionNode (robot_control)
          │  targets 파라미터로 심볼 → (관절값, 힘) 변환
          ▼
[액션]  /panda_hand_controller/gripper_cmd    (control_msgs/GripperCommand)
          │                          ← panda_hand_controller
          │  position_controllers/GripperActionController
          ▼
[하드웨어] panda_finger_joint1 position 명령
          │  panda_finger_joint2 는 URDF mimic 으로 자동 추종
          ▼
        mock_components/GenericSystem  또는 실물 드라이버
```

MoveIt 도 같은 액션 서버를 쓴다 — [클라이언트가 둘이다](#클라이언트가-둘이다) 참고.

---

### 누가 제공하나

`panda_hand_controller` 컨트롤러가 띄운다. 타입은
`moveit_resources_panda_moveit_config/config/ros2_controllers.yaml` 에 선언된다.

```yaml
controller_manager:
  ros__parameters:
    panda_hand_controller:
      type: position_controllers/GripperActionController

panda_hand_controller:
  ros__parameters:
    joint: panda_finger_joint1      # ← 단일 관절만 지정한다
```

액션 이름은 컨트롤러가 `~/gripper_cmd` 로 만들기 때문에 컨트롤러 이름이 붙어 `/panda_hand_controller/gripper_cmd` 가 된다. arm 쪽 `panda_arm_controller` 가 `follow_joint_trajectory` 를 내는 것과 같은 구조다.

기동은 [controller_startup_launch_helper.py](../../src/robot_control/robot_control/launch_helpers/controller_startup.py) 의 순차 체인 마지막 단계(`panda_hand_controller` spawner)에서 이뤄진다.

> **JGPC 스택에서도 그대로 동작한다.** `panda_mock_jgpc.launch.py` 는 arm
> 컨트롤러 타입만 바꾸고 `panda_hand_controller` 는 건드리지 않으므로, arm 과
> 달리 그리퍼는 `FollowJointTrajectory` 문제의 영향을 받지 않는다.
> ([move_group_jgpc_client_user_guide.md](move_group_jgpc_client_user_guide.md) 참고)

---

### 인터페이스

`control_msgs/action/GripperCommand` — 필드가 매우 단순하다.

```
# Goal
float64 position      # 목표 관절 위치 [m]
float64 max_effort    # 최대 힘 [N]. 0 이면 컨트롤러 파라미터 기본값 사용
---
# Result / Feedback (구조 동일)
float64 position      # 현재 위치
float64 effort        # 현재 가하는 힘
bool    stalled       # max_effort 를 내면서도 움직이지 못하는 중
bool    reached_goal  # 목표 도달
```

`GripperActionNode` 는 `targets.<심볼>` 파라미터가 주는 `[position, max_effort]` 를 그대로
싣는다 (기본 `open = [0.035, 10.0]`, `close = [0.0, 10.0]`, `grasp = [0.0, 30.0]`).

⚠️ **여기의 `position` 은 관절값이지 개구 폭이 아니다.** 관측 채널
(`GripperState.width`)은 폭이라 Panda 에서 **2배** 차이가 난다 — 두 단위를 직접
비교하면 `at_goal` 이 어긋난다.

---

### 클라이언트가 둘이다

```text
$ ros2 action info /panda_hand_controller/gripper_cmd
Action: /panda_hand_controller/gripper_cmd
Action clients: 2
    /moveit_simple_controller_manager     ← MoveIt
    /gripper                              ← GripperActionNode (robot_control)
Action servers: 1
    /panda_hand_controller
```

MoveIt 은 `gripper_moveit_controllers.yaml` 에서 같은 서버를 등록해 쓴다.

```yaml
moveit_simple_controller_manager:
  panda_hand_controller:
    action_ns: gripper_cmd
    type: GripperCommand
    joints: [panda_finger_joint1]
```

즉 RViz 의 MotionPlanning 패널에서 `hand` 그룹을 `open`/`close` 로 움직여도, `/gripper_cmds` 에 심볼을 발행해도 **결국 같은 액션 서버로 들어간다**. 두 경로에서 동시에 goal 을 보내면 나중 goal 이 앞의 것을 선점(preempt)한다.

---

### `position` 은 gap 이 아니다

액션 정의의 주석은 `position` 을 "The current gripper gap size" 라고 하지만, **Panda 에서는 gap 이 아니라 `panda_finger_joint1` 한 관절의 위치**다.

`panda_finger_joint2` 는 URDF 에서 mimic 으로 묶여 있다.

```xml
<joint name="panda_finger_joint1" type="prismatic">
  <limit effort="20" lower="0.0" upper="0.04" velocity="0.2"/>
<joint name="panda_finger_joint2" type="prismatic">
  <limit effort="20" lower="0.0" upper="0.04" velocity="0.2"/>
  <mimic joint="panda_finger_joint1"/>       <!-- multiplier 1 -->
```

실측:

```text
position=0.04 요청 → panda_finger_joint1=0.04, panda_finger_joint2=0.04
```

두 손가락이 각각 0.04 m 이므로 **실제 벌어진 폭은 0.08 m** 다. 파지 대상의 폭으로 명령값을 계산할 때 2 로 나눠야 한다.

| 값 | 의미 |
|---|---|
| `0.0` | 완전히 닫힘 (폭 0 m) |
| `0.02` | 반개방 (폭 0.04 m) |
| `0.04` | 완전히 열림 (폭 0.08 m) — 관절 상한 |

---

### Result 와 Feedback 의 관계

**필드 구조가 완전히 동일하다.** 액션 정의에서 같은 4줄이 두 번 반복된다.

```text
Feedback: {position: double, effort: double, stalled: bool, reached_goal: bool}
Result  : {position: double, effort: double, stalled: bool, reached_goal: bool}
→ 구조 동일? True
```

의도는 "실행 중 스냅샷"(Feedback)과 "종료 시 스냅샷"(Result)을 같은 형식으로 보되 시점만 달리하는 것이다. 다만 **전달 방식과 정보량이 다르다.**

| | Feedback | Result |
|---|---|---|
| 전송 수단 | 토픽 (`.../_action/feedback`) | 서비스 (`.../_action/get_result`) |
| 래퍼 타입 | `GripperCommand_FeedbackMessage` | `GetResult.Response` |
| 래퍼 필드 | `goal_id` + `feedback` | **`status`** + `result` |
| 횟수 | 0~N 회 | 정확히 1 회 |
| 이 컨트롤러에서 | **0 회 (미발행)** | 정상 도착 |

결정적 차이는 **Result 에만 `status` 가 붙는다**는 점이다.

```text
status: int8   4=SUCCEEDED  5=CANCELED  6=ABORTED
```

`reached_goal` 필드만으로는 성공/실패를 가릴 수 없다. `allow_stalling=False` 상태에서 물체에 막히면 `reached_goal=False, stalled=True` 로 **abort** 되는데, 이건 `status` 를 봐야 구분된다. Feedback 에는 그 정보가 없으므로 **Feedback 만으로는 결과 판정을 할 수 없다** — 설령 발행되더라도 마찬가지다.

컨트롤러가 두 값을 채우는 지점은 `check_for_success()` 한 곳뿐이다.

```cpp
if (fabs(error_position) < params_.goal_tolerance) {
  pre_alloc_result_->reached_goal = true;
  pre_alloc_result_->stalled = false;
  active_goal->setSucceeded(pre_alloc_result_);        // status = SUCCEEDED
} else if (/* stall_timeout 초과 */) {
  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = true;
  if (params_.allow_stalling)  active_goal->setSucceeded(pre_alloc_result_);
  else                         active_goal->setAborted(pre_alloc_result_);   // ABORTED
}
```

따라서 `reached_goal` 과 `stalled` 는 **항상 배타적**이다. 둘 다 `true` 인 Result 는 나오지 않는다.

> **⚠️ `status` 는 더 이상 토픽으로 나가지 않는다 (2026-09-02).** 예전에는
> `GripperControlNode` 가 Result 를 `~/gripper_action_states`
> (`rdfp_msgs/GripperActionState`) 로 재발행하며 `status` 를 그대로 실었다. 그 메시지와
> 토픽은 삭제됐고, 지금 `GripperActionNode` 는 **status 를 로그로만 남긴다.**
> `/gripper_states` 가 싣는 것은 `at_goal` 이며, `CANCELED`(선점)와 `ABORTED`(실패)는
> 둘 다 `at_goal=false` 로 뭉개진다 — 구분이 필요해지면 되살릴 방법을 정해야 한다
> ([gripper_node_design.md](gripper_node_design.md) §8 미결).

---

### 컨트롤러 파라미터

`ros2 param get /panda_hand_controller <name>` 으로 확인한 현재 값이다.

| 파라미터 | 현재 값 | 의미 |
|---|---|---|
| `joint` | `panda_finger_joint1` | 제어 대상. mimic 관절은 자동 추종 |
| `goal_tolerance` | `0.01` | 도달 판정 허용 오차 [m] |
| `max_effort` | `0.0` | goal 의 `max_effort` 가 0 일 때 쓰는 기본값 |
| `allow_stalling` | `False` | stall 을 성공으로 인정하지 않음 |
| `stall_velocity_threshold` | `0.001` | 이 속도 미만이면 정지로 간주 |
| `stall_timeout` | `1.0` | 정지 상태가 이만큼 지속되면 stall 판정 [s] |
| `action_monitor_rate` | `20.0` | goal 상태 전이(succeed/abort/cancel) 확인 주기 [Hz]. **feedback 과 무관** |

`goal_tolerance` 가 `0.01` 인 점에 주의한다. open(0.04)/close(0.0) 은 그 차이가 0.04 라 문제없지만, 0.005 단위로 미세 제어하려 하면 허용 오차 안에서 "도달" 판정이 나 버린다.

물체를 잡을 때 `allow_stalling` 이 `False` 라는 것도 중요하다. 물체에 막혀 멈추면 goal 이 성공이 아니라 **abort** 된다. 파지 동작을 stall 기반으로 성공 판정하려면 이 파라미터를 `True` 로 바꿔야 한다.

---

### 주의점 — 실측 기반

#### 1. 관절 한계를 검사하지 않는다

상한 0.04 를 넘는 값을 보냈는데 그대로 통과했다.

```text
[한계 밖] position=0.10 요청  (upper limit 0.04)
  status=4 (SUCCEEDED)   reached_goal=True
  joint_states: {'panda_finger_joint1': 0.1, 'panda_finger_joint2': 0.1}
```

`GripperActionController` 도, `mock_components/GenericSystem` 도 클램프하지 않는다. 실물 하드웨어에서는 드라이버나 펌웨어가 막겠지만, **mock 에서는 물리적으로 불가능한 자세가 조용히 만들어진다**. RViz 표시도 그에 맞춰 벌어져서 얼핏 정상으로 보인다.

→ 값 검증은 **호출하는 쪽 책임**이다. 임의 위치를 보낼 때는 `[0.0, 0.04]` 로 직접 clamp 한다.

#### 2. feedback 은 발행되지 않는다

`GripperActionController` 는 **액션 feedback 을 전혀 발행하지 않는다.** mock 이든 실물이든, 동작이 빠르든 느리든 마찬가지인 **구조적 특성**이다.

근거는 두 단계다. 먼저 feedback 발행 경로는 `RealtimeGoalHandle::runNonRealtime()` 안에 있고, `setFeedback()` 으로 채워진 경우에만 발행한다.

```cpp
// realtime_tools/realtime_server_goal_handle.hpp
if (req_feedback_ && gh_->is_executing()) {
  gh_->publish_feedback(req_feedback_);
}
```

그런데 `gripper_action_controller` 헤더 전체에 `setFeedback()` 호출이 한 건도 없다 (템플릿 클래스라 구현이 전부 헤더에 있다).

```text
$ grep -rn "setFeedback" /opt/ros/humble/include/gripper_action_controller/
  → 한 건도 없음
```

`action_monitor_rate`(20 Hz) 타이머는 goal 상태 전이(succeed/abort/cancel)를 처리할 뿐 feedback 과 무관하다.

실측:

```text
[open]  position=0.04  status=4(SUCCEEDED)  feedback수=0
[close] position=0.0   status=4(SUCCEEDED)  feedback수=0

/gripper_cmds  수신: ['open', 'close']   ← 의도 토픽은 정상
```

> 측정 당시 토픽 이름은 `/gripper_control/gripper_cmds` 였다. 액션 계층의 관찰은
> 그대로 유효하다.

**파급 효과**

| 대상 | 상태 |
|---|---|
| `/panda_hand_controller/gripper_cmd/_action/feedback` | 항상 비어 있음 |
| `/gripper_states` | **액션과 무관하게 주기 발행** (기본 10 Hz) |

**이것이 `GripperActionNode` 가 액션 결과로 상태를 만들지 않는 이유다.** 결과는 명령당
1 건이라 "지금 어떤 상태인가"에 답할 수 없고, `at_goal` 은 매 주기 재평가여야 한다
(물체를 놓쳐 손이 벌어지면 다시 `false` 가 되어야 한다). 그래서 노드는 액션 결과를
**로그로만** 남기고, 상태는 `/joint_states` 의 손가락 관절에서 만든다.

→ 진행 중 폭이 필요하면 `/gripper_states` 의 `width` 를 본다. **`/joint_states` 를
직접 읽지 않는다** — 백엔드마다 믿을 수 없고(펑션베이는 고정값을 주입한다), 관절값은
개구 폭의 절반이다.

→ feedback 이 꼭 필요하면 gripper 컨트롤러를 feedback 지원 구현으로 교체한다. 지금
구현은 feedback 을 구독조차 하지 않으므로 그 자체로는 아무 영향이 없다.

#### 3. 연속 명령은 서로를 선점(preempt)한다

앞 goal 이 끝나기 전에 새 goal 이 오면 컨트롤러가 앞 goal 을 취소한다. 취소된 goal 의 Result 는 `status=CANCELED`, `reached_goal=False` 로 온다.

open/close 를 완료 대기 없이 10 회 연속 호출한 실측:

```text
status 분포:  CANCELED 9 건 / SUCCEEDED 3 건   (총 12 건)
[gripper] open  result: status=CANCELED  position=0.0000  reached_goal=False
[gripper] close result: status=SUCCEEDED position=0.0000  reached_goal=True
```

`reached_goal=False` 만 보고 "실패" 로 판정하면 안 된다. 선점은 정상적인 취소다.
**명령을 겹쳐 보내지 않는 것이 원칙**이며, 지금은 그 원칙에 기대는 수밖에 없다 —
`status` 를 싣던 토픽이 사라져 `at_goal=false` 만으로는 선점과 실패를 가릴 수 없다
(위 경고 상자 참조).

MoveIt 과 `GripperActionNode` 가 동시에 goal 을 보내는 경우도 마찬가지다 ([클라이언트가 둘이다](#클라이언트가-둘이다) 참고).

#### 4. 명령 발행은 완료를 뜻하지 않는다

[gripper_action_node.py](../../src/robot_control/robot_control/gripper/gripper_action_node.py) 의 `_on_cmd` 는 goal 전송 직후 반환하고, Result 는 별도 콜백 체인에서 받는다.

```python
send_future = self._action.send_goal_async(action_goal)
send_future.add_done_callback(lambda fut: self._on_goal_response(fut, goal))

self._goal = goal      # ← 상태에 목표만 실린다; 결과를 기다리지 않는다
```

즉 `/gripper_cmds` 에 발행이 끝났다는 것은 **"의도를 보냈다"** 는 뜻이지 **"그리퍼가 다
움직였다"** 는 뜻이 아니다. goal 이 거절되거나 abort 되어도 발행 쪽은 이미 돌아온 뒤다.

→ 완료를 확인하려면 `/gripper_states` 의 `at_goal` 을 본다. 판정식은 `goal` 마다 다르며
노드가 이미 적용했으므로 **이 값 하나만 보면 된다**
([gripper_node_design.md](gripper_node_design.md) §2.3).
→ ⚠️ mock 은 `stalled` 관측 수단이 없어 **`grasp` 의 `at_goal` 이 영영 서지 않는다.**

> **로거 주의** — Result 처리에서 status 에 따라 로그 severity 를 바꿀 때, 삼항 연산자로 `self.get_logger().info` / `.warning` 을 골라 **같은 줄에서** 호출하면 안 된다. rclpy 로거는 호출 지점 단위로 severity 를 캐시하며 `ValueError: Logger severity cannot be changed between calls.` 를 던져 **노드가 죽는다**. `if/else` 로 호출 지점을 분리해야 한다.

#### 5. goal 거절은 응답에 반영되지 않는다

위와 같은 이유로, `_dispatch` 가 확인하는 것은 `server_is_ready()` 뿐이다.
서버가 떠 있기만 하면 `success=True` 다. 컨트롤러가 `inactive` 상태라면 액션
서버 자체가 없으므로 이때는 `success=False` 와 함께
`"gripper action server ... not ready"` 메시지가 온다.

---

### 진단 명령 모음

```bash
# 액션 서버가 떠 있나 / 누가 붙어 있나
ros2 action list -t | grep gripper_cmd
ros2 action info /panda_hand_controller/gripper_cmd

# 컨트롤러가 active 인가
ros2 control list_controllers | grep panda_hand_controller

# 파라미터 확인
ros2 param get /panda_hand_controller joint
ros2 param get /panda_hand_controller goal_tolerance
ros2 param get /panda_hand_controller allow_stalling

# 액션 직접 호출 (GripperNode 우회 — 임의 위치 지정 가능)
ros2 action send_goal -f /panda_hand_controller/gripper_cmd \
    control_msgs/action/GripperCommand "{command: {position: 0.02, max_effort: 0.0}}"

# 그리퍼 상태 (개구 폭 + 판정) — 이것이 정규 채널이다
ros2 topic echo /gripper_states --once

# 액션 계층을 의심할 때만 원천을 본다 (관절값이며 폭의 절반이다)
ros2 topic echo /joint_states --once | grep -A2 panda_finger

# feedback 이 실제로 오는지 (mock 에서는 조용한 것이 정상)
ros2 topic echo /panda_hand_controller/gripper_cmd/_action/feedback
```

`ros2 action send_goal` 의 `-f` 는 feedback 출력 옵션이다. mock 에서는 아무것도 찍히지 않지만 Result 는 정상 출력된다.

---

## 10. 관련 문서

- [robotiq_2f_gripper_node_guide.md](robotiq_2f_gripper_node_guide.md) — 관절 지령 기반 자매 구현 (펑션베이)
- [gripper_node_design.md](gripper_node_design.md) — **계약 정본.** 메시지 필드의 뜻,
- **§9 액션 서버 계층** — 그 아래 `/panda_hand_controller/gripper_cmd` 액션 서버의 동작 노트 (옛 `gripper_action_server_notes.md`)
- [../topic_naming_contract.md](../topic_naming_contract.md) §2.2 — 채널 이름이 `~/`
- [../../src/robot_control/launch/README.md](../../src/robot_control/launch/README.md)
- [gripper_node_design.md](gripper_node_design.md) — 이 액션을 감싼 `GripperNode` 인터페이스 설계 (메시지 계약, `at_goal` 판정식, 백엔드별 실현 가능성)
- [move_group_jgpc_client_user_guide.md](move_group_jgpc_client_user_guide.md) — JGPC 스택에서 arm 은 왜 다른가 (그리퍼는 영향 없음)
- [../../src/robot_control/launch/README.md](../../src/robot_control/launch/README.md) — 컨트롤러 순차 기동 순서
