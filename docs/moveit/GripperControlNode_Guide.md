# GripperControlNode Programmer's Guide

`rdfp_msgs/GripperCommand` 토픽을 `control_msgs/GripperCommand` **액션으로 중계**하는 노드다. 그리퍼 액션을 부르는 주체를 하나로 모으고, 그 앞에 **기록 가능한 명령 토픽**을 두는 것이 이 노드의 존재 이유다.

---

## 목차

1. [개요](#1-개요)
2. [실행](#2-실행)
3. [인터페이스](#3-인터페이스)
4. [기본 사용법](#4-기본-사용법)
5. [에러 처리](#5-에러-처리)
6. [Best Practices](#6-best-practices)
7. [트러블슈팅](#7-트러블슈팅)
8. [관련 문서](#8-관련-문서)

---

## 1. 개요

```
[호출자]                          [이 노드]                    [컨트롤러]
키보드 teleop  ┐
robot twin     ├─▶ /gripper_control/gripper_cmds ─▶ gripper_control ─▶ gripper_cmd 액션
데이터셋 재생  ┘   (rdfp_msgs/GripperCommand)              │
                                                            ▼
                                        /gripper_control/gripper_action_states
                                              (rdfp_msgs/GripperActionState)
```

### 1.1 왜 토픽을 한 번 거치는가

액션을 직접 부르면 될 것 같지만, 두 가지 이유로 앞에 토픽을 둔다.

| 이유 | 내용 |
|---|---|
| **기록 가능성** | 액션 goal 전송은 **서비스**라 rosbag2(Humble)가 기록하지 못한다. 명령을 토픽으로 흘려야 학습 데이터의 **action 채널**이 남는다 |
| **경로 일관성** | 액션 호출자가 여럿이면 결과(`gripper_action_states`)를 누가 발행할지 갈리고, 에피소드 생성기에 따라 결과 채널이 있다가 없어진다 |

즉 이 노드는 편의 계층이 아니라 **데이터 계층의 일부**다.

### 1.2 명령은 숫자다 — 심볼이 아니다

명령에는 `position`(m)과 `max_effort`(N)를 싣는다. `"open"` 같은 심볼을 보내지 않는 이유는, 심볼을 쓰면 **그 의미(몇 m 인가)가 노드 상수에 남아 데이터셋이 자기 완결적이지 않게** 되기 때문이다. 상수를 바꾸는 순간 과거 에피소드의 의미가 조용히 바뀐다.

사람이 읽을 이름이 필요하면 `label` 필드를 쓴다 — **제어에는 사용되지 않는다.**

> **이전 버전과 호환되지 않는다 ⚠️**
> `~/open_gripper` / `~/close_gripper` (`std_srvs/Trigger`) 서비스는 **제거되었고**, `rdfp_msgs/GripperCommand` 의 `string command` 필드도 숫자 필드로 대체되었다. 기존 DB 데이터는 마이그레이션하지 않는다 (5장).

---

## 2. 실행

```bash
# 기본 노드명 'gripper_control'
ros2 run rdfp gripper_control_node

# launch 로는 panda_mock / rdfp_panda_mock / replay_panda_mock 계열에 포함되어 있다
ros2 launch rdfp rdfp_panda_mock.launch.py
```

기동 로그:

```
GripperControlNode started (subscribe: ~/gripper_cmds, action: /panda_hand_controller/gripper_cmd)
```

**액션 서버가 없어도 노드는 뜬다.** 명령이 들어온 시점에 `server_is_ready()` 로 확인해 경고만 남기고 넘어간다 — 컨트롤러보다 먼저 떠도 죽지 않게 하기 위해서다.

---

## 3. 인터페이스

| 종류 | 이름 | 타입 | 방향 |
|---|---|---|:-:|
| 토픽 | `~/gripper_cmds` | `rdfp_msgs/GripperCommand` | 구독 |
| 토픽 | `~/gripper_action_states` | `rdfp_msgs/GripperActionState` | 발행 |
| 액션 | `/panda_hand_controller/gripper_cmd` | `control_msgs/GripperCommand` | 호출 |

기본 노드명이 `gripper_control` 이므로 실제 토픽은 `/gripper_control/gripper_cmds` · `/gripper_control/gripper_action_states` 로 resolve 된다.

### 3.1 명령 — `rdfp_msgs/GripperCommand`

```
std_msgs/Header header
float64 position      # 목표 위치 (m)
float64 max_effort    # 최대 파지력 (N). 0 = 드라이버 기본값
string  label         # 선택. 'open'/'close'/'grasp' 등. 제어에 쓰이지 않는다
```

**`position` 은 손가락 사이 거리가 아니다 ⚠️** 컨트롤러가 물린 관절 하나 (`panda_finger_joint1`)의 목표값이며 Panda 는 `0.0 ~ 0.04` 다. 실기 franka 의 `move`/`grasp` 가 쓰는 `width` 와 **2배 차이**가 난다.

Panda 관례값:

| 용도 | `position` | `max_effort` |
|---|---|---|
| 열기 | `0.04` | `0.0` |
| 빈손으로 닫기 | `0.0` | `0.0` |
| **물체 파지** | `0.0` | `> 0` (예: `30.0`) |

**파지 시 폭을 물체 두께로 맞출 필요가 없다. 끝까지 닫으라고 명령하면 물체에 닿는 순간 `max_effort` 로 버티며 멈춘다.**

#### 왜 `control_msgs/GripperCommand` 를 그대로 쓰지 않는가

값이 `position` / `max_effort` 로 같으니 표준 타입을 쓰면 될 것 같지만, **헤더가 없다.**

```
control_msgs/GripperCommand        rdfp_msgs/GripperCommand
                                   std_msgs/Header header      ← 이것이 차이
float64 position                   float64 position
float64 max_effort                 float64 max_effort
                                   string  label
```

`control_msgs` 쪽은 **액션 goal 안에 들어가는 값 묶음**이라 시각을 담을 이유가 없다. 반면 이 토픽은 **기록되는 채널**이므로 시각이 반드시 필요하다.

**stamp 가 없으면 이 명령은 어느 에피소드에도 속하지 못한다 ⚠️** 적재 코드가 헤더 시각을 그대로 쓰기 때문이다.

```python
def extract_stamp(msg):
    stamp = msg.header.stamp
    return int(stamp.sec), int(stamp.nanosec)
```

저장기는 `/session` 전이 시각으로 구간을 자르고 그 안에 드는 메시지를 모은다. 시각이 없으면 비교 자체가 성립하지 않아 **epoch 0 에 적재된다.** 실제로 트윈이 stamp 를 채우지 않아 이 증상이 났던 적이 있다 — 명령은 정상 동작하고 로봇도 움직이므로 **데이터를 열어보기 전까지 드러나지 않는다.**

rosbag2 가 수신 시각을 따로 기록하긴 하지만 이 파이프라인은 **메시지 자신의 시각**을 쓴다. 그래야 재생 때 원래 간격이 복원되고 여러 토픽을 같은 시간축에 맞출 수 있다.

#### `label` 은 왜 두는가

제어에 쓰이지 않으므로 **없어도 시스템은 동작한다.** 데이터셋 쪽 편의를 위한 필드다.

| | `label` 있음 | 없음 |
|---|---|---|
| 제어 | 영향 없음 | 영향 없음 |
| 데이터셋 조회 | `WHERE label='grasp'` 로 파지 구간을 바로 뽑는다 | `position`/`max_effort` 조합으로 역추론 |
| 목표 이름 추적 | 트윈의 `target` 이 그대로 남는다 | 어떤 이름으로 불렸는지 사라진다 |

`close` 와 `grasp` 가 그 예다. 둘은 `position` 이 같고 `max_effort` 만 다른데, `label` 이 있으면 의도가 그대로 보이고 없으면 `max_effort > 0` 여부로 추정해야 한다. **목표가 늘어날수록 이 역추론은 더 불안해진다.**

다만 `label` 로 분기하는 제어 코드를 만들면 안 된다 — 그 순간 심볼 시절의 문제(의미가 데이터셋 밖에 있음)로 되돌아간다.

### 3.2 결과 — `rdfp_msgs/GripperActionState`

액션 result 를 그대로 옮긴 것에 goal 상태(`status`)를 더한 메시지다.

```jsonc
{ "position": 0.0182, "effort": 30.0,
  "stalled": true, "reached_goal": false, "status": 4 }   // 4 = SUCCEEDED
```

| 조합 | 해석 |
|---|---|
| `reached_goal: true` | 목표 폭 도달 (빈손) |
| `stalled: true` | 힘을 내는데 안 움직인다 = **물체를 물었다** |
| 둘 다 `false` | 중단되었거나 목표에 못 미쳤다 |

**`reached_goal: false` 를 실패로 읽지 않는다.** 물체를 쥐면 목표까지 갈 수 없다. 파지 판정의 축은 `stalled` 이며, 성공/취소/abort 구분은 `status` 로 한다.

> **이벤트성이다.** `GripperActionController` 는 feedback 을 발행하지 않으므로 이 토픽은 **명령당 1건(result 기반)** 이다. 연속 상태가 아니므로 "지금 얼마나 벌어져 있는가"는 `/joint_states` 의 `panda_finger_joint1` 에서 읽는다.

---

## 4. 기본 사용법

### 4.1 CLI

```bash
# 열기
ros2 topic pub --once /gripper_control/gripper_cmds rdfp_msgs/msg/GripperCommand \
  '{position: 0.04, max_effort: 0.0, label: "open"}'

# 파지 (힘을 준다)
ros2 topic pub --once /gripper_control/gripper_cmds rdfp_msgs/msg/GripperCommand \
  '{position: 0.0, max_effort: 30.0, label: "grasp"}'

# 결과 확인
ros2 topic echo /gripper_control/gripper_action_states --once
```

### 4.2 Python — 명령 발행

```python
from rdfp_msgs.msg import GripperCommand

pub = node.create_publisher(GripperCommand, '/gripper_control/gripper_cmds', 10)

msg = GripperCommand()
msg.header.stamp = node.get_clock().now().to_msg()
msg.position = 0.0
msg.max_effort = 30.0
msg.label = 'grasp'        # 기록용. 제어에는 영향이 없다
pub.publish(msg)
```

> **퍼블리셔를 만들자마자 publish 하면 첫 명령이 사라진다 ⚠️**
> DDS 매칭 전에는 구독자가 없어 메시지가 버려진다. 퍼블리셔는 **기동 시점에 만들어 두고** 재사용한다 (robot twin 의 `_start_command_publishers` 가 그 예다).

### 4.3 Python — 결과 확인

```python
from rdfp_msgs.msg import GripperActionState

def on_state(msg: GripperActionState) -> None:
    if msg.stalled:
        print(f'파지 성공 — 폭 {msg.position:.4f} m, 힘 {msg.effort:.1f} N')
    elif msg.reached_goal:
        print('목표 폭 도달 (빈손)')

node.create_subscription(GripperActionState, '/gripper_control/gripper_action_states', on_state, 10)
```

**구독은 명령을 보내기 전에 만들어 둔다.** 명령 직후에 만들면 결과가 지나가 버린다.

### 4.4 연속 폭 추적

결과 토픽은 이벤트성이라 동작 중 궤적을 볼 수 없다. 연속값은 `/joint_states` 를 쓴다.

```bash
ros2 topic echo /joint_states --field position   # panda_finger_joint1 이 half-width
```

---

## 5. 에러 처리

| 증상 | 원인 | 대응 |
|---|---|---|
| `action server '...' not ready` 경고 후 무시 | 컨트롤러 미기동 | `ros2 control list_controllers` 로 `panda_hand_controller` 확인 |
| 명령을 보냈는데 아무 로그도 없다 | 토픽 이름 불일치 | `ros2 topic info /gripper_control/gripper_cmds` 로 구독자 확인 |
| 첫 명령만 무시된다 | 퍼블리셔 DDS 매칭 전 발행 | 퍼블리셔를 미리 만들어 두거나 잠시 뒤 재시도 (4.2) |
| `goal rejected by action server` | 컨트롤러가 goal 거부 (관절 범위 밖 등) | `position` 이 `0.0 ~ 0.04` 인지 확인 |
| 결과가 안 온다 | 액션은 실행 중이거나 preempt 됨 | `status` 확인. 선점(CANCELED)은 실패가 아니다 |

**노드는 명령값을 검증하지 않는다.** 관절 한계 판정은 컨트롤러의 몫이고, 상위 계층 (트윈 등)이 이미 검증한 값을 다시 막으면 실패 지점만 흐려지기 때문이다.

### 5.1 과거 데이터 (마이그레이션 없음)

`gripper_cmds` 테이블이 `command TEXT` → `position` / `max_effort` / `label` 로 바뀌었다. **과거 데이터는 마이그레이션하지 않는다.** 스키마를 다시 만든다.

```bash
ros2 run rdfp init-db --drop --yes    # 기존 데이터가 모두 삭제된다
```

---

## 6. Best Practices

| 규칙 | 이유 |
|---|---|
| **퍼블리셔·구독은 기동 시 만든다** | 호출 시점에 만들면 첫 메시지를 잃는다 |
| **파지는 `max_effort > 0` 으로** | `0` 은 드라이버 기본값이며, 실기 franka 에서는 파지 없는 이동으로 해석될 수 있다 |
| **`label` 에 의미를 담지 않는다** | 제어에 쓰이지 않는다. 이 값으로 분기하는 코드를 만들면 심볼 시절로 되돌아간다 |
| **고빈도로 쏘지 않는다** | 새 goal 이 이전 goal 을 선점한다. 10~20 Hz 이상은 컨트롤러가 선점 처리에 밀린다 |
| **완료 판정은 `stalled` / `status`** | `reached_goal` 단독으로는 파지를 실패로 오판한다 |

---

## 7. 트러블슈팅

```bash
# 1) 노드가 떠 있고 구독 중인가
ros2 node info /gripper_control

# 2) 명령이 실제로 나가는가
ros2 topic hz /gripper_control/gripper_cmds

# 3) 액션 서버가 있는가
ros2 action list | grep gripper_cmd

# 4) 결과가 돌아오는가
ros2 topic echo /gripper_control/gripper_action_states
```

체인 어디가 끊겼는지는 이 순서대로 좁힌다 — 1~2 는 호출자 문제, 3~4 는 컨트롤러 문제다.

---

## 8. 관련 문서

- [gripper_action_server_notes.md](gripper_action_server_notes.md) — 액션 서버 계층, mock 에서 feedback 이 오지 않는 이유
- [../rdfp_framework_design.md](../rdfp_framework_design.md) — 이 노드가 데이터 계층에서 갖는 역할 (2.2 action/observation)
- [../robot_twin/robot_twin_user_guide.md](../robot_twin/robot_twin_user_guide.md) — HTTP 로 그리퍼를 조작하는 `move_gripper_to_target` 연산
- [../teleop/leader_gripper_mapping_design.md](../teleop/leader_gripper_mapping_design.md) — 리더 트리거를 이 명령으로 바꾸는 설계
