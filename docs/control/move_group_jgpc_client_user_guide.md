# `MoveGroupJgpcClient` 사용 가이드 (JGPC / 스트리밍 실행)

`FollowJointTrajectory` 액션이 없는 스택에서 **계획은 MoveIt 에 맡기고 실행은 명령 토픽 스트리밍으로** 하는 방법. named target · joint 목표 · Cartesian 경로 이동, 중단, `TrajectoryStreamer` 직접 사용, `publish_rate` 튜닝까지 이 구현 전반을 다룬다. 백엔드 `mock_jgpc` 와 `functionbay` 가 여기에 해당한다.

공통 인터페이스와 JTC 구현은 [move_group_client_user_guide.md](move_group_client_user_guide.md)가 갖는다 — 두 문서는 그렇게 역할을 나눈다.
**JGPC 전용 내용은 이 문서에만 있다** (2026-09-21 경계 정리 — 공통 가이드의 옛 §4.10 · §12 를 여기로 몰았다). 공통 가이드에서 JGPC 는 §1.2 대조표와 포인터로만 나타난다.

---

## 목차

1. [왜 필요한가](#1-왜-필요한가)
2. [동작 원리](#2-동작-원리)
3. [Quick Start](#3-quick-start)
4. [API 레퍼런스](#4-api-레퍼런스)
5. [비동기 사용법](#5-비동기-사용법)
6. [TrajectoryStreamer — 직접 사용과 API](#6-trajectorystreamer--직접-사용과-api)
7. [replay_gui 연동](#7-replay_gui-연동)
8. [파라미터 튜닝](#8-파라미터-튜닝)
9. [에러 처리](#9-에러-처리)
10. [주의사항](#10-주의사항)
11. [관련 문서](#11-관련-문서)

---

## 1. 왜 필요한가

`panda_mock_jgpc.launch.py` 는 arm 컨트롤러를 `joint_trajectory_controller/JointTrajectoryController` (JTC) 대신 `position_controllers/JointGroupPositionController` (JGPC) 로 띄운다. JGPC 는 `forward_command_controller` 기반이라 **`FollowJointTrajectory` 액션을 제공하지 않는다**.

그런데 MoveIt 의 `moveit_simple_controller_manager` 는 `FollowJointTrajectory` 와 `GripperCommand` 두 타입만 다룰 수 있다. 그래서 기존 API 는 이렇게 갈린다.

| API 부류 | JTC 환경 | JGPC 환경 | 이유 |
|---|:-:|:-:|---|
| SRDF 조회 (`get_*`) | ✅ | ✅ | `move_group` 파라미터 조회 — 컨트롤러 무관 |
| 계획 전용 (`plan_*`) | ✅ | ✅ | Cartesian 서비스 / `MoveGroup` **plan_only** — 실행 안 함 |
| 계획 + 실행 (`move_to_*`, `follow_trajectory`) | ✅ | ✅ | **구현이 다르다** — JTC 는 액션, JGPC 는 스트리밍 |
| `execute_trajectory()` | ✅ | — | `MoveGroupJtcClient` 에만 있음 |
| `stream_trajectory()` / `*_streamed()` | — | ✅ | `MoveGroupJgpcClient` 에만 있음 |

메서드 단위 전체 대조표는 [move_group_client_user_guide.md](move_group_client_user_guide.md) §1.2 「구현별 API 가용성」에 있다.

즉 **계획은 원래 잘 된다. 잃은 건 실행 경로 하나뿐**이다.

그래서 클라이언트를 계획(공통) + 실행(구현별)으로 나눴다. `MoveGroupClient` 는 추상 공통 인터페이스이고, 실행은 `MoveGroupJtcClient` / `MoveGroupJgpcClient` 가 각자 구현한다. `create_move_group_client(node, backend=...)` 가 백엔드 프로파일에서 알맞은 쪽을 골라 돌려주므로, 호출부는 명령 채널을 몰라도 된다.

---

## 2. 동작 원리

JTC 는 궤적을 통째로 받아 **컨트롤러 내부에서 시간 보간**을 한다. 그러나 JGPC 는 받은 값을 그대로 hardware 에 전달하기 때문에, 그 시간 보간을 클라이언트가 대신해야 한다.

> **Note — '시간 보간' 이란**
>
> 계획 결과인 `RobotTrajectory` 는 연속 곡선이 아니라 **`time_from_start` 가 붙은 이산
> point 의 목록**이다. MoveIt 은 시간 파라미터화 단계에서 이것을 0.1초 격자로 리샘플하므로
> 대개 **10 Hz** 짜리 목록이 된다. 반면 로봇은 그보다 훨씬 촘촘한 주기로 위치 명령을 받아야
> 매끄럽게 움직인다. 그래서 **두 point 사이의 중간 위치를 시각에 맞춰 만들어 내는 단계**가
> 어딘가에는 반드시 있어야 하고, 그 단계가 시간 보간이다.
>
> **JTC 는 그 단계를 컨트롤러 안에서 한다.** 궤적을 통째로 받아 두고, 자기 update rate 마다
> "지금 시각에 해당하는 위치"를 계산해 hardware 에 쓴다. 그래서 10 Hz 궤적을 줘도 팔은
> 매끄럽게 움직인다 — 클라이언트는 궤적을 넘기고 나면 할 일이 없다.
>
> **JGPC 에는 그 단계가 없다.** 받은 배열을 그대로 hardware 에 write 하므로 **팔이 실제로
> 받는 것은 우리가 발행한 명령 그 자체**다. 10 Hz 로 발행하면 10 Hz 계단이 그대로 팔에
> 들어간다. 그래서 `TrajectoryStreamer` 가 이 일을 대신하며, 밀도는 `publish_rate` 인자를 통해
> 정한다 (§8.3).
>
> ⚠️ **mock 에서는 이 차이가 안 보인다** — `GenericSystem` 이 명령 위치로 즉시 점프하므로
> 계단이 그대로 흡수된다. 실기·시뮬레이터에서만 드러난다.

```text
[JTC 경로]
  MoveGroup 액션 (plan + execute)
    └→ /panda_arm_controller/follow_joint_trajectory  (액션)
         └→ JTC 내부에서 궤적 시간 보간 → hardware

[JGPC 경로 — 본 가이드]
  MoveGroup 액션 (plan_only=True)
    └→ RobotTrajectory 반환
         └→ TrajectoryStreamer 가 time_from_start 에 맞춰 point 를 하나씩 발행
              └→ /panda_arm_controller/commands  (std_msgs/Float64MultiArray)
                   └→ JGPC 가 그대로 hardware 에 write
```

### 2.1 joint 순서 문제

JGPC 는 `forward_command_controller` 계열이라 궤적이 아니라 **값의 배열**을 받아 그대로 hardware 인터페이스에 쓴다. 해석할 대상이 없으므로 메시지도 실수 목록 하나로 충분하며, joint 이름·시각·속도를 함께 싣는 JTC 의 `FollowJointTrajectory` 와 달리 `Float64MultiArray` 를 사용한다.

**`Float64MultiArray` 에는 joint 이름이 없다.** 배열 순서가 컨트롤러의 `joints` 파라미터 순서와 어긋나면 엉뚱한 관절이 움직인다.

`TrajectoryStreamer` 는 이를 방지하기 위해 컨트롤러 노드의 `get_parameters` 서비스에서 `joints` 파라미터를 직접 읽어 순서를 확정하고, 궤적의 `joint_names` 를 그 순서로 재배열한다. 사용자가 신경 쓸 필요는 없지만, 직접 발행할 때는 반드시 같은 재배열을 해야 한다.

---

## 3. Quick Start

### 3.1 스택 실행

```bash
ros2 launch robot_control panda_mock_jgpc.launch.py
```

### 3.2 가장 간단한 사용법 (동기)

```python
import rclpy
from rclpy.node import Node

from robot_control.moveit import create_move_group_client

rclpy.init()
node = Node('named_target_demo')

# 프로파일이 JGPC 로 정해져 있으므로 MoveGroupJgpcClient 가 돌아온다.
with create_move_group_client(node, backend='mock_jgpc') as mgc:
    print(mgc.get_named_targets())            # ['extended', 'ready', 'transport']
    mgc.move_to_named_target('ready')         # JGPC 구현 = 계획 + 스트리밍

node.destroy_node()
rclpy.shutdown()
```

`MoveGroupJgpcClient.move_to_named_target()` 은 계획 + 스트리밍을 한 번에 수행하며 궤적 길이만큼 블로킹된다. 발행한 명령 개수가 필요하면 같은 뜻의 `move_to_named_target_streamed()` 를 직접 호출한다.

### 3.3 구현 선택

**`backend` 이름 하나로 구현이 정해진다.** 어느 스택이 JTC 이고 어느 쪽이 JGPC 인가는
백엔드 프로파일이 갖고 있으므로, 호출부는 컨트롤러 종류도 명령 채널도 알 필요가 없다.

```python
mgc = create_move_group_client(node, backend='mock_jgpc')    # -> MoveGroupJgpcClient
mgc = create_move_group_client(node, backend='functionbay')  # -> MoveGroupJgpcClient
mgc = create_move_group_client(node, backend='isaac')        # -> MoveGroupJtcClient
```

**`backend` 와 `mode` 중 하나는 반드시 지정한다** — 둘 다 생략하면 `ValueError` 가
발생한다. `mode` 는 프로파일이 없는 스택을 붙일 때 구현을 직접 지정하는 대안이며,
프로파일 표와 함께 [move_group_client_user_guide.md](move_group_client_user_guide.md) §1.1 에서 다룬다.

### 3.4 목표 조인트로 이동 (`move_to_joints()`)

`move_to_joints()` 는 SRDF 조회 단계만 없을 뿐 같은 경로를 탄다.

```python
mgc.move_to_joints({'panda_joint1': 0.5, 'panda_joint4': -2.0}, publish_rate=200.0)
```

### 3.5 계획과 실행 분리

궤적을 검사하거나 재사용하고 싶을 때:

```python
trajectory = mgc.plan_named_target('extended')
print(f'{len(trajectory.joint_trajectory.points)} points')

mgc.stream_trajectory(trajectory)                     # 원래 속도
mgc.stream_trajectory(trajectory, time_scaling=2.0)   # 두 배 느리게

# joint 목표값 판 — SRDF 조회가 없다
trajectory = mgc.plan_joints({'panda_joint1': 0.5})
mgc.stream_trajectory(trajectory, publish_rate=200.0)
```

---

## 4. API 레퍼런스

### 4.1 `MoveGroupJgpcClient` 생성자 추가 인자

`create_move_group_client()` 가 그대로 전달한다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `arm_command_topic` | `/panda_arm_controller/commands` | 스트리밍 대상 명령 토픽. `backend` 를 주면 프로파일이 채운다 |
| `arm_command_joint_names` | `None` | 명령 배열 joint 순서. `None` 이면 컨트롤러에서 자동 조회 |

명령 퍼블리셔는 **lazy 생성**된다. 스트리밍을 한 번도 하지 않으면 토픽이 생기지 않는다. JTC 환경에서는 `MoveGroupJtcClient` 가 생성되므로 이 인자들이 아예 쓰이지 않는다.

### 4.2 스트리밍 실행 메서드

`MoveGroupJgpcClient` 에만 있다.

| 메서드 | 블로킹 | 반환 | 역할 |
|---|:-:|---|---|
| `stream_trajectory(traj, ...)` | ✅ | `int` (발행 명령 수) | 궤적을 명령 토픽으로 발행. JTC 의 `execute_trajectory()` 대응 |
| `stop_streaming()` | ❌ | `None` | 진행 중인 스트리밍 중단 |
| `follow_trajectory_streamed(wps, ...)` | ✅ | `int` | 카테시안 계획 + 스트리밍 |
| `move_to_named_target_streamed(name, ...)` | ✅ | `int` | named target 계획 + 스트리밍 |
| `move_to_named_target_streamed_async(name, ...)` | ❌ | `Future[int]` | 위의 비동기판 |
| `move_to_joints_streamed(joint_values, ...)` | ✅ | `int` | joint 목표값 계획 + 스트리밍 |
| `move_to_joints_streamed_async(joint_values, ...)` | ❌ | `Future[int]` | 위의 비동기판 |

`stream_trajectory(trajectory, *, time_scaling=1.0, publish_rate=None, externally_spun=False)` — `externally_spun` 은
joint 순서 자동 조회 시에만 뜻이 있다(§5.1). `stop_streaming()` 은 인자가 없다. JTC 클라이언트에서 이 메서드들을 부르면
`AttributeError` 다 — 그 클래스에 없다.

### 4.3 공통 API 의 JGPC 구현

이름은 base 와 같지만 실행이 스트리밍으로 바뀐 메서드다. **각각 대응하는 `*_streamed` 를 그대로 호출하는 얇은 래퍼**이며, 반환값을 버려 base 시그니처에 맞추는 것이 존재 이유다. 발행한 명령 수가 필요하면 `*_streamed` 쪽을 직접 부른다.

| 메서드 | 블로킹 | 반환 | 위임 대상 |
|---|:-:|---|---|
| `move_to_named_target(name, ...)` | ✅ | `None` | `move_to_named_target_streamed()` |
| `move_to_named_target_async(name, ...)` | ❌ | `Future[int]` | `move_to_named_target_streamed_async()` |
| `move_to_joints(joint_values, ...)` | ✅ | `None` | `move_to_joints_streamed()` |
| `move_to_joints_async(joint_values, ...)` | ❌ | `Future[int]` | `move_to_joints_streamed_async()` |
| `follow_trajectory(wps, ...)` | ✅ | `None` | `follow_trajectory_streamed()` |
| `follow_trajectory_async(wps, ...)` | ❌ | `Future[int]` | 계획 콜백 체인 + 스트리밍 스레드 |

이 여섯 개는 base 시그니처에 없는 `time_scaling` / `publish_rate` 를 추가로 받는다.

### 4.4 중단 — `cancel()`

`cancel()` 은 두 구현 모두에 있지만 정지 특성이 다르다.

| 구현 | 수단 | 정지 특성 |
|---|---|---|
| JTC | 컨트롤러 `FollowJointTrajectory` 액션에 `CancelGoal` | 컨트롤러가 감속 후 정지 |
| **JGPC** | 스트리밍 중단(`stop_streaming()`) + 계획 단계 MoveGroup goal 취소 | 마지막으로 보낸 명령 위치에서 정지 — **감속 프로파일 없음** |

컨트롤러가 마지막 명령 위치를 그대로 유지하므로, JGPC 에서 `cancel()` 은 사실상 급정지다. 반환값 `True` 는 중단 대상이 있었다는 뜻이며, 정지 후 로봇은 경로 중간의 불확정 자세에 있다.

### 4.5 계획 메서드 (컨트롤러 무관, base 제공)

JGPC 환경에서 실행 전에 궤적을 얻을 때 쓴다. 세 메서드 모두 JTC 환경에서도 동일하다.

| 메서드 | 반환 | 비고 |
|---|---|---|
| `plan_named_target(name, ...)` (+ `_async`) | `RobotTrajectory` | `MoveGroup` **plan_only** — 실행 안 함 |
| `plan_joints(joint_values, ...)` (+ `_async`) | `RobotTrajectory` | 위와 같되 SRDF 조회 없음 |
| `plan_trajectory(wps, ...)` (+ `_async`) | `RobotTrajectory` | `/compute_cartesian_path` 서비스 |

### 4.6 스트리밍 전용 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `time_scaling` | `1.0` | 재생 시간 배율. `2.0` = 두 배 느리게 |
| `publish_rate` | `None` | 발행 주파수(Hz). `None` 이면 궤적 point 시각에만 발행(≈10Hz). 값을 주면 그 주기로 위치를 선형 보간 |
| `externally_spun` | `False` | Node 가 다른 스레드에서 이미 spin 중이면 `True` |
| `timeout` | `120.0` | **여기서는 SRDF 조회 + 계획까지만** 제한한다. 스트리밍 시간은 궤적 길이가 결정하므로 포함되지 않는다 |

### 4.7 계획 파라미터

`velocity_scaling`, `planning_time`, `tolerance`, `fraction_threshold`, `max_step`, `jump_threshold` 는 **두 구현이 동일하다** (계획은 컨트롤러와 무관하다). [move_group_client_user_guide.md](move_group_client_user_guide.md) 의 「API 레퍼런스」·「파라미터 튜닝 가이드」를 본다.

---

## 5. 비동기 사용법

Tk GUI 처럼 호출 스레드를 블로킹할 수 없는 환경에서 사용한다.

```python
import threading
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor()
executor.add_node(node)
threading.Thread(target=executor.spin, daemon=True).start()

# executor 가 이미 spin 중이므로 externally_spun=True 가 필수다.
future = mgc.move_to_named_target_async('ready', externally_spun=True)

def _on_done(fut):
    exc = fut.exception()
    if exc is not None:
        node.get_logger().error(f'초기화 실패: {exc}')
    else:
        node.get_logger().info(f'완료: {fut.result()} points 발행')

future.add_done_callback(_on_done)
```

### 5.1 `externally_spun` 을 언제 `True` 로 두는가

명령 발행 자체는 spin 이 필요 없지만, **joint 순서 자동 조회**는 서비스 호출이라 spin 이 필요하다. 이때 누가 spin 할지가 갈린다.

| 상황 | 값 | 이유 |
|---|:-:|---|
| 아무도 spin 하지 않음 (일반 스크립트) | `False` | 라이브러리가 직접 `spin_until_future_complete` |
| `MultiThreadedExecutor` 등이 백그라운드 spin 중 | `True` | 직접 spin 하면 "node already spinning" 오류 |

`arm_command_joint_names` 를 생성자에서 명시하면 조회 자체가 생략되므로 이 값은 무의미해진다.

---

## 6. `TrajectoryStreamer` — 직접 사용과 API

MoveGroup 클라이언트 없이 임의의 궤적을 스트리밍하려면 `TrajectoryStreamer` 를 직접 쓴다. Cartesian 계획 결과(`plan_trajectory()`)도 그대로 넣을 수 있다.

```python
from robot_control.moveit.trajectory_streamer import TrajectoryStreamer

with TrajectoryStreamer(node) as streamer:
    print(streamer.resolve_joint_names())   # 컨트롤러에서 순서 조회
    streamer.stream(trajectory)                        # 궤적 point 시각에만 발행
    streamer.stream(trajectory, publish_rate=200.0)    # 200Hz 로 보간 발행
```

### 6.1 생성자

**`node` 를 제외한 나머지는 키워드 전용이다** (`*`) — 순서로 넘길 수 없다.

```python
TrajectoryStreamer(
    node, *,
    command_topic='/panda_arm_controller/commands',
    joint_names=None,            # None → 컨트롤러에서 자동 조회
    controller_node_name=None,   # None → command_topic 에서 유도
    command_format='float64_multi_array',   # 또는 'joint_state'
    qos_depth=10,
    gravity_compensator=None,
)
```

| 인자 | 설명 |
|---|---|
| `command_topic` | 명령을 발행할 토픽. `controller_node_name` 을 여기서 유도한다 (`/panda_arm_controller/commands` → `/panda_arm_controller`) |
| `joint_names` | 명령 배열의 joint 순서. `None` 이면 첫 스트리밍 때 컨트롤러의 `joints` 파라미터를 조회한다 |
| `controller_node_name` | `joints` 를 조회할 컨트롤러 노드 이름. 이름 규칙이 다른 스택에서만 준다 |
| **`command_format`** | `'float64_multi_array'`(기본, ros2_control JGPC) 또는 `'joint_state'`(토픽 연동형 시뮬레이터). 아래 주의 |
| `qos_depth` | 명령 퍼블리셔의 QoS depth |
| **`gravity_compensator`** | `robot_control.gravity.GravityCompensator` 또는 `None`. 주면 **발행 직전** 모든 명령점에 `τ_g(q)/Kp` 를 더해, 중력 보상이 없는 백엔드가 지령보다 처지는 것을 상쇄한다 |

> ### ⚠️ `command_format` 이 맞지 않으면 오류 없이 연결되지 않는다
>
> 펑션베이처럼 **`sensor_msgs/JointState` 를 받는 시뮬레이터**에 기본값
> (`float64_multi_array`) 으로 보내면 ROS 2 는 타입이 달라 **연결만 하지 않고 오류를
> 내지 않는다** — 관측되는 증상은 "팔이 명령에 반응하지 않는다" 뿐이다. 그 백엔드에서는
> `command_format='joint_state'` 를 준다.
>
> **그때는 `joint_names` 도 함께 줘야 한다.** ros2_control 이 없어 `joints` 를 조회할
> 컨트롤러 노드가 존재하지 않으므로, 자동 조회가
> `TimeoutError: Timed out waiting for /input/get_parameters` 로 실패한다.
>
> `create_move_group_client(node, backend=...)` 를 쓰면 이 둘이 프로파일에서 채워진다 —
> 직접 만들 때만 신경 쓴다.

유효하지 않은 값은 생성자가 `ValueError` 로 거부한다 (`command_format`, 빈
`command_topic`, 빈 `joint_names` 리스트).

### 6.2 상태 조회와 정리

| 멤버 | 설명 |
|---|---|
| `command_topic` / `command_format` | 생성 시 확정된 값 |
| `joint_names` | 확정된 명령 배열 순서. **조회 전이면 `None`** |
| `is_streaming` | 지금 궤적을 발행 중인지 |
| `close()` | 퍼블리셔 정리. **멱등**이며 `with` 블록이 자동 호출한다 |

### 6.3 스트리밍 인자

`stream()` 과 `stream_async()` 는 같은 키워드 인자를 받는다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `time_scaling` | `1.0` | 재생 시간 배율. `velocity_scaling` 과의 차이는 아래 §8.2 에 있다 |
| `publish_rate` | `None` | 균일 격자로 보간할 주파수(Hz). `None` 이면 궤적 원래 point 시각을 따른다 |
| `stop_event` | `None` | 외부 중단 이벤트 |
| `joint_names_timeout` | `10.0` | `joints` 자동 조회 한도(초) |
| `externally_spun` | `False` | Node 를 이미 다른 곳에서 spin 중이면 `True` |

`stream()` 은 궤적 길이만큼 블로킹하고 **발행한 point 수**를 돌려준다.
`stream_async()` 는 같은 일을 별도 스레드에서 하고 `Future` 를 돌려준다 — Tk GUI 처럼
호출 스레드를 막을 수 없을 때 쓴다.

### 6.4 중단

```python
stop_event = threading.Event()
future = streamer.stream_async(trajectory, stop_event=stop_event)
...
stop_event.set()          # 또는 streamer.stop()
print(future.result())    # 중단 전까지 발행한 point 수
```

---

## 7. replay_gui 연동

`replay_gui` 의 **"위치 초기화"** 버튼은 두 컨트롤러 환경을 모두 지원한다. 구현 선택은 **노드 생성 시 한 번만** 일어나며(`create_move_group_client()`), 이후 호출부는 분기 없이 `move_to_named_target_async()` 하나만 부른다. 어느 스택인지는 **`backend` 파라미터**로 알려준다.

**`backend` 에는 기본값이 없으며, 지정하지 않으면 노드가 기동하지 않는다.** 기본값을 두면 잘못된 추정이 오류 없이 통과하고, 그 결과는 "'위치 초기화' 가 동작하지 않는다" 로만 나타난다.

```bash
# JTC 환경 (replay_panda_mock.launch.py)
ros2 run rdfp replay_gui --config dataset_config.yaml \
    --ros-args -p backend:=mock

# JGPC 환경 (panda_mock_jgpc.launch.py)
ros2 run rdfp replay_gui --config dataset_config.yaml \
    --ros-args -p backend:=mock_jgpc
```

이동 대상은 기존과 동일하게 `init_named_target` 파라미터(기본 `ready`)로 바꾼다.

---

## 8. 파라미터 튜닝

### 8.1 `tolerance` — JGPC 에서 더 중요한 이유

`tolerance` 자체는 계획 파라미터라 두 구현이 같다 (기본 `1e-4` rad, 설명은 [프로그래머 가이드](move_group_client_user_guide.md) 참고).

다만 **JGPC 에서는 이 값이 곧 최종 자세를 결정한다.** JTC 는 컨트롤러가 `FollowJointTrajectory` 의 goal tolerance 를 따로 검사하고 미달이면 실패를 보고하지만, 스트리밍 경로에는 그런 재검사가 없다. 마지막으로 발행한 point 가 그대로 최종 명령이므로, 플래너가 tolerance 안에서 멈춘 위치가 곧 로봇이 서는 위치다.

```python
mgc.move_to_named_target('ready')                  # 기본 1e-4 rad
mgc.move_to_named_target('ready', tolerance=0.01)  # 최대 0.01 rad 벗어난 곳에 정지
```

### 8.2 `time_scaling` vs `velocity_scaling`

둘 다 느리게 만들지만 작용 지점이 다르다.

| 파라미터 | 작용 시점 | 효과 |
|---|---|---|
| `velocity_scaling` | 계획 | 궤적 자체를 느리게 재생성 (가감속 프로파일 반영) |
| `time_scaling` | 재생 | 계획된 궤적을 그대로 두고 재생 속도만 늘림 |

새 자세를 처음 시험할 때는 `time_scaling=2.0` 으로 한 번 확인한 뒤 `1.0` 으로 돌리는 것을 권장한다.

### 8.3 `publish_rate` — 명령 밀도

**기본값 `None` 은 궤적 point 시각에만 발행한다.** 그런데 MoveIt 은 시간 파라미터화 단계(TOTG, `resample_dt = 0.1`)에서 궤적을 **0.1초 격자로 리샘플**하므로, 궤적은 경로 길이와 무관하게 항상 약 10Hz 다. `GetCartesianPath` 요청에는 이를 바꿀 필드가 없고, `max_step` 을 줄여도 리샘플이 그 뒤에 일어나므로 point 수는 변하지 않는다.

JTC 는 컨트롤러가 point 사이를 자기 주기로 보간해 주지만 스트리밍에는 그 주체가 없다. 즉 **보간 없이 흘리면 10Hz 계단식 명령**이 된다. mock(`GenericSystem` 이 명령 위치로 즉시 점프)에서는 드러나지 않고 실기에서 나타난다.

`publish_rate` 를 주면 그 주기의 균일 격자를 만들어 위치를 선형 보간해 발행한다.

```python
mgc.follow_trajectory(waypoints)                      # 기본 — 약 10Hz
mgc.follow_trajectory(waypoints, publish_rate=200.0)  # 200Hz 보간 발행
```

실측 (같은 카테시안 경로, 재생 시간은 세 경우 모두 3.19초로 동일):

| `publish_rate` | 발행 명령 수 | 실측 주파수 | 명령 간격 |
|---|---|---|---|
| `None` (기본) | 33 | 10.0 Hz | 100.0 ms |
| `50.0` | 161 | 50.2 Hz | 20.0 ms |
| `200.0` | 639 | 200.2 Hz | 5.0 ms |

보간은 **재생 시간을 바꾸지 않는다** — 밀도만 올린다. 컨트롤러 update rate 를 넘는 값은 의미가 없으므로 그 이하로 잡는다. 궤적의 velocity/acceleration 필드는 어느 경우에도 사용하지 않고 position 만 발행한다.

---

## 9. 에러 처리

```python
try:
    mgc.move_to_named_target('ready')
except ValueError as exc:
    # name 이 SRDF 에 없음, 또는 컨트롤러가 요구하는 joint 이 궤적에 없음
    ...
except TimeoutError as exc:
    # SRDF 조회 / 계획 / joint 순서 조회 시간 초과
    ...
except RuntimeError as exc:
    # 계획 실패(MoveIt 오류 코드), 빈 궤적, close() 된 클라이언트
    ...
```

### 9.1 자주 보는 MoveIt 오류 코드

| 코드 | 이름 | 흔한 원인 |
|---|---|---|
| `-1` | `PLANNING_FAILED` | 경로를 못 찾음 |
| `-12` | `GOAL_IN_COLLISION` | 목표 자세가 충돌 |
| `-27` | `GOAL_STATE_INVALID` | 목표 자세가 자기충돌 / 한계 위반 |

> **참고**: 기본 panda SRDF 의 `transport` 자세는 이 스택에서 `-27` 로 실패한다
> (`panda_joint4 = -2.97` 로 접힌 자세가 자기충돌). JGPC 와 무관한 SRDF 자체의
> 성질이며 `ready` / `extended` 는 정상 동작한다.

### 9.2 증상별

| 증상 | 원인 | 해결 |
|---|---|---|
| 정상 반환했는데 도달 안 함 | open loop — 명령 발행 완료일 뿐 (§10) | 도달 확인이 필요하면 `/joint_states` 로 직접 확인하거나 정착을 기다린다. JTC 스택은 정상 반환이 도달을 뜻한다 |
| 동작이 10 Hz 로 끊긴다 / 접촉 작업이 실패한다 | MoveIt 궤적이 ~10 Hz 로 리샘플되고 스트리밍에는 보간 주체가 없다 (§8.3) | `publish_rate=50.0`~`200.0` — 프로파일의 `arm_command.publish_rate` 가 기본값을 준다 |
| `TimeoutError: … get_parameters` | ros2_control 이 없는 스택에서 joint 순서 자동 조회 (§2.1) | `arm_command_joint_names` 를 준다 — `backend` 를 주면 프로파일이 채운다 |
| 두 스트림이 뒤섞인다 | servo 가 같은 명령 토픽에 발행 중 (§10) | 스트리밍 전에 servo 를 정지시킨다 |

---

## 10. 주의사항

- **스트리밍 중 다른 명령 소스와 겹치면 안 된다.** servo 가 같은 `/panda_arm_controller/commands` 토픽에 발행 중이면 두 스트림이 뒤섞인다. 스트리밍 전에 servo 를 정지시키거나, 명령 시점을 분리한다.

- **첫 point 는 계획 시점의 현재 자세**다. 계획과 스트리밍 사이에 로봇이 움직였다면 첫 발행에서 급격한 이동이 생긴다. `plan_named_target()` 과 `stream_trajectory()` 사이를 길게 벌리지 않는다.

- **스트리밍은 블로킹이다.** `move_to_named_target()` 은 궤적 길이만큼 호출 스레드를 잡는다. `timeout` 파라미터는 계획 단계까지만 적용된다.

- **`cancel()` 은 급정지다.** 스트리밍을 멈추면 컨트롤러가 마지막으로 받은 명령 위치를 그대로 유지하므로 감속 프로파일이 없다. 정지 후 로봇은 경로 중간의 불확정 자세에 있으므로, 이어서 동작시키기 전에 자세를 확인하거나 named target 으로 복귀시킨다.

- **재사용 가능하다.** `TrajectoryStreamer` 는 replay 계열 replayer 와 달리
  one-shot 이 아니다. 같은 인스턴스로 여러 번 `stream()` 할 수 있다.

- **JTC 환경에서는 쓰지 않는다.** JTC 는 `/panda_arm_controller/commands` 를 제공하지 않으므로 발행해도 아무 일도 일어나지 않는다. `create_move_group_client(node, backend=...)` 를 쓰면 프로파일이 알맞은 구현을 고르므로 직접 고를 일이 없다.

- **open loop 다.** 명령을 발행할 뿐 도달 여부를 확인하지 않는다. 반환값은 "발행한 명령 수"이지 "도달했는지"가 아니다. 로봇이 명령을 못 따라가도 정상 종료한다.

- **velocity/acceleration 은 버려진다.** 궤적의 position 만 발행한다. MoveIt 은 시간 파라미터화 단계에서 궤적을 0.1초 격자(=10Hz)로 리샘플하므로, 보간 없이 그대로 흘리면 실기에서 계단식 명령이 된다. `publish_rate` 인자로 발행 주파수를 올려 위치를 선형 보간할 수 있다 (예: `publish_rate=200.0`). `max_step` 을 줄여도 point 밀도는 늘지 않는다 — 리샘플이 그 뒤에 일어나기 때문이다.

---

## 11. 관련 문서

- [move_group_client_user_guide.md](move_group_client_user_guide.md) — 공통 인터페이스 + JTC 구현 + 팩토리 (JGPC 전용 내용은 이 문서에만 있다)
- [../../src/robot_control/launch/README.md](../../src/robot_control/launch/README.md) — `panda_mock_jgpc.launch.py` 포함 제어 계열 launch 인벤토리
- [../replay/](../collect/) — replay 서브시스템 문서
