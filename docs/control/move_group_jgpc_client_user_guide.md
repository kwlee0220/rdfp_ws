# `MoveGroupJgpcClient` 사용 가이드 (JGPC / 스트리밍 실행)

이 문서는 `FollowJointTrajectory` 액션이 없는 스택에서 **MoveIt 으로 궤적을 계획하고 명령 토픽으로 관절 목표값을 순차 발행하는(스트리밍) 방법**을 설명한다. named target · joint 목표 · Cartesian 경로 이동, 중단, `TrajectoryStreamer` 직접 사용, `publish_rate` 튜닝까지 이 구현 전반을 다룬다. 백엔드 `panda_functionbay` 가 여기에 해당한다.

공통 인터페이스와 JTC 구현은 [move_group_client_user_guide.md](move_group_client_user_guide.md)에서 설명한다. **JGPC 전용 내용은 이 문서에만 있다.** 공통 가이드에서 JGPC 는 §1.2 대조표와 포인터로만 나타난다.

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

펑션베이 백엔드는 ros2_control 없이 시뮬레이터와 토픽으로 직접 연결된다. 팔 명령은 `/input/panda_joint`(`sensor_msgs/JointState`)로 보내고, 그 앞에 궤적을 받아 줄 컨트롤러가 없으므로 **`FollowJointTrajectory` 액션이 존재하지 않는다**. ros2_control 의 `position_controllers/JointGroupPositionController`(JGPC)처럼 위치 배열을 그대로 받는 컨트롤러를 쓰는 스택도 동일하게 액션 기반 궤적 실행을 지원하지 않는다.

그런데 MoveIt 의 `moveit_simple_controller_manager` 는 `FollowJointTrajectory` 와 `GripperCommand` 두 타입만 다룰 수 있다. 그래서 기존 API 는 이렇게 갈린다.

| API 부류 | JTC 환경 | JGPC 환경 | 이유 |
|---|:-:|:-:|---|
| SRDF 조회 (`get_*`) | ✅ | ✅ | `move_group` 파라미터 조회 — 컨트롤러 무관 |
| 계획 전용 (`plan_*`) | ✅ | ✅ | Cartesian 서비스 / `MoveGroup` **plan_only** — 실행 안 함 |
| 계획 + 실행 (`move_to_*`, `follow_trajectory`) | ✅ | ✅ | **구현이 다르다** — JTC 는 액션, JGPC 는 스트리밍 |
| `execute_trajectory()` | ✅ | — | `MoveGroupJtcClient` 에만 있음 |
| `stream_trajectory()` / `*_streamed()` | — | ✅ | `MoveGroupJgpcClient` 에만 있음 |

메서드 단위 전체 대조표는 [move_group_client_user_guide.md](move_group_client_user_guide.md) §1.2 「구현별 API 가용성」에 있다.

즉 **계획 기능은 그대로 사용할 수 있으며, 실행에는 명령 토픽을 이용하는 별도 경로가 필요하다.**

그래서 클라이언트를 계획(공통) + 실행(구현별)으로 나눴다. `MoveGroupClient` 는 추상 공통 인터페이스이고, 실행은 `MoveGroupJtcClient` / `MoveGroupJgpcClient` 가 각자 구현한다. `create_move_group_client(node, backend=...)` 가 백엔드 프로파일에서 알맞은 쪽을 골라 돌려주므로, 호출부는 명령 채널을 몰라도 된다.

---

## 2. 동작 원리

JTC 는 궤적을 통째로 받아 **컨트롤러 내부에서 시간 보간**을 한다. 그러나 스트리밍 경로의 수신자(펑션베이 시뮬레이터나 JGPC)는 받은 값을 그대로 목표 위치로 쓰기 때문에, 그 시간 보간을 클라이언트가 대신해야 한다.

> **Note — '시간 보간' 이란**
>
> 계획 결과인 `RobotTrajectory` 는 연속 곡선이 아니라 **`time_from_start` 가 붙은 이산 point 의 목록**이다. 현재 구성의 TOTG 재샘플링 간격은 0.1초다. 다만 최종 경유점 간격은 계획 방식과 속도 · 재생 시간 배율에 따라 달라진다(§8.3). 반면 로봇은 그보다 훨씬 촘촘한 주기로 위치 명령을 받아야 매끄럽게 움직인다. 그래서 **두 point 사이의 중간 위치를 시각에 맞춰 만들어 내는 단계**가 어딘가에는 반드시 있어야 하고, 그 단계가 시간 보간이다.
>
> **JTC 는 그 단계를 컨트롤러 안에서 한다.** 궤적을 통째로 받아 두고, 자기 update rate 마다 "지금 시각에 해당하는 위치"를 계산해 hardware 에 쓴다. 그래서 10 Hz 궤적을 줘도 팔은 매끄럽게 움직인다 — 클라이언트는 궤적을 넘기고 나면 할 일이 없다.
>
> **스트리밍 경로에는 그 단계가 없다.** 받은 값을 그대로 목표로 쓰므로 **팔이 실제로 받는 것은 우리가 발행한 명령 그 자체**다. 10 Hz 로 발행하면 10 Hz 계단이 그대로 팔에 들어간다. 그래서 `TrajectoryStreamer` 가 이 일을 대신하며, 밀도는 `publish_rate` 인자를 통해 정한다 (§8.3).

```text
[JTC 경로]
  MoveGroup 액션 (plan_only=True)        ← 데카르트 경로는 GetCartesianPath 서비스
    └→ RobotTrajectory 반환
         └→ ExecuteTrajectory 액션 (move_group)
              └→ FollowJointTrajectory 액션 (/panda_arm_controller/follow_joint_trajectory)
                   └→ JTC 내부에서 궤적 시간 보간 → hardware

[JGPC 경로 — 본 가이드]
  MoveGroup 액션 (plan_only=True)
    └→ RobotTrajectory 반환
         └→ TrajectoryStreamer 가 time_from_start 에 맞춰 point 를 발행 (publish_rate 로 보간 가능)
              └→ 펑션베이: /input/panda_joint  (sensor_msgs/JointState)
                 JGPC   : /panda_arm_controller/commands  (std_msgs/Float64MultiArray)
                   └→ 수신자가 그대로 목표 위치로 사용
```

### 2.1 joint 순서 문제

명령 형식은 두 가지다. 펑션베이가 받는 `sensor_msgs/JointState` 는 joint 이름을 함께 싣고, `TrajectoryStreamer` 는 이름 목록을 백엔드 프로파일(`arm_command.joint_names`)에서 받는다. 펑션베이 시뮬레이터는 `velocity` 가 비어 있으면 명령을 통째로 무시하므로, 스트리머는 `velocity` 를 0 으로 채워 보낸다.

ros2_control JGPC 가 받는 `std_msgs/Float64MultiArray` 에는 **joint 이름이 없다.** 배열 순서가 컨트롤러의 `joints` 파라미터 순서와 어긋나면 엉뚱한 관절이 움직인다. 이 형식에서 `TrajectoryStreamer` 는 컨트롤러 노드의 `get_parameters` 서비스로 `joints` 파라미터를 읽어 순서를 확정하고, 궤적의 `joint_names` 를 그 순서로 재배열한다. 사용자가 신경 쓸 필요는 없지만, 직접 발행할 때는 반드시 같은 재배열을 해야 한다.

### 2.2 완료와 중단의 의미

스트리밍 메서드의 정상 반환은 **명령 발행이 끝났다**는 뜻이지 **로봇이 목표에 도달했다**는 뜻이 아니다. 스트리머는 관측값으로 도달 여부를 검사하지 않으며, 이것은 하위 제어기가 피드백 제어를 하는지와는 별개다. `stream_trajectory()` 와 동기 `*_streamed()` 메서드는 발행한 명령 수를 반환하고, `move_to_named_target()` · `move_to_joints()` · `follow_trajectory()` 는 `None` 을 반환한다. 도달 여부는 `/joint_states` 같은 관측값으로 확인하거나 정착을 기다린다. 중단(`cancel()` · `stop_streaming()`)도 명령 발행을 멈출 뿐 물리적 정지 완료를 뜻하지 않는다(§4.4).

---

## 3. Quick Start

### 3.1 스택 실행

```bash
ros2 launch robot_control panda_functionbay.launch.py
```

시뮬레이터를 띄우는 순서는 [펑션베이_시뮬레이터_사용법.md](../backends/functionbay/펑션베이_시뮬레이터_사용법.md) §2 를 따른다.

### 3.2 가장 간단한 사용법 (동기)

```python
import rclpy
from rclpy.node import Node

from robot_control.moveit import create_move_group_client

rclpy.init()
node = Node('named_target_demo')

# 프로파일이 JGPC 로 정해져 있으므로 MoveGroupJgpcClient 가 돌아온다.
with create_move_group_client(node, backend='panda_functionbay') as mgc:
    print(mgc.get_named_targets())            # ['extended', 'ready', 'transport']
    mgc.move_to_named_target('ready')         # JGPC 구현 = 계획 + 스트리밍

node.destroy_node()
rclpy.shutdown()
```

`MoveGroupJgpcClient.move_to_named_target()` 은 계획 + 스트리밍을 한 번에 수행하며 궤적 길이만큼 블로킹된다. 발행한 명령 개수가 필요하면 같은 뜻의 `move_to_named_target_streamed()` 를 직접 호출한다.

### 3.3 구현 선택

**`backend` 이름 하나로 구현이 정해진다.** 어느 스택이 JTC 이고 어느 쪽이 JGPC 인가는 백엔드 프로파일이 갖고 있으므로, 호출부는 컨트롤러 종류도 명령 채널도 알 필요가 없다.

```python
mgc = create_move_group_client(node, backend='panda_functionbay')  # -> MoveGroupJgpcClient
mgc = create_move_group_client(node, backend='panda_isaac')        # -> MoveGroupJtcClient
```

**`backend` 와 `mode` 중 하나는 반드시 지정한다** — 둘 다 생략하면 `ValueError` 가 발생한다. `mode` 는 프로파일이 없는 스택을 붙일 때 구현을 직접 지정하는 대안이며, 프로파일 표와 함께 [move_group_client_user_guide.md](move_group_client_user_guide.md) §1.1 에서 다룬다.

### 3.4 목표 조인트로 이동 (`move_to_joints()`)

`move_to_joints()` 는 SRDF 조회 단계만 없을 뿐 named target 이동과 동일한 계획 · 스트리밍 절차를 사용한다. 지정하지 않은 관절은 현재 값으로 채운다.

```python
mgc.move_to_joints({'panda_joint1': 0.5, 'panda_joint4': -2.0}, publish_rate=200.0)
```

### 3.5 계획과 실행 분리

궤적을 검사하거나 재사용하고 싶을 때:

```python
trajectory = mgc.plan_named_target('extended')
print(f'{len(trajectory.joint_trajectory.points)} points')

# 아래 실행 방식 중 하나를 선택한다.
mgc.stream_trajectory(trajectory)                       # 원래 속도
# mgc.stream_trajectory(trajectory, time_scaling=2.0)   # 재생 시간 두 배 (느리게)

# joint 목표값 판 — SRDF 조회가 없다. 위 실행 뒤의 현재 자세에서 다시 계획한다
trajectory = mgc.plan_joints({'panda_joint1': 0.5})
mgc.stream_trajectory(trajectory, publish_rate=200.0)
```

궤적을 다시 실행하기 전에는 관측값으로 현재 자세와 궤적 시작 자세가 일치하는지 확인해야 한다. 첫 실행으로 자세가 바뀐 상태에서 같은 궤적을 다시 보내면 이전 시작점이 다시 명령되어 급격한 이동이 발생할 수 있다. 자세가 달라졌다면 현재 상태에서 다시 계획한다.

---

## 4. API 레퍼런스

### 4.1 `MoveGroupJgpcClient` 생성자 추가 인자

`create_move_group_client()` 가 백엔드 프로파일에서 채워 그대로 전달한다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `arm_command_topic` | `/panda_arm_controller/commands` | 스트리밍 대상 명령 토픽. 펑션베이 프로파일은 `/input/panda_joint` |
| `arm_command_joint_names` | `None` | 명령 joint 순서. `None` 이면 컨트롤러에서 자동 조회한다 — 컨트롤러가 없는 펑션베이에서는 반드시 줘야 하며 프로파일이 채운다 |
| `arm_command_format` | `float64_multi_array` | 명령 메시지 형식. 펑션베이는 `joint_state` |
| `gravity_compensator` | `None` | 명령점에 중력 보상 오프셋을 더하는 보상기. 프로파일에서 고른 scene 에 `gravity_compensation` 블록이 있을 때만 생긴다 |
| `publish_rate` | `None` | 스트리밍 기본 발행 주파수(Hz). 펑션베이 프로파일의 `arm_command.publish_rate` 는 50.0 이다. 호출 인자가 우선한다 |

명령 퍼블리셔는 처음 스트리밍할 때 생성한다. 스트리밍 전에는 이 클라이언트가 해당 명령 토픽의 퍼블리셔를 만들지 않는다. JTC 환경에서는 `MoveGroupJtcClient` 가 생성되므로 이 인자들이 아예 쓰이지 않는다.

### 4.2 스트리밍 실행 메서드

`MoveGroupJgpcClient` 에만 있다.

| 메서드 | 블로킹 | 반환 | 역할 |
|---|:-:|---|---|
| `stream_trajectory(traj, ...)` | ✅ | `int` (발행 명령 수) | 궤적을 명령 토픽으로 발행. JTC 의 `execute_trajectory()` 대응 |
| `stop_streaming()` | ❌ | `None` | 진행 중인 스트리밍 중단 |
| `follow_trajectory_streamed(wps, ...)` | ✅ | `int` | 카테시안 계획 + 스트리밍 |
| `move_to_named_target_streamed(name, ...)` | ✅ | `int` | named target 계획 + 스트리밍 |
| `move_to_named_target_streamed_async(name, ...)` | ❌ | `Future` → `int` | 위의 비동기판 |
| `move_to_joints_streamed(joint_values, ...)` | ✅ | `int` | joint 목표값 계획 + 스트리밍 |
| `move_to_joints_streamed_async(joint_values, ...)` | △ | `Future` → `int` | 위의 비동기판. 아래 주의 |

`stream_trajectory(trajectory, *, time_scaling=1.0, velocity_scaling=1.0, publish_rate=None, externally_spun=False, target=None)` — `velocity_scaling` 은 궤적에 이미 적용한 배율을 실행 궤적(R1)에 기록하기 위한 값이며 궤적을 다시 스케일하지 않는다. `target` 은 연산 이벤트(R2)에 실을 목표다. `externally_spun` 은 joint 순서 자동 조회 시에만 뜻이 있다(§5.1). `stop_streaming()` 은 인자가 없다. JTC 클라이언트에서 이 메서드들을 부르면 `AttributeError` 다 — 그 클래스에 없다.

### 4.3 공통 API 의 JGPC 구현

이름은 base 와 같지만 실행이 스트리밍으로 바뀐 메서드다. **동기 메서드 셋은 대응하는 `*_streamed` 를 그대로 호출하는 얇은 래퍼**이며, 반환값을 버려 base 시그니처에 맞춘다. 발행한 명령 수가 필요하면 `*_streamed` 쪽을 직접 부른다. 비동기 메서드는 결과를 `Future` 로 전달하며, `follow_trajectory_async()` 는 별도의 계획 콜백 체인을 구현한다.

| 메서드 | 블로킹 | 반환 | 위임 대상 |
|---|:-:|---|---|
| `move_to_named_target(name, ...)` | ✅ | `None` | `move_to_named_target_streamed()` |
| `move_to_named_target_async(name, ...)` | ❌ | `Future` → `int` | `move_to_named_target_streamed_async()` |
| `move_to_joints(joint_values, ...)` | ✅ | `None` | `move_to_joints_streamed()` |
| `move_to_joints_async(joint_values, ...)` | △ | `Future` → `int` | `move_to_joints_streamed_async()` |
| `follow_trajectory(wps, ...)` | ✅ | `None` | `follow_trajectory_streamed()` |
| `follow_trajectory_async(wps, ...)` | ❌ | `Future` → `CartesianPathResult` | 계획 콜백 체인 + 스트리밍 스레드 |

이 여섯 개는 base 시그니처에 없는 `time_scaling` / `publish_rate` 를 추가로 받는다. `follow_trajectory_async()` 의 완료 결과 `CartesianPathResult` 에서 `fraction` 은 계획된 경로의 비율이고 `published` 는 발행한 명령 수다.

△ 표시는 비동기 관절 목표 API 다. 이 메서드는 `Future` 를 돌려주기 전에, 계획에 필요한 관절 정보와 지정하지 않은 관절의 현재값을 확보하는 동안 호출 스레드를 대기시킬 수 있다.

### 4.4 중단 — `cancel()`

`cancel()` 은 두 구현 모두에 있지만 정지 특성이 다르다.

| 구현 | 수단 | 정지 특성 |
|---|---|---|
| JTC | 컨트롤러 `FollowJointTrajectory` 액션에 `CancelGoal` | 컨트롤러가 감속 후 정지 |
| **JGPC** | 스트리밍 중단(`stop_streaming()`) + 계획 단계 MoveGroup goal 취소 | 마지막 목표값이 유지된다 — **감속 궤적 없음** |

JGPC 의 `cancel()` 은 명령 스트리밍을 중단하며 별도의 감속 궤적을 생성하지 않는다. 수신자는 마지막 목표값을 유지하므로(펑션베이 팔도 마지막 지령을 붙든다), 실제 정지 위치와 시간은 하위 제어기와 로봇의 동특성에 따라 달라진다. 반환값 `True` 는 중단 대상이 있었다는 뜻이며 물리적 정지 완료를 의미하지 않는다. 중단 후의 실제 자세는 관측값으로 확인하고, 후속 이동은 확인된 현재 자세를 기준으로 계획한다.

### 4.5 계획 메서드 (컨트롤러 무관, base 제공)

JGPC 환경에서 실행 전에 궤적을 얻을 때 쓴다. 세 메서드 모두 JTC 환경에서도 동일하다.

| 메서드 | 반환 | 비고 |
|---|---|---|
| `plan_named_target(name, ...)` / `_async` | `RobotTrajectory` / `Future` → `RobotTrajectory` | `MoveGroup` **plan_only** — 실행 안 함 |
| `plan_joints(joint_values, ...)` / `_async` | `RobotTrajectory` / `Future` → `RobotTrajectory` | 위와 같되 SRDF 조회 없음 |
| `plan_trajectory(wps, ...)` / `_async` | `RobotTrajectory` / `Future` → `GetCartesianPath.Response` | `/compute_cartesian_path` 서비스. 비동기판은 서비스 응답을 그대로 주므로 호출자가 `error_code` · `fraction` · `solution` 을 확인한다 |

### 4.6 스트리밍 전용 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `time_scaling` | `1.0` | 재생 시간 배율. `2.0` = 두 배 느리게 |
| `publish_rate` | `None` | 발행 주파수(Hz). 값을 주면 그 주기로 위치를 선형 보간한다. `None` 이면 생성자의 기본 발행 주파수를 쓰고, 그것도 `None` 일 때만 궤적 point 시각에 발행한다 |
| `externally_spun` | `False` | Node 가 다른 스레드에서 이미 spin 중이면 `True` |
| `timeout` | 120.0 / 140.0 | 동기 named target · 관절 이동은 120초, 동기 데카르트 이동(`follow_trajectory` · `follow_trajectory_streamed`)은 140초다. 스트리밍 실행 시간은 이 제한에 포함되지 않는다. 비동기 메서드에는 이 인자가 없다 |

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

명령 발행 자체는 spin 이 필요 없지만, 서비스 · 토픽으로 정보를 조회하는 단계는 spin 이 필요하다. `TrajectoryStreamer` 에서는 `externally_spun` 이 joint 순서 자동 조회의 대기 방식에 영향을 준다. `move_to_joints_async()` 에서는 계획에 필요한 관절 정보와 지정하지 않은 관절의 현재값을 조회할 때도 쓴다. 노드가 외부 executor 에서 실행 중이면 `True` 로 지정한다.

| 상황 | 값 | 이유 |
|---|:-:|---|
| 아무도 spin 하지 않음 (일반 스크립트) | `False` | 라이브러리가 직접 `spin_until_future_complete` |
| `MultiThreadedExecutor` 등이 백그라운드 spin 중 | `True` | 직접 spin 하면 "node already spinning" 오류 |

위 표는 관절 순서와 현재 상태 등을 조회할 때의 내부 대기 방식에 관한 것이다. 비동기 계획 · 이동 API 의 응답 콜백을 처리하려면 호출자가 노드를 spin 해야 한다. `externally_spun=False` 가 비동기 연산 전체의 spin 을 대신하지는 않는다.

`arm_command_joint_names` 를 생성자에서 명시하면(펑션베이 프로파일처럼) joint 순서 조회는 생략되지만, 관절 목표 API 의 현재값 조회에는 여전히 이 값이 쓰인다.

---

## 6. `TrajectoryStreamer` — 직접 사용과 API

MoveGroup 클라이언트 없이 임의의 궤적을 스트리밍하려면 `TrajectoryStreamer` 를 직접 쓴다. Cartesian 계획 결과(`plan_trajectory()`)도 그대로 넣을 수 있다.

```python
from robot_control.moveit.trajectory_streamer import TrajectoryStreamer

with TrajectoryStreamer(node, command_topic='/input/panda_joint', command_format='joint_state',
                        joint_names=[f'panda_joint{i}' for i in range(1, 8)]) as streamer:
    # 두 방식 중 하나를 선택한다.
    streamer.stream(trajectory)                        # 경유점 시각에만 발행
    # streamer.stream(trajectory, publish_rate=50.0)   # 50Hz 로 보간하여 발행
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
| `joint_names` | 명령 joint 순서. `None` 이면 첫 스트리밍 때 컨트롤러의 `joints` 파라미터를 조회한다 |
| `controller_node_name` | `joints` 를 조회할 컨트롤러 노드 이름. 이름 규칙이 다른 스택에서만 준다 |
| **`command_format`** | `'float64_multi_array'`(기본, ros2_control JGPC) 또는 `'joint_state'`(펑션베이 같은 토픽 연동형 시뮬레이터). 아래 주의 |
| `qos_depth` | 명령 퍼블리셔의 QoS depth |
| **`gravity_compensator`** | `robot_control.gravity.GravityCompensator` 또는 `None`. 주면 **발행 직전** 모든 명령점에 `τ_g(q)/Kp` 를 더해, 중력 보상이 없는 백엔드에서 중력으로 인한 관절 위치의 정상상태 오차를 줄인다 |

> ### ⚠️ `command_format` 이 맞지 않으면 오류 없이 연결되지 않는다
>
> 펑션베이처럼 **`sensor_msgs/JointState` 를 받는 시뮬레이터**에 기본값(`float64_multi_array`)으로 보내면 ROS 2 는 타입이 달라 **연결만 하지 않고 오류를 내지 않는다** — 관측되는 증상은 "팔이 명령에 반응하지 않는다" 뿐이다. 그 백엔드에서는 `command_format='joint_state'` 를 준다.
>
> **그때는 `joint_names` 도 함께 줘야 한다.** ros2_control 이 없어 `joints` 를 조회할 컨트롤러 노드가 존재하지 않으므로, 자동 조회가 `TimeoutError: Timed out waiting for /input/get_parameters` 로 실패한다.
>
> `create_move_group_client(node, backend=...)` 를 쓰면 이 둘이 프로파일에서 채워진다 — 직접 만들 때만 신경 쓴다.

유효하지 않은 값은 생성자가 `ValueError` 로 거부한다 (`command_format`, 빈 `command_topic`, 빈 `joint_names` 리스트).

### 6.2 상태 조회와 정리

| 멤버 | 설명 |
|---|---|
| `command_topic` / `command_format` | 생성 시 확정된 값 |
| `joint_names` | 확정된 명령 순서. **조회 전이면 `None`** |
| `resolve_joint_names()` | 명령 순서를 확정해 돌려준다. 생성자에 `joint_names` 를 줬으면 그 값이고, 아니면 컨트롤러에서 조회한다 |
| `is_streaming` | 지금 궤적을 발행 중인지 |
| `stop()` | 진행 중인 스트리밍을 멈춘다 |
| `close()` | 퍼블리셔 정리. **멱등**이며 `with` 블록이 자동 호출한다 |

### 6.3 스트리밍 인자

`stream()` 과 `stream_async()` 는 같은 키워드 인자를 받는다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `time_scaling` | `1.0` | 재생 시간 배율. `velocity_scaling` 과의 차이는 아래 §8.2 에 있다 |
| `velocity_scaling` | `1.0` | 궤적에 이미 적용한 속도 배율. 실행 궤적(R1)에 기록할 뿐 궤적을 다시 스케일하지 않는다 |
| `publish_rate` | `None` | 균일 격자로 보간할 주파수(Hz). `None` 이면 궤적 원래 point 시각을 따른다 |
| `stop_event` | `None` | 외부 중단 이벤트 |
| `joint_names_timeout` | `10.0` | `joints` 자동 조회 한도(초) |
| `externally_spun` | `False` | Node 를 이미 다른 곳에서 spin 중이면 `True` |
| `target` | `None` | 연산 이벤트(R2)에 실을 목표 |

`stream()` 은 궤적 길이만큼 블로킹하고 **발행한 point 수**를 돌려준다. `stream_async()` 는 같은 일을 별도 스레드에서 하고 `Future` 를 돌려준다 — Tk GUI 처럼 호출 스레드를 막을 수 없을 때 쓴다.

### 6.4 중단

```python
stop_event = threading.Event()
future = streamer.stream_async(trajectory, stop_event=stop_event)

def on_done(fut):
    try:
        print(fut.result())  # 중단 처리 완료 후 발행 명령 수
    except Exception as exc:
        node.get_logger().error(f'스트리밍 실패: {exc}')

future.add_done_callback(on_done)
...
stop_event.set()          # 또는 streamer.stop()
```

`stream_async()` 가 돌려주는 것은 `rclpy.task.Future` 다. 이 객체의 `result()` 는 완료를 기다리지 않으므로, `stop_event.set()` 직후에 부르면 아직 결과가 없어 `None` 이 나올 수 있다. 결과는 완료 콜백에서 읽는다.

---

## 7. replay_gui 연동

`replay_gui` 의 **"위치 초기화"** 버튼은 두 컨트롤러 환경을 모두 지원한다. 구현 선택은 **노드 생성 시 한 번만** 일어나며(`create_move_group_client()`), 이후 호출부는 분기 없이 `move_to_named_target_async()` 하나만 부른다. 어느 스택인지는 **`backend` 파라미터**로 알려준다.

**`backend` 에는 기본값이 없으며, 지정하지 않으면 노드가 기동하지 않는다.** 기본값을 두면 잘못된 추정이 오류 없이 통과하고, 그 결과는 "'위치 초기화' 가 동작하지 않는다" 로만 나타난다.

```bash
# JGPC 환경 (펑션베이)
ros2 run rdfp replay_gui --config dataset_config.yaml \
    --ros-args -p backend:=panda_functionbay

# JTC 환경 (isaac)
ros2 run rdfp replay_gui --config dataset_config.yaml \
    --ros-args -p backend:=panda_isaac
```

이동 대상은 `init_named_target` 파라미터(기본 `ready`)로 바꾼다.

---

## 8. 파라미터 튜닝

### 8.1 `tolerance` — JGPC 에서 더 중요한 이유

`tolerance` 자체는 계획 파라미터라 두 구현이 같다 (기본 `1e-4` rad, 설명은 [프로그래머 가이드](move_group_client_user_guide.md) 참고).

`tolerance` 는 계획된 끝점이 목표 관절값에서 벗어날 수 있는 범위를 지정한다. JTC 는 컨트롤러가 `FollowJointTrajectory` 의 goal tolerance 를 따로 검사하고 미달이면 실패를 보고하지만, JGPC 경로는 실행 후 실제 도달 오차를 검사하지 않는다. 그래서 이 값만으로 최종 관절 오차를 보장할 수 없으며(중력 보상이 설정되면 발행 명령에 오프셋도 더해진다), 도달 여부는 `/joint_states` 등 관측값으로 확인해야 한다.

```python
mgc.move_to_named_target('ready')                  # 기본 1e-4 rad
mgc.move_to_named_target('ready', tolerance=0.01)  # 계획 목표의 관절 허용오차를 0.01 rad 로 지정한다
```

### 8.2 `time_scaling` vs `velocity_scaling`

둘 다 느리게 만들지만 작용 지점이 다르다.

| 파라미터 | 작용 시점 | 효과 |
|---|---|---|
| `velocity_scaling` | 계획 | 관절 공간 계획에서는 MoveIt 계획 요청에 반영한다. 데카르트 계획에서는 반환된 궤적의 시간 · 속도 · 가속도를 클라이언트가 조정한다 |
| `time_scaling` | 재생 | 계획된 궤적은 그대로 두고 스트리밍할 상대 시각에 배율을 곱한다. `2.0` 이면 재생 시간이 두 배가 된다 |

새 자세를 처음 시험할 때는 `time_scaling=2.0` 으로 한 번 확인한 뒤 `1.0` 으로 돌리는 것을 권장한다.

### 8.3 `publish_rate` — 명령 밀도

**보간하지 않으면 궤적 point 시각에만 발행한다.** 현재 구성의 TOTG 재샘플링 간격(`resample_dt`)은 0.1초이지만, 최종 경유점 간격은 계획 방식과 배율 적용에 따라 달라진다 — 데카르트 궤적은 클라이언트가 시각을 `1/vs` 배 늘리므로 속도 배율이 낮을수록 더 성겨진다. `GetCartesianPath` 요청에는 이 간격을 바꿀 필드가 없다. `max_step` 은 데카르트 경로 계산의 공간 간격을 조절하는 값이며, 최종 명령 발행 주파수를 직접 지정하는 값은 아니다.

JTC 는 컨트롤러가 point 사이를 자기 주기로 보간해 주지만 스트리밍에는 그 주체가 없다. 보간하지 않으면 배율이 반영된 각 경유점 시각에만 위치 명령을 발행하므로, 경유점 사이에서 목표값이 계단식으로 바뀐다. 아래 측정 사례에서는 발행 주파수가 약 10Hz 였다. 펑션베이에서는 여유 0.5 mm 인 구멍에 peg 을 넣을 때 이 계단 때문에 실제로 실패했다.

`publish_rate` 를 주면 그 주기의 균일 격자를 만들어 위치를 선형 보간해 발행한다. 펑션베이 프로파일은 `arm_command.publish_rate: 50.0` 을 클라이언트 기본값으로 준다. 클라이언트 메서드에 `publish_rate=None` 을 넘기면 이 기본값을 쓰므로, 보간 없이 point 시각에만 발행하려면 `TrajectoryStreamer.stream()` 을 직접 부른다.

```python
mgc.follow_trajectory(waypoints)                      # 프로파일 기본값 — 펑션베이 50Hz 보간
mgc.follow_trajectory(waypoints, publish_rate=200.0)  # 200Hz 보간 발행
```

`TrajectoryStreamer.stream()` 으로 같은 카테시안 경로를 발행한 실측이다(재생 시간은 세 경우 모두 3.19초로 동일).

| `publish_rate` | 발행 명령 수 | 실측 주파수 | 명령 간격 |
|---|---|---|---|
| `None` | 33 | 10.0 Hz | 100.0 ms |
| `50.0` | 161 | 50.2 Hz | 20.0 ms |
| `200.0` | 639 | 200.2 Hz | 5.0 ms |

보간은 **재생 시간을 바꾸지 않고** 명령 발행 간격만 줄인다. 발행 주파수는 수신자의 명령 처리 주파수와 통신 · 연산 부하를 고려해 정한다. 궤적의 velocity/acceleration 필드는 어느 경우에도 명령에 싣지 않고 position 만 발행한다(`joint_state` 형식의 `velocity` 는 0 이다).

---

## 9. 에러 처리

```python
from robot_control.errors import PlanningError

try:
    mgc.move_to_named_target('ready')
except ValueError as exc:
    # name 이 SRDF 에 없음, 또는 컨트롤러가 요구하는 joint 이 궤적에 없음
    ...
except TimeoutError as exc:
    # SRDF 조회 / 계획 / joint 순서 조회 시간 초과
    ...
except PlanningError as exc:
    # 계획 실패(MoveIt 오류 코드), 빈 궤적 — robot_control.errors, RuntimeError 하위
    ...
except RuntimeError as exc:
    # close() 된 클라이언트 등
    ...
```

### 9.1 자주 보는 MoveIt 오류 코드

| 코드 | 이름 | 흔한 원인 |
|---|---|---|
| `-1` | `PLANNING_FAILED` | 경로를 못 찾음 |
| `-12` | `GOAL_IN_COLLISION` | 목표 자세가 충돌 |
| `-27` | `GOAL_STATE_INVALID` | 목표 자세가 자기충돌 / 한계 위반 |

> **참고**: 기본 panda SRDF 의 `transport` 자세는 이 스택에서 `-27` 로 실패한다 (`panda_joint4 = -2.97` 로 접힌 자세가 자기충돌). JGPC 와 무관한 SRDF 자체의 성질이며 `ready` / `extended` 는 정상 동작한다.

### 9.2 증상별

| 증상 | 원인 | 해결 |
|---|---|---|
| 정상 반환했는데 도달 안 함 | open loop — 명령 발행 완료일 뿐 (§10) | 도달 확인이 필요하면 `/joint_states` 로 직접 확인하거나 정착을 기다린다. JTC 스택은 정상 반환이 도달을 뜻한다 |
| 동작이 계단식으로 끊긴다 / 접촉 작업이 실패한다 | 보간 없이 경유점 시각에만 명령을 발행하여 목표값이 계단식으로 바뀜 (§8.3) | `publish_rate=50.0`~`200.0` — 프로파일의 `arm_command.publish_rate` 가 기본값을 준다 |
| `TimeoutError: … get_parameters` | ros2_control 이 없는 스택에서 joint 순서 자동 조회 (§2.1) | `arm_command_joint_names` 를 준다 — `backend` 를 주면 프로파일이 채운다 |
| 두 스트림이 뒤섞인다 | servo 가 같은 명령 토픽에 발행 중 (§10) | 스트리밍 전에 servo 를 정지시킨다 |

---

## 10. 주의사항

- **스트리밍 중 다른 명령 소스와 겹치면 안 된다.** 펑션베이에서는 servo 출력도 `servo_command_bridge` 를 거쳐 같은 `/input/panda_joint` 로 나간다. servo 가 발행 중이면 두 스트림이 뒤섞이므로, 스트리밍 전에 servo 를 정지시키거나 명령 시점을 분리한다.
- **첫 point 는 계획 시점의 현재 자세**다. 계획과 스트리밍 사이에 로봇이 움직였다면 첫 발행에서 급격한 이동이 생긴다. `plan_named_target()` 과 `stream_trajectory()` 사이를 길게 벌리지 않는다.
- **스트리밍은 블로킹이다.** `move_to_named_target()` 은 궤적 길이만큼 호출 스레드를 잡는다. `timeout` 파라미터는 계획 단계까지만 적용된다.
- **`cancel()` 은 감속 궤적을 만들지 않는다.** 스트리밍을 멈추면 수신자가 마지막 목표값을 유지하며, 실제 정지 위치와 시간은 하위 제어기와 로봇의 동특성에 따른다(§4.4). 이어서 동작시키기 전에 자세를 확인하거나 named target 으로 복귀시킨다.
- **재사용 가능하다.** `TrajectoryStreamer` 는 동일한 인스턴스로 여러 궤적을 순차 실행할 수 있다. 다만 궤적마다 시작 자세가 현재 자세와 일치해야 한다(§3.5).
- **JTC 환경에서는 쓰지 않는다.** JTC 스택은 스트리밍 명령 토픽을 받지 않으므로 발행해도 아무 일도 일어나지 않는다. `create_move_group_client(node, backend=...)` 를 쓰면 프로파일이 알맞은 구현을 고르므로 직접 고를 일이 없다.
- **도달 여부를 검사하지 않는다.** 스트리머는 관측값에 따른 도달 여부를 검사하지 않으며, 하위 제어기의 피드백 제어 여부와는 별개다(§2.2). 로봇이 명령을 못 따라가도 정상 종료한다.
- **velocity/acceleration 은 명령에 싣지 않는다.** 궤적의 position 만 발행하므로, 보간 없이 그대로 흘리면 계단식 명령이 된다. 발행 밀도는 `publish_rate` 로 정한다(§8.3).

---

## 11. 관련 문서

- [move_group_client_user_guide.md](move_group_client_user_guide.md) — 공통 인터페이스 + JTC 구현 + 팩토리 (JGPC 전용 내용은 이 문서에만 있다)
- [../../src/robot_control/launch/README.md](../../src/robot_control/launch/README.md) — `panda_functionbay.launch.py` 포함 제어 계열 launch 인벤토리
- [../backends/functionbay/펑션베이_백엔드_설계.md](../backends/functionbay/펑션베이_백엔드_설계.md) — 펑션베이 명령 채널과 메시지 형식
- [../collect/](../collect/) — replay 서브시스템 문서
