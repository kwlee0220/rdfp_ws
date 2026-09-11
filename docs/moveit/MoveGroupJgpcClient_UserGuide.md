# Named Target 이동 (JGPC / 스트리밍 실행) — 사용 가이드

`FollowJointTrajectory` 액션을 제공하지 않는 컨트롤러 환경에서 SRDF named target
으로 이동하는 방법. `panda_jgpc_mock.launch.py` 와 함께 사용한다.

---

## 목차

1. [왜 필요한가](#왜-필요한가)
2. [동작 원리](#동작-원리)
3. [Quick Start](#quick-start)
4. [API 레퍼런스](#api-레퍼런스)
5. [비동기 사용법](#비동기-사용법)
6. [TrajectoryStreamer 직접 사용](#trajectorystreamer-직접-사용)
7. [replay_gui 연동](#replay_gui-연동)
8. [파라미터 튜닝](#파라미터-튜닝)
9. [에러 처리](#에러-처리)
10. [주의사항](#주의사항)
11. [관련 문서](#관련-문서)

---

## 왜 필요한가

`panda_jgpc_mock.launch.py` 는 arm 컨트롤러를 `joint_trajectory_controller/JointTrajectoryController` (JTC) 대신 `position_controllers/JointGroupPositionController` (JGPC) 로 띄운다. JGPC 는 `forward_command_controller` 기반이라 **`FollowJointTrajectory` 액션을 제공하지 않는다**.

그런데 MoveIt 의 `moveit_simple_controller_manager` 는 `FollowJointTrajectory` 와 `GripperCommand` 두 타입만 다룰 수 있다. 그래서 기존 API 는 이렇게 갈린다.

| API 부류 | JTC 환경 | JGPC 환경 | 이유 |
|---|:-:|:-:|---|
| SRDF 조회 (`get_*`) | ✅ | ✅ | `move_group` 파라미터 조회 — 컨트롤러 무관 |
| 계획 전용 (`plan_*`) | ✅ | ✅ | Cartesian 서비스 / `MoveGroup` **plan_only** — 실행 안 함 |
| 계획 + 실행 (`move_to_*`, `follow_trajectory`) | ✅ | ✅ | **구현이 갈린다** — JTC 는 액션, JGPC 는 스트리밍 |
| `execute_trajectory()` | ✅ | — | `MoveGroupJtcClient` 에만 있음 |
| `stream_trajectory()` / `*_streamed()` | — | ✅ | `MoveGroupJgpcClient` 에만 있음 |

메서드 단위 전체 대조표는 [MoveGroupClient_UserGuide.md](MoveGroupClient_UserGuide.md) 의 「구현별 API 가용성」에 있다.

즉 **계획은 원래 잘 된다. 잃은 건 실행 경로 하나뿐**이다.

그래서 클라이언트를 계획(공통) + 실행(구현별)으로 나눴다. `MoveGroupClient` 는 추상 공통 인터페이스이고, 실행은 `MoveGroupJtcClient` / `MoveGroupJgpcClient` 가 각자 구현한다. `create_move_group_client(node)` 가 명령 토픽 존재 여부로 컨트롤러를 판별해 알맞은 쪽을 돌려주므로, 호출부는 어느 스택인지 몰라도 된다.

---

## 동작 원리

JTC 는 궤적을 통째로 받아 **컨트롤러 내부에서 시간 보간**을 한다. JGPC 는 받은 값을 그대로 hardware 에 write 할 뿐이므로, 그 시간 보간을 클라이언트가 대신해야 한다.

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

### joint 순서 문제

`Float64MultiArray` 에는 **joint 이름이 없다**. 배열 순서가 컨트롤러의 `joints` 파라미터 순서와 어긋나면 엉뚱한 관절이 움직인다.

`TrajectoryStreamer` 는 이를 방지하기 위해 컨트롤러 노드의 `get_parameters` 서비스에서 `joints` 파라미터를 직접 읽어 순서를 확정하고, 궤적의 `joint_names` 를 그 순서로 재배열한다. 사용자가 신경 쓸 필요는 없지만, 직접 발행할 때는 반드시 같은 재배열을 해야 한다.

---

## Quick Start

### 스택 실행

```bash
ros2 launch robot_control panda_jgpc_mock.launch.py
```

### 가장 간단한 사용법 (동기)

```python
import rclpy
from rclpy.node import Node

from robot_control.moveit import create_move_group_client

rclpy.init()
node = Node('named_target_demo')

# 컨트롤러를 판별해 JGPC 스택에서는 MoveGroupJgpcClient 를 돌려준다.
with create_move_group_client(node) as mgc:
    print(mgc.get_named_targets())            # ['extended', 'ready', 'transport']
    mgc.move_to_named_target('ready')         # JGPC 구현 = 계획 + 스트리밍

node.destroy_node()
rclpy.shutdown()
```

`MoveGroupJgpcClient.move_to_named_target()` 은 계획 + 스트리밍을 한 번에 수행하며 궤적 길이만큼 블로킹된다. 발행한 명령 개수가 필요하면 같은 뜻의 `move_to_named_target_streamed()` 를 직접 호출한다.

### 구현을 명시적으로 고르기

`mode='auto'` 판별은 토픽 그래프 조회라 **DDS discovery 가 끝나기 전에 호출하면 JGPC 스택을 JTC 로 오판할 수 있다.** 어느 스택인지 이미 알고 있다면 `mode` 를 못박는다.

```python
mgc = create_move_group_client(node, mode='jgpc')   # -> MoveGroupJgpcClient
mgc = create_move_group_client(node, mode='jtc')    # -> MoveGroupJtcClient
mgc = create_move_group_client(node)                # 'auto' (기본)
```

판별 규칙과 `detect_controller_mode()` 는 [MoveGroupClient_UserGuide.md](MoveGroupClient_UserGuide.md)의 「구현 선택」에 있다.

### SRDF 에 없는 자세로 — joint 목표값

`move_to_joints()` 는 SRDF 조회 단계만 없을 뿐 같은 경로를 탄다.

```python
mgc.move_to_joints({'panda_joint1': 0.5, 'panda_joint4': -2.0}, publish_rate=200.0)
```

### 계획과 실행을 나눠서

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

## API 레퍼런스

### `MoveGroupJgpcClient` 생성자 추가 인자

`create_move_group_client()` 가 그대로 전달한다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `arm_command_topic` | `/panda_arm_controller/commands` | 스트리밍 대상 명령 토픽. `mode='auto'` 판별에도 쓰인다 |
| `arm_command_joint_names` | `None` | 명령 배열 joint 순서. `None` 이면 컨트롤러에서 자동 조회 |

명령 퍼블리셔는 **lazy 생성**된다. 스트리밍을 한 번도 하지 않으면 토픽이 생기지 않는다. JTC 환경에서는 `MoveGroupJtcClient` 가 생성되므로 이 인자들이 아예 쓰이지 않는다.

### 스트리밍 실행 메서드

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

### 공통 API 의 JGPC 구현

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

### 중단 — `cancel()`

`cancel()` 은 두 구현 모두에 있지만 정지 특성이 다르다.

| 구현 | 수단 | 정지 특성 |
|---|---|---|
| JTC | 컨트롤러 `FollowJointTrajectory` 액션에 `CancelGoal` | 컨트롤러가 감속 후 정지 |
| **JGPC** | 스트리밍 중단(`stop_streaming()`) + 계획 단계 MoveGroup goal 취소 | 마지막으로 보낸 명령 위치에서 정지 — **감속 프로파일 없음** |

컨트롤러가 마지막 명령 위치를 그대로 유지하므로, JGPC 에서 `cancel()` 은 사실상 급정지다. 반환값 `True` 는 중단 대상이 있었다는 뜻이며, 정지 후 로봇은 경로 중간의 불확정 자세에 있다.

### 계획 메서드 (컨트롤러 무관, base 제공)

JGPC 환경에서 실행 전에 궤적을 얻을 때 쓴다. 세 메서드 모두 JTC 환경에서도 동일하다.

| 메서드 | 반환 | 비고 |
|---|---|---|
| `plan_named_target(name, ...)` (+ `_async`) | `RobotTrajectory` | `MoveGroup` **plan_only** — 실행 안 함 |
| `plan_joints(joint_values, ...)` (+ `_async`) | `RobotTrajectory` | 위와 같되 SRDF 조회 없음 |
| `plan_trajectory(wps, ...)` (+ `_async`) | `RobotTrajectory` | `/compute_cartesian_path` 서비스 |

### 스트리밍 전용 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `time_scaling` | `1.0` | 재생 시간 배율. `2.0` = 두 배 느리게 |
| `publish_rate` | `None` | 발행 주파수(Hz). `None` 이면 궤적 point 시각에만 발행(≈10Hz). 값을 주면 그 주기로 위치를 선형 보간 |
| `externally_spun` | `False` | Node 가 다른 스레드에서 이미 spin 중이면 `True` |
| `timeout` | `120.0` | **여기서는 SRDF 조회 + 계획까지만** 제한한다. 스트리밍 시간은 궤적 길이가 결정하므로 포함되지 않는다 |

### 계획 파라미터

`velocity_scaling`, `planning_time`, `tolerance`, `fraction_threshold`, `max_step`, `jump_threshold` 는 **두 구현이 동일하다** (계획은 컨트롤러와 무관하다). [MoveGroupClient_UserGuide.md](MoveGroupClient_UserGuide.md) 의 「API 레퍼런스」·「파라미터 튜닝 가이드」를 본다.

---

## 비동기 사용법

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

### `externally_spun` 을 언제 `True` 로 두는가

명령 발행 자체는 spin 이 필요 없지만, **joint 순서 자동 조회**는 서비스 호출이라 spin 이 필요하다. 이때 누가 spin 할지가 갈린다.

| 상황 | 값 | 이유 |
|---|:-:|---|
| 아무도 spin 하지 않음 (일반 스크립트) | `False` | 라이브러리가 직접 `spin_until_future_complete` |
| `MultiThreadedExecutor` 등이 백그라운드 spin 중 | `True` | 직접 spin 하면 "node already spinning" 오류 |

`arm_command_joint_names` 를 생성자에서 명시하면 조회 자체가 생략되므로 이 값은 무의미해진다.

---

## TrajectoryStreamer 직접 사용

MoveGroup 클라이언트 없이 임의의 궤적을 스트리밍하려면 `TrajectoryStreamer` 를 직접 쓴다. Cartesian 계획 결과(`plan_trajectory()`)도 그대로 넣을 수 있다.

```python
from robot_control.moveit.trajectory_streamer import TrajectoryStreamer

with TrajectoryStreamer(node) as streamer:
    print(streamer.resolve_joint_names())   # 컨트롤러에서 순서 조회
    streamer.stream(trajectory)                        # 궤적 point 시각에만 발행
    streamer.stream(trajectory, publish_rate=200.0)    # 200Hz 로 보간 발행
```

### 생성자

```python
TrajectoryStreamer(
    node,
    command_topic='/panda_arm_controller/commands',
    joint_names=None,            # None → 컨트롤러에서 자동 조회
    controller_node_name=None,   # None → command_topic 에서 유도
    qos_depth=10,
)
```

### 중단

```python
stop_event = threading.Event()
future = streamer.stream_async(trajectory, stop_event=stop_event)
...
stop_event.set()          # 또는 streamer.stop()
print(future.result())    # 중단 전까지 발행한 point 수
```

---

## replay_gui 연동

`replay_gui` 의 **"위치 초기화"** 버튼은 두 컨트롤러 환경을 모두 지원한다. 판별은 **노드 생성 시 한 번만** 일어나며(`create_move_group_client()`), 이후 호출부는 분기 없이 `move_to_named_target_async()` 하나만 부른다. `init_exec_mode` 파라미터로 판별을 강제할 수 있다.

| 값 | 동작 |
|---|---|
| `auto` (기본) | `/panda_arm_controller/commands` 토픽 존재 여부로 자동 판별 |
| `action` | `MoveGroupJtcClient` 강제 — JTC 환경 (`replay_panda_mock.launch.py`) |
| `stream` | `MoveGroupJgpcClient` 강제 — JGPC 환경 (`panda_jgpc_mock.launch.py`) |

```bash
# 자동 판별 (기본)
ros2 run rdfp replay_gui --config dataset_config.yaml

# 명시적으로 지정
ros2 run rdfp replay_gui --config dataset_config.yaml \
    --ros-args -p init_exec_mode:=stream
```

이동 대상은 기존과 동일하게 `init_named_target` 파라미터(기본 `ready`)로 바꾼다.

---

## 파라미터 튜닝

### `tolerance` — JGPC 에서 더 중요한 이유

`tolerance` 자체는 계획 파라미터라 두 구현이 같다 (기본 `1e-4` rad, 설명은 [프로그래머 가이드](MoveGroupClient_UserGuide.md) 참고).

다만 **JGPC 에서는 이 값이 곧 최종 자세를 결정한다.** JTC 는 컨트롤러가 `FollowJointTrajectory` 의 goal tolerance 를 따로 검사하고 미달이면 실패를 보고하지만, 스트리밍 경로에는 그런 재검사가 없다. 마지막으로 발행한 point 가 그대로 최종 명령이므로, 플래너가 tolerance 안에서 멈춘 위치가 곧 로봇이 서는 위치다.

```python
mgc.move_to_named_target('ready')                  # 기본 1e-4 rad
mgc.move_to_named_target('ready', tolerance=0.01)  # 최대 0.01 rad 벗어난 곳에 정지
```

### `time_scaling` vs `velocity_scaling`

둘 다 느리게 만들지만 작용 지점이 다르다.

| 파라미터 | 작용 시점 | 효과 |
|---|---|---|
| `velocity_scaling` | 계획 | 궤적 자체를 느리게 재생성 (가감속 프로파일 반영) |
| `time_scaling` | 재생 | 계획된 궤적을 그대로 두고 재생 속도만 늘림 |

새 자세를 처음 시험할 때는 `time_scaling=2.0` 으로 한 번 확인한 뒤 `1.0` 으로 돌리는 것을 권장한다.

### `publish_rate` — 명령 밀도

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

## 에러 처리

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

### 자주 보는 MoveIt 오류 코드

| 코드 | 이름 | 흔한 원인 |
|---|---|---|
| `-1` | `PLANNING_FAILED` | 경로를 못 찾음 |
| `-12` | `GOAL_IN_COLLISION` | 목표 자세가 충돌 |
| `-27` | `GOAL_STATE_INVALID` | 목표 자세가 자기충돌 / 한계 위반 |

> **참고**: 기본 panda SRDF 의 `transport` 자세는 이 스택에서 `-27` 로 실패한다
> (`panda_joint4 = -2.97` 로 접힌 자세가 자기충돌). JGPC 와 무관한 SRDF 자체의
> 성질이며 `ready` / `extended` 는 정상 동작한다.

---

## 주의사항

- **스트리밍 중 다른 명령 소스와 겹치면 안 된다.** servo 가 같은 `/panda_arm_controller/commands` 토픽에 발행 중이면 두 스트림이 뒤섞인다. 스트리밍 전에 servo 를 정지시키거나, 명령 시점을 분리한다.

- **첫 point 는 계획 시점의 현재 자세**다. 계획과 스트리밍 사이에 로봇이 움직였다면 첫 발행에서 급격한 이동이 생긴다. `plan_named_target()` 과 `stream_trajectory()` 사이를 길게 벌리지 않는다.

- **스트리밍은 블로킹이다.** `move_to_named_target()` 은 궤적 길이만큼 호출 스레드를 잡는다. `timeout` 파라미터는 계획 단계까지만 적용된다.

- **`cancel()` 은 급정지다.** 스트리밍을 멈추면 컨트롤러가 마지막으로 받은 명령 위치를 그대로 유지하므로 감속 프로파일이 없다. 정지 후 로봇은 경로 중간의 불확정 자세에 있으므로, 이어서 동작시키기 전에 자세를 확인하거나 named target 으로 복귀시킨다.

- **재사용 가능하다.** `TrajectoryStreamer` 는 replay 계열 replayer 와 달리
  one-shot 이 아니다. 같은 인스턴스로 여러 번 `stream()` 할 수 있다.

- **JTC 환경에서는 쓰지 않는다.** JTC 는 `/panda_arm_controller/commands` 를 제공하지 않으므로 발행해도 아무 일도 일어나지 않는다. `create_move_group_client()` 를 쓰면 컨트롤러를 판별해 알맞은 구현을 돌려주므로 직접 고를 일이 없다.

- **open loop 다.** 명령을 발행할 뿐 도달 여부를 확인하지 않는다. 반환값은 "발행한 명령 수"이지 "도달했는지"가 아니다. 로봇이 명령을 못 따라가도 정상 종료한다.

- **velocity/acceleration 은 버려진다.** 궤적의 position 만 발행한다. MoveIt 은 시간 파라미터화 단계에서 궤적을 0.1초 격자(=10Hz)로 리샘플하므로, 보간 없이 그대로 흘리면 실기에서 계단식 명령이 된다. `publish_rate` 인자로 발행 주파수를 올려 위치를 선형 보간할 수 있다 (예: `publish_rate=200.0`). `max_step` 을 줄여도 point 밀도는 늘지 않는다 — 리샘플이 그 뒤에 일어나기 때문이다.

---

## 관련 문서

- [MoveGroupClient_UserGuide.md](MoveGroupClient_UserGuide.md) — MoveGroup 클라이언트 전체 API (base + JTC/JGPC + 팩토리)
- [../../src/robot_control/launch/README.md](../../src/robot_control/launch/README.md) — `panda_jgpc_mock.launch.py` 포함 제어 계열 launch 인벤토리
- [../replay/](../replay/) — replay 서브시스템 문서
