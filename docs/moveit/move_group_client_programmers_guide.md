# MoveGroupClient — Programmer's Guide

MoveIt2 기반 motion planning(카테시안 경로 + joint-space named target)을 간소화하는 클라이언트 클래스 사용 가이드.

---

## 목차

1. [개요](#개요)
2. [사전 요구사항](#사전-요구사항)
3. [Quick Start](#quick-start)
4. [API 레퍼런스](#api-레퍼런스)
5. [Named Target](#named-target)
6. [Waypoint 작성법](#waypoint-작성법)
7. [파라미터 튜닝 가이드](#파라미터-튜닝-가이드)
8. [비동기 API 사용법](#비동기-api-사용법)
9. [에러 처리](#에러-처리)
10. [Threading 주의사항](#threading-주의사항)
11. [실전 예제](#실전-예제)

---

## 개요

`MoveGroupClient`는 MoveIt2의 여러 인터페이스를 하나로 묶어 두 가지 이동 모드를 제공한다:

| MoveIt2 인터페이스 | 타입 | 역할 |
|---|---|---|
| `/compute_cartesian_path` | Service (`GetCartesianPath`) | Waypoint 목록 → Cartesian trajectory 계획 |
| `/execute_trajectory` | Action (`ExecuteTrajectory`) | 계획된 `RobotTrajectory` 실행 |
| `/move_action` | Action (`MoveGroup`) | SRDF의 `group_state` 로의 joint-space plan + execute |
| `/move_group/get_parameters` | Service (`GetParameters`) | named target 조회용 SRDF 획득 (캐시됨) |

**이동 모드:**

| 모드 | 진입점 | 좌표계 | 사용 예 |
|---|---|---|---|
| Cartesian path | `follow_trajectory`, `plan_trajectory` | Task space (Pose list) | 사각형/지그재그 경로, 직선 이동 |
| Named target | `move_to_named_target` | Joint space (SRDF group_state) | `ready`, `extended`, `transport` 등 미리 등록된 자세 |

**핵심 설계 원칙:**
- **Node 주입**: 자체 Node를 생성하지 않고, 호출자가 만든 Node를 받아 사용한다.
- **Lazy 초기화**: 생성자에서 서버 준비를 기다리지 않는다. 명시적으로 `wait_until_ready()`를 호출해야 한다.
- **SRDF 캐시**: named target 관련 API는 첫 호출 시 `move_group` 노드에서 SRDF를 한 번만 조회하여 캐시한다.
- **Context Manager 지원**: `with` 문으로 리소스를 자동 정리할 수 있다.

---

## 사전 요구사항

### MoveIt2 스택이 실행 중이어야 한다

```bash
# 터미널 1: MoveIt2 스택 실행
ros2 launch rdfp panda_mock.launch.py
```

이 launch가 완료되면 `move_group` 노드가 `/compute_cartesian_path` 서비스와 `/execute_trajectory` 액션 서버를 제공한다.

### 의존 패키지

```python
# 필수
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

# MoveGroupClient
from rdfp.moveit import MoveGroupClient, pose
```

---

## Quick Start

### 가장 간단한 사용법

```python
import rclpy
from rclpy.node import Node
from rdfp.moveit import MoveGroupClient, pose


def main():
    rclpy.init()
    node = Node('my_cartesian_node')

    try:
        with MoveGroupClient(node) as client:
            client.wait_until_ready()

            waypoints = [
                pose(0.4,  0.3, 0.4, 3.14, 0.0, 0.0),
                pose(0.4, -0.3, 0.4, 3.14, 0.0, 0.0),
                pose(0.7, -0.3, 0.4, 3.14, 0.0, 0.0),
            ]

            client.follow_trajectory(waypoints, velocity_scaling=0.5)

    except Exception as exc:
        node.get_logger().error(f'Error: {exc}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

`follow_trajectory()`는 내부적으로 경로 계획 → 실행을 한번에 처리한다.

---

## API 레퍼런스

### 생성자

```python
MoveGroupClient(
    node,                               # 필수: rclpy.node.Node 인스턴스
    *,
    frame_id='panda_link0',             # 기준 좌표계
    moveit_group_name='panda_arm',      # MoveIt 플래닝 그룹
    fraction_threshold=0.60,            # 최소 계획 성공 비율 (0.0~1.0)
    velocity_scaling=1.0,               # 속도 배율 (1.0 = 원속)
    max_step=0.01,                      # Cartesian 보간 간격 (m)
    jump_threshold=5.0,                 # 관절 공간 점프 임계값 (rad)
    cartesian_path_service='/compute_cartesian_path',
    execute_trajectory_action='/execute_trajectory',
    move_group_action='/move_action',   # named target 이동용 MoveGroup 액션
    move_group_node_name='/move_group', # SRDF 조회 대상 노드
)
```

생성자에서 지정한 파라미터는 **인스턴스 기본값**으로 저장된다. 각 메서드 호출 시 override할 수 있다.

### Lifecycle 메서드

| 메서드 | 설명 |
|---|---|
| `is_ready() -> bool` | Cartesian 서비스/액션 서버 준비 여부 즉시 반환 (non-blocking). MoveGroup 액션은 포함되지 않는다. |
| `wait_until_ready(timeout_sec=30.0)` | Cartesian 서비스/액션 서버 준비까지 블로킹 대기. `TimeoutError` 가능 |
| `close()` / `destroy()` | 리소스 정리 (멱등). Node 자체는 파괴하지 않음 |

> **Note:** `wait_until_ready()` 는 MoveGroup 액션 서버는 기다리지 않는다. named target 이동에 필요한 MoveGroup 서버 준비는 `move_to_named_target()` 내부에서 lazy 로 처리된다.

### High-level API — Cartesian 경로 (동기)

| 메서드 | 설명 |
|---|---|
| `follow_trajectory(waypoints, ...)` | 계획 + 실행 원스톱. 단계별 timeout 관리 |
| `plan_trajectory(waypoints, ...) -> RobotTrajectory` | 계획만 수행 |
| `execute_trajectory(trajectory, ...)` | 실행만 수행 |
| `scale_trajectory_velocity(trajectory, scaling) -> RobotTrajectory` | Trajectory 속도 조절 (deep copy 반환) |

### High-level API — Named Target (동기)

| 메서드 | 설명 |
|---|---|
| `move_to_named_target(name, ...)` | SRDF `group_state` 로 joint-space 이동 (plan + execute) |
| `get_named_targets(*, group=None, ...) -> list[str]` | SRDF에 등록된 named target 이름 목록을 정렬해 반환 (빈 리스트 가능) |

상세 사용법은 [Named Target](#named-target) 섹션 참조.

### High-level API (비동기)

| 메서드 | 설명 |
|---|---|
| `follow_trajectory_async(waypoints, ...) -> Future` | Cartesian 계획 + 실행을 콜백 체인으로 수행. 성공 시 `None`으로 resolve |
| `move_to_named_target_async(name, ...) -> Future` | SRDF 조회(필요 시) → MoveGroup goal 전송 → 결과 대기를 콜백 체인으로 수행 |

두 메서드 모두 Node를 직접 spin하지 않으므로 `MultiThreadedExecutor` 환경에서 사용할 수 있다.

### Low-level API (비동기)

| 메서드 | 설명 |
|---|---|
| `plan_trajectory_async(waypoints, ...) -> Future` | Cartesian 계획 서비스 요청, Future 반환 |
| `execute_trajectory_async(trajectory, ...) -> Future` | goal 수락까지 블로킹 대기 후, 실행 결과 Future 반환 |

`execute_trajectory_async`는 이름과 달리 goal 수락 단계에서 `await_future_spin`으로 블로킹 대기한다. 메서드가 반환된 시점에 goal이 수락되어 로봇이 움직이기 시작했음이 보장된다. 반환된 Future는 순수하게 실행 완료 여부만 의미한다.

---

## Named Target

SRDF(semantic robot description)에 등록된 `group_state`를 이용해 joint-space 로 로봇을 이동시킬 수 있다. 이 기능은 **MoveGroup 액션**(`/move_action`)을 통해 plan + execute 를 통합적으로 수행한다. Cartesian 경로와 달리 관절 공간에서 직접 보간하므로 로봇 자세를 빠르고 안정적으로 전환할 수 있다.

### SRDF group_state 예시

Panda 로봇의 경우 `moveit_resources_panda_moveit_config/config/panda.srdf` 에 다음과 같은 named target이 정의되어 있다.

| 그룹 | 이름 | 설명 |
|---|---|---|
| `panda_arm` | `ready` | 기본 준비 자세 |
| `panda_arm` | `extended` | 팔을 뻗은 자세 |
| `panda_arm` | `transport` | 이송 자세 |
| `hand` | `open` | 그리퍼 열림 |
| `hand` | `close` | 그리퍼 닫힘 |

### 등록된 target 조회

```python
with MoveGroupClient(node) as client:
    client.wait_until_ready()

    # 생성자에 지정된 기본 그룹 (panda_arm) 의 모든 named target
    names = client.get_named_targets()
    print(names)  # ['extended', 'ready', 'transport']

    # 다른 그룹 지정
    hand_states = client.get_named_targets(group='hand')
    print(hand_states)  # ['close', 'open']
```

최초 호출 시에만 `/move_group/get_parameters` 서비스로 SRDF를 조회하여 파싱한 뒤 캐시한다. 이후 호출은 즉시 반환된다.

### 기본 이동 (동기)

```python
with MoveGroupClient(node, velocity_scaling=0.3) as client:
    client.wait_until_ready()

    # 기본 동작 — 생성자 velocity_scaling(0.3) 적용
    client.move_to_named_target('ready')

    # 호출별 파라미터 override
    client.move_to_named_target('extended', velocity_scaling=0.5, planning_time=10.0)

    # 그리퍼 이동 (moveit_group_name='hand' 로 새 인스턴스 권장)
    # 또는 그리퍼 전용 클라이언트를 별도로 생성
```

### `move_to_named_target` 파라미터

| 파라미터 | 기본값 | 용도 |
|---|---|---|
| `name` | (필수) | SRDF `group_state` 이름 |
| `velocity_scaling` | 생성자 기본값 | 계획된 trajectory 속도 배율 |
| `planning_time` | `5.0` | MoveGroup 계획 허용 시간(초). 어려운 자세 이동 시 증가 |
| `tolerance` | `1e-4` | 각 관절 목표의 허용 오차(라디안) |
| `timeout` | `120.0` | SRDF 조회 + 계획 + 실행 전체 최대 시간(초) |

### 비동기 이동

```python
from rclpy.executors import MultiThreadedExecutor
from threading import Thread

executor = MultiThreadedExecutor()
executor.add_node(node)
Thread(target=executor.spin, daemon=True).start()

with MoveGroupClient(node) as client:
    client.wait_until_ready()

    future = client.move_to_named_target_async('ready', velocity_scaling=0.3)

    # 다른 작업 병행 가능
    future.add_done_callback(
        lambda f: node.get_logger().info(
            'Reached' if f.exception() is None else f'Failed: {f.exception()}'
        )
    )
```

### 동작 체인

```
move_to_named_target_async(name)
 │
 ├─ (캐시 hit) ─────────────────────────────────┐
 │                                              │
 └─ (캐시 miss) _fetch_srdf_async               │
       └─ _on_srdf_done: SRDF 파싱 → 캐시 저장  ▼
                                          _start_move
                                                │
                                                ▼
                                       send_goal_async
                                                │
                                                ▼
                                     _on_goal_response
                                                │
                                                ▼
                                      _on_execute_done
                                                │
                                                ▼
                                        result_future ← None(성공) / Exception(실패)
```

### 주의사항

- **잘못된 이름**: SRDF에 없는 이름 전달 시 사용 가능한 이름 목록과 함께 `ValueError` 가 발생한다.
- **계획 실패**: 장애물 회피가 불가능하거나 IK 해가 없는 경우 `planning_time` 초과 후 `RuntimeError` (MoveIt error code 포함) 발생.
- **그룹 일치**: `move_to_named_target(name)` 은 `moveit_group_name` 으로 지정된 그룹에 속한 state만 대상으로 한다. 다른 그룹의 state로 이동하려면 해당 그룹을 `moveit_group_name` 으로 지정한 새 클라이언트를 생성한다.
- **is_ready() 무관**: `wait_until_ready()` 는 MoveGroup 액션 서버를 기다리지 않는다. named target 이동의 MoveGroup 준비 대기는 메서드 내부에서 처리된다.

---

## Waypoint 작성법

### `pose()` 헬퍼 사용 (권장)

```python
from rdfp.moveit import pose

# pose(x, y, z, roll, pitch, yaw) — 단위: 미터, 라디안
wp = pose(0.4, 0.3, 0.4, 3.14, 0.0, 0.0)
```

내부적으로 RPY를 quaternion으로 변환하여 `geometry_msgs.msg.Pose`를 생성한다.

### 직접 Pose 생성

```python
from geometry_msgs.msg import Pose

wp = Pose()
wp.position.x = 0.4
wp.position.y = 0.3
wp.position.z = 0.4
wp.orientation.x = 0.0
wp.orientation.y = 0.0
wp.orientation.z = 0.0
wp.orientation.w = 1.0  # 단위 쿼터니언 필수
```

### Waypoint 유효성 검증 규칙

`plan_trajectory()` 호출 시 자동으로 검증된다:

- 최소 1개 이상의 waypoint 필요
- 각 waypoint는 `Pose` 타입이어야 함
- position의 x, y, z가 모두 유한(finite)한 값이어야 함
- quaternion의 x, y, z, w가 모두 유한한 값이어야 함
- quaternion norm ≈ 1.0 (오차 ± 0.01 허용, norm < 0.001이면 거부)

---

## 파라미터 튜닝 가이드

### `velocity_scaling` — 속도 조절

```python
# 생성자 기본값 설정
client = MoveGroupClient(node, velocity_scaling=0.2)  # 전체 20% 속도

# 호출별 override
client.follow_trajectory(waypoints, velocity_scaling=0.5)  # 이 경로만 50%
client.follow_trajectory(waypoints)                         # 기본값 0.2 적용
```

| 값 | 용도 |
|---|---|
| `0.1 ~ 0.2` | 실제 하드웨어 초기 테스트, 안전 우선 |
| `0.5` | 일반 작업 |
| `1.0` | 원래 속도 (시뮬레이션용) |

내부 동작: `time_from_start`를 `1/scaling` 배로 늘리고, velocity를 `scaling` 배, acceleration을 `scaling²` 배로 조절한다.

### `fraction_threshold` — 계획 성공 기준

MoveIt은 요청한 경로를 100% 계획하지 못할 수 있다. 이 값은 허용할 최소 비율이다.

```python
# 60% 이상 계획되면 실행 (기본값)
client = MoveGroupClient(node, fraction_threshold=0.60)

# 95% 이상만 허용 (엄격)
client = MoveGroupClient(node, fraction_threshold=0.95)

# 계획된 만큼만이라도 실행 (위험 — 예상치 못한 경로 가능)
client = MoveGroupClient(node, fraction_threshold=0.0)
```

`fraction < threshold`이면 `RuntimeError`가 발생한다.

### `max_step` — Cartesian 보간 해상도

Waypoint 사이를 보간할 때 각 단계의 최대 간격(미터).

| 값 | 특징 |
|---|---|
| `0.005` | 정밀한 경로 (계획 시간 증가) |
| `0.01` | 기본값, 일반적인 균형점 |
| `0.05` | 빠른 계획 (경로 정밀도 감소) |

### `jump_threshold` — 관절 점프 감지

연속된 두 waypoint 사이에서 관절값이 크게 변하면 비정상 움직임일 가능성이 높다. 이 값(라디안)을 초과하면 MoveIt이 해당 구간에서 계획을 중단한다.

| 값 | 의미 |
|---|---|
| `0.0` | 점프 검사 비활성화 |
| `5.0` | 기본값 |

---

## 비동기 API 사용법

동기 메서드(`follow_trajectory`, `plan_trajectory`, `execute_trajectory`)는 내부적으로 `await_future_spin`(`rclpy.spin_until_future_complete` 기반)으로 블로킹 대기한다. 자체 executor를 사용하거나, 콜백 기반으로 처리하고 싶다면 비동기 API를 사용한다.

### 계획과 실행 분리

```python
# 1단계: 계획 (비동기 요청 → 동기 대기)
trajectory = client.plan_trajectory(waypoints, velocity_scaling=0.3)

# 중간에 trajectory 검사/수정 가능
print(f'Waypoints in trajectory: {len(trajectory.joint_trajectory.points)}')

# 2단계: 실행
client.execute_trajectory(trajectory)
```

### `follow_trajectory_async` — 완전 비동기 실행

`follow_trajectory_async`는 콜백 체인으로 계획과 실행을 연결한다. Node를 직접 spin하지 않으므로 `MultiThreadedExecutor` 환경에서 안전하다.

```python
from threading import Thread
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor()
executor.add_node(node)
spin_thread = Thread(target=executor.spin)
spin_thread.start()

with MoveGroupClient(node) as client:
    client.wait_until_ready()

    future = client.follow_trajectory_async(waypoints, velocity_scaling=0.3)

    # 다른 작업 수행 가능
    node.get_logger().info('Robot is moving, doing other work...')

    # 완료 대기
    rclpy.spin_until_future_complete(node, future)
    exc = future.exception()
    if exc is not None:
        node.get_logger().error(f'Failed: {exc}')

executor.shutdown()
```

**콜백 체인 구조:**

```
plan_trajectory_async()
  └─ _on_plan_done: 응답 검증, velocity scaling, goal 전송
       └─ _on_goal_response: goal 수락 확인, 결과 대기
            └─ _on_execute_done: 실행 결과 확인, result_future 설정
```

**중단 제한사항:**
반환된 `result_future`를 cancel해도 이미 등록된 콜백 체인은 계속 진행된다.
콜백 내부의 `goal_handle`에 외부에서 접근할 수 없어 실행 중인 로봇을 멈출 수 없다.
중단이 필요한 경우 `plan_trajectory` + `execute_trajectory`를 분리하여 각 단계를 직접 제어해야 한다.

### Future 직접 처리

```python
# 경로 계획 Future 획득
future = client.plan_trajectory_async(waypoints)

# 자체 executor에서 처리
rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
response = future.result()

if response.error_code.val == MoveItErrorCodes.SUCCESS:
    trajectory = response.solution
    # 속도 스케일링 적용
    scaled = client.scale_trajectory_velocity(trajectory, 0.3)
    client.execute_trajectory(scaled)
```

---

## 에러 처리

### 예외 계층

```
ValueError          — 잘못된 파라미터 (즉시 발생)
TimeoutError        — 서버 응답/실행 시간 초과
RuntimeError        — 계획 실패, goal 거부, 실행 실패, close 후 호출
```

`follow_trajectory_async`의 경우 예외가 반환된 `Future`에 설정된다. `future.exception()`으로 확인한다.

### 실전 에러 처리 패턴

```python
with MoveGroupClient(node, velocity_scaling=0.2) as client:
    try:
        client.wait_until_ready(timeout_sec=30.0)
    except TimeoutError:
        node.get_logger().error('MoveIt2 stack is not running')
        return

    for i, waypoints in enumerate(trajectory_list):
        try:
            client.follow_trajectory(waypoints)
            node.get_logger().info(f'Trajectory {i} completed')
        except RuntimeError as e:
            node.get_logger().warning(f'Trajectory {i} failed: {e}, skipping')
            continue
        except TimeoutError as e:
            node.get_logger().error(f'Trajectory {i} timed out: {e}, aborting')
            break
```

### 주요 실패 시나리오

| 증상 | 원인 | 해결 |
|---|---|---|
| `wait_until_ready()` timeout | MoveIt2 스택 미실행 | `ros2 launch rdfp panda_mock.launch.py` 확인 |
| `fraction` 부족으로 `RuntimeError` | Waypoint가 도달 불가능한 위치 | 작업 영역 내로 waypoint 수정 또는 `fraction_threshold` 하향 |
| `execute_trajectory` timeout | 로봇이 움직이다 멈춤 | timeout 증가 또는 trajectory 길이 확인 |
| `Goal rejected` | Controller 미준비 | Controller spawner 완료 여부 확인 |

---

## Threading 주의사항

동기 메서드는 내부적으로 `await_future_spin`(`rclpy.spin_until_future_complete` 기반)을 호출한다. 이로 인해 **주입된 Node를 직접 spin**한다.

### 안전한 사용 패턴

```python
# 메인 스레드에서만 호출 — 가장 안전
rclpy.init()
node = Node('my_node')
with MoveGroupClient(node) as client:
    client.wait_until_ready()
    client.follow_trajectory(waypoints)
node.destroy_node()
rclpy.shutdown()
```

### 위험한 패턴

```python
# 이중 spin — 데드락/경합 발생
executor = MultiThreadedExecutor()
executor.add_node(node)
thread = Thread(target=executor.spin)
thread.start()

# 이 상태에서 동기 메서드 호출하면 동일 Node를 두 곳에서 spin
client.follow_trajectory(waypoints)  # 위험!
```

### MultiThreadedExecutor 환경에서의 대안

다른 스레드에서 Node를 이미 spin 중이라면 `follow_trajectory_async`를 사용한다. 이 메서드는 콜백 체인 기반으로 Node를 직접 spin하지 않으므로 이중 spin 문제가 없다.

```python
executor = MultiThreadedExecutor()
executor.add_node(node)
thread = Thread(target=executor.spin)
thread.start()

# follow_trajectory_async는 Node를 spin하지 않으므로 안전
future = client.follow_trajectory_async(waypoints, velocity_scaling=0.3)
future.add_done_callback(lambda f: node.get_logger().info('Done!'))
```

Low-level 비동기 API(`plan_trajectory_async`)도 Future를 반환하므로 자체 executor에서 처리할 수 있다. 단, `execute_trajectory_async`는 goal 수락 단계에서 `await_future_spin`을 사용하므로 이중 spin 환경에서는 주의가 필요하다.

---

## 실전 예제

### 사각형 경로 + 속도 제어

```python
from rdfp.moveit import MoveGroupClient, pose

with MoveGroupClient(node, velocity_scaling=0.2) as client:
    client.wait_until_ready()

    square = [
        pose(0.4,  0.3, 0.4, 3.14, 0.0, 0.0),
        pose(0.4, -0.3, 0.4, 3.14, 0.0, 0.0),
        pose(0.7, -0.3, 0.4, 3.14, 0.0, 0.0),
        pose(0.7,  0.3, 0.4, 3.14, 0.0, 0.0),
        pose(0.4,  0.3, 0.4, 3.14, 0.0, 0.0),  # 시작점 복귀
    ]
    client.follow_trajectory(square)
```

### 계획/실행 분리 + Trajectory 재사용

```python
with MoveGroupClient(node) as client:
    client.wait_until_ready()

    waypoints = [
        pose(0.5, 0.0, 0.4, 3.14, 0.0, 0.0),
        pose(0.5, 0.0, 0.6, 3.14, 0.0, 0.0),
    ]

    # 한 번 계획
    trajectory = client.plan_trajectory(waypoints)

    # 다른 속도로 여러 번 실행
    slow = client.scale_trajectory_velocity(trajectory, 0.1)
    client.execute_trajectory(slow)

    fast = client.scale_trajectory_velocity(trajectory, 0.8)
    client.execute_trajectory(fast)
```

### 여러 경로 순차 실행

```python
with MoveGroupClient(node, velocity_scaling=0.3) as client:
    client.wait_until_ready()

    paths = [
        [pose(0.4, 0.3, 0.4, 3.14, 0.0, 0.0),
         pose(0.4, -0.3, 0.4, 3.14, 0.0, 0.0)],

        [pose(0.3, 0.0, 0.6, 3.14, 0.0, 0.0),
         pose(0.6, 0.0, 0.3, 3.14, 0.0, 0.0)],
    ]

    for i, path in enumerate(paths):
        try:
            node.get_logger().info(f'Executing path {i}...')
            client.follow_trajectory(path)
        except (RuntimeError, TimeoutError) as e:
            node.get_logger().warning(f'Path {i} failed: {e}')
```

---

## 기본 Timeout 상수 및 기본값

| 상수 | 기본값 | 용도 |
|---|---|---|
| `READY_TIMEOUT_SEC` | 30초 | `wait_until_ready()`, `get_named_targets()` SRDF 조회 |
| `PLAN_TIMEOUT_SEC` | 20초 | `plan_trajectory()` |
| `GOAL_ACCEPT_TIMEOUT_SEC` | 10초 | `execute_trajectory_async()` goal 수락 대기 |
| `TRAJECTORY_EXEC_TIMEOUT_SEC` | 120초 | `execute_trajectory()` 실행 완료 대기 |
| `MOVE_GROUP_TIMEOUT_SEC` | 120초 | `move_to_named_target()` SRDF 조회 + 계획 + 실행 전체 |
| `DEFAULT_PLANNING_TIME` | 5.0초 | `move_to_named_target()` MoveGroup 계획 허용 시간 |
| `DEFAULT_JOINT_TOLERANCE` | `1e-4` rad | `move_to_named_target()` 각 관절 목표 허용 오차 |

`follow_trajectory()`의 기본 timeout은 `PLAN_TIMEOUT_SEC + TRAJECTORY_EXEC_TIMEOUT_SEC` = **140초**이다.
`follow_trajectory_async()` 및 `move_to_named_target_async()`에는 timeout 파라미터가 없다. 콜백 체인이 각 단계의 응답을 기다리며, 외부에서 timeout을 관리해야 한다.
