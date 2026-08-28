# MoveGroupClient — Programmer's Guide

MoveIt2 기반 motion planning(카테시안 경로 + joint-space named target)을 간소화하는 클라이언트 클래스 사용 가이드.

---

## 목차

1. [개요](#1-개요)
2. [사전 요구사항](#2-사전-요구사항)
3. [Quick Start](#3-quick-start)
4. [API 레퍼런스](#4-api-레퍼런스)
   - [4.1 생성자](#41-생성자)
   - [4.2 Lifecycle 메서드](#42-lifecycle-메서드)
   - [4.3 Cartesian 경로 (동기)](#43-high-level-api--cartesian-경로-동기)
   - [4.4 Named Target (동기)](#44-high-level-api--named-target-동기)
   - [4.5 Joint 목표값 (동기)](#45-high-level-api--joint-목표값-동기)
   - [4.6 계획 전용 API](#46-계획-전용-api-컨트롤러-무관)
   - [4.7 High-level 비동기](#47-high-level-api-비동기)
   - [4.8 Low-level 비동기](#48-low-level-api-비동기)
   - [4.9 실행 제어](#49-실행-제어)
   - [4.10 JGPC 전용 API](#410-jgpc-전용-api)
5. [Named Target](#5-named-target)
6. [Joint 목표값 이동](#6-joint-목표값-이동)
7. [계획 전용 API](#7-계획-전용-api)
8. [Waypoint 작성법](#8-waypoint-작성법)
9. [파라미터 튜닝 가이드](#9-파라미터-튜닝-가이드)
10. [비동기 API 사용법](#10-비동기-api-사용법)
11. [동작 중단](#11-동작-중단)
12. [JGPC command 스트리밍](#12-jgpc-command-스트리밍)
13. [에러 처리](#13-에러-처리)
14. [Threading 주의사항](#14-threading-주의사항)
15. [실전 예제](#15-실전-예제)
16. [기본 Timeout 상수 및 기본값](#16-기본-timeout-상수-및-기본값)

---

## 1. 개요

`MoveGroupClient` 계열은 MoveIt2의 여러 인터페이스를 하나로 묶어 두 가지 이동 모드를 제공한다. **계획은 공통이고 실행만 컨트롤러별로 갈리므로**, `MoveGroupClient` 는 추상 공통 인터페이스이고 실행은 두 서브클래스가 구현한다.

| 클래스 | 역할 |
|---|---|
| `MoveGroupClient` | 계획 + SRDF 조회 (추상, 직접 생성 불가) |
| `MoveGroupJtcClient` | `JointTrajectoryController` 환경 — 액션으로 실행 |
| `MoveGroupJgpcClient` | forward command 컨트롤러 환경 — command 스트리밍으로 실행 |

`create_move_group_client(node)` 가 컨트롤러를 판별해 알맞은 구현을 돌려주므로, 호출부는 어느 스택인지 몰라도 된다.

> **이 문서의 예제는 `MoveGroupJtcClient` (JTC 스택) 기준이다.** 계획 API 와 공통 실행 API(`move_to_named_target` / `move_to_joints` / `follow_trajectory`)는 두 구현에서 똑같이 동작하지만, `execute_trajectory()` 를 쓰는 예제는 JGPC 스택에서 `AttributeError` 가 난다 (그 클래스에 없는 메서드다). JGPC 전용 API 와 스트리밍 실행의 상세는 [MoveGroupJgpcClient_UserGuide.md](MoveGroupJgpcClient_UserGuide.md)를 본다.

### 1.1 구현 선택

명시적으로 구현을 고르려면 `mode` 를 준다.

```python
client = create_move_group_client(node)                 # 자동 판별 (기본)
client = create_move_group_client(node, mode='jtc')     # -> MoveGroupJtcClient
client = create_move_group_client(node, mode='jgpc')    # -> MoveGroupJgpcClient
```

`mode='auto'` 판별은 `/panda_arm_controller/commands` 토픽 존재 여부를 보는 **토픽 그래프 조회**다. 따라서 **DDS discovery 가 끝나기 전에 호출하면 JGPC 스택을 JTC 로 오판할 수 있다.** 어느 스택인지 이미 알고 있다면 `mode` 를 못박는 편이 낫다.

판별 결과만 따로 확인하려면:

```python
from rdfp.moveit import detect_controller_mode

print(detect_controller_mode(node))   # 'jgpc' 또는 'jtc'
```

`detect_controller_mode(node, *, arm_command_topic='/panda_arm_controller/commands')` —
`arm_command_topic` 은 판별에 쓸 명령 토픽 이름이며, 컨트롤러 이름을 바꾼 스택에서만 건드린다.
토픽 그래프만 조회하므로 서비스 호출도 대기도 없다.

### 1.2 구현별 API 가용성

계획은 컨트롤러와 무관하므로 두 구현에서 모두 동작한다. **갈리는 것은 실행 경로뿐이다.**

| API | JTC | JGPC | 이유 |
|---|:-:|:-:|---|
| `get_named_targets()` / `get_all_named_targets()` / `get_planning_groups()` | ✅ | ✅ | SRDF 파라미터 조회 — 컨트롤러 무관 |
| `plan_trajectory()` / `plan_trajectory_async()` | ✅ | ✅ | `/compute_cartesian_path` 서비스 — 컨트롤러 무관 |
| `plan_named_target()` / `plan_joints()` (+ `_async`) | ✅ | ✅ | `MoveGroup` 액션 **plan_only** — 실행 안 함 |
| `move_to_named_target()` / `move_to_joints()` / `follow_trajectory()` (+ `_async`) | ✅ | ✅ | **구현이 갈린다** — JTC 는 액션, JGPC 는 스트리밍 |
| `scale_trajectory_velocity()` | ✅ | ✅ | 메시지 가공 — ROS 통신 없음 |
| `cancel()` / `has_active_goal()` | ✅ | ✅ | **구현이 갈린다** — JTC 는 goal 취소, JGPC 는 스트리밍 중단 |
| `execute_trajectory()` / `execute_trajectory_async()` | ✅ | — | `MoveGroupJtcClient` 에만 있음 |
| `stream_trajectory()` / `stop_streaming()` / `*_streamed()` | — | ✅ | `MoveGroupJgpcClient` 에만 있음 |

JGPC 스택에서 `move_to_*` / `follow_trajectory` 는 **open loop** 다 — 정상 반환이 목표 도달을 뜻하지 않는다. 상세는 [MoveGroupJgpcClient_UserGuide.md](MoveGroupJgpcClient_UserGuide.md).

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
| Named target | `move_to_named_target`, `plan_named_target` | Joint space (SRDF group_state) | `ready`, `extended`, `transport` 등 미리 등록된 자세 |
| Joint 목표값 | `move_to_joints`, `plan_joints` | Joint space (`{관절이름: 라디안}`) | SRDF에 없는 임의 자세, 계산으로 구한 관절값 |

세 모드 모두 **계획만 하는 `plan_*`** 과 **계획 + 실행을 묶은 `move_to_*` / `follow_*`**가 짝을 이룬다. `plan_*` 은 컨트롤러 종류와 무관하게 동작하므로 JGPC 환경에서도 쓸 수 있다 ([계획 전용 API](#7-계획-전용-api) 참조).

**핵심 설계 원칙:**
- **Node 주입**: 자체 Node를 생성하지 않고, 호출자가 만든 Node를 받아 사용한다.
- **Lazy 초기화**: 생성자에서 서버 준비를 기다리지 않는다. 명시적으로 `wait_until_ready()`를 호출해야 한다.
- **SRDF 캐시**: named target 관련 API는 첫 호출 시 `move_group` 노드에서 SRDF를 한 번만 조회하여 캐시한다.
- **Context Manager 지원**: `with` 문으로 리소스를 자동 정리할 수 있다.
- **중단 가능**: 진행 중인 동작은 `cancel()` 로 중단한다 ([동작 중단](#11-동작-중단) 참조).

---

## 2. 사전 요구사항

### 2.1 MoveIt2 스택이 실행 중이어야 한다

```bash
# 터미널 1: MoveIt2 스택 실행
ros2 launch robot_control panda_mock.launch.py
```

이 launch가 완료되면 `move_group` 노드가 `/compute_cartesian_path` 서비스와 `/execute_trajectory` 액션 서버를 제공한다.

### 2.2 의존 패키지

```python
# 필수
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

# MoveGroupClient
from rdfp.moveit import create_move_group_client, pose
```

---

## 3. Quick Start

### 3.1 가장 간단한 사용법

```python
import rclpy
from rclpy.node import Node
from rdfp.moveit import create_move_group_client, pose


def main():
    rclpy.init()
    node = Node('my_cartesian_node')

    try:
        with create_move_group_client(node) as client:
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

## 4. API 레퍼런스

### 4.1 생성자

```python
create_move_group_client(
    node,                               # 필수: rclpy.node.Node 인스턴스
    *,
    mode='auto',                        # 'auto' | 'jtc' | 'jgpc'
    arm_command_topic='/panda_arm_controller/commands',   # JGPC 명령 토픽 (auto 판별에도 사용)
    arm_command_joint_names=None,       # JGPC 명령 배열 joint 순서 (None = 자동 조회)
    # --- 아래는 공통 base 생성자 인자 ---
    frame_id='panda_link0',             # 기준 좌표계
    moveit_group_name='panda_arm',      # MoveIt 플래닝 그룹
    fraction_threshold=0.60,            # 최소 계획 성공 비율 (0.0~1.0)
    velocity_scaling=1.0,               # 속도 배율 (1.0 = 원속)
    max_step=0.01,                      # Cartesian 보간 간격 (m)
    jump_threshold=5.0,                 # 관절 공간 점프 임계값 (rad)
    cartesian_path_service='/compute_cartesian_path',
    move_group_action='/move_action',   # named target 계획/이동용 MoveGroup 액션
    move_group_node_name='/move_group', # SRDF 조회 대상 노드
    # --- JTC 전용 ---
    execute_trajectory_action='/execute_trajectory',
)
```

생성자에서 지정한 파라미터는 **인스턴스 기본값**으로 저장된다. 다만 **호출별로 override 할 수 있는 것은 4개뿐이다.**

| 구분 | 파라미터 | 비고 |
|---|---|---|
| 호출별 override 가능 | `velocity_scaling`, `fraction_threshold`, `max_step`, `jump_threshold` | 메서드 인자로 같은 이름을 주면 그 호출에만 적용된다. `None` 이면 생성자 값 |
| 생성자 고정 | `frame_id`, `moveit_group_name` | 다른 좌표계/그룹이 필요하면 **새 인스턴스**를 만든다 (예: 그리퍼용 `moveit_group_name='hand'`) |
| 생성자 고정 (배선) | `mode`, `arm_command_topic`, `arm_command_joint_names`, `cartesian_path_service`, `move_group_action`, `move_group_node_name`, `execute_trajectory_action` | 접속할 서비스/액션/토픽 이름. 기본 Panda 스택이면 건드릴 일이 없다 |

`planning_time` / `tolerance` / `timeout` 은 생성자에 없다 — **메서드 인자로만** 준다.

### 4.2 Lifecycle 메서드

| 메서드 | 설명 |
|---|---|
| `is_ready() -> bool` | 준비 여부 즉시 반환 (non-blocking). base 는 Cartesian 서비스 + MoveGroup 액션, JTC 는 여기에 `ExecuteTrajectory` 액션을 더한다 |
| `wait_until_ready(timeout_sec=30.0)` | 위 서버들이 준비될 때까지 블로킹 대기. `TimeoutError` 가능 |
| `close()` / `destroy()` | 리소스 정리 (멱등). Node 자체는 파괴하지 않음 |
| `__enter__()` / `__exit__()` | Context manager. `with` 블록을 벗어날 때 `close()` 를 호출한다 |

> **Note:** 기다리는 대상이 구현마다 다르다. JGPC 구현은 `ExecuteTrajectory` 액션을 쓰지 않으므로 그 서버를 기다리지 않는다.

인자를 받는 것은 `wait_until_ready` 하나뿐이다.

| 파라미터 | 기본값 | 용도 |
|---|---|---|
| `timeout_sec` | `30.0` | 모든 서버를 합쳐 기다릴 **총** 시간(초). 서버를 순차로 기다리며 남은 예산을 이어 쓴다. 0 이하면 `ValueError` |

### 4.3 High-level API — Cartesian 경로 (동기)

| 메서드 | 설명 |
|---|---|
| `follow_trajectory(waypoints, ...)` | 계획 + 실행 원스톱. 단계별 timeout 관리 |
| `plan_trajectory(waypoints, ...) -> RobotTrajectory` | 계획만 수행 |
| `execute_trajectory(trajectory, ...)` | 실행만 수행. **`MoveGroupJtcClient` 전용** — JGPC 구현에는 없고 `stream_trajectory()` 가 대응한다 |
| `scale_trajectory_velocity(trajectory, velocity_scaling) -> RobotTrajectory` | Trajectory 속도 조절 (deep copy 반환) |

#### 4.3.1 `follow_trajectory` / `plan_trajectory` 파라미터

두 메서드는 `waypoints` 를 제외한 나머지가 **모두 키워드 전용(keyword-only)** 이며 목록이 같다.
`timeout` 의 기본값과 의미만 다르다.

| 파라미터 | 기본값 | 용도 |
|---|---|---|
| `waypoints` | (필수) | 통과할 Cartesian `Pose` 목록. 위치·쿼터니언이 모두 유한하고 norm ≈ 1 이어야 한다 ([검증 규칙](#83-waypoint-유효성-검증-규칙)) |
| `velocity_scaling` | 생성자 기본값 (`1.0`) | 계획된 trajectory 의 속도 배율. `None` 이면 생성자 값 |
| `fraction_threshold` | 생성자 기본값 (`0.60`) | 계획 성공으로 인정할 최소 경로 비율. 미달이면 `RuntimeError` |
| `max_step` | 생성자 기본값 (`0.01`) | Cartesian 보간 최대 단계 간격(미터) |
| `jump_threshold` | 생성자 기본값 (`5.0`) | 관절 공간 점프 임계값(라디안). `0.0` 이면 검사 비활성 |
| `timeout` | `follow_trajectory` → `140.0`<br>`plan_trajectory` → `20.0` | 허용 최대 시간(초). 아래 주석 참고 |

`None` 을 넘기는 것과 인자를 생략하는 것은 같다 — 둘 다 생성자 기본값으로 해석된다.
값별 선택 기준은 [파라미터 튜닝 가이드](#9-파라미터-튜닝-가이드) 를 본다.

`timeout` 의 범위는 구현마다 다르다.

| 메서드 / 구현 | 기본값 | 포함 단계 |
|---|---|---|
| `plan_trajectory` | `20.0` | `GetCartesianPath` 서비스 응답만 |
| `follow_trajectory` (JTC) | `140.0` | 계획 + 실행 완료까지 전체 |
| `follow_trajectory` (JGPC) | `140.0` | **계획만**. 스트리밍 시간은 궤적 길이가 정하므로 포함되지 않는다 |

> 추상 base 선언에는 `timeout=120.0` 으로 적혀 있으나 본문이 없는 `@abc.abstractmethod` 이므로
> 실제로 적용되는 값이 아니다. 두 구현 모두 `140.0`(`PLAN_TIMEOUT_SEC + TRAJECTORY_EXEC_TIMEOUT_SEC`) 이다.

#### 4.3.2 `execute_trajectory` / `execute_trajectory_async` 파라미터

**`MoveGroupJtcClient` 전용**이다. JGPC 는 `stream_trajectory()` 를 쓴다.

| 파라미터 | 기본값 | 용도 |
|---|---|---|
| `trajectory` | (필수) | 실행할 `RobotTrajectory`. `plan_*` 이 반환한 것을 그대로 넣는다 |
| `timeout` | 동기 → `120.0`<br>비동기 → `10.0` | 동기는 **실행 완료까지**, 비동기는 **goal 수락까지**의 허용 시간(초) |

비동기 판의 `timeout` 이 짧은 것은 오타가 아니다 — `execute_trajectory_async` 는 goal 수락까지만
블로킹 대기하고, 실행 완료는 반환된 `Future` 가 담는다.

#### 4.3.3 `scale_trajectory_velocity` 파라미터

유일하게 **키워드 전용이 아닌** 메서드다. 둘 다 위치 인자로 넘긴다.

| 파라미터 | 기본값 | 용도 |
|---|---|---|
| `trajectory` | (필수) | 원본 `RobotTrajectory`. 입력은 변경되지 않는다 (deep copy 반환) |
| `velocity_scaling` | (필수) | 양수 배율. `1.0` = 원속, `0.5` = 절반 속도 |

### 4.4 High-level API — Named Target (동기)

| 메서드 | 설명 |
|---|---|
| `move_to_named_target(name, ...)` | SRDF `group_state` 로 joint-space 이동 (plan + execute) |
| `get_named_targets(*, group=None, ...) -> list[str]` | **한 그룹**의 named target 이름 목록을 정렬해 반환 (빈 리스트 가능). `group=None` 은 "전체"가 아니라 생성자의 기본 그룹 |
| `get_all_named_targets(...) -> dict[str, list[str]]` | **모든** planning 그룹의 named target 을 `{group: [name, ...]}` 로 반환. 키는 `group_state` 를 가진 그룹만 |
| `get_planning_groups(...) -> list[str]` | SRDF 의 `<group>` 정의를 읽어 planning group 이름 전체를 반환 (named target 이 없는 그룹 포함) |

#### 4.4.1 SRDF 조회 3종의 파라미터

세 메서드 모두 인자가 **키워드 전용**이며 목록이 거의 같다. `group` 만 `get_named_targets`
전용이다.

| 파라미터 | 기본값 | 용도 |
|---|---|---|
| `group` | `None` | 조회할 planning 그룹 이름. **`None` 은 "전체"가 아니라 생성자의 `moveit_group_name`(기본 `panda_arm`)** 이다. `get_named_targets` 에만 있다 |
| `timeout` | `30.0` | SRDF 조회(`/move_group/get_parameters`) 허용 시간(초). **캐시가 이미 차 있으면 무시된다** |
| `externally_spun` | `False` | executor 가 다른 스레드에서 Node 를 spin 중이면 `True`. 자체 spin 대신 콜백 완료를 기다리는 방식(`await_future_spin_nospin`)으로 바꿔 **이중 spin** 을 피한다 |

세 메서드는 SRDF 캐시를 공유하므로, 어느 하나를 먼저 호출했다면 나머지는 `timeout` 과 무관하게
즉시 반환된다.

`move_to_named_target` 의 인자는 [`move_to_named_target` 파라미터](#54-move_to_named_target-파라미터),
상세 사용법은 [Named Target](#5-named-target) 섹션 참조.

### 4.5 High-level API — Joint 목표값 (동기)

| 메서드 | 설명 |
|---|---|
| `move_to_joints(joint_values, ...)` | `{관절이름: 라디안}` 목표로 joint-space 이동 (plan + execute). SRDF 조회가 없다는 점만 빼면 `move_to_named_target` 과 같은 경로 |

`joint_values` 에는 **planning group 에 속한 관절만** 넣는다 — `panda_arm` 클라이언트에
finger joint 를 섞으면 계획이 실패한다. 인자는
[`move_to_joints` 파라미터](#61-move_to_joints-파라미터), 상세는
[Joint 목표값 이동](#6-joint-목표값-이동) 참조.

### 4.6 계획 전용 API (컨트롤러 무관)

| 메서드 | 설명 |
|---|---|
| `plan_named_target(name, ...) -> RobotTrajectory` | named target 궤적을 `plan_only=True` 로 **계획만** 수행 |
| `plan_joints(joint_values, ...) -> RobotTrajectory` | joint 목표값 궤적을 **계획만** 수행 (SRDF 조회 없음) |
| `plan_trajectory(waypoints, ...) -> RobotTrajectory` | Cartesian 궤적을 계획만 수행 |

`plan_only=True` 라 `move_group` 이 실행을 시도하지 않으므로, `FollowJointTrajectory`
액션이 없는 컨트롤러 환경에서도 동작한다. 상세는 [계획 전용 API](#7-계획-전용-api) 참조.

### 4.7 High-level API (비동기)

| 메서드 | 설명 |
|---|---|
| `follow_trajectory_async(waypoints, ...) -> Future` | Cartesian 계획 + 실행을 콜백 체인으로 수행 |
| `move_to_named_target_async(name, ...) -> Future` | SRDF 조회(필요 시) → MoveGroup goal 전송 → 결과 대기를 콜백 체인으로 수행 |
| `move_to_joints_async(joint_values, ...) -> Future` | `move_to_joints` 의 비동기 버전 |

세 메서드 모두 Node를 직접 spin하지 않으므로 `MultiThreadedExecutor` 환경에서 사용할 수 있다.
JTC 구현은 성공 시 `None` 으로, JGPC 구현은 **발행한 point 개수**로 resolve 된다.

**인자는 동기 판과 같되 두 가지가 다르다.**

| 차이 | 내용 |
|---|---|
| `timeout` **없음** | 콜백 체인이 각 단계의 응답을 기다린다. 시간 제한이 필요하면 호출자가 `Future` 를 감시한다 |
| `externally_spun` **추가** | 기본 `False`. JTC 구현에서는 무시된다(콜백 체인만 쓰므로 동작이 같다). JGPC 구현에서는 명령 배열 joint 순서 자동 조회 방식에 영향을 준다 — 다른 스레드에서 spin 중이면 `True` 로 준다 |

즉 `follow_trajectory_async(waypoints, *, velocity_scaling, fraction_threshold, max_step,
jump_threshold, externally_spun)`, `move_to_named_target_async(name, *, velocity_scaling,
planning_time, tolerance, externally_spun)`, `move_to_joints_async(joint_values, *, ...)` 이며
각 인자의 의미와 기본값은 동기 판의 표와 동일하다.

### 4.8 Low-level API (비동기)

| 메서드 | 설명 |
|---|---|
| `plan_trajectory_async(waypoints, ...) -> Future` | Cartesian 계획 서비스 요청, Future 반환 |
| `plan_named_target_async(name, ...) -> Future` | SRDF 조회 → MoveGroup `plan_only` goal → 결과 콜백. `RobotTrajectory` 로 resolve |
| `plan_joints_async(joint_values, ...) -> Future` | 위와 같되 SRDF 조회 단계가 없음 |
| `execute_trajectory_async(trajectory, ...) -> Future` | goal 수락까지 블로킹 대기 후, 실행 결과 Future 반환. **`MoveGroupJtcClient` 전용** |

`execute_trajectory_async`는 이름과 달리 goal 수락 단계에서 `await_future_spin`으로 블로킹 대기한다. 메서드가 반환된 시점에 goal이 수락되어 로봇이 움직이기 시작했음이 보장된다. 반환된 Future는 순수하게 실행 완료 여부만 의미한다.

`plan_named_target_async` / `plan_joints_async` 는 Node 를 직접 spin 하지 않으므로 executor 가 다른 스레드에서 spin 중인 환경에서도 안전하다.

#### 4.8.1 Low-level 비동기 API 의 파라미터

| 메서드 | 시그니처 | 비고 |
|---|---|---|
| `plan_trajectory_async` | `(waypoints, *, max_step=None, jump_threshold=None)` | **`velocity_scaling` / `fraction_threshold` / `timeout` 을 받지 않는다** |
| `plan_named_target_async` | `(name, *, velocity_scaling=None, planning_time=5.0, tolerance=1e-4)` | `timeout` / `externally_spun` 없음 |
| `plan_joints_async` | `(joint_values, *, velocity_scaling=None, planning_time=5.0, tolerance=1e-4)` | 위와 같되 SRDF 조회 단계가 없음 |
| `execute_trajectory_async` | `(trajectory, *, timeout=10.0)` | `timeout` 은 **goal 수락까지**. JTC 전용 |

> **`plan_trajectory_async` 는 다른 `plan_*` 과 성격이 다르다.** 반환 `Future` 는
> `RobotTrajectory` 가 아니라 **`GetCartesianPath` 서비스 응답 원본**으로 resolve 되며,
> `fraction` 검사도 속도 스케일링도 적용되지 않는다. 계획 결과를 쓰려면 호출자가
> `response.error_code` / `response.fraction` 을 직접 확인하고 `response.solution` 을
> 꺼낸 뒤 필요하면 `scale_trajectory_velocity()` 로 스케일해야 한다. 동기 판
> `plan_trajectory()` 는 이 세 단계를 대신 해 준다.

### 4.9 실행 제어

| 메서드 | 설명 |
|---|---|
| `cancel() -> bool` | 진행 중인 동작을 중단한다. **비동기** — `True` 는 취소 요청을 보냈다는 뜻이지 로봇이 이미 멈췄다는 뜻이 아니다 |
| `has_active_goal() -> bool` | 취소할 수 있는 진행 중 goal 이 있는지 반환 |

두 메서드 모두 **인자를 받지 않는다.** `cancel()` 은 이 클라이언트가 시작한 goal 만 골라 취소하는 것이 아니므로(상세는 아래) 대상 지정 인자도 없다.

상세는 [동작 중단](#11-동작-중단) 참조.

### 4.10 JGPC 전용 API

`MoveGroupJgpcClient` 에만 있는 메서드다. JTC 클라이언트에서 호출하면 `AttributeError` 가 난다.
`stream_trajectory()` 가 JTC 의 `execute_trajectory()` 에 대응하고, `*_streamed` 계열은 공통 이름 메서드와 같은 동작에 **발행한 point 개수를 반환**하는 판이다.

`stream_trajectory` / `stop_streaming` / `follow_trajectory_streamed` /
`move_to_named_target_streamed` / `move_to_joints_streamed` (+ 뒤 둘의 `_async`) —
시그니처·파라미터·주의사항은
[MoveGroupJgpcClient_UserGuide.md](MoveGroupJgpcClient_UserGuide.md) 의 「API 레퍼런스」를 본다.

JGPC 구현은 공통 이름 메서드(`move_to_named_target` / `move_to_joints` / `follow_trajectory` 와
각 `_async`)에서도 base 시그니처에 없는 `time_scaling` / `publish_rate` 를 추가로 받는다.

#### 4.10.1 JGPC 가 추가로 받는 파라미터

| 파라미터 | 기본값 | 용도 |
|---|---|---|
| `time_scaling` | `1.0` | **재생 시간** 배율. `2.0` 이면 두 배 느리게 재생한다. 계획 결과를 바꾸는 `velocity_scaling` 과 달리, 이미 계획된 궤적의 발행 시각만 늘린다 |
| `publish_rate` | `None` | 명령 발행 주기(Hz). `None` 이면 궤적 point 의 원래 시각을 그대로 따른다 — MoveIt Cartesian 궤적은 항상 ~10 Hz 로 resample 되므로 **명령이 10 Hz 로 계단처럼 나간다**. `200.0` 처럼 주면 균일 격자에 선형 보간해 발행한다 |

`publish_rate` 를 왜 거의 항상 지정해야 하는지는 아래 [JGPC command 스트리밍](#12-jgpc-command-스트리밍)
절과 [MoveGroupJgpcClient_UserGuide.md](MoveGroupJgpcClient_UserGuide.md) 를 본다.

`stream_trajectory(trajectory, *, time_scaling=1.0, publish_rate=None, externally_spun=False)` 는
위 두 인자에 `externally_spun`(joint 순서 자동 조회 시에만 의미 있음)을 더 받는다.
`stop_streaming()` 은 인자가 없다.

---

## 5. Named Target

SRDF(semantic robot description)에 등록된 `group_state`를 이용해 joint-space 로 로봇을 이동시킬 수 있다. 이 기능은 **MoveGroup 액션**(`/move_action`)을 통해 plan + execute 를 통합적으로 수행한다. Cartesian 경로와 달리 관절 공간에서 직접 보간하므로 로봇 자세를 빠르고 안정적으로 전환할 수 있다.

> **JGPC (JointGroupPositionController) 환경 주의**: 위 설명은 `MoveGroupJtcClient` 기준이다. `panda_jgpc_mock.launch.py` 처럼 forward command 컨트롤러를 쓰는 스택에서는 `create_move_group_client()` 가 `MoveGroupJgpcClient` 를 돌려주며, 같은 `move_to_named_target()` 이 계획 + command 스트리밍으로 동작한다. **단 open loop 라 정상 반환이 도달을 뜻하지 않는다** — [MoveGroupJgpcClient_UserGuide.md](MoveGroupJgpcClient_UserGuide.md) 참고.

### 5.1 SRDF group_state 예시

Panda 로봇의 경우 `moveit_resources_panda_moveit_config/config/panda.srdf` 에 다음과 같은 named target이 정의되어 있다.

| 그룹 | 이름 | 설명 |
|---|---|---|
| `panda_arm` | `ready` | 기본 준비 자세 |
| `panda_arm` | `extended` | 팔을 뻗은 자세 |
| `panda_arm` | `transport` | 이송 자세 |
| `hand` | `open` | 그리퍼 열림 |
| `hand` | `close` | 그리퍼 닫힘 |

### 5.2 등록된 target 조회

```python
with create_move_group_client(node) as client:
    client.wait_until_ready()

    # 생성자에 지정된 기본 그룹 (panda_arm) 의 모든 named target
    names = client.get_named_targets()
    print(names)  # ['extended', 'ready', 'transport']

    # 다른 그룹 지정
    hand_states = client.get_named_targets(group='hand')
    print(hand_states)  # ['close', 'open']

    # 모든 그룹을 한 번에 — 그룹 이름을 미리 알 필요가 없다
    print(client.get_all_named_targets())
    # {'hand': ['close', 'open'], 'panda_arm': ['extended', 'ready', 'transport']}

    # SRDF 에 정의된 planning group 전체
    print(client.get_planning_groups())
    # ['hand', 'panda_arm', 'panda_arm_hand']
```

`get_all_named_targets()` 의 키는 `group_state` 를 하나 이상 가진 그룹만 포함한다.
위 예의 `panda_arm_hand` 처럼 group_state 가 없는 그룹은 `get_planning_groups()` 로만 보인다.

최초 호출 시에만 `/move_group/get_parameters` 서비스로 SRDF를 조회하여 파싱한 뒤 캐시한다. 이후 호출은 즉시 반환된다.

### 5.3 기본 이동 (동기)

```python
with create_move_group_client(node, velocity_scaling=0.3) as client:
    client.wait_until_ready()

    # 기본 동작 — 생성자 velocity_scaling(0.3) 적용
    client.move_to_named_target('ready')

    # 호출별 파라미터 override
    client.move_to_named_target('extended', velocity_scaling=0.5, planning_time=10.0)

    # 그리퍼 이동 (moveit_group_name='hand' 로 새 인스턴스 권장)
    # 또는 그리퍼 전용 클라이언트를 별도로 생성
```

### 5.4 `move_to_named_target` 파라미터

`name` 을 제외한 나머지는 **키워드 전용**이다.

| 파라미터 | 기본값 | 용도 |
|---|---|---|
| `name` | (필수) | SRDF `group_state` 이름 |
| `velocity_scaling` | 생성자 기본값 | 계획된 trajectory 속도 배율. `None` 이면 생성자 값 |
| `planning_time` | `5.0` | MoveGroup 계획 허용 시간(초). 어려운 자세 이동 시 증가 |
| `tolerance` | `1e-4` | 각 관절 목표의 허용 오차(라디안). 플래너가 이 오차 안에서 멈출 수 있으므로 정밀도가 필요하면 작게 준다 |
| `timeout` | `120.0` | SRDF 조회 + 계획 + 실행 전체 최대 시간(초) |

JGPC 구현은 여기에 [`time_scaling` / `publish_rate`](#4101-jgpc-가-추가로-받는-파라미터) 를 더 받으며,
`timeout` 은 스트리밍 시간을 포함하지 않는다.

### 5.5 비동기 이동

```python
from rclpy.executors import MultiThreadedExecutor
from threading import Thread

executor = MultiThreadedExecutor()
executor.add_node(node)
Thread(target=executor.spin, daemon=True).start()

with create_move_group_client(node) as client:
    client.wait_until_ready()

    future = client.move_to_named_target_async('ready', velocity_scaling=0.3)

    # 다른 작업 병행 가능
    future.add_done_callback(
        lambda f: node.get_logger().info(
            'Reached' if f.exception() is None else f'Failed: {f.exception()}'
        )
    )
```

### 5.6 동작 체인

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

### 5.7 주의사항

- **잘못된 이름**: SRDF에 없는 이름 전달 시 사용 가능한 이름 목록과 함께 `ValueError` 가 발생한다.
- **계획 실패**: 장애물 회피가 불가능하거나 IK 해가 없는 경우 `planning_time` 초과 후 `RuntimeError` (MoveIt error code 포함) 발생.
- **그룹 일치**: `move_to_named_target(name)` 은 `moveit_group_name` 으로 지정된 그룹에 속한 state만 대상으로 한다. 다른 그룹의 state로 이동하려면 해당 그룹을 `moveit_group_name` 으로 지정한 새 클라이언트를 생성한다.
- **is_ready() 무관**: `wait_until_ready()` 는 MoveGroup 액션 서버를 기다리지 않는다. named target 이동의 MoveGroup 준비 대기는 메서드 내부에서 처리된다.

---

## 6. Joint 목표값 이동

SRDF에 등록되지 않은 자세로 이동하려면 관절값을 직접 준다. `move_to_named_target` 과
**같은 MoveGroup 액션 경로**를 쓰며, SRDF 조회 단계만 없다.

```python
with create_move_group_client(node, velocity_scaling=0.3) as client:
    client.wait_until_ready()

    client.move_to_joints({
        'panda_joint1': 0.0,
        'panda_joint2': -0.785,
        'panda_joint3': 0.0,
        'panda_joint4': -2.356,
        'panda_joint5': 0.0,
        'panda_joint6': 1.571,
        'panda_joint7': 0.785,
    })
```

### 6.1 `move_to_joints` 파라미터

`joint_values` 를 제외한 나머지는 **키워드 전용**이다.

| 파라미터 | 기본값 | 용도 |
|---|---|---|
| `joint_values` | (필수) | `{관절이름: 라디안}`. planning group 에 속한 관절만. **일부만 줘도 되며, 나머지는 현재값으로 고정된다**(아래) |
| `velocity_scaling` | 생성자 기본값 | 계획된 trajectory 속도 배율. `None` 이면 생성자 값 |
| `planning_time` | `5.0` | MoveGroup 계획 허용 시간(초) |
| `tolerance` | `1e-4` | 각 관절 목표의 허용 오차(라디안) |
| `timeout` | `120.0` | 계획 + 실행 전체 최대 시간(초) |

JGPC 구현은 여기에 [`time_scaling` / `publish_rate`](#4101-jgpc-가-추가로-받는-파라미터) 를 더 받는다.

**지정하지 않은 관절은 현재값으로 고정된다.** 그래서 축 하나만 움직이려면 그 축만
주면 된다.

```python
client.move_to_joints({'panda_joint1': 0.5})   # 1번 축만 움직인다
```

내부적으로는 SRDF 의 `<group_state>` 에서 그룹 관절 목록을 얻고(`group_joint_names()`),
`/joint_states` 에서 현재값을 읽어 **전 관절에 제약을 건다.** 그룹 밖 관절
(`panda_finger_*`)은 채우지 않는다 — 섞이면 계획이 실패한다.

> **왜 채우는가.** 지정한 관절에만 제약을 걸면 목표가 자세 하나가 아니라 "그 조건을
> 만족하는 자세의 **집합**"이 되고, 플래너가 그중 아무거나 고른다. `panda_joint1` 만
> 준 호출이 나머지 6축을 최대 3.5 rad 움직여 엔드이펙터가 로봇 뒤쪽 위로 넘어간 것이
> 실측된다 — **호출 전에 결과를 알 수 없다.**
>
> 그룹에 `<group_state>` 가 하나도 없는 SRDF 면 관절 목록을 알 수 없어
> `RuntimeError` 로 실패한다. 그때는 전 관절을 명시한다.

### 6.2 주의사항

- **그룹 밖 관절 금지**: `moveit_group_name='panda_arm'` 클라이언트에 finger joint 를 섞으면
  계획이 실패한다. 그리퍼는 별도 클라이언트(`moveit_group_name='hand'`)를 쓴다.
- **부분 지정 가능**: 그룹의 모든 관절을 넣을 필요는 없다. 지정하지 않은 관절은 플래너가 정한다.
- **값 검증**: 비어 있거나 유한하지 않은 값(`NaN`/`inf`)이면 `ValueError` 가 즉시 발생한다.
- **도달 보장은 구현별**: JTC 는 컨트롤러가 목표까지 추종하므로 정상 반환이 도달을 뜻하지만, JGPC 는 open loop 라 명령을 다 발행했다는 뜻일 뿐이다.

### 6.3 비동기 버전

```python
future = client.move_to_joints_async(joint_values, velocity_scaling=0.3)
future.add_done_callback(
    lambda f: node.get_logger().info(
        'Reached' if f.exception() is None else f'Failed: {f.exception()}'
    )
)
```

---

## 7. 계획 전용 API

`plan_named_target` / `plan_joints` / `plan_trajectory` 는 궤적을 **계획만** 하여
`RobotTrajectory` 를 돌려준다. 앞의 두 개는 MoveGroup 액션에 `plan_only=True` 로 요청하므로
`move_group` 이 실행을 시도하지 않는다. 따라서 `FollowJointTrajectory` 액션이 없는 컨트롤러
(`position_controllers/JointGroupPositionController` 등) 환경에서도 정상 동작한다.

용도는 세 가지다.

1. **계획과 실행 분리** — 궤적을 검사·수정하거나 여러 속도로 재사용한다.
2. **JGPC 실행** — 반환된 궤적을 `stream_trajectory()` 로 발행한다.
3. **Dry run** — 로봇을 움직이지 않고 계획 가능 여부만 확인한다.

```python
# named target 궤적을 계획만 한다
trajectory = client.plan_named_target('ready', velocity_scaling=0.3)
print(f'points: {len(trajectory.joint_trajectory.points)}')

# joint 목표값 판 — SRDF 조회가 없어 그만큼 단계가 짧다
trajectory = client.plan_joints({'panda_joint1': 0.5}, planning_time=10.0)
```

### 7.1 `plan_named_target` / `plan_joints` 파라미터

`move_to_*` 와 인자가 같다 — 실행을 하지 않을 뿐 계획 요청 자체는 동일하기 때문이다.
`waypoints` 를 받는 `plan_trajectory` 는 [Cartesian 경로 파라미터](#431-follow_trajectory--plan_trajectory-파라미터) 를 본다.

| 파라미터 | 기본값 | 용도 |
|---|---|---|
| `name` / `joint_values` | (필수) | 각각 SRDF `group_state` 이름 / `{관절이름: 라디안}` |
| `velocity_scaling` | 생성자 기본값 (`1.0`) | 반환될 궤적의 속도 배율 |
| `planning_time` | `5.0` | MoveGroup 계획 허용 시간(초). 어려운 자세면 늘린다 |
| `tolerance` | `1e-4` | 각 관절 목표의 허용 오차(라디안). 플래너가 이 오차 안에서 멈출 수 있으므로 정밀도가 필요하면 작게 준다 |
| `timeout` | `120.0` | 아래 표의 단계 전체에 허용할 최대 시간(초) |

| 메서드 | timeout 기본값 | 포함 단계 |
|---|---|---|
| `plan_named_target(name, ...)` | `120.0` | SRDF 조회 + goal 수락 + 계획 |
| `plan_joints(joint_values, ...)` | `120.0` | goal 수락 + 계획 |
| `plan_trajectory(waypoints, ...)` | `20.0` | `GetCartesianPath` 서비스 응답 |

`velocity_scaling` 은 세 메서드 모두 계획 결과에 적용되어 반환된다. 즉 반환된 궤적은 이미 스케일된 상태이며, 나중에 `scale_trajectory_velocity()` 로 다시 조절할 수 있다.

### 7.2 비동기 버전

```python
plan_future = client.plan_named_target_async('ready', velocity_scaling=0.3)

def _on_plan(f):
    exc = f.exception()
    if exc is not None:
        node.get_logger().error(f'Plan failed: {exc}')
        return
    client.stream_trajectory(f.result())   # JGPC

plan_future.add_done_callback(_on_plan)
```

`plan_named_target_async` / `plan_joints_async` 는 Node 를 직접 spin 하지 않으므로 executor 가 다른 스레드에서 spin 중인 환경에서 안전하다. 두 메서드에는 `timeout` 파라미터가 없다 — 콜백 체인이 각 단계의 응답을 기다리며, 외부에서 timeout 을 관리한다.

---

## 8. Waypoint 작성법

### 8.1 `pose()` 헬퍼 사용 (권장)

```python
from rdfp.moveit import pose

# pose(x, y, z, roll, pitch, yaw) — 단위: 미터, 라디안
wp = pose(0.4, 0.3, 0.4, 3.14, 0.0, 0.0)
```

내부적으로 RPY를 quaternion으로 변환하여 `geometry_msgs.msg.Pose`를 생성한다.

### 8.2 직접 Pose 생성

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

### 8.3 Waypoint 유효성 검증 규칙

`plan_trajectory()` 호출 시 자동으로 검증된다:

- 최소 1개 이상의 waypoint 필요
- 각 waypoint는 `Pose` 타입이어야 함
- position의 x, y, z가 모두 유한(finite)한 값이어야 함
- quaternion의 x, y, z, w가 모두 유한한 값이어야 함
- quaternion norm ≈ 1.0 (오차 ± 0.01 허용, norm < 0.001이면 거부)

---

## 9. 파라미터 튜닝 가이드

### 9.1 `velocity_scaling` — 속도 조절

```python
# 생성자 기본값 설정
client = create_move_group_client(node, velocity_scaling=0.2)  # 전체 20% 속도

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

### 9.2 `fraction_threshold` — 계획 성공 기준

MoveIt은 요청한 경로를 100% 계획하지 못할 수 있다. 이 값은 허용할 최소 비율이다.

```python
# 60% 이상 계획되면 실행 (기본값)
client = create_move_group_client(node, fraction_threshold=0.60)

# 95% 이상만 허용 (엄격)
client = create_move_group_client(node, fraction_threshold=0.95)

# 계획된 만큼만이라도 실행 (위험 — 예상치 못한 경로 가능)
client = create_move_group_client(node, fraction_threshold=0.0)
```

`fraction < threshold`이면 `RuntimeError`가 발생한다.

### 9.3 `max_step` — Cartesian 보간 해상도

Waypoint 사이를 보간할 때 각 단계의 최대 간격(미터).

| 값 | 특징 |
|---|---|
| `0.005` | 정밀한 경로 (계획 시간 증가) |
| `0.01` | 기본값, 일반적인 균형점 |
| `0.05` | 빠른 계획 (경로 정밀도 감소) |

### 9.4 `jump_threshold` — 관절 점프 감지

연속된 두 waypoint 사이에서 관절값이 크게 변하면 비정상 움직임일 가능성이 높다. 이 값(라디안)을 초과하면 MoveIt이 해당 구간에서 계획을 중단한다.

| 값 | 의미 |
|---|---|
| `0.0` | 점프 검사 비활성화 |
| `5.0` | 기본값 |

---

## 10. 비동기 API 사용법

동기 메서드(`follow_trajectory`, `plan_trajectory`, `execute_trajectory`, `move_to_named_target`, `move_to_joints`, `plan_named_target`, `plan_joints`)는 내부적으로 `await_future_spin`(`rclpy.spin_until_future_complete` 기반)으로 블로킹 대기한다. 자체 executor를 사용하거나, 콜백 기반으로 처리하고 싶다면 비동기 API를 사용한다.

### 10.1 계획과 실행 분리

> `execute_trajectory()` 는 **JTC 전용**이다. JGPC 에서는 2단계를
> `client.stream_trajectory(trajectory)` 로 바꾼다
> ([JGPC command 스트리밍](#12-jgpc-command-스트리밍) 참조).
> joint-space 목표는 `plan_named_target()` / `plan_joints()` 로 같은 패턴을 쓴다.

```python
# 1단계: 계획 (비동기 요청 → 동기 대기)
trajectory = client.plan_trajectory(waypoints, velocity_scaling=0.3)

# 중간에 trajectory 검사/수정 가능
print(f'Waypoints in trajectory: {len(trajectory.joint_trajectory.points)}')

# 2단계: 실행
client.execute_trajectory(trajectory)
```

### 10.2 `follow_trajectory_async` — 완전 비동기 실행

`follow_trajectory_async`는 콜백 체인으로 계획과 실행을 연결한다. Node를 직접 spin하지 않으므로 `MultiThreadedExecutor` 환경에서 안전하다.

```python
from threading import Thread
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor()
executor.add_node(node)
spin_thread = Thread(target=executor.spin)
spin_thread.start()

with create_move_group_client(node) as client:
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

**중단:**
반환된 `result_future`를 cancel해도 이미 등록된 콜백 체인은 계속 진행된다 — `Future.cancel()` 은
로봇을 멈추지 못한다. 실행 중인 동작을 멈추려면 클라이언트의 `cancel()` 을 호출한다
([동작 중단](#11-동작-중단) 참조).

### 10.3 Future 직접 처리

> 마지막 줄의 `execute_trajectory()` 는 **JTC 전용**이다.

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

## 11. 동작 중단

진행 중인 동작은 `cancel()` 로 멈춘다. 동기/비동기 어느 API로 시작했든 동작한다.

```python
from threading import Thread

# 다른 스레드에서 이동 시작
future = client.move_to_named_target_async('extended')

# 조건이 맞으면 중단
if emergency:
    if client.cancel():
        node.get_logger().warning('Cancel requested')
```

| 메서드 | 반환 | 의미 |
|---|---|---|
| `cancel()` | `bool` | 취소를 보낼 대상이 있었으면 `True`, 진행 중인 동작이 없으면 `False` |
| `has_active_goal()` | `bool` | 취소할 수 있는 진행 중 goal 이 있는지 |

### 11.1 주의사항

- **비동기다.** `True` 는 **취소 요청을 보냈다**는 뜻이지 로봇이 이미 멈췄다는 뜻이 아니다.
  감속 정지에는 시간이 걸리며, 정지 후 로봇은 경로 중간의 **불확정 자세**에 있다.
  이어서 동작을 시킬 때는 현재 자세를 확인하거나 named target 으로 복귀시킨다.
- **`Future.cancel()` 과 다르다.** 비동기 API가 돌려준 `Future` 를 cancel 해도 콜백 체인은
  계속 진행되고 로봇도 멈추지 않는다.

### 11.2 구현별 중단 수단

| 구현 | 중단 방법 | 정지 특성 |
|---|---|---|
| `MoveGroupJtcClient` | 컨트롤러의 `FollowJointTrajectory` 액션에 `CancelGoal` 전송 + 추적 중인 MoveGroup/ExecuteTrajectory goal 취소 | 컨트롤러가 감속 후 정지 |
| `MoveGroupJgpcClient` | 명령 스트리밍 중단(`stop_streaming()`) + 계획 단계의 MoveGroup goal 취소 | 마지막으로 보낸 명령 위치에서 정지 — **감속 프로파일 없음** |

JTC 구현이 컨트롤러를 직접 끊는 이유: `move_group` 이 goal 을 실행 중일 때 `CancelGoal` 요청에 응답하지 않는 것으로 관측되었다(mock/Gazebo 무관). 궤적을 실제로 실행하는 주체는 컨트롤러이므로 거기서 끊는 것이 확실하다.

> **부작용**: JTC 구현의 취소 요청은 `goal_id` 와 `stamp` 를 채우지 않는다. ROS 2 action 규격상
> 이는 **해당 서버의 모든 goal 취소**를 뜻하므로, 이 클라이언트가 시작하지 않은 궤적도 함께
> 취소된다. 한 팔을 여러 주체가 동시에 지휘하지 않는다는 전제 위에서 안전한 동작이다.

---

## 12. JGPC command 스트리밍

`MoveGroupJgpcClient` 는 계획은 MoveIt 에 맡기고 **실행만 직접** 한다. 계획된 궤적의 각 point 를 `time_from_start` 시각에 맞춰 `std_msgs/Float64MultiArray` 명령 토픽 (기본 `/panda_arm_controller/commands`)으로 발행한다. 즉 `JointTrajectoryController` 가 컨트롤러 내부에서 하던 시간 보간을 클라이언트가 대신 수행하는 구조다.

> **open loop 다.** 명령을 발행할 뿐 컨트롤러의 도달 여부를 확인하지 않으므로, 정상 반환이 목표
> 도달을 뜻하지 않는다. 명령 배열에는 관절 이름이 없어 **순서가 어긋나면 조용히 엉뚱한 관절이
> 움직인다** (순서는 컨트롤러의 `joints` 파라미터에서 자동 조회한다). 궤적의
> velocity/acceleration 필드는 사용하지 않고 position 만 발행한다.

호출부 관점에서 달라지는 점은 셋이다.

1. **계획/실행 분리**는 `execute_trajectory()` 대신 `plan_*` + `stream_trajectory()` 로 한다.
2. **발행한 point 개수**가 필요하면 공통 이름 대신 `*_streamed` 를 직접 호출한다.
3. **`publish_rate` 를 반드시 검토한다.** MoveIt 궤적은 시간 파라미터화 단계에서 항상 ~10 Hz 로 resample 되며 `max_step` 을 줄여도 point 밀도는 늘지 않는다. JTC 는 컨트롤러가 point 사이를 보간하므로 드러나지 않지만, 스트리밍 경로는 그대로 10 Hz 계단 명령이 된다. `publish_rate=200.0` 처럼 주면 균일 격자로 선형 보간한다.

```python
client = create_move_group_client(node, mode='jgpc')
client.wait_until_ready()

trajectory = client.plan_named_target('ready', velocity_scaling=0.3)
published = client.stream_trajectory(trajectory, publish_rate=200.0)
```

**스트리밍 원리, `TrajectoryStreamer` 직접 사용, `time_scaling` / `publish_rate` 실측치, `tolerance` 가 최종 자세를 결정하는 이유, servo 와의 명령 토픽 충돌, `replay_gui` 연동은 [MoveGroupJgpcClient_UserGuide.md](MoveGroupJgpcClient_UserGuide.md) 에 있다.** 이 문서는 JGPC 전용 API 를 목록으로만 다룬다.

---

## 13. 에러 처리

### 13.1 예외 계층

```
ValueError          — 잘못된 파라미터 (즉시 발생)
TimeoutError        — 서버 응답/실행 시간 초과
RuntimeError        — 계획 실패, goal 거부, 실행 실패, close 후 호출
```

`follow_trajectory_async`의 경우 예외가 반환된 `Future`에 설정된다. `future.exception()`으로 확인한다.

### 13.2 실전 에러 처리 패턴

```python
with create_move_group_client(node, velocity_scaling=0.2) as client:
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

### 13.3 주요 실패 시나리오

| 증상 | 원인 | 해결 |
|---|---|---|
| `wait_until_ready()` timeout | MoveIt2 스택 미실행 | `ros2 launch robot_control panda_mock.launch.py` 확인 |
| `fraction` 부족으로 `RuntimeError` | Waypoint가 도달 불가능한 위치 | 작업 영역 내로 waypoint 수정 또는 `fraction_threshold` 하향 |
| `execute_trajectory` timeout | 로봇이 움직이다 멈춤 | timeout 증가 또는 trajectory 길이 확인 |
| `Goal rejected` | Controller 미준비 | Controller spawner 완료 여부 확인 |
| `move_to_joints` 계획 실패 | 그룹 밖 관절을 `joint_values` 에 포함 | 해당 그룹의 관절만 남기거나 그룹별 클라이언트 분리 |
| JGPC 에서 정상 반환했는데 도달 안 함 | open loop — 명령 발행 완료일 뿐 | 도달 확인이 필요하면 JTC 스택 사용, 또는 `/joint_states` 로 직접 확인 |
| JGPC 동작이 10 Hz 로 끊김 | MoveIt 궤적이 ~10 Hz 로 resample 됨 | `publish_rate=200.0` — [MoveGroupJgpcClient_UserGuide.md](MoveGroupJgpcClient_UserGuide.md) |

JGPC 고유의 실패(SRDF `transport` 자세의 `-27` 자기충돌, servo 와 명령 토픽 충돌 등)는
[MoveGroupJgpcClient_UserGuide.md](MoveGroupJgpcClient_UserGuide.md) 의 「에러 처리」·「주의사항」에 있다.

---

## 14. Threading 주의사항

동기 메서드는 내부적으로 `await_future_spin`(`rclpy.spin_until_future_complete` 기반)을 호출한다. 이로 인해 **주입된 Node를 직접 spin**한다.

### 14.1 안전한 사용 패턴

```python
# 메인 스레드에서만 호출 — 가장 안전
rclpy.init()
node = Node('my_node')
with create_move_group_client(node) as client:
    client.wait_until_ready()
    client.follow_trajectory(waypoints)
node.destroy_node()
rclpy.shutdown()
```

### 14.2 위험한 패턴

```python
# 이중 spin — 데드락/경합 발생
executor = MultiThreadedExecutor()
executor.add_node(node)
thread = Thread(target=executor.spin)
thread.start()

# 이 상태에서 동기 메서드 호출하면 동일 Node를 두 곳에서 spin
client.follow_trajectory(waypoints)  # 위험!
```

### 14.3 MultiThreadedExecutor 환경에서의 대안

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

Low-level 비동기 API(`plan_trajectory_async`, `plan_named_target_async`, `plan_joints_async`)도 Future를 반환하며 Node를 직접 spin하지 않으므로 자체 executor에서 처리할 수 있다. 단, `execute_trajectory_async`는 goal 수락 단계에서 `await_future_spin`을 사용하므로 이중 spin 환경에서는 주의가 필요하다.

| 상황 | 쓸 것 |
|---|---|
| 메인 스레드 단독 | 동기 API 전부 |
| 다른 스레드가 Node를 spin 중 | `*_async` 전부 (`execute_trajectory_async` 제외) |
| JGPC + 호출 스레드 블로킹 불가 (Tk GUI 등) | `move_to_named_target_streamed_async(externally_spun=True)` |

---

## 15. 실전 예제

### 15.1 사각형 경로 + 속도 제어

```python
from rdfp.moveit import create_move_group_client, pose

with create_move_group_client(node, velocity_scaling=0.2) as client:
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

### 15.2 계획/실행 분리 + Trajectory 재사용

> `execute_trajectory()` 는 **JTC 전용**이다. JGPC 에서는 `stream_trajectory()` 로 바꾼다.

```python
with create_move_group_client(node) as client:
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

### 15.3 여러 경로 순차 실행

```python
with create_move_group_client(node, velocity_scaling=0.3) as client:
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

### 15.4 중단 가능한 joint 목표 이동

```python
import time

from rclpy.executors import MultiThreadedExecutor
from threading import Thread

executor = MultiThreadedExecutor()
executor.add_node(node)
Thread(target=executor.spin, daemon=True).start()

with create_move_group_client(node, velocity_scaling=0.2) as client:
    client.wait_until_ready()

    future = client.move_to_joints_async({'panda_joint1': 1.5, 'panda_joint4': -1.0})

    # 최대 5초만 기다리고 중단한다
    deadline = time.monotonic() + 5.0
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.1)

    if client.has_active_goal():
        client.cancel()   # 비동기 — 로봇은 감속 후 멈춘다
        node.get_logger().warning('Motion cancelled, pose is indeterminate')
        client.move_to_named_target('ready')   # 알려진 자세로 복귀
```

JGPC 스택 예제는 [MoveGroupJgpcClient_UserGuide.md](MoveGroupJgpcClient_UserGuide.md) 의 「Quick Start」·「TrajectoryStreamer 직접 사용」을 본다.

---

## 16. 기본 Timeout 상수 및 기본값

| 상수 | 기본값 | 용도 |
|---|---|---|
| `READY_TIMEOUT_SEC` | 30초 | `wait_until_ready()`, `get_named_targets()` / `get_all_named_targets()` / `get_planning_groups()` SRDF 조회 |
| `PLAN_TIMEOUT_SEC` | 20초 | `plan_trajectory()` |
| `GOAL_ACCEPT_TIMEOUT_SEC` | 10초 | `execute_trajectory_async()` goal 수락 대기 |
| `TRAJECTORY_EXEC_TIMEOUT_SEC` | 120초 | `execute_trajectory()` 실행 완료 대기 |
| `MOVE_GROUP_TIMEOUT_SEC` | 120초 | `move_to_named_target()` / `move_to_joints()` / `plan_named_target()` / `plan_joints()` 전체 |
| `DEFAULT_PLANNING_TIME` | 5.0초 | joint-space API 의 MoveGroup 계획 허용 시간 |
| `DEFAULT_JOINT_TOLERANCE` | `1e-4` rad | joint-space API 의 각 관절 목표 허용 오차 |

`follow_trajectory()`의 기본 timeout은 `PLAN_TIMEOUT_SEC + TRAJECTORY_EXEC_TIMEOUT_SEC` = **140초**이다.
JGPC 의 `follow_trajectory()` / `follow_trajectory_streamed()` 도 같은 값을 쓰지만, 스트리밍 시간은
궤적 길이가 결정하므로 timeout 에 포함되지 않는다 — 계획 단계에만 적용된다.

**비동기 메서드에는 timeout 파라미터가 없다** (`follow_trajectory_async`, `move_to_named_target_async`, `move_to_joints_async`, `plan_named_target_async`, `plan_joints_async`, `plan_trajectory_async`, `*_streamed_async`). 콜백 체인이 각 단계의 응답을 기다리며, 외부에서 timeout을 관리해야 한다.
