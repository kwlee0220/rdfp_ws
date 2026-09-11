# ServoClient Programmer's Guide

MoveIt2의 **Servo 노드**(`/servo_node`)를 Python 쪽에서 쉽게 시작/정지/상태 확인할 수
있도록 해주는 **유틸리티 클래스**다. 본 가이드는 [servo_client.py](../../src/robot_control/robot_control/moveit/servo_client.py)의
`ServoClient` 클래스를 **언제, 어떻게 사용하고, 어떤 한계가 있는지**를 다룬다.

> **주의**: `ServoClient`는 이름과 달리 `rclpy.node.Node`가 **아니다**. 별도의 ROS2
> 노드가 아니라, 호출자가 소유한 `Node` 인스턴스를 받아서 그 위에 서비스 클라이언트와
> 구독자를 생성하는 **래퍼(wrapper)** 이다. 실제 Servo 프로세스는 launch 파일에서
> `moveit_servo`의 `servo_node_main` 실행 파일로 기동되며
> ([src/robot_control/launch/panda_mock.launch.py:68-70](../../src/robot_control/launch/panda_mock.launch.py#L68-L70)),
> 본 클래스는 그 프로세스와 ROS2로 통신할 뿐이다.

## 개요

```
  사용자의 rclpy.Node
         │
         ▼
   ┌──────────────────────┐           ┌─────────────────────────┐
   │   ServoClient        │           │   servo_node (C++)      │
   │ ─ Trigger 클라이언트   │──서비스 ──►│  /start_servo           │
   │   × 5                │           │  /stop_servo            │
   │ ─ /status 구독자      │◄──토픽 ────│  /pause_servo           │
   │ ─ 상태 폴링 로직       │           │  /unpause_servo         │
   │ ─ auto_start()       │           │  /reset_servo_status    │
   └──────────────────────┘           │  /status (std_msgs/Int8)│
                                      └─────────────────────────┘
```

### 책임 분담

| 구성요소 | 역할 |
|---------|------|
| 호출자의 `rclpy.Node` | 이벤트 루프(`spin`), 서비스 클라이언트/구독자 소유 |
| `ServoClient` | Servo 서비스 호출 + 상태 수신 + `auto_start` 편의 로직 |
| `ServoStatus` | `std_msgs/Int8` 코드를 가독 가능한 IntEnum으로 매핑 |
| `moveit_servo` (외부) | 실제 서보 계산/퍼블리시를 수행하는 C++ 노드 |

### 주요 특성

- **Stateless utility** — 내부 타이머/콜백 체인을 두지 않고, 메서드 호출 시마다
  호출자의 `Node`에 대해 `rclpy.spin_once(...)`로 동기 폴링
- **서비스 타입은 전부 `std_srvs/Trigger`** — 요청 본문 없음
- **상태 토픽 QoS는 BEST_EFFORT**
  ([servo_client.py:316-322](../../src/robot_control/robot_control/moveit/servo_client.py#L316-L322)) — MoveIt Servo의 status
  publisher와 호환성을 맞추기 위한 선택
- **`auto_start()`는 fire-and-forget** — `start_servo` 응답을 기다리지 않고 즉시 반환
  ([servo_client.py:293-296](../../src/robot_control/robot_control/moveit/servo_client.py#L293-L296))
- **ROS2 노드 자체는 생성하지 않음** — 생명주기는 호출자가 관리

## 빠른 시작

### 1. 최소 예제 (편의 함수)

```python
import rclpy
from rclpy.node import Node
from robot_control.moveit.servo_client import auto_start_servo_simple

rclpy.init()
node = Node("my_teleop")

if auto_start_servo_simple(node):
    node.get_logger().info("servo ready")
else:
    node.get_logger().error("servo start failed")

# ... 이후 /servo_node/delta_twist_cmds 등 퍼블리시 ...

node.destroy_node()
rclpy.shutdown()
```

### 2. 클래스 직접 사용

```python
from robot_control.moveit.servo_client import ServoClient, ServoStatus

class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop")
        self.servo = ServoClient.create(self, "/servo_node")

        if not self.servo.auto_start():
            raise RuntimeError("servo auto start failed")

    def emergency_stop(self) -> None:
        ok, msg = self.servo.pause()
        if not ok:
            self.get_logger().warning(f"pause failed: {msg}")
```

`ServoClient.create(node, servo_node_name)`는 생성자와 동등한 factory다. 두 방식 중
어느 쪽을 써도 된다.

### 3. 무인 스택 — `servo_auto_start_node` 를 launch 에 넣기

replay 처럼 **사람이 개입하지 않는 스택**에서는 `start_servo` 를 호출할 주체가 없다.
이때는 직접 `ServoClient` 를 들지 말고
[servo_auto_start_node](../../src/robot_control/robot_control/moveit/servo_auto_start_node.py) 를 launch 에
함께 띄운다. 서비스 준비를 기다린 뒤 `auto_start()` 를 한 번 호출하고 종료하는
일회성 노드다.

```bash
ros2 run robot_control servo_auto_start_node --ros-args -p service_timeout:=30.0
```

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `servo_node_name` | `/servo_node` | 대상 servo 노드 이름 |
| `service_timeout` | `30.0` | 서비스 준비 대기 한도(초) |
| `startup_delay` | `0.0` | 첫 시도 전 지연(초) |
| `exit_after_start` | `True` | 시작 요청 후 노드를 종료할지 여부 |

> **이 노드가 존재하는 이유**: `auto_start()` 내부의 서비스 대기는 **3초 고정**이다
> ([auto_start()](#auto_start---bool) 참고). launch 에서 `servo_node` 와 동시에 spawn 하면
> 3초 안에 서비스 등록이 끝나지 않아 경합이 난다. 이 노드는 자체 `service_timeout`
> (기본 30초)으로 먼저 기다린 뒤 `auto_start()` 를 부르므로 그 문제를 피한다.

## API 레퍼런스

### 생성

```python
ServoClient(node: Node, servo_node_name: str = "/servo_node")
ServoClient.create(node, servo_node_name="/servo_node") -> ServoClient
```

- `node`: 서비스 클라이언트와 구독자가 붙을 `rclpy.node.Node`. **필수**.
- `servo_node_name`: Servo 프로세스의 노드 이름(= 서비스/토픽 prefix).
  `moveit_servo` 노드를 다른 이름으로 띄웠다면 변경.

생성 시점에 즉시 아래 리소스를 만든다.

- 서비스 클라이언트 5개 — `start`, `stop`, `pause`, `unpause`, `reset_status`
- 구독자 1개 — `<servo_node_name>/status` (`std_msgs/Int8`, BEST_EFFORT)

### 서비스 제어 메서드

모두 `(success: bool, message: str)` 튜플을 반환하고 타임아웃은 기본 5초다.

| 메서드 | 호출 서비스 | 동작 |
|------|------------|------|
| `start(timeout_sec=5.0)` | `start_servo` | Servo loop 시작 |
| `stop(timeout_sec=5.0)` | `stop_servo` | Servo loop 종료 |
| `pause(timeout_sec=5.0)` | `pause_servo` | 명령 수신은 유지하되 출력 중지 |
| `unpause(timeout_sec=5.0)` | `unpause_servo` | pause 해제 |
| `reset_status(timeout_sec=5.0)` | `reset_servo_status` | 경고 상태(singularity/limit/collision) 클리어 |

내부적으로 공통 로직은 [`call_servo_service()`](../../src/robot_control/robot_control/moveit/servo_client.py#L122-L161)에 있다.

```python
def call_servo_service(client, service_name, timeout_sec=5.0) -> tuple[bool, str]:
    # 1. 서비스 ready 체크
    # 2. Trigger.Request()로 call_async
    # 3. future.done() 될 때까지 spin_once(0.1) 폴링
    # 4. response.success 확인
```

**중요**: 이 호출 패턴은 `spin_once`를 직접 돌리기 때문에 **호출자의 Node에
`MultiThreadedExecutor`가 걸려 있거나 별도 스레드가 이미 spin을 돌리고 있는 경우
경합이 발생할 수 있다.** 다중 스레드/executor 환경에서는 ServoClient를 사용하는
모든 호출이 같은 스레드에서 직렬화되도록 보장한다.

### 상태 조회 메서드

#### `check_status(wait_for_status=True, timeout_sec=3.0) -> tuple[ServoStatus, str]`

현재 상태와 사람이 읽을 수 있는 문자열을 반환한다. `wait_for_status=True`이면
아직 상태 메시지를 1건도 받지 못한 경우 타임아웃까지 대기(폴링)한다.

```python
status, status_str = servo.check_status(timeout_sec=2.0)
if status == ServoStatus.HALT_FOR_COLLISION:
    handle_collision()
```

#### `is_healthy() -> bool`

현재 상태가 `NO_WARNING`(= 0)일 때만 `True`. 경고/정지 상태는 모두 `False`.

#### `is_started() -> bool`

`UNKNOWN`(-1)이 아닌 상태 메시지를 **한 번이라도** 받았으면 `True`. Servo가 실제로
명령을 퍼블리시 중인지 여부를 대략적으로 판단하는 지표다.

#### `get_status_string(status=None) -> str`

`ServoStatus` enum을 사람이 읽기 좋은 문자열로 변환한다. 인자를 생략하면 현재 상태를
변환한다.

```python
servo.get_status_string()                                    # "OK"
servo.get_status_string(ServoStatus.HALT_FOR_SINGULARITY)   # "HALT_FOR_SINGULARITY"
```

### `wait_for_services_ready(timeout_sec=10.0) -> bool`

`start_servo` 클라이언트의 `service_is_ready()`가 `True`가 될 때까지 폴링한다.
기본 동작은 **`start_servo` 하나만** 검사하므로, Servo 프로세스가 뜨기만 했다면
나머지 서비스도 이미 올라와 있다고 가정한다.

### `auto_start() -> bool`

고수준 편의 로직. 다음 단계를 순차 실행한다.

1. `wait_for_services_ready(timeout_sec=3.0)` — 3초 안에 준비 안 되면 실패 반환
2. `check_status(timeout_sec=2.0)` — 현재 상태 조회
3. 이미 `is_healthy()`면 조기 성공 반환
4. `start_servo`를 **fire-and-forget**으로 호출하고 즉시 `True` 반환

```python
# 내부 구조 (요약)
def auto_start(self) -> bool:
    if not self.wait_for_services_ready(3.0):
        return False
    status, _ = self.check_status(wait_for_status=True, timeout_sec=2.0)
    if self.is_healthy():
        return True
    if not self.start_client.service_is_ready():
        return False
    self.start_client.call_async(Trigger.Request())  # 응답 검증 생략
    return True
```

> **왜 fire-and-forget인가?** 과거 구현에서는 `start_servo` 응답을 기다렸으나, Servo
> 내부 상태 전이가 응답 타이밍과 정확히 맞지 않아 응답이 `success=True`여도 직후의
> `/status`가 여전히 `UNKNOWN`인 경우가 있었다. 현재 구현은 상태 구독자가 별도로
> 동작하므로, 시작 요청만 쏴두고 호출자가 필요할 때 `check_status()`로 확인하는 모델을
> 채택했다.

### 편의 함수 `auto_start_servo_simple(node, servo_node_name="/servo_node") -> bool`

한 줄로 `ServoClient.create(node).auto_start()`를 수행하는 래퍼다. `ServoClient`
인스턴스를 계속 들고 다닐 필요가 없는 스크립트용.

```python
from robot_control.moveit.servo_client import auto_start_servo_simple
auto_start_servo_simple(self)  # self는 rclpy.Node
```

단, 이 함수는 `ServoClient` 인스턴스를 지역 변수로만 잡고 버리므로, 이후 **상태
구독자도 함께 사라진다**. `check_status()`나 이후 `stop()` 호출이 필요하다면
편의 함수 대신 `ServoClient`를 직접 생성해 멤버로 유지한다.

## `ServoStatus` enum

```python
class ServoStatus(IntEnum):
    UNKNOWN = -1
    NO_WARNING = 0
    DECELERATE_FOR_APPROACHING_SINGULARITY = 1
    HALT_FOR_SINGULARITY = 2
    DECELERATE_FOR_LEAVING_SINGULARITY = 3
    DECELERATE_FOR_JOINT_LIMIT = 4
    HALT_FOR_JOINT_LIMIT = 5
    DECELERATE_FOR_COLLISION = 6
    HALT_FOR_COLLISION = 7
```

- `UNKNOWN`은 라이브러리 쪽 센티넬(`-1`). 상태 메시지를 받기 전의 초기값.
- `NO_WARNING ~ HALT_FOR_COLLISION`은 MoveIt Servo의
  [`moveit_servo::StatusCode`](https://moveit.picknik.ai/main/api/html/namespacemoveit__servo.html)와
  일대일 매핑.
- `DECELERATE_*`는 감속 경고, `HALT_*`는 Servo가 명령을 0으로 고정한 상태.
- `HALT_*`에서 복구하려면 원인(관절 한계/충돌 기하)을 제거한 뒤 `reset_status()`를
  호출해야 한다.

### 상태 전이 (대략)

```
    /status 첫 수신
 UNKNOWN ─────────────► NO_WARNING
                           │
              (특이점 근접)│
                           ▼
              DECELERATE_FOR_APPROACHING_SINGULARITY
                           │
                     (넘어섬)
                           ▼
                  HALT_FOR_SINGULARITY
                           │
                           │ reset_status() + 관절 이동
                           ▼
                      NO_WARNING
```

조인트 한계/충돌도 동일 패턴(`DECELERATE → HALT → reset`)이다.

## 사용 패턴

### 패턴 1: 텔레옵 시작 직전 헬스체크

[src/rdfp/rdfp/teleop/teleop_keyboard.py:172](../../src/rdfp/rdfp/teleop/teleop_keyboard.py#L172)에서
사용되는 패턴이다.

```python
class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_keyboard")
        self.servo = ServoClient.create(self, "/servo_node")

        # Servo가 준비되지 않으면 노드 자체 기동을 막음
        if not self.servo.auto_start():
            raise RuntimeError("failed to auto-start MoveIt Servo")
```

### 패턴 2: 상태 모니터링 루프

```python
def spin_with_servo_watchdog(self) -> None:
    while rclpy.ok():
        rclpy.spin_once(self, timeout_sec=0.05)

        status, status_str = self.servo.check_status(wait_for_status=False)
        if status in (ServoStatus.HALT_FOR_SINGULARITY,
                      ServoStatus.HALT_FOR_JOINT_LIMIT,
                      ServoStatus.HALT_FOR_COLLISION):
            self.get_logger().warning(f"servo halted: {status_str}")
            self._freeze_user_input()
```

`check_status(wait_for_status=False)`는 블로킹하지 않고 마지막으로 받은 상태를
즉시 반환한다.

### 패턴 3: HALT 상태 복구

```python
def recover_from_halt(self) -> bool:
    status, _ = self.servo.check_status(wait_for_status=False)
    if status not in (ServoStatus.HALT_FOR_SINGULARITY,
                      ServoStatus.HALT_FOR_JOINT_LIMIT,
                      ServoStatus.HALT_FOR_COLLISION):
        return True  # 복구 불필요

    ok, msg = self.servo.reset_status()
    if not ok:
        self.get_logger().error(f"reset_status failed: {msg}")
        return False

    # reset 후 상태 갱신 대기
    time.sleep(0.1)
    rclpy.spin_once(self, timeout_sec=0.1)
    return self.servo.is_healthy()
```

### 패턴 4: 안전 종료

```python
def destroy_node(self) -> None:
    try:
        if self.servo.is_started():
            self.servo.stop(timeout_sec=2.0)
    except Exception as e:
        self.get_logger().warning(f"servo stop failed: {e}")
    finally:
        super().destroy_node()
```

`stop()`은 `ExecuteTrajectory` 같은 다른 인터페이스와 충돌하지 않도록 Servo loop를
명시적으로 끈다. 생략해도 프로세스 종료 시 문제는 없지만, launch에서 다른 노드와
연계될 때 깔끔하다.

## 한계와 주의사항

### 1. 호출자 스레드에서 `spin_once`를 직접 호출함

모든 서비스/상태 폴링은 호출자의 `Node`에 대해 `rclpy.spin_once(node, 0.1)`을
직접 돌린다([servo_client.py:116](../../src/robot_control/robot_control/moveit/servo_client.py#L116),
[145](../../src/robot_control/robot_control/moveit/servo_client.py#L145), [239](../../src/robot_control/robot_control/moveit/servo_client.py#L239)). 이로 인해:

- **호출자가 이미 다른 스레드에서 spin 중이면** 동일 `Node`에 대한 `spin_once`가
  예외를 던질 수 있다.
- **`MultiThreadedExecutor` + reentrant callback 그룹** 환경에서 timing 경합이
  발생할 수 있다.
- **서비스 콜백 안에서 `ServoClient` 메서드를 호출하면 데드락**이 될 수 있다
  (자기 자신의 executor를 spin하려 시도).

가능하면 **메인 스레드 또는 전용 스레드** 한 곳에서만 `ServoClient` 메서드를 호출하는
것을 권장한다.

### 2. `auto_start()`는 성공 여부를 보장하지 않음

fire-and-forget이므로 실제로 Servo가 움직일 수 있게 됐는지는 직접 확인해야 한다.

```python
servo.auto_start()
time.sleep(0.2)                      # Servo 내부 전이 대기
if not servo.is_healthy():
    raise RuntimeError("servo did not become healthy")
```

### 3. `is_started()`는 "살아있음" 확인이 아님

`UNKNOWN`이 아닌 상태 메시지를 한 번이라도 받았는지만 검사한다. Servo 프로세스가
중간에 죽었더라도 마지막 상태값이 남아 있어 `True`를 반환한다. 살아있음을
엄밀하게 확인하려면:

```python
# 방법 A: 노드 그래프 확인
node_names = self.get_node_names()
assert "servo_node" in node_names

# 방법 B: 서비스 ready 재확인
assert servo.start_client.service_is_ready()
```

### 4. 타임아웃은 폴링 루프 상한

`call_servo_service()`의 타임아웃은 `future.done()`을 `spin_once(0.1)`로 폴링하면서
경과시간을 재는 방식이다. `rclpy.spin_once`가 블로킹되는 상황(다른 콜백이 길게
잡혀있는 등)에서는 **실제 타임아웃이 파라미터보다 길어질 수 있다**.

### 5. `wait_for_services_ready`는 `start_servo`만 검사

5개 서비스를 모두 확인하지 않는다. 일반적으로 `moveit_servo`는 모든 서비스를
한 번에 등록하므로 문제되지 않지만, **커스텀 Servo 구현에서는 확인 필요**.

### 6. 타입 힌트는 구형 스타일

프로젝트 컨벤션(`tuple[int, str]`, `list[str]` 권장)과 달리 `Tuple[...]`, `Optional[...]`
스타일을 사용한다([servo_client.py:9](../../src/robot_control/robot_control/moveit/servo_client.py#L9)). 수정 시에는 프로젝트
컨벤션에 맞춰 점진적으로 바꾸는 것을 권장한다.

## 확장 지점

`ServoClient` 가 제공하지 않는 동작이 필요할 때 손댈 수 있는 곳이다. **아래는 현재
구현에 실제로 있는 hook 이며, 여기서 파생시키는 코드는 직접 작성해야 한다.**

| 목적 | 손댈 곳 |
|---|---|
| 상태 전이에 반응 (현재는 내부 저장만 함) | `_status_callback(msg)` 를 서브클래스에서 오버라이드하고 `super()` 호출 전후로 `self.current_status` 를 비교 |
| status 토픽 QoS 변경 (커스텀 Servo 가 RELIABLE 인 경우) | `_create_status_subscriber()` 오버라이드 |
| 비동기 서비스 호출 | 서비스 클라이언트가 `self.start_client` / `stop_client` / … 로 공개되어 있으므로 `call_async(Trigger.Request())` 를 직접 호출. 반환된 `Future` 는 호출자가 자체 executor 에서 spin 해야 완료된다 |

> **`HALT_*` 자동 복구는 넣지 않았다.** `reset_status()` 는 경고 플래그만 지우므로
> 원인(특이점/관절 한계/충돌 기하)이 남아 있으면 다음 cycle 에 다시 HALT 로 빠진다.
> 원인 해결 없이 reset 을 반복하는 워치독은 위험하다
> ([트러블슈팅 3](#3-halt_for_singularity-상태가-reset_status-후에도-유지됨) 참고).

## 트러블슈팅

### 1. `auto_start()`가 `False` 반환 — "services not ready"

**원인**: `moveit_servo`가 아직 기동 중이거나, 서비스 이름 prefix 불일치.

**확인**:
```bash
# Servo가 살아있는지
ros2 node list | grep servo_node

# 서비스가 실제로 등록됐는지
ros2 service list | grep servo_node

# 서비스 경로가 /servo_node/start_servo인지 확인 (prefix 포함)
```

prefix가 다르면 생성자에 `servo_node_name="/ns/servo_node"` 형태로 넘긴다.

### 2. `check_status()`가 계속 `UNKNOWN` 반환

**원인**: `/status` 토픽이 publish되지 않거나, QoS 불일치.

**확인**:
```bash
ros2 topic echo /servo_node/status
ros2 topic info /servo_node/status -v   # QoS 확인
```

Servo status publisher는 일반적으로 BEST_EFFORT이다. 본 클래스도 BEST_EFFORT로
구독하므로 기본 상태에서는 맞다. 만약 커스텀 Servo에서 RELIABLE을 쓴다면
`_create_status_subscriber()`를 오버라이드한다.

### 3. `HALT_FOR_SINGULARITY` 상태가 `reset_status()` 후에도 유지됨

**원인**: `reset_status`는 경고 플래그만 클리어한다. **로봇이 여전히 특이점
근처에 있으면** 다음 cycle에서 다시 HALT로 빠진다.

**해결**: reset 전에 조인트 공간으로 로봇을 약간 움직여 특이점에서 빠져나오거나,
명령 스케일을 낮춰서 특이점을 피한다. Servo의 singularity threshold
(`lower_singularity_threshold`, `hard_stop_singularity_threshold`)를 조정하는 것도 선택지.

### 4. 서비스 호출이 타임아웃 (`… call timed out after 5.0s`)

**원인**:
- Servo 프로세스가 서비스는 등록했지만 콜백이 블로킹됨
- 호출자 Node의 executor가 이미 다른 스레드에서 spin 중이라 `spin_once`가 제대로
  돌지 못함

**확인**:
```bash
# Servo 프로세스 CPU 상태
top -p $(pgrep -f servo_node_main)

# 다른 노드가 Servo를 막고 있지 않은지
ros2 topic hz /servo_node/delta_twist_cmds
```

호출자 쪽 경합이 의심되면 메인 스레드에서만 `ServoClient`를 호출하도록 바꾼다.

### 5. `teleop_keyboard` 기동 시 `auto_start` 실패

**원인**: 대개는 launch 순서 문제다. `panda_mock.launch.py`는 컨트롤러 chain이
모두 뜬 **후에** `servo_node`를 기동하므로, teleop을 너무 일찍 실행하면 Servo가
아직 서비스 등록을 끝내지 못했을 수 있다. `auto_start()` 내부 대기가 **3초
고정**이라 그 안에 등록이 끝나지 않으면 바로 `False`가 된다.

**해결**:
- `panda_mock.launch.py` 기동 후 RViz 창이 열린 뒤에 teleop을 실행
- 또는 `auto_start()` 대신 `wait_for_services_ready(timeout_sec=30.0)`를 먼저 호출해
  여유를 준 뒤 `start()`를 호출
- launch 에서 자동으로 처리하려면
  [servo_auto_start_node](#3-무인-스택--servo_auto_start_node-를-launch-에-넣기) 를 쓰기

## 관련 파일/문서

- [servo_client.py](../../src/robot_control/robot_control/moveit/servo_client.py) — 본 문서가 설명하는 소스
- [servo_auto_start_node.py](../../src/robot_control/robot_control/moveit/servo_auto_start_node.py) —
  무인 스택용 일회성 기동 노드 (`ServoClient` 를 감싼다)
- [src/robot_control/launch/panda_mock.launch.py](../../src/robot_control/launch/panda_mock.launch.py) —
  실제 `servo_node` 프로세스를 기동하는 launch 파일
- [MoveIt Servo 공식 문서](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html) —
  Servo 노드의 파라미터/인터페이스 전반

**실제 사용처** (`ServoClient` 를 직접 생성하는 코드):

| 파일 | 용도 |
|---|---|
| [teleop/teleop_keyboard.py](../../src/rdfp/rdfp/teleop/teleop_keyboard.py) | 기동 시 헬스체크 — `auto_start()` 실패면 노드 기동 중단 |
| [moveit/servo_auto_start_node.py](../../src/robot_control/robot_control/moveit/servo_auto_start_node.py) | launch 에서 무인 기동 |
| [dataset/replay_cmd.py](../../src/rdfp/rdfp/dataset/replay_cmd.py) | replay 시작 전 servo 기동 |
| [dataset/replay_gui_cmd.py](../../src/rdfp/rdfp/dataset/replay_gui_cmd.py) | Tk GUI — 외부 executor 가 spin 중이라 서비스 클라이언트를 재사용 |
