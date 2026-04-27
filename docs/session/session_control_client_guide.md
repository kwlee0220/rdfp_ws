# SessionControlClient Programmer's Guide

이 문서는 [src/rdfp/rdfp/session/session_control_client.py](../../src/rdfp/rdfp/session/session_control_client.py)
의 `SessionControlClient` 를 **사용하는 프로그래머** 관점에서 작성되었습니다.
내부 구현보다는 언제·어떻게 이 클래스를 쓰고, 어떤 제약을 염두에 두어야
하는지를 다룹니다.

서버 쪽(즉 `SessionControlNode`) 설명은
[session_control_guide.md](./session_control_guide.md) 를, 요구사항 명세는
[session_control_srs.md](./session_control_srs.md) 를 참조하세요.

---

## 1. 개요

`SessionControlClient` 는 `SessionControlNode` 가 제공하는 **6 개 서비스**를
파이썬 메서드로 래핑한 클라이언트 클래스입니다.

| 서비스 | 타입 | wrapper 메서드 |
|---|---|---|
| `start_session` | `std_srvs/srv/Trigger` | `start_session()` / `start_session_async()` |
| `stop_session` | `std_srvs/srv/Trigger` | `stop_session()` / `stop_session_async()` |
| `start_episode` | `std_srvs/srv/Trigger` | `start_episode()` / `start_episode_async()` |
| `stop_episode` | `std_srvs/srv/Trigger` | `stop_episode()` / `stop_episode_async()` |
| `set_task_label` | `rdfp_msgs/srv/SetString` | `set_task_label()` / `set_task_label_async()` |
| `get_session_state` | `rdfp_msgs/srv/GetSessionState` | `get_session_state()` / `get_session_state_async()` |

### 왜 쓰는가?

- **보일러플레이트 제거**: 6 개 클라이언트 생성, `wait_for_service`, 요청
  객체 조립, future 콜백 래핑을 한 줄씩으로 압축합니다.
- **동기·비동기 대칭 API**: 초기화·스크립트 용도엔 동기, 타이머·콜백 루프
  용도엔 비동기 — 같은 이름 규칙(`xxx` / `xxx_async`)으로 전환이 쉽습니다.
- **일관된 오류 표현**: 서버가 `success=false` 로 거부한 경우, 서비스가
  아직 준비되지 않은 경우, 호출이 타임아웃된 경우, 예외가 발생한 경우 모두
  `(success: bool, message: str)` 형태로 통일되어 호출 측 분기가 단순해집니다.
- **Node 주입**: [ServoClient](../../src/rdfp/rdfp/moveit/servo_client.py)
  와 동일한 패턴이라 이미 존재하는 노드에 한 줄로 붙일 수 있습니다.

> **주의**: `SessionControlClient` 는 이름과 달리 자체 `rclpy.Node` 가 **아닙니다**.
> 호출자의 `Node` 를 주입받아 그 위에 서비스 클라이언트만 생성하는 유틸리티
> 클래스입니다.

---

## 2. 아키텍처

```
  사용자의 rclpy.Node
         │
         ▼
   ┌──────────────────────────┐           ┌──────────────────────────┐
   │  SessionControlClient    │           │  session_control         │
   │ ─ 6 서비스 클라이언트    │──서비스──▶│  (SessionControlNode)    │
   │ ─ 동기/비동기 API        │           │  /start_session          │
   │ ─ is_ready() 폴링 유틸   │           │  /stop_session           │
   │ ─ wait_for_services_     │           │  /start_episode          │
   │    ready (생성 시 블록)  │           │  /stop_episode           │
   └──────────────────────────┘           │  /set_task_label         │
                                          │  /get_session_state      │
                                          └──────────────────────────┘
```

생성 시점에 6 개 서비스가 모두 `ready` 가 될 때까지 블로킹 대기합니다. 각
서비스는 `namespace="session_control"` 하위의 상대 경로로 바인딩되며,
필요하다면 생성자 인자로 변경 가능합니다.

---

## 3. 빠른 시작

### 3.1 스크립트 — 동기 API

가장 간단한 사용 패턴입니다. `rclpy.init()` 후 노드를 하나 만들고 클라이언트를
생성한 뒤 바로 메서드를 호출합니다. **동기 메서드는 `rclpy.spin_until_future_complete`
를 내부에서 호출하므로 이 스크립트가 executor 를 별도 스레드에서 돌리고
있지 않아야 합니다.**

```python
import rclpy
from rclpy.node import Node

from rdfp.session.session_control_client import SessionControlClient


def main() -> None:
    rclpy.init()
    node = Node("session_script")

    try:
        # 생성 시 6 개 서비스 모두 ready 될 때까지 블로킹 대기 (10초)
        client = SessionControlClient.create(node, wait_timeout_sec=10.0)

        ok, msg = client.set_task_label("pick_and_place")
        node.get_logger().info(f"set_task_label: ok={ok} msg={msg}")

        ok, msg = client.start_session()
        ok, msg = client.start_episode()
        # ... 실험 수행 ...
        ok, msg = client.stop_episode()
        ok, msg = client.stop_session()

        # task clear 는 None 으로 전달
        client.set_task_label(None)

        # 현재 상태 조회
        state, label = client.get_session_state()
        node.get_logger().info(f"final state={state} task_label={label!r}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### 3.2 노드 통합 — 비동기 API

이미 타이머·구독자·서비스 콜백이 도는 노드에서는 **반드시 비동기 API 를**
사용해야 합니다. 동기 API 를 콜백 안에서 호출하면 동일 executor 의 재귀
spin 으로 데드락이 발생합니다.

```python
from rdfp.session.session_control_client import SessionControlClient


class MyNode(Node):
    def __init__(self) -> None:
        super().__init__("my_node")

        # 생성자에서 블로킹 대기 — 노드 기동 순서상 session_control 이
        # 먼저 떠 있어야 한다. 순서가 불확실하면 wait_timeout_sec 를 넉넉히.
        self.session = SessionControlClient.create(self, wait_timeout_sec=15.0)

        self.create_timer(1.0, self._on_tick)

    def _on_tick(self) -> None:
        # 타이머 콜백에서는 비동기 호출만 사용
        self.session.start_session_async(done_callback=self._on_session_started)

    def _on_session_started(self, success: bool, message: str) -> None:
        if success:
            self.get_logger().info("session started")
        else:
            self.get_logger().warning(f"session start failed: {message}")
```

---

## 4. API 레퍼런스

### 4.1 생성

```python
SessionControlClient(
    node: Node,
    namespace: str = "session_control",
    wait_timeout_sec: float = 10.0,
) -> SessionControlClient

SessionControlClient.create(node, namespace="session_control", wait_timeout_sec=10.0)
```

| 인자 | 설명 |
|---|---|
| `node` | 서비스 클라이언트가 바인딩될 호출자의 `rclpy.node.Node`. **필수**. |
| `namespace` | `SessionControlNode` 의 노드 이름(서비스 prefix). 기본 `"session_control"`. 절대 경로(`"/ns/session_control"`)도 허용. |
| `wait_timeout_sec` | 6 개 서비스가 모두 `ready` 가 될 때까지 대기할 **총** 타임아웃. `0.0` 이하이면 대기하지 않음 (테스트·스크립트용). |

**Raises**: `RuntimeError` — 타임아웃 안에 하나 이상의 서비스가 준비되지 않은 경우.
메시지에 어느 서비스가 준비되지 않았는지 이름이 포함됩니다.

생성자와 `create()` 는 시그니처가 동일합니다. 팩토리는 프로젝트 내
`ServoClient` 등 다른 `*Client` 와의 스타일 통일을 위해 제공됩니다.

### 4.2 동기 API — 세션/에피소드 제어

```python
client.start_session(timeout_sec=5.0) -> tuple[bool, str]
client.stop_session(timeout_sec=5.0)  -> tuple[bool, str]
client.start_episode(timeout_sec=5.0) -> tuple[bool, str]
client.stop_episode(timeout_sec=5.0)  -> tuple[bool, str]
```

모두 `(success, message)` 튜플을 반환합니다.

- `success=True`, `message=""` → 서버가 정상 처리
- `success=False`, `message="invalid command"` → 현재 상태에서 허용되지 않는
  호출 (예: `IDLE` 에서 `stop_session`)
- `success=False`, `message="service not ready"` → 생성 후 서비스가 다시
  사라진 경우 (보통 서버 프로세스 다운)
- `success=False`, `message="service call timed out"` → `timeout_sec` 안에
  응답이 오지 않음
- `success=False`, `message="no response"` → future 는 완료됐으나 결과가 `None`

### 4.3 동기 API — `set_task_label`

```python
client.set_task_label(
    task_label: str | None,
    timeout_sec: float = 5.0,
) -> tuple[bool, str]
```

- `task_label=<문자열>` → 해당 라벨로 설정
- `task_label=None` → **task clear** (내부에서 빈 문자열로 변환되어 서버로 전송)
- `task_label=""` → 동일하게 clear (호출자가 빈 문자열을 선호하면 허용)
- `IN_EPISODE` 상태에서는 서버가 거부하므로 `success=False`,
  `message="invalid command"` 반환

### 4.4 동기 API — `get_session_state`

```python
client.get_session_state(timeout_sec=5.0) -> tuple[str, str]
```

`(state, task_label)` 튜플을 반환합니다.

- 성공 시 `state` 는 `"IDLE"` / `"IN_SESSION"` / `"IN_EPISODE"` 중 하나,
  `task_label` 은 현재 값 (빈 문자열 가능).
- 실패 시 `("", "")` 를 반환하고 warning 로그를 남깁니다.

**빈 문자열이 "호출 실패" 인지 "실제 IDLE/빈 라벨" 인지 구분이 필요하면**
`get_session_state_async()` 를 써서 future 결과를 직접 다루거나, `is_ready()` /
별도 호출로 선행 검사하세요.

### 4.5 비동기 API — 세션/에피소드 제어

```python
client.start_session_async(done_callback=None) -> Future
client.stop_session_async(done_callback=None)  -> Future
client.start_episode_async(done_callback=None) -> Future
client.stop_episode_async(done_callback=None)  -> Future
```

- `done_callback` 은 `Callable[[bool, str], None]`. 응답 수신 시
  `(success, message)` 로 호출됩니다.
- 서비스 호출이 예외로 실패하면 `(False, "exception: ...")` 로 호출됩니다.
- 응답이 `None` 이면 `(False, "no response")` 로 호출됩니다.
- `done_callback=None` 이면 raw `rclpy.task.Future` 만 반환되며, 호출자가
  직접 `future.add_done_callback()` / `future.result()` 로 처리합니다.

### 4.6 비동기 API — `set_task_label`

```python
client.set_task_label_async(
    task_label: str | None,
    done_callback=None,
) -> Future
```

- `task_label=None` 은 동기 버전과 동일하게 clear 의미.
- `done_callback` 은 `(success, message)` 수신.

### 4.7 비동기 API — `get_session_state`

```python
client.get_session_state_async(done_callback=None) -> Future
```

- `done_callback` 은 `Callable[[str, str], None]`. 인자는 `(state, task_label)`.
- 예외·None response 시 `("", "")` 로 호출됩니다.

### 4.8 유틸리티

```python
client.is_ready() -> bool
```

6 개 서비스 **모두** `service_is_ready()` 인 경우에만 `True` 를 반환합니다.
호출 시점에 블로킹 없이 즉시 확인합니다. 초기화 후에도 서버가 재기동되어
연결이 끊길 수 있는 환경에서 방어적으로 체크할 때 유용합니다.

---

## 5. 동기 vs. 비동기 선택 가이드

| 상황 | 권장 API | 이유 |
|---|---|---|
| 스크립트·REPL·테스트 (노드를 직접 돌리지 않음) | **동기** | 순차 실행이 자연스럽고 결과를 바로 반환 |
| `main()` 에서 `rclpy.spin()` 시작 **전에** 초기 설정 | **동기** | 아직 executor 가 돌지 않아 안전 |
| 타이머 콜백 내부 | **비동기** | 타이머 블로킹 금지, 동기 호출은 executor 재귀 spin 야기 |
| 서비스/구독 콜백 내부 | **비동기** | 위와 동일한 이유 |
| 별도 스레드에서 호출 | **비동기 권장** | 동기는 `rclpy.spin_until_future_complete` 의 executor 경합 위험 |
| `keyboard_twist_teleop` 같이 이미 spin 중인 노드 | **비동기** | 동기 호출은 데드락 |

### 동기 API 의 구체적 동작

동기 메서드는 내부에서 다음을 수행합니다.

```python
if not client.service_is_ready():
    return False, "service not ready"

future = client.call_async(request)
rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)

if not future.done():
    return False, "service call timed out"
# ... response 언패킹 ...
```

`rclpy.spin_until_future_complete(self._node, ...)` 는 `self._node` 에
대해 임시 executor 를 구성해 spin 하므로, 같은 노드가 다른 곳에서 이미
spin 중이면 충돌합니다. **executor 가 돌기 시작한 뒤에는 동기 API 를
호출하지 마세요**.

---

## 6. 사용 패턴

### 6.1 노드 초기화 직전에 task/session 설정 후 spin 시작

```python
def main() -> None:
    rclpy.init()
    node = MyNode()

    # 아직 spin 안 함 → 동기 API 안전
    client = SessionControlClient.create(node, wait_timeout_sec=10.0)
    client.set_task_label("pick_and_place")
    client.start_session()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()  # 여기서부터 동기 API 호출 금지
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### 6.2 타이머 콜백에서 한 단계씩 진행

```python
class SessionRunner(Node):
    def __init__(self) -> None:
        super().__init__("session_runner")
        self.client = SessionControlClient.create(self, wait_timeout_sec=10.0)
        self.phase = 0
        self.create_timer(1.0, self._tick)

    def _tick(self) -> None:
        if self.phase == 0:
            self.client.set_task_label_async("push", done_callback=self._advance)
        elif self.phase == 1:
            self.client.start_session_async(done_callback=self._advance)
        elif self.phase == 2:
            self.client.start_episode_async(done_callback=self._advance)
        # ... 이후 단계 동일 패턴 ...

    def _advance(self, success: bool, message: str) -> None:
        if not success:
            self.get_logger().error(f"phase {self.phase} failed: {message}")
            return
        self.phase += 1
```

### 6.3 raw future 를 이용한 고급 제어

`done_callback` 을 쓰지 않고 future 를 직접 다루면 `rclpy` 의 원본 응답
객체에 접근할 수 있습니다. 예를 들어 여러 호출 결과를 모아 처리하고 싶을 때.

```python
future_a = self.client.start_session_async()
future_b = self.client.set_task_label_async("push")

# 이후 콜백 내부에서 결과를 조합
def _when_both_done():
    if future_a.done() and future_b.done():
        resp_a = future_a.result()  # Trigger.Response
        resp_b = future_b.result()  # SetString.Response
        ...
```

### 6.4 task clear

```python
client.set_task_label(None)         # 동기
client.set_task_label_async(None)   # 비동기
client.set_task_label("")           # 빈 문자열도 동일하게 허용
```

모두 서버 내부 `task_label` 을 `""` 로 초기화합니다. `None` 은 호출 측에서
"clear 를 원한다"는 의도를 명시적으로 드러낼 수 있어 권장됩니다.

### 6.5 현재 상태 조회

```python
state, label = client.get_session_state()
if state == "IN_SESSION":
    client.start_episode()
```

상태가 자주 바뀌는 환경에서는 `session` 토픽 구독이 더 적합합니다
(Publisher 가 `TRANSIENT_LOCAL` 이라 late joiner 도 즉시 직전 상태를
수신). `get_session_state` 는 **일회성 확인**이나 **토픽 연결이 안 된
경우의 폴백**으로 쓰세요.

---

## 7. 에러 처리

### 7.1 표준 실패 메시지

동기/비동기 모두 실패 경로는 다음 문자열로 통일되어 있습니다.

| 메시지 | 의미 |
|---|---|
| `"invalid command"` | 서버가 현재 상태에서 거부 |
| `"service not ready"` | 호출 직전 `service_is_ready()` 가 `False` (서버 다운 가능) |
| `"service call timed out"` | `timeout_sec` 안에 응답 없음 |
| `"no response"` | future 는 완료됐으나 `result()` 가 `None` |
| `"exception: <str>"` | 비동기 경로에서 future 가 예외로 완료 |

호출자는 `message` 문자열을 정확 비교하는 대신 **`success` 불 값으로 분기**
하는 것을 권장합니다. 문자열은 로그/디버깅 용입니다.

### 7.2 예외는 생성자에서만

정상 경로에서는 어떤 메서드도 예외를 던지지 않습니다. 예외가 발생하는
경우는 **생성자에서 서비스 ready 타임아웃**(`RuntimeError`) 하나뿐입니다.
이 예외는 launch 순서 문제를 빨리 드러내기 위한 것이며, 호출자는 기동
시점에 try/except 로 감싸거나 `wait_timeout_sec` 를 충분히 길게 주는 쪽을
선택하세요.

### 7.3 서버 재기동 복원력

`SessionControlClient` 는 내부적으로 연결 상태를 추적하지 않습니다. 서버가
재기동되어 서비스가 다시 올라오면 `rclpy` 의 client 가 자동으로 재연결
하고, `is_ready()` 도 다시 `True` 가 됩니다. 단, 재기동 직후 즉시 호출하면
`service_is_ready()` 가 아직 `False` 일 수 있으므로, 호출 실패 시 재시도
로직을 호출자 측에 두세요.

---

## 8. 한계와 주의사항

### 8.1 상태 캐시를 두지 않음

`SessionControlClient` 는 `session` 토픽을 구독하지 않습니다. 현재 상태를
알고 싶으면:

1. `get_session_state()` / `get_session_state_async()` 로 일회성 조회
2. 또는 호출자가 `rdfp_msgs/msg/SessionCommand` 를 `/session_control/session`
   에서 직접 구독 (TRANSIENT_LOCAL QoS)

두 방식을 혼용해도 되지만, 같은 호출자 노드에 구독을 이미 가지고 있다면
`get_session_state` 는 중복이 됩니다.

### 8.2 동기 API 는 "외부 spin 없음" 가정

`rclpy.spin_until_future_complete(self._node, ...)` 는 **해당 노드가 아직
외부 executor 에 의해 spin 되고 있지 않다는 가정**에서 안전합니다. 이미
spin 중이면:

- `spin_until_future_complete` 가 executor 충돌 경고 또는 정의되지 않은 동작
- 서비스 콜백 안에서 호출하면 재귀 spin 으로 데드락

이 경우에는 반드시 `*_async` 변형을 쓰세요.

### 8.3 MultiThreadedExecutor 에서의 비동기 호출

비동기 API 자체는 스레드 세이프합니다 (rclpy client 가 내부적으로 락을
걸어 보호). 다만 `done_callback` 은 future 가 완료될 때 **executor 스레드
중 하나** 에서 호출되므로, 콜백 내부에서 접근하는 사용자 상태는 호출자
책임으로 보호해야 합니다.

### 8.4 `wait_timeout_sec` 예산은 **공유되지 않음**

내부 구현은 각 서비스에 대해 **남은 시간 예산 전체**를 부여합니다. 즉
첫 서비스가 9초를 쓰면 다음 서비스에는 1초가 남습니다. 극단적 환경에서
6 개 서비스가 거의 동시에 뜨는 일반 경우는 문제가 없지만, 한두 서비스만
유독 느리게 뜨면 뒷 서비스가 시간이 부족해질 수 있으니 `wait_timeout_sec`
를 여유 있게 잡으세요.

### 8.5 네임스페이스 변경 시 prefix 직접 지정 필요

서버 노드를 다른 이름으로 띄우거나 launch 에서 네임스페이스를 push 한
경우, `namespace` 인자로 **절대 경로** 또는 실제 prefix 를 전달해야
합니다.

```python
# 예: launch 에서 __ns:=/robot 으로 띄운 경우
client = SessionControlClient.create(node, namespace="/robot/session_control")
```

---

## 9. 트러블슈팅

### 9.1 `RuntimeError: service '...' not available within N.Ns`

**원인**: 생성 시점에 `session_control_node` 가 아직 뜨지 않았거나 서비스
이름이 틀림.

**확인**:

```bash
ros2 node list | grep session_control
ros2 service list | grep session_control
```

**해결**:

- launch 순서를 조정해 `session_control_node` 를 먼저 기동
- `wait_timeout_sec` 를 더 길게 (예: 30초)
- 네임스페이스가 기본값이 아니면 `namespace` 인자로 전달

### 9.2 동기 호출이 멈추거나 타임아웃됨

**원인**: 호출자 노드가 이미 executor 에서 spin 중인데 동기 API 를 호출함.

**해결**: `*_async` 변형으로 전환. 필요하면 [session_control_client.py](../../src/rdfp/rdfp/session/session_control_client.py)
의 동기/비동기 판별 가이드(클래스 docstring) 재확인.

### 9.3 `success=False`, `message="invalid command"`

**원인**: 현재 상태에서 허용되지 않는 서비스를 호출.

**해결**: `get_session_state()` 또는 `session` 토픽 구독으로 현재 상태를
확인한 뒤 호출. 상태 전이표는
[session_control_srs.md 4.3](./session_control_srs.md) 참조.

### 9.4 `set_task_label` 이 IN_EPISODE 에서 계속 거부됨

**원인**: 설계 의도대로 `IN_EPISODE` 에서는 task 변경이 금지됩니다.

**해결**: 먼저 `stop_episode()` 로 `IN_SESSION` 으로 복귀한 뒤
`set_task_label()` 호출.

### 9.5 `get_session_state` 가 `('', '')` 반환

**원인**: 서비스가 준비되지 않았거나 호출이 타임아웃됨.

**해결**: `is_ready()` 로 선행 확인. `timeout_sec` 를 늘리거나
`get_session_state_async` 를 써서 future 로 원인 추적.

---

## 10. 관련 문서

- [session_control_guide.md](./session_control_guide.md) — 서버 (SessionControlNode) 사용 가이드
- [session_control_srs.md](./session_control_srs.md) — 요구사항 명세서 (상태 전이표 포함)
- [session_control_client.py](../../src/rdfp/rdfp/session/session_control_client.py) — 클라이언트 구현 소스
- [session_control_node.py](../../src/rdfp/rdfp/session/session_control_node.py) — 서버 구현 소스
- [servo_client_programmers_guide.md](../../src/rdfp/rdfp/moveit/docs/servo_client_programmers_guide.md) — 동일한 Node 주입 패턴을 쓰는 `ServoClient` 가이드
