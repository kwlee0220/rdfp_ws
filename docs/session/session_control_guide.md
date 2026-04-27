# SessionControlNode Programmer's Guide

이 문서는 `rdfp.session.session_control_node.SessionControlNode` 를 **사용하는
프로그래머** 관점에서 작성되었습니다. 내부 구현 세부사항이 아니라, 이 노드를
실행하고 서비스/토픽으로 통신하는 외부 클라이언트가 알아야 할 사항을 다룹니다.

상세한 요구사항 명세는 [session_control_srs.md](./session_control_srs.md),
소스 코드는 [session_control_node.py](../../src/rdfp/rdfp/session/session_control_node.py) 를 참고하세요.

---

## 1. 개요

`SessionControlNode` 는 로봇 실험의 **세션(session)** 과 **에피소드(episode)**
생명주기를 제어하는 상태 머신 노드입니다.

- **입력 (Services)**:
  - `start_session`, `stop_session`, `start_episode`, `stop_episode` — `std_srvs/srv/Trigger`
  - `set_task_label` — `rdfp_msgs/srv/SetString`
  - `get_session_state` — `rdfp_msgs/srv/GetSessionState`
- **출력 (Topic)**: `session` — 상태 변경 알림 (`rdfp_msgs/msg/SessionCommand`)
- **노드 이름**: `session_control`
- **서비스/토픽 기본 경로**: root (`/start_session`, `/session` 등) — 다중 인스턴스
  분리가 필요하면 [3.4절 Namespace 를 이용한 분리 운영](#34-namespace-를-이용한-분리-운영) 을 참고하세요.
- **실행 엔트리포인트**: `ros2 run rdfp session_control_node`

### 왜 이 노드를 써야 하는가?

- **단일 진실원천(Single Source of Truth)**: 여러 recorder/모니터가 동일한
  세션 상태를 공유할 수 있습니다.
- **Late-join 구독자 지원**: TRANSIENT_LOCAL QoS 로 나중에 붙은 구독자도
  직전 상태를 즉시 수신합니다.
- **단일 스레드 실행**: 서비스 콜백 간 race condition 이 없어 디버깅이
  쉽습니다.
- **명령별 분할 서비스**: 각 명령이 독립된 서비스로 노출되므로 CLI 에서
  호출하기 쉽고, 서비스 타입이 타이트해 잘못된 요청을 컴파일/런타임 단계에서
  조기에 잡을 수 있습니다.

---

## 2. 상태 머신

### 2.1 상태 정의

| 상태 | 의미 |
|---|---|
| `IDLE` | 세션이 시작되지 않은 초기 상태 |
| `IN_SESSION` | 세션 진행 중, 에피소드는 진행 중이 아님 |
| `IN_EPISODE` | 세션 내에서 에피소드가 진행 중 |

### 2.2 상태 다이어그램

```
        start_session            start_episode
  ┌──────────────────────▶ ┌──────────────────▶
IDLE                    IN_SESSION           IN_EPISODE
  ◀──────────────────────┘ ◀──────────────────┘
        stop_session              stop_episode
                                       │
                                       │ stop_session
                                       ▼
                                    (IDLE)
```

- `IN_EPISODE` 에서 `stop_session` 을 받으면 **한 번의 서비스 호출로**
  `IN_EPISODE → IN_SESSION → IDLE` 의 2단계 전이를 수행합니다. 토픽은
  `(IN_SESSION, <label>)` → `(IDLE, <label>)` 순서로 **두 번** 발행됩니다.

### 2.3 서비스별 허용 상태

| 서비스 | 허용 상태 | 비고 |
|---|---|---|
| `start_session` | `IDLE` | — |
| `stop_session` | `IN_SESSION`, `IN_EPISODE` | `IN_EPISODE` 에서 호출 시 에피소드를 먼저 종료한 뒤 세션을 종료 |
| `start_episode` | `IN_SESSION` | — |
| `stop_episode` | `IN_EPISODE` | — |
| `set_task_label` | `IDLE`, `IN_SESSION` | `IN_EPISODE` 에서는 **거부** |

> **주의**: `IN_EPISODE` 상태에서는 `set_task_label` 가 **거부**됩니다. 에피소드
> 진행 중에는 task label 이 바뀌어서는 안 되기 때문입니다.

---

## 3. 실행

### 3.1 빌드

```bash
# rdfp_msgs (서비스/메시지 정의) → rdfp (노드 구현) 순서로 빌드
colcon build --packages-select rdfp_msgs
colcon build --packages-select rdfp
source install/setup.bash
```

### 3.2 노드 기동

```bash
# 단독 실행
ros2 run rdfp session_control_node

# launch 파일로 실행
ros2 launch rdfp rdfp.launch.py
```

노드 이름은 `session_control` 이며, 서비스/토픽은 기본적으로 **root 네임스페이스
(`/`)** 에 노출됩니다. 즉 실제 경로는 `/start_session`, `/session` 과 같이 prefix
없이 드러납니다. namespace 를 부여해 `/<ns>/session` 형태로 운영하려면
[3.4절](#34-namespace-를-이용한-분리-운영) 을 참고하세요.

### 3.3 실행 여부 확인

```bash
ros2 node list | grep session_control
# 서비스/토픽은 기본적으로 root 에 노출되므로 노드 이름으로 필터되지 않는다.
ros2 service list | grep -E '(start|stop)_(session|episode)|_task_label|_session_state'
ros2 topic list | grep -E '^/session$'
```

### 3.4 Namespace 를 이용한 분리 운영

dual-arm 구성이나 다중 로봇 환경에서는 `SessionControlNode` 를 여러 개 띄워야
할 수 있습니다. 이 경우 실행 시 namespace 를 부여해 **서비스/토픽 경로를 물리적으로
분리**합니다.

#### CLI 로 실행

```bash
ros2 run rdfp session_control_node --ros-args -r __ns:=/session_control
```

#### launch 파일로 실행

```python
Node(
    package='rdfp',
    executable='session_control_node',
    namespace='session_control',
    name='session_control',
)
```

이렇게 실행하면 모든 서비스/토픽 경로 앞에 `/session_control/` 이 붙어
`/session_control/start_session`, `/session_control/session` 등으로 노출됩니다.
복수 인스턴스가 필요한 경우에는 각각 다른 namespace (`/left_arm`, `/right_arm` 등)
를 부여하세요.

> 본 문서의 이후 예제들은 **default (root) 경로** 를 기준으로 작성되었습니다.
> namespace 를 부여해 운영하는 경우, 예제의 경로 앞에 `/<ns>` 를 붙여 읽으세요.
> (예: `/start_session` → `/session_control/start_session`)

---

## 4. 명령 ↔ 서비스 매핑

| 명령 | 서비스 | 타입 | 요청 본문 |
|---|---|---|---|
| 세션 시작 | `start_session` | `std_srvs/srv/Trigger` | (없음) |
| 세션 종료 | `stop_session` | `std_srvs/srv/Trigger` | (없음) |
| 에피소드 시작 | `start_episode` | `std_srvs/srv/Trigger` | (없음) |
| 에피소드 종료 | `stop_episode` | `std_srvs/srv/Trigger` | (없음) |
| task label 설정 | `set_task_label` | `rdfp_msgs/srv/SetString` | `task_label: string` |

### 규칙

- `set_task_label` 의 `task_label` 은 알파벳 대소문자, 숫자, 언더스코어(`_`)
  조합을 가정합니다.
- `set_task_label(task_label="")` (빈 문자열) 은 **task clear** 를 의미하여
  기존 `task_label` 이 `""`로 초기화됩니다.
- 허용되지 않은 상태에서 서비스를 호출하면 `success=false` /
  `message='invalid command'` 로 거부되며 상태는 변하지 않습니다.

---

## 5. 인터페이스

### 5.1 제어 서비스 (Trigger 4종)

**타입**: `std_srvs/srv/Trigger`

```
                        # Request: 필드 없음
---
bool success            # Response
string message
```

4 개 서비스 (`start_session`, `stop_session`, `start_episode`, `stop_episode`)
모두 동일한 Trigger 타입을 사용합니다.

| 필드 | 설명 |
|---|---|
| `success` | 현재 상태에서 명령이 유효하면 `true`, 아니면 `false` |
| `message` | 성공 시 `""`, 실패 시 `"invalid command"` |

### 5.2 `set_task_label` (service)

**타입**: `rdfp_msgs/srv/SetString`

```
string task_label       # Request
---
bool success            # Response
string message
```

| 필드 | 타입 | 설명 |
|---|---|---|
| `task_label` | `string` | 설정할 task label. `""` 이면 task clear |
| `success` | `bool` | `IN_EPISODE` 가 아니면 `true`, 그 외에는 `false` |
| `message` | `string` | 성공 시 `""`, 실패 시 `"invalid command"` |

### 5.3 `get_session_state` (service)

**타입**: `rdfp_msgs/srv/GetSessionState`

```
                        # Request: 필드 없음
---
string state            # Response
string task_label
```

내부 상태를 변경하지 않고 현재 값을 조회합니다.

### 5.4 `session` (topic)

**타입**: `rdfp_msgs/msg/SessionCommand`

```
std_msgs/Header header
string state
string task_label
```

| 필드 | 설명 |
|---|---|
| `header.stamp` | 메시지 발행 시점의 타임스탬프 |
| `header.frame_id` | 항상 `""` |
| `state` | 전이 후의 세션 상태 (`IDLE` / `IN_SESSION` / `IN_EPISODE`) |
| `task_label` | 전이 후의 task label |

#### QoS (Publisher 측)

| 항목 | 값 |
|---|---|
| `reliability` | `RELIABLE` |
| `durability` | `TRANSIENT_LOCAL` |
| `history depth` | `1` |

늦게 붙은 구독자도 **직전 상태를 즉시 수신**할 수 있도록 설계되었습니다.
구독자는 호환되는 QoS 로 연결해야 하며, 보통 동일한 조합을 사용합니다.

**중요**: 서비스 응답에는 `state`/`task_label` 이 포함되지 않으므로, 클라이언트가
현재 상태를 알아야 한다면 반드시 이 토픽을 구독하거나 `get_session_state` 를
호출해야 합니다.

---

## 6. 클라이언트 예제

### 6.1 CLI 로 빠르게 확인

```bash
# 세션 시작
ros2 service call /start_session std_srvs/srv/Trigger "{}"

# task label 설정
ros2 service call /set_task_label rdfp_msgs/srv/SetString \
  "{task_label: 'pick_and_place'}"

# task clear
ros2 service call /set_task_label rdfp_msgs/srv/SetString \
  "{task_label: ''}"

# 에피소드 시작/종료
ros2 service call /start_episode std_srvs/srv/Trigger "{}"
ros2 service call /stop_episode std_srvs/srv/Trigger "{}"

# 세션 종료
ros2 service call /stop_session std_srvs/srv/Trigger "{}"

# 현재 상태 조회
ros2 service call /get_session_state \
  rdfp_msgs/srv/GetSessionState "{}"

# 토픽 모니터링 (QoS 플래그 필수)
ros2 topic echo /session rdfp_msgs/msg/SessionCommand \
  --qos-durability transient_local \
  --qos-reliability reliable
```

### 6.2 Python 클라이언트

```python
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from rdfp_msgs.srv import GetSessionState, SetString


class SessionController(Node):
    """SessionControlNode 에 대한 간단한 Python 클라이언트 래퍼."""

    def __init__(self) -> None:
        super().__init__('session_controller')
        self._start_session_cli = self.create_client(
            Trigger, '/start_session'
        )
        self._stop_session_cli = self.create_client(
            Trigger, '/stop_session'
        )
        self._start_episode_cli = self.create_client(
            Trigger, '/start_episode'
        )
        self._stop_episode_cli = self.create_client(
            Trigger, '/stop_episode'
        )
        self._set_task_cli = self.create_client(
            SetString, '/set_task_label'
        )
        self._state_cli = self.create_client(
            GetSessionState, '/get_session_state'
        )
        for cli, name in (
            (self._start_session_cli, 'start_session'),
            (self._stop_session_cli, 'stop_session'),
            (self._start_episode_cli, 'start_episode'),
            (self._stop_episode_cli, 'stop_episode'),
            (self._set_task_cli, 'set_task_label'),
            (self._state_cli, 'get_session_state'),
        ):
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'waiting for {name} ...')

    def _call_trigger(self, cli, label: str) -> bool:
        future = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if not resp.success:
            self.get_logger().warning(
                f"command '{label}' rejected: {resp.message}"
            )
        return resp.success

    def start_session(self) -> bool:
        return self._call_trigger(self._start_session_cli, 'start_session')

    def stop_session(self) -> bool:
        return self._call_trigger(self._stop_session_cli, 'stop_session')

    def start_episode(self) -> bool:
        return self._call_trigger(self._start_episode_cli, 'start_episode')

    def stop_episode(self) -> bool:
        return self._call_trigger(self._stop_episode_cli, 'stop_episode')

    def set_task_label(self, task_label: str) -> bool:
        req = SetString.Request(task_label=task_label)
        future = self._set_task_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if not resp.success:
            self.get_logger().warning(
                f"set_task_label('{task_label}') rejected: {resp.message}"
            )
        return resp.success

    def query(self) -> GetSessionState.Response:
        future = self._state_cli.call_async(GetSessionState.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main() -> None:
    rclpy.init()
    ctrl = SessionController()
    try:
        ctrl.set_task_label('pick_and_place')
        ctrl.start_session()
        ctrl.start_episode()
        # ... 실험 수행 ...
        ctrl.stop_episode()
        ctrl.stop_session()

        state = ctrl.query()
        ctrl.get_logger().info(
            f'final state={state.state} task_label={state.task_label!r}'
        )
    finally:
        ctrl.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 6.3 토픽 구독자 (recorder 예시)

```python
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from rdfp_msgs.msg import SessionCommand


class SessionSubscriber(Node):
    """session 토픽을 구독해 상태 변경에 반응하는 예시."""

    def __init__(self) -> None:
        super().__init__('session_subscriber')
        # publisher 와 동일한 QoS 를 사용해야 매칭된다.
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            SessionCommand, '/session', self._on_msg, qos
        )

    def _on_msg(self, msg: SessionCommand) -> None:
        self.get_logger().info(
            f'state={msg.state} task_label={msg.task_label!r} '
            f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
        )
        # 여기서 상태에 따라 recorder 동작을 제어 (예: IN_EPISODE 로 진입하면
        # 녹화 시작, IN_SESSION 으로 복귀하면 녹화 종료 등).


def main() -> None:
    rclpy.init()
    node = SessionSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 7. 토픽 발행 순서 주의

`IN_EPISODE` 에서 `stop_session` 을 호출하면 구독자는 **한 번의 서비스 호출에
대해 두 개의 토픽 메시지** 를 받습니다.

```
(호출)   /stop_session
(발행 1) /session → state='IN_SESSION', task_label='<L>'
(발행 2) /session → state='IDLE',       task_label='<L>'
(응답)   Trigger.Response       → success=true, message=''
```

구독자는 이 순서를 활용해 "에피소드 종료 후처리 → 세션 종료 후처리" 와 같은
2단계 정리 작업을 구분해서 실행할 수 있습니다. 예를 들어 recorder 는 먼저
에피소드 파일을 닫고, 이어서 세션 디렉토리를 마무리할 수 있습니다.

---

## 8. 오류 처리

### 8.1 거부되는 경우

- 현재 상태에서 허용되지 않는 서비스를 호출한 경우 (예: `IDLE` 에서 `stop_session`)
- `IN_EPISODE` 에서 `set_task_label` 을 호출한 경우

이때 응답은 `success=false`, `message='invalid command'` 이며 노드의 상태와
`task_label` 은 **그대로 유지**되고 토픽은 **발행되지 않습니다**. 노드의
로그에는 `warning` 레벨로 기록됩니다.

```
[WARN] [session_control]: invalid command 'stop_session' in state IDLE
```

### 8.2 서비스 호출 자체 실패

네트워크/타이밍 문제로 서비스 호출이 실패하면 `call_async()` 의 future 가
`None` 또는 예외로 완료됩니다. 일반적인 `rclpy` 서비스 클라이언트 오류
처리와 동일하게 다루면 됩니다.

---

## 9. 로깅

| 이벤트 | 레벨 | 예시 |
|---|---|---|
| 노드 시작 | `info` | `SessionControlNode started (state=IDLE, task_label='')` |
| 상태 전이 | `info` | `state transition: IDLE -> IN_SESSION (command=start_session)` |
| `set_task_label` 처리 | `info` | `task_label set to 'pick_and_place'` |
| 거부된 명령 | `warning` | `invalid command 'start_episode' in state IDLE` |

로그 메시지는 영어로 작성되어 있으며, 로그 레벨을 조정하려면 ROS2 표준
`--ros-args --log-level session_control:=debug` 옵션을 사용하세요.

---

## 10. 동시성 / 수명 주기

- **실행기**: `SingleThreadedExecutor` 를 사용합니다. 따라서 서비스 콜백 간
  race condition 이 없고, 클라이언트 입장에서는 서비스 호출이 항상 순차적
  으로 처리된다고 가정할 수 있습니다.
- **종료**: Ctrl+C (SIGINT) 수신 시 `destroy_node()` → `rclpy.try_shutdown()`
  경로로 깨끗하게 종료됩니다. 종료 시점에 마지막 상태는 저장되지 **않으며**
  (재시작 시 상태는 항상 `IDLE`, `task_label=''` 로 초기화됨), 영속성이
  필요하면 호출자가 직접 관리해야 합니다.
- **다중 클라이언트**: 여러 클라이언트가 동시에 서비스를 호출할 수 있으며,
  각 호출은 순차적으로 처리됩니다. 다만 "현재 상태 조회 → 명령 전송" 사이에
  다른 클라이언트가 끼어들 수 있으므로, 이런 race 가 문제가 된다면 상위
  레벨에서 조율해야 합니다.

---

## 11. 범위 밖 기능

다음은 이 노드의 책임이 **아닙니다**.

- 세션/에피소드 데이터의 실제 기록 (recorder 측 책임)
- 명령 이력 저장/재생
- 다중 세션 동시 관리
- 권한/인증
- 명령 큐잉 (모든 명령은 즉시 동기 처리)
- ROS2 lifecycle node 인터페이스 (본 노드는 일반 `Node`)
- 재시작 후 상태 복원 (시작 시 항상 `IDLE`)

이러한 기능이 필요하다면 `SessionControlNode` 위에 별도의 상위 노드/서비스를
구축해서 조합하는 것을 권장합니다.

---

## 12. 관련 문서

- [session_control_srs.md](./session_control_srs.md) — 소프트웨어 요구사항 명세서
- [session_control_node.py](../../src/rdfp/rdfp/session/session_control_node.py) — 노드 구현 소스
- [../../src/rdfp/README.md](../../src/rdfp/README.md) — `rdfp` 패키지 전체 README (세션 제어 노드 섹션 포함)
