# GripperControlNode Programmer's Guide

Franka Panda 의 gripper(`panda_hand_controller`) 를 안전하고 일관된 서비스
인터페이스로 제어하기 위한 ROS 2 노드입니다.

## 개요

`GripperControlNode` 클래스는 `control_msgs/GripperCommand` 액션을
`std_srvs/Trigger` 서비스로 감싸서, 외부 클라이언트가 액션 프로토콜을 직접
다루지 않고도 gripper 를 open/close 할 수 있게 해주는 **단일 책임 노드**입니다.

### 주요 특징

- **간단한 Trigger 인터페이스**: 두 개의 `~/open_gripper`, `~/close_gripper`
  서비스만 호출하면 됨
- **Non-blocking 동작**: 서비스 호출은 대기 없이 즉시 반환 (goal 큐잉만 수행)
- **Intent 기록**: 요청 의도를 `/teleop/gripper_cmds` 토픽에 별도 발행 —
  로깅 / rosbag 리플레이 / 세션 재생에 활용
- **Feedback 재발행**: 액션 feedback 을 `/teleop/gripper_states` 로 브로드캐스트 —
  여러 구독자가 상태를 공유 가능
- **자가 진단 응답**: 액션 서버 미준비 시 Trigger 응답의 `message` 필드에
  실패 원인 기록
- **Panda 기본값 내장**: open = 0.04 m, close = 0.0 m 로 상수화

## 설치 및 의존성

### 필수 의존성

```bash
# ROS 2 Humble 기본 패키지 (대부분 설치되어 있음)
sudo apt install ros-humble-control-msgs ros-humble-std-srvs

# 본 워크스페이스 메시지 패키지 — 소스에서 빌드
colcon build --packages-select rdfp_msgs

# 본 노드가 포함된 패키지
colcon build --packages-select rdfp
source install/setup.bash
```

### 실행

```bash
# 기본 노드명 'gripper_control' 로 실행
ros2 run rdfp gripper_control_node

# 노드명 remap
ros2 run rdfp gripper_control_node --ros-args -r __node:=my_gripper
```

### Import (Python 클라이언트 측)

```python
# 서비스 호출에 필요한 타입
from std_srvs.srv import Trigger

# (옵션) 상태 토픽 구독
from rdfp_msgs.msg import GripperCommand, GripperState
```

### 노드 인터페이스

| 종류 | 이름 | 타입 | 역할 |
|------|------|------|------|
| Service (server) | `~/open_gripper` | `std_srvs/srv/Trigger` | gripper 열기 요청 |
| Service (server) | `~/close_gripper` | `std_srvs/srv/Trigger` | gripper 닫기 요청 |
| Action (client) | `/panda_hand_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | 실제 구동 요청 |
| Publisher | `/teleop/gripper_cmds` | `rdfp_msgs/msg/GripperCommand` | intent 기록 |
| Publisher | `/teleop/gripper_states` | `rdfp_msgs/msg/GripperState` | feedback 재발행 |

- 서비스는 `~/` prefix 이므로 노드 네임스페이스에 귀속됩니다
  (기본 노드명 `gripper_control` 일 때 `/gripper_control/open_gripper`).
- 토픽은 **전역 경로**로 고정되어 있어 remap 없이도 항상 같은 경로에서
  구독 가능합니다.

## 지원되는 명령 및 위치

### Trigger 서비스 (권장)

| 서비스 | 내부 command 문자열 | 위치 (m) | 용도 |
|--------|--------------------|----------|------|
| `~/open_gripper` | `"open"` | `0.04` | 물체를 놓거나 파지 전 준비 |
| `~/close_gripper` | `"close"` | `0.0` | 물체 파지 |

```python
# 서비스 호출
await open_client.call_async(Trigger.Request())
await close_client.call_async(Trigger.Request())
```

### 직접 Action 호출 (비권장 — 고급 사용자용)

특정 중간 위치(예: `0.02 m` 반개방) 가 필요하면 본 노드를 건너뛰고
`/panda_hand_controller/gripper_cmd` 액션을 직접 호출해야 합니다. 본 노드는
open/close 두 가지 고정 위치만 지원합니다.

```python
# 본 노드를 우회하는 예시
from control_msgs.action import GripperCommand as GripperCommandAction

goal = GripperCommandAction.Goal()
goal.command.position = 0.02  # 반개방
goal.command.max_effort = 0.0
```

### Intent 토픽 (`/teleop/gripper_cmds`)

`rdfp_msgs/msg/GripperCommand` 의 `command` 필드에 `"open"` 또는 `"close"`
문자열이 실립니다. 타임스탬프는 `header.stamp` 에 기록됩니다.

```yaml
header:
  stamp: {sec: 1713600000, nanosec: 0}
  frame_id: ""
command: "open"
```

### 상태 토픽 (`/teleop/gripper_states`)

`rdfp_msgs/msg/GripperState` 로 액션 feedback 을 재발행합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `header.stamp` | `builtin_interfaces/Time` | 재발행 시각 |
| `position` | `float64` | 현재 gripper 위치 [m] |
| `effort` | `float64` | 현재 토크/힘 [N or Nm] |
| `stalled` | `bool` | 힘 한계에 도달해 정지 중 여부 |
| `reached_goal` | `bool` | 목표 위치 도달 여부 |

> `stalled` 와 `reached_goal` 은 드라이버에 따라 항상 기본값(`False`) 이
> 나올 수 있습니다. 본 노드는 `getattr(fb, 'field', default)` 로 누락을
> 허용합니다.

## 기본 사용법

### 0. 노드 기동 확인

먼저 본 노드가 실행되고 있는지 확인합니다.

```bash
ros2 node list | grep gripper_control
ros2 service list | grep gripper_control
# 출력 예:
#   /gripper_control/open_gripper
#   /gripper_control/close_gripper
```

`panda_hand_controller` 가 spawn 되어 있어야 실제 동작이 이루어집니다.

```bash
ros2 action list | grep gripper_cmd
# 출력 예: /panda_hand_controller/gripper_cmd
```

### 1. CLI 에서 직접 호출

가장 간단한 사용법. 쉘에서 바로 open/close 를 테스트할 수 있습니다.

```bash
# 열기
ros2 service call /gripper_control/open_gripper std_srvs/srv/Trigger {}
# 응답 예:
#   success: true
#   message: 'open dispatched'

# 닫기
ros2 service call /gripper_control/close_gripper std_srvs/srv/Trigger {}

# 상태 확인
ros2 topic echo /teleop/gripper_states
```

### 2. Python 클라이언트 기본 패턴

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class GripperDemo(Node):
    def __init__(self) -> None:
        super().__init__('gripper_demo')
        self._open_cli = self.create_client(Trigger, '/gripper_control/open_gripper')
        self._close_cli = self.create_client(Trigger, '/gripper_control/close_gripper')

    def open(self) -> bool:
        if not self._open_cli.service_is_ready():
            self.get_logger().warning('open service not ready')
            return False
        future = self._open_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is None:
            return False
        return future.result().success


def main() -> None:
    rclpy.init()
    node = GripperDemo()
    try:
        if node.open():
            node.get_logger().info('open dispatched')
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

> Trigger 응답의 `success=True` 는 **goal 큐잉 성공**을 의미하지, 실제로
> gripper 가 닫혔다는 의미가 아닙니다. 동작 완료 확인은 다음 절의 상태 토픽
> 구독으로 수행합니다.

### 3. 상태 토픽 구독

실제 동작 완료 여부를 알고 싶을 때는 `/teleop/gripper_states` 를 구독합니다.

```python
from rdfp_msgs.msg import GripperState


class GripperMonitor(Node):
    def __init__(self) -> None:
        super().__init__('gripper_monitor')
        self.create_subscription(
            GripperState, '/teleop/gripper_states', self._on_state, 10,
        )

    def _on_state(self, msg: GripperState) -> None:
        self.get_logger().info(
            f"pos={msg.position:.4f}m effort={msg.effort:.2f} "
            f"stalled={msg.stalled} reached={msg.reached_goal}"
        )
```

### 4. Intent 토픽 기록/리플레이

`/teleop/gripper_cmds` 는 rosbag2 로 녹화하면 사용자의 의도를 나중에 재생할 수
있습니다.

```bash
# 녹화
ros2 bag record /teleop/gripper_cmds

# 재생 (기록된 시점의 command 가 그대로 다시 토픽으로 나감 — 참고용)
ros2 bag play <bag_dir>
```

> 재생만으로는 실제 gripper 가 움직이지 않습니다. 리플레이 스크립트가 토픽을
> 보고 다시 Trigger 서비스를 호출해야 합니다.

## 고급 사용법

### 1. 비동기 연쇄 동작 (open → 이동 → close)

Pick-and-place 시나리오에서 open → 이동 → close 를 순차 수행하려면 각 단계의
완료를 기다려야 합니다.

```python
import asyncio
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rdfp_msgs.msg import GripperState


class PickPlace(Node):
    def __init__(self) -> None:
        super().__init__('pick_place')
        self._open_cli = self.create_client(Trigger, '/gripper_control/open_gripper')
        self._close_cli = self.create_client(Trigger, '/gripper_control/close_gripper')
        self._state_sub = self.create_subscription(
            GripperState, '/teleop/gripper_states', self._on_state, 10,
        )
        self._last_state: GripperState | None = None

    def _on_state(self, msg: GripperState) -> None:
        self._last_state = msg

    async def wait_reached(self, timeout_sec: float = 3.0) -> bool:
        start = self.get_clock().now()
        while rclpy.ok():
            if self._last_state and self._last_state.reached_goal:
                return True
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                return False
            await asyncio.sleep(0.05)
        return False
```

### 2. 서비스 이름 remap

여러 gripper 를 사용하는 환경에서는 노드명을 바꿔 서비스 경로를 분리합니다.

```bash
# 왼팔 gripper
ros2 run rdfp gripper_control_node --ros-args -r __node:=left_gripper

# 오른팔 gripper
ros2 run rdfp gripper_control_node --ros-args -r __node:=right_gripper
```

각각 `/left_gripper/open_gripper`, `/right_gripper/open_gripper` 로 분리됩니다.
단, 액션 경로(`/panda_hand_controller/gripper_cmd`) 와 토픽 경로
(`/teleop/gripper_*`) 는 소스에 하드코딩되어 있으므로 두 노드를 동시에
사용하려면 소스 수정이 필요합니다.

### 3. Launch 파일 통합

`panda_mock.launch.py` 에 본 노드를 함께 띄우는 전형적인 패턴:

```python
from launch_ros.actions import Node

gripper_control = Node(
    package='rdfp',
    executable='gripper_control_node',
    name='gripper_control',
    output='screen',
)
# ld.add_action(gripper_control)
```

`panda_hand_controller` spawn 이후에 기동하도록 `RegisterEventHandler` +
`OnProcessExit` 체인에 연결하는 것이 안전합니다.

### 4. 파라미터화 확장

현재 버전은 상수(`_GRIPPER_OPEN_POSITION`, `_GRIPPER_CLOSE_POSITION`,
`_GRIPPER_MAX_EFFORT`) 가 하드코딩되어 있습니다. 이를 ROS 2 파라미터로
바꾸면 다른 그리퍼에도 재사용 가능합니다.

```python
# 소스 수정 예시
self.declare_parameter('open_position', 0.04)
self.declare_parameter('close_position', 0.0)
self.declare_parameter('max_effort', 0.0)
```

이후 launch 에서:

```python
Node(
    package='rdfp',
    executable='gripper_control_node',
    parameters=[{'open_position': 0.05, 'max_effort': 20.0}],
)
```

## 에러 처리

### 1. 액션 서버 미준비

`panda_hand_controller` 가 아직 spawn 되지 않았거나 죽었을 때.

```python
future = open_cli.call_async(Trigger.Request())
rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)

response = future.result()
if response is None:
    # 서비스 콜 자체가 타임아웃 — 본 노드가 죽어있을 가능성
    pass
elif not response.success:
    # 본 노드는 살아있지만 내부 액션 서버가 ready 하지 않음
    # message 예: "gripper action server '/panda_hand_controller/gripper_cmd' not ready"
    node.get_logger().error(response.message)
```

> 본 노드는 `wait_for_server()` 를 호출하지 않습니다. 즉시 확인 후 실패
> 응답을 반환하므로, 클라이언트는 **반드시 `success` 를 확인**해야 합니다.

### 2. 서비스 자체 미준비

본 노드가 실행되지 않았거나 네임스페이스가 달라서 서비스 자체가 없을 때.

```python
if not open_cli.service_is_ready():
    # ros2 node list 로 gripper_control 이 떠있는지 확인 권장
    node.get_logger().error(
        '/gripper_control/open_gripper not found — is gripper_control_node running?'
    )
    return
```

### 3. 상태 토픽 무응답

서비스 응답은 `success=True` 였지만 `/teleop/gripper_states` 가 일정 시간 내
도달하지 않는 경우.

```python
async def wait_reached(self, timeout_sec: float) -> bool:
    """일정 시간 내 reached_goal 수신 여부 반환."""
    deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=timeout_sec)
    while rclpy.ok() and self.get_clock().now() < deadline:
        if self._last_state and self._last_state.reached_goal:
            return True
        await asyncio.sleep(0.05)
    self.get_logger().warning(
        f'gripper did not reach goal within {timeout_sec:.1f}s'
    )
    return False
```

이 경우 원인은 보통 다음 중 하나입니다:
- 드라이버가 `reached_goal` 필드를 채우지 않음 → 타임아웃 기반으로 재설계
- 물체에 막혀 `stalled` 만 True → `stalled` 도 성공 조건에 포함
- 액션 goal 이 거절됨 → `/panda_hand_controller` 로그 확인

### 4. 잘못된 호출 순서

본 노드는 요청 순서에 상태를 가지지 않으므로 open → open, close → close 연속
호출도 모두 정상 처리됩니다. 다만 물리적으로는 의미가 없을 수 있으니
클라이언트 측에서 상태 관리를 권장합니다.

## 로깅

### 1. 노드 로그

본 노드는 ROS 2 기본 logger 를 사용합니다. `ros2 run` 실행 터미널에 출력됩니다.

| 레벨 | 내용 | 예시 |
|------|------|------|
| `INFO` | 기동 완료 | `GripperControlNode started (services: ~/open_gripper, ~/close_gripper; action: /panda_hand_controller/gripper_cmd)` |
| `INFO` | 명령 디스패치 | `[gripper] open (position=0.040 m)` |
| `WARNING` | 액션 서버 미준비 | `[gripper] open: gripper action server '/panda_hand_controller/gripper_cmd' not ready` |

### 2. 로그 레벨 조정

```bash
# 기동 시 DEBUG 레벨
ros2 run rdfp gripper_control_node --ros-args --log-level debug

# 특정 노드 로그만 변경
ros2 run rdfp gripper_control_node --ros-args \
    --log-level gripper_control:=debug
```

### 3. 로그 분석

```bash
# 런타임 로그 grep
ros2 run rdfp gripper_control_node 2>&1 | grep -E 'gripper|WARN|ERROR'

# ros2 log dir
ls ~/.ros/log/latest/gripper_control/
```

## Best Practices

### 1. 상태 구독을 기본으로 유지

```python
# 권장: 상태 구독을 상시 유지하고, 명령 후 reached_goal 로 완료 확인
self._state_sub = self.create_subscription(
    GripperState, '/teleop/gripper_states', self._on_state, 10,
)
```

### 2. 타임아웃을 항상 설정

```python
# 권장: spin_until_future_complete 에 timeout 지정
rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
if future.result() is None:
    handle_timeout()
```

### 3. 응답 success 체크는 필수

```python
# 권장
resp = future.result()
if resp is None or not resp.success:
    logger.error(f'gripper open failed: {resp.message if resp else "timeout"}')
    return
```

### 4. 서비스 이름은 네임스페이스 인식하여 구성

```python
# 권장: 노드명/네임스페이스를 변수화
GRIPPER_NS = '/gripper_control'
open_cli = self.create_client(Trigger, f'{GRIPPER_NS}/open_gripper')
```

### 5. 재시도 대신 대기

액션 서버가 미준비 상태일 때 즉시 재시도보다 상태 토픽이나
`wait_for_service` 로 준비 상태를 기다리는 편이 안전합니다.

```python
# 권장: 서비스 준비 대기
self._open_cli.wait_for_service(timeout_sec=5.0)
```

## 예제 코드

### 1. 간단한 open → close 시퀀스

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class SimpleSequence(Node):
    def __init__(self) -> None:
        super().__init__('simple_sequence')
        self._open = self.create_client(Trigger, '/gripper_control/open_gripper')
        self._close = self.create_client(Trigger, '/gripper_control/close_gripper')

    def run(self) -> None:
        for cli, label in [(self._open, 'open'), (self._close, 'close')]:
            if not cli.wait_for_service(timeout_sec=3.0):
                self.get_logger().error(f'{label} service not ready')
                return
            future = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            resp = future.result()
            if resp is None or not resp.success:
                self.get_logger().error(f'{label} failed: {resp.message if resp else "timeout"}')
                return
            self.get_logger().info(f'{label}: {resp.message}')


def main() -> None:
    rclpy.init()
    node = SimpleSequence()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Pick-and-place (상태 기반 완료 대기)

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from rdfp_msgs.msg import GripperState


class PickPlace(Node):
    def __init__(self) -> None:
        super().__init__('pick_place')
        self._open = self.create_client(Trigger, '/gripper_control/open_gripper')
        self._close = self.create_client(Trigger, '/gripper_control/close_gripper')
        self._state_sub = self.create_subscription(
            GripperState, '/teleop/gripper_states', self._on_state, 10,
        )
        self._last: GripperState | None = None

    def _on_state(self, msg: GripperState) -> None:
        self._last = msg

    def _wait_done(self, timeout_sec: float) -> bool:
        deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=timeout_sec)
        while rclpy.ok() and self.get_clock().now() < deadline:
            if self._last and (self._last.reached_goal or self._last.stalled):
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def _call(self, cli, label: str) -> bool:
        self._last = None
        future = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        resp = future.result()
        if resp is None or not resp.success:
            self.get_logger().error(f'{label} dispatch failed')
            return False
        return self._wait_done(timeout_sec=3.0)

    def pick(self) -> None:
        self.get_logger().info('open → (이동) → close 시퀀스 시작')
        if not self._call(self._open, 'open'):
            return
        # 여기서 MoveIt2 로 pre-grasp → grasp 이동
        if not self._call(self._close, 'close'):
            return
        self.get_logger().info('pick 완료')


def main() -> None:
    rclpy.init()
    node = PickPlace()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        node.pick()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
```

### 3. Intent 토픽 리플레이

rosbag2 에 녹화된 의도를 다시 재생해 gripper 를 구동합니다.

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rdfp_msgs.msg import GripperCommand


class IntentReplayer(Node):
    """/teleop/gripper_cmds 를 구독해 Trigger 서비스로 중계한다."""

    def __init__(self) -> None:
        super().__init__('intent_replayer')
        self._open = self.create_client(Trigger, '/gripper_control/open_gripper')
        self._close = self.create_client(Trigger, '/gripper_control/close_gripper')
        self.create_subscription(
            GripperCommand, '/teleop/gripper_cmds', self._on_cmd, 10,
        )

    def _on_cmd(self, msg: GripperCommand) -> None:
        cli = self._open if msg.command == 'open' else self._close
        if not cli.service_is_ready():
            self.get_logger().warning(f'service not ready for {msg.command}')
            return
        cli.call_async(Trigger.Request())
        self.get_logger().info(f'replayed: {msg.command}')


def main() -> None:
    rclpy.init()
    node = IntentReplayer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

사용:

```bash
# 터미널 1: 본 노드
ros2 run rdfp gripper_control_node

# 터미널 2: 리플레이어
python3 intent_replayer.py

# 터미널 3: 녹화된 bag 재생
ros2 bag play <bag_dir> --topics /teleop/gripper_cmds
```

## 트러블슈팅

### 자주 발생하는 문제들

#### 1. 서비스 호출은 되는데 gripper 가 안 움직임

**증상**: Trigger 응답 `success=True` / `message="open dispatched"` 지만 실제
gripper 가 움직이지 않음.
**원인**: 액션 서버가 goal 을 받았지만 하드웨어/컨트롤러 문제로 실행되지 않음.
**해결**:
```bash
# 액션 서버 상태 확인
ros2 action info /panda_hand_controller/gripper_cmd -t

# 컨트롤러 상태 확인
ros2 control list_controllers
# panda_hand_controller 가 'active' 인지 확인

# 없거나 inactive 면 재spawn
ros2 run controller_manager spawner panda_hand_controller
```

#### 2. `gripper action server ... not ready`

**증상**: Trigger 응답 `success=False`, `message="gripper action server '/panda_hand_controller/gripper_cmd' not ready"`.
**원인**: 본 노드 기동 시점에 `panda_hand_controller` 가 아직 spawn 되지 않음.
**해결**:
- launch 파일에서 본 노드를 `panda_hand_controller` spawn 완료 이후로 순서화
- 또는 잠시 기다렸다가 재시도

#### 3. 서비스 자체가 없음

**증상**: `ros2 service list` 에 `/gripper_control/open_gripper` 가 없음.
**해결**:
```bash
# 노드가 살아있는지
ros2 node list | grep gripper

# 없으면 기동
ros2 run rdfp gripper_control_node

# 네임스페이스가 다를 때
ros2 node info /<actual_name>
```

#### 4. `/teleop/gripper_states` 가 안 나옴

**증상**: 서비스 호출은 성공하지만 상태 토픽이 조용함.
**원인**: 드라이버가 feedback 을 발행하지 않거나, 동작이 너무 빨라 feedback
주기 전에 끝남.
**해결**:
```bash
# 액션 feedback 이 발행되는지 직접 확인
ros2 topic echo /panda_hand_controller/gripper_cmd/_action/feedback

# 아무것도 안 나오면 드라이버 측 문제
```

#### 5. 메시지 타입 임포트 에러

**증상**: `ModuleNotFoundError: No module named 'rdfp_msgs'`.
**해결**:
```bash
colcon build --packages-select rdfp_msgs
source install/setup.bash
```

#### 6. 다중 gripper 노드 이름 충돌

**증상**: 두 번째 노드 기동 시 이름 충돌로 종료.
**해결**:
```bash
ros2 run rdfp gripper_control_node --ros-args -r __node:=gripper_control_2
```

### 디버깅 팁

```python
# 1. 로그 레벨을 DEBUG 로
# 실행 시 --ros-args --log-level debug

# 2. 액션 goal 직접 호출로 본 노드 우회 테스트
# ros2 action send_goal /panda_hand_controller/gripper_cmd \
#     control_msgs/action/GripperCommand "{command: {position: 0.04, max_effort: 0.0}}"

# 3. 토픽 흐름 실시간 확인
# ros2 topic hz /teleop/gripper_states
# ros2 topic bw /teleop/gripper_states

# 4. rqt_graph 로 연결 관계 시각화
# ros2 run rqt_graph rqt_graph
```

## 결론

`GripperControlNode` 는 Panda gripper 제어의 **얇은 래퍼**로서, 복잡한
액션 프로토콜을 단순한 Trigger 서비스로 감싸고, 상태를 별도 토픽으로
브로드캐스트해 여러 구독자가 재사용할 수 있게 합니다. Non-blocking 설계와
state-based 완료 대기 패턴을 함께 사용하면 안정적인 pick-and-place 시퀀스를
구성할 수 있습니다.

추가 질문이나 기능 제안이 있다면 본 저장소 이슈로 보고하거나
[`keyboard_twist_teleop.py`](../teleop/keyboard_twist_teleop.py) 의 호출 예를
참고하시기 바랍니다.
