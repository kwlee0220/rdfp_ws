"""Isaac 의 물체 TF → ``rdfp_msgs/SceneObjects`` 변환 노드.

Isaac 은 scene 물체의 pose 를 **TF 로** 내보내고(`ROS2PublishTransformTree`), 이 노드가
그것을 `/scene/objects` 계약으로 바꾼다. `mock_scene_state_node` 가 MoveIt planning
scene 을 폴링해 같은 토픽을 내는 것과 같은 자리다.

**TF 를 경유하는 것이 이 설계의 핵심이다.** `SceneObjects` 는 pose 를 로봇 베이스
(`panda_link0`) 기준으로 요구하는데 Isaac 은 world 기준이고, 게다가 Isaac 의
쿼터니언은 wxyz(스칼라 우선)다. TF 를 타면 `lookup_transform` 한 번으로 **좌표 변환과
쿼터니언 규약이 동시에 해결된다** — 손으로 뒤집다 틀릴 자리가 사라진다. 순서를 틀려도
norm 이 1 이라 어떤 검사도 통과하고 결과가 '그럴듯하게 틀린 자세'라, 애초에 그 코드를
안 쓰는 편이 낫다 (`rdfp_msgs/SceneObject.msg` 의 경고를 참고한다).

물체의 **이름·종류·크기는 TF 에 없다.** 그래서 시뮬레이터 쪽 생성 스크립트와 **같은
JSON**(``config/isaac_scene.json``)을 읽는다. 두 곳에 적으면 조용히 어긋난다.

쓰기 경로 — ``/scene/reset``
---------------------------

물체를 다시 놓는 것은 **시뮬레이터 안에서 USD 스테이지를 써야** 하고, 이 노드는 그럴 수
없다. 대신 Isaac 이 ``isaacsim.ros2.sim_control`` 확장으로 여는 **표준 서비스**
(`simulation_interfaces`)를 부른다.

    /scene/reset (rdfp_msgs/ResetScene)   ← 트윈. mock 과 **같은 계약**이다
        -> 물체마다 /set_entity_state (simulation_interfaces)
        -> 읽어서 확인 /get_entity_state

**응답을 믿지 않고 읽어서 확인한다.** 실측(2026-09-02)에서 `result=1` 이고 메시지도
`Successfully set state ...` 인데 **pose 가 그대로인 경우**가 있었다(rigid body 가 아닌
prim). 성공 보고와 실제 적용이 어긋나므로 판정은 읽기로 한다.

**Isaac 은 물체를 만들거나 지우지 않는다.** 스테이지 구성은 `setup_scene.py` 가 정하고
이 서비스는 **다시 놓기**만 한다. mock 이 planning scene 을 통째로 갈아끼우는 것과
다르며, 그래서 모르는 이름이나 빈 배열은 성공시키지 않고 **거부한다** — 조용히 넘기면
"배치했다"는 거짓이 데이터셋에 남는다.
"""

from __future__ import annotations

from typing import Any, Optional

import json
import math
import os
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from ament_index_python.packages import get_package_share_directory
from rdfp_msgs.msg import SceneObject, SceneObjects
from rdfp_msgs.srv import ResetScene
from tf2_ros import Buffer, TransformListener

from robot_control.ros2_utils import get_parameter, parse_stripped_str

# `simulation_interfaces` 는 Isaac 의 sim_control 확장이 쓰는 **표준 패키지**이며
# apt 로 따로 깔아야 한다(`ros-humble-simulation-interfaces`). 없으면 발행은 그대로
# 하고 `/scene/reset` 만 열지 않는다 — 이 노드의 원래 일(관측)까지 막을 이유는 없다.
try:
    from simulation_interfaces.srv import GetEntityState, SetEntityState
except ImportError:                                   # pragma: no cover - 환경 의존
    GetEntityState = None
    SetEntityState = None

_DEFAULT_SCENE_TOPIC = '/scene/objects'
_DEFAULT_BASE_FRAME = 'panda_link0'
_DEFAULT_CONFIG_RELPATH = os.path.join('config', 'isaac_scene.json')
_DEFAULT_RESET_SERVICE = '/scene/reset'
# Isaac 의 sim_control 확장이 여는 이름. `SERVICE_PREFIX` 가 빈 문자열이라 루트다.
_SET_ENTITY_STATE = '/set_entity_state'
_GET_ENTITY_STATE = '/get_entity_state'
# 서비스 콜백 안에서 기다리는 한도.
_ENTITY_TIMEOUT_SEC = 5.0
# 읽어서 확인할 때의 허용 오차. Isaac 이 float32 로 돌려주므로 1e-3 이면 넉넉하다.
_POSITION_TOLERANCE_M = 1e-3
_ORIENTATION_TOLERANCE = 1e-3
# 중력 가속도. 재생 중에는 **놓자마자 떨어지기 시작하므로** 왕복 시간만큼의 자유낙하를
# 허용 오차에 더한다. 30 ms 면 4.4 mm 라 고정 1 mm 로는 정상 배치도 실패로 읽힌다
# (실측 2026-09-02: 공중 배치가 그렇게 거부됐다).
_GRAVITY_M_S2 = 9.81

# `SceneObjects.msg` 가 권장하는 QoS. **구독 측도 맞춰야 한다** — volatile 로
# 구독하면 매칭 자체가 되지 않아 값이 영영 오지 않는다.
_SCENE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _drift_allowance(elapsed_sec: float) -> float:
    """왕복 `elapsed_sec` 동안 물체가 정당하게 움직일 수 있는 거리.

    **재생 중에는 놓는 즉시 물리가 작용한다.** 지지면 없이 놓인 물체는 자유낙하하므로,
    쓰고 읽는 사이의 시간만큼 목표에서 벗어나 있는 것이 정상이다. 그것을 실패로 읽으면
    공중 배치가 늘 거부된다.
    """
    return 0.5 * _GRAVITY_M_S2 * elapsed_sec * elapsed_sec


def _same_pose(actual: Any, wanted: Any, elapsed_sec: float = 0.0) -> bool:
    """두 pose 가 같은가. 위치와 자세를 **둘 다** 본다.

    위치만 보면 자세만 조용히 무시되는 경우를 놓친다. 쿼터니언은 `q` 와 `-q` 가 같은
    회전이므로 **내적의 절대값**으로 비교한다 — 부호를 그대로 견주면 같은 자세를
    다르다고 판정한다.

    `elapsed_sec` 은 쓰고 읽는 사이의 시간이며, 그동안의 자유낙하를 허용한다. 0 이면
    정지 상태(또는 시뮬레이션 정지)를 가정한 엄격한 비교다.
    """
    dx = actual.position.x - wanted.position.x
    dy = actual.position.y - wanted.position.y
    dz = actual.position.z - wanted.position.z
    limit = _POSITION_TOLERANCE_M + _drift_allowance(elapsed_sec)
    if math.sqrt(dx * dx + dy * dy + dz * dz) > limit:
        return False
    a, b = actual.orientation, wanted.orientation
    dot = abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w)
    return dot >= 1.0 - _ORIENTATION_TOLERANCE


class IsaacSceneStateNode(Node):
    """Isaac 물체 TF 를 읽어 `/scene/objects` 로 발행한다."""

    def __init__(self) -> None:
        super().__init__('isaac_scene_state')

        self.declare_parameter('scene_topic', _DEFAULT_SCENE_TOPIC)
        self.declare_parameter('base_frame', _DEFAULT_BASE_FRAME)
        self.declare_parameter('config_file', '')
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('warn_interval_sec', 5.0)

        scene_topic = get_parameter(self, 'scene_topic', parse_stripped_str)
        self._base_frame = get_parameter(self, 'base_frame', parse_stripped_str)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self._warn_interval = float(self.get_parameter('warn_interval_sec').value)

        self.declare_parameter('reset_service', _DEFAULT_RESET_SERVICE)
        reset_service = get_parameter(self, 'reset_service', parse_stripped_str)

        self._objects = self._load_objects()
        self._by_name = {o['name']: o for o in self._objects}

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._publisher = self.create_publisher(SceneObjects, scene_topic, _SCENE_QOS)
        self.create_timer(1.0 / max(publish_rate, 0.1), self._on_timer)

        # 서비스 콜백 안에서 다른 서비스를 기다리므로 **콜백 그룹을 나눈다.**
        # 같은 그룹이면 응답 콜백이 이 콜백 뒤에 줄을 서서 영영 오지 않는다.
        # `main` 이 MultiThreadedExecutor 를 쓰는 이유도 이것이다.
        if SetEntityState is None:
            self._reset_srv = None
            self._set_cli = None
            self._get_cli = None
            self.get_logger().error(
                "'simulation_interfaces' is not installed — /scene/reset stays closed. "
                'Install it with: sudo apt install ros-humble-simulation-interfaces')
        else:
            self._set_cli = self.create_client(
                SetEntityState, _SET_ENTITY_STATE,
                callback_group=MutuallyExclusiveCallbackGroup())
            self._get_cli = self.create_client(
                GetEntityState, _GET_ENTITY_STATE,
                callback_group=MutuallyExclusiveCallbackGroup())
            self._reset_srv = self.create_service(
                ResetScene, reset_service, self._on_reset,
                callback_group=MutuallyExclusiveCallbackGroup())
            self.get_logger().info(
                f'  reset: {reset_service} -> {_SET_ENTITY_STATE} (root {self._root_prim!r})')

        self._missing_logged: dict[str, float] = {}
        self._last_heartbeat = 0.0

        self.get_logger().info(
            f'IsaacSceneStateNode started: {len(self._objects)} object(s) -> {scene_topic}')
        self.get_logger().info(f'  base frame: {self._base_frame}')
        self.get_logger().info(
            f'  objects: {[obj["name"] for obj in self._objects]} (dynamic only)')

    def _config_path(self) -> str:
        """물체 정의 JSON 경로. 지정이 없으면 패키지 share 에서 찾는다."""
        # `parse_stripped_str` 을 쓰지 않는다 — 그 검증기는 빈 문자열을 거부하는데,
        # 여기서는 **빈 값이 "지정 안 함"이라는 유효한 뜻**이다.
        explicit = str(self.get_parameter('config_file').value or '').strip()
        if explicit:
            return explicit
        return os.path.join(get_package_share_directory('robot_control'),
                            _DEFAULT_CONFIG_RELPATH)

    def _load_objects(self) -> list[dict[str, Any]]:
        """`dynamic: true` 인 물체만 싣는다 — `/scene/objects` 는 조작 대상 채널이다.

        `dynamic: false` 인 것(탁자 등)은 **시뮬레이터에는 그대로 존재한다** —
        `setup_scene.py` 가 같은 JSON 으로 prim 을 만든다. 여기서 빼는 것은 발행뿐이다.

        기존 `dynamic` 플래그를 그대로 쓰고 구분자를 새로 만들지 않는다. 물리적으로
        움직이지 않는 물체는 pose 가 변하지 않아 상태 채널에 실을 값이 없고, 그래서
        "rigid body 인가"와 "조작 대상인가"가 이 씬에서 같은 집합이 된다.
        """
        path = self._config_path()
        with open(path, encoding='utf-8') as f:
            config = json.load(f)
        self.get_logger().info(f'  scene config: {path}')
        # prim 경로 조립에 쓴다 — `setup_scene.py` 가 `f"{root_prim}/{name}"` 로 만든다.
        self._root_prim = str(config.get('root_prim', '/World/Scene')).rstrip('/')
        return [o for o in config['objects'] if o.get('dynamic', False)]

    def _lookup(self, frame: str) -> Optional[Any]:
        """물체 프레임을 베이스 프레임 기준으로 조회한다."""
        try:
            return self._buffer.lookup_transform(
                self._base_frame, frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.0))
        except Exception as exc:  # tf2 예외 계열이 여럿이라 통째로 잡는다
            now = self.get_clock().now().nanoseconds * 1e-9
            # `-inf` 기본값이라야 **첫 번째는 반드시 남는다.** 0.0 이면
            # sim time 이 0 에서 시작한 직후 warn_interval 동안 삼켜지는데,
            # Isaac 은 Stop 마다 시계가 0 으로 되돌아가므로 늘 그 구간을 지난다.
            last = self._missing_logged.get(frame, float('-inf'))
            if now - last >= self._warn_interval:
                self._missing_logged[frame] = now
                self.get_logger().warning(
                    f"no transform '{self._base_frame}' -> '{frame}': "
                    f'{type(exc).__name__}. Is the Isaac scene TF graph running?')
            return None

    def _on_timer(self) -> None:
        msg = SceneObjects()
        # stamp 를 비우면 적재 시 epoch 0 으로 들어가 **어느 에피소드에도 속하지
        # 못한다** (SceneObjects.msg 의 경고).
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame

        for spec in self._objects:
            transform = self._lookup(spec.get('frame', spec['name']))
            if transform is None:
                continue
            obj = SceneObject()
            obj.name = spec['name']
            obj.type = spec['type']
            obj.dimensions = [float(v) for v in spec.get('dimensions', [])]
            obj.pose.position.x = transform.transform.translation.x
            obj.pose.position.y = transform.transform.translation.y
            obj.pose.position.z = transform.transform.translation.z
            # tf2 가 이미 ROS 규약(xyzw)으로 준다. 여기서 뒤집지 않는다.
            obj.pose.orientation = transform.transform.rotation
            msg.objects.append(obj)

        self._publisher.publish(msg)
        self._heartbeat(len(msg.objects))

    # ── 쓰기 (/scene/reset) ─────────────────────────────────────

    def _entity_path(self, name: str) -> str:
        """물체 이름 → USD prim 절대 경로. `setup_scene.py` 와 같은 규칙이다."""
        return f'{self._root_prim}/{name}'

    def _on_reset(self, request: ResetScene.Request,
                  response: ResetScene.Response) -> ResetScene.Response:
        """요청한 배치대로 물체를 **다시 놓는다.**

        mock 과 달리 **만들지도 지우지도 않는다.** Isaac 의 스테이지 구성은
        `setup_scene.py` 가 정하므로, 여기서 할 수 있는 것은 이미 있는 물체를 옮기는
        것뿐이다. 그래서 모르는 이름과 빈 배열을 **거부한다** — 성공으로 돌려주면
        "그 배치로 놓았다"는 거짓이 에피소드 metadata 에 남는다.
        """
        if not request.objects:
            return self._fail(response,
                              'Isaac cannot clear the scene; it can only reposition '
                              'objects defined in isaac_scene.json')
        if not self._set_cli.service_is_ready() or not self._get_cli.service_is_ready():
            return self._fail(response,
                              f"'{_SET_ENTITY_STATE}' not ready — is Isaac running with "
                              'the isaacsim.ros2.sim_control extension enabled?')

        for obj in request.objects:
            problem = self._check_known(obj)
            if problem is not None:
                return self._fail(response, problem)

        for obj in request.objects:
            problem = self._place(obj)
            if problem is not None:
                return self._fail(response, problem)

        response.success = True
        response.message = ''
        response.applied_count = len(request.objects)
        self.get_logger().info(
            f'scene reset applied: {response.applied_count} object(s) '
            f'(scene={request.scene!r}, seed={request.seed})')
        return response

    def _check_known(self, obj: SceneObject) -> Optional[str]:
        """요청한 물체가 스테이지에 있는 그것과 같은지 본다.

        **기하가 다르면 거부한다.** Isaac 은 크기를 바꿀 수 없으므로, 다른 치수로
        요청받고 그냥 옮기면 트윈이 아는 크기와 실제가 어긋난 채로 파지 좌표가
        계산된다 — 에러 없이 빗나가는 종류의 실패다.
        """
        spec = self._by_name.get(obj.name)
        if spec is None:
            return (f"unknown object {obj.name!r}; Isaac scene is fixed by "
                    f'isaac_scene.json: {sorted(self._by_name)}')
        if obj.type and obj.type != spec['type']:
            return (f"{obj.name!r}: type {obj.type!r} != {spec['type']!r} in "
                    'isaac_scene.json; Isaac cannot change geometry')
        want = [float(v) for v in obj.dimensions]
        have = [float(v) for v in spec.get('dimensions', [])]
        if want and (len(want) != len(have)
                     or any(abs(a - b) > _POSITION_TOLERANCE_M for a, b in zip(want, have))):
            return (f'{obj.name!r}: dimensions {want} != {have} in isaac_scene.json; '
                    'Isaac cannot change geometry')
        return None

    def _place(self, obj: SceneObject) -> Optional[str]:
        """물체 하나를 옮기고 **읽어서 확인한다.** 문제가 있으면 사유를 돌려준다."""
        entity = self._entity_path(obj.name)

        set_request = SetEntityState.Request()
        set_request.entity = entity
        set_request.state.pose = obj.pose
        # twist 를 명시적으로 0 으로 둔다. 굴러가던 물체를 옮기기만 하면 속도가 남아
        # 놓자마자 다시 움직인다.
        set_request.state.twist.linear.x = 0.0
        set_request.state.twist.linear.y = 0.0
        set_request.state.twist.linear.z = 0.0
        set_request.state.twist.angular.x = 0.0
        set_request.state.twist.angular.y = 0.0
        set_request.state.twist.angular.z = 0.0

        sent_at = time.monotonic()
        result = self._await(self._set_cli.call_async(set_request), _SET_ENTITY_STATE)
        if result is None:
            return f"{_SET_ENTITY_STATE} did not respond for '{entity}'"

        # **응답이 성공이어도 믿지 않는다** — 실측에서 result=1 이고 메시지도 성공인데
        # pose 가 그대로인 경우가 있었다. 판정은 읽기로 한다.
        get_result = self._await(self._get_cli.call_async(
            GetEntityState.Request(entity=entity)), _GET_ENTITY_STATE)
        if get_result is None:
            return f"{_GET_ENTITY_STATE} did not respond for '{entity}'"

        actual = get_result.state.pose
        # 쓰고 읽는 사이에 물리가 돈다 — 그 시간만큼의 자유낙하는 정상으로 본다.
        elapsed = time.monotonic() - sent_at
        if not _same_pose(actual, obj.pose, elapsed):
            reported = getattr(getattr(result, 'result', None), 'error_message', '')
            return (f"'{entity}' did not move: asked "
                    f'({obj.pose.position.x:.4f}, {obj.pose.position.y:.4f}, '
                    f'{obj.pose.position.z:.4f}) but read '
                    f'({actual.position.x:.4f}, {actual.position.y:.4f}, '
                    f'{actual.position.z:.4f}). '
                    f'after {elapsed * 1e3:.0f} ms. '
                    f'{_SET_ENTITY_STATE} reported: {reported!r}')
        return None

    def _await(self, future, what: str, timeout_sec: float = _ENTITY_TIMEOUT_SEC):
        """`call_async` 결과를 기다린다. 시간 초과면 ``None``.

        `spin_until_future_complete` 를 쓰지 않는다 — executor 가 다른 스레드에서 이미
        노드를 돌리고 있어, 여기서 또 spin 하면 콜백이 그쪽으로 가서 영영 오지 않는다.
        """
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                try:
                    return future.result()
                except Exception as exc:   # noqa: BLE001
                    self.get_logger().error(f'{what} failed: {exc}')
                    return None
            time.sleep(0.01)
        return None

    def _fail(self, response: ResetScene.Response, message: str) -> ResetScene.Response:
        self.get_logger().warning(f'scene reset failed: {message}')
        response.success = False
        response.message = message
        response.applied_count = 0
        return response

    def _heartbeat(self, resolved: int) -> None:
        """주기적으로 "몇 개를 풀었는지"와 "TF 에 무엇이 있는지"를 남긴다.

        조회 실패 경고만으로는 **TF 를 못 받는 것**인지 **프레임 이름이 다른 것**인지
        가릴 수 없어서, 버퍼가 아는 프레임을 함께 찍는다.
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_heartbeat < self._warn_interval:
            return
        self._last_heartbeat = now
        try:
            known = sorted(self._buffer.all_frames_as_yaml().splitlines())
            frames = [line.split(':')[0] for line in known if line and not line.startswith(' ')]
        except Exception:
            frames = []
        self.get_logger().info(
            f'scene: resolved {resolved}/{len(self._objects)} object(s); '
            f'tf knows {len(frames)} frame(s): {frames[:12]}')


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = IsaacSceneStateNode()
        # `/scene/reset` 콜백이 다른 서비스를 기다리므로 단일 스레드로는 막힌다.
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
