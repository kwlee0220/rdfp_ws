"""펑션베이의 물체 TF → ``rdfp_msgs/SceneObjects`` 변환 노드.

시뮬레이터가 조작 대상 body 의 pose 를 **`/tf` 로** 내보내고(2026-09-08 새 빌드,
벤더 요청 B-12), 이 노드가 그것을 `/scene/objects` 계약으로 바꾼다.
`isaac_scene_state_node` 가 같은 자리에 있고 읽기 경로가 사실상 동일하다.

**TF 를 경유하는 것이 이 설계의 핵심이다.** `SceneObjects` 는 pose 를 로봇 베이스
(``panda_link0``) 기준으로 요구하는데 시뮬레이터는 ``world`` 기준이다. TF 를 타면
``lookup_transform`` 한 번으로 좌표 변환이 해결되고, 쿼터니언 성분 순서를 손으로
뒤집을 자리가 사라진다. 실측(2026-09-08)으로 시뮬레이터가 **ROS 규약 xyzw** 로 준다는
것까지 확인했으므로(XML 의 ``rpy="0 0 0"`` 이 항등으로 나온다) 변환 코드를 두지 않는다.

물체의 **이름·종류·크기는 TF 에 없다.** 그래서 ``config/functionbay_scene.json`` 을
읽는다. **그 파일은 Isaac 쪽 JSON 과 성격이 다르다** — Isaac 은 같은 파일로 씬을
만들지만, 여기서는 시뮬레이터가 씬을 소유하고 우리는 pose 만 받으므로 **읽기 전용
메타데이터**다. 씬을 바꾸려면 시뮬레이터 XML(``t1__floating_*_vm.xml``)을 고친다.

**쓰기 경로(``/scene/reset``)는 없다 — 의도적이다.** 시뮬레이터에 런타임으로 body pose
를 설정하는 수단이 없고(현재는 XML 편집 + 재시작뿐), 벤더 요청에서도 일부러 제외했다.
따라서 이 백엔드에서는 트윈의 ``reset_scene`` 이 **서버 없음으로 실패한다** — 매 에피소드
같은 배치로 수집하는 것이 전제다. `mock`/`isaac` 이 서비스를 제공하는 것과 다르다.

주의 — **`w` 가 음수로 오는 경우가 있다** (실측: 항등이 ``(0,0,0,-1)``). ``q`` 와 ``-q``
는 같은 회전이라 tf2 는 정상 처리하지만, ``w >= 0`` 을 가정하거나 성분을 그대로 비교해
항등을 판정하는 코드는 오판한다. 이 노드는 tf2 결과를 그대로 싣는다.
"""
from __future__ import annotations

from typing import Any, Optional

import json
import os
import time

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from ament_index_python.packages import get_package_share_directory
from rdfp_msgs.msg import SceneObject, SceneObjects
from tf2_ros import Buffer, TransformListener

from robot_control.ros2_utils import get_parameter, parse_stripped_str

_DEFAULT_SCENE_TOPIC = '/scene/objects'
_DEFAULT_BASE_FRAME = 'panda_link0'
_DEFAULT_CONFIG_RELPATH = os.path.join('config', 'functionbay_scene.json')

# 늦게 붙은 트윈·recorder 가 현재 scene 을 즉시 보게 한다. **구독 측도 durability 를
# 맞춰야 한다** — volatile 로 구독하면 매칭 자체가 되지 않아 값이 영영 오지 않는다.
_SCENE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)

# 이 간격으로만 '몇 개를 풀었는지' 를 남긴다. 2 Hz 타이머라 매 회 찍으면 로그가 묻힌다.
_HEARTBEAT_SEC = 30.0


class FunctionbaySceneStateNode(Node):
    """펑션베이 물체 TF 를 읽어 `/scene/objects` 로 발행한다."""

    def __init__(self, **kwargs) -> None:
        # ``**kwargs`` 는 테스트가 ``parameter_overrides`` 로 설정 파일을 갈아끼우기
        # 위한 통로다 — 그것이 없으면 생성자가 share 의 기본 설정을 읽어버린다.
        super().__init__('functionbay_scene_state', **kwargs)

        self.declare_parameter('scene_topic', _DEFAULT_SCENE_TOPIC)
        self.declare_parameter('base_frame', _DEFAULT_BASE_FRAME)
        self.declare_parameter('config_file', '')
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('warn_interval_sec', 5.0)

        scene_topic = get_parameter(self, 'scene_topic', parse_stripped_str)
        self._base_frame = get_parameter(self, 'base_frame', parse_stripped_str)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self._warn_interval = float(self.get_parameter('warn_interval_sec').value)

        self._objects = self._load_objects()

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._publisher = self.create_publisher(SceneObjects, scene_topic, _SCENE_QOS)
        self.create_timer(1.0 / max(publish_rate, 0.1), self._on_timer)

        self._missing_logged: dict[str, float] = {}
        self._last_heartbeat = 0.0
        # 직전에 몇 개를 풀었는지. None 은 '아직 한 번도 안 찍었다' 라 첫 회는 반드시 남는다.
        self._last_resolved: Optional[int] = None

        self.get_logger().info(
            f'FunctionbaySceneStateNode started: {len(self._objects)} object(s) -> {scene_topic}')
        self.get_logger().info(f'  base frame: {self._base_frame}')
        self.get_logger().info(
            f'  objects: {[obj["name"] for obj in self._objects]} (dynamic only)')
        self.get_logger().info('  /scene/reset is NOT served on this backend (no runtime '
                               'set-pose API); scene layout comes from the simulator XML')

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

        펑션베이에서는 이 필터가 사실상 이중 안전장치다. 시뮬레이터가 이미
        `static="false"` 인 body 만 TF 로 내보내므로 고정 물체는 조회할 프레임 자체가
        없다. 그래도 같은 플래그를 두는 이유는 mock·Isaac 과 **설정 형식을 맞춰** 두면
        백엔드를 옮길 때 읽는 사람이 헤매지 않기 때문이다.
        """
        path = self._config_path()
        with open(path, encoding='utf-8') as f:
            config = json.load(f)
        self.get_logger().info(f'  scene config: {path}')
        return [o for o in config['objects'] if o.get('dynamic', False)]

    def _lookup(self, frame: str) -> Optional[Any]:
        """물체 프레임을 베이스 프레임 기준으로 조회한다."""
        try:
            return self._buffer.lookup_transform(
                self._base_frame, frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.0))
        except Exception as exc:  # tf2 예외 계열이 여럿이라 통째로 잡는다
            now = self.get_clock().now().nanoseconds * 1e-9
            # `-inf` 기본값이라야 첫 번째는 반드시 남는다.
            last = self._missing_logged.get(frame, float('-inf'))
            if now - last >= self._warn_interval:
                self._missing_logged[frame] = now
                self.get_logger().warning(
                    f"no transform '{self._base_frame}' -> '{frame}': "
                    f'{type(exc).__name__}. Is the simulator publishing object TF? '
                    "The 'world -> panda_link0' static TF must also be up.")
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
                continue  # 물체 하나가 실패하면 그것만 건너뛴다 (guide §6.1)
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

    def _heartbeat(self, resolved: int) -> None:
        """풀린 물체 수가 바뀌거나 일정 시간이 지나면 한 줄 남긴다.

        발행은 2 Hz 주기이므로 매 회 찍으면 로그가 묻힌다. 반대로 아무것도 안 찍으면
        **'조용히 0개 발행' 과 정상을 구분할 수 없다.**
        """
        now = time.time()
        changed = resolved != self._last_resolved
        if not changed and now - self._last_heartbeat < _HEARTBEAT_SEC:
            return
        self._last_heartbeat = now
        self._last_resolved = resolved
        if resolved:
            self.get_logger().info(f'scene: {resolved}/{len(self._objects)} object(s) resolved')
        else:
            self.get_logger().warning(
                f'scene: 0/{len(self._objects)} object(s) resolved — publishing an empty '
                'SceneObjects. Check the simulator object TF and the world->base static TF.')


def main(args=None) -> int:
    rclpy.init(args=args)
    node = FunctionbaySceneStateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
