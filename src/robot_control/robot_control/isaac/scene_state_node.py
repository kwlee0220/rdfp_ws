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
"""

from __future__ import annotations

from typing import Any, Optional

import json
import os

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
_DEFAULT_CONFIG_RELPATH = os.path.join('config', 'isaac_scene.json')

# `SceneObjects.msg` 가 권장하는 QoS. **구독 측도 맞춰야 한다** — volatile 로
# 구독하면 매칭 자체가 되지 않아 값이 영영 오지 않는다.
_SCENE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


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

        self._objects = self._load_objects()

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._publisher = self.create_publisher(SceneObjects, scene_topic, _SCENE_QOS)
        self.create_timer(1.0 / max(publish_rate, 0.1), self._on_timer)

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
        rclpy.spin(node)
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
