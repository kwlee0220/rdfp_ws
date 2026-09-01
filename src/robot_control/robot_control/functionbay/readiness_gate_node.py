"""시뮬레이터 기동 완료를 기다렸다가 종료하는 게이트 노드.

mock / Gazebo 백엔드는 `panda_hand_controller` spawner 가 종료되는 것을 "백엔드가
준비됨" 신호로 쓰고, launch 가 `OnProcessExit` 로 상위 노드를 spawn 한다
(`launch_helpers/controller_startup.py`). 펑션베이는 ros2_control 을 쓰지 않아
spawner 자체가 없으므로 그 신호를 만들 주체가 필요하다.

이 노드는 **지정한 토픽의 첫 메시지를 받으면 즉시 종료**한다. 그래서 launch 에서
기존 spawner 와 똑같이 `OnProcessExit` 로 엮을 수 있다 — 백엔드가 달라도 기동
orchestration 코드는 그대로다.

타임아웃 안에 메시지가 오지 않으면 **0 이 아닌 코드로 종료**한다. 시뮬레이터가
떠 있지 않은데 상위 스택이 올라가 조용히 멈춘 것처럼 보이는 상황을 막는다.

파라미터
--------
============= ====================== ==========================================
이름           기본값                  설명
============= ====================== ==========================================
topic         /output/panda_joint    기다릴 토픽 (`sensor_msgs/JointState`)
timeout_sec   60.0                   이 시간 안에 안 오면 실패 종료
============= ====================== ==========================================
"""

from __future__ import annotations

import time

import sys

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from robot_control.ros2_utils import get_parameter, parse_float, parse_str

_DEFAULT_TOPIC = '/output/panda_joint'
_DEFAULT_TIMEOUT_SEC = 60.0
_DEFAULT_QUEUE_SIZE = 1
# 대기 중 진행 상황을 알리는 간격(초).
_PROGRESS_INTERVAL_SEC = 5.0


class ReadinessGateNode(Node):
    """지정 토픽의 첫 메시지를 받을 때까지 대기한다."""

    def __init__(self) -> None:
        super().__init__('readiness_gate')

        self.declare_parameter('topic', _DEFAULT_TOPIC)
        self.declare_parameter('timeout_sec', _DEFAULT_TIMEOUT_SEC)

        self._topic = get_parameter(self, 'topic', parse_str)
        self._timeout_sec = get_parameter(self, 'timeout_sec', parse_float)
        if self._timeout_sec <= 0.0:
            raise ValueError(f"'timeout_sec' must be > 0, got {self._timeout_sec}")

        self._received = False
        self._elapsed = 0.0
        self._subscription = self.create_subscription(
            JointState, self._topic, self._on_message, _DEFAULT_QUEUE_SIZE)
        self.create_timer(_PROGRESS_INTERVAL_SEC, self._on_progress)

        self.get_logger().info(
            f"Waiting for the simulator: first message on '{self._topic}' "
            f'(timeout {self._timeout_sec:.0f}s)')

    @property
    def received(self) -> bool:
        """첫 메시지를 받았는지 여부."""
        return self._received

    @property
    def topic(self) -> str:
        """기다리는 토픽 이름."""
        return self._topic

    @property
    def timeout_sec(self) -> float:
        """대기 한도(초)."""
        return self._timeout_sec

    def _on_message(self, msg: JointState) -> None:
        """첫 메시지만 처리한다. 이후 spin 은 호출자가 멈춘다."""
        if self._received:
            return
        self._received = True
        self.get_logger().info(
            f"Simulator is ready: got {len(msg.position)} joint position(s) on "
            f"'{self._topic}'")

    def _on_progress(self) -> None:
        """대기 중임을 주기적으로 알린다 (조용히 멈춘 것처럼 보이지 않게)."""
        if self._received:
            return
        self._elapsed += _PROGRESS_INTERVAL_SEC
        self.get_logger().warning(
            f"Still waiting for '{self._topic}' ({self._elapsed:.0f}/"
            f'{self._timeout_sec:.0f}s) — is the simulator running?')


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = ReadinessGateNode()
        # **벽시계(monotonic)로 잰다. ROS 클럭을 쓰면 안 된다.**
        #
        # `use_sim_time` 이 켜진 백엔드(Isaac)에서는 첫 `/clock` 을 받기 전까지
        # `now()` 가 0 이다. 그 상태로 deadline 을 잡으면 "0 + timeout" 이 되는데,
        # 시뮬레이터가 이미 오래 돌아 sim time 이 그보다 크면 **첫 /clock 이 도착하는
        # 순간 곧바로 타임아웃**한다 — 실제로 60초 한도가 113 ms 만에 터졌다.
        # 반대로 `/clock` 이 영영 오지 않으면 시간이 0 에 멈춰 **영원히 대기**한다.
        # 기동 게이트가 재려는 것은 시뮬레이션 시간이 아니라 사람이 기다리는
        # 실제 시간이므로 monotonic 이 맞다.
        deadline = time.monotonic() + node.timeout_sec
        while rclpy.ok() and not node.received:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= deadline:
                node.get_logger().error(
                    f"Timed out after {node.timeout_sec:.0f}s waiting for "
                    f"'{node.topic}'; the simulator did not publish")
                return 1
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
