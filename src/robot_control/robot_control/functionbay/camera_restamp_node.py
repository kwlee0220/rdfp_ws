"""펑션베이 카메라 이미지의 stamp 를 관절과 같은 벽시계 축으로 옮겨 재발행한다.

시뮬레이터는 `/camera_image/compressed` 를 **관절 보고와 같은 시뮬레이터 시계**로 찍는다.
관절은 `joint_state_fusion` 이 오프셋을 더해 벽시계 축으로 옮기므로(`stamp_source:
aligned`), 카메라도 **같은 오프셋**을 받아야 둘의 관계가 보존된다. 이 노드가 그 일을 한다.
세션 레코더 · 뷰어는 프로파일의 `camera.compressed_topic` 을 따라 **이 노드의 출력**을
받으므로 수집 계층은 이 변환을 모른다.

짝 찾기
-------
`joint_state_fusion` 이 표본마다 내는 ``sensor_msgs/TimeReference``
(``header.stamp`` = 정렬 시각, ``time_ref`` = 시뮬레이터 stamp) 를 모아 두고, 프레임의
시뮬레이터 stamp 와 **sim stamp 가 가장 가까운 관절 표본의 오프셋**을 쓴다.

**수신 순서로 고르지 않는다.** 주기적 정지 직후에는 카메라와 관절이 몰려 들어와, 받은
시점의 최신 오프셋을 쓰면 정지 경계의 프레임이 약 1.1 s 틀어진다 (2026-09-17 원자료
재현). 그래서 프레임은 자기보다 sim stamp 가 늦은 관절 짝이 도착할 때까지 **대기열에서
기다린다** — 보통 약 45 ms, 정지 직전 프레임은 정지가 끝날 때까지다. `max_hold_sec` 를
넘기면 이미 아는 짝 중 가장 가까운 것을 쓴다 (정지 직전 프레임이면 그것이 맞는 짝이다).

소비자는 stamp 로 정렬하므로 이 지연은 데이터에 영향이 없다.

**출력 stamp 는 거꾸로 가지 않는다.** 정지 직후에는 짝 오프셋 차이가 프레임 간 sim 간격보다
커질 수 있어(2026-09-17 실측 35 s 에 1건, −5.8 ms) 직전 출력 바로 뒤로 누른다.

파라미터
--------
======================== ======================================= =============================
이름                      기본값                                   설명
======================== ======================================= =============================
input_topic              /camera_image/compressed                시뮬레이터 원본
output_topic             /camera_image_aligned/compressed        정렬된 재발행
time_reference_topic     /functionbay/stamp_reference            `joint_state_fusion` 의 쌍
max_hold_sec             2.0                                     짝을 기다리는 최대 시간
======================== ======================================= =============================
"""

from __future__ import annotations

from typing import Optional

from collections import deque

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from sensor_msgs.msg import CompressedImage, TimeReference

from robot_control.functionbay.stamp_alignment import (
    STAMP_REFERENCE_TOPIC, MonotonicStamps, OffsetHistory, ns_to_stamp, stamp_to_ns)
from robot_control.ros2_utils import get_parameter, log_periodic, parse_float, parse_str

_DEFAULT_INPUT_TOPIC = '/camera_image/compressed'
_DEFAULT_OUTPUT_TOPIC = '/camera_image_aligned/compressed'
_DEFAULT_MAX_HOLD_SEC = 2.0
_IMAGE_QUEUE_SIZE = 10
_REFERENCE_QUEUE_SIZE = 50
# 대기열 상한. 10 Hz × 5 s — 관절 흐름이 멈춘 채 쌓이면 가장 오래된 것부터 대체값으로 낸다.
_MAX_PENDING = 50
_FLUSH_PERIOD_SEC = 0.1
_WARN_INTERVAL_SEC = 5.0


class CameraRestampNode(Node):
    """카메라 프레임을 관절 표본의 오프셋으로 재stamp 해 재발행한다."""

    def __init__(self) -> None:
        super().__init__('camera_restamp')

        self.declare_parameter('input_topic', _DEFAULT_INPUT_TOPIC)
        self.declare_parameter('output_topic', _DEFAULT_OUTPUT_TOPIC)
        self.declare_parameter('time_reference_topic', STAMP_REFERENCE_TOPIC)
        self.declare_parameter('max_hold_sec', _DEFAULT_MAX_HOLD_SEC)

        input_topic = get_parameter(self, 'input_topic', parse_str)
        output_topic = get_parameter(self, 'output_topic', parse_str)
        reference_topic = get_parameter(self, 'time_reference_topic', parse_str)
        max_hold_sec = get_parameter(self, 'max_hold_sec', parse_float)
        if max_hold_sec <= 0.0:
            raise ValueError(f"'max_hold_sec' must be > 0, got {max_hold_sec}")
        self._max_hold_ns = int(max_hold_sec * 1e9)

        self._history = OffsetHistory()
        # 정지 직후에는 짝 오프셋 차이가 프레임 간 sim 간격보다 커져 순서가 뒤집힐 수 있다.
        self._monotonic = MonotonicStamps()
        # (수신 ns, 메시지). 도착 순서를 지켜 내보낸다.
        self._pending: deque[tuple[int, CompressedImage]] = deque()
        self._last_fallback_warn = 0.0
        self._last_empty_warn = 0.0

        self._publisher = self.create_publisher(CompressedImage, output_topic, _IMAGE_QUEUE_SIZE)
        self.create_subscription(
            TimeReference, reference_topic, self._on_reference, _REFERENCE_QUEUE_SIZE)
        self.create_subscription(CompressedImage, input_topic, self._on_image, _IMAGE_QUEUE_SIZE)
        # 관절 흐름이 멈춰도 대기열이 `max_hold_sec` 안에 비워지도록 주기적으로 본다.
        self.create_timer(_FLUSH_PERIOD_SEC, self._flush)

        self.get_logger().info(
            f'CameraRestampNode started: {input_topic} -> {output_topic} '
            f'(pairs from {reference_topic}, max_hold={max_hold_sec:.1f}s)')

    def _on_reference(self, msg: TimeReference) -> None:
        sim_ns = stamp_to_ns(msg.time_ref)
        self._history.add(sim_ns, stamp_to_ns(msg.header.stamp) - sim_ns)
        self._flush()

    def _on_image(self, msg: CompressedImage) -> None:
        self._pending.append((self.get_clock().now().nanoseconds, msg))
        while len(self._pending) > _MAX_PENDING:
            self._release_with_fallback(*self._pending.popleft())
        self._flush()

    def _flush(self) -> None:
        """짝이 확정된 프레임을 도착 순서대로 내보낸다."""
        now_ns = self.get_clock().now().nanoseconds
        while self._pending:
            received_ns, msg = self._pending[0]
            sim_ns = stamp_to_ns(msg.header.stamp)
            if sim_ns == 0:
                # 시뮬레이터가 stamp 를 안 채웠다 — 정렬할 원본이 없으니 수신 시각을 쓴다.
                self._pending.popleft()
                self._publish(msg, received_ns)
                continue
            offset_ns = self._history.lookup(sim_ns)
            if offset_ns is not None:
                self._pending.popleft()
                self._publish(msg, sim_ns + offset_ns)
                continue
            if now_ns - received_ns < self._max_hold_ns:
                return                       # 더 가까운 관절 짝이 곧 올 수 있다
            self._pending.popleft()
            self._release_with_fallback(received_ns, msg)

    def _release_with_fallback(self, received_ns: int, msg: CompressedImage) -> None:
        """기다릴 수 없는 프레임을 가장 가까운 알려진 짝으로 내보낸다."""
        sim_ns = stamp_to_ns(msg.header.stamp)
        offset_ns = self._history.nearest(sim_ns)
        if offset_ns is None:
            self._last_empty_warn = log_periodic(
                self.get_logger().warning,
                'no stamp pairs from joint_state_fusion yet; dropping camera frame '
                '(is joint_state_fusion running with stamp_source=aligned?)',
                self._last_empty_warn, _WARN_INTERVAL_SEC)
            return
        self._last_fallback_warn = log_periodic(
            self.get_logger().warning,
            'no newer joint pair within max_hold; using the nearest known offset',
            self._last_fallback_warn, _WARN_INTERVAL_SEC)
        self._publish(msg, sim_ns + offset_ns)

    def _publish(self, msg: CompressedImage, stamp_ns: int) -> None:
        msg.header.stamp = ns_to_stamp(self._monotonic(stamp_ns), Time)
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[CameraRestampNode] = None
    try:
        node = CameraRestampNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
