#!/usr/bin/env python3
"""`/output/panda_joint` 의 수신 공백을 **벽시계 시각과 함께** 찍는다.

`fb_checkup` 은 정지 횟수·합계만 내고 각 정지가 *언제* 일어났는지는 안 준다.
Player.log 의 `[SolverTCPConnector] ... 뒤처짐` 줄과 시각으로 맞춰 보려면
정지마다 시작·끝이 필요해서 따로 만들었다.

    ./fb_stall_when.py 150          # 150초 관측, 공백 0.3s 이상을 정지로 본다
    ./fb_stall_when.py 180 0.5      # 임계를 바꾼다

ROS 를 source 하고 시뮬레이터와 같은 `ROS_DOMAIN_ID` 로 돌린다.

**팔을 움직이지 않는다** — 관측만 하므로 다른 측정과 동시에 돌려도 된다.
"""

from __future__ import annotations

from typing import Optional

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

TOPIC = '/output/panda_joint'


class StallWatch(Node):
    """수신 간격이 임계를 넘으면 그 구간을 벽시계로 기록한다."""

    def __init__(self, threshold: float) -> None:
        super().__init__('stall_when')
        self.threshold = threshold
        self.prev: Optional[float] = None
        self.count = 0
        self.first: Optional[float] = None
        self.total_gap = 0.0
        # BEST_EFFORT 구독자는 RELIABLE 발행자와도 맞는다 (요구 ≤ 제공).
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(JointState, TOPIC, self.on_msg, qos)

    def on_msg(self, _msg: JointState) -> None:
        now = time.time()
        if self.first is None:
            self.first = now
        if self.prev is not None:
            gap = now - self.prev
            if gap >= self.threshold:
                self.count += 1
                self.total_gap += gap
                print('STALL #%-3d start=%.3f end=%.3f dur=%.3f' % (
                    self.count, self.prev, now, gap), flush=True)
        self.prev = now


def main() -> int:
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    threshold = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3

    rclpy.init()
    node = StallWatch(threshold)
    print('watch %s  duration=%.0fs  threshold=%.3fs  wall_start=%.3f' % (
        TOPIC, duration, threshold, time.time()), flush=True)

    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    span = (node.prev - node.first) if (node.first and node.prev) else 0.0
    print('---', flush=True)
    print('wall_end=%.3f  관측구간=%.1fs  정지=%d회  합=%.3fs (%.1f%%)' % (
        time.time(), span, node.count, node.total_gap,
        (100.0 * node.total_gap / span) if span > 0 else 0.0), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
