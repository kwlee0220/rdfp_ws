#!/usr/bin/env python3
"""명령(`/input/panda_joint`)과 실제(`/output/panda_joint`)를 함께 기록해 추종 성능을 낸다.

명령을 내는 쪽(`fb_move.py`)과 **별도 프로세스**로 돌린다. 같은 Node 를 두 곳에서
spin 하면 MoveGroupClient 의 동기 메서드와 충돌한다(문서 §14.2 '위험한 패턴').

    ./fb_probe.py 30 /tmp/probe.json &
    ./fb_move.py named ready 50.0 /tmp/start.json

인자: <기록 지속시간(초)> [결과 JSON 경로]
"""
from __future__ import annotations

import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

COMMAND_TOPIC = '/input/panda_joint'
REPORT_TOPIC = '/output/panda_joint'
# 지연 스윕 범위 — 시뮬레이터 응답이 늦어도 정합점을 찾도록 0~0.3초를 훑는다.
LAG_STEP_SEC = 0.005
LAG_STEPS = 61


class Probe(Node):
    def __init__(self):
        super().__init__('fb_probe')
        # 명령이 짧고 빠르게 몰리므로 depth 를 넉넉히 잡아 유실을 막는다.
        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        self.cmd: list[tuple[float, list[float]]] = []
        self.out: list[tuple[float, list[float]]] = []
        self.create_subscription(JointState, COMMAND_TOPIC,
                                 lambda m: self.cmd.append((time.time(), list(m.position))), qos)
        self.create_subscription(JointState, REPORT_TOPIC,
                                 lambda m: self.out.append((time.time(), list(m.position))), qos)


def rate_of(samples):
    if len(samples) < 2:
        return None, None
    span = samples[-1][0] - samples[0][0]
    return ((len(samples) - 1) / span if span > 0 else None), span


def sample_at(series, t):
    """`series` 에서 시각 `t` 에 가장 가까운 샘플의 위치 배열을 돌려준다."""
    lo, hi = 0, len(series) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    candidates = [i for i in (lo - 1, lo) if 0 <= i < len(series)]
    best = min(candidates, key=lambda i: abs(series[i][0] - t))
    return series[best][1]


def peak_error(window, out, lag):
    """명령 구간 전체에서 관절별 최대 오차와 평균 오차를 낸다."""
    errs = []
    for t, cmd_pos in window:
        act = sample_at(out, t + lag)
        n = min(len(cmd_pos), len(act))
        errs.append(max(abs(cmd_pos[i] - act[i]) for i in range(n)) if n else 0.0)
    return max(errs), sum(errs) / len(errs)


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    out_path = sys.argv[2] if len(sys.argv) > 2 else '/tmp/fb_probe.json'

    rclpy.init()
    node = Probe()
    deadline = time.time() + duration
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    cmd, out = node.cmd, node.out
    result = {'cmd_count': len(cmd), 'out_count': len(out)}
    result['cmd_rate'], result['cmd_span'] = rate_of(cmd)
    result['out_rate'], result['out_span'] = rate_of(out)

    if cmd and out:
        # 정지 구간은 오차가 0 이라 평균을 왜곡한다. 명령이 흐른 구간만 평가한다.
        window = cmd
        best = min(((step * LAG_STEP_SEC,) + peak_error(window, out, step * LAG_STEP_SEC)
                    for step in range(LAG_STEPS)), key=lambda r: r[1])
        result['best_lag_sec'] = round(best[0], 4)
        result['max_abs_err_rad_at_best_lag'] = round(best[1], 5)
        result['mean_abs_err_rad_at_best_lag'] = round(best[2], 5)
        result['max_abs_err_rad_no_lag'] = round(peak_error(window, out, 0.0)[0], 5)

        # 명령이 끝나고 1.5초 뒤 값과 마지막 명령의 차 — 최종 정착 오차.
        settled = sample_at(out, cmd[-1][0] + 1.5)
        last = cmd[-1][1]
        n = min(len(last), len(settled))
        result['settle_err_rad'] = round(max(abs(last[i] - settled[i]) for i in range(n)), 6)
        result['final_cmd'] = [round(v, 4) for v in last]
        result['final_out'] = [round(v, 4) for v in settled]

    with open(out_path, 'w') as f:
        json.dump(result, f, indent=2)
    print(json.dumps(result, indent=2))

    node.destroy_node()
    rclpy.shutdown()


main()
