#!/usr/bin/env python3
"""펑션베이 ``header.stamp`` 특성을 **주기별로 비교할 수 있게** 분해 측정한다.

``fb_checkup.py`` 의 [3] 절은 오프셋 평균·표준편차와 ``stamp_rate`` 를 낸다. 그런데
``stamp_rate`` 는 창의 **양 끝점 두 샘플**만 쓰기 때문에, 지터가 큰 구간에서는
추정 오차가 보고하려는 편차보다 커진다 (지터 0.475 s · 창 30 s 이면 ±2.2%).
그래서 주기를 바꿔가며 비교하려면 이 스크립트가 필요하다.

여기서는 오프셋을 세 성분으로 가른다.

  offset(t) = stamp(t) - recv(t) = a + b*t + residual(t)

  a + b*t_mid   창 중앙에서의 **일정 지연**. 재스탬프로 없앨 수 있는 성분
  b             stamp 진행률 편차 (``stamp_rate = 1 + b``). 표준오차를 함께 낸다
  residual      **남는 지터**. 재스탬프해도 안 없어지고 카메라-관절 정합에 직접 걸린다

잔차가 백색잡음인지 느린 흔들림인지도 가른다. 백색이면 1차 차분의 표준편차가
``sqrt(2)`` 배로 커지므로, 그 비 ``R`` 이 1 에 가까우면 백색, 0 에 가까우면 흔들림이다.
둘은 대처가 다르다 — 백색은 평균으로 줄어들지만 흔들림은 안 줄어든다.

    ./fb_stamp.py 60 hz30.json      # 60초 관측 후 JSON 저장
    ./fb_stamp.py 60                # 저장 없이 표만

인자: [관측 시간(초), 기본 60] [결과 JSON 경로, 생략 가능]
"""
from __future__ import annotations

from typing import Any, Optional

import json
import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState

# 2026-09-08 새 빌드(``MMR-Build_260907``) 기준. 카메라가 ``/camera_image``
# (``sensor_msgs/Image``) 에서 ``/camera_image/compressed`` (``CompressedImage``) 로 바뀌었다.
TOPICS = {'/output/panda_joint': JointState, '/output/gripper_joint': JointState,
          '/output/endeffector': PoseStamped,
          '/camera_image/compressed': CompressedImage}


# 정합의 기준이 되는 쌍. 데이터셋에서 이미지와 관절을 맞추는 것이 목적이므로
# 카메라 - 관절 차이가 실질적인 판정값이다.
# ``/clock`` 은 이 표에 넣지 않는다 — ``header`` 가 없어 ``header.stamp`` 분해 대상이
# 아니다. 다만 sim 시각을 직접 주므로 **정지의 독립 근거**가 되며(수신 공백 동안 값이
# 멈춘다) 그 관측은 ``fb_checkup.py`` 가 낸다.
REFERENCE_TOPIC = '/output/panda_joint'


def _qos() -> QoSProfile:
    return QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=200,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.VOLATILE)


class StampProbe(Node):
    """관측 토픽의 (수신시각, 스탬프) 쌍만 모은다. 팔은 건드리지 않는다."""

    def __init__(self):
        super().__init__('fb_stamp')
        self.rows: dict[str, list[tuple[float, float]]] = {t: [] for t in TOPICS}
        graph = dict(self.get_topic_names_and_types())
        self.missing: list[str] = []
        for topic, msg_type in TOPICS.items():
            # 그래프에 없으면 구독해도 안 붙으므로 기록만 남기고 넘어간다.
            if topic not in graph:
                self.missing.append(topic)
                continue
            self.create_subscription(msg_type, topic, self._make_cb(topic), _qos())

    def _make_cb(self, topic: str):
        def cb(msg):
            now = self.get_clock().now().nanoseconds * 1e-9
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.rows[topic].append((now, stamp))
        return cb


def _fit(rows: list[tuple[float, float]]) -> Optional[dict[str, Any]]:
    """오프셋을 일정 지연 · 진행률 편차 · 잔차로 분해한다.

    최소제곱으로 ``offset = a + b*t`` 를 맞추고, 기울기의 표준오차까지 낸다.
    표준오차를 안 내면 주기별 비교에서 잡음을 신호로 읽는다.
    """
    if len(rows) < 3:
        return None
    t0 = rows[0][0]
    xs = [n - t0 for n, _ in rows]
    ys = [s - n for n, s in rows]
    n = len(xs)
    mx = statistics.fmean(xs)
    my = statistics.fmean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:
        return None
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    a = my - b * mx
    resid = [y - (a + b * x) for x, y in zip(xs, ys)]
    sse = sum(r * r for r in resid)
    s_resid = math.sqrt(sse / (n - 2)) if n > 2 else float('nan')
    stderr_b = s_resid / math.sqrt(sxx) if sxx > 0 else float('nan')

    # 잔차가 백색이면 1차 차분의 표준편차가 sqrt(2) 배가 된다. 그 비로 성질을 가른다.
    diffs = [resid[i + 1] - resid[i] for i in range(len(resid) - 1)]
    sd_resid = statistics.pstdev(resid) if len(resid) > 1 else 0.0
    sd_diff = statistics.pstdev(diffs) if len(diffs) > 1 else 0.0
    whiteness = sd_diff / (math.sqrt(2) * sd_resid) if sd_resid > 0 else float('nan')

    span = xs[-1] - xs[0]
    recv_dt = [xs[i + 1] - xs[i] for i in range(len(xs) - 1)]
    stamp_dt = [rows[i + 1][1] - rows[i][1] for i in range(len(rows) - 1)]
    return {'n': n, 'span_sec': span, 'rate_hz': (n - 1) / span if span > 0 else float('nan'),
            'offset_mid_sec': a + b * (xs[-1] / 2.0), 'offset_mean_sec': my,
            'drift_b': b, 'drift_stderr': stderr_b, 'stamp_rate': 1.0 + b,
            'resid_std_sec': sd_resid, 'resid_p2p_sec': max(resid) - min(resid),
            'whiteness': whiteness,
            'recv_dt_median': statistics.median(recv_dt) if recv_dt else float('nan'),
            'stamp_dt_median': statistics.median(stamp_dt) if stamp_dt else float('nan'),
            'stamp_dt_min': min(stamp_dt) if stamp_dt else float('nan'),
            'stamp_dt_max': max(stamp_dt) if stamp_dt else float('nan')}


def _report(fits: dict[str, Optional[dict[str, Any]]]) -> None:
    print()
    print('[1] 오프셋 분해 — offset(t) = 일정지연 + 진행률편차*t + 잔차')
    print('    %-24s %5s %8s %11s %13s %10s %9s' % (
        'topic', 'n', 'rate', '일정지연', '진행률(±오차)', '잔차std', 'p2p'))
    for topic, fit in fits.items():
        if fit is None:
            print('    %-24s (수신 부족)' % topic)
            continue
        print('    %-24s %5d %7.2fHz %+10.4fs  %.5f±%.5f %9.4fs %8.4fs' % (
            topic, fit['n'], fit['rate_hz'], fit['offset_mid_sec'],
            fit['stamp_rate'], fit['drift_stderr'], fit['resid_std_sec'], fit['resid_p2p_sec']))
    print('    * 일정지연은 재스탬프로 제거 가능하다. 잔차는 남는다.')
    print('    * 진행률이 1.0 에서 벗어났다고 보려면 편차가 표준오차보다 커야 한다.')

    print()
    print('[2] 잔차의 성질 — 백색잡음인가 느린 흔들림인가')
    for topic, fit in fits.items():
        if fit is None:
            continue
        w = fit['whiteness']
        if math.isnan(w):
            kind = '판정 불가'
        elif w > 0.8:
            kind = '백색잡음 — 평균하면 줄어든다'
        elif w < 0.3:
            kind = '느린 흔들림 — 평균해도 안 줄어든다'
        else:
            kind = '혼합'
        print('    %-24s R=%.3f  %s' % (topic, w, kind))

    print()
    print('[3] 발행 간격 — 스탬프상 간격과 수신 간격')
    print('    %-24s %12s %12s %12s %12s' % (
        'topic', 'recv dt(중앙)', 'stamp dt(중앙)', 'stamp dt 최소', 'stamp dt 최대'))
    for topic, fit in fits.items():
        if fit is None:
            continue
        print('    %-24s %11.5fs %11.5fs %11.5fs %11.5fs' % (
            topic, fit['recv_dt_median'], fit['stamp_dt_median'],
            fit['stamp_dt_min'], fit['stamp_dt_max']))

    print()
    print('[4] 정합 — %s 기준 상대 오프셋 (데이터셋에 직접 걸리는 값)' % REFERENCE_TOPIC)
    ref = fits.get(REFERENCE_TOPIC)
    if ref is None:
        print('    기준 토픽 수신 없음 — 판정 불가')
        return
    for topic, fit in fits.items():
        if fit is None or topic == REFERENCE_TOPIC:
            continue
        delta = fit['offset_mid_sec'] - ref['offset_mid_sec']
        print('    %-24s %+.4f s' % (topic, delta))
    print('    * 0 에서 벗어난 만큼 이미지와 관절이 어긋난다. 일정하면 보정 가능하다.')


def main() -> None:
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    out_path = sys.argv[2] if len(sys.argv) > 2 else None

    rclpy.init()
    node = StampProbe()

    print('=' * 100)
    print('펑션베이 stamp 특성 측정 — %.0f초 관측, wall clock = %.3f' % (duration, time.time()))
    print('=' * 100)
    if node.missing:
        print('경고: 그래프에 없는 토픽 — %s' % ', '.join(node.missing))

    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    fits = {topic: _fit(rows) for topic, rows in node.rows.items()}
    _report(fits)

    if out_path:
        payload = {'wall_clock': time.time(), 'duration_sec': duration,
                   'fits': fits, 'raw': {t: node.rows[t] for t in node.rows}}
        with open(out_path, 'w') as handle:
            json.dump(payload, handle)
        print()
        print('저장: %s' % out_path)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
