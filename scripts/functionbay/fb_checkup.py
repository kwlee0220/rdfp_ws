#!/usr/bin/env python3
"""펑션베이 시뮬레이터 인수 체크리스트 — **수동 관측 단계**를 한 번에 확인한다.

팔을 움직이지 않고 확인할 수 있는 항목만 다룬다. 토픽 존재·타입·발행 주기·
``header.stamp`` 시간 기준·메시지 필드 형식을 기준선(BASELINE)과 대조해 표로 낸다.

능동 확인(중력 처짐·dead time·시정수·추종 성능·그리퍼)은 ``fb_hold.py`` /
``fb_raw.py`` / ``fb_probe.py`` + ``fb_move.py`` 가 맡는다. 절차는
``docs/simulation/functionbay_checklist.md`` 를 본다.

    ./fb_checkup.py                       # 기본 30초 관측
    ./fb_checkup.py 60                    # 관측 시간 지정
    ./fb_checkup.py 60 --joint-hz 30 --cam-hz 5   # 시뮬레이터 설정을 바꿨을 때

기본 기준선은 **조인트 50 Hz · 카메라 10 Hz** 이고 카메라 토픽은
``/camera_image/compressed`` (``sensor_msgs/CompressedImage``, jpeg) 다 — 2026-09-08 새
빌드 ``MMR-Build_260907`` 기준이다. 그 전 빌드는 ``/camera_image`` (``Image``, bgr8) 였다.
``/clock`` 도 함께 관측하지만 **주기는 대조하지 않는다** — 내부 스텝(약 72 Hz)이라 보고
주기와 무관하다.

인자: [관측 시간(초), 기본 30] [--joint-hz N] [--cam-hz N]

**발행 주기는 시뮬레이터 XML 로 설정하는 값이다** (`VariableModel/t1__*_vm.xml` 의
``Hz`` · ``Publish_Frequency``). 그래서 고정 상수와 대조하면 안 되고, 설정을 바꿨으면
``--joint-hz`` / ``--cam-hz`` 로 알려줘야 판정이 성립한다.

``stamp`` 를 더 깊게 봐야 하면 ``fb_stamp.py`` 를 쓴다. 이 스크립트는 일정 지연·진행률·
잔차까지만 내고, 잔차의 성질(백색잡음 대 톱니)과 토픽 간 정합은 그쪽이 분해한다.

**토픽이 목록에 있다는 것과 발행되고 있다는 것은 다르다.** ``ros_tcp_endpoint`` 는
Unity 가 끊겨도 등록된 퍼블리셔를 유지하므로, 이 스크립트는 반드시 실제 수신
건수로 판정한다.
"""
from __future__ import annotations

from typing import Any, Optional

import argparse
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CompressedImage, Image, JointState

# 기준선 — ``hz`` 는 시뮬레이터 XML 의 **설정값**이고 기본값은 2026-08-31 3차 실측 당시의
# 설정이다. 실측값이 아니므로 관측이 낮게 나와도 여기를 고치면 안 된다 (설정을 바꿨으면
# ``--joint-hz`` / ``--cam-hz`` 로 넘긴다). ``type`` 은 문서 §7 ⑥ 이 기록한 형식이다.
#
# 한때 여기에 ``hz_seen``(직전 관측값)을 두고 그것과 대조했는데, 출력이 그 값을 '기준선'
# 이라고 불러서 **설정을 되돌려 정상이 된 것을 CHANGED 로 표시**했다. 관측값을 기준으로
# 삼지 않는다.
# 2026-09-08 새 빌드(``MMR-Build_260907``) 기준이다. 카메라는 토픽 이름과 타입이 함께
# 바뀌었고(``Image`` -> ``CompressedImage``), 조인트 설정 주기는 50 Hz 로 재설정했다.
CAMERA_TOPIC = '/camera_image/compressed'
CLOCK_TOPIC = '/clock'

# discovery 대기 상한. 노드 생성 직후 그래프는 비어 있다 (``_wait_for_graph``).
DISCOVERY_TIMEOUT_SEC = 5.0

BASELINE: dict[str, dict[str, Any]] = {
    '/output/panda_joint': {'type': 'sensor_msgs/msg/JointState', 'hz': 50.0, 'group': 'joint'},
    '/output/gripper_joint': {'type': 'sensor_msgs/msg/JointState', 'hz': 50.0, 'group': 'joint'},
    '/output/endeffector': {'type': 'geometry_msgs/msg/PoseStamped', 'hz': 50.0,
                            'group': 'joint'},
    CAMERA_TOPIC: {'type': 'sensor_msgs/msg/CompressedImage', 'hz': 10.0, 'group': 'camera'},
    CLOCK_TOPIC: {'type': 'rosgraph_msgs/msg/Clock', 'hz': None, 'group': 'clock'}}

# 구독 가능한 타입만 실제로 붙는다. 그래프의 타입이 여기 없으면 형식 확인은 건너뛴다.
SUBSCRIBABLE = {'sensor_msgs/msg/JointState': JointState, 'sensor_msgs/msg/Image': Image,
                'sensor_msgs/msg/CompressedImage': CompressedImage,
                'geometry_msgs/msg/PoseStamped': PoseStamped,
                'rosgraph_msgs/msg/Clock': Clock}

# Unity 가 구독해야 하는 명령 채널. 구독자가 사라지면 명령 경로가 끊긴 것이다.
INPUT_TOPICS = ['/input/panda_joint', '/input/endeffector', '/input/gripper_joint']

# 주기 판정 허용 오차(비율). 이보다 벗어나면 CHANGED 로 본다.
HZ_TOLERANCE = 0.10
# stamp 가 벽시계라고 볼 최대 오차(초).
WALL_CLOCK_TOLERANCE_SEC = 5.0
# 이 값을 넘는 수신 공백은 시뮬레이터 정지로 본다. 2026-09-03 실측 기준 정지는
# 34.3초마다 2.0초이며, 정상 발행 간격(30 Hz 에서 0.033초)과 두 자릿수 차이라 오검출이 없다.
STALL_GAP_SEC = 0.3
# 정지가 있을 때 진행률을 판정하려면 톱니를 이만큼은 담아야 한다. 34초 주기이므로
# 대략 3분 이상 관측하라는 뜻이다.
MIN_STALLS_FOR_RATE = 5


def _qos() -> QoSProfile:
    return QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=100,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.VOLATILE)


class Checkup(Node):
    """관측 대상 토픽을 모두 구독해 수신 시각·스탬프·마지막 메시지를 모은다."""

    def __init__(self):
        super().__init__('fb_checkup')
        self.rows: dict[str, list[tuple[float, float]]] = {}
        self.last: dict[str, Any] = {}
        self.actual_type: dict[str, Optional[str]] = {}
        graph = self._wait_for_graph()
        for topic in BASELINE:
            self.rows[topic] = []
            self.last[topic] = None
            found = graph.get(topic)
            self.actual_type[topic] = found[0] if found else None
            # 그래프가 아직 비어 있어도 **기준선 타입으로 구독한다.** 구독을 건너뛰면
            # 관측이 통째로 0 이 되어 '시뮬레이터가 안 보낸다'로 오진한다.
            msg_type = (SUBSCRIBABLE.get(self.actual_type[topic] or '')
                        or SUBSCRIBABLE.get(BASELINE[topic]['type']))
            if msg_type is None:
                continue
            self.create_subscription(msg_type, topic, self._make_cb(topic), _qos())

    def _wait_for_graph(self, timeout_sec: float = DISCOVERY_TIMEOUT_SEC) -> dict[str, Any]:
        """BASELINE 토픽이 그래프에 나타날 때까지 기다렸다가 그래프를 돌려준다.

        **노드를 만든 직후에는 DDS discovery 가 끝나지 않아 그래프가 비어 있다.** 그
        상태로 타입을 읽어 구독을 결정하면 **아무것도 구독하지 않고** 전 항목이
        '수신 없음' 으로 나온다 — 실제로 그렇게 오진했다 (2026-09-08).
        """
        deadline = time.time() + timeout_sec
        graph: dict[str, Any] = {}
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            graph = dict(self.get_topic_names_and_types())
            if all(t in graph for t in BASELINE):
                return graph
        missing = [t for t in BASELINE if t not in graph]
        print('    ! discovery %.0fs 안에 그래프에 안 나타난 토픽: %s' % (timeout_sec, missing))
        print('    ! 기준선 타입으로 구독을 시도한다 (§0 함정 2 도 함께 본다)')
        return graph

    def _make_cb(self, topic: str):
        def cb(msg):
            now = self.get_clock().now().nanoseconds * 1e-9
            # ``/clock`` 은 ``header`` 가 없다 — sim 시각 자체가 스탬프 자리에 온다.
            # 그 덕에 아래 [3] 표가 '경과시간 vs 벽시계' 를 나란히 보여 준다.
            src = msg.clock if isinstance(msg, Clock) else msg.header.stamp
            self.rows[topic].append((now, src.sec + src.nanosec * 1e-9))
            self.last[topic] = msg
        return cb


def _classify_stamp(offsets: list[float]) -> str:
    """stamp 가 벽시계인지 기동 후 경과 시간인지 가른다."""
    mean = statistics.fmean(offsets)
    if abs(mean) < WALL_CLOCK_TOLERANCE_SEC:
        return 'wall-clock'
    if mean < -1e8:
        return 'elapsed-since-start'
    return 'skewed'


def _fit_offset(rows: list[tuple[float, float]]) -> Optional[dict[str, float]]:
    """``stamp - recv`` 를 일정 지연 · 진행률 · 잔차로 분해한다 (최소제곱).

    **끝점 두 샘플로 진행률을 재면 안 된다.** 예전 구현이 그랬는데, 잔차가 0.5초인
    구간에서 30초 창의 끝점 추정은 오차가 ±2%라 재려는 편차보다 잡음이 컸다. 같은
    데이터가 끝점 기준으로는 0.973x · 0.994x 로 나왔지만 회귀로는 1.00029±0.00015 다.
    표준오차를 함께 내는 이유가 그것이다 — 편차가 오차보다 작으면 판정하지 않는다.
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
    s_resid = math.sqrt(sum(r * r for r in resid) / (n - 2))
    return {'offset_mid': a + b * (xs[-1] / 2.0), 'mean': my, 'stamp_rate': 1.0 + b,
            'stderr': s_resid / math.sqrt(sxx),
            'resid_std': statistics.pstdev(resid) if len(resid) > 1 else 0.0}


def _stalls(rows: list[tuple[float, float]]) -> list[float]:
    """수신이 끊긴 구간의 길이를 낸다. 시뮬레이터가 통째로 멈춘 시간이다.

    stamp 는 그동안 한 프레임밖에 안 나가므로, 전송 지연이 아니라 생산 정지다.
    """
    return [rows[i + 1][0] - rows[i][0] for i in range(len(rows) - 1)
            if rows[i + 1][0] - rows[i][0] > STALL_GAP_SEC]


def _verdict(actual: Optional[float], expected: Optional[float]) -> str:
    if actual is None or expected is None:
        return '?'
    return 'OK' if abs(actual - expected) <= expected * HZ_TOLERANCE else 'CHANGED'


def _describe(msg) -> str:
    """메시지 형식에서 체크리스트가 보는 필드만 뽑는다."""
    if isinstance(msg, JointState):
        named = 'name filled' if msg.name else 'name EMPTY'
        return '%s, pos=%d vel=%d eff=%d' % (named, len(msg.position), len(msg.velocity),
                                             len(msg.effort))
    if isinstance(msg, Image):
        return 'encoding=%s %dx%d step=%d' % (msg.encoding, msg.width, msg.height, msg.step)
    if isinstance(msg, CompressedImage):
        # 압축 형식에는 encoding·step 이 없다. 대신 format 과 바이트 수를 본다.
        return 'format=%r bytes=%d frame=%s' % (msg.format, len(msg.data),
                                                msg.header.frame_id)
    if isinstance(msg, Clock):
        secs = msg.clock.sec + msg.clock.nanosec * 1e-9
        base = 'wall-clock epoch' if msg.clock.sec > 1000000 else 'elapsed-since-start'
        return 'clock=%.3f s (%s)' % (secs, base)
    if isinstance(msg, PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        return 'frame=%s xyz=(%.4f,%.4f,%.4f) quat=(%.4f,%.4f,%.4f,%.4f)' % (
            msg.header.frame_id, p.x, p.y, p.z, o.x, o.y, o.z, o.w)
    return type(msg).__name__


def _report_inputs(node: Checkup) -> None:
    print('[1] 명령 채널 — Unity 가 구독 중인가')
    for topic in INPUT_TOPICS:
        count = node.count_subscribers(topic)
        state = 'OK' if count >= 1 else 'NO SUBSCRIBER (Unity 미연결 또는 명령 경로 끊김)'
        print('    %-24s subscribers=%d  %s' % (topic, count, state))


def _report_rates(node: Checkup, expected_hz: dict[str, float]) -> None:
    print('[2] 관측 채널 — 타입 · 주기 · 정지')
    for topic, base in BASELINE.items():
        actual = node.actual_type[topic] or '(그래프에 없음)'
        type_state = 'OK' if actual == base['type'] else 'CHANGED (기준선 %s)' % base['type']
        rows = node.rows[topic]
        pubs = node.count_publishers(topic)
        if not rows:
            note = 'NO MESSAGE (등록만 되어 있다)' if pubs else 'NO PUBLISHER'
            print('    %-24s %-38s pub=%d  %s' % (topic, actual, pubs, note))
            print('    %-24s   type %s' % ('', type_state))
            continue
        span = rows[-1][0] - rows[0][0]
        hz = (len(rows) - 1) / span if span > 0 else float('nan')
        expected = expected_hz[topic]
        if expected is None:
            print('    %-24s %-38s pub=%d  n=%-5d rate=%6.2f Hz  [설정 대조 없음]' % (
                topic, actual, pubs, len(rows), hz))
        else:
            print('    %-24s %-38s pub=%d  n=%-5d rate=%6.2f Hz  [%s vs 설정 %.2f]' % (
                topic, actual, pubs, len(rows), hz, _verdict(hz, expected), expected))
        print('    %-24s   type %s' % ('', type_state))
        # 정지 구간은 발행률 평균에 묻히므로 따로 낸다. 묻히면 데이터 구멍을 놓친다.
        stalls = _stalls(rows)
        if stalls:
            print('    %-24s   정지 %d회, 합 %.2fs (%.1f%%), 최대 %.2fs  ← 시뮬레이터 멈춤' % (
                '', len(stalls), sum(stalls), sum(stalls) / span * 100.0, max(stalls)))
            print('    %-24s   정지 제외 실효 발행률 %.2f Hz' % (
                '', (len(rows) - 1 - len(stalls)) / (span - sum(stalls))))
        else:
            print('    %-24s   정지 없음 (>%.1fs 공백 0회)' % ('', STALL_GAP_SEC))


def _report_stamps(node: Checkup) -> None:
    print('[3] header.stamp — 시간 기준 · 일정지연 · 진행률 · 잔차')
    for topic in BASELINE:
        rows = node.rows[topic]
        if not rows:
            print('    %-24s (수신 없음)' % topic)
            continue
        fit = _fit_offset(rows)
        if fit is None:
            print('    %-24s (표본 부족)' % topic)
            continue
        base = _classify_stamp([s - n for n, s in rows])
        drift = abs(fit['stamp_rate'] - 1.0)
        # 편차가 표준오차보다 작으면 잡음이다. 그럴 때 수치를 판정으로 읽으면 오진한다.
        mark = '유의' if drift > 2.0 * fit['stderr'] else '잡음수준'
        # 정지가 있으면 오프셋이 톱니라, 창이 톱니 주기의 정수배가 아닌 한 기울기가
        # 톱니 위상 쪽으로 편향된다. 표준오차는 그 편향을 못 잡으므로 따로 막는다.
        # 실측: 정지 1회짜리 45초 창은 0.98876±0.00124 를 냈지만 참값은 1.0 이다.
        if len(_stalls(rows)) < MIN_STALLS_FOR_RATE:
            mark = '창부족'
        print('    %-24s base=%-18s 지연=%+.4fs  진행률=%.5f±%.5f (%s)  잔차std=%.4fs' % (
            topic, base, fit['offset_mid'], fit['stamp_rate'], fit['stderr'], mark,
            fit['resid_std']))
    print('    * 기대: 전 토픽 wall-clock. 일정지연은 재스탬프로 제거되지만 잔차는 남는다.')
    print('    * elapsed-since-start 이면 데이터셋 정합이 깨진다 (벤더 요청 A-3 / B-7).')
    print('    * 잔차가 크면 지터가 아니라 정지+과속 톱니일 수 있다 — fb_stamp.py 로 분해한다.')
    print('    * 창부족 = 정지 %d회 미만이라 진행률을 판정하지 않는다. 180초 이상 관측한다.'
          % MIN_STALLS_FOR_RATE)


def _report_formats(node: Checkup, graph: dict[str, Any]) -> None:
    print('[4] 메시지 형식')
    for topic in BASELINE:
        msg = node.last[topic]
        print('    %-24s %s' % (topic, _describe(msg) if msg else '(수신 없음)'))
    print('    * 기대: JointState 는 name 채움. vel·eff 길이 0 이면 미발행 (벤더 요청 신규 항목)')
    print('    * 기대: CompressedImage 는 format=jpeg (2026-09-08 부터 — 그 전에는 Image bgr8)')
    print('    * 기대: /clock 은 elapsed-since-start. **use_sim_time 을 켜면 안 된다** —'
          ' /output/* 는 벽시계라 시간축이 섞인다 (벤더 B-13)')
    print()
    print('[5] camera_info')
    has_info = any(name.endswith('camera_info') for name in graph)
    print('    %s' % ('발행됨 — 기준선에서 바뀌었다' if has_info else
                      '없음 — 기준선과 동일(미제공)'))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='펑션베이 수동 관측 체크 (팔을 움직이지 않는다)')
    parser.add_argument('duration', nargs='?', type=float, default=30.0, help='관측 시간(초)')
    parser.add_argument('--joint-hz', type=float, default=None,
                        help='/output/* 의 시뮬레이터 설정 주기 (기본 %.0f)' % BASELINE[
                            '/output/panda_joint']['hz'])
    cam_default = BASELINE[CAMERA_TOPIC]['hz']
    parser.add_argument('--cam-hz', type=float, default=None,
                        help='%s 의 설정 주기 (기본 %.0f)' % (CAMERA_TOPIC, cam_default))
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    duration = args.duration
    override = {'joint': args.joint_hz, 'camera': args.cam_hz}
    # ``clock`` 처럼 override 가 없는 group 은 기준선 값을 그대로 쓴다. ``hz=None`` 이면
    # 주기 판정을 하지 않는다 — /clock 은 보고 주기와 무관한 내부 스텝이라 대조 대상이 아니다.
    expected_hz = {
        topic: (override.get(base['group']) or base['hz'])
        for topic, base in BASELINE.items()}

    rclpy.init()
    node = Checkup()
    graph = dict(node.get_topic_names_and_types())

    print('=' * 100)
    print('펑션베이 수동 관측 체크 — %.0f초 관측, wall clock = %.3f' % (duration, time.time()))
    print('설정 주기 대조: /output/* %.2f Hz, %s %.2f Hz' % (
        expected_hz['/output/panda_joint'], CAMERA_TOPIC, expected_hz[CAMERA_TOPIC]))
    print('=' * 100)

    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    print()
    _report_inputs(node)
    print()
    _report_rates(node, expected_hz)
    print()
    _report_stamps(node)
    print()
    _report_formats(node, graph)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
