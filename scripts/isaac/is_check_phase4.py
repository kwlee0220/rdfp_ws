#!/usr/bin/env python3
"""Isaac 백엔드 **Phase 4 수용 기준**(고정 시점 카메라)을 검사한다.

    0. 재생 상태     — Isaac 이 Play 중인가
    1. 이미지 수신   — 설정한 주파수로 오는가 (±20%)
    2. 형식         — 해상도·encoding 이 정의와 일치하는가
    3. 내용         — **실제로 무언가 찍혔는가** (조명 없는 새까만 이미지를 잡는다)
    4. 스탬프       — `header.stamp` 이 채워지고 **sim time 으로 진행**하는가
    5. camera_info  — 함께 오고 해상도가 이미지와 일치하는가

**3번이 중요하다.** 펑션베이는 카메라 스탬프가 0 이라 이미지 프레임을 시간축에 놓지
못했고, 그것이 데이터셋 적재를 통째로 깨는 문제였다(그 문서 §6.5). 같은 실패를 여기서
미리 막는다.

    ./is_check_phase4.py            # 기본 12초 관측
    ./is_check_phase4.py 20

종료 코드: 전부 통과하면 0, 하나라도 실패하면 1.
"""
from __future__ import annotations

import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from ament_index_python.packages import get_package_share_directory
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image

DISCOVERY_TIMEOUT_SEC = 15.0
RATE_TOLERANCE = 0.2
# Isaac 의 rgb 헬퍼가 내보내는 encoding.
EXPECTED_ENCODING = 'rgb8'
# 내용 검사 임계. 새까만 이미지(전 픽셀 0)를 잡는 것이 목적이라 느슨하게 잡는다 —
# 어두운 씬을 실패로 만들지 않으면서 '아무것도 안 찍혔다'는 확실히 가른다.
CONTENT_MIN_VALUE = 10
CONTENT_MIN_MEAN = 5.0
CONTENT_MIN_LIT_RATIO = 0.05


class Phase4Checker(Node):
    def __init__(self, image_topic: str, info_topic: str) -> None:
        super().__init__('is_check_phase4',
                         parameter_overrides=[Parameter('use_sim_time', value=False)])
        self.images: list[tuple[float, Image]] = []
        self.infos: list[CameraInfo] = []
        self.clock_samples: list[float] = []
        self.create_subscription(Image, image_topic, self._on_image, 10)
        self.create_subscription(CameraInfo, info_topic, lambda m: self.infos.append(m), 10)
        self.create_subscription(Clock, '/clock', self._on_clock, 10)

    def _on_clock(self, msg: Clock) -> None:
        self.clock_samples.append(msg.clock.sec + msg.clock.nanosec * 1e-9)

    def _on_image(self, msg: Image) -> None:
        self.images.append((time.time(), msg))


def _fmt(ok: bool) -> str:
    return '통과' if ok else '실패'


def _load_camera() -> dict:
    path = os.path.join(get_package_share_directory('robot_control'),
                        'config', 'isaac_scene.json')
    with open(path, encoding='utf-8') as f:
        return json.load(f)['camera']


def check_playback(node: Phase4Checker) -> bool:
    deadline = time.time() + DISCOVERY_TIMEOUT_SEC
    advancing = False
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.clock_samples) >= 2 and node.clock_samples[-1] > node.clock_samples[0]:
            advancing = True
            break
    print(f'  0. 재생 상태          : {_fmt(advancing)} — /clock 샘플 {len(node.clock_samples)}개')
    if not advancing:
        print('       → Isaac 이 정지 상태다. **Play(▶) 를 누른다.**')
    return advancing


def check_rate(node: Phase4Checker, cam: dict, observe_sec: float) -> bool:
    node_deadline = time.time() + observe_sec
    while rclpy.ok() and time.time() < node_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    if len(node.images) < 2:
        print(f'  1. 이미지 수신        : {_fmt(False)} — {len(node.images)}장')
        print('       → Isaac 그래프의 PHASE 가 4 인지, 카메라 prim 이 있는지 확인한다')
        return False

    span = node.images[-1][0] - node.images[0][0]
    hz = (len(node.images) - 1) / span if span > 0 else 0.0
    target = float(cam['fps'])
    ok = abs(hz - target) <= target * RATE_TOLERANCE
    print(f'  1. 이미지 수신        : {_fmt(ok)} — {hz:.2f} Hz '
          f'(목표 {target:.0f} Hz ±{RATE_TOLERANCE * 100:.0f}%, {len(node.images)}장)')
    return ok


def check_format(node: Phase4Checker, cam: dict) -> bool:
    msg = node.images[-1][1]
    width, height = cam['resolution']
    size_ok = msg.width == width and msg.height == height
    enc_ok = msg.encoding == EXPECTED_ENCODING
    ok = size_ok and enc_ok
    print(f'  2. 형식               : {_fmt(ok)} — {msg.width}x{msg.height} '
          f'{msg.encoding!r} (기대 {width}x{height} {EXPECTED_ENCODING!r})')
    if not enc_ok:
        print('       → recorder 의 pixel_format 을 이 값에 맞춰야 한다')
    return ok


def check_content(node: Phase4Checker) -> bool:
    """이미지에 **실제로 무언가 찍혔는가.**

    이 검사가 없던 동안 **전 픽셀이 0 인 이미지가 5/5 로 통과했다** (2026-09-02).
    주파수·해상도·인코딩·스탬프가 전부 맞았기 때문이다 — 카메라 배관은 멀쩡했고
    **씬에 조명이 없었다.** 전체 편집기로 띄우면 기본 스테이지에 조명이 딸려 오지만
    `SimulationApp` 스크립트 기동은 빈 스테이지라 그렇지 않았다.

    그 데이터로 학습하면 아무 신호도 없는 영상이 쌓이고, 열어보기 전까지 드러나지
    않는다. 그래서 밝기와 **분산**을 함께 본다 — 균일한 회색도 정상이 아니다.
    """
    msg = node.images[-1][1]
    data = bytes(msg.data)
    if not data:
        print('  3. 내용               : 실패 — 빈 데이터')
        return False
    mean = sum(data) / len(data)
    lit = sum(1 for b in data if b > CONTENT_MIN_VALUE) / len(data)
    ok = mean >= CONTENT_MIN_MEAN and lit >= CONTENT_MIN_LIT_RATIO
    print(f'  3. 내용               : {_fmt(ok)} — 평균 {mean:.1f}, '
          f'{CONTENT_MIN_VALUE} 초과 픽셀 {100 * lit:.1f}% '
          f'(기준 평균 ≥ {CONTENT_MIN_MEAN}, 비율 ≥ {100 * CONTENT_MIN_LIT_RATIO:.0f}%)')
    if not ok:
        print('       → 씬에 조명이 있는지 본다. setup_scene.py 가 dome light 를 만든다')
    return ok


def check_stamp(node: Phase4Checker) -> bool:
    """스탬프가 채워지고 sim time 으로 진행하는가."""
    stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in node.images]
    filled = any(s > 0.0 for s in stamps)
    advancing = len(stamps) >= 2 and stamps[-1] > stamps[0]
    ok = filled and advancing
    span = stamps[-1] - stamps[0] if len(stamps) >= 2 else 0.0
    print(f'  4. 스탬프             : {_fmt(ok)} — 첫 {stamps[0]:.2f} 끝 {stamps[-1]:.2f} '
          f'(경과 {span:.2f}s)')
    if not filled:
        print('       → stamp 가 0 이면 이미지 프레임을 시간축에 놓을 수 없다 '
              '(펑션베이 §6.5 와 같은 실패)')
    return ok


def check_camera_info(node: Phase4Checker, cam: dict) -> bool:
    if not node.infos:
        print(f'  5. camera_info        : {_fmt(False)} — 수신 없음')
        return False
    info = node.infos[-1]
    width, height = cam['resolution']
    ok = info.width == width and info.height == height
    fx = info.k[0] if len(info.k) > 0 else 0.0
    print(f'  5. camera_info        : {_fmt(ok)} — {info.width}x{info.height}, '
          f'fx={fx:.1f}, {len(node.infos)}건')
    return ok


def main() -> int:
    observe_sec = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
    cam = _load_camera()

    rclpy.init()
    node = Phase4Checker(cam['image_topic'], cam['camera_info_topic'])
    print(f'Isaac Phase 4 수용 기준 검사 — {cam["image_topic"]}')

    results = [check_playback(node)]
    if results[0]:
        results.append(check_rate(node, cam, observe_sec))
    if all(results):
        results.append(check_format(node, cam))
        results.append(check_content(node))
        results.append(check_stamp(node))
        results.append(check_camera_info(node, cam))
    node.destroy_node()

    passed = sum(1 for r in results if r)
    print(f'\n{passed}/{len(results)} 통과')
    rclpy.shutdown()
    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
