#!/usr/bin/env python3
"""`/move_group/get_parameters` 가 **어떤 조건에서** 응답하는지 가른다.

Phase 0 검사에서 이상한 조합이 나왔다.

    3. use_sim_time  : 통과 — /move_group 이 응답한다
    5. 계획(ready)   : 실패 — /move_group/get_parameters 타임아웃 (30초)

같은 서비스인데 하나는 되고 하나는 안 된다. 다른 점이 둘이라 2×2 로 잘라 본다.

    축 1  파라미터  : `use_sim_time`(bool) vs `robot_description_semantic`(SRDF ~9 KB)
    축 2  노드 시계 : use_sim_time False vs True

    ./is_probe_srdf.py

넷 다 통과하면 원인은 `MoveGroupClient` 내부(spin 방식 등)에 있다.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from rcl_interfaces.srv import GetParameters

TARGET = '/move_group'
TIMEOUT_SEC = 10.0
CASES = [
    ('use_sim_time', 'bool'),
    ('robot_description_semantic', 'SRDF'),
]


def probe(sim_time: bool, param: str) -> tuple[bool, float, int]:
    """파라미터 하나를 조회하고 (성공, 걸린 시간, 값 길이) 를 돌려준다."""
    node = Node(f'is_probe_srdf_{"sim" if sim_time else "wall"}_{param[:8]}',
                parameter_overrides=[Parameter('use_sim_time', value=sim_time)])
    try:
        client = node.create_client(GetParameters, f'{TARGET}/get_parameters')
        if not client.wait_for_service(timeout_sec=TIMEOUT_SEC):
            return False, TIMEOUT_SEC, -1
        started = time.time()
        future = client.call_async(GetParameters.Request(names=[param]))
        rclpy.spin_until_future_complete(node, future, timeout_sec=TIMEOUT_SEC)
        elapsed = time.time() - started
        response = future.result()
        if response is None or not response.values:
            return False, elapsed, -1
        value = response.values[0]
        length = len(value.string_value) if value.string_value else 1
        return True, elapsed, length
    finally:
        node.destroy_node()


def main() -> int:
    rclpy.init()
    print(f'{TARGET}/get_parameters 조건별 응답 (타임아웃 {TIMEOUT_SEC:.0f}초)')
    print(f'{"파라미터":<32}{"노드 시계":<12}{"결과":<8}{"시간":>8}{"길이":>10}')
    results = []
    for param, label in CASES:
        for sim_time in (False, True):
            ok, elapsed, length = probe(sim_time, param)
            clock = 'sim' if sim_time else 'wall'
            mark = '응답' if ok else '타임아웃'
            print(f'{param:<32}{clock:<12}{mark:<8}{elapsed:>7.2f}s{length:>10}')
            results.append((param, clock, ok))

    print()
    failed = [f'{p}({c})' for p, c, ok in results if not ok]
    if not failed:
        print('넷 다 응답 — 원인은 MoveGroupClient 내부(spin 방식 등)에 있다')
    else:
        print(f'실패: {failed}')
    rclpy.shutdown()
    return 0 if not failed else 1


if __name__ == '__main__':
    raise SystemExit(main())
