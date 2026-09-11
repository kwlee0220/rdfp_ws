# -*- coding: utf-8 -*-
"""Isaac **안에서** ROS 2 토픽이 보이는지 확인한다 (Script Editor 에서 실행).

WSL 에서 토픽이 하나도 안 보일 때, 원인이 **브리지**인지 **네트워크**인지 가른다.
Isaac 이 쓰는 내부 rclpy 로 같은 도메인을 들여다보므로 판정이 명확하다.

    Isaac 에서도 안 보임   →  브리지/그래프 문제 (Isaac 쪽)
    Isaac 에서는 보임      →  Windows ↔ WSL2 네트워크 문제

    path = "//wsl.localhost/Ubuntu-22.04/home/kwlee/development/ros/rdfp_ws"
    exec(open(path + "/scripts/isaac/sim_side/check_bridge.py", encoding="utf-8").read())

**Play 중에 실행한다.** 정지 상태에서는 그래프가 tick 하지 않아 발행 자체가 없다.

브리지가 쓰는 컨텍스트를 건드리지 않도록 **별도 Context** 를 만들어 쓴다.
"""

import os as _os

# 워크스페이스·로그 경로. 어느 배포에서도 동작한다 (문서 §1).
#
#   환경변수 RDFP_WORKSPACE / RDFP_LOG_DIR 가 있으면 그것을 쓴다. **Isaac 머신과
#   스택 머신이 다른 구성(§1 C·D)에서는 반드시 지정한다** — 그때는 Isaac 쪽에
#   저장소 사본이 따로 있고, 로그도 Isaac 머신에 떨어진다.
#
#   없으면 같은 머신을 가정한 기본값을 쓴다.
#     Windows : UNC 로 WSL 파일시스템 (§1 부록)
#     Linux   : 로컬 경로 (§1 B)
_IS_WINDOWS = _os.name == "nt"
_DEFAULT_WORKSPACE = ("//wsl.localhost/Ubuntu-22.04/home/kwlee/development/ros/rdfp_ws"
                      if _IS_WINDOWS else "/home/kwlee/development/ros/rdfp_ws")
_DEFAULT_LOG_DIR = "//wsl.localhost/Ubuntu-22.04/tmp" if _IS_WINDOWS else "/tmp"
WORKSPACE = _os.environ.get("RDFP_WORKSPACE") or _DEFAULT_WORKSPACE
LOG_DIR = _os.environ.get("RDFP_LOG_DIR") or _DEFAULT_LOG_DIR
LOG_PATH = LOG_DIR + "/isaac_check_bridge.log"


ROS_DOMAIN_ID = 31
OBSERVE_SEC = 5.0

_LOG_LINES = []


def _log(message: str) -> None:
    print(message)
    _LOG_LINES.append(message)


def _flush_log() -> None:
    try:
        with open(LOG_PATH, "w", encoding="utf-8") as f:
            f.write("\n".join(_LOG_LINES) + "\n")
        print(f"[bridge] log written: {LOG_PATH}")
    except Exception as exc:
        print(f"[bridge] could not write log: {type(exc).__name__}: {exc}")


def main() -> None:
    import os
    import time

    os.environ.setdefault("ROS_DOMAIN_ID", str(ROS_DOMAIN_ID))
    _log(f"[bridge] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')} "
         f"RMW={os.environ.get('RMW_IMPLEMENTATION')}")

    import rclpy
    from rclpy.context import Context

    _log(f"[bridge] rclpy from: {rclpy.__file__}")

    # 브리지가 쓰는 기본 컨텍스트를 건드리지 않는다.
    context = Context()
    rclpy.init(context=context, args=None)
    node = rclpy.create_node("isaac_side_probe", context=context)
    try:
        # **spin 하지 않는다.** `rclpy.spin_once()` 는 전역 executor 를 만드는데,
        # 그것이 기본(전역) 컨텍스트를 쓴다 — 우리는 별도 Context 만 초기화했으므로
        # `'NoneType' object does not support the context manager protocol` 로 죽는다.
        # 그래프 정보(get_topic_names_and_types / count_publishers)는 rmw 계층에서
        # 오므로 spin 없이 조회된다. 디스커버리가 퍼질 시간만 기다리면 된다.
        started = time.time()
        while time.time() - started < OBSERVE_SEC:
            time.sleep(0.1)

        topics = sorted(name for name, _ in node.get_topic_names_and_types())
        _log(f"[bridge] topics visible from inside Isaac ({len(topics)}):")
        for name in topics:
            _log(f"[bridge]   {name}")

        for name in ("/clock", "/isaac_joint_states", "/tf"):
            count = node.count_publishers(name)
            _log(f"[bridge] publishers on {name}: {count}")
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)

    _log("[bridge] verdict: if /clock is missing here too, the bridge is the problem; "
         "if it is present, the Windows<->WSL2 network is")


try:
    main()
finally:
    _flush_log()
