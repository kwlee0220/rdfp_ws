# -*- coding: utf-8 -*-
"""Isaac Sim **전 과정 기동** — 로봇 로드부터 Play 까지.

스테이지를 처음부터 재현한다. Script Editor 로 하던 일을 한 번에 하므로 **손으로
붙여넣을 필요가 없다.**

    ./scripts/run_isaac_sim.sh --headless   창 없이 (검사·CI)
    ./scripts/run_isaac_sim.sh --gui        창을 띄우고 같은 일을 한다

이 파일을 직접 부르지 않는다 — 환경 변수를 맞춰야 해서 위 스크립트를 거친다.

**스테이지를 저장해 두는 방식은 쓰지 않는다.** 저장하면 물체 이름·크기가 USD 와
``isaac_scene.json`` **두 곳에** 살게 되고, 그 둘은 조용히 갈라진다 — JSON 은
`isaac_scene_state_node` 도 읽는 정본이다. 매번 스크립트로 짓는 편이 느려도 어긋나지
않는다.

순서
----

    1. 확장 활성화     ros2.bridge · ros2.sim_control
    2. load_robot      Franka 를 스테이지에 올린다. 없으면 setup_graph 가
                       'no articulation root' 로 멈춘다
    3. setup_scene     테이블·블록·카메라 prim
    4. setup_graph     OmniGraph (PHASE 는 환경변수로 받는다)
    5. place_robot     panda_link0 을 월드 원점으로
    6. set_home_pose   Play 시작 자세를 ready 로
    7. tune_drive      팔 관절 gain — **없으면 Phase 1 τ 가 177 ms 다** (기준 50)
    8. tune_grasp      손가락 마찰 — **없으면 Phase 6 에서 블록이 미끄러진다**
    9. Stop → Play     그래프와 gain 은 재생 중에 붙지 않는다

`tune_*` 두 개가 빠지기 쉬운데, 빠뜨리면 검사가 "시뮬레이터가 이상하다"로 보이지
실제 원인(기본 자산이 튜닝되지 않았다)을 가리키지 않는다.

실시간 보조
----------

**headless 는 렌더링이 없어 물리가 벽시계보다 빨리 돈다** (실측 2.2배). Phase 0 의
`/clock` 배속 검사가 그것을 잡아낸다 — 검사가 옳고 기동이 틀린 것이다. 물리 dt 기본값이
1/60 이므로 업데이트를 60 Hz 로 묶어 1.0배를 맞춘다.
"""
from __future__ import annotations

import os as _os
import time

# **Isaac 임포트를 최상위에 두지 않는다.** 같은 폴더의 다른 스크립트와 같은 규칙이며,
# `robot_control/tests/test_isaac_sim_side_scripts.py` 가 Isaac 없이 이 파일을 읽어
# 경로 계약을 검사하기 때문이다. 최상위에 두면 그 검사가 통째로 깨진다.

# 워크스페이스·로그 경로. 배포 구성 네 가지를 모두 지원한다 (문서 §1).
# **같은 블록이 이 폴더의 모든 스크립트에 복사돼 있다** — Isaac 쪽에서는
# `robot_control` 을 import 할 수 없어 공용 헬퍼로 뽑을 수 없기 때문이다. 하나를
# 고치면 나머지도 함께 고친다 (test_isaac_sim_side_scripts.py 가 그것을 잡는다).
_IS_WINDOWS = _os.name == "nt"
_DEFAULT_WORKSPACE = ("//wsl.localhost/Ubuntu-22.04/home/kwlee/development/ros/rdfp_ws"
                      if _IS_WINDOWS else "/home/kwlee/development/ros/rdfp_ws")
_DEFAULT_LOG_DIR = "//wsl.localhost/Ubuntu-22.04/tmp" if _IS_WINDOWS else "/tmp"
WORKSPACE = _os.environ.get("RDFP_WORKSPACE") or _DEFAULT_WORKSPACE
LOG_DIR = _os.environ.get("RDFP_LOG_DIR") or _DEFAULT_LOG_DIR
LOG_PATH = LOG_DIR + "/isaac_headless_bringup.log"

# 물리 dt 기본값. 업데이트를 이 주기로 묶으면 sim 시간이 벽시계와 같이 간다.
PHYSICS_PERIOD_SEC = 1.0 / 60.0
RUN_SECONDS = float(_os.environ.get("ISAAC_RUN_SECONDS", "0")) or None
# 창을 띄울지. `run_isaac_sim.sh --gui` 가 0 으로 넘긴다.
HEADLESS = _os.environ.get("ISAAC_HEADLESS", "1") != "0"

# **`load_robot` 이 먼저다.** 로봇이 없으면 `setup_graph` 가 articulation root 를
# 찾지 못해 멈춘다. GUI 에서 손으로 도는 순서와 **같은 목록**이라, 한쪽만 고치고
# 다른 쪽을 빠뜨리는 일이 없다.
SIM_SIDE_SCRIPTS = ("load_robot", "setup_scene", "setup_graph", "place_robot",
                    "set_home_pose", "tune_drive", "tune_grasp")


_LOG_LINES: list = []


def _log(message: str) -> None:
    line = f"[bringup] {message}"
    print(line, flush=True)
    _LOG_LINES.append(line)


def _flush_log() -> None:
    """Script Editor 가 아니라 셸에서 돌지만 **같은 자리에 로그를 남긴다.**

    검사 스크립트와 인수인계 문서가 `/tmp/isaac_*.log` 를 보라고 안내하므로, 여기만
    다르면 찾는 사람이 없다.
    """
    try:
        with open(LOG_PATH, "w", encoding="utf-8") as handle:
            handle.write("\n".join(_LOG_LINES) + "\n")
        print(f"[bringup] log written: {LOG_PATH}", flush=True)
    except Exception as exc:                          # noqa: BLE001
        print(f"[bringup] could not write log: {type(exc).__name__}: {exc}", flush=True)


def main() -> int:
    from isaacsim import SimulationApp

    workspace = WORKSPACE
    # 로그에 이 경고가 한 번 나오는데 **무해하며, 끄려는 시도는 이미 실패했다**:
    #
    #     [omni.rtx] DLSS increasing input dimensions: Render resolution of
    #     (320, 240) is below minimal input resolution of 300.
    #
    # 뷰포트가 아니라 우리 카메라(`.../HydraTextures/Replicator`)에 붙은 것으로,
    # 640x480 출력을 절반 해상도에서 DLSS 로 올리다가 최소 입력(300)에 못 미쳐
    # **DLSS 가 스스로 입력을 키운다**는 뜻이다. 2026-09-02 실측:
    #
    #   * `SimulationApp(anti_aliasing=0)`, `--/rtx/post/aa/op=0`,
    #     `--/rtx-defaults/post/aa/op=0` 을 **모두** 줘도 경고와 동작이 그대로다 —
    #     런치 인자까지 전달되는 것을 확인했다. 이 경로는 끌 수 없다.
    #   * 이미지 차이도 없다. 라플라시안 분산이 정지 174.6(DLSS)/176.2(끔),
    #     팔을 흔드는 중 145.0/145.3 으로 0.2~0.9% — 잡음 수준이다.
    #
    # 그래서 노브를 두지 않는다. 없는 노브를 다시 만들지 않도록 여기 남긴다.
    app = SimulationApp({"headless": HEADLESS})
    _log(f"window: {'off (headless)' if HEADLESS else 'on'}")

    from isaacsim.core.utils.extensions import enable_extension

    for ext in ("isaacsim.ros2.bridge", "isaacsim.ros2.sim_control"):
        _log(f"{ext}: {enable_extension(ext)}")
    for _ in range(120):
        app.update()

    failures = 0
    for name in SIM_SIDE_SCRIPTS:
        path = _os.path.join(workspace, "scripts/isaac/sim_side", name + ".py")
        try:
            exec(open(path, encoding="utf-8").read(), {"__name__": "__main__"})
            _log(f"{name}: ran")
        except Exception as exc:                      # noqa: BLE001 - 무엇이든 이어서 간다
            failures += 1
            _log(f"{name}: FAILED {type(exc).__name__}: {exc}")
        # 참조는 비동기로 풀린다 — 원격 자산이라 첫 실행은 내려받는다. 로봇을 올린
        # 직후에는 넉넉히 기다려야 다음 스크립트가 articulation 을 찾을 수 있다.
        for _ in range(240 if name == "load_robot" else 30):
            app.update()
    if failures:
        _log(f"{failures} script(s) failed — /tmp/isaac_*.log 를 본다")

    import omni.timeline

    timeline = omni.timeline.get_timeline_interface()
    # **Stop 을 먼저 한다.** 새로 만든 그래프와 gain 은 이미 도는 재생에 붙지 않는다.
    timeline.stop()
    for _ in range(60):
        app.update()
    timeline.play()
    _log("PLAY")
    _flush_log()

    started = time.monotonic()
    next_tick = time.monotonic()
    while RUN_SECONDS is None or time.monotonic() - started < RUN_SECONDS:
        app.update()
        next_tick += PHYSICS_PERIOD_SEC
        delay = next_tick - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            # 따라잡지 못하면 벌어진 만큼을 버린다 — 빚이 쌓이면 이후가 전속력이 된다.
            next_tick = time.monotonic()
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
