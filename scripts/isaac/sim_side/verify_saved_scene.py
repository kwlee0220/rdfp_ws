# -*- coding: utf-8 -*-
"""저장된 scene USD 에 무엇이 담겼는지 확인한다 (Isaac Script Editor 에서 실행).

``File > Save As`` 로 저장한 scene 이 정말 재사용 가능한지 보는 용도다. 확인 대상은
넷이다 — **ActionGraph · 타임라인 구간 · drive gain · scene 물체**. 하나라도 없으면 그 scene 을 다시
열었을 때 스크립트를 또 돌려야 한다.

**현재 열려 있는 스테이지가 아니라 파일을 따로 연다.** 작업 중인 스테이지를 건드리지
않으므로 언제 실행해도 안전하다.

    path = "//wsl.localhost/Ubuntu-22.04/home/kwlee/development/ros/rdfp_ws"
    exec(open(path + "/scripts/isaac/sim_side/verify_saved_scene.py", encoding="utf-8").read())

> WSL 에서 ``strings`` 로 USD 를 들여다보는 것은 **판정 근거가 되지 못한다.** USDC
> crate 는 토큰 테이블을 압축하므로 실제로 들어 있는 이름도 검색되지 않는다.
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
LOG_PATH = LOG_DIR + "/isaac_verify_scene.log"


# 저장된 scene 위치. Windows 는 Isaac 설치 경로 아래, Ubuntu 단독 호스트는 홈 아래를
# 기본으로 둔다. 다른 곳에 저장했으면 이 값만 바꾼다.
SCENE_PATH = (r"C:\isaacsim\scenes\panda_rdfp.usd" if _IS_WINDOWS
              else _os.path.expanduser("~/isaacsim/scenes/panda_rdfp.usd"))
ARM_JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]

_LOG_LINES = []


def _log(message: str) -> None:
    print(message)
    _LOG_LINES.append(message)


def _flush_log() -> None:
    try:
        with open(LOG_PATH, "w", encoding="utf-8") as f:
            f.write("\n".join(_LOG_LINES) + "\n")
        print(f"[verify] log written: {LOG_PATH}")
    except Exception as exc:
        print(f"[verify] could not write log: {type(exc).__name__}: {exc}")


def main() -> None:
    from pxr import Usd, UsdPhysics

    _log(f"[verify] opening {SCENE_PATH}")
    stage = Usd.Stage.Open(SCENE_PATH)
    if stage is None:
        _log("[verify] FAILED to open stage")
        return

    prims = [p.GetPath().pathString for p in stage.Traverse()]
    _log(f"[verify] prims: {len(prims)}")

    graph_prims = [p for p in prims if "ActionGraph" in p]
    _log(f"[verify] ActionGraph prims: {len(graph_prims)}")
    for path in graph_prims[:12]:
        _log(f"[verify]   {path}")

    _log(f"[verify] timeline: start={stage.GetStartTimeCode()} "
         f"end={stage.GetEndTimeCode()}")

    for name in ARM_JOINT_NAMES[:2] + ARM_JOINT_NAMES[3:4]:
        for prim in stage.Traverse():
            if prim.GetName() != name:
                continue
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if drive:
                _log(f"[verify] {name}: stiffness={drive.GetStiffnessAttr().Get()} "
                     f"damping={drive.GetDampingAttr().Get()}")
            break

    # scene 물체도 확인한다 — 저장 시점에 물체가 없으면 다음 세션에 setup_scene.py 를
    # 다시 돌려야 하는데, 그 사실이 로그에 드러나야 한다.
    scene_prims = [p for p in prims if "/World/Scene/" in p]
    _log(f"[verify] scene object prims: {len(scene_prims)}")
    for path in scene_prims[:8]:
        _log(f"[verify]   {path}")

    ok = bool(graph_prims)
    verdict = "SAVED OK" if ok else "GRAPH MISSING - rerun setup scripts"
    if ok and not scene_prims:
        verdict = "GRAPH OK but NO SCENE OBJECTS - rerun setup_scene.py after opening"
    _log(f"[verify] verdict: {verdict}")


try:
    main()
finally:
    _flush_log()
