# -*- coding: utf-8 -*-
"""Isaac Sim **안에서** 실행하는 충돌 설정 진단 스크립트.

**"명령은 정상인데 물체를 통과한다"** 를 가른다. 손가락이 블록을 뚫고 끝까지 닫히거나,
팔이 테이블 안에 박힌 채로 쉬고 있으면 PhysX 가 그 둘을 충돌로 보지 않는 것이다.
원인 후보가 여럿이라 눈으로 봐서는 갈리지 않는다.

  * 콜라이더가 아예 없다 (`CollisionAPI` 미적용)
  * 있는데 꺼져 있다 (`physics:collisionEnabled = False`)
  * 로봇 링크의 콜라이더가 **자식 메시**에 있고 링크에는 없다
  * 물체가 kinematic 이라 밀리지 않는 것을 '충돌 안 함'으로 오독한 경우

아무것도 바꾸지 않는다 — 읽기만 한다.

    path = "//wsl.localhost/Ubuntu-22.04/home/kwlee/development/ros/rdfp_ws"
    exec(open(path + "/scripts/isaac/sim_side/check_colliders.py", encoding="utf-8").read())
"""

import os as _os

_IS_WINDOWS = _os.name == "nt"
_DEFAULT_WORKSPACE = ("//wsl.localhost/Ubuntu-22.04/home/kwlee/development/ros/rdfp_ws"
                      if _IS_WINDOWS else "/home/kwlee/development/ros/rdfp_ws")
_DEFAULT_LOG_DIR = "//wsl.localhost/Ubuntu-22.04/tmp" if _IS_WINDOWS else "/tmp"
WORKSPACE = _os.environ.get("RDFP_WORKSPACE") or _DEFAULT_WORKSPACE
LOG_DIR = _os.environ.get("RDFP_LOG_DIR") or _DEFAULT_LOG_DIR
SCENE_JSON = WORKSPACE + "/src/robot_control/config/isaac_scene.json"
LOG_PATH = LOG_DIR + "/isaac_check_colliders.log"

# 로봇 쪽에서 들여다볼 링크. 손가락은 파지, 손은 그 위, link0 는 대조군이다.
ROBOT_LINKS = ["panda_leftfinger", "panda_rightfinger", "panda_hand", "panda_link0"]

_LOG_LINES = []


def _log(message: str) -> None:
    print(message)
    _LOG_LINES.append(message)


def _flush_log() -> None:
    try:
        with open(LOG_PATH, "w", encoding="utf-8") as f:
            f.write("\n".join(_LOG_LINES) + "\n")
        print(f"[coll] log written: {LOG_PATH}")
    except Exception as exc:
        print(f"[coll] could not write log: {type(exc).__name__}: {exc}")


def _describe(stage, prim, indent: str = "") -> None:
    """prim 하나의 물리 설정을 한 줄로 남긴다."""
    from pxr import UsdGeom, UsdPhysics

    flags = []
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        collision = UsdPhysics.CollisionAPI(prim)
        enabled = collision.GetCollisionEnabledAttr().Get()
        flags.append(f"collision={'on' if enabled is not False else 'OFF'}")
    else:
        flags.append("collision=NONE")

    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        body = UsdPhysics.RigidBodyAPI(prim)
        kinematic = body.GetKinematicEnabledAttr().Get()
        body_enabled = body.GetRigidBodyEnabledAttr().Get()
        flags.append(f"rigid={'kinematic' if kinematic else 'dynamic'}"
                     f"{'' if body_enabled is not False else '(disabled)'}")

    if prim.HasAPI(UsdPhysics.MassAPI):
        flags.append(f"mass={UsdPhysics.MassAPI(prim).GetMassAttr().Get()}")

    # 콜라이더 근사. convexHull / none / meshSimplification 등.
    approx = prim.GetAttribute("physics:approximation")
    if approx and approx.Get():
        flags.append(f"approx={approx.Get()}")

    xform_cache = UsdGeom.XformCache()
    position = xform_cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    _log(f"{indent}{str(prim.GetPath()):58s} {prim.GetTypeName():12s} "
         f"pos=({position[0]:+.3f},{position[1]:+.3f},{position[2]:+.3f})  "
         f"{'  '.join(flags)}")


def _find_by_name(stage, names: list) -> dict:
    wanted = set(names)
    found = {}
    for prim in stage.Traverse():
        name = prim.GetName()
        if name in wanted and name not in found:
            found[name] = prim
    return found


def _walk(stage, prim, depth: int = 0, limit: int = 2) -> None:
    """prim 과 그 아래를 ``limit`` 깊이까지 훑는다."""
    from pxr import UsdPhysics

    for child in prim.GetChildren():
        has_physics = (child.HasAPI(UsdPhysics.CollisionAPI)
                       or child.HasAPI(UsdPhysics.RigidBodyAPI))
        if has_physics or child.GetTypeName() in ("Mesh", "Cube", "Capsule", "Sphere"):
            _describe(stage, child, indent="    " * (depth + 1))
        if depth + 1 < limit:
            _walk(stage, child, depth + 1, limit)


def main() -> None:
    import json

    import omni.usd
    from pxr import UsdPhysics

    with open(SCENE_JSON, encoding="utf-8") as f:
        config = json.load(f)

    stage = omni.usd.get_context().get_stage()

    _log("[coll] ===== physics scene =====")
    scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    if not scenes:
        _log("[coll] WARNING: no UsdPhysics.Scene on the stage - nothing is simulated")
    for scene in scenes:
        gravity = scene.GetAttribute("physics:gravityMagnitude")
        _log(f"[coll] {scene.GetPath()}  gravity={gravity.Get() if gravity else '?'}")

    _log("[coll] ===== scene objects =====")
    root_prim = config["root_prim"]
    for spec in config["objects"]:
        prim = stage.GetPrimAtPath(f"{root_prim}/{spec['name']}")
        if not prim or not prim.IsValid():
            _log(f"[coll] MISSING {root_prim}/{spec['name']}")
            continue
        _describe(stage, prim)
        _walk(stage, prim, limit=1)

    _log("[coll] ===== robot links =====")
    prims = _find_by_name(stage, ROBOT_LINKS)
    for link in ROBOT_LINKS:
        prim = prims.get(link)
        if prim is None:
            _log(f"[coll] MISSING link {link}")
            continue
        _describe(stage, prim)
        _walk(stage, prim, limit=2)

    _log("[coll] ===== articulation roots =====")
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            enabled = prim.GetAttribute("physics:articulationEnabled")
            _log(f"[coll] {prim.GetPath()}  "
                 f"enabled={enabled.Get() if enabled else '(default true)'}")

    _log("[coll] done")


try:
    main()
finally:
    _flush_log()
