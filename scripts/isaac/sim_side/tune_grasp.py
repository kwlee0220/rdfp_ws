# -*- coding: utf-8 -*-
"""Isaac Sim **안에서** 실행하는 Phase 6 파지 준비 스크립트.

손가락이 물체를 **실제로 붙잡게** 만든다. Phase 2 는 자유공간에서 손가락이 움직이는
것까지만, Phase 5 는 MoveIt planning scene 의 기하학까지만 확인했다 — 둘 다 통과해도
PhysX 안에서 블록은 손가락 사이로 미끄러져 빠진다. 빠진 것이 두 가지다.

**1. 마찰** — 재질을 안 붙이면 PhysX 기본값(정적·동적 0.5)이다. 손끝과 블록 양쪽이
0.5 면 실효 0.5 로, 매끈한 상자를 옆에서 눌러 드는 데는 모자란다.

**2. 손가락 drive** — 손가락은 위치 제어다. 닫힘(0.0)을 명령했는데 블록이 0.025 에서
막으면 그 오차가 그대로 파지력이 된다.

    파지력 = stiffness x 오차

에셋 기본 stiffness 로는 이 힘이 얼마인지 알 수 없으므로 **먼저 읽어서 로그에
남긴다.** ``config/isaac_scene.json`` 의 ``gripper`` 블록이 목표값이며, scene 물체의
마찰과 같은 파일에 있다 — 둘은 접촉면의 양쪽이라 따로 두면 어긋난다.

사용법 — **Stop 상태에서 실행하고 다시 Play 한다.** 재생 중 변경은 물리 엔진에
반영되지 않을 수 있다. ``setup_scene.py`` 를 다시 돌린 뒤에도 이 스크립트는
**다시 돌릴 필요가 없다** — 로봇 에셋을 만지므로 scene 재생성과 무관하다.

    path = "//wsl.localhost/Ubuntu-22.04/home/kwlee/development/ros/rdfp_ws"
    exec(open(path + "/scripts/isaac/sim_side/tune_grasp.py", encoding="utf-8").read())

``DRY_RUN = True`` 로 두면 **현재 값만 읽어 로그에 남기고 아무것도 바꾸지 않는다.**
"""

import os as _os

# 워크스페이스·로그 경로. 어느 배포에서도 동작한다 (문서 §1).
_IS_WINDOWS = _os.name == "nt"
_DEFAULT_WORKSPACE = ("//wsl.localhost/Ubuntu-22.04/home/kwlee/development/ros/rdfp_ws"
                      if _IS_WINDOWS else "/home/kwlee/development/ros/rdfp_ws")
_DEFAULT_LOG_DIR = "//wsl.localhost/Ubuntu-22.04/tmp" if _IS_WINDOWS else "/tmp"
WORKSPACE = _os.environ.get("RDFP_WORKSPACE") or _DEFAULT_WORKSPACE
LOG_DIR = _os.environ.get("RDFP_LOG_DIR") or _DEFAULT_LOG_DIR
SCENE_JSON = WORKSPACE + "/src/robot_control/config/isaac_scene.json"
LOG_PATH = LOG_DIR + "/isaac_tune_grasp.log"

# 손끝 마찰 재질을 두는 곳. **scene root(/World/Scene) 밖이다** — setup_scene.py 가
# scene root 를 통째로 지우고 다시 만들기 때문에, 안에 두면 scene 을 재생성할 때마다
# 손가락의 바인딩이 끊어진 재질을 가리키게 된다.
MATERIAL_ROOT = "/World/GraspMaterials"

# True 면 읽기만 하고 쓰지 않는다.
DRY_RUN = False

_LOG_LINES = []


def _log(message: str) -> None:
    print(message)
    _LOG_LINES.append(message)


def _flush_log() -> None:
    try:
        with open(LOG_PATH, "w", encoding="utf-8") as f:
            f.write("\n".join(_LOG_LINES) + "\n")
        print(f"[grasp] log written: {LOG_PATH}")
    except Exception as exc:
        print(f"[grasp] could not write log to {LOG_PATH}: {type(exc).__name__}: {exc}")


def _use_root_layer(stage) -> None:
    """편집 대상을 **root layer** 로 고정한다.

    session layer 에 쓰면 화면에서는 멀쩡히 동작하지만 ``File > Save As`` 에 담기지
    않는다 — 저장한 scene 을 다시 열면 파지가 조용히 안 되는 상태로 돌아간다.
    """
    root = stage.GetRootLayer()
    current = stage.GetEditTarget().GetLayer()
    if current != root:
        stage.SetEditTarget(root)
        _log(f"[grasp] edit target: {current.identifier} -> root layer")
    else:
        _log("[grasp] edit target: root layer (ok)")


def _find_prims_by_name(stage, names: list) -> dict:
    """스테이지에서 이름으로 prim 을 찾는다.

    경로 구조는 에셋마다 다르지만 관절·링크 **이름**은 URDF 와 맞춰져 있다.
    """
    wanted = set(names)
    found = {}
    for prim in stage.Traverse():
        name = prim.GetName()
        if name in wanted and name not in found:
            found[name] = prim
    return found


def _tune_finger_drives(stage, config: dict) -> int:
    """손가락 prismatic drive 게인을 설정한다. 건드린 관절 수를 돌려준다."""
    from pxr import UsdPhysics

    spec = config["gripper"]
    drive_spec = spec["drive"]
    stiffness = float(drive_spec["stiffness"])
    damping = float(drive_spec["damping"])
    max_force = float(drive_spec["max_force"])

    prims = _find_prims_by_name(stage, spec["finger_joints"])
    missing = [j for j in spec["finger_joints"] if j not in prims]
    if missing:
        _log(f"[grasp] WARNING: finger joint prims not found: {missing}")

    touched = 0
    for joint in spec["finger_joints"]:
        prim = prims.get(joint)
        if prim is None:
            continue
        # 손가락은 prismatic 이므로 angular 가 아니라 **linear** drive 다.
        drive = UsdPhysics.DriveAPI.Get(prim, "linear")
        if not drive:
            _log(f"[grasp] {joint}: no linear DriveAPI at {prim.GetPath()}")
            continue

        _log(f"[grasp] {joint}: stiffness={drive.GetStiffnessAttr().Get()} "
             f"damping={drive.GetDampingAttr().Get()} "
             f"maxForce={drive.GetMaxForceAttr().Get()}")
        if DRY_RUN:
            continue

        drive.GetStiffnessAttr().Set(stiffness)
        drive.GetDampingAttr().Set(damping)
        drive.GetMaxForceAttr().Set(max_force)
        touched += 1
        _log(f"[grasp]   -> stiffness={stiffness} damping={damping} maxForce={max_force} "
             f"(0.025 m 물림 = {stiffness * 0.025:.0f} N)")
    return touched


def _tune_finger_friction(stage, config: dict) -> int:
    """손끝에 마찰 재질을 바인딩한다. 바인딩한 prim 수를 돌려준다."""
    from pxr import UsdPhysics, UsdShade

    spec = config["gripper"]
    friction = spec["friction"]

    material = UsdShade.Material.Define(stage, f"{MATERIAL_ROOT}/fingertip")
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    if not DRY_RUN:
        api.CreateStaticFrictionAttr(float(friction["static"]))
        api.CreateDynamicFrictionAttr(float(friction["dynamic"]))
        api.CreateRestitutionAttr(float(friction.get("restitution", 0.0)))
    _log(f"[grasp] fingertip material {MATERIAL_ROOT}/fingertip "
         f"mu={friction['static']}/{friction['dynamic']}")

    prims = _find_prims_by_name(stage, spec["finger_links"])
    missing = [link for link in spec["finger_links"] if link not in prims]
    if missing:
        _log(f"[grasp] WARNING: finger link prims not found: {missing}")

    bound = 0
    for link in spec["finger_links"]:
        prim = prims.get(link)
        if prim is None:
            continue
        # 링크 자체와, 그 아래에서 실제로 충돌을 담당하는 prim 모두에 바인딩한다.
        # 메시가 자식 prim 에 있고 거기 CollisionAPI 가 붙은 에셋이 있어서,
        # 링크에만 걸면 조용히 기본 마찰이 쓰이는 경우가 생긴다.
        targets = [prim]
        targets += [p for p in stage.Traverse()
                    if str(p.GetPath()).startswith(str(prim.GetPath()) + "/")
                    and p.HasAPI(UsdPhysics.CollisionAPI)]
        for target in targets:
            if DRY_RUN:
                continue
            binding = UsdShade.MaterialBindingAPI.Apply(target)
            binding.Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")
            bound += 1
        _log(f"[grasp] {link}: bound to {len(targets)} prim(s) at {prim.GetPath()}")
    return bound


def main() -> None:
    import json

    import omni.usd

    _log(f"[grasp] reading {SCENE_JSON}")
    with open(SCENE_JSON, encoding="utf-8") as f:
        config = json.load(f)

    if "gripper" not in config:
        _log("[grasp] ERROR: no 'gripper' block in the scene config")
        return

    stage = omni.usd.get_context().get_stage()
    if not DRY_RUN:
        _use_root_layer(stage)

    _log(f"[grasp] grasp tuning (dry_run={DRY_RUN})")
    touched = _tune_finger_drives(stage, config)
    bound = _tune_finger_friction(stage, config)

    if DRY_RUN:
        _log("[grasp] dry run - nothing written")
    else:
        _log(f"[grasp] done - {touched} drive(s) tuned, {bound} prim(s) bound")
        _log("[grasp] Press Stop then Play so PhysX picks up the new material and gains")
        _log("[grasp] next: run is_check_phase6.py from the robot stack")


try:
    main()
finally:
    _flush_log()
