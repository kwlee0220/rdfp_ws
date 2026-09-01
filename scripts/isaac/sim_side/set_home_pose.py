# -*- coding: utf-8 -*-
"""Isaac Sim **안에서** 실행하는 시작 자세 설정 스크립트.

**Play 를 누르는 순간의 관절값을 `ready` 로 못박는다.** 에셋 기본 자세는 손목이 말려
있어 손이 탁자 안으로 들어가는데, 그러면 Play 직후 팔이 탁자 위로 '뚝 떨어지는' 것처럼
보이고 그 자세에서는 **시작 자세가 충돌이라 MoveIt 이 계획 자체를 거부한다**
(`INVALID_MOTION_PLAN`, -2).

로봇이 월드 원점에서 벗어나 있으면(`place_robot.py` 전) 탁자를 비껴가므로 이 증상이
안 보인다 — 배치를 고친 뒤에야 드러난다.

두 곳을 함께 쓴다. 하나만 쓰면 어긋난다.

  * ``state:angular:physics:position`` — PhysX 가 **시작 상태**로 삼는 값
  * ``drive:angular:physics:targetPosition`` — 드라이브가 **붙잡으려는** 값

앞의 것만 쓰면 시작하자마자 드라이브가 옛 목표로 끌고 가고, 뒤의 것만 쓰면 시작
자세에서 목표까지 한 번 크게 휘두른다.

**단위 주의 — USD 의 각도는 도(degree)다.** 설정은 라디안으로 적고 여기서 바꾼다.
틀려도 크래시하지 않는다: 1.571 rad 를 그대로 넣으면 1.6도가 되어 '그럴듯하게 틀린'
자세가 된다.

**Stop 상태에서 실행한다.** 멱등이라 여러 번 돌려도 같은 상태가 된다.

    path = "//wsl.localhost/Ubuntu-22.04/home/kwlee/development/ros/rdfp_ws"
    exec(open(path + "/scripts/isaac/sim_side/set_home_pose.py", encoding="utf-8").read())
"""

import math as _math
import os as _os

_IS_WINDOWS = _os.name == "nt"
_DEFAULT_WORKSPACE = ("//wsl.localhost/Ubuntu-22.04/home/kwlee/development/ros/rdfp_ws"
                      if _IS_WINDOWS else "/home/kwlee/development/ros/rdfp_ws")
_DEFAULT_LOG_DIR = "//wsl.localhost/Ubuntu-22.04/tmp" if _IS_WINDOWS else "/tmp"
WORKSPACE = _os.environ.get("RDFP_WORKSPACE") or _DEFAULT_WORKSPACE
LOG_DIR = _os.environ.get("RDFP_LOG_DIR") or _DEFAULT_LOG_DIR
SCENE_JSON = WORKSPACE + "/src/robot_control/config/isaac_scene.json"
LOG_PATH = LOG_DIR + "/isaac_set_home_pose.log"

# 그리퍼는 열어 둔다. 닫힌 채로 시작하면 조 사이에 물체가 낀 상태로 출발할 수 있다.
FINGER_OPEN_M = 0.04

DRY_RUN = False

_LOG_LINES = []


def _log(message: str) -> None:
    print(message)
    _LOG_LINES.append(message)


def _flush_log() -> None:
    try:
        with open(LOG_PATH, "w", encoding="utf-8") as f:
            f.write("\n".join(_LOG_LINES) + "\n")
        print(f"[home] log written: {LOG_PATH}")
    except Exception as exc:
        print(f"[home] could not write log: {type(exc).__name__}: {exc}")


def _use_root_layer(stage) -> None:
    root = stage.GetRootLayer()
    if stage.GetEditTarget().GetLayer() != root:
        stage.SetEditTarget(root)
        _log("[home] edit target -> root layer")


def _find_prims_by_name(stage, names) -> dict:
    wanted = set(names)
    found = {}
    for prim in stage.Traverse():
        name = prim.GetName()
        if name in wanted and name not in found:
            found[name] = prim
    return found


def _apply(prim, axis: str, value: float, unit: str) -> bool:
    """관절 하나에 시작 상태와 드라이브 목표를 함께 쓴다."""
    from pxr import PhysxSchema, UsdPhysics

    drive = UsdPhysics.DriveAPI.Get(prim, axis)
    if not drive:
        _log(f"[home] {prim.GetName()}: no {axis} DriveAPI at {prim.GetPath()}")
        return False

    before = drive.GetTargetPositionAttr().Get()
    if DRY_RUN:
        _log(f"[home] {prim.GetName()}: target={before} ({unit}) - dry run")
        return False

    drive.GetTargetPositionAttr().Set(value)
    # PhysX 의 시작 상태. 이것이 없으면 Play 순간 에셋 기본 자세에서 출발해
    # 목표까지 한 번 크게 휘두른다.
    state = PhysxSchema.JointStateAPI.Apply(prim, axis)
    state.CreatePositionAttr(value)
    state.CreateVelocityAttr(0.0)
    _log(f"[home] {prim.GetName()}: target {before} -> {value:.4f} {unit}")
    return True


def main() -> None:
    import json

    import omni.usd

    with open(SCENE_JSON, encoding="utf-8") as f:
        config = json.load(f)

    home = config.get("home_pose")
    if not home:
        _log("[home] ERROR: no 'home_pose' block in the scene config")
        return

    stage = omni.usd.get_context().get_stage()
    if not DRY_RUN:
        _use_root_layer(stage)

    _log(f"[home] setting startup pose (dry_run={DRY_RUN})")

    # 팔 관절은 revolute 다 — USD 는 도(degree)를 쓰므로 라디안에서 바꾼다.
    prims = _find_prims_by_name(stage, home.keys())
    missing = [j for j in home if j not in prims]
    if missing:
        _log(f"[home] WARNING: joint prims not found: {missing}")

    touched = 0
    for joint, radians in home.items():
        prim = prims.get(joint)
        if prim is None:
            continue
        if _apply(prim, "angular", _math.degrees(float(radians)), "deg"):
            touched += 1

    # 손가락은 prismatic 이라 단위가 미터 그대로다.
    finger_joints = (config.get("gripper") or {}).get("finger_joints") or []
    finger_prims = _find_prims_by_name(stage, finger_joints)
    for joint in finger_joints:
        prim = finger_prims.get(joint)
        if prim is None:
            continue
        if _apply(prim, "linear", FINGER_OPEN_M, "m"):
            touched += 1

    if DRY_RUN:
        _log("[home] dry run - nothing written")
    else:
        _log(f"[home] done - {touched} joint(s) set")
        _log("[home] Press Stop then Play - the arm should start at 'ready'")
        _log("[home] Save the scene so this survives a reload")


try:
    main()
finally:
    _flush_log()
