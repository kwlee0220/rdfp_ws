# -*- coding: utf-8 -*-
"""Isaac Sim **안에서** 실행하는 로봇 배치 정렬 스크립트.

**`panda_link0` 를 월드 원점에 놓는다.** ROS 쪽 static TF 가

    world -> panda_link0 = 항등변환

으로 못박혀 있기 때문이다(`launch_helpers/common.py`). 에셋을 스테이지에 드래그해
올리면 원점이 아닌 자리에 떨어지는데, 그러면 **양쪽이 조용히 어긋난다** —

  * 로봇 내부는 일관되다. `/joint_states` -> robot_state_publisher -> TF 가 전부
    `panda_link0` 기준이라 어디에 놓여 있든 자기들끼리는 맞는다.
  * scene 물체는 `setup_scene.py` 가 **월드 좌표로** 만든다.
  * 그래서 그리퍼는 "목표에 0.5 mm 오차로 도달"했다고 보고하면서 실제로는 물체에서
    수십 cm 떨어진 허공을 쥔다. 에러도 경고도 없다.

실제로 이것 때문에 Phase 6 이 막혔다 — 손가락이 0 까지 닫히고 블록은 소수점
넷째 자리까지 움직이지 않았다. 마찰과 콜라이더를 한참 의심한 뒤에야 드러났다.

**Stop 상태에서 실행한다.** 재생 중에는 PhysX 가 트랜스폼을 덮어쓴다.

    path = "//wsl.localhost/Ubuntu-22.04/home/kwlee/development/ros/rdfp_ws"
    exec(open(path + "/scripts/isaac/sim_side/place_robot.py", encoding="utf-8").read())

여러 번 돌려도 같은 상태가 된다 — 이미 원점이면 아무것도 하지 않는다.
"""

import os as _os

_IS_WINDOWS = _os.name == "nt"
_DEFAULT_LOG_DIR = "//wsl.localhost/Ubuntu-22.04/tmp" if _IS_WINDOWS else "/tmp"
LOG_DIR = _os.environ.get("RDFP_LOG_DIR") or _DEFAULT_LOG_DIR
LOG_PATH = LOG_DIR + "/isaac_place_robot.log"

# 원점에 놓을 링크. ROS 의 base frame 과 같아야 한다.
BASE_LINK = "panda_link0"
# 이 이하로 어긋나 있으면 건드리지 않는다 (m).
TOLERANCE = 1e-4

_LOG_LINES = []


def _log(message: str) -> None:
    print(message)
    _LOG_LINES.append(message)


def _flush_log() -> None:
    try:
        with open(LOG_PATH, "w", encoding="utf-8") as f:
            f.write("\n".join(_LOG_LINES) + "\n")
        print(f"[place] log written: {LOG_PATH}")
    except Exception as exc:
        print(f"[place] could not write log: {type(exc).__name__}: {exc}")


def _use_root_layer(stage) -> None:
    """편집 대상을 root layer 로 고정한다 (저장에 담기게)."""
    root = stage.GetRootLayer()
    if stage.GetEditTarget().GetLayer() != root:
        stage.SetEditTarget(root)
        _log("[place] edit target -> root layer")


def _find_articulation_root(stage):
    from pxr import UsdPhysics

    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return prim
    return None


def _find_link(stage, name: str):
    for prim in stage.Traverse():
        if prim.GetName() == name:
            return prim
    return None


def main() -> None:
    import omni.usd
    from pxr import Gf, UsdGeom

    stage = omni.usd.get_context().get_stage()

    root = _find_articulation_root(stage)
    if root is None:
        _log("[place] ERROR: no articulation root on the stage - is the robot loaded?")
        return
    link = _find_link(stage, BASE_LINK)
    if link is None:
        _log(f"[place] ERROR: link '{BASE_LINK}' not found under {root.GetPath()}")
        return

    cache = UsdGeom.XformCache()
    link_world = cache.GetLocalToWorldTransform(link)
    offset = link_world.ExtractTranslation()
    _log(f"[place] articulation root : {root.GetPath()}")
    _log(f"[place] {BASE_LINK} world  : ({offset[0]:+.4f}, {offset[1]:+.4f}, {offset[2]:+.4f})")

    if Gf.Vec3d(offset).GetLength() <= TOLERANCE:
        _log("[place] already at the world origin - nothing to do")
        return

    # USD 는 행벡터 규약이다: worldPoint = localPoint * M.
    #   link_world = A * root_world   (A = root 기준 link 의 상대 변환)
    # 원하는 것은 A * root_world' = I 이므로
    #   root_world' = A^-1 = (link_world * root_world^-1)^-1 = root_world * link_world^-1
    root_world = cache.GetLocalToWorldTransform(root)
    parent_world = cache.GetParentToWorldTransform(root)
    new_root_world = root_world * link_world.GetInverse()
    new_root_local = new_root_world * parent_world.GetInverse()

    _use_root_layer(stage)
    xform = UsdGeom.Xformable(root)
    xform.ClearXformOpOrder()
    xform.AddTransformOp().Set(new_root_local)

    # **되읽어 확인한다.** 행렬 규약을 틀려도 크래시하지 않고 '그럴듯하게 틀린'
    # 자리에 놓이므로, 눈으로 봐서는 드러나지 않는다.
    verify = UsdGeom.XformCache().GetLocalToWorldTransform(link).ExtractTranslation()
    residual = Gf.Vec3d(verify).GetLength()
    _log(f"[place] {BASE_LINK} world' : ({verify[0]:+.4f}, {verify[1]:+.4f}, {verify[2]:+.4f})  "
         f"residual={residual:.6f}")
    if residual > TOLERANCE:
        _log("[place] ERROR: still off the origin - the transform was not applied as expected")
        return

    _log("[place] done. Press Stop then Play, then rerun setup_graph.py if TF looks wrong")


try:
    main()
finally:
    _flush_log()
