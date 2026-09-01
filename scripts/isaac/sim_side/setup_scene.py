# -*- coding: utf-8 -*-
"""Isaac Sim **안에서** 실행하는 Phase 3 scene 물체 생성 스크립트.

``config/isaac_scene.json`` 을 읽어 테이블과 블록을 만든다. **같은 파일을 ROS 쪽
`isaac_scene_state_node` 도 읽는다** — 이름·크기를 두 곳에 적으면 조용히 어긋나기
때문이다.

    path = "//wsl.localhost/Ubuntu-22.04/home/kwlee/development/ros/rdfp_ws"
    exec(open(path + "/scripts/isaac/sim_side/setup_scene.py", encoding="utf-8").read())

기존 ``root_prim`` 을 통째로 지우고 다시 만들므로 **여러 번 실행해도 같은 상태**가
된다. 물체를 손으로 옮긴 뒤 초기화하고 싶을 때도 이 스크립트를 다시 돌린다.

쿼터니언 순서에 주의한다 — JSON 은 **ROS 규약 xyzw**, USD 는 **wxyz(스칼라 우선)**
다. 여기서 변환한다. 틀려도 norm 은 1 이라 어떤 검사도 통과하고, 결과는 크래시가
아니라 '그럴듯하게 틀린 자세'다.
"""

import os as _os

# 워크스페이스·로그 경로. 배포 구성 네 가지를 모두 지원한다 (문서 §1).
#
#   환경변수 RDFP_WORKSPACE / RDFP_LOG_DIR 가 있으면 그것을 쓴다. **Isaac 머신과
#   스택 머신이 다른 구성(§1 C·D)에서는 반드시 지정한다** — 그때는 Isaac 쪽에
#   저장소 사본이 따로 있고, 로그도 Isaac 머신에 떨어진다.
#
#   없으면 같은 머신을 가정한 기본값을 쓴다.
#     Windows : UNC 로 WSL 파일시스템 (§1 A)
#     Linux   : 로컬 경로 (§1 B)
_IS_WINDOWS = _os.name == "nt"
_DEFAULT_WORKSPACE = ("//wsl.localhost/Ubuntu-22.04/home/kwlee/development/ros/rdfp_ws"
                      if _IS_WINDOWS else "/home/kwlee/development/ros/rdfp_ws")
_DEFAULT_LOG_DIR = "//wsl.localhost/Ubuntu-22.04/tmp" if _IS_WINDOWS else "/tmp"
WORKSPACE = _os.environ.get("RDFP_WORKSPACE") or _DEFAULT_WORKSPACE
LOG_DIR = _os.environ.get("RDFP_LOG_DIR") or _DEFAULT_LOG_DIR
SCENE_JSON = WORKSPACE + "/src/robot_control/config/isaac_scene.json"
LOG_PATH = LOG_DIR + "/isaac_setup_scene.log"

_LOG_LINES = []


def _log(message: str) -> None:
    print(message)
    _LOG_LINES.append(message)


def _flush_log() -> None:
    try:
        with open(LOG_PATH, "w", encoding="utf-8") as f:
            f.write("\n".join(_LOG_LINES) + "\n")
        print(f"[scene] log written: {LOG_PATH}")
    except Exception as exc:
        print(f"[scene] could not write log: {type(exc).__name__}: {exc}")


def _use_root_layer(stage) -> None:
    """편집 대상을 root layer 로 고정한다 (저장에 담기게)."""
    root = stage.GetRootLayer()
    if stage.GetEditTarget().GetLayer() != root:
        stage.SetEditTarget(root)
        _log("[scene] edit target -> root layer")


def _physics_material(stage, root_prim: str, name: str, spec: dict):
    """마찰 재질을 만들어 돌려준다. 없으면 ``None``.

    USD 에서 마찰은 **prim 의 속성이 아니라 별도 Material prim** 이고, 콜라이더에
    ``physics`` purpose 로 바인딩해서 붙인다. 재질을 안 붙이면 PhysX 기본값
    (정적·동적 모두 0.5)이 적용되는데, 그 값으로는 손가락이 블록을 **미끄러뜨린다**.

    두 물체가 닿을 때의 유효 마찰은 양쪽 재질의 평균이다 (PhysX 기본 combine mode).
    손끝 1.2 와 블록 1.0 이면 실효 1.1 이 된다.
    """
    from pxr import UsdPhysics, UsdShade

    friction = spec.get("friction")
    if not friction:
        return None

    material = UsdShade.Material.Define(stage, f"{root_prim}/PhysicsMaterials/{name}")
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(float(friction.get("static", 0.5)))
    api.CreateDynamicFrictionAttr(float(friction.get("dynamic", 0.5)))
    api.CreateRestitutionAttr(float(friction.get("restitution", 0.0)))
    return material


def _bind_physics_material(prim, material) -> None:
    """콜라이더에 마찰 재질을 ``physics`` purpose 로 바인딩한다."""
    from pxr import UsdShade

    binding = UsdShade.MaterialBindingAPI.Apply(prim)
    binding.Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")


def _make_object(stage, root_prim: str, spec: dict) -> str:
    """물체 하나를 만든다. prim 경로를 돌려준다."""
    from pxr import Gf, UsdGeom, UsdPhysics

    path = f"{root_prim}/{spec['name']}"
    cube = UsdGeom.Cube.Define(stage, path)
    # 단위 큐브를 만들고 scale 로 크기를 준다. 축마다 크기가 다른 상자를 하나의
    # 프리미티브로 표현하는 표준 방법이다.
    cube.CreateSizeAttr(1.0)
    prim = cube.GetPrim()

    dx, dy, dz = spec["dimensions"]
    px, py, pz = spec["position"]
    qx, qy, qz, qw = spec["orientation"]

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(px, py, pz))
    # JSON 은 ROS 규약 xyzw, USD 는 wxyz 다. 여기서 뒤집는다.
    xform.AddOrientOp().Set(Gf.Quatf(qw, Gf.Vec3f(qx, qy, qz)))
    xform.AddScaleOp().Set(Gf.Vec3f(dx, dy, dz))

    color = spec.get("color")
    if color:
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])

    UsdPhysics.CollisionAPI.Apply(prim)
    rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
    if spec.get("dynamic", False):
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr(float(spec.get("mass", 0.1)))
    else:
        # **정적 물체에도 RigidBodyAPI 를 붙이고 kinematic 으로 만든다.**
        #
        # collider 만 있는 prim 도 PhysX 에서는 제자리에 고정되지만, ROS2
        # PoseTree(TF) 노드가 그것을 물리 객체로 인식하지 못해 매 틱마다
        #   [PoseTree] target getObjectType eInvalid for '/World/Scene/table'
        # 를 60 Hz 로 쏟아낸다. kinematic body 는 중력에도 충돌에도 밀리지 않으면서
        # 물리 객체로는 온전하므로 경고가 사라진다. 동적 물체가 그 위에 얹히는
        # 것도 그대로 동작한다.
        rigid_body.CreateKinematicEnabledAttr(True)

    # 마찰 재질 (Phase 6). 없으면 PhysX 기본 0.5 가 적용돼 파지가 미끄러진다.
    material = _physics_material(stage, root_prim, spec["name"], spec)
    if material is not None:
        _bind_physics_material(prim, material)
    return path


def _make_camera(stage, spec: dict) -> str:
    """고정 시점 카메라 prim 을 만든다.

    USD 카메라는 **-Z 방향을 본다.** JSON 의 orientation 은 그 규약 그대로이며,
    ROS 규약 xyzw 로 적고 여기서 USD 의 wxyz 로 뒤집는다 (물체와 같은 처리다).

    카메라 파라미터(초점거리·클리핑)는 prim 에 두고, **해상도는 여기 두지 않는다** —
    해상도는 렌더 프로덕트가 정하므로 `setup_graph.py` 쪽에서 준다.
    """
    from pxr import Gf, UsdGeom

    path = spec["prim"]
    camera = UsdGeom.Camera.Define(stage, path)
    prim = camera.GetPrim()

    px, py, pz = spec["position"]
    qx, qy, qz, qw = spec["orientation"]
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(px, py, pz))
    xform.AddOrientOp().Set(Gf.Quatf(qw, Gf.Vec3f(qx, qy, qz)))

    camera.CreateFocalLengthAttr(float(spec.get("focal_length", 24.0)))
    near, far = spec.get("clipping_range", [0.05, 10.0])
    camera.CreateClippingRangeAttr(Gf.Vec2f(float(near), float(far)))
    return path


def main() -> None:
    import json

    import omni.usd

    _log(f"[scene] reading {SCENE_JSON}")
    with open(SCENE_JSON, encoding="utf-8") as f:
        config = json.load(f)

    stage = omni.usd.get_context().get_stage()
    _use_root_layer(stage)

    root_prim = config["root_prim"]
    existing = stage.GetPrimAtPath(root_prim)
    if existing and existing.IsValid():
        stage.RemovePrim(root_prim)
        _log(f"[scene] removed existing {root_prim}")

    from pxr import UsdGeom

    UsdGeom.Xform.Define(stage, root_prim)

    for spec in config["objects"]:
        path = _make_object(stage, root_prim, spec)
        kind = "dynamic" if spec.get("dynamic") else "static"
        friction = spec.get("friction") or {}
        friction_text = (f"mu={friction.get('static')}/{friction.get('dynamic')}"
                         if friction else "mu=default(0.5)")
        _log(f"[scene] {spec['name']:>8} ({kind}) -> {path} "
             f"dims={spec['dimensions']} pos={spec['position']} {friction_text}")

    camera_spec = config.get("camera")
    if camera_spec:
        camera_path = _make_camera(stage, camera_spec)
        _log(f"[scene] {camera_spec['name']:>8} (camera) -> {camera_path} "
             f"res={camera_spec['resolution']} fps={camera_spec['fps']} "
             f"pos={camera_spec['position']}")

    _log(f"[scene] done - {len(config['objects'])} object(s) under {root_prim}")
    _log("[scene] next: rerun setup_graph.py with PHASE=3 so TF is published")
    _log("[scene] next: run tune_grasp.py so the fingers can actually hold a block")


try:
    main()
finally:
    _flush_log()
