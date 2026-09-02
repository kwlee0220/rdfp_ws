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


# 조명 세기. 렌더 결과만 좌우하고 물리에는 영향이 없다.
#
# **`DistantLight` 를 주광으로 쓴다.** `DomeLight` 는 **배경 자체를 칠하므로** 세기를
# 올리면 화면이 하얗게 날아간다 — 처음 1000 으로 넣었다가 배경이 완전히 백색이 됐다.
# 평행광은 표면만 비추고 배경은 그대로 둔다. Isaac 기본 스테이지의
# `/Environment/defaultLight` 도 이것이다.
DISTANT_LIGHT_INTENSITY = 1500.0
# 그림자 안쪽이 새까맣지 않도록 아주 약한 환경광을 얹는다. 배경을 칠하므로 **낮게**
# 둔다 — 이 값이 곧 배경 밝기다.
DOME_LIGHT_INTENSITY = 60.0


def _make_light(stage, root_prim: str) -> list:
    """씬 조명을 만든다. **없으면 카메라가 새까만 이미지를 낸다.**

    Isaac 을 전체 편집기(`isaac-sim.sh`)로 띄우면 기본 스테이지에
    `/Environment/defaultLight` 가 딸려 오지만, `SimulationApp` 으로 스크립트 기동하면
    **빈 스테이지**라 조명이 하나도 없다. 그래서 뷰포트도 까맣고
    `/isaac/camera/image_raw` 의 전 픽셀이 0 이 된다.

    **그 상태로도 Phase 4 검사는 통과했다** — 주파수·해상도·인코딩·스탬프만 보고
    내용을 안 봤기 때문이다(2026-09-02). 검사에 밝기 확인을 넣어 함께 막았다.

    **평행광 + 약한 환경광**의 조합이다. 환경광만으로 밝기를 올리면 배경이 하얗게
    날아가고(실측), 평행광만 쓰면 그림자 안쪽이 새까맣다.

    `root_prim` 아래에 두므로 `setup_scene` 을 다시 돌리면 물체와 함께 재생성된다.
    """
    from pxr import Gf, UsdGeom, UsdLux

    key_path = f"{root_prim}/key_light"
    key = UsdLux.DistantLight.Define(stage, key_path)
    key.CreateIntensityAttr(DISTANT_LIGHT_INTENSITY)
    # 위에서 비스듬히 — 물체에 그림자가 져야 깊이가 보인다.
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 45.0))

    fill_path = f"{root_prim}/dome_light"
    fill = UsdLux.DomeLight.Define(stage, fill_path)
    fill.CreateIntensityAttr(DOME_LIGHT_INTENSITY)
    return [key_path, fill_path]


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

    카메라 파라미터(초점거리·조리개·클리핑)는 prim 에 두고, **픽셀 수는 여기서 정하지
    않는다** — 그것은 렌더 프로덕트의 몫이라 `setup_graph.py` 가 준다. 다만 세로 조리개는
    화면 비율을 따라야 해서 같은 `resolution` 값을 읽는다 (아래).
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

    # **조리개는 해상도 비율에 맞춰 준다.** 픽셀 수는 렌더 프로덕트가 정하지만, 세로
    # 조리개는 prim 이 갖는다 — USD 기본값(20.955 x 15.2908)의 비율 1.3704 가
    # 640x480(1.3333)과 어긋나 렌더러가 말없이 정사각 픽셀로 강제하고 한 줄만 남긴다:
    #
    #     Forcing fy to fx (753.39 != 733.00) ... as renderer assumes square pixels
    #
    # 그러면 **렌더된 세로 화각이 prim 에 적힌 값과 다르다.** `camera_info` 는 렌더러가
    # 실제로 쓴 값(fx=fy)을 실으므로 이미지와는 맞지만, prim 만 보고 화각을 계산하는
    # 쪽(캘리브레이션 대조, 시선 검증)은 4% 틀린 값을 얻는다.
    #
    # 해상도와 같은 키에서 유도하므로 `setup_graph.py` 의 렌더 프로덕트 크기와 자동으로
    # 함께 움직인다 — 해상도만 바꾸고 조리개를 잊는 일이 생기지 않는다.
    #
    # **가로 조리개 기본값이 USD 표준 20.955 가 아니라 21.0 인 이유**: USD 는 조리개를
    # float32 로 저장하고 Isaac 은 `fx == fy` 를 **정확히** 비교한다. 20.955 를 쓰면
    # 20.955*3/4 = 15.716249942... 가 float32 에 정확히 담기지 않아 fx 와 fy 가
    # 마지막 자리에서 갈라지고(2e-5), 비율을 맞춰 놓고도 경고가 계속 나온다. 21.0 은
    # 21.0 과 15.75 가 둘 다 float32 에 정확해 4:3·16:9 어디서도 정확히 일치한다
    # (848x480 같은 비표준 비율은 여전히 어긋난다). 화각 차이는 0.2% 로 무의미하다.
    width, height = spec.get("resolution", [640, 480])
    h_aperture = float(spec.get("horizontal_aperture", 21.0))
    camera.CreateHorizontalApertureAttr(h_aperture)
    camera.CreateVerticalApertureAttr(h_aperture * float(height) / float(width))

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
    _make_light(stage, root_prim)

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

    _log(f"[scene] lights: distant {DISTANT_LIGHT_INTENSITY} + dome {DOME_LIGHT_INTENSITY} "
         f"- dome paints the background, so keep it low")
    _log(f"[scene] done - {len(config['objects'])} object(s) under {root_prim}")
    _log("[scene] next: rerun setup_graph.py with PHASE=3 so TF is published")
    _log("[scene] next: run tune_grasp.py so the fingers can actually hold a block")


try:
    main()
finally:
    _flush_log()
