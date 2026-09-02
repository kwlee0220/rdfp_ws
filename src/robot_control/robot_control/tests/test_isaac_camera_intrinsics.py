"""Isaac 카메라의 조리개가 해상도 비율과 맞는가.

Isaac 은 `camera_info` 의 초점거리를 USD 카메라 속성에서 뽑는다
(`isaacsim/ros2/core/impl/camera_info_utils.py`):

    fx = width  * focalLength / horizontalAperture
    fy = height * focalLength / verticalAperture

그리고 **`fy != fx` 면 fy 를 fx 로 덮어쓴다** — 렌더러가 정사각 픽셀을 가정하기
때문이다. 즉 조리개 비율이 해상도 비율과 다르면, prim 이 적어 놓은 세로 화각과 실제로
렌더된 세로 화각이 갈라지고 로그에는 경고 한 줄만 남는다:

    Forcing fy to fx (753.39 != 733.00) ... as renderer assumes square pixels

`setup_scene.py` 가 오랫동안 초점거리만 설정하고 조리개는 USD 기본값
(20.955 x 15.2908, 비율 1.3704)에 맡겨 640x480(1.3333)과 어긋나 있었다. 이미지 자체는
멀쩡해 보이므로 **경고를 읽지 않으면 드러나지 않는다.**

세 가지를 함께 본다.

1. 비율이 맞는가 — 4% 어긋나던 실제 결함.
2. float32 로 저장했을 때도 `fx == fy` 가 **정확히** 성립하는가. USD 는 조리개를
   단정밀도로 담고 Isaac 은 정확 비교를 하므로, 비율만 맞춰서는 마지막 자리가 갈라져
   (20.955 -> 2e-5) 경고가 계속 나온다. 그래서 기본값이 21.0 이다.
3. 스크립트가 그 산술을 **실제로 쓰는가**. 1·2 만으로는 세로 조리개를 상수로 박아 넣어도
   통과한다(검사를 만들며 변이로 확인했다). 그래서 `pxr` 을 흉내 내 `_make_camera` 를
   돌리고 prim 에 실제로 들어간 값을 받아 본다.
"""

from __future__ import annotations

import ast
import json
import pathlib

import pytest

# .../src/robot_control/robot_control/tests/<this> -> 저장소 루트
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
SETUP_SCENE = _REPO_ROOT / 'scripts' / 'isaac' / 'sim_side' / 'setup_scene.py'
SCENE_JSON = _REPO_ROOT / 'src' / 'robot_control' / 'config' / 'isaac_scene.json'


def _default_horizontal_aperture() -> float:
    """`setup_scene.py` 의 `spec.get("horizontal_aperture", <기본>)` 에서 기본값을 읽는다.

    스크립트를 import 할 수 없어(말미에서 `main()` 을 돈다) AST 로 뽑는다.
    """
    if not SETUP_SCENE.is_file():
        pytest.skip('설치 트리 (scripts/ 없음)')
    tree = ast.parse(SETUP_SCENE.read_text(encoding='utf-8'))
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'get' and len(node.args) == 2
                and isinstance(node.args[0], ast.Constant)
                and node.args[0].value == 'horizontal_aperture'):
            return float(ast.literal_eval(node.args[1]))
    pytest.fail('setup_scene.py 에서 horizontal_aperture 기본값을 찾지 못했다')


def _camera_resolution() -> tuple:
    if not SCENE_JSON.is_file():
        pytest.skip('설치 트리 (config/isaac_scene.json 없음)')
    with open(SCENE_JSON, encoding='utf-8') as handle:
        camera = json.load(handle)['camera']
    width, height = camera['resolution']
    return int(width), int(height), float(camera.get('focal_length', 24.0))


def _as_float32(value: float) -> float:
    """USD 가 조리개를 단정밀도로 담는 것을 흉내 낸다."""
    import struct

    return struct.unpack('f', struct.pack('f', value))[0]


def test_aperture_ratio_matches_the_resolution():
    """비율이 어긋나면 **렌더된 세로 화각이 prim 의 값과 다르다.**"""
    width, height, _ = _camera_resolution()
    h_aperture = _default_horizontal_aperture()
    v_aperture = h_aperture * height / width
    assert v_aperture / h_aperture == pytest.approx(height / width)
    # USD 기본값을 그대로 두면 이 검사가 잡아내는 바로 그 상태가 된다.
    assert v_aperture != pytest.approx(15.2908, abs=1e-4)


def test_fx_equals_fy_exactly_in_single_precision():
    """`camera_info_utils` 는 **정확 비교**를 한다 — 근사로는 경고가 남는다."""
    width, height, focal = _camera_resolution()
    h_aperture = _default_horizontal_aperture()
    v_aperture = h_aperture * height / width

    focal32 = _as_float32(focal)
    fx = width * focal32 / _as_float32(h_aperture)
    fy = height * focal32 / _as_float32(v_aperture)
    assert fx == fy, (
        f'fx={fx!r} != fy={fy!r} — Isaac 이 fy 를 덮어쓰며 경고를 남긴다. '
        f'horizontal_aperture({h_aperture})*{height}/{width} 가 float32 에 '
        f'정확히 담기지 않는다')


def test_usd_default_aperture_would_fail_this_check():
    """검사가 실제로 무엇을 막는지 못 박는다 — 회귀했을 때 통과하면 안 된다."""
    width, height, focal = _camera_resolution()
    focal32 = _as_float32(focal)
    fx = width * focal32 / _as_float32(20.955)      # USD 기본 가로
    fy = height * focal32 / _as_float32(15.2908)    # USD 기본 세로
    assert fx != fy
    assert abs(fy - fx) > 20.0                      # 실측 753.39 vs 733.00


# ── 3. 스크립트가 실제로 그 값을 넣는가 ──────────────────────────────────────


def _exec_header(path: pathlib.Path) -> dict:
    """말미의 `main()` 호출 앞까지만 실행한다 (test_isaac_sim_side_scripts 와 같은 수법)."""
    source = path.read_text(encoding='utf-8')
    tree = ast.parse(source, filename=str(path))

    def calls_main(node):
        return any(isinstance(i, ast.Call) and isinstance(i.func, ast.Name)
                   and i.func.id == 'main' for i in ast.walk(node))

    cut = next((n.lineno for n in tree.body if calls_main(n)), None)
    head = source if cut is None else '\n'.join(source.splitlines()[:cut - 1])
    namespace: dict = {'__name__': '_simside_setup_scene', '__file__': str(path)}
    exec(compile(head, str(path), 'exec'), namespace)
    return namespace


class _FakeCamera:
    """`UsdGeom.Camera.Define` 이 돌려주는 것 — 넣은 값을 그대로 들고 있는다."""

    def __init__(self):
        self.attrs: dict = {}

    def GetPrim(self):
        return object()

    def CreateFocalLengthAttr(self, value):
        self.attrs['focalLength'] = value

    def CreateHorizontalApertureAttr(self, value):
        self.attrs['horizontalAperture'] = value

    def CreateVerticalApertureAttr(self, value):
        self.attrs['verticalAperture'] = value

    def CreateClippingRangeAttr(self, value):
        self.attrs['clippingRange'] = value


def _fake_pxr(camera: '_FakeCamera'):
    import types

    class _Op:
        def Set(self, *_a, **_k):
            pass

    class _Xformable:
        def __init__(self, *_a):
            pass

        def ClearXformOpOrder(self):
            pass

        def AddTranslateOp(self):
            return _Op()

        def AddOrientOp(self):
            return _Op()

    gf = types.SimpleNamespace(Vec3d=lambda *a: a, Vec3f=lambda *a: a,
                               Vec2f=lambda *a: a, Quatf=lambda *a: a)
    usdgeom = types.SimpleNamespace(
        Camera=types.SimpleNamespace(Define=lambda _stage, _path: camera),
        Xformable=_Xformable)
    pxr = types.ModuleType('pxr')
    pxr.Gf = gf                                       # type: ignore[attr-defined]
    pxr.UsdGeom = usdgeom                             # type: ignore[attr-defined]
    return pxr


def test_make_camera_writes_the_derived_apertures():
    """세로 조리개를 상수로 박아 넣는 회귀를 잡는다."""
    import sys

    if not SETUP_SCENE.is_file() or not SCENE_JSON.is_file():
        pytest.skip('설치 트리')
    with open(SCENE_JSON, encoding='utf-8') as handle:
        spec = json.load(handle)['camera']

    namespace = _exec_header(SETUP_SCENE)
    camera = _FakeCamera()
    saved = sys.modules.get('pxr')
    sys.modules['pxr'] = _fake_pxr(camera)
    try:
        namespace['_make_camera'](object(), spec)
    finally:
        if saved is None:
            sys.modules.pop('pxr', None)
        else:
            sys.modules['pxr'] = saved

    width, height = spec['resolution']
    h_aperture = camera.attrs['horizontalAperture']
    v_aperture = camera.attrs['verticalAperture']
    assert h_aperture == pytest.approx(_default_horizontal_aperture())
    assert v_aperture == pytest.approx(h_aperture * height / width), (
        f'세로 조리개 {v_aperture} 가 해상도에서 유도되지 않았다')

    focal32 = _as_float32(camera.attrs['focalLength'])
    assert (width * focal32 / _as_float32(h_aperture)
            == height * focal32 / _as_float32(v_aperture))
