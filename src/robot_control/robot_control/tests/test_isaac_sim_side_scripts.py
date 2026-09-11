"""Isaac 쪽(`scripts/isaac/sim_side/`) 스크립트의 **경로 해석 계약**을 고정한다.

이 스크립트들은 Isaac Sim 안에서 돌아 ROS 도 ament 도 쓸 수 없다. 그래서 워크스페이스
경로와 로그 경로를 **각자 상수로 들고 있고**, 어느 배포(문서 §1)를 그 상수의
분기와 환경변수 override 로 지탱한다.

    A. Windows Isaac + WSL2 스택 (같은 머신)  -> UNC 기본값
    B. Ubuntu 단일 머신                       -> 로컬 경로 기본값
    C. Ubuntu 두 대                           -> RDFP_WORKSPACE / RDFP_LOG_DIR
    D. Windows Isaac + 별도 Ubuntu 스택       -> RDFP_WORKSPACE / RDFP_LOG_DIR

**여기서 검증하지 못하는 것이 있다.** 구성 B~D 는 다른 머신이 있어야 실제로 확인할 수
있고, 이 테스트는 그중 *경로가 올바로 갈라지는지*만 본다. 그것만으로도 값이 있는 이유는,
아홉 개 스크립트에 같은 블록이 복사되어 있어 **하나만 고치고 나머지를 빠뜨리기 쉽기**
때문이다. 공용 헬퍼로 뽑을 수 없는 것은 Isaac 쪽에서 `robot_control` 을 import 할 수
없어서다.

스크립트는 말미에서 `main()` 을 실행하므로 그대로 import 할 수 없다. 그 블록 앞까지만
잘라서 exec 한다.
"""

from __future__ import annotations

import ast
import contextlib
import pathlib
import sys
import types

import pytest

# .../src/robot_control/robot_control/tests/<this> -> 저장소 루트
#   parents[0]=tests  [1]=robot_control(모듈)  [2]=robot_control(패키지)  [3]=src  [4]=루트
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
SIM_SIDE_DIR = _REPO_ROOT / 'scripts' / 'isaac' / 'sim_side'

WSL_DISTRO_PREFIX = '//wsl.localhost/'


def _scripts() -> list[pathlib.Path]:
    if not SIM_SIDE_DIR.is_dir():
        return []
    return sorted(p for p in SIM_SIDE_DIR.glob('*.py'))


def _calls_main(node: ast.AST) -> bool:
    """이 최상위 노드가 `main()` 을 부르는가."""
    for inner in ast.walk(node):
        if (isinstance(inner, ast.Call) and isinstance(inner.func, ast.Name)
                and inner.func.id == 'main'):
            return True
    return False


def _exec_header(path: pathlib.Path) -> dict:
    """모듈 상단(실행 블록 이전)만 실행하고 네임스페이스를 돌려준다.

    실행 블록의 모양이 스크립트마다 다르다 — 대부분은 `try: main() finally:
    _flush_log()` 이고 `setup_graph.py` 는 맨 `main()` 이다. 문자열로 자르면 한쪽만
    맞으므로 AST 에서 **`main()` 을 부르는 첫 최상위 노드**를 찾아 그 앞까지만 쓴다.
    """
    source = path.read_text(encoding='utf-8')
    tree = ast.parse(source, filename=str(path))
    cut = next((node.lineno for node in tree.body if _calls_main(node)), None)
    head = source if cut is None else '\n'.join(source.splitlines()[:cut - 1])

    namespace: dict = {'__name__': f'_simside_{path.stem}', '__file__': str(path)}
    exec(compile(head, str(path), 'exec'), namespace)
    return namespace


pytestmark = pytest.mark.skipif(
    not _scripts(), reason='scripts/isaac/sim_side/ not present (installed tree)')


@contextlib.contextmanager
def _pretend_os_name(value: str):
    """스크립트가 보는 `os.name` 만 바꾼다.

    **`os.name` 을 전역으로 바꾸면 안 된다.** `pathlib` 이 그 값으로 구현을 고르므로
    `Path()` 가 `WindowsPath` 를 만들려다 예외를 내고, 그 예외가 pytest 의 리포터
    안에서 터져 실행 전체가 INTERNALERROR 로 죽는다.

    스크립트는 `import os as _os` 로 `sys.modules` 를 거쳐 가져오므로, exec 하는
    동안만 그 자리에 대역을 끼운다. `pathlib` 은 진짜 모듈을 이미 붙들고 있어
    영향받지 않는다.
    """
    real = sys.modules['os']
    fake = types.ModuleType('os')
    fake.__dict__.update(real.__dict__)   # environ 은 같은 객체라 setenv 가 그대로 통한다
    fake.name = value
    sys.modules['os'] = fake
    try:
        yield
    finally:
        sys.modules['os'] = real


def _ids(path: pathlib.Path) -> str:
    return path.name


@pytest.mark.parametrize('path', _scripts(), ids=_ids)
def test_header_runs_without_isaac(path: pathlib.Path):
    """상단 블록은 `omni` / `pxr` 없이도 실행돼야 한다.

    Isaac 모듈을 함수 안에서 import 하는 규약이 깨지면 여기서 걸린다. 모듈 최상단에서
    import 하면 스크립트를 Script Editor 밖에서 읽어보는 것조차 불가능해진다.
    """
    namespace = _exec_header(path)
    assert 'LOG_PATH' in namespace, f'{path.name} 에 LOG_PATH 상수가 없다'


@pytest.mark.parametrize('path', _scripts(), ids=_ids)
def test_log_dir_honors_env_override(path: pathlib.Path, monkeypatch):
    """구성 C·D — Isaac 머신과 스택 머신이 다르면 로그도 Isaac 머신에 떨어진다."""
    monkeypatch.setenv('RDFP_LOG_DIR', '/srv/isaac-logs')
    namespace = _exec_header(path)
    assert namespace['LOG_DIR'] == '/srv/isaac-logs'
    assert namespace['LOG_PATH'].startswith('/srv/isaac-logs/')


@pytest.mark.parametrize('path', _scripts(), ids=_ids)
def test_workspace_honors_env_override(path: pathlib.Path, monkeypatch):
    """저장소 사본 경로도 override 되어야 한다 (JSON 을 읽는 스크립트만 해당)."""
    namespace = _exec_header(path)
    if 'WORKSPACE' not in namespace:
        pytest.skip(f'{path.name} 은 워크스페이스를 읽지 않는다')

    monkeypatch.setenv('RDFP_WORKSPACE', '/srv/rdfp_ws')
    namespace = _exec_header(path)
    assert namespace['WORKSPACE'] == '/srv/rdfp_ws'
    for key in ('SCENE_JSON',):
        if key in namespace:
            assert namespace[key].startswith('/srv/rdfp_ws/')


@pytest.mark.parametrize('path', _scripts(), ids=_ids)
def test_linux_default_is_a_local_path(path: pathlib.Path, monkeypatch):
    """구성 B — Ubuntu 단일 머신에서는 UNC 가 아니라 로컬 경로여야 한다."""
    monkeypatch.delenv('RDFP_LOG_DIR', raising=False)
    monkeypatch.delenv('RDFP_WORKSPACE', raising=False)
    with _pretend_os_name('posix'):
        namespace = _exec_header(path)

    assert not namespace['LOG_DIR'].startswith(WSL_DISTRO_PREFIX)
    assert namespace['LOG_DIR'].startswith('/')
    if 'WORKSPACE' in namespace:
        assert not namespace['WORKSPACE'].startswith(WSL_DISTRO_PREFIX)


@pytest.mark.parametrize('path', _scripts(), ids=_ids)
def test_windows_default_is_a_unc_path(path: pathlib.Path, monkeypatch):
    """구성 A — Windows Isaac 은 WSL 파일시스템을 UNC 로 본다."""
    monkeypatch.delenv('RDFP_LOG_DIR', raising=False)
    monkeypatch.delenv('RDFP_WORKSPACE', raising=False)
    with _pretend_os_name('nt'):
        namespace = _exec_header(path)

    assert namespace['LOG_DIR'].startswith(WSL_DISTRO_PREFIX)
    if 'WORKSPACE' in namespace:
        assert namespace['WORKSPACE'].startswith(WSL_DISTRO_PREFIX)


def test_every_script_writes_a_log():
    """출력창을 복사할 수 없으므로 **모든 스크립트가 파일로도 남겨야 한다.**"""
    missing = [p.name for p in _scripts()
               if '_flush_log' not in p.read_text(encoding='utf-8')]
    assert not missing, f'로그를 남기지 않는 스크립트: {missing}'


def test_wsl_distro_name_is_consistent():
    """배포판 이름이 스크립트마다 다르면 일부만 조용히 실패한다.

    실제로 `Ubuntu` 와 `Ubuntu-22.04` 를 혼동해 파일을 못 여는 일이 있었다.
    """
    distros = set()
    for path in _scripts():
        for line in path.read_text(encoding='utf-8').splitlines():
            if WSL_DISTRO_PREFIX in line:
                tail = line.split(WSL_DISTRO_PREFIX, 1)[1]
                distros.add(tail.split('/')[0])
    assert len(distros) <= 1, f'배포판 이름이 여럿이다: {sorted(distros)}'


def _supported_object_types() -> set:
    """`setup_scene._make_object` 가 분기하는 `type` 문자열을 소스에서 뽑는다.

    호출해서 확인할 수 없다 — 함수 본문이 `pxr` 를 import 하기 때문이다. 그래서
    AST 로 `kind == "..."` 비교 상수만 걷는다.
    """
    path = SIM_SIDE_DIR / 'setup_scene.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    fn = next((n for n in ast.walk(tree)
               if isinstance(n, ast.FunctionDef) and n.name == '_make_object'), None)
    assert fn is not None, 'setup_scene._make_object 가 사라졌다'
    found = set()
    for node in ast.walk(fn):
        if isinstance(node, ast.Compare) and any(isinstance(o, ast.Eq) for o in node.ops):
            for side in [node.left] + list(node.comparators):
                if isinstance(side, ast.Constant) and isinstance(side.value, str):
                    found.add(side.value)
    return found


def test_setup_scene_can_build_every_type_in_the_json():
    """JSON 에 적힌 `type` 을 시뮬레이터 쪽이 실제로 만들 줄 아는가.

    **한쪽만 고치기 쉬운 자리다.** `isaac_scene.json` 은 ROS 쪽(`scene_state_node`)이
    타입을 그대로 옮기므로 새 종류를 적어도 `/scene/objects` 는 멀쩡히 그것을 말한다 —
    못 만드는 것은 `setup_scene` 뿐이고, 그 실패는 Isaac 출력창에만 남는다.
    """
    scene_json = _REPO_ROOT / 'src' / 'robot_control' / 'config' / 'isaac_scene.json'
    if not scene_json.is_file() or not (SIM_SIDE_DIR / 'setup_scene.py').is_file():
        pytest.skip('소스 트리가 아니다 (installed)')

    import json
    with open(scene_json, encoding='utf-8') as handle:
        objects = json.load(handle)['objects']

    supported = _supported_object_types()
    assert 'box' in supported, '분기 추출이 깨졌다'
    for spec in objects:
        kind = spec.get('type', 'box')
        assert kind in supported, (
            f'{spec["name"]}: isaac_scene.json 의 type {kind!r} 를 setup_scene 이 '
            f'만들지 못한다 (지원: {sorted(supported)})')
