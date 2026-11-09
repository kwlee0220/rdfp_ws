"""계층 경계 테스트 — `rdfp` 가 GUI 계층을 import 하지 않음을 고정한다.

데이터셋 GUI(`rdfp_gui`)는 `rdfp` **위에** 있다 — `rdfp.dataset.inspect` 가 공유 코드이고 GUI 가
그것을 쓴다. 방향이 뒤집히면(rdfp 가 Qt 나 GUI 패키지를 import) CLI 가 Qt 없는 환경에서 못 돌고,
GUI 전용 상태가 적재 경로에 섞인다. `package.xml` 의존 선언은 빌드 순서만 강제할 뿐 import 방향은
막지 않으므로, `robot_control/tests/test_layer_boundary.py` 와 같은 모양으로 AST 를 검사한다.
"""

from __future__ import annotations

import ast
import pathlib

import pytest

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]

# 이 패키지가 import 해서는 안 되는 최상위 모듈 — 데이터셋 GUI 패키지와 Qt 바인딩.
# (`replay_gui_cmd` 는 tkinter 라 걸리지 않는다.)
FORBIDDEN_ROOTS = {'rdfp_gui', 'rdfp_gui_codex', 'PySide6', 'PyQt5', 'PyQt6', 'shiboken6'}


def _source_files() -> list[pathlib.Path]:
    """테스트를 제외한 패키지 내 모든 파이썬 소스."""
    return [p for p in PACKAGE_ROOT.rglob('*.py') if 'tests' not in p.parts]


def _imported_roots(path: pathlib.Path) -> set[str]:
    """파일이 import 하는 최상위 모듈 이름 집합."""
    try:
        tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    except SyntaxError:
        return set()
    roots: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level == 0 and node.module:
                roots.add(node.module.split('.')[0])
    return roots


@pytest.mark.parametrize('path', _source_files(),
                         ids=lambda p: str(p.relative_to(PACKAGE_ROOT)))
def test_does_not_import_gui_layer(path: pathlib.Path) -> None:
    """GUI 계층(rdfp_gui · Qt)을 import 하지 않는다."""
    violations = _imported_roots(path) & FORBIDDEN_ROOTS
    assert not violations, (
        f'{path.relative_to(PACKAGE_ROOT)} imports GUI-layer module(s) {sorted(violations)}; '
        f'shared logic belongs in rdfp.dataset.inspect and the screen in rdfp_gui')
