"""계층 경계 테스트 — 아래 계층이 위 계층을 import 하지 않음을 고정한다.

패키지를 나눈 이유가 계층 분리이므로, 그 방향이 지켜지는지 기계로 확인한다.
문서로만 적어 두면 다음 사람이 편의상 import 하나를 추가하는 순간 조용히
깨진다. `package.xml` 의존 선언은 빌드 순서만 강제할 뿐 import 방향은 막지
않으므로 이 테스트가 유일한 방어선이다.
"""

from __future__ import annotations

import ast
import pathlib

import pytest

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]

# 이 패키지가 import 해서는 안 되는 최상위 모듈.
FORBIDDEN_ROOTS = {'rdfp'}

# 상위 계층(수집 계층)이 정의한 개념을 실어 나르는 메시지.
# `rdfp_msgs` 자체는 IDL 패키지라 FORBIDDEN_ROOTS 로 걸리지 않지만,
# 세션 상태 기계는 `rdfp.session` 이 소유하는 개념이므로 이 메시지를 쓰는
# 순간 계층이 뒤집힌다 — import 방향 검사만으로는 잡히지 않는 누수라 따로 막는다.
FORBIDDEN_MSGS = {'SessionCommand'}


def _source_files() -> list[pathlib.Path]:
    """테스트를 제외한 패키지 내 모든 파이썬 소스."""
    return [p for p in PACKAGE_ROOT.rglob('*.py')
            if 'tests' not in p.parts]


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
            # 상대 import (level > 0) 는 패키지 내부이므로 대상이 아니다.
            if node.level == 0 and node.module:
                roots.add(node.module.split('.')[0])
    return roots


@pytest.mark.parametrize('path', _source_files(), ids=lambda p: str(p.relative_to(PACKAGE_ROOT)))
def test_does_not_import_upper_layers(path: pathlib.Path) -> None:
    """상위 계층 패키지를 import 하지 않는다."""
    violations = _imported_roots(path) & FORBIDDEN_ROOTS
    assert not violations, (
        f'{path.relative_to(PACKAGE_ROOT)} imports upper-layer package(s) '
        f'{sorted(violations)}; use an entry-point seam instead '
        f'(see robot_twin.backend_registry)'
    )


def _imported_names(path: pathlib.Path) -> set[str]:
    """파일이 `rdfp_msgs` 에서 가져오는 메시지 이름 집합."""
    try:
        tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    except SyntaxError:
        return set()

    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            if node.module.split('.')[0] == 'rdfp_msgs':
                names.update(alias.name for alias in node.names)
    return names


@pytest.mark.parametrize('path', _source_files(), ids=lambda p: str(p.relative_to(PACKAGE_ROOT)))
def test_does_not_use_upper_layer_messages(path: pathlib.Path) -> None:
    """수집 계층이 소유한 메시지를 사용하지 않는다."""
    violations = _imported_names(path) & FORBIDDEN_MSGS
    assert not violations, (
        f'{path.relative_to(PACKAGE_ROOT)} uses upper-layer message(s) '
        f'{sorted(violations)}; a session-aware node belongs in the `rdfp` package '
        f'(see rdfp/camera/)'
    )
