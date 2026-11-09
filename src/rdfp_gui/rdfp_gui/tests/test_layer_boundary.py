"""계층 경계 — GUI 는 ROS 런타임을 import 하지 않고, 다른 GUI 구현에도 기대지 않는다.

GUI 와 조회는 **ROS 없이** 돌아야 한다(설계안 §5.1) — `rclpy` 나 메시지 패키지를 끌어오면 소싱 안 한
셸에서 창이 안 뜬다. 적재처럼 ROS 가 필요한 일은 서브프로세스(`ros2 run rdfp import`)로 넘긴다.
"""

from __future__ import annotations

import ast
import pathlib

import pytest

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]

FORBIDDEN_ROOTS = {'rclpy', 'rdfp_msgs', 'sensor_msgs', 'std_msgs', 'geometry_msgs',
                   'builtin_interfaces', 'rdfp_gui_codex'}


def _source_files() -> list[pathlib.Path]:
    return [p for p in PACKAGE_ROOT.rglob('*.py') if 'tests' not in p.parts]


def _imported_roots(path: pathlib.Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    roots: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            roots.add(node.module.split('.')[0])
    return roots


@pytest.mark.parametrize('path', _source_files(),
                         ids=lambda p: str(p.relative_to(PACKAGE_ROOT)))
def test_gui_stays_ros_free(path: pathlib.Path) -> None:
    violations = _imported_roots(path) & FORBIDDEN_ROOTS
    assert not violations, (
        f'{path.relative_to(PACKAGE_ROOT)} imports {sorted(violations)}; the GUI must run without '
        f'ROS — run ROS-dependent work through a CLI subprocess (rdfp_gui.jobs)')
