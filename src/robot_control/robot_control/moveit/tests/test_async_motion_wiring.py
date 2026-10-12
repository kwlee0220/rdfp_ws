#!/usr/bin/env python3

"""비동기 이동 경로의 배선 — **실측으로 드러난 두 결함의 재발 방지**.

둘 다 2026-09-18 에 R2 를 붙이며 mock 스택에서 드러났고, **에러도 로그도 없이**
"연산이 안 돌아온다" / "첫 스트리밍이 죽는다" 로만 나타났다.

1. **`_execute_joint_goal_async` 가 `execute_trajectory_async` 를 부르면 안 된다.**
   그쪽은 goal 수락을 `rclpy.spin_until_future_complete` 로 기다리는데, 이 경로는
   executor 콜백 안에서 돌기 때문에 그 spin 이 성립하지 않는다. 실측에서 45초를
   기다려도 future 가 안 끝났다 — **트윈이 쓰는 바로 그 경로다.**
2. **`TrajectoryStreamer` 가 발행자 필드를 `__init__` 에서 선언해야 한다.**
   `stream()` 이 `if self._motion_pub is None` 으로 지연 생성하는데 선언이 없어
   첫 호출이 `AttributeError` 로 죽었다.

소스를 AST 로 검사한다 — 실행하면 시뮬레이터가 필요하고, 두 결함 다 "구조"의
문제라 구조를 보는 것이 맞다.
"""

from __future__ import annotations

import ast
import inspect

from robot_control.moveit import move_group_jtc_client, trajectory_streamer


def _func(module, class_name: str, func_name: str) -> ast.FunctionDef:
    tree = ast.parse(inspect.getsource(module))
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            for item in ast.walk(node):
                if isinstance(item, ast.FunctionDef) and item.name == func_name:
                    return item
    raise AssertionError(f'{class_name}.{func_name} not found')


def _calls(func: ast.FunctionDef) -> set:
    out = set()
    for node in ast.walk(func):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            out.add(node.func.attr)
    return out


# ── 1. 비동기 관절 경로는 블로킹 호출을 타지 않는다 ─────────────────────────

def test_async_joint_goal_does_not_use_the_blocking_executor():
    """**트윈 경로가 영영 안 끝나던 원인이다.**

    `execute_trajectory_async` 는 goal 수락을 spin 으로 기다린다. 이 함수는
    계획 완료 콜백(= executor 스레드) 안에서 실행을 이어가므로 그 spin 이
    성립하지 않는다. 데카르트 비동기 경로처럼 `send_goal_async` +
    `add_done_callback` 이어야 한다.
    """
    called = _calls(_func(move_group_jtc_client, 'MoveGroupJtcClient',
                          '_execute_joint_goal_async'))

    assert 'execute_trajectory_async' not in called, (
        'goal 수락을 spin 으로 기다리는 함수다 — 콜백 안에서 부르면 안 끝난다')
    assert 'send_goal_async' in called and 'add_done_callback' in called


def test_async_joint_goal_forwards_externally_spun_to_planning():
    """`plan_joints_async` 는 이 값을 `_complete_joint_values` 로 흘리고, 거기서
    `False` 면 **노드를 직접 spin** 한다. 안 넘기면 executor 환경에서 계획이 멈춘다.
    """
    func = _func(move_group_jtc_client, 'MoveGroupJtcClient', '_execute_joint_goal_async')
    for node in ast.walk(func):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'plan_joints_async'):
            assert 'externally_spun' in {k.arg for k in node.keywords}
            return
    raise AssertionError('plan_joints_async 호출을 못 찾았다')


def test_async_entry_points_pass_externally_spun_down():
    """`move_to_named_target_async` · `move_to_joints_async` 가 인자를 흘려야 한다."""
    for name in ('move_to_named_target_async', 'move_to_joints_async'):
        func = _func(move_group_jtc_client, 'MoveGroupJtcClient', name)
        found = False
        for node in ast.walk(func):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                    and node.func.attr == '_execute_joint_goal_async'):
                assert 'externally_spun' in {k.arg for k in node.keywords}, name
                found = True
        assert found, f'{name} 이 _execute_joint_goal_async 를 안 부른다'


# ── 2. 스트리머의 지연 생성 필드는 선언돼 있어야 한다 ───────────────────────

def test_streamer_declares_its_lazy_publishers():
    """선언이 없으면 **첫 `stream()` 이 AttributeError 로 죽는다** — 실제로 그랬다."""
    init = _func(trajectory_streamer, 'TrajectoryStreamer', '__init__')
    assigned = set()
    for node in ast.walk(init):
        targets = ([node.target] if isinstance(node, ast.AnnAssign) else
                   list(node.targets) if isinstance(node, ast.Assign) else [])
        for t in targets:
            if isinstance(t, ast.Attribute) and isinstance(t.value, ast.Name) \
                    and t.value.id == 'self':
                assigned.add(t.attr)

    assert {'_motion_pub', '_event_pub'} <= assigned


def test_streamer_publishes_both_channels():
    """R1 은 첫 명령 직전, R2 는 끝나는 자리 — 둘 다 `stream` 안에 있어야 한다."""
    called = _calls(_func(trajectory_streamer, 'TrajectoryStreamer', 'stream'))

    assert 'publish' in called          # R1
    assert '_publish_event' in called   # R2
