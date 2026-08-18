"""`rdfp.twin.backends` 의 입력 검증과 핸들러 배선 테스트.

백엔드 실행 자체는 MoveGroup 클라이언트와 실제 로봇이 필요하므로, 여기서는
**로봇에 닿기 전에 걸러지는 부분**(설계서 6.9)과 연산-핸들러 배선만 확인한다.

`backends` 모듈은 최상위에서 ROS 를 import 하지 않지만(무거운 것은 지연 import),
`config` / `session` 은 필요하므로 pydantic 만 있으면 동작한다.
"""

from __future__ import annotations

from typing import Any

import pytest

from rdfp.twin.backends import (_HANDLERS, _require_joint_values, gripper_targets,
                                requires_move_group, validate_inputs, validate_operation_config)
from rdfp.twin.config import OperationConfig

VALID = {'panda_joint1': 0.0, 'panda_joint4': -1.57}

GRIPPER_BACKEND = {'topic': '/gripper_control/gripper_cmds',
                   'topic_type': 'rdfp_msgs/msg/GripperCommand',
                   'result_variable': 'gripper_last_command_result'}

# 그리퍼 목표 표를 가진 최소 연산 정의. 목표 이름도 폭도 코드가 아니라 설정이 정한다.
GRIPPER_OP = OperationConfig(
    name='move_gripper_to_target', kind='sync', resource='gripper',
    backend={**GRIPPER_BACKEND,
             'targets': {'open': {'position': 0.04}, 'close': {'position': 0.0}}}
)


# ----------------------------------------------------------------------
# 배선
# ----------------------------------------------------------------------

def test_implemented_operations_have_real_handlers() -> None:
    """구현된 연산이 `_not_wired` 로 남아 있지 않은지 본다.

    `_HANDLERS` 는 dict 리터럴이라 같은 키를 두 번 쓰면 **나중 것이 조용히
    이긴다.** move_to_joints 를 구현할 때 실제로 옛 `_not_wired` 항목이 남아
    새 핸들러를 덮고 있었다.
    """
    for name in ('move_to_named_target', 'move_to_joints', 'move_linear',
                 'move_gripper_to_target'):
        handler = _HANDLERS[name]
        assert handler.__name__ != '_handler', f"'{name}' is still wired to _not_wired()"


def test_unimplemented_operations_stay_unwired() -> None:
    for name in ('move_to_pose', 'move_gripper'):
        assert _HANDLERS[name].__name__ == '_handler'


def test_gripper_operations_are_not_split_per_target() -> None:
    """목표별 연산(open_gripper/close_gripper)으로 되돌아가지 않았는지 본다.

    목표는 `move_gripper_to_target` 의 입력이지 연산 이름이 아니다.
    """
    assert 'open_gripper' not in _HANDLERS
    assert 'close_gripper' not in _HANDLERS


# ----------------------------------------------------------------------
# move_gripper_to_target 입력 검증
# ----------------------------------------------------------------------

def test_gripper_targets_come_from_config() -> None:
    assert sorted(gripper_targets(GRIPPER_OP)) == ['close', 'open']
    # 목표 표가 없는 정의는 빈 dict 이다 — 조회 측이 방어할 필요가 없다.
    assert gripper_targets(OperationConfig(name='x', kind='sync')) == {}


@pytest.mark.parametrize('target', ['open', 'close'])
def test_configured_gripper_target_passes(target: str) -> None:
    validate_inputs('move_gripper_to_target', {'target': target}, GRIPPER_OP)


@pytest.mark.parametrize('raw', [None, '', 123, {'name': 'open'}])
def test_gripper_target_must_be_non_empty_string(raw: Any) -> None:
    with pytest.raises(ValueError, match='target'):
        validate_inputs('move_gripper_to_target', {'target': raw}, GRIPPER_OP)


def test_unknown_gripper_target_is_rejected_with_available_list() -> None:
    """jsonschema 미설치 환경에서도 400 으로 끊겨야 한다."""
    with pytest.raises(ValueError, match='available targets: close, open'):
        validate_inputs('move_gripper_to_target', {'target': 'half_open'}, GRIPPER_OP)


def test_gripper_target_check_is_skipped_without_config() -> None:
    """연산 정의가 없으면 목표 목록 검사는 건너뛴다 (형식 검사만 남는다)."""
    validate_inputs('move_gripper_to_target', {'target': 'half_open'})


# ----------------------------------------------------------------------
# 연산 정의 검증 (기동 시점)
# ----------------------------------------------------------------------

def test_configured_gripper_operation_passes_startup_check() -> None:
    validate_operation_config(GRIPPER_OP)


@pytest.mark.parametrize('backend', [{}, {'targets': {}}, {'targets': None}, {'targets': []}])
def test_gripper_operation_without_targets_is_rejected(backend: Any) -> None:
    """목표 표가 없으면 호출 시점이 아니라 기동 시점에 실패해야 한다.

    없어도 트윈은 뜨고 카탈로그에도 연산이 보이므로, 이 검사가 없으면 "연산은
    있는데 호출만 항상 EXECUTION_ABORTED" 가 된다.
    """
    op = OperationConfig(name='move_gripper_to_target', kind='sync', backend=backend)

    with pytest.raises(ValueError, match='backend.targets'):
        validate_operation_config(op)


@pytest.mark.parametrize('missing', ['topic', 'topic_type', 'result_variable'])
def test_gripper_operation_without_wiring_is_rejected(missing: str) -> None:
    """토픽·타입·결과 변수는 설정에서 온다 — 하나라도 없으면 기동을 막는다."""
    backend = {k: v for k, v in GRIPPER_BACKEND.items() if k != missing}
    op = OperationConfig(name='move_gripper_to_target', kind='sync',
                         backend={**backend, 'targets': {'open': {'position': 0.04}}})

    with pytest.raises(ValueError, match=f'backend.{missing}'):
        validate_operation_config(op)


@pytest.mark.parametrize('spec', ['0.04', {}, {'positon': 0.04}, {'position': None},
                                  {'position': 'wide'}, {'position': float('nan')},
                                  {'position': True}])
def test_target_without_valid_position_is_rejected(spec: Any) -> None:
    op = OperationConfig(name='move_gripper_to_target', kind='sync',
                         backend={**GRIPPER_BACKEND, 'targets': {'open': spec}})

    with pytest.raises(ValueError, match='open'):
        validate_operation_config(op)


def test_removed_service_form_is_rejected_with_migration_hint() -> None:
    """예전 `service:` 형태로 남은 설정이 조용히 무시되지 않도록 한다."""
    op = OperationConfig(name='move_gripper_to_target', kind='sync',
                         backend={**GRIPPER_BACKEND,
                                  'targets': {'open': {'service': '/open_gripper'}}})

    with pytest.raises(ValueError, match="removed 'service' form"):
        validate_operation_config(op)


def test_optional_max_effort_is_validated() -> None:
    validate_operation_config(OperationConfig(
        name='move_gripper_to_target', kind='sync',
        backend={**GRIPPER_BACKEND, 'targets': {'grasp': {'position': 0.02,
                                                          'max_effort': 30.0}}}
    ))

    bad = OperationConfig(name='move_gripper_to_target', kind='sync',
                          backend={**GRIPPER_BACKEND,
                                   'targets': {'grasp': {'position': 0.02,
                                                         'max_effort': 'hard'}}})
    with pytest.raises(ValueError, match='max_effort'):
        validate_operation_config(bad)


def test_move_group_requirement_comes_from_the_backend_declaration() -> None:
    """전제 조건은 연산이 **실제로 쓰는** 백엔드만 본다 (설계서 6.8).

    그리퍼 연산은 액션을 직접 호출하므로 팔 스택이 없어도 동작해야 한다. 코드에
    연산 이름 목록을 두지 않고 `backend.method` 유무로 판별한다.
    """
    assert requires_move_group(OperationConfig(name='move_to_named_target', kind='async',
                                               backend={'method': 'move_to_named_target_async'}))
    assert not requires_move_group(GRIPPER_OP)
    assert not requires_move_group(OperationConfig(name='move_gripper', kind='async',
                                                   backend={'action': '/gripper_cmd'}))
    assert not requires_move_group(OperationConfig(name='x', kind='sync'))


def test_operations_without_targets_are_not_required_to_have_them() -> None:
    validate_operation_config(OperationConfig(name='move_to_joints', kind='async'))
    validate_operation_config(OperationConfig(name='move_gripper', kind='async',
                                              backend={'action': '/gripper_cmd'}))


# ----------------------------------------------------------------------
# 명령 발행 — 결과 대기와 시간 예산
#
# 메시지 생성만 `_make_command` 에 격리되어 있으므로, 그 함수를 갈아끼우면 ROS 없이
# 호출 흐름 전체를 검증할 수 있다.
# ----------------------------------------------------------------------

class _FakeSnapshot:
    """`rdfp.twin.snapshot.Snapshot` 과 같은 모양 — 원본 메시지는 `msg` 에 있다.

    필드 이름을 `value` 로 잘못 짚었다가 실기동에서 터진 적이 있다. 실제 클래스와
    같은 속성명을 쓰는 것이 이 가짜의 유일한 계약이다.
    """

    def __init__(self, gen: int, **fields: Any) -> None:
        self.gen = gen
        self.msg = type('Msg', (), fields)()


class _FakeEntry:
    def __init__(self, snapshot: Any = None) -> None:
        self.snapshot = snapshot


class _FakeVariables:
    def __init__(self, entry: Any) -> None:
        self._entry = entry

    def entry(self, name: str) -> Any:
        return self._entry if name == 'gripper_last_command_result' else None


class _FakePublisher:
    def __init__(self, on_publish: Any = None) -> None:
        self.published: list[Any] = []
        self._on_publish = on_publish

    def publish(self, msg: Any) -> None:
        self.published.append(msg)
        if self._on_publish is not None:
            self._on_publish()


class _FakeRuntime:
    def __init__(self, publisher: Any, entry: Any) -> None:
        self._publisher = publisher
        self._node = object()
        self.variables = _FakeVariables(entry)

    def command_publisher(self, topic: str) -> Any:
        return self._publisher if topic == GRIPPER_BACKEND['topic'] else None


@pytest.fixture
def fake_command(monkeypatch) -> None:
    """ROS 메시지 생성만 갈아끼운다 — 나머지 흐름은 그대로 돈다."""
    monkeypatch.setattr('rdfp.twin.backends._make_command',
                        lambda node, position, max_effort, label: {'position': position,
                                                                   'max_effort': max_effort,
                                                                   'label': label})


def test_command_carries_a_stamp() -> None:
    """stamp 가 비면 저장기가 이 명령을 어느 에피소드에도 넣지 못한다.

    실기동에서 `sec: 0` 으로 나가는 것을 발견해 추가한 회귀 테스트다.
    """
    pytest.importorskip('rdfp_msgs')
    from builtin_interfaces.msg import Time

    from rdfp.twin.backends import _make_command

    # ROS 메시지 setter 는 타입을 검사하므로 진짜 Time 을 돌려줘야 한다.
    stamp = Time(sec=123, nanosec=456)

    class _Clock:
        def now(self):
            return type('T', (), {'to_msg': lambda _self: stamp})()

    class _Node:
        def get_clock(self):
            return _Clock()

    msg = _make_command(_Node(), 0.04, 0.0, 'open')

    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (123, 456)
    assert (msg.position, msg.max_effort, msg.label) == (0.04, 0.0, 'open')


def test_fake_snapshot_matches_the_real_one() -> None:
    """가짜가 실제 `Snapshot` 과 다른 속성을 쓰면 이 테스트가 먼저 깨진다."""
    from rdfp.twin.snapshot import Snapshot

    real = Snapshot(msg=object(), received_at=0.0, gen=1)
    fake = _FakeSnapshot(1, position=0.0)

    for attr in ('msg', 'gen'):
        assert hasattr(real, attr) and hasattr(fake, attr)


def test_command_result_is_reported_as_is(fake_command: None) -> None:
    """오차를 근거로 성패를 뒤집지 않는다 — 결과를 그대로 옮긴다 (설계서 6.7)."""
    from rdfp.twin.backends import _send_gripper_command

    entry = _FakeEntry(_FakeSnapshot(1, position=0.04, effort=0.0,
                                     stalled=False, reached_goal=True))

    def _arrive() -> None:
        # 명령이 나가면 gripper_control_node 가 결과를 재발행한 셈이다.
        entry.snapshot = _FakeSnapshot(2, position=0.018, effort=30.0,
                                       stalled=True, reached_goal=False)

    publisher = _FakePublisher(on_publish=_arrive)
    outputs = _send_gripper_command(_FakeRuntime(publisher, entry), GRIPPER_OP,
                                    position=0.0, max_effort=30.0, label='grasp', timeout=1.0)

    assert outputs == {'position': 0.018, 'effort': 30.0,
                       'stalled': True, 'reached_goal': False}
    assert publisher.published == [{'position': 0.0, 'max_effort': 30.0, 'label': 'grasp'}]


def test_stale_result_is_not_mistaken_for_this_command(fake_command: None) -> None:
    """이전 명령의 결과를 자기 것으로 착각하면 안 된다.

    발행 전 세대를 기억해 두고 **그보다 새 스냅샷**만 결과로 인정한다.
    """
    from rdfp.twin.backends import _send_gripper_command

    entry = _FakeEntry(_FakeSnapshot(7, position=0.04, effort=0.0,
                                     stalled=False, reached_goal=True))
    publisher = _FakePublisher()   # 결과가 영영 갱신되지 않는다

    with pytest.raises(TimeoutError, match='no time left'):
        _send_gripper_command(_FakeRuntime(publisher, entry), GRIPPER_OP,
                              position=0.04, max_effort=0.0, label='open', timeout=0.2)
    assert len(publisher.published) == 1


def test_missing_publisher_fails_fast(fake_command: None) -> None:
    """기동 시 퍼블리셔가 만들어지지 않았으면 명령을 보낼 수 없다."""
    from rdfp.twin.backends import BackendUnavailable, _send_gripper_command

    entry = _FakeEntry(_FakeSnapshot(1, position=0.0, effort=0.0,
                                     stalled=False, reached_goal=True))
    runtime = _FakeRuntime(None, entry)

    with pytest.raises(BackendUnavailable, match='no publisher'):
        _send_gripper_command(runtime, GRIPPER_OP, position=0.04, max_effort=0.0,
                              label='open', timeout=1.0)


def test_missing_result_variable_fails_fast(fake_command: None) -> None:
    from rdfp.twin.backends import BackendUnavailable, _send_gripper_command

    runtime = _FakeRuntime(_FakePublisher(), None)

    with pytest.raises(BackendUnavailable, match='result_variable'):
        _send_gripper_command(runtime, GRIPPER_OP, position=0.04, max_effort=0.0,
                              label='open', timeout=1.0)


# ----------------------------------------------------------------------
# move_to_joints 입력 검증
# ----------------------------------------------------------------------

@pytest.mark.parametrize('raw', [
    None, {}, [], 'panda_joint1', 3.14,
    {'panda_joint1': 'x'},          # 숫자가 아니다
    {'panda_joint1': True},         # bool 은 숫자로 받지 않는다
    {'panda_joint1': float('nan')},  # 유한하지 않다
    {'panda_joint1': float('inf')},
    {'': 1.0},                      # 이름이 비었다
])
def test_invalid_joint_values_are_rejected(raw: Any) -> None:
    with pytest.raises(ValueError):
        _require_joint_values(raw)


def test_valid_joint_values_are_normalized_to_float() -> None:
    out = _require_joint_values({'panda_joint1': 0, 'panda_joint2': -1})

    assert out == {'panda_joint1': 0.0, 'panda_joint2': -1.0}
    assert all(isinstance(v, float) for v in out.values())


def test_validate_inputs_requires_joints() -> None:
    with pytest.raises(ValueError):
        validate_inputs('move_to_joints', {})

    validate_inputs('move_to_joints', {'joints': VALID})   # 예외가 없어야 한다


def test_joint_names_are_not_checked_against_the_group() -> None:
    """그룹 소속 검사는 트윈이 하지 않는다 — 계획 단계에서 실패한다.

    이 동작에 기대는 문서(사용자 가이드 6.2)가 있으므로 고정해 둔다.
    """
    _require_joint_values({'panda_finger_joint1': 0.04, 'not_a_joint': 1.0})


def test_velocity_scaling_type_is_checked() -> None:
    with pytest.raises(ValueError):
        validate_inputs('move_to_joints', {'joints': VALID, 'velocity_scaling': 'fast'})
