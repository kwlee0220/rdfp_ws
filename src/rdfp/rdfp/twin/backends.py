"""연산 백엔드 — rdfp 자산에 실제로 연결하는 계층 (설계서 6.14, 9장).

각 백엔드는 세션을 갱신하며 실행하고 ``outputs`` 를 반환한다. 트윈은 오차를
근거로 결과를 뒤집지 않는다 — ``goal_error`` / ``fraction`` 을 그대로 보고하고
성패 판단은 클라이언트에 맡긴다 (설계서 6.7).

미구현 백엔드는 ``NotImplementedError`` 대신 명시적 예외 메시지를 남긴다. 설계서
9장에 격차로 기록된 항목이다.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Callable, Optional

import json
import math
import random
import time

from rdfp.twin.config import OperationConfig
from rdfp.twin.session import Session

if TYPE_CHECKING:
    from rdfp.twin.runtime import RobotTwinRuntime

# 그리퍼 목표 이름이 어느 폭에 대응하는지, 어느 토픽으로 보내는지, 결과를 어느 변수로
# 받는지는 **설정이 정한다** (`move_gripper_to_target` 의 `backend.*`). 목표 추가가 설정
# 편집만으로 끝나야 하므로 이름도 토픽도 여기에 중복해 두지 않는다 — 두 곳에 두면
# 조용히 어긋난다.

# 목표 표(`backend.targets`) 와 명령 토픽 없이는 아무것도 할 수 없는 연산.
_TARGET_BACKED_OPERATIONS = frozenset({'move_gripper_to_target'})


class BackendUnavailable(RuntimeError):
    """백엔드가 아직 구현되지 않았거나 사용할 수 없을 때."""


class PreconditionFailed(RuntimeError):
    """백엔드가 **현재 상태에서 그 명령을 받을 수 없다**고 거절했을 때.

    `BackendUnavailable` 과 구분하는 이유는 오류 코드가 달라야 하기 때문이다.
    세션 상태 기계가 전이를 거부한 것은 실행이 중단된 것(`EXECUTION_ABORTED`)이
    아니라 **아무것도 실행되지 않은 것**이며, 클라이언트의 대응도 다르다 — 재시도가
    아니라 상태를 먼저 맞춰야 한다.
    """


def requires_move_group(op: OperationConfig) -> bool:
    """이 연산이 MoveGroup 클라이언트를 필요로 하는지 (설계서 6.8).

    **설정의 백엔드 선언으로 판별한다** — ``backend.method`` 는 ``MoveGroupClient`` 의
    메서드 이름이고, ``backend.topic`` 은 명령 토픽 발행이다. 코드에 연산 이름 목록을
    두면 연산을 늘릴 때마다 두 곳을 고쳐야 하고, 한쪽을 잊으면 멀쩡한 연산이 ``503``
    으로 거부된다.
    """
    return bool(op.backend.get('method'))


def validate_operation_config(op: OperationConfig) -> None:
    """연산 정의가 백엔드를 구동할 수 있는지 검사한다 — **기동 시점에** 호출한다.

    설정 스키마(pydantic)로는 표현할 수 없는 **백엔드 쪽 요구**를 본다. 어떤 연산이
    무엇을 요구하는지는 핸들러를 가진 이 모듈이 안다.

    이 검사가 없으면 목표 표가 빠진 설정으로도 트윈이 정상 기동하고 ``/health`` 도
    정상이며 카탈로그에도 연산이 보인다 — 호출해 봐야 ``EXECUTION_ABORTED`` 로
    알게 된다. 설정 오류는 기동 시점에 드러나야 한다 (설계서 2.6).

    Raises:
        ValueError: 정의가 백엔드를 구동할 수 없을 때. 호출자가 기동 실패로 바꾼다.
    """
    targets = op.backend.get('targets')

    if op.name in _TARGET_BACKED_OPERATIONS:
        if not (isinstance(targets, dict) and targets):
            raise ValueError(
                f"operation '{op.name}' requires a non-empty 'backend.targets' mapping, "
                f'got {targets!r}'
            )
        for key in ('topic', 'topic_type', 'result_variable'):
            if not op.backend.get(key):
                raise ValueError(f"operation '{op.name}' requires 'backend.{key}'")

    if targets is None:
        return
    if not isinstance(targets, dict):
        raise ValueError(f"operation '{op.name}': backend.targets must be a mapping")

    for name, spec in sorted(targets.items()):
        if not isinstance(spec, dict):
            raise ValueError(f"operation '{op.name}': target '{name}' must be a mapping")
        # 목표는 폭으로만 정의한다. 예전의 `service:` 형태는 `gripper_control_node`
        # 를 경유했는데, 그 경로는 goal 을 보내고 결과를 기다리지 않아 완료 판정이
        # 불가능했다.
        if 'service' in spec:
            raise ValueError(
                f"operation '{op.name}': target '{name}' uses the removed 'service' form; "
                "declare 'position' (and optionally 'max_effort') instead"
            )
        _require_finite_number(spec.get('position'), f"{op.name}: target '{name}' position")
        if spec.get('max_effort') is not None:
            _require_finite_number(spec['max_effort'], f"{op.name}: target '{name}' max_effort")


def _require_finite_number(raw: Any, what: str) -> float:
    """유한한 실수인지 확인한다 — bool 은 숫자로 받지 않는다."""
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise ValueError(f'{what} must be a number, got {raw!r}')
    if not math.isfinite(float(raw)):
        raise ValueError(f'{what} is not finite: {raw!r}')
    return float(raw)


def validate_inputs(op_name: str, inputs: dict[str, Any],
                    op: Optional[OperationConfig] = None) -> None:
    """연산 인자를 로봇 도메인 규칙으로 검증한다 (설계서 6.9).

    **세션 생성 전에** 호출되어야 한다. JSON Schema 로는 표현하기 어려운 규칙
    (quaternion 정규화 등)을 여기서 본다. 검증 없이 MoveIt 까지 보내면 수 초 뒤에
    불친절한 메시지로 실패하고, 그 사이 자원이 잠긴다.

    Args:
        op_name: 연산 이름.
        inputs: 검증할 입력.
        op: 연산 정의. 설정에 의존하는 검사(그리퍼 목표 목록 등)에만 쓰이며,
            없으면 그 검사를 건너뛴다.

    Raises:
        ValueError: 인자가 규칙을 만족하지 않을 때. 호출자가 ``400`` 으로 바꾼다.
    """
    if op_name == 'move_to_named_target':
        target = inputs.get('target')
        if not isinstance(target, str) or not target:
            raise ValueError("input 'target' (SRDF group_state name) is required")

    elif op_name == 'move_to_joints':
        _require_joint_values(inputs.get('joints'))

    elif op_name == 'move_linear':
        _require_pose(inputs.get('pose'))

    elif op_name == 'move_gripper_to_target':
        _require_gripper_target(inputs.get('target'), op)

    elif op_name == 'stop_episode':
        _require_outcome(inputs.get('outcome'))
        _require_metadata(inputs.get('metadata'))

    for key in ('velocity_scaling', 'acceleration_scaling'):
        raw = inputs.get(key)
        if raw is not None and not isinstance(raw, (int, float)):
            raise ValueError(f'{key} must be a number')

    duration = inputs.get('max_duration_sec')
    if duration is not None and (not isinstance(duration, (int, float)) or duration <= 0):
        raise ValueError('max_duration_sec must be a positive number')


def dispatch(runtime: 'RobotTwinRuntime', client: Optional[Any], op: OperationConfig,
             session: Session, *, timeout: float) -> dict[str, Any]:
    """연산 이름으로 백엔드를 골라 실행한다.

    Args:
        runtime: 상태 캐시·세션 접근을 위한 런타임.
        client: ``MoveGroupClient`` 구현. 준비 전이면 ``None``.
        op: 연산 정의.
        session: 실행 중인 세션.
        timeout: 이 호출의 상한(초).

    Returns:
        ``outputs`` 로 쓸 dict.

    Raises:
        BackendUnavailable: 미구현 연산이거나 백엔드가 없을 때.
        TimeoutError: 상한을 넘겼을 때.
    """
    handler = _HANDLERS.get(op.name)
    if handler is None:
        raise BackendUnavailable(
            f"operation '{op.name}' has no backend wired; "
            'see docs/robot_twin/robot_twin_design.md chapter 9'
        )
    return handler(runtime, client, op, session, timeout)


# ----------------------------------------------------------------------
# arm — 사용 가능한 백엔드
# ----------------------------------------------------------------------

def _move_to_named_target(runtime: 'RobotTwinRuntime', client: Optional[Any],
                          op: OperationConfig, session: Session,
                          timeout: float) -> dict[str, Any]:
    """SRDF named target 으로 이동한다 — `move_to_named_target_async` 를 쓴다."""
    if client is None:
        raise BackendUnavailable('MoveGroup client is not ready')

    target = session.inputs.get('target')
    if not isinstance(target, str) or not target:
        raise ValueError("input 'target' (SRDF group_state name) is required")

    # externally_spun=True 는 필수다 — executor 가 다른 스레드에서 spin 중임을
    # 알려야 스트리밍 구현이 자체 spin 을 시도하지 않는다 (설계서 2.3).
    future = client.move_to_named_target_async(
        target, velocity_scaling=_scaling(session, 'velocity_scaling'),
        externally_spun=True
    )
    _await(future, timeout=timeout, what=f"move_to_named_target('{target}')")
    return _motion_outputs(runtime, closed_loop=_is_closed_loop(runtime))


def _move_to_joints(runtime: 'RobotTwinRuntime', client: Optional[Any],
                    op: OperationConfig, session: Session,
                    timeout: float) -> dict[str, Any]:
    """관절 목표값으로 이동한다 — `move_to_joints_async` 를 쓴다."""
    if client is None:
        raise BackendUnavailable('MoveGroup client is not ready')

    joints = _require_joint_values(session.inputs.get('joints'))
    future = client.move_to_joints_async(
        joints, velocity_scaling=_scaling(session, 'velocity_scaling'), externally_spun=True
    )
    _await(future, timeout=timeout, what=f'move_to_joints({len(joints)} joints)')
    return _motion_outputs(runtime, closed_loop=_is_closed_loop(runtime))


def _move_linear(runtime: 'RobotTwinRuntime', client: Optional[Any],
                 op: OperationConfig, session: Session,
                 timeout: float) -> dict[str, Any]:
    """Cartesian 직선 경로로 이동한다 — `follow_trajectory_async` 를 쓴다.

    ``fraction`` 을 ``outputs`` 에 담는다. ``fraction < 1.0`` 을 성공으로 볼지는
    **클라이언트가 판단**한다 (설계서 6.14).
    """
    if client is None:
        raise BackendUnavailable('MoveGroup client is not ready')

    pose = _require_pose(session.inputs.get('pose'))
    waypoints = [_to_ros_pose(pose)]

    kwargs: dict[str, Any] = {'externally_spun': True,
                              'velocity_scaling': _scaling(session, 'velocity_scaling')}
    for key in ('max_step', 'jump_threshold'):
        if session.inputs.get(key) is not None:
            kwargs[key] = float(session.inputs[key])

    future = client.follow_trajectory_async(waypoints, **kwargs)
    result = _await(future, timeout=timeout, what='move_linear')

    outputs = _motion_outputs(runtime, closed_loop=_is_closed_loop(runtime))
    fraction = getattr(result, 'fraction', None)
    if fraction is not None:
        outputs['fraction'] = float(fraction)
    return outputs


# ----------------------------------------------------------------------
# gripper — 사용 가능한 백엔드
# ----------------------------------------------------------------------

def gripper_targets(op: OperationConfig) -> dict[str, dict[str, Any]]:
    """연산 정의에서 목표 이름 → 목표 정의 매핑을 꺼낸다.

    ``backend.targets`` 가 곧 지원 목표 목록이다. 목표를 늘리는 일이 설정 편집만으로
    끝나도록 코드에는 목표 이름을 두지 않는다.
    """
    targets = op.backend.get('targets')
    return targets if isinstance(targets, dict) else {}


def _make_command(node: Any, position: float, max_effort: float, label: str) -> Any:
    """``rdfp_msgs/GripperCommand`` 메시지를 만든다.

    ROS 의존성을 이 함수 하나에 가둔다 — 호출 흐름(예산 배분, 결과 해석)은 ROS 없이
    테스트할 수 있어야 한다.

    ``position`` 은 컨트롤러가 물린 관절 하나의 목표값이며(Panda 는
    ``panda_finger_joint1``), 손가락 **사이 거리가 아니라 그 절반**이다 — 실기 franka 의
    ``move``/``grasp`` 가 쓰는 ``width`` 와 2배 차이가 난다.
    """
    from rdfp_msgs.msg import GripperCommand

    msg = GripperCommand()
    # **stamp 를 반드시 채운다.** 저장기가 에피소드를 시각으로 자르므로, 비워 두면
    # 이 명령이 epoch 0 에 적재되어 어느 에피소드에도 속하지 않는다.
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.position = float(position)
    msg.max_effort = float(max_effort)
    # label 은 제어에 쓰이지 않는다. 데이터셋 가독성을 위한 값이다.
    msg.label = str(label)
    return msg


def _send_gripper_command(runtime: 'RobotTwinRuntime', op: OperationConfig, *,
                          position: float, max_effort: float, label: str,
                          timeout: float) -> dict[str, Any]:
    """명령 토픽에 발행하고 **결과 변수가 갱신될 때까지** 기다린다.

    액션을 직접 부르지 않는 이유는 두 가지다 (설계서 6.14).

    1. 액션 goal 전송은 서비스라 **rosbag2 가 기록하지 못한다.** 토픽으로 흘려야
       학습 데이터의 action 채널이 남는다.
    2. 액션 호출자를 ``gripper_control_node`` 하나로 모아야 결과 채널
       (``gripper_action_states``)이 생성기마다 갈리지 않는다.

    결과는 ``backend.result_variable`` 이 가리키는 상태 변수로 받는다. 이 변수는
    기동 시점부터 구독되어 있으므로 **발행 직후의 결과를 놓치지 않는다** — 호출 때마다
    구독을 만들면 DDS 매칭 전에 결과가 지나가 버린다.

    Returns:
        결과 필드를 그대로 담은 dict. 오차를 근거로 성패를 뒤집지 않는다 (설계서 6.7).

    Raises:
        BackendUnavailable: 퍼블리셔나 결과 변수가 준비되지 않았을 때.
        TimeoutError: 예산 안에 결과가 오지 않았을 때.
    """
    topic = op.backend.get('topic')
    publisher = runtime.command_publisher(topic) if topic else None
    if publisher is None:
        raise BackendUnavailable(f"operation '{op.name}' has no publisher for topic {topic!r}")

    entry = runtime.variables.entry(op.backend.get('result_variable') or '')
    if entry is None:
        raise BackendUnavailable(
            f"operation '{op.name}': backend.result_variable is not a declared variable"
        )

    deadline = time.monotonic() + timeout
    # 발행 **전에** 현재 세대를 기록한다. 이후 세대가 올라간 스냅샷만 이 명령의
    # 결과로 인정한다 — 이전 명령의 결과를 자기 것으로 착각하지 않기 위해서다.
    before = _snapshot_gen(entry)
    node = runtime._node  # noqa: SLF001 — 런타임 내부 협력자다
    publisher.publish(_make_command(node, position, max_effort, label))

    while True:
        snap = entry.snapshot
        if snap is not None and snap.gen != before:
            # `Snapshot.msg` 는 원본 ROS 메시지다 — 변환하지 않고 그대로 들고 있다
            # (설계서 2.2). 필드가 없는 구현을 만나도 죽지 않게 방어한다.
            result = snap.msg
            return {'position': float(getattr(result, 'position', 0.0)),
                    'effort': float(getattr(result, 'effort', 0.0)),
                    'stalled': bool(getattr(result, 'stalled', False)),
                    'reached_goal': bool(getattr(result, 'reached_goal', False))}
        _remaining(deadline, f"{op.name} result on '{topic}'", timeout)
        time.sleep(0.02)


def _snapshot_gen(entry: Any) -> Any:
    """결과 변수의 현재 세대. 스냅샷이 없으면 ``None``."""
    snap = entry.snapshot
    return None if snap is None else snap.gen


def _remaining(deadline: float, what: str, budget: float) -> float:
    """예산의 남은 시간을 돌려준다. 이미 소진했으면 기다리지 않고 실패한다."""
    remaining = deadline - time.monotonic()
    if remaining <= 0.0:
        raise TimeoutError(f"no time left for '{what}' within {budget}s")
    return remaining


def _move_gripper_to_target(runtime: 'RobotTwinRuntime', client: Optional[Any],
                            op: OperationConfig, session: Session,
                            timeout: float) -> dict[str, Any]:
    """이름 붙은 그리퍼 목표로 이동한다 — 목표 정의는 설정에서 읽는다.

    목표는 ``{position, max_effort}`` 로 정의한다. ``max_effort`` 는 생략하면 ``0``
    이며, 그 값은 "드라이버 기본 효과치"라는 뜻이다 (mock 의 ``GripperActionController``
    기준). 실기 franka 에서는 ``max_effort = 0`` 이 파지 없는 ``move`` 로 해석될 수
    있으므로 물체를 쥐려면 명시해야 한다.
    """
    del client  # MoveGroup 을 쓰지 않는다
    target = session.inputs.get('target')
    targets = gripper_targets(op)
    spec = targets.get(target) if isinstance(target, str) else None
    if not isinstance(spec, dict):
        available = ', '.join(sorted(targets)) or '(none configured)'
        raise BackendUnavailable(
            f"unknown gripper target {target!r}; configured targets: {available}"
        )

    outputs = _send_gripper_command(runtime, op, position=float(spec['position']),
                                    max_effort=float(spec.get('max_effort') or 0.0),
                                    label=str(target), timeout=timeout)
    return {'target': target, **outputs}


# ----------------------------------------------------------------------
# 미구현 — 설계서 9장의 격차
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
# 씬 — 백엔드별 scene_state_node 에 명령 토픽으로 보낸다
# ----------------------------------------------------------------------

def scene_recipes(op: OperationConfig) -> dict[str, Any]:
    """`reset_scene` 의 씬 레시피 표. 설정이 단일 출처다."""
    recipes = op.backend.get('scenes')
    return recipes if isinstance(recipes, dict) else {}


def _sample_scene(recipe: dict[str, Any], seed: int) -> list[dict[str, Any]]:
    """레시피에서 물체 배치를 뽑는다.

    **무작위 추출은 트윈이 한다.** 백엔드 노드가 뽑으면 (1) 같은 seed 로도 백엔드마다
    다른 배치가 나오고, (2) 트윈이 실제 배치를 몰라 에피소드 metadata 에 남길 수 없다.

    각 축은 숫자(고정) 또는 ``[최소, 최대]`` (균등 추출)다. `random.Random(seed)` 를
    쓰므로 같은 seed 는 같은 배치를 준다 — 다만 **재현의 근거는 seed 가 아니라 반환된
    배치 자체**다. 추출 방식이 바뀌면 같은 seed 가 다른 결과를 낸다.
    """
    rng = random.Random(seed)
    out: list[dict[str, Any]] = []
    for index, spec in enumerate(recipe.get('objects') or []):
        name = spec.get('name') or f"{spec.get('type', 'object')}_{index}"
        out.append({
            'name': str(name),
            'type': str(spec.get('type', 'box')),
            'dimensions': [float(v) for v in (spec.get('size') or [])],
            'position': {axis: _sample_axis(rng, spec.get(axis, 0.0), f'{name}.{axis}')
                         for axis in ('x', 'y', 'z')},
        })
    return out


def _sample_axis(rng: 'random.Random', raw: Any, what: str) -> float:
    """숫자면 그대로, ``[최소, 최대]`` 면 그 구간에서 균등 추출한다."""
    if isinstance(raw, (int, float)) and not isinstance(raw, bool):
        return float(raw)
    if isinstance(raw, (list, tuple)) and len(raw) == 2:
        low, high = (_require_finite_number(v, what) for v in raw)
        if low > high:
            raise ValueError(f'{what}: range is inverted ({low} > {high})')
        return rng.uniform(low, high)
    raise ValueError(f'{what} must be a number or a [min, max] pair, got {raw!r}')


def _reset_scene(runtime: 'RobotTwinRuntime', client: Optional[Any],
                 op: OperationConfig, session: Session, timeout: float) -> dict[str, Any]:
    """씬을 레시피대로 새로 만든다.

    저수준 spawn/remove 를 열지 않고 레시피 이름 하나로 받는 이유는, 씬 구성 로직이
    클라이언트로 새어 나가면 **"어떤 배치였는지"가 데이터셋 밖에 남기** 때문이다.

    ``outputs.objects`` 가 실제 배치이며 **이것이 재현의 근거다.** seed 만 남기면
    추출 방식이 바뀌는 순간 재현이 깨진다 — 호출자는 이 값을 `stop_episode` 의
    metadata 로 넘겨 에피소드에 붙인다.
    """
    recipes = scene_recipes(op)
    name = session.inputs.get('scene')
    recipe = recipes.get(name) if isinstance(name, str) else None
    if recipe is None:
        raise ValueError(f"unknown scene {name!r}; declared: {sorted(recipes)}")

    seed = session.inputs.get('seed')
    seed = 0 if seed is None else int(seed)
    objects = _sample_scene(recipe, seed)

    result = _send_scene_command(runtime, op, scene=str(name), seed=seed, objects=objects,
                                 timeout=timeout)
    return {'scene': name, 'seed': seed, 'objects': objects, **result}


def _send_scene_command(runtime: 'RobotTwinRuntime', op: OperationConfig, *, scene: str,
                        seed: int, objects: list[dict[str, Any]],
                        timeout: float) -> dict[str, Any]:
    """명령 토픽에 발행하고 결과 변수가 갱신될 때까지 기다린다.

    그리퍼와 같은 구조다 (설계서 6.14) — 결과를 서비스 응답으로만 받으면 rosbag2 에
    남지 않아 나중에 리셋 실패를 알 수 없다.

    **`/scene/objects` 를 결과 채널로 쓰지 않는다.** 그쪽은 주기 발행이라 명령과
    무관하게 계속 갱신되므로, '갱신됨'을 완료 신호로 삼으면 리셋 이전 상태를 완료로
    오인한다.
    """
    topic = op.backend.get('topic')
    publisher = runtime.command_publisher(topic) if topic else None
    if publisher is None:
        raise BackendUnavailable(f"operation '{op.name}' has no publisher for topic {topic!r}")

    entry = runtime.variables.entry(op.backend.get('result_variable') or '')
    if entry is None:
        raise BackendUnavailable(
            f"operation '{op.name}': backend.result_variable is not a declared variable"
        )

    deadline = time.monotonic() + timeout
    before = _snapshot_gen(entry)
    node = runtime._node  # noqa: SLF001 — 런타임 내부 협력자다
    publisher.publish(_make_scene_command(node, scene, seed, objects))

    while True:
        snap = entry.snapshot
        if snap is not None and snap.gen != before:
            result = snap.msg
            if not getattr(result, 'success', False):
                raise BackendUnavailable(
                    f'scene reset failed: {getattr(result, "message", "") or "unknown"}')
            return {'applied_count': int(getattr(result, 'applied_count', 0))}
        _remaining(deadline, f"{op.name} result on '{topic}'", timeout)
        time.sleep(0.02)


def _make_scene_command(node: Any, scene: str, seed: int,
                        objects: list[dict[str, Any]]) -> Any:
    """`rdfp_msgs/SceneCommand` 를 만든다. `header.stamp` 는 반드시 채운다."""
    from rdfp_msgs.msg import SceneCommand, SceneObject

    msg = SceneCommand()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.command = 'reset'
    msg.scene = scene
    msg.seed = int(seed)
    for spec in objects:
        obj = SceneObject()
        obj.name = spec['name']
        obj.type = spec['type']
        obj.dimensions = [float(v) for v in spec['dimensions']]
        position = spec['position']
        obj.pose.position.x = float(position['x'])
        obj.pose.position.y = float(position['y'])
        obj.pose.position.z = float(position['z'])
        # 계약은 ROS xyzw 다. 레시피에 자세가 없으므로 회전 없음으로 둔다.
        obj.pose.orientation.w = 1.0
        msg.objects.append(obj)
    return msg


# ----------------------------------------------------------------------
# 세션 / 에피소드 경계 — session_control_node 의 클라이언트로 동작한다
# ----------------------------------------------------------------------

def _session_client(runtime: 'RobotTwinRuntime') -> Any:
    """준비된 `SessionControlClient` 를 돌려준다.

    Raises:
        BackendUnavailable: `session_control_node` 가 아직 없을 때. 런타임이
            ``PRECONDITION_FAILED`` 로 바꾼다.
    """
    client = runtime.session_control()
    if not client.is_ready():
        raise BackendUnavailable(
            'session_control_node is not available; '
            'start it with `ros2 run rdfp session_control_node`'
        )
    return client


def _call_session(future: Any, *, timeout: float, what: str) -> None:
    """세션 서비스 호출 결과를 확인한다.

    서비스는 예외를 던지지 않고 ``(success, message)`` 를 돌려준다. 여기서 success 는
    **명령 수용 여부**이며 작업의 성패가 아니다 — 상태 기계가 전이를 거부하면
    False 가 오고, 그 사유를 그대로 올려야 클라이언트가 원인을 안다.

    Raises:
        PreconditionFailed: 서버가 거부했을 때. 런타임이 ``PRECONDITION_FAILED``
            로 바꾼다 (설계 초안 §4.2).
        BackendUnavailable: 응답 자체가 없을 때.
        TimeoutError: 상한 내에 끝나지 않았을 때.
    """
    response = _await(future, timeout=timeout, what=what)
    if response is None:
        raise BackendUnavailable(f'{what}: no response from session_control_node')
    if not response.success:
        raise PreconditionFailed(f'{what} rejected: {response.message or "invalid command"}')


def _start_session(runtime: 'RobotTwinRuntime', client: Optional[Any],
                   op: OperationConfig, session: Session, timeout: float) -> dict[str, Any]:
    """세션을 연다. ``task_label`` 을 함께 설정한다.

    라벨을 별도 연산으로 두지 않고 여기서 받는 이유는, 빈 라벨로 시작하면 **그
    세션의 모든 에피소드 행에 빈 라벨이 박히고 기록이 끝난 뒤에는 재import 로도
    고칠 수 없기** 때문이다 (bag 안의 `/session` 메시지가 이미 비어 있다).

    순서가 중요하다 — `set_task_label` 은 `IN_EPISODE` 에서 거부되므로 `IDLE` 일 때
    먼저 부른다.
    """
    sc = _session_client(runtime)
    deadline = time.monotonic() + timeout

    task_label = session.inputs.get('task_label')
    if task_label is not None:
        remaining = _remaining(deadline, 'set_task_label', timeout)
        _call_session(sc.set_task_label_async(str(task_label)),
                      timeout=remaining, what='set_task_label')

    remaining = _remaining(deadline, 'start_session', timeout)
    _call_session(sc.start_session_async(), timeout=remaining, what='start_session')
    return {'task_label': '' if task_label is None else str(task_label)}


def _stop_session(runtime: 'RobotTwinRuntime', client: Optional[Any],
                  op: OperationConfig, session: Session, timeout: float) -> dict[str, Any]:
    """세션을 닫는다. `IN_EPISODE` 였다면 에피소드도 함께 닫힌다.

    노드가 (IN_SESSION → IDLE) 을 2회 발행하므로 구독자가 두 전이를 순서대로
    관찰한다 — 수집 루프가 시작 전에 이것을 한 번 부르면 이전 실행이 남긴 열린
    에피소드가 정리된다 (멱등한 시작).
    """
    _call_session(_session_client(runtime).stop_session_async(),
                  timeout=timeout, what='stop_session')
    return {}


def _start_episode(runtime: 'RobotTwinRuntime', client: Optional[Any],
                   op: OperationConfig, session: Session, timeout: float) -> dict[str, Any]:
    """에피소드를 연다. `IN_SESSION` 에서만 허용된다."""
    _call_session(_session_client(runtime).start_episode_async(),
                  timeout=timeout, what='start_episode')
    return {}


def _stop_episode(runtime: 'RobotTwinRuntime', client: Optional[Any],
                  op: OperationConfig, session: Session, timeout: float) -> dict[str, Any]:
    """에피소드를 닫고 성패·부가정보를 함께 남긴다.

    두 값은 `SessionCommand` 로 발행되어 rosbag2 에 기록되고, 후처리 시 `sessions`
    테이블의 `success` / `metadata` 가 된다. 서비스 응답으로만 주고받으면 기록되지
    않아 데이터셋에 남지 않는다.

    **트윈은 이 연산을 자동으로 부르지 않는다.** 작업이 끝났든 실패했든 에피소드를
    닫는 것은 클라이언트의 역할이다 — 자동 수집에서 실패는 정상 경로이므로(파지
    실패도 학습 데이터다) 트윈이 판단하면 유효한 실패 에피소드를 잘라먹는다.
    """
    outcome = session.inputs.get('outcome')
    metadata = session.inputs.get('metadata')
    _call_session(
        _session_client(runtime).stop_episode_async(
            outcome='' if outcome is None else str(outcome),
            metadata='' if metadata is None else json.dumps(metadata)),
        timeout=timeout, what='stop_episode')
    return {'outcome': '' if outcome is None else str(outcome)}


def _not_wired(reason: str) -> Callable[..., dict[str, Any]]:
    """rdfp 쪽 공개 API 가 없어 아직 연결하지 못한 연산."""

    def _handler(runtime: 'RobotTwinRuntime', client: Optional[Any], op: OperationConfig,
                 session: Session, timeout: float) -> dict[str, Any]:
        raise BackendUnavailable(f"operation '{op.name}' is not implemented yet: {reason}")

    return _handler


_HANDLERS: dict[str, Callable[..., dict[str, Any]]] = {
    'move_to_named_target': _move_to_named_target,
    'move_to_joints': _move_to_joints,
    'move_linear': _move_linear,
    'move_gripper_to_target': _move_gripper_to_target,
    'reset_scene': _reset_scene,
    'start_session': _start_session,
    'stop_session': _stop_session,
    'start_episode': _start_episode,
    'stop_episode': _stop_episode,
    'move_to_pose': _not_wired(
        'free-space planning to a pose goal requires PositionConstraint/'
        'OrientationConstraint or an IK call; MoveGroupClient exposes neither'
    ),
    'move_gripper': _not_wired(
        'arbitrary width is not exposed; declare the width as a named target on '
        'move_gripper_to_target instead'
    )
}


# ----------------------------------------------------------------------
# 공통 헬퍼
# ----------------------------------------------------------------------

def _await(future: Any, *, timeout: float, what: str) -> Any:
    """``Future`` 완료를 기다린다.

    **spin 하지 않는다** — executor 가 별도 스레드에서 이미 돌고 있으므로 여기서
    spin 하면 중복 실행이 된다 (설계서 2.3).

    Raises:
        TimeoutError: 상한 내에 끝나지 않았을 때.
    """
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() >= deadline:
            future.cancel()
            raise TimeoutError(f'{what} did not finish within {timeout}s')
        time.sleep(0.02)

    exc = future.exception()
    if exc is not None:
        raise exc
    return future.result()


def _scaling(session: Session, key: str) -> Optional[float]:
    """속도·가속 스케일링 인자를 안전 상한으로 clamp 한다 (설계서 6.10)."""
    raw = session.inputs.get(key)
    if raw is None:
        return None
    return max(0.01, min(1.0, float(raw)))


def _require_gripper_target(raw: Any, op: Optional[OperationConfig]) -> str:
    """그리퍼 목표 이름을 검증한다 — 설정에 정의된 목표만 받는다.

    ``inputs_schema`` 의 ``enum`` 으로도 걸리지만 ``jsonschema`` 는 선택적
    의존성이라 미설치 환경에서는 스키마 검증이 통째로 생략된다. 알 수 없는 목표가
    실행 단계까지 가서 ``EXECUTION_ABORTED`` 로 끝나지 않도록 여기서 ``400`` 으로
    끊는다.
    """
    if not isinstance(raw, str) or not raw:
        raise ValueError("input 'target' (gripper target name) is required")
    if op is None:
        return raw
    targets = gripper_targets(op)
    if targets and raw not in targets:
        available = ', '.join(sorted(targets))
        raise ValueError(f"unknown target '{raw}'; available targets: {available}")
    return raw


def _require_outcome(raw: Any) -> str:
    """`stop_episode` 의 성패 값을 검증한다.

    생략하면 '판정 없음'이며 실패가 아니다 — teleop 처럼 판정 수단이 없는 경로가
    그렇다. `bool` 을 쓰지 않는 이유는 그 세 번째 상태를 표현할 수 없기 때문이다.
    """
    if raw is None:
        return ''
    if not isinstance(raw, str) or raw not in ('', 'success', 'failure'):
        raise ValueError(
            f"'outcome' must be omitted or one of 'success' / 'failure', got {raw!r}")
    return raw


def _require_metadata(raw: Any) -> Optional[dict[str, Any]]:
    """`stop_episode` 의 부가 정보를 검증한다.

    **JSON object 여야 한다** — DB 의 jsonb 컬럼에 그대로 들어가므로 배열이나
    스칼라면 에피소드마다 형태가 달라져 조회(`metadata->>'seed'`)가 성립하지 않는다.
    HTTP 로는 객체를 그대로 받고(이중 인코딩을 요구하지 않는다), 서비스로 보낼 때
    문자열로 만든다.
    """
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise ValueError(f"'metadata' must be a JSON object, got {type(raw).__name__}")
    try:
        json.dumps(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"'metadata' is not JSON-serialisable: {exc}") from None
    return raw


def _require_joint_values(raw: Any) -> dict[str, float]:
    """관절 목표값 입력을 검증한다 — 로봇에 닿기 전에 거른다 (설계서 6.9).

    관절 **이름**이 planning group 에 속하는지는 검사하지 않는다. 트윈은 그룹의
    관절 목록을 갖고 있지 않으므로, 그룹 밖 관절이 섞이면 MoveIt 계획 단계에서
    실패한다 (예: ``panda_arm`` 목표에 ``panda_finger_joint1`` 을 섞는 경우).
    """
    if not isinstance(raw, dict) or not raw:
        raise ValueError("input 'joints' must be a non-empty {joint_name: radians} object")

    values: dict[str, float] = {}
    for name, value in raw.items():
        if not isinstance(name, str) or not name:
            raise ValueError(f'invalid joint name: {name!r}')
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"joints['{name}'] must be a number, got {value!r}")
        if not math.isfinite(float(value)):
            raise ValueError(f"joints['{name}'] is not finite: {value!r}")
        values[name] = float(value)
    return values


def _require_pose(raw: Any) -> dict[str, Any]:
    """pose 입력을 검증한다 — 로봇에 닿기 전에 거른다 (설계서 6.9)."""
    if not isinstance(raw, dict):
        raise ValueError("input 'pose' must be an object with 'position' and 'orientation'")

    pos = raw.get('position') or {}
    ori = raw.get('orientation') or {}
    for axis in ('x', 'y', 'z'):
        if not isinstance(pos.get(axis), (int, float)):
            raise ValueError(f"pose.position.{axis} must be a number")
    for axis in ('x', 'y', 'z', 'w'):
        if not isinstance(ori.get(axis), (int, float)):
            raise ValueError(f"pose.orientation.{axis} must be a number (ROS order x,y,z,w)")

    norm = math.sqrt(sum(float(ori[a]) ** 2 for a in ('x', 'y', 'z', 'w')))
    if not math.isclose(norm, 1.0, abs_tol=1e-3):
        raise ValueError(f'pose.orientation must be a unit quaternion (norm={norm:.6f})')
    return raw


def _to_ros_pose(raw: dict[str, Any]) -> Any:
    """검증된 dict 를 ``geometry_msgs/Pose`` 로 바꾼다."""
    from geometry_msgs.msg import Pose

    pose = Pose()
    pos, ori = raw['position'], raw['orientation']
    pose.position.x, pose.position.y, pose.position.z = (
        float(pos['x']), float(pos['y']), float(pos['z'])
    )
    pose.orientation.x, pose.orientation.y = float(ori['x']), float(ori['y'])
    pose.orientation.z, pose.orientation.w = float(ori['z']), float(ori['w'])
    return pose


def _is_closed_loop(runtime: 'RobotTwinRuntime') -> bool:
    """JGPC 스트리밍은 open loop 다 — 정상 종료가 도달을 보장하지 않는다."""
    return runtime.config.moveit.move_group_mode == 'jtc'


def _motion_outputs(runtime: 'RobotTwinRuntime', *, closed_loop: bool) -> dict[str, Any]:
    """이동 연산의 공통 ``outputs`` 를 만든다 (설계서 6.7).

    측정은 상태 캐시의 최신 스냅샷을 쓴다 — 별도 조회를 하지 않으므로 추가 지연이
    없다. 다만 종료 시각과 정확히 일치하지 않으므로 ``measured_age_ms`` 로
    신선도를 함께 드러낸다.
    """
    outputs: dict[str, Any] = {'closed_loop': closed_loop}

    for name, key in (('ee_pose', 'final_pose'), ('joint_states', 'final_joints')):
        entry = runtime.variables.entry(name)
        if entry is None or entry.snapshot is None:
            continue
        try:
            outputs[key] = entry.value()
        except Exception:
            continue
        status = entry.status()
        if status.age_ms is not None:
            outputs['measured_age_ms'] = max(outputs.get('measured_age_ms', 0), status.age_ms)

    return outputs


__all__ = ['BackendUnavailable', 'PreconditionFailed', 'dispatch', 'requires_move_group',
           'validate_inputs', 'validate_operation_config']
