"""`moveit.arm_command_*` 설정의 검증과 전달 테스트.

Isaac Sim 처럼 **시뮬레이터가 직접 JointState 를 받는** 백엔드를 트윈으로 몰려면
명령 토픽과 형식을 지정해야 한다. 토픽 remap 으로는 못 바꾼다 — 메시지 타입이 다르다.

이 설정이 틀렸을 때의 증상이 고약하다. 팔이 **'성공했다고 보고하면서 아무것도 하지
않는다'** — 계획은 MoveIt 이 정상으로 하고, 스트리밍도 개루프라 발행만 하면 성공이며,
아무도 안 듣는 토픽으로 나갈 뿐이다. 그래서 조용히 무시되는 키를 만들지 않고 설정
단계에서 막는다.
"""

from __future__ import annotations

import pytest

from robot_twin.config import MoveItConfig

ISAAC_JOINTS = [f'panda_joint{i}' for i in range(1, 8)]


def _isaac(**overrides) -> dict:
    base = {
        'move_group_mode': 'jgpc',
        'arm_command_topic': '/isaac/arm_command',
        'arm_command_format': 'joint_state',
        'arm_command_joint_names': list(ISAAC_JOINTS),
    }
    base.update(overrides)
    return base


# ---------- 기본값 ----------

def test_defaults_are_none_so_the_factory_default_applies():
    """생략하면 ros2_control 의 JGPC 기본값(Float64MultiArray)이 쓰인다."""
    config = MoveItConfig(move_group_mode='jgpc')
    assert config.arm_command_topic is None
    assert config.arm_command_format is None
    assert config.client_kwargs() == {}


def test_jtc_without_arm_command_is_fine():
    assert MoveItConfig(move_group_mode='jtc').client_kwargs() == {}


# ---------- 전달 ----------

def test_isaac_settings_reach_the_factory():
    kwargs = MoveItConfig(**_isaac()).client_kwargs()
    assert kwargs == {
        'arm_command_topic': '/isaac/arm_command',
        'arm_command_format': 'joint_state',
        'arm_command_joint_names': ISAAC_JOINTS,
    }


def test_client_kwargs_copies_the_joint_list():
    """설정 객체의 리스트를 그대로 넘기면 클라이언트가 제자리에서 고칠 수 있다."""
    config = MoveItConfig(**_isaac())
    kwargs = config.client_kwargs()
    kwargs['arm_command_joint_names'].append('panda_joint8')
    assert config.arm_command_joint_names == ISAAC_JOINTS


def test_partial_settings_are_allowed_for_float64_format():
    """토픽만 바꾸는 것은 정상이다 — 이름은 컨트롤러에서 조회할 수 있다."""
    kwargs = MoveItConfig(move_group_mode='jgpc',
                          arm_command_topic='/other/commands').client_kwargs()
    assert kwargs == {'arm_command_topic': '/other/commands'}


# ---------- 거부 ----------

@pytest.mark.parametrize('key', ['arm_command_topic', 'arm_command_format',
                                 'arm_command_joint_names'])
def test_jtc_rejects_arm_command_keys(key):
    """JTC 는 이 값을 보지 않는다 — 조용히 무시되면 원인을 찾을 수 없다."""
    value = {'arm_command_topic': '/x', 'arm_command_format': 'joint_state',
             'arm_command_joint_names': ISAAC_JOINTS}[key]
    with pytest.raises(ValueError, match="only applies to move_group_mode 'jgpc'"):
        MoveItConfig(move_group_mode='jtc', **{key: value})


def test_joint_state_format_requires_joint_names():
    """joint_state 에는 조회할 컨트롤러가 없다 — 빠지면 첫 스트리밍에서 멈춘다."""
    with pytest.raises(ValueError, match='arm_command_joint_names is required'):
        MoveItConfig(**_isaac(arm_command_joint_names=None))


def test_empty_joint_names_is_rejected_too():
    """빈 리스트는 '지정했다'가 아니다."""
    with pytest.raises(ValueError, match='arm_command_joint_names is required'):
        MoveItConfig(**_isaac(arm_command_joint_names=[]))


def test_unknown_format_is_rejected():
    with pytest.raises(ValueError):
        MoveItConfig(move_group_mode='jgpc', arm_command_format='rosbag')


def test_auto_mode_is_still_forbidden():
    with pytest.raises(ValueError, match="'auto' is forbidden"):
        MoveItConfig(move_group_mode='auto')


# ---------- 실제 배포 설정 파일 ----------

def _config_dir():
    import pathlib
    # robot_twin/robot_twin/tests/<this> -> src/robot_twin/config
    return pathlib.Path(__file__).resolve().parents[2] / 'config'


def test_shipped_isaac_config_loads():
    """`robot_twin_panda_isaac.yaml` 이 실제로 로드되고 Isaac 채널을 담고 있는가."""
    from robot_twin.config import load_config

    path = _config_dir() / 'robot_twin_panda_isaac.yaml'
    if not path.is_file():
        pytest.skip('배포 설정이 없는 트리 (installed)')

    config = load_config(str(path))
    assert config.moveit.move_group_mode == 'jgpc'
    assert config.moveit.arm_command_topic == '/isaac/arm_command'
    assert config.moveit.arm_command_format == 'joint_state'
    assert config.moveit.arm_command_joint_names == ISAAC_JOINTS
    # Isaac 은 /clock 을 발행한다 — 벽시계로 두면 stamp 가 에피소드와 어긋난다.
    assert config.ros.use_sim_time is True


def test_shipped_isaac_config_exposes_the_same_operations_as_mock():
    """오퍼레이션 목록이 백엔드마다 갈리면 상위가 백엔드를 알아야 한다.

    `reset_scene` 은 한동안 Isaac 에서 빠져 있었다 — scene 이 USD 스테이지에 있어
    ROS 노드가 못 고친다고 봤기 때문이다. Isaac 6.0 이 표준
    `simulation_interfaces` 서비스를 열면서 그 전제가 깨졌고(2026-09-02),
    `isaac_scene_state_node` 가 그것을 대신 불러 계약을 맞춘다.
    상세: docs/scene/isaac_scene_reset.md
    """
    from robot_twin.config import load_config

    isaac_path = _config_dir() / 'robot_twin_panda_isaac.yaml'
    mock_path = _config_dir() / 'robot_twin_panda01.yaml'
    if not isaac_path.is_file() or not mock_path.is_file():
        pytest.skip('배포 설정이 없는 트리 (installed)')

    isaac = {op.name for op in load_config(str(isaac_path)).operations}
    mock = {op.name for op in load_config(str(mock_path)).operations}
    assert 'reset_scene' in isaac
    assert isaac == mock, (
        f'only in mock: {sorted(mock - isaac)}; only in isaac: {sorted(isaac - mock)}')
