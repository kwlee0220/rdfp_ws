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
    """`robot_twin_panda_isaac.yaml` 이 로드되고 **명령 채널을 안 담는가**.

    Isaac 은 `topic_based_ros2_control` 로 ros2_control 을 쓰므로 팔이 JTC 다 —
    MoveGroup/ExecuteTrajectory 액션으로 실행되니 `arm_command_*` 를 줄 이유가 없다.
    **비어 있는 것이 계약이다**: 값이 생기면 누군가 옛 bridge 설정을 되살린 것이고,
    그 조합(jtc + 명령 채널)은 `_check_arm_command` 가 **설정 로드에서 거부한다**.
    """
    from robot_twin.config import load_config

    path = _config_dir() / 'robot_twin_panda_isaac.yaml'
    if not path.is_file():
        pytest.skip('배포 설정이 없는 트리 (installed)')

    config = load_config(str(path))
    assert config.moveit.move_group_mode == 'jtc'
    assert config.moveit.arm_command_topic is None
    assert config.moveit.arm_command_format is None
    assert not config.moveit.arm_command_joint_names
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


# ----- moveit.backend — 프로파일로 채우기 -------------------------------------

def test_backend_fills_the_mode():
    """**백엔드 표가 정본이다.** YAML 에 손으로 적으면 백엔드가 바뀔 때 뒤처진다."""
    assert MoveItConfig(backend='isaac').move_group_mode == 'jtc'


def test_backend_fills_the_whole_command_channel():
    """펑션베이는 넷이 다 필요하다 — 하나라도 빠지면 스트리밍에서 멈춘다."""
    config = MoveItConfig(backend='functionbay')

    assert config.move_group_mode == 'jgpc'
    assert config.arm_command_topic == '/input/panda_joint'
    assert config.arm_command_format == 'joint_state'
    assert config.arm_command_joint_names == [f'panda_joint{i}' for i in range(1, 8)]


def test_explicit_key_beats_the_profile():
    """프로파일에 없는 변형을 붙이는 탈출구다."""
    config = MoveItConfig(backend='functionbay', arm_command_topic='/other/joint')

    assert config.arm_command_topic == '/other/joint'
    assert config.arm_command_format == 'joint_state', '나머지는 프로파일이 채운다'


def test_mock_stacks_are_two_usable_backends():
    """mock 계열은 스택이 둘이라 **프로파일도 둘**이다.

    예전에는 하나로 묶여 `auto` 였고, 트윈은 auto 를 금지하므로 쓸 수 없었다. 나눈
    뒤로는 둘 다 확정 모드라 트윈에서도 쓴다.
    """
    assert MoveItConfig(backend='mock').move_group_mode == 'jtc'
    assert MoveItConfig(backend='mock_jgpc').move_group_mode == 'jgpc'


def test_unknown_backend_is_rejected():
    """오타를 조용히 넘기면 "이 스택에서만 안 되네" 로 나타난다."""
    with pytest.raises(ValueError, match='backend'):
        MoveItConfig(backend='issac')


def test_jtc_backend_does_not_inject_a_command_channel():
    """JTC 로 풀린 프로파일이 명령 채널을 채우면, 사용자가 **적지도 않은 키** 때문에
    `_check_arm_command` 에 걸려 트윈이 안 뜬다.
    """
    config = MoveItConfig(backend='isaac')

    assert config.client_kwargs() == {}


def test_backend_and_explicit_mode_disagreeing_uses_the_explicit_one():
    """명시가 이긴다 — 프로파일은 기본값일 뿐이다."""
    config = MoveItConfig(backend='isaac', move_group_mode='jgpc')

    assert config.move_group_mode == 'jgpc'


def test_mode_is_still_required_without_a_backend():
    """`backend` 를 안 주면 예전 그대로 필수다."""
    with pytest.raises(ValueError, match='move_group_mode'):
        MoveItConfig()


# ----- 중력 보상 — 넣는 자리가 갈린다 -----------------------------------------

def test_client_kwargs_stays_json_serializable():
    """**`client_kwargs()` 는 `/health` 로도 나간다** — 직렬화 안 되는 값을 담으면 안 된다.

    2026-09-11 에 중력 보상기를 여기 담았다가 `/health` 가
    `PydanticSerializationError` 로 **500** 을 냈다. 연산은 멀쩡히 돌아서 증상이
    "health 만 죽는다" 였다.
    """
    import json

    from robot_twin.config import MoveItConfig

    kwargs = MoveItConfig(backend='functionbay').client_kwargs()

    json.dumps(kwargs)          # 던지면 실패다
    assert 'gravity_compensator' not in kwargs


def test_gravity_compensator_comes_from_the_backend():
    """**팩토리가 대신 해 주지 않는다** — 트윈은 백엔드를 값으로 펼쳐 넘기기 때문이다.

    이것이 `None` 이 되면 같은 로봇인데 스크립트로 움직일 때만 보상이 걸린다.
    """
    from robot_twin.config import MoveItConfig

    compensator = MoveItConfig(backend='functionbay').gravity_compensator()

    assert compensator is not None
    assert compensator.kp[0] == 2000.0


def test_backends_without_the_key_get_none():
    """mock 은 컨트롤러가 중력을 스스로 잡으므로 설정이 없다."""
    from robot_twin.config import MoveItConfig

    assert MoveItConfig(backend='mock').gravity_compensator() is None
    assert MoveItConfig(move_group_mode='jtc').gravity_compensator() is None


# --- 프레임 (2026-09-11) --------------------------------------------------
#
# EE 프레임 이름은 **프로파일 하나가 갖는다** — launch 의 `ee_pose_node` 와 트윈의
# `move_linear` 변환이 같은 값을 읽어야 하기 때문이다. 두 곳에 적으면 어긋남이
# 에러 없이 "149 mm 빗나간 이동" 으로 나타난다.

def test_backend_fills_frames_from_the_profile() -> None:
    cfg = MoveItConfig(backend='functionbay')
    assert cfg.ee_frame == 'grasp_center'
    assert cfg.tip_frame == 'panda_link8'


def test_explicit_frames_beat_the_profile() -> None:
    """`backend` 의 다른 값들과 같은 규칙이다 — 명시한 키가 이긴다."""
    cfg = MoveItConfig(backend='functionbay', ee_frame='panda_hand')
    assert cfg.ee_frame == 'panda_hand'


def test_mock_declares_no_ee_frame_on_purpose() -> None:
    """mock 계열은 **일부러 비어 있다.**

    `panda_hand` 는 `panda_link8` 과 평행이동이 0 이고 z 축 45° 회전만 다르다.
    위치가 같아 149 mm 류의 어긋남이 없고, 그 45° 는 MCP 클라이언트의
    `to_arm_command()` 가 이미 곱한다 — 트윈이 또 돌리면 **90° 이중 회전**이다.
    채우는 것은 그 보정을 트윈으로 옮기는 계획서 5단계의 일이다.
    """
    assert MoveItConfig(backend='mock').ee_frame is None
    assert MoveItConfig(backend='mock').tip_frame == 'panda_link8'


# --- 명령 스트리밍 주기 (2026-09-11) ------------------------------------------
#
# 안 채우면 데카르트 궤적이 **10 Hz 계단**으로 나간다 (MoveIt 이 TOTG 로 재샘플하고
# 이 경로에는 보간할 컨트롤러가 없다). 접촉을 동반하는 작업에서 그 계단이 실제로
# 실패를 만든다 — 여유 0.5 mm 인 트레이 삽입이 10 Hz 로는 3회 모두 입구에서 튕겼다.

def test_publish_rate_comes_from_the_profile() -> None:
    cfg = MoveItConfig(backend='functionbay')
    assert cfg.arm_command_publish_rate == 50.0
    assert cfg.client_kwargs()['publish_rate'] == 50.0


def test_jtc_backend_does_not_get_a_publish_rate() -> None:
    """**JTC 생성자는 이 인자를 모른다** — 넘기면 `TypeError` 로 클라이언트가 안 선다.

    JTC 는 컨트롤러가 보간하므로 개념 자체가 없다.
    """
    assert 'publish_rate' not in MoveItConfig(backend='mock').client_kwargs()


def test_explicit_publish_rate_beats_the_profile() -> None:
    cfg = MoveItConfig(backend='functionbay', arm_command_publish_rate=120.0)
    assert cfg.client_kwargs()['publish_rate'] == 120.0
