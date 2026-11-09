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
    return pathlib.Path(__file__).resolve().parents[2] / 'config' / 'twins'


def test_shipped_isaac_config_loads():
    """`twins/panda01.yaml` 이 로드되고 **명령 채널을 안 담는가**.

    Isaac 은 `topic_based_ros2_control` 로 ros2_control 을 쓰므로 팔이 JTC 다 —
    MoveGroup/ExecuteTrajectory 액션으로 실행되니 `arm_command_*` 를 줄 이유가 없다.
    **비어 있는 것이 계약이다**: 값이 생기면 누군가 옛 bridge 설정을 되살린 것이고,
    그 조합(jtc + 명령 채널)은 `_check_arm_command` 가 **설정 로드에서 거부한다**.
    """
    from robot_twin.config import load_config

    path = _config_dir() / 'panda01.yaml'
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
    상세: docs/backends/isaac/isaac_scene_reset.md
    """
    from robot_twin.config import load_config

    isaac_path = _config_dir() / 'panda01.yaml'
    mock_path = _config_dir() / 'mock01.yaml'
    if not isaac_path.is_file() or not mock_path.is_file():
        pytest.skip('배포 설정이 없는 트리 (installed)')

    isaac = {op.name for op in load_config(str(isaac_path)).operations}
    mock = {op.name for op in load_config(str(mock_path)).operations}
    assert 'reset_scene' in isaac
    assert isaac == mock, (
        f'only in mock: {sorted(mock - isaac)}; only in isaac: {sorted(isaac - mock)}')


# ----- backend — 프로파일로 채우기 (`twin.backend` 가 내려보낸 값) ----------------

def test_backend_fills_the_mode():
    """**백엔드 표가 정본이다.** YAML 에 손으로 적으면 백엔드가 바뀔 때 뒤처진다."""
    assert MoveItConfig(backend='panda_isaac').move_group_mode == 'jtc'


def test_backend_fills_the_whole_command_channel():
    """펑션베이는 넷이 다 필요하다 — 하나라도 빠지면 스트리밍에서 멈춘다."""
    config = MoveItConfig(backend='panda_functionbay')

    assert config.move_group_mode == 'jgpc'
    assert config.arm_command_topic == '/input/panda_joint'
    assert config.arm_command_format == 'joint_state'
    assert config.arm_command_joint_names == [f'panda_joint{i}' for i in range(1, 8)]


def test_explicit_key_beats_the_profile():
    """프로파일에 없는 변형을 붙이는 탈출구다."""
    config = MoveItConfig(backend='panda_functionbay', arm_command_topic='/other/joint')

    assert config.arm_command_topic == '/other/joint'
    assert config.arm_command_format == 'joint_state', '나머지는 프로파일이 채운다'


def test_mock_stacks_are_two_usable_backends():
    """mock 계열은 스택이 둘이라 **프로파일도 둘**이다.

    예전에는 하나로 묶여 `auto` 였고, 트윈은 auto 를 금지하므로 쓸 수 없었다. 나눈
    뒤로는 둘 다 확정 모드라 트윈에서도 쓴다.
    """
    assert MoveItConfig(backend='panda_mock').move_group_mode == 'jtc'
    assert MoveItConfig(backend='panda_mock_jgpc').move_group_mode == 'jgpc'


def test_unknown_backend_is_rejected():
    """오타를 조용히 넘기면 "이 스택에서만 안 되네" 로 나타난다."""
    with pytest.raises(ValueError, match='backend'):
        MoveItConfig(backend='issac')


def test_jtc_backend_does_not_inject_a_command_channel():
    """JTC 로 풀린 프로파일이 명령 채널을 채우면, 사용자가 **적지도 않은 키** 때문에
    `_check_arm_command` 에 걸려 트윈이 안 뜬다.
    """
    config = MoveItConfig(backend='panda_isaac')

    assert config.client_kwargs() == {}


def test_backend_and_explicit_mode_disagreeing_uses_the_explicit_one():
    """명시가 이긴다 — 프로파일은 기본값일 뿐이다."""
    config = MoveItConfig(backend='panda_isaac', move_group_mode='jgpc')

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

    kwargs = MoveItConfig(backend='panda_functionbay').client_kwargs()

    json.dumps(kwargs)          # 던지면 실패다
    assert 'gravity_compensator' not in kwargs


def test_gravity_compensator_comes_from_the_backend():
    """**팩토리가 대신 해 주지 않는다** — 트윈은 백엔드를 값으로 펼쳐 넘기기 때문이다.

    이것이 `None` 이 되면 같은 로봇인데 스크립트로 움직일 때만 보상이 걸린다.
    """
    from robot_twin.config import MoveItConfig

    from robot_control.backend_profiles import BACKEND_PROFILES

    compensator = MoveItConfig(backend='panda_functionbay').gravity_compensator()

    # **선언과 결과가 일치하는지만 본다 — 값도, 있음/없음도 박지 않는다.**
    # `Kp` 는 씬·빌드마다 다르고(2026-09-29 t1 2000 → 20000), **선언 자체가 씬에 딸린다** —
    # 2026-09-30 에 r2 가 벤더 보상을 받게 되어 블록을 지웠고 그때 이 시험이 깨졌다.
    # 지킬 것은 "트윈이 프로파일을 그대로 물려받는다"(팩토리가 대신 해 주지 않는다)이다.
    spec = (BACKEND_PROFILES['panda_functionbay'].get('motion') or {}).get('gravity_compensation')

    if spec:
        assert compensator is not None
        assert compensator.kp[0] == spec.get('kp', 2000.0)
    else:
        assert compensator is None, '프로파일에 선언이 없는데 트윈이 보상기를 만들었다'


def test_backends_without_the_key_get_none():
    """mock 은 컨트롤러가 중력을 스스로 잡으므로 설정이 없다."""
    from robot_twin.config import MoveItConfig

    assert MoveItConfig(backend='panda_mock').gravity_compensator() is None
    assert MoveItConfig(move_group_mode='jtc').gravity_compensator() is None


# --- 프레임 (2026-09-11) --------------------------------------------------
#
# EE 프레임 이름은 **프로파일 하나가 갖는다** — launch 의 `ee_pose_node` 와 트윈의
# `move_linear` 변환이 같은 값을 읽어야 하기 때문이다. 두 곳에 적으면 어긋남이
# 에러 없이 "149 mm 빗나간 이동" 으로 나타난다.

def test_backend_fills_frames_from_the_profile() -> None:
    cfg = MoveItConfig(backend='panda_functionbay')
    assert cfg.ee_frame == 'grasp_center'
    assert cfg.tip_frame == 'panda_link8'


def test_explicit_frames_beat_the_profile() -> None:
    """`backend` 의 다른 값들과 같은 규칙이다 — 명시한 키가 이긴다."""
    cfg = MoveItConfig(backend='panda_functionbay', ee_frame='panda_hand')
    assert cfg.ee_frame == 'panda_hand'


def test_mock_declares_no_ee_frame_on_purpose() -> None:
    """mock 계열은 **일부러 비어 있다.**

    `panda_hand` 는 `panda_link8` 과 평행이동이 0 이고 z 축 45° 회전만 다르다.
    위치가 같아 149 mm 류의 어긋남이 없고, 그 45° 는 MCP 클라이언트의
    `to_arm_command()` 가 이미 곱한다 — 트윈이 또 돌리면 **90° 이중 회전**이다.
    채우는 것은 그 보정을 트윈으로 옮기는 계획서 5단계의 일이다.
    """
    assert MoveItConfig(backend='panda_mock').ee_frame is None
    assert MoveItConfig(backend='panda_mock').tip_frame == 'panda_link8'


# --- 명령 스트리밍 주기 (2026-09-11) ------------------------------------------
#
# 안 채우면 데카르트 궤적이 **10 Hz 계단**으로 나간다 (MoveIt 이 TOTG 로 재샘플하고
# 이 경로에는 보간할 컨트롤러가 없다). 접촉을 동반하는 작업에서 그 계단이 실제로
# 실패를 만든다 — 여유 0.5 mm 인 트레이 삽입이 10 Hz 로는 3회 모두 입구에서 튕겼다.

def test_publish_rate_comes_from_the_profile() -> None:
    cfg = MoveItConfig(backend='panda_functionbay')
    assert cfg.arm_command_publish_rate == 50.0
    assert cfg.client_kwargs()['publish_rate'] == 50.0


def test_jtc_backend_does_not_get_a_publish_rate() -> None:
    """**JTC 생성자는 이 인자를 모른다** — 넘기면 `TypeError` 로 클라이언트가 안 선다.

    JTC 는 컨트롤러가 보간하므로 개념 자체가 없다.
    """
    assert 'publish_rate' not in MoveItConfig(backend='panda_mock').client_kwargs()


def test_explicit_publish_rate_beats_the_profile() -> None:
    cfg = MoveItConfig(backend='panda_functionbay', arm_command_publish_rate=120.0)
    assert cfg.client_kwargs()['publish_rate'] == 120.0


# ---------- 시계 (`ros.use_sim_time`) 도 프로파일에서 온다 (2026-10-21) ----------
#
# 시계 축은 로봇 스택의 사정이라 launch · `rdfp_collect` 와 같은 프로파일 값을 따라야 한다.
# 트윈 YAML 에 따로 두면 한쪽만 고쳤을 때 변수 stamp 가 에피소드 경계와 다른 시계로 찍히고,
# 에러 없이 데이터셋에서 짝이 안 맞는다.

def _twin(backend=None, **sections):
    from robot_twin.config import TwinConfig
    twin = {'id': 'panda_test'}
    if backend is not None:
        twin['backend'] = backend
    return TwinConfig.model_validate({'twin': twin, **sections})


@pytest.mark.parametrize('backend, expected', [
    ('panda_mock', False), ('panda_mock_jgpc', False), ('panda_isaac', True),
    ('panda_functionbay', False)])
def test_sim_time_comes_from_the_backend_profile(backend, expected):
    from robot_control.backend_profiles import BACKEND_PROFILES
    assert expected == BACKEND_PROFILES[backend]['use_sim_time']      # 표가 프로파일과 같은지
    assert _twin(backend).ros.use_sim_time is expected


def test_explicit_sim_time_beats_the_profile():
    cfg = _twin('panda_isaac', ros={'use_sim_time': False, 'node_name': 'n'})
    assert cfg.ros.use_sim_time is False
    assert cfg.ros.node_name == 'n'


def test_other_ros_keys_survive_the_fill():
    """`ros` 블록의 다른 키를 지우지 않고 `use_sim_time` 만 더한다."""
    cfg = _twin('panda_isaac',
                ros={'node_name': 'robot_twin_x', 'domain_id': 31})
    got = (cfg.ros.use_sim_time, cfg.ros.node_name, cfg.ros.domain_id)
    assert got == (True, 'robot_twin_x', 31)


def test_sim_time_defaults_to_false_without_a_backend():
    cfg = _twin(moveit={'move_group_mode': 'jtc'})
    assert cfg.ros.use_sim_time is False


@pytest.mark.parametrize('name', ['mock01.yaml', 'panda01.yaml', 'rdfp01.yaml'])
def test_shipped_configs_leave_sim_time_to_the_profile(name):
    """출하 설정은 시계를 적지 않는다 — 적으면 프로파일과 두 곳이 되어 다시 어긋날 수 있다."""
    import yaml
    from robot_control.backend_profiles import BACKEND_PROFILES
    from robot_twin.config import load_config

    path = _config_dir() / name
    if not path.is_file():
        pytest.skip('배포 설정이 없는 트리 (installed)')
    raw = yaml.safe_load(path.read_text(encoding='utf-8'))
    assert 'use_sim_time' not in (raw.get('ros') or {}), name
    config = load_config(str(path))
    assert config.ros.use_sim_time is bool(
        BACKEND_PROFILES[config.twin.backend]['use_sim_time'])


def test_twin_config_maps_a_legacy_backend_name():
    """옛 이름(`isaac`)으로 적은 트윈 설정도 뜨고, 안에서는 지금 이름으로 바뀐다."""
    with pytest.warns(FutureWarning):
        cfg = MoveItConfig(backend='isaac')
    assert cfg.backend == 'panda_isaac'
    assert cfg.move_group_mode == 'jtc'


# ----- `twin.backend` 의 자리 ------------------------------------------------
#
# 백엔드는 트윈 전체의 속성이라 `twin` 아래에 둔다 — 팔 명령 채널만이 아니라 시계 · 고정물 ·
# 중력 보상도 이 이름에서 나온다. 출하 트윈은 환경마다 id 가 다르지만(`mock01` · `panda01` ·
# `rdfp01`) id 는 사용자가 바꿀 수 있으므로, 클라이언트는 트윈 메타의 `backend` 로 환경을 안다.

def test_twin_backend_fills_the_moveit_section():
    cfg = _twin('panda_functionbay', moveit={'planning_group': 'panda_arm'})
    assert cfg.twin.backend == cfg.moveit.backend == 'panda_functionbay'
    assert cfg.moveit.move_group_mode == 'jgpc'
    assert cfg.moveit.arm_command_format == 'joint_state'


def test_moveit_section_may_be_omitted_with_a_backend():
    """백엔드 하나로 `moveit` 이 다 채워지므로 블록이 없어도 뜬다."""
    assert _twin('panda_isaac').moveit.move_group_mode == 'jtc'


@pytest.mark.parametrize('backend', [None, 'panda_isaac'])
def test_old_moveit_backend_key_is_rejected(backend):
    """옛 자리를 받아 주면 두 자리에 다른 값을 적은 복사본이 어느 쪽이 이기는지 모른 채 돈다."""
    with pytest.raises(ValueError, match='moveit.backend has moved to twin.backend'):
        _twin(backend, moveit={'backend': 'panda_isaac'})


def test_twin_backend_maps_a_legacy_name():
    """트윈 메타에 보이는 이름은 늘 지금 이름이다."""
    with pytest.warns(FutureWarning):
        cfg = _twin('isaac')
    assert cfg.twin.backend == cfg.moveit.backend == 'panda_isaac'


def test_unknown_twin_backend_is_rejected():
    with pytest.raises(ValueError, match='backend'):
        _twin('issac')


@pytest.mark.parametrize('twin_id, backend', [
    ('mock01', 'panda_mock'),
    ('panda01', 'panda_isaac'),
    ('rdfp01', 'panda_functionbay')
])
def test_shipped_configs_share_one_port(twin_id, backend):
    """출하 설정 셋은 **포트(8801)가 같다** — 동시에 띄우지 않는다.

    실수로 둘을 띄우면 두 번째가 포트 충돌로 못 뜬다. id 는 환경마다 다르고(2026-10-25) 파일
    이름이 곧 id 이며, 노드 이름은 id 를 따른다.
    """
    from robot_twin.config import load_config

    path = _config_dir() / f'{twin_id}.yaml'
    if not path.is_file():
        pytest.skip('배포 설정이 없는 트리 (installed)')
    cfg = load_config(str(path))
    got = (cfg.twin.id, cfg.http.port, cfg.ros.node_name)
    assert got == (twin_id, 8801, f'robot_twin_{twin_id}')
    assert cfg.twin.backend == backend


def test_every_shipped_config_is_named_after_its_twin_id():
    """`--twin <id>` 는 `<id>.yaml` 을 바로 연다 — 파일 이름과 `twin.id` 가 다르면 못 쓴다."""
    from robot_twin.shipped_configs import find_shipped, list_shipped

    config_dir = _config_dir()
    if not config_dir.is_dir():
        pytest.skip('배포 설정이 없는 트리 (installed)')
    configs = list_shipped(config_dir)          # 이름이 다르면 여기서 ValueError
    assert configs
    for config in configs:
        assert find_shipped(config_dir, config.twin_id).path == config.path


def test_ids_the_scripts_default_to_are_shipped():
    """수집 스크립트 · MCP 설정이 기본으로 붙는 트윈이 출하 설정에 있어야 한다.

    위 시험들은 파일이 없으면 **skip** 하므로 id 를 바꿔도 조용하다. 어긋나면 수집 스크립트는
    트윈을 못 띄우고, MCP 는 도구 목록을 가져올 때 404 로만 드러난다.
    """
    import json
    import pathlib
    import re

    from robot_twin.shipped_configs import list_shipped

    # robot_twin/robot_twin/tests/<this> -> 워크스페이스 루트
    ws = pathlib.Path(__file__).resolve().parents[4]
    if not (ws / 'scripts').is_dir() or not _config_dir().is_dir():
        pytest.skip('워크스페이스가 없는 트리 (installed)')
    shipped = {c.twin_id for c in list_shipped(_config_dir())}

    used = {}
    for name in ('run_collect_isaac.sh', 'run_collect_functionbay.sh'):
        text = (ws / 'scripts' / name).read_text(encoding='utf-8')
        used[name] = re.search(r'RDFP_TWIN_ID:-([\w-]+)', text).group(1)
    for name in ('.mcp.json', '.mcp_isaac.json', '.mcp_functionbay.json'):
        server = json.loads((ws / name).read_text(encoding='utf-8'))['mcpServers']['robot-twin']
        args = server['args']
        used[name] = args[args.index('--twin-id') + 1]
    assert {name: twin_id for name, twin_id in used.items() if twin_id not in shipped} == {}
    assert used['run_collect_isaac.sh'] == used['.mcp_isaac.json'] == 'panda01'
    assert used['run_collect_functionbay.sh'] == used['.mcp_functionbay.json'] == 'rdfp01'
