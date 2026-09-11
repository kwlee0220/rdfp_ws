#!/usr/bin/env python3

"""프로파일에 옮겨 적은 값이 **실제와 맞는가.**

프로파일은 launch·노드에 흩어져 있던 값을 한곳에 모은 것이다. 옮겨 적다 틀리면
에러가 아니라 **"명령이 안 먹는다" / "이미지가 안 담긴다"** 로 나타난다. 그래서 옮긴
값을 그 출처와 직접 대조한다.

여기서 보는 것은 두 가지다.

1. **파일을 가리키는 값은 그 파일이 실재하는가** — 경로 오타는 런타임에야 드러난다.
2. **아직 안 옮긴 launch 의 상수와 같은가** — 옮기기 전까지 값이 두 곳에 있으므로,
   갈라지면 프로파일을 읽는 쪽과 안 읽는 쪽이 다르게 동작한다. 옮긴 뒤에는 그 대조가
   필요 없어지고, 대신 **launch 가 정말 프로파일에서 파생하는지**를 본다.
"""

from __future__ import annotations

import os

import pytest

from robot_control import backend_profiles as bp

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_LAUNCH = os.path.join(_REPO, 'launch')


def _load_launch_module(name: str):
    """launch 파일을 모듈로 읽는다 (실행은 `generate_launch_description` 에서만)."""
    import importlib.util
    import sys

    path = os.path.join(_LAUNCH, f'{name}.launch.py')
    if not os.path.isfile(path):
        pytest.skip('설치 트리에는 launch 소스가 없다')
    spec = importlib.util.spec_from_file_location(f'_{name}_under_test', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
    finally:
        sys.modules.pop(spec.name, None)
    return module


# ----- 가리키는 파일이 실재하는가 ---------------------------------------------

def _referenced_paths() -> list:
    found = []
    for name, profile in bp.BACKEND_PROFILES.items():
        for block, key in (('ros2_control', 'controllers_file'), ('scene', 'file')):
            value = (profile.get(block) or {}).get(key)
            if value:
                found.append((name, f'{block}.{key}', value))
    return found


def test_something_points_at_a_file():
    """대조할 것이 없으면 아래 검사가 조용히 아무 일도 안 한다."""
    assert _referenced_paths()


@pytest.mark.parametrize('backend,key,relpath',
                         _referenced_paths(),
                         ids=lambda v: v if isinstance(v, str) else str(v))
def test_referenced_files_exist(backend, key, relpath):
    """경로 오타는 **로봇을 띄운 뒤에야** 드러난다."""
    assert os.path.isfile(os.path.join(_REPO, relpath)), \
        f'{backend}: {key} -> {relpath} 가 없다'


# ----- launch 기본값과 같은가 --------------------------------------------------

# Isaac 은 launch 가 프로파일에서 **파생**하므로 상수 대조가 필요 없다 —
# `test_isaac_launch_defaults_come_from_the_profile` 이 그 파생을 직접 확인한다.
# 아래 펑션베이는 아직 자기 상수를 들고 있어 대조가 남아 있다.

def test_functionbay_launch_constants_come_from_the_profile():
    """`panda_functionbay.launch.py` 가 프로파일에서 파생하는가.

    이 백엔드는 값이 **remap 대상**으로도 쓰여서, 프로파일 조회를 틀리면 상수가
    `None` 이 되고 launch 가 기동 중에 죽는다 — 실제로 그렇게 깨졌다
    (`arm_command` 블록은 최상위로 평탄화되어 `value('arm_command', ...)` 로는 안 잡힌다).
    """
    module = _load_launch_module('panda_functionbay')
    p = bp.BACKEND_PROFILES['functionbay']

    assert module.FB_JOINT_REPORT_TOPIC == p['simulator']['joint_report_topic']
    assert module.FB_JOINT_COMMAND_TOPIC == p['simulator']['joint_command_topic']
    assert module.FB_GRIPPER_COMMAND_TOPIC == p['simulator']['gripper_command_topic']
    assert module.FB_GRIPPER_REPORT_TOPIC == p['simulator']['gripper_report_topic']
    assert module.ARM_COMMAND_TOPIC == p['arm_command_topic']
    assert module.ARM_JOINT_NAMES == p['arm_command_joint_names']
    # 브리지의 입력은 **servo 의 출력**이다 — 아래 테스트가 그 키가 하나임을 지킨다.
    assert module.SERVO_COMMAND_TOPIC == p['servo']['command_out_topic']
    assert module.CAMERA_IMAGE_TOPIC == p['camera']['image_topic']
    assert module.CAMERA_COMPRESSED_TOPIC == p['camera']['compressed_topic']


def test_servo_output_channel_is_declared_once():
    """servo 출력 채널을 두 키로 적으면 **조용히 갈린다.**

    ros2_control 이 없는 백엔드는 `servo_command_bridge` 가 servo 출력을 받아 시뮬레이터로
    옮긴다. 다리의 입력은 정의상 servo 의 출력인데, 한때 `simulator.servo_command_topic`
    에 같은 값을 한 번 더 적어 두었다 — launch 가 다리 remap 은 그쪽에서, servo 파라미터는
    `servo.command_out_topic` 에서 읽었으므로 둘이 갈리면 **servo 는 발행하고 다리는 못
    받는다.** 증상은 "모션 키가 아무것도 안 한다" 뿐이라 원인을 안 가리킨다.
    """
    for name, profile in bp.BACKEND_PROFILES.items():
        simulator = profile.get('simulator') or {}
        assert 'servo_command_topic' not in simulator, \
            f'{name}: servo 출력 채널은 servo.command_out_topic 하나다'


def test_functionbay_launch_defaults_come_from_the_profile():
    """`panda_functionbay.launch.py` 의 인자 기본값이 프로파일에서 오는가.

    카메라 raw 토픽이 한때 `image_pipeline.yaml`(OpenCV 카메라용)에서 왔다 — 프로파일은
    `/camera_image` 인데 실효값은 `/camera/image_raw` 였다. 각 launch 안에서는 republish
    출력과 뷰어 구독이 같은 값을 써서 **동작은 했고, 그래서 안 드러났다.**
    """
    module = _load_launch_module('panda_functionbay')
    defaults = {}
    for action in module.generate_launch_description().entities:
        if type(action).__name__ != 'DeclareLaunchArgument':
            continue
        raw = action.default_value
        defaults[action.name] = (
            ''.join(getattr(part, 'text', str(part)) for part in raw) if raw else None)

    p = bp.BACKEND_PROFILES['functionbay']
    assert defaults['camera_image_topic'] == p['camera']['image_topic']
    assert defaults['camera_compressed_topic'] == p['camera']['compressed_topic']
    assert defaults['servo_linear_scale'] == str(p['servo']['linear_scale'])
    assert defaults['fb_joint_report_topic'] == p['simulator']['joint_report_topic']
    assert defaults['fb_gripper_command_topic'] == p['simulator']['gripper_command_topic']


def test_isaac_arm_command_is_not_the_hardware_topic():
    """**이 둘을 섞으면 컨트롤러를 우회해 같은 채널을 두고 싸운다.**

    옛 bridge 시절 `arm_command.topic: /isaac/arm_command` 로 두었던 잔재다. 지금 그
    토픽은 `TopicBasedSystem` 이 쓰는 쪽이고, MoveGroup 은 JTC 액션으로 간다.
    """
    p = bp.BACKEND_PROFILES['isaac']

    assert 'arm_command_topic' not in p, 'JTC 백엔드는 명령 채널을 갖지 않는다'
    assert p['ros2_control']['joint_commands_topic'] == '/isaac/arm_command'


# ----- 블록 구조 --------------------------------------------------------------

def test_null_means_the_backend_lacks_it():
    """`null` 은 "빠뜨렸다" 가 아니라 "이 백엔드에는 없다" 는 뜻이다."""
    assert bp.BACKEND_PROFILES['functionbay']['ros2_control'] is None, \
        '펑션베이에는 ros2_control 이 없다'
    assert bp.BACKEND_PROFILES['mock']['scene']['file'] is None, \
        'mock 은 scene 정의 파일이 없다 — /scene/reset 이 놓은 것만 있다'


def test_functionbay_declares_its_compressed_camera():
    """펑션베이는 **압축으로** 발행한다 (2026-09-08 새 빌드).

    한동안 프로파일이 `camera: null`("카메라가 없다")이었다 — 옮겨 적을 때의 오류다.
    두 이름이 다 필요하다: 레코더는 압축을 직접 받고, 뷰어는 `camera_republish` 가
    되살린 raw 를 구독한다.
    """
    camera = bp.BACKEND_PROFILES['functionbay']['camera']

    assert camera['compressed_topic'] == '/camera_image/compressed'
    assert camera['image_topic'] == '/camera_image'


def test_unknown_block_key_is_rejected(tmp_path):
    """새 블록의 키도 오타를 거른다."""
    (tmp_path / 'x.yaml').write_text('servo: { linaer_scale: 0.4 }\n', encoding='utf-8')

    with pytest.raises(ValueError, match='unknown servo key'):
        bp.load_profiles(str(tmp_path))


def test_every_backend_declares_use_sim_time():
    """빠뜨리면 스탬프가 벽시계로 찍혀 에피소드 경계와 어긋난다."""
    for name, profile in bp.BACKEND_PROFILES.items():
        assert isinstance(profile.get('use_sim_time'), bool), f'{name}: use_sim_time 누락'


# ----- launch 가 프로파일을 실제로 쓰는가 ---------------------------------------

def test_isaac_launch_defaults_come_from_the_profile():
    """`panda_isaac.launch.py` 의 인자 기본값이 프로파일 값과 같은가.

    상수를 손으로 되적으면 두 곳이 갈리고, 그 어긋남은 에러가 아니라 "명령이 안
    먹는다" 로 나타난다. launch 를 **실제로 만들어** 기본값을 꺼내 비교한다.
    """
    module = _load_launch_module('panda_isaac')
    defaults = {}
    for action in module.generate_launch_description().entities:
        if type(action).__name__ != 'DeclareLaunchArgument':
            continue
        raw = action.default_value
        defaults[action.name] = (
            ''.join(getattr(part, 'text', str(part)) for part in raw) if raw else None)

    p = bp.BACKEND_PROFILES['isaac']
    assert defaults['use_sim_time'] == str(p['use_sim_time']).lower()
    assert defaults['servo_linear_scale'] == str(p['servo']['linear_scale'])
    assert defaults['servo_joint_source'] == p['servo']['joint_source']
    assert defaults['camera_image_topic'] == p['camera']['image_topic']
    assert defaults['controllers_file'].endswith(p['ros2_control']['controllers_file'])
    assert defaults['joint_state_topic'] == p['ros2_control']['joint_states_topic']
    assert defaults['arm_command_topic'] == p['ros2_control']['joint_commands_topic']
    assert defaults['gripper_command_topic'] == p['ros2_control']['gripper_commands_topic']
