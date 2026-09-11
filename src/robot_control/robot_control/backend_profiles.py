#!/usr/bin/env python3

"""백엔드 프로파일 — 백엔드마다 갈리는 설정을 한 파일에 모은다.

**백엔드 하나 = 파일 하나.** `config/backends/<이름>.yaml` 이며, 파일 이름이 곧
`backend` 이름이다. 새 백엔드는 가장 가까운 기존 파일을 복사해 값만 바꾼다.

**이 표가 "어느 스택이 무엇인가"의 정본이다.** 같은 사실을 launch·노드·트윈 설정에
각각 적으면 백엔드가 바뀔 때 한쪽만 뒤처지고, 그 어긋남은 에러가 아니라 "명령이 안
먹는다"로 나타난다 — Isaac 이 bridge → ros2_control 로 바뀔 때 실제로 그랬다.

**이 모듈은 `rclpy` 를 import 하지 않는다.** 설정만 읽는 경로(트윈의 `config.py`,
순수 파이썬 도구)가 ROS 없이 동작해야 하기 때문이다. `move_group_factory` 는 여기서
가져다 쓴다 — 그쪽은 `rclpy` 를 끌어온다.

파일을 고친 뒤에는 **`colcon build` 를 다시 돌린다.** 설치본(`share/`)이 먼저 읽히므로
소스만 고치면 낡은 값이 그대로 쓰인다.
"""

from __future__ import annotations

from typing import Any, Optional

import os

import yaml

# `arm_command` 블록의 키 → 소비자가 쓰는 이름. 블록으로 묶는 이유는 앞으로 관절 상태
# 토픽·servo scale·카메라 토픽 같은 다른 갈래가 형제 블록으로 들어오기 때문이다.
_ARM_COMMAND_KEYS = {
    'mode': 'arm_command_mode',
    'topic': 'arm_command_topic',
    'format': 'arm_command_format',
    'joint_names': 'arm_command_joint_names',
    'publish_rate': 'arm_command_publish_rate',
}

# 프로파일이 안 정하면 쓰는 값. `arm_command_topic` 의 기본은 ros2_control JGPC 의
# 표준 토픽이며 `mode='auto'` 판별의 탐침이기도 하다.
#
# `arm_command_mode` 의 폴백이 `auto` 인 것은 **프로파일이 mode 를 빠뜨렸을 때**의
# 안전망일 뿐이다. 지금 모든 프로파일은 확정 모드를 갖는다 — `auto` 라는 백엔드는 없다.
DEFAULT_ARM_COMMAND_TOPIC = '/panda_arm_controller/commands'

# 프로파일이 담을 수 있는 것. **모르는 키는 거부한다** — 조용히 무시하면 "설정했는데
# 안 먹는다" 가 되고, 그 증상은 원인을 안 가리킨다.
#
# 블록 값이 `null` 이면 **그 백엔드에는 그것이 없다**는 뜻이다 (펑션베이의
# `ros2_control: null`, mock 의 `scene.file: null`).
_BLOCK_KEYS = {
    'arm_command': {'mode', 'topic', 'format', 'joint_names', 'publish_rate'},
    # controller_manager 배선. `joint_commands_topic` 은 `arm_command.topic` 과 **다른
    # 것이다** — 이쪽은 하드웨어 인터페이스가 시뮬레이터와 주고받는 채널이다.
    'ros2_control': {'hardware_type', 'controllers_file', 'joint_commands_topic',
                     'gripper_commands_topic', 'joint_states_topic'},
    # ros2_control 이 없는 백엔드가 시뮬레이터와 직접 주고받는 채널.
    #
    # **servo 출력 채널은 여기 없다** — `servo.command_out_topic` 하나다. servo 가
    # 내는 곳이 곧 `servo_command_bridge` 가 받는 곳이라 키가 둘이면 갈릴 수 있고,
    # 갈리면 servo 는 발행하고 다리는 못 받아 모션 전체가 조용히 무동작이 된다.
    'simulator': {'joint_report_topic', 'joint_command_topic', 'gripper_command_topic',
                  'gripper_report_topic'},
    'servo': {'linear_scale', 'joint_source', 'command_out_type', 'command_out_topic'},
    # 팔 이동의 **전역 기본값**. 백엔드마다 안전한 속도가 다르다 — 시뮬레이터가
    # 내부 보간을 안 하거나 처짐이 크면 느리게 도는 편이 낫다. 생략하면 코드
    # 기본값(`DEFAULT_VELOCITY_SCALING` = 1.0)을 쓴다.
    # `gravity_compensation` 은 **우리가 중력 보상을 대신 하는** 설정이다. 펑션베이의
    # 관절 제어에는 중력 보상이 없어 `실제 − 지령 = τ_g(q)/Kp` 로 처지는데, `Kp` 를 씬
    # XML 이 알려 주므로(t1·r2 모두 2000) 지령에 `τ_g/Kp` 를 미리 더해 상쇄할 수 있다.
    # **키가 없으면 보상하지 않는다** — 벤더가 A-1 을 반영하면 이 키를 지워 끈다.
    'motion': {'velocity_scaling', 'gravity_compensation'},
    # 그리퍼 **명령 규약**이 담긴 파일. `/input/gripper_joint` 는 이름 없는 배열이라
    # 순서·단위·packing 이 계약인데 **씬마다 다르다** (t1 은 6축·rad, r2 는 2축·deg).
    # 백엔드를 통째로 복제하는 대신 파일 하나를 가리킨다 — `scene.fixtures_file` 과
    # 같은 방식이다. 틀리면 에러가 아니라 "그리퍼가 1.7% 만 움직인다"로 나타난다.
    'gripper': {'profile_file'},
    # `source` 는 **raw 를 어떻게 얻는가**다 (`device`/`compressed`/`native`).
    # 키 유무로 추론하지 않는다 — `device` 와 `native` 는 둘 다 `image_topic` 만 갖는다.
    'camera': {'source', 'image_topic', 'compressed_topic'},
    # 팔 끝을 가리키는 **두 프레임**. `tip` 은 MoveIt 계획 그룹의 끝 링크라 데카르트
    # 목표가 해석되는 기준이고, `ee` 는 `/ee_pose` 가 발행하는 **작업 기준점**이다.
    # **둘은 같지 않다** — 펑션베이의 `grasp_center` 는 149 mm, mock 의 `panda_hand` 는
    # 107 mm 떨어져 있다. 그래서 `ee_pose` 를 읽어 그대로 데카르트 목표로 넣으면
    # 에러가 아니라 **그만큼 엉뚱한 곳으로 간다.**
    #
    # 이름을 여기 두는 것은 소비자가 둘이기 때문이다 — launch 의 `ee_pose_node` 와
    # 트윈의 `move_linear` 프레임 변환. 각자 적으면 한쪽만 뒤처진다.
    'frames': {'ee', 'tip'},
    # `file` 은 **움직이는** 물체의 이름·종류·크기 (pose 는 TF 에서 온다).
    # `fixtures_file` 은 **안 움직이는** 물체 — TF 에 아예 없어 pose 까지 파일이 갖는다.
    'scene': {'file', 'fixtures_file'},
}
# `factory` 는 백엔드 고유 코드가 있는 클래스 (`module:Class`). 없으면 기본 Backend.
# `solver` 는 **지금 띄운 씬의 솔버**다. 데이터 파일(그리퍼 규약·처짐 캘리브레이션)이
# 각자 `solver` 를 들고 있고, 로더가 이 값과 대조해 한쪽만 낡는 것을 막는다.
_TOP_LEVEL_KEYS = ({'name', 'description', 'factory', 'use_sim_time', 'solver'}
                   | set(_BLOCK_KEYS))
_FALLBACKS = {
    'arm_command_mode': 'auto',
    'arm_command_topic': DEFAULT_ARM_COMMAND_TOPIC,
    'arm_command_format': 'float64_multi_array',
    'arm_command_joint_names': [],
}


def profiles_dir() -> str:
    """프로파일 디렉터리. **설치본이 먼저다.**

    설치본을 먼저 보는 것은 배포된 값이 실제로 쓰이는 값이어야 하기 때문이다. 소스
    트리는 아직 빌드하지 않은 개발 중에만 쓰인다 — 그래서 소스를 고치고 빌드를
    빠뜨리면 옛 값이 읽힌다(README 의 주의).
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = os.path.join(get_package_share_directory('robot_control'),
                                 'config', 'backends')
        if os.path.isdir(installed):
            return installed
    except Exception:       # noqa: BLE001 — ament 가 없거나 미설치면 소스로 떨어진다
        pass

    # robot_control/robot_control/<this> -> robot_control/config/backends
    source = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          'config', 'backends')
    if os.path.isdir(source):
        return source
    raise FileNotFoundError(
        'backend profiles not found; looked for share/robot_control/config/backends '
        f'and {source}. Run colcon build from the workspace root.')


def _read(path: str) -> dict:
    """프로파일 하나를 소비자 키로 펼친다."""
    with open(path, encoding='utf-8') as handle:
        raw = yaml.safe_load(handle) or {}
    if not isinstance(raw, dict):
        raise ValueError(f'{path}: profile must be a mapping')

    unknown = set(raw) - _TOP_LEVEL_KEYS
    if unknown:
        # 조용히 무시하면 "설정했는데 안 먹는다" 가 된다. 새 블록을 더할 때는
        # `_TOP_LEVEL_KEYS` 와 `_BLOCK_KEYS` 를 함께 넓힌다.
        raise ValueError(f'{path}: unknown key(s) {sorted(unknown)}')

    for block, allowed in _BLOCK_KEYS.items():
        value = raw.get(block)
        if value is None:       # `null` 은 "이 백엔드에는 없다" 는 뜻이다
            continue
        if not isinstance(value, dict):
            raise ValueError(f'{path}: {block} must be a mapping or null')
        bad = set(value) - allowed
        if bad:
            raise ValueError(f'{path}: unknown {block} key(s) {sorted(bad)}')

    arm = raw.get('arm_command') or {}
    flat = {name: arm[key] for key, name in _ARM_COMMAND_KEYS.items() if key in arm}
    # 나머지 블록은 그대로 실어 나른다 — 해석은 소비자가 한다.
    for block in _BLOCK_KEYS:
        if block != 'arm_command' and block in raw:
            flat[block] = raw[block]
    for key in ('factory', 'use_sim_time'):
        if key in raw:
            flat[key] = raw[key]
    return flat


def load_profiles(directory: Optional[str] = None) -> dict:
    """디렉터리의 모든 프로파일을 읽는다. 키는 **파일 이름**(확장자 제외)이다.

    `name` 필드와 파일 이름이 다르면 오류다 — 참조는 파일 이름으로 하므로, 어긋나면
    문서와 실제가 갈린다.
    """
    directory = directory or profiles_dir()
    out = {}
    for entry in sorted(os.listdir(directory)):
        if not entry.endswith(('.yaml', '.yml')):
            continue
        stem = os.path.splitext(entry)[0]
        path = os.path.join(directory, entry)
        with open(path, encoding='utf-8') as handle:
            declared = (yaml.safe_load(handle) or {}).get('name')
        if declared is not None and declared != stem:
            raise ValueError(f"{path}: name {declared!r} does not match the file name "
                             f"{stem!r}; the file name is what `backend:` refers to")
        out[stem] = _read(path)
    if not out:
        raise FileNotFoundError(f'no backend profiles in {directory}')
    return out


class _Profiles(dict):
    """처음 읽을 때까지 파일을 건드리지 않는 지연 dict.

    import 시점에 파일을 읽으면 아직 빌드하지 않은 트리에서 **import 자체가** 실패해,
    프로파일과 무관한 코드까지 못 쓰게 된다.
    """

    _loaded = False

    def _ensure(self) -> None:
        if not self._loaded:
            super().update(load_profiles())
            self._loaded = True

    def __getitem__(self, key):
        self._ensure()
        return super().__getitem__(key)

    def __contains__(self, key):
        self._ensure()
        return super().__contains__(key)

    def __iter__(self):
        self._ensure()
        return super().__iter__()

    def __len__(self):
        self._ensure()
        return super().__len__()

    def keys(self):
        self._ensure()
        return super().keys()

    def items(self):
        self._ensure()
        return super().items()

    def get(self, key, default=None):
        self._ensure()
        return super().get(key, default)

    def reload(self) -> None:
        """파일을 다시 읽는다 (테스트·수정 후 확인용)."""
        super().clear()
        super().update(load_profiles())
        self._loaded = True


BACKEND_PROFILES = _Profiles()


def backends() -> tuple:
    """쓸 수 있는 백엔드 이름."""
    return tuple(sorted(BACKEND_PROFILES))


def resolve_backend_channel(backend: str, **overrides) -> dict:
    """프로파일과 명시 인자를 합쳐 arm 명령 채널 네 값을 정한다.

    우선순위는 **명시 인자 > 프로파일 > 전역 기본값** 이다. ``None`` 과 빈 문자열,
    빈 리스트가 "미설정" 을 뜻하므로 프로파일만 주고 한 값만 덮어쓸 수 있다.

    Args:
        backend: 프로파일 파일 이름 (확장자 제외). **기본값이 없다** — 모든 프로파일이
            확정 모드(`jtc`/`jgpc`)를 가지므로 "모르겠다" 를 뜻하는 이름이 없다.
            붙어 있는 스택에 맞추고 싶으면 `backend` 없이 `mode='auto'` 를 쓴다.
        **overrides: ``arm_command_mode`` / ``arm_command_topic`` /
            ``arm_command_format`` / ``arm_command_joint_names``.

    Returns:
        위 네 키를 모두 채운 dict.

    Raises:
        ValueError: 모르는 ``backend`` 일 때. **조용히 auto 로 떨어뜨리지 않는다** —
            오타가 "왜 이 스택에서만 안 되지" 로 나타난다.
    """
    if backend not in BACKEND_PROFILES:
        raise ValueError(f'backend must be one of {list(backends())}, got {backend!r}')
    profile = BACKEND_PROFILES[backend]

    def pick(name: str) -> Any:
        value = overrides.get(name)
        if isinstance(value, str) and value.strip():
            return value.strip()
        if isinstance(value, (list, tuple)) and [v for v in value if v]:
            return [v for v in value if v]
        return profile.get(name, _FALLBACKS[name])

    return {name: pick(name) for name in _FALLBACKS}


__all__ = ['BACKEND_PROFILES', 'DEFAULT_ARM_COMMAND_TOPIC', 'backends', 'load_profiles',
           'profiles_dir', 'resolve_backend_channel']
