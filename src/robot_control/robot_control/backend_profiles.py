#!/usr/bin/env python3

"""백엔드 프로파일 — 백엔드마다 갈리는 설정을 한 파일에 모은다.

**백엔드 하나 = 파일 하나.** `config/backends/<이름>.yaml` 이며, 파일 이름이 곧
`backend` 이름이다. 새 백엔드는 가장 가까운 기존 파일을 복사해 값만 바꾼다.

**이름은 `<로봇>_<환경>` 이다** (`panda_mock` · `panda_mock_jgpc` · `panda_isaac` ·
`panda_functionbay`, 2026-10-21). 프로파일의 값(관절 이름 · 컨트롤러 · 프레임 · 그리퍼 · 중력
모델)은 환경만이 아니라 로봇에도 묶여 있어서, 환경 이름(`isaac`)만 쓰면 두 번째 로봇(예: Isaac
위의 UR3 → `ur3_isaac`)이 올 때 한 이름이 두 조합을 가리키게 된다. 옛 이름은 한동안
`LEGACY_BACKEND_NAMES` 로 받는다(경고를 낸다).

**이 표가 "어느 스택이 무엇인가"의 정본이다.** 같은 사실을 launch·노드·트윈 설정에
각각 적으면 백엔드가 바뀔 때 한쪽만 뒤처지고, 그 어긋남은 에러가 아니라 "명령이 안
먹는다"로 나타난다 — Isaac 이 bridge → ros2_control 로 바뀔 때 실제로 그랬다.

**이 모듈은 `rclpy` 를 import 하지 않는다.** 설정만 읽는 경로(트윈의 `config.py`,
순수 파이썬 도구)가 ROS 없이 동작해야 하기 때문이다. `move_group_factory` 는 여기서
가져다 쓴다 — 그쪽은 `rclpy` 를 끌어온다.

**한 백엔드 안의 시뮬레이터 씬**(펑션베이 `r2`/`t1`)은 파일을 나누지 않고 `sim_scene` 한 줄로
고른다 — 씬에 딸린 값(솔버 · 그리퍼 규약 · 중력 보상)만 `sim_scenes.<씬>` 에 두고 로더가 공통 블록
위에 얹는다(`_apply_sim_scene`). 파일을 씬마다 복제하면 공통 값이 갈라지고, 백엔드 이름을 나누면
launch · 트윈 · 스크립트가 각자 이름을 받아 서로 다른 씬을 읽을 수 있다.

파일을 고친 뒤에는 **`colcon build` 를 다시 돌린다.** 설치본(`share/`)이 먼저 읽히므로
소스만 고치면 낡은 값이 그대로 쓰인다.
"""

from __future__ import annotations

from typing import Any, Optional

import os
import warnings

import yaml

# 옛 백엔드 이름 → 지금 이름 (2026-10-21). 손에 익은 명령과 외부 스크립트가 바로 깨지지 않게
# 한동안 받는다. **새 코드 · 문서는 지금 이름만 쓴다.** 언젠가 이 표를 지우면 옛 이름은
# 모르는 이름이 되어 거부된다.
LEGACY_BACKEND_NAMES = {
    'mock': 'panda_mock',
    'mock_jgpc': 'panda_mock_jgpc',
    'isaac': 'panda_isaac',
    'functionbay': 'panda_functionbay',
}


def canonical_backend(name: str) -> str:
    """옛 이름이면 지금 이름으로 바꾸고 경고한다. 그 밖의 이름은 그대로 돌려준다.

    모르는 이름인지는 판정하지 않는다 — 호출자가 `BACKEND_PROFILES` 로 확인한다. 사용자
    입력을 받는 입구(`get_backend` · 팩토리 · 노드 파라미터 · 트윈 설정)에서 부른다.
    """
    new = LEGACY_BACKEND_NAMES.get(name)
    if new is None:
        return name
    warnings.warn(f"backend '{name}' is deprecated; use '{new}'", FutureWarning, stacklevel=2)
    return new


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
# 표준 토픽이다.
#
# **`arm_command_mode` 에는 폴백이 없다 (`None`).** 모든 프로파일이 확정 모드를 갖고,
# 빠뜨렸으면 짐작하지 말고 팩토리가 그 사실을 말해야 한다 — 예전의 `'auto'` 폴백은
# 런타임 판별로 되돌아가 펑션베이를 JTC 로 오판하는 길을 열어 두고 있었다.
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
    #
    # `camera_compressed_topic` 은 **시뮬레이터 원본** 카메라다. 소비자가 받는
    # `camera.compressed_topic` 과 다를 수 있다 — 펑션베이는 그 사이에서 stamp 를
    # 관절과 같은 축으로 옮기는 중계(`fb_camera_restamp`)가 돈다.
    'simulator': {'joint_report_topic', 'joint_command_topic', 'gripper_command_topic',
                  'gripper_report_topic', 'camera_compressed_topic'},
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
    #
    # `resolution`/`fps` 는 **정본이 다른 곳에 없을 때만** 적는다. 레코더는 CFR 이라
    # 이 값이 곧 영상의 시간축이 되므로 두 곳에 적으면 어긋났을 때 조용히 틀린 영상이
    # 나온다. 그래서:
    #   · mock 계열  — `image_pipeline.yaml` 이 정본이다 (여기 적지 않는다)
    #   · Isaac      — `scene.file`(`isaac_scene.json`)의 `camera` 블록이 정본이고
    #                  시뮬레이터도 그것을 읽는다 (여기 적지 않는다)
    #   · 펑션베이   — 공유 정의 파일이 없다 (여기가 정본이다)
    'camera': {'source', 'image_topic', 'compressed_topic', 'resolution', 'fps',
               'pixel_format'},
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
# `solver` 는 **지금 띄운 씬의 솔버**다. 데이터 파일(그리퍼 규약)이 각자 `solver` 를
# 들고 있고, 로더가 이 값과 대조해 한쪽만 낡는 것을 막는다.
#
# `sim_scene` · `sim_scenes` 는 **한 백엔드 안에서 시뮬레이터 씬을 바꿔 띄우는** 경우다
# (펑션베이의 `r2`/`t1` — 솔버·그리퍼 규약·중력 보상 주체가 씬마다 다르다). 씬에 딸린
# 값만 `sim_scenes.<씬>` 에 두고 `sim_scene` 한 줄로 고른다. `scene` 블록(조작 대상 ·
# 고정물 파일)과는 다른 것이다 — 이름이 겹쳐 `sim_` 을 붙였다.
_TOP_LEVEL_KEYS = ({'name', 'description', 'factory', 'use_sim_time', 'solver',
                    'sim_scene', 'sim_scenes'}
                   | set(_BLOCK_KEYS))
# `sim_scenes.<씬>` 에 둘 수 있는 것. 블록(`gripper` · `motion`)은 **공통 블록 위에 키
# 단위로 얹힌다** — 같은 키를 공통과 씬 양쪽에 적으면 어느 쪽이 이기는지 헷갈리므로 거부한다.
_SIM_SCENE_KEYS = {'solver', 'gripper', 'motion'}
_FALLBACKS = {
    'arm_command_mode': None,
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


def _scene_keys(block: dict) -> set:
    """씬 블록이 정하는 키 — `solver` 와 `(블록, 키)` 쌍."""
    keys = {'solver'} if 'solver' in block else set()
    for sub in ('gripper', 'motion'):
        keys |= {(sub, key) for key in (block.get(sub) or {})}
    return keys


def _apply_sim_scene(path: str, raw: dict) -> dict:
    """`sim_scenes.<sim_scene>` 의 값을 공통 값 위에 얹은 사본을 돌려준다.

    **모든 씬이 같은 키를 정해야 한다.** 한 씬에만 있는 키는 다른 씬으로 바꿨을 때 공통
    값(또는 '없음')으로 조용히 떨어지는데, 그것이 의도인지 빠뜨림인지 파일만 봐서는 모른다.
    '이 씬에는 없다' 는 `null` 로 적는다 (r2 의 `gravity_compensation: null`).
    """
    scenes = raw.get('sim_scenes')
    active = raw.get('sim_scene')
    if scenes is None and active is None:
        return raw
    if not isinstance(scenes, dict) or not scenes:
        raise ValueError(f'{path}: sim_scenes must be a non-empty mapping when sim_scene is set')
    if active not in scenes:
        raise ValueError(f'{path}: sim_scene {active!r} is not one of {sorted(scenes)}')

    shape = None
    for name, block in scenes.items():
        if not isinstance(block, dict):
            raise ValueError(f'{path}: sim_scenes.{name} must be a mapping')
        bad = set(block) - _SIM_SCENE_KEYS
        if bad:
            raise ValueError(f'{path}: unknown sim_scenes.{name} key(s) {sorted(bad)}')
        if 'solver' in block and 'solver' in raw:
            raise ValueError(
                f'{path}: solver is set both at the top level and in sim_scenes.{name}')
        for sub in ('gripper', 'motion'):
            value = block.get(sub)
            if value is None:
                continue
            if not isinstance(value, dict):
                raise ValueError(f'{path}: sim_scenes.{name}.{sub} must be a mapping')
            unknown = set(value) - _BLOCK_KEYS[sub]
            if unknown:
                raise ValueError(
                    f'{path}: unknown sim_scenes.{name}.{sub} key(s) {sorted(unknown)}')
            twice = set(value) & set(raw.get(sub) or {})
            if twice:
                raise ValueError(f'{path}: {sub} key(s) {sorted(twice)} are set both in the common '
                                 f'block and in sim_scenes.{name}; keep them in one place')
        keys = _scene_keys(block)
        if shape is None:
            shape = keys
        elif keys != shape:
            raise ValueError(f'{path}: every sim_scenes entry must set the same keys; '
                             f'{name!r} differs by {sorted(map(str, keys ^ shape))}')

    chosen = scenes[active]
    merged = {key: value for key, value in raw.items() if key != 'sim_scenes'}
    if 'solver' in chosen:
        merged['solver'] = chosen['solver']
    for sub in ('gripper', 'motion'):
        if sub in chosen:
            merged[sub] = {**(raw.get(sub) or {}), **(chosen[sub] or {})}
    return merged


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

    raw = _apply_sim_scene(path, raw)

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
    # `solver` · `sim_scene` 도 싣는다 — 데이터 파일 대조(`gripper.profile._verify_solver`)와
    # 기동 시 씬 대조가 이 값을 읽는다. 2026-10-20 까지 `solver` 를 빠뜨려 그 대조가
    # **한 번도 작동하지 않았다** (항상 `None` 이라 건너뛰었다).
    for key in ('factory', 'use_sim_time', 'solver', 'sim_scene'):
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

    조회(`[]` · `in` · `get`)는 옛 이름도 **조용히** 받는다 — 경고는 사용자 입력을 받는 입구가
    `canonical_backend` 로 한 번 낸다. 여기서도 내면 같은 이름에 경고가 여러 번 찍힌다.
    순회(`keys` · `items` · `backends()`)는 지금 이름만 낸다.
    """

    _loaded = False

    def _ensure(self) -> None:
        if not self._loaded:
            super().update(load_profiles())
            self._loaded = True

    def __getitem__(self, key):
        self._ensure()
        return super().__getitem__(LEGACY_BACKEND_NAMES.get(key, key))

    def __contains__(self, key):
        self._ensure()
        return super().__contains__(LEGACY_BACKEND_NAMES.get(key, key))

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
        return super().get(LEGACY_BACKEND_NAMES.get(key, key), default)

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
            프로파일에 없는 스택이면 `backend` 없이 `mode='jtc'`/`'jgpc'` 를 명시한다.
        **overrides: ``arm_command_mode`` / ``arm_command_topic`` /
            ``arm_command_format`` / ``arm_command_joint_names``.

    Returns:
        위 네 키를 모두 채운 dict.

    Raises:
        ValueError: 모르는 ``backend`` 일 때. **조용히 auto 로 떨어뜨리지 않는다** —
            오타가 "왜 이 스택에서만 안 되지" 로 나타난다.
    """
    backend = canonical_backend(backend)
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


__all__ = ['BACKEND_PROFILES', 'DEFAULT_ARM_COMMAND_TOPIC', 'LEGACY_BACKEND_NAMES', 'backends',
           'canonical_backend', 'load_profiles', 'profiles_dir', 'resolve_backend_channel']
