#!/usr/bin/env python3

"""백엔드 하나를 대표하는 객체.

**백엔드마다 갈리는 것이 두 가지다** — *설정 값*과 *만들어야 하는 객체*. 값은
`config/backends/<이름>.yaml` 에 있고, 객체 생성은 이 클래스(와 그 하위 클래스)에 있다.
소비자는 이름 하나로 시작한다::

    backend = get_backend('isaac')
    client = backend.create_move_group_client(node)
    params = backend.servo_parameters()

**이 모듈은 `rclpy` 를 import 하지 않는다.** launch 파일과 설정만 읽는 도구가 ROS 없이
쓸 수 있어야 하기 때문이다 — ROS 객체를 만드는 메서드만 그 안에서 지연 import 한다.

하위 클래스는 **그 백엔드에서만 다른 코드**를 담는다. 값만 다른 것은 YAML 에 두고,
값으로 표현할 수 없는 것(어떤 노드를 띄우는가, 어떤 보정이 필요한가)만 코드로 내린다.
"""

from __future__ import annotations

from typing import Any, Optional

import os

from robot_control.backend_profiles import BACKEND_PROFILES, backends, profiles_dir


class Backend:
    """프로파일 하나를 들고 그것으로 객체를 만든다.

    기본 구현은 **값만 읽는다.** 백엔드 고유 동작이 필요해지면 하위 클래스가 덮는다.
    """

    def __init__(self, name: str, profile: dict) -> None:
        self.name = name
        self._profile = dict(profile)

    def __repr__(self) -> str:
        return f'{type(self).__name__}({self.name!r})'

    # ----- 값 조회 -------------------------------------------------------

    def block(self, name: str) -> Optional[dict]:
        """블록 하나. **``None`` 은 "이 백엔드에는 없다"** 는 뜻이다."""
        return self._profile.get(name)

    def value(self, block: str, key: str, default: Any = None) -> Any:
        """블록 안의 값 하나. 블록이 `None` 이면 `default`."""
        return (self.block(block) or {}).get(key, default)

    @property
    def use_sim_time(self) -> bool:
        return bool(self._profile.get('use_sim_time', False))

    # `arm_command` 는 다른 블록과 달리 **최상위로 평탄화되어** 있다
    # (`arm_command_mode` 등) — `resolve_backend_channel` 소비자가 평평한 키를
    # 기대하기 때문이다. 그 비대칭을 밖으로 새지 않게 접근자로 감싼다.

    @property
    def arm_command_mode(self) -> str:
        """`jtc` 또는 `jgpc`."""
        return self._profile.get('arm_command_mode', 'auto')

    @property
    def arm_command_topic(self) -> Optional[str]:
        """JGPC 클라이언트가 궤적을 흘릴 토픽. JTC 백엔드는 `None` 이다."""
        return self._profile.get('arm_command_topic')

    @property
    def arm_command_format(self) -> Optional[str]:
        return self._profile.get('arm_command_format')

    @property
    def arm_command_joint_names(self) -> list:
        """JGPC 명령 배열의 관절 순서. 비어 있으면 컨트롤러에서 조회한다."""
        return list(self._profile.get('arm_command_joint_names') or [])

    @property
    def velocity_scaling(self) -> Optional[float]:
        """팔 이동의 전역 기본 속도 배율. 없으면 `None` (코드 기본값 1.0 을 쓴다).

        **인자로 준 값이 이긴다** — `create_move_group_client(velocity_scaling=...)`
        이나 각 이동 메서드의 인자를 주면 그쪽이 먼저다. 이 값은 아무것도 안 줬을 때의
        바닥값이다.
        """
        value = self.value('motion', 'velocity_scaling')
        return None if value is None else float(value)

    @property
    def camera_source(self) -> Optional[str]:
        """raw 이미지를 **어떻게 얻는가**. `camera` 블록이 없으면 `None` (카메라가 없다).

        `device` 는 OpenCV `camera_node` 가 장치/파일에서 캡처하고, `compressed` 는
        시뮬레이터가 압축만 주어 `image_transport/republish` 가 raw 를 만들며, `native`
        는 시뮬레이터가 raw 를 직접 낸다(우리가 만드는 발행자가 없다).

        **키 유무로 추론하지 않는 이유**는 `device` 와 `native` 가 둘 다 `image_topic`
        만 갖기 때문이다 — 추론하면 mock 에 `compressed_topic` 을 실수로 넣었을 때
        조용히 다른 노드가 뜬다. 설계: `docs/camera/compressed_image_pipeline_design.md`.
        """
        return self.value('camera', 'source')

    @property
    def hardware_type(self) -> Optional[str]:
        """`ros2_control` 의 xacro 하드웨어 분기. 없으면 `None` (ros2_control 미사용)."""
        return self.value('ros2_control', 'hardware_type')

    @property
    def uses_ros2_control(self) -> bool:
        return self.block('ros2_control') is not None

    # ----- 경로 ----------------------------------------------------------

    def share_path(self, relpath: Optional[str]) -> Optional[str]:
        """패키지 share 기준 상대 경로를 절대 경로로. `None` 은 그대로 `None`.

        프로파일의 경로는 **`robot_control` 패키지 안**을 가리킨다 — 설치본에서
        해석되므로 패키지 밖 파일은 담을 수 없다.
        """
        if not relpath:
            return None
        return os.path.join(_share_dir(), relpath)

    def controllers_file(self) -> Optional[str]:
        """ros2_control 컨트롤러 params 파일의 절대 경로.

        `None` 이면 **호출자가 기본값을 쓴다** — mock 은 `moveit_resources` 것을 그대로
        쓰므로 프로파일에 적지 않는다.

        내용이 아니라 경로인 이유: Humble 의 controller_manager 는 컨트롤러 파라미터를
        **params 파일로만** 넘긴다. launch 의 인라인 dict 는 CM 노드에는 실리지만
        컨트롤러 노드에는 닿지 않는다 (실측: CM `True`, 컨트롤러 `False`).
        """
        return self.share_path(self.value('ros2_control', 'controllers_file'))

    def scene_file(self) -> Optional[str]:
        """scene 물체 정의 파일의 절대 경로. `None` 이면 정의 파일이 없다."""
        return self.share_path(self.value('scene', 'file'))

    # ----- 객체 생성 ------------------------------------------------------

    def create_move_group_client(self, node, **kwargs):
        """이 백엔드에 맞는 :class:`MoveGroupClient` 구현.

        `rclpy` 는 여기서만 필요하므로 **지연 import** 한다.
        """
        from robot_control.moveit.move_group_factory import create_move_group_client

        return create_move_group_client(node, backend=self.name, **kwargs)

    def servo_parameters(self) -> dict:
        """`moveit_servo` 파라미터에 덮어쓸 값. **키에 접두사를 붙이지 않는다.**

        launch 헬퍼가 `parameter_namespace="moveit_servo"` 로 읽은 dict 안쪽에 그대로
        합쳐지므로, 여기서 `moveit_servo.` 를 붙이면 이중이 되어 조용히 안 먹는다.

        기본 구현은 프로파일에 적힌 것만 옮긴다. 값으로 표현되지 않는 보정이 필요한
        백엔드는 하위 클래스가 덮는다.
        """
        out: dict = {}
        scale = self.value('servo', 'linear_scale')
        if scale is not None:
            # **중첩 dict 다.** launch 가 파라미터로 넘길 때 `moveit_servo.scale.linear`
            # 로 평탄화된다 — 점 있는 한 키로 두면 같게 풀린다는 보장이 없다. 기존
            # launch 들이 모두 `params['scale']['linear']` 로 쓰는 관례를 따른다.
            out['scale'] = {'linear': float(scale)}
        out_type = self.value('servo', 'command_out_type')
        if out_type is not None:
            out['command_out_type'] = out_type
            out.update(_float64_multi_array_rule(out_type))
        topic = self.value('servo', 'command_out_topic')
        if topic is not None:
            out['command_out_topic'] = topic
        return out

    def apply_servo_parameters(self, servo_params: dict) -> dict:
        """`load_servo_params()` 가 돌려준 dict 에 이 백엔드의 값을 얹는다.

        구조가 ``{'moveit_servo': {...}}`` 라 그 안쪽에 합친다. 원본을 바꾸지 않는다.
        """
        merged = {k: (dict(v) if isinstance(v, dict) else v) for k, v in servo_params.items()}
        target = merged.setdefault('moveit_servo', {})
        for key, value in self.servo_parameters().items():
            if isinstance(value, dict) and isinstance(target.get(key), dict):
                # `scale` 처럼 하위 키가 있는 블록은 **통째로 갈아치우지 않는다** —
                # `scale.rotational` 같은 형제 값이 사라진다.
                target[key] = {**target[key], **value}
            else:
                target[key] = value
        return merged


def _float64_multi_array_rule(command_out_type: str) -> dict:
    """`Float64MultiArray` 출력이면 **반드시** 따라오는 servo 설정.

    **백엔드 특성이 아니라 servo 의 제약이다.** 그 형식에서 positions 와 velocities 를
    함께 발행하도록 두면 servo 의 파라미터 검증이 실패를 반환하고 **노드가 아예 뜨지
    않는다.** 그래서 백엔드마다 되적지 않고 여기 한 곳에 둔다 — 되적으면 새 백엔드에서
    빠뜨리고, 증상은 "servo 가 안 뜬다" 뿐이라 원인을 안 가리킨다.
    """
    if command_out_type != 'std_msgs/Float64MultiArray':
        return {}
    return {'publish_joint_positions': True,
            'publish_joint_velocities': False,
            'publish_joint_accelerations': False}


def _share_dir() -> str:
    """프로파일을 읽어 온 트리의 share 루트.

    **`profiles_dir()` 에서 파생시킨다** — 프로파일과 그것이 가리키는 파일은 같은 트리에서
    와야 한다. 예전에는 여기서 `get_package_share_directory` 를 따로 불렀는데, 그러면
    설치본에 `config/backends/` 가 아직 없는 동안 `profiles_dir()` 은 소스로 떨어지고
    이쪽만 설치본을 가리켜 **값과 경로의 출처가 갈렸다** (실측으로 재현했다).
    """
    # <root>/config/backends -> <root>
    return os.path.dirname(os.path.dirname(profiles_dir()))


def get_backend(name: str) -> 'Backend':
    """이름으로 백엔드 객체를 만든다.

    YAML 의 ``factory`` 가 클래스를 지정하면 그것을, 없으면 :class:`Backend` 를 쓴다.

    Raises:
        ValueError: 모르는 이름일 때. **조용히 기본값으로 떨어뜨리지 않는다** —
            오타가 "이 스택에서만 안 되네" 로 나타난다.
    """
    if name not in BACKEND_PROFILES:
        raise ValueError(f'backend must be one of {list(backends())}, got {name!r}')
    profile = BACKEND_PROFILES[name]

    spec = profile.get('factory')
    if not spec:
        return Backend(name, profile)

    module_name, _, class_name = spec.partition(':')
    if not class_name:
        raise ValueError(f"{name}: factory must be 'module:Class', got {spec!r}")
    import importlib
    cls = getattr(importlib.import_module(module_name), class_name)
    return cls(name, profile)


__all__ = ['Backend', 'get_backend']
