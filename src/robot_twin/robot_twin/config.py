"""트윈 정의 YAML 의 스키마와 로더 (설계서 3장).

변수와 연산을 코드에 하드코딩하지 않고 YAML 로 선언한다. 스키마 검증은 Pydantic
v2 가 담당하며, 잘못된 설정은 **기동 시점에** 실패한다 (설계서 2.6 의 1단계).

ROS 의존성이 없으므로 ROS 를 source 하지 않아도 import 할 수 있다.
"""

from __future__ import annotations

from typing import Any, Literal, Optional, Union

from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator


# `create_move_group_client()` 에 넘길 수 있는 값 중 트윈이 허용하는 것.
# 'auto' 는 의도적으로 제외한다 — 설계서 2.6 참조.
MoveGroupMode = Literal['jtc', 'jgpc']

# JGPC 명령 메시지 형식. `float64_multi_array` 는 ros2_control 의 JGPC 이고,
# `joint_state` 는 토픽 연동형 시뮬레이터(Isaac Sim · 펑션베이)다.
ArmCommandFormat = Literal['float64_multi_array', 'joint_state']

# 상태 변수 소스 종류 (설계서 4.3).
SourceType = Literal['topic', 'tf', 'service', 'static', 'derived']

# 연산의 동기/비동기 구분. 연산별로 고정되며 런타임에 바뀌지 않는다 (설계서 6.2).
OperationKind = Literal['sync', 'async']

# 자원 배타 락의 단위 (설계서 6.3). None 이면 자원을 점유하지 않는다.
ResourceName = Literal['arm', 'gripper', 'scene']


class _Base(BaseModel):
    """설정 모델 공통 베이스 — 오타를 조용히 넘기지 않는다."""

    model_config = ConfigDict(extra='forbid')


class TwinMeta(_Base):
    """트윈 식별 정보."""

    id: str
    description: str = ''

    @field_validator('id')
    @classmethod
    def _check_id(cls, v: str) -> str:
        # URL 경로 세그먼트로 쓰이므로 안전한 문자만 허용한다.
        if not v or not all(c.isalnum() or c in '-_' for c in v):
            raise ValueError(f"twin.id must be alphanumeric with '-'/'_' only, got {v!r}")
        return v


class HttpConfig(_Base):
    """HTTP 서버 바인딩 설정 (설계서 8.4).

    현 단계에서는 인증·TLS 를 도입하지 않으므로 노출 범위 통제는 망 수준(방화벽,
    네트워크 분리)의 책임이다. 트윈은 신뢰된 폐쇄망 안에 있다고 전제한다.
    """

    host: str = '0.0.0.0'
    port: int = Field(default=8801, ge=1, le=65535)
    tls: bool = False

    @field_validator('tls')
    @classmethod
    def _reject_tls(cls, v: bool) -> bool:
        # TLS 는 현재 구현 범위 밖이다 (설계서 10.4). 설정만 켜고 평문으로 뜨는
        # 상황을 막기 위해 명시적으로 거부한다.
        if v:
            raise ValueError('http.tls is not supported yet; terminate TLS in front of the twin')
        return v


class RosConfig(_Base):
    """ROS 2 환경 설정.

    ``domain_id`` / ``rmw`` 는 프로세스 환경변수로 적용되므로 rclpy 초기화 전에
    설정되어야 한다 (설계서 2.4 — 1 트윈 = 1 프로세스).
    """

    domain_id: Optional[int] = Field(default=None, ge=0, le=232)
    rmw: Optional[str] = None
    node_name: str = 'robot_twin'
    use_sim_time: bool = False


class MoveItConfig(_Base):
    """MoveIt 연동 설정.

    ``move_group_mode`` 는 **필수**이며 ``'auto'`` 를 허용하지 않는다. auto 판별은
    토픽 그래프 lookup 이라 DDS 디스커버리 전에 호출하면 조용히 오판하기 때문이다
    (설계서 2.6).
    """

    # 백엔드 이름 하나로 아래 넷을 채운다 (`robot_control.moveit.BACKEND_PROFILES`).
    # **그 표가 "어느 스택이 JTC 인가"의 정본이다** — 여기 손으로 적으면 백엔드가
    # 바뀔 때 한쪽만 뒤처지고, 어긋남은 에러가 아니라 "명령이 안 먹는다"로 나온다.
    #
    # 명시한 키가 프로파일을 이기므로 `backend` 와 개별 키를 섞어 쓸 수 있다.
    # 모든 프로파일이 확정 모드(`jtc`/`jgpc`)를 가지므로 어느 것이든 쓸 수 있다 —
    # mock 계열은 `mock`(JTC) 과 `mock_jgpc`(JGPC) 로 나뉘어 있다.
    backend: Optional[str] = None

    move_group_mode: Optional[MoveGroupMode] = None
    planning_group: str = 'panda_arm'

    # --- 프레임 ---
    #
    # **데카르트 목표는 `tip_frame` 기준으로 해석된다** (MoveIt 그룹의 tip link).
    # 그런데 사람이 다루는 프레임은 보통 그것이 아니다 — 펑션베이는 `/ee_pose` 가
    # `grasp_center` 를 가리키고 tip(`panda_link8`) 과 **149 mm** 떨어져 있다.
    #
    # `move_linear` 의 `frame` 입력이 `ee_frame` 이면 트윈이 TF 로 변환한다. 둘이
    # 같으면 변환이 항등이라 mock·Isaac 은 영향이 없다.
    ee_frame: Optional[str] = None
    tip_frame: str = 'panda_link8'

    # --- JGPC 명령 채널 (토픽 연동형 시뮬레이터용) ---
    #
    # 셋 다 생략하면 `create_move_group_client` 의 기본값을 쓴다 — ros2_control 의
    # JGPC(`/panda_arm_controller/commands`, Float64MultiArray)다.
    #
    # Isaac Sim 처럼 **시뮬레이터가 직접 JointState 를 받는** 백엔드는 셋을 모두
    # 지정해야 한다. 토픽 remap 으로는 못 바꾼다 — 메시지 타입이 다르다.
    arm_command_topic: Optional[str] = None
    arm_command_format: Optional[ArmCommandFormat] = None
    arm_command_joint_names: Optional[list[str]] = None

    # --- 팔 이동 속도 ---
    #
    # 생략하면 `backend` 프로파일의 `motion.velocity_scaling` 이 채우고, 그것도 없으면
    # `create_move_group_client` 의 코드 기본값(1.0)이 된다. 여기 적으면 프로파일보다
    # 우선한다 — **명시한 키가 이긴다**는 이 클래스의 다른 값들과 같은 규칙이다.
    velocity_scaling: Optional[float] = None

    # --- 명령 스트리밍 주기 (JGPC 전용) ---
    #
    # 생략하면 `backend` 프로파일의 `arm_command.publish_rate` 가 채운다. 안 채워지면
    # 데카르트 궤적이 **10 Hz 계단**으로 나간다 (MoveIt 이 TOTG 로 재샘플하고 이 경로에는
    # 보간할 컨트롤러가 없다). 접촉을 동반하는 작업에서 그 계단이 실제로 실패를 만든다 —
    # 여유 0.5 mm 인 `peg_tray` 삽입이 10 Hz 로는 3회 모두 입구에서 튕겼다 (2026-09-11).
    arm_command_publish_rate: Optional[float] = None

    @model_validator(mode='before')
    @classmethod
    def _expand_backend(cls, data: Any) -> Any:
        """`backend` 를 네 값으로 펼친다. **명시한 키가 이긴다.**

        `robot_control.backend_profiles` 는 **`rclpy` 를 끌어오지 않는다** — 이 모듈이
        ROS 없이도 import 되어야 하기 때문이다(설정만 읽는 경로가 있다). 지연 import 는
        `robot_control` 이 없는 환경에서도 나머지 설정을 읽을 수 있게 남겨 둔다.
        """
        if not isinstance(data, dict) or not data.get('backend'):
            return data

        from robot_control.backend_profiles import resolve_backend_channel

        channel = resolve_backend_channel(
            data['backend'],
            arm_command_mode=data.get('move_group_mode'),
            arm_command_topic=data.get('arm_command_topic'),
            arm_command_format=data.get('arm_command_format'),
            arm_command_joint_names=data.get('arm_command_joint_names'))

        # 우선순위(명시 > 프로파일)는 `resolve_backend_channel` 이 이미 적용했다.
        # `setdefault` 는 그것을 다시 확인하는 것일 뿐이라 결과가 같다.
        filled = dict(data)
        # **팔 이동 속도도 프로파일에서 온다.** 트윈은 `backend=` 를 팩토리에 넘기지 않고
        # 여기서 값으로 펼치므로(`client_kwargs`), 이 줄이 없으면 프로파일에 적어도
        # 트윈에만 안 먹는다 — 그 어긋남은 "왜 트윈으로 움직일 때만 빠르지" 로 나타난다.
        from robot_control.backend_profiles import BACKEND_PROFILES
        profile = BACKEND_PROFILES[data['backend']]
        motion = profile.get('motion') or {}
        if motion.get('velocity_scaling') is not None:
            filled.setdefault('velocity_scaling', motion['velocity_scaling'])

        # **프레임 이름도 프로파일에서 온다.** launch 의 `ee_pose_node` 가 읽는 것과
        # 같은 값이어야 `ee_pose` 가 가리키는 프레임과 트윈의 변환 기준이 일치한다 —
        # 두 곳에 적으면 어긋남이 에러 없이 **149 mm 빗나간 이동**으로 나타난다.
        rate = profile.get('arm_command_publish_rate')
        if rate is not None:
            filled.setdefault('arm_command_publish_rate', rate)

        frames = profile.get('frames') or {}
        for key in ('ee', 'tip'):
            if frames.get(key):
                filled.setdefault(f'{key}_frame', frames[key])
        filled.setdefault('move_group_mode', channel['arm_command_mode'])
        # JTC 로 결정되면 명령 채널은 넣지 않는다 — 아래 `_check_arm_command` 가
        # 그 조합을 거부하는데, 프로파일이 채운 값 때문에 걸리면 사용자는 자기가
        # 안 적은 키로 실패하는 셈이 된다.
        if filled['move_group_mode'] != 'jtc':
            for key in ('arm_command_topic', 'arm_command_format',
                        'arm_command_joint_names'):
                if channel[key]:
                    filled.setdefault(key, channel[key])
        return filled

    @field_validator('move_group_mode', mode='before')
    @classmethod
    def _reject_auto(cls, v: Any) -> Any:
        if v == 'auto':
            raise ValueError(
                "moveit.move_group_mode must be 'jtc' or 'jgpc'; 'auto' is forbidden "
                'because topic-graph detection can silently mis-detect the stack'
            )
        return v

    @model_validator(mode='after')
    def _check_arm_command(self) -> 'MoveItConfig':
        """명령 채널 설정의 짝을 맞춘다. **조용히 무시되는 키를 만들지 않는다.**"""
        # `backend` 로도 안 채워졌으면 여전히 필수다.
        if self.move_group_mode is None:
            raise ValueError(
                'moveit.move_group_mode is required (or give moveit.backend to fill it)')
        # **`velocity_scaling` 은 여기 없다** — JGPC 전용이 아니라 두 클라이언트가 다
        # 쓰는 값이라 JTC 에서도 정당하다.
        given = {
            'arm_command_topic': self.arm_command_topic,
            'arm_command_format': self.arm_command_format,
            'arm_command_joint_names': self.arm_command_joint_names,
        }
        present = sorted(k for k, v in given.items() if v is not None)

        # JTC 는 이 값들을 아예 보지 않는다. 남겨 두면 '설정했는데 안 먹는' 상태가
        # 되고, 증상은 팔이 엉뚱한 토픽으로 명령을 보내는 것뿐이라 원인이 안 보인다.
        if self.move_group_mode == 'jtc' and present:
            raise ValueError(
                f"moveit.{'/'.join(present)} only applies to move_group_mode 'jgpc'; "
                "the JTC client executes through the MoveGroup action instead"
            )

        # joint_state 형식에는 컨트롤러가 없다 — 관절 이름을 조회할 곳이 없으므로
        # 반드시 명시해야 한다. 빠지면 첫 스트리밍에서 조회를 기다리다 멈춘다.
        if self.arm_command_format == 'joint_state' and not self.arm_command_joint_names:
            raise ValueError(
                "moveit.arm_command_joint_names is required when arm_command_format is "
                "'joint_state'; there is no controller to query the joint order from"
            )
        return self

    def client_kwargs(self) -> dict:
        """`create_move_group_client` 에 넘길 추가 인자. 지정한 것만 담는다.

        **JSON 으로 나갈 수 있는 값만 담는다** — `/health` 가 이 결과를 `arm_command`
        로 그대로 실어 보내기 때문이다. 중력 보상기 같은 객체는 여기 넣지 않고
        :meth:`gravity_compensator` 로 따로 준다 (2026-09-11 에 여기 넣었다가
        `/health` 가 500 으로 죽었다).
        """
        kwargs = {}
        if self.arm_command_topic is not None:
            kwargs['arm_command_topic'] = self.arm_command_topic
        if self.arm_command_format is not None:
            kwargs['arm_command_format'] = self.arm_command_format
        if self.arm_command_joint_names is not None:
            kwargs['arm_command_joint_names'] = list(self.arm_command_joint_names)
        if self.velocity_scaling is not None:
            kwargs['velocity_scaling'] = self.velocity_scaling
        # **JGPC 로 풀릴 때만 넘긴다** — JTC 생성자는 이 인자를 모른다. 팩토리도 JTC
        # 분기에서 지우지만, 여기서 안 넣는 편이 의도가 분명하다.
        if self.arm_command_publish_rate is not None and self.move_group_mode != 'jtc':
            kwargs['publish_rate'] = self.arm_command_publish_rate
        return kwargs

    def gravity_compensator(self):
        """백엔드의 중력 보상기. 설정이 없으면 `None`.

        **팩토리가 대신 해 주지 않는다.** 팩토리는 `backend=` 를 받았을 때만 프로파일을
        읽는데, 트윈은 백엔드를 **값으로 펼쳐** 넘기므로(`client_kwargs`) 그 훅이 안
        걸린다. 이것이 없으면 **같은 로봇인데 스크립트로 움직일 때는 보상이 걸리고
        트윈으로 움직일 때는 안 걸린다** — 증상은 "트윈으로만 팔이 몇 mm 어긋난 곳에
        선다" 뿐이다 (2026-09-11 실측).
        """
        if not self.backend:
            return None
        from robot_control.gravity import load_gravity_compensator

        return load_gravity_compensator(self.backend)


class QosConfig(_Base):
    """토픽 구독 QoS. TRANSIENT_LOCAL 토픽은 durability 를 맞춰야 수신된다."""

    reliability: Literal['reliable', 'best_effort'] = 'reliable'
    durability: Literal['volatile', 'transient_local'] = 'volatile'
    depth: int = Field(default=1, ge=1)


class SourceConfig(_Base):
    """상태 변수의 값 획득 방법 (설계서 4.3)."""

    type: SourceType

    # type == 'topic'
    topic: Optional[str] = None
    msg: Optional[str] = None
    qos: QosConfig = Field(default_factory=QosConfig)

    # type == 'tf'
    base_frame: Optional[str] = None
    tip_frame: Optional[str] = None
    rate_hz: float = Field(default=10.0, gt=0.0)

    # type == 'service' / 'static'
    backend: Optional[dict[str, Any]] = None
    timeout_sec: float = Field(default=2.0, gt=0.0)

    # type == 'derived'
    source_variable: Optional[str] = None
    extract: Optional[str] = None

    @model_validator(mode='after')
    def _check_required_by_type(self) -> 'SourceConfig':
        # 타입별 필수 필드를 확인한다. 누락을 런타임까지 미루지 않는다.
        required: dict[str, list[str]] = {
            'topic': ['topic', 'msg'],
            'tf': ['base_frame', 'tip_frame'],
            'service': ['backend'],
            'static': ['backend'],
            'derived': ['source_variable', 'extract']
        }
        missing = [f for f in required[self.type] if getattr(self, f) is None]
        if missing:
            raise ValueError(f"source.type={self.type!r} requires: {', '.join(missing)}")
        return self


class VariableConfig(_Base):
    """상태 변수 정의 (설계서 4~5장)."""

    name: str
    # 사람이 아니라 **에이전트가 읽는 문자열**이다. `/variables` 카탈로그로 나가며
    # MCP 서버는 이를 그대로 리소스/도구 설명으로 쓴다. 무엇인지뿐 아니라 **언제
    # 읽어야 하는지**까지 적는다 — 비어 있으면 에이전트는 변수 이름만 보고 추측한다.
    description: str = ''
    source: SourceConfig
    # None 이면 staleness 검사를 하지 않는다 — 이벤트성/정적 소스용 (설계서 5.1).
    staleness_ms: Optional[int] = Field(default=None, ge=0)
    # 배열을 이름 map 으로 정규화할지 (설계서 5.5). 값은 **입력 메시지 타입 기준**으로
    # 이름 붙인다 — 변환 방식처럼 이름 지으면 범용으로 오해받는다 (옛 `name_value_map`).
    #   joint_state_map   sensor_msgs/JointState 의 병렬 배열 → {관절이름: 값}
    #   scene_object_map  rdfp_msgs/SceneObjects 의 objects[] → {물체이름: {...}}
    # 추가 시 `variables.PROJECTIONS` 표에도 함께 등록한다.
    projection: Optional[Literal['joint_state_map', 'scene_object_map']] = None
    units: dict[str, str] = Field(default_factory=dict)
    # 정수 enum 상수를 문자열 심볼로 바꾸기 위한 매핑 (설계서 5.5).
    enums: dict[str, dict[int, str]] = Field(default_factory=dict)
    schema_version: int = 1

    @field_validator('name')
    @classmethod
    def _check_name(cls, v: str) -> str:
        # URL 경로 세그먼트이자 배치 조회의 쉼표 구분 토큰으로 쓰인다.
        if not v or not all(c.isalnum() or c in '-_' for c in v):
            raise ValueError(f"variable name must be alphanumeric with '-'/'_' only, got {v!r}")
        return v


class OperationConfig(_Base):
    """연산 정의 (설계서 6장).

    ``kind`` 와 ``resource`` 는 연산별로 고정된 값이며 런타임에 바뀌지 않는다.
    ``idempotent`` 는 카탈로그로 노출되어 클라이언트의 재시도 안전성 판단에 쓰인다
    (설계서 6.11).
    """

    name: str
    # 에이전트의 도구 선택은 이 문자열에 거의 전적으로 의존한다. "무엇을 하는가" 와
    # "언제 부르는가" 를 함께 적고, **완료가 성공을 뜻하지 않는 연산은 그 사실도**
    # 적는다 — 그러지 않으면 에이전트가 거짓 성공 위에 다음 동작을 쌓는다.
    description: str = ''
    kind: OperationKind
    # 단일 자원이거나 목록이다. `reset_scene` 처럼 둘을 함께 잡아야 하는 연산이
    # 있다 — scene 을 바꾸는 동안 팔이 그 공간으로 들어오면 안 되기 때문이다.
    # 읽을 때는 `resource_names` 를 쓴다.
    resource: Optional[Union[ResourceName, list[ResourceName]]] = None
    idempotent: bool = True
    backend: dict[str, Any] = Field(default_factory=dict)
    inputs_schema: dict[str, Any] = Field(default_factory=dict)
    # 비동기 연산의 워치독 기본 시한 (설계서 6.5).
    default_timeout_sec: float = Field(default=60.0, gt=0.0)
    # 동기 연산의 응답 상한 (설계서 6.2).
    sync_timeout_sec: float = Field(default=2.0, gt=0.0)

    @field_validator('name')
    @classmethod
    def _check_name(cls, v: str) -> str:
        if not v or not all(c.isalnum() or c in '-_' for c in v):
            raise ValueError(f"operation name must be alphanumeric with '-'/'_' only, got {v!r}")
        return v

    @property
    def resource_names(self) -> tuple[str, ...]:
        """점유할 자원 이름들. 단일 문자열도 목록도 같은 형태로 돌려준다.

        호출부가 두 형태를 따로 다루면 한쪽을 빠뜨렸을 때 **락이 조용히 안 걸린다.**
        """
        if self.resource is None:
            return ()
        if isinstance(self.resource, str):
            return (self.resource,)
        return tuple(self.resource)

    @model_validator(mode='after')
    def _derive_target_enum(self) -> 'OperationConfig':
        """설정의 이름 표에서 ``inputs_schema`` 의 enum 을 파생한다.

        대상은 두 쌍이다 — ``backend.targets`` → ``target.enum`` (그리퍼 목표),
        ``backend.scenes`` → ``scene.enum`` (scene 레시피).

        목표 이름을 두 곳에 적으면 조용히 어긋난다. 특히 ``targets`` 에만 추가하고
        ``enum`` 을 빠뜨리면 **jsonschema 설치 여부에 따라** 같은 요청이 ``400`` 이
        되기도 하고 성공하기도 한다 — 스키마 검증은 선택적 의존성이라 미설치 환경에서
        통째로 생략되기 때문이다 (설계서 6.9).

        카탈로그(``GET /operations``)는 ``backend`` 를 노출하지 않으므로 ``enum`` 이
        클라이언트가 지원 목표를 발견하는 유일한 경로이기도 하다. 따라서 ``labels``
        를 단일 출처로 삼는다.

        ``enum`` 을 직접 적었다면 지우지 않고 **불일치를 기동 시점에 실패**시킨다.
        입력 dict 는 건드리지 않는다 — 호출자의 자료구조를 바꾸지 않기 위해서다.
        """
        # `labels` 는 리스트, `scenes` 는 dict 다 — 둘 다 키(값)를 enum 으로 옮긴다.
        for backend_key, input_key in (('labels', 'target'), ('scenes', 'scene')):
            self._derive_enum_from(backend_key, input_key)
        return self

    def _derive_enum_from(self, backend_key: str, input_key: str) -> None:
        """``backend[backend_key]`` 의 키를 ``inputs_schema`` 의 enum 으로 옮긴다."""
        table = self.backend.get(backend_key)
        # dict 면 키를, list 면 항목을 이름으로 쓴다.
        if isinstance(table, list):
            table = {str(v): None for v in table}
        if not isinstance(table, dict) or not table:
            return

        properties = self.inputs_schema.get('properties')
        prop = properties.get(input_key) if isinstance(properties, dict) else None
        if not isinstance(prop, dict):
            return

        names = sorted(str(name) for name in table)
        declared = prop.get('enum')
        if declared is not None:
            if not isinstance(declared, list) or sorted(str(v) for v in declared) != names:
                raise ValueError(
                    f"operation '{self.name}': inputs_schema {input_key} enum {declared!r} does "
                    f'not match backend.{backend_key} {names}; omit the enum to derive it'
                )
            return

        self.inputs_schema = {**self.inputs_schema,
                              'properties': {**properties, input_key: {**prop, 'enum': names}}}


class SessionPolicy(_Base):
    """연산 세션의 수명주기 정책 (설계서 6.12)."""

    # 종료 세션 보존 기간. 폴링 주기보다 충분히 길어야 "폴링 중 404 = 재시작"
    # 이라는 추론이 성립한다 (설계서 6.13).
    retention_sec: float = Field(default=900.0, gt=0.0)
    # 종료 세션 개수 상한. 초과분은 LRU 로 축출한다. RUNNING 은 대상이 아니다.
    max_sessions: int = Field(default=200, ge=1)


class TwinConfig(_Base):
    """트윈 정의 전체."""

    twin: TwinMeta
    http: HttpConfig = Field(default_factory=HttpConfig)
    ros: RosConfig = Field(default_factory=RosConfig)
    moveit: MoveItConfig
    variables: list[VariableConfig] = Field(default_factory=list)
    operations: list[OperationConfig] = Field(default_factory=list)
    sessions: SessionPolicy = Field(default_factory=SessionPolicy)

    @model_validator(mode='after')
    def _check_unique_names(self) -> 'TwinConfig':
        for label, items in (('variable', self.variables), ('operation', self.operations)):
            names = [i.name for i in items]
            dupes = sorted({n for n in names if names.count(n) > 1})
            if dupes:
                raise ValueError(f"duplicate {label} name(s): {', '.join(dupes)}")

        # derived 변수가 가리키는 원본이 실재하는지 확인한다.
        known = {v.name for v in self.variables}
        for var in self.variables:
            ref = var.source.source_variable
            if ref is not None and ref not in known:
                raise ValueError(
                    f"variable {var.name!r} derives from unknown variable {ref!r}"
                )
        return self

    @model_validator(mode='after')
    def _check_backend_wiring(self) -> 'TwinConfig':
        """연산 정의가 백엔드를 구동할 수 있는지 확인한다 (설계서 2.6).

        어떤 연산이 무엇을 요구하는지는 핸들러를 가진 ``backends`` 가 안다. 순환
        import 를 피하려고 여기서 지연 import 한다 — ``backends`` 는 이 모듈을
        최상위에서 import 하고, ROS 는 최상위에서 import 하지 않으므로 이 검사에
        ROS 의존성이 생기지 않는다.
        """
        from robot_twin.backends import validate_operation_config

        for op in self.operations:
            validate_operation_config(op)
        return self

    def variable(self, name: str) -> Optional[VariableConfig]:
        """이름으로 변수 정의를 찾는다. 없으면 ``None``."""
        return next((v for v in self.variables if v.name == name), None)

    def operation(self, name: str) -> Optional[OperationConfig]:
        """이름으로 연산 정의를 찾는다. 없으면 ``None``."""
        return next((o for o in self.operations if o.name == name), None)


def load_config(path: str | Path) -> TwinConfig:
    """YAML 파일을 읽어 :class:`TwinConfig` 로 검증한다.

    Args:
        path: 트윈 정의 YAML 경로.

    Returns:
        검증된 설정 객체.

    Raises:
        FileNotFoundError: 파일이 없을 때.
        ValueError: YAML 이 매핑이 아니거나 스키마 검증에 실패했을 때.
    """
    p = Path(path)
    if not p.is_file():
        raise FileNotFoundError(f'twin config not found: {p}')

    with p.open('r', encoding='utf-8') as f:
        raw = yaml.safe_load(f)

    if not isinstance(raw, dict):
        raise ValueError(f'twin config must be a YAML mapping: {p}')

    return TwinConfig.model_validate(raw)


__all__ = [
    'HttpConfig', 'MoveItConfig', 'OperationConfig', 'QosConfig', 'RosConfig',
    'SessionPolicy', 'SourceConfig', 'TwinConfig', 'TwinMeta', 'VariableConfig',
    'load_config'
]
