"""ROS 메시지 → JSON 직렬화 규약 (설계서 5.5).

언어 중립 인터페이스이므로 표현을 고정한다. 특히 다음이 실제 문제를 일으킨다.

- ``float`` 의 ``NaN`` / ``Inf`` — JSON 에 해당 리터럴이 없다 → ``null``
- ``int64`` / ``uint64`` — JS ``Number`` 정밀도 손실 → 문자열
- 병렬 배열(``name[]`` + ``position[]``) — 인덱스 순서 의존은 컨트롤러 설정이
  바뀌면 **조용히** 틀린다 → ``{name: value}`` map

단위 변환은 하지 않는다. ROS 원본 단위를 그대로 노출하고 변수 메타로 명시만 한다.

ROS 메시지 클래스를 import 하지 않고 덕 타이핑(``get_fields_and_field_types``)으로
순회하므로, ROS 를 source 하지 않아도 이 모듈은 import 된다.
"""

from __future__ import annotations

from typing import Any, Optional

import math

# JS Number 로 안전하게 표현 가능한 정수 범위. 이를 넘는 정수는 문자열로 낸다.
_JS_SAFE_INT_MAX = 2 ** 53 - 1

# ROS 메시지의 필드 메타데이터 접근자. 모든 rosidl 파이썬 메시지가 제공한다.
_FIELD_META = 'get_fields_and_field_types'


class SerializationError(Exception):
    """메시지를 JSON 표현으로 바꾸지 못했을 때 발생한다."""


def is_ros_message(obj: Any) -> bool:
    """rosidl 이 생성한 메시지 객체인지 판별한다."""
    return hasattr(obj, _FIELD_META)


def _scalar(value: Any) -> Any:
    """스칼라 값을 JSON 안전한 형태로 변환한다."""
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        # int64/uint64 가 JS 정밀도를 넘으면 문자열로 낸다.
        return str(value) if abs(value) > _JS_SAFE_INT_MAX else value
    if isinstance(value, float):
        # NaN / Inf 는 JSON 에 리터럴이 없다.
        return None if math.isnan(value) or math.isinf(value) else value
    if isinstance(value, (bytes, bytearray)):
        # 바이트 배열은 상태 변수로 노출하지 않는다는 방침이지만, 중첩 필드로
        # 딸려오는 경우를 대비해 길이만 남긴다.
        return {'__bytes__': len(value)}
    return value


def to_jsonable(obj: Any) -> Any:
    """ROS 메시지 / 배열 / 스칼라를 JSON 직렬화 가능한 값으로 변환한다.

    Args:
        obj: 변환 대상.

    Returns:
        dict / list / 스칼라로 이루어진 JSON 안전한 값.
    """
    if is_ros_message(obj):
        fields = getattr(obj, _FIELD_META)()
        return {name: to_jsonable(getattr(obj, name)) for name in fields}

    if isinstance(obj, (list, tuple)):
        return [to_jsonable(v) for v in obj]

    if isinstance(obj, dict):
        return {str(k): to_jsonable(v) for k, v in obj.items()}

    # numpy array 등 sequence 프로토콜만 갖춘 타입 (rosidl 이 float[] 에 사용).
    if hasattr(obj, 'tolist'):
        return [to_jsonable(v) for v in obj.tolist()]

    return _scalar(obj)


def extract_stamp(msg: Any) -> Optional[dict[str, int]]:
    """메시지의 ``header.stamp`` 를 원본 형태로 뽑는다.

    ISO8601 로 변환하지 않는다 — 정밀도가 손실되고, ``use_sim_time`` 환경에서는
    벽시계와 의미가 다르기 때문이다 (설계서 4.2).

    Args:
        msg: ROS 메시지.

    Returns:
        ``{'sec': int, 'nanosec': int}``. 헤더가 없는 타입이면 ``None``.
    """
    header = getattr(msg, 'header', None)
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return None
    return {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)}


# JointState 의 병렬 배열 필드. 이 셋만 map 으로 변환된다.
_JOINT_STATE_ARRAY_FIELDS = ('position', 'velocity', 'effort')


def project_joint_state_map(value: dict[str, Any]) -> dict[str, Any]:
    """``sensor_msgs/JointState`` 를 ``{name: value}`` map 으로 정규화한다 (설계서 5.5).

    **JointState 전용이며 범용 배열→map 변환이 아니다.** 최상위 ``name`` 리스트와
    ``position``/``velocity``/``effort`` 라는 **필드 이름**에 묶여 있다. 각 원소가
    자기 이름을 갖는 배열-of-구조체(예: ``SceneObjects``)에는 쓸 수 없다.

    Args:
        value: :func:`to_jsonable` 을 거친 메시지 dict.

    Returns:
        각 병렬 배열이 map 으로 바뀐 dict. ``name`` 키 자체는 제거한다.

    Raises:
        SerializationError: ``name`` 이 없거나·비었거나·리스트가 아닐 때, 배열 길이가
            맞지 않을 때, 또는 변환 대상이 아닌 병렬 배열이 함께 있을 때. 재생
            경로에서 ``name`` 이 비어 오는 사례가 실재하므로 조용히 넘기지 않는다
            (설계서 5.7).
    """
    names = value.get('name')
    if not names:
        raise SerializationError('JointState-like message has empty "name"; cannot build map')
    if not isinstance(names, list):
        # 스칼라 ``name`` 을 가진 메시지도 truthy 라 여기까지 온다. 그대로 두면
        # 매핑할 배열이 없어 ``name`` 필드만 지워진 채 반환된다.
        raise SerializationError(
            f'"name" must be a list to build a map, got {type(names).__name__}'
        )

    # 화이트리스트 밖의 병렬 배열을 그냥 흘려보내면, 그 배열은 map 이 되지 못한 채
    # ``name`` 만 사라져 **대응 관계를 복원할 수 없다.** JointState 가 아닌 메시지에
    # 이 projection 을 붙인 설정 오류이므로 조용한 손실 대신 명시적으로 실패시킨다.
    unmapped = sorted(
        key for key, item in value.items()
        if key != 'name' and key not in _JOINT_STATE_ARRAY_FIELDS
        and isinstance(item, list) and len(item) == len(names)
    )
    if unmapped:
        raise SerializationError(
            f'field(s) {unmapped} look like parallel arrays but are not mapped; '
            'this projection handles sensor_msgs/JointState only'
        )

    out: dict[str, Any] = {k: v for k, v in value.items() if k not in ('name',)}
    for key in _JOINT_STATE_ARRAY_FIELDS:
        arr = value.get(key)
        if not isinstance(arr, list) or not arr:
            out.pop(key, None)
            continue
        if len(arr) != len(names):
            raise SerializationError(
                f'field "{key}" has {len(arr)} entries but "name" has {len(names)}'
            )
        out[key] = dict(zip(names, arr))
    return out


def project_scene_object_map(value: dict[str, Any]) -> dict[str, Any]:
    """``rdfp_msgs/SceneObjects`` 의 ``objects[]`` 를 ``{name: {...}}`` 로 정규화한다.

    `project_joint_state_map` 과 성격이 다르다. 저쪽은 병렬 배열이라 인덱스가 밀리면
    값이 **뒤섞이는** 정확성 문제였지만, 여기는 각 원소가 자기 ``name`` 을 들고 있어
    순서가 바뀌어도 정보를 잃지 않는다 — 클라이언트가 인덱스 대신 이름으로 접근하게
    하는 **편의**가 목적이다. 그럼에도 처음부터 적용하는 이유는 배열 → map 이
    breaking change 라서, 클라이언트가 붙기 전이 유일하게 공짜인 시점이기 때문이다.

    Args:
        value: :func:`to_jsonable` 을 거친 메시지 dict.

    Returns:
        ``objects`` 가 map 으로 바뀐 dict. 각 원소의 ``name`` 키는 제거한다
        (map 의 키로 이미 들어가 있다 — `joint_states` 와 같은 처리).

    Raises:
        SerializationError: ``objects`` 가 리스트가 아니거나, 원소가 dict 가 아니거나,
            ``name`` 이 없거나 비었거나, 이름이 중복될 때.
    """
    return _project_struct_array(value, array_field='objects', key_field='name')


def _project_struct_array(value: dict[str, Any], *, array_field: str,
                          key_field: str,) -> dict[str, Any]:
    """배열-of-구조체를 ``{키: 나머지필드}`` map 으로 바꾼다.

    설정 어휘는 메시지 타입별로 따로 두지만(그래야 이름이 적용 범위를 정직하게
    드러낸다), 구현까지 복제할 이유는 없어 여기로 모은다.
    """
    items = value.get(array_field)
    if not isinstance(items, list):
        raise SerializationError(
            f'field {array_field!r} must be a list to build a map, '
            f'got {type(items).__name__}'
        )

    mapped: dict[str, Any] = {}
    for index, item in enumerate(items):
        if not isinstance(item, dict):
            raise SerializationError(
                f'{array_field}[{index}] must be an object, got {type(item).__name__}'
            )
        key = item.get(key_field)
        if not key or not isinstance(key, str):
            raise SerializationError(
                f'{array_field}[{index}] has an empty or non-string {key_field!r}; '
                'cannot build map'
            )
        if key in mapped:
            # 조용히 덮어쓰면 물체 하나가 데이터에서 사라진다. 이름은 백엔드 발행
            # 노드가 정하므로, 중복은 그쪽 버그이며 드러나야 한다.
            raise SerializationError(f'duplicate {key_field} {key!r} in {array_field}')
        mapped[key] = {k: v for k, v in item.items() if k != key_field}

    out = dict(value)
    out[array_field] = mapped
    return out


def apply_enums(value: Any, enums: dict[str, dict[int, str]]) -> Any:
    """정수 enum 필드를 문자열 심볼로 바꾼다 (설계서 5.5).

    클라이언트가 매직넘버를 하드코딩하는 것을 막는다. 원본 숫자는
    ``<field>_value`` 로 함께 남긴다.

    Args:
        value: :func:`to_jsonable` 결과.
        enums: ``{필드명: {정수: 심볼}}``.

    Returns:
        변환된 값. ``value`` 가 dict 가 아니면 그대로 반환한다.
    """
    if not enums or not isinstance(value, dict):
        return value

    out = dict(value)
    for field, mapping in enums.items():
        if field not in out:
            continue
        raw = out[field]
        if isinstance(raw, int) and raw in mapping:
            out[field] = mapping[raw]
            out[f'{field}_value'] = raw
    return out


__all__ = [
    'SerializationError', 'apply_enums', 'extract_stamp', 'is_ros_message',
    'project_joint_state_map', 'project_scene_object_map', 'to_jsonable'
]
