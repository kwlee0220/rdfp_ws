"""상태 변수 캐시 — 구독 등록과 스냅샷 보관 (설계서 2.2, 4장).

ROS executor 스레드의 구독 콜백이 스냅샷을 만들어 참조 하나를 교체하고, HTTP
스레드가 그 참조를 읽는다. 단일 변수 조회에는 락이 필요 없다 — CPython 에서 속성
참조 대입은 GIL 하에서 원자적이므로 읽는 쪽은 항상 이전 또는 새 스냅샷을 얻고
찢어진 중간 상태를 볼 수 없다.

배치 조회만 락으로 감싼다. 다만 이 락은 **읽는 순간에 여러 변수가 서로 뒤섞이지
않게** 할 뿐이며, 각 값의 생성 시각을 맞춰 주지 않는다 (설계서 5.4).

본 모듈은 rclpy 를 import 하므로 ROS 를 source 해야 한다.
"""

from __future__ import annotations

from typing import Any, Callable, Optional

import threading
import time

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from rdfp.twin.config import QosConfig, VariableConfig
from rdfp.twin.serialize import (
    SerializationError, apply_enums, extract_stamp, project_joint_state_map,
    project_scene_object_map, to_jsonable
)
from rdfp.twin.snapshot import (
    GenerationCounter, Quality, Reason, Snapshot, VariableStatus, judge
)

# 정적 소스 조회가 실패한 뒤 다시 시도하기까지의 최소 간격(초). 백엔드가 죽어 있는
# 동안 매 조회마다 `timeout_sec` 만큼 블로킹되는 것을 막는다 — 폴링 클라이언트가
# 몇 Hz 로 때리든 실제 백엔드 호출은 이 간격당 한 번이다.
STATIC_RETRY_SEC = 5.0

# `VariableConfig.projection` 값 → 변환 함수.
#
# 이름을 **입력 메시지 타입 기준**으로 붙인다 (`joint_state_map` / `scene_object_map`).
# 이전 이름 `name_value_map` 은 변환 방식처럼 들려 범용으로 오해받았고, 실제로
# `SceneObjects` 에 그대로 쓰려다 걸렸다. 적용 범위를 이름이 드러내게 한 것이다.
#
# projection 을 추가할 때는 이 표에 한 줄, `config.py` 의 Literal 에 값 하나를 더한다.
PROJECTIONS = {
    'joint_state_map': project_joint_state_map,
    'scene_object_map': project_scene_object_map,
}


class StaticSourceUnavailable(RuntimeError):
    """정적 소스의 백엔드를 아직 쓸 수 없을 때 resolver 가 던진다.

    일시적인 상황(백엔드 클라이언트 미준비)임을 뜻하며, 캐시는 이를
    ``SOURCE_NODE_DOWN`` 으로 보고하고 나중에 다시 시도한다.
    """


def build_qos(cfg: QosConfig) -> QoSProfile:
    """설정을 rclpy QoS 프로파일로 변환한다.

    TRANSIENT_LOCAL 토픽을 VOLATILE 로 구독하면 아무것도 받지 못하므로, QoS 는
    변수 정의에 명시적으로 두고 여기서 그대로 반영한다.
    """
    return QoSProfile(
        depth=cfg.depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=(ReliabilityPolicy.RELIABLE if cfg.reliability == 'reliable'
                     else ReliabilityPolicy.BEST_EFFORT),
        durability=(DurabilityPolicy.TRANSIENT_LOCAL if cfg.durability == 'transient_local'
                    else DurabilityPolicy.VOLATILE)
    )


def import_message_type(type_name: str) -> type:
    """``sensor_msgs/msg/JointState`` 형식의 이름을 메시지 클래스로 해석한다.

    Args:
        type_name: ``<pkg>/msg/<Type>`` 또는 ``<pkg>/<Type>``.

    Returns:
        메시지 클래스.

    Raises:
        ValueError: 형식이 잘못되었거나 import 에 실패했을 때.
    """
    parts = type_name.split('/')
    if len(parts) == 3:
        pkg, sub, name = parts
    elif len(parts) == 2:
        pkg, sub, name = parts[0], 'msg', parts[1]
    else:
        raise ValueError(f'invalid message type name: {type_name!r}')

    try:
        module = __import__(f'{pkg}.{sub}', fromlist=[name])
        return getattr(module, name)
    except (ImportError, AttributeError) as exc:
        raise ValueError(f'cannot import message type {type_name!r}: {exc}') from exc


class VariableEntry:
    """변수 하나의 캐시 슬롯.

    ``snapshot`` 속성은 ROS 콜백이 통째로 교체한다. 기존 객체를 제자리에서
    수정하지 않는 것이 락을 생략할 수 있는 근거다.
    """

    def __init__(self, config: VariableConfig) -> None:
        self.config = config
        self.snapshot: Optional[Snapshot] = None
        # 소스 가용성. 그래프 폴링이 주기적으로 갱신한다.
        self.source_available: bool = True
        self.unavailable_reason: Optional[Reason] = None
        # 직렬화 메모이즈: (세대 번호, 변환 결과). 폴링 클라이언트가 N명이어도
        # 세대당 1회만 변환한다 (설계서 8.1).
        self._memo: tuple[int, Any] = (-1, None)
        self._memo_lock = threading.Lock()
        # 정적 소스 전용. `fetch_lock` 은 동시 조회를 하나로 합치고(single-flight),
        # `retry_after` 는 실패 후 재시도 가능 시각을 담는다. 두 값 모두
        # `VariableCache` 가 다룬다.
        self.fetch_lock = threading.Lock()
        self.retry_after: float = 0.0

    @property
    def etag(self) -> str:
        """조건부 요청용 ETag — 스냅샷 세대 번호를 그대로 쓴다."""
        snap = self.snapshot
        return f'"{snap.gen if snap else 0}"'

    def status(self) -> VariableStatus:
        """현재 품질을 판정한다."""
        return judge(
            self.snapshot, staleness_ms=self.config.staleness_ms,
            source_available=self.source_available, reason=self.unavailable_reason
        )

    def value(self) -> Any:
        """스냅샷을 JSON 표현으로 변환한다 (세대 단위 메모이즈).

        변환은 조회 시점에 지연 수행한다 — 50 Hz 로 아무도 보지 않는 값을 미리
        변환하면 GIL 을 낭비한다.

        Returns:
            JSON 안전한 값. 스냅샷이 없으면 ``None``.

        Raises:
            SerializationError: 변환/추출에 실패했을 때.
        """
        snap = self.snapshot
        if snap is None:
            return None

        with self._memo_lock:
            gen, cached = self._memo
            if gen == snap.gen:
                return cached

        value = to_jsonable(snap.msg)
        projection = self.config.projection
        if projection is not None and isinstance(value, dict):
            value = PROJECTIONS[projection](value)
        value = apply_enums(value, self.config.enums)

        with self._memo_lock:
            self._memo = (snap.gen, value)
        return value

    def stamp(self) -> Optional[dict[str, int]]:
        """원본 ``header.stamp``. 헤더가 없으면 ``None``."""
        snap = self.snapshot
        return extract_stamp(snap.msg) if snap is not None else None


class VariableCache:
    """트윈의 모든 상태 변수를 담는 캐시.

    구독 등록은 ROS 노드가 생성된 직후(기동 2단계)에 이루어지며, MoveGroup
    클라이언트 준비 여부와 무관하게 동작한다 (설계서 2.6).
    """

    def __init__(self, node: Node, configs: list[VariableConfig], *,
                 clock: Callable[[], float] = time.time,
                 static_resolver: Optional[Callable[[str, float], Any]] = None) -> None:
        """캐시를 만든다.

        Args:
            node: 구독을 등록할 ROS 노드.
            configs: 변수 정의 목록.
            clock: wall clock. 테스트에서 주입한다.
            static_resolver: ``source.type: static`` 의 백엔드 호출자.
                ``(method_name, timeout_sec) -> value`` 이며, 백엔드가 아직
                준비되지 않았으면 :class:`StaticSourceUnavailable` 을 던진다.
                캐시가 런타임을 직접 참조하면 순환 import 가 되므로 호출 가능
                객체 하나만 주입받는다. ``None`` 이면 정적 변수는 값을 갖지 못한다.
        """
        self._node = node
        self._clock = clock
        self._gen = GenerationCounter()
        self._entries: dict[str, VariableEntry] = {c.name: VariableEntry(c) for c in configs}
        self._batch_lock = threading.Lock()
        self._subscriptions: list[Any] = []
        self._logger = node.get_logger()
        self._static_resolver = static_resolver

    # ------------------------------------------------------------------
    # 구독 등록
    # ------------------------------------------------------------------

    def start(self) -> None:
        """토픽 구독을 등록한다.

        ``static`` 소스는 여기서 조회하지 않는다 — 트윈이 백엔드(``move_group``)
        보다 먼저 뜨면 실패하기 때문이다. 최초 조회 요청에서 lazy 하게 가져온다
        (설계서 4.3).

        나머지 소스(tf / service / derived)는 아직 구현 대상이 아니며, 조회 시
        ``NO_DATA`` 로 보고된다.
        """
        for entry in self._entries.values():
            src = entry.config.source
            if src.type == 'static':
                method = (src.backend or {}).get('method')
                self._logger.info(
                    f"variable '{entry.config.name}': static source "
                    f"({method}) will be fetched lazily on first read"
                )
                continue

            if src.type != 'topic':
                self._logger.info(
                    f"variable '{entry.config.name}': source type '{src.type}' "
                    'is not wired yet; it will report NO_DATA'
                )
                continue

            try:
                msg_type = import_message_type(src.msg)
            except ValueError as exc:
                self._logger.error(f"variable '{entry.config.name}': {exc}")
                entry.source_available = False
                entry.unavailable_reason = Reason.NO_PUBLISHER
                continue

            sub = self._node.create_subscription(
                msg_type, src.topic, self._make_callback(entry), build_qos(src.qos)
            )
            self._subscriptions.append(sub)
            self._logger.info(
                f"variable '{entry.config.name}' <- {src.topic} ({src.msg}, "
                f'{src.qos.reliability}/{src.qos.durability})'
            )

    def _make_callback(self, entry: VariableEntry) -> Callable[[Any], None]:
        """구독 콜백을 만든다 — 스냅샷 생성 후 참조 하나만 교체한다."""

        def _on_message(msg: Any) -> None:
            # 수신 시각은 반드시 여기서(콜백에서) 찍는다. HTTP 스레드에서 계산하면
            # 응답 지연이 age 에 섞인다. 또한 wall clock 을 쓴다 — msg.header.stamp
            # 는 use_sim_time 환경에서 다른 시계다 (설계서 4.2).
            entry.snapshot = Snapshot(msg=msg, received_at=self._clock(), gen=self._gen.next())

        return _on_message

    # ------------------------------------------------------------------
    # 가용성 갱신
    # ------------------------------------------------------------------

    def refresh_availability(self) -> None:
        """퍼블리셔 존재 여부를 갱신한다 (설계서 5.1).

        퍼블리셔가 없는 것과 퍼블리셔가 멈춘 것은 age 만 보면 구분되지 않으므로
        그래프를 직접 조회해야 한다. 타이머로 주기 호출한다.
        """
        for entry in self._entries.values():
            src = entry.config.source
            if src.type != 'topic':
                continue
            try:
                count = self._node.count_publishers(src.topic)
            except Exception:  # 그래프 조회 실패는 판정 불가로 두고 기존 값을 유지한다
                continue
            entry.source_available = count > 0
            entry.unavailable_reason = None if count > 0 else Reason.NO_PUBLISHER

    # ------------------------------------------------------------------
    # 정적 소스 — lazy 조회 + 캐시
    # ------------------------------------------------------------------

    def ensure_static(self, entry: VariableEntry, *, force: bool = False) -> None:
        """정적 소스를 lazy 하게 채운다 (설계서 4.3).

        **배치 락 밖에서 호출해야 한다.** 백엔드 호출은 최대 ``timeout_sec`` 동안
        블로킹되므로 락 안에서 부르면 무관한 변수의 조회까지 함께 멈춘다.

        성공한 값은 무기한 캐시한다 — 자동 무효화는 하지 않으며 ``force`` 또는
        트윈 재시작만이 갱신 수단이다. 조회 실패는 캐시하지 않고 최소
        :data:`STATIC_RETRY_SEC` 뒤에 다시 시도한다. 백엔드가 트윈보다 늦게 떠도
        재시작 없이 회복되어야 하기 때문이다.

        Args:
            entry: 대상 슬롯. 정적 소스가 아니면 아무 일도 하지 않는다.
            force: 캐시와 재시도 간격을 모두 무시하고 다시 조회한다.
        """
        if entry.config.source.type != 'static':
            return
        if entry.snapshot is not None and not force:
            return
        if not force and self._clock() < entry.retry_after:
            return

        with entry.fetch_lock:
            # 락을 기다리는 동안 다른 스레드가 이미 채웠을 수 있다. 이 재검사가
            # 없으면 동시에 첫 조회를 요청한 클라이언트 수만큼 백엔드 호출이 나간다.
            if entry.snapshot is not None and not force:
                return

            source = entry.config.source
            method = (source.backend or {}).get('method')
            if not isinstance(method, str) or not method:
                # 설정 오류다. 런타임에 저절로 고쳐지지 않으므로 재시도 대상이
                # 아니라 ERROR 로 고정한다.
                self._set_static_error(
                    entry, f"static source requires backend.method, got {method!r}"
                )
                return

            if self._static_resolver is None:
                self._mark_static_unavailable(
                    entry, Reason.SOURCE_NODE_DOWN, 'no static resolver is wired'
                )
                return

            try:
                value = self._static_resolver(method, source.timeout_sec)
            except StaticSourceUnavailable as exc:
                self._mark_static_unavailable(entry, Reason.SOURCE_NODE_DOWN, exc)
                return
            except Exception as exc:  # 백엔드 호출 실패 — 타임아웃 포함
                self._mark_static_unavailable(entry, Reason.SERVICE_UNAVAILABLE, exc)
                return

            entry.source_available = True
            entry.unavailable_reason = None
            entry.retry_after = 0.0
            entry.snapshot = Snapshot(msg=value, received_at=self._clock(), gen=self._gen.next())
            self._logger.info(
                f"variable '{entry.config.name}': fetched from backend {method}()"
            )

    def _mark_static_unavailable(self, entry: VariableEntry, reason: Reason,
                                 detail: Any) -> None:
        """정적 소스 조회 실패를 기록한다.

        **이미 캐시된 값이 있으면 품질을 떨어뜨리지 않는다.** 강제 갱신이 실패했다고
        멀쩡히 들고 있던 SRDF 값을 ``SOURCE_UNAVAILABLE`` 로 가리는 것은, 값이
        런타임에 변하지 않는다는 정적 소스의 전제와 어긋난다.
        """
        if entry.snapshot is None:
            entry.source_available = False
            entry.unavailable_reason = reason
        entry.retry_after = self._clock() + STATIC_RETRY_SEC
        self._logger.warning(f"variable '{entry.config.name}': static fetch failed: {detail}")

    def _set_static_error(self, entry: VariableEntry, message: str) -> None:
        """설정 오류를 ``ERROR`` 품질로 고정한다 (재시도하지 않는다)."""
        entry.snapshot = Snapshot(msg=None, received_at=self._clock(),
                                  gen=self._gen.next(), error=message)
        self._logger.error(f"variable '{entry.config.name}': {message}")

    # ------------------------------------------------------------------
    # 조회
    # ------------------------------------------------------------------

    def names(self) -> list[str]:
        """정의된 변수 이름 목록 (정의 순서를 유지한다)."""
        return list(self._entries)

    def entry(self, name: str) -> Optional[VariableEntry]:
        """변수 슬롯을 반환한다. 정의되지 않은 이름이면 ``None``."""
        return self._entries.get(name)

    def read(self, name: str, *, refresh: bool = False) -> Optional[dict[str, Any]]:
        """단일 변수의 조회 응답 본문을 만든다 (설계서 5.2).

        Args:
            name: 변수 이름.
            refresh: 정적 소스의 캐시를 강제로 다시 채운다. 다른 소스에는 영향이
                없다.

        Returns:
            envelope dict. 정의되지 않은 변수면 ``None``.
        """
        entry = self._entries.get(name)
        if entry is None:
            return None
        self.ensure_static(entry, force=refresh)
        return self._envelope(entry)

    def read_many(self, names: list[str]) -> dict[str, Any]:
        """여러 변수를 한 번에 읽는다 (설계서 5.4).

        락은 읽는 순간 여러 변수가 뒤섞이지 않게 할 뿐, 각 값의 **생성 시각을
        맞춰 주지 않는다.** 정합이 필요한 클라이언트는 각 ``stamp`` 를 비교한다.
        """
        # 정적 소스의 lazy 조회는 반드시 배치 락 밖에서 끝낸다 — 블로킹 백엔드
        # 호출을 락 안에 두면 무관한 변수의 배치 조회까지 함께 멈춘다.
        for name in names:
            entry = self._entries.get(name)
            if entry is not None:
                self.ensure_static(entry)

        with self._batch_lock:
            return {n: (self._envelope(self._entries[n]) if n in self._entries else None)
                    for n in names}

    def _envelope(self, entry: VariableEntry) -> dict[str, Any]:
        """조회 응답 envelope 를 구성한다.

        값만 벗겨 주는 옵션은 제공하지 않는다 — 클라이언트가 staleness 검사를
        건너뛰게 만드는 지름길이기 때문이다.
        """
        status = entry.status()
        body: dict[str, Any] = {
            'name': entry.config.name,
            'quality': status.quality.value,
            'schema_version': entry.config.schema_version
        }
        if status.reason is not None:
            body['reason'] = status.reason.value
        if status.age_ms is not None:
            body['age_ms'] = status.age_ms
        if status.error is not None:
            # 스냅샷에 실린 오류(정적 소스의 설정 오류 등). 이유를 빼고 quality 만
            # ERROR 로 내보내면 클라이언트가 원인을 알 방법이 없다.
            body['error'] = {'code': 'SOURCE_ERROR', 'message': status.error}

        snap = entry.snapshot
        if snap is not None:
            body['stamp'] = entry.stamp()
            body['received_at'] = _iso_utc(snap.received_at)

        if status.quality in (Quality.NO_DATA, Quality.SOURCE_UNAVAILABLE):
            body['value'] = None
            return body

        try:
            body['value'] = entry.value()
        except SerializationError as exc:
            # 변환 실패는 500 이 아니라 quality=ERROR 다 — 배치 조회에서 나머지
            # 변수는 살아야 한다 (설계서 5.3).
            body['quality'] = Quality.ERROR.value
            body['value'] = None
            body['error'] = {'code': 'SERIALIZATION_FAILED', 'message': str(exc)}
        return body


def _iso_utc(epoch_sec: float) -> str:
    """epoch 초를 ISO8601 UTC 문자열로 바꾼다 (트윈이 찍는 시각 전용)."""
    from datetime import datetime, timezone
    return datetime.fromtimestamp(epoch_sec, tz=timezone.utc).isoformat().replace('+00:00', 'Z')


__all__ = [
    'STATIC_RETRY_SEC', 'StaticSourceUnavailable', 'VariableCache', 'VariableEntry',
    'build_qos', 'import_message_type'
]
