from __future__ import annotations

from typing import Any, Callable, TypeVar

import threading
import time

import rclpy
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.node import Node

T = TypeVar("T")

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=1
)
SYSTEM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1
)

def parse_str(value: Any, name: str) -> str:
    """값을 문자열로 변환한다."""
    try:
        return str(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name!r} must be a string") from exc
    
    
def parse_stripped_str(value: Any, name: str) -> str:
    """값을 공백이 아닌 문자열로 변환한다."""
    s = parse_str(value, name).strip()
    if not s:
        raise ValueError(f"{name!r} cannot be empty or whitespace")
    return s


def parse_int(value: Any, name: str) -> int:
    """값을 정수로 변환한다."""
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name!r} must be an integer") from exc


def parse_float(value: Any, name: str) -> float:
    """값을 실수로 변환한다."""
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name!r} must be a number: {value}") from exc
    

def parse_bool(value: Any, name: str) -> bool:
    """값을 불리언으로 변환한다."""
    if isinstance(value, bool):
        return value
    raise ValueError(f"{name!r} must be a boolean (true/false), got {value!r})")


def parse_str_list(value: Any, name: str) -> list[str]:
    """값을 문자열 리스트로 변환한다."""
    if isinstance(value, list):
        try:
            return [str(v) for v in value]
        except (TypeError, ValueError) as exc:
            raise ValueError(f"All elements of {name!r} must be strings") from exc
    raise ValueError(f"{name!r} must be a list of strings")


def parse_topic_name(topic_name: Any, name: str) -> str:
    """topic_name 파라미터를 검증한다."""
    if not topic_name or not topic_name.strip():
        raise ValueError(f"{name!r} parameter is required and cannot be empty")

    topic_name = topic_name.strip()
    if not topic_name.startswith('/'):
        topic_name = '/' + topic_name

    return topic_name


def log_periodic(
    log_fn: Callable[[str], Any],
    message: str,
    last_timestamp: float,
    interval_sec: float,
) -> float:
    """일정 간격으로만 로그를 기록한다.

    Args:
        log_fn: 로그를 남길 callable (예: ``logger.warning``,
            ``node.get_logger().error``).
        message: 로그 메시지.
        last_timestamp: 마지막으로 로그를 남긴 ``time.monotonic()`` 값.
            최초 호출 시 ``0.0`` 을 전달하면 즉시 기록된다.
        interval_sec: 로그 기록 최소 간격(초).

    Returns:
        갱신된 타임스탬프. 호출자가 다음 호출에 그대로 전달해야 한다.
    """
    now = time.monotonic()
    if now - last_timestamp >= interval_sec:
        log_fn(message)
        return now
    return last_timestamp


def get_parameter(
    node: Node,
    param_name: str,
    validator: Callable[[Any, str], T],
    default: T | None = None,
) -> T:
    """노드에서 파라미터 값을 가져와 검증/변환한다."""
    param = node.get_parameter(param_name)
    raw = param.value if param is not None else None
    if raw is None:
        if default is not None:
            return default
        raise ValueError(f"{param_name!r} parameter is required")
    return validator(raw, param_name)


def get_optional_parameter(
    node: Node,
    param_name: str,
    validator: Callable[[Any, str], T],
    default: T | None = None,
) -> T | None:
    """노드에서 선택적 파라미터 값을 가져와 검증/변환한다."""
    param = node.get_parameter(param_name)
    value = param.value if param is not None else default
    return validator(value, param_name) if value is not None else default


def await_future_spin(
    node: Node,
    future: rclpy.Future,
    timeout_sec: float,
    what: str,
) -> Any:
    """안전한 future 대기 및 예외 처리.

    Args:
        node: spin 및 로깅에 사용할 rclpy Node.
        future: 대기할 Future 객체.
        timeout_sec: 최대 대기 시간(초).
        what: 로그에 표시할 작업 설명 문자열.

    Returns:
        성공 시 ``future.result()``.

    Raises:
        TimeoutError: 주어진 시간 내에 future 가 완료되지 않았을 때.
    """
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        node.get_logger().error(f'Timeout while waiting for {what}')
        raise TimeoutError(f'Timed out waiting for {what}: timeout={timeout_sec}s')
    exception = future.exception()
    if exception is not None:
        node.get_logger().error(f'{what} failed with exception: {exception}')
        return None
    return future.result()


def await_future_spin_nospin(
    node: Node,
    future: rclpy.Future,
    timeout_sec: float,
    what: str,
) -> Any:
    """콜백 기반 future 대기 및 예외 처리.

    ``await_future_spin`` 와 달리 Node 를 직접 spin 하지 않는다. 호출 전에
    다른 스레드에서 Node 가 spin 중이어야 콜백이 발화된다.
    ``MultiThreadedExecutor`` 환경에서 이중 spin 없이 사용할 수 있다.

    Args:
        node: 로깅에 사용할 rclpy Node.
        future: 대기할 Future 객체.
        timeout_sec: 최대 대기 시간(초).
        what: 로그에 표시할 작업 설명 문자열.

    Returns:
        성공 시 ``future.result()``.

    Raises:
        TimeoutError: 주어진 시간 내에 future 가 완료되지 않았을 때.
    """
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())

    if not event.wait(timeout=timeout_sec):
        node.get_logger().error(f'Timeout while waiting for {what}')
        raise TimeoutError(f'Timed out waiting for {what}: timeout={timeout_sec}s')
    exception = future.exception()
    if exception is not None:
        node.get_logger().error(f'{what} failed with exception: {exception}')
        return None
    return future.result()


def check_timeout_and_get_remains(start_time: float, timeout: float) -> float:
    """경과 시간을 확인하고 남은 시간을 반환한다.

    Args:
        start_time: ``time.monotonic()`` 으로 기록한 시작 시각.
        timeout: 허용된 총 시간(초).

    Returns:
        남은 시간(초).

    Raises:
        TimeoutError: 이미 시간이 초과되었을 때.
    """
    remains = timeout - (time.monotonic() - start_time)
    if remains <= 0:
        raise TimeoutError(f'Operation timed out (timeout={timeout:.3f}s)')
    return remains