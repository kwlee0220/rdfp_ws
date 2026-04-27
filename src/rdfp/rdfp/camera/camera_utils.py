from __future__ import annotations

from typing import Any

from urllib.parse import urlsplit, urlunsplit

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from rdfp.ros2_utils import get_parameter


def parse_camera_id(camera_id:Any, name:str) -> int | str:
    """camera_id 값을 검증하고 적절히 변환한다.

    Args:
        camera_id: 카메라 ID (int 또는 str)
        name: 오류 메시지에 사용할 파라미터 이름

    Returns:
        int | str: 검증된 카메라 ID

    Raises:
        ValueError: 유효하지 않은 카메라 ID인 경우
    """

    if isinstance(camera_id, int):
        if camera_id < 0:
            raise ValueError(f"{name} cannot be negative: {camera_id}")
        return camera_id
    elif isinstance(camera_id, str):
        normalized_camera_id = camera_id.strip()
        if not normalized_camera_id:
            raise ValueError(f"{name} string cannot be empty")
        # 숫자 문자열은 int로 변환해 처리한다.
        if normalized_camera_id.isdigit():
            return int(normalized_camera_id)
        return normalized_camera_id
    else:
        raise ValueError(f"{name} must be int or str, got {type(camera_id).__name__}")


def mask_camera_id_for_log(camera_id: int | str) -> int | str:
    """로그 출력용 camera_id에서 URL 자격증명을 마스킹한다."""
    if isinstance(camera_id, int):
        return camera_id

    parsed = urlsplit(camera_id)
    if not parsed.scheme or not parsed.netloc or '@' not in parsed.netloc:
        return camera_id

    userinfo, hostinfo = parsed.netloc.rsplit('@', 1)
    if ':' in userinfo:
        username, _ = userinfo.split(':', 1)
        masked_userinfo = f"{username}:***"
    else:
        masked_userinfo = userinfo

    masked_netloc = f"{masked_userinfo}@{hostinfo}"
    return urlunsplit((parsed.scheme, masked_netloc, parsed.path, parsed.query, parsed.fragment))