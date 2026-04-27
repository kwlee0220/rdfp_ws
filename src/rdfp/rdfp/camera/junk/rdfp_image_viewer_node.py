#!/usr/bin/env python3

"""RdfpImageViewerNode 모듈.

세션 토픽(``rdfp_msgs/msg/SessionCommand``) 상태를 구독하여 이미지 프레임
좌상단에 상태 텍스트를 오버레이하는 뷰어 노드.

동작 요약
    - 기본 뷰어 동작은 :class:`ImageViewerNode` 에서 상속한다.
    - 오버레이 렌더링은 부모의 ``_decorate_frame`` 훅을 오버라이드하여
      반투명 배경 박스 위에 ``"<task_label> (<state>)"`` 텍스트를 그린다
      (배경색/글자색은 :data:`_OVERLAY_BG_COLOR` / :data:`_OVERLAY_TEXT_COLOR`,
      투명도는 :data:`_OVERLAY_BG_ALPHA`).
    - 세션 메시지 수신 전에는 ``state='IDLE'`` / ``task_label=''`` 로 간주한다.

구독 토픽
    - ``image`` (``sensor_msgs/msg/Image``) — 부모 :class:`ImageViewerNode` 에서
      상속. QoS: ``qos_profile_sensor_data`` (``BEST_EFFORT / KEEP_LAST(5)``).
    - ``session`` (``rdfp_msgs/msg/SessionCommand``) — 세션 상태.
      QoS: ``TRANSIENT_LOCAL / RELIABLE / KEEP_LAST(depth=1)``
      (``SessionControlNode`` 발행 QoS 와 동일).

파라미터
    - ``resolution`` (선택) — 부모에서 상속. ``WIDTHxHEIGHT`` 문자열로 지정 시
      해당 크기로 resize. 미지정 시 첫 수신 프레임 크기를 사용.

상태 → 오버레이 접미사 매핑

    ``<label>`` 은 ``task_label`` 값이며, 빈 문자열일 경우 ``"No Task"`` 로
    대체된다.

    - ``IDLE``       → ``"<label> (Idle)"``
    - ``IN_SESSION`` → ``"<label> (Ready)"``
    - ``IN_EPISODE`` → ``"<label> (Recording)"``
    - 그 외         → ``"<label> (Unknown State)"``

사용 예
    기본 실행::

        ros2 run rdfp rdfp_image_viewer_node

    세션 토픽 이름 remap::

        ros2 run rdfp rdfp_image_viewer_node --ros-args -r session:=my_session

    해상도 지정::

        ros2 run rdfp rdfp_image_viewer_node --ros-args -p resolution:=640x480

확장 지점
    - 표시 문자열 규칙 변경: :data:`_STATE_SUFFIX` / :func:`_format_overlay_text`
    - 렌더 스타일 변경: ``_OVERLAY_*`` 모듈 상수
    - 렌더 파이프라인 자체 변경: :meth:`RdfpImageViewerNode._decorate_frame`
"""

from __future__ import annotations

from typing import Any, Optional

import sys

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from rdfp_msgs.msg import SessionCommand  # type: ignore[import-not-found]

from ...ros2_utils import SYSTEM_QOS, log_periodic
from ..image_viewer_node import ImageViewerNode


_DEFAULT_SESSION_TOPIC = 'session'
_DEFAULT_NODE_NAME = 'rdfp_image_viewer_node'
_DEFAULT_WINDOW_NAME = 'rdfp_image_viewer'

# 상태 → 오버레이 접미사 매핑. 알 수 없는 값은 _UNKNOWN_SUFFIX 로 대체한다.
_STATE_SUFFIX = {
    'IDLE': 'Idle',
    'IN_SESSION': 'Ready',
    'IN_EPISODE': 'Recording',
}
_UNKNOWN_SUFFIX = 'Unknown State'
_NO_TASK_LABEL = 'No Task'

# 오버레이 렌더 상수
_OVERLAY_MARGIN_PX = 10       # 좌상단 여백
_OVERLAY_PADDING_PX = 6       # 텍스트 박스 내부 패딩
_OVERLAY_FONT = cv2.FONT_HERSHEY_SIMPLEX
_OVERLAY_FONT_SCALE = 0.7
_OVERLAY_FONT_THICKNESS = 2
_OVERLAY_TEXT_COLOR = (2, 0, 153)      # BGR: RGB(153, 0, 2) 짙은 적색
_OVERLAY_BG_COLOR = (255, 255, 255)    # BGR: 흰색
_OVERLAY_BG_ALPHA = 0.5                # 배경 사각형 투명도

# 오버레이 렌더 에러 로그 rate-limit 간격
_OVERLAY_FAIL_LOG_INTERVAL_SEC = 5.0


def _format_overlay_text(state: str, task_label: str) -> str:
    """세션 상태와 task_label 로 오버레이 문자열을 구성한다.

    ``task_label`` 이 빈 문자열이면 ``"No Task"`` 로 대체되고, ``state`` 가
    :data:`_STATE_SUFFIX` 에 없는 값이면 ``"Unknown State"`` 접미사가 붙는다.
    반환 형식은 ``"<label> (<suffix>)"``.
    """
    label = task_label if task_label else _NO_TASK_LABEL
    suffix = _STATE_SUFFIX.get(state, _UNKNOWN_SUFFIX)
    return f'{label} ({suffix})'


class RdfpImageViewerNode(ImageViewerNode):
    """ImageViewerNode 에 세션 상태 오버레이 기능을 추가한 뷰어 노드.

    - 이미지 파라미터·토픽·윈도우 동작은 부모 클래스와 동일하다.
    - 부모의 :meth:`ImageViewerNode._decorate_frame` 훅을 오버라이드하여
      세션 상태 텍스트를 프레임 좌상단에 오버레이한다.
    - 세션 토픽(``session``)은 remap 으로 다른 이름으로도 구독 가능하며,
      ``SessionControlNode`` 와 호환되도록
      ``TRANSIENT_LOCAL / RELIABLE / KEEP_LAST(depth=1)`` QoS 를 사용한다.
    - 부모 생성자에 전달되는 ``node_name`` / ``window_name`` 은 모듈 상수
      :data:`_DEFAULT_NODE_NAME` / :data:`_DEFAULT_WINDOW_NAME` 로 고정된다.
      변경이 필요하면 서브클래싱으로 재정의한다.
    """

    def __init__(self, **node_kwargs: Any) -> None:
        """노드를 초기화한다.

        부모 :class:`ImageViewerNode` 초기화(이미지 구독·OpenCV 윈도우 생성)
        후, 세션 상태 내부 버퍼를 초기값 ``('IDLE', '')`` 로 세팅하고 세션
        토픽을 구독한다. 세션 메시지 수신 전에도 초기값 기반 오버레이는 정상
        동작한다.

        Args:
            **node_kwargs: ``rclpy.node.Node.__init__`` 에 전달되는 키워드 인자.
                테스트에서 ``parameter_overrides`` 주입 용도로 사용한다.

        Raises:
            RuntimeError: GUI 백엔드 초기화에 실패한 경우
                (부모 :meth:`ImageViewerNode._create_window` 에서 발생).
        """
        super().__init__(node_name=_DEFAULT_NODE_NAME, window_name=_DEFAULT_WINDOW_NAME,
                         **node_kwargs)

        # 세션 상태 내부 보관 (세션 메시지 수신 전까지의 초기값)
        self._session_state: str = 'IDLE'
        self._task_label: str = ''

        # 오버레이 렌더 실패 로그 rate-limit 타임스탬프
        self._last_overlay_fail_log_ts: float = 0.0

        # 세션 토픽 구독: SessionControlNode 발행 QoS 와 동일 설정을 사용해야
        # TRANSIENT_LOCAL 로 직전 상태를 즉시 수신할 수 있다.
        self._session_sub = self.create_subscription(SessionCommand, _DEFAULT_SESSION_TOPIC,
                                                     self._on_session, SYSTEM_QOS,)

        self.get_logger().info(
            f'RdfpImageViewerNode initialized: session_topic={_DEFAULT_SESSION_TOPIC}'
        )

    def _on_session(self, msg: SessionCommand) -> None:
        """세션 토픽 콜백: state / task_label 을 내부 상태로 저장한다.

        ``msg.state`` 와 ``msg.task_label`` 만 참조하며, 프레임 갱신은 수행하지
        않는다. 저장된 값은 다음 이미지 프레임의 :meth:`_decorate_frame` 호출
        시점에 반영된다.
        """
        self._session_state = msg.state
        self._task_label = msg.task_label
        self.get_logger().debug(
            f'session update: state={self._session_state!r} '
            f'task_label={self._task_label!r}'
        )

    def _decorate_frame(self, frame: np.ndarray) -> np.ndarray:
        """현재 세션 상태를 반영한 텍스트를 프레임 좌상단에 오버레이한다.

        부모 :meth:`ImageViewerNode._decorate_frame` 훅을 오버라이드한다.
        반투명 :data:`_OVERLAY_BG_COLOR` 사각형 위에
        :data:`_OVERLAY_TEXT_COLOR` 색 텍스트를 그려 가독성을 확보한다.
        렌더링 중 ``cv2.error`` 가 발생하면 원본 프레임을 그대로 반환하고
        :data:`_OVERLAY_FAIL_LOG_INTERVAL_SEC` 간격으로 WARN 로그를 남긴다
        (화면 표시가 끊기지 않도록 방어).

        Args:
            frame: ``np.ndarray`` (BGR, uint8). resize 가 적용된 프레임.

        Returns:
            오버레이가 그려진 프레임. 렌더 실패 시 원본과 동일한 프레임.
        """
        try:
            text = _format_overlay_text(self._session_state, self._task_label)
            (text_w, text_h), baseline = cv2.getTextSize(
                text, _OVERLAY_FONT, _OVERLAY_FONT_SCALE, _OVERLAY_FONT_THICKNESS,
            )

            # 배경 박스 좌표 (좌상단 기준)
            box_x1 = _OVERLAY_MARGIN_PX
            box_y1 = _OVERLAY_MARGIN_PX
            box_x2 = box_x1 + text_w + 2 * _OVERLAY_PADDING_PX
            box_y2 = box_y1 + text_h + baseline + 2 * _OVERLAY_PADDING_PX

            # 반투명 배경: overlay 에 solid rect 를 그린 뒤 원본과 addWeighted 합성.
            # 사각형 바깥 영역은 overlay == frame 이므로 변화가 없다.
            overlay = frame.copy()
            cv2.rectangle(overlay, (box_x1, box_y1), (box_x2, box_y2),
                          _OVERLAY_BG_COLOR, thickness=cv2.FILLED)
            frame = cv2.addWeighted(overlay, _OVERLAY_BG_ALPHA,
                                    frame, 1.0 - _OVERLAY_BG_ALPHA, 0)

            # 텍스트 기준선은 박스 내부의 하단(패딩 + baseline 반영)
            text_x = box_x1 + _OVERLAY_PADDING_PX
            text_y = box_y2 - _OVERLAY_PADDING_PX - baseline
            cv2.putText(frame, text, (text_x, text_y),
                        _OVERLAY_FONT, _OVERLAY_FONT_SCALE,
                        _OVERLAY_TEXT_COLOR, _OVERLAY_FONT_THICKNESS, cv2.LINE_AA)
            return frame
        except cv2.error as exc:
            self._last_overlay_fail_log_ts = log_periodic(
                self.get_logger().warning,
                f'overlay rendering failed: {exc}',
                self._last_overlay_fail_log_ts,
                _OVERLAY_FAIL_LOG_INTERVAL_SEC,
            )
            return frame


def main(args: Optional[list[str]] = None) -> None:
    """콘솔 엔트리 포인트.

    ``SingleThreadedExecutor`` 로 spin 하며, SIGINT/Ctrl-C 및
    ``ExternalShutdownException`` 경로에서도 ``destroy_node()`` 와
    ``rclpy.try_shutdown()`` 이 실행되도록 finally 로 보호한다.
    """
    rclpy.init(args=args)

    from ...logging_bridge import configure_logging_bridge
    configure_logging_bridge(package_logger_name='rdfp')

    node: Optional[RdfpImageViewerNode] = None
    try:
        node = RdfpImageViewerNode()
    except Exception as exc:
        print(f'[FATAL] RdfpImageViewerNode init failed: {exc}', file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(1)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
