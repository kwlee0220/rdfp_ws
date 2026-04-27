from __future__ import annotations

import logging

import rclpy


class _LoggingHandler(logging.Handler):
    """Python logging 레코드를 ROS2 로거로 전달하는 핸들러."""

    def emit(self, record: logging.LogRecord) -> None:
        """레코드 레벨에 맞춰 ROS2 logger 메서드로 전달한다."""
        try:
            ros_logger = rclpy.logging.get_logger(record.name)
            message = self.format(record)

            if record.levelno >= logging.CRITICAL:
                ros_logger.fatal(message)
            elif record.levelno >= logging.ERROR:
                ros_logger.error(message)
            elif record.levelno >= logging.WARNING:
                ros_logger.warning(message)
            elif record.levelno >= logging.INFO:
                ros_logger.info(message)
            else:
                ros_logger.debug(message)
        except Exception:
            # 브리지 실패 시 애플리케이션 동작에 영향을 주지 않도록 무시한다.
            pass


def configure_logging_bridge(*, package_logger_name: str = 'rdfp') -> None:
    """Python logger 출력을 ROS2 logger로 브리지한다.

    - 라이브러리 코드의 logging 출력은 ROS2 로그 체계로 통일한다.
    - 동일 핸들러 중복 등록을 방지한다.
    """
    package_logger = logging.getLogger(package_logger_name)
    for handler in package_logger.handlers:
        if isinstance(handler, _LoggingHandler):
            return

    bridge_handler = _LoggingHandler()
    bridge_handler.setLevel(logging.NOTSET)
    bridge_handler.setFormatter(logging.Formatter('%(levelname)s | %(name)s | %(message)s'))

    package_logger.handlers.clear()
    package_logger.addHandler(bridge_handler)
    package_logger.setLevel(logging.INFO)
    package_logger.propagate = False
