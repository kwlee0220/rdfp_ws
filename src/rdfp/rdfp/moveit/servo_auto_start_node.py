#!/usr/bin/env python3
"""moveit_servo 를 자동으로 기동시키는 일회성 노드.

``moveit_servo`` 는 ``start_servo`` (``std_srvs/Trigger``) 서비스가 호출되기
전까지 ``delta_twist_cmds`` / ``delta_joint_cmds`` 입력을 무시한다. teleop 처럼
사람이 개입하는 경로에서는 클라이언트가 직접 호출하지만, replay 처럼 무인으로
동작하는 스택에는 호출 주체가 없다. 본 노드가 그 역할을 담당한다.

동작은 일회성이다.

1. ``servo_node_name`` 의 서비스가 준비될 때까지 ``service_timeout`` 만큼 대기
2. :meth:`ServoClient.auto_start` 로 ``start_servo`` 요청 전송
3. 결과를 로그로 남기고 종료

launch 에서 servo_node 와 동시에 spawn 되어도 안전하도록 대기 시간을
:class:`ServoClient` 의 기본값(3초)보다 넉넉하게 잡는다. ``startup_delay`` 로
첫 시도 전 추가 지연을 줄 수 있다.

파라미터
--------

==================  =======  =========================================
이름                기본값   설명
==================  =======  =========================================
servo_node_name     /servo_node  대상 servo 노드 이름
service_timeout     30.0     서비스 준비 대기 한도(초)
startup_delay       0.0      첫 시도 전 지연(초)
exit_after_start    True     시작 요청 후 노드를 종료할지 여부
==================  =======  =========================================
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from rdfp.moveit.servo_client import ServoClient
from rdfp.ros2_utils import get_parameter, parse_bool, parse_float, parse_str


class ServoAutoStartNode(Node):
    """servo 의 ``start_servo`` 를 한 번 호출하고 종료하는 노드."""

    def __init__(self):
        super().__init__('servo_auto_start')

        self.declare_parameter('servo_node_name', '/servo_node')
        self.declare_parameter('service_timeout', 30.0)
        self.declare_parameter('startup_delay', 0.0)
        self.declare_parameter('exit_after_start', True)

        self._servo_node_name = get_parameter(self, 'servo_node_name', parse_str)
        self._service_timeout = get_parameter(self, 'service_timeout', parse_float)
        self._startup_delay = get_parameter(self, 'startup_delay', parse_float)
        self._exit_after_start = get_parameter(self, 'exit_after_start', parse_bool)

        if self._service_timeout <= 0.0:
            raise ValueError(f"'service_timeout' must be > 0, got {self._service_timeout}")
        if self._startup_delay < 0.0:
            raise ValueError(f"'startup_delay' must be >= 0, got {self._startup_delay}")

        self._started = False
        self.get_logger().info(
            f'ServoAutoStart target={self._servo_node_name} '
            f'service_timeout={self._service_timeout}s startup_delay={self._startup_delay}s'
        )

    @property
    def exit_after_start(self) -> bool:
        """시작 요청 후 노드를 종료할지 여부를 반환한다."""
        return self._exit_after_start

    @property
    def started(self) -> bool:
        """``start_servo`` 요청이 성공적으로 전송되었는지 여부를 반환한다."""
        return self._started

    def run_once(self) -> bool:
        """servo 서비스가 준비되길 기다린 뒤 ``start_servo`` 를 호출한다.

        ``ServoClient.wait_for_services_ready`` 는 내부에서 ``rclpy.spin_once`` 를
        수행하므로 외부 executor 없이 단독 호출해도 동작한다.

        Returns:
            시작 요청이 전송되었거나 이미 정상 동작 중이면 True.
        """
        if self._startup_delay > 0.0:
            self._sleep_spinning(self._startup_delay)

        client = ServoClient.create(self, self._servo_node_name)
        if not client.wait_for_services_ready(timeout_sec=self._service_timeout):
            self.get_logger().error(
                f'servo services not available within {self._service_timeout}s '
                f'({self._servo_node_name}/start_servo); servo will ignore twist commands'
            )
            return False

        self._started = client.auto_start()
        if self._started:
            self.get_logger().info('start_servo requested; servo now accepts twist commands')
        else:
            self.get_logger().error('Failed to request start_servo')
        return self._started

    def _sleep_spinning(self, duration_sec: float) -> None:
        """콜백을 처리하면서 지정 시간만큼 대기한다."""
        remaining = duration_sec
        while remaining > 0.0 and rclpy.ok():
            step = min(0.1, remaining)
            rclpy.spin_once(self, timeout_sec=step)
            remaining -= step


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoAutoStartNode()
        node.run_once()
        # exit_after_start 가 False 면 노드를 살려 둔다(수동 재시도/디버깅 용도).
        if not node.exit_after_start:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
