"""`robot_twin` console_script 진입점 (설계서 2.6).

기동 순서가 설계의 핵심이다.

1. 설정 로드·검증 — ``move_group_mode`` 미지정이면 **여기서 실패**한다.
2. rclpy 초기화, 노드 생성, 변수 구독 등록, executor 스레드 시작.
3. **HTTP 서버 기동** — 이 시점부터 ``/health`` 응답이 가능하다.
4. MoveGroup 클라이언트 생성 (백그라운드, 실패해도 프로세스는 유지).

ROS 초기화 실패가 프로세스 기동 실패로 이어지면 ``/health`` 로 "준비 안 됨"을
알릴 수단조차 사라진다. 그래서 3단계가 4단계보다 앞선다.

사용 예::

    ros2 run rdfp robot_twin --config <twin.yaml>
"""

from __future__ import annotations

from typing import Optional

import argparse
import os
import sys
import threading

from rdfp.twin.config import TwinConfig, load_config


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog='robot_twin',
        description='Expose a ROS 2 robot over a RESTful/HTTP gateway (extern_op protocol)'
    )
    parser.add_argument('--config', required=True, help='path to the twin definition YAML')
    parser.add_argument('--host', default=None, help='override http.host from the config')
    parser.add_argument('--port', type=int, default=None, help='override http.port from the config')
    parser.add_argument('--log-level', default='info',
                        choices=['critical', 'error', 'warning', 'info', 'debug'],
                        help='uvicorn log level')
    return parser.parse_args(argv)


def _apply_ros_env(config: TwinConfig) -> None:
    """rclpy 초기화 **전에** ROS 환경변수를 적용한다.

    ``ROS_DOMAIN_ID`` / ``RMW_IMPLEMENTATION`` 은 프로세스 환경변수이므로 노드를
    만든 뒤에는 바꿀 수 없다. 이것이 "1 트윈 = 1 프로세스" 원칙의 근거다 (설계서 2.4).
    """
    if config.ros.domain_id is not None:
        os.environ['ROS_DOMAIN_ID'] = str(config.ros.domain_id)
    if config.ros.rmw:
        os.environ['RMW_IMPLEMENTATION'] = config.ros.rmw


def main(argv: Optional[list[str]] = None) -> int:
    """콘솔 진입점."""
    args = _parse_args(argv)

    # --- 1단계: 설정 로드·검증 (여기서 실패하면 기동하지 않는다) ---
    try:
        config = load_config(args.config)
    except Exception as exc:
        print(f'twin config error: {exc}', file=sys.stderr)
        return 2

    host = args.host or config.http.host
    port = args.port or config.http.port

    _apply_ros_env(config)

    # 지연 import — 설정 오류를 ROS 초기화보다 먼저 보고하기 위함이다.
    import rclpy
    import uvicorn
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    from rdfp.twin.api import create_app
    from rdfp.twin.runtime import RobotTwinRuntime

    # --- 2단계: rclpy 초기화, 구독 등록, executor 스레드 시작 ---
    rclpy.init()
    node: Optional[Node] = None
    executor: Optional[MultiThreadedExecutor] = None
    spin_thread: Optional[threading.Thread] = None

    try:
        node = Node(config.ros.node_name)
        if config.ros.use_sim_time:
            node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        logger = node.get_logger()
        logger.info(
            f"robot twin '{config.twin.id}' starting "
            f'(move_group_mode={config.moveit.move_group_mode}, http={host}:{port})'
        )

        runtime = RobotTwinRuntime(node, config)
        runtime.start()

        executor = MultiThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, name='twin-ros-executor',
                                       daemon=True)
        spin_thread.start()

        # --- 4단계: MoveGroup 클라이언트 (백그라운드) ---
        # 3단계(HTTP)보다 먼저 시작하지만 블로킹하지 않으므로 순서 보장은 유지된다.
        runtime.start_move_group_async()
        runtime.cancel_all_on_startup()

        # --- 3단계: HTTP 서버 (블로킹) ---
        app = create_app(runtime)
        logger.warning(
            f'no authentication or TLS is configured; binding {host}:{port}. '
            'Access control is the responsibility of the network layer'
        )
        uvicorn.run(app, host=host, port=port, log_level=args.log_level)
        return 0

    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f'robot twin failed: {exc}', file=sys.stderr)
        return 1
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
