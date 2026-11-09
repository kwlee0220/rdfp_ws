"""`robot_twin` console_script 진입점 (설계서 2.6).

기동 순서가 설계의 핵심이다.

1. 설정 로드·검증 — ``move_group_mode`` 미지정이면 **여기서 실패**한다.
   이어서 **HTTP 포트를 잡는다** — 이미 쓰이고 있으면 ROS 를 건드리기 전에 끝난다.
2. rclpy 초기화, 노드 생성, 변수 구독 등록, executor 스레드 시작.
3. **HTTP 서버 기동** — 이 시점부터 ``/health`` 응답이 가능하다.
4. MoveGroup 클라이언트 생성 (백그라운드, 실패해도 프로세스는 유지).

ROS 초기화 실패가 프로세스 기동 실패로 이어지면 ``/health`` 로 "준비 안 됨"을
알릴 수단조차 사라진다. 그래서 3단계가 4단계보다 앞선다.

포트를 1단계에서 잡는 이유는 :func:`_bind_http_socket` 에 있다.

사용 예::

    ros2 run robot_twin robot_twin --twin panda01      # 출하 설정 config/twins/<id>.yaml
    ros2 run robot_twin robot_twin --config <twin.yaml>      # 임의의 설정 파일

`--twin <id>` 는 설치본의 `config/twins/<id>.yaml` 을 바로 연다 — :mod:`robot_twin.shipped_configs`.
"""

from __future__ import annotations

from typing import Optional

import argparse
import os
import socket
import sys
import threading

from robot_twin.config import TwinConfig, load_config


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog='robot_twin',
        description='Expose a ROS 2 robot over a RESTful/HTTP gateway (extern_op protocol)'
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--twin', default=None, metavar='ID',
                        help='run the shipped twin config/twins/<ID>.yaml')
    source.add_argument('--config', default=None, help='path to a twin definition YAML')
    parser.add_argument('--host', default=None, help='override http.host from the config')
    parser.add_argument('--port', type=int, default=None, help='override http.port from the config')
    parser.add_argument('--log-level', default='info',
                        choices=['critical', 'error', 'warning', 'info', 'debug'],
                        help='uvicorn log level')
    # 출처는 하나만 받는다 — 둘을 주면 어느 쪽이 이겼는지 알 수 없어 엉뚱한 트윈이 조용히 뜬다.
    return parser.parse_args(argv)


def _resolve_config_path(args: argparse.Namespace) -> str:
    """`--config` 면 그대로, 아니면 설치본의 `config/twins/<id>.yaml` (``ValueError``)."""
    if args.config is not None:
        return args.config
    from robot_twin.shipped_configs import find_shipped, shipped_config_dir

    return str(find_shipped(shipped_config_dir(), args.twin).path)


def _apply_ros_env(config: TwinConfig) -> list[tuple[str, str]]:
    """rclpy 초기화 **전에** ROS 환경변수를 적용하고, 셸 값과 달랐는지 알린다.

    ``ROS_DOMAIN_ID`` / ``RMW_IMPLEMENTATION`` 은 프로세스 환경변수이므로 노드를
    만든 뒤에는 바꿀 수 없다. 이것이 "1 트윈 = 1 프로세스" 원칙의 근거다 (설계서 2.4).

    **설정이 셸을 이긴다** — 그래서 트윈은 늘 설정의 도메인으로 뜨지만, 로봇 스택은 자기
    셸의 값으로 뜬다. 둘이 다르면 서로 못 보고 증상은 변수의 ``NO_DATA`` 뿐이라, 덮어쓴
    사실을 로그로 남긴다. 노드가 생기기 전이라 ``(level, message)`` 목록으로 돌려주고
    호출자가 노드 로거로 찍는다.

    Returns:
        ``('warning' | 'info', message)`` 목록. 셸 값과 같으면 빈 목록이다.
    """
    notes: list[tuple[str, str]] = []
    for key, value in (('ROS_DOMAIN_ID', config.ros.domain_id),
                       ('RMW_IMPLEMENTATION', config.ros.rmw)):
        if value is None or value == '':
            continue
        value = str(value)
        shell = os.environ.get(key)
        if shell is None:
            notes.append(('info', f'{key}={value} from the twin config (unset in this shell); '
                                  'the robot stack must run with the same value'))
        elif shell != value:
            notes.append(('warning', f'{key}={value} from the twin config overrides {shell!r} '
                                     'from this shell; a robot stack started from a shell like '
                                     'this one will not see the twin'))
        os.environ[key] = value
    return notes


def _bind_http_socket(host: str, port: int) -> socket.socket:
    """HTTP 포트를 **ROS 를 건드리기 전에** 잡는다. 이미 쓰이고 있으면 ``OSError``.

    출하 트윈 셋은 같은 포트(8801)라 실수로 두 번 띄우면 두 번째가 포트 충돌로 끝나야
    한다. 그런데 bind 를 uvicorn 에 맡기면 충돌을 ROS 초기화 **뒤에야** 알게 되고, 그 사이
    MoveGroup 초기화 스레드가 출발해 **컨트롤러의 진행 중 궤적을 취소**할 수 있다
    (설계서 6.13) — 떠 있는 트윈이 실행하던 궤적을 두 번째가 끊는 것이다. 같은 이름의
    노드도 잠시 둘이 된다.

    잡은 소켓을 uvicorn 에 그대로 넘기므로 확인과 사용 사이의 틈이 없다. 주소족 판정과
    ``SO_REUSEADDR`` 는 uvicorn 이 스스로 bind 할 때와 같다 — ``SO_REUSEADDR`` 는
    ``TIME_WAIT`` 만 허용하고 **듣고 있는 소켓과의 충돌은 그대로 거부한다.**
    """
    family = socket.AF_INET6 if ':' in host else socket.AF_INET
    sock = socket.socket(family, socket.SOCK_STREAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, port))
        sock.listen(2048)       # uvicorn 의 기본 backlog
    except OSError:
        sock.close()
        raise
    return sock


def main(argv: Optional[list[str]] = None) -> int:
    """콘솔 진입점."""
    args = _parse_args(argv)

    # --- 1단계: 설정 로드·검증 (여기서 실패하면 기동하지 않는다) ---
    try:
        config_path = _resolve_config_path(args)
        print(f'[twin] config: {config_path}', flush=True)
        config = load_config(config_path)
    except Exception as exc:
        print(f'twin config error: {exc}', file=sys.stderr)
        return 2

    host = args.host or config.http.host
    port = args.port or config.http.port

    # 포트를 **ROS 보다 먼저** 잡는다 — 같은 트윈이 이미 떠 있으면 여기서 끝난다.
    try:
        http_socket = _bind_http_socket(host, port)
    except OSError as exc:
        print(f'robot twin: cannot bind {host}:{port} ({exc.strerror}) — another twin may '
              'already be running on it; stop that one first', file=sys.stderr)
        return 1

    env_notes = _apply_ros_env(config)

    # 지연 import — 설정 오류를 ROS 초기화보다 먼저 보고하기 위함이다.
    import rclpy
    import uvicorn
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    from robot_twin.api import create_app
    from robot_twin.runtime import RobotTwinRuntime

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
            f'(backend={config.twin.backend}, move_group_mode={config.moveit.move_group_mode}, '
            f'http={host}:{port})'
        )
        # rclpy 로거는 호출 지점마다 severity 를 기억하므로 레벨별로 줄을 나눈다.
        for level, message in env_notes:
            if level == 'warning':
                logger.warning(message)
            else:
                logger.info(message)

        runtime = RobotTwinRuntime(node, config)
        runtime.start()

        executor = MultiThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, name='twin-ros-executor',
                                       daemon=True)
        spin_thread.start()

        # --- 4단계: MoveGroup 클라이언트 (백그라운드) ---
        # 3단계(HTTP)보다 먼저 시작하지만 블로킹하지 않으므로 순서 보장은 유지된다.
        # 이전 트윈이 남긴 컨트롤러 궤적의 정리(설계서 6.13)도 이 안에서, 클라이언트를
        # 연산에 내놓기 직전에 한다.
        runtime.start_move_group_async()

        # --- 3단계: HTTP 서버 (블로킹) ---
        app = create_app(runtime)
        logger.warning(
            f'no authentication or TLS is configured; binding {host}:{port}. '
            'Access control is the responsibility of the network layer'
        )
        # 1단계에서 잡은 소켓으로 서빙한다 — uvicorn 이 다시 bind 하지 않는다.
        server = uvicorn.Server(uvicorn.Config(app, host=host, port=port,
                                               log_level=args.log_level))
        server.run(sockets=[http_socket])
        return 0

    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f'robot twin failed: {exc}', file=sys.stderr)
        return 1
    finally:
        http_socket.close()
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
