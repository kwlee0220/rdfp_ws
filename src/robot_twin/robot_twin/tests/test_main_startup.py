"""`robot_twin` 진입점의 기동 순서 — **포트는 ROS 보다 먼저 잡는다.**

출하 트윈 셋은 같은 포트(8801)라 실수로 두 번 띄우면 두 번째가 포트 충돌로 끝나야 한다.
bind 를 uvicorn 에 맡기면 충돌을 ROS 초기화 뒤에야 알게 되고, 그 사이 MoveGroup 초기화
스레드가 떠 있는 트윈의 진행 중 궤적을 취소할 수 있다 (설계서 6.13). 여기서는 그 순서를
ROS 없이 확인한다.
"""

from __future__ import annotations

import socket

import pytest

from robot_twin import main as main_module


def _listening_socket() -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('127.0.0.1', 0))
    sock.listen(1)
    return sock


def _write_config(tmp_path) -> str:
    path = tmp_path / 'twin.yaml'
    path.write_text('twin:\n  id: panda01\nmoveit:\n  move_group_mode: jtc\n', encoding='utf-8')
    return str(path)


def test_bind_refuses_a_port_that_is_already_listening() -> None:
    """`SO_REUSEADDR` 는 `TIME_WAIT` 만 허용한다 — 듣고 있는 소켓과의 충돌은 그대로 거부한다."""
    busy = _listening_socket()
    try:
        with pytest.raises(OSError):
            main_module._bind_http_socket('127.0.0.1', busy.getsockname()[1])
    finally:
        busy.close()


def test_bind_returns_a_listening_socket() -> None:
    sock = main_module._bind_http_socket('127.0.0.1', 0)
    try:
        probe = socket.create_connection(sock.getsockname(), timeout=1.0)
        probe.close()
    finally:
        sock.close()


def test_busy_port_stops_before_ros_is_touched(tmp_path, monkeypatch, capsys) -> None:
    """두 번째 트윈은 ROS 환경을 적용하기도 전에 끝난다 — 노드도, 궤적 취소도 없다."""
    touched = []
    monkeypatch.setattr(main_module, '_apply_ros_env', lambda config: touched.append(config))
    busy = _listening_socket()
    try:
        code = main_module.main(['--config', _write_config(tmp_path),
                                 '--host', '127.0.0.1', '--port', str(busy.getsockname()[1])])
    finally:
        busy.close()

    assert code == 1
    assert touched == []
    assert 'another twin may already be running' in capsys.readouterr().err


# ----- 도메인 덮어쓰기 알림 ---------------------------------------------------
#
# 설정이 셸을 이기므로 트윈은 늘 설정의 도메인으로 뜨지만 로봇 스택은 자기 셸의 값으로
# 뜬다. 다르면 서로 못 보고 증상은 `NO_DATA` 뿐이라 덮어쓴 사실을 남긴다.

def _config(domain_id=31, rmw='rmw_fastrtps_cpp'):
    from robot_twin.config import TwinConfig
    return TwinConfig.model_validate({
        'twin': {'id': 'panda01'}, 'moveit': {'move_group_mode': 'jtc'},
        'ros': {'domain_id': domain_id, 'rmw': rmw}})


def test_a_different_shell_domain_is_a_warning(monkeypatch) -> None:
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')

    notes = main_module._apply_ros_env(_config())

    assert [level for level, _ in notes] == ['warning']
    assert "overrides '0'" in notes[0][1]
    assert main_module.os.environ['ROS_DOMAIN_ID'] == '31'


def test_an_unset_shell_domain_is_only_noted(monkeypatch) -> None:
    monkeypatch.delenv('ROS_DOMAIN_ID', raising=False)
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')

    notes = main_module._apply_ros_env(_config())

    assert [level for level, _ in notes] == ['info']
    assert main_module.os.environ['ROS_DOMAIN_ID'] == '31'


def test_matching_shell_values_say_nothing(monkeypatch) -> None:
    monkeypatch.setenv('ROS_DOMAIN_ID', '31')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')

    assert main_module._apply_ros_env(_config()) == []
