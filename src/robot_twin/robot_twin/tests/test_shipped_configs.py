"""출하 트윈 설정 찾기 (`--twin <id>` → `config/twins/<id>.yaml`) — ROS 없이 임시 디렉터리로 본다.

id 가 곧 파일 이름이라 같은 값이 두 곳에 있다. 둘이 어긋나면 `--twin a` 로 띄웠는데 REST 경로는
`b` 인 트윈이 뜨므로 거부해야 한다.
"""

from __future__ import annotations

import pytest

from robot_twin import main as main_module
from robot_twin.shipped_configs import find_shipped, list_shipped


def _write(directory, twin_id: str, backend: str, name: str = '') -> None:
    (directory / (name or f'{twin_id}.yaml')).write_text(
        f'twin:\n  id: {twin_id}\n  backend: {backend}\n', encoding='utf-8')


@pytest.fixture
def shipped(tmp_path):
    """출하 셋과 같은 모양 + Isaac 위의 두 번째 로봇."""
    _write(tmp_path, 'mock01', 'panda_mock')
    _write(tmp_path, 'rdfp01', 'panda_functionbay')
    _write(tmp_path, 'panda01', 'panda_isaac')
    _write(tmp_path, 'panda02', 'panda_isaac')
    return tmp_path


def test_twin_id_opens_its_own_file(shipped) -> None:
    picked = find_shipped(shipped, 'panda02')
    assert picked.path == shipped / 'panda02.yaml'
    assert picked.backend == 'panda_isaac'


def test_unknown_id_lists_what_is_shipped(shipped) -> None:
    """`--backend` 가 없으니 "Isaac 트윈 id 가 뭐였지?" 를 이 메시지가 알려 준다."""
    with pytest.raises(ValueError, match='no shipped twin config for --twin panda03'
                                         r'.*panda01 \(backend panda_isaac\)'
                                         r'.*rdfp01 \(backend panda_functionbay\)'):
        find_shipped(shipped, 'panda03')


@pytest.mark.parametrize('twin_id', ['../mock01', 'a/b', '', 'mock01.yaml', 'mock 01'])
def test_an_id_that_is_not_a_plain_name_is_refused(shipped, twin_id) -> None:
    """id 는 경로의 한 조각이 된다 — 다른 디렉터리나 다른 파일을 열 수 없어야 한다."""
    with pytest.raises(ValueError, match='invalid twin id'):
        find_shipped(shipped, twin_id)


def test_file_name_and_twin_id_must_agree(shipped) -> None:
    _write(shipped, 'panda09', 'panda_isaac', name='panda03.yaml')
    with pytest.raises(ValueError, match='differs from the file name'):
        find_shipped(shipped, 'panda03')
    with pytest.raises(ValueError, match='panda03.yaml'):
        list_shipped(shipped)


def test_a_file_without_twin_id_is_refused(shipped) -> None:
    (shipped / 'broken.yaml').write_text('twin:\n  backend: panda_mock\n', encoding='utf-8')
    with pytest.raises(ValueError, match='differs from the file name'):
        find_shipped(shipped, 'broken')


# ----- 진입점 인자 ------------------------------------------------------------

def test_config_and_twin_cannot_be_combined() -> None:
    """출처가 둘이면 어느 쪽이 이겼는지 알 수 없다."""
    with pytest.raises(SystemExit):
        main_module._parse_args(['--config', 'x.yaml', '--twin', 'mock01'])


def test_one_source_is_required() -> None:
    with pytest.raises(SystemExit):
        main_module._parse_args([])


def test_backend_is_no_longer_an_option() -> None:
    with pytest.raises(SystemExit):
        main_module._parse_args(['--backend', 'panda_isaac'])


def test_twin_resolves_to_a_shipped_path(shipped, monkeypatch) -> None:
    from robot_twin import shipped_configs

    monkeypatch.setattr(shipped_configs, 'shipped_config_dir', lambda: shipped)
    args = main_module._parse_args(['--twin', 'rdfp01'])
    assert main_module._resolve_config_path(args) == str(shipped / 'rdfp01.yaml')


def test_an_unknown_twin_stops_before_the_port_is_bound(shipped, monkeypatch, capsys) -> None:
    """없는 id 는 설정 오류(종료 코드 2)로 끝난다 — 포트도 ROS 도 건드리지 않는다."""
    from robot_twin import shipped_configs

    monkeypatch.setattr(shipped_configs, 'shipped_config_dir', lambda: shipped)
    bound = []
    monkeypatch.setattr(main_module, '_bind_http_socket', lambda host, port: bound.append(port))

    assert main_module.main(['--twin', 'isaac01']) == 2
    assert bound == []
    assert 'no shipped twin config for --twin isaac01' in capsys.readouterr().err
