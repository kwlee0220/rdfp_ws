"""출하 트윈 설정 찾기 — `--twin <id>` 가 설치본의 `config/twins/<id>.yaml` 을 바로 연다.

**id 가 곧 파일 이름이다** (2026-10-25). 트윈을 더하거나 뺄 때는 `src/robot_twin/config/twins/` 에
파일을 넣거나 빼고 빌드하면 되고, 런처(`run_robot_twin.sh`)는 고치지 않는다. 예전에는 런처가
백엔드 → 파일 이름 표를 따로 들고 있어, YAML 의 값과 표가 어긋나면 엉뚱한 파일을 조용히 골랐다.

같은 값이 파일 이름과 `twin.id` 두 곳에 있으므로 **둘이 다르면 거부한다** — 그대로 두면
`--twin a` 로 띄웠는데 REST 경로는 `/robot_twins/b` 인 트윈이 뜬다.

`twins/` 하위 디렉터리를 쓰는 것은 그 안의 이름 전체를 트윈 id 로 두기 위해서다. `config/` 에
트윈이 아닌 YAML 이 들어와도 `--twin <그 이름>` 으로 열리지 않는다.

`rclpy` 를 import 하지 않는다 — 설정 파일만 읽는 경로가 ROS 를 요구하면 안 된다.
"""

from __future__ import annotations

from typing import Optional

import os
import re
from dataclasses import dataclass
from pathlib import Path

import yaml


# id 는 경로의 한 조각이 되므로 `/` · `..` 같은 것이 들어오지 못하게 한다.
_TWIN_ID = re.compile(r'[A-Za-z0-9_-]+')


@dataclass(frozen=True)
class ShippedConfig:
    """출하 설정 하나 — 경로와 파일에 적힌 두 값."""

    path: Path
    twin_id: str
    backend: Optional[str]

    def describe(self) -> str:
        return f'{self.twin_id} (backend {self.backend or "<none>"})'


def shipped_config_dir() -> Path:
    """설치된 `robot_twin` 패키지의 `share/robot_twin/config/twins`.

    소스 트리(`src/`)가 아니라 설치본이다 — scene 노드 · Isaac 스테이지가 같은 시점의 값을
    보게 하려는 것이며, 그래서 설정을 고치면 `colcon build` 를 해야 반영된다.
    """
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory('robot_twin')) / 'config' / 'twins'


def _read(path: Path) -> ShippedConfig:
    raw = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    twin = raw.get('twin') or {}
    twin_id = twin.get('id')
    if twin_id != path.stem:
        raise ValueError(f'twin.id {twin_id!r} in {path} differs from the file name — '
                         'a shipped twin config must be named <twin.id>.yaml')
    return ShippedConfig(path=path, twin_id=str(twin_id), backend=twin.get('backend'))


def list_shipped(config_dir: str | os.PathLike) -> list[ShippedConfig]:
    """`config_dir` 의 출하 설정을 id 순으로 읽는다. 파일 이름과 `twin.id` 가 다르면 예외다."""
    return [_read(path) for path in sorted(Path(config_dir).glob('*.yaml'))]


def find_shipped(config_dir: str | os.PathLike, twin_id: str) -> ShippedConfig:
    """`<config_dir>/<twin_id>.yaml` 을 돌려준다. 없거나 형식이 틀리면 `ValueError`."""
    if not _TWIN_ID.fullmatch(twin_id):
        raise ValueError(f'invalid twin id {twin_id!r} — use letters, digits, "_" and "-" only')
    path = Path(config_dir) / f'{twin_id}.yaml'
    if not path.is_file():
        try:
            shipped = ', '.join(c.describe() for c in list_shipped(config_dir)) or '<none>'
        except (OSError, ValueError) as exc:
            shipped = f'<unreadable: {exc}>'
        raise ValueError(f'no shipped twin config for --twin {twin_id} in {config_dir} '
                         f'(shipped: {shipped})')
    return _read(path)


__all__ = ['ShippedConfig', 'find_shipped', 'list_shipped', 'shipped_config_dir']
