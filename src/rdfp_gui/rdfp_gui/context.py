"""앱 컨텍스트 — 설정 파일 하나와 거기서 파생되는 경로들. **Qt 를 모른다.**

GUI 의 모든 화면이 같은 `DatasetConfig` 를 본다. 설정을 못 읽어도 창은 떠야 하므로(④ 설정 탭에서
고칠 수 있게) 오류를 들고 있다가 화면이 보이게 한다.
"""

from __future__ import annotations

from typing import Optional

import os
import re
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

from rdfp.dataset.cli_common import DEFAULT_CONFIG_FILE_PATH
from rdfp.dataset.config import DatasetConfig, load_dataset_config
from rdfp.dataset.db.config import resolve_dsn

_DSN_PASSWORD = re.compile(r'(://[^:/@]+:)[^@]*(@)')

#: sim 시계(에포크 기준 수십 초)와 벽시계를 가르는 경계 — 1973년 이전 값은 sim 시계로 본다.
_SIM_CLOCK_LIMIT_NS = 10 ** 17


def mask_dsn(dsn: str) -> str:
    """`postgresql://user:secret@host/db` → `postgresql://user:***@host/db`. 비밀번호는 화면에 안 보인다."""
    return _DSN_PASSWORD.sub(r'\1***\2', dsn)


def fmt_time(ns: Optional[int]) -> str:
    """epoch ns 를 사람이 읽는 시각으로. sim 시계(작은 값)는 초로 보인다 — 1970-01-01 로 보이면 헷갈린다."""
    if ns is None:
        return '-'
    if ns < _SIM_CLOCK_LIMIT_NS:
        return f'{ns / 1e9:.3f} s (sim)'
    return datetime.fromtimestamp(ns / 1e9).strftime('%Y-%m-%d %H:%M:%S')


def fmt_duration(ns: int) -> str:
    sec = ns / 1e9
    if sec < 90:
        return f'{sec:.1f} s'
    return f'{sec / 60:.1f} min'


@dataclass
class AppContext:
    """설정 파일과 그 해석 결과. `reload()` 로 다시 읽는다."""

    config_path: Optional[Path] = None
    cfg: Optional[DatasetConfig] = None
    error: Optional[str] = None
    workspace_root: Path = field(default_factory=lambda: Path(os.environ.get('RDFP_HOME', '.')))

    @classmethod
    def load(cls, config_path: Optional[str] = None) -> 'AppContext':
        path = Path(config_path) if config_path else DEFAULT_CONFIG_FILE_PATH
        ctx = cls(config_path=path)
        ctx.reload()
        return ctx

    def reload(self) -> None:
        self.cfg = None
        self.error = None
        if self.config_path is None:
            self.error = 'no config file given'
            return
        try:
            self.cfg = load_dataset_config(self.config_path)
        except (FileNotFoundError, ValueError) as exc:
            self.error = str(exc)

    def switch(self, config_path: str) -> None:
        self.config_path = Path(config_path)
        self.reload()

    @property
    def loaded(self) -> bool:
        return self.cfg is not None

    @property
    def rosbag_dir(self) -> Optional[Path]:
        return Path(self.cfg.rosbag_dir) if self.cfg else None

    def mp4_roots(self) -> list[str]:
        return self.cfg.mp4_roots() if self.cfg else []

    @property
    def dsn_env(self) -> str:
        return self.cfg.db.dsn_env if self.cfg else 'RDFP_DB_DSN'

    def db_configured(self) -> bool:
        """DSN 환경변수가 있는가 — 접속이 되는지는 `preflight.check_db` 가 본다."""
        if self.cfg is None:
            return False
        try:
            resolve_dsn(self.cfg.db)
        except RuntimeError:
            return False
        return True

    def masked_dsn(self) -> str:
        if not self.db_configured():
            return f'(환경변수 {self.dsn_env} 없음)'
        return mask_dsn(resolve_dsn(self.cfg.db))

    def default_export_dir(self) -> Path:
        root = os.environ.get('RDFP_DATA_ROOT')
        base = Path(root) / 'datasets' if root else Path.home() / 'rdfp_datasets'
        return base / datetime.now().strftime('%Y%m%d')

    def path_rows(self) -> list[tuple[str, Optional[str], Optional[bool]]]:
        """④ 설정 탭의 경로 표 — (이름, 값, 존재 여부). 값이 없으면 ``None``."""
        if self.cfg is None:
            return []
        rows = []
        for name in ('rosbag_dir', 'output_mp4_dir', 'recordings_dir'):
            value = getattr(self.cfg, name)
            rows.append((name, value, Path(value).is_dir() if value else None))
        return rows


__all__ = ['AppContext', 'fmt_duration', 'fmt_time', 'mask_dsn']
