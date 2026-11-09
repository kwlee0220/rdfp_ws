"""`ros2 run rdfp_gui dataset_gui [--config <yaml>]` 진입점.

Qt import 는 `run()` 안에서 한다 — `--help` 만 보려는 셸이 PySide6 를 요구하지 않게.
"""

from __future__ import annotations

from typing import Optional, Sequence

import argparse
import logging
import sys

from rdfp.dataset.cli_common import (DEFAULT_CONFIG_FILE_PATH, LOG_LEVEL_CHOICES,
                                     configure_logging)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='dataset_gui',
        description='rdfp 데이터셋 관리 GUI — bag 토픽 · DB 테이블 · 매핑 · import · 에피소드 · export')
    p.add_argument('--config', help=f'dataset_config.yaml (기본: {DEFAULT_CONFIG_FILE_PATH})')
    p.add_argument('--log-level', choices=LOG_LEVEL_CHOICES, default='info')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    configure_logging(args.log_level)
    try:
        from .app import run
    except ImportError as exc:
        logging.error('PySide6 is required for the GUI: %s '
                      '(pip install --user PySide6-Essentials)', exc)
        return 2
    return run(args.config)


if __name__ == '__main__':
    sys.exit(main())
