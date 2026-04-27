"""후처리기 CLI 공통 유틸.

`rosbag` / `dataset` 두 CLI 가 공유하는 로깅 초기화와 공통 argparse 옵션,
dataset 설정 로더 헬퍼를 모아 둔다.
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

from .config import DatasetConfig, load_dataset_config


LOG_LEVEL_CHOICES: tuple[str, ...] = ('debug', 'info', 'warning', 'error')
LOG_FORMAT: str = '%(asctime)s [%(levelname)s] %(name)s: %(message)s'

# --config 미지정 시 현재 작업 디렉터리에서 자동 탐색할 기본 설정 파일명.
DEFAULT_CONFIG_FILENAME: str = 'dataset_config.yaml'


def add_common_args(parser: argparse.ArgumentParser) -> None:
    """모든 CLI 의 최상위 파서에 공통 옵션을 추가한다.

    현재는 `--log-level` 한 개만 제공한다.
    """
    parser.add_argument('--log-level', choices=LOG_LEVEL_CHOICES, default='info')


def add_config_arg(parser: argparse.ArgumentParser, *, required: bool = True) -> None:
    """서브파서에 `--config` 옵션을 표준 형태로 추가한다 (dataset CLI 전용)."""
    parser.add_argument(
        '--config', required=required,
        help='path to dataset_config.yaml',
    )


def configure_logging(level_name: str) -> None:
    """logging.basicConfig 를 표준 포맷으로 초기화한다."""
    logging.basicConfig(
        level=getattr(logging, level_name.upper()),
        format=LOG_FORMAT,
    )


def load_dataset_or_fail(path: str) -> DatasetConfig | None:
    """dataset 용 설정을 로드하고 실패 시 로깅 후 None 을 반환한다.

    호출 측은 None 을 받으면 CLI exit code `2` 로 귀결시킨다 (인자/설정 오류).
    """
    try:
        return load_dataset_config(path)
    except (FileNotFoundError, ValueError) as e:
        logging.error('failed to load config: %s', e)
        return None


def resolve_config_path(args: argparse.Namespace) -> str | None:
    """`args.config` 를 우선 사용하고, 없으면 cwd 의 기본 파일을 찾는다.

    Returns:
        ``args.config`` 가 설정되어 있으면 그 값, 아니면 현재 작업 디렉터리의
        ``dataset_config.yaml`` 경로 (파일 존재 시). 파일도 없으면 ``None``.
    """
    if getattr(args, 'config', None):
        return args.config
    candidate = Path(DEFAULT_CONFIG_FILENAME)
    if candidate.is_file():
        return str(candidate)
    return None


__all__ = [
    'LOG_LEVEL_CHOICES',
    'LOG_FORMAT',
    'DEFAULT_CONFIG_FILENAME',
    'add_common_args',
    'add_config_arg',
    'configure_logging',
    'load_dataset_or_fail',
    'resolve_config_path',
]
