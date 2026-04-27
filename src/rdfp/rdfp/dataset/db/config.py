"""DB 접속 설정 및 DSN 해소.

DSN 은 설정 파일에 평문으로 두지 않고 `dsn_env` 에 환경변수 이름만 기록하며,
실제 값은 런타임에 `resolve_dsn()` 을 통해 환경변수에서 읽어온다.
"""

from __future__ import annotations

import os

from pydantic import BaseModel, Field


DEFAULT_DSN_ENV: str = 'RDFP_DB_DSN'


class DbConfig(BaseModel):
    """DB 접속 설정.

    DSN 은 본 모델에 평문으로 저장하지 않는다. `dsn_env` 에 환경변수 이름을
    지정하고, 실제 DSN 값은 그 환경변수에서 읽어온다.
    """

    dsn_env: str = DEFAULT_DSN_ENV
    schema_: str = Field('public', alias='schema')

    model_config = {'populate_by_name': True}


def resolve_dsn(cfg: DbConfig) -> str:
    """`dsn_env` 환경변수에서 DSN 문자열을 읽어온다.

    Raises:
        RuntimeError: 환경변수가 설정되지 않았거나 빈 문자열인 경우.
    """
    value = os.environ.get(cfg.dsn_env, '').strip()
    if not value:
        raise RuntimeError(
            f"environment variable {cfg.dsn_env!r} is not set; "
            f"set it to a PostgreSQL DSN before running the post-processor"
        )
    return value


__all__ = ['DEFAULT_DSN_ENV', 'DbConfig', 'resolve_dsn']
