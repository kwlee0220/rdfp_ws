"""DB 접속 설정 / DSN 해소 단위 테스트."""

from __future__ import annotations

import pytest

from rdfp.dataset.db.config import DbConfig, resolve_dsn


def test_resolve_dsn_from_env(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv('RDFP_DB_DSN_TEST', 'postgresql://u@h/db')
    cfg = DbConfig(dsn_env='RDFP_DB_DSN_TEST')
    assert resolve_dsn(cfg) == 'postgresql://u@h/db'


def test_resolve_dsn_missing_env(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv('RDFP_DB_DSN_ABSENT', raising=False)
    cfg = DbConfig(dsn_env='RDFP_DB_DSN_ABSENT')
    with pytest.raises(RuntimeError, match='not set'):
        resolve_dsn(cfg)
