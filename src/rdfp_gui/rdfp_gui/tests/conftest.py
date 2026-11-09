"""Qt 테스트 공통 — 화면 없는 환경에서 돈다 (`offscreen` 플랫폼 플러그인)."""

from __future__ import annotations

import os

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


@pytest.fixture(scope='session')
def qapp():
    """세션에 하나뿐인 QApplication. PySide6 가 없으면 Qt 테스트는 통째로 skip 한다."""
    QtWidgets = pytest.importorskip('PySide6.QtWidgets')
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app
