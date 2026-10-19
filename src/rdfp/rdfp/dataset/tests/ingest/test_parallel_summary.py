"""병렬 모드 결과 집계(`_apply_result_to_summary`) 단위 테스트.

대상 함수는 dict 만 다루지만 `pipeline` 이 `sensor_msgs` 를 전이 의존하므로,
ROS sourcing 이 안 된 환경에서는 전체 테스트를 skip 한다.
"""

from __future__ import annotations

import pytest

pytest.importorskip('sensor_msgs', reason='requires ROS 2 runtime (transitive dependency)')

from rdfp.dataset.ingest.pipeline import _apply_result_to_summary, _empty_summary  # noqa: E402


def test_failed_result_is_counted() -> None:
    """워커는 실패를 예외가 아니라 `status='failed'` 로 돌려준다 — 집계가 세야 한다."""
    summary = _empty_summary(splits=1)
    _apply_result_to_summary({'status': 'done', 'inserted': {'joint_states': 5}}, summary, 'skip')
    _apply_result_to_summary({'status': 'skipped'}, summary, 'skip')
    _apply_result_to_summary({'status': 'failed', 'error': 'boom'}, summary, 'skip')

    assert summary['episodes'] == 1
    assert summary['skipped'] == 1
    assert summary['failed'] == 1
    assert summary['inserted'] == {'joint_states': 5}
