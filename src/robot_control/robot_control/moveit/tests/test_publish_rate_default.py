"""**스트리밍 진입점이 모두 기본 발행 주기를 거치는가.**

`publish_rate` 기본값을 동기 경로(`stream_trajectory`)에만 넣었다가 **비동기 경로 셋이
그대로 10 Hz 로 나갔다** (2026-09-11). 트윈의 `move_linear` 가 그 비동기 경로라
"설정했는데 안 먹는다" 가 됐고, `/health` 는 50 Hz 라고 보고하는데 팔은 뚝뚝 끊겨
움직였다 — **코드로는 안 드러나고 사람이 눈으로 보고 알려 줘서 찾았다.**

그래서 소스를 직접 본다. 새 진입점을 만들면 여기서 걸린다.
"""

from __future__ import annotations

import inspect
import re

from robot_control.moveit.move_group_jgpc_client import MoveGroupJgpcClient

SOURCE = inspect.getsource(MoveGroupJgpcClient)


def _calls() -> list:
    out = []
    for match in re.finditer(r'self\._get_streamer\(\)\.stream(_async)?\(', SOURCE):
        start = match.end()
        depth, i = 1, start
        while i < len(SOURCE) and depth:
            depth += {'(': 1, ')': -1}.get(SOURCE[i], 0)
            i += 1
        out.append(SOURCE[start:i - 1])
    return out


def test_there_are_streaming_entry_points_to_check() -> None:
    """시험이 조용히 0건을 통과하지 않게 한다."""
    assert len(_calls()) >= 4


def test_every_streaming_call_applies_the_default_rate() -> None:
    """네 곳 전부 `self._rate(...)` 를 거쳐야 한다.

    `publish_rate=publish_rate` 로 그대로 넘기면 생성자 기본값(프로파일의
    `arm_command.publish_rate`)이 **조용히 무시된다.**
    """
    bad = [args for args in _calls() if 'self._rate(' not in args]

    assert bad == [], (
        f'{len(bad)}곳이 기본 발행 주기를 안 거친다 — 그 경로는 10 Hz 계단으로 나간다:\n'
        + '\n'.join(f'  {a.strip()[:90]}' for a in bad))


def test_rate_prefers_the_call_argument() -> None:
    """호출 인자가 생성자 기본값을 이긴다."""
    rate = MoveGroupJgpcClient._rate

    class _Fake:
        _publish_rate = 50.0

    assert rate(_Fake(), None) == 50.0
    assert rate(_Fake(), 200.0) == 200.0
    assert rate.__get__(type('F', (), {'_publish_rate': None})())(None) is None
