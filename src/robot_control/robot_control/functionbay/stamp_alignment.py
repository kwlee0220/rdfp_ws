"""펑션베이 시뮬레이터 stamp 를 호스트 벽시계 축으로 옮긴다.

왜 필요한가
-----------
우리 스택은 전부 벽시계로 돈다 (`use_sim_time: false`). 명령 · `/session` · 세션 레코더의
구간 판정 · rosbag 기록 시각 · DB 의 세션 키가 모두 그렇다. 그런데 시뮬레이터가 싣는
`header.stamp` 는 **회차마다 벽시계와 기동 후 경과시간 사이를 오가고**, 벽시계 회차에도
주기적 정지(체크리스트 A-17)와 그 사이 과속 때문에 실제 시각에서 초 단위로 벗어난다.
그대로 넘기면 명령과 관측이 다른 축에 기록되어 적재가 조용히 데이터를 버린다.

**받은 시각으로 덮어쓰면 안 된다.** 관절 보고는 2~3건씩 묶여 약 48 ms 마다 도착하므로
(체크리스트 A-21) 수신 시각을 stamp 로 쓰면 절반 넘는 표본이 5 ms 안에 몰린다. 그래서
**시뮬레이터 stamp 에 천천히 변하는 오프셋을 더한다.**

정렬 stamp 가 나타내는 것은 **"시뮬레이터가 그 표본을 만든 벽시계 시각"** 이다 — sim 간격이
아니다. 정지 사이에 sim 시각이 벽시계보다 5~7 % 빨리 가므로(2026-09-17 실측) 오프셋이 조금씩
내려가고, 20 ms 짜리 sim 간격이 벽시계로 19~20 ms 대부분, 약 10 % 는 13~15 ms 로 기록된다.
벽시계로 찍히는 명령과 짝짓기에는 이 축이 맞다.

오프셋 추정
-----------
오프셋은 ``수신 시각 − 시뮬레이터 stamp`` 의 **최솟값**(하한 포락선)이다. 한 묶음 안에서
가장 늦게 만들어진 표본이 가장 작은 값을 주므로 전달 지연과 묶음 대기가 함께 빠진다.
벽시계 회차면 수 ms, 경과시간 회차면 epoch 규모가 되어 **회차를 몰라도 된다.**

창은 수신 시각 기준 `window` 로 둔다. 정지 중에는 sim 시각이 거의 멈추므로 정지가 끝나면
오프셋이 약 1 s 뛴다 — **수신 공백이 `gap` 을 넘으면 창을 비워** 정지 직후 표본이 공백
안으로 당겨지지 않게 한다.

⚠️ **창이 차는 동안(기동 직후, 창을 비운 직후)에는 오프셋이 수렴 중이다.** 더 신선한 묶음이
올수록 최솟값이 내려가므로 첫 묶음의 표본들이 같은 시각에 모일 수 있다(간격이 0 이 될 수
있다). 역순은 생기지 않고, 창 하나(0.5 s)가 지나면 간격이 정확히 보존된다. 2026-09-17 원자료
재현에서 180 s 동안 5 ms 미만 간격이 0.1 % 였다.

다른 토픽 (카메라)
------------------
카메라는 관절과 같은 시뮬레이터 시계로 찍히므로 **같은 오프셋을 공유해야** 둘의 관계가
보존된다. 수신 순서로 오프셋을 고르면 정지 경계에서 프레임이 약 1.1 s 틀어지므로
(2026-09-17 원자료 재현), `OffsetHistory` 가 **sim stamp 가 가장 가까운 관절 표본**의
오프셋을 찾는다.

설계 근거: `docs/backends/functionbay/펑션베이_인수_체크리스트.md` §A-9 상세.
"""

from __future__ import annotations

from typing import Optional

import bisect
from collections import deque

# 관절 표본마다 (정렬 시각, 시뮬레이터 stamp) 쌍을 싣는 토픽. `joint_state_fusion` 이 내고
# 카메라 중계가 받는다. 두 노드의 기본값이 이 상수 하나를 본다.
STAMP_REFERENCE_TOPIC = '/functionbay/stamp_reference'
# `sensor_msgs/TimeReference.source` 에 적는 값.
STAMP_REFERENCE_SOURCE = 'functionbay'

DEFAULT_WINDOW_NS = 500_000_000
DEFAULT_GAP_NS = 300_000_000
DEFAULT_HISTORY_NS = 5_000_000_000


class StampAligner:
    """시뮬레이터 stamp → 벽시계 오프셋을 하한 포락선으로 추적한다."""

    def __init__(self, window_ns: int = DEFAULT_WINDOW_NS, gap_ns: int = DEFAULT_GAP_NS) -> None:
        if window_ns <= 0 or gap_ns <= 0:
            raise ValueError(f'window_ns and gap_ns must be > 0, got {window_ns}, {gap_ns}')
        self._window_ns = window_ns
        self._gap_ns = gap_ns
        self._samples: deque[tuple[int, int]] = deque()      # (수신 ns, 수신 − stamp ns)
        self._last_recv_ns: Optional[int] = None
        self._offset_ns: Optional[int] = None
        self.resets = 0

    @property
    def offset_ns(self) -> Optional[int]:
        """마지막으로 추정한 오프셋. 표본을 한 번도 못 봤으면 ``None``."""
        return self._offset_ns

    def observe(self, recv_ns: int, stamp_ns: int) -> int:
        """표본 하나를 반영하고 현재 오프셋을 돌려준다."""
        if self._last_recv_ns is not None and recv_ns - self._last_recv_ns > self._gap_ns:
            # 정지(또는 연결 공백) 뒤다 — 이전 창의 오프셋은 더 이상 맞지 않는다.
            self._samples.clear()
            self.resets += 1
        self._last_recv_ns = recv_ns
        self._samples.append((recv_ns, recv_ns - stamp_ns))
        while recv_ns - self._samples[0][0] > self._window_ns:
            self._samples.popleft()
        self._offset_ns = min(delay for _, delay in self._samples)
        return self._offset_ns

    def align(self, stamp_ns: int) -> Optional[int]:
        """stamp 에 현재 오프셋을 더한다. 오프셋이 아직 없으면 ``None``."""
        if self._offset_ns is None:
            return None
        return stamp_ns + self._offset_ns


class MonotonicStamps:
    """출력 stamp 가 직전 출력보다 작아지지 않게 누른다.

    오프셋은 정지 직후에 바뀐다 — 창을 비운 뒤 수렴하며 내려가고, 시뮬레이터가 밀린 보고를
    몰아 보내는 동안 sim 시각이 벽시계보다 빨리 간다. 그 구간에 들어온 표본끼리는 오프셋
    차이가 sim 간격보다 커질 수 있어, 도착 순서로 내보내면 stamp 가 **거꾸로 갈 수 있다**
    (2026-09-17 실측: 카메라 35 s 에 1건, −5.8 ms). 역순은 TF 버퍼가 `TF_OLD_DATA` 로 버리고
    시각 순서를 전제하는 소비자(frame clock, 보간)를 깨므로 여기서 막는다.
    """

    def __init__(self) -> None:
        self._last_ns: Optional[int] = None
        self.clamped = 0

    def __call__(self, stamp_ns: int) -> int:
        if self._last_ns is not None and stamp_ns <= self._last_ns:
            stamp_ns = self._last_ns + 1
            self.clamped += 1
        self._last_ns = stamp_ns
        return stamp_ns


class OffsetHistory:
    """관절 표본의 (sim stamp, 오프셋) 기록에서 **sim stamp 로** 오프셋을 찾는다."""

    def __init__(self, keep_ns: int = DEFAULT_HISTORY_NS) -> None:
        if keep_ns <= 0:
            raise ValueError(f'keep_ns must be > 0, got {keep_ns}')
        self._keep_ns = keep_ns
        self._sims: list[int] = []
        self._offsets: list[int] = []

    def __len__(self) -> int:
        return len(self._sims)

    def add(self, sim_ns: int, offset_ns: int) -> None:
        """기록 하나를 넣는다. 도착 순서가 뒤섞여도 sim stamp 순으로 유지한다."""
        index = bisect.bisect_left(self._sims, sim_ns)
        if index < len(self._sims) and self._sims[index] == sim_ns:
            self._offsets[index] = offset_ns
        else:
            self._sims.insert(index, sim_ns)
            self._offsets.insert(index, offset_ns)
        cutoff = self._sims[-1] - self._keep_ns
        drop = bisect.bisect_left(self._sims, cutoff)
        if drop:
            del self._sims[:drop]
            del self._offsets[:drop]

    def lookup(self, sim_ns: int) -> Optional[int]:
        """``sim_ns`` 를 양쪽에서 감싸는 기록이 있을 때 더 가까운 쪽의 오프셋.

        뒤쪽 기록(sim stamp 가 ``sim_ns`` 이상)이 아직 없으면 ``None`` 이다 — 곧 도착할
        관절 표본이 더 가까울 수 있으므로 호출자가 기다린다.
        """
        index = bisect.bisect_left(self._sims, sim_ns)
        if index >= len(self._sims):
            return None
        if index == 0 or self._sims[index] == sim_ns:
            return self._offsets[index]
        before, after = index - 1, index
        if sim_ns - self._sims[before] <= self._sims[after] - sim_ns:
            return self._offsets[before]
        return self._offsets[after]

    def nearest(self, sim_ns: int) -> Optional[int]:
        """알고 있는 기록 중 sim stamp 가 가장 가까운 오프셋 (기다리지 않는 대체값)."""
        if not self._sims:
            return None
        index = bisect.bisect_left(self._sims, sim_ns)
        candidates = [i for i in (index - 1, index) if 0 <= i < len(self._sims)]
        best = min(candidates, key=lambda i: abs(self._sims[i] - sim_ns))
        return self._offsets[best]


def stamp_to_ns(stamp) -> int:
    """`builtin_interfaces/Time` 을 나노초 정수로 바꾼다."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def ns_to_stamp(value_ns: int, stamp_type):
    """나노초 정수를 `builtin_interfaces/Time` 으로 바꾼다 (``stamp_type`` 인스턴스)."""
    out = stamp_type()
    out.sec = value_ns // 1_000_000_000
    out.nanosec = value_ns % 1_000_000_000
    return out
