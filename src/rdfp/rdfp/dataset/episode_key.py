"""에피소드 자연키 — bag 과 DB 를 잇는 단 하나의 공통 값.

에피소드는 **`sessions` 테이블**에 담긴다 — 이름이 개념과 어긋나지만 스키마가 그렇다
(`schema.sql`). 그 테이블의 유일성 제약이 ``UNIQUE (start_sec, start_nanosec)`` 이고, 그
값은 에피소드 시작 `/session` 메시지의 `header.stamp` 다. bag 쪽 `Episode.start_ns` 와 **같은
값**이므로, DB 의 대리키(`id`, BIGSERIAL)를 모르고도 "이 에피소드가 적재됐는가" 를 키
조회 하나로 판정할 수 있다.

대리키를 쓰지 않는 이유는 두 가지다. bag 만 보고는 알 수 없고, `on_existing_episode`
정책이 `replace` 면 재적재할 때 **값이 바뀐다.**

⚠️ **`start_ns` 가 언제나 유일하지는 않다.** `header.stamp` 가 벽시계가 아니라 경과시간인
회차에서는 시뮬레이터를 재시작할 때 0 부터 다시 시작하므로, 서로 다른 세션의 에피소드가
같은 키를 갖는다. 사람이 보는 화면에는 세션 이름을 함께 싣는다.

이 모듈은 **stdlib 만 쓴다** — ROS 도 DB 드라이버도 요구하지 않는다.
"""

from __future__ import annotations

from typing import Iterable, Optional

NS_PER_SEC = 1_000_000_000


def split_ns(ns: int) -> tuple[int, int]:
    """epoch ns 를 `(sec, nanosec)` 으로 나눈다 — DB 컬럼 짝과 같은 형태다."""
    return divmod(int(ns), NS_PER_SEC)


def join_ns(sec: int, nanosec: int) -> int:
    """`(sec, nanosec)` 을 epoch ns 로 합친다. :func:`split_ns` 의 역이다."""
    return int(sec) * NS_PER_SEC + int(nanosec)


def fetch_exported_keys(conn, start_ns_lo: Optional[int] = None,
                        start_ns_hi: Optional[int] = None) -> set[int]:
    """DB 에 적재된 에피소드의 `start_ns` 집합을 **한 번의 질의로** 가져온다.

    에피소드마다 따로 묻지 않는다 — 목록이 길어지면 N+1 질의가 된다.

    Args:
        conn: 열린 DB 커넥션 (psycopg).
        start_ns_lo: 조회 하한 (inclusive). `None` 이면 제한 없음.
        start_ns_hi: 조회 상한 (inclusive). `None` 이면 제한 없음.

    Returns:
        `start_ns` 집합. 비어 있으면 그 구간에 적재된 에피소드가 없다는 뜻이다.
    """
    # ⚠️ 테이블 이름은 `sessions` 다 (`episodes` 가 아니다).
    sql = 'SELECT start_sec, start_nanosec FROM sessions'
    params: list[int] = []
    # `start_sec` 으로 좁혀야 인덱스를 탄다 — ns 를 SQL 쪽에서 합치면 못 탄다.
    bounds: list[str] = []
    if start_ns_lo is not None:
        bounds.append('start_sec >= %s')
        params.append(split_ns(start_ns_lo)[0])
    if start_ns_hi is not None:
        bounds.append('start_sec <= %s')
        params.append(split_ns(start_ns_hi)[0] + 1)
    if bounds:
        sql += ' WHERE ' + ' AND '.join(bounds)

    with conn.cursor() as cur:
        cur.execute(sql, tuple(params))
        rows = cur.fetchall()
    return {join_ns(sec, nanosec) for sec, nanosec in rows}


def key_range(start_ns_values: Iterable[int]) -> tuple[Optional[int], Optional[int]]:
    """`start_ns` 목록의 `(최소, 최대)`. 비었으면 `(None, None)`."""
    values = sorted(start_ns_values)
    if not values:
        return None, None
    return values[0], values[-1]


__all__ = ['NS_PER_SEC', 'split_ns', 'join_ns', 'fetch_exported_keys', 'key_range']
