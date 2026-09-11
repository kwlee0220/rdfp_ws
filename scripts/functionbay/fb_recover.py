#!/usr/bin/env python3
"""이상 자세에 빠진 팔을 `ready` 로 되돌린다 — **MoveIt 을 쓰지 않는다.**

실제 이동 로직은 `fb_ready.py` 에 있다. 이 스크립트는 "복구" 라는 의도로 부를 때의
이름을 유지하기 위한 얇은 래퍼다 — B 단계 각 스크립트가 시작할 때 부르는 것과
**같은 코드**를 쓴다.

    ./fb_recover.py 5.0        # 5초에 걸쳐 ready 로 램프한 뒤 정착

인자: [램프 시간(초), 기본 5]
"""
from __future__ import annotations

import fb_ready

if __name__ == '__main__':
    fb_ready.main()
