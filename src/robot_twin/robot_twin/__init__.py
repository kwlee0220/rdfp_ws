"""로봇 트윈 — ROS 2 로봇을 RESTful/HTTP 로 노출하는 게이트웨이.

설계서: `docs/robot_twin/robot_twin_design.md`

ROS 2 를 직접 사용하기 어려운 환경에서 상태 변수 조회(polling)와 미리 구현된
연산 실행을 REST 로 제공한다. 트윈 자체가 ROS 노드이므로 별도 브리지를 두지
않는다.

패키지 경계 규칙 (설계서 2.5)
-----------------------------
본 서브패키지는 ``robot_control.moveit`` 외의 rdfp 서브패키지를 import 하지 않는다.
반대로 rdfp 의 기존 코드도 ``robot_twin`` 을 import 하지 않는다. 이 단방향
의존을 지켜야 나중에 별도 패키지로 분리할 때 기계적 이동으로 끝난다.

본 최상위 모듈은 ROS·웹 의존성을 import 하지 않는다. 하위 모듈이 필요에 따라
개별적으로 import 한다.
"""

from __future__ import annotations

__all__ = ['__version__']

__version__ = '0.1.0'
