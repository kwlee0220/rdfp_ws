"""inspect — bag · DB · 매핑을 **읽고 점검하는** 계층. GUI(`rdfp_gui`)와 CLI 가 함께 쓴다.

적재 경로에서 데이터가 **조용히** 빠지는 함정(설계안 §2)을 숫자로 보이게 하는 것이 목적이다.
그래서 세는 기준이 적재와 같아야 하고, 그 기준을 여기 한 곳에 둔다 — GUI 가 따로 세면
적재와 어긋난 숫자를 보이게 된다.

| 모듈 | 역할 |
|---|---|
| `bag_inspect` | 세션 트리(미완료 포함) · 토픽 요약 · 메시지 정의 텍스트 · stamp 진단 · 메시지 열람 |
| `mapping` | 토픽 → 테이블 매핑 상태 (내장 registry 기준) |
| `db_inspect` | 테이블 · 스키마 · 행 수 · 에피소드 요약과 점검 · 행 열람 · 삭제 |
| `preflight` | import · export 사전 점검 (CLI 와 GUI 가 같은 함수를 부른다) |
| `findings` | 점검 결과 한 줄의 공통 형태 |

**ROS 를 요구하지 않는다.** `mcap` · `mcap_ros2` · `psycopg` · `yaml` 만 쓴다. Qt 도 여기
들어오지 않는다 — 화면은 `rdfp_gui` 의 몫이고, 그 방향은 `rdfp/tests/test_layer_boundary.py`
가 지킨다.
"""
