"""rdfp_gui — 데이터셋 관리 GUI (PySide6). `rdfp` **위에** 있는 패키지다.

rosbag2 녹화물(bag) · PostgreSQL 적재 결과(DB) · 그 사이의 매핑을 한 화면에 놓고, 적재(import) 와
학습 샘플 export 를 기존 `rdfp` CLI 를 **서브프로세스로** 돌려 실행한다 — 적재 경로를 두 벌로
만들지 않는다. 설계: `docs/collect/데이터셋_관리_GUI_설계안.md`.

나누는 기준은 한 줄이다: **ROS 없이 돌고 CLI 도 쓰면 `rdfp.dataset.inspect`, Qt 를 import 하거나
화면 상태를 들면 여기.** 그래서 이 패키지에는 bag · DB 를 읽는 질의가 없고, 전부
`rdfp.dataset.inspect` 를 부른다.

| 모듈 | 역할 |
|---|---|
| `main` | `ros2 run rdfp_gui dataset_gui` 진입점 |
| `context` | 설정 파일 · 경로 · DSN 가리기 (Qt 없음) |
| `snapshot` | bag · DB 를 읽어 화면이 쓰는 모양으로 묶는다 (Qt 없음 — 테스트 가능) |
| `jobs` | CLI 명령 조립 · 진행률 파싱 · `QProcess` 실행기 |
| `workers` | `QThreadPool` 위의 백그라운드 작업과 GUI 스레드 콜백 |
| `store` | 스냅샷을 들고 패널에 변경을 알리는 공유 상태 |
| `panels/` | ① 매핑 · ② 에피소드 · ③ 세션 · ④ 설정 · 하단 작업 패널 · 상세 · 브라우저 · 다이얼로그 |
"""
