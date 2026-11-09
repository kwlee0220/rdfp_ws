# rdfp_gui — 데이터셋 관리 GUI

rosbag2 녹화물(bag) · PostgreSQL 적재 결과(DB) · 그 사이의 **매핑**을 한 화면에 놓고, 적재(import) 와
학습 샘플 export 를 기존 `rdfp` CLI 로 실행하는 PySide6 데스크톱 도구다.
설계: [docs/collect/데이터셋_관리_GUI_설계안.md](../../docs/collect/데이터셋_관리_GUI_설계안.md).

가장 큰 가치는 적재 경로에서 **데이터가 조용히 빠지는 함정**을 보이게 하는 것이다 — 매핑 없는 토픽(✖),
`metadata.yaml` 없는 미완료 세션, `header.stamp` 진단(없음 · 0 · 에피소드 창 밖), 겹치는 에피소드 창,
`image_frames: 0`, 이미지 토픽 + 레코더 등록 충돌.

## 1. 구조

```text
src/rdfp_gui/                      (ament_python, exec_depend rdfp)
  rdfp_gui/
    main.py          ros2 run rdfp_gui dataset_gui [--config <yaml>]
    app.py           MainWindow — 탭 ① 매핑 ② 에피소드 ③ 세션 ④ 설정 + 하단 작업 패널
    context.py       설정 파일 · 경로 · DSN 가리기 (Qt 없음)
    snapshot.py      bag · DB 를 화면 모양으로 묶는다 (Qt 없음)
    store.py         공유 스냅샷 + 변경 시그널
    jobs.py          CLI 명령 조립 · 진행률 파싱 · QProcess 실행기
    workers.py       QThreadPool 백그라운드 + GUI 스레드 콜백
    models.py        QAbstractTableModel
    panels/          mapping · detail · browser · episode · session · config · job · common
    tests/           offscreen Qt 테스트 + 순수 파이썬 테스트
src/rdfp/rdfp/dataset/inspect/     공유 조회 · 점검 (bag_inspect · mapping · db_inspect · preflight)
```

나누는 기준은 한 줄이다 — **ROS 없이 돌고 CLI 도 쓰면 `rdfp.dataset.inspect`, Qt 를 import 하거나 화면
상태를 들면 `rdfp_gui`.** 의존은 `rdfp_gui → rdfp` 한쪽이고 `rdfp/tests/test_layer_boundary.py` 가 역방향을
막는다. GUI 자체는 ROS 를 import 하지 않는다(`rdfp_gui/tests/test_layer_boundary.py`).

**장시간 작업은 전부 서브프로세스다.** Import 는 `ros2 run rdfp import …`, Export 는 `ros2 run rdfp export …`,
에피소드 삭제는 `ros2 run rdfp episode-delete …`, 미완료 세션 복구는 `ros2 bag reindex -s mcap …`. 하단 패널에
**그 명령이 그대로 보이고 복사된다** — 터미널에서 같은 명령을 돌리면 같은 결과가 난다. 적재가 ROS 를
요구하므로 **GUI 는 ROS 를 소싱한 셸에서 띄운다** (조회만 하려면 소싱 없이도 뜬다).

## 2. 설치 · 실행

```bash
pip install --user PySide6-Essentials            # apt 에 없다. Qt 위젯만 있으면 된다 (Addons 불필요)
sudo apt install libxcb-cursor0                  # X11 세션에서 Qt 6.5+ xcb 플러그인이 요구한다 (없으면 창이 안 뜬다)
cd ~/development/ros/rdfp_ws
colcon build --packages-select rdfp rdfp_gui       # rdfp 의 inspect 모듈 · episode-delete 도 함께
source install/setup.bash
ros2 run rdfp_gui dataset_gui --config config/dataset_config.yaml
# 또는
./scripts/run_dataset_gui.sh
```

`--config` 를 생략하면 다른 dataset CLI 와 같이 `$RDFP_HOME/config/dataset_config.yaml` 을 찾는다. DSN 은
설정의 `db.dsn_env`(기본 `RDFP_DB_DSN`) 환경변수에서만 읽고 **화면에 비밀번호를 보이지 않는다.**

## 3. 화면

| 탭 | 보는 것 | 조작 |
|---|---|---|
| **① 매핑** | 왼쪽 bag 토픽(상태 기호 · 타입 · 수) · 오른쪽 DB 테이블(행 수 · 출처 · 들어오는 토픽) · 가운데 매핑 선 · 아래 상세(메시지 정의 / 스키마 / stamp 진단) | 토픽 체크 → **Import…**(에피소드 선택 · 정책 · 사전 점검 · 명령 미리 보기), 데이터 보기(bag 메시지 · DB 행 페이지, 같은 에피소드의 DB 행 수 비교), 전체 stamp 진단 |
| **② 에피소드** | DB 에피소드(테이블별 행 수 · 영상 · 점검) + bag 의 미적재 에피소드(회색) | 상세(점검 · 테이블별 데이터 보기 · metadata), **Export…**(npz, 사전 점검), **삭제…**(행 수 · 파일 목록 보여 주고 id 를 타이핑) |
| **③ 세션 (bag)** | 날짜 → 세션 → split 트리, **미완료 세션 표시**, 세션별 적재 수 | reindex(`metadata.yaml` 복구), 세션 디렉터리 삭제(이름 타이핑) |
| **④ 설정** | `dataset_config.yaml` 텍스트 · Pydantic 검증 · `topics_file` 이 녹화 목록과 같은 파일인지 · 경로 존재 · DB 연결 시험 | 편집 · 저장 · 검증 |

토픽 상태 기호: `●` 매핑됨 · `◇` 특수(`/session` 에피소드 경계, `sensor_msgs/Image` → mp4) · `✖` **매핑 없음**
(적재가 조용히 무시한다) · `△` 매핑은 있는데 stamp 진단에 걸림.

## 4. 구현 범위 (2026-11-04)

설계안 §8 의 **1 · 2 · 3 단계**다. **4 단계(동적 매핑 — 테이블 생성 · 등록 해제 · 테이블 삭제, `topic_tables`)
와 5 단계(LeRobot v3 export)는 미구현**이며 버튼은 비활성으로 두고 이유를 툴팁에 적었다 — 둘 다 설계 확정이
먼저다(설계안 §9 · §6.5). Import 의 토픽 부분 적재는 `import --topics` 로 되지만, 빠진 토픽을 **기존**
에피소드에 채우는 `--only-topics-replace` 는 없다 (사전 점검이 그 사실을 경고한다).

`rdfp` 쪽에 함께 들어간 것: `rdfp.dataset.inspect` 패키지, `import --start-ns/--topics/--on-existing` 와
사전 점검(`preflight.check_import` — CLI 도 같은 함수를 부른다), `episode-delete` 콘솔 스크립트.

## 5. 테스트

```bash
cd src/rdfp_gui && PYTHONPATH=.:../rdfp:$PYTHONPATH python3 -m pytest rdfp_gui/tests -q   # offscreen
cd src/rdfp && PYTHONPATH=.:$PYTHONPATH python3 -m pytest rdfp/dataset/inspect/tests rdfp/tests -q
```

Qt 테스트는 `QT_QPA_PLATFORM=offscreen` 으로 돈다(conftest 가 설정). PySide6 가 없으면 Qt 테스트만 skip 된다.
`setup.cfg` 의 `addopts = -p no:anyio` 는 다른 패키지와 같은 이유로 지우면 안 된다(워크스페이스 CLAUDE.md "Tests").
