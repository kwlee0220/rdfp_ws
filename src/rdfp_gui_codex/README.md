# RDFP Dataset Manager (`rdfp_gui_codex`)

rosbag2 MCAP 토픽과 PostgreSQL 테이블을 비교하고, 선택한 데이터를 적재한 뒤 NPZ·LeRobot v3 학습 데이터셋으로 내보내는 PySide6 데스크톱 도구다.

## 1. 패키지 구성

```text
rdfp_gui_codex/
├── package.xml, setup.py, setup.cfg
├── resource/rdfp_gui_codex
├── requirements.txt                 # GUI·DB·MCAP·영상 처리 의존성
├── requirements-lerobot.txt         # LeRobot v3 Export 추가 의존성
├── rdfp_gui_codex/
│   ├── manager_cmd.py               # GUI·요청 파일 CLI 진입점
│   ├── gui/                         # Qt 화면·목록 모델
│   └── management/                  # 조회·매핑·Import·Export·작업 관리
└── tests/                           # 임시 DB·합성 bag·Qt·Export 테스트
```

의존 방향은 `rdfp_gui_codex → rdfp`다. 설정, 메시지 타입별 writer, 에피소드 감지, 레코더 등록과 기존 NPZ 변환은 `rdfp.dataset`의 코드를 사용한다. `rdfp`는 이 패키지를 import하지 않는다. DB 스키마의 원본도 `rdfp`에서 관리한다.

## 2. 소스에서 실행

아래 명령은 `rdfp_ws` 루트에서 실행한다. Python 3.10 이상을 사용하고, 시스템 패키지를 섞지 않는 별도 환경을 만든다.

```bash
uv venv --python /usr/bin/python3 .venv-dataset-manager
uv pip install --python .venv-dataset-manager/bin/python \
  -r src/rdfp_gui_codex/requirements.txt

# LeRobot v3 Export를 사용할 때 추가 설치
uv pip install --python .venv-dataset-manager/bin/python --torch-backend=cpu \
  -r src/rdfp_gui_codex/requirements-lerobot.txt

# DB 접속 정보는 실제 환경에 맞게 지정한다.
export RDFP_DB_DSN='postgresql://rdfp@localhost:5432/rdfp'
./scripts/run_dataset_manager.sh
```

실행 스크립트는 두 패키지의 소스 경로를 연결하고 `config/dataset_config.yaml`을 사용한다. 다른 Python은 `RDFP_MANAGER_PYTHON`, 다른 설정 파일은 `--config`로 지정할 수 있다. 소스 실행에는 `colcon build`나 실행 중인 ROS 노드가 필요하지 않다.

직접 모듈을 실행하려면 두 패키지가 설치되어 있거나 `PYTHONPATH`에 있어야 한다.

```bash
PYTHONPATH=src/rdfp_gui_codex:src/rdfp \
  .venv-dataset-manager/bin/python -m rdfp_gui_codex.manager_cmd \
  --config config/dataset_config.yaml
```

GUI가 없는 환경에서는 `--request /path/to/request.json`으로 같은 서비스를 호출한다. 요청 파일 형식과 자세한 사용법은 [사용자 가이드](../../docs/collect/Dataset_Manager_사용자_가이드.md)를 참고한다.

## 3. ROS 패키지로 실행

`ament_python` 패키지이며 `package.xml`에 `rdfp` 런타임 의존성을 선언했다. 필요한 ROS 환경과 기존 `rdfp` 패키지를 빌드한 상태에서 다음을 실행한다.

```bash
colcon build --symlink-install --packages-select rdfp_gui_codex
source install/setup.bash
ros2 run rdfp_gui_codex dataset_manager --config config/dataset_config.yaml
```

`ros2 run`은 빌드할 때 선택된 Python으로 실행하므로 그 Python에도 `requirements.txt`의 의존성이 있어야 한다. 가상환경의 의존성을 쓰려면 2절의 소스 실행 방법을 이용하는 것이 간단하다. Qt와 LeRobot은 기본 `rdfp` 패키지의 설치 의존성에 추가하지 않는다.

기존 빌드 산출물에 `ros2 run rdfp dataset_manager`가 남아 있을 수 있다. 새 진입점은 **`ros2 run rdfp_gui_codex dataset_manager`**다.

## 4. 테스트

```bash
uv pip install --python .venv-dataset-manager/bin/python pytest
PYTHONPATH=src/rdfp_gui_codex:src/rdfp HF_HUB_OFFLINE=1 \
  .venv-dataset-manager/bin/python -m pytest src/rdfp_gui_codex/tests -q
```

DB 통합 테스트는 별도 PostgreSQL 프로세스와 임시 schema를 사용한다. PostgreSQL 서버 실행 파일이 없으면 해당 테스트를 건너뛴다. LeRobot 미설치 시 LeRobot 통합 테스트만 건너뛴다. Qt 테스트는 화면을 띄우지 않는 offscreen 모드로 실행한다.

기능 범위와 Import·Export 운영 규칙은 [사용자 가이드](../../docs/collect/Dataset_Manager_사용자_가이드.md), 설계 배경은 [design_gpt.md](../../docs/collect/design_gpt.md)에 정리되어 있다.
