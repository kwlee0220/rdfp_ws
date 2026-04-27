# RdfpImageViewerNode 개발 계획

본 문서는 [prompt.md](prompt.md) 의 1차 요구사항을 기반으로
`RdfpImageViewerNode` 구현을 위한 단계별 작업 계획을 정의한다.

## 진행 상태

- [x] Phase 1 — 부모 클래스에 `_decorate_frame` 훅 추가
- [x] Phase 2 — `RdfpImageViewerNode` 골격 및 세션 구독
- [x] Phase 3 — 오버레이 렌더링
- [x] Phase 4 — 린터·테스트 및 수동 검증 (GUI 실물 확인은 별도)
- [x] Phase 5 — 정리 및 인수인계

## 대상 산출물

- `rdfp/camera/rdfp_image_viewer_node.py` — 신규 노드 파일
- `rdfp/camera/image_viewer_node.py` — 부모에 `_decorate_frame` 훅 추가 (소규모 수정)
- `setup.py` — 엔트리 포인트는 이미 등록됨 (`rdfp_image_viewer_node`), 확인만 수행

## 전제 / 참조

- 세션 메시지: `rdfp_msgs/msg/SessionCommand` ([SessionCommand.msg](../../../rdfp_msgs/msg/SessionCommand.msg))
- 퍼블리셔 QoS: `TRANSIENT_LOCAL / RELIABLE / KEEP_LAST(depth=1)`
  ([session_control_node.py:58-63](../session/session_control_node.py#L58-L63))
- 이미지 토픽은 부모 `ImageViewerNode` 와 동일하게 `image` 이름 사용 (remap 가능)

---

## Phase 1 — 부모 클래스에 렌더 훅 추가

**목표:** 자식이 프레임에 오버레이를 끼워 넣을 수 있도록
`ImageViewerNode` 에 후킹 포인트를 마련한다. 기존 동작은 변경 없음.

**작업 항목**

- [image_viewer_node.py:95-132](image_viewer_node.py#L95-L132) `_on_image` 내부
  `imshow` 직전에 `frame = self._decorate_frame(frame)` 호출 추가
- 기본 구현:
  `def _decorate_frame(self, frame: np.ndarray) -> np.ndarray: return frame`
- 훅 메서드에는 짧은 한국어 주석으로 확장 용도 명시

**완료 조건**

- `colcon build --packages-select rdfp` 성공
- 기존 `image_viewer_node` 실행 시 시각적 동작 변화 없음
- flake8 / pep257 린터 통과

---

## Phase 2 — RdfpImageViewerNode 골격 및 세션 구독

**목표:** 신규 노드 파일을 만들고 세션 토픽 구독·상태 보관까지 구현.
오버레이는 Phase 3 에서 다룬다.

**작업 항목**

- `rdfp_image_viewer_node.py` 생성, `RdfpImageViewerNode(ImageViewerNode)` 선언
- 노드 이름: `rdfp_image_viewer_node`, 윈도우명도 동일 계열로 구분
- 초기 상태 멤버 세팅 (prompt.md 요구사항):
  - `self._session_state: str = 'IDLE'`
  - `self._task_label: str = ''`
- 세션 구독:
  - 토픽명 상수 `_DEFAULT_SESSION_TOPIC = 'session'`
  - QoS: `QoSProfile(depth=1, history=KEEP_LAST, reliability=RELIABLE, durability=TRANSIENT_LOCAL)`
  - 콜백에서 `msg.state`, `msg.task_label` 을 내부 상태로 저장
- `main()` 엔트리 포인트 — 부모 패턴을 그대로 따라 `SingleThreadedExecutor`
  사용 및 `destroy_node` / `try_shutdown` 보호

**완료 조건**

- `ros2 run rdfp rdfp_image_viewer_node` 로 기동 가능
- `ros2 topic echo /session` 과 동일한 상태가 내부 멤버에 반영됨 (로그로 확인)
- 이미지는 표시되나 아직 오버레이 없음

---

## Phase 3 — 오버레이 렌더링

**목표:** `_decorate_frame` 오버라이드로 좌상단 텍스트 및 반투명 배경
사각형을 그린다.

**작업 항목**

- 상태 → 표시 문자열 매핑 함수:
  - `IDLE` → `"<label> (Idle)"`
  - `IN_SESSION` → `"<label> (Ready)"`
  - `IN_EPISODE` → `"<label> (Recording)"`
  - 그 외 → `"<label> (Unknown State)"`
  - `task_label == ''` 이면 `<label>` 자리에 `"No Task"` 사용
- 렌더 상수 정의 (모듈 상단):
  - 폰트: `cv2.FONT_HERSHEY_SIMPLEX`, `scale=0.7`, `thickness=2`
  - 글자색: 흰색 `(255, 255, 255)`
  - 배경색: 검정, 알파 `0.5`
  - 좌상단 여백: 10 px, 텍스트 주변 패딩: 6 px
- 구현 절차:
  1. `cv2.getTextSize` 로 텍스트 박스 크기 계산
  2. 텍스트 박스 영역을 `frame.copy()` 에 `cv2.rectangle` 로 검정 채우기
  3. `cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)` 로 합성
  4. `cv2.putText` 로 최종 텍스트 그리기
- 예외 방어: 오버레이 실패 시 `self.get_logger().warning` 으로 rate-limited 로그
  후 원본 프레임 반환 (표시 자체는 유지)

**완료 조건**

- `session_control_node` 를 함께 실행하여 상태를 바꿀 때 오버레이가 즉시 반영
- 세션 메시지 수신 전 초기 상태에서도 `"No Task (Idle)"` 가 표시됨
- 장시간 실행 시 메모리 사용량이 일정 수준 유지 (누수 없음)

---

## Phase 4 — 린터·테스트 및 수동 검증

**목표:** 프로젝트 규약 준수 확인 및 기능 검증.

**작업 항목**

- 린터: `colcon test --packages-select rdfp` + `colcon test-result --verbose`
  (copyright / flake8 / pep257)
- 코드 컨벤션 점검 (CLAUDE.md 기준):
  - import 순서 (`__future__` → `typing` → 표준 → 서드파티 → 로컬)
  - built-in 타입 힌트 사용 (`list[str]`, `dict[str, int]`, `str | None`)
  - 주석은 한국어, 로깅·예외 메시지는 영어
  - 라인 100 자 원칙, 함수 인자는 한 줄 이어쓰기
- 수동 검증 시나리오:
  1. 이미지 퍼블리셔 없이 기동 — 윈도우가 열리고 이벤트 루프가 멈추지 않음
  2. `remap` 으로 다른 세션 토픽 이름 구독 확인
     (`ros2 run ... --ros-args -r session:=my_session`)
  3. `session_control_node` 에 `start_session` / `start_episode` /
     `stop_episode` / `stop_session` 순차 호출 → 오버레이가
     `Idle → Ready → Recording → Ready → Idle` 로 전이
  4. `task_label` 변경 후 표시 문자열 업데이트 확인
  5. 뷰어를 세션 노드보다 늦게 기동 — `TRANSIENT_LOCAL` 덕에 직전 상태가
     즉시 수신되는지 확인

**완료 조건**

- 모든 린터 테스트 통과
- 수동 검증 5 개 시나리오 모두 기대대로 동작

---

## Phase 5 — 정리 및 인수인계

**목표:** 2 차 요구사항 추가에 대비해 확장 지점을 명확히 남긴다.

**작업 항목**

- 파일 상단 docstring 에 노드 역할·파라미터·토픽·QoS 요약 기술
- 상태 → 표시 문자열 매핑이 확장되기 쉽도록 dict 또는 함수 형태로 유지
- prompt.md 와의 트레이서빌리티를 위해 plan.md 갱신
  (이슈·스코프 변경 시 본 문서를 최신 상태로 유지)
- 필요 시 `launch/panda_mock.launch.py` 에 선택적 포함 여부 검토
  (현 단계에서는 선택 사항, 별도 합의 필요)

**완료 조건**

- 신규 개발자가 docstring 만 읽고 노드 사용/remap 방법을 파악 가능
- 2 차 요구사항 추가 시 수정 지점이 최소 1 ~ 2 개 함수에 국한

---

## 리스크 / 미확정 사항

- **launch 통합:** 현재는 독립 실행만 목표. 전체 스택에 통합할지는
  요구사항 확인 후 결정.
- **해상도 파라미터:** 부모의 `resolution` 파라미터를 그대로 상속하여
  사용. 오버레이 크기와 연동되지는 않으므로 저해상도에서 텍스트가
  상대적으로 커 보일 수 있음 — 필요 시 폰트 스케일 파라미터화 검토.
- **폰트 한글:** 현재 영문만 사용하므로 OpenCV 기본 폰트로 충분.
  한글 오버레이가 추가 요구될 경우 PIL/freetype 경로 도입 필요.
