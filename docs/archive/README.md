# archive/ — 보관 문서

**갱신하지 않는 문서**를 둔다. 설계가 결정으로 굳어 현행 문서로 흡수됐거나, 인수 기준선이 바뀌어 옛 판이 된 것,
발송이 끝난 벤더 문안, 전제가 무효가 된 설계안이다. 2026-09-21 재편 때 만들었다
([문서_재편_계획.md](문서_재편_계획.md)). 재편 **직전의 원본 그대로**(링크 갱신 전 · PDF 포함)는 저장소 밖
`~/development/ros/rdfp_docs_backup_20260921/` 에 있다.

## 규칙

1. **갱신하지 않는다.** 본문의 "상태" · "아직 없다" · "미결" 서술은 **작성 시점 기준**이고, 지금과 다를 수 있다.
   현행은 각 문서 머리의 "현행 정본" 링크가 가리키는 곳이다.
2. **코드 · 설정에서 archive 를 가리키지 않는다.** 가리키던 곳은 정본으로 바꾼다 (`scripts/check_doc_links.sh` 가
   링크 존재만 검사하므로 방향은 사람이 지킨다).
3. 실측 부록(t₀ 후보 22시행, 영상 인코딩 측정, 펑션베이 3회 실측 등)은 **다시 만들 수 없으므로** 삭제하지 않는다.

## 목록

| 문서 | 무엇이었나 | 현행 정본 |
|---|---|---|
| `learning/학습_데이터_수집_설계.md` | 팔 action 채널 재검토 — 대안 A · A′ · B 평가와 §0 결정 | [../learning/결정_기록.md](../learning/결정_기록.md) |
| `learning/MoveIt_명령_체인과_트윈_데이터_생성.md` | 명령 체인 L0~L4 · 대리 환경 실측 · 대안 M1~M3 | [../learning/학습_샘플_사양.md](../learning/학습_샘플_사양.md) |
| `learning/학습_데이터의_좌표_공간_비교.md` | 관절 대 데카르트 — 저장 · 표현 공간 (106 KB) | 결정은 결정_기록, 보류된 데카르트 단계는 [../learning/열린_결정.md](../learning/열린_결정.md) |
| `learning/ACT_학습_샘플_구성.md` | 경우 A(관절-명령) · B(관절-계획) 샘플 구조 비교 | 학습_샘플_사양 (경우 B 만) |
| `learning/R1_실행_궤적_채널_구현_설계.md` · `R2_연산_이벤트_채널_구현_설계.md` | 두 채널의 구현 설계 · t₀ 실측 · 검증표 | 학습_샘플_사양 §채널 |
| `learning/R4_…` · `Q13_…` · `R8_…` | metadata 규약 · 중력 보상 기록 · 보상 전 명령 — 결정 전 검토 | 열린_결정 |
| `functionbay/펑션베이_미해결_작업.md` | 인수인계 원문 — 3회 실측 사연 · 해소된 블로커 · 정정 대기 항목 | [../backends/functionbay/펑션베이_남은_작업.md](../backends/functionbay/펑션베이_남은_작업.md) |
| `functionbay/펑션베이_인수_체크리스트_이력_20260917.md` | 2026-09-17 재기준선 이전 판 (Crisp 기준선) | [../backends/functionbay/펑션베이_인수_체크리스트.md](../backends/functionbay/펑션베이_인수_체크리스트.md) |
| `functionbay/펑션베이_인수_체크리스트_이력_20261001.md` | 결과를 떼어 내기 직전 판 — 2026-09-17 `r2` 기준선(A·B·C 표) · 09-29 `t1` · 09-30 `r2` 회차 기록(§G) · 회차별 stamp 변동표(§G.1a) | [../backends/functionbay/펑션베이_인수_체크리스트.md](../backends/functionbay/펑션베이_인수_체크리스트.md) (절차) · 결과는 저장소 루트 `checklist_results/` |
| `functionbay/펑션베이_벤더_메일_20260903_정지.md` · `…_20260909_중력.md` | 발송한 문안 | 상태는 [../backends/functionbay/펑션베이_요청_목록.md](../backends/functionbay/펑션베이_요청_목록.md) §0 |
| `learning/1차_목표_진척_기록_20260920.md` | 1차 과제(isaac 쌓기)의 진척표 A0~E3 · 실측 · 결정 D1~D8 — 목표 문서 §3.1 원문 | [../learning/미구현_목록.md](../learning/미구현_목록.md) §0 · [../learning/결정_기록.md](../learning/결정_기록.md) |
| `isaac/isaac_phase_log.md` | Isaac 백엔드 Phase 0~11 단계 계획 · 실측 · 닫힌 결정 | [../backends/isaac/isaac_backend.md](../backends/isaac/isaac_backend.md) |
| `isaac/multi_simulator_backend_design.md` | 멀티 시뮬레이터 launch 재배치 설계안 (§4 미채택) | [../backends/README.md](../backends/README.md) |
| `recorder_initial_requirements.md` | MP4 recorder 노드 최초 요구사항 | [../collect/rdfp_image_recorder_node_guide.md](../collect/rdfp_image_recorder_node_guide.md) |
| `dataset_postproc_verification_runbook.md` | 후처리기 Phase 1~3 실환경 검증 runbook (구 CLI · 구 경로) | [../collect/dataset_postproc_user_guide.md](../collect/dataset_postproc_user_guide.md) |
| `rosbag2_operation_report.md` | rosbag2 장기 저장 운영 방안 보고서 (systemd · 인덱스 · retention 은 미구현) | [../collect/dataset_ingest_design.md](../collect/dataset_ingest_design.md) §0 |
| `leader_gripper_mapping_design.md` | 리더 트리거 → 그리퍼 연속 매핑 — `GripperCommand` 가 숫자를 안 실어 전제 무효 | [../control/gripper_node_design.md](../control/gripper_node_design.md) |
| `문서_재편_계획.md` | 이 재편의 계획서 | — |
