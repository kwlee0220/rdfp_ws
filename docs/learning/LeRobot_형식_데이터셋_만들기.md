# LeRobot 형식 데이터셋 만들기

DB 에 적재한 에피소드로 LeRobot v3 형식의 데이터셋을 만드는 방법을 설명한다. LeRobot 형식이 무엇인지, RDFP 데이터를 그대로 넣을 수 없는 이유가 무엇인지, 그 문제를 어떻게 풀어 데이터셋을 만드는지를 차례로 다룬다.

## 목차

1. [개요](#1-개요)
   - [1.1 문서의 범위](#11-문서의-범위)
   - [1.2 핵심 결론](#12-핵심-결론)
2. [LeRobot 데이터셋 형식](#2-lerobot-데이터셋-형식)
   - [2.1 디렉터리 구조](#21-디렉터리-구조)
   - [2.2 프레임 행과 데이터 항목](#22-프레임-행과-데이터-항목)
   - [2.3 로더의 시계열 조회 방식](#23-로더의-시계열-조회-방식)
3. [RDFP 데이터를 그대로 넣을 수 없는 이유](#3-rdfp-데이터를-그대로-넣을-수-없는-이유)
   - [3.1 RDFP 데이터의 시간 구조](#31-rdfp-데이터의-시간-구조)
   - [3.2 충돌 요약](#32-충돌-요약)
   - [3.3 고정 FPS 와 실제 카메라 시각](#33-고정-fps-와-실제-카메라-시각)
   - [3.4 타임스탬프 허용오차](#34-타임스탬프-허용오차)
   - [3.5 영상 저장 방식](#35-영상-저장-방식)
   - [3.6 학습 정본으로 삼지 않는 이유](#36-학습-정본으로-삼지-않는-이유)
4. [해결 방법 — fps 격자 재샘플링](#4-해결-방법--fps-격자-재샘플링)
   - [4.1 전체 흐름](#41-전체-흐름)
   - [4.2 격자 만들기](#42-격자-만들기)
   - [4.3 영상 — 격자 시각 이하의 최근 프레임 선택](#43-영상--격자-시각-이하의-최근-프레임-선택)
   - [4.4 관측과 행동](#44-관측과-행동)
   - [4.5 데이터셋 feature 구성](#45-데이터셋-feature-구성)
   - [4.6 품질 검사와 검증](#46-품질-검사와-검증)
   - [4.7 NPZ 정본과의 차이](#47-npz-정본과의-차이)
5. [핵심 함수와 클래스](#5-핵심-함수와-클래스)
   - [5.1 `export_lerobot_v3`](#51-export_lerobot_v3)
   - [5.2 `grid_frames`](#52-grid_frames)
   - [5.3 `_ForwardDecoder`](#53-_forwarddecoder)
   - [5.4 함께 쓰는 보조 함수](#54-함께-쓰는-보조-함수)
   - [5.5 사용하는 LeRobot API](#55-사용하는-lerobot-api)
6. [사용 예제](#6-사용-예제)
   - [6.1 실행 환경 준비](#61-실행-환경-준비)
   - [6.2 fps 와 시차 한도 고르기](#62-fps-와-시차-한도-고르기)
   - [6.3 명령줄에서 export](#63-명령줄에서-export)
   - [6.4 GUI 에서 export](#64-gui-에서-export)
   - [6.5 만든 데이터셋 읽기](#65-만든-데이터셋-읽기)
   - [6.6 manifest 확인](#66-manifest-확인)
7. [문제 대응](#7-문제-대응)
   - [7.1 증상별 대응](#71-증상별-대응)
8. [코드 위치](#8-코드-위치)

---

## 1. 개요

### 1.1 문서의 범위

이 문서는 적재가 끝난 DB(PostgreSQL)와 레코더 mp4 에서 LeRobot v3 데이터셋을 만드는 경로를 다룬다. 수집과 적재 절차는 [학습_데이터_파이프라인_개요.md](../collect/학습_데이터_파이프라인_개요.md) 를 따른다. 학습에 사용하는 기준 데이터셋(이하 학습 정본)은 NPZ 형식이며 [저장_형식.md](저장_형식.md) 가 정의한다.

### 1.2 핵심 결론

| 질문 | 답 |
|---|---|
| LeRobot 형식으로 만들 수 있는가 | 있다. `export --format lerobot_v3` 가 공식 `lerobot==0.4.4` writer 로 v3.0 데이터셋을 쓴다 |
| 그대로 넣을 수 없는 이유 | LeRobot 은 프레임이 고정 fps 격자에 있다고 전제하지만, RDFP 의 프레임 시각은 실제 카메라 stamp 라 간격이 고르지 않다 |
| 해결 방법 | 사용자가 고른 fps 격자 위에서 영상 · 관측 · 행동을 다시 샘플링한다 |
| 변환에 따른 영향 | 영상과 레이블의 시간 위상이 최대 `max_skew_sec` 만큼 어긋나고, 영상이 다시 인코딩된다 |
| 용도 | 다른 도구와 데이터를 주고받기 위한 교환용 파생물이다. 학습 정본은 계속 NPZ 다(결정 28 · 39) |

---

## 2. LeRobot 데이터셋 형식

LeRobot 은 Hugging Face 가 관리하는 로봇 학습 라이브러리이며, 데이터셋 형식과 정책 모델(ACT 등)을 함께 제공한다. 이 시스템은 `lerobot==0.4.4` 를 쓰며, 이 버전이 쓰는 데이터셋 형식은 `codebase_version: v3.0` 이다.

### 2.1 디렉터리 구조

export 의 출력 경로(`--out`) 아래에는 LeRobot 데이터셋 루트인 `dataset/` 과 RDFP 가 남기는 `manifest.json` 이 함께 생긴다.

```
<--out 경로>/
├── manifest.json        # RDFP 기록 — 요청 · 채택 · 제외 에피소드, 재샘플 조건, 파일별 sha256
└── dataset/             # LeRobot 데이터셋 루트 (LeRobotDataset 의 root)
```

v3.0 은 에피소드마다 파일을 따로 두지 않고, 여러 에피소드를 크기 한도(데이터 100 MB, 영상 200 MB)까지 한 파일에 이어 붙인다. 에피소드 경계는 메타데이터가 행 번호와 영상 시각 오프셋으로 기록한다.

```
dataset/
├── meta/
│   ├── info.json                  # fps · feature 정의 · 경로 규칙 · codebase_version
│   ├── stats.json                 # feature 별 통계
│   ├── tasks.parquet              # 과제 문자열 목록
│   └── episodes/chunk-000/file-000.parquet   # 에피소드별 행 범위 · 영상 오프셋
├── data/chunk-000/file-000.parquet           # 프레임 행 (여러 에피소드)
└── videos/observation.images.camera/chunk-000/file-000.mp4   # 이어 붙인 영상
```

### 2.2 프레임 행과 데이터 항목

데이터셋은 프레임 하나를 행 하나로 저장한다. 행에는 데이터를 만드는 쪽이 정의한 데이터 항목(feature)과 writer 가 채우는 색인 열이 함께 들어간다. 이 문서의 절차에서 데이터 항목을 정의하고 값을 넣는 쪽은 RDFP exporter 다.

| 열 | 기록 주체 | 내용 |
|---|---|---|
| `observation.state` | RDFP exporter | 관측 벡터 |
| `action` | RDFP exporter | 그 시각의 행동 벡터 한 행 |
| `observation.images.<key>` | RDFP exporter | 영상 데이터 항목. 행에는 값이 없고 영상 파일에서 시각으로 찾는다 |
| `frame_index` | writer | 에피소드 안의 프레임 번호 |
| `timestamp` | writer (또는 RDFP exporter) | 따로 주지 않으면 `frame_index / fps` |
| `episode_index` · `index` · `task_index` | writer | 에피소드 번호 · 전체 행 번호 · 과제 번호 |

`timestamp` 를 생략하면 writer 는 `frame_index / fps` 를 기록한다. 사용자 지정 시각을 저장할 수도 있지만, 로더는 `delta_timestamps` 를 FPS 기반 행 오프셋으로 변환하므로 불규칙한 시간 간격을 그대로 반영하지 못한다.

### 2.3 로더의 시계열 조회 방식

학습 로더(`LeRobotDataset`)는 `delta_timestamps` 로 현재 행 앞뒤의 값을 함께 조회한다. ACT 의 action chunk 도 이렇게 만든다.

| 단계 | 구현 | 의미 |
|---|---|---|
| 시차 검사 | `check_delta_timestamps(delta_timestamps, fps, tolerance_s)` | 모든 시차가 `1/fps` 의 정수배인지 확인한다 |
| 행 오프셋 변환 | `get_delta_indices`: `[round(d * fps) for d in delta_ts]` | 초 단위 시차를 행 오프셋으로 바꾼다 |
| 값 조회 | `query = 현재 행 + 오프셋` | 행을 이어 붙여 시계열을 만든다. 에피소드 밖은 `<key>_is_pad` 로 표시한다 |
| 영상 조회 | 조회할 행의 `timestamp` 에 영상 안의 에피소드 시작 오프셋(`from_timestamp`)을 더한 시각의 프레임을 디코드하고, 그 차이가 `tolerance_s`(기본 `1e-4` 초) 안인지 확인한다 | 넘으면 예외로 멈춘다 |

즉 로더는 "행 간격 = 시간 간격 = `1/fps`" 를 전제로 시계열을 조회한다.

---

## 3. RDFP 데이터를 그대로 넣을 수 없는 이유

### 3.1 RDFP 데이터의 시간 구조

RDFP 는 학습 샘플의 시각을 카메라 프레임에서 얻는다. 이것을 frame clock 이라 부르며, 프레임 하나가 학습 샘플 하나다. 다른 채널은 모두 이 시각에 맞춰 계산한다([학습_샘플_사양.md §3.2](학습_샘플_사양.md#32-frame-clock-과-재표본화)).

| 요소 | RDFP 의 방식 |
|---|---|
| 샘플 시각 | 레코더 sidecar 에 남은 실제 카메라 `header.stamp`. 고정 격자가 아니다 |
| 관측 | `/joint_states` 를 샘플 시각에서 선형 보간하고, 그리퍼 폭은 최근접 값을 쓴다 |
| 행동 | 레이블 함수(`plan(τ)` 또는 관절-명령 보간)를 `τ_j = t + Δ + j × step` 에서 평가한 chunk 를 미리 계산해 저장한다 |
| 영상 | 레코더가 녹화한 H.264 mp4 를 에피소드마다 하나씩 그대로 둔다 |

### 3.2 충돌 요약

| LeRobot 의 전제 | RDFP 의 실제 | 그대로 넣으면 생기는 일 |
|---|---|---|
| 행 간격이 정확히 `1/fps` | 프레임 간격이 고르지 않다 | 행 오프셋으로 만든 chunk 가 실제 시각과 어긋난다 |
| 행동 시차는 `1/fps` 의 정수배여야 한다 | 관절-명령의 행동 간격은 20 ms 이고, 카메라 프레임 간격은 약 100 ms 다 | 10 FPS 에서는 20 ms 간격의 시차 요청이 거부된다. 대신 연속 행을 쓰면 행동 간격이 100 ms 로 늘어난다 |
| 영상 조회 시각과 프레임 시각의 차이가 0.1 ms 이내 | 프레임 간격 편차가 수십 ms | 행 시각과 영상 재생 시각을 맞추지 않으면 영상 조회에서 예외가 난다 |
| 영상을 다시 인코딩해 이어 붙인다 | 에피소드별 H.264 mp4 를 원본으로 유지한다 | 원본 영상을 그대로 쓸 수 없다 |

### 3.3 고정 FPS 와 실제 카메라 시각

실측한 카메라 프레임률은 설정값과 다르고, 회차마다도 다르다. 아래 값은 [저장_형식.md §2.2](저장_형식.md#22-단일-fps-규약과의-충돌) 에 정리된 측정 결과이며, 수집 환경과 회차에 따라 달라진다.

| 백엔드 | 설정 프레임률 | 실측 | 측정 조건 |
|---|---|---|---|
| 펑션베이 | 10 Hz | 9.54 ~ 9.60 Hz (회차마다 변동) | `r2` scene 수집 회차들 |
| isaac | 10 fps | 중앙값 간격 100 ms 이지만 평균 약 5 Hz. 렌더링 누락으로 최대 1.3 ~ 2.1 s 공백 | isaac 수집 에피소드([학습_데이터_파이프라인_개요.md](../collect/학습_데이터_파이프라인_개요.md)) |

#### 3.3.1 관절-계획 레이블

관절-계획 레이블의 행동 간격은 100 ms 라 명목상 프레임 간격과 같다. 그러나 100 ms 간격을 의도하고 10개 행 뒤의 값을 조회하더라도, 실제 행 간격이 105 ms 이면 그 값은 1.0 초가 아니라 1.05 초 뒤의 상태를 나타낸다. 이러한 차이는 행 오프셋 변환 자체로는 보정되지 않으며, 의도한 행동 시각과 레이블 시각의 불일치를 유발한다. 로더의 검사는 시차가 `1/fps` 의 정수배인지만 보므로 이 불일치를 오류로 알리지 않는다.

#### 3.3.2 관절-명령 레이블

관절-명령 레이블의 행동 간격은 20 ms 다. 이 간격은 10 FPS 데이터셋의 행 간격인 100 ms 로 표현할 수 없으며, 기본 허용오차에서는 `check_delta_timestamps()` 가 이를 거부한다. 검사를 통과하도록 허용오차를 늘리더라도 `round(d × fps)` 가 여러 시차를 같은 행으로 반올림하므로(0, 0, 0, 1, 1, …) 20 ms 간격의 정보를 복원할 수 없다. 데이터셋을 50 FPS 로 만들면 행 간격은 맞지만, 영상 행이 다섯 배로 늘어나 같은 프레임을 반복하게 된다.

### 3.4 타임스탬프 허용오차

로더는 행에 저장된 시각과 영상 안의 에피소드 시작 오프셋으로 조회 시각을 계산한다. 이 시각과 디코드한 프레임 시각의 차이가 `tolerance_s`(기본 0.1 ms)를 넘으면 예외가 발생한다. writer 는 영상을 `fps` 로 인코딩하므로, 카메라 stamp 를 행 시각으로 저장하려면 시간 원점과 프레임 간격이 영상의 재생 시각과 일치하는지 확인해야 한다. 간격이 불규칙한 실제 stamp 는 이 조건을 만족하기 어렵다.

### 3.5 영상 저장 방식

LeRobot writer 는 프레임 이미지를 받아 영상을 새로 인코딩하고(기본 코덱 `libsvtav1`), 여러 에피소드를 한 파일에 이어 붙인다. RDFP 는 수집 시 생성한 H.264 영상을 원본으로 보관하고, 학습 정본을 만드는 과정에서 추가로 재인코딩하지 않는다. 측정 결과 AV1 은 디코드가 무거워 무작위 접근이 오히려 느렸다([저장_형식.md 부록 B](저장_형식.md#부록-b-영상-인코딩-및-디코더-성능-측정-2026-09-19)).

### 3.6 학습 정본으로 삼지 않는 이유

고정 격자로 다시 샘플링하면 위의 충돌은 모두 사라진다. 대신 영상과 레이블의 시간 위상이 어긋난다. 격자 시각과 촬영 시각이 일치하지 않을 수 있으므로, 현재 구현은 격자 시각 이하의 가장 최근 프레임을 선택한다. RDFP 는 실제 카메라 시각을 유지하는 편이 정확하다고 보고, 학습에는 chunk 를 미리 계산한 NPZ 를 쓴다(결정 28). 정책 모델 `ACTPolicy` 는 데이터셋 형식과 무관하게 배치 `dict` 만 받으므로, LeRobot 데이터셋 형식 없이도 LeRobot 의 ACT 를 그대로 쓸 수 있다([저장_형식.md §3.4](저장_형식.md#34-lerobot-형식-미채택-및-actpolicy-채택-근거)).

---

## 4. 해결 방법 — fps 격자 재샘플링

LeRobot 형식이 필요한 경우는 다른 도구와 데이터를 주고받을 때다. 이때는 위상 오차를 받아들이고, 사용자가 고른 fps 격자 위에서 모든 채널을 다시 샘플링한다(결정 39).

### 4.1 전체 흐름

```mermaid
flowchart LR
    DB[("PostgreSQL<br/>image_frames · joint_states<br/>motion_* · gripper_*")] --> FC["load_frame_clock<br/>실제 카메라 시각"]
    FC --> GRID["grid_frames<br/>fps 격자 · 최근 프레임 · 시차 검사"]
    MP4["레코더 mp4"] --> DEC["_ForwardDecoder<br/>앞으로만 디코드"]
    GRID --> DEC
    DB --> OBS["qpos(t_i)"]
    DB --> ACT["label.at(t_i) + one_hot(t_i)"]
    GRID --> OBS
    GRID --> ACT
    DEC --> ADD["LeRobotDataset.add_frame"]
    OBS --> ADD
    ACT --> ADD
    ADD --> SAVE["save_episode → finalize"]
    SAVE --> CHECK["공식 로더로 다시 읽어 검증"]
    CHECK --> PUB["manifest.json 작성 후 원자적 게시"]
```

NPZ 가 아니라 DB 에서 직접 만든다. NPZ 의 chunk 는 실제 카메라 시각 기준이라 격자 시각의 값을 꺼낼 수 없기 때문이다. 대신 NPZ export 와 같은 레이블 함수와 관측 함수를 격자 시각에서 평가한다.

### 4.2 격자 만들기

에피소드마다 격자의 시작과 끝을 다음처럼 정한다.

| 값 | 정의 |
|---|---|
| `start` | `max(에피소드 시작 시각, 첫 프레임 시각)` (나노초) |
| `end` | `min(에피소드 종료 시각, 마지막 프레임 시각 + 1)` (나노초) |
| 격자 시각 | `stamp_ns_i = start + (i × 10⁹) // fps` (나노초 정수) |
| 범위 | `end` 미만인 점만 생성한다 |

따라서 에피소드 종료 시각은 제외하고, 마지막 프레임 시각은 격자와 일치하면 포함한다. 관측과 행동을 계산할 때는 이 값을 초 단위의 `t_i = stamp_ns_i / 10⁹` 로 바꿔 쓴다. 이후 절의 `t_i` 는 이 초 단위 시각이다. 첫 프레임보다 앞에서 시작하지 않으므로 모든 격자 시각에는 그 이하의 프레임이 반드시 있다.

### 4.3 영상 — 격자 시각 이하의 최근 프레임 선택

격자 시각마다 그 시각 이하의 가장 최근 프레임을 고른다. 격자 시각보다 나중에 촬영된 프레임은 쓰지 않는다.

```
카메라 프레임   F0        F1             F2   F3                    F4
시간 ──────────●─────────●──────────────●────●─────────────────────●──────▶
격자 (10 fps)  t0    t1    t2    t3    t4    t5    t6    t7    t8    t9
고른 프레임    F0    F0    F1    F1    F1    F3    F3    F3    F3    F4
                                                   └── 시차가 max_skew_sec 를 넘으면 예외
```

| 처리 | 동작 |
|---|---|
| 같은 프레임 반복 | 프레임이 격자보다 성기면 같은 프레임을 여러 행에 쓴다. 반복 수는 `manifest.json` 의 `repeated_frames` 에 남는다 |
| 시차 한도 | 격자 시각과 고른 프레임의 차이가 `max_skew_sec`(기본 0.2 s)를 넘으면 공백을 메우지 않고 `ExportError` 를 낸다 |
| 시각 검사 | 프레임 시각이 비었거나 엄격히 증가하지 않으면 `ExportError` 를 낸다 |
| 디코드 | 격자에서 고른 프레임 번호가 단조 증가하므로 PyAV 로 앞으로만 디코드한다. 탐색이 필요 없다 |

시간 허용치가 두 군데 나오는데, 적용 단계와 의미가 다르다.

| 값 | 적용 단계 | 의미 | 기본값 |
|---|---|---|---|
| `max_skew_sec` | RDFP export (데이터셋을 만들 때) | 격자 시각과 원본 영상에서 고른 프레임의 촬영 시각 사이에 허용하는 차이 | 0.2 s |
| `tolerance_s` | LeRobot 로더 (데이터셋을 읽을 때) | 조회 시각과 재인코딩된 영상에서 디코드한 프레임 시각 사이에 허용하는 차이 | 0.1 ms |

export 가 영상을 격자 시각에 맞춰 다시 인코딩하므로, 만든 데이터셋은 `tolerance_s` 검사를 통과한다. 위상 오차는 `max_skew_sec` 단계에서 이미 데이터에 들어간 것이다.

### 4.4 관측과 행동

| 항목 | 격자 시각 `t_i` 에서의 값 | 사용하는 함수 |
|---|---|---|
| 팔 관측 | `/joint_states` 를 선형 보간 | `qpos(joints, widths, t)` |
| 그리퍼 관측 | 그리퍼 폭의 최근접 값. 없으면 `NaN` | `qpos` 에 포함 |
| 팔 행동 | 레이블 함수를 `t_i` 에서 평가한 한 행. 관절-계획이면 `plan(t_i)`, 관절-명령이면 명령 보간 | `label_fn.at(t)` |
| 그리퍼 행동 | `t_i` 이전 마지막 그리퍼 명령의 one-hot (`open` · `close` · `grasp`) | `grip.one_hot(t)` |

행동은 chunk 가 아니라 격자 시각의 한 행 `a(t_i)` 이다. chunk 는 데이터셋을 읽는 쪽이 `delta_timestamps` 로 만든다. 관측이나 행동에 `NaN` · `inf` 가 하나라도 있으면 `ExportError` 로 멈춘다.

### 4.5 데이터셋 feature 구성

| feature | dtype | shape | 이름 |
|---|---|---|---|
| `observation.state` | `float32` | (관절 수 + 1,) | 팔 관절 이름들, `gripper_width_m` |
| `action` | `float32` | (관절 수 + 3,) | 팔 관절 이름들, `open`, `close`, `grasp` |
| `observation.images.camera` | `video` | (높이, 너비, 3) | `height`, `width`, `channels` |

데이터셋 정보는 `repo_id='local/rdfp'`, `robot_type='panda'` 이고, 프레임의 `task` 는 에피소드의 `sessions.task_label` 을 쓴다(없으면 `unspecified`). 관절 순서는 첫 에피소드의 행동 채널에서 가져오며, 다른 에피소드의 관절 순서가 다르면 `ExportError` 를 낸다.

### 4.6 품질 검사와 검증

| 단계 | 동작 |
|---|---|
| 정상 종료를 가정한 에피소드 | 종료 이벤트가 누락되어 정상 종료로 가정한 연산(`assumed_full_duration`)이 있는 에피소드는 기본으로 뺀다. `--keep-untrustworthy` 로 포함한다 |
| 재읽기 검증 | 기록 완료 후 공식 로더(`LeRobotDataset(..., video_backend='pyav')`)로 데이터셋을 다시 열어 전체 프레임 수를 확인하고, 각 에피소드의 첫 행과 마지막 행을 조회해 읽기 가능 여부를 검증한다 |
| 원자적 게시 | 옆의 임시 디렉터리(`.<이름>.partial-*`)에 쓰고, 작업이 성공한 경우에만 출력 경로로 이름을 바꾼다. 일반적인 예외가 나면 임시 디렉터리를 정리한다. 프로세스가 강제 종료되면 임시 디렉터리가 남을 수 있다 |
| manifest | 요청 · 채택 · 제외 에피소드, 파일별 sha256, 재샘플 조건, 에피소드별 대응표를 `manifest.json` 에 남긴다 |

### 4.7 NPZ 정본과의 차이

| 항목 | NPZ (학습 정본) | LeRobot v3 (교환용) |
|---|---|---|
| 샘플 시각 | 실제 카메라 stamp | 고정 fps 격자 |
| 영상 | 원본 mp4 의 프레임을 그대로 가리킨다 | 격자 시각 직전 프레임을 다시 인코딩한다 |
| 행동 | 미리 계산한 chunk (`k × 관절`) | 격자 시각의 한 행. chunk 는 로더가 만든다 |
| 영상 · 레이블 위상 | 일치 | 최대 `max_skew_sec` 어긋남 |
| 미래 행동의 패딩 기준 | 요청 시각이 원래 에피소드 종료 시각을 넘는지로 판단한다 | 요청 행이 저장된 에피소드의 행 범위를 벗어나는지로 판단하고, 범위 밖 조회값은 경계 행의 값으로 대체한다 |
| 정규화 통계 | `metadata.json` | `meta/stats.json` (writer 가 계산) |
| 만드는 곳 | ROS 파이썬 또는 순수 파이썬 | `lerobot==0.4.4` 가 있는 파이썬 |

두 결과는 수치가 같지 않다. 무엇을 어떻게 바꿨는지는 LeRobot 쪽 `manifest.json` 의 `resampling` · `action_meaning` 항목이 설명한다.

---

## 5. 핵심 함수와 클래스

모두 `rdfp/dataset/export/lerobot_v3.py` 에 있다. `lerobot` 은 함수 안에서 늦게 import 하므로, 이 모듈은 lerobot 이 없는 환경에서도 import 된다.

### 5.1 `export_lerobot_v3`

| 항목 | 내용 |
|---|---|
| signature | `export_lerobot_v3(conn, episode_ids: Sequence[int], *, out_dir: str, fps: int, max_skew_sec: float = 0.2, label: str = LABEL_PLAN, camera_topic: str = '', joint_topic: str = DEFAULT_JOINT_TOPIC, command_topic: str = DEFAULT_COMMAND_TOPIC, media_roots: Sequence[str] = (), repo_id: str = 'local/rdfp', skip_untrustworthy: bool = True) -> dict` |
| 입력 검사 | 에피소드가 없거나, `fps` 가 1 ~ 120 밖이거나, `max_skew_sec` 가 양수가 아니면 DB 를 건드리기 전에 `ExportError` 를 낸다 |
| 수행 내용 | 에피소드마다 frame clock · 관절 · 레이블 · 그리퍼 데이터를 읽고, `grid_frames` 가 준 격자 시각마다 영상 프레임 · `observation.state` · `action` 을 만들어 `add_frame` 한다. 에피소드 끝에 `save_episode`, 모두 끝나면 `finalize` 하고 다시 읽어 검증한 뒤 manifest 를 쓰고 게시한다 |
| 반환값 | `{'output': 출력 경로, 'kept': [채택 에피소드], 'excluded': [{'episode_id', 'reason', 'count'}], 'frames': 전체 프레임 수}` |
| 실패 | 입력 조건과 데이터 검증 실패에는 `ExportError` 를 쓴다. DB 접근, 영상 처리, LeRobot API 에서 발생한 예외는 원래 형식 그대로 전달될 수 있다. 어느 경우든 출력 경로로 게시되지 않는다 |

### 5.2 `grid_frames`

| 항목 | 내용 |
|---|---|
| signature | `grid_frames(clock: FrameClock, fps: int, max_skew_sec: float) -> Iterator[tuple]` |
| 수행 내용 | 격자 시각마다 `(stamp_ns, frame_index, skew_sec)` 를 낸다. `frame_index` 는 그 시각 이전의 가장 가까운 프레임의 mp4 안 번호이고, `skew_sec` 는 격자 시각과 그 프레임 시각의 차이다 |
| 예외 | 프레임 시각이 비었거나 엄격히 증가하지 않을 때, 시차가 `max_skew_sec` 를 넘을 때 `ExportError` |

### 5.3 `_ForwardDecoder`

| 메서드 | signature | 수행 내용 |
|---|---|---|
| 생성자 | `_ForwardDecoder(path: str)` | PyAV 로 mp4 를 열고 첫 영상 스트림의 디코더를 준비한다 |
| `get` | `get(index: int) -> np.ndarray` | 요청한 프레임 번호까지 앞으로 디코드해 RGB 배열을 돌려준다. 같은 번호를 다시 요청하면 마지막 프레임을 그대로 준다. 번호가 줄어들거나 영상이 먼저 끝나면 `ExportError` |
| `close` | `close() -> None` | 컨테이너를 닫는다 |

### 5.4 함께 쓰는 보조 함수

NPZ export 와 같은 함수를 쓰므로 관측과 레이블의 정의가 두 형식에서 같다.

| 함수 | 위치 | signature | 수행 내용 |
|---|---|---|---|
| `load_frame_clock` | `export/frame_clock.py` | `load_frame_clock(conn, episode_id: int, *, topic: Optional[str] = None) -> FrameClock` | 에피소드의 카메라 프레임 시각열과 에피소드 경계를 읽는다. 카메라가 여럿이면 `topic` 을 요구한다 |
| `FrameClock.interval_stats` | `export/frame_clock.py` | `interval_stats() -> dict` | 프레임 간격의 `count` · `min` · `max` · `mean` · `hz`. fps 와 시차 한도를 고를 때 본다 |
| `qpos` | `export/observation.py` | `qpos(joints, widths, t: float) -> tuple` | 팔 관절 선형 보간과 그리퍼 폭 최근접 값을 이은 관측 벡터 |
| `default_arm_joints` | `export/writer.py` | `default_arm_joints(conn, episode_id: int, *, label: str, command_topic: str) -> tuple` | 행동 채널에서 팔 관절 순서를 얻는다 |
| `staged_output` | `export/writer.py` | `staged_output(out_dir: str)` (context manager) | 임시 디렉터리에 쓰고 성공하면 이름을 바꿔 게시한다. 내용이 있는 출력 경로는 거부한다 |
| `write_manifest` | `export/writer.py` | `write_manifest(root: str, *, fmt: str, media_root: str, requested, kept, excluded, extra: Optional[dict] = None) -> dict` | 파일별 sha256 과 요청 · 채택 · 제외 내역을 `manifest.json` 으로 쓴다 |

### 5.5 사용하는 LeRobot API

| API | 쓰는 방식 |
|---|---|
| `LeRobotDataset.create(repo_id, fps, features, root, robot_type, use_videos)` | 첫 채택 에피소드에서 한 번 만든다. `root` 는 존재하지 않는 디렉터리여야 한다 |
| `add_frame(frame: dict)` | 격자 시각마다 `observation.state` · `action` · 영상 배열 · `task` 를 넣는다. `timestamp` 는 주지 않아 writer 가 `frame_index / fps` 로 채운다 |
| `save_episode()` | 에피소드 하나를 데이터 · 영상 파일에 기록한다. 이때 영상을 인코딩한다 |
| `finalize()` | 메타데이터를 마무리한다 |
| `LeRobotDataset(repo_id, root=..., video_backend='pyav')` | 검증을 위해 다시 연다 |

---

## 6. 사용 예제

### 6.1 실행 환경 준비

현재 구성된 ROS 파이썬 환경에는 lerobot 이 설치되어 있지 않다. 이 환경에서 `ros2 run rdfp export --format lerobot_v3` 를 실행하면 의존성을 불러오는 단계에서 `ExportError` 로 멈추므로, `lerobot==0.4.4` 와 `psycopg` · `pydantic` · `pyyaml` 이 설치된 별도 파이썬 환경을 쓴다. 이 환경에서 export 모듈을 직접 실행하고, `PYTHONPATH` 로 워크스페이스의 `rdfp` 소스를 보게 한다. 아래는 `rdfp-train` 의 venv 를 쓰는 예다.

```bash
export RDFP_DB_DSN='<DSN>'     # dataset_config.yaml 의 db.dsn_env 가 가리키는 변수
# GUI 가 쓰는 실행 명령. 줄바꿈 없이, 홈 경로를 확장한 값으로 지정한다
export RDFP_LEROBOT_PYTHON="uv run --no-project --python $HOME/development/rdfp/rdfp-train/.venv/bin/python --with psycopg[binary] --with pydantic --with pyyaml python"
```

`RDFP_LEROBOT_PYTHON` 은 GUI 가 `shlex.split()` 으로 나눠 명령 접두로 쓰는 값이다. 작은따옴표 안에 줄바꿈과 `\` 를 넣으면 줄바꿈 문자가 별도 인자로 남고, 문자열 안의 `~` 는 홈 디렉터리로 확장되지 않으므로 위처럼 한 줄로 쓴다. 명령줄에서 직접 실행할 때는 이 변수를 쓰지 않고 §6.3 처럼 명령을 그대로 적는다.

### 6.2 fps 와 시차 한도 고르기

출력 FPS 와 `max_skew_sec` 는 카메라의 명목상 설정값만으로 정하지 않고, 실측 프레임 간격과 필요한 행동 간격을 함께 고려해 정한다. 다음은 에피소드별 간격 통계를 출력하는 예다.

```python
import os
import psycopg
from rdfp.dataset.export.frame_clock import load_frame_clock

with psycopg.connect(os.environ['RDFP_DB_DSN']) as conn:
    for ep in (1, 2, 3):
        s = load_frame_clock(conn, ep).interval_stats()
        print(ep, f"hz={s['hz']:.2f} mean={s['mean']:.3f} max={s['max']:.3f}")
```

| 실측 | 고르는 방법 |
|---|---|
| `max` 가 `max_skew_sec` 보다 크다 | 허용 시차를 넘을 가능성이 있다. 실제 실패 여부는 선택한 FPS 의 각 격자 시각에서 계산한 시차로 정해지므로, 예를 들어 프레임 시각이 0 s · 0.21 s 이고 10 FPS · 한도 0.2 s 이면 격자 시차가 0 · 0.1 · 0.2 s 라 통과한다. 실패하면 한도를 올리거나 에피소드를 뺀다 |
| `hz` 가 fps 보다 한참 낮다 | 같은 프레임이 여러 행에 반복된다. fps 를 실측 주기에 가깝게 낮춘다 |
| 학습 쪽 chunk 설계 | 행동 간격은 `1/fps` 의 정수배로 지정해야 한다. 예를 들어 100 ms 간격은 10 FPS 에서 한 행, 50 FPS 에서 다섯 행 간격으로 표현할 수 있다 |

참고로 구현 당시(2026-11-23) 개발 DB 의 isaac 에피소드 세 개를 10 FPS 로 export 한 측정에서는([데이터셋_관리_GUI_설계안.md §6.5](../collect/데이터셋_관리_GUI_설계안.md)), 중앙 간격이 0.1 s 였지만 최대 1.3 s 공백이 있어 기본 한도 0.2 s 로는 export 가 실패했다. 한도를 1.5 s 로 풀면 544 프레임이 나왔고, 그중 약 절반이 같은 프레임의 반복이었다. 실효 카메라 프레임률이 약 5 Hz 였기 때문이다. 에피소드 번호와 프레임 수는 DB 에 따라 달라진다.

### 6.3 명령줄에서 export

```bash
cd ~/development/ros/rdfp_ws
PYTHONPATH=src/rdfp uv run --no-project \
    --python ~/development/rdfp/rdfp-train/.venv/bin/python \
    --with 'psycopg[binary]' --with pydantic --with pyyaml \
    python -m rdfp.dataset.export_cmd \
    --config config/dataset_config.yaml \
    --out /data/rdfp/datasets/lerobot_v1 \
    --format lerobot_v3 --fps 10 --max-skew-sec 0.2 \
    --label joint_plan --episodes 1-20
```

| 인자 | 기본값 | 뜻 |
|---|---|---|
| `--format` | `npz` | `lerobot_v3` 로 LeRobot 데이터셋을 만든다 |
| `--fps` | 10 | 격자 fps |
| `--max-skew-sec` | 0.2 | 격자 시각과 영상 프레임의 최대 시차 |
| `--label` | `joint_plan` | `joint_plan`(관절-계획) 또는 `joint_command`(관절-명령) |
| `--episodes` | 영상이 있는 전부 | 예: `1-3,7` |
| `--camera-topic` | (자동) | 카메라가 여럿이면 필수 |
| `--keep-untrustworthy` | 꺼짐 | 정상 종료를 가정한 에피소드도 넣는다 |

성공하면 채택한 에피소드 수와 프레임 수를 출력하고, 제외한 에피소드를 이유와 함께 한 줄씩 보여 준다. `--chunk-len` · `--delta-sec` · `--step-sec` · `--copy-media` 는 NPZ 전용이라 이 형식에서는 쓰지 않는다.

### 6.4 GUI 에서 export

데이터셋 관리 GUI(`ros2 run rdfp_gui dataset_gui`)의 에피소드 패널에서 export 형식을 LeRobot v3 로 고르면, GUI 가 `RDFP_LEROBOT_PYTHON` 의 파이썬으로 위 명령을 만들어 실행한다. 이 변수가 없으면 실행 전에 오류를 낸다.

### 6.5 만든 데이터셋 읽기

출력 경로 아래 `dataset/` 이 LeRobot 데이터셋 루트다. ACT 처럼 chunk 가 필요하면 `delta_timestamps` 로 요청한다. 시차는 `1/fps` 의 정수배여야 한다.

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

FPS, K = 10, 20
ds = LeRobotDataset(
    'local/rdfp', root='/data/rdfp/datasets/lerobot_v1/dataset', video_backend='pyav',
    delta_timestamps={'action': [(j + 1) / FPS for j in range(K)]})   # Δ = 0.1 s, step = 0.1 s

sample = ds[0]
print(sample['observation.state'].shape, sample['action'].shape, sample['action_is_pad'].sum())
```

위 예의 `(j + 1) / FPS` 는 NPZ 의 기본 시간 설정인 Δ = 0.1 s, step = 0.1 s 와 같은 상대 시각을 요청한 것이다. 다만 샘플 기준 시각이 다르므로(NPZ 는 실제 카메라 stamp, 이 데이터셋은 격자 시각) 값이 일치한다는 뜻은 아니다. 또한 저장된 격자의 마지막 시각은 마지막 카메라 프레임까지라 원래 에피소드 종료 시각보다 이를 수 있으므로, 같은 상대 시각을 요청해도 NPZ 와 패딩 마스크가 달라질 수 있다. 현재 시각부터 시작하려면 `j / FPS` 를 쓴다.

### 6.6 manifest 확인

`manifest.json` 은 출력 경로 바로 아래에 있다. 재샘플 조건과 에피소드별 대응을 확인한다.

```bash
jq '{fps, max_skew_sec, label, camera_topic, kept_episodes, excluded}' \
   /data/rdfp/datasets/lerobot_v1/manifest.json
jq '.episodes[] | {source_episode_id, episode_index, frames, repeated_frames, max_image_skew_sec}' \
   /data/rdfp/datasets/lerobot_v1/manifest.json
```

| 항목 | 내용 |
|---|---|
| `source_episode_id` · `episode_index` | DB 에피소드 id 와 LeRobot 에피소드 번호의 대응 |
| `first_index` · `frames` | 그 에피소드가 차지하는 전체 행 범위 |
| `origin_ns` | 첫 격자 시각(원래 시계 기준). LeRobot 의 `timestamp` 0 이 이 시각이다 |
| `repeated_frames` · `max_image_skew_sec` | 같은 프레임 반복 수와 최대 시차. 위상 오차의 크기를 보여 준다 |
| `lerobot_version` · `resampling` · `action_meaning` | 만든 환경과 재샘플 규칙 |

---

## 7. 문제 대응

### 7.1 증상별 대응

| 증상 · 메시지 | 확인 항목 | 대응 |
|---|---|---|
| `LeRobot v3 export needs lerobot==0.4.4` | 실행한 파이썬에 lerobot 이 있는가 | §6.1 의 별도 파이썬 환경에서 실행한다 |
| `video gap exceeds max_skew_sec: <시차> s at <경과> s into the episode` | 메시지의 시차는 격자 시각과 선택 프레임 사이의 차이이고, 위치는 격자 시작 시각부터의 경과 시간이다. 한 에피소드라도 이 조건에 걸리면 export 전체가 실패한다 | 그 에피소드를 `--episodes` 에서 빼거나 `--max-skew-sec` 를 올린다. 한도를 올리면 그만큼 위상 오차를 받아들이는 것이다 |
| `joint_command label: do not mix backends` | 관절-명령 레이블로 서로 다른 백엔드의 에피소드를 골랐는가. 관절-명령은 백엔드별 보간 방식과 보상 설정의 영향을 받는다(펑션베이도 scene 에 따라 중력 보상 오프셋이 포함될 수 있다) | 백엔드별로 나눠 export 하거나 관절-계획 레이블을 쓴다. 관절-계획도 관절 구성과 순서는 같아야 한다 |
| `arm joints differ from episode <id>` | 에피소드마다 행동 채널의 관절 순서가 같은가 | 관절 구성이 같은 에피소드끼리 export 한다 |
| `non-finite state/action at t=…` | 그 시각에 그리퍼 폭이나 관절 관측이 비어 있는가 | 해당 에피소드의 적재 상태를 확인하거나 에피소드를 뺀다 |
| `nothing to export after quality checks` | 고른 에피소드가 모두 정상 종료 가정으로 제외됐는가 | 출력의 `excluded` 이유를 확인하고, 필요하면 `--keep-untrustworthy` 를 쓴다 |
| `mp4 not found` | `dataset_config.yaml` 의 영상 루트(`recordings_dir` · `output_mp4_dir`)와 mp4 위치가 맞는가 | 설정을 고치거나 mp4 를 제자리에 둔다 |
| `output directory is not empty` | 출력 경로에 이전 결과가 있는가 | 기존 결과를 지우거나 새 경로를 준다 |
| 출력 경로 옆에 `.<이름>.partial-*` 가 남아 있다 | 이전 export 가 강제 종료됐는가 | 내용을 확인한 뒤 지운다 |
| 영상 화질 · 크기가 원본과 다르다 | 재인코딩 결과다(기본 코덱 `libsvtav1`, §3.5 · §4.7) | 원본 영상이 필요하면 NPZ 경로의 mp4 를 쓴다 |

---

## 8. 코드 위치

| 파일 | 내용 |
|---|---|
| [`rdfp/dataset/export/lerobot_v3.py`](../../src/rdfp/rdfp/dataset/export/lerobot_v3.py) | `export_lerobot_v3` · `grid_frames` · `_ForwardDecoder` |
| [`rdfp/dataset/export_cmd.py`](../../src/rdfp/rdfp/dataset/export_cmd.py) | `export` 명령과 `--format` · `--fps` · `--max-skew-sec` 인자 |
| [`rdfp/dataset/export/frame_clock.py`](../../src/rdfp/rdfp/dataset/export/frame_clock.py) | `FrameClock` · `load_frame_clock` · `interval_stats` |
| [`rdfp/dataset/export/observation.py`](../../src/rdfp/rdfp/dataset/export/observation.py) | `qpos` · 그리퍼 폭 · 그리퍼 명령 one-hot |
| [`rdfp/dataset/export/writer.py`](../../src/rdfp/rdfp/dataset/export/writer.py) | `staged_output` · `write_manifest` · `default_arm_joints` · 레이블 생성 |
| [`rdfp_gui/jobs.py`](../../src/rdfp_gui/rdfp_gui/jobs.py) | GUI 의 export 명령 생성과 `RDFP_LEROBOT_PYTHON` |
| [`rdfp/dataset/tests/export/test_lerobot_v3.py`](../../src/rdfp/rdfp/dataset/tests/export/test_lerobot_v3.py) | 격자 · 시차 · 인자 검사 시험 |

관련 문서는 [저장_형식.md](저장_형식.md)(NPZ 정본과 LeRobot 형식 미채택 근거), [결정_기록.md](결정_기록.md)(결정 28 · 39), [데이터셋_관리_GUI_설계안.md §6.5](../collect/데이터셋_관리_GUI_설계안.md)(설계 배경)다.
