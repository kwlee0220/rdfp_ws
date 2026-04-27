# rosbag2 운영 방안 보고서

**주제:** 지정 토픽 대상 MCAP 기반 장기 저장 및 날짜-시간/토픽 기준 조회를 고려한 운영 설계

---

## 1. 개요

본 문서는 ROS 2 Humble 환경에서 `rosbag2`를 사용하여 지정된 토픽을 안정적으로 저장하고, 이후 **날짜-시간 범위** 및 **토픽 이름 목록**을 기준으로 데이터를 조회할 수 있도록 하기 위한 운영 방안을 정리한 것이다.

다음 조건을 전제로 한다.

* 저장 포맷은 **MCAP**
* 상위 디렉터리는 **날짜별로 구성**
* 내부 bag 파일은 **1시간 단위로 split**
* 저장 대상에는 **영상 토픽 포함**
* `rosbag2`가 운용 중 **중간 종료 후 일정 시간 뒤 다시 실행될 수 있음**
* 추후 **시간 범위 + 토픽 목록** 기준 질의 필요

---

## 2. 운영 목표

본 운영 방안의 목표는 다음과 같다.

1. 지정된 토픽만 안정적으로 저장
2. 장시간 운용 중 프로세스 종료/재시작 상황에 대응
3. 영상 토픽 포함 시에도 관리 가능한 디렉터리 구조 유지
4. 사후 분석 시 특정 시간 범위와 특정 토픽 기준으로 데이터 탐색이 쉬운 구조 확보

---

## 3. 설계 원칙

운영 설계는 아래 4가지 원칙을 따른다.

### 3.1 세션 단위 저장

한 번 실행된 `rosbag2` recorder는 하나의 **세션(session)** 디렉터리만 담당한다.

### 3.2 세션 내부 파일 분할

세션 내부 저장 파일은 **시간 상한(1시간)** 과 **크기 상한** 을 함께 적용하여 split한다. 자세한 기준은 [5.3절](#53-파일-분할-기준)을 참조한다.

### 3.3 재실행 시 새 세션 생성

`rosbag2`가 중간 종료된 뒤 다시 실행되면 기존 bag에 이어 쓰지 않고 **새로운 세션 디렉터리**를 생성한다.

### 3.4 조회를 고려한 메타데이터 관리

사후 질의를 위해 각 세션의 시작/종료 시간, 포함 토픽 목록 등을 별도 인덱스로 관리한다.

---

## 4. 권장 디렉터리 구조

상위 디렉터리는 날짜별로 구성하고, 그 아래에 실행 시각 기준 세션 디렉터리를 둔다.

```text
/data/rosbag/
├── 2026-04-11/
│   ├── session_2026-04-11_09-00-03/
│   │   ├── metadata.yaml
│   │   ├── session_2026-04-11_09-00-03_0.mcap
│   │   ├── session_2026-04-11_09-00-03_1.mcap
│   │   └── ...
│   └── session_2026-04-11_14-22-10/
│       ├── metadata.yaml
│       ├── session_2026-04-11_14-22-10_0.mcap
│       └── ...
├── 2026-04-12/
│   └── session_2026-04-12_00-05-41/
```

### 4.1 split 파일명 규칙

rosbag2가 생성하는 split 파일의 이름은 **`-o` 옵션으로 지정한 경로의 basename**을 접두로 사용하고, 뒤에 split 인덱스(`_0`, `_1`, …)와 저장 포맷 확장자(`.mcap`)가 붙는다.

예를 들어 `-o /data/rosbag/2026-04-11/session_2026-04-11_09-00-03` 으로 기록하면, 생성되는 파일은 다음과 같다.

```text
session_2026-04-11_09-00-03_0.mcap
session_2026-04-11_09-00-03_1.mcap
...
```

즉, 본 운영 규칙에서는 **세션 디렉터리명과 bag basename이 동일**하다는 전제를 둔다. 이는 세션 디렉터리만 보아도 내부 split 파일을 식별할 수 있도록 하기 위함이다.

### 4.2 구조의 장점

* 날짜 기준 보관/백업/삭제가 용이함
* 프로세스 재시작 시에도 세션 단위로 기록이 분리됨
* 세션 내부 파일은 1시간 단위로 관리 가능
* 추후 특정 시간 범위 조회 시 후보 세션을 빠르게 좁힐 수 있음

---

## 5. 기록 방식

### 5.1 저장 포맷

저장소(storage)는 **MCAP**을 사용한다.

### 5.2 저장 대상

기록 대상은 `-a`가 아닌 **명시적 토픽 목록** 방식으로 지정한다.

### 5.3 파일 분할 기준

세션 내부 파일은 **시간 기준(1시간)** 과 **크기 기준 상한** 을 함께 적용하여 split한다.

* `-d 3600` — 하나의 split 파일의 최대 지속 시간을 3600초(1시간)로 제한한다.
* `--max-bag-size <bytes>` — 하나의 split 파일의 최대 크기를 바이트 단위로 제한한다.

두 옵션을 함께 지정하면 **둘 중 먼저 도달한 조건**에서 split이 발생한다. 영상 토픽이 포함된 환경에서는 1시간 이내에도 파일이 과도하게 커질 수 있으므로, 크기 상한을 안전망으로 함께 거는 것을 권장한다.

크기 상한을 함께 두면 다음 효과가 있다.

* 손상·크래시 발생 시 영향 범위를 마지막 split 파일로 국한
* 파일시스템 및 전송 한계(네트워크 업로드 제한, NAS quota 등) 회피
* MCAP 인덱스 로딩·재생 시 메모리 부담 완화

권장값은 다음과 같다.

* 압축 영상 중심: 2~5 GB
* raw 영상(1080p 이상) 중심: 1~2 GB
* 저대역(`/delta_twist_stamp`, `/ee_pose_publisher/ee_pose` 등 상태 토픽 위주): 크기 상한 생략 가능

> 옵션 값은 바이트 정수만 지원되며 `5G` 와 같은 접미사는 허용되지 않는다. 스크립트에서는 `$((5 * 1024**3))` 형태로 계산하여 전달한다. 또한 값을 너무 작게 잡으면 split 파일 수가 폭증하여 메타데이터 오버헤드와 조회 비용이 늘어나므로 주의한다.

### 5.4 시간 표기 정책

세션 디렉터리명과 인덱스 타임스탬프 표기에 사용할 타임존을 **운영 정책으로 단일 고정**한다.

* **로컬 타임존 (본 문서 기본값)**
  * 명령: `date +%F_%H-%M-%S`
  * 장점: 운영자·디버거가 디렉터리명만 보고 시각을 직관적으로 인지한다.
  * 주의: 서머타임(DST)이 적용되는 지역에서는 시각이 되돌아가거나 건너뛰는 구간이 생길 수 있다. DST가 없는 지역(예: 대한민국)에서는 문제가 되지 않는다.

* **UTC**
  * 명령: `date -u +%F_%H-%M-%SZ` (UTC임을 나타내는 `Z` 접미사 포함)
  * 장점: 지역·DST 변화에 영향받지 않으며, 외부 시스템 로그와 시각을 직접 맞추기 쉽다.
  * 주의: 현지 시각과의 환산이 필요하므로 디버깅 편의성은 다소 떨어진다.

한 번 정한 정책은 운영 도중 변경하지 않는다. 중간에 전환하면 디렉터리 정렬 순서와 조회 필터 조건이 어긋나 인덱스 구축이 복잡해진다. 정책 변경이 불가피한 경우 변경 시점 이전/이후를 **별도 데이터셋**으로 분리하여 관리한다.

본 문서의 이후 예시는 로컬 타임존을 전제로 작성한다. UTC 정책을 채택하는 경우, 스크립트에서 `date` 호출을 `date -u`로 바꾸고 세션 basename 끝에 `Z`를 덧붙인다.

### 5.5 QoS 프로파일 오버라이드

ROS 2 토픽은 발행자마다 QoS(Quality of Service) 설정이 다를 수 있다. `ros2 bag record`는 기본적으로 발행자의 QoS를 자동 감지하여 subscribe하지만, 다음과 같은 경우에는 **명시적 오버라이드**가 필요하다.

* 같은 토픽에 서로 다른 QoS를 가진 복수 발행자가 존재하는 경우
* 센서 토픽이 `BEST_EFFORT`로 발행되어 recorder의 기본 설정(`RELIABLE`)과 어긋나는 경우
* `/tf_static`처럼 `TRANSIENT_LOCAL`(late-joiner 수신)이 필요한 토픽을 안정적으로 수신해야 하는 경우
* 재생(`ros2 bag play`) 시 구독자 측과 QoS 호환성을 맞추어야 하는 경우

#### 오버라이드 사용 방법

`--qos-profile-overrides-path <yaml>` 옵션으로 토픽별 QoS를 지정한다.

```yaml
# qos_overrides.yaml
/camera/image_raw:
  reliability: best_effort
  history: keep_last
  depth: 10
  durability: volatile
/tf_static:
  reliability: reliable
  history: keep_all
  durability: transient_local
# /session 은 TRANSIENT_LOCAL 로 발행되므로 recorder 쪽도 맞춰야
# 녹화 시작 이전에 발행된 마지막 상태 메시지를 수신·기록할 수 있다.
# (SessionControlNode 를 namespace 를 부여해 운영하면 키를 /<ns>/session 으로 조정한다.)
/session:
  reliability: reliable
  history: keep_all
  durability: transient_local
```

사용 예:

```bash
ros2 bag record \
  -s mcap \
  -o "${OUT_DIR}" \
  --qos-profile-overrides-path /etc/rosbag/qos_overrides.yaml \
  /camera/image_raw /tf_static ...
```

#### 운영 지침

* QoS 오버라이드 YAML은 **세션 기록과 함께 버전 관리**한다. 동일 세션을 재현 가능하게 하기 위함이다.
* 재생 시에도 동일 파일을 `ros2 bag play --qos-profile-overrides-path`로 전달하여 구독자 측 호환성을 확보한다.
* 오버라이드 파일 경로는 세션 인덱스([10장](#10-사후-조회를-위한-인덱싱-방안))에 함께 기록하여 사후에도 어떤 QoS 정책이 적용되었는지 추적 가능하도록 한다.

---

## 6. 기본 기록 명령 예시

본 장은 `ros2 bag record` 호출을 개념 수준에서 설명하기 위한 최소 예시이다. 실제 운영에서는 [9장 권장 실행 스크립트](#9-권장-실행-스크립트)를 사용한다.

예를 들어 다음 토픽을 저장한다고 가정한다.

* `/camera/image_raw`
* `/camera/info`
* `/session`
* `/delta_twist_stamp`
* `/ee_pose_publisher/ee_pose`
* `/tf`
* `/tf_static`

기록 명령은 아래와 같다.

```bash
DAY_DIR="/data/rosbag/$(date +%F)"
SESSION_NAME="session_$(date +%F_%H-%M-%S)"
OUT_DIR="${DAY_DIR}/${SESSION_NAME}"
MAX_BAG_SIZE=$((5 * 1024**3))   # 5 GB

mkdir -p "${DAY_DIR}"

ros2 bag record \
  -s mcap \
  -o "${OUT_DIR}" \
  -d 3600 \
  --max-bag-size ${MAX_BAG_SIZE} \
  /camera/image_raw \
  /camera/info \
  /session \
  /delta_twist_stamp \
  /ee_pose_publisher/ee_pose \
  /tf \
  /tf_static
```

### 6.1 명령어 의미

* `-s mcap`
  MCAP 저장 방식 사용

* `-o "${OUT_DIR}"`
  날짜별 상위 디렉터리 아래 세션 디렉터리 지정

* `-d 3600`
  하나의 split 파일의 최대 지속 시간을 1시간으로 제한한다. `--max-bag-size`와 함께 사용하면 둘 중 먼저 도달한 조건에서 split된다.

* `--max-bag-size ${MAX_BAG_SIZE}`
  하나의 split 파일의 최대 크기(바이트)를 제한한다. 영상 토픽 포함 시 시간 기준만으로는 파일이 과도하게 커질 수 있으므로 크기 상한을 안전망으로 함께 건다. 기본값은 5 GB로 설정한다.

* 토픽 나열
  지정한 토픽만 기록

---

## 7. 영상 토픽 포함 시 고려사항

영상 토픽이 포함되면 저장 용량과 디스크 쓰기 대역폭이 주요 이슈가 된다.

### 7.1 raw image 저장 시

`sensor_msgs/Image`를 기록하면 원본 픽셀 데이터가 그대로 저장되므로 용량이 매우 커질 수 있다.

### 7.2 compressed image 저장 시

`CompressedImage` 또는 `image_transport/compressed` 계열 토픽을 사용하면 저장 용량을 크게 줄일 수 있다.

### 7.3 디스크 I/O 및 메시지 드롭 방지

고대역폭 토픽(raw/압축 영상 포함)을 기록할 때는 디스크 쓰기 지연으로 인한 **메시지 드롭**이 운영상 가장 큰 이슈가 된다. `rosbag2`는 내부적으로 메시지 캐시를 두고 비동기 기록을 수행하지만, 캐시가 가득 차면 들어오는 메시지가 버려진다.

#### `--max-cache-size`

recorder의 내부 쓰기 큐 크기를 바이트 단위로 지정한다. 기본값은 100 MB(`100 * 1024 * 1024`)이다. 영상 토픽 포함 시 다음 기준으로 상향한다.

* 단일 카메라 1080p 압축 영상 기준: 256 MB ~ 512 MB
* 복수 카메라 또는 raw 영상 포함: 1 GB 이상

과도하게 크면 메모리를 많이 점유하고 크래시 시 손실 범위가 커지므로, 디스크 I/O 여유(sync write throughput)를 먼저 측정한 후 결정한다.

#### `--storage-config-file`

MCAP storage의 **chunk 크기, 압축 모드, 인덱스 전략** 등을 YAML로 세밀하게 조정할 수 있다.

```yaml
# mcap_storage.yaml
chunk_size: 4194304       # 4 MB
compression: zstd
compression_level: 1      # 최소 압축으로 CPU 비용 절감
```

기본값은 대부분 환경에서 충분하지만, 다음과 같은 경우에 조정한다.

* 기본 chunk(768 KB)가 고대역폭 영상에 작아서 파일 메타데이터 오버헤드가 큰 경우 → chunk 크기 상향
* 디스크가 SSD·NVMe이고 CPU 여유가 있다면 → `zstd` 압축으로 파일 크기 감소
* HDD 환경에서 쓰기 지연이 원인일 때 → 압축 비활성 또는 레벨 하향으로 CPU 오버헤드 축소

#### 운영 점검 포인트

* `ros2 bag record` 실행 중 로그에 `Dropped message` 관련 경고가 출력되면 즉시 캐시 크기를 상향한다.
* 녹화 종료 후 `ros2 bag info`의 메시지 수가 예상치와 크게 차이나면 드롭을 의심하고 디스크 I/O·캐시·QoS를 검토한다.
* 기록 중 `iostat`·`dstat` 등으로 디스크 write throughput과 `await` 지연을 모니터링한다.

### 7.4 권장 방안

* 원본 품질이 반드시 필요하면 `/image_raw` 기록
* 장기 보관 및 조회 효율이 중요하면 압축 영상 토픽 우선 고려
* `camera_info`는 영상 토픽과 함께 기록
* 시계열 해석 및 재생을 위해 `/tf`, `/tf_static`도 포함
* 영상 포함 시 **`--max-cache-size` 상향**을 기본으로 적용하고, 필요 시 `--storage-config-file`로 MCAP 튜닝

---

## 8. 중간 종료 및 재실행 대응 방안

### 8.1 기본 방침

`rosbag2`가 종료된 뒤 다시 실행될 경우, **기존 bag에 이어쓰기하지 않는다.**

### 8.2 운영 규칙

재실행 시에는 항상 새로운 세션 디렉터리를 생성한다.

예:

```text
2026-04-11/session_2026-04-11_09-00-03/
2026-04-11/session_2026-04-11_14-22-10/
2026-04-11/session_2026-04-11_18-05-44/
```

### 8.3 이유

이 방식은 다음과 같은 장점이 있다.

* bag 무결성 유지
* 세션 간 경계 명확
* 장애 발생 시 영향 범위 축소
* 조회 시 세션별 필터링 용이

---

## 9. 권장 실행 스크립트

실제 운영에서는 아래와 같이 wrapper script로 관리하는 것이 바람직하다.

```bash
#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="/data/rosbag"
# 타임존 정책: 로컬 시간 기준. UTC 정책 채택 시 'date -u +%F' 및 'date -u +%F_%H-%M-%SZ' 로 교체한다.
DAY_DIR="${BASE_DIR}/$(date +%F)"
SESSION_NAME="session_$(date +%F_%H-%M-%S)"
OUT_DIR="${DAY_DIR}/${SESSION_NAME}"
MAX_BAG_SIZE=$((5 * 1024**3))        # 5 GB   - 단일 split 파일 크기 상한
MAX_CACHE_SIZE=$((256 * 1024**2))    # 256 MB - recorder 내부 쓰기 큐 크기 (드롭 방지)

# QoS 오버라이드(선택). 파일이 존재할 때만 옵션으로 전달한다.
QOS_OVERRIDES_PATH="${QOS_OVERRIDES_PATH:-/etc/rosbag/qos_overrides.yaml}"

# MCAP storage 튜닝(선택). 파일이 존재할 때만 옵션으로 전달한다.
STORAGE_CONFIG_PATH="${STORAGE_CONFIG_PATH:-/etc/rosbag/mcap_storage.yaml}"

TOPICS=(
  /camera/image_raw
  /camera/info
  /session
  /delta_twist_stamp
  /ee_pose_publisher/ee_pose
  /tf
  /tf_static
)

OPT_ARGS=()
if [ -n "${QOS_OVERRIDES_PATH}" ] && [ -f "${QOS_OVERRIDES_PATH}" ]; then
    OPT_ARGS+=(--qos-profile-overrides-path "${QOS_OVERRIDES_PATH}")
fi
if [ -n "${STORAGE_CONFIG_PATH}" ] && [ -f "${STORAGE_CONFIG_PATH}" ]; then
    OPT_ARGS+=(--storage-config-file "${STORAGE_CONFIG_PATH}")
fi

mkdir -p "${DAY_DIR}"

echo "[INFO] output:         ${OUT_DIR}"
echo "[INFO] max bag size:   ${MAX_BAG_SIZE} bytes"
echo "[INFO] max cache size: ${MAX_CACHE_SIZE} bytes"
echo "[INFO] qos overrides:  ${QOS_OVERRIDES_PATH:-(none)}"
echo "[INFO] storage config: ${STORAGE_CONFIG_PATH:-(none)}"
echo "[INFO] topics:         ${TOPICS[*]}"

exec ros2 bag record \
  -s mcap \
  -o "${OUT_DIR}" \
  -d 3600 \
  --max-bag-size ${MAX_BAG_SIZE} \
  --max-cache-size ${MAX_CACHE_SIZE} \
  "${OPT_ARGS[@]}" \
  "${TOPICS[@]}"
```

### 9.1 스크립트 사용 효과

* 사람이 출력 경로를 수동으로 지정하는 실수를 줄일 수 있음
* 토픽 목록 관리가 쉬움
* systemd, supervisor, cron 등과 연계 가능
* 재실행 시 자동으로 새 세션 생성 가능

### 9.2 systemd 연계 예시

9장 스크립트를 systemd 서비스로 등록하면 부팅 시 자동 실행, 비정상 종료 시 자동 재시작, 로그 통합 관리가 가능하다.

```ini
# /etc/systemd/system/rosbag2-recorder.service
[Unit]
Description=rosbag2 Session Recorder
After=network-online.target
Wants=network-online.target

[Service]
Type=exec
User=ros
Group=ros
Environment=ROS_DOMAIN_ID=0
Environment=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 선택: QoS / storage 설정 파일 경로를 환경변수로 주입
Environment=QOS_OVERRIDES_PATH=/etc/rosbag/qos_overrides.yaml
Environment=STORAGE_CONFIG_PATH=/etc/rosbag/mcap_storage.yaml

ExecStart=/usr/local/bin/rosbag2-recorder.sh

# 재시작 정책: 비정상 종료 시에만 자동 재시작 (정상 종료 시는 유지)
Restart=on-failure
RestartSec=5

# rosbag2는 SIGINT에서만 안전하게 세션을 닫고 인덱스를 기록한다.
KillSignal=SIGINT
TimeoutStopSec=30

[Install]
WantedBy=multi-user.target
```

#### 핵심 설계 포인트

* `Restart=on-failure` — 크래시 시 스크립트가 새 `SESSION_NAME`으로 자동 재시작된다. [8장](#8-중간-종료-및-재실행-대응-방안)의 "재실행 시 새 세션 생성" 규칙이 자연스럽게 구현된다.
* `KillSignal=SIGINT` + `TimeoutStopSec=30` — `systemctl stop` 시 SIGINT를 전송하고 최대 30초 대기한다. `rosbag2`가 이 시간 내에 MCAP chunk_index를 완결시킬 수 있도록 한다. `SIGKILL`로 강제 종료되면 파일이 열린 상태로 남아 복구가 필요할 수 있다.
* `Type=exec` — `ExecStart`의 `exec ros2 bag record ...` (9장 스크립트 마지막 줄)로 쉘 프로세스가 recorder로 교체되므로 시그널이 정확히 전달된다.
* 환경변수 주입으로 QoS / storage 설정 파일을 서비스 단위에서 스위치할 수 있다.

#### 등록 및 기동

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now rosbag2-recorder.service
sudo systemctl status rosbag2-recorder.service
journalctl -u rosbag2-recorder.service -f
```

### 9.3 토픽 프로파일 YAML 분리

스크립트에 토픽 배열을 하드코딩하면 실험 조건(전체/카메라만/동작만)별로 스크립트를 복제해야 해서 유지보수 비용이 커진다. 토픽 목록을 별도 YAML 파일로 분리하고 **프로파일 이름으로 스위치**하는 구조가 더 유연하다.

```yaml
# /etc/rosbag/topic_profiles.yaml
profiles:
  default:
    - /camera/image_raw
    - /camera/info
    - /session
    - /delta_twist_stamp
    - /ee_pose_publisher/ee_pose
    - /tf
    - /tf_static
  camera_only:
    - /camera/image_raw
    - /camera/info
    - /tf
    - /tf_static
  motion_only:
    - /session
    - /delta_twist_stamp
    - /ee_pose_publisher/ee_pose
    - /tf
    - /tf_static
```

스크립트에서는 `yq` 또는 Python으로 읽어 배열로 사용한다.

```bash
PROFILE="${PROFILE:-default}"
TOPICS_FILE="${TOPICS_FILE:-/etc/rosbag/topic_profiles.yaml}"

# yq v4 기준
mapfile -t TOPICS < <(yq ".profiles.${PROFILE}[]" "${TOPICS_FILE}")

if [ ${#TOPICS[@]} -eq 0 ]; then
    echo "[ERROR] profile '${PROFILE}' not found or empty in ${TOPICS_FILE}" >&2
    exit 1
fi
```

실험 조건 전환은 환경변수 한 줄로 끝난다.

```bash
PROFILE=camera_only systemctl restart rosbag2-recorder.service
```

### 9.4 사전 검증 체크리스트

녹화 시작 전 다음 항목을 확인하면 운영 중 장애를 크게 줄일 수 있다. 스크립트의 `ExecStartPre`나 본체 상단에 체크 함수로 구현하는 것을 권장한다.

* **토픽 존재 여부** — `ros2 topic list`에 대상 토픽이 모두 노출되는지 확인한다. 지연 발행 토픽이 있다면 타임아웃을 두고 재시도한다.
* **디스크 여유 공간** — `df --output=avail "${BASE_DIR}"`로 여유 바이트를 구해 최소 임계치 이상인지 확인한다. 임계치는 `MAX_BAG_SIZE × 예상 split 수 + 안전 마진`을 기준으로 잡는다.
* **시계 동기화 상태** — `chronyc tracking` 또는 `timedatectl show`로 NTP 동기화 여부를 확인한다. bag 타임스탬프의 신뢰는 시계 정확도에 의존하므로 동기화되지 않은 상태에서는 녹화를 거부하도록 한다.
* **쓰기 권한** — `BASE_DIR`에 대한 recorder 사용자의 쓰기 권한 확인.
* **QoS / storage 파일 유효성** — 경로가 지정된 경우 파일이 실제 존재하는지 확인(9장 스크립트에 이미 조건부 로직 있음).
* **이전 세션 정리 상태** — 동일 경로에 미완결 bag 디렉터리(잠금 파일 잔존 등)가 있다면 경고.

체크 실패 시에는 즉시 비정상 종료(`exit 1`)하여 systemd의 `Restart=on-failure`가 과도하게 재시작하지 않도록 한다(필요 시 `RestartSec`를 크게 설정하거나 `StartLimitBurst`로 제한한다).

### 9.5 멀티 recorder 전략 (고려사항)

한 recorder 프로세스가 **여러 고대역폭 영상 토픽을 동시에** 기록하면 단일 디스크 쓰기 큐 병목이 발생할 수 있다. 이런 경우 토픽을 성격별로 나누어 **여러 개의 recorder를 병렬로 실행**하는 것을 고려한다.

* **분리 축 예시**
  * 영상용 recorder: `/camera/*` — 큰 캐시, zstd 압축 비활성
  * 저대역 recorder: `/session`, `/delta_twist_stamp`, `/ee_pose_publisher/ee_pose`, `/tf`, `/tf_static` — 기본 설정
* **디렉터리 구성**
  * `session_<time>/video/` 와 `session_<time>/state/` 처럼 **같은 세션 하위에 서브디렉터리**를 두면, 조회 시에는 한 세션 단위로 동시에 읽을 수 있다.
  * 또는 별도 세션으로 병렬 기록하고 세션 이름에 공통 타임스탬프를 두어 후처리로 묶는다.
* **동기화 주의**
  * 각 recorder의 시계는 ROS time 기준으로 동일하지만, 파일 단위 split 시점이 다르므로 split 경계가 어긋난다. split 인덱스는 recorder별로 독립 관리한다.
  * `/tf`, `/tf_static`은 **재생 시 필수**이므로 어느 쪽에 포함시킬지 정책적으로 정한다(권장: 저대역 recorder).

단일 recorder로 대역폭 여유가 충분한 환경에서는 분리하지 않는다. 디스크 I/O 모니터링([7.3절](#73-디스크-io-및-메시지-드롭-방지))에서 drop이 반복될 때만 적용을 검토한다.

---

## 10. 사후 조회를 위한 인덱싱 방안

추후 사용자는 아래 조건으로 질의할 예정이다.

* 날짜-시간 범위
* 토픽 이름 목록

이를 위해 bag 파일 자체만 직접 탐색하는 것보다, **세션 단위 메타 인덱스**를 별도로 관리하는 것이 효율적이다.

### 10.1 세션 단위 인덱스

세션 전체를 한 엔트리로 보는 상위 인덱스이다. 날짜·세션 디렉터리 단위 조회에 사용한다.

```text
session_path
date
session_start
session_end
storage_id
topics[]
message_count_by_topic
qos_overrides_path     # 적용된 QoS 오버라이드 파일 경로(있는 경우)
storage_config_path    # 적용된 MCAP storage 설정 경로(있는 경우)
```

### 10.2 split 단위 인덱스

세션 단위 인덱스만으로는 **긴 세션에서 후보 구간이 과도하게 커져** 1차 필터링 정밀도가 떨어진다. [5.3절](#53-파일-분할-기준)의 split 정책(시간+크기)에 따라 한 세션이 여러 개의 split 파일로 나뉘므로, **각 split 파일을 별도의 인덱스 엔트리**로 취급하면 시간 범위 필터가 훨씬 정밀해진다.

split 단위 인덱스 항목 예시는 다음과 같다.

```text
session_path
split_path             # 예: session_..._2.mcap
split_index            # 0, 1, 2, ...
split_start
split_end
duration
message_count
topics[]
message_count_by_topic
```

`metadata.yaml`의 `files[]` 섹션에 split별 `starting_time`, `duration`, `message_count`가 기록되어 있으므로 이를 파싱하면 그대로 인덱스 엔트리로 사용할 수 있다.

#### 적용 기준

* 세션 길이가 **수 시간 이상**인 경우: split 단위 인덱스를 기본 채택한다.
* 세션이 짧고(1시간 미만) split이 거의 발생하지 않는 운영이라면 세션 단위만으로도 충분하다.
* 세션 단위 + split 단위를 **계층적으로** 구성하여 1차(날짜/세션) → 2차(split) 필터 흐름으로 사용한다.

### 10.3 인덱스 생성 방법

각 세션의 다음 정보를 활용한다.

* `metadata.yaml` — 세션·split 정보가 모두 포함되어 있어 **1차 소스**로 사용
* `ros2 bag info <session_dir>` — 사람이 확인하거나 스크립트에서 파싱

> 진행 중인(recording) 세션에 대해 `ros2 bag info`를 호출하면 결과가 불안정할 수 있다. 세션 **종료 후**에 최종 인덱스를 갱신하거나, 실시간 인덱싱이 필요하면 `metadata.yaml`을 직접 파싱하는 방식을 사용한다.

### 10.4 인덱스의 역할

* 어떤 세션·split이 어느 시간 구간을 포함하는지 판단
* 요청한 토픽이 해당 세션·split에 포함되어 있는지 확인
* 실제 데이터를 열기 전에 후보를 빠르게 필터링
* QoS·storage 설정의 사후 추적(재현성 확보)

### 10.5 인덱스 저장 형식

인덱스를 실제로 저장하는 방식은 운영 특성에 따라 두 가지 중 하나를 선택한다.

#### (a) SQLite 단일 파일 (권장)

세션/split 수가 많고, **시간 범위 + 토픽 조합 쿼리**가 빈번한 환경에 적합하다. 단일 파일이라 백업과 원자적 갱신이 쉽다.

**스키마 예시**

```sql
-- 세션 단위 엔트리
CREATE TABLE sessions (
    session_path         TEXT    PRIMARY KEY,
    date                 TEXT    NOT NULL,           -- YYYY-MM-DD
    session_start        INTEGER NOT NULL,           -- epoch nanoseconds
    session_end          INTEGER NOT NULL,
    storage_id           TEXT    NOT NULL,           -- 'mcap'
    message_count        INTEGER NOT NULL,
    qos_overrides_path   TEXT,
    storage_config_path  TEXT
);
CREATE INDEX idx_sessions_date  ON sessions(date);
CREATE INDEX idx_sessions_range ON sessions(session_start, session_end);

-- split 단위 엔트리
CREATE TABLE splits (
    split_path      TEXT    PRIMARY KEY,             -- 절대 경로
    session_path    TEXT    NOT NULL
                    REFERENCES sessions(session_path) ON DELETE CASCADE,
    split_index     INTEGER NOT NULL,                -- 0, 1, 2, ...
    split_start     INTEGER NOT NULL,                -- epoch nanoseconds
    split_end       INTEGER NOT NULL,
    duration_nsec   INTEGER NOT NULL,
    message_count   INTEGER NOT NULL
);
CREATE INDEX idx_splits_session ON splits(session_path);
CREATE INDEX idx_splits_range   ON splits(split_start, split_end);

-- split × 토픽 관계 및 메시지 수
CREATE TABLE split_topics (
    split_path     TEXT    NOT NULL
                   REFERENCES splits(split_path) ON DELETE CASCADE,
    topic_name     TEXT    NOT NULL,
    message_count  INTEGER NOT NULL,
    PRIMARY KEY (split_path, topic_name)
);
CREATE INDEX idx_split_topics_topic ON split_topics(topic_name);
```

**대표 쿼리: 시간 범위 + 복수 토픽을 모두 포함하는 split 검색**

```sql
SELECT s.split_path
FROM   splits s
JOIN   split_topics t ON t.split_path = s.split_path
WHERE  s.split_end   >= :query_start
  AND  s.split_start <= :query_end
  AND  t.topic_name  IN ('/camera/image_raw', '/ee_pose_publisher/ee_pose')
GROUP BY s.split_path
HAVING COUNT(DISTINCT t.topic_name) = 2;
```

#### (b) 세션별 sidecar JSON + 통합 카탈로그

각 세션 디렉터리 내부에 `session_index.json` 사본을 두고, 상위에 전체 카탈로그(`/data/rosbag/catalog.json` 또는 날짜별 카탈로그)를 생성한다.

* 장점: 세션 디렉터리를 그대로 복사/이동/백업하면 인덱스가 함께 따라간다. 분산 운영·원격 저장소에 유리하다.
* 단점: 조건 조합 쿼리가 느리며, 카탈로그 일관성 유지를 직접 구현해야 한다.

#### 선택 지침

* 단일 노드에서 운영·조회 모두 수행 → **SQLite** 권장
* 다중 노드가 각자 기록하고 중앙에서 수집·조회 → **sidecar JSON + 통합 카탈로그**
* 어느 방식이든 **split 단위 인덱스는 공통**으로 포함한다.

---

## 11. 조회 절차

예를 들어 사용자가 다음과 같이 질의한다고 가정한다.

* 시간 범위: `2026-04-11 13:10:00 ~ 2026-04-11 15:20:00`
* 토픽 목록: `/camera/image_raw`, `/ee_pose_publisher/ee_pose`

조회 절차는 아래와 같다.

### 11.1 날짜 폴더 선택

우선 해당 날짜 폴더를 선택한다.

```text
/data/rosbag/2026-04-11/
```

### 11.2 세션 후보 선택

각 세션의 시작/종료 시각이 질의 시간 범위와 겹치는지 검사한다.

겹침 조건은 다음과 같다.

```text
session_end >= query_start
AND
session_start <= query_end
```

### 11.3 split 후보 선택

후보 세션 내부에서 각 split 파일의 시간 범위가 질의 시간 범위와 겹치는지 검사한다. [10.2절](#102-split-단위-인덱스)의 split 단위 인덱스 또는 `metadata.yaml`의 `files[]` 정보를 직접 활용한다.

겹침 조건은 세션 후보 선택과 동일하다.

```text
split_end >= query_start
AND
split_start <= query_end
```

이 단계에서 걸러지면 해당 split 파일은 전혀 열지 않아도 되므로, 긴 세션에서도 조회 범위를 최소화할 수 있다.

### 11.4 토픽 필터링

후보 split(또는 세션) 중 요청한 토픽을 포함하는 것만 남긴다.

### 11.5 실제 추출

필터링된 split(또는 세션)만 대상으로 데이터를 읽거나 재생한다.

#### (a) `ros2 bag play` — 재생 기반 조회

필터링된 세션을 대상으로, 질의 시간 범위 중 해당 세션 경계 내부 구간만 재생한다.

```bash
ros2 bag play <session_dir> \
  --start-offset <sec> \
  --playback-duration <sec> \
  --topics /camera/image_raw /ee_pose_publisher/ee_pose
```

* `--start-offset` 은 **세션 시작 기준** 상대 초 단위이다. 질의 시간 범위(예: `2026-04-11 13:10:00`)에서 세션 시작 시각을 빼서 계산한다.
* `--playback-duration` 은 재생 길이(초). 질의 종료 시각과 세션 종료 시각 중 이른 쪽을 기준으로 계산한다.
* `--topics` 로 필요한 토픽만 재생하여 구독 측 부하를 줄인다.
* QoS 오버라이드를 기록 시에 사용했다면 `--qos-profile-overrides-path` 로 동일 파일을 전달한다.

#### (b) MCAP CLI — 부분 추출

재생이 아니라 **범위를 잘라낸 새 MCAP 파일**이 필요한 경우 MCAP CLI(`mcap` 명령)를 사용한다.

```bash
mcap filter \
  --start-nsec <query_start_ns> \
  --end-nsec <query_end_ns> \
  --include-topic /camera/image_raw \
  --include-topic /ee_pose_publisher/ee_pose \
  <input.mcap> \
  -o /tmp/extracted.mcap
```

* 여러 split 파일에 걸쳐 있는 경우, split별로 추출한 뒤 `mcap merge`로 합친다.
* MCAP 단독 도구라서 ROS 2 런타임 없이도 실행 가능하다(외부 분석 파이프라인과 연계하기 용이).

#### 계산 보조 (query 시각 → 상대 오프셋)

```bash
# 예: 세션 시작 2026-04-11 09:00:03, query_start 2026-04-11 13:10:00
SESSION_START_S=$(date -d '2026-04-11 09:00:03' +%s)
QUERY_START_S=$(date -d '2026-04-11 13:10:00' +%s)
QUERY_END_S=$(date -d '2026-04-11 15:20:00' +%s)

START_OFFSET=$(( QUERY_START_S - SESSION_START_S ))
DURATION=$(( QUERY_END_S - QUERY_START_S ))
```

> 여러 세션에 걸친 질의인 경우 각 세션마다 `START_OFFSET`·`DURATION`을 재계산하여 순차 재생하거나, 각 split에서 MCAP 추출 후 병합한다.

---

## 12. 디스크 운영 정책

장기 저장을 전제로 하는 운영에서는 디스크가 한정된 자원이라는 점에 주의한다. 기록 스크립트가 아무리 안정적이어도 디스크가 가득 차면 기록이 중단되거나 OS 레벨 장애로 확대될 수 있다. 본 장은 장기 운용을 위한 정책을 정리한다.

### 12.1 디스크 사용량 모니터링

* **경고 임계치**: 전체 용량의 **75%** 에 도달하면 알림을 발송한다.
* **강제 조치 임계치**: 전체 용량의 **90%** 에 도달하면 오래된 세션을 자동 정리한다.
* 모니터링 수단: `node_exporter` + Prometheus + Alertmanager 조합, 또는 단순 cron + `df` + 메신저 webhook.
* 기록 프로세스는 시작 시점에 **남은 디스크 공간을 점검**하고 한계 이하이면 녹화를 거부한다([9장 스크립트](#9-권장-실행-스크립트)에 선행 체크 로직을 추가 권장).

### 12.2 보관 기간(retention) 정책

* **기본 보관 기간**: 30일
* 보관 기간이 경과한 날짜 디렉터리는 다음 중 하나로 처리한다.
  * 완전 삭제
  * cold storage(별도 NAS, 외부 아카이브)로 이동
  * 중요도 태그가 부여된 세션만 선별 보관
* 보관 기간은 프로젝트·규정 요건(예: 사고 조사·법적 보존 의무)에 맞춰 조정한다.
* retention 수행 시 **세션 인덱스([10장](#10-사후-조회를-위한-인덱싱-방안))도 함께 갱신**한다. SQLite의 경우 `DELETE ... WHERE date < :cutoff` 로 일괄 삭제한 뒤 `VACUUM` 으로 공간을 회수한다.

### 12.3 백업 및 아카이빙

* **백업 대상**: MCAP 파일 + `metadata.yaml` + 인덱스 DB(또는 JSON 카탈로그)
* **백업 주기**: 매일 1회 (전일자 디렉터리가 닫힌 직후 수행)
* **백업 방식**: `rsync` / `rclone` 등으로 외부 저장소에 증분 복사하거나, cold storage로 정기 이동
* 인덱스는 별도로 백업하여 **재구축 없이 복원 가능**하도록 유지한다.
* MCAP의 `chunk_index`가 손상되지 않은 상태에서 복제되도록, **기록이 진행 중인 날짜**의 최신 세션은 일시적으로 백업 대상에서 제외한다.

### 12.4 자동화 예시

retention 정리를 매일 새벽에 수행하는 cron 예시:

```bash
# /etc/cron.d/rosbag_retention
0 4 * * * root /usr/local/bin/rosbag-retention.sh 30 >> /var/log/rosbag-retention.log 2>&1
```

`rosbag-retention.sh` 는 다음 작업을 수행한다.

1. 인자로 받은 기준일(예: 30일) 이전의 날짜 디렉터리 식별
2. 해당 디렉터리를 삭제 또는 cold storage로 이동
3. 인덱스(10장)에서 관련 엔트리를 제거 또는 상태 필드 갱신
4. 실패 시 알림 발송

---

## 13. 본 구조가 적합한 이유

### 12.1 날짜 폴더

운영 보관 단위가 됨

### 12.2 세션 디렉터리

프로세스 실행 단위가 됨

### 12.3 1시간 split

파일 크기 제어, 손상 범위 제한, 재생 단위 분리에 유리함

### 12.4 세션 인덱스

질의 성능과 관리 효율을 높임

즉, 저장 구조와 조회 구조를 분리함으로써 운용성과 분석성을 모두 확보할 수 있다.

---

## 14. 권장하지 않는 방식

### 14.1 하루 전체를 하나의 세션으로 유지

중간 종료/재시작이 발생하는 환경에서는 불리하다.

### 14.2 모든 토픽 기록 (`-a`)

불필요한 토픽까지 저장되어 디스크 사용량과 조회 복잡도가 증가한다.

### 14.3 raw 이미지 토픽을 무조건 장기 저장

디스크 용량과 쓰기 대역폭 부담이 급격히 증가할 수 있다.

---

## 15. 최종 권장안

### 15.1 기록 규칙

* 저장소: **MCAP**
* 상위 디렉터리: **`/data/rosbag/YYYY-MM-DD/`**
* 세션 디렉터리: **`session_YYYY-MM-DD_HH-MM-SS/`**
* split 기준: **시간 상한 `-d 3600` + 크기 상한 `--max-bag-size`** (둘 중 먼저 도달하는 조건에서 split)
* 타임존: **운영 정책으로 단일 고정** ([5.4절](#54-시간-표기-정책) 참조)
* 기록 대상: **명시적 토픽 목록**
* 재실행 시: **항상 새 세션 생성**
* 실행 방식: **[9장 권장 실행 스크립트](#9-권장-실행-스크립트) 사용**
* 보관 정책: **retention + 모니터링 병행** ([12장](#12-디스크-운영-정책) 참조)

### 15.2 조회 규칙

* 세션별 `metadata.yaml` 기반 카탈로그 생성 (진행 중 세션은 종료 후 인덱스 갱신)
* 시간 범위와 토픽 목록으로 후보 세션·split 필터링
* 후보만 대상으로 `ros2 bag play` 또는 `mcap filter`를 통해 실제 데이터 열람

---

## 16. 결론

본 요구사항에서는 다음 구조가 가장 적합하다.

> **날짜별 상위 디렉터리 + 실행 시각별 세션 디렉터리 + 세션 내부 1시간 split + 세션 메타 인덱스 관리**

이 구조는 다음을 동시에 만족한다.

* 지정 토픽 안정적 기록
* 중간 종료/재실행 대응
* 영상 토픽 포함 운용 가능
* 날짜-시간 및 토픽 기준 질의 지원

즉, 저장과 조회를 모두 고려한 실용적인 `rosbag2` 운영 구조로 볼 수 있다.

원하시면 다음 단계로 이 보고서를 바탕으로
**“운영 절차서 버전”** 또는 **“실행 스크립트 포함 상세 설계서 버전”**으로 확장해 드리겠습니다.
