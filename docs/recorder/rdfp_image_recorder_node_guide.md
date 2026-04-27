# RdfpImageRecorderNode — Programmer's Guide

세션 제어 토픽(`/session`)에 따라 ROS2 이미지 토픽을 타임스탬프 기반으로 MP4
녹화하는 노드 사용 가이드.

- 실행 파일(CLI): `ros2 run rdfp rdfp_image_recorder`
- Python 클래스: `RdfpImageRecorderNode`
- ROS2 노드 이름: `rdfp_image_recorder_node`

---

## 목차

1. [개요](#개요)
2. [사전 요구사항](#사전-요구사항)
3. [Quick Start](#quick-start)
4. [파라미터](#파라미터)
5. [녹화 제어 흐름](#녹화-제어-흐름)
6. [타임스탬프 기반 녹화 경계](#타임스탬프-기반-녹화-경계)
7. [출력 파일](#출력-파일)
8. [토픽 연결](#토픽-연결)
9. [에러 처리](#에러-처리)
10. [실전 예제](#실전-예제)
11. [트러블슈팅](#트러블슈팅)

---

## 개요

`RdfpImageRecorderNode`는 `SessionControlNode` 가 발행하는 세션 토픽
(`rdfp_msgs/msg/SessionCommand`, 기본 `/session`) 의 상태 변경
(`IN_EPISODE` / `IN_SESSION`) 에 따라 이미지 스트림을 MP4 파일로 녹화한다.

**`ImageRecorderNode` 와의 차이:**

| 항목 | `ImageRecorderNode` | `RdfpImageRecorderNode` |
|---|---|---|
| 녹화 제어 | 서비스 호출 (`~/start_session`, `~/stop_session`) | 세션 토픽 구독 (자동) |
| 녹화 경계 | 메시지 도착 순서 | **타임스탬프 기반** |
| 프레임 버퍼링 | 없음 | `pending_image_queue` |
| 필수 파라미터 | `resolution` (optional), `fps` (기본 10) | `output_dir`, `resolution` |

**핵심 특징:**
- 메시지 `header.stamp` 을 기준으로 녹화 경계를 판정하여 도착 순서 차이를 보상
- `pending_image_queue` (DropOldest) 로 start/stop 경계 근처 프레임을 정확히 처리
- `FFMpegMp4Recorder` 를 녹화 엔진으로 재사용
- 세션 퍼블리셔와 호환되는 `TRANSIENT_LOCAL` QoS 로 late-join 시 직전 상태 즉시 수신
- 녹화 1회마다 **MP4 + sidecar(jsonl) + metadata(json)** 3개 파일 생성 — mp4
  프레임과 sidecar 라인은 1:1 매치되어 프레임별 `header.stamp` 복원이 가능
  (자세한 스키마는 [출력 파일](#출력-파일) 참조)

---

## 사전 요구사항

```bash
# ffmpeg 설치
sudo apt install ffmpeg

# rdfp_msgs + rdfp 빌드
colcon build --packages-select rdfp_msgs rdfp
source install/setup.bash
```

`SessionControlNode` 가 실행 중이어야 세션 토픽(`/session`) 이 발행된다.
[`ros2 launch rdfp rdfp.launch.py`](../../src/rdfp/launch/rdfp.launch.py) 로
session_control + camera + viewer + recorder 를 한 번에 띄울 수도 있다
(아래 [실전 예제](#실전-예제) 참조).

---

## Quick Start

```bash
# 터미널 1: 세션 제어 노드
ros2 run rdfp session_control_node

# 터미널 2: 카메라 노드
ros2 run rdfp camera_node --ros-args \
  -p camera_id:=0 -p fps:=30 -p resolution:=640x480

# 터미널 3: 레코더 노드
# session 토픽은 기본이 /session 이므로 별도 remap 불필요.
ros2 run rdfp rdfp_image_recorder --ros-args \
  -r image:=/camera_node/image_raw \
  -p output_dir:=/tmp/recordings \
  -p fps:=30 \
  -p resolution:=640x480

# 터미널 4: 녹화 제어 (SessionControlNode 서비스 호출로 상태 전이)
ros2 service call /session_control/start_session std_srvs/srv/Trigger
ros2 service call /session_control/start_episode std_srvs/srv/Trigger
# ... 녹화 진행 ...
ros2 service call /session_control/stop_episode std_srvs/srv/Trigger
ros2 service call /session_control/stop_session std_srvs/srv/Trigger

# 결과 확인
ls /tmp/recordings/*.mp4
```

`start_episode` → 녹화 시작, `stop_episode` → 녹화 종료. 레코더 자체에는
별도 서비스 호출이 없고, 세션 토픽 구독으로 자동 제어된다.

---

## 파라미터

### 필수 파라미터

| 파라미터 | 타입 | 설명 |
|----------|------|------|
| `output_dir` | string | MP4 저장 디렉터리. 없으면 자동 생성 |
| `resolution` | string | `"WIDTHxHEIGHT"` 형식 (예: `"640x480"`) |

### 선택 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `fps` | `10` | 녹화 프레임 레이트 (CFR). **입력 이미지 스트림의 실제 frame rate 와 일치시켜야 한다** — 불일치 시 재생 속도 왜곡 또는 프레임 drop |
| `session_prefix` | `"session"` | 파일명 접두사 |
| `pixel_format` | `"bgr8"` | `bgr8`/`rgb8`/`mono8` |
| `encoder_mode` | `"auto"` | `auto`/`cpu`/`gpu` |
| `queue_size` | `120` | FFMpegMp4Recorder 내부 프레임 큐 크기 |
| `pending_queue_length` | `60` | 타임스탬프 경계 보상 큐 크기 (프레임 수) |
| `bitrate` | `"4M"` | ffmpeg `-b:v` |
| `gop_size` | `0` | `0`이면 `fps*2` 적용 |
| `preset` | `"medium"` | libx264 preset |
| `preferred_hw_codec` | `""` | 빈 문자열이면 자동 probe |
| `ffmpeg_binary` | `"ffmpeg"` | ffmpeg 실행 파일 경로 |
| `vaapi_device` | `"/dev/dri/renderD128"` | VAAPI 디바이스 경로 |

> ⚠ `fps` 기본값(10)은 보수적인 값이다. 카메라가 30fps 로 발행한다면 반드시
> `-p fps:=30` 으로 맞춰야 한다. CFR 기반이므로 불일치 시 저장된 영상 재생
> 속도가 실시간과 어긋난다.

> 출력 파일명 규칙과 sidecar·metadata 스키마는 아래 [출력 파일](#출력-파일)
> 섹션에서 종합적으로 설명한다.

---

## 녹화 제어 흐름

```
SessionControlNode                    RdfpImageRecorder
      │                                      │
      │  start_session (Trigger)             │
      ├─────────────────────────────────────►│ (무시 — 세션 시작만으로는 녹화 안 함)
      │                                      │
      │  start_episode (Trigger)             │
      ├─ topic: state="IN_EPISODE" ─────────►│ recorder.start() → 녹화 시작
      │                                      │ ← 이미지 프레임 recorder.write()
      │  stop_episode (Trigger)              │
      ├─ topic: state="IN_SESSION" ─────────►│ 큐 flush → recorder.stop() → MP4 저장
      │                                      │
      │  start_episode → stop_episode        │
      ├─ (반복 가능) ──────────────────────►│ 새 MP4 파일로 녹화
      │                                      │
      │  stop_session (Trigger)              │
      ├─ topic: state="IN_SESSION" ─────────►│ 녹화 중이면 stop 처리
      │  topic: state="IDLE"                 │ (IDLE은 무시)
```

- `IN_EPISODE` → 녹화 시작
- `IN_SESSION` → 녹화 종료 (녹화 중일 때만)
- `IDLE` → 무시
- 에피소드를 반복하면 매번 새 MP4 파일이 생성됨

---

## 타임스탬프 기반 녹화 경계

### 문제

네트워크 지연으로 `start_episode` 메시지가 도착하기 전에 이미 `start_ts` 이후의
이미지가 도착할 수 있다. `stop_episode`도 마찬가지.

### 해결: `pending_image_queue`

이미지가 도착하면 즉시 녹화하지 않고 큐에 버퍼링한다:

1. **큐 삽입**: 모든 이미지는 `pending_image_queue`에 먼저 들어감
2. **overflow 처리**: 큐가 가득 차면 가장 오래된 프레임(victim)이 밀려남
   - 녹화 중: `frame_ts >= start_ts`이면 recorder에 전달, 아니면 drop
   - 비녹화 중: `frame_ts < stop_ts`이면 recorder에 전달, 아니면 drop
3. **start 도착 시**: 큐에서 `frame_ts < start_ts`인 프레임 제거
4. **stop 도착 시**: 큐에서 `frame_ts < stop_ts`인 프레임을 recorder에 전달

### `pending_queue_length` 튜닝

| 값 | 효과 |
|---|---|
| 작음 (예: 10) | 메모리 절약, 경계 정밀도 낮음 (프레임 누락/초과 가능) |
| 기본값 (60) | 30fps에서 2초 버퍼, 일반적인 네트워크 지연 보상 |
| 큼 (예: 120) | 경계 정밀도 높음, 메모리 사용 증가 |

경계 정밀도 손실 시 WARNING 로그가 출력된다.

---

## 출력 파일

녹화 1 회(`IN_EPISODE` → `IN_SESSION`)마다 다음 **3 개 파일**이 생성된다.
`start_ts` 는 `_build_output_path()` 가 녹화 시작 시점에 생성한
`YYYYMMDD-HHMMSS.SSS` 타임스탬프 문자열이다 (예: `20260416-153012.456`).

```
<output_dir>/
├── <session_prefix>_<start_ts>.mp4      ← 영상
├── <session_prefix>_<start_ts>.jsonl    ← frame-level sidecar
└── <session_prefix>_metadata.json       ← recording metadata (start_ts 미포함)
```

> `_metadata.json` 파일은 `<start_ts>` 가 없으므로 **같은 디렉터리에서
> 여러 번 녹화하면 매 `stop` 시 덮어쓰여진다**. 세션별 보존이 필요하면
> `output_dir` 또는 `session_prefix` 를 다르게 지정한다.

### MP4 파일

기존과 동일. `FFMpegMp4Recorder` 를 통해 H.264/H.265 로 인코딩된 lossy
영상. CFR(Constant Frame Rate) 기반이므로 `fps` 파라미터가 실제 입력
frame rate 와 일치해야 한다.

### Sidecar 파일 (`.jsonl`)

매 프레임이 recorder 에 성공적으로 기록될 때마다 한 줄씩 append 되는
JSON Lines 파일. mp4 의 N번째 프레임과 sidecar 의 N번째 라인이 1:1 매치된다.

**스키마 (한 줄)**
```json
{"frame_index": 0, "stamp": {"sec": 1713340800, "nanosec": 123456789}}
```

| 필드 | 타입 | 설명 |
|------|------|------|
| `frame_index` | int | 0부터 시작하는 순번. mp4 프레임 인덱스와 동일 |
| `stamp.sec` | int | `header.stamp.sec` (ROS 원본) |
| `stamp.nanosec` | int | `header.stamp.nanosec` (ROS 원본) |

`recorder.write()` 가 실패한 프레임은 sidecar 에도 기록되지 않으므로
1:1 매치 불변식이 유지된다.

### Recording metadata 파일 (`.json`)

녹화 종료(`_handle_stop`) 시 1 회 생성되는 요약 JSON.

**스키마**
```json
{
  "resolution": "640x480",
  "encoding": "bgr8",
  "frame_id": "camera_color_optical_frame",
  "is_bigendian": false,
  "nframe": 450,
  "start_ts": {"sec": 1713340800, "nanosec": 123456789},
  "end_ts":   {"sec": 1713340844, "nanosec": 987654321}
}
```

| 필드 | 타입 | 출처 | 설명 |
|------|------|------|------|
| `resolution` | string | 파라미터 | `"WIDTHxHEIGHT"` |
| `encoding` | string | 파라미터 (`pixel_format`) | `bgr8`/`rgb8`/`mono8` 등 |
| `frame_id` | string | 첫 유효 이미지의 `header.frame_id` | TF 좌표계 이름 |
| `is_bigendian` | bool | 첫 유효 이미지의 `is_bigendian` | 8-bit encoding 에서는 거의 항상 `false` |
| `nframe` | int | 실제 기록된 프레임 수 | sidecar 라인 수와 동일 |
| `start_ts` | object | 첫 프레임 stamp | `{sec, nanosec}`. 프레임 0 개면 생략 |
| `end_ts` | object | 마지막 프레임 stamp | `{sec, nanosec}`. 프레임 0 개면 생략 |

### 원본 메시지 복원 가능 범위

| 항목 | 복원 가능? | 출처 |
|------|-----------|------|
| `header.stamp` | ✅ | sidecar `stamp` |
| `header.frame_id` | ✅ | metadata `frame_id` |
| `width`, `height` | ✅ | metadata `resolution` |
| `encoding` | ✅ | metadata `encoding` |
| `is_bigendian` | ✅ | metadata `is_bigendian` |
| `step` | ⚠️ | `width × channels` 로 재계산 (원본 padding 유실) |
| `data` (픽셀) | ⚠️ | mp4 lossy 압축 — 시각적으로 거의 동일하나 bit-exact 아님 |

---

## 토픽 연결

### 구독 토픽

| 토픽 | 타입 | QoS | 연결 방법 |
|------|------|-----|-----------|
| `image` | `sensor_msgs/Image` | `sensor_data` (BEST_EFFORT, KEEP_LAST 5) | `-r image:=/<publisher_topic>` |
| `session` | `rdfp_msgs/SessionCommand` | `TRANSIENT_LOCAL / RELIABLE / KEEP_LAST(1)` | 기본 `/session` (remap 필요 시 `-r session:=/<topic>`) |

`session` 토픽 QoS 는 `SessionControlNode` 발행 QoS 와 동일하게 내부 고정되어
있어, 퍼블리셔가 먼저 기동되고 레코더가 나중에 떠도 직전 상태를 즉시 수신한다.

### 서비스

이 노드는 서비스를 제공하지 않는다. 녹화 제어는 세션 토픽 구독으로 자동 수행된다.

---

## 에러 처리

| 상황 | 동작 |
|------|------|
| 필수 파라미터 누락 | `[FATAL]` → 노드 종료 |
| GPU 인코더 미사용 가능 (`encoder_mode=gpu`) | `[FATAL]` → 노드 종료 |
| `output_dir` 생성 실패 | `[FATAL]` → 노드 종료 |
| encoding/해상도 불일치 이미지 | WARNING (5초 throttle) + drop |
| `recorder.write()` 예외 | WARNING + drop (sidecar 에도 미기록 → 1:1 매치 유지) |
| `recorder.start()`/`stop()` 예외 | ERROR 로그 |
| stop 후 뒤늦은 녹화 대상 프레임 | WARNING + drop |
| 큐 overflow 경계 정밀도 손실 | WARNING |
| sidecar open/write 실패 | ERROR/WARNING 로그 (녹화는 계속 진행, metadata 만 비기록) |
| metadata 작성 실패 | ERROR 로그 (mp4/sidecar 는 정상 유지) |
| SIGINT/SIGTERM | 큐 flush → `recorder.stop()` → sidecar close + metadata 작성 → `recorder.shutdown()` |

---

## 실전 예제

### 기본 사용

```bash
ros2 run rdfp rdfp_image_recorder --ros-args \
  -r image:=/camera_node/image_raw \
  -p output_dir:=/tmp/recordings \
  -p fps:=30 \
  -p resolution:=640x480
```

### GPU 인코딩

```bash
ros2 run rdfp rdfp_image_recorder --ros-args \
  -r image:=/camera_node/image_raw \
  -p output_dir:=/tmp/recordings \
  -p fps:=30 \
  -p resolution:=1920x1080 \
  -p encoder_mode:=auto \
  -p bitrate:=8M
```

### 통합 Launch (`rdfp.launch.py`)

session_control + `rdfp_camera_node` + `rdfp_image_viewer_node` +
`rdfp_image_recorder` 를 한 번에 띄우는
[rdfp.launch.py](../../src/rdfp/launch/rdfp.launch.py) 사용. 본 가이드의
`RdfpImageRecorderNode` 가 launch 의 **기본 레코더**다.

```bash
# 기본 구성 — RdfpImageRecorderNode 가 세션 토픽 기반으로 자동 제어됨
ros2 launch rdfp rdfp.launch.py

# 녹화 안 하고 카메라 + 뷰어만
ros2 launch rdfp rdfp.launch.py enable_image_recorder_node:=false

# 출력 디렉터리 변경 + fps 맞추기 (카메라 fps 와 반드시 일치)
ros2 launch rdfp rdfp.launch.py \
  image_recorder_output_dir:=/data/recordings \
  image_recorder_fps:=30 \
  camera_fps:=30
```

launch 인자:

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `enable_image_recorder_node` | `true` | 레코더 기동 여부 |
| `image_recorder_fps` | `10` | 레코더 `fps` 파라미터 — 카메라 fps 와 일치시켜야 한다 |
| `image_recorder_output_dir` | `/tmp/recordings` | MP4 / sidecar / metadata 저장 경로 |

> resolution 은 `camera_resolution` 인자를 레코더에 그대로 넘겨 카메라/
> 레코더 해상도 불일치로 인한 프레임 drop 을 방지한다. 이미지 토픽은
> `camera_image_topic` (기본 `/camera/image_raw`) 으로 remap 된다.

### 직접 Launch 파일 작성

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rdfp',
            executable='session_control_node',
            name='session_control',
        ),
        Node(
            package='rdfp',
            executable='camera_node',
            parameters=[{
                'camera_id': '0',
                'fps': 30.0,
                'resolution': '640x480',
            }],
            remappings=[('image', '/camera_node/image_raw')],
        ),
        Node(
            package='rdfp',
            executable='rdfp_image_recorder',
            parameters=[{
                'output_dir': '/tmp/recordings',
                'fps': 30,                 # 카메라 fps 와 반드시 일치
                'resolution': '640x480',
                'encoder_mode': 'auto',
                'pending_queue_length': 60,
            }],
            remappings=[('image', '/camera_node/image_raw')],
            # session 토픽은 기본이 /session 이므로 remap 불필요
        ),
    ])
```

---

## 트러블슈팅

### 1. 녹화가 시작되지 않음

**확인:**
```bash
# 세션 토픽이 발행되는지 확인 (TRANSIENT_LOCAL QoS)
ros2 topic echo /session --qos-durability transient_local --qos-reliability reliable

# 세션 토픽 publisher / subscriber 엔드포인트 확인
ros2 topic info /session -v

# 이미지 토픽이 발행되는지 확인
ros2 topic hz /camera_node/image_raw
```

- `start_episode` 가 호출되었는지 확인 (세션 토픽에 `IN_EPISODE` 상태가 나타나야 함)
- 이미지 토픽 remap 이 올바르게 설정되었는지 확인
- 세션 토픽은 기본이 `/session` — 다른 이름을 쓴다면 레코더에 remap 필요

### 2. 0프레임 MP4

**원인**: 이미지 토픽의 encoding/해상도가 파라미터와 불일치.

```bash
# 실제 이미지 정보 확인
ros2 topic echo /camera_node/image_raw --field encoding --once
ros2 topic echo /camera_node/image_raw --field width --once
ros2 topic echo /camera_node/image_raw --field height --once
```

`pixel_format`, `resolution` 파라미터가 실제 값과 정확히 일치해야 한다.

### 3. 프레임 드롭

`FFMpegMp4Recorder` 내부 큐(`queue_size`)가 가득 차면 오래된 프레임을 버린다.

**해결:**
- `encoder_mode:=auto`로 GPU 인코더 사용
- `queue_size` 증가
- `preset:=fast` 또는 `ultrafast`

### 4. 경계 정밀도 문제

`pending_queue_length`가 짧으면 start/stop 경계 근처 프레임이 누락되거나 초과될 수 있다.

```
[WARN] late frame after stop: frame_ts < stop_ts, dropping
```

**해결:** `pending_queue_length`를 늘린다 (기본 60, 30fps에서 2초 버퍼).

### 5. `start received but already recording, ignoring`

`TRANSIENT_LOCAL` QoS로 인해 세션 토픽의 직전 상태가 구독 시점에 즉시
전달되어 발생할 수 있다. 정상 동작이며 무시해도 안전하다.

---

## 관련 문서

- [ImageRecorderNode Guide](./image_recorder_node_guide.md) — 서비스 기반 레코더 (세션 비의존)
- [FFMpegMp4Recorder Guide](./ffmpeg_mp4_recorder_guide.md) — 녹화 엔진 상세
- [SessionControlNode Guide](../session/session_control_guide.md) — 세션 제어 노드
- [CameraNode Guide](../camera/camera_node_guide.md) — 이미지 토픽 발행자
- [RdfpImageViewerNode Guide](../camera/rdfp_image_viewer_node_guide.md) — 세션 상태 오버레이 뷰어
- [rdfp.launch.py](../../src/rdfp/launch/rdfp.launch.py) — session_control + camera + viewer + recorder 통합 launch
