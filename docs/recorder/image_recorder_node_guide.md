# ImageRecorderNode — Programmer's Guide

ROS2 이미지 토픽(`sensor_msgs/Image`)을 MP4 파일로 녹화하는 ROS2 노드 사용 가이드.

---

## 목차

1. [개요](#개요)
2. [사전 요구사항](#사전-요구사항)
3. [Quick Start](#quick-start)
4. [파라미터](#파라미터)
5. [서비스 인터페이스](#서비스-인터페이스)
6. [토픽](#토픽)
7. [녹화 생명주기](#녹화-생명주기)
8. [프레임 검증 및 자동 종료](#프레임-검증-및-자동-종료)
9. [노드 종료 시 동작](#노드-종료-시-동작)
10. [실전 예제](#실전-예제)
11. [에러 처리](#에러-처리)
12. [트러블슈팅](#트러블슈팅)

---

## 개요

`ImageRecorderNode`는 ROS2 이미지 토픽을 구독하여 `FFMpegMp4Recorder`로
MP4 파일을 생성하는 어댑터 노드이다.

**핵심 설계:**
- **서비스 기반 제어**: `~/start_session`, `~/stop_session` 서비스로 녹화 시작/종료
- **자동 시작 옵션**: `auto_start=true` 로 노드 기동과 동시에 녹화 시작 가능
- **파라미터 기반 설정**: fps, 해상도, 인코더 등을 ROS2 파라미터로 지정
- **해상도 자동 추론**: `resolution` 미지정 시 첫 수신 이미지의 해상도를 사용 (recorder lazy 생성)
- **자동 파일명 생성**: `<prefix>_YYYYMMDD-HHMMSS.SSS.mp4` 형식, `session_prefix` 는 경로 탈출 불가하도록 검증
- **자동 세션 종료**: 연속 불일치 프레임이 누적되면 자동으로 녹화 중단
- **콜백 안전성**: 서비스 핸들러에서 발생하는 예외는 모두 흡수되어 응답으로 변환됨
- **안전한 종료**: SIGINT/SIGTERM 시에도 녹화 파일이 올바르게 finalize됨

---

## 사전 요구사항

### 시스템

```bash
# ffmpeg 설치
sudo apt install ffmpeg

# rdfp_msgs 빌드 (StartSession, StopSession 서비스 정의)
colcon build --packages-select rdfp_msgs rdfp
source install/setup.bash
```

### 이미지 소스

녹화 대상인 이미지 토픽이 publish되고 있어야 한다.
카메라 노드, RViz의 이미지 publish, rosbag 재생 등 어떤 소스든 가능하다.

---

## Quick Start

```bash
# 터미널 1: 카메라 노드 실행
ros2 run rdfp camera_node --ros-args -p camera_id:=0 -p fps:=30 -p resolution:=640x480

# 터미널 2: 레코더 노드 실행
ros2 run rdfp image_recorder_node --ros-args \
  -p fps:=30 \
  -p resolution:=640x480 \
  -p output_dir:=/tmp/recordings \
  -r image:=/camera_node/image_raw

# 터미널 3: 녹화 제어
ros2 service call /image_recorder/start_session rdfp_msgs/srv/StartSession
# → success: true, mp4_path: "/tmp/recordings/session_20260416-143025.123.mp4"

# 10초 후 중단
ros2 service call /image_recorder/stop_session rdfp_msgs/srv/StopSession
# → success: true, mp4_path: "/tmp/recordings/session_20260416-143025.123.mp4"
```

`auto_start=true` 로 기동 즉시 녹화를 시작할 수도 있다 (서비스 호출 없이):

```bash
ros2 run rdfp image_recorder_node --ros-args \
  -p fps:=30 -p resolution:=640x480 \
  -p auto_start:=true \
  -r image:=/camera_node/image_raw
```

`resolution` 파라미터를 생략하면 첫 수신 이미지의 해상도를 자동으로 사용한다:

```bash
ros2 run rdfp image_recorder_node --ros-args \
  -p fps:=30 \
  -p auto_start:=true \
  -r image:=/camera_node/image_raw
# → 첫 프레임 도착 시 그 해상도로 recorder 를 생성하고 녹화 시작
```

---

## 파라미터

### 필수 파라미터

없음. 모든 파라미터가 기본값을 가진다.

### 선택 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `fps` | int | `10` | 녹화 프레임 레이트 (CFR). **입력 이미지 스트림의 실제 frame rate 와 일치시켜야 한다** — 불일치 시 재생 속도 왜곡 또는 프레임 drop 발생 |
| `resolution` | string | `None` | 녹화 해상도 `"WIDTHxHEIGHT"`. 미지정 시 첫 수신 이미지 해상도를 사용 (recorder lazy 생성) |
| `output_dir` | string | `"/tmp/recordings"` | MP4 파일 저장 디렉터리. 없으면 자동 생성 |
| `session_prefix` | string | `"recording"` | 파일명 접두사. 경로 구분자(`/`, `\`)·`.`·`..`·빈 문자열 금지 (기동 시 검증) |
| `pixel_format` | string | `"bgr8"` | 입력 이미지 encoding (`bgr8`, `rgb8`, `mono8`) |
| `encoder_mode` | string | `"auto"` | `"auto"` / `"cpu"` / `"gpu"` |
| `queue_size` | int | `120` | 프레임 백프레셔 큐 크기 |
| `auto_start` | bool | `false` | `true` 면 노드 기동 시 `~/start_session` 경로를 자동 호출 |

> ⚠ `fps` 기본값(10)은 보수적인 값이다. 카메라가 30fps 로 발행한다면 반드시
> `-p fps:=30` 으로 맞춰야 한다. 불일치 시 `FFMpegMp4Recorder` 내부 CFR 타이밍이
> 어긋나 저장된 영상의 재생 속도가 실시간과 맞지 않는다.

### `resolution` 동작 방식

- **지정** (예: `-p resolution:=640x480`): `__init__` 시점에 recorder 를 즉시 생성하고
  인코더 probe 를 수행한다. 프레임 해상도가 다르면 invalid 로 drop 된다.
- **미지정** (`None`): recorder 생성이 첫 이미지 도착까지 지연된다. `start_session` 이
  호출되면 pending 상태로 진입해 경로만 예약하고, 첫 유효 이미지가 도착하면 그
  해상도로 recorder 를 만들어 `recorder.start(pending_path)` 를 호출한다. 이후
  프레임은 추론된 해상도에 일치해야 한다.

### 파라미터 예시

```bash
ros2 run rdfp image_recorder_node --ros-args \
  -p fps:=30 \
  -p resolution:=1920x1080 \
  -p output_dir:=/home/user/recordings \
  -p session_prefix:=experiment_01 \
  -p pixel_format:=bgr8 \
  -p encoder_mode:=auto \
  -p queue_size:=240 \
  -p auto_start:=false
```

생성되는 파일명: `/home/user/recordings/experiment_01_20260416-143025.123.mp4`

`session_prefix` 는 단순 파일명 조각만 허용된다. 다음은 기동 시점에 거부되는 예다:

| 입력값 | 사유 |
|---|---|
| `""` | 빈 문자열 |
| `"."` / `".."` | 디렉터리 참조 |
| `"/tmp/pwn"` | 절대 경로 (path separator 포함) |
| `"nested/foo"` | 하위 경로 (path separator 포함) |
| `"foo\\bar"` | 백슬래시 포함 |

---

## 서비스 인터페이스

### `~/start_session` (`rdfp_msgs/srv/StartSession`)

녹화 세션을 시작한다. 요청 필드는 없으며, 세션 설정은 노드 파라미터로 결정된다.

**Request**: (비어있음)

**Response**:

| 필드 | 타입 | 설명 |
|------|------|------|
| `success` | bool | 녹화 시작 성공 여부 |
| `mp4_path` | string | 성공 시 MP4 파일 절대 경로, 실패 시 빈 문자열 |

**실패 조건:**
- 이미 녹화 중 (`RECORDING` 상태)
- pending 시작이 이미 진행 중 (deferred 모드에서 이전 `start_session` 의 첫 이미지를 대기 중)
- 파일이 이미 존재 (`FileExistsError` — 밀리초 단위 타임스탬프가 충돌하는 극히 드문 경우)
- recorder 내부 오류 (`RecorderStateError` 등)

`FileExistsError` / `RecorderStateError` / 그 외 예외는 모두 핸들러 내부에서
흡수되어 `success=false` 응답으로 변환된다 — 서비스 콜백 밖으로 전파되지 않는다.

**deferred 모드** (`resolution` 미지정): recorder 가 아직 없으면 핸들러는
경로만 예약하고 pending 상태(`success=true`, `mp4_path=<예약 경로>`)로 응답한다.
실제 파일 생성·녹화 시작은 첫 유효 이미지 도착 시점에 일어난다. 첫 이미지가
검증에 실패(`width`/`height` ≤ 0 또는 encoding 불일치)하면 로그만 남기고
pending 을 유지하며 다음 프레임에서 재시도한다. recorder 생성/시작 자체가
예외를 던지면 pending 을 클리어하고 ERROR 로그를 남긴다 (사용자 수동 재시도 가능).

```bash
# CLI
ros2 service call /image_recorder/start_session rdfp_msgs/srv/StartSession

# Python
from rdfp_msgs.srv import StartSession
client = node.create_client(StartSession, '/image_recorder/start_session')
future = client.call_async(StartSession.Request())
```

### `~/stop_session` (`rdfp_msgs/srv/StopSession`)

녹화 세션을 종료하고 MP4 파일을 finalize한다.

**Request**: (비어있음)

**Response**:

| 필드 | 타입 | 설명 |
|------|------|------|
| `success` | bool | 정상 종료 여부 (`stop()` 후 `IDLE` 복귀 시 true) |
| `mp4_path` | string | 성공 시 finalize된 MP4 파일 경로, 실패 시 빈 문자열 |

**실패 조건:**
- 녹화 중이 아님 (recorder 가 `RECORDING` 상태가 아니거나 아직 존재하지 않음)
- `recorder.stop()` 후 `FAILED` 상태로 전이
- `recorder.stop()` 자체가 예외를 던짐 (writer 스레드 race 등)

**deferred pending 취소**: `start_session` 이 pending 상태(첫 이미지 대기 중)에서
`stop_session` 이 호출되면, 예약을 취소하고 `success=false` 를 반환한다
(아직 녹화가 시작되지 않아 파일 없음). pending 상태 변수(`_pending_start`,
`_pending_mp4_path`)가 함께 리셋되어 다음 `start_session` 을 새로 호출할 수 있다.

위 모든 실패 경로에서 핸들러는 `success=false` 응답을 채워 반환하며,
세션 상태 변수(`_current_mp4_path`, `_consecutive_invalid`, `_invalid_log_count`)도
함께 리셋되어 다음 `start_session` 이 깨끗한 상태에서 시작된다.

```bash
ros2 service call /image_recorder/stop_session rdfp_msgs/srv/StopSession
```

---

## 토픽

### 구독 토픽

| 토픽 | 타입 | QoS | 설명 |
|------|------|-----|------|
| `image` | `sensor_msgs/Image` | sensor_data | 녹화 대상 이미지 |

토픽 이름은 `image`로 고정되며, ROS2 remap으로 실제 토픽에 연결한다:

```bash
# 카메라 raw 이미지에 연결
ros2 run rdfp image_recorder_node --ros-args \
  -r image:=/camera_node/image_raw \
  -p fps:=30 -p resolution:=640x480

# depth 이미지에 연결 (mono8)
ros2 run rdfp image_recorder_node --ros-args \
  -r image:=/camera/depth/image_rect_raw \
  -p fps:=30 -p resolution:=640x480 -p pixel_format:=mono8
```

---

## 녹화 생명주기

```
Node start
  │
  ├─ Load parameters (validate session_prefix)
  ├─ Create output_dir
  ├─ if resolution is set → Create FFMpegMp4Recorder (encoder probe)
  │   else                → Defer recorder creation
  ├─ Subscribe to image topic
  ├─ Register services
  └─ if auto_start=true → invoke _handle_start_session()
       │
       ▼
  ┌─ Idle (receive images but drop) ◄───────────────┐
  │                                                 │
  │  ~/start_session called (or auto_start)         │
  ▼                                                 │
  ┌────────────────────────────────────────┐        │
  │ (deferred) Pending start               │        │
  │  — wait for first valid image          │        │
  │  — infer resolution, lazy-create       │        │
  │    recorder, call recorder.start()     │        │
  │  ~/stop_session → cancel pending ──────┤        │
  └────────────────┬───────────────────────┘        │
                   ▼                                │
  Recording (images → recorder.write())             │
  │                                                 │
  ├─ ~/stop_session called ────► finalize ──────────┘
  ├─ 5 consecutive mismatches ──► auto stop ────────┘
  └─ SIGINT/SIGTERM ───────────► destroy_node()
```

- 녹화 중이 아닐 때 수신되는 이미지는 **로그 없이 조용히 버려진다**
- `stop_session` 후 recorder는 `IDLE`로 복귀하며, 다시 `start_session`으로 새 파일 녹화 가능
- `FAILED` 상태에서도 `start_session`으로 복구 가능
- `auto_start=true` 시 자동 시작이 실패해도 노드는 살아 있으며, 사용자가 수동으로 `~/start_session` 호출로 재시도할 수 있다
- deferred 모드에서 첫 이미지가 유효하지 않으면 invalid 카운터가 누적되어 최종적으로 auto-stop 으로 빠질 수 있다

---

## 프레임 검증 및 자동 종료

이미지 콜백에서 다음을 검증한다 (deferred 모드의 첫 이미지는 추가 검증이 앞에 붙는다):

1. **(deferred only) 첫 이미지 해상도 유효성**: `msg.width`/`msg.height` > 0
2. **(deferred only) 첫 이미지 encoding 일치**: `msg.encoding` == `pixel_format`
3. **encoding 일치**: `msg.encoding`이 `pixel_format` 파라미터와 동일한지
4. **해상도 일치**: `msg.width`, `msg.height`가 `_effective_resolution`(파라미터 또는 추론값)과 동일한지
5. **버퍼 크기**: `msg.data` 가 `step * height` 이상인지
6. **stride 검증**: `msg.step` 이 한 행에 필요한 `width * channels` 이상인지 (mono8 의
   silent slicing 같은 회피 케이스 차단)

위 어느 하나라도 실패한 프레임은 recorder에 전달되지 않고 drop된다. deferred
모드에서 1·2번 실패는 recorder 를 아직 생성하지 않은 상태이므로 pending 을 유지하고
다음 프레임을 기다린다 (단, invalid 카운터는 누적된다).

### 로그 억제 정책

프레임 불일치 로그는 폭주를 방지하기 위해 억제된다:
- **최초 1회** ERROR 로그
- **이후 100건마다** 1회 ERROR 로그

### 자동 세션 종료

**연속 5회** 불일치 프레임이 발생하면 세션을 자동으로 종료한다.
이는 카메라 설정이 런타임에 바뀌거나, 잘못된 토픽을 구독한 경우를 감지하기 위한 안전장치이다.
인코딩별 처리 정책이 일관되어 (bgr8/rgb8 과 mono8 모두) 같은 종류의 malformed
frame은 동일하게 카운터에 반영된다.

자동 종료 후에도 `start_session`으로 새 녹화를 시작할 수 있다.

---

## 노드 종료 시 동작

`destroy_node()`가 호출되면:

1. 녹화 중이면 `recorder.stop(timeout=5.0)`으로 MP4를 finalize
2. `recorder.shutdown()`으로 ffmpeg 프로세스와 writer 스레드 정리
3. 부모 `Node.destroy_node()` 호출

이 흐름은 SIGINT, SIGTERM, `rclpy.shutdown()` 경로 모두에서 `main()`의 `finally` 블록으로 보장된다.

---

## 실전 예제

### 카메라 녹화 기본

```bash
# 카메라 + 레코더 동시 실행
ros2 run rdfp camera_node --ros-args \
  -p camera_id:=0 -p fps:=30 -p resolution:=640x480 &

ros2 run rdfp image_recorder_node --ros-args \
  -p fps:=30 -p resolution:=640x480 \
  -r image:=/camera_node/image_raw

# 다른 터미널에서 녹화 제어
ros2 service call /image_recorder/start_session rdfp_msgs/srv/StartSession
sleep 10
ros2 service call /image_recorder/stop_session rdfp_msgs/srv/StopSession
```

### 자동 시작 (서비스 호출 없이 바로 녹화)

```bash
ros2 run rdfp image_recorder_node --ros-args \
  -p fps:=30 -p resolution:=640x480 \
  -p auto_start:=true \
  -p output_dir:=/tmp/recordings \
  -r image:=/camera_node/image_raw
# 노드 기동 즉시 녹화 시작 → SIGINT 시 finalize
```

스크립트/launch 에서 "기동만 하면 곧장 녹화" 워크플로우에 적합하다. 종료는 SIGINT
또는 명시적 `~/stop_session` 호출 둘 다 가능하다.

### 해상도 자동 추론 (resolution 생략)

```bash
ros2 run rdfp image_recorder_node --ros-args \
  -p fps:=30 \
  -p auto_start:=true \
  -r image:=/camera_node/image_raw
# → pending 상태 진입 → 첫 유효 이미지 도착 시 해상도 추론 + 녹화 시작
```

카메라 해상도를 사전에 알 수 없거나 공용 launch 파일을 여러 카메라와 함께
쓸 때 유용하다. 단, 첫 프레임이 도착하기 전까지는 pending 상태이므로
`start_session` 응답은 `success=true` + 예약 경로를 돌려주지만 실제 파일은
아직 생성되지 않는다.

### GPU 인코딩 사용

```bash
ros2 run rdfp image_recorder_node --ros-args \
  -p fps:=30 -p resolution:=1920x1080 \
  -p encoder_mode:=auto \
  -r image:=/camera_node/image_raw
```

`auto` 모드는 GPU 인코더(h264_nvenc, h264_qsv, h264_vaapi)를 우선 시도하고,
없으면 CPU(libx264)로 자동 폴백한다.

### Python에서 서비스 호출

```python
import rclpy
from rclpy.node import Node
from rdfp_msgs.srv import StartSession, StopSession


def main():
    rclpy.init()
    node = Node('recorder_client')

    start_client = node.create_client(StartSession, '/image_recorder/start_session')
    stop_client = node.create_client(StopSession, '/image_recorder/stop_session')

    # 서비스 대기
    start_client.wait_for_service(timeout_sec=5.0)

    # 녹화 시작
    future = start_client.call_async(StartSession.Request())
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    if result.success:
        print(f'Recording to: {result.mp4_path}')

    # ... 녹화 진행 ...

    # 녹화 종료
    future = stop_client.call_async(StopSession.Request())
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    if result.success:
        print(f'Saved: {result.mp4_path}')

    node.destroy_node()
    rclpy.shutdown()
```

### Launch 파일에서 사용

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rdfp',
            executable='camera_node',
            parameters=[{
                'camera_id': '0',
                'fps': 30.0,
                'resolution': '640x480',
            }],
        ),
        Node(
            package='rdfp',
            executable='image_recorder_node',
            parameters=[{
                'fps': 30,
                'resolution': '640x480',
                'output_dir': '/tmp/recordings',
                'encoder_mode': 'auto',
                'auto_start': True,  # launch 와 동시에 녹화 시작
            }],
            remappings=[
                ('image', '/camera_node/image_raw'),
            ],
        ),
    ])
```

---

## 에러 처리

### 노드 초기화 실패

노드 생성 시 다음 경우 `RuntimeError`/`ValueError`가 발생하고 프로세스가 종료된다:

| 원인 | 메시지 예시 |
|------|------------|
| `fps` 가 양의 정수가 아님 | `'fps' must be a positive integer: ...` |
| 잘못된 해상도 형식 (지정 경로) | `invalid recorder configuration: ...` |
| GPU 인코더 미사용 가능 (지정 경로) | `GPU encoder unavailable: ...` |
| 출력 디렉터리 생성 실패 | `failed to create output_dir ...: Permission denied` |
| `session_prefix` 가 경로 구분자/`.`/`..`/빈 문자열 포함 | `session_prefix must not contain path separators: '...'` |

> `resolution` 미지정 경로에서는 인코더 probe 가 첫 이미지 도착 시점까지 지연된다.
> 따라서 `GPU encoder unavailable` 같은 오류는 노드 기동 직후가 아니라 첫 프레임
> 도착 시점의 ERROR 로그로 나타나며, pending 이 클리어되어 노드는 계속 실행된다.

### auto_start 실패

`auto_start=true` 인 상태에서 자동 시작이 실패해도 노드는 종료되지 않는다.
`auto_start failed; waiting for manual ~/start_session call` ERROR 로그가 남고,
이후 사용자가 `~/start_session` 서비스 호출로 재시도할 수 있다.

### 서비스 호출 실패

서비스는 예외를 발생시키지 않고 `response.success = false`로 실패를 보고한다.
`recorder.stop()` 이 race condition 으로 던진 `RecorderStateError` 도 핸들러
내부에서 흡수되어 응답으로 변환된다. 실패 원인은 노드 로그에 ERROR/WARNING으로 기록된다.

---

## 트러블슈팅

### 1. `start_session`이 항상 실패함

**확인 사항:**
- `ros2 param get /image_recorder fps` — 파라미터가 올바르게 설정되었는지
- `ros2 node info /image_recorder` — 서비스가 등록되었는지
- 노드 로그에서 `start_session failed` 또는 `start_session rejected` 메시지 확인

### 2. 녹화는 시작되지만 0프레임

**원인**: 이미지 토픽이 remap되지 않았거나 encoding/해상도가 불일치.

**확인:**
```bash
# 구독 중인 토픽 확인
ros2 topic info /image_recorder/image

# 실제 토픽의 encoding/해상도 확인
ros2 topic echo /camera_node/image_raw --field encoding --once
ros2 topic echo /camera_node/image_raw --field width --once
ros2 topic echo /camera_node/image_raw --field height --once
```

`pixel_format`, `resolution` 파라미터가 실제 토픽의 값과 정확히 일치해야 한다.

### 3. `auto-stopping session: 5 consecutive invalid frames`

**원인**: 수신되는 이미지의 encoding 또는 해상도가 파라미터와 5회 연속 불일치.
`step < width*channels` 로 잘못 채워진 메시지도 invalid 로 카운트된다.

**해결**:
- 위 "0프레임" 확인 사항과 동일
- 카메라가 런타임에 해상도를 변경했는지 확인
- publisher 가 stride padding 을 잘못 계산하고 있는지 확인 (`msg.step` vs `msg.width`)

### 4. 프레임 드롭이 많음

`FFMpegMp4Recorder`의 큐가 가득차면 오래된 프레임을 버린다.

**해결:**
- `queue_size` 파라미터 증가 (기본 120)
- `encoder_mode:=auto`로 GPU 인코더 사용
- 입력 fps/해상도 낮추기

### 5. SIGINT 후 MP4가 재생 불가

정상적으로 `destroy_node()`가 호출되면 `recorder.stop()`으로 finalize된다.
그러나 `kill -9` 등으로 강제 종료하면 finalize가 생략되어 파일이 손상될 수 있다.

### 6. `stop_session failed: state race`

writer 스레드가 RECORDING → FAILED 로 전이하는 도중 `stop_session` 이 호출된 경합 상황이다.
서비스는 `success=false` 로 응답하고 노드는 정상 동작을 계속한다.
이전 세션의 출력 파일은 손상됐을 가능성이 높으니 검증 후 폐기하고 새 `start_session` 을 호출한다.

### 7. `start_session` 은 성공했는데 파일이 안 생김

deferred 모드(`resolution` 미지정)의 정상 동작이다. pending 상태에서는 경로만
예약되고 첫 이미지 도착 시 실제 파일이 만들어진다.

**확인:**
- 이미지 토픽에 publisher 가 있고 메시지가 흐르는지 (`ros2 topic hz /camera_node/image_raw`)
- 노드 로그에 `resolution inferred from first image: ...` 또는
  `deferred session started: path=...` 가 떴는지
- 첫 이미지 validation 실패가 반복되면 `first image has invalid resolution ...` /
  `first image encoding ... does not match` ERROR 로그 확인

### 8. `session_prefix must not contain path separators`

파라미터로 `session_prefix` 에 `/`, `\`, `.`, `..`, 빈 문자열 등을 전달한 경우 기동 시점에
`ValueError` 로 거부된다. 이는 `os.path.join` 으로 합쳐질 때 절대 경로(`/tmp/...`)나 예기치 않은
하위 디렉터리 경로(`nested/foo`)가 만들어져 `output_dir` 를 우회하는 것을 막기 위한 안전장치다.

**해결:** 단순 파일명 조각만 사용한다. 예: `session`, `experiment_01`, `cam0-run`.

---

## 관련 문서

- [FFMpegMp4Recorder Programmer's Guide](./ffmpeg_mp4_recorder_guide.md) — 녹화 엔진 상세
- [OpenCvCamera Programmer's Guide](../camera/opencv_camera_guide.md) — 카메라 입력 소스
