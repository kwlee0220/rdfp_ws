# 카메라 노드 가이드 — `camera_node` · `image_capture_node`

`robot_control` 의 카메라 노드 둘을 다룬다. 둘 다 하드웨어 추상화는
[`OpenCvCamera`](opencv_camera_guide.md) 하나를 쓴다. **세션에 맞춰 켜고 끄는 노드는 여기 없다** —
`rdfp_camera_node` · `rdfp_image_viewer_node` 는 수집 계층이라
[../collect/session_camera_nodes_guide.md](../collect/session_camera_nodes_guide.md) 가 갖는다.

---

## 목차

- [0. 어느 노드를 쓸까](#0-어느-노드를-쓸까)
- [A. `camera_node` — raw `Image` + `CameraInfo` + 상태 토픽](#a-camera_node--raw-image--camerainfo--상태-토픽)
  - [A.1 개요](#a1-개요)
  - [A.2 사전 요구사항](#a2-사전-요구사항)
  - [A.3 Quick Start](#a3-quick-start)
  - [A.4 파라미터](#a4-파라미터)
  - [A.5 `camera_id` 입력 형태](#a5-camera_id-입력-형태)
  - [A.6 토픽 연결](#a6-토픽-연결)
  - [A.7 이미지 발행 모드](#a7-이미지-발행-모드)
  - [A.8 동작 흐름](#a8-동작-흐름)
  - [A.9 에러 처리](#a9-에러-처리)
  - [A.10 실전 예제](#a10-실전-예제)
  - [A.11 트러블슈팅](#a11-트러블슈팅)
- [B. `image_capture_node` — JPEG `CompressedImage` 전용](#b-image_capture_node--jpeg-compressedimage-전용)
  - [B.1 개요](#b1-개요)
  - [B.2 Quick Start](#b2-quick-start)
  - [B.3 파라미터](#b3-파라미터)
  - [B.4 토픽](#b4-토픽)
  - [B.5 동작 흐름](#b5-동작-흐름)
  - [B.6 재연결 동작](#b6-재연결-동작)
  - [B.7 에러 처리](#b7-에러-처리)
  - [B.8 트러블슈팅](#b8-트러블슈팅)
- [관련 문서](#관련-문서)

---

## 0. 어느 노드를 쓸까

카메라 노드가 **셋**이다. 이름이 비슷해 고르기 어려우므로 여기서 먼저 가른다.

| | `camera_node` (§A) | `image_capture_node` (§B) | [`rdfp_camera_node`](../collect/session_camera_nodes_guide.md) |
|---|---|---|---|
| 패키지 | `robot_control` | `robot_control` | **`rdfp`** |
| 세션에 맞춰 켜고 끄는가 | ❌ 항상 발행 | ❌ 항상 발행 | ✅ `IN_SESSION` 에 열고 `IN_EPISODE` 에 발행 |
| 내보내는 것 | raw `Image` (+ 선택적 JPEG) | **JPEG `CompressedImage` 전용** | raw `Image` |
| 끊기면 누가 복구하나 | **supervisor** (`exit 1` → `respawn`) | 자체 (`ReconnectingCamera`) | 자체 (`reconnect_interval_sec`) |

**고르는 법은 세 질문이다.**

1. **세션에 맞춰 켜고 꺼야 하는가** → `rdfp_camera_node`. 수집 계층 전용이고,
   에피소드 밖에서는 이미지가 나가지 않으므로 rosbag 이 깨끗하다.
2. **JPEG 만 필요한가** (대역폭이 문제이거나 저장이 목적) → `image_capture_node`.
3. 그 밖 — 제어 스택의 일반 카메라 입력, 뷰어 확인, 단독 시험 → `camera_node`.

> **`ros2 run` 의 패키지를 틀리기 쉽다.** `rdfp_camera_node` 만 `rdfp` 소속이고
> 나머지 둘은 `robot_control` 이다. 반대로 부르면 실행 파일을 못 찾는다.

시뮬레이터가 압축(JPEG)으로 발행할 때 뷰어·레코더를 어떻게 잇는지는
[compressed_image_pipeline_design.md](compressed_image_pipeline_design.md) 가 갖는다.

## A. `camera_node` — raw `Image` + `CameraInfo` + 상태 토픽

OpenCV `VideoCapture` 로 카메라·비디오 스트림·파일을 열어 `sensor_msgs/Image`
(또는 `CompressedImage`) 와 `CameraInfo`, 상태 토픽을 함께 발행하는 ROS2 노드
사용 가이드.

### A.1 개요

`CameraNode` 는 OpenCV `VideoCapture` 로 영상 소스를 열어, 지정한 FPS 에 맞춰
프레임을 읽고 ROS2 토픽으로 발행한다. USB / 내장 카메라, RTSP·HTTP 스트림,
비디오 파일까지 동일 인터페이스로 다룬다.

**세션도 재연결도 얹지 않은 것이 이 노드의 성격이다.** 노드가 살아 있는 동안 계속
발행하고, 끊기면 `exit 1` 로 끝내 supervisor 에 재기동을 맡긴다. 다른 성격이
필요하면 카메라 노드가 더 있다 — 선택 표는 [README](README.md) 에 있다.

**핵심 특징:**
- 정수 인덱스·비디오 파일·RTSP/HTTP URL 을 단일 파라미터(`camera_id`) 로 수용
- 요청 `fps` / `resolution` 이 카메라 미지원 시 가장 가까운 값으로 자동 조정,
  **실제 값** 을 `CameraInfo` 및 타이머 주기에 반영
- 연결 끊김 감지 시 `status` 토픽에 `DISCONNECTED` → `ERROR` 후 프로세스
  종료(`exit 1`) — 외부 supervisor(launch `Restart`, systemd) 재기동에 위임
- `sensor_msgs/Image` 또는 `sensor_msgs/CompressedImage`(JPEG) 발행 선택 가능
- `status` 토픽은 `TRANSIENT_LOCAL` 이라 late-join 구독자도 마지막 상태 즉시 수신

---

### A.2 사전 요구사항

```bash
# cv_bridge + OpenCV
sudo apt install ros-humble-cv-bridge python3-opencv

# (선택) compress_image=true 사용 시
sudo apt install libturbojpeg
pip install PyTurboJPEG

# 빌드
colcon build --packages-select robot_control
source install/setup.bash
```

`camera_id` 가 정수 디바이스 인덱스인 경우 `/dev/video*` 에 접근 가능한 사용자
권한이 필요하다 (일반적으로 `video` 그룹).

---

### A.3 Quick Start

```bash
# USB 웹캠(/dev/video0) 을 30fps 640x480 로 기본 토픽(/camera_node/image_raw) 에 발행
ros2 run robot_control camera_node --ros-args \
  -p camera_id:=0 \
  -p fps:=30 \
  -p resolution:=640x480

# 다른 터미널에서 확인 (노드 이름이 camera_node 이므로 private 토픽은 /camera_node/* 로 resolve)
ros2 topic hz /camera_node/image_raw              # 실제 발행 rate
ros2 topic echo /camera_node/camera_status        # CONNECTED / DISCONNECTED 상태
ros2 run rqt_image_view rqt_image_view /camera_node/image_raw

# 외부 토픽명으로 remap 하려면 (예: /camera/image_raw)
ros2 run robot_control camera_node --ros-args \
  -p camera_id:=0 \
  -r ~/image_raw:=/camera/image_raw
```

---

### A.4 파라미터

#### 필수 파라미터

없음. 토픽 이름은 파라미터가 아니라 ROS2 **remap** 으로 지정한다.

#### 선택 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `camera_id` | `int` \| `string` | `'0'` | 디바이스 인덱스 또는 영상 소스 문자열. 아래 §`camera_id` 입력 형태 참조 |
| `fps` | `float` | (없음) | 목표 FPS. 미지정 시 카메라 기본값. 미지원 시 가장 가까운 값으로 조정 |
| `resolution` | `string` | (없음) | `"WIDTHxHEIGHT"` 형식. 미지정 시 카메라 기본 해상도 |
| `frame_id` | `string` | `"camera_link"` | `Image` / `CameraInfo` 의 `header.frame_id` 값 |
| `encoding` | `string` | `"bgr8"` | 비압축 모드의 `sensor_msgs/Image.encoding` (`bgr8`/`rgb8`/`mono8`) |
| `compress_image` | `bool` | `false` | `true` 이면 JPEG 압축 `CompressedImage` 로 발행 (`PyTurboJPEG` 필요) |
| `use_sim_time` | `bool` | `false` | ROS2 표준 sim time 플래그 |

> 💡 요청한 `fps` / `resolution` 과 실제 설정값이 다르면 WARNING 로그가
> 출력되며 **실제 값** 이 반영된다. 다른 노드(예: 레코더)에서 동일한 값을
> 쓸 때는 `ros2 topic hz` 로 실제 값을 먼저 확인한다.

---

### A.5 `camera_id` 입력 형태

정수 인덱스 또는 문자열을 받으며, 형태에 따라 OpenCV 백엔드가 자동 결정된다.

| 형태 | 예 | 비고 |
|---|---|---|
| 정수 인덱스 | `-p camera_id:=0` | `/dev/video0`. `'0'` 같은 숫자 문자열도 자동 int 변환 |
| 비디오 파일 | `-p camera_id:=/home/user/samples/sample.mp4` | 파일 끝에서 **처음으로 되감아 계속 재생** |
| RTSP/HTTP URL | `-p camera_id:=rtsp://user:pass@192.168.1.10:554/stream` | 자격증명은 로그에 `user:***@host` 로 마스킹 |

---

### A.6 토픽 연결

#### 발행 토픽

토픽은 모두 **private namespace(`~/`)** 로 선언되어 있어 노드 이름이
자동으로 prepend 된다. 기본 노드 이름 `camera_node` 기준 resolve 결과는
아래 "기본 경로" 열에 표기한다.

| 토픽(선언) | 기본 경로 | 타입 | QoS | 용도 |
|---|---|---|---|---|
| `~/image_raw` | `/camera_node/image_raw` | `sensor_msgs/Image` | sensor (BEST_EFFORT, VOLATILE, depth=1) | 비압축 프레임 이미지 |
| `~/image_compressed` | `/camera_node/image_compressed` | `sensor_msgs/CompressedImage` | sensor | `compress_image:=true` 시 JPEG 프레임 |
| `~/camera_info` | `/camera_node/camera_info` | `sensor_msgs/CameraInfo` | sensor | 해상도·기본 K/P 매트릭스 |
| `~/camera_status` | `/camera_node/camera_status` | `std_msgs/String` | reliable (RELIABLE, TRANSIENT_LOCAL, depth=1) | `CONNECTED` / `DISCONNECTED` / `ERROR` |

#### CameraInfo 내용

본 노드는 **캘리브레이션을 수행하지 않는다.** 기본 CameraInfo 는 다음과 같이
채워진다:

- `width`, `height`: 실제 카메라 해상도
- `distortion_model = "plumb_bob"`, `d = [0,0,0,0,0]` (왜곡 없음)
- `k`: `fx = fy = max(W, H)`, `cx = W/2`, `cy = H/2`
- `r`: 단위 행렬, `p`: `k` 확장형

정확한 내/외부 파라미터가 필요하면 `camera_calibration` 으로 캘리브레이션한
뒤 별도 `camera_info_manager` 기반 퍼블리셔를 사용하거나 본 노드를 포크해
확장한다.

#### `status` 토픽 활용

```bash
ros2 topic echo /camera_node/camera_status
```

| 값 | 발행 시점 |
|---|---|
| `CONNECTED` | 카메라를 성공적으로 열어 타이머를 시작했을 때 |
| `DISCONNECTED` | 프레임 읽기 실패로 연결 상실 판정 시 |
| `ERROR` | 복구 불가 에러로 노드가 종료되기 직전 |

`TRANSIENT_LOCAL` durability 이므로 구독자가 **나중에 붙어도 마지막 상태**
를 즉시 받는다.

---

### A.7 이미지 발행 모드

#### Raw (기본)

```bash
ros2 run robot_control camera_node --ros-args \
  -p encoding:=bgr8
# → /camera_node/image_raw 에 발행 (remap 필요 시 -r ~/image_raw:=<new>)
```

- `sensor_msgs/Image` 로 발행
- `encoding` 값이 `cv_bridge.cv2_to_imgmsg()` 에 그대로 전달됨
- 원본 데이터량이 크므로(1080p@30fps ≈ 180 MB/s) 같은 호스트 내 소비 권장

#### JPEG 압축

```bash
ros2 run robot_control camera_node --ros-args \
  -p compress_image:=true
# → /camera_node/image_compressed 에 발행 (remap 필요 시 -r ~/image_compressed:=<new>)
```

- `sensor_msgs/CompressedImage` 로 발행 (`format='jpeg'`)
- 네트워크 배포 / 녹화 대상에 적합
- **전제**: `libturbojpeg` + `PyTurboJPEG`
- JPEG 압축은 항상 `BGR` 픽셀 포맷을 가정 (OpenCV 기본과 일치)

---

### A.8 동작 흐름

```mermaid
sequenceDiagram
    autonumber
    participant Cam as OpenCvCamera
    participant Node as CameraNode
    participant Topic as ROS2 Topics

    Note over Node: 파라미터 검증<br/>(frame_id / encoding 필수)
    Node->>Cam: open()
    Cam-->>Node: 실제 fps / 해상도
    Node->>Topic: status = "CONNECTED"<br/>/camera_node/camera_status
    Note over Node: 타이머 시작 (1/fps 주기)

    loop 매 tick
        Node->>Cam: read()
        alt 프레임 획득
            Cam-->>Node: frame
            Node->>Topic: Image — cv_bridge → imgmsg<br/>/camera_node/image_raw
            Node->>Topic: CameraInfo<br/>/camera_node/camera_info
        else read() 실패 · is_opened = true
            Cam-->>Node: None
            Node->>Node: WARNING 만 남기고 다음 tick
        else read() 실패 · is_opened = false
            Cam-->>Node: None
            Node->>Topic: status = "DISCONNECTED"
            Node->>Topic: status = "ERROR"
            Node->>Node: sys.exit(1) — supervisor 재시작에 위임
        end
    end

    Note over Node: 사용자 Ctrl-C
    Node->>Cam: release() — _cleanup()
```

- **기동 순서**: 파라미터 검증 → 카메라 open → 실제 fps 조회 → 퍼블리셔
  3종 생성 → `CONNECTED` 발행 → 타이머 시작
- **프레임 드롭 허용**: `is_opened == True` 이면 단일 빈 프레임은 WARNING
  만 남기고 다음 tick 으로 넘어감
- **자동 재연결 없음**: `DISCONNECTED` 판정 시 `sys.exit(1)` — 재시작은
  외부 supervisor 담당

---

### A.9 에러 처리

| 상황 | 동작 |
|---|---|
| `frame_id` / `encoding` 이 빈 문자열 | `RuntimeError` → 기동 실패 |
| `OpenCvCamera.open()` 실패 | `RuntimeError` → 기동 실패 + 진단 로그 |
| `compress_image=true` 인데 `PyTurboJPEG` 미설치 | `RuntimeError` → 기동 실패 |
| 요청 fps/해상도 ≠ 실제값 | WARNING + **실제값** 사용 |
| 단일 빈 프레임(`is_opened=True`) | WARNING + 다음 tick 진행 |
| 프레임 읽기 실패(`is_opened=False`) | ERROR + `status=DISCONNECTED` → `ERROR` → `sys.exit(1)` |
| 파일 소스 EOF | INFO + **되감아 계속** (끊김이 아니다) |
| SIGINT/SIGTERM | `_cleanup()` → 카메라 release + 타이머 취소 |

---

### A.10 실전 예제

#### 기본 USB 카메라

```bash
ros2 run robot_control camera_node --ros-args \
  -p camera_id:=0 \
  -p fps:=30 \
  -p resolution:=640x480 \
  -p frame_id:=camera_link
# 기본 발행 경로: /camera_node/image_raw
```

#### RTSP 스트림 + JPEG 압축

```bash
ros2 run robot_control camera_node --ros-args \
  -p camera_id:='rtsp://user:pass@192.168.1.10:554/stream' \
  -p compress_image:=true \
  -p frame_id:=ipcam_link \
  -r ~/image_compressed:=/ipcam/image_compressed
```

#### 비디오 파일 재생 (일회성)

```bash
ros2 run robot_control camera_node --ros-args \
  -p camera_id:=/home/$USER/samples/sample.mp4 \
  -p fps:=30 \
  -p frame_id:=video_link \
  -r ~/image_raw:=/video/image
```

**파일 끝은 연결 끊김이 아니다 — 처음으로 되감아 계속 재생한다.** OpenCV 는 EOF 에서도
`isOpened()` 를 참으로 두므로, 되감지 않으면 노드가 살아 있는 채 아무것도 못 내고 초당
fps 회의 경고만 남긴다. 증상이 "카메라가 조용하다"뿐이라 원인이 안 보여서 되감기를
넣었다 (`OpenCvCamera._rewind_if_file_ended`). 한 프레임도 못 읽은 소스는 되감지 않으며
(깨진 파일에서 무한 반복이 된다), 장치·RTSP 는 프레임 수가 0 이라 대상이 아니다.

#### 명시적 `camera_info` 토픽 지정

```bash
ros2 run robot_control camera_node --ros-args \
  -r ~/camera_info:=/calibration/camera_info
```

#### Launch 파일

재사용 가능한 헬퍼가 준비되어 있다.

```python
# my_launch.py
from launch import LaunchDescription
from robot_control.launch_helpers.camera import declare_camera_arguments, create_camera_node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        *declare_camera_arguments(),
        create_camera_node(),
    ])
```

직접 `Node` 로 구성하는 경우:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='camera_node',
            parameters=[{
                'camera_id': 0,
                'fps': 30.0,
                'resolution': '1280x720',
                'frame_id': 'camera_link',
                'compress_image': False,
            }],
            remappings=[
                ('~/image_raw', '/camera/image_raw'),
                ('~/camera_info', '/camera/camera_info'),
                ('~/camera_status', '/camera/camera_status'),
            ],
            # 연결 끊김 → sys.exit(1) 이후 자동 재시작이 필요하면:
            # respawn=True, respawn_delay=2.0,
        ),
    ])
```

---

### A.11 트러블슈팅

#### A.1 `Failed to open camera`

원인이 다양하므로 기동 로그의 후속 INFO 진단 줄을 참고한다.

- **정수 index**: 장치 미연결, 다른 프로세스 점유, 번호 오류

  ```bash
  v4l2-ctl --list-devices
  fuser /dev/video0
  ```

- **URI 스트림**: 네트워크·자격증명·스트림 가용성 점검

  ```bash
  ffprobe rtsp://...    # ffmpeg 으로 먼저 연결 테스트
  ```

- **파일 경로**: 존재 여부 / 읽기 권한 / OpenCV 의 해당 코덱 지원 여부

#### A.2 `compress_image=true requires PyTurboJPEG`

```bash
sudo apt install libturbojpeg
pip install PyTurboJPEG
```

두 가지 모두 필요하다.

#### A.3 `Requested resolution ... differs from actual resolution`

카메라가 요청 해상도를 미지원한다. 지원 해상도를 확인하고 맞추거나 실제 값을
그대로 사용한다 (CameraInfo 는 실제 값으로 발행됨).

```bash
v4l2-ctl --list-formats-ext -d /dev/video0
```

#### A.4 `Requested FPS ... differs from actual FPS`

동일 원인. 로그에 표시된 실제 FPS 를 다른 노드(레코더 등) 파라미터의 기준으로
삼는다.

#### A.5 `frame_id cannot be empty`

`frame_id` 가 빈 문자열이면 기동 실패한다.

```bash
-p frame_id:=camera_link
```

#### A.6 `DISCONNECTED` → `ERROR` 로 끝나며 노드가 종료

**의도된 동작.** 자동 재연결 로직이 없으므로 외부 supervisor 재시작에 맡긴다.
Launch 의 `respawn=True` 또는 systemd `Restart=on-failure` 유닛으로 복구한다.

#### A.7 CameraInfo 의 K 매트릭스가 부정확

기본 CameraInfo 는 캘리브레이션이 아닌 **더미값** 이다. 정확한 내부 파라미터가
필요하면 `ros2 run camera_calibration cameracalibrator` 로 캘리브레이션한 뒤
`camera_info_manager` 기반 퍼블리셔로 대체하거나 본 노드를 확장한다.

#### A.8 RTSP URL 의 비밀번호가 로그에 노출되지 않았는가

로그에는 `user:***@host` 형태로 마스킹되어 출력된다. 원문 URL 이 보인다면
버그이므로 재현 로그와 함께 리포트한다.

---

## B. `image_capture_node` — JPEG `CompressedImage` 전용

USB 카메라에서 프레임을 캡처해 **JPEG 압축 이미지(`sensor_msgs/CompressedImage`)** 로
발행하는 노드 사용 가이드.

### B.1 개요

`ImageCaptureNode` 는 `ReconnectingCamera` 로 카메라를 열고, 타이머 주기마다
프레임을 읽어 `cv2.imencode('.jpg', ...)` 로 압축한 뒤 `CompressedImage` 로
발행한다. 소속 패키지는 **`robot_control`**
(`src/robot_control/robot_control/camera/image_capture.py`) 이며, 세션 상태를
구독하지 않는 순수 제어 계층 노드다.

**`CameraNode` 와의 차이:**

| 항목 | `CameraNode` | `ImageCaptureNode` |
|---|---|---|
| 발행 타입 | `Image` (+ `compress_image` 시 `CompressedImage`) + `CameraInfo` + status | `CompressedImage` 단일 |
| 카메라 래퍼 | `OpenCvCamera` (재연결 없음) | `ReconnectingCamera` (백그라운드 재연결 스레드) |
| QoS | `qos_profile_sensor_data` | BEST_EFFORT / KEEP_LAST(10) |
| 토픽명 | `~/image_raw` (private, remap 전제) | `topic` 파라미터로 지정하는 **절대 경로** |
| 용도 | 표준 카메라 파이프라인 | 대역폭이 제한된 경로로 JPEG 만 밀어 넣는 경우 |

---

### B.2 Quick Start

```bash
colcon build --packages-select robot_control
source install/setup.bash

# 기본값(camera_id=0, 5fps, /stor/real/image/jpeg)으로 실행
ros2 run robot_control image_capture_node

# 파라미터 지정
ros2 run robot_control image_capture_node --ros-args \
    -p camera_id:=0 \
    -p resolution:=1280x720 \
    -p fps:=10 \
    -p topic:=/camera/image/compressed \
    -p jpeg_quality:=90

# 발행 확인
ros2 topic hz /stor/real/image/jpeg
```

---

### B.3 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `camera_id` | int \| str | `0` | 카메라 디바이스 ID 또는 영상 소스 경로 |
| `resolution` | str | None | `"WIDTHxHEIGHT"` 형식. None 이면 카메라 기본값 |
| `fps` | float | `5` | 캡처·발행 주기. 타이머 주기는 `1/fps` |
| `topic` | str | `/stor/real/image/jpeg` | 발행 토픽 이름 (절대 경로) |
| `frame_id` | str | `camera` | 메시지의 `header.frame_id` |
| `jpeg_quality` | int | `80` | JPEG 품질 0~100. 범위를 벗어나면 `ValueError` 로 기동 실패 |

`resolution` 을 지정하면 캡처한 프레임 크기가 다를 때 `cv2.resize` 로 맞춘다.

---

### B.4 토픽

| 토픽 | 타입 | QoS | 방향 |
|---|---|---|---|
| `topic` 파라미터 값 (기본 `/stor/real/image/jpeg`) | `sensor_msgs/CompressedImage` (`format='jpeg'`) | BEST_EFFORT / KEEP_LAST / depth 10 | 발행 |

구독 토픽·서비스는 없다.

---

### B.5 동작 흐름

```mermaid
sequenceDiagram
    autonumber
    participant RC as ReconnectingCamera
    participant Node as ImageCaptureNode
    participant Topic as CompressedImage topic

    Node->>RC: ReconnectingCamera(camera_id, resolution, fps,<br/>reconnect_interval=5.0, max_attempts=0)
    alt 초기 연결 성공
        RC-->>Node: CONNECTED
    else 초기 연결 실패
        RC-->>Node: CONNECTING — 백그라운드 재연결 스레드 시작
    end
    Note over Node: 타이머 시작 (1/fps 주기)

    loop 매 tick
        Node->>RC: is_available?
        alt CONNECTED
            Node->>RC: read(timeout_sec=0)
            RC-->>Node: frame
            Node->>Node: resize (resolution 지정 + 크기 불일치 시)
            Node->>Node: cv2.imencode('.jpg', frame, jpeg_quality)
            Node->>Topic: CompressedImage (format='jpeg')
        else CONNECTING
            Node->>Node: 즉시 반환 — 발행 없음
        else CLOSED
            RC-->>Node: ConnectionError
            Node->>Node: 타이머 취소 → SystemExit(1)
        end
    end

    Note over Node: 사용자 Ctrl-C
    Node->>RC: close() — destroy_node()
```

- **발행은 `is_available` 이 `True` 일 때만** 일어난다. `CONNECTING` 구간에서는
  타이머가 계속 돌지만 아무것도 발행하지 않는다.
- **`read(timeout_sec=0)`** 이므로 연결 대기로 타이머 콜백이 막히지 않는다.
- **JPEG 인코딩은 콜백 스레드에서 동기 수행**한다. 고해상도 + 높은 fps 조합에서는
  인코딩 시간이 타이머 주기를 넘길 수 있다.

---

### B.6 재연결 동작

재연결은 `ReconnectingCamera` 가 담당하며, 노드는 상수로 고정된 값을 넘긴다
(`reconnect_interval=5.0`, `max_attempts=0`). **파라미터로 노출돼 있지 않다.**

```mermaid
stateDiagram-v2
    [*] --> CONNECTED: 기동 시 open() 성공
    [*] --> CONNECTING: 기동 시 open() 실패
    CONNECTING --> CONNECTING: 5초 대기 후 재시도 (max_attempts=0 이면 무한)
    CONNECTING --> CONNECTED: 재연결 성공
    CONNECTED --> CLOSED: 사용 중 끊김 (max_attempts=0 이면 즉시)
    CLOSED --> [*]: read() 가 ConnectionError → 노드 SystemExit(1)
```

`max_attempts=0` 의 의미가 경로에 따라 다르다는 점에 주의한다:

| 상황 | 동작 |
|---|---|
| 기동 시 카메라가 없음 | `CONNECTING` 상태로 5초 주기 **무한 재시도**. 노드는 살아 있고 발행만 하지 않는다 |
| 연결된 뒤 카메라가 빠짐 | **즉시 `CLOSED`** 로 전이. 다음 `read()` 가 `ConnectionError` → 타이머 취소 후 `SystemExit(1)` 로 노드 종료 |

즉 "처음부터 없으면 기다리고, 쓰다가 빠지면 죽는다". 두 경로가 비대칭이므로
장시간 무인 운용에는 supervisor(런치의 `respawn=True` 등)를 함께 두는 것이 좋다.

---

### B.7 에러 처리

| 상황 | 동작 |
|---|---|
| `jpeg_quality` 범위 초과 | ERROR 로그 + `ValueError` 로 기동 실패 |
| `read()` 가 None (일시적 실패) | ERROR 로그 (5초 throttle) 후 다음 주기 대기 |
| 카메라 CLOSED (`ConnectionError`) | 타이머 취소 + ERROR 로그 + `SystemExit(1)` |
| JPEG 인코딩 실패 | ERROR 로그 (5초 throttle) 후 다음 주기 대기 |
| 기타 예외 | ERROR 로그 후 계속 동작 |
| SIGINT / SIGTERM | `destroy_node()` → `camera.close()` → `rclpy.try_shutdown()` (traceback 없이 종료) |

---

### B.8 트러블슈팅

**1. 노드는 떠 있는데 토픽이 비어 있다**

```bash
# 로그에 'Initial connection failed. Starting reconnect loop.' 가 있는지 확인
ls -l /dev/video*      # 디바이스 존재 여부
```
카메라를 못 연 채 재연결 루프를 도는 중이다. `camera_id` 를 확인한다.

**2. 구독자가 메시지를 못 받는다**

발행 QoS 가 **BEST_EFFORT** 다. RELIABLE 구독자와는 매칭되지 않는다.

```bash
ros2 topic echo --qos-reliability best_effort /stor/real/image/jpeg --field format
```

**3. 사용 중 카메라를 뽑았더니 노드가 죽었다**

의도된 동작이다 — 위 [재연결 동작](#b6-재연결-동작) 참고.

---

## 관련 문서

- [README.md](README.md) — 제어 계층 문서 진입점 (어느 카메라 노드를 쓸지 고르는 표)
- [../collect/session_camera_nodes_guide.md](../collect/session_camera_nodes_guide.md) — 세션 인지 카메라·뷰어 (수집 계층)
- [카메라 서브시스템 README](README.md) — 어느 카메라 노드를 쓸지 고르는 표
- [ImageViewerNode Guide](image_viewer_node_guide.md) — 발행된 이미지 토픽 미리보기
- [OpenCvCamera Guide](opencv_camera_guide.md) — 내부 카메라 래퍼
- [launch_helpers/camera.py](../../src/robot_control/robot_control/launch_helpers/camera.py) — 재사용 가능한 launch 헬퍼
- [CameraNode Guide](camera_nodes_guide.md) — 표준 카메라 노드 (`Image` + `CameraInfo` + status)
- [OpenCvCamera Guide](opencv_camera_guide.md) — `ReconnectingCamera` 가 감싸는 하위 래퍼
- [ImageViewerNode Guide](image_viewer_node_guide.md) — 이미지 토픽 뷰어
