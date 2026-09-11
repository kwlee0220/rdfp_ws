# CameraNode — Programmer's Guide

OpenCV `VideoCapture` 로 카메라·비디오 스트림·파일을 열어 `sensor_msgs/Image`
(또는 `CompressedImage`) 와 `CameraInfo`, 상태 토픽을 함께 발행하는 ROS2 노드
사용 가이드.

---

## 목차

1. [개요](#1-개요)
2. [사전 요구사항](#2-사전-요구사항)
3. [Quick Start](#3-quick-start)
4. [파라미터](#4-파라미터)
5. [`camera_id` 입력 형태](#5-camera_id-입력-형태)
6. [토픽 연결](#6-토픽-연결)
7. [이미지 발행 모드](#7-이미지-발행-모드)
8. [동작 흐름](#8-동작-흐름)
9. [에러 처리](#9-에러-처리)
10. [실전 예제](#10-실전-예제)
11. [트러블슈팅](#11-트러블슈팅)
12. [관련 문서](#12-관련-문서)

---

## 1. 개요

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

## 2. 사전 요구사항

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

## 3. Quick Start

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

## 4. 파라미터

### 필수 파라미터

없음. 토픽 이름은 파라미터가 아니라 ROS2 **remap** 으로 지정한다.

### 선택 파라미터

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

## 5. `camera_id` 입력 형태

정수 인덱스 또는 문자열을 받으며, 형태에 따라 OpenCV 백엔드가 자동 결정된다.

| 형태 | 예 | 비고 |
|---|---|---|
| 정수 인덱스 | `-p camera_id:=0` | `/dev/video0`. `'0'` 같은 숫자 문자열도 자동 int 변환 |
| 비디오 파일 | `-p camera_id:=/home/user/samples/sample.mp4` | 파일 끝에서 **처음으로 되감아 계속 재생** |
| RTSP/HTTP URL | `-p camera_id:=rtsp://user:pass@192.168.1.10:554/stream` | 자격증명은 로그에 `user:***@host` 로 마스킹 |

---

## 6. 토픽 연결

### 발행 토픽

토픽은 모두 **private namespace(`~/`)** 로 선언되어 있어 노드 이름이
자동으로 prepend 된다. 기본 노드 이름 `camera_node` 기준 resolve 결과는
아래 "기본 경로" 열에 표기한다.

| 토픽(선언) | 기본 경로 | 타입 | QoS | 용도 |
|---|---|---|---|---|
| `~/image_raw` | `/camera_node/image_raw` | `sensor_msgs/Image` | sensor (BEST_EFFORT, VOLATILE, depth=1) | 비압축 프레임 이미지 |
| `~/image_compressed` | `/camera_node/image_compressed` | `sensor_msgs/CompressedImage` | sensor | `compress_image:=true` 시 JPEG 프레임 |
| `~/camera_info` | `/camera_node/camera_info` | `sensor_msgs/CameraInfo` | sensor | 해상도·기본 K/P 매트릭스 |
| `~/camera_status` | `/camera_node/camera_status` | `std_msgs/String` | reliable (RELIABLE, TRANSIENT_LOCAL, depth=1) | `CONNECTED` / `DISCONNECTED` / `ERROR` |

### CameraInfo 내용

본 노드는 **캘리브레이션을 수행하지 않는다.** 기본 CameraInfo 는 다음과 같이
채워진다:

- `width`, `height`: 실제 카메라 해상도
- `distortion_model = "plumb_bob"`, `d = [0,0,0,0,0]` (왜곡 없음)
- `k`: `fx = fy = max(W, H)`, `cx = W/2`, `cy = H/2`
- `r`: 단위 행렬, `p`: `k` 확장형

정확한 내/외부 파라미터가 필요하면 `camera_calibration` 으로 캘리브레이션한
뒤 별도 `camera_info_manager` 기반 퍼블리셔를 사용하거나 본 노드를 포크해
확장한다.

### `status` 토픽 활용

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

## 7. 이미지 발행 모드

### Raw (기본)

```bash
ros2 run robot_control camera_node --ros-args \
  -p encoding:=bgr8
# → /camera_node/image_raw 에 발행 (remap 필요 시 -r ~/image_raw:=<new>)
```

- `sensor_msgs/Image` 로 발행
- `encoding` 값이 `cv_bridge.cv2_to_imgmsg()` 에 그대로 전달됨
- 원본 데이터량이 크므로(1080p@30fps ≈ 180 MB/s) 같은 호스트 내 소비 권장

### JPEG 압축

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

## 8. 동작 흐름

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

## 9. 에러 처리

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

## 10. 실전 예제

### 기본 USB 카메라

```bash
ros2 run robot_control camera_node --ros-args \
  -p camera_id:=0 \
  -p fps:=30 \
  -p resolution:=640x480 \
  -p frame_id:=camera_link
# 기본 발행 경로: /camera_node/image_raw
```

### RTSP 스트림 + JPEG 압축

```bash
ros2 run robot_control camera_node --ros-args \
  -p camera_id:='rtsp://user:pass@192.168.1.10:554/stream' \
  -p compress_image:=true \
  -p frame_id:=ipcam_link \
  -r ~/image_compressed:=/ipcam/image_compressed
```

### 비디오 파일 재생 (일회성)

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

### 명시적 `camera_info` 토픽 지정

```bash
ros2 run robot_control camera_node --ros-args \
  -r ~/camera_info:=/calibration/camera_info
```

### Launch 파일

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

## 11. 트러블슈팅

### 1. `Failed to open camera`

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

### 2. `compress_image=true requires PyTurboJPEG`

```bash
sudo apt install libturbojpeg
pip install PyTurboJPEG
```

두 가지 모두 필요하다.

### 3. `Requested resolution ... differs from actual resolution`

카메라가 요청 해상도를 미지원한다. 지원 해상도를 확인하고 맞추거나 실제 값을
그대로 사용한다 (CameraInfo 는 실제 값으로 발행됨).

```bash
v4l2-ctl --list-formats-ext -d /dev/video0
```

### 4. `Requested FPS ... differs from actual FPS`

동일 원인. 로그에 표시된 실제 FPS 를 다른 노드(레코더 등) 파라미터의 기준으로
삼는다.

### 5. `frame_id cannot be empty`

`frame_id` 가 빈 문자열이면 기동 실패한다.

```bash
-p frame_id:=camera_link
```

### 6. `DISCONNECTED` → `ERROR` 로 끝나며 노드가 종료

**의도된 동작.** 자동 재연결 로직이 없으므로 외부 supervisor 재시작에 맡긴다.
Launch 의 `respawn=True` 또는 systemd `Restart=on-failure` 유닛으로 복구한다.

### 7. CameraInfo 의 K 매트릭스가 부정확

기본 CameraInfo 는 캘리브레이션이 아닌 **더미값** 이다. 정확한 내부 파라미터가
필요하면 `ros2 run camera_calibration cameracalibrator` 로 캘리브레이션한 뒤
`camera_info_manager` 기반 퍼블리셔로 대체하거나 본 노드를 확장한다.

### 8. RTSP URL 의 비밀번호가 로그에 노출되지 않았는가

로그에는 `user:***@host` 형태로 마스킹되어 출력된다. 원문 URL 이 보인다면
버그이므로 재현 로그와 함께 리포트한다.

---

## 12. 관련 문서

- [카메라 서브시스템 README](./README.md) — 어느 카메라 노드를 쓸지 고르는 표
- [ImageViewerNode Guide](./image_viewer_node_guide.md) — 발행된 이미지 토픽 미리보기
- [OpenCvCamera Guide](./opencv_camera_guide.md) — 내부 카메라 래퍼
- [launch_helpers/camera.py](../../src/robot_control/robot_control/launch_helpers/camera.py) — 재사용 가능한 launch 헬퍼
