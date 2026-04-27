# OpenCvCamera Programmer's Guide

OpenCV VideoCapture를 사용한 카메라 제어를 위한 안전하고 사용하기 쉬운 Python 클래스입니다.

## 개요

`OpenCvCamera` 클래스는 다양한 카메라 소스(웹캠, 비디오 파일, 네트워크 스트림)를 통합된 인터페이스로 제어할 수 있게 해주는 래퍼 클래스입니다.

### 주요 특징

- **다양한 소스 지원**: 웹캠, 파일, RTSP/HTTP 스트림
- **안전한 리소스 관리**: Context manager 및 자동 해제
- **상세한 진단**: 실패 시 구체적인 원인 분석 (예외 메시지에 포함)
- **유연한 설정**: 해상도, FPS 자동 조정 지원
- **풍부한 로깅**: 프레임 읽기 실패 시 주기적 warning, 설정 불일치 감지

## 설치 및 의존성

### 필수 의존성

```bash
# OpenCV Python 패키지
pip install opencv-python

# NumPy (일반적으로 OpenCV와 함께 설치됨)
pip install numpy
```

### Import

```python
from rdfp.camera.opencv_camera import OpenCvCamera

# 선택: 해상도/FPS 타입을 명시적으로 사용할 때
from rdfp.types import Resolution, Fps
```

### 생성자 시그니처

```python
OpenCvCamera(
    camera_id: int | str,
    *,
    resolution: str | tuple[int, int] | Resolution | None = None,
    fps: float | Fps | None = None,
) -> None
```

- `camera_id`만 위치 인자이며, `resolution`/`fps`는 **keyword-only** 인자입니다.
- `resolution`은 다음 세 가지 형식을 모두 허용합니다:
  - `Resolution` NamedTuple (`from rdfp.types import Resolution`)
  - `(width, height)` 튜플 — 예: `(1280, 720)`
  - `"WIDTHxHEIGHT"` 형식의 문자열 — 예: `"1280x720"`
- `fps`는 0보다 큰 임의의 실수값이면 유효하며, 내부적으로 `Fps`(float 서브클래스)로 래핑됩니다.
- `resolution`/`fps`를 생략하면 카메라의 기본 설정을 그대로 사용합니다.

```python
from rdfp.camera.opencv_camera import OpenCvCamera
from rdfp.types import Resolution, Fps

OpenCvCamera(0, resolution=(640, 480), fps=30.0)
OpenCvCamera(0, resolution="1920x1080", fps=29.97)
OpenCvCamera(0, resolution=Resolution(1280, 720), fps=Fps(60))
OpenCvCamera(0)  # 카메라 기본값 사용
```

> 숫자 문자열로 `camera_id`를 전달하면 내부적으로 정수로 정규화됩니다 (`"0"` → `0`).

## 지원되는 카메라 소스

### 디바이스 ID (정수)

```python
# 기본 카메라 (대부분 웹캠)
camera = OpenCvCamera(0, resolution=(640, 480), fps=30.0)

# 두 번째 카메라
camera = OpenCvCamera(1, resolution=(1920, 1080), fps=60.0)

# USB 카메라 (일반적으로 1, 2, 3...)
camera = OpenCvCamera(2, resolution=(1280, 720), fps=30.0)
```

### 파일 경로

```python
# 상대 경로
camera = OpenCvCamera('video.mp4', resolution=(1920, 1080), fps=25.0)

# 절대 경로
camera = OpenCvCamera('/path/to/video.avi', resolution=(640, 480), fps=30.0)

# 이미지 시퀀스
camera = OpenCvCamera('/path/to/images/img_%03d.jpg', resolution=(1920, 1080), fps=10.0)
```

### 디바이스 경로 (Linux/Unix)

```python
# Video4Linux 디바이스
camera = OpenCvCamera('/dev/video0', resolution=(640, 480), fps=30.0)
camera = OpenCvCamera('/dev/video1', resolution=(1920, 1080), fps=60.0)
```

### 지원되는 URI 스킴들

#### **네트워크 스트림**

```python
# RTSP 스트림 (IP 카메라)
camera = OpenCvCamera('rtsp://192.168.1.100:554/stream', resolution=(1920, 1080), fps=30.0)
camera = OpenCvCamera('rtsp://username:password@camera.local/live', resolution=(1280, 720), fps=25.0)

# HTTP MJPEG 스트림
camera = OpenCvCamera('http://camera.local:8080/mjpeg', resolution=(640, 480), fps=15.0)

# HTTPS 스트림
camera = OpenCvCamera('https://secure-camera.com/stream', resolution=(1920, 1080), fps=30.0)

# FTP 비디오 파일
camera = OpenCvCamera('ftp://server.com/video.mp4', resolution=(1280, 720), fps=24.0)
```

#### **로컬 프로토콜**

```python
# 파일 URI
camera = OpenCvCamera('file:///absolute/path/to/video.mp4', resolution=(1920, 1080), fps=30.0)

# Video4Linux2 (일부 시스템)
camera = OpenCvCamera('v4l2:///dev/video0', resolution=(640, 480), fps=30.0)
```

#### **특수 소스**

```python
# GStreamer Pipeline (시스템에 GStreamer가 설치된 경우)
pipeline = 'videotestsrc ! videoconvert ! appsink'
camera = OpenCvCamera(pipeline, resolution=(640, 480), fps=30.0)

# 더 복잡한 GStreamer Pipeline
pipeline = 'v4l2src device=/dev/video0 ! video/x-raw,width=1920,height=1080 ! videoconvert ! appsink'
camera = OpenCvCamera(pipeline, resolution=(1920, 1080), fps=30.0)
```

## 기본 사용법

### 0. 기본값 사용 (resolution/fps 생략)

`resolution`과 `fps`를 생략하면 카메라의 기본 설정을 그대로 사용합니다.
실제 사용된 해상도/FPS는 `open()` 반환값에서 확인할 수 있습니다.

```python
camera = OpenCvCamera(0)
(actual_width, actual_height), actual_fps = camera.open()
print(f"실제 설정: {actual_width}x{actual_height} @ {actual_fps}fps")
```

### 1. 기본 패턴

```python
from rdfp.camera.opencv_camera import OpenCvCamera

# 카메라 생성
camera = OpenCvCamera(0, resolution=(640, 480), fps=30.0)

try:
    # 카메라 열기 — 실패 시 RuntimeError 발생
    (actual_width, actual_height), actual_fps = camera.open()
    print(f"카메라 열기 성공: {actual_width}x{actual_height} @ {actual_fps}fps")

    # 프레임 읽기
    frame = camera.read()
    if frame is not None:
        print(f"프레임 크기: {frame.shape}")
        # 프레임 처리...
    else:
        print("프레임 읽기 실패")

except RuntimeError as e:
    print(f"카메라 열기 실패: {e}")

finally:
    # 리소스 해제
    camera.release()
```

### 2. Context Manager 패턴 (권장)

```python
# 자동 리소스 관리 - 실패 시 RuntimeError 발생
try:
    with OpenCvCamera(0, resolution=(1920, 1080), fps=30.0) as camera:
        # 카메라가 자동으로 열림
        frame = camera.read()
        if frame is not None:
            # 프레임 처리...
            pass
    # 자동으로 release() 호출됨
except RuntimeError as e:
    print(f"카메라 열기 실패: {e}")
```

> `open()`과 Context manager 모두 카메라 열기 실패 시 `RuntimeError`를 발생시킵니다.
> 예외 메시지에 실패 원인 진단 정보가 포함됩니다.

### 3. 프로퍼티 활용

| 프로퍼티 | 반환 타입 | 설명 |
|---------|----------|------|
| `camera_id` | `int \| str` | 생성자에 전달된 ID (숫자 문자열은 int로 정규화됨) |
| `resolution` | `Resolution \| None` | 요청한 해상도 (생략 시 `None`) |
| `fps` | `float \| None` | 요청한 FPS (생략 시 `None`) |
| `is_opened` | `bool` | 현재 카메라가 열려 있는지 여부 |

```python
camera = OpenCvCamera(0, resolution=(1920, 1080), fps=60.0)

# 요청한 설정 확인
print(f"요청 해상도: {camera.resolution}")  # Resolution(width=1920, height=1080)
print(f"요청 FPS: {camera.fps}")            # 60.0
print(f"카메라 ID: {camera.camera_id}")     # 0
print(f"열림 상태: {camera.is_opened}")     # False

actual_resolution, actual_fps = camera.open()
# 실제 설정된 값과 비교
if camera.resolution != actual_resolution:
    print(f"해상도 조정됨: {camera.resolution} → {actual_resolution}")
if camera.fps != actual_fps:
    print(f"FPS 조정됨: {camera.fps} → {actual_fps}")
```

> `resolution`은 `Resolution(width, height)` NamedTuple이므로 `camera.resolution.width`,
> `camera.resolution.height`로 개별 필드에 접근하거나 `(w, h) = camera.resolution` 형태로
> 언패킹할 수 있습니다.

## 고급 사용법

### 1. 여러 카메라 동시 사용

```python
cameras = []
try:
    # 여러 카메라 초기화
    for i in range(3):
        camera = OpenCvCamera(i, resolution=(640, 480), fps=30.0)
        try:
            camera.open()
            cameras.append(camera)
            print(f"카메라 {i} 열기 성공")
        except RuntimeError:
            print(f"카메라 {i} 열기 실패")
            camera.release()

    # 동시 프레임 읽기
    while True:
        frames = []
        for i, camera in enumerate(cameras):
            frame = camera.read()
            if frame is not None:
                frames.append(frame)
            else:
                print(f"카메라 {i} 프레임 읽기 실패")
                break
        
        if len(frames) == len(cameras):
            # 모든 프레임 처리...
            pass
        else:
            break

finally:
    # 모든 카메라 해제
    for camera in cameras:
        camera.release()
```

### 2. 설정 검증 및 폴백

```python
def setup_camera_with_fallback():
    """높은 품질부터 시도하여 동작하는 설정 찾기"""
    configs = [
        ((3840, 2160), 30),  # 4K
        ((1920, 1080), 60),  # Full HD 고프레임
        ((1920, 1080), 30),  # Full HD 표준
        ((1280, 720), 30),   # HD
        ((640, 480), 30),    # VGA (최후)
    ]
    
    for resolution, fps in configs:
        camera = OpenCvCamera(0, resolution=resolution, fps=fps)
        try:
            result = camera.open()
            print(f"성공한 설정: {result[0]} @ {result[1]}fps")
            return camera, result
        except RuntimeError:
            camera.release()
    
    return None, None

camera, result = setup_camera_with_fallback()
if camera:
    # 카메라 사용...
    camera.release()
```

### 3. 스트림 모니터링

```python
import time

def monitor_camera_stream(camera, duration_seconds=60):
    """카메라 스트림 안정성 모니터링"""
    start_time = time.time()
    frame_count = 0
    error_count = 0
    
    while time.time() - start_time < duration_seconds:
        frame = camera.read()
        if frame is not None:
            frame_count += 1
        else:
            error_count += 1
            # read() 내부에서 5초 간격으로 warning 로그가 자동 출력됨
            time.sleep(0.1)  # 오류 시 잠시 대기
        
        if frame_count % 100 == 0:  # 100프레임마다 상태 출력
            elapsed = time.time() - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0
            print(f"진행: {frame_count}프레임, {fps:.1f}fps, 오류: {error_count}")

    return frame_count, error_count

# 사용 예시
with OpenCvCamera('rtsp://camera.ip/stream', resolution=(1920, 1080), fps=30.0) as camera:
    frames, errors = monitor_camera_stream(camera, 60)
    print(f"완료: {frames}프레임 수신, {errors}오류 발생")
```

## 에러 처리

### 1. 생성자 예외

```python
try:
    # 음수 camera_id
    camera = OpenCvCamera(-1, resolution=(640, 480), fps=30.0)
except ValueError as e:
    print(f"매개변수 오류: {e}")
    # "camera_id cannot be negative: -1"

try:
    # 잘못된 해상도 값
    camera = OpenCvCamera(0, resolution=(1920, -1080), fps=30.0)
except ValueError as e:
    print(f"해상도 오류: {e}")
    # "resolution width and height must be positive integers: (1920, -1080)"

try:
    # 잘못된 해상도 문자열 형식
    camera = OpenCvCamera(0, resolution="1920-1080", fps=30.0)
except ValueError as e:
    print(f"해상도 형식 오류: {e}")
    # "resolution must be like 1280x720 with positive integers"

try:
    # 0 이하 FPS
    camera = OpenCvCamera(0, resolution=(640, 480), fps=0)
except ValueError as e:
    print(f"FPS 오류: {e}")
    # "fps must be > 0, got 0"
```

### 2. 카메라 열기 실패

`open()`은 카메라 열기에 실패하면 진단 메시지를 포함한 `RuntimeError`를 발생시킵니다.

```python
camera = OpenCvCamera(999, resolution=(640, 480), fps=30.0)  # 존재하지 않는 카메라

try:
    camera.open()
except RuntimeError as e:
    print(f"열기 실패: {e}")
    # "Failed to open camera (camera_id=999): device not connected,
    #  already in use by another process, or invalid device number"
```

진단 메시지는 소스 타입에 따라 다릅니다:

| 소스 | 진단 메시지 예시 |
|------|-----------------|
| 정수 ID | `device not connected, already in use by another process, or invalid device number` |
| URI 스트림 | `check network connectivity, credentials, or stream availability` |
| 파일 경로 (미존재) | `path does not exist: /path/to/video.mp4` |
| 파일 경로 (권한 없음) | `no read permission: /path/to/video.mp4` |
| 파일 경로 (기타) | `file exists but failed to open: /path/to/video.mp4` |

### 3. 중복 열기 시도

```python
camera = OpenCvCamera(0, resolution=(640, 480), fps=30.0)
camera.open()

try:
    camera.open()  # 이미 열린 상태에서 다시 시도
except RuntimeError as e:
    print(f"중복 열기 오류: {e}")
    # "Camera is already opened"
```

### 4. 프레임 읽기 실패

`read()`는 프레임 획득 실패 시 `None`을 반환하며, 내부적으로 5초 간격으로 warning 로그를 자동 출력합니다.

```python
frame = camera.read()
if frame is None:
    # 로그에 자동 출력됨: "Frame read failed (camera_id=0)"
    # 별도의 로깅 없이 None 체크만 하면 됨
    pass
```

카메라가 열려있지 않은 상태에서 `read()`를 호출하면 매 호출마다 warning이 출력됩니다:
`"read() called but camera is not opened"`

## 로깅

OpenCvCamera는 Python의 표준 logging 모듈을 사용합니다.

### 1. 로깅 설정

```python
import logging

# 기본 로깅 설정
logging.basicConfig(
    level=logging.WARNING,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

# 특정 모듈만 로깅
logger = logging.getLogger('rdfp.camera.opencv_camera')
logger.setLevel(logging.INFO)

# 파일로 로깅
handler = logging.FileHandler('camera.log')
handler.setFormatter(logging.Formatter(
    '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
))
logger.addHandler(handler)
```

### 2. 로그 레벨별 메시지

| 레벨 | 내용 | 예시 |
|------|------|------|
| `WARNING` | 설정 불일치, 프레임 읽기 실패 | 요청한 해상도와 실제 해상도 다름, `Frame read failed` (5초 간격) |
| `WARNING` | 잘못된 read() 호출 | `read() called but camera is not opened` |

> 카메라 열기 실패는 로그가 아닌 `RuntimeError` 예외로 보고됩니다. 진단 정보는 예외 메시지에 포함됩니다.

### 3. 로그 분석

```bash
# 프레임 읽기 실패 확인
grep "Frame read failed" camera.log

# 해상도 조정 패턴 분석
grep "Requested resolution.*differs" camera.log
```

## Best Practices

### 1. 리소스 관리

```python
# 권장: Context Manager 사용
with OpenCvCamera(0, resolution=(640, 480), fps=30.0) as camera:
    # 작업 수행
    pass

# 권장: 명시적 해제
camera = OpenCvCamera(0, resolution=(640, 480), fps=30.0)
try:
    camera.open()
    # 작업 수행
finally:
    camera.release()
```

### 2. 설정 선택

```python
# 권장: 합리적인 기본값
camera = OpenCvCamera(0, resolution=(1920, 1080), fps=30.0)  # 일반적인 설정

# 권장: 하드웨어 제한 고려
camera = OpenCvCamera(0, resolution=(640, 480), fps=30.0)    # 저사양에서 안정적
```

### 3. 에러 처리

```python
# 권장: 구체적인 예외 처리
try:
    camera = OpenCvCamera(camera_id, resolution=resolution, fps=fps)
    camera.open()
    # 성공 시 처리
except ValueError as e:
    logger.error(f"매개변수 오류: {e}")
except RuntimeError as e:
    logger.error(f"카메라 열기 실패: {e}")
finally:
    camera.release()
```

### 4. 스트림 처리

```python
# 권장: 프레임 유효성 확인
# read()가 실패 시 5초 간격으로 warning을 자동 출력하므로
# 호출자가 별도 로깅을 할 필요 없음
while True:
    frame = camera.read()
    if frame is None:
        break

    # 프레임 처리
    if process_frame(frame):
        break
```

## 예제 코드

### 1. 웹캠으로 실시간 처리

```python
import cv2
from rdfp.camera.opencv_camera import OpenCvCamera

def webcam_demo():
    with OpenCvCamera(0, resolution=(640, 480), fps=30.0) as camera:
        print(f"카메라 설정: {camera.resolution} @ {camera.fps}fps")
        
        while True:
            frame = camera.read()
            if frame is None:
                break
            
            # 프레임 처리 (예: 그레이스케일 변환)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # 화면 출력
            cv2.imshow('Camera', gray)
            
            # 'q' 키로 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cv2.destroyAllWindows()

if __name__ == "__main__":
    webcam_demo()
```

### 2. RTSP 스트림 녹화

```python
import cv2
from rdfp.camera.opencv_camera import OpenCvCamera

def record_rtsp_stream(rtsp_url, output_file, duration_seconds=60):
    camera = OpenCvCamera(rtsp_url, resolution=(1920, 1080), fps=30.0)
    
    try:
        (width, height), fps = camera.open()
        print(f"스트림 설정: {width}x{height} @ {fps}fps")
        
        # 비디오 라이터 설정
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_file, fourcc, fps, (width, height))
        
        frame_count = 0
        max_frames = int(duration_seconds * fps)
        
        while frame_count < max_frames:
            frame = camera.read()
            if frame is None:
                print("프레임 읽기 실패")
                break
            
            out.write(frame)
            frame_count += 1
            
            status_interval = max(1, int(fps * 10))
            if frame_count % status_interval == 0:  # 10초마다 상태 출력
                elapsed = frame_count / fps
                print(f"녹화 진행: {elapsed:.1f}초 / {duration_seconds}초")
        
        out.release()
        print(f"녹화 완료: {output_file}")
        return True
    
    except RuntimeError as e:
        print(f"RTSP 스트림 연결 실패: {e}")
        return False
    
    finally:
        camera.release()

if __name__ == "__main__":
    record_rtsp_stream(
        'rtsp://192.168.1.100:554/stream', 
        'recorded_stream.mp4', 
        60
    )
```

### 3. 다중 카메라 동기화

```python
import cv2
import numpy as np
from rdfp.camera.opencv_camera import OpenCvCamera

def multi_camera_view(camera_ids, resolution=(640, 480)):
    cameras = []
    
    try:
        # 카메라들 초기화
        for cam_id in camera_ids:
            camera = OpenCvCamera(cam_id, resolution=resolution, fps=30.0)
            try:
                camera.open()
                cameras.append(camera)
                print(f"카메라 {cam_id} 준비 완료")
            except RuntimeError:
                print(f"카메라 {cam_id} 열기 실패")
                camera.release()
        
        if not cameras:
            print("사용 가능한 카메라가 없습니다")
            return
        
        print(f"{len(cameras)}개 카메라로 시작")
        
        while True:
            frames = []
            
            # 모든 카메라에서 프레임 읽기
            for camera in cameras:
                frame = camera.read()
                if frame is not None:
                    frames.append(frame)
                else:
                    # 실패한 프레임은 검은 화면으로 대체
                    frames.append(np.zeros((resolution[1], resolution[0], 3), dtype=np.uint8))
            
            if frames:
                # 프레임들을 격자로 배열
                rows = int(np.ceil(np.sqrt(len(frames))))
                cols = int(np.ceil(len(frames) / rows))
                
                # 빈 슬롯을 검은 화면으로 채움
                while len(frames) < rows * cols:
                    frames.append(np.zeros((resolution[1], resolution[0], 3), dtype=np.uint8))
                
                # 격자 생성
                grid_rows = []
                for r in range(rows):
                    row_frames = frames[r*cols:(r+1)*cols]
                    grid_rows.append(np.hstack(row_frames))
                
                combined = np.vstack(grid_rows)
                
                # 화면 출력
                cv2.imshow('Multi-Camera View', combined)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    
    finally:
        # 모든 카메라 해제
        for camera in cameras:
            camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # 카메라 0, 1, 2 동시 표시
    multi_camera_view([0, 1, 2])
```

## 트러블슈팅

### 자주 발생하는 문제들

#### 1. 카메라를 찾을 수 없음

**증상**: `RuntimeError: Failed to open camera (camera_id=X): device not connected, already in use...`
**해결**:
```bash
# Linux에서 사용 가능한 카메라 확인
ls /dev/video*

# 카메라 정보 확인
v4l2-ctl --list-devices
```

#### 2. 권한 문제

**증상**: `RuntimeError: Failed to open camera (camera_id=...): no read permission: ...`
**해결**:
```bash
# 카메라 디바이스 권한 확인
ls -l /dev/video0

# 사용자를 video 그룹에 추가
sudo usermod -a -G video $USER

# 재로그인 필요
```

#### 3. 다른 프로세스가 카메라 점유

**증상**: `RuntimeError: Failed to open camera (camera_id=...): device not connected, already in use...`
**해결**:
```bash
# 카메라를 사용하는 프로세스 찾기
sudo lsof /dev/video0

# 또는 fuser 사용
sudo fuser /dev/video0
```

#### 4. 네트워크 스트림 연결 실패

**증상**: `RuntimeError: Failed to open camera (camera_id=rtsp://...): check network connectivity...`
**해결**:
- 네트워크 연결 확인
- 스트림 URL 및 포트 확인
- 방화벽 설정 확인
- 카메라 인증 정보 확인

#### 5. 해상도/FPS 제한

**증상**: `Requested resolution ... differs from actual resolution` (WARNING 로그)
**해결**:
- 카메라 사양 확인
- 더 낮은 해상도나 FPS로 시도
- USB 대역폭 확인 (USB 2.0 vs 3.0)

#### 6. 프레임 읽기 반복 실패

**증상**: `Frame read failed (camera_id=...)` (WARNING 로그, 5초 간격)
**해결**:
- 카메라 연결 상태 확인
- USB 케이블 접촉 불량 확인
- 네트워크 스트림의 경우 대역폭/안정성 확인

### 디버깅 팁

```python
# 1. 로깅 레벨을 DEBUG로 설정
import logging
logging.getLogger('rdfp.camera.opencv_camera').setLevel(logging.DEBUG)

# 2. 카메라 백엔드 정보 확인
import cv2
print("Available backends:", [cv2.videoio_registry.getBackendName(b) for b in cv2.videoio_registry.getBackends()])

# 3. OpenCV 빌드 정보 확인
print("OpenCV version:", cv2.__version__)
print("OpenCV build info:")
print(cv2.getBuildInformation())
```

## 결론

`OpenCvCamera` 클래스는 다양한 카메라 소스를 안전하고 편리하게 사용할 수 있게 해주는 도구입니다. Context Manager 패턴을 활용하고, 적절한 에러 처리와 로깅을 통해 안정적인 비디오 처리 애플리케이션을 개발할 수 있습니다.

추가 질문이나 이슈가 있다면 프로젝트 로그를 확인하거나 개발팀에 문의하시기 바랍니다.
