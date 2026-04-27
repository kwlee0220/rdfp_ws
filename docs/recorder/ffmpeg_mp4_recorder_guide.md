# FFMpegMp4Recorder Programmer's Guide

`ffmpeg` subprocess를 이용해 OpenCV 이미지(`numpy.ndarray`)를 MP4 파일로
녹화하는 Python 클래스입니다. 본 가이드는 `FFMpegMp4Recorder`를 **어떻게
사용하는지**에 초점을 두며, 공개 API의 상세 레퍼런스는
[ffmpeg_mp4_recorder_api.md](./ffmpeg_mp4_recorder_api.md)를 참고하세요.

## 개요

`FFMpegMp4Recorder`는 호출자가 주입하는 프레임을 별도 writer 스레드가
`ffmpeg` stdin으로 전달하여 CFR(Constant Frame Rate) MP4 파일을 생성합니다.
ROS2에 의존하지 않는 순수 Python 클래스이므로 독립 스크립트, ROS2 노드,
테스트 하네스 어디서든 동일하게 사용할 수 있습니다.

### 주요 특징

- **CFR 패스스루**: `write()` 호출 순서를 그대로 `ffmpeg`에 전달 (리샘플링 없음)
- **GPU/CPU 인코더 자동 선택**: 생성자에서 1회 probe 후 고정
- **명시적 상태 모델**: `IDLE`/`RECORDING`/`STOPPING`/`FAILED`/`SHUTDOWN`
- **drop_oldest 백프레셔**: 큐가 가득차면 가장 오래된 프레임을 버리고 최신 프레임 유지
- **재사용 가능**: `stop()` 이후 동일 인스턴스로 재녹화 가능
- **컨텍스트 매니저 지원**: 예외 경로에서도 `shutdown()` 보장
- **오디오 미지원**: 비디오 전용

## 설치 및 의존성

### 시스템 의존성

```bash
# ffmpeg 설치 (Ubuntu)
sudo apt install ffmpeg

# GPU 인코딩을 사용하려면 해당 인코더가 포함된 ffmpeg 빌드 필요
# - NVIDIA: h264_nvenc
# - Intel QSV: h264_qsv
# - VAAPI: h264_vaapi (+ /dev/dri/renderD128 접근 권한)
```

### Python 의존성

```python
# 필수
import numpy as np  # 프레임 입력 타입
# ffmpeg은 subprocess로 호출되므로 Python 바인딩 불필요
```

### Import

```python
from rdfp.recorder import FFMpegMp4Recorder
from rdfp.recorder import (
    EncoderUnavailableError,
    InvalidFrameError,
    RecorderError,
    RecorderStateError,
)
from rdfp.types import Resolution
```

## 생성자 시그니처

```python
FFMpegMp4Recorder(
    *,
    fps: int,
    resolution: str | tuple[int, int] | Resolution,
    pixel_format: str = "bgr8",
    encoder_mode: str = "auto",
    preferred_hw_codec: str | None = None,
    bitrate: str = "4M",
    gop_size: int | None = None,
    preset: str = "medium",
    ffmpeg_binary: str = "ffmpeg",
    vaapi_device: str = "/dev/dri/renderD128",
    queue_size: int = 120,
    logger: logging.Logger | None = None,
) -> None
```

- **모든 인자가 keyword-only**입니다.
- `fps`, `resolution`, `pixel_format`, 인코더 관련 설정은 생성자에서 **고정**되며,
  이후 변경할 수 없습니다. 다른 해상도로 녹화하려면 새 인스턴스를 만드세요.
- `resolution`은 세 가지 형식을 모두 허용합니다:
  - `Resolution(1280, 720)` NamedTuple
  - `(1280, 720)` 튜플
  - `"1280x720"` 문자열
- `gop_size`를 `None`으로 두면 `fps * 2`가 적용됩니다.
- `encoder_mode`:
  - `"auto"` — GPU 인코더를 우선 시도하고 실패 시 CPU(libx264)로 폴백
  - `"cpu"` — 항상 libx264
  - `"gpu"` — GPU 인코더만 허용 (없으면 `EncoderUnavailableError`)
- `queue_size`는 프레임 백프레셔 큐의 최대 길이입니다. 가득차면 **가장 오래된
  프레임부터 버립니다** (drop_oldest).

### 생성 시 발생 가능한 예외

| 예외 | 발생 조건 |
|------|----------|
| `ValueError` | `fps`가 양수가 아니거나, `pixel_format`이 지원 목록에 없거나, `queue_size`/`gop_size`가 잘못된 경우 |
| `ValueError` | `resolution` 파싱 실패 (음수, 잘못된 문자열 등) |
| `EncoderUnavailableError` | `encoder_mode="gpu"`인데 사용 가능한 HW 인코더가 없는 경우 |

## 상태 모델

```
       start()                stop() (success)
 IDLE ─────────► RECORDING ─────────────────────► IDLE
  ▲                │                                │
  │                │ stop() (error) / writer error  │
  │                ▼                                │
  │             STOPPING ────────► FAILED           │
  │                                   │             │
  └───────────────────────────────────┘             │
                  start() from FAILED               │
                                                    │
  IDLE / RECORDING / STOPPING / FAILED ──shutdown()─┴─► SHUTDOWN (terminal)
```

- `start()`는 `IDLE` 또는 `FAILED`에서만 호출 가능
- `write()`는 `RECORDING`에서만 호출 가능
- `stop()`은 `RECORDING`에서만 호출 가능
- `shutdown()`은 어느 상태에서든 호출 가능하고 **idempotent**
- `SHUTDOWN`은 종착 상태 — 이후 어떤 public 메서드도 호출할 수 없음

현재 상태는 `recorder.state` 프로퍼티로 확인할 수 있습니다 (스냅샷).

## 기본 사용법

### 1. 가장 단순한 패턴

```python
import numpy as np
from rdfp.recorder import FFMpegMp4Recorder
from rdfp.types import Resolution

rec = FFMpegMp4Recorder(
    fps=30,
    resolution=Resolution(640, 480),
    pixel_format="bgr8",
    encoder_mode="cpu",
)
try:
    rec.start("/tmp/out.mp4")
    for i in range(300):  # 10초
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        rec.write(frame, timestamp_ms=i * 33)
    rec.stop()
finally:
    rec.shutdown()
```

### 2. Context Manager 패턴 (권장)

```python
with FFMpegMp4Recorder(fps=30, resolution=(640, 480)) as rec:
    rec.start("/tmp/out.mp4")
    for frame in frames():
        rec.write(frame, timestamp_ms=0)
    rec.stop()
# __exit__ 이 shutdown() 을 호출 — 예외가 발생해도 자원이 정리됨
```

> `__enter__`는 단순히 `self`를 반환할 뿐 `start()`를 자동 호출하지 않습니다.
> 녹화를 시작하려면 블록 안에서 명시적으로 `rec.start(output_path)`를
> 호출해야 합니다. 이는 동일 인스턴스로 여러 파일을 연속 녹화하는 시나리오를
> 지원하기 위함입니다.

### 3. 동일 인스턴스로 여러 파일 녹화

생성자 probe 비용을 한 번만 지불하고 여러 파일을 녹화할 수 있습니다.

```python
with FFMpegMp4Recorder(fps=30, resolution=(640, 480)) as rec:
    for idx, output_path in enumerate(["/tmp/clip1.mp4", "/tmp/clip2.mp4"]):
        rec.start(output_path)
        for frame in capture_clip(idx):
            rec.write(frame, timestamp_ms=0)
        rec.stop()  # → IDLE 복귀 (재녹화 가능)
```

### 4. pixel_format 별 프레임 shape

| `pixel_format` | 허용 shape | dtype |
|---------------|------------|-------|
| `"bgr8"` | `(H, W, 3)` | `uint8` |
| `"rgb8"` | `(H, W, 3)` | `uint8` |
| `"mono8"` | `(H, W)` 또는 `(H, W, 1)` | `uint8` |

`(H, W, 1)` 형태의 mono8 프레임은 내부에서 자동으로 `(H, W)`로 reshape됩니다.
잘못된 shape/dtype은 `InvalidFrameError`(ValueError 서브클래스)로 거부됩니다.

## 고급 사용법

### 1. 인코더 폴백 전략

GPU 인코딩이 실패해도 녹화를 계속하고 싶다면 `"auto"`를 사용합니다.

```python
rec = FFMpegMp4Recorder(
    fps=30,
    resolution="1920x1080",
    pixel_format="bgr8",
    encoder_mode="auto",          # GPU 우선, 실패 시 CPU 폴백
    preferred_hw_codec="h264_nvenc",  # 지정 시 해당 코덱만 probe
    bitrate="8M",
)
print(f"selected codec: {rec.selected_codec}")
# 가능한 값: "h264_nvenc", "h264_qsv", "h264_vaapi", "libx264"
```

반대로 반드시 GPU를 요구해야 한다면:

```python
try:
    rec = FFMpegMp4Recorder(
        fps=60, resolution=(1920, 1080), encoder_mode="gpu",
    )
except EncoderUnavailableError as e:
    print(f"GPU encoder not available: {e}")
    # 대체 코드 경로로 전환
```

### 2. FAILED 상태에서 재시작

녹화 중 writer 스레드가 ffmpeg stdin 쓰기에 실패하면 인스턴스는 `FAILED`로
전이됩니다. `start()`를 다시 호출하여 새 파일로 복귀할 수 있습니다.

```python
with FFMpegMp4Recorder(fps=30, resolution=(640, 480)) as rec:
    rec.start("/tmp/first.mp4")
    try:
        run_recording_loop(rec)
    except RecorderStateError:
        if rec.state == "FAILED":
            rec.logger.warning("recorder in FAILED state; restarting")
            rec.start("/tmp/second.mp4")
            run_recording_loop(rec)
```

### 3. 드롭 통계 모니터링

큐가 가득차서 드롭되는 프레임은 카운트되며 주기적으로 경고 로그가 남습니다.

```python
with FFMpegMp4Recorder(fps=30, resolution=(640, 480), queue_size=240) as rec:
    rec.start("/tmp/out.mp4")
    producer_loop(rec)
    rec.stop()
    print(f"written: {rec.frames_written}, dropped: {rec.frames_dropped}")
    drop_ratio = rec.frames_dropped / max(1, rec.frames_written + rec.frames_dropped)
    if drop_ratio > 0.01:
        print(f"WARNING: drop ratio {drop_ratio:.2%} > 1%")
```

드롭이 많다면:
- `queue_size`를 늘려 일시적 버스트를 흡수하거나
- `encoder_mode`/`preset`/`bitrate`를 조정하여 인코딩 속도를 높이거나
- 생산자 쪽 프레임 레이트를 낮춥니다.

### 4. 사용자 지정 로거 주입

여러 인스턴스를 운영하거나 별도 로그 채널로 분리하고 싶을 때:

```python
import logging

app_logger = logging.getLogger("my_app.recorder.front_camera")
app_logger.setLevel(logging.DEBUG)

rec = FFMpegMp4Recorder(
    fps=30,
    resolution=(640, 480),
    logger=app_logger,
)
```

로거를 생략하면 `rdfp.recorder.ffmpeg_mp4_recorder.FFMpegMp4Recorder` 이름의
기본 로거가 사용됩니다.

### 5. `stop()` 타임아웃 조정

`stop()`은 drain + finalize **전체**에 대한 단일 시간 예산을 사용합니다.
큰 해상도나 느린 CPU에서는 finalize가 지연될 수 있으므로 넉넉히 주는 것이
좋습니다.

```python
output_path = rec.stop(timeout=15.0)  # 기본값 5초
```

예산이 소진되면 `terminate` → `kill` 순으로 ffmpeg을 강제 종료하며, 이 경우
인스턴스는 `FAILED`로 전이되고 출력 파일은 사용 불가능할 수 있습니다.

## 에러 처리

### 예외 계층

```
RuntimeError
└── RecorderError                 (기본 클래스)
    ├── RecorderStateError        (잘못된 상태에서 호출)
    └── EncoderUnavailableError   (GPU probe 실패)

ValueError
└── InvalidFrameError             (프레임 shape/dtype 불일치)
```

### 잘못된 상태 호출

```python
rec = FFMpegMp4Recorder(fps=30, resolution=(640, 480))

try:
    rec.write(frame, 0)  # 아직 start() 안 함
except RecorderStateError as e:
    print(e)
    # "operation not allowed in state IDLE; expected one of ['RECORDING']"

rec.start("/tmp/out.mp4")
try:
    rec.start("/tmp/other.mp4")  # 이미 RECORDING
except RecorderStateError as e:
    print(e)
    # "operation not allowed in state RECORDING; expected one of ['IDLE', 'FAILED']"
```

### 파일 충돌

`start()`는 경로에 파일이 이미 존재하면 즉시 `FileExistsError`로 실패합니다
(덮어쓰기 방지). ffmpeg의 `-n` 옵션과 이중 방어 구조입니다.

```python
try:
    rec.start("/tmp/out.mp4")
except FileExistsError as e:
    print(f"output exists: {e}")
    # 타임스탬프를 포함한 경로로 재시도
    rec.start(f"/tmp/out_{int(time.time())}.mp4")
```

### 프레임 검증 실패

```python
try:
    rec.write(np.zeros((480, 640), dtype=np.uint8), 0)  # bgr8인데 2D
except InvalidFrameError as e:
    print(e)
    # "bgr8 image shape mismatch: expected (480, 640, 3), got (480, 640)"

try:
    rec.write(np.zeros((480, 640, 3), dtype=np.float32), 0)  # 잘못된 dtype
except InvalidFrameError as e:
    print(e)
    # "image dtype must be uint8, got float32"
```

### GPU 인코더 요구 실패

```python
try:
    rec = FFMpegMp4Recorder(fps=30, resolution=(1920, 1080), encoder_mode="gpu")
except EncoderUnavailableError as e:
    print(f"no GPU encoder: {e}")
```

## 로깅

`FFMpegMp4Recorder`는 Python 표준 `logging` 모듈을 사용합니다.

### 로그 레벨

| 레벨 | 내용 | 예시 |
|------|------|------|
| `ERROR` | 치명적 실패 | ffmpeg 비정상 종료, writer 스레드 오류, 출력 파일 부재 |
| `WARNING` | 주의 필요 상황 | 프레임 드롭, 비연속 numpy 배열 복사 |
| `INFO` | 주요 생명주기 이벤트 | `recorder started`, `recorder stopped` |
| `DEBUG` | 상세 진단 | 상태 전이, ffmpeg stderr 출력, 생성 시 설정값 |

### 로깅 설정 예시

```python
import logging

# 전체 recorder 패키지 로그 활성화
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logging.getLogger("rdfp.recorder").setLevel(logging.DEBUG)
```

### ffmpeg stderr 확인

`ffmpeg` 의 stderr 출력은 DEBUG 레벨로 전달됩니다. 디버깅할 때만 켜세요.

```python
logging.getLogger(
    "rdfp.recorder.ffmpeg_mp4_recorder.FFMpegMp4Recorder"
).setLevel(logging.DEBUG)
```

## Best Practices

### 1. 자원 관리

```python
# 권장: Context manager
with FFMpegMp4Recorder(fps=30, resolution=(640, 480)) as rec:
    rec.start(output_path)
    # ...
    rec.stop()

# 권장: try/finally
rec = FFMpegMp4Recorder(fps=30, resolution=(640, 480))
try:
    rec.start(output_path)
    # ...
    rec.stop()
finally:
    rec.shutdown()  # idempotent

# 비권장: shutdown() 생략
rec = FFMpegMp4Recorder(...)
rec.start(...)
# shutdown() 없음 → writer/ffmpeg 좀비 가능성
```

### 2. 프레임은 C-contiguous로

`numpy` slicing(`frame[::2]`, `frame[:, ::2]` 등)이나 `np.transpose`로
만든 뷰는 non-contiguous일 수 있습니다. 이 경우 recorder가 내부에서 복사본을
만들어 성능에 영향을 주므로, 가능하다면 호출 전에 미리 연속 배열로 만들어
두는 것이 좋습니다.

```python
frame = np.ascontiguousarray(raw_frame)
rec.write(frame, timestamp_ms=ts)
```

첫 번째 non-contiguous 프레임을 받으면 WARNING 로그가 1회 남습니다.

### 3. 프로듀서/컨슈머 분리

`write()`는 즉시 큐에 넣고 반환하므로 호출 스레드를 블록하지 않습니다.
프로듀서 루프에서 부담 없이 호출할 수 있지만, 인코딩이 느리면 드롭이
발생합니다. `queue_size`는 생산자의 버스트와 인코더 처리 속도의 간극을 메울
수 있을 만큼 잡으세요 (기본 120프레임 = 30fps에서 4초).

### 4. `timestamp_ms`는 진단용

`timestamp_ms` 인자는 현재 구현에서 출력 영상 타이밍에 영향을 **주지
않습니다** (CFR 패스스루). 호출 순서대로 프레임이 순서 기록되며, 진단이나
추후 확장을 위한 정보성 필드로 예약되어 있습니다.

### 5. `FAILED` 상태 자동 재시작

장시간 녹화 파이프라인이라면 writer 스레드 실패 시에도 복구를 시도하는 래퍼를
작성하는 것이 안전합니다.

```python
def record_with_retry(rec, base_path, frames_source, max_retries=3):
    for attempt in range(max_retries + 1):
        path = f"{base_path}_{attempt:02d}.mp4"
        rec.start(path)
        try:
            for frame in frames_source:
                rec.write(frame, 0)
            return rec.stop()
        except RecorderStateError:
            if rec.state != "FAILED" or attempt == max_retries:
                raise
            rec.logger.warning("attempt %d failed, retrying", attempt)
```

## 예제

### 1. OpenCV 웹캠 → MP4 녹화

```python
import cv2
import numpy as np

from rdfp.camera.opencv_camera import OpenCvCamera
from rdfp.recorder import FFMpegMp4Recorder
from rdfp.types import Resolution

RES = Resolution(1280, 720)
FPS = 30

with OpenCvCamera(0, resolution=RES, fps=FPS) as cam, \
     FFMpegMp4Recorder(fps=FPS, resolution=RES, encoder_mode="auto") as rec:
    rec.start("/tmp/webcam.mp4")
    try:
        for i in range(FPS * 10):  # 10초
            frame = cam.read()
            if frame is None:
                break
            rec.write(frame, timestamp_ms=int(i * 1000 / FPS))
    finally:
        path = rec.stop()
        print(f"saved: {path} ({rec.frames_written} frames, "
              f"{rec.frames_dropped} dropped)")
```

### 2. 합성 프레임 녹화 (테스트용)

```python
import numpy as np
from rdfp.recorder import FFMpegMp4Recorder

WIDTH, HEIGHT, FPS, DURATION = 640, 480, 30, 5

with FFMpegMp4Recorder(
    fps=FPS, resolution=(WIDTH, HEIGHT), pixel_format="bgr8", encoder_mode="cpu",
) as rec:
    rec.start("/tmp/synthetic.mp4")
    for i in range(FPS * DURATION):
        # 시간에 따라 색상이 변하는 단색 프레임
        hue = int(255 * (i / (FPS * DURATION)))
        frame = np.full((HEIGHT, WIDTH, 3), [hue, 128, 255 - hue], dtype=np.uint8)
        rec.write(frame, timestamp_ms=int(i * 1000 / FPS))
    rec.stop()
```

### 3. 세그먼트 녹화 (N초마다 파일 분할)

```python
import time
from rdfp.recorder import FFMpegMp4Recorder

SEGMENT_SECONDS = 60
FPS = 30

def produce_frames():
    """사용자 구현 — 프레임을 yield."""
    ...

with FFMpegMp4Recorder(fps=FPS, resolution=(1280, 720)) as rec:
    segment_idx = 0
    segment_start = time.monotonic()
    rec.start(f"/tmp/seg_{segment_idx:04d}.mp4")

    for frame in produce_frames():
        rec.write(frame, timestamp_ms=0)

        if time.monotonic() - segment_start >= SEGMENT_SECONDS:
            rec.stop()
            segment_idx += 1
            segment_start = time.monotonic()
            rec.start(f"/tmp/seg_{segment_idx:04d}.mp4")

    rec.stop()
```

## 트러블슈팅

### 1. `EncoderUnavailableError: no hardware encoder available`

**원인**: `encoder_mode="gpu"`인데 시스템에 사용 가능한 HW 인코더가 없음.

**확인**:
```bash
# ffmpeg이 지원하는 인코더 목록
ffmpeg -hide_banner -encoders | grep -E "nvenc|qsv|vaapi"

# NVIDIA GPU 확인
nvidia-smi

# VAAPI 디바이스 확인
ls -l /dev/dri/renderD*
```

**해결**:
- `encoder_mode="auto"`로 변경하여 CPU 폴백 허용
- VAAPI 사용 시 `$USER`가 `render` 그룹에 속해 있는지 확인:
  `sudo usermod -aG render $USER` 후 재로그인

### 2. 출력 파일이 비어있거나 생성되지 않음

**증상**: `stop()` 후 로그에 `output file not found` 또는 `output file is empty`.

**원인 가능성**:
- ffmpeg이 시작 직후 실패 (인코더/픽셀 포맷 불일치 등) → stderr DEBUG 로그 확인
- 모든 프레임이 드롭되어 ffmpeg이 유효한 스트림을 받지 못함
- `stop(timeout=...)`이 너무 짧아 finalize가 잘렸음

**해결**:
- DEBUG 로그 활성화하여 ffmpeg stderr 확인
- `rec.frames_written`을 확인하여 실제로 프레임이 전달됐는지 검증
- `stop(timeout=15.0)` 등으로 타임아웃 여유 확보

### 3. 프레임 드롭이 많음

**증상**: `rec.frames_dropped`가 지속적으로 증가하고 `frame dropped: queue full` 경고.

**원인**: 인코더 처리 속도가 입력 속도를 따라가지 못함.

**해결**:
- `encoder_mode="auto"` 또는 `"gpu"`로 HW 인코더 사용
- `preset`을 `"fast"` 또는 `"ultrafast"`로 변경 (CPU 인코딩 시)
- `bitrate`를 낮춰 인코딩 부하 감소
- `queue_size`를 늘려 일시적 버스트 흡수 (근본 해결책은 아님)
- 입력 해상도/FPS 낮추기

### 4. `InvalidFrameError: ... shape mismatch`

**원인**: recorder 생성자에 지정한 해상도/픽셀 포맷과 입력 프레임이 불일치.

**확인**:
```python
print(f"recorder expects: {rec.state} {rec._resolution} {rec._pixel_format}")
print(f"frame shape: {frame.shape}, dtype: {frame.dtype}")
```

**해결**:
- `cv2.resize(frame, (width, height))` 로 크기 맞추기
  (주의: OpenCV는 `(width, height)`, numpy shape는 `(height, width)`)
- mono8인데 `(H, W, 3)` 프레임이 들어오는 경우
  `cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)` 로 변환

### 5. `RecorderStateError: operation not allowed in state SHUTDOWN`

**원인**: `shutdown()` 또는 `__del__`이 이미 호출된 인스턴스를 계속 사용.

**해결**: 새 인스턴스를 만드세요. `SHUTDOWN`은 종착 상태입니다.

### 6. 디버깅 팁

```python
# 1. 상태 전이와 ffmpeg stderr를 모두 확인
import logging
logging.basicConfig(level=logging.DEBUG)

# 2. 선택된 코덱 확인 (생성자 probe 결과)
rec = FFMpegMp4Recorder(fps=30, resolution=(640, 480), encoder_mode="auto")
print(f"codec: {rec.selected_codec}")

# 3. ffmpeg 명령줄을 직접 확인하려면 build_ffmpeg_command() 호출
from rdfp.recorder.ffmpeg_command import build_ffmpeg_command
cmd = build_ffmpeg_command(
    ffmpeg_binary="ffmpeg", pixel_format="bgr8", width=640, height=480,
    fps=30, codec="libx264", bitrate="4M", gop_size=60,
    output_path="/tmp/test.mp4", preset="medium",
    vaapi_device="/dev/dri/renderD128",
)
print(" ".join(cmd))
```

## 관련 문서

- [OpenCvCamera Programmer's Guide](../camera/opencv_camera_guide.md) — 카메라 입력 소스
