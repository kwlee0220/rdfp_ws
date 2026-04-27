# FFMpegMp4Recorder

ROS2 비의존 MP4 녹화 모듈. OpenCV 이미지(`numpy.ndarray`)를 받아 ffmpeg
subprocess 로 MP4 파일을 생성한다.

## 특징

- **CFR + passthrough** — 쓴 순서대로 그대로 인코딩 (재샘플링 없음)
- **CPU / GPU 인코더** — `libx264` / `h264_nvenc` / `h264_qsv` / `h264_vaapi`
- **GPU probe 는 생성자에서 1 회** — 런타임 오버헤드 없음
- **Drop-oldest 큐 정책** — 큐가 가득 차면 가장 오래된 프레임 제거
- **오디오 미지원**, **ROS2 비의존**

## 요구사항

- ffmpeg ≥ 4.x (시스템 PATH)
- Python ≥ 3.8
- numpy

## 빠른 시작

```python
import numpy as np
from rdfp.recorder import FFMpegMp4Recorder
from rdfp.types import Resolution

with FFMpegMp4Recorder(
    fps=30, resolution=Resolution(640, 480),
    pixel_format="bgr8",
    encoder_mode="cpu",  # "auto" / "cpu" / "gpu"
) as rec:
    rec.start("/tmp/out.mp4")
    for _ in range(300):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        rec.write(frame)
    rec.stop()
```

더 완전한 예제는 [`rdfp/samples/sample_ffmpeg_mp4_recorder.py`](../samples/sample_ffmpeg_mp4_recorder.py) 참조.

## 생성자 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `fps` | (필수) | Constant frame rate (양의 정수) |
| `resolution` | (필수) | `Resolution(width, height)` NamedTuple (px). OpenCV 순서 |
| `pixel_format` | `"bgr8"` | `bgr8` / `rgb8` / `bgra8` / `rgba8` / `mono8` |
| `encoder_mode` | `"auto"` | `auto` / `cpu` / `gpu` |
| `preferred_hw_codec` | `None` | 지정 시 해당 HW 코덱만 probe |
| `bitrate` | `"4M"` | ffmpeg `-b:v` 값 |
| `gop_size` | `fps * 2` | GOP 크기 |
| `preset` | `"medium"` | libx264 preset |
| `ffmpeg_binary` | `"ffmpeg"` | ffmpeg 실행 파일 경로 |
| `vaapi_device` | `/dev/dri/renderD128` | VAAPI 디바이스 |
| `queue_size` | `120` | 프레임 큐 크기 (≈4 초 @ 30fps) |
| `logger` | `None` | 외부 주입 로거 |

## 공개 메서드

| 메서드 | 설명 |
|---|---|
| `start(output_path)` | 녹화 시작. 파일 존재 시 `FileExistsError` |
| `write(image)` | 프레임 큐잉 (CFR-passthrough — stamp 는 받지 않는다) |
| `stop(timeout=5.0)` | 녹화 종료, 최종 경로 반환 |
| `shutdown()` | 자원 반환, idempotent |

## 상태 모델

```
        start()              stop() 완료
 IDLE ───────────► RECORDING ──────────► IDLE
  │                  │                     │
  │                  │ 치명 오류            │
  │                  ▼                     │
  │                FAILED ◄────────────────┘
  │                  │
  │                  │ shutdown()
  │  shutdown()      ▼
  └──────────────► SHUTDOWN
```

| 상태 | 허용 연산 | 비허용 호출 시 |
|---|---|---|
| `IDLE` | `start`, `shutdown` | `RecorderStateError` |
| `RECORDING` | `write`, `stop`, `shutdown` | `RecorderStateError` |
| `STOPPING` | `shutdown` | `RecorderStateError` |
| `FAILED` | `start`, `shutdown` | `RecorderStateError` |
| `SHUTDOWN` | `shutdown` (no-op) | `RecorderStateError` |

## 예외 체계

모듈 외부로 노출되는 예외:

- `RecorderError` (base) — `RuntimeError` 를 상속
- `RecorderStateError` — 허용되지 않는 상태에서의 연산 (SHUTDOWN 포함)
- `InvalidFrameError` — 입력 프레임의 dtype/shape 불일치 (`ValueError` 상속)
- `EncoderUnavailableError` — `encoder_mode="gpu"` 인데 HW 인코더 없음
- `FileExistsError` — `start()` 의 출력 경로가 이미 존재 (built-in)

## 덮어쓰기 금지 이중 방어

출력 파일 덮어쓰기 금지 요구사항은 두 단계로 보장된다:

1. **Python 선검증** — `start()` 진입 시 `os.path.exists()` 체크 → `FileExistsError` 즉시 반환
2. **ffmpeg `-n` 옵션** — 선검증 이후 파일이 생성되는 TOCTOU 경쟁 조건 차단

## timestamp 정책

CFR + passthrough 전략에서 각 프레임의 출력 presentation time 은
`frame_index / fps` 로 결정된다. 따라서 `write()` 는 stamp 인자를 받지
않으며, **원본 stamp 가 필요하면 호출자가 별도 sidecar 에 기록한다**
(참고: `rdfp/dataset/ingest/media/mp4_image_recorder.py` 의 `Mp4ImageRecorder` 는
프레임별 stamp 를 PostgreSQL `image_frames` 테이블에 적재한다).

## 파일 구조

```
rdfp/recorder/
├── __init__.py              # 공개 API export
├── exceptions.py            # 예외 계층
├── state.py                 # RecorderState / RecorderStateMachine
├── encoder_probe.py         # GPU 인코더 probe 로직
├── ffmpeg_command.py        # ffmpeg 커맨드 빌더
├── ffmpeg_mp4_recorder.py   # FFMpegMp4Recorder 본체
├── README.md                # ← 본 문서
├── tests/                   # 단위/통합 테스트
│   ├── __init__.py
│   ├── test_ffmpeg_command.py
│   ├── test_encoder_probe.py
│   ├── test_state.py
│   ├── test_ffmpeg_mp4_recorder.py
│   └── test_integration.py
└── docs/
    ├── prompt.md            # 원 요구사항
    └── ffmpeg_mp4_recorder_plan.md  # 개발 계획서
```

## 테스트 실행

```bash
# 전체
python3 -m pytest rdfp/recorder/tests/ -p no:anyio

# 단위 테스트만 (ffmpeg 바이너리 불필요)
python3 -m pytest rdfp/recorder/tests/ -p no:anyio \
    --ignore=rdfp/recorder/tests/test_integration.py

# 통합 테스트 (ffmpeg/ffprobe 필요)
python3 -m pytest rdfp/recorder/tests/test_integration.py -p no:anyio
```

## 샘플 실행

```bash
# 5초 기본 설정
python3 -m rdfp.samples.sample_ffmpeg_mp4_recorder /tmp/out.mp4

# 옵션 지정
python3 -m rdfp.samples.sample_ffmpeg_mp4_recorder /tmp/out.mp4 \
    --fps 60 --width 1280 --height 720 --duration 3 --encoder-mode auto
```
