## ROS2 MP4 Recorder Node 요구사항 명세서

### 1. 목표

Ubuntu 22.04 + ROS 2 Humble 환경에서, 입력 토픽의 이미지 메시지를 받아 mp4 파일로 기록하는 Python 기반 recorder node를 개발한다. 입력은 ROS2 topic 기반이지만, recording core는 ROS2에 종속되지 않도록 설계한다. 

### 2. 입력 요구사항

* 개발 언어는 Python이다. 
* 지원 메시지 타입은 다음과 같다. 

  * `sensor_msgs/msg/Image`

    * 지원 encoding: `bgr8`, `rgb8`, `mono8`
  * `sensor_msgs/msg/CompressedImage`

    * 지원 format: `jpeg`
* 비지원 encoding/format 입력 시 해당 프레임은 스킵하고 warning 로그를 남긴다. 
* 입력 프레임 주기는 평균 10~30 FPS를 가정하되, 지터와 일시적 burst를 허용한다. 
* 동일 세션에서는 입력 이미지의 해상도나 encoding/format 등이 변하지 않는다.

### 3. 내부 구조 요구사항

시스템은 아래 계층으로 분리한다. ROS2 node는 adaptor 역할만 담당한다. 

* `FrameSourceAdapter`

  * ROS `Image` / `CompressedImage`를 내부 공통 프레임 구조로 변환
    * 입력 토픽 (image_topic)을 통해 들어 오는 이미지 메시지 타입은 중간에 바뀌지
      않는 것을 가정함. 즉 Image 와 CompressedImage가 섞이지 않는다.
    * 입력 토픽이 결정되면 해당 토픽의 메시지 타입으로 Image인지 CompressedImage인지를
      구분할 수 있다. 즉, 별도의 파라미터로 제공할 필요가 없음.
  * 향후 카메라 직접 입력도 동일 구조로 변환 가능해야 함
* `RecorderCore`

  * start/stop 상태 관리
  * timestamp 기반 frame filtering
  * session 관리 및 session metadata 파일 생성
  * sidecar 정보 생성
* `EncoderBackend`

  * ffmpeg subprocess 관리
  * CPU/GPU encoder 선택
  * 파일 finalize 보장

RecorderCore와 EncoderBackend 사이의 내부 공통 프레임 표현은 numpy.ndarray 기반으로 정의한다. 컬러 프레임의 canonical pixel format은 bgr8, grayscale은 mono8로 한다. ROS Image 및 CompressedImage 입력은 FrameSourceAdapter에서 이 공통 포맷으로 변환한다. 프레임의 timestamp, frame_id, 원본 message type, 원본 encoding/format 등의 메타데이터는 이미지 버퍼와 분리된 구조체로 관리한다.

### 4. 인코딩 요구사항

* mp4 생성은 ffmpeg를 사용한다. 
* ffmpeg는 subprocess로 실행하고 stdin pipe를 통해 raw frame을 전달한다. 
* `CompressedImage`는 adaptor 계층에서 decode 후 공통 raw frame 구조로 변환하여 encoder backend에 전달한다.
  * Image 이미지, CompressedImage 이미지 모두 최종적으로 encoder backend에 전달될 때의
    이미지의 encoding이나 포맷 정보를 나중에 기술될 세션 메타데이터 파일에 기록하여
    복원 과정에서 이미지 메시지를 생성할 때 사용한다.
* mp4 프레임간 시간 간격은 FPS를 파라미터로 받고, 들어오는 이미지를 FPS로 계산된
  간격으로 추가하는 방법을 사용한다.
  - 생성된 mp4로 직접 play하는 경우 영상이 실제보다 길어지거나 짧아질 수 있으나
    이것은 단지 간단한 이미지 확인용임
  - 이미지 복원 과정에서는 세션 메타데이터 파일과 sidecar 파일을 활용해서 복원될 수 있다.
* ffmpeg 비정상 종료를 감지하고 현재 recording session을 안전하게 종료해야 한다. 
* 세션 종료 및 노드 종료 시 mp4 파일 finalize를 보장해야 한다.

### 5. 인코더 선택 정책

인코더 선택은 파라미터 `encoder_mode`로 제어한다. 

* `auto`
  * 하드웨어 인코더를 탐지하여 우선 사용
  * 실패 시 `libx264`로 fallback
* `cpu`
  * `libx264` 사용
* `gpu`
  * 하드웨어 인코더 사용 강제
  * 사용 가능한 인코더가 없으면 노드 초기화 실패. 오류 메시지 출력 후 노드는 종료함.

우선 고려 인코더:

* `h264_nvenc`
* `h264_qsv`
* `h264_vaapi`

선택된 최종 인코더는 시작 시 로그로 출력한다.

### 6. timestamp 및 복원 요구사항

* 본 문서에는 *복원* 과 관련된 내용이 기술되지만 'ROS2 MP4 Recorder Node'의 구현 범위에는
  포함되지 않는다. 여기서 언급된 복원과 관련된 내용은 추후 복원에 필요한 정보를 제공하기 위한 목적임
* 모든 프레임의 기준 시각은 입력 메시지의 `msg.header.stamp`로 정의한다. 
* 복원될 이미지의 FPS는 입력으로 설정된 FPS를 사용하기 때문에 세션 메타데이터 파일에 기록된 fps를
  사용한다.
* 저장된 mp4에서 프레임을 복원할 때, 복원 결과 메시지는 항상 `sensor_msgs/msg/Image`로 생성한다. 
* mp4 인코딩/디코딩 과정에서 비트 단위 원본 복원은 요구하지 않는다. 
* 대신 timestamp 및 주요 메타데이터는 원본 메시지와 동일하게 복원 가능해야 한다. 
* 이를 위해 각 mp4 파일과 동일 basename의 sidecar 메타데이터 파일을 생성한다. 
* sidecar 형식은 기본적으로 JSONL을 권장하며, 최소한 아래 정보를 프레임별로 저장해야 한다.
  * `frame_index`
  * `original_message_type`
  * `header_stamp_sec`
  * `header_stamp_nanosec`
  * `frame_id`
<!-- 
  * `width`
  * `height`
  * `recording_session_id`
  * `included_by_ring_buffer`
-->
* 복원 과정은 필요할 수 있는 이미지의 해상도 정보는 세션 메타데이터 파일을 활용한다.

### 7. 녹화 제어 요구사항

#### 7.1 녹화 제어 메시지
* 별도 control topic을 subscribe하여 녹화 시작/종료를 제어한다. 
  * `control topic`의 이름은 파라미터로 지정됨.
* 이를 위해 새로운 control 메시지를 정의함.
  * 새 메시지는 recorder의 패키지에서 구현됨
  * 새 메시지는 header와 `command`라는 문자열로 정의됨
* 녹화 시작과 종료 메시지는 각각 `command` 필드 값이 `start`와 `stop`으로 정의됨
  * 각 메시지 header에 정의된 stamp를 각각 `start_ts`와 `stop_ts`라 한다.
  * `start`와 `stop` 이외의 control 메시지는 무시한다.

#### 7.2 녹화 구간
* `control topic`에서 `start` 메시지가 도착하면 **녹화 구간** 상태가 활성화되고,
  이 후 `stop` 메시지가 도착할 때까지 유지된다. `stop` 메시지 도착 이후에는
  **비 녹화 구간** 상태가 된다.
  * 참고로 Recorder Node가 시작되면 바로 비 녹화 구간 상태가 된다.
  * 만일 `stop` 메시지가 도착한 이후에 다시 `start` 메시지가 도착하면 또 새로운 녹화 구간이 시작된다.
* 반열림 구간 `[start_ts, stop_ts)`으로 녹화 구간이 정의된다. 하나의 녹화 구간은
  **녹화 세션**이라 부르기로 한다.
  * 이미지 메시지의 frame_ts가 이 구간에 포함되면 녹화 대상이 되고, 그렇지 않은 이미지들은
    녹화에 포함되지 않아야 한다.
* 녹화 구간(세션)에 포함된 이미지들로 하나의 mp4 파일을 구성한다.
* control topic으로 발생되는 start와 stop는 pair를 이룬다고 가정한다.
  * start -> start 또는 stop -> stop와 같은 메시지는 토픽으로 발행되지 않는다.
  * 만일 이러한 순서의 메시지가 발행되면 warning 메시지를 출력하고,
    메시지를 무시한다.

#### 7.3 out-of-order 메시지 처리
* 녹화 구간이 Recorder Node에 도착하는 메시지 순서가 아니라
  메시지 header에 포함된 stamp를 사용하기 때문에 다음의 고려사항이 발생
  * control topic에 'start' 메시지가 도착할 때 'start_ts'보다 큰 frame_ts를
    갖는 이미지 메시지가 이미 도착하는 경우가 발생
  * control topic에 'stop' 메시지가 도착하기 이전에 이미 'stop_ts'보다 크거나 같은
    frame_ts를 갖는 이미지가 이미 도착한 경우 발생
* 이 상태를 해결하기 위해 내부적으로 `pending_image_queue`라는 FIFO로 관리되는
  queue를 사용한다.
  * 이미지 메시지가 도착하면 바로 녹화하거나 버리지 말고 `pending_image_queue`에 넣고,
    queue의 overflow 발생으로 가장 오래된 메시지로 선택된 msg의 header stamp (`frame_ts`)를
    이용하여 다음과 같은 동작을 수행한다.
    * 녹화 구간인 경우: frame_ts >= start_ts 이면 녹화 대상으로
      간주하여 worker queue에 삽입. 그렇지 않은 경우는 drop.
    * 비 녹화 구간인 경우: frame_ts < stop_ts이면 녹화 대상으로 
      간주하여 worker queue에 삽입. 그렇지 않은 경우는 drop.
      Recorder Node가 시작된 직후는 비 녹화 구간이지만 `stop_ts`가 존재하지 않기 때문에
      이때는 stop_ts를 0/0 으로 간주한다.
  * `start` 메시지가 도착하는 경우에는 `pending_image_queue`에 포함된 이미지 메시지에 대해
    시간 순서대로 이미지 msg를 iteration 하여 frame_ts < start_ts 인 메시지를
    모두 drop 시킨다.
  * `stop` 메시지가 도착하는 경우에는 `pending_image_queue`에 포함된 이미지 메시지에 대해
    시간 순서대로 이미지 msg를 iteration 하여 frame_ts < stop_ts인 메시지를 worker_queue에
    삽입하고 나머지는 queue에 계속 유지시킨다.
  * Recorder node가 종료하는 경우는 `pending_image_queue`에 포함된 각 msg에 대해
    녹화/비녹화 구간 여부에 따라 위 작업을 반복함.
* 이 queue의 길이는 `pending_queue_length` 이름의 파라미터를 통해 결정된다.
  * `pending_queue_length`는 프레임 갯수로 표현됨
  * Queue의 길이가 짧으면 비 녹화 단계에서 `start`가 도착할 때, 충분한 수의 이미지가
    queue에 있지 않아 start_ts보다 크거나 같은 frame_ts를 갖은 이미지 queue에 overflow로
    drop되어 녹화에서 누락되거나, 녹화 단계에서 `stop` 메시지가 도착할 때 stop_ts보다 크거나 같은 메시지가 `pending_image_queue`에서 overflow로 이미 queue에서 나와 녹화되는 현상이 발생할 가능성이 있다. 이 경우는 warning 메시지를 출력한다.

#### 7.4 녹화 메시지 처리
* 앞서의 방식으로 녹화 대상이 되는 이미지는 EncoderBackend에게 제공되어 mp4 파일에 기록된다.
  * 9절에서 언급된 worker queue에 기록되고, worker queue에 기록된 이미지는
    worker thread에 의해 ffmpeg 서브 프로세스에게 제공된다.
* 주의할 점은 `pending_image_queue`는 녹화 제어 메시지와 이미지 메시지 사이의 순서가
  어긋나는 문제를 위해 사용되고, `worker queue`는 ffmpeg 서브 프로세스와의
  연동하는 과정에서 메시지 처리 모듈이 blocking되는 현상은 회피하기 위한 목적이다.
  * `pending_image_queue`를 통해 어떤 이미지 메시지를 녹화 대상으로 삼을지를 결정하고,
    녹화 대상 메시지는 `worker queue`에 삽입하여 worker thread로 하여금 ffmpeg 서브 프로세스에게 전달한다.

### 8. 파일 저장 요구사항

* 하나의 세션에는 3개의 파일이 생성된다. 참고로 세션의 시작과 끝은 control 메시지의
  'start'/'stop'에 의해 결정됨
  * mp4 영상 파일
  * 세션 메타 데이터 파일 (metadata.json)
  * mp4 영상을 다시 Image 형태의 메시지로 복원할 때 활용할 sidecar 파일 (jsonl 형태)
* 세션 메타 데이터 (metadata.json) 파일에는 다음과 같은 정보가 기록됨
  * resolution: 영상 해상도 (width x height)
  * fps: int형
  * nframe: 총 프레임 수
  * encoding / is_big_endian
    * CompressedImage인 경우 내부적으로 decoding한 이미지의 정보를 사용 
  * start_ts: 첫번째 프레임의 timestamp (sec/nsec 형식)
  * end_ts: 마지막 프레임의 timestamp (sec/nsec 형식)
* 세션에 포함된 3개의 파일은 고유해야 한다.
* 권장 형식:

  * `<base_dir>/<recording_id>_<start_ts>.mp4`
  * `<base_dir>/<recording_id>_<start_ts>.jsonl`
  * `<base_dir>/<recording_id>_<start_ts>_metadata.json`
* <base_dir>은 파라미터로 지정된다. 만일 별도로 지정되지 않는 경우에는
  현재 작업 디렉토리로 간주한다.
* `<recording_id>`는 Node 가 시작될 때 파라미터로 제공됨.
  * 동일한 '<recording_id>'를 갖는 Node는 없다고 가정함.
  * 하나의 Node는 한순간에 최대 1개의 세션만 유지.
  * 이를 통해 생성된 3가지 파일 이름은 <base_dir> 하에서는 고유하다
* 만일 동일 stamp를 갖는 start 메시지가 도착하지 않는다면 각 세션의 파일 이름이 중복되지 않는다.

### 9. 버퍼링/동시성 요구사항

* ROS callback이 decode/encode 처리로 block되지 않도록 별도 worker thread와
  bounded queue를 사용한다.
  * queue의 길이는 파라미터 (`worker_queue_length`)를 통해 지정한다.
  * overflow가 발생되면 drop-oldest 정책을 사용함.

### 10. 종료 및 오류 처리 요구사항
* 노드 종료 시 active recording은 자동 종료한다. 
* ffmpeg 비정상 종료, 디스크 쓰기 실패, 미지원 포맷, invalid timestamp, queue overflow 등 주요 오류에 대해 복구 가능한 범위에서 안전하게 처리하고 로그를 남긴다.
* 세션 비정상 종료 시에도 가능한 범위에서 sidecar와 mp4를 일관성 있게 마무리한다.
  * 상황에 따라 비정상 종료 당시의 일부 이미지는 누락될 수는 있지만, 저장되는 이미지와
    sidecar의 데이터는 서로 일치해야 한다.

### 11. 파라미터 목록 정리
* image_topic (str): 입력 이미지 토픽
* control_topic (str): 녹화 제어 토픽
* recording_id (str): 세션 파일명 prefix
* base_dir (str): 최상위 저장 디렉토리 경로명
* fps (int): 입력 이미지 fps 및 복원 fps
* encoder_mode (str): auto / cpu / gpu
* pending_queue_length (int): out-of-order 보정용 queue 길이 (프레임)
* worker_queue_length (int): worker thread 앞단 queue 길이

### 12. 기타 요구사항
* 개발될 ROS2 Node의 클래스 이름은 MP4RecorderNode 로 한다.
* 실제 구현 단계에서는 3절에 따라 ROS2와 연관성이 먼 부분 먼저 개발하여 별도로 동작되는지 확인한다.
  * 특히 RecorderCore 부분과 EncoderBackend 부분을 먼저 구현하여 ROS2가 아닌 환경에서도 동작하는 것을 확인해 본다.
* RecorderCore 부분을 개발할 때는 다음을 고려한다.
  * MP4Recorder 클래스가 가장 외부의 인터페이스로 이 클래스가 제공하는 인터페이스를 통해 녹화 세션의 시작/종료 등을 제어할 수 있다.