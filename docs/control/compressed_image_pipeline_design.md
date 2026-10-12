# 압축 이미지 파이프라인 설계 — camera · image_viewer · image_recorder

시뮬레이터가 이미지를 **압축(JPEG)으로** 발행할 때, 누가 그것을 풀고 누가 그대로 받는가를
정한다. 2026-09-09 결정.

계기는 펑션베이다. 2026-09-08 새 빌드부터 `/camera_image`(`Image`, bgr8) 대신
`/camera_image/compressed`(`CompressedImage`, jpeg)로 발행한다. 대역폭이 35배 줄었지만
(8.9 → 0.25 MB/s, §2 실측) **소비자 배선이 갈렸다** — 뷰어는 `sensor_msgs/Image` 만 구독하고,
ROS 2 는 타입이 다르면 **연결만 하지 않고 오류를 내지 않으므로** 증상이 "빈 창" 뿐이었다.

---

## 1. 결정 요약

| 소비자 | 무엇을 구독하나 | 왜 |
|---|---|---|
| `image_viewer_node` · `rdfp_image_viewer_node` | **raw** (`camera/image_raw`) | 픽셀이 필요하다. 디코드는 피할 수 없고, **어디서 하느냐만** 문제다 |
| `rdfp_image_recorder` (세션 구동) | **압축 그대로** | 디코드하지 않는다 — JPEG 바이트를 ffmpeg 에 넘긴다 |
| `image_recorder_node` (서비스 구동) | raw | 압축 경로가 아직 없다 (§7) |
| rosbag2 | **압축 그대로** | 바이트를 그대로 담는다 |

**가르는 기준은 하나다.**

> **픽셀이 필요한 소비자는 raw 를 구독한다. 바이트를 그대로 넘길 수 있는 소비자만 압축을
> 직접 받는다.**

레코더가 예외인 것은 특혜가 아니라 **레코더가 소비자가 아니라 트랜스코더**이기 때문이다.
어차피 ffmpeg 이 인코딩하므로 JPEG 을 그대로 넘기면 디코드 한 번과 raw 토픽 하나가 통째로
사라진다. 뷰어는 화면에 그려야 하므로 픽셀이 없으면 아무것도 못 한다.

---

## 2. 뷰어가 직접 디코드하지 않는 이유

두 안을 놓고 골랐다.

**(A) 뷰어가 `CompressedImage` 를 구독해 `cv2.imdecode` 로 푼다.**
**(B) 디코드 전용 노드 하나가 raw 를 발행하고, 뷰어는 지금처럼 raw 만 구독한다.** ← 채택

### 성능은 결정 근거가 아니다

| 항목 | 실측 |
|---|---|
| `cv2.imdecode`, 640×480 / 26.8 KB | **0.75 ms/frame** — 10 Hz 에서 코어의 0.75 % |
| GIL | **놓는다** (디코드 스레드를 붙여도 파이썬 루프가 101 % 유지) |
| 디코더 구현 | `image_transport/republish` 는 C++ ELF, `cv2.imdecode` 도 네이티브 libjpeg-turbo |

**둘 다 같은 네이티브 코드를 부른다.** 파이썬이라서 느린 것이 아니다. 그러니 성능으로는
못 고른다 — 소비자가 여럿일 때 (A) 는 N 번 디코드하고 (B) 는 한 번 디코드하는 차이만
남는데, 0.75 ms 에서 그 차이는 무의미하다.

### 결정 근거는 **소비자가 백엔드를 몰라야 한다**는 것이다

백엔드마다 raw 의 출처가 다르다.

| 백엔드 | 시뮬레이터/장치가 내는 것 |
|---|---|
| mock | 없다 — OpenCV `camera_node` 가 만든다 |
| Isaac · Gazebo | **raw 를 직접** 낸다 |
| 펑션베이 | **압축만** 낸다 |

(A) 를 택하면 뷰어가 "이 스택은 압축이냐 raw 냐"를 알아야 한다. 그러면 **제어 계층 노드가
백엔드를 아는 것**이 되어 계층 규칙이 깨지고, 같은 분기를 `ImageViewerNode` ·
그것을 상속하는 `RdfpImageViewerNode` · `image_recorder_node` 에 각각 써야 한다. 새 raw
소비자를 붙일 때마다 또 쓴다.

(B) 는 [토픽 이름 규약](../topic_naming_contract.md) §4 와 같은 형태다 —
**백엔드 경계에서 정규 이름으로 맞추고, 상위는 정규 이름만 안다.**

```
[백엔드가 주는 것]        ──백엔드 launch──▶  [정규 채널]      ──▶  상위 계층
 camera_image/compressed      republish        camera/image_raw      뷰어·레코더
 isaac/camera/image_raw       remap 뿐          camera/image_raw
 (장치)                       camera_node       camera/image_raw
```

이 표의 왼쪽이 무엇이든 오른쪽은 같다. 그것이 이 설계의 전부다.

### 치르는 비용 — 대역폭

raw 를 만드는 순간 압축으로 아낀 것이 돌아온다.

실측 (2026-09-09, 펑션베이 · 640×480 · 같은 스트림을 두 토픽에서 15초 동시 측정):

| 형식 | 프레임 | 주기 | 대역폭 | 배수 |
|---|---|---|---|---|
| JPEG (`CompressedImage`) | 25.4 KB | 9.72 Hz | **0.253 MB/s** | 1× |
| bgr8 (`Image`) | 900.0 KB | 9.66 Hz | **8.899 MB/s** | **35.2×** |

그래서 **raw 발행자는 필요할 때만 뜬다** (§4). 소비자가 하나도 없으면 압축만 흐른다.

---

## 3. `camera.source` — raw 의 출처는 프로파일이 정한다

백엔드 프로파일([config/backends/](../../src/robot_control/config/backends/README.md))의
`camera` 블록에 `source` 를 둔다. **어느 스택이 무엇인가의 정본이 프로파일**이라는 기존
원칙 그대로다.

| `camera.source` | 뜻 | raw 발행자 | 쓰는 백엔드 |
|---|---|---|---|
| `device` | 장치/파일에서 우리가 캡처한다 | `camera_node` (OpenCV) | `mock` · `mock_jgpc` |
| `compressed` | 시뮬레이터가 **압축만** 준다 | `image_transport/republish` | `functionbay` |
| `native` | 시뮬레이터가 **raw 를 직접** 준다 | 없다 (시뮬레이터 자신) | `isaac` |

`compressed` 일 때만 `camera.compressed_topic` 이 필요하다. `image_topic` 은 셋 다
필요하다 — 그것이 정규 채널의 이름이기 때문이다.

> **`source` 를 값으로 두는 이유**는 키 유무로 추론할 수 없어서다. `device` 와 `native` 는
> 둘 다 `image_topic` 만 갖는다. 추론하면 mock 에 `compressed_topic` 을 실수로 넣었을 때
> 조용히 다른 노드가 뜬다.

---

## 4. `enable_camera` — 스위치는 하나다

`enable_camera_node` 를 **`enable_camera` 로 바꾼다.** 이름이 노드가 아니라 **역할**을
가리켜야 한다 — 같은 스위치가 백엔드에 따라 `camera_node` 를 띄우기도, `republish` 를
띄우기도, 아무것도 안 띄우기도 하기 때문이다.

### 두 이름을 두면 안 되는 이유

`enable_camera`(디코드)와 `enable_camera_node`(장치)를 나란히 두면 **둘 다 켰을 때 같은
토픽에 발행자가 둘**이 된다. 오류가 아니라 프레임이 섞여 나오고, 원인이 안 보인다. 하나로
합치면 그 조합이 **만들어질 수 없다.**

장치 카메라를 시뮬레이터 스택에 함께 붙여야 하면 `camera_node` 를 손으로 띄우고 토픽을
따로 준다 — 드문 경우를 위해 상시 함정을 남기지 않는다.

### 기본값이 소비자에서 파생된다

`enable_camera` 를 손으로 켜야 하면 "뷰어를 켰는데 빈 창"이 반복된다. 그래서 **기본값이
그 launch 의 raw 소비자에서 파생된다.**

```python
# 제어 계열 — raw 소비자는 뷰어뿐이다
DeclareLaunchArgument('enable_camera', default_value=LaunchConfiguration('enable_image_viewer'))

# 수집 계열 — 뷰어 + rawvideo 로 돌리는 레코더
DeclareLaunchArgument('enable_camera', default_value=PythonExpression([...]))
```

| 상황 | `enable_camera` |
|---|---|
| 뷰어 on | **자동 on** |
| 레코더 `input_format:=rawvideo` | **자동 on** |
| 둘 다 off | off — 압축만 흐른다 |
| `enable_camera:=true` 명시 | on (rosbag 에 raw 를 담거나 rviz 를 붙일 때) |

`enable_camera:=false` 로 **덮어써서** 끄면 뷰어가 빈 창이 된다. 그 조합은 launch 가
`LogWarn` 으로 알린다 — 조용히 실패하는 것만 막고, 막지는 않는다(의도적으로 끄는 경우가
있다).

---

## 5. 레코더 — 압축이 기본이다

`rdfp_image_recorder` 는 `input_format` 파라미터로 갈린다.

| `input_format` | 구독 | 경로 |
|---|---|---|
| `mjpeg` (**압축 백엔드의 기본**) | `CompressedImage` | JPEG → ffmpeg `-f mjpeg` → H.264 |
| `rawvideo` | `Image` | 픽셀 → ffmpeg `-f rawvideo` → H.264 |

`mjpeg` 가 기본인 이유는 **디코드를 우리가 안 하기 때문**만이 아니다. 그 경로는 raw 토픽
자체를 필요로 하지 않으므로, **헤드리스 수집에서 8.9 MB/s 가 아예 안 생긴다.**

출력은 어느 쪽이든 H.264 라 데이터셋·`Mp4ImageReplayer` 에는 영향이 없다.

> ⚠️ 방향이 어긋난 호출(`write_bytes()` on mjpeg / `write_compressed()` on raw)은
> `InvalidFrameError` 로 거부한다. 조용히 통과시키면 JPEG 이 raw 픽셀로 해석돼 깨진 영상이
> 나오고 원인을 못 찾는다.

### 예전에 여기서 깨졌던 것

`camera_republish` 가 **`enable_image_viewer` 조건으로** 떴다. 서비스 구동
`image_recorder_node` 는 raw 만 받으므로, 헤드리스로 돌리려고 뷰어를 끄면 **raw 발행자가
사라져 레코더가 조용히 0 프레임을 담았다.** 오류도 경고도 없었다.

이 설계는 그 결합을 끊는다 — 디코더는 `enable_image_viewer` 가 아니라 `enable_camera` 가
켜고, `enable_camera` 는 **모든** raw 소비자에서 파생된다.

---

## 6. 토픽 이름 — `~/image_raw` 를 쓰지 않는다

정규 이름은 **`camera/image_raw`** 다 (루트 상대). [규약](../topic_naming_contract.md)
§2.1 이 정한 값이며, 이 설계에서 바꾸지 않는다.

`~/image_raw` 로 두자는 안이 있었지만 **쓸 수 없다.** `~/` 는 네임스페이스가 아니라 **노드
이름**을 붙이므로, 같은 논리 채널이 구현에 따라 이름이 달라진다.

```
source: device      →  /camera_node/image_raw
source: compressed  →  /camera_republish/image_raw     ← 같은 채널인데 이름이 다르다
```

이것은 규약 §2.2 가 기록한 그리퍼 사고와 **같은 실패**다 — `gripper_control_node` 를
`GripperActionNode` 로 갈아치웠더니 `/gripper_control/gripper_cmds` 가 따라 바뀌어 녹화
목록과 트윈 설정이 함께 깨졌다. 여기서는 노드를 갈아치우는 것도 아니고 **백엔드를 바꾸기만
해도** 이름이 바뀐다.

루트 상대(`camera/image_raw`)면 백엔드 launch 의 remap 도, §5 의 로봇별 네임스페이스도
모두 통한다.

---

## 7. 적용 범위와 남은 것

### 지금 적용되는 곳

`camera.source` 가 `compressed` 인 백엔드 — 현재 **펑션베이 하나**다. `device`/`native` 는
동작이 예전과 같고, 바뀌는 것은 `enable_camera_node` → `enable_camera` 라는 이름뿐이다.

### 남은 것

| 항목 | 왜 지금 안 하나 |
|---|---|
| `image_recorder_node`(서비스 구동)에 `input_format` 추가 | 그 노드는 **첫 이미지로 해상도를 정하는** 구조라 JPEG 헤더를 읽어야 한다. 압축 백엔드는 세션 구동 레코더를 쓰므로 지금 막히는 곳이 없다 |
| `camera_info` | 시뮬레이터가 주지 않는다. `device` 경로에만 있다 |
| 해상도·fps 를 프로파일로 | 펑션베이는 Unity 씬 안에 있어 **우리가 읽을 수 없다.** 못 읽는 값을 프로파일에 적으면 실측과 갈려도 아무도 모른다 — 실측값임이 드러나게 launch 상수로 둔다 |

### 새 압축 백엔드를 붙일 때

프로파일에 세 줄이면 끝난다. launch 는 고치지 않는다.

```yaml
camera:
  source: compressed
  compressed_topic: /<시뮬레이터가 내는 이름>
  image_topic: /camera/image_raw
```

---

## 8. 함정

- **remap 대상은 `in/compressed` 다 — `in` 이 아니다.** `image_transport` 의 구독
  플러그인은 base 토픽에 transport 접미사를 붙인 **완성된 이름**으로 구독하므로 `in` 만
  remap 하면 한 장도 못 받는다. 오류도 없다.
- **include 된 launch 는 부모 argument 를 물려받는다.** 자기 `DeclareLaunchArgument` 의
  기본값으로 되돌아가지 않으므로, 수집 계열은 백엔드에 **`enable_image_viewer:=false` 와
  `enable_camera:=false` 를 둘 다** 명시적으로 넘긴다. 앞을 빠뜨리면 창이 두 개 뜨고,
  뒤를 빠뜨리면 **`republish` 가 둘 떠 같은 토픽에 발행한다** — 프레임이 섞이고 오류는
  없다. raw 는 수집 계층이 만든다.
- **타입이 다르면 ROS 2 는 연결만 안 하고 오류를 내지 않는다.** 이 문서의 모든 "조용히"는
  그 성질에서 온다. 배선을 의심할 때는 `ros2 topic info -v <토픽>` 으로 **양끝의 타입과
  개수**를 먼저 본다.
