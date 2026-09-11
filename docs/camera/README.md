# 카메라 서브시스템 — 어느 노드를 쓸까

카메라 노드가 **셋**이다. 이름이 비슷해 고르기 어려우므로 여기서 먼저 가른다.
셋 다 하드웨어 추상화는 `robot_control.camera.opencv_camera.OpenCvCamera` 하나를 쓴다.

| | [`camera_node`](camera_node_guide.md) | [`image_capture_node`](image_capture_node_guide.md) | [`rdfp_camera_node`](rdfp_camera_node_guide.md) |
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

## 세션 토픽은 `/session` 이다 — remap 이 필요 없다

`session_control` 은 토픽을 **루트 상대**(`session`)로 발행하므로 노드 이름이 붙지
않는다. `rdfp_camera_node` 도 같은 이름으로 구독하니 **그대로 맞는다.**
`/session_control/session` 은 namespace 를 줬을 때만 생기는 경로다
([session_control_guide](../session/session_control_guide.md)).

## 그 밖의 문서

| 문서 | 내용 |
|---|---|
| [opencv_camera_guide.md](opencv_camera_guide.md) | 공용 하드웨어 추상화. 장치 인덱스·파일·RTSP, 파일 끝 되감기 |
| [image_viewer_node_guide.md](image_viewer_node_guide.md) | 뷰어 (제어 계층) |
| [rdfp_image_viewer_node_guide.md](rdfp_image_viewer_node_guide.md) | 뷰어 + 세션 상태 오버레이 (수집 계층) |
| [initial_requirements.md](initial_requirements.md) | 최초 요구사항 (이력) |
