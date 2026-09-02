# Isaac Sim + 로봇 스택 기동 절차

시뮬레이터를 띄우고 ROS 스택을 붙여 **조작·수집이 가능한 상태**까지 가는 순서.

설계 배경은 [isaac_backend_skeleton.md](isaac_backend_skeleton.md) 에 있다 — 여기는
**무엇을 어떤 순서로 실행하는가**만 다룬다.

---

## 1. 어느 모드로 띄울까

```bash
./scripts/run_isaac_sim.sh --gui        # 창 + 전 과정 자동  ← 평소 이것
./scripts/run_isaac_sim.sh --headless   # 창 없이 전 과정 자동 (검사·CI)
./scripts/run_isaac_sim.sh              # 전체 편집기 — 스테이지는 손으로 구성
```

| 모드 | 창 | Script Editor | 쓸 때 |
|---|:-:|:-:|---|
| `--gui` | ✅ | **불필요** | 로봇이 움직이는 것을 보면서 작업 |
| `--headless` | ❌ | 불필요 | 수용 기준 검사, 데이터 수집, CI |
| (인자 없음) | ✅ | **필요** | 자산 브라우저·프로퍼티 편집 등 **편집기가 필요할 때만** |

**세 모드가 같은 스크립트 목록을 돈다.** 차이는 누가 실행하느냐뿐이다.

`--phase 3` 을 붙이면 카메라를 빼고 조작 계열만 만든다.

> **`~/isaac_ros2_env.sh` 를 소싱하지 않는다.** `/opt/ros/humble` 이 Isaac 의 내장
> ROS(파이썬 3.12)를 가려 **ROS 확장이 조용히 죽는다.** 증상은 `/isaac_sim_control`
> 노드가 없는 것뿐이다. `run_isaac_sim.sh` 가 환경을 대신 맞춘다.

---

## 2. 절차

### 터미널 1 — 시뮬레이터

```bash
cd ~/development/ros/rdfp_ws
./scripts/run_isaac_sim.sh --gui
```

`[bringup] PLAY` 가 찍히면 준비된 것이다.

> **첫 실행은 로봇이 몇 초 늦게 나타난다.** 자산이 원격(S3)이라 내려받는 동안 뷰포트가
> 비어 있는데, 실패처럼 보인다.

### 터미널 2 — ROS 스택

```bash
cd ~/development/ros/rdfp_ws
source install/setup.bash
export ROS_DOMAIN_ID=31
ros2 launch robot_control panda_isaac.launch.py
```

**기본값이 조작 작업에 맞춰져 있어 인자를 붙일 일이 드물다.**

| 인자 | 기본 | |
|---|:-:|---|
| `enable_gripper` | **`true`** | `gripper_action_bridge` + `gripper_action_node` |
| `enable_scene` | **`true`** | `isaac_scene_state_node` → `/scene/objects` · `/scene/reset` |
| `enable_servo` | **`true`** | `servo_node` + 브리지 — teleop 두 경로가 여기로 수렴한다 |
| `enable_image_viewer` | **`true`** | 카메라 이미지 창 (`image_viewer_node`) |
| `enable_rviz` | **`false`** | Isaac 이 이미 뷰포트를 그린다. 계획 결과를 보려면 켠다 |

⚠️ **화면이 없는 곳에서는 뷰어를 끈다.**

```bash
ros2 launch robot_control panda_isaac.launch.py enable_image_viewer:=false
```

켠 채로 원격 셸에서 돌리면 `image_viewer_node` 가 `exit code -6` 으로 죽는다. 다만
**나머지 스택은 그대로 산다** — launch 가 함께 내려가지는 않으므로 로그의 그 한 줄만
무시하면 계속 쓸 수 있다. RViz 의 Image 디스플레이나
`ros2 run rqt_image_view rqt_image_view` 를 대신 쓸 수도 있다.

> **수집 계열은 자기 뷰어가 따로 있다.** `rdfp_panda_isaac.launch.py` 의
> `enable_image_viewer_node:=true` 는 `rdfp_image_viewer_node` 를 띄우는데, 그쪽은
> 프레임에 **`/session` 상태를 겹쳐 그린다** — 에피소드 경계를 눈으로 확인하기 위한
> 것이다. 제어 계열의 것은 그냥 보기 위한 것이다.

**도메인은 31 이다.** `setup_graph.py` 가 그 값을 그래프에 박으므로, 다르면 브리지
토픽이 다른 도메인으로 나가 **에러 없이 아무것도 안 보인다.**

### 터미널 3 (선택) — 로봇 트윈

```bash
ros2 run robot_twin robot_twin --config src/robot_twin/config/robot_twin_panda_isaac.yaml
```

`http://localhost:8802/api/v1/robot_twins/panda_isaac` 로 REST 조작이 열린다.

---

## 3. Script Editor 목록 — 인자 없이 띄웠을 때만

`--gui` / `--headless` 는 아래를 대신 돌린다. 전체 편집기로 띄웠다면 **Window ▸ Script
Editor** 에서 이것을 붙여넣는다.

```python
path = "/home/kwlee/development/ros/rdfp_ws"
for _s in ("load_robot", "setup_scene", "setup_graph", "place_robot",
           "set_home_pose", "tune_drive", "tune_grasp"):
    exec(open(path + "/scripts/isaac/sim_side/%s.py" % _s, encoding="utf-8").read())
```

그다음 툴바에서 **Stop → Play**.

같은 텍스트를 `./scripts/run_isaac_sim.sh` 가 터미널에도 출력하므로 외울 필요는 없다.

### 각 스크립트가 하는 일

| 순서 | 스크립트 | 하는 일 | **빠뜨리면** |
|:-:|---|---|---|
| 1 | `load_robot` | Franka 를 `/World/franka` 에 올린다 | `setup_graph` 가 `no articulation root` 로 멈춘다 |
| 2 | `setup_scene` | 테이블 · 블록 3개 · 카메라 prim · **돔 라이트** | scene TF 가 없어 `/scene/objects` 가 빈다 |
| 3 | `setup_graph` | OmniGraph — clock · joint_states · 팔·그리퍼 명령 · scene TF · 카메라 | **ROS 로 아무것도 안 나간다** |
| 4 | `place_robot` | `panda_link0` 을 월드 원점으로 | 그리퍼가 "도달했다"면서 **허공을 쥔다** (§6) |
| 5 | `set_home_pose` | Play 시작 자세를 `ready` 로 | 접힌 자세로 시작해 첫 계획이 막힌다 |
| 6 | `tune_drive` | 팔 관절 drive 게인 | Phase 1 시정수 τ 가 **177 ms** (기준 50) |
| 7 | `tune_grasp` | 손가락 drive · 손끝 마찰 | Phase 6 에서 **블록이 손가락 사이로 미끄러진다** |

**순서가 의미를 갖는다.** `load_robot` 이 먼저여야 그래프가 로봇을 찾고, `setup_scene`
이 `setup_graph` 보다 앞서야 TF 대상 prim 이 존재한다.

**전부 멱등이다** — 여러 번 돌려도 같은 상태가 된다. `load_robot` 은 이미 로봇이 있으면
아무것도 하지 않아 튜닝·자세를 덮어쓰지 않는다.

> **`tune_drive` / `tune_grasp` 가 특히 빠뜨리기 쉽다.** 없으면 검사가 "시뮬레이터가
> 이상하다"처럼 보이지 실제 원인(**기본 자산이 튜닝되지 않았다**)을 가리키지 않는다.

### 그 밖의 sim_side 스크립트 (필요할 때만)

| 스크립트 | 하는 일 |
|---|---|
| `check_colliders.py` | 충돌·배치 진단 (읽기 전용) |
| `check_bridge.py` | Isaac **안에서** ROS 토픽이 보이는지 — 브리지 문제인지 네트워크 문제인지 가른다 |
| `verify_saved_scene.py` | 저장된 scene 검증 |
| `headless_bringup.py` | 위 일곱을 순서대로 도는 진입점 (`--gui`/`--headless` 가 쓴다) |

---

## 4. 확인

```bash
source install/setup.bash && export ROS_DOMAIN_ID=31

python3 scripts/isaac/is_topics.py          # 배관 진단 — 먼저 이것
python3 scripts/isaac/is_check_phase0.py    # 골격          5/5
python3 scripts/isaac/is_check_phase1.py    # 팔 명령       4/5 (τ 는 기준 초과)
python3 scripts/isaac/is_check_phase2.py    # 그리퍼        6/6
python3 scripts/isaac/is_check_phase3.py    # scene 객체    5/5
python3 scripts/isaac/is_check_phase4.py    # 카메라        6/6
python3 scripts/isaac/is_check_phase6.py    # 물리 파지     6/6
```

빠른 확인만 필요하면:

```bash
ros2 topic hz /clock                        # 약 60 Hz (발행 주기다 — 배속은 phase0 이 잰다)
ros2 topic hz /isaac/camera/image_raw       # 약 5 Hz
ros2 topic echo /gripper_states --once      # width · stalled · at_goal
ros2 run tf2_ros tf2_echo panda_link0 block_a
```

> ⚠️ **`kill_stack.sh` 직후에는 `ros2` CLI 가 빈 결과를 준다.** 데몬을 함께 지우므로
> 캐시가 비어 있어서다 — `ros2 node list` / `topic echo` 가 아무것도 못 찾으면 몇 초
> 뒤 다시 하거나, 확실히 하려면 직접 구독하는 스크립트로 본다. **노드가 죽은 것이
> 아니다.**

---

## 5. 정리

```bash
./scripts/kill_stack.sh --dry-run   # 무엇을 죽일지 먼저 본다
./scripts/kill_stack.sh             # Isaac + 스택 + 고아 노드 + ros2 데몬
./scripts/kill_stack.sh --ros       # 시뮬레이터는 두고 스택만
```

---

## 6. 자주 물리는 함정

### 시뮬레이터를 재시작하면 **ROS 스택도 다시 띄운다**

Isaac 을 다시 띄우면 sim 시계가 0 으로 돌아가는데, 살아남은 스택의 TF 버퍼에는 **더 큰
타임스탬프의 옛 데이터**가 남아 새 데이터가 버려진다.

```
Warning: TF_OLD_DATA ignoring data from the past for frame panda_link1 at time 7.31
```

경고로만 끝나지 않는다 — `/scene/objects` 가 **직전 실행에서 물체를 놓았던 자리**를
계속 말하고, 그것을 믿은 파지가 빗나간다(실측 10 cm). `tf2_echo` 로 TF 를 직접 보면
갈린다.

**조치:** `./scripts/kill_stack.sh --ros` 후 스택만 다시 띄운다.

### `ros2 launch` 를 죽여도 **자식 노드는 산다**

터미널에서 Ctrl-C 로 끝내면 정리되지만, 프로세스를 `kill` 하거나 세션이 끊기면
`move_group` · `isaac_scene_state` 등이 부모 없이 계속 돈다. 스택을 다시 띄우면 노드가
둘씩 되고, **액션 서버가 둘이면 클라이언트가 엉뚱한 쪽 응답을 받는다.**

```bash
ros2 node list | sort | uniq -c    # 2 이상인 것이 있으면 고아다
```

### 스테이지를 저장해 두지 않는다

`.usd` 로 저장하면 매번 짓지 않아도 되지만, 물체 이름·크기가 USD 와
`isaac_scene.json` **두 곳에** 살게 된다. JSON 은 `isaac_scene_state_node` 도 읽는
정본이라, 갈라지면 시뮬레이터와 ROS 가 다른 크기를 믿는다 — **에러 없이.** 매번
스크립트로 짓는 편이 느려도 어긋나지 않는다.

### 화면이 검다 / 카메라 이미지가 새까맣다

**씬에 조명이 없으면 그렇다.** 전체 편집기로 띄우면 기본 스테이지에
`/Environment/defaultLight` 가 딸려 오지만 `--gui`/`--headless` 는 **빈 스테이지**에서
시작하므로 조명을 직접 만들어야 한다. `setup_scene` 이 돔 라이트를 만든다.

Stage 탭에 물체는 보이는데 뷰포트만 검다면 이 경우다. `ros2 topic echo` 로는 드러나지
않으니 픽셀을 봐야 한다:

```bash
python3 scripts/isaac/is_check_phase4.py   # 3. 내용 항목이 잡는다
```

> **이 검사가 없던 동안 새까만 이미지가 5/5 로 통과했다** (2026-09-02). 주파수·해상도·
> 인코딩·스탬프가 전부 맞았기 때문이다 — 배관은 멀쩡했고 조명만 없었다. 그 데이터로
> 학습하면 아무 신호도 없는 영상이 쌓이고, 열어보기 전까지 드러나지 않는다.

### 화면이 하얗게 날아간다

`DomeLight` 는 **배경 자체를 칠한다.** 세기를 올리면 물체가 밝아지는 게 아니라 배경이
백색이 된다. 주광은 `DistantLight`(배경을 칠하지 않는다)로 두고 돔은 그림자 안쪽을
채울 만큼만 낮게 둔다 — `setup_scene.py` 의 `DISTANT_LIGHT_INTENSITY` /
`DOME_LIGHT_INTENSITY`.

검사의 「내용」 항목이 **표준편차**도 보므로 하얗게 날아간 평면도 잡힌다.

### 카메라가 엉뚱한 곳을 본다

**검사로는 못 잡는다.** 노출과 편차가 정상이어도 프레임에 작업 영역이 없을 수 있다 —
실제로 그랬다(시선이 테이블을 지나쳐 빈 바닥을 봤고, 그다음엔 화면이 90° 돌아갔다).
**눈으로 한 번 봐야 한다:**

```bash
python3 - <<'EOF'
import rclpy, numpy as np, cv2
from sensor_msgs.msg import Image
rclpy.init(); n = rclpy.create_node('peek'); got = []
n.create_subscription(Image, '/isaac/camera/image_raw', lambda m: got.append(m), 5)
while not got: rclpy.spin_once(n, timeout_sec=0.2)
m = got[-1]
a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
cv2.imwrite('/tmp/isaac_cam.png', cv2.cvtColor(a, cv2.COLOR_RGB2BGR))
print('/tmp/isaac_cam.png')
EOF
```

자세를 고칠 때는 **look-at 으로 세 축을 다 잡는다** — 축 하나만 돌리면 시선은 맞아도
화면이 돌아간다. 계산식은 `isaac_scene.json` 의 `_camera_comment` 에 있다.

### 기동 로그의 경고 — 무시해도 되는 것

`--headless` / `--gui` 출력에 매번 나오지만 **손댈 필요가 없는** 줄들이다. 다 아는
것으로 적어 두는 이유는, 이 중 하나를 쫓다가 반나절을 쓴 적이 있어서다.

| 줄 | 뜻 | 조치 |
|---|---|---|
| `[ROS2 Publish Joint State] Reading from targetPrim is deprecated` | 그래프가 구식 입력 방식을 쓴다 | 없음. 동작한다 |
| `OgnROS2PublishTransformTree: using targetPrims ... deprecated` | 같은 종류 | 없음 |
| `OgnROS2CameraInfoHelper: frameSkipCount deprecated` | `omni:sensor:tickRate` 로 옮기라는 안내 | **없음.** 옮겨 봤고 5 Hz 가 9.9 Hz 로 깨진다 (`setup_graph.py` 주석) |
| `Deprecated: direct use of ITimeline callbacks` | Kit 내부 | 없음 |
| `[omni.rtx] DLSS increasing input dimensions: Render resolution of (320, 240) is below minimal input resolution of 300.` | 카메라 렌더에 DLSS 가 붙는다 | **없음 — 아래** |

DLSS 줄만 설명이 필요하다. 뷰포트가 아니라 **우리 카메라**
(`/Render/OmniverseKit/HydraTextures/Replicator`)에 붙은 것으로, 640×480 출력을 절반
해상도에서 올리려다 DLSS 최소 입력(300)에 못 미쳐 **DLSS 가 스스로 입력을 키운다**는
뜻이다. 2026-09-02 에 두 가지를 재 보았다.

* **끌 수 없다.** `SimulationApp(anti_aliasing=0)` 과 런치 인자
  `--/rtx/post/aa/op=0` · `--/rtx-defaults/post/aa/op=0` 을 **모두** 줘도(인자가
  전달되는 것까지 확인) 경고와 동작이 그대로다.
* **끌 이유도 없다.** 라플라시안 분산이 정지 174.6(DLSS) / 176.2(끔), 팔을 흔드는
  중 145.0 / 145.3 으로 0.2~0.9% 차이 — 잡음 수준이다.

그래서 노브를 두지 않았다. `headless_bringup.py` 주석에 같은 내용이 있다.

**한때 여기 있다가 고쳐진 줄**: `Forcing fy to fx (753.39 != 733.00) ... as renderer
assumes square pixels`. 이건 무해하지 않았다 — `setup_scene.py` 가 초점거리만 넣고
조리개는 USD 기본값(20.955 × 15.2908, 비율 1.3704)에 맡겨 640×480(1.3333)과 어긋나
있었고, 그래서 **prim 이 적어 놓은 세로 화각과 실제로 렌더된 화각이 4% 달랐다.**
이미지는 멀쩡해 보이므로 경고를 읽지 않으면 드러나지 않는다. 지금은 세로 조리개를
해상도에서 유도하고 가로 기본값을 21.0 으로 두어(20.955 는 float32 에서 마지막 자리가
갈라진다) `fx == fy` 가 정확히 성립한다. `camera_info` 의 `fx` 는 733.0 에서 731.4 로
바뀌었다. 회귀 검사: `robot_control/tests/test_isaac_camera_intrinsics.py`.

### 로봇이 원점에 있어야 한다

ROS 쪽 static TF 가 `world → panda_link0 = 항등변환`으로 못박혀 있다. 자산을 드래그해
올리면 원점이 아닌 자리에 떨어지는데, 그러면 **로봇 내부는 일관되고 scene 물체만
어긋나** 그리퍼가 "0.5 mm 오차로 도달"했다고 보고하며 허공을 쥔다. `place_robot` 이
그 일을 한다.

---

## 7. 관련 문서

- [isaac_backend_skeleton.md](isaac_backend_skeleton.md) — Phase 0~10 설계·수용 기준,
  배포 구성 네 가지, Isaac 6.0 재검증 결과
- [../scene/isaac_scene_reset.md](../scene/isaac_scene_reset.md) — `/scene/reset` 경로와
  선행 조건
- [../gripper/GripperActionNode_Guide.md](../gripper/GripperActionNode_Guide.md) —
  그리퍼 노드 파라미터 (`stall_effort` 등)
- [../../scripts/run_isaac_sim.sh](../../scripts/run_isaac_sim.sh) — 기동 스크립트
  (환경 변수의 이유가 머리에 적혀 있다)
- [../../scripts/kill_stack.sh](../../scripts/kill_stack.sh) — 정리 스크립트
