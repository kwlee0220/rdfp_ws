# 백엔드 프로파일

**백엔드 하나 = 파일 하나.** 그 백엔드에서만 달라지는 설정을 여기 모은다.

새 백엔드를 붙일 때는 가장 가까운 기존 파일을 복사해 값만 바꾼다. 파일 이름(확장자
제외)이 곧 `backend` 이름이며, `backend: <이름>` 으로 참조된다.

    ros2 run rdfp teleop_keyboard --ros-args -p backend:=isaac
    # robot_twin YAML:  moveit: { backend: isaac }

**이 표가 "어느 스택이 무엇인가"의 정본이다.** 같은 사실을 launch·노드·트윈 설정에
각각 적으면 백엔드가 바뀔 때 한쪽만 뒤처지고, 그 어긋남은 에러가 아니라 "명령이 안
먹는다"로 나타난다 — Isaac 이 bridge → ros2_control 로 바뀔 때 실제로 그랬다.

## 블록

| 블록 | 내용 | `null` 이면 |
|---|---|---|
| `arm_command` | **MoveGroup 클라이언트**가 팔에 명령을 보내는 채널 (`mode`/`topic`/`format`/`joint_names`) | — |
| `ros2_control` | controller_manager 배선 (`hardware_type`/`controllers_file`/`joint_commands_topic`/`gripper_commands_topic`/`joint_states_topic`) | ros2_control 을 안 쓴다 (펑션베이) |
| `simulator` | ros2_control 없는 백엔드가 시뮬레이터와 직접 주고받는 채널 (**servo 출력은 여기 없다** — 아래 참조) | — |
| `motion` | 팔 이동의 **전역 기본** `velocity_scaling` · 중력 보상 `gravity_compensation` | 이 백엔드만의 값이 없다 (코드 기본값 1.0) |
| `servo` | `linear_scale` / `joint_source` / `command_out_type` | — |
| `camera` | `source` (`device`/`compressed`/`native`) · `image_topic` (raw) · `compressed_topic` | 카메라가 없다 |
| `scene` | 움직이는 물체 정의 `file` · **안 움직이는** 물체 정의 `fixtures_file` | 정의 파일이 없다 (mock 은 `/scene/reset` 이 놓은 것만) |
| `use_sim_time` | 시계 출처 | — |

**`null` 은 "빠뜨렸다" 가 아니라 "이 백엔드에는 없다" 는 뜻이다.** 모르는 키는 거부한다
— 조용히 무시하면 "설정했는데 안 먹는다" 가 되고, 그 증상은 원인을 안 가리킨다.

### ⚠️ `arm_command.topic` 과 `ros2_control.joint_commands_topic` 은 다른 것이다

앞은 **MoveGroup 클라이언트**가 쓰는 채널, 뒤는 **하드웨어 인터페이스**가 시뮬레이터와
주고받는 채널이다. 옛 bridge 시절 Isaac 이 이 둘을 섞어 `arm_command.topic:
/isaac/arm_command` 로 두었더니, 클라이언트가 **컨트롤러를 우회해** 같은 채널에 끼어들어
서로 싸웠다. JTC 백엔드는 `arm_command` 에 채널을 갖지 않는다.

### `scene.file` 과 `scene.fixtures_file` 은 다른 것이다

| | 무엇이 들어 있나 | pose 는 어디서 | 발행 채널 |
|---|---|---|---|
| `file` | **움직이는** 물체의 이름·종류·크기 | 시뮬레이터 `/tf` | `/scene/objects` |
| `fixtures_file` | **안 움직이는** 물체 (구멍·트레이) — pose 까지 | **이 파일** | 없음 (라이브러리로 읽는다) |

벤더가 `static="false"` 인 body 만 TF 로 내보내므로 고정물은 **조회할 프레임이 아예
없다.** 그래서 pose 를 파일이 갖는다.

**`/scene/objects` 로 발행하지 않는다.** 그 채널은 조작 대상 전용이고 `SceneObject` 에는
고정물을 구분할 필드가 없다 (`fixture` 는 2026-09-01 에 삭제됐다) — 실으면 모든 소비자에게
peg 과 똑같이 보인다. 값이 상수라 에피소드마다 기록할 이유도 없다.

```python
from robot_control.scene.fixtures import load_fixtures
hole = load_fixtures('functionbay')['peg_hole']
x, y, z = hole.entry_point()          # 구멍 입구 (panda_link0 기준)
```

명령줄로는 `./scripts/functionbay/fb_fixtures.py` (`--json` 지원). **ROS 도 시뮬레이터도
필요 없다.**

### `motion.velocity_scaling` — 아무것도 안 줬을 때의 바닥값

팔 이동 속도의 **전역 기본값**이다. 우선순위는 이렇다.

    이동 메서드의 인자  >  create_move_group_client(velocity_scaling=…)  >  프로파일  >  1.0

`create_move_group_client(node, backend='functionbay')` 만 해도 프로파일 값이 실린다.
**블록을 생략하면 키를 아예 안 넘기므로** 코드 기본값(1.0)이 산다 — `0.0` 이나 `1.0` 을
돌려주면 "프로파일이 정했다"와 "안 정했다"를 구별할 수 없다.

> ⚠️ **트윈은 `backend=` 를 팩토리에 넘기지 않는다** — 설정 단계에서 값으로 펼친다
> (`MoveItConfig._expand_backend` → `client_kwargs()`). 새 `motion` 키를 더할 때는
> 그쪽도 함께 이어야 하며, 안 그러면 **프로파일에 적어도 트윈에만 안 먹는다.**

> ⚠️ **관절공간 계획에는 거의 안 먹는다.** MoveIt 이 스케일하는 것은 관절 속도이고 EE
> 경로 길이와 선형 관계가 아니다 — 실측(펑션베이 `ready` 복귀 430 mm): 1.0 → 0.2 로
> 5배 줄여도 EE 속도는 243 → 154 mm/s (1.6배)였다. 데카르트 구간은 267 → 54 mm/s 로
> 비례한다.

### `motion.gravity_compensation` — 처짐을 우리가 상쇄한다

중력 보상이 없는 백엔드(펑션베이)는 `실제 − 지령 = τ_g(q)/Kp` 로 처진다. `Kp` 를 씬
XML 이 알려 주므로(`<씬>__Manipulator_1_vm.xml`, t1·r2 모두 2000) **스트리밍하는 모든
명령점에 `−τ_g(q)/Kp` 를 더해** 상쇄한다.

```yaml
motion:
  gravity_compensation:
    urdf_package: panda_ftsensor_robotiq          # 벤더가 준 대상 로봇 기술
    urdf_relative_path: urdf/panda_ftsensor_robotiq.urdf
    kp: 2000.0
    # scale: 0.5        # 시뮬레이터가 일부만 보상할 때
```

**키가 없으면 보상하지 않는다** — mock·Isaac 은 컨트롤러가 중력을 잡으므로 없다.
**JGPC 전용**이며(스트리밍 경로에만 얹는다) `create_move_group_client()` 가 자동으로
구성한다.

> **벤더가 A-1 을 반영하면 이 블록을 지운다** — 안 지우면 이중 보정이다. 판별·절차:
> [gravity_compensation](../../../../docs/simulation/functionbay_gravity_compensation.md) §5.

> **~~`motion.sag_calibration_file`~~ 은 2026-09-11 에 삭제됐다.** 지점별로 잰 처짐
> 계수를 지령에 얹던 방식인데, 잰 지점 ±20 mm 에서만 유효했고 자세를 바꾸면 무효였다.
> 위 중력 보상이 작업 영역 전체를 덮으므로 대체했다 — **둘을 함께 켜면 이중 보정이다.**

### ⚠️ servo 출력 채널은 `servo.command_out_topic` 하나다

ros2_control 이 없는 백엔드는 servo 출력을 `servo_command_bridge` 가 받아 시뮬레이터로
옮긴다. 그 **다리의 입력은 정의상 servo 의 출력**이므로 키가 하나여야 한다.

한때 `simulator.servo_command_topic` 에 같은 값을 한 번 더 적어 두었다. launch 는 다리의
remap 을 그쪽에서, servo 파라미터를 `servo.command_out_topic` 에서 읽었으므로 **둘이
갈리면 servo 는 발행하고 다리는 못 받는다** — 증상은 "모션 키가 아무것도 안 한다" 뿐이고,
그것이 정확히 이 백엔드가 예전에 겪은 증상이라 원인을 못 가린다. 같은지 검사하는 곳도
없었다. 그래서 `simulator` 블록에서 뺐다.

### 값으로 표현 안 되는 것은 클래스에 둔다

`factory: <module>:<Class>` 를 적으면 그 클래스가 만들어진다. 없으면 값만 읽는
`Backend` 기본 구현이다 — **빈 하위 클래스를 만들지 않는다.**

| 백엔드 | 클래스 | 클래스가 하는 일 |
|---|---|---|
| `isaac` | `IsaacBackend` | `servo.joint_source: commanded` 를 servo 의 `joint_topic` 으로 옮긴다 |
| `functionbay` · `mock` · `mock_jgpc` | (기본) | 값만 다르다 |

한때 `FunctionbayBackend` 가 있었다 — servo 출력 형식을 바꾸고 `publish_joint_velocities`
를 끄는 클래스였다. 지금 그 둘은 **값에서 파생된다**: 형식은 `servo.command_out_type` 이
갖고, 딸려 오는 `publish_joint_*` 는 `_float64_multi_array_rule` 이 넣는다. 남은 코드가
없어 빈 하위 클래스가 되므로 지웠다.

**값 하나가 파라미터 하나로 안 갈 때**가 클래스의 자리다. 프로파일은 "무엇을 기준으로
삼는가"를 말하고 servo 는 "어느 토픽을 읽는가"를 받는데, 그 사이 변환은 값으로 적을 수
없다.

```python
from robot_control.backends import get_backend

backend = get_backend('isaac')
client = backend.create_move_group_client(node)
params = backend.apply_servo_parameters(load_servo_params(...))
```

### `camera.source` — raw 를 어떻게 얻는가

| 값 | 뜻 | 뜨는 노드 | 백엔드 |
|---|---|---|---|
| `device` | 장치/파일에서 우리가 캡처한다 | `camera_node` (OpenCV) | `mock` · `mock_jgpc` |
| `compressed` | 시뮬레이터가 **압축만** 준다 | `image_transport/republish` | `functionbay` |
| `native` | 시뮬레이터가 **raw 를 직접** 준다 | 없다 | `isaac` |

launch 는 `create_raw_image_source_node(backend)` 하나만 부르고, **소비자는 어느 쪽이
떴는지 모른다** — `camera_image_topic` 만 구독한다. 스위치도 `enable_camera` 하나이며
그 기본값은 그 launch 의 **raw 소비자**에서 파생된다(뷰어를 켜면 디코더가 함께 뜬다).

**키 유무로 추론하지 않는 이유**는 `device` 와 `native` 가 둘 다 `image_topic` 만 갖기
때문이다. 설계: [docs/camera/compressed_image_pipeline_design.md](../../../../docs/camera/compressed_image_pipeline_design.md).

### 압축 카메라는 이름이 둘이다

펑션베이처럼 시뮬레이터가 **압축으로 발행**하면 `compressed_topic` 과 `image_topic` 이
모두 필요하다. 소비자가 갈리기 때문이다.

| 소비자 | 구독 | 근거 |
|---|---|---|
| 세션 구동 레코더 (`rdfp_image_recorder`) | **압축** | `input_format: mjpeg` 으로 JPEG 를 디코드 없이 ffmpeg 에 넘긴다 |
| 뷰어 (`image_viewer_node`) | raw | `sensor_msgs/Image` 만 구독한다 |

raw 는 `image_transport/republish` 가 **뷰어를 켤 때만** 만든다 — 되살리는 순간 압축으로
아낀 대역폭이 돌아오기 때문이다(0.27 → 9.2 MB/s, 36배).

**두 이름 다 이 파일에서만 온다.** 제어 계열 `panda_functionbay` 와 수집 계열
`rdfp_panda_functionbay` 가 같은 프로파일을 읽는다. 한때 제어 쪽이 `image_pipeline.yaml`
(OpenCV 카메라용)에서 `camera_image_topic` 을 가져와, 프로파일은 `/camera_image` 라고
적혀 있는데 실효값은 `/camera/image_raw` 였다 — 각 launch 안에서는 republish 출력과 뷰어
구독이 같은 값을 써서 **동작은 했고, 그래서 안 드러났다.**

> ⚠️ **서비스 구동 `image_recorder_node` 는 압축을 못 받는다.** 그것을 쓰는 launch 에서
> 뷰어를 끄면 raw 발행자가 사라져 **레코더가 조용히 0 프레임을 담는다.**
> `rdfp_panda_functionbay` 가 2026-09-08 까지 그 상태였다.

### 경로는 패키지 share 기준이다

`controllers_file` / `scene.file` 은 `robot_control` 패키지 안의 상대 경로다. 설치본에서
해석되므로 **패키지 밖 파일은 넣을 수 없다.**

그래서 녹화 토픽 목록(`config/recording_topics*.list`)은 여기 없다 — 워크스페이스 루트에
있고 `record_rosbag.sh` 가 `RECORDING_TOPICS_FILE` 로, `dataset_config.yaml` 이
`topics_file` 로 읽는다. 옮기려면 파일 위치와 그 두 소비자를 함께 고쳐야 한다.

## 주의

**고친 뒤에는 `colcon build` 를 다시 돌린다.** 이 파일들은
`share/robot_control/config/backends/` 로 설치된 사본이 읽힌다 — 소스만 고치면
설치본이 낡은 채로 남아 "고쳤는데 안 바뀐다"가 된다.

**파일을 지웠을 때는 재빌드로 부족하다.** colcon 은 복사만 하고 없어진 파일을 치우지
않으므로 설치본에 그대로 남아 **지운 백엔드가 계속 보인다.** 설치본 쪽도 함께 지운다.

```bash
rm install/robot_control/share/robot_control/config/backends/<이름>.yaml
colcon build --packages-select robot_control
```

## `auto` 라는 백엔드는 없다

모든 프로파일은 **확정 모드**(`jtc` / `jgpc`)를 갖는다. 표가 있는 이유가 런타임 판별의
조용한 오판을 없애는 것인데, 표 안에 판별로 되돌아가는 항목을 두면 앞뒤가 안 맞기
때문이다. mock 계열이 스택 둘이라 예전엔 `auto` 였지만, 지금은 `mock`(JTC)과
`mock_jgpc`(JGPC)로 나뉘어 있다.

붙어 있는 스택에 맞추고 싶은 저수준 경로는 `backend` 없이 `mode='auto'` 를 쓴다 —
샘플 스크립트처럼 백엔드를 모르고 붙는 코드가 그 경우다.
