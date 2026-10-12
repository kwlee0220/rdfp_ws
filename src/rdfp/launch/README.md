# rdfp launch 안내

**학습 데이터 수집 계층의 launch 진입점.** 로봇 제어 스택 위에 세션·녹화·재생·
teleop 노드를 얹는 `rdfp_*` 계열이다.

```bash
ros2 launch robot_control panda_mock.launch.py               # 터미널 1 — 제어 스택
ros2 launch rdfp rdfp_collect.launch.py backend:=mock        # 터미널 2 — 수집 계층
```

> **제어 스택만 필요하면** `robot_control` 패키지의 `panda_mock` 계열을 쓴다 —
> [src/robot_control/launch/README.md](../../robot_control/launch/README.md).
> 그 문서는 제어 계열 launch 3개와 **helper 모듈 전체**를 자체적으로 담고 있다.
> 패키지를 나눈 근거는 설계서 [§7.6](../../../docs/rdfp_framework_design.md).

## helper 는 하위 패키지에 있다

이 디렉터리에는 launch 파일만 있고 helper 는 없다. 전부
`robot_control/launch_helpers/` 의 설치 모듈이며 정규 import 로 가져온다.

```python
from robot_control.launch_helpers.camera import create_camera_node
```

helper 목록과 각각의 역할은
[robot_control launch README §5](../../robot_control/launch/README.md) 에 있다.

---

## 목차

- [1. Launch 파일](#1-launch-파일)
- [2. Launch 인자 — `--show-args` 가 쓸모없다](#2-launch-인자----show-args-가-쓸모없다)
- [3. YAML ↔ 인자 대응표](#3-yaml--인자-대응표)
- [4. Launch × Helper 의존 관계](#4-launch--helper-의존-관계)
- [5. 실행 구조와 순차 기동](#5-실행-구조와-순차-기동)
- [6. 어느 launch 를 쓸까](#6-어느-launch-를-쓸까)
- [7. 유지보수 메모](#7-유지보수-메모)

---

## 1. Launch 파일

| launch | 요약 | 설정 YAML |
|---|---|---|
| `replay_panda_mock.launch.py` | 데이터셋 **재생 전용** variant | `replay_panda_mock.yaml` |
| `teleop_mirror.launch.py` | leader-follower 미러링 체인 | `teleop_mirror.yaml` |
| `rdfp_collect.launch.py` | **수집 노드 4종만** — 이미 떠 있는 제어 스택 위에 얹는다 | `image_pipeline.yaml` |

### (이력) mock 번들 둘은 2026-09-13 에 삭제했다

`rdfp_panda_mock` · `rdfp_panda_mock_jgpc` 은 제어 스택을 **복제**해 담고 있었다
(348줄 / 406줄). `panda_mock` + `rdfp_collect` 조합이 **같은 노드 그래프**를 만든다는
것을 실측으로 확인한 뒤 지웠다 — 격리 도메인에서 노드 18개, 수집 노드 4종의 전체
파라미터, 토픽 엔드포인트를 덤프해 diff 했고 전부 일치했다 (§6.1).

```bash
# JTC
ros2 launch robot_control panda_mock.launch.py
ros2 launch rdfp rdfp_collect.launch.py backend:=mock

# JGPC
ros2 launch robot_control panda_mock_jgpc.launch.py
ros2 launch rdfp rdfp_collect.launch.py arm_cmd_source:=float64_multi_array
```

`backend:=mock` / `backend:=mock_jgpc` 를 쓰면 arm 채널·카메라 토픽·`use_sim_time` 이
프로파일에서 오므로 인자를 외울 필요가 없다.

⚠️ **딸려 간 것 둘** — 도커 이미지 `rdfp-panda-mock` · `rdfp-panda-mock-jgpc`
(컨테이너 하나에 런치 둘을 담을 수 없다; [docker/README.md](../../../docker/README.md)),
그리고 `config/panda_robot.yaml` 자체(그 둘만 읽던 파일이라 함께 지웠다).

### `replay_panda_mock`

재생 도구가 공급하는 토픽을 그대로 쓰기 위해 `camera`, `ee_pose`,
`session_control`, `image_recorder`, `target_joint_cmds_publisher` 를 **기동하지
않는다.** `joint_state_broadcaster` 는 재생 결과 관측값이므로 유지한다.
`gripper_action_node` 도 유지되어 재생된 gripper 명령을 액션으로 전달한다.

**scene 노드는 의도적으로 제외한다** — 재생 중 물체 상태는 데이터셋에서 나와야 하고,
scene 노드를 띄우면 라이브 planning scene 이 두 번째 출처로 섞인다.

arm 구동 노드는 `replay_arm_path` 로 **배타 선택**한다. 두 경로가 동시에
`/panda_arm_controller/joint_trajectory` 를 쓰면 명령이 충돌한다.

| 값 | 기동 노드 | 배선 |
|---|---|---|
| `target_joint_cmds` | `target_joint_cmds_executor` | `/target_joint_cmds` (JointState) → JointTrajectory(길이 1) → `/panda_arm_controller/joint_trajectory` |
| `ee_twist` (기본) | `ee_twist_publisher` + `servo_auto_start` | `/ee_pose` → 유한 차분 → `/servo_node/delta_twist_cmds` → servo → JTC |
| `none` | — | `MoveGroupClient.follow_trajectory` 등 외부 클라이언트로 구동 |

`target_joint_cmds` 는 위치 폐루프라 드리프트가 없고 여유자유도까지 재현되지만,
`ee_twist` 는 속도 개루프라 오차가 누적된다. 전체 사용법·주의점은
[docs/collect/replay_mock_stack_guide.md](../../../docs/collect/replay_mock_stack_guide.md).

### `teleop_mirror`

leader-follower 동작 미러링 체인 2종을 기동한다 — `teleop_retarget` (클러치 기반
retargeting), `ee_twist_node` (목표 pose → `/servo_node/delta_twist_cmds`).

- **leader pose (`leader_pose_topic`, 기본 `/leader/ee_pose`) 는 외부 어댑터가
  공급한다는 전제** — 본 런치는 그 토픽의 발행자를 만들지 않는다. OMY-L100 은
  별도 저장소 `omy_leader_bridge` 가 담당한다. 계약:
  [docs/teleop/external_input_adapters.md](../../../docs/teleop/external_input_adapters.md).
  leader `/tf` 를 직접 조회해야 하면 `ros2 run robot_control ee_pose_node` 를 따로 띄운다.
- follower Panda + MoveIt2 스택이 **이미 실행 중**이라는 전제 (servo 포함).
- **USB 풋페달**로 클러치를 잡으려면 `enable_clutch_pedal:=true` 를 준다. 기본
  `false` 인 이유는 `python3-evdev` 와 실제 장치가 없으면 노드가 기동에 실패하기
  때문이다.
- 데드맨은 **페달 노드 + `pedal_timeout`(하트비트 감시)** 두 조각이 함께 있어야
  성립하므로 런치가 정합을 자동으로 맞춘다 (§3.4 의 표).

설계: [docs/teleop/leader_follower_mirroring_design.md](../../../docs/teleop/leader_follower_mirroring_design.md),
운영: [docs/teleop/omy_leader_teleop_guide.md](../../../docs/teleop/omy_leader_teleop_guide.md),
노드: [docs/teleop/teleop_retarget_node_guide.md](../../../docs/teleop/teleop_retarget_node_guide.md),
풋페달: [docs/teleop/clutch_pedal_guide.md](../../../docs/teleop/clutch_pedal_guide.md).

### (이력) `rdfp` · `rdfp_advanced` 는 2026-09-12 에 삭제했다

**MoveIt 없이 세션·뷰어·녹화만 띄우던 앱 둘**이며, `rdfp_collect` 가 둘을 다 덮는다.
노드·파라미터·remap 을 대조해 확인한 차이는 아래가 전부였다.

| 없어진 launch | `rdfp_collect` 로 어떻게 하나 |
|---|---|
| `rdfp` (세션 + 뷰어 + **서비스** 레코더) | 세션 구동 레코더로 대체 — 서비스 구동은 launch 에서 빠졌다 (2026-09-13) |
| `rdfp_advanced` (세션 + `rdfp_camera_node` + 뷰어 + **세션** 레코더) | 기본값 그대로 (레코더가 이미 세션 구동) |
| `rdfp_advanced` 의 arm 탭 지점 `/servo_node/joint_trajectory` | `target_joint_cmds_input_topic:=/servo_node/joint_trajectory` |

남은 차이 둘은 **의도적으로 옮기지 않았다.**

- **뷰어가 갈렸다** — `rdfp` 는 `robot_control` 의 평범한 `image_viewer_node`, 나머지는
  `rdfp` 의 세션 오버레이 `rdfp_image_viewer_node` 였다. `rdfp_collect` 는 항상
  `session_control` 을 띄우므로 오버레이가 늘 동작한다 — 상위 호환이라 인자로 열지
  않았다.
- ⚠️ **`rdfp_camera_node` 를 띄우는 launch 가 없어졌다.** 콘솔 스크립트로는 남아 있으니
  `ros2 run rdfp rdfp_camera_node` 로 쓴다
  ([가이드](../../../docs/collect/session_camera_nodes_guide.md)). `rdfp_collect` 는 카메라를
  띄우지 않는다 — 제어 스택이나 시뮬레이터가 발행하는 이미지를 받는 것이 전제다.

### `rdfp_collect`

> 📘 **절차서는 따로 있다** — [docs/collect/수집_계층_얹기_절차서.md](../../../docs/collect/수집_계층_얹기_절차서.md).
> 이 절은 인자·구조 레퍼런스이고, 백엔드별 기동 명령·에피소드 수집 절차·함정은 그쪽이 갖는다.

수집 노드 4종(`session_control_node`, `rdfp_image_viewer_node`, `image_recorder_node`,
`target_joint_cmds_publisher`)**만** 띄운다. 제어 스택은 별도 터미널에서 미리 올린다.

```bash
ros2 launch robot_control panda_mock.launch.py    # 터미널 1 — 로봇
ros2 launch rdfp rdfp_collect.launch.py          # 터미널 2 — 수집
```

**이 조합이 옛 `rdfp_panda_mock` 을 대체한다.** 동일한 노드 그래프를 만드는 것을 실측으로 확인했다
(§6.1). 제어 스택을 재시작하지 않고 수집 계층만 껐다 켤 수 있어서, 녹화 설정을
바꿔가며 반복 실험할 때 MoveIt·RViz 재기동을 아낀다.

분리가 가능한 이유는 **수집 노드 넷에 기동 순서 의존이 없기 때문**이다. 넷 다
구독자·상태머신·서비스 서버라 발행자보다 먼저 떠도 되고, ROS 2 의 discovery 가
동적이라 순서가 자유롭다. 유일한 예외인 `target_joint_cmds_publisher` 의 관절 이름
조회도 `__init__` 이 아니라 타이머에서 `joint_names_timeout`(기본 10초) 안에
비동기로 하며, 실패해도 `JointState.name` 이 비는 것으로 그친다. 제어 스택을 먼저
띄우는 이 방식에서는 조회 시점에 컨트롤러가 확실히 올라와 있다.

`--show-args` 가 정상 동작한다 — YAML 에 의존하지 않는 인자는
`OpaqueFunction` 밖에서 선언했다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `arm_cmd_source` | `joint_trajectory` | arm 명령 채널. `joint_trajectory`(JTC) / `float64_multi_array`(JGPC) / `none` |
| `target_joint_cmds_input_topic` | `''` | 비우면 `arm_cmd_source` 에 맞는 기본값 (`/panda_arm_controller/joint_trajectory` 또는 `/commands`) |
| `arm_controller_node_name` | `/panda_arm_controller` | 관절 이름 조회 대상 |
| `use_sim_time` | `false` | Gazebo 백엔드와 함께 쓸 때 `true` |
| `log_level` | `info` | `session_control` 로그 레벨 |
| `config_file` | `<robot_control share>/config/image_pipeline.yaml` | 카메라·뷰어·레코더 인자 출처 |

> ⚠️ **설정값 정합은 사용자 책임이 된다.** `camera_image_topic` / `camera_resolution`
> / `camera_fps` 는 제어 스택과 공유한다. 기본값이 같은 YAML 에서 오므로 손대지
> 않으면 자동으로 일치하지만, **한쪽에만 override 하면 조용히 어긋난다** — 레코더가
> `invalid frame dropped` 를 내고 auto-stop 한다. 바꿀 때는 양쪽에 같은
> `config_file:=` 을 준다. 예전 묶음 launch 는 한 곳에서 선언했으므로
> 구조적으로 어긋날 수 없다 — **그것이 묶음 launch 를 남겨 둔 이유다.**

---

> **백엔드마다 인자 기본값이 어떻게 다른지**는 제어 계열 README 의
> [§2.0 백엔드 횡단표](../../robot_control/launch/README.md#20-백엔드-횡단표--같은-인자가-스택마다-어떻게-다른가)
> 를 본다. `rdfp_*` 는 그 백엔드 launch 를 include 하므로 거기 값이 그대로 바닥에 깔린다
> (이 계층이 덮어쓰는 것만 아래 §3 표에 있다). 그 표는 손으로 맞추지 않고
> `test_backend_argument_matrix.py` 가 네 launch 를 실제로 만들어 대조한다.

## 2. Launch 인자 — `--show-args` 가 쓸모없다

> ⚠️ **이 계열에서는 `--show-args` 를 믿으면 안 된다.** `config_file` 이
> `OpaqueFunction` 안에서 resolve 된 **뒤에야** 나머지 argument 를 선언하므로
> 출력에 20여 개 중 1~2 개만 나온다. **§3 의 표가 유일한 전체 목록이다.**

`--show-args` 로 실제 노출되는 인자 수 (측정값):

| launch | 노출 | 실제 |
|---|:-:|:-:|
| `teleop_mirror` | **1** (`config_file`) | 23 |
| `replay_panda_mock` | **1** (`config_file`) | 13 |
| `rdfp_collect` | **10** | 20 |

`rdfp_collect` 만 절반이 보이는 것은 그 인자들을 `_build_actions` 가 `perform()` 하기
때문이다 — perform 하려면 `OpaqueFunction` **밖에서** 선언해야 하고, 그래서 덤으로
`--show-args` 에 드러난다.

`robot_control` 의 `panda_mock` 계열은 반대다 — 선언 시점에 전부 노출하므로
`--show-args` 가 그대로 동작한다 ([그쪽 README §2](../../robot_control/launch/README.md)).

### 설정 파일은 관심사별로 나뉜다

로봇 설정과 이미지 파이프라인 설정은 바뀌는 주기도, 쓰는 launch 도 다르다.

| 설정 파일 | 소유 패키지 | 읽는 launch | 지정 argument |
|---|---|---|---|
| `config/image_pipeline.yaml` | **`robot_control`** | `rdfp_collect` | `config_file` |
| | | `rdfp`, `rdfp_collect` | `config_file` |
| `config/replay_panda_mock.yaml` | `rdfp` | `replay_panda_mock` | `config_file` |
| `config/teleop_mirror.yaml` | `rdfp` | `teleop_mirror` | `config_file` |
| `config/panda_jgpc_ros2_controllers.yaml` | **`robot_control`** | `panda_mock_jgpc` (제어 계열) | — |
| `config/panda.rviz` | **`robot_control`** | RViz 를 띄우는 전부 | — |

**launch 를 소유한 패키지와 설정 파일을 소유한 패키지가 다를 수 있다** —
`rdfp_collect` 는 `rdfp` 에 있지만 이미지 파이프라인 YAML 은 `robot_control` 의
share 에서 읽는다.

---

## 3. YAML ↔ 인자 대응표

### 3.1 `image_pipeline.yaml` — YAML ↔ 인자 대응

camera → image_viewer / image_recorder 파이프라인 설정. **여섯 launch 의 공통
출처**이며 파일은 `robot_control` 패키지가 소유한다
(`<robot_control share>/config/image_pipeline.yaml`). 세 블록을 한 파일에 둔 이유는 값을 서로 공유하기 때문이다.

```
camera.image_topic  ─▶ image_viewer / image_recorder 의 `image` remap 대상
camera.resolution   ─▶ image_recorder 의 resolution (별도 키가 없다)
camera.fps          ◀▶ image_recorder.fps  (어긋나면 프레임 drop)
```

| YAML 블록 | 키 | launch argument | 현재 YAML 값 | 비고 |
|---|---|---|---|---|
| `camera` | `enabled` | `enable_camera` | `true` | **raw 이미지 발행자를 둘 것인가.** 무엇이 뜨는지는 백엔드 프로파일의 `camera.source` 가 정한다 (`device`→`camera_node` / `compressed`→`republish` / `native`→없음). 압축 백엔드에서는 기본값이 raw 소비자(뷰어·`rawvideo` 레코더)에서 파생된다 — [설계](../../../docs/control/compressed_image_pipeline_design.md) |
| | `id` | `camera_id` | `…/data/shibuya_7_8.mp4` | 장치 인덱스 또는 파일/URI |
| | `image_topic` | `camera_image_topic` | `/camera/image_raw` | 뷰어·레코더도 같이 remap 된다 |
| | `info_topic` | `camera_info_topic` | `/camera/camera_info` | |
| | `status_topic` | `camera_status_topic` | `/camera/camera_status` | |
| | `fps` | `camera_fps` | `10` | |
| | `resolution` | `camera_resolution` | `1280x720` | **`image_recorder` 가 그대로 쓴다** |
| | `frame_id` | `camera_frame_id` | `camera_link` | |
| | `compress_image` | `camera_compress_image` | `false` | `rdfp_camera_node` 는 미지원 |
| `image_viewer` | `enabled` | `enable_image_viewer` | `true` | 헤드리스면 `false` |
| `image_recorder` | `enabled` | `enable_image_recorder` | `true` | |
| | `fps` | `image_recorder_fps` | `10` | **`camera.fps` 와 맞춘다** |
| | `output_dir` | `image_recorder_output_dir` | `/data/rdfp/recordings` | |
| | `auto_start` | `image_recorder_auto_start` | `true` | 서비스 구동 레코더 전용 |

`RdfpImageRecorderNode`(세션 구동)는 `/session` 상태로 녹화를 시작하므로
`auto_start` 파라미터를 받지 않는다. 그 노드만 쓰는 launch 는
`declare_image_pipeline_arguments(config, include_auto_start=False)` 로 선언에서
뺀다 — 소비자 없는 argument 를 만들지 않기 위해서다. **레코더를 고를 수 있는
`rdfp_collect` 도 선언하지 않는다** — 세션 구동 레코더만 띄우기 때문이다.

### (삭제) 3.2 mock 번들의 YAML ↔ 인자 대응

`rdfp_panda_mock` / `rdfp_panda_mock_jgpc` 전용 절이었다. 두 launch 와 함께
2026-09-13 에 없앴다 — 그 조합은 이제 `panda_mock` + `rdfp_collect` 이고, 인자는
제어 계열 README 의 `panda_mock` 절과 이 문서의 §3.5(`rdfp_collect`)로 나뉜다.

`config/panda_robot.yaml` 은 이 두 launch 만 읽던 파일이라 **함께 삭제했다.**

### 3.3 `replay_panda_mock` — YAML ↔ 인자 대응

설정 파일은 **전용** `config/replay_panda_mock.yaml` 이다. 13개가 전부다.

| YAML 블록 | 키 | launch argument | 현재 YAML 값 | 비고 |
|---|---|---|---|---|
| `ros2_control` | `hardware_type` | `ros2_control_hardware_type` | `mock_components` | URDF 생성에 쓰인다 |
| *(최상위)* | `log_level` | `log_level` | `info` | `move_group` / `servo` |
| `ee_pose` | `base_frame` | `base_frame` | `panda_link0` | `ee_pose_node` 는 안 띄우지만 `ee_twist` 가 프레임을 쓴다 |
| | `ee_frame` | `ee_frame` | `panda_hand` | |
| `image_viewer` | `enabled` | `enable_image_viewer` | `true` | |
| | `image_topic` | `camera_image_topic` | `/camera/image_raw` | 재생된 이미지 토픽. argument 이름은 호환 위해 유지 |
| `replay` | `arm_path` | `replay_arm_path` | `ee_twist` | arm 구동 경로 **배타 선택**. `ee_twist` \| `target_joint_cmds` \| `none` |
| `replay.ee_twist` | `source_topic` | `ee_twist_source_topic` | `/ee_pose` | 미분 대상 PoseStamped 입력 |
| | `output_topic` | `ee_twist_output_topic` | `/servo_node/delta_twist_cmds` | servo 가 소비하는 출력 |
| | `linear_gain` | `ee_twist_linear_gain` | `2.5` | `1/scale.linear`. servo 가 `command_in_type: unitless` 라서 필요 |
| | `angular_gain` | `ee_twist_angular_gain` | `1.25` | `1/scale.rotational` |
| | `max_dt` | `ee_twist_max_dt` | `1.0` | 이 값을 넘는 샘플 공백 뒤에는 속도 스파이크를 피해 한 주기 건너뛴다 |
| `replay` | `servo_start_timeout` | `servo_start_timeout` | `30.0` | `/servo_node/start_servo` 대기 시간 |

블록 이름이 비어 있는 행은 **바로 위 블록에 속한다** (markdown 은 셀 병합이 없다).

`replay.ee_twist.*` / `servo_start_timeout` 은 `arm_path: ee_twist` 일 때만 의미가
있다. 경로별 특성 비교와 함정은
[docs/collect/replay_mock_stack_guide.md](../../../docs/collect/replay_mock_stack_guide.md).

**없는 블록이 정보다.** `camera` / `image_recorder` / `session_control` 블록이 없는
것은 본 런치가 그 노드를 띄우지 않는다는 뜻이다 (재생 도구가 해당 토픽을 공급).
공유 YAML 시절에는 그 키들이 존재하면서 조용히 무시되었다.

`replay_arm_path` 는 `--show-args` 에서 사라졌지만, 값이 잘못되면 launch 가
`RuntimeError` 로 즉시 거부하므로 오타가 조용히 넘어가지는 않는다.

```
'replay_arm_path' must be one of ['target_joint_cmds', 'ee_twist', 'none'], got 'bogus'
```

### 3.4 `teleop_mirror` — YAML ↔ 인자 대응

설정 파일은 **전용** `config/teleop_mirror.yaml` 이다.
leader pose 발행자는 **본 launch 가 만들지 않는다** (외부 어댑터 담당).

| YAML 블록 | 키 | launch argument | 현재 YAML 값 | 비고 |
|---|---|---|---|---|
| `leader` | `pose_topic` | `leader_pose_topic` | `/leader/ee_pose` | 외부 어댑터가 발행한다 |
| `follower` | `pose_topic` | `follower_pose_topic` | `/ee_pose` | 팔로워 스택의 `ee_pose_publisher` |
| | `target_pose_topic` | `target_pose_topic` | `/follower/target_pose` | retarget 결과 |
| | `base_frame` | `follower_base_frame` | `panda_link0` | target pose / twist 의 `frame_id` |
| `retarget` | `position_scale` | `position_scale` | `1.0` | leader→follower 변위 배율 |
| | `align_roll` | `align_roll` | `0.0` | `R_align` (rad). leader base → follower base 자세 정렬 |
| | `align_pitch` | `align_pitch` | `0.0` | |
| | `align_yaw` | `align_yaw` | `0.0` | `pi` 면 x/y 가 뒤집혀 거울 매핑 |
| | `lpf_cutoff_hz` | `lpf_cutoff_hz` | `3.0` | 저역통과 차단주파수. `<=0` 이면 비활성 |
| | `watchdog_timeout` | `watchdog_timeout` | `0.5` | leader 스트림이 이만큼 끊기면 자동 disengage (초) |
| | `max_sample_jump` | `max_sample_jump` | `0.2` | 샘플 간 이동이 이보다 크면 자동 disengage (m) |
| | `workspace_min` | `workspace_min` | `[]` | 워크스페이스 박스 `[x, y, z]`. **min/max 둘 다** 채워야 적용 |
| | `workspace_max` | `workspace_max` | `[]` | |
| `clutch_pedal` | `enabled` | `enable_clutch_pedal` | `false` | `python3-evdev` + 실제 장치가 없으면 노드가 기동 실패 |
| | `device_path` | `pedal_device_path` | `''` | 비우면 이름 탐색. `/dev/input/by-id/...` 권장 |
| | `device_name` | `pedal_device_name` | `pedal` | 장치 이름 **부분 문자열** (대소문자 무시) |
| | `key_code` | `pedal_key_code` | `''` | 비우면 그 장치의 **아무 키나** 페달로 취급 |
| | `mode` | `pedal_mode` | `hold` | `hold` = 데드맨(권장) / `toggle` = 전환 |
| | `grab` | `pedal_grab` | `false` | 장치 독점(`EVIOCGRAB`) |
| | `timeout` | `pedal_timeout` | `0.0` | 하트비트 감시. **런치가 자동 보정한다** — 아래 |
| `twist` | `topic` | `twist_topic` | `/servo_node/delta_twist_cmds` | MoveIt Servo 입력 |
| | `linear_gain` | `twist_linear_gain` | `2.5` | `1/scale.linear` (servo `command_in_type: unitless`) |
| | `angular_gain` | `twist_angular_gain` | `1.25` | `1/scale.rotational` |
| *(YAML 밖)* | — | `config_file` | `<rdfp share>/config/teleop_mirror.yaml` | 다른 YAML 지정 |

블록 이름이 비어 있는 행은 **바로 위 블록에 속한다** (markdown 은 셀 병합이 없다).

argument 이름은 YAML 키와 1:1 이 아니다 — `clutch_pedal.*` 는 `pedal_*` 접두사가
붙고 (`enabled` 만 `enable_clutch_pedal`), `follower.base_frame` 은
`follower_base_frame`, `twist.topic` 은 `twist_topic` 이 된다. 접두사가 생기는
이유는 `base_frame` / `topic` 같은 이름이 다른 launch 의 공통 인자와 충돌하기
때문이다.

`pedal_timeout` 은 launch 가 `enable_clutch_pedal` / `pedal_mode` 를 보고 값을
**자동으로 맞춘다.** 데드맨은 페달 노드와 하트비트 감시 **두 조각**이 함께 있어야
성립하는데, 한쪽만 켠 구성은 조용히 잘못 동작하기 때문이다.

| 입력 | 적용값 | 이유 |
|---|---|---|
| `enable_clutch_pedal:=true`, `hold`, 미지정 | **0.3** | 데드맨 아닌 채로 뜨는 것을 방지 |
| `enable_clutch_pedal:=true`, `hold`, `0.5` | 0.5 | 명시값 존중 |
| `pedal_mode:=toggle`, `0.3` | **0.0** | toggle 은 하트비트를 안 보내 engage 즉시 풀린다 |
| `enable_clutch_pedal:=false`, `0.3` | 0.3 | 페달 노드를 따로 띄우는 구성 |

실제 적용값은 `ros2 param get /teleop_retarget pedal_timeout` 으로 확인한다
(위 4행 모두 실측 확인됨). `pedal_mode` 에 `hold`/`toggle` 외의 값을 주면 정합
로직이 조용히 흘러 데드맨이 사라지므로, launch 가 `RuntimeError` 로 거부한다.

### 3.5 `rdfp_collect`

이미지 파이프라인 설정만 쓰므로 `config_file` 이 곧 `image_pipeline.yaml` 이다.
**백엔드를 주면 카메라 토픽·형식·`use_sim_time` 이 프로파일에서 온다** (§2 참조).

```bash
ros2 launch rdfp rdfp_collect.launch.py config_file:=$HOME/my_camera.yaml
ros2 launch rdfp rdfp_collect.launch.py backend:=functionbay
```

| 인자 | 출처 | 비고 |
|---|---|---|
| `log_level` | 하드코딩 `info` | |
| `camera_*` 9종 | `image_pipeline.yaml` | `backend` 가 `camera_image_topic` 을 덮는다 |
| `enable_image_viewer` | `image_pipeline.yaml` | |
| `enable_image_recorder` / `_fps` / `_output_dir` | `image_pipeline.yaml` | 레코더를 **켜고 끈다** |
| `image_recorder_input_format` / `camera_compressed_topic` | `backend` | 압축 카메라용 |
| `backend` / `arm_cmd_source` / `target_joint_cmds_input_topic` / `arm_controller_node_name` / `use_sim_time` | — | arm 채널·시계 |
| `config_file` | — | |

카메라 노드는 띄우지 않는다 — **argument 만** 재사용해 토픽 이름을 맞춘다.

> `rdfp_camera_node` / `rdfp_image_viewer_node` 는 **`rdfp` 패키지 소속**이다
> (`rdfp/camera/`). `/session` 상태에 종속되는 수집 계층 노드라 제어 계층에서
> 옮겨 왔으므로, 런치의 `package=` 는 `robot_control` 이 아니라 `rdfp` 다.

---

## 4. Launch × Helper 의존 관계

helper 는 전부 `robot_control.launch_helpers.*` 다. `image_pipeline` 열의 ⭕ 는
`camera` 를 통한 **간접** 의존이다.

| launch | common | controller | controller_startup | camera | image_pipeline | ee_pose | gripper | scene |
|---|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| `replay_panda_mock` | ✅ | ✅ | ✅ | | | | | |
| `rdfp_collect` | | | | | ✅ | | | |
| `teleop_mirror` | | | | | | | | |

`teleop_mirror` 는 helper 를 쓰지 않는다 — 자체 YAML 로더로 모든 인자를 선언한다.

---

## 5. 실행 구조와 순차 기동

**묶음 launch 는 전부 없앴다.** 제어 스택과 수집 계층을 런치 둘로 나눠 띄운다.

```text
panda_mock.launch.py          (robot_control — 제어 스택)
rdfp_collect.launch.py        (rdfp — 수집 노드 묶음, 별도 터미널)
 ├─ session_control_node
 ├─ rdfp_image_viewer_node
 ├─ rdfp_image_recorder | image_recorder_node   (recorder 인자가 고른다)
 └─ target_joint_cmds_publisher
```

```text
replay_panda_mock.launch.py
 ├─ ros2_control 스택 (joint_state_broadcaster 포함 — 재생 결과 관측용)
 ├─ move_group / servo_node / rviz2
 ├─ gripper_action_node              (항상)
 ├─ rdfp_image_viewer_node            (enable_image_viewer)
 └─ arm 어댑터 — replay_arm_path 로 택일
     ├─ ee_twist_publisher + servo_auto_start (ee_twist, 기본)
     ├─ target_joint_cmds_executor            (target_joint_cmds)
     └─ (없음)                                 (none)
```

순차 기동 정책(`ros2_control_node` → `joint_state_broadcaster` →
`panda_arm_controller` → `panda_hand_controller` → `post_hand_actions`)은
`robot_control.launch_helpers.controller_startup` 에 있다. 각 launch 의
`post_hand_actions`:

| launch | post_hand_actions |
|---|---|
| `replay_panda_mock` | `move_group`, `servo_node`, `rviz2`, `rdfp_image_viewer_node`, `gripper_action_node`, arm 어댑터 |

---

## 6. 어느 launch 를 쓸까

| 하려는 것 | launch |
|---|---|
| 학습 데이터 수집 전체 스택 | `panda_mock` + `rdfp_collect backend:=mock` |
| 위의 JGPC 판 | `panda_mock_jgpc` + `rdfp_collect backend:=mock_jgpc` |
| Gazebo 백엔드로 수집 | `panda_gazebo` + `rdfp_collect` (인자 4개, §6.1) |
| Isaac Sim 백엔드로 수집 | `panda_isaac` + `rdfp_collect backend:=isaac` |
| **제어 스택은 그대로 두고 수집만 재시작** | `rdfp_collect.launch.py` (§6.1) |
| 적재된 에피소드 재생 | `replay_panda_mock.launch.py` |
| leader-follower teleop | `teleop_mirror.launch.py` (follower 스택이 이미 떠 있어야 함) |
| 세션·뷰어·녹화만 (제어 스택 위에) | `rdfp_collect.launch.py` |
| **로봇 제어만** | `ros2 launch robot_control panda_mock.launch.py` |

### 6.1 묶음 launch ↔ 분리 조합 대응

같은 결과를 두 방식으로 얻을 수 있다. 아래 대응은 **실측으로 검증했다** — 격리된
`ROS_DOMAIN_ID` 에서 양쪽을 띄우고 노드 목록·수집 노드 4종의 전체 파라미터·토픽
엔드포인트를 덤프해 diff 했다.

> ⚠️ **레코더만은 같지 않다.** 삭제된 번들 셋(mock · mock_jgpc · isaac)은 **서비스 구동**
> `image_recorder_node` 를 썼고, `rdfp_collect` 는 **세션 구동** `rdfp_image_recorder`
> 만 띄운다 — `/session` 이 IN_EPISODE 인 구간만 담는다. 아래 "완전 일치" 는 노드
> 그래프를 대조한 시점(2026-09-12, `recorder` 인자가 있던 때)의 것이다. 그 인자는
> 2026-09-13 에 없앴다 — 서비스 구동 레코더의 start/stop 을 부르는 production 코드가
> 없었기 때문이다. **새로 수집하는 데는 세션 구동이 맞다.**

| 묶음 | 동등한 분리 조합 | 검증 |
|---|---|---|
| ~~`rdfp_panda_mock`~~ (삭제됨) | `panda_mock` + `rdfp_collect backend:=mock` | 노드 그래프 일치 ✅ (레코더는 세션 구동으로 바뀐다) |
| ~~`rdfp_panda_mock_jgpc`~~ (삭제됨) | `panda_mock_jgpc` + `rdfp_collect backend:=mock_jgpc` | 〃 |
| ~~`rdfp_panda_gazebo`~~ (삭제됨) | `panda_gazebo` + `rdfp_collect` (인자 필요, 아래) | **인자를 맞춰야 한다** |
| ~~`rdfp_panda_isaac`~~ (삭제됨) | `panda_isaac` + `rdfp_collect backend:=isaac` | **인자 없이 동등** — 해상도·fps 는 `isaac_scene.json` 에서 따라온다 |
| ~~`rdfp_panda_functionbay`~~ (삭제됨) | `panda_functionbay` + `rdfp_collect backend:=functionbay`. **시뮬레이터가 먼저 떠 있어야 한다** | 압축 배선·해상도·fps 전부 프로파일이 준다. `fps: 10` 은 시뮬레이터 카메라 XML 의 `Publish_Frequency` 와 같은 값이다(실측 9.54~9.60 Hz — mp4 시간축이 약 4 % 짧고, 프레임별 시각은 sidecar stamp 가 맞다). 2026-09-17 까지는 5 여서 영상이 1.9배 느리게 재생됐다 |

> **`enable_image_viewer` 는 뷰어와 `camera_republish` 를 함께 켠다** (2026-09-08).
> 시뮬레이터가 `sensor_msgs/CompressedImage` 로 발행하고 뷰어는 `sensor_msgs/Image` 만
> 구독하므로, `image_transport/republish` 가 중간에서 raw(`camera_image_topic`)로
> 되살린다. 그 노드가 없으면 **타입이 달라 ROS 2 가 연결만 안 하고 오류도 내지 않아**
> 증상이 "빈 창"뿐이다.
>
> **(이력) include 함정은 사라졌다.** 묶음 launch 는 include 한 백엔드에
> `enable_image_viewer:=false` 를 명시적으로 넘겨야 했다 — include 된 launch 가 부모의
> argument 값을 물려받아 자기 기본값으로 되돌아가지 않으므로, 안 넘기면 같은 영상을
> 띄우는 창이 두 개 떴다(오류는 없다). 묶음을 전부 없애면서 include 자체가 사라졌고,
> 이제 두 런치가 각자 자기 뷰어를 켠다.
>
> 살아남은 계약은 제어 계층 안에 있다 — 펑션베이의 raw 발행자는 `enable_camera` 가
> 켜고 그 기본값이 raw 소비자에서 파생된다
> (`rdfp/camera/tests/test_functionbay_camera_wiring.py` 가 고정한다).

```bash
# JTC
ros2 launch robot_control panda_mock.launch.py
ros2 launch rdfp rdfp_collect.launch.py backend:=mock

# JGPC
ros2 launch robot_control panda_mock_jgpc.launch.py
ros2 launch rdfp rdfp_collect.launch.py backend:=mock_jgpc
```

> 등가는 **묶음 launch 를 재현할 때**의 이야기다. 새로 수집한다면 `rdfp_collect` 의
> 기본값(세션 구동 레코더)이 낫다 — `/session` 이 `IN_EPISODE` 인 구간만 자동으로
> 담고 `.jsonl` 사이드카와 `metadata.json` 을 함께 쓴다.

#### Gazebo 조합은 인자를 명시해야 한다

Isaac 도 **`image_pipeline.yaml` 을 읽지 않는다** — `backend:=isaac` 이 대신
`isaac_scene.json` 의 `camera` 블록을 읽는다. 이미지를 카메라 노드가 아니라
**시뮬레이터가 직접** 발행하므로 해상도·주파수를 정하는 쪽이 Isaac 이기 때문이다.
`rdfp_collect` 로 분리해 쓰려면 네 인자를 맞춰야 한다.

```bash
ros2 launch robot_control panda_isaac.launch.py enable_gripper:=true enable_scene:=true
ros2 launch rdfp rdfp_collect.launch.py use_sim_time:=true \
    arm_cmd_source:=joint_state \
    target_joint_cmds_input_topic:=/isaac/arm_command \
    camera_image_topic:=/isaac/camera/image_raw \
    camera_resolution:=640x480 image_recorder_fps:=10
```

`arm_cmd_source:=joint_state` 는 **컨트롤러의 `joints` 파라미터를 조회하지 않는다** —
메시지에 이름이 이미 있기 때문이다. ros2_control 컨트롤러가 없는 Isaac 스택에서
중요한 성질이며, 기본값(`joint_trajectory`)으로 두면 조회가 실패해 `JointState.name`
이 빈 채로 적재된다.

삭제된 `rdfp_panda_gazebo` 는 **`image_pipeline.yaml` 을 읽지 않고 자체 기본값을 썼다.**
그 값들이 백엔드에 묶여 있기 때문이며, 그래서 분리 조합에서는 손으로 맞춰야 한다 — 해상도는 `description/panda.gazebo.xacro`
의 gz 카메라 센서(`width:=640 height:=480`)를 따르고, 뷰어·레코더가 기본 off 인
것은 `simulate_camera` 기본이 `false` 라 카메라 토픽 자체가 없기 때문이다.

| 인자 | Gazebo 에 맞는 값 | `rdfp_collect` 기본 (YAML) |
|---|---|---|
| `camera_resolution` | `640x480` | `1280x720` |
| `image_recorder_auto_start` | `false` | `true` |
| `enable_image_viewer` | `false` | `true` |
| `enable_image_recorder` | `false` | `true` |
| `camera_image_topic` · `image_recorder_fps` · `image_recorder_output_dir` · `target_joint_cmds_input_topic` | — | 동일 ✅ |

그대로 조합하면 뷰어·레코더가 켜진 채 뜨고 해상도가 어긋난다. 동등하게 맞추려면:

```bash
ros2 launch robot_control panda_gazebo.launch.py
ros2 launch rdfp rdfp_collect.launch.py use_sim_time:=true \
    camera_resolution:=640x480 \
    enable_image_viewer:=false enable_image_recorder:=false
```

`use_sim_time:=true` 는 필수다. Gazebo 백엔드는 `/clock` 을 쓰므로 수집 노드만
벽시계를 쓰면 타임스탬프가 어긋나 데이터셋이 틀어진다.

> Gazebo 쌍은 이 저장소의 검증 환경에 `ros_gz_sim` / `ign_ros2_control` 이 없어
> **백엔드 실기동 없이** 확인했다 — 수집 절반은 `rdfp_collect` 단독 기동으로
> 실측하고(4개 노드 정상, `use_sim_time` 전파 확인), 백엔드 쪽은 소스 대조로
> 확인했다.

---

## 7. 유지보수 메모

- **helper 를 이 디렉터리에 만들지 않는다.** 전부 `robot_control/launch_helpers/`
  로 간다 — 그래야 제어 계열 launch 와 공유된다. 예전의 sibling
  `sys.path.insert(0, os.path.dirname(__file__))` 패턴은 제거되었고, 되살리면
  패키지 경계를 넘는 import 가 다시 불가능해진다.
- **`Node(package=...)` 를 정확히 적는다.** 이 계열 launch 는 `robot_control` 의
  실행파일(`rdfp_image_viewer_node`, `target_joint_cmds_publisher`,
  `ee_twist_node`, `gripper_action_node`, `servo_auto_start_node` …)을 자주
  띄운다. 패키지를 틀리면 `--show-args` 는 통과하고 **실제 기동 때만**
  `executable not found` 로 죽는다.
- YAML 계열은 argument 기본값을 손대려면 YAML 을 먼저 확인한다. CLI 에서
  `config_file:=<path>` 또는 개별 `arg:=value` 로 덮어쓸 수 있다.
- **카메라 argument 를 고칠 때는 `robot_control.launch_helpers.image_pipeline`
  하나만 본다.** 여섯 launch 가 이 모듈을 (직접 또는 `camera` 경유로) 쓰므로 한
  곳을 고치면 전부 반영된다. 어느 한 launch 에서만 기본값을 바꾸려 하면 값이 다시
  갈라지므로, 그 launch 에 `config_file` 을 주는 쪽을 택한다.
- **수집 노드를 추가·변경하면 `rdfp_collect` 한 곳만 고치면 된다** (묶음이 사라졌다).
  함께 볼 것은 §6.1 의
  대응표. 묶음 launch 와 `rdfp_collect` 가 어긋나면 §6.1 의 "완전 일치" 가
  거짓이 된다.
- launch 를 추가하면 §1 표와 §4 의존 관계 표를 함께 갱신한다.
