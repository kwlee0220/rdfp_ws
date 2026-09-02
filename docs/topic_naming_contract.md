# 토픽 이름 규약

**논리 채널마다 이름을 하나로 고정한다.** 백엔드(mock · Isaac · 펑션베이 · Gazebo)가
무엇이든 상위 계층이 보는 이름은 같아야 한다.

기준은 **`panda_mock` 이 쓰는 이름**이다. 가장 오래 쓰였고 데이터셋 적재 경로가 그
이름으로 굳어 있다(`config/recording_topics.list`).

> 이 문서는 **1단계(이름 고정)** 의 결정 기록이다. 2단계(로봇별 네임스페이스)는
> §5 에 설계만 적어 두고 구현하지 않았다.

---

## 1. 왜 필요한가

지금은 같은 채널이 백엔드마다 다른 이름을 쓴다.

| 논리 채널 | mock | Isaac | 펑션베이 |
|---|---|---|---|
| arm 명령 | `/panda_arm_controller/commands` | `/isaac/arm_command` | `/input/panda_joint` |
| 카메라 | `/camera/image_raw` | `/isaac/camera/image_raw` | `/camera_image` |
| 관절 상태 | `/joint_states` | `/joint_states` | `/output/panda_joint` |

**백엔드 이름이 토픽에 새어 나와 있다** — `/isaac/…`, `/input`·`/output`. 그러면
상위(수집·트윈·teleop)가 백엔드를 알아야 하고, 백엔드를 바꿀 때마다 설정이 갈라진다.

**새는 방식이 하나 더 있다 — 구현 노드 이름.** 그리퍼 채널이 그랬다. 노드가 `~/` 상대로
발행해 `/gripper_control/gripper_cmds` 가 되어 있었는데, `gripper_control` 은 논리 채널이
아니라 **그때 그 노드의 이름**이다. 노드를 갈면(`gripper_control_node` → `GripperActionNode`)
채널 이름이 따라 바뀌어 녹화 목록과 트윈 설정이 함께 깨진다. 백엔드 이름이 샌 것과 증상은
같고, 고치는 방법도 같다 — 루트 상대로 두는 것이다(§2.2).

실제로 그렇게 갈라진 흔적이 있다. `rdfp_panda_isaac.launch.py` 는 카메라 토픽을
`isaac_scene.json` 에서 읽어 넘기고, `rdfp_collect.launch.py` 로 분리해 쓰려면 인자
네 개를 손으로 맞춰야 한다.

---

## 2. 정규 이름

**상대 경로로 적는다.** 앞에 `/` 를 붙이지 않는다 — 그래야 2단계에서 네임스페이스가
자동으로 붙는다(§5).

### 2.1 관측 (observation)

| 채널 | 정규 이름 | 타입 | 비고 |
|---|---|---|---|
| 관절 상태 | `joint_states` | `sensor_msgs/JointState` | ROS 표준 이름. 바꾸지 않는다 |
| 엔드이펙터 자세 | `ee_pose` | `geometry_msgs/PoseStamped` | **단수** — 한 시점의 값 하나다 |
| 카메라 이미지 | `camera/image_raw` | `sensor_msgs/Image` | `image_transport` 규약 |
| 카메라 압축 이미지 | `camera/image_compressed` | `sensor_msgs/CompressedImage` | |
| 카메라 정보 | `camera/camera_info` | `sensor_msgs/CameraInfo` | `image_raw` 와 짝 |
| 그리퍼 상태 | `gripper_states` | `rdfp_msgs/GripperState` | **연속 상태.** `/joint_states` 로 대신할 수 없다 — §2.2 |
| scene 물체 | `scene/objects` | `rdfp_msgs/SceneObjects` | **복수** — 메시지가 실제로 배열이다 |

> **단수/복수는 메시지 모양을 따른다.** `ee_pose` 는 값 하나라 단수, `scene/objects` 는
> `SceneObject[]` 를 담으므로 복수다. `camera_images` 처럼 채널이라는 이유로 복수를
> 쓰지 않는다 — 그러면 `SceneObjects` 와 규칙이 어긋난다.

### 2.2 그리퍼 — 채널은 둘, 이름은 루트 상대

| 정규 이름 | 타입 | 방향 |
|---|---|---|
| `gripper_cmds` | `rdfp_msgs/GripperCommand` | 명령 |
| `gripper_states` | `rdfp_msgs/GripperState` | 관측 |

**규약이 아는 그리퍼 채널은 이 둘뿐이다.** 필드의 뜻(개구 폭 단위, `stalled` 와
`at_goal` 의 차이, 판정 주체)은 [GripperNode_Design.md](gripper/GripperNode_Design.md)
§2 가 정본이다 — 이름 규약 문서가 함께 설명하면 두 문서가 갈라진다. 아래는 **이름에
관한 결정**만 적는다.

#### 왜 `/joint_states` 로 대신할 수 없나

손가락 관절이 `/joint_states` 에 실리므로 전용 채널이 불필요해 보이지만, **백엔드마다
사정이 다르다.**

| 백엔드 | `/joint_states` 의 손가락 |
|---|---|
| mock · Isaac | 실제 값 |
| 펑션베이 | **TF 성립용 고정값** — 없는 것보다 나쁘다 |

펑션베이는 `0.04` 를 주입해 **"항상 열려 있다"고 거짓말**을 하고, 데이터를 열어봐도
드러나지 않는다. 그래서 별도 채널을 두고, 값을 실을 수 없는 스택은 `width` 를 `NaN`
으로 둔다 — "모른다"가 사실대로 남는다.

#### `~/` 가 아니라 루트 상대다 (2026-09-02)

전에는 노드가 `~/gripper_cmds` 로 발행해 실제 이름이 `/gripper_control/gripper_cmds`
였다. 지금은 `gripper_cmds` — **루트 상대**다.

`~/` 는 네임스페이스가 아니라 **노드 이름**을 접두사로 붙인다. 그래서 두 가지가 어긋난다.

1. **채널이 구현에 묶인다.** `gripper_control` 은 논리 채널이 아니라 그때 그 노드의
   이름이라, 노드를 갈아치우면 채널 이름이 따라 바뀐다. 실제로 `gripper_control_node`
   를 `GripperActionNode` 로 교체하면서 녹화 목록·트윈 설정·DB 주석이 함께 바뀌었다.
2. **§5 의 네임스페이스가 통하지 않는다.** 루트 상대여야 `PushRosNamespace` 로
   `/abc/gripper_cmds` 가 된다. `~/` 는 노드명에 묶여 있어 그 경로를 타지 못한다.

**servo 예외(§2.4)와 헷갈리지 않는다.** `servo_node/delta_twist_cmds` 도 노드 이름을
달고 있지만 그쪽은 *우리가 짓지 않은 외부 규약*이고 모든 백엔드에서 같다. 그리퍼는
우리 노드이고 구현마다 갈렸다 — 남길 이유가 없다.

#### 폐기 기록 — `gripper_action_states` (2026-09-02)

`GripperActionState` 와 `/gripper_control/gripper_action_states` 는 **삭제됐다.**
`GripperState` 와 필드가 겹쳤고, 남길 근거로 들었던 둘이 모두 해소됐기 때문이다.

| 남길 근거였던 것 | 어떻게 해소됐나 |
|---|---|
| 트윈의 명령 완료 신호 (주기 발행인 `gripper_states` 로는 "갱신됨"이 "끝남"을 뜻하지 않는다) | `at_goal` 이 **판정을 값 안에 담는다.** 갱신 여부를 볼 필요가 없어졌다 |
| Isaac 의 유일한 파지 지표 (`stalled = not reached`) | 해소되지 않았다 — 아래 |

**남은 부채: Isaac 의 파지 여부가 데이터에서 빠져 있다.** `GripperActionNode` 의
`stalled` 가 미구현이라 `at_goal` 이 `grasp` 에서 늘 `false` 다. 파지 시 effort 를
실측해 임계값을 잡는 것이 선행 작업이다
([GripperNode_Design.md](gripper/GripperNode_Design.md) §4).

> **학습 신호로서는 잃은 것이 없다.** action 은 `gripper_cmds`(의도)가, 관측은
> `gripper_states` 가 담는다. 사라진 `reached_goal`/`status` 는 진단값이었다.
>
> **mock 은 애초에 파지를 관측할 수 없다** — planning scene 물체에 물리가 없어 어느
> 채널이든 `stalled` 가 무의미하다.

### 2.3 명령 (action)

| 채널 | 정규 이름 | 타입 | 비고 |
|---|---|---|---|
| EE twist | `servo_node/delta_twist_cmds` | `geometry_msgs/TwistStamped` | **예외** — §2.4 |
| 관절 jog | `servo_node/delta_joint_cmds` | `control_msgs/JointJog` | **예외** — §2.4 |
| 목표 관절값 | `target_joint_cmds` | `sensor_msgs/JointState` | **데이터셋의 action 채널** |
| 목표 EE 자세 | `target_ee_pose` | `geometry_msgs/PoseStamped` | **단수**. teleop retarget 결과 |
| 그리퍼 명령 | `gripper_cmds` | `rdfp_msgs/GripperCommand` | **심볼만 싣는다** — §2.2 |

### 2.4 예외 — servo 입력 두 개는 노드 이름을 달고 있다

`servo_node/delta_twist_cmds` 와 `servo_node/delta_joint_cmds` 만 **구현체 이름이
토픽에 들어 있다.** 다른 정규 이름은 그렇지 않다 — `ee_pose` 이지
`ee_pose_publisher/ee_pose` 가 아니다.

**원칙대로면 `delta_twist_cmds` 로 줄이는 것이 맞다.** 그런데도 두는 이유가 셋이다.

1. **우리가 지은 이름이 아니다.** `~/delta_twist_cmds` 는 moveit_servo 의 기본값
   (`cartesian_command_in_topic`)이고, MoveIt 문서·튜토리얼·예제가 모두
   `/servo_node/delta_twist_cmds` 로 쓴다. 바꾸면 외부 자료와 어긋난다.
2. **이미 네임스페이스 친화적이다.** `~/` 라 노드 상대이므로 2단계에서
   `/abc/servo_node/delta_twist_cmds` 로 저절로 따라온다 — 절대 경로 문제가 **없는**
   몇 안 되는 채널이다.
3. **지금은 이름이 사실이다.** 세 백엔드 모두 twist 를 servo 가 받는다.

`/isaac/…` 이나 `/input/…` 을 정리하는 것과는 이득이 다르다. 그쪽은 **백엔드마다
이름이 갈라져** 상위가 백엔드를 알아야 했지만, `servo_node/` 는 모든 백엔드에서 같다.

> **바꿔야 하는 조건** — twist 를 **servo 가 아닌 것이 소비하는** 백엔드가 생기면
> 이름이 거짓이 된다. 그때는 바꾼다. 단, **2단계(네임스페이스) 전에** 해야 한다 —
> 나중에 하면 네임스페이스가 붙은 상태에서 이름까지 바뀌어 회귀 추적이 어려워진다.

바꿀 때의 비용은 조사해 뒀다 — 코드·설정 12곳, **문서 17곳**. DB 는 깨지지 않는다:
적재는 토픽 이름이 아니라 **메시지 타입**으로 writer 를 고르고
(`resolve_message_type`), 이름으로 특별 취급되는 것은 `session` 하나뿐이다.

### 2.5 제어 (수집되지 않음)

| 채널 | 정규 이름 | 비고 |
|---|---|---|
| 세션 상태 | `session` | `TRANSIENT_LOCAL` |
| scene 리셋 | `scene/reset` | **서비스** (`rdfp_msgs/srv/ResetScene`) |

> **그리퍼의 하드웨어 접점은 규약에 없다.** mock 은
> `panda_hand_controller/gripper_cmd` 액션을 부르고, 펑션베이는 관절 토픽을 쓰며,
> Isaac 은 브릿지를 거친다 — arm 명령을 규약에서 뺀 것과 **같은 이유**다(§3).
> 그 접점을 아는 것은 `GripperNode` 구현체 하나뿐이고, 상위 계층이 보는 것은
> `gripper_cmds` / `gripper_states` 둘뿐이다.

---

## 3. arm 명령은 **규약이 아니다**

로봇을 실제로 움직이는 채널은 스택 구현마다 다르다.

| 스택 | 채널 | 타입 |
|---|---|---|
| mock (JTC) | `panda_arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` |
| mock (JGPC) | `panda_arm_controller/commands` | `std_msgs/Float64MultiArray` |
| Isaac | `isaac/arm_command` | `sensor_msgs/JointState` |
| 펑션베이 | `input/panda_joint` | `sensor_msgs/JointState` |

**이것을 통일하지 않는다.** 컨트롤러 유무와 메시지 타입이 근본적으로 다르고, 통일하려면
어느 한쪽에 억지 어댑터를 넣어야 한다.

대신 **`target_joint_cmds` 가 그 위의 정규 채널**이다. `target_joint_cmds_publisher` 가
스택별 명령을 이 하나로 변환해 발행하고, 데이터셋은 그것만 본다.

```
스택 고유 arm 명령  ──▶ target_joint_cmds_publisher ──▶ target_joint_cmds (정규)
```

> **채워지는지는 백엔드가 결정한다.** MoveIt 액션으로 움직인 구간은 명령 토픽에
> 아무것도 흐르지 않아 `target_joint_cmds` 가 비고, 스트리밍 방식(Isaac·JGPC)은
> 1:1 로 채워진다. 실측 — Isaac 에서 `/target_joint_cmds` 277건 =
> `/isaac/arm_command` 277건, mock JTC 에서는 0건.

---

## 4. remap 은 **백엔드 경계에서만** 한다

```
[시뮬레이터 고유 이름]  ──remap──▶  [정규 상대 이름]  ──▶  상위 계층
 isaac/camera/image_raw             camera/image_raw
 output/panda_joint                 joint_states
```

- remap 은 **백엔드 launch 안에서만** 쓴다.
- 상위 계층(수집·트윈·teleop·검사 스크립트)은 **정규 이름만** 안다.
- 노드 코드에는 **절대 경로를 쓰지 않는다.** 상대로 두면 remap 도 네임스페이스도 모두
  통한다.

### 지금 어긋나 있는 곳

| 위치 | 문제 |
|---|---|
| `image_pipeline.yaml` 의 `image_topic` 등 | 기본값이 절대(`/camera/image_raw`) |
| `ARM_COMMAND_TOPIC` 류 상수 | 절대. 백엔드 이름이 박혀 있다 |
| `scripts/isaac/is_check_*.py` | 절대 이름 11종 |
| `robot_twin/config/*.yaml` | 절대 이름 8종 |
| `config/recording_topics.list` | 절대 이름 11종 |

**노드 계층은 이미 대부분 상대다** — `arm_command`, `target_joint_cmds`, `session`,
`ee_pose`, `image`, `gripper_cmds`, `gripper_states`. 절대화는 launch 의 remap 이
하고 있다. 그리퍼 둘은 `~/` 였다가 루트 상대로 바꾼 것이다(§2.2).

---

## 5. 2단계 — 로봇별 네임스페이스 (미구현)

정규 이름이 상대이므로, launch 를 감싸는 것만으로 끝난다.

```python
GroupAction([
    PushRosNamespace(LaunchConfiguration('name')),
    *nodes,
])
```

`name` 을 지정하지 않으면 빈 문자열이라 **네임스페이스 없이 현행 그대로** 동작한다.
`name:=abc` 면 `/abc/joint_states`, `/abc/camera/image_raw` 가 된다.

### 그때 따로 정해야 하는 것

| 항목 | 판단 |
|---|---|
| `/clock` | **전역 유지.** 네임스페이스에 들어가면 sim time 이 깨진다 |
| `/tf`, `/tf_static` | 전역 유지 + **프레임 이름에 접두사**(`abc/panda_link0`). URDF·SRDF·MoveIt 설정까지 번지는 **별도 작업**이다 |
| MoveIt · ros2_control | 네임스페이스 안에서 동작하지만 검증 비용이 가장 크다 |
| 소비자(스크립트·트윈·녹화 목록) | 네임스페이스를 주입할 경로가 필요하다 |

**로봇 두 대를 한 도메인에 띄우려면 TF 프레임 접두사가 필수다.** 안 하면 두 로봇의
`panda_link0` 가 같은 트리에 섞여 tf2 가 **에러 없이 틀린 변환**을 낸다.

---

## 6. 적용 순서 (제안)

| 단계 | 내용 | 위험 |
|:-:|---|---|
| 1 | 이 문서 — 이름 확정 | 없음 |
| 2 | Isaac 백엔드만 전환해 실측 검증 (`is_check_phase*.py` 가 회귀를 잡는다) | 낮음 |
| 3 | 나머지 백엔드 + 소비자(스크립트·트윈·녹화 목록) | 중간 |
| 4 | 네임스페이스 (`name` 인자) | 중간 |
| 5 | TF 프레임 접두사 | **큼** — 다중 로봇이 실제로 필요해질 때 |
