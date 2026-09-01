# 펑션베이 백엔드 설계 — 토픽 연동형 시뮬레이터

`robot_control` 에 펑션베이 시뮬레이터를 백엔드로 붙인 구성이다.
[멀티 시뮬레이터 백엔드 설계안](multi_simulator_backend_design.md) §6.3 의
**"(B) 토픽 브리지"** 방식을 구체화한 첫 사례다.

- 상태: **팔 연동 `현행`** / 그리퍼 `보류` (§6)
- 실행: `ros2 launch robot_control panda_functionbay.launch.py`

> **⚠️ 실제 시뮬레이터 실측에서 이 문서와 어긋난 점이 확인됐다.**
> 특히 **§2.1 은 현실과 반대다** — 시뮬레이터는 `name` 을 채워 보낸다. 또한 실측
> 으로만 드러난 세 가지가 이 문서에 없다 — **중력 처짐**(명령과 실제가 자세에
> 따라 계통적으로 어긋난다), **자기충돌**(시뮬레이터가 막지 않아 MoveIt 으로
> 복구 불가능한 자세에 빠질 수 있다), **servo(twist) 경로 무동작**(출력에 소비자가
> 없고 단순 중계 브리지는 3 사이클 만에 발산한다). 시간 특성(dead time 34 ms ·
> 시정수 133 ms · 시계 오프셋 2.22 s)도 그쪽에만 있다.
> 정정 대기 항목과 남은 작업은 [functionbay_open_work.md](functionbay_open_work.md)
> 에 정리돼 있다. **이 문서를 근거로 작업하기 전에 그쪽을 먼저 읽는다.**

---

## 1. 무엇이 다른가

| | mock / Gazebo | **펑션베이** |
|---|---|---|
| ros2_control 연동 | 하드웨어 플러그인 | **없음 — 토픽만** |
| `controller_manager` | 있음 | **없음** |
| `/joint_states` 발행 | `joint_state_broadcaster` | **`joint_state_fusion` 노드** |
| 기동 완료 신호 | `panda_hand_controller` spawner 의 `OnProcessExit` | **`readiness_gate` 노드의 종료** |
| arm 명령 | `FollowJointTrajectory` 또는 `Float64MultiArray` | **`sensor_msgs/JointState`** |
| 시뮬 시간 | Gazebo 는 `/clock` | **사용 안 함 (벽시계)** |

즉 `panda_mock` 에서 **ros2_control 계층을 통째로 들어낸** 형태이며,
`panda_jgpc_mock` 은 템플릿이 아니다 — 그쪽은 컨트롤러 *타입* 만 바꿨을 뿐
`controller_manager` · spawner · 순차 기동 체인을 그대로 쓴다.

---

## 2. 시뮬레이터가 제공하는 토픽

| 방향 | 토픽 | 타입 | 비고 |
|---|---|---|---|
| 명령 | `/input/panda_joint` | `sensor_msgs/JointState` | 위치 제어. **내부 보간 없음** |
| 보고 | `/output/panda_joint` | `sensor_msgs/JointState` | 50 Hz |
| 보고 | `/output/endeffector` | `std_msgs/Float64MultiArray` | x,y,z,r,p,y — **사용하지 않는다** (§5.3) |
| 명령 | `/input/gripper_joint` | `std_msgs/Float64MultiArray` | 보류 (§6) |
| 보고 | `/output/gripper_joint` | `std_msgs/Float64MultiArray` | 보류 (§6) |
| 센서 | `/camera_image` | `sensor_msgs/Image` | remap 으로 흡수 |

### 2.1 `name` 이 비어 있다 — 순서가 계약이다

`JointState` 를 쓰면서도 `name` 필드를 채우지 않는다. 배열 순서
(`panda_joint1` … `panda_joint7`)가 암묵 규약이다.

**이것이 이 백엔드의 가장 큰 위험이다.** JGPC 의 `Float64MultiArray` 와 같은
성질로, 순서가 어긋나면 에러 없이 엉뚱한 관절이 움직인다. 대응:

- `joint_state_fusion` 이 기동 시 기대 순서를 로그로 남긴다.
- 입력 길이가 `joint_names` 개수와 다르면 **발행하지 않고** 주기적 에러를 낸다.
- 입력이 나중에 `name` 을 채우기 시작하면 **그것을 신뢰**하도록 되어 있다 —
  시뮬레이터 쪽을 고칠 수 있다면 순서 계약이 통째로 사라지므로 **채워 달라고
  요청하는 것이 최선**이다.

---

## 3. 구성

```text
panda_functionbay.launch.py
 ├─ static_tf + robot_state_publisher        (mock 과 동일)
 ├─ joint_state_fusion    /output/panda_joint ─→ /joint_states
 ├─ readiness_gate        첫 보고 대기 후 종료 (exit 0/1)
 └─(OnProcessExit, 종료코드 0 일 때만)
     ├─ move_group / servo_node / rviz2
     ├─ ee_pose_node
     └─ camera_node
```

브리지 노드는 `robot_control/functionbay/` 에 있다.

### 3.1 `joint_state_fusion`

이름 없는 배열에 이름을 부여하고, URDF 가 요구하지만 시뮬레이터가 보고하지 않는
관절을 덧붙인다.

```
/output/panda_joint (7축, 이름 없음)
    → name = [panda_joint1..7]
    → panda_finger_joint1 = 0.04 (고정, §6)
    → /joint_states (8축)
```

`panda_finger_joint2` 는 URDF 에서 `<mimic joint="panda_finger_joint1"/>` 이므로
`robot_state_publisher` 가 파생시킨다 — 채울 필요가 없다.

`stamp_source=auto` 가 기본이다. 입력 stamp 가 0 이면 노드 시계로 채운다.

### 3.2 `readiness_gate`

**첫 메시지를 받으면 종료**한다. 그래야 기존 spawner 와 똑같이 `OnProcessExit`
로 엮여, 백엔드가 달라도 orchestration 코드가 그대로다.

> ⚠️ **종료 코드를 반드시 본다.** `OnProcessExit` 는 실패 종료에도 발동하므로,
> 그냥 엮으면 시뮬레이터가 없어 타임아웃된 뒤에도 `move_group` 이 올라온다.
> 실제로 구현 중 이 버그를 냈고, spawner 체인이 쓰는
> `launch_helpers.controller_startup._chain_or_shutdown` 을 재사용해 고쳤다.

타임아웃(기본 60초) 시 0 이 아닌 코드로 끝나며 런치 전체가 종료된다. 시뮬레이터가
안 떴는데 상위 스택이 올라가 조용히 멈춘 것처럼 보이는 상황을 막는다.

---

## 4. 명령 경로 — 보간이 필수다

펑션베이는 **내부 보간을 하지 않는다.** 받은 관절 위치로 바로 간다. 그런데 MoveIt
의 궤적은 TOTG 로 **~10 Hz 로 리샘플**되므로, 그대로 흘리면 10 Hz 계단 동작이 된다.

`TrajectoryStreamer` 에 `command_format` 을 추가해 `JointState` 발행을 지원한다.

```python
client = create_move_group_client(
    node, mode='jgpc',                                  # auto 금지 — 아래 참고
    arm_command_topic='/input/panda_joint',
    arm_command_joint_names=[f'panda_joint{i}' for i in range(1, 8)],
    arm_command_format='joint_state')
client.move_to_named_target('ready', publish_rate=50.0)
```

### 4.1 실측 — 보간 유무 비교

같은 `move_to_named_target('ready')` 를 두 조건으로 실행한 결과다.

| `publish_rate` | 궤적 point | 발행 명령 | 실측 주기 |
|---|---|---|---|
| 미지정 | 14 | 14 | **9.998 Hz** |
| `50.0` | 14 | **62** | **49.962 Hz** |

CLAUDE.md 에 적힌 "Cartesian 궤적은 ~10 Hz 로 resample 된다"가 그대로 관측된다.
**보고 주기(50 Hz)에 맞춰 `publish_rate=50.0` 을 준다.**

### 4.2 `mode='auto'` 를 쓰지 않는다

`create_move_group_client` 의 auto 판별은 `/panda_arm_controller/commands` 토픽
존재 여부를 본다. 펑션베이 스택에는 그 토픽이 없으므로 **JTC 로 오판**한다.
반드시 `mode='jgpc'` 를 명시한다.

---

## 5. 계약 충족 현황

[백엔드 계약](multi_simulator_backend_design.md) §5 기준이다.

### 5.1 충족

| 항목 | 방법 |
|---|---|
| `/joint_states` (이름 = URDF 와 일치) | `joint_state_fusion` |
| `robot_description` | mock 과 동일 (`build_moveit_config`) |
| TF 트리 `world → panda_link0 → … → panda_hand` | `robot_state_publisher` |
| 기동 완료 신호 | `readiness_gate` (§3.2) |
| `/camera/image_raw` | `camera_image_topic:=/camera_image` remap |

### 5.2 미충족 — 의도적

- **`FollowJointTrajectory` 액션이 없다.** 따라서 `move_group` 의 arm plan &
  execute 가 동작하지 않는다. JGPC 스택과 같은 제약이며, 실행은
  `MoveGroupJgpcClient` 의 명령 스트리밍으로 한다. 계획 · IK · RViz 표시는 정상.
- **그리퍼 액션이 없다.** §6.

### 5.3 `/output/endeffector` 를 쓰지 않는 이유

`/joint_states` 가 정확하면 `ee_pose_node` 가 TF 에서 EE pose 를 계산한다. 별도
경로를 두면 **같은 값의 출처가 둘**이 되어 어긋났을 때 어느 쪽이 맞는지 알 수 없다.
시뮬레이터의 값은 **검증용**으로만 쓴다.

덧붙여 RPY 는 회전 순서와 축 규약(고정축/오일러)을 명시하지 않으면 조용히 틀어진다.
쿼터니언 기반인 `/ee_pose` 를 단일 출처로 두는 편이 안전하다.

---

## 6. 그리퍼 — 보류 (로봇 모델 불일치)

실물은 **Robotiq 2F-85** 인데 스택 전체가 **Franka Panda Hand** 를 전제한다.

| 위치 | 전제 |
|---|---|
| URDF | `panda_finger_joint1` prismatic **0 ~ 0.04 m** |
| SRDF `hand` 그룹 | `open` = 0.035 / `close` = 0 |
| robot_twin 목표표 | `open: 0.04` / `close: 0.0` / `grasp: 0.0 + 30 N` |
| `gripper_control_node` | `control_msgs/GripperCommand` **액션** 클라이언트 |

2F-85 는 **스트로크 85 mm 의 링크 구동식**이라 기구학 자체가 다르다. 손가락 개폐가
직동(prismatic)이 아니라 **회전 관절**이며, 제공된 URDF 기준 `finger_joint` 가
**0 ~ 0.725 rad** 이다. Panda Hand 의 미터 단위 값을 그대로 넘기면 TF · 충돌검사 ·
robot_twin 파지 판정 · 데이터셋 그리퍼 채널이 **에러 없이** 어긋난다.

**그래서 팔 연동을 먼저 완성하고 그리퍼는 분리했다.** 그동안
`panda_finger_joint1` 은 `fb_finger_position`(기본 0.04) 고정값으로 채워 TF 만
성립시킨다 — **실제 그리퍼 상태가 아니다.**

### 향후 작업

1. `panda_arm` + `robotiq_2f_85` xacro 조합으로 URDF 재구성, SRDF `hand` 그룹
   재정의.
2. `rdfp_msgs/GripperCommand` ↔ `/input,/output/gripper_joint` 브리지 노드.
   액션의 `reached_goal` / `stalled` 을 위치 보고만으로 재구성해야 한다
   (목표 대비 오차가 남은 채 정지 = stalled) — robot_twin 의 파지 성공 판정
   근거가 여기에 걸려 있다.
3. robot_twin 설정의 `backend.targets` 를 2F-85 스케일로 재작성.

> **Humble apt 에 Robotiq description 이 없다** (`ros-humble-robotiq-description`
> 미제공). 서드파티 저장소를 vcs 로 가져오거나 직접 작성해야 한다.

---

## 7. 수집 계층 얹기

`rdfp_collect` 를 그대로 얹으면 학습 데이터 수집이 된다. 시뮬 시간을 쓰지 않으므로
`use_sim_time` 조정이 필요 없다 (Gazebo 조합과 다른 점).

```bash
ros2 launch robot_control panda_functionbay.launch.py          # 터미널 1
ros2 launch rdfp rdfp_collect.launch.py arm_cmd_source:=joint_state   # 터미널 2
```

`arm_cmd_source:=joint_state` 가 핵심이다. 펑션베이의 명령은 이미 `JointState`
이므로 `target_joint_cmds_publisher` 가 재stamp 만 해서 `/target_joint_cmds` 로
흘린다. **이 노드를 거치는 이유는 action 채널 토픽 이름을 백엔드 무관하게
고정하기 위해서다** — 그래야 mock / JGPC / 펑션베이 데이터셋을 같은 스키마로
섞을 수 있다.

### 7.1 실측

3계층(시뮬레이터 + 제어 + 수집)을 동시에 띄우고 확인했다.

```
노드: fake_functionbay, joint_state_fusion, robot_state_publisher, move_group,
      servo_node, rviz2, ee_pose_publisher, session_control,
      target_joint_cmds_publisher

/joint_states        49.86 Hz   name = panda_joint1..7 + panda_finger_joint1
/ee_pose             49.98 Hz   frame_id = panda_link0
/target_joint_cmds   49.92 Hz   name = panda_joint1..7
/session             rdfp_msgs/SessionCommand, publisher 1
```

---

## 8. 인자

`--show-args` 가 정상 동작한다 (제어 계열 관례).

| 인자 | 기본값 | 설명 |
|---|---|---|
| `fb_joint_report_topic` | `/output/panda_joint` | 시뮬레이터 관절 보고 |
| `fb_finger_position` | `0.04` | 그리퍼 미연동 구간의 `panda_finger_joint1` 고정값(m) |
| `fb_ready_timeout` | `60.0` | 첫 보고 대기 한도(초). 초과 시 런치 실패 |

그 외 `log_level` · `base_frame` · `ee_frame` · `publish_rate` · `camera_*` 는
제어 계열 공통이다 —
[robot_control launch README §2](../../src/robot_control/launch/README.md).

---

## 9. 미결 항목

> 실측으로 갱신된 최신 목록과 우선순위는 [functionbay_open_work.md §8](functionbay_open_work.md) 에 있다.

| 항목 | 내용 |
|---|---|
| 그리퍼 | §6. Robotiq description 확보가 선행 |
| 시뮬레이터 `name` 채우기 | 벤더가 채워 주면 §2.1 의 순서 계약이 사라진다 |
| 실시간 배속 | 시뮬레이터가 실시간보다 느리면 벽시계 타임스탬프와 실제 진행이 어긋나 데이터셋 시간축이 왜곡된다. 배속 확인 필요 |
| scene | `mock_scene_state_node` 는 MoveIt planning scene 폴링이라 시뮬레이터 물체 상태를 반영하지 않는다. 물체를 다루려면 `functionbay_scene_state_node` 가 필요 |
| 카메라 | `/camera_image` remap 은 설계상 가능하나 **미실측** (`camera_info` 미제공 영향 포함) |

---

## 참고

- [펑션베이 — 남은 작업 (인수인계)](functionbay_open_work.md) — 실측 결과 · 중력 처짐 · 자기충돌 · servo 경로 · 시간 특성 · 이 문서의 정정 대기 항목
- [멀티 시뮬레이터 백엔드 설계안](multi_simulator_backend_design.md) — §5 백엔드 계약, §6.3 연결형 백엔드
- [robot_control launch README](../../src/robot_control/launch/README.md)
- [MoveGroupJgpcClient 사용 가이드](../moveit/MoveGroupJgpcClient_UserGuide.md) — 스트리밍 실행, `publish_rate`
- [rdfp launch README §6.1](../../src/rdfp/launch/README.md) — 수집 계층 조합
