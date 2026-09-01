# 펑션베이 백엔드 설계 — 토픽 연동형 시뮬레이터

`robot_control` 에 펑션베이 시뮬레이터를 백엔드로 붙인 구성이다.
[멀티 시뮬레이터 백엔드 설계안](multi_simulator_backend_design.md) §6.3 의
**"(B) 토픽 브리지"** 방식을 구체화한 첫 사례다.

- 상태: **팔 연동 `현행`** / 그리퍼 **채널 `확정`·스택 통합 `보류`** (§6)
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
| 보고 | `/output/panda_joint` | `sensor_msgs/JointState` | 50 Hz. **`name` 이 채워진다** (§2.1) |
| 보고 | `/output/endeffector` | `std_msgs/Float64MultiArray` | x,y,z,r,p,y — **사용하지 않는다** (§5.3). 2026-09-01 현재 **발행되지 않음** |
| 명령 | `/input/gripper_joint` | `std_msgs/Float64MultiArray` | 6축 목표각 [rad] (§6.1) |
| 보고 | `/output/gripper_joint` | `sensor_msgs/JointState` | 30 Hz, 6축 · position/velocity/effort (§6.1) |
| 센서 | `/camera_image` | `sensor_msgs/Image` | remap 으로 흡수 |

### 2.1 `name` — 순서 계약이었으나 해소됐다

**2026-09-01 실측: `/output/panda_joint` 가 `panda_joint1`~`panda_joint7` 을 채워
보낸다.** `/output/gripper_joint` 도 `gripper_joint1`~`6` 을 채운다.

원래 이 백엔드의 가장 큰 위험이 여기 있었다 — `name` 을 비운 채 배열 순서만 계약이면
JGPC 의 `Float64MultiArray` 와 같은 성질이라, 순서가 어긋나도 에러 없이 엉뚱한 관절이
움직인다. `joint_state_fusion` 은 그 상황을 전제로 만들어졌고 **입력이 `name` 을
채우기 시작하면 그것을 신뢰**하도록 되어 있었으므로(`if msg.name:` 분기), 코드 변경
없이 그대로 이득을 본다.

남은 경로는 **입력 방향**이다. `/input/gripper_joint` 는 여전히
`Float64MultiArray` 라 순서가 계약이다 (§6.1).

- `joint_state_fusion` 이 기동 시 이름 출처(메시지 / 파라미터)를 로그로 남긴다.
- 입력 길이가 `joint_names` 개수와 다르면 **발행하지 않고** 주기적 에러를 낸다.

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
/output/panda_joint (7축, 이름 채워짐 — 없으면 파라미터로 보완)
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

### 4.3 명령 setpoint 는 latch 된다 — 발행을 끊어도 자세를 유지한다

**2026-09-01 실측.** 목표를 1초만 보내고 즉시 끊은 뒤 25초를 관찰했다.

```
j2 목표 -0.63398 로 1초 명령 → 중단
  t=0s   -0.63219    t=5s   -0.63063    t=25s  -0.63063
정상상태 오차 +0.00335 rad (0.19°), 이후 변화 없음
```

`ready` 자세에서도 30초간 **0.00000 rad** 로 정지했다. 위치 제어의 setpoint 가
유지되므로 **자세를 지키려고 계속 발행할 필요가 없다.**

> ⚠️ **이전 판(2026-09-01 오전)에는 "명령을 멈추면 중력 평형으로 내려앉는다"고
> 적혀 있었다. 그것은 오진이었고 여기서 철회한다.**
>
> 근거였던 관측은 스크립트 사이 무명령 구간에서 EE 가 40 mm 내려간 것 하나였는데,
> **재현되지 않았고** 위 세 측정과 정면으로 어긋난다. 당시 상황을 보면 **그리퍼가
> 닫힌 채 무언가에 눌려 있었고, 그 직후 그리퍼를 연 시점에 40 mm 가 관측됐다** —
> 접촉이 팔을 받치고 있다가 풀리면서 원래 지령 자세로 내려앉은 것으로 읽는 편이
> 모든 관측과 맞는다. 같은 시기에 "IK 루프가 고착됐다"고 본 것도 하강이 접촉에
> 막힌 것으로 설명된다.
>
> **판정 절차의 교훈은 남는다** — 무반응처럼 보일 때 `joint1`/`joint7`(중력 토크가
> 없는 축)에 단독 명령을 넣어 명령 경로부터 가른다. 접촉·부하 문제와 고착(A-4)은
> 그렇게만 구분된다.

### 4.4 정상상태 오차가 방향에 따라 다르다

중력 보상이 없으므로(A-1) 남는 오차의 **부호가 이동 방향에 따라 뒤집힌다.**

| 이동 | 지령 | 실제 도달 | |
|---|---|---|---|
| EE 상승 | +20 mm | **+10.0 mm** | 중력이 거슬러 **부족** |
| EE 하강 | −20 mm | **−30 mm** | 중력이 도와 **초과** |
| `joint2` 단독 (하강측) | +0.080 rad | +0.0474 rad (59%) | |

**한 방향에서 얻은 보정 계수를 반대 방향에 쓰면 발진한다.** 상승용으로 잔차 2배
과지령을 쓰다가 그대로 하강에 적용해 실제로 진동시켰다(−10 mm 목표에서 ±30 mm 왕복).
보정은 **부호별로 따로 두거나, 잔차 부호가 뒤집히면 게인을 줄이는 적응형**이어야 한다.

또한 한 관절만 지령해도 **나머지 관절이 함께 밀린다.** `joint2` 를 +0.0277 rad 움직였을
때 FK 상 −14.02 mm 여야 할 EE 가 실측 **−1.06 mm** 만 내려갔다 — 유지 지령을 받은
`joint4`·`joint6` 이 하중을 받아 움직여 상쇄했다. **관절 단위 개루프 추론이 성립하지
않으므로, EE 위치는 반드시 측정값 기반 폐루프로 맞춘다.**
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


> **그래서 `/ee_pose` 는 `/output/panda_joint` 에서 계산된 값이다.** 체인은 이렇다 —
> `/output/panda_joint` → `joint_state_fusion` → `/joint_states` →
> `robot_state_publisher` → `/tf` → `ee_pose_publisher`(`lookup_transform`) → `/ee_pose`.
> 노드 구독을 보면 `/tf` 와 `/tf_static` 뿐이며 `/output/endeffector` 는 코드에 등장하지
> 않는다.
>
> 두 가지가 따라온다. (1) **독립 검증이 되지 않는다** — 관절값이 틀리면 EE 도 똑같이
> 틀린다. (2) **기준 모델이 다르다** — 우리 URDF 는 Panda Hand 인데 시뮬레이터는
> Robotiq 2F-85 라 `panda_hand` 프레임과 실제 손끝이 어긋난다(§7 ⑥).

### 5.4 servo 경로 — 브리지로 연결했다 (2026-09-01)

**연결 전에는 모션 키 14개 전부가 무동작이었다.** servo 는 기본값으로
`trajectory_msgs/JointTrajectory` 를 `/panda_arm_controller/joint_trajectory` 에
발행하는데, 이 스택에는 ros2_control 컨트롤러가 없어 **그 토픽의 구독자가 0** 이다.
계산은 정상이고 실행 경로만 끊겨 있어서, 증상이 "키가 안 먹는다"로만 보였다.

```
teleop_keyboard ─ delta_twist_cmds ┐
teleop_retarget ─ ee_twist_node ───┴─> servo_node
                                        │  std_msgs/Float64MultiArray
                                        ↓  /servo_node/commands
                                   servo_command_bridge
                                        │  sensor_msgs/JointState
                                        ↓  /input/panda_joint
                                     시뮬레이터
```

**노드는 새로 만들지 않았다.** Isaac 이 쓰는 `servo_command_bridge_node` 가 토픽을
상대 경로로 두어 백엔드 중립이므로, remap 만 바꿔 그대로 쓴다. 콘솔 스크립트에
백엔드 중립 이름 `servo_command_bridge` 를 더했다 (`isaac_servo_bridge` 도 유지).

`publish_joint_velocities: false` 는 **선택이 아니라 필수**다. `command_out_type` 이
Float64MultiArray 인데 positions 와 velocities 를 모두 발행하도록 두면 servo 의
파라미터 검증이 실패해 **노드가 아예 기동하지 못한다** (JGPC mock · Isaac 과 같은 제약).

실측 (2026-09-01, 브리지 기동 후):

| 입력 | 최대 관절 이동 |
|---|---|
| `j` (+x twist) | 0.11249 rad |
| `q` (+z) / `a` (−z) | 0.07845 / 0.15261 rad |
| `'` / `;` (joint1 jog) | 0.09159 / 0.07937 rad |

> **움직인다는 것과 잘 따라간다는 것은 다르다.** servo 경로의 추종 품질은 여전히
> 미해결이다 (§5.2 · 요청 B-1). 여기서 확인한 것은 **명령이 시뮬레이터에 도달한다**는
> 사실까지다.

---

### 5.5 servo 속도 스케일 — `0.8` 로 정했다 (2026-09-01 실측)

`scale.linear` 은 `unitless` twist 를 m/s 로 바꾸는 계수다. MoveIt 원본값은 `0.4` 인데
**`0.8` 을 기본으로 쓴다.** 세 값을 실측해 고른 결과다.

측정 조건: `ready` 자세에서 `cmd 0.5` 를 1초 유지, `/ee_pose` 변위. 매 측정 전
`ready` 복귀 + `start_servo` 재호출로 상태를 맞췄다.

| | `0.4` (원본) | **`0.8`** | `3.2` |
|---|---|---|---|
| 명령 비례 | ✅ 1.98배 | ✅ **1.93~2.03배** | ❌ **포화** |
| +x 변위 | 25.1 mm | **54.0 mm** | 99.0 mm |
| x/y 균일성 | 25~28 mm | **52.9~55.1 (4% 이내)** | 99 vs 188 — **1.9배 벌어짐** |
| z 비대칭 (아래/위) | 47.0/7.0 = **6.7배** | **72.4/34.7 = 2.09배** | 148/75 = 2.0배 |
| `cmd 1.0` 최대 | 52 mm | **102.5 mm** | 96 mm |

**`3.2` 를 쓰지 않는 이유는 포화다.** 명령을 4배 바꿔도 이동량이 같다 —
`cmd 0.25/0.5/1.0` 에 각각 101.7 / 100.7 / 96.1 mm. 미세 조작이 불가능해지며,
Peg-in-Hole 처럼 접근이 정밀해야 하는 과제에서 치명적이다. x/y 균일성도 깨진다.

**`0.8` 은 포화 직전이다.** 실효 상한이 약 100 mm/s 인데 `cmd 1.0` 에서 102.5 mm 가
나온다 — **비례성을 지키는 최대값**이다. 기본값 대비 속도 2배, z 비대칭 1/3.

> ⚠️ **런타임 파라미터로는 바뀌지 않는다.** `ros2 param set /servo_node
> moveit_servo.scale.linear` 가 성공을 반환하고 조회값도 바뀌지만 **동작은 그대로다**
> (실측). moveit_servo 는 기동 시점에만 읽는다. `servo_linear_scale:=<값>` 으로
> launch 를 다시 띄워야 한다.

### 남는 특성 — 설정값과 실효 속도가 다르다

`0.4` 설정에서 기대 200 mm(=0.5×0.4×1s) 대비 실측 25 mm 로 **약 1/8** 이다. 이 구간에서
servo status 는 전 구간 `NO_WARNING` 이라 특이점·충돌·관절한계로 인한 감속이 아니다.
원인은 미확정이며, **파이프라인 문제는 아니다** — servo 지령을 시뮬레이터가 109~113%
로 추종함을 따로 확인했다(servo 61건 → 브리지 61건 → 실제 0.185 rad).

실용적으로는 **선형이라 예측 가능하다**. `0.8` 기준 대략:

| 키를 누르는 시간 (`linear_step 0.5`) | 이동 |
|---|---|
| 1초 | 약 54 mm |
| 2초 | 약 94 mm |

---

## 6. 그리퍼 — 채널은 확정, 스택 통합이 보류

**채널 사양은 §6.1 에서 확정됐다.** 보류인 것은 그 위의 스택이다 — 실물은
**Robotiq 2F-85** 인데 스택 전체가 **Franka Panda Hand** 를 전제한다.

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

### 6.1 채널 사양 — 실측 (2026-09-01)

**보류였던 것은 스택 통합이지 채널 자체가 아니다.** peg-in-hole 모델에 접속해
측정한 결과 입출력 형식이 확정됐다.

```
/output/gripper_joint   sensor_msgs/JointState   30 Hz   RELIABLE / VOLATILE
    name[i]     = gripper_joint{i+1}          (i = 0..5)
    position[i] = 관절 i 각도    [rad]
    velocity[i] = 관절 i 각속도  [rad/s]
    effort[i]   = 관절 i 토크    [N·m]

/input/gripper_joint    std_msgs/Float64MultiArray
    data[i]     = 관절 i 목표 각도 [rad]
    길이 가변 — data=[x] 는 관절 0 만 움직이고 나머지는 유지한다
```

**팔보다 정보가 많다.** `/output/panda_joint` 는 `velocity`/`effort` 가 빈 배열인데
그리퍼는 셋 다 채워진다. 벤더 패킹 규약(`res[3*i+0/1/2] = q/v/f`, `TCP_정의.txt`)이
**관절마다** 세 값을 만들고, 그리퍼 발행자만 전부 내보내는 것이다.

> "6개 값 = finger 2개 × (angle, velocity, torque)" 로 설명되기도 하는데 **그 해석은
> 측정과 맞지 않는다.** 6개는 **관절** 수다. 슬롯별로 명령을 넣어 대각선 응답을
> 확인했고(슬롯 k → 출력 k, 타축 간섭 ≤ 0.028), `position[1]` 을 속도로 읽으면
> 12 초에 7.37 rad 이 움직여야 하는데 실제 변화는 0 이었다. "3개의 값"은 finger
> 단위가 아니라 **관절 단위**다.

#### 부호가 축마다 반대다

`data=[0.30]×6` 을 주면 `[0.3, 0.3, -0.0, -0.0, -0.0, 0.3]` 이 된다 — 인덱스 2·3·4 는
음수 영역이 정상 범위라 **0 에서 클램프**된다.

| idx | 관측 기준값 | 부호 | URDF 대응 (부호·크기 기준 추정) |
|---|---|---|---|
| 0 | +0.650 | + | `finger_joint` (구동축, `[0, 0.725]`) |
| 1 | +0.619 | + | `left_inner_knuckle_joint` |
| 2 | −0.663 | − | `left_inner_finger_joint` |
| 3 | −0.650 | − | `right_outer_knuckle_joint` |
| 4 | −0.620 | − | `right_inner_knuckle_joint` |
| 5 | +0.662 | + | `right_inner_finger_joint` |

#### ⚠️ mimic 을 시뮬레이터가 강제하지 않는다

`data=[0.30]` 만 보내면 인덱스 0 만 0.30 으로 가고 **나머지 5축은 그대로 남는다.**
URDF 대로면 mimic 으로 함께 움직여야 하므로, 그 상태는 링키지가 어긋난 자세다.
**6축을 일관되게 채워 보내는 것이 브리지 노드의 책임**이며, 한 축만 보내는 API 를
노출하면 그 오류가 TF·파지 판정·데이터셋에 에러 없이 스며든다.

**어긋난 자세를 지령하면 추종 오차로 나타난다.** 인덱스 1·4(inner_knuckle)만 25%
오차(0.15 지령에 0.113 도달)가 남는데, 이는 축의 특성이 아니라 **모순된 지령에
링키지가 저항한 결과**다. 6축을 일관되게 주면 그 오차가 사라진다(아래 open/close).
즉 이 오차의 존재 자체가 "지령이 어긋났다"는 신호로 읽힌다.

#### open / close — 스칼라 하나로 지령한다

6축을 `s · [+1, +1, −1, −1, −1, +1]` 로 채우면 **전 구간에서 오차 0.0000** 으로 도달한다
(s = 0.0 / 0.1 / … / 0.725 를 모두 확인).

| 동작 | `s` | `data` |
|---|---|---|
| `open` | `0.0` | `[0, 0, 0, 0, 0, 0]` |
| `close` | `0.725` | `[+0.725, +0.725, −0.725, −0.725, −0.725, +0.725]` |

> 앞서 "인덱스 1·4 의 추종 오차 25%" 로 기록했던 것은 **축 특성이 아니라 측정 방법
> 문제였다.** 한 축만 바꾼 모순된 자세를 지령해 링키지가 저항한 결과이며, 6축을
> 일관되게 주면 전 축이 완벽히 추종한다.

계단 응답 (개방 → 완전 폐쇄):

| dead time | τ (63%) | 95% | 99% |
|---|---|---|---|
| 88 ms | 283 ms | 725 ms | 1051 ms |

#### grasp — 판정 신호가 둘, 여유가 압도적이다

파지 성공은 **위치 잔차**와 **토크** 두 신호가 독립적으로 알려 준다. 실측:

| 상황 | 잔차 max | \|effort\| max |
|---|---|---|
| 빈손 폐쇄 | **0.0000 rad** | **0.004 N·m** |
| 자유 이동 중 (가속 구간) | — | 0.845 N·m |
| **파지** | **0.0859 rad** | **17.19 N·m** |
| 파지 유지하며 팔 이동 | 0.086 rad | 14.5 ~ 17.2 N·m |

토크 비가 **4300배**라 임계를 어디에 두어도 갈린다.

파지 시 축이 두 그룹으로 갈리는 것도 근거가 된다 — **구동축(0·3)은 목표에 정확히
도달하고, 물체에 닿는 축(1·2·4·5)에만 잔차와 토크가 남는다.** 모터는 명령대로 돌았는데
손가락이 막혀 링키지가 벌어진 것이므로 물리적으로 옳은 그림이다.

```
파지됨 (stalled)      : |effort| > 1.0 N·m  AND  |velocity| ≈ 0
빈손 (reached_goal)   : 잔차 < 0.005 rad     AND  |effort| < 0.05 N·m
이동 중               : |velocity| > 0.001
타임아웃              : 1.5 s (99% 도달 1051 ms + 여유)
```

임계 `1.0 N·m` 는 자유 이동 최대(0.845)보다 위, 파지(17.19)보다 한참 아래라 **양쪽에서
17배 이상 여유**가 있다. 빈손·파지·파지유지·높이이탈 **네 상황에서 오분류 0건**이었다.

**파지한 채 팔을 움직이는 것도 확인했다** — EE 를 +2 cm 들어올리는 동안 effort 가
16~17 N·m 로 끊기지 않았다. pick 동작이 성립한다.

### 향후 작업

1. `panda_arm` + `robotiq_2f_85` xacro 조합으로 URDF 재구성, SRDF `hand` 그룹
   재정의.
2. `rdfp_msgs/GripperCommand` ↔ `/input,/output/gripper_joint` 브리지 노드.
   **URDF/SRDF(1번)와 무관하므로 지금 착수할 수 있다** — 토픽↔액션 변환이고
   필요한 상수는 §6.1 에 전부 있다. `reached_goal`/`stalled` 은 위치 잔차와
   `effort` **두 신호**로 판정한다 (한때 위치만으로 재구성해야 한다고 보았으나,
   `effort` 가 함께 발행되므로 그럴 필요가 없다). robot_twin 의 파지 성공 판정
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
