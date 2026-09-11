# 펑션베이 백엔드 설계 — 토픽 연동형 시뮬레이터

`robot_control` 에 펑션베이 시뮬레이터를 백엔드로 붙인 구성이다. [멀티 시뮬레이터 백엔드 설계안](multi_simulator_backend_design.md) §6.3 의 **"(B) 토픽 브리지"** 방식을 구체화한 첫 사례다.

- 상태: **팔 연동 `현행`** / 그리퍼 **채널 `확정`·스택 통합 `보류`** (§6)
- 실행: `ros2 launch robot_control panda_functionbay.launch.py`

> **⚠️ 실측 결과가 이 문서에 일부만 반영돼 있다.**
> **반영이 끝난 것** — §2.1 `name`(시뮬레이터가 채워 보내므로 순서 계약이 해소됐다),
> §4.3 setpoint latch(이전 판의 "중력으로 내려앉는다"는 철회했다),
> §4.4 중력에 의한 방향별 정상상태 오차, §5.4 servo(twist) 경로 브리지 연결.
> **아직 이 문서에 없는 것** —
> **자기충돌**(시뮬레이터가 막지 않아 MoveIt 으로 복구 불가능한 자세에 빠질 수 있다),
> **팔의 시간 특성**(dead time 34 ms · 시정수 133 ms · 시계 오프셋 2.22 s —
> §6.1 의 계단 응답 표는 그리퍼 값이라 이것과 다르다).
> 그 둘과 남은 작업은 [functionbay_open_work.md](functionbay_open_work.md)에 있다. **이 문서를 근거로 작업하기 전에 그쪽을 먼저 읽는다.**

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

즉 `panda_mock` 에서 **ros2_control 계층을 통째로 들어낸** 형태이며, `panda_jgpc_mock` 은 템플릿이 아니다 — 그쪽은 컨트롤러 *타입* 만 바꿨을 뿐 `controller_manager` · spawner · 순차 기동 체인을 그대로 쓴다.

---

## 2. 시뮬레이터가 제공하는 토픽

| 방향 | 토픽 | 타입 | 비고 |
|---|---|---|---|
| 명령 | `/input/panda_joint` | `sensor_msgs/JointState` | 위치 제어. **내부 보간 없음** |
| 보고 | `/output/panda_joint` | `sensor_msgs/JointState` | **주기는 XML 설정값** (2026-09-08 부터 **50 Hz**; 새 빌드 기본값 10 을 덮어썼다). **`name` 이 채워진다** (§2.1). **`velocity`·`effort` 도 2026-09-03 부터 채워진다** |
| 보고 | `/output/endeffector` | **`geometry_msgs/PoseStamped`** | **사용하지 않는다** (§5.3). 2026-09-03 에 타입이 `Float64MultiArray[6]`(x,y,z,r,p,y) 에서 바뀌었고 `frame_id` 는 빈 문자열이다. 우리가 쓰지 않으므로 벤더 요청 B-4 는 **철회**했다 |
| 명령 | `/input/gripper_joint` | `std_msgs/Float64MultiArray` | ⚠ **배열 규약이 씬마다 다르다** — 축 수·단위·packing 전부 (§6.1a). **팔과 타입이 다르다** — 아래 경고 |
| 보고 | `/output/gripper_joint` | `sensor_msgs/JointState` | position/velocity/effort. **축 수가 씬마다 다르다** (t1 6축 / r2 2축, §6.1a) |
| 센서 | **`/camera_image/compressed`** | **`sensor_msgs/CompressedImage`** | ⚠ **2026-09-08 새 빌드에서 `/camera_image`(`Image`, `bgr8` 640×480) 에서 교체됐다.** `format=jpeg`, 약 25.8 KB (대역폭 36배 감소). 주기는 XML 설정값(현재 10 Hz). `frame_id=camera_link`. **실시간 렌더링이다.** **`camera_info` 는 제공되지 않는다.** 소비자 배선은 **미결** — 체크리스트 §A-6 상세 |
| 시계 | **`/clock`** | `rosgraph_msgs/Clock` | ⚠ **2026-09-08 신설.** 71.94 Hz, 증분 13.89 ms (내부 스텝 ~72 Hz). **`use_sim_time` 은 켜지 않는다** — `/clock` 은 경과시간인데 `/output/*` 는 벽시계라 56.7년 어긋난다(벤더 **B-13**). A-17 정지 계측에만 쓴다 |

> ⚠️ **명령 타입이 팔과 그리퍼에서 다르다** — 팔은 `JointState`, 그리퍼는
> `Float64MultiArray` 다. ROS 2 는 같은 토픽 이름에 다른 타입의 퍼블리셔를 허용하고
> **연결만 안 될 뿐 오류를 내지 않으므로**, 팔의 코드를 복사해 그리퍼에 쓰면 증상이
> "그리퍼가 ROS 명령을 무시한다"로 나타난다. 2026-09-03 에 `fb_gripper.py` 가 실제로
> 그렇게 깨져 있었고, 그 결과 체크리스트 B-12~14 가 **잘못된 근거로 '측정 불가'로
> 닫혔다** (시뮬레이터가 아니라 우리 스크립트 문제였다). 타입을 확인하는 법:
> `ros2 topic info /input/gripper_joint`.

### 2.1 `name` — 순서 계약이었으나 해소됐다

**2026-09-01 실측: `/output/panda_joint` 가 `panda_joint1`~`panda_joint7` 을 채워 보낸다.** `/output/gripper_joint` 도 `gripper_joint1`~`6` 을 채운다.

원래 이 백엔드의 가장 큰 위험이 여기 있었다 — `name` 을 비운 채 배열 순서만 계약이면 JGPC 의 `Float64MultiArray` 와 같은 성질이라, 순서가 어긋나도 에러 없이 엉뚱한 관절이 움직인다. `joint_state_fusion` 은 그 상황을 전제로 만들어졌고 **입력이 `name` 을 채우기 시작하면 그것을 신뢰**하도록 되어 있었으므로(`if msg.name:` 분기), 코드 변경 없이 그대로 이득을 본다.

남은 경로는 **입력 방향**이다. `/input/gripper_joint` 는 여전히 `Float64MultiArray` 라 순서가 계약이다 (§6.1).

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
     ├─ move_group / servo_node / rviz2 (rviz 는 enable_rviz, 기본 false)
     ├─ servo_command_bridge          /servo_node/commands ─→ /input/panda_joint
     ├─ camera_republish              (enable_camera — 압축→raw, 뷰어를 켜면 자동)
     ├─ camera_republish              /camera_image/compressed ─→ raw (image_transport)
     ├─ image_viewer_node             (둘 다 enable_image_viewer, 기본 false)
     ├─ grasp_center static TF + ee_pose_node
     ├─ robotiq_2f_gripper_node       (enable_gripper)
     └─ functionbay_scene_state_node  물체 TF ─→ /scene/objects (enable_scene)
```

**그리퍼·scene 을 준비 후에 띄우는 것은 의도적이다** — 그리퍼는 관절 보고를 받아야
`at_goal` 판정이 서고, scene 은 `world → panda_link0` 정적 TF 가 먼저 있어야 조회가
성립한다.

브리지 노드는 `robot_control/functionbay/` 에 있다.

### 3.1 `joint_state_fusion`

이름 없는 배열에 이름을 부여하고, URDF 가 요구하지만 시뮬레이터가 보고하지 않는 관절을 덧붙인다.

```
/output/panda_joint (7축, 이름 채워짐 — 없으면 파라미터로 보완)
    → name = [panda_joint1..7]
    → panda_finger_joint1 = 0.04 (고정, §6)
    → /joint_states (8축)
```

`panda_finger_joint2` 는 URDF 에서 `<mimic joint="panda_finger_joint1"/>` 이므로 `robot_state_publisher` 가 파생시킨다 — 채울 필요가 없다.

`stamp_source=auto` 가 기본이다. 입력 stamp 가 0 이면 노드 시계로 채운다.

### 3.2 `readiness_gate`

**첫 메시지를 받으면 종료**한다. 그래야 기존 spawner 와 똑같이 `OnProcessExit`로 엮여, 백엔드가 달라도 orchestration 코드가 그대로다.

> ⚠️ **종료 코드를 반드시 본다.** `OnProcessExit` 는 실패 종료에도 발동하므로, 그냥 엮으면 시뮬레이터가 없어 타임아웃된 뒤에도 `move_group` 이 올라온다. 실제로 구현 중 이 버그를 냈고, spawner 체인이 쓰는 `launch_helpers.controller_startup._chain_or_shutdown` 을 재사용해 고쳤다.

타임아웃(기본 60초) 시 0 이 아닌 코드로 끝나며 런치 전체가 종료된다. 시뮬레이터가 안 떴는데 상위 스택이 올라가 조용히 멈춘 것처럼 보이는 상황을 막는다.

---

## 4. 명령 경로 — 보간이 필수다

펑션베이는 **내부 보간을 하지 않는다.** 받은 관절 위치로 바로 간다. 그런데 MoveIt의 궤적은 TOTG 로 **~10 Hz 로 리샘플**되므로, 그대로 흘리면 10 Hz 계단 동작이 된다.

> **TOTG (Time-Optimal Trajectory Generation)** 는 MoveIt 의 기본 시간 파라미터화 단계다 (Kunz & Stilman 2012). 플래너나 `GetCartesianPath` 가 내놓는 것은 시각이 없는 기하 경로뿐이고, TOTG 가 관절 속도·가속도 한계를 지키며 각 point 의 `time_from_start` 를 채운다. 이때 경로를 **`resample_dt = 0.1` 초 격자로 다시 샘플링**하기 때문에 궤적 주기가 경로 길이와 무관하게 항상 ~10 Hz 로 고정된다.
> **`GetCartesianPath` 요청에는 이 값을 바꿀 필드가 없고, `max_step` 을 줄여도 리샘플이 그 뒤에 일어나므로 point 수는 늘지 않는다.** JTC 스택에서는 컨트롤러가 point 사이를 보간해 이 사실이 드러나지 않지만, 내부 보간이 없는 펑션베이에서는 그대로 100 ms 계단으로 나타난다. 더 자세한 근거: [MoveGroupJgpcClient_UserGuide.md](../moveit/MoveGroupJgpcClient_UserGuide.md), [replay_approaches.md](../replay/replay_approaches.md).

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

`create_move_group_client` 의 auto 판별은 `/panda_arm_controller/commands` 에
**퍼블리셔 또는 서브스크라이버가 있는지**를 본다 — 토픽 이름의 존재가 아니라
**엔드포인트의 존재**다 ([move_group_factory.py](../../src/robot_control/robot_control/moveit/move_group_factory.py)
의 `detect_controller_mode`: `return MODE_JGPC if (publishers or subscribers) else MODE_JTC`).

펑션베이 스택에는 그 토픽에 아무 엔드포인트도 없으므로 **JTC 로 오판**한다.
반드시 `mode='jgpc'` 를 명시한다.

**두 가지 귀결이 있다.** ① 판별은 **토픽 그래프 조회 한 번**이라 서비스 대기가 없지만,
**DDS discovery 가 끝나기 전에 부르면 JGPC 스택도 JTC 로 읽는다.** ② 우리 쪽 노드가
그 토픽에 퍼블리셔를 하나만 만들어도 **JGPC 로 뒤집힌다** — 진단 스크립트를 쓸 때 주의한다.

**인자가 넷이다 — `arm_command_joint_names` 를 빠뜨리면 실행 단계에서 죽는다.**

```python
create_move_group_client(node, mode='jgpc',
                         arm_command_topic='/input/panda_joint',
                         arm_command_format='joint_state',
                         arm_command_joint_names=[f'panda_joint{i}' for i in range(1, 8)])
```

넷째를 생략하면 `TrajectoryStreamer` 가 명령 토픽 이름에서 컨트롤러 파라미터 서비스를
유도해 `joints` 를 조회하는데, **ros2_control 이 없어 그 서비스가 존재하지 않는다.**

```
TimeoutError: Timed out waiting for /input/get_parameters
```

**계획은 성공한 뒤에 터진다** (`fraction: 1.0` 로그가 먼저 찍힌다). 계획 실패로 오진하기
쉬우니 예외 메시지의 서비스 이름을 본다.

**`move_group` 서비스가 뜰 때까지 기다려야 한다.** TF 만 보고 준비를 판정하면
`robot_state_publisher` 가 먼저 떠서 2 초 만에 통과하는데 `/compute_cartesian_path` 는
아직이라 첫 호출이 응답 없이 멈춘다. `service_is_ready()` 로 확인한다.

**실측 (2026-09-04).** `move_linear`(= `follow_trajectory`)로 `panda_link8` 을 수직
+100 mm 지령 → 계획 `fraction 1.0`, 7 점을 100 Hz 로 252 개 보간 발행, 손끝 실제
**+89.0 mm / 수평 −5.0 mm**. 같은 이동을 자체 IK 로 준 결과(+88.9 / −6.9)와 일치하며,
**11 mm 부족분은 중력 처짐이다** — MoveIt 이 계획해도 같다. **개루프이므로 정상 반환이
도달을 뜻하지 않는다.**

#### 왕복은 **절대 좌표 목표**로 준다 — 증분을 누적하면 흘러내린다

`move_linear` 목표를 매번 *"현재 자세 + Δ"* 로 만들면 **직전 이동의 오차 위에 쌓인다.**
±60 mm 왕복 5회 실측:

| 지령 방식 | 시작점 복귀 오차 |
|---|---|
| 현재 자세 + Δ (누적) | **dz −102.8 mm / dx −55.0 mm** |
| **절대 좌표 2개를 번갈아** | **dz −4.9 mm / dx −1.9 mm** |

누적 방식은 **상승 +49 / 하강 −70 mm** 로 비대칭이라(중력 방향으로 매 사이클 21 mm 씩
더 간다) 다섯 번에 10 cm 를 잃는다. **그런데 10회 호출이 모두 정상 반환했고 계획도 매번
`fraction 1.0` 이었다** — 반환값으로는 알 수 없다.

절대 좌표로 고정하면 **다섯 사이클이 소수점 넷째 자리까지 같다**(`z=0.4914` / `0.4312`
열 번 동일). 시뮬레이터의 반복 정밀도는 0.1 mm 이내이므로, 흘러내림은 장비 특성이
아니라 **지령 방식의 문제다.**

남는 오차 둘은 **체계적이라 보정할 수 있다** — 진폭이 60 → 55.3 mm 로 덜 가고,
복귀점이 시작보다 4.9 mm 아래에 선다. 매번 같은 값이다.

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

| 이동 | 지령 | 2026-09-01 | **2026-09-04** | |
|---|---|---|---|---|
| EE 상승 | +20 mm | **+10.0 mm** | **+15.13 mm** (76%) | 중력이 거슬러 **부족** |
| EE 하강 | −20 mm | **−30 mm** | **−25.00 mm** (125%) | 중력이 도와 **초과** |
| 상승·하강 크기 차 | — | **20.0 mm** | **9.87 mm** | 절반으로 줄고 **거기서 정체** |
| `joint2` 단독 (하강측) | +0.080 rad | +0.0474 rad (59%) | — | |

> **측정 프레임에 주의한다.** 위 EE 값은 MoveIt 그룹 tip(`panda_link8`) 기준이다.
> `/ee_pose` 는 `ee_frame`(현재 `grasp_center`, tip 대비 z **−149.2 mm**)을 가리키므로,
> **`/ee_pose` 를 읽어 데카르트 목표를 만들면 그 오프셋만큼 엉뚱한 곳을 지령한다** —
> `fb_ee.py` 가 실제로 그렇게 깨져 상승·하강이 둘 다 하강이 된 적이 있다(2026-09-04).
> **부하가 얹히면 더 나빠진다** — 파지 상태에서 +20 mm 상승은 **+9.79 mm(49%)** 였다.

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
백엔드 중립 이름 `servo_command_bridge` 를 더했다 (`servo_command_bridge` 도 유지).

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

> ⚠️ **중력 처짐은 2026-09-11 부터 우리 쪽에서 상쇄한다.** `실제 − 지령 = τ_g(q)/Kp`
> 이고 `Kp` 를 씬 XML 이 알려 주므로(t1·r2 모두 2000), 스트리밍 경로에서 모든 명령점에
> `−τ_g/Kp` 를 더한다. 작업 영역 8지점 도달 오차 **12.37 → 0.00 mm**.
> 전용 문서: [functionbay_gravity_compensation.md](functionbay_gravity_compensation.md).

## 6. 그리퍼 — 채널·브리지 완료, URDF/SRDF 만 남았다

**채널 사양은 §6.1 에서 확정됐고 브리지 노드도 붙었다.** 남은 것은 description 이다 —
실물은 **Robotiq 2F-85** 인데 URDF/SRDF 가 **Franka Panda Hand** 를 전제한다.

| 위치 | 전제 |
|---|---|
| URDF | `panda_finger_joint1` prismatic **0 ~ 0.04 m** |
| SRDF `hand` 그룹 | `open` = 0.035 / `close` = 0 |
| `GripperActionNode` 의 `targets` | `open: [0.035, 10]` / `close: [0.0, 10]` / `grasp: [0.0, 30]` — 관절값(m) + 힘(N) |
| 그리퍼 실행 경로 | `control_msgs/GripperCommand` **액션 서버** 존재 |

**마지막 두 줄은 해소됐다 (2026-09-02).** [`Robotiq2FGripperNode`](../gripper/Robotiq2FGripperNode_Guide.md)
가 6축 목표각을 `/input/gripper_joint` 로 직접 쓰고, `targets` 는 2F-85 스칼라
(`close: 0.725 rad`)를 갖는다. **계약(`GripperCommand`/`GripperState`)은 그대로**이므로
상위(teleop·트윈·데이터셋)는 백엔드를 모른다.

**남은 것은 URDF/SRDF 다.** 위 표의 첫 두 줄이며, TF 와 충돌검사가 여기에 걸린다 —
그래서 launch 가 `panda_finger_joint1` 에 고정값을 주입해 TF 만 성립시키고, **실제
그리퍼 상태는 `/gripper_states` 로 따로 낸다.**

2F-85 는 **스트로크 85 mm 의 링크 구동식**이라 기구학 자체가 다르다. 손가락 개폐가
직동(prismatic)이 아니라 **회전 관절**이며, 제공된 URDF 기준 `finger_joint` 가
**0 ~ 0.725 rad** 이다. Panda Hand 의 미터 단위 값을 그대로 넘기면 TF · 충돌검사 ·
robot_twin 파지 판정 · 데이터셋 그리퍼 채널이 **에러 없이** 어긋난다.

**그래서 팔 연동을 먼저 완성하고 그리퍼는 분리했다.** 그동안
`panda_finger_joint1` 은 `fb_finger_position`(기본 0.04) 고정값으로 채워 TF 만
성립시킨다 — **실제 그리퍼 상태가 아니다.**

### 6.1a ⚠️ 그리퍼 명령 규약은 **씬마다 다르다** (2026-09-10 실측)

**아래 §6.1 의 "6축 · 라디안 · 위치 나열" 은 `t1` 씬 하나의 성질이다.** 같은 배포판
(`MMR-Build_260907`)의 `r2` 씬은 셋 다 다르다. 이름 규약은 체크리스트 §전제 앞 블록에
있다 — **`Crisp-260907` / `RecurDyn-260907`**.

| | **`Crisp-260907`** (`t1` 씬) | **`RecurDyn-260907`** (`r2` 씬) |
|---|---|---|
| 관절 수 | 6 | **2** (`inner_knuckle` 좌·우) |
| 명령 단위 | 라디안 | **도(degree)** |
| 명령 packing | 위치만 나열 (6값) | **관절당 `(position, velocity, force)`** (2×3 = 6값) |
| 보고 | `JointState` 6/6/6 | `JointState` 2/2/2 |
| 씬 XML | `t1__Gripper6_1_vm.xml` (6관절, Kp 200) | `r2__Gripper_1_vm.xml` (2관절, Kp 200) |
| 선언 | `<Gripper Number="18">` | `<Gripper Number="6">` |

`Number` 는 관절 수가 아니라 **값 개수**(관절 수 × 3)다. 팔도 같다 — `Number="21"` = 7×3.

**틀리면 에러가 아니라 "그리퍼가 거의 안 움직인다"로 나타난다.** 도로 읽는 씬에
라디안을 보내면 `π/180 = 1.745%` 만 움직이고, `effort` 도 안 오르므로 접촉으로도
안 잡힌다. 요청서에 열려 있던 *"그리퍼 지령 미반영 1.6~4%"* 항목이 바로 이것이었다 —
시뮬레이터 결함이 아니라 우리가 단위를 몰랐던 것이다.

**어떻게 갈랐나.** 예측이 서로 반대인 두 지령을 넣었다 (force 슬롯은 항상 0 으로 둔다).

```
A = [0.3, 0, 0, -0.3, 0, 0]   2x(p,v,f) 예측: j1 +0.3, j2 -0.3  /  6각도 예측: j2 = 0
    실측 → j1 +0.0053, j2 -0.0053            ← 2x(p,v,f) 쪽
B = [0.3, -0.3, 0, 0, 0, 0]   2x(p,v,f) 예측: j1 만 움직임      /  6각도 예측: j2 = -0.3
    실측 → j1 +0.0048, j2 -0.0001            ← 2x(p,v,f) 쪽
```

단위는 도달 비율이 지령 크기와 무관하게 1.8% 로 **일정한 것**에서 나왔다(= π/180).
도로 환산해 보내면 0.1 / 0.3 / 0.5 / 0.725 rad 전부 **오차 0.0001 rad 이하**로 선다.

**보고는 어느 규약에서도 라디안이다** — 환산은 명령 방향뿐이다.

#### 어디에 적혀 있나

규약은 코드가 아니라 **파일**이 갖는다. 백엔드 프로파일이 그 파일을 가리킨다.

```yaml
# config/backends/functionbay.yaml
solver: RecurDyn                                        # 지금 띄운 씬의 솔버
gripper:
  profile_file: config/functionbay_gripper_recurdyn.json
```

`config/functionbay_gripper_{recurdyn,crisp}.json` 이 `axis_count` · `axis_signs` ·
`command_units` · `command_packing` · `targets` 를 담고,
`robot_control.gripper.profile.GripperProfile` 이 읽어 `command(s)` 로 배열을 만든다.
`Robotiq2FGripperNode` 는 `gripper_profile_file` 파라미터로 그 파일을 받는다 —
**주지 않으면 옛 동작(6축·라디안)** 이므로 mock·Isaac 경로는 영향이 없다.

**씬을 바꾸면 `profile_file` 도 바꿔야 한다.** 안 바꾸면 노드가 보고 축 수와 선언을
대조해 **명령을 거부한다** (예전에는 5초마다 경고만 하고 무시했고, 그 로그가 계속
찍히는데도 원인에 도달하는 데 오래 걸렸다). 단위와 packing 은 보고에서 읽을 수
없으므로 선언에 의존한다 — 씬을 바꿨을 때 가장 먼저 틀리는 값이 축 수라 이것을
방어선으로 쓴다.

---

### 6.1 채널 사양 — 실측 (2026-09-01, **`t1` 씬 기준**)

> **이 절의 6축·라디안 서술은 `t1` 씬에만 해당한다** — §6.1a 를 먼저 본다.

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

~~**팔보다 정보가 많다.**~~ → **해소됐다 (2026-09-03).** 팔도 `velocity`·`effort` 를 채운다
(`pos=7 vel=7 eff=7`, 체크리스트 A-8). 벤더 패킹 규약(`res[3*i+0/1/2] = q/v/f`,
`TCP_정의.txt`)이 **관절마다** 세 값을 만드는데, 예전에는 그리퍼 발행자만 전부 내보냈고
이제 양쪽이 내보낸다.

> "6개 값 = finger 2개 × (angle, velocity, torque)" 로 설명되기도 하는데 **그 해석은
> 측정과 맞지 않는다.** 6개는 **관절** 수다. 슬롯별로 명령을 넣어 대각선 응답을
> 확인했고(슬롯 k → 출력 k, 타축 간섭 ≤ 0.028), `position[1]` 을 속도로 읽으면
> 12 초에 7.37 rad 이 움직여야 하는데 실제 변화는 0 이었다. "3개의 값"은 finger
> 단위가 아니라 **관절 단위**다.

#### 부호가 축마다 반대다

`data=[0.30]×6` 을 주면 `[0.3, 0.3, -0.0, -0.0, -0.0, 0.3]` 이 된다 — 인덱스 2·3·4 는
음수 영역이 정상 범위라 **0 에서 클램프**된다.

| idx | 관측 기준값 | 부호 | 대응 관절 (**모델 파일로 확인**) |
|---|---|---|---|
| 0 | +0.650 | + | `finger_joint` (구동축, `[0, 0.725]`) |
| 1 | +0.619 | + | `left_inner_knuckle_joint` |
| 2 | −0.663 | − | `left_inner_finger_joint` |
| 3 | −0.650 | − | `right_outer_knuckle_joint` |
| 4 | −0.620 | − | `right_inner_knuckle_joint` |
| 5 | +0.662 | + | `right_inner_finger_joint` |

> **추정이 아니라 확인된 값이다 (2026-09-04).** 모델의
> `MMR_Database/Database/VariableModel/t1__Gripper6_1_vm.xml` 이 순서를 명시한다 —
> `Joint_1 finger_joint` / `Joint_2 left_inner_knuckle` / `Joint_3 left_inner_finger` /
> `Joint_4 right_outer_knuckle` / `Joint_5 right_inner_knuckle` /
> `Joint_6 right_inner_finger`. 위 표와 일치한다.
>
> 같은 파일의 **`Kp="200 200 200 200 200 200"`** 이 `effort = 200 × 잔차` 의 그 계수다
> (`Kd=20`). 팔은 별도로 `t1__Manipulator_1_vm.xml` 에서 **`Kp=2000`, `Kd=200`** 이므로
> **`/output/panda_joint` 의 `effort` 는 계수가 다르다** — 그리퍼 값을 그대로 옮기면 안 된다.

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
(s = 0.0 / 0.15 / 0.2 / 0.3 / 0.4 / 0.5 / 0.6 / 0.725 확인).

| 동작 | `s` | `data` |
|---|---|---|
| `open` | `0.0` | `[0, 0, 0, 0, 0, 0]` |
| `close` | `0.725` | `[+0.725, +0.725, −0.725, −0.725, −0.725, +0.725]` |

> **⚠️ 도달하지 못하면 그리퍼 안에 물체가 있는지부터 본다.** 2026-09-04 측정에서 `s = 0`
> 이 0.04 ~ 0.14 rad 에서 멈추고 시행마다 값이 달라지는 현상을 관측해 **한때 "개방 방향
> 저항"이라는 시뮬레이터 특성으로 기록했으나, 그 진단은 틀렸다.** 장면을 정리한 뒤
> 4회 반복에서 `open` 은 **매번 잔차 0.0001** 로 도달한다. 당시 멈춘 지점이 계속
> 올라간 것(0.04 → 0.14)이 물체가 밀려나던 흔적이었다.
>
> **구별법은 축의 패턴이다** — 물체에 막히면 구동축(0·3)은 목표에 정확히 도달하고
> 접촉축(1·2·4·5)에만 잔차가 남는다(아래 grasp 참조). 여섯 축이 함께 못 가면 그때야
> 구동 문제를 의심한다.

> 앞서 "인덱스 1·4 의 추종 오차 25%" 로 기록했던 것은 **축 특성이 아니라 측정 방법
> 문제였다.** 한 축만 바꾼 모순된 자세를 지령해 링키지가 저항한 결과이며, 6축을
> 일관되게 주면 전 축이 완벽히 추종한다.

계단 응답 (개방 → 완전 폐쇄):

| dead time | τ (63%) | 95% | 99% |
|---|---|---|---|
| 88 ms | 283 ms | 725 ms | 1051 ms |

2026-09-03 재측정(3회 중앙값)에서 **105 / 361 / — / 1112 ms** 로 일치했다. 보고 주기가
30 Hz 라 dead time 분해능이 **±33 ms** 이므로 88 ↔ 105 는 한 표본 이내다.

#### grasp — 잔차로 판정한다 (`effort` 는 그 환산값이다)

파지 성공은 **위치 잔차**로 판정한다. 실측:

| 상황 | 잔차 max | \|effort\| max |
|---|---|---|
| 빈손 폐쇄 | **0.0000 rad** | **0.004 N·m** |
| 자유 이동 중 (가속 구간) | — | 0.845 N·m |
| **파지** | **0.0859 rad** | **17.19 N·m** |
| 파지 유지하며 팔 이동 | 0.086 rad | 14.5 ~ 17.2 N·m ⚠ **이동 중에는 버티지만 정지 후 놓친다** — 아래 주 |

토크 비가 **4300배**라 임계를 어디에 두어도 갈린다.

> ⚠️ **"팔 이동 중 유지"는 파지가 끝까지 유지된다는 뜻이 아니다 (2026-09-04).** 상승 중에는
> 15.4 N·m 로 버티지만 **정지 후 약 6.5초 뒤 `|effort|` 가 14.20 → 0.15 N·m 로 붕괴**한다 —
> 물체가 미끄러져 떨어진다. `t1` 씬에 **손끝 접촉 정의가 없어서**이며 우리 쪽 파지력으로는
> 해결할 수 없다(`grasp` 가 이미 `finger_joint` 상한 0.725 rad). 벤더 요청 **B-11**,
> 근거는 체크리스트 **B-16** 이다. 떨어지는 순간 팔이 **+3.2 mm 올라가는 것**이 독립 지표가 된다.

> ⚠️ **`effort` 는 독립 신호가 아니다 (2026-09-03).** 세 가지 전혀 다른 구간에서 18점을
> 대조한 결과 **`effort = 200.0 × (지령 − 실제)`** 가 정지 시 오차 0.037 이하로 성립한다
> (과구동 지령 20.8 rad → effort 4015.00, 예측 4015.00). 즉 이 값은 **측정된 접촉
> 토크가 아니라 드라이브의 비례 오차항**이며 K = 200 N·m/rad 이다. 클램프도 없다.
>
> **위 표 자체가 그 증거다** — 파지 행의 `0.0859 rad × 200 = 17.18` 이 같은 행의
> `17.19 N·m` 다. 두 열은 독립 관측이 아니라 **한 값의 두 표현**이다.
>
> 따라서 아래 판정식의 `|effort| > 1.0` 은 **`잔차 > 0.005`** 와 같은 조건이다
> (`1.0 ÷ 200 = 0.005`). 이 값이 `Robotiq2FGripperNode` 의 `position_tolerance` 기본값과
> 같으므로, 그 노드 `_at_goal()` 의 `and not stalled` 항은 **결과를 바꿀 수 없다**
> (잔차가 허용치 이내면 effort 는 반드시 임계 이하다).
>
> **`grasp` 판정 자체는 성립한다** — "목표까지 못 갔고 멈췄다"는 파지의 옳은 정의이고,
> 잔차가 그것을 그대로 재기 때문이다. 무너지는 것은 *두 신호가 서로 검증한다*는
> 근거뿐이다. 힘을 재는 관측이 필요하면 이 채널로는 얻을 수 없다.

파지 시 축이 두 그룹으로 갈리는 것도 근거가 된다 — **구동축(0·3)은 목표에 정확히
도달하고, 물체에 닿는 축(1·2·4·5)에만 잔차와 토크가 남는다.** 모터는 명령대로 돌았는데
손가락이 막혀 링키지가 벌어진 것이므로 물리적으로 옳은 그림이다.

```
파지됨 (stalled)      : |effort| > 1.0 N·m  AND  |velocity| ≈ 0
빈손 (reached_goal)   : 잔차 < 0.005 rad     AND  |effort| < 0.05 N·m
이동 중               : |velocity| > 0.001
타임아웃              : 1.5 s (99% 도달 1051 ms + 여유)
```

> ⚠️ **`|velocity| < 0.001` 은 잡음 문턱에 걸려 있다.** 막혀 멈춘 상태에서 실측한
> 속도가 0.0001 · 0.0004 · **0.0015** 로 임계를 넘나들어, 같은 자세인데 `stalled` 이
> `true` ↔ `false` 로 깜빡인다. Isaac 은 같은 이유로 `stall_velocity: 0.0` 으로 **끈다**
> (`panda_isaac.launch.py`). 펑션베이는 아직 켜 둔 상태다.

임계 `1.0 N·m` 는 자유 이동 최대(0.845)보다 위, 파지(17.19)보다 한참 아래라 **양쪽에서
17배 이상 여유**가 있다. 빈손·파지·파지유지·높이이탈 **네 상황에서 오분류 0건**이었다.

**파지한 채 팔을 움직이는 것도 확인했다** — EE 를 +2 cm 들어올리는 동안 effort 가
16~17 N·m 로 끊기지 않았다.

> ## ⚠️ 그러나 물체를 실제로 **들어 올리면 미끄러져 떨어진다** (2026-09-04)
>
> **원인은 제어가 아니라 모델 설정이다.** 활성 씬 `t1` 에는 **손끝 접촉 정의가 없다.**
> 등록된 접촉은 `Contact_1/2/3` 세 개뿐이고 전부 물체 쪽이며
> (`peg_hole` · `peg` · `peg_tray`), 셋 다 **`friction_coefficient="0.15"`** 다.
>
> **같은 빌드의 `r2` 씬에는 있다** — 손끝마다 전용 접촉을 걸고 마찰을 **1.0** 으로 잡는다.
>
> ```xml
> <!-- r2__GGeomContact_1_vm.xml — t1 에 없는 정의 -->
> <Force name="GGeomContact_1" type="Contact">
>   <child  IGEOSURFNAME="peg_round_16_surf" />
>   <friction D_F_C="1" S_T_V="0.01" D_T_V="0.015" S_F_C="1" />
>   <normal K="1000000" C="2000" MAXPEN="1" />
>   <parent JGEOSURFNAME="left_inner_finger_surf" />
> </Force>
> <!-- GGeomContact_2 → right_inner_finger_surf -->
> ```
>
> **실측 (P2 자세, 손끝 기준 수직 상승 +88 mm):** 상승 중에는 접촉축 잔차가 90% 이상
> 보존되는데, **정지한 뒤에 빠진다** (`0.0690` → `0.0053` → `0`). 정지 후 상실이므로
> **정지마찰조차 부족하다**는 뜻이다. 느리게 할수록 오히려 일찍 놓친다(중력에 계속
> 미끄러진다). **파지력을 더 올릴 수단도 없다** — `grasp` 가 이미 `finger_joint` 상한
> `0.725 rad` 다.
>
> **경로 문제가 아니다.** 단일 관절 이동과 손끝 기준 수직 이동 양쪽에서 재현된다.
> 파지 판정(`at_goal`/`stalled`)은 매번 정상이었다 — **쥐는 것과 유지하는 것은 다른
> 문제다.**

### 6.2 EE 프레임 — 실물은 Robotiq 인데 description 은 Panda Hand 다

**`/ee_pose` 의 기본 프레임 `panda_hand` 는 실제 파지점이 아니다.** 이 스택이 싣는
description 은 `robot_control/description/` 의 Panda Hand 계열이고(`robotiq` 문자열 0건),
시뮬레이터 실물은 Robotiq 2F-85 다.

| 기준 | `panda_link7` 로부터 |
|---|---|
| `panda_hand` (`/ee_pose` 기본값) | z **+0.107** |
| **실제 파지 중심** | z **+0.256**, yaw **+1.5708** |
| **차이** | **149 mm + 90°** |

파지 중심은 URDF 에서 계산했다 — `robotiq2panda`(xyz `0 0 0.130`, rpy `0 0 π/2`) 다음에
`left_inner_finger` / `right_inner_finger` 원점의 중점(`robotiq_85_base_link` 기준
z **+0.12627**, `grasp` 자세 `s=0.725`).

#### 현재 조치 (**EE 프레임 A안**) — 정적 TF 로 가리키는 점만 옮겼다

`panda_functionbay.launch.py` 가 `grasp_center` 프레임을 발행하고 `ee_frame` 기본값을
그쪽으로 둔다 (`launch_helpers/ee_pose.py` 의 `create_grasp_center_tf_node()`).

```
panda_link7 → grasp_center   xyz=(0, 0.0003, 0.25627)  yaw=1.5708
```

**`/ee_pose` 만 맞춰진다.** TF 형상·충돌 검사·RViz 는 여전히 Panda Hand 이고, 그리퍼
관절은 TF 에 실리지 않는다. 다른 백엔드는 `panda_hand` 기본값 그대로다.

#### 근본 해결 (**EE 프레임 B안**) — description 교체

**필요한 자산은 이미 있다.** `panda_ftsensor_robotiq` 패키지가 이걸 위해 존재한다.

| 자산 | 상태 |
|---|---|
| URDF | **시뮬레이터가 쓰는 것과 운동학 차이 0건** (2026-09-04 관절 8개 원점·축 전수 비교) |
| SRDF | `panda_arm` · `hand` 그룹, `group_state` **9개** |
| 메시 | 포함 |

시뮬레이터 쪽 원본은
`MMR_Database/Database/ConstantModel/panda_ftsensor_robotiq.urdf` 이며 우리 패키지의
사본과 운동학이 같다. **베이스도 정합한다** — `t1__Fixed_1_vm.xml` 이 `panda_link0` 를
ground 원점에 `rpy 0 0 0` 으로 고정하므로, `link0` 기준 수직이 월드 수직이다.

**바꿔야 할 곳.**

1. `build_moveit_config()` — 현재 `moveit_resources_panda_moveit_config` 의 SRDF·kinematics 를 쓴다. `panda_ftsensor_robotiq` 의 SRDF 로 갈아끼운다.
2. `ee_frame` 기본값 — `grasp_center` 정적 TF 대신 URDF 의 실제 링크를 가리킨다. 그러면 `create_grasp_center_tf_node()` 는 제거한다.
3. 충돌 행렬(ACM) — 손가락 링크가 6개 늘어나므로 SRDF 의 `disable_collisions` 를 확인한다.
4. `panda_hand_controller` 전제 — **펑션베이는 ros2_control 이 없어 해당 없다.** mock·Gazebo·Isaac 으로 확장할 때만 걸린다.

**펑션베이가 EE 프레임 B안을 먼저 하기 좋은 백엔드다** — ros2_control 이 없어 컨트롤러·hardware
인터페이스 분기를 건드리지 않는다. 반대로 **description 은 다른 백엔드와 공유하므로**,
공용 경로를 바꾸면 mock·Gazebo·Isaac 회귀 확인이 필요하다. 백엔드별로 분기하는 편이
안전하다.

**지금 EE 프레임 B안을 미룬 이유.** 아픈 것이 `/ee_pose` 하나뿐이고 소비자가 사실상 없다.
그 이점인 정확한 충돌 형상도 현재는 쓰이지 않는다 — `/scene/objects` 가 planning
scene 에 들어가지 않고(§ CLAUDE.md) 파지 경로는 cartesian 이라 `avoid_collisions` 가
기본 `False` 다. **더 급한 것은 파지 유지(마찰)이고 그것은 벤더 쪽이다** (B-11).

> ⚠️ **그 전제가 2026-09-09 에 깨졌다.** 소비자가 생겼고(파지 자세 계산), 아픈 것은
> `/ee_pose` 의 **값**이 아니라 그것을 **손끝으로 오해하는 것**이었다.
>
> `grasp_center` 는 두 패드 사이의 *파지 중심*이고 **최하단이 아니다** — 정의가
> `inner_finger` **링크 원점**의 중점이기 때문이다. 패드는 거기서 손끝 방향으로
> **44 mm 더** 뻗는다(충돌 메시 바운딩 박스). 이것을 모르고 파지 중심을 z = 0.035 로
> 내렸다가 **손끝이 탁자에 박혔다**(`effort` 6.194 N·m, 빈손 기준선 0.01).
>
> 실측 오프셋은 **39~40 mm**(그리퍼 열림 — 닫을수록 44 mm 에 가까워진다). 수치, 파지
> 높이 산정법, `panda_link8` 역변환 절차는
> **[grasp_center_frame.md](../gripper/grasp_center_frame.md)** 에 있다.

---

### 향후 작업

1. **description 을 Robotiq 조합으로 교체 (EE 프레임 B안).** 자산·정합성·바꿀 곳은 §6.2 에
   정리돼 있다. 현재는 `grasp_center` 정적 TF 로 `/ee_pose` 만 보정한 상태다(EE 프레임 A안).
2. ~~`rdfp_msgs/GripperCommand` ↔ `/input,/output/gripper_joint` 브리지 노드.~~
   **✅ 완료 (2026-09-02)** — `Robotiq2FGripperNode` (`robot_control`, 실행 파일
   `robotiq_2f_gripper_node`). 예상대로 URDF/SRDF(1번)와 무관했다. `at_goal`/`stalled` 은
   관절 잔차와 `effort` **두 신호**로 판정하며, 임계값은 §6.1 실측 그대로다.
   `width` 는 링키지 기하가 없어 **NaN** 이다 — `at_goal` 은 잔차로 판정하므로
   영향받지 않는다. [가이드](../gripper/Robotiq2FGripperNode_Guide.md).
3. ~~robot_twin 설정의 `backend.targets` 를 2F-85 스케일로 재작성.~~ **불필요해졌다** — 명령이 심볼만 싣게 되어(2026-09-01) 트윈은 `backend.labels` 만 갖고, 2F-85 수치는 `Robotiq2FGripperNode` 의 `targets` 파라미터에 있다. 다만 **펑션베이용 트윈 설정 자체가 아직 없다.**

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
| 그리퍼 | **채널·노드·측정은 완료** — `robotiq_2f_gripper_node` 배선, 체크리스트 B-12~B-14 통과. 남은 것은 **description 교체(§6.2 EE 프레임 B안)** 와 **손끝 접촉 정의(벤더 B-11)** 다 |
| **`grasp_center` 가 손끝이 아니다** | **2026-09-09 실측으로 확인.** 정의가 `inner_finger` **링크 원점**의 중점이라, 실제 손끝은 **약 40 mm 아래**(모델 44 mm)다. 모르고 파지 높이를 정했다가 **손끝이 탁자를 파고들었다**(`effort` 6.194 N·m). 수치·파지 높이 산정법·재발 방지 절차는 [grasp_center_frame.md](../gripper/grasp_center_frame.md). **EE 프레임 B안(§6.2)이 되면 사라지는 문제다** |
| **작은 데카르트 지령이 안 먹는다** | 수평 185 mm 이동은 **95.7 %** 도달인데, 8.2 mm 보정은 29 %, 5.8 mm 보정은 **10 %** 만 움직였다. 비례 오차가 아니라 **작은 변위에서의 무응답**이라 같은 보정을 반복해도 5 mm 근처에서 선다. 과보정은 벤더 A-1 의 발진 경로다 — **물러났다 다시 접근하는 편이 안전하다.** 실측: [grasp_center_frame.md §5](../gripper/grasp_center_frame.md) |
| 실시간 배속 | 시뮬레이터가 실시간보다 느리면 벽시계 타임스탬프와 실제 진행이 어긋나 데이터셋 시간축이 왜곡된다. 배속 확인 필요 |
| ~~scene~~ | **완료 (2026-09-08).** 시뮬레이터가 `static="false"` 인 body 의 pose 를 `world → <body 이름>` TF 로 내보내고(벤더 B-12), `functionbay_scene_state_node` 가 `/scene/objects` 로 바꾼다. 이름·종류·크기는 `config/functionbay_scene.json` 이 채운다. **`/scene/reset` 은 제공하지 않는다** (런타임 배치 변경 수단 없음) |
| 카메라 | `/camera_image` remap 은 설계상 가능하나 **미실측** (`camera_info` 미제공 영향 포함) |

---

## 참고

- [펑션베이 — 남은 작업 (인수인계)](functionbay_open_work.md) — 실측 결과 · 중력 처짐 · 자기충돌 · servo 경로 · 시간 특성 · 이 문서의 정정 대기 항목
- [멀티 시뮬레이터 백엔드 설계안](multi_simulator_backend_design.md) — §5 백엔드 계약, §6.3 연결형 백엔드
- [robot_control launch README](../../src/robot_control/launch/README.md)
- [MoveGroupJgpcClient 사용 가이드](../moveit/MoveGroupJgpcClient_UserGuide.md) — 스트리밍 실행, `publish_rate`
- [`grasp_center` 는 손끝이 아니다](../gripper/grasp_center_frame.md) — 파지 높이 산정, 손끝 오프셋 실측, 작은 지령 데드밴드
- [rdfp launch README §6.1](../../src/rdfp/launch/README.md) — 수집 계층 조합
