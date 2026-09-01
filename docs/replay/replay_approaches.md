# 재생(replay) 방법 비교 — twist vs pose vs joint

녹화된 로봇 동작을 다시 재생할 때, **무엇을 명령(command)하느냐** 에 따라 재현 충실도가 크게 달라진다. 이 문서는 "녹화된 동작을 드리프트 없이 재생하려면 어떻게 해야 하는가" 를 정리한다.

명령 대상은 크게 둘로 갈린다 — **EE 경로를 재생**하거나(A·B), **관절을 재생**한다(C).
앞의 둘은 로봇이 달라도 옮길 여지가 있는 대신 팔꿈치 형상을 맞추지 못하고, 뒤는 그
반대다.

## 핵심 원리: 위치 명령은 폐루프, 속도 명령은 개루프

- **twist(속도) 명령** — Servo 가 `q̇ = J⁻¹·v` 를 적분해 위치를 만든다. 매 주기의 명령이 "얼마나 빨리" 이므로, 오차를 되돌릴 기준 위치가 없다. 미세 오차(노이즈, 이산화, 타이밍, Jacobian 불일치)가 시간에 따라 **누적(drift)** 된다.
- **pose(위치) 명령** — 매 주기 `error = target_pose − current_pose` 를 계산해 그 오차를 줄이는 방향으로 움직인다. 목표 위치 자체가 기준이므로 **오차가 스스로 교정** 된다 → 드리프트 없음.

즉 되먹임(feedback) 대상이 "속도" 냐 "위치" 냐의 차이다.

## 방법 A) Servo Pose Tracking (실시간 폐루프)

MoveIt Servo 에는 velocity(delta_twist) 모드 외에 **pose tracking 모드** (`moveit_servo` 의 PoseTracking)가 있다. 목표 `PoseStamped` 를 토픽으로 흘려주면 Servo 가 다음을 수행한다.

```
매 제어주기:  error = target_pose − current_ee_pose(FK)
             v_cmd = Kp_lin·error_pos + Kp_ang·error_rot   (PID)
             q̇ = J⁻¹·v_cmd  → 컨트롤러로
```

- 녹화된 EE pose 스트림을 순서대로 `target_pose` 로 발행하면 각 pose 를 추종한다.
- **폐루프** 라 pose 오차를 계속 교정 → 장시간에도 드리프트 없음.

**특징 / 한계**

- 실시간·반응형이라 목표가 계속 바뀌어도 된다(온라인 replay 에 적합).
- PID 게인 튜닝이 필요하다. 빠른 구간에선 **정상상태 lag**(목표보다 살짝 뒤처짐)이 생길 수 있다.
- 여전히 6-DOF 명령이라 **팔꿈치(여유자유도) 형상은 원본과 다를 수 있다** — Servo 가 null-space 를 자체적으로 푼다.
- 특이점 / 관절한계 근처에서 속도 스케일다운·halt 가 일어난다.

**repo 상태 (Humble — 실측 2026-08-18)**: `/servo_node` 는 이미 런치에서 뜨지만 **pose 를 받지 못한다.** 설정만 바꿔서 되는 일이 아니다.

- `servo_node` 의 명령 입력 토픽은 두 개뿐이다 —
  `cartesian_command_in_topic: ~/delta_twist_cmds`, `joint_command_in_topic: ~/delta_joint_cmds`
  (`/opt/ros/humble/share/moveit_servo/config/panda_simulated_config.yaml`).
  pose 토픽도, 모드를 바꾸는 파라미터도 없다.
- pose tracking 은 **C++ 라이브러리 클래스**로만 제공된다
  (`moveit_servo/pose_tracking.h`, `libpose_tracking.so`). `PoseStamped` 구독자는
  `servo_node` 가 아니라 **그 클래스가** 만든다.
- 그래서 공식 예제 런치(`pose_tracking_example.launch.py`)도 `servo_node` 가 아니라
  **`servo_pose_tracking_demo`** 라는 별개 실행 파일을 띄운다.

따라서 Humble 에서 이 방법을 쓰려면 `moveit_servo::PoseTracking` 을 품은 **C++ 노드를
새로 작성**해야 한다. 파이썬 경로는 없다. 비용이 "YAML 한 줄"이 아니라 "노드 신규 개발"이다.

**Jazzy 이상**: Servo 가 재작성되어 JOINT_JOG / TWIST / **POSE** 가 `servo_node` 의 1급 명령 타입이 되고, 명령 타입 전환 서비스와 pose 목표 토픽이 노드 자체에 노출된다(구형 `PoseTracking` 클래스는 제거됨). 그 배포판에서는 실제로 "구성"에 가까워진다 — 다만 이 워크스페이스에서 검증한 것이 아니므로, 올릴 때 `ros2 topic list` 로 `servo_node` 의 토픽을 직접 확인한다.

**적합한 경우**: 녹화 pose 가 스트림으로 계속 들어오고, 실시간으로 따라가게 하고 싶을 때.

## 방법 B) MoveIt Cartesian Path (계획 후 실행)

녹화된 EE pose 들을 **waypoint 리스트** 로 넘겨 `computeCartesianPath` 로 직교공간 직선보간 궤적을 미리 생성한 뒤, JointTrajectoryController 로 실행한다.

```
녹화 EE poses  →  computeCartesianPath(waypoints)  →  관절 궤적(q(t)) 생성
              →  panda_arm_controller 로 실행 (관절 폐루프 위치제어)
```

- 실행이 **관절 위치제어** 라 정확하고 드리프트 없음. 궤적을 미리 시간 파라미터화(속도 / 가속 제한 반영)한다.

**특징 / 한계**

- **plan-then-execute** (반응형 아님). 알려진 녹화 경로 전체를 한 번에 계획·실행하는데 최적이다.
- waypoint 간격이 너무 크거나 특이점 부근이면 `computeCartesianPath` 가 **fraction < 1.0**(경로 일부만 생성)으로 실패할 수 있다. 다만 **촘촘히 준다고 좋아지지도 않는다** — 아래 "고주기 녹화" 참고.
- IK 를 waypoint 마다 풀어 관절해를 정하므로 **팔꿈치 형상은 원본과 다를 수 있다** (유효하고 일관된 해이긴 하다).
- **녹화된 속도 프로파일이 재현되지 않는다.** 요청에 시각 필드가 없어(아래) MoveIt 이 속도·가속 한계로 시간을 새로 매긴다. 어디서 느렸고 어디서 빨랐는지가 사라진다.

### 고주기(50 Hz 이상) 녹화를 그대로 넣으면

waypoint 수가 문제가 될 것 같지만, **막히는 곳은 계산량이 아니다.**

**계산량은 견딜 만하다.** `max_step` 기본값이 `0.01`(1 cm)인데, 50 Hz 녹화면 EE 가 1 m/s 로 움직여도 waypoint 간격이 2 cm 고 보통은 훨씬 촘촘하다. **간격이 `max_step`보다 작으면 추가 보간이 일어나지 않으므로** IK 호출 수는 waypoint 수와 같다 — 60초 녹화면 3,000번이다. 수 초 걸릴 수 있으나 실패하지는 않는다. 서비스 메시지 크기도 3,000 pose ≈ 170 KB 로 문제없다.

**진짜 문제 ① — 시간 정보가 전달되지 않는다.** `GetCartesianPath` 요청은 다음이 전부다.

```
geometry_msgs/Pose[] waypoints    ← 타임스탬프가 없다
float64 max_step
float64 jump_threshold
```

`Pose` 배열일 뿐 시각이 없다. 녹화 **케이던스**(샘플들이 찍힌 시각의 열 — 속도는 "관절값 변화량 ÷ 시간 간격"이므로 속도 프로파일이 여기 담긴다)가 **입력 단계에서 통째로 버려진다.** waypoint 를 아무리 촘촘히 줘도 복원되지 않는다.

> "주기"가 아니라 "케이던스"라고 쓰는 이유는 간격이 고정이 아니기 때문이다 `target_joint_cmds_publisher` 는 `header.stamp` 를 **수신 시각**으로 채우므로 DDS 전달 지연이 섞인다 — 50 Hz 근처에서 흔들리는 간격들의 열이지 20 ms 고정이 아니다.

**진짜 문제 ② — 출력이 항상 ~10 Hz 로 리샘플된다.** MoveIt 은 시간 파라미터화 단계(TOTG, `resample_dt = 0.1`)에서 궤적을 0.1초 격자로 리샘플한다. 요청에 이를 바꿀 필드가 없고, `max_step` 을 줄여도 리샘플이 그 뒤에 일어나므로 point 수는 변하지 않는다 (실측 근거: [MoveGroupJgpcClient_UserGuide.md](../moveit/MoveGroupJgpcClient_UserGuide.md)).

> 즉 **3,000개를 넣든 300개를 넣든 출력 궤적의 밀도는 같다.** 50 Hz 로 녹화한 시간 해상도가 두 번 뭉개진다 — 입력에서 버려지고, 출력에서 10 Hz 로 깎인다. JTC 는 컨트롤러가 point 사이를 보간해 육안으로 드러나지 않지만, JGPC 스택에서는 10 Hz 계단 명령으로 그대로 보인다.

**부수적으로 — fraction 실패의 디버깅이 어려워진다.** waypoint 가 많을수록 특이점 근처를 지날 확률이 오르는데, 응답은 `fraction` 숫자 하나뿐이라 **어디서 잘렸는지 알려주지 않는다.** 3,000개 중 실패 지점을 찾으려면 이분 탐색을 해야 한다.

**대응**

- **녹화 케이던스를 살리는 것이 목적이면 B 가 아니라 [C](#방법-c-joint-replay-관절-위치-직접-재생) 다.** C 는 리샘플이 없고 녹화 케이던스가 곧 재생 속도다.
- B 를 써야 한다면 **데시메이션한다.** 출력이 어차피 10 Hz 이므로 입력을 10~20 Hz 로 줄여도 잃는 것이 없고, IK 호출이 1/3~1/5 로 줄어 계획 시간과 fraction 실패 위험이 함께 내려간다. 균일 간격이 아니라 **거리·각도 기반**으로 해야 코너가 살아남는다 — 방법과 임계값은 [부록: 데시메이션](#부록-데시메이션) 참고.

**repo 상태**: 이미 API 가 있다 — `src/robot_control/robot_control/moveit/move_group_client.py` 의 `follow_trajectory(waypoints: list[Pose])` / `follow_trajectory_async` / `plan_trajectory` (CLAUDE.md 에도 "cartesian planning + execution" 으로 명시).
녹화 pose 를 `Pose` 리스트로 만들어 넘기면 그대로 동작한다.

**적합한 경우**: 녹화된 Cartesian 경로가 이미 확보돼 있고, 부드럽고 검증된 한 번의 replay 를 원할 때.

## 방법 C) Joint replay (관절 위치 직접 재생)

EE 를 거치지 않고 **녹화된 관절 위치를 그대로 arm 컨트롤러에 명령한다.** IK 를 풀지 않으므로 팔꿈치까지 원본과 같아진다.

```
녹화: arm 명령  →  target_joint_cmds_publisher  →  /target_joint_cmds (JointState)
재생: /target_joint_cmds  →  target_joint_cmds_executor
                          →  JointTrajectory(point 1개)  →  panda_arm_controller
```

**무엇을 녹화하는지가 A·B 와 다르다.** A·B 는 EE pose 를 재생하지만 C 는 **관절 명령**을 재생한다. 이때 `/joint_states`(관측값)가 아니라 `/target_joint_cmds`(명령값)를 쓰는 것이 규약이다 — 관측값은 컨트롤러가 따라간 *결과*라 명령으로 되먹이면 한 주기씩 지연이 섞인다. `target_joint_cmds_publisher` 가 JTC 의 `JointTrajectory` 와 JGPC의 `Float64MultiArray` 를 **하나의 `sensor_msgs/JointState`** 로 통일해 주므로, 스택이 달라도 재생 경로는 같다.

**특징 / 한계**

- **재현 충실도가 가장 높다.** 7-DOF 관절값을 그대로 주므로 여유자유도(팔꿈치 형상)가 원본과 일치한다. A·B 는 6-DOF EE 명령이라 여기까지는 맞출 수 없다.
- 컨트롤러가 관절 위치 폐루프라 **드리프트가 없다.**
- **로봇이 바뀌면 쓸 수 없다.** 관절값은 그 기구학에만 유효하다. EE 경로를 재생하는 A·B 는 다른 팔로 옮길 여지가 있지만 C 는 없다.
- **반응형이 아니다.** 물체가 녹화 때와 다른 곳에 있어도 같은 관절 궤적을 그대로 따라간다 — 허공을 집는다. scene 이 고정된 재생에만 맞는다.
- **시작 자세가 다르면 첫 명령에서 튄다.** 관절 *절대* 위치를 명령하므로 첫 point 로 급격히 이동하려 한다. 재생 전에 녹화 시작 자세로 먼저 옮겨 놓아야 한다.
- **속도는 발행 주기가 정한다.** executor 는 `time_from_start: 0.0` 인 point 1개를 보내 컨트롤러가 즉시 반영하게 하므로, 재생기가 빠르게 뿌리면 로봇도 빨라진다. 배속 재생이 그대로 동작 속도가 된다. 뒤집어 말하면 **녹화 케이던스대로 뿌리면 원본 속도가 그대로 재현된다** — 시각을 버리고 리샘플하는 B 와 정반대다. 고주기(50 Hz 이상) 녹화의 시간 해상도를 살리려면 이 방법을 쓴다.
- **DB 재생 경로에서는 joint 이름이 유실된다.** `joint_states` 테이블이 이름을 저장하지 않아 `JointState.name` 이 비어 온다. executor 의 `joint_names` 파라미터가 폴백이며, `replay_panda_mock.launch.py` 가 panda_joint1~7 을 넘긴다.

**repo 상태**: 그대로 쓸 수 있다.

```bash
ros2 launch rdfp replay_panda_mock.launch.py replay_arm_path:=target_joint_cmds
ros2 run rdfp replay <episode_id> --topic /target_joint_cmds
```

`replay` CLI 의 `--topic` **기본값은 `/servo_node/delta_twist_cmds`** 라 이 경로와 맞지 않는다. 위처럼 명시해야 한다 — 기본값 그대로 두면 executor 에 아무것도 도착하지 않고 로봇은 움직이지 않는다.

**적합한 경우**: 같은 로봇·같은 scene 에서 동작을 있는 그대로 되살릴 때(데모 재생, teaching 동작 재현). 팔꿈치 형상까지 맞아야 한다면 사실상 이 방법뿐이다.

## 네 방법 비교

| 방식 | 명령 종류 | 루프 | 드리프트 | 관절(팔꿈치) 재현 | 성격 | repo 지원 |
|---|---|---|---|---|---|---|
| delta_twist (twist) | 속도 | 개루프 적분 | ❌ 누적 | ❌ | 실시간 | `EeTwistPublisher`(모니터링용) |
| **Servo pose tracking** | 위치 | 폐루프 | ✅ 없음 | ❌ | 실시간·반응형 | **Humble: C++ 노드 신규 작성 필요** (Jazzy 부터 `servo_node` 기본 지원) |
| **Cartesian path** | 위치(궤적) | 폐루프 | ✅ 없음 | ❌ | 계획 후 실행 | `follow_trajectory` (JTC 구현 기준. JGPC 구현은 개루프) |
| **Joint replay** | 관절 위치 | 폐루프 | ✅ 없음 | ✅ 정확 | 재생 전용 | `target_joint_cmds_executor` (`replay_arm_path:=target_joint_cmds`) |

## 요약 및 권장

무엇을 지켜야 하는지가 선택을 정한다.

| 지켜야 하는 것 | 선택 | 근거 |
|---|---|---|
| 같은 로봇에서 **동작을 있는 그대로** | **C (joint replay)** | 팔꿈치까지 맞는 유일한 방법. 이 repo 에서 인자 하나로 바로 된다 |
| **녹화 속도 프로파일**까지 (고주기 녹화) | **C (joint replay)** | B 는 시각을 버리고 출력을 ~10 Hz 로 리샘플한다. C 는 녹화 케이던스가 곧 재생 속도다 |
| **EE 경로**만 (다른 팔로 옮길 여지 포함) | **B (Cartesian path)** | `follow_trajectory` 가 이미 있다. 가장 손쉽게 검증 가능한 출발점 |
| 목표가 **실시간으로 계속 바뀜** | A (Servo pose tracking) | 단 Humble 에서는 C++ 노드 신규 작성이 필요하다 |
| — | ~~delta_twist~~ | 개루프 적분이라 드리프트가 원리적으로 누적된다. 재생용으로 쓰지 않는다 |

- **A, B 는 EE 경로만 맞추고 팔꿈치(여유자유도)까지는 원본과 다를 수 있다.** 6-DOF 명령으로 7-DOF 를 지정할 수 없기 때문이며, 튜닝으로 해결되는 문제가 아니다.
- 반대로 **C 는 scene 이 바뀌면 대응하지 못한다.** 물체가 다른 곳에 있어도 같은 관절 궤적을 따라가므로, 물체 위치가 매번 달라지는 수집 데이터의 재생에는 맞지 않는다.
- A 와 B 의 구현 비용은 이 워크스페이스(Humble)에서 **대등하지 않다** — B 는 기존 API 호출이고 A 는 노드 신규 개발이다.

## 참고: 왜 twist → delta_twist_cmds 는 replay 에 부적합한가

녹화된 joint_states 를 다시 읽어 `EeTwistPublisher` 로 twist 를 만들고 `/delta_twist_cmds` 에 발행하는 방식은 다음 이유로 원래 동작을 충실히 재현하지 못한다.

1. **적분 드리프트** — 속도 명령만 주면 위치 오차를 되돌릴 기준이 없어 오차가 시간에 따라 누적된다(원리적 한계).
2. **절대 위치 부재** — twist 는 "얼마나 빨리" 만 담고 "어디에" 는 담지 않는다. 시작 pose 가 다르면 전체 재생이 오프셋된다.
3. **여유자유도(7-DOF) 미재현** — EE twist 는 6-DOF 라 팔꿈치 null-space 가 원본과 달라진다.
4. **특이점 / 한계 스케일링, 타이밍, 미분 노이즈** — 모두 오차로 적분되어 lag·이탈을 만든다.

따라서 `EeTwistPublisher` 의 twist 는 **모니터링 / 학습 데이터(관측 EE 속도)** 용으로 쓰고, 로봇을 움직이는 replay 명령 소스로는 사용하지 않는다.

## 부록: 데시메이션

신호처리 용어로 **샘플 수를 솎아내 줄이는 것**이다(다운샘플링과 같은 뜻). 여기서는 50 Hz 로 녹화한 3,000개 waypoint 중 일부만 골라 300개로 줄여 `computeCartesianPath`에 넘긴다는 뜻이다. 어차피 출력이 ~10 Hz 로 리샘플되므로 결과는 같으면서 IK 호출만 1/10 로 줄어든다.

### 균일 데시메이션 — 쉽지만 모양이 망가진다

"5개마다 하나만 남긴다"(`poses[::5]`). 한 줄이면 되지만 **경로의 모양을 고려하지 않는다.**

```
원본:   ●●●●●●●●●●●●●●●●●●●●
        ─────────────┐
                     └──────      직선 구간은 점이 남아돌고
                                  코너는 점 하나가 아쉽다

균일:   ●    ●    ●    ●    ●
        ─────────────┐
                  ╲  └──          코너 점이 빠지면 모서리가 잘린다
```

직선 구간에는 필요 없는 점을 남기고, 정작 중요한 코너의 점을 버릴 수 있다.

### 거리·각도 기반 — 이쪽을 쓴다

"직전에 남긴 점에서 **1 cm 이상 떨어졌거나 자세가 2° 이상 돌아갔을 때만** 남긴다." 직선 구간은 알아서 성겨지고 코너에는 점이 몰린다. 점 수를 줄이면서 경로 모양을 유지하는 것이 목적이므로 이 기준이 맞다.

임계값을 `max_step`(기본 1 cm)보다 작게 잡을 이유는 없다 — 그보다 촘촘해도 MoveIt 이 추가 보간을 하지 않기 때문이다(§방법 B).

### 제대로 하려면 — RDP

폴리라인 단순화의 표준 알고리즘은 **Ramer–Douglas–Peucker(RDP)** 다. "원본 경로에서 최대 ε 이상 벗어나지 않는 선에서 점을 최대한 버린다"를 보장한다.

**주의: 위치만 보면 안 된다.** 자세(쿼터니언)도 함께 판정해야 한다 — 위치는 거의 그대로인데 손목만 도는 구간이 통째로 사라질 수 있다.

실무적으로는 위의 "위치 1 cm / 각도 2°" 정도의 단순한 규칙으로도 충분한 효과를 본다.

## 관련 문서

- `src/robot_control/robot_control/moveit/move_group_client.py` — `follow_trajectory` /
  `plan_trajectory` (Cartesian path).
- `docs/moveit/MoveGroupClient_UserGuide.md`
- `docs/moveit/servo_client_programmers_guide.md`
- `src/robot_control/robot_control/moveit/target_joint_cmds_executor.py` — joint 위치 replay 경로. `/target_joint_cmds` (`sensor_msgs/JointState`) 를
  point 1개짜리 `JointTrajectory` 로 감싸 `panda_arm_controller` 로 보낸다.
  (구형 `target_joint_states_executor.py` 는 `rdfp_msgs/TargetJointStates` 를 쓰며
  구현만 남아 있고 어느 launch 에서도 기동하지 않는다.)
- `src/rdfp/launch/replay_panda_mock.launch.py` — replay 전용 런치 스택.
