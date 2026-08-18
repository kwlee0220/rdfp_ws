# Leader-Follower 이기종 로봇 동작 미러링 설계안

> 📖 teleop 문서 진입점: [README.md](README.md) — 하려는 일부터 문서를 고른다.

leader 로봇을 사람이 직접 움직이고, follower 로봇이 그 동작을 실시간으로
따라 하게(mirroring) 만드는 teleoperation 시스템의 설계안이다.

- **구현 환경**: Ubuntu 22.04 + ROS 2 Humble
- **전제**: leader 와 follower 는 **관절 체계(관절 수·길이·구조)가 서로 다르다.**
- **leader 인터페이스**: `joint_states` 토픽과 `/tf` 토픽을 **50Hz** 로 발행한다.
- **follower**: 본 워크스페이스의 Panda + MoveIt2 스택 (`moveit_servo` 포함).

## 1. 핵심 아이디어 — Cartesian Retargeting

관절 체계가 다르면 관절값을 그대로 복사하는 joint-space 미러링은 원리적으로
불가능하다. 따라서 두 로봇의 **공통 언어인 task-space(EE pose)** 로 변환하여
미러링한다.

```
"leader 의 관절값" 이 아니라 "leader 의 손끝(EE) 움직임" 을 따라 한다.
```

leader 와 follower 가 아무리 달라도 EE pose 는 둘 다 6-DOF SE(3) 로 표현되므로
이 수준에서는 1:1 매핑이 가능하다.

## 2. 전체 파이프라인

```
[Leader] ──/tf (50Hz, 이미 발행 중)──▶
ee_pose_node (기존 노드 재사용, base_frame/ee_frame 만 leader 프레임으로) ──▶ /leader/ee_pose
   │
   ▼
teleop_retarget_node (신규) ── 클러치·스케일·R_align·필터 ──▶ /follower/target_pose
   │
   ▼
EeTwistPublisher (source:=ee_pose, ee_pose_topic:=/follower/target_pose)
   │                                   ──▶ /servo_node/delta_twist_cmds
   ▼
moveit_servo (follower MoveIt config) ──▶ follower JointTrajectoryController
```

> **leader `/tf` 가 같은 ROS 그래프에 없는 경우** — 위 다이어그램은 leader 의
> `/tf` 를 직접 조회할 수 있다고 전제한다. OMY-L100 처럼 leader 가 다른 ROS
> 배포판/RMW(Jazzy + `rmw_zenoh`)에서 동작하면 이 전제가 성립하지 않는다. 그때는
> 외부 어댑터(별도 저장소 `omy_leader_bridge`)가 `/leader/ee_pose` 를 직접
> 발행한다. **`teleop_mirror.launch.py` 는 이 토픽의 발행자를 만들지 않는 것이
> 기본 구성이다** — 외부 공급이 일반적이고, 런치가 발행자를 함께 만들면 둘이
> 충돌하기 때문이다. leader `/tf` 를 직접 조회해야 하는 경우에만 `ee_pose_node`
> 를 별도로 띄운다(§3.1). 계약 상세는
> [external_input_adapters.md](external_input_adapters.md) 참고.

새로 작성해야 하는 노드는 `teleop_retarget_node` **하나** 다. 나머지는 본
워크스페이스의 기존 자산을 재사용한다.

| 파이프라인 단계 | 담당 | 상태 |
|---|---|---|
| leader FK (joint → EE pose) | leader 가 `/tf` 로 이미 제공 | 불필요 (제공됨) |
| TF lookup → PoseStamped | `ee_pose_node` (`rdfp.moveit.ee_pose_publisher`) | **재사용** (파라미터만 변경) |
| retargeting (클러치/스케일/필터) | `teleop_retarget_node` | **구현됨** (`ros2 run rdfp teleop_retarget`) |
| pose 스트림 → twist | `EeTwistPublisher` (`rdfp.moveit.ee_twist_publisher`) | **재사용** (`source:=ee_pose`) |
| twist → 관절 속도 | `moveit_servo` (`/servo_node/delta_twist_cmds`) | 기존 스택 |

전체 체인은 `teleop_mirror.launch.py` 로 일괄 기동한다 (follower Panda +
MoveIt2 스택은 별도로 먼저 실행). 가짜 leader TF(50Hz 사인파) + 정지 follower
pose 하니스로 launch 전체 체인을 검증했다 — twist 최대값이 이론값
(진폭 0.05m·0.2Hz → 0.0628 m/s × gain 2.5 ≈ 0.157)과 일치.

```bash
ros2 launch rdfp teleop_mirror.launch.py \
    position_scale:=1.0 align_yaw:=0.0 \
    workspace_min:="[0.1, -0.5, 0.05]" workspace_max:="[0.8, 0.5, 0.9]"

ros2 service call /teleop_retarget/clutch std_srvs/srv/SetBool "{data: true}"   # engage
ros2 service call /teleop_retarget/clutch std_srvs/srv/SetBool "{data: false}"  # disengage
```

**Servo 단위 주의**: 본 워크스페이스 servo 설정은 `command_in_type: unitless`
(scale.linear=0.4 m/s, scale.rotational=0.8 rad/s) 이므로, launch 의 twist 게인
기본값은 물리 단위 → unitless 변환치인 `twist_linear_gain=2.5`,
`twist_angular_gain=1.25` 다. servo 를 `speed_units` 로 바꾸면 둘 다 1.0 으로
지정한다.

## 3. 단계별 설계

### 3.1 Leader EE pose 추출

leader 가 `/tf` 를 이미 발행하므로 FK 를 직접 계산할 필요가 없다. 기존
`ee_pose_node` 가 하는 일이 정확히 "TF 에서 base→ee lookup → PoseStamped 발행"
이므로 파라미터만 바꿔 재사용한다.

```bash
ros2 run rdfp ee_pose_node --ros-args \
  -p base_frame:=<leader_base_frame> \
  -p ee_frame:=<leader_ee_frame> \
  -p publish_rate:=50.0 \
  -r ee_pose:=/leader/ee_pose
```

leader 의 `joint_states` 는 이 경로에서 사용하지 않는다 (TF 에 이미 FK 결과가
들어 있다).

### 3.2 Retargeting 노드 (핵심, 신규 작성)

leader 와 follower 는 base 위치·작업공간 크기·EE 방향 관례가 다르므로 **절대
pose 를 그대로 넘기면 안 되고, "상대 변위" 매핑** 을 쓴다. 표준 teleop 방식:

```
클러치(engage) 시점에 저장:  L₀ = leader EE pose,  F₀ = follower EE pose

이후 매 샘플:
  Δp = R_align · (p_L − p_L₀)            # leader 위치 변위를 follower 좌표계로 회전 정렬
  p_target = p_F₀ + s · Δp               # s = 위치 스케일 (작업공간 크기 비율)
  ΔR = R_align · R_L·R_L₀⁻¹ · R_align⁻¹  # 상대 회전을 좌표 정렬
  R_target = ΔR · R_F₀
```

이 방식의 구성 요소:

- **클러치(clutching)** — 사람이 스페이스바 등으로 engage/disengage 한다.
  leader 를 편한 자세로 되돌린 뒤 다시 engage 하면 작업공간 매핑을 "다시 잡을"
  수 있다 (마우스를 들어 옮기는 것과 동일). 서로 다른 작업공간 크기 문제의
  실질적 해법이다.
- **위치 스케일 `s`** — 작은 leader → 큰 follower 확대 매핑 등 작업공간 크기
  비율을 보정한다.
- **`R_align` 캘리브레이션** — "leader 의 앞" 과 "follower 의 앞" 을 정렬한다.
  좌우 반전 거울상(mirror-image)을 원하면 여기서 y 축 부호만 뒤집으면 된다.
- **필터링** — 사람 손떨림 + FK 노이즈 제거용 low-pass. teleop 에서
  지연/떨림 균형이 좋은 One-Euro filter 를 권장한다. 필터를 **차분(twist 생성)
  전 단계인 retarget 에 두므로** 이후 생성되는 twist 도 함께 평활화된다.

### 3.3 Follower 구동 — 3가지 선택지

| 방식 | 원리 | 장점 | 단점 |
|---|---|---|---|
| **(a) Servo + twist 스트리밍** | target_pose 를 차분 → `/delta_twist_cmds` | Humble 에서 토픽만으로 동작, 기존 스택 그대로 | 개루프 → 드리프트. 단 **teleop 은 사람이 눈으로 보정하는 폐루프** 라 실용상 견딜 만함 |
| **(b) Servo PoseTracking (최종 권장)** | 매 주기 `error = target − 현재` 를 PID → servo | **폐루프, 드리프트 없음**. 특이점/한계 보호 내장 | Humble 에선 토픽 인터페이스가 아니라 **C++ API**(`moveit_servo::PoseTracking`) → 작은 C++ 노드 작성 필요 |
| **(c) IK 스트리밍** | 매 샘플 IK(bio_ik / TRAC-IK) → 관절 위치 스트림 | Servo 불필요, 제어 단순 | IK 해 점프(관절 불연속) 처리, 속도 제한, smoothing 을 직접 구현해야 함 |

**권장: (b) 를 목표로 하되, 1차 프로토타입은 (a) 로 시작한다.**

- (a) 는 본 워크스페이스에 거의 다 있다 — `EeTwistPublisher`(`source:=ee_pose`)
  가 정확히 "pose 스트림 → twist" 변환기이고, servo 스택과 `teleop_keyboard` 가
  `/servo_node/delta_twist_cmds` 경로를 이미 검증했다. **retarget 노드 하나만
  새로 쓰면 되는 구성** 이다.
- 장시간 정밀 미러링이 필요해지면 (b) 로 승격한다. 폐루프/드리프트 논리는
  [../replay/cartesian_path_replay_approaches.md](../replay/cartesian_path_replay_approaches.md)
  참고.

## 4. 50Hz 기준 설계 확정값

leader `/tf` 가 50Hz(주기 20ms)로 확인되었다. 사람 팔 동작의 유효 대역폭이
5~10Hz 수준이므로 teleop 미러링에 충분하다.

| 단계 | 설정 | 근거 |
|---|---|---|
| `ee_pose_node` (leader) | `publish_rate:=50.0` | TF 가 50Hz 인데 더 높여봐야 같은 stamp 의 중복 pose 만 나옴 |
| `EeTwistPublisher` | 그대로 사용 | 중복 stamp 는 `dt <= 0` 가드가 걸러냄. `max_dt` 기본 1.0s 도 충분 |
| retarget 필터 | LPF cutoff 2~5Hz (또는 One-Euro `min_cutoff ≈ 1.5`) | 50Hz 샘플링에서 손떨림(8~12Hz) 제거, 지연 최소화 |
| Servo | `incoming_command_timeout ≈ 0.1s` | 명령 주기 20ms 의 5배 여유 — 2~3 프레임 유실은 견디고, 그 이상 끊기면 정지 |
| 총 지연 예상 | 약 50~80ms | 샘플 20ms + 필터 지연 + servo 주기. 시각 피드백 teleop 으로 무리 없는 수준 |

## 5. 안전장치 (필수)

1. **속도/가속 상한** — retarget 출력에 clamp (Servo 에도 한계가 있지만 이중으로).
2. **작업공간 박스 제한** — target_pose 가 follower 안전영역 밖이면 경계에 clamp.
3. **데드맨 스위치** — 클러치 해제·leader 토픽 끊김(watchdog timeout) 시 즉시
   정지 twist 를 발행한다.
4. **점프 감지** — engage 직후 또는 통신 끊김 후 target 이 현재 pose 에서 멀면
   무시하거나 천천히 수렴시킨다. Servo pose tracking 이면 자동, twist 방식이면
   직접 구현해야 한다.
5. **Servo 내장 보호 활용** — 특이점/관절한계 스케일다운, 충돌 체크.

## 6. 한계 (미리 인지할 것)

- **EE 만 미러링된다.** 팔꿈치 등 관절 형상은 follower 의 IK/null-space 가
  알아서 정한다 — 관절 체계가 다르므로 원리적으로 불가피하다.
- leader 가 follower 보다 도달범위가 넓으면 follower 가 못 따라가는 영역이
  생긴다 → 클러치 + 스케일로 운용상 회피한다.
- 그리퍼는 별도 채널로 매핑한다 (leader 그리퍼 상태 →
  `/gripper_control/gripper_cmds`).

## 7. 통합 전 확인 사항

같은 ROS 도메인에서 leader 와 follower(Panda 스택)가 공존하므로 아래를
확인해야 한다. 두 가지 모두 **구현을 막는 요소는 아니다** (런타임 파라미터로
분리 가능).

### 7.1 leader TF 프레임 이름 충돌 (미확인)

leader 의 base/EE 프레임 이름이 follower(Panda: `panda_link0`~`panda_link8`,
`panda_hand`, `world`)와 겹치면 두 TF 트리가 잘못 합쳐져 lookup 이 오염된다.
leader 가 `world` 나 `base_link` 같은 흔한 이름을 쓰면 충돌한다.

```bash
ros2 run tf2_tools view_frames    # frames.pdf 생성 — 트리 구조와 프레임 이름 확인
```

충돌 시 대안: leader 드라이버에 frame_prefix 설정, 또는
도메인 분리(`ROS_DOMAIN_ID`) + `domain_bridge`.

### 7.2 `/joint_states` 토픽 혼선 (미확인)

leader 가 `/joint_states` 를 발행하면 follower 스택의 `joint_state_broadcaster`
출력과 같은 토픽에 섞인다. move_group / robot_state_publisher 는 모르는 관절
이름을 무시하므로 대체로 동작은 하지만 경고 로그가 계속 남는다. leader
드라이버를 네임스페이스(`/leader/joint_states`)로 띄울 수 있으면 그렇게 하는
것을 권장한다.

```bash
ros2 topic echo /joint_states --once   # 관절 이름 목록으로 어느 로봇 것인지 확인
ros2 topic hz /tf                      # 발행 주기 확인 (50Hz 확인 완료)
```

## 8. 구현 로드맵

1. **Follower 시뮬레이션 준비** — follower MoveIt config + mock hardware
   (본 워크스페이스의 panda mock 스택 그대로).
2. **Leader EE pose 체인 확인** — `ee_pose_node` 를 leader 프레임으로 띄워
   `/leader/ee_pose` 발행 확인 (`ros2 topic hz` 로 50Hz 확인).
3. **`teleop_retarget_node` 작성** — 클러치는 우선 서비스/키 입력으로 구현.
   상대 매핑 공식 등 순수 수학부는 ROS 없이 단위테스트한다.
4. **(a) 경로 연결** — `EeTwistPublisher` + servo 로 mock 환경에서 미러링 확인.
5. **필터/스케일 튜닝 → 실기 연결** — 필요 시 **(b) PoseTracking C++ 노드로
   승격** 한다.

## 9. 왜 twist(속도) 직접 미러링이 아닌가 (배경)

leader 의 EE 속도(twist)를 구해 그대로 `/delta_twist_cmds` 에 흘리는 단순한
구성은 다음 이유로 채택하지 않는다.

1. **적분 드리프트** — 속도 명령은 개루프 적분이라 위치 오차를 되돌릴 기준이
   없어 오차가 누적된다.
2. **절대 위치 부재** — twist 는 "얼마나 빨리" 만 담고 "어디에" 는 담지 않아
   시작 pose 차이가 그대로 오프셋으로 남는다.
3. **클러치/스케일 불가** — 속도 공간에는 작업공간 재정렬(클러치) 개념을 넣기
   어렵다.

단, teleop 에서는 사람이 시각으로 오차를 보정하는 "인간 폐루프" 가 있으므로
1차 프로토타입 (a) 안의 twist 스트리밍은 실용적으로 허용된다. 이때도 twist 는
retarget 된 target_pose 를 차분해 만들므로 클러치/스케일은 유지된다.
자세한 폐루프/개루프 논의는
[../replay/cartesian_path_replay_approaches.md](../replay/cartesian_path_replay_approaches.md)
를 참고한다.

### 9.1 그렇다면 `/tf → twist` 로 바로 만들면 안 되나? (자주 되묻게 되는 질문)

`PoseStamped → retarget → TwistStamped` 의 중간 pose 단계가 군더더기처럼 보여
"`/tf` 를 바로 차분해 twist 를 만들면 되지 않나" 하는 의문이 반복해서 든다.
결론은 **그 방식은 위 9절이 기각한 "속도 직접 미러링" 과 사실상 같으며, 이기종
전제에서는 권장하지 않는다** 이다.

**중간 pose 단계는 낭비가 아니라 알고리즘 그 자체다.** `retarget → twist` 에서
실제로 일을 하는 것은 twist 변환이 아니라 retarget(절대 pose 앵커링)이다. twist
는 servo 가 요구하는 마지막 전송 포맷일 뿐이다. retarget 의 각 요소를 좌표
성격으로 나누면:

| retarget 요소 | twist(속도) 공간에서 가능? | 이유 |
|---|---|---|
| `R_align` 좌표 정렬 | ✅ 가능 | 속도 벡터를 회전만 하면 됨 |
| 위치 스케일 `s` | ✅ 가능 | 속도에 상수를 곱함 |
| 클러치(재앵커) | ❌ 불가 | 절대 기준점 `L₀`,`F₀` 필요 |
| 작업공간 박스 clamp | ❌ 불가 | 절대 위치 필요 |
| 점프 가드 | ❌ 불가 | 절대 위치 필요 |

`/tf` 를 바로 차분하면 위 표의 ✅(R_align·scale)만 남기고 ❌ 를 통째로 버리는데,
이 설계의 존재 이유(1절: 이기종·작업공간 다름)를 해결하는 게 바로 그 ❌ 부분이다.

직접 twist 방식에서 구체적으로 잃는 것:

1. **클러치 불가** — 가장 치명적이다. 3.2절이 "작업공간 크기 문제의 실질적
   해법" 이라 부른 클러치는 절대 pose 기준점(`F₀`)이 있어야 성립한다. 속도만
   미러링하면 재앵커할 기준 자체가 없다.
2. **시작 오프셋 영구 잔존** — leader/follower 초기 pose 가 다르면(이기종이라
   당연히 다름) 속도 미러링으로는 그 차이가 사라지지 않는다.
3. **절대 작업공간 안전 박스 불가** — 속도 상한은 걸어도 "이 박스 밖으로 나가지
   마라" 는 위치 제약은 못 건다. follower 가 안전영역 밖으로 서서히 드리프트한다.
4. **드리프트** — 개루프 적분이라 위치 오차를 되돌릴 기준이 없다.

**코드상 이미 직접 경로가 존재한다.** `ee_twist_publisher.py` 의
`source=joint_states` 모드가 정확히 "TF 에서 pose 조회 → 차분 → twist" 를 한다.
leader 에 이 모드로 붙이고 retarget 을 빼면 그게 곧 직접 twist 미러링이다.
당장 돌릴 수는 있지만 위 4가지 문제를 그대로 안은 raw 속도 미러링이다.

**직접 twist 가 괜찮은 경우:** base 프레임·작업공간이 거의 같고, 시작 pose 를
정렬해두고, 클러치가 필요 없는 짧은 세션. 하지만 이 문서의 전제(이기종)는
정확히 이 조건이 깨지는 경우다.

**노드 수를 줄이려는 의도라면:** `ee_pose_node + teleop_retarget_node +
ee_twist_node` 3노드를 하나로 합치는 것은 합리적이다. 단 그 노드 내부에서도
흐름은 `/tf → leader pose → 절대 target(클러치·clamp·scale) → 차분 → twist`
순서를 유지해야 한다. 즉 노드는 합칠 수 있어도 "절대 target 을 만든 뒤
차분한다" 는 순서는 못 버린다. 절약되는 것은 노드 홉 1개(지연 수 ms)뿐이고
알고리즘은 그대로다.

## 10. 관련 자산

- `src/rdfp/launch/teleop_mirror.launch.py` — 미러링 체인 3종 일괄 기동 launch
  (leader `ee_pose_node` + `teleop_retarget` + `ee_twist_node`).
- `src/rdfp/rdfp/teleop/teleop_retarget_node.py` — retargeting 노드
  (`teleop_retarget` console_script). 클러치 서비스 `~/clutch`
  (std_srvs/SetBool), 상대 매핑 + 스케일 + R_align + LPF + 점프 가드 +
  watchdog + hold 재발행 + 작업공간 clamp.
- `src/rdfp/rdfp/teleop/retarget_math.py` — 매핑/쿼터니언 순수 수학부
  (ROS 미소싱 환경에서 단위테스트 가능,
  `rdfp/teleop/tests/test_retarget_math.py`).
- `src/rdfp/rdfp/moveit/ee_pose_publisher.py` — TF lookup → PoseStamped
  (`ee_pose_node`). leader EE pose 추출에 재사용.
- `src/rdfp/rdfp/moveit/ee_twist_publisher.py` — pose/joint_states →
  TwistStamped (`ee_twist_node`). `source:=ee_pose` 로 재사용.
- `src/rdfp/rdfp/teleop/teleop_keyboard.py` — `/servo_node/delta_twist_cmds`
  경로의 기존 검증 사례.
- `docs/moveit/servo_client_programmers_guide.md` — Servo 클라이언트 사용법.
- `docs/replay/cartesian_path_replay_approaches.md` — 폐루프/개루프,
  드리프트, pose tracking 배경 설명.
