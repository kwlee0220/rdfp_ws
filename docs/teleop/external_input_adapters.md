# 외부 입력 어댑터 계약

> 📖 teleop 문서 진입점: [README.md](README.md) — 하려는 일부터 문서를 고른다.

rdfp 로봇(`rdfp_panda_mock` / `rdfp_panda_jgpc_mock` 등)은 **입력 장치를 직접
알지 않는다.** 키보드든, 게임패드든, 별도 시스템의 리더 암이든, 정해진 토픽에
정해진 형식의 메시지를 지속적으로 발행하기만 하면 로봇이 움직인다.

이 문서는 그 **계약(contract)** 을 정의한다. 새 입력 장치를 붙이려는 쪽은 이
문서만 보면 되고, rdfp 는 어댑터의 구현·언어·ROS 배포판을 알 필요가 없다.

---

## 1. 왜 계약으로 분리하는가

입력 어댑터는 rdfp 와 **런타임을 공유할 수 없는 경우가 있다.**

| | rdfp | OMY-L100 |
|---|---|---|
| ROS 배포판 | Humble | **Jazzy** |
| RMW | `rmw_fastrtps_cpp` (기본) | **`rmw_zenoh_cpp`** |
| 제약 사유 | Isaac Sim 등 외부 시스템 연동 | 장비 자체 제약 |

같은 colcon 워크스페이스에 넣을 수도, 같은 오버레이를 공유할 수도 없다. 그래서
어댑터는 **별도 저장소로 유지하고 토픽 계약으로만 연결한다.**

---

## 2. 두 가지 진입 경로

```
[어댑터]                          [rdfp]

경로 A ─ TwistStamped ──▶ /servo_node/delta_twist_cmds
                              └─▶ servo_node ─▶ /panda_arm_controller/... ─▶ arm

경로 B ─ PoseStamped ───▶ /leader/ee_pose
                              └─▶ teleop_retarget ─▶ /follower/target_pose
                                    └─▶ ee_twist_publisher ─▶ 경로 A
```

### 경로 A — twist 직접 발행

가장 단순하다. EE 속도를 직접 만들 수 있는 입력(게임패드, 조이스틱)에 적합하다.

| | |
|---|---|
| 토픽 | `/servo_node/delta_twist_cmds` |
| 타입 | `geometry_msgs/TwistStamped` |
| `frame_id` | follower base frame (기본 `panda_link0`) |
| 예시 | [`teleop_keyboard`](../../src/rdfp/rdfp/teleop/teleop_keyboard.py) |

### 경로 B — pose 발행 후 retarget 경유

리더 암처럼 **절대 pose 를 내는 입력**에 적합하다. 좌표계·작업공간이 달라도
클러치 앵커 기반 상대 매핑으로 흡수된다.

| | |
|---|---|
| 토픽 | `/leader/ee_pose` (`teleop_mirror.launch.py` 의 `leader_pose_topic`) |
| 타입 | `geometry_msgs/PoseStamped` |
| `frame_id` | leader base frame |
| 예시 | OMY-L100 (외부 저장소 `omy_leader_bridge`) |

경로 B 를 쓸 때 rdfp 쪽은 이렇게 기동한다.

```bash
ros2 launch rdfp teleop_mirror.launch.py
```

**본 런치는 `leader_pose_topic` 의 발행자를 만들지 않는다.** 어댑터가 공급한다는
전제이므로, 어댑터가 죽으면 `teleop_retarget` 의 watchdog 이 클러치를 자동
해제한다 (토픽만 흐르고 값이 안 변하는 상태가 된다).

---

## 3. 공통 요구사항

아래 네 가지는 **어기면 에러 없이 조용히 로봇이 안 움직인다.** 디버깅이 매우
어려우므로 어댑터 구현 시 반드시 확인한다.

### 3-1. ROS 환경

```
ROS_DOMAIN_ID      = 31
RMW_IMPLEMENTATION = rmw_fastrtps_cpp
```

**중요한 것은 특정 RMW 가 아니라 "양쪽이 같을 것"이다.** `.ros2rc` 와 docker
이미지 모두 `rmw_fastrtps_cpp` 로 통일돼 있으므로 위 값이 표준이다. 다른 RMW 를
강제하는 외부 시스템과 붙일 때만 **양쪽 모두** 그쪽에 맞춘다.

**터미널·스크립트마다 직접 켜야 한다** — 셸을 여는 것만으로는 적용되지 않는다.
rdfp 개발 머신에서는 `rdfp_env` 함수가 이 값을 적용하고, 그 밖의 환경에서는 위
두 변수를 직접 export 한다.

둘 중 하나만 달라도 DDS 디스커버리가 실패하고, 증상은 "토픽이 아예 안 보인다"
이다. `ros2 daemon` 프로세스가 두 개 뜨면 환경이 갈린 것이다.

```bash
ps -eo args | grep ros2-daemon | grep -v grep
```

### 3-2. `header.stamp` 은 **소비자 시계 기준으로 신선** 해야 한다

`moveit_servo` 는 매 주기 이렇게 판정한다.

```
now - command.header.stamp >= incoming_command_timeout (기본 0.1s)  →  stale, 폐기
```

**경고도 에러도 없이 버린다.** 증상은 "토픽은 흐르는데 로봇이 안 움직인다" 라
원인을 짚기 매우 어렵다.

요구사항은 "발행 시각을 실어라"가 아니라 **"소비자의 `now` 기준 0.1초 이내여야
한다"** 이다. 달성 방법은 상황에 따라 다르다.

| 상황 | 원본 stamp 로 충분한가 | 권장 |
|---|:-:|---|
| 같은 머신·같은 시계, 지연 무시할 수준 | ✅ | 원본 유지 (시간축 정확) |
| 별도 머신 / clock skew 가능 | ❌ | **재발행 시점에 restamp** |
| 드라이버·relay 지연이 임계에 근접 | 위험 | restamp |
| 녹화 데이터(rosbag) 재생 | ❌ | **반드시 restamp** — 원본 stamp 가 수십만 초 과거 |

#### restamp 의 대가

restamp 를 켜면 stale 문제는 사라지지만, 하류에서 **stamp 차분으로 속도를
계산하는 노드**의 `dt` 가 원본 표본 간격이 아니라 **도착 간격**이 된다. 전송
지터가 속도 노이즈로 섞인다.

로컬 UDP 처럼 지터가 sub-ms 인 경로에서는 무시할 만하지만, 속도 출력이 거칠면
`restamp` 를 끄고 **시계 동기를 보장하는 쪽**으로 바꾸는 것이 낫다.

#### rdfp 내부의 규약

[`ee_twist_publisher`](../../src/robot_control/robot_control/moveit/ee_twist_publisher.py) 와
[`target_joint_cmds_publisher`](../../src/robot_control/robot_control/moveit/target_joint_cmds_publisher.py)
는 **출력 stamp 에 발행 시각을 싣는다.** 다만 `ee_twist_publisher` 의 차분 간격
`dt` 는 **입력 stamp** 로 계산하므로, 입력이 원본 stamp 를 유지하는 한 속도 값은
원본 시간축을 그대로 반영한다. 즉 어댑터가 restamp 하지 않을수록 속도가 정확하고,
할수록 stale 에 강하다.

#### 확인

```bash
ros2 topic echo /servo_node/delta_twist_cmds --field header.stamp --once
date +%s        # 두 값의 차이가 0.1초 이내여야 한다
```

경로 B 를 쓰는 경우 [`teleop_retarget`](../../src/rdfp/rdfp/teleop/teleop_retarget_node.py)
의 `pose_staleness`(기본 1.0s)와 `watchdog_timeout`(기본 0.5s)도 같은 판정을
한다. stamp 가 과거면 클러치 engage 자체가 거부된다.

### 3-3. 발행 주기가 아니라 **실효 주기** 가 기준이다

경로 B 에서 `ee_twist_publisher` 는 `dt <= 0` 인 표본(중복·역행 stamp)을
버린다. pose 를 낮은 주기로 갱신하면서 높은 주기로 반복 발행하면, 발행 주기는
높아 보여도 실효 주기는 훨씬 낮다.

실측 예(`ee_pose_bag`): 발행 200 Hz → 고유 stamp 기준 **19.3 Hz**
(90.4% 가 직전과 완전히 동일한 메시지).

| 실효 주기 | 상태 |
|---|---|
| < 10 Hz | ❌ servo stale → 끊김. 감속분이 개루프 오차로 남음 |
| 10 ~ 29 Hz | ⚠️ 동작하나 servo 출력 주기(29.4 Hz)를 못 채워 계단식 |
| ≈ 30 Hz 이상 | ✅ 정합 |

확인:

```bash
ros2 topic hz /leader/ee_pose                  # 발행 주기
ros2 topic hz /servo_node/delta_twist_cmds     # 실효 주기
```

### 3-4. servo 는 `start_servo` 전까지 입력을 무시한다

`moveit_servo` 는 `start_servo`(`std_srvs/Trigger`)가 호출되기 전까지 모든
입력을 **조용히 버린다.**

```bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger
```

`replay_panda_mock.launch.py` 의 `ee_twist` 경로는
[`servo_auto_start_node`](../../src/robot_control/robot_control/moveit/servo_auto_start_node.py)
를 함께 띄워 이를 처리한다. 다른 경로는 직접 호출해야 한다.

---

## 4. 현재 어댑터 목록

| 어댑터 | 경로 | 위치 | 비고 |
|---|:-:|---|---|
| `teleop_keyboard` | A | rdfp 내장 (`ros2 run rdfp teleop_keyboard`) | 세션 서비스 의존 — `session_control_node` 필요 |
| 게임패드 | A | — | [joystick_setup.md](joystick_setup.md) 참고 |
| **OMY-L100** | B | **외부 저장소 `omy_leader_bridge`** | Jazzy/rmw_zenoh → UDP relay → 호스트 재발행 |
| Isaac Sim | 미정 | — | 예정 |

### OMY-L100 (`omy_leader_bridge`)

OMY-L100 은 ROS 2 Jazzy + `rmw_zenoh` 를 써야 하는데 rdfp 는 `rmw_fastrtps_cpp`
를 쓴다. zenoh RMW 는 버전 간 wire 비호환이라 DDS 로도 zenoh 로도 직접
연결되지 않는다.

그래서 컨테이너 안에서 구독한 pose 를 **UDP 로 호스트에 넘겨 재발행**한다.

```
[컨테이너: Jazzy / rmw_zenoh / domain 30]      [호스트: Humble / fastdds / domain 31]
  ee_pose_node.py                                 host_republisher.py
    tf2: leader_link0 -> leader_link7               UDP 수신 → 역직렬화
    → PoseStamped 발행                              → /leader/ee_pose 재발행
         │                                                 ▲
         └─ relay_node.py ─[1B 인덱스+CDR] UDP ────┘
```

**계약 준수 현황** (2026-08-01 기준, 반영 완료):

| 항목 | 값 | 근거 |
|---|---|---|
| 호스트 `ROS_DOMAIN_ID` | `31` | §3-1 |
| 호스트 RMW | `rmw_fastrtps_cpp` | §3-1 |
| 재발행 토픽 | `/leader/ee_pose` | §2 경로 B |
| 재발행 `header.stamp` | `bridge_topics.json` 의 `"restamp": true` | §3-2 |
| 실효 주기 | 50 Hz (`RATE` 환경변수) | §3-3 |

`restamp` 는 브릿지의 토픽별 옵션이며 **기본값은 `false`**(원본 보존)다. 본
구성에서 `true` 로 켠 이유는 원본 stamp 가 관측 시각(TF stamp)이라 드라이버 지연
+ relay 지연이 servo 의 0.1초 임계에 닿을 여지가 있고, 실패가 조용하기 때문이다.

속도 출력이 거칠면 §3-2 의 "restamp 의 대가"를 참고해 `false` + 시계 동기
확인으로 바꿀 수 있다.

rdfp 쪽 기동:

```bash
# follower 스택
ros2 launch rdfp rdfp_panda_jgpc_mock.launch.py

# 미러링 체인 — leader pose 발행자를 띄우지 않는다
ros2 launch rdfp teleop_mirror.launch.py

# 클러치 engage
ros2 service call /teleop_retarget/clutch std_srvs/srv/SetBool "{data: true}"
```

---

## 5. 새 어댑터를 추가할 때

1. 경로 A / B 중 하나를 고른다. 절대 pose 를 낼 수 있으면 B 가 좌표계 정렬·
   작업공간 스케일링을 대신 해주므로 유리하다.
2. §3 의 네 항목을 모두 만족시킨다.
3. 어댑터 구현은 **rdfp 밖에 둔다.** rdfp 는 계약만 정의하고 구현을 알지 않는다.
4. 본 문서 §4 표에 한 줄 추가하고, 저장소 위치와 계약 준수 여부를 적는다.

---

## 관련 문서

- [leader_follower_mirroring_design.md](leader_follower_mirroring_design.md) — 경로 B 의 설계 배경과 retarget 수식
- [joystick_setup.md](joystick_setup.md) — 게임패드 연결
- [../moveit/servo_client_programmers_guide.md](../moveit/servo_client_programmers_guide.md) — servo 서비스/상태
- [../replay/replay_mock_stack_guide.md](../replay/replay_mock_stack_guide.md) — 같은 제약이 재생 경로에도 적용된다 (§3-1, §3-4-2, §3-4-3)
