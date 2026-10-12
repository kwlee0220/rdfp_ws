# Isaac Sim 백엔드 — 골격 설계와 단계 계획

> **설계 근거** [README.md](../README.md) §6.3 ·
> **선행 사례** [펑션베이_백엔드_설계.md](../functionbay/펑션베이_백엔드_설계.md) ·
> [펑션베이_남은_작업.md](../functionbay/펑션베이_남은_작업.md)
>
> Isaac Sim 을 `robot_control` 의 시뮬레이터 백엔드로 붙이는 작업의 **골격과 진행 순서**를 정한다. 어느 배포(§1)에서도 같은 그래프·같은 검사로 돌아간다. 한 번에 전부 붙이지 않고 **팔 → 그리퍼 → scene → 카메라** 순으로 한 축씩 열며, 각 단계는 *launch 인자 하나 + 검증 스크립트 하나 + 수용 기준*으로 닫는다.

---

## 0. 결정 사항 요약

| # | 결정 | 근거 |
|:-:|---|---|
| D1″ | **연동 방식은 하나다 — bridge 를 삭제했다 (2026-09-05)** | D1′ 로 둘이 됐던 것을 다시 하나로 줄였다. bridge 가 **고유하게 덮는 것이 없었다** — `MoveGroupJgpcClient`·`TrajectoryStreamer` 는 `panda_mock_jgpc` 이, `arm_command_format='joint_state'` 와 `servo_command_bridge` 는 펑션베이가 덮는다. 반면 방식이 둘이면 **launch 와 어긋났을 때 에러 없이 관절 상태가 오지 않는** 함정이 생기고(§7), 검사 다섯을 모드 인지로 유지해야 하며, 모든 변경을 두 번 검증해야 했다. 게다가 bridge 의 팔 실행은 개루프라 **"성공을 반환했는데 도달하지 않음"이 원리적으로 가능**해 수집 데이터의 신뢰도에 직접 걸린다. 지운 것: launch 2개 · 트윈 설정 · `isaac_gripper_bridge` (합계 1,235줄) + `is_backend.py` 의 모드 분기 |
| D1′ | **(구) 연동 방식이 둘이 됐다 — 기본은 `plugin` (2026-09-05)** | D1 의 전제가 틀렸다. 연동 A안·B안 말고 **세 번째 길**이 있었다 — `topic_based_ros2_control/TopicBasedSystem` 은 `controller_manager` 를 **ROS 쪽**에서 돌리고 하드웨어만 토픽으로 Isaac 과 말한다. Isaac 이 CM 을 호스팅할 필요가 없으니 **Windows 여부와 무관**하다. 게다가 배선이 이미 있었다 — 상위 MoveIt 원본 xacro 의 `isaac` 분기가 그것이다. 실측 결과는 §2 Phase 11 | 
| D1 | **(구) 연동 방식은 토픽 브리지** — [설계 문서](../README.md) §6.3 의 **연동 B안**(Isaac 은 `/joint_states` 발행 + `JointState` 명령 구독만 하고, trajectory adapter 와 readiness gate 는 ROS 쪽에 둔다) | **연동 A안**(`ros2_control` 하드웨어 플러그인 — Gazebo 처럼 `controller_manager` 를 Isaac 이 호스팅)은 Windows 에서 불가능하다. 두 호스트에서 같은 코드를 쓰려면 연동 B안뿐이다. 구현체는 `MoveGroupJgpcClient`+`TrajectoryStreamer`(adapter) · `fb_readiness_gate`(gate) · `isaac_servo_bridge` · `isaac_gripper_bridge` · `setup_graph.py` 다 |
| D2 | **경계 메시지는 표준 타입만** (`rdfp_msgs` 금지) | **적용 조건은 "시뮬레이터가 별도 프로세스이고 그쪽이 우리 빌드가 아닐 때"** 다. **이유는 결합이다**: `rdfp_msgs` 를 경계에 쓰면 시뮬레이터가 우리 IDL 을 자기 환경에 빌드해 넣어야 하는데 그건 통제할 수 없다. 변환은 **우리 쪽에서** 한다 — `isaac_scene_state_node` 가 TF → `rdfp_msgs/SceneObjects` 로 바꾼다. 표준 패키지를 우리가 설치해 쓰는 것은 무방하다 (`simulation_interfaces` 로 `/scene/reset` 을 구현했다). |
| D3 | **팔 명령은 `sensor_msgs/JointState`** | Isaac OmniGraph 가 기본 제공한다(`ROS2SubscribeJointState`). 그 토픽을 채우는 것은 `TopicBasedSystem` 이고, MoveGroup 은 **JTC** 판이다 |
| D4 | **`use_sim_time:=true` 전역** | Isaac 은 `/clock` 을 발행한다. 펑션베이(§6.3 에서 sim time 포기)와 갈리는 가장 큰 지점이며, **시계 오프셋 문제가 구조적으로 사라진다** |
| D5 | **배포별 분기는 DDS 설정과 경로뿐** | 노드 그래프·토픽 이름·QoS 는 어느 배포에서도 동일해야 한다. §1 참조 |
| D6 | **노드를 켜고 끄는 것은 launch 인자로** | `enable_gripper`·`enable_scene`·`enable_servo` 는 기본 `true`, `enable_rviz` 와 `enable_image_viewer` 는 `false` 다(Isaac 이 이미 뷰포트를 그리고, 뷰어는 2026-09-20 부터 — 수집 계층의 `rdfp_image_viewer` 와 창이 둘 뜨던 것을 `run_collect_isaac.sh --viewer` 로 갈랐다). 끄면 노드가 아예 뜨지 않는다 — 화면 없는 곳에서 뷰어를 끄는 것이 대표적인 용도다. 카메라는 노드가 아니라 Isaac 그래프가 발행하므로 인자가 없다 |

---

## 1. 배포

**기본은 Ubuntu 한 머신이다.** Isaac 을 다른 호스트에 두거나 Windows 에서 돌리는 것도 되지만, 그때 추가로 챙길 것이 있다 — 아래 두 절이 그것이다.

**노드 그래프·토픽 계약·검사 스크립트는 어느 배포에서도 같다.** 차이는 전부 launch 바깥(환경변수·방화벽·`.wslconfig`)에서 흡수한다.

### 공통

```bash
ROS_DOMAIN_ID=31            # 양쪽이 같아야 한다. 다르면 에러 없이 서로 안 보인다
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_LOCALHOST_ONLY=0
```

Isaac 쪽 `ROS2Context` 노드는 `useDomainIDEnvVar=False` + `domain_id=31` 로 값을 못박는다 (`setup_graph.py`). GUI 로 띄운 프로세스에 환경변수가 없어 도메인 0 으로 떨어지는 것을 막기 위해서다.

**검증 순서도 같다** — `is_topics.py` 로 배관을 먼저 보고, 그다음 스택, 그다음 단계 검사.

---

### 기본 — Ubuntu 한 머신 (**검증 완료** 2026-09-05)

**가장 단순하고, 실측이 전부 여기서 나왔다.** 검사 7종·트윈 REST·수집 전 과정 (세션 → `/scene/reset` → 에피소드 → rosbag → mp4 → DB)이 이 구성에서 통과했다.

```bash
./scripts/run_isaac_sim.sh --gui      # 런처가 환경을 맞춘다
```

- **DDS 프로파일이 필요 없다.** 진짜로 같은 호스트라 SHM 이 제대로 동작한다. WSL 용 프로파일을 들고 오면 **SHM 을 막아 느려질 뿐 이득이 없다** — `unset FASTRTPS_DEFAULT_PROFILES_FILE`.
- **배포판 자동 선택** — Linux 에서는 `system_default` 가 Ubuntu 버전을 보고 humble 을
  고른다. Windows 에서 jazzy 로 떨어지던 문제가 없다.
- 워크스페이스를 source 한 셸에서 Isaac 을 띄우면 시스템 ROS 2 를 쓴다(내부 라이브러리가
  아니라). custom message 도 가능해지지만 **D2 에 따라 경계는 표준 타입만** 유지한다 —
  쓸 수 있다고 쓰면 배포마다 코드가 갈라진다.

---

### Isaac 을 다른 머신에 둘 때 — 챙길 것 셋

진짜로 다른 호스트이므로 Fast DDS 가 알아서 UDP 를 고른다. **프로파일은 기본적으로 필요 없다.** 대신 셋이 새로 생긴다.

1. **저장소 사본** — `setup_scene.py` 가 `config/isaac_scene.json` 을 읽는데 그 파일은 스택 머신에 있다. Isaac 머신에도 사본이 있어야 하고(NFS·rsync·git clone 중 택일), 스크립트에 위치를 알려 준다.

   ```bash
   # Isaac 머신에서
   export RDFP_WORKSPACE=/home/<user>/rdfp_ws
   export RDFP_LOG_DIR=/tmp
   ```

   **사본이 갈라지면 물체 크기가 두 곳에서 달라진다** — JSON 을 단일 진실원본으로 둔 의미가 사라지므로 한 방향 동기화를 권한다.
2. **방화벽** — `ufw` 를 쓴다면 도메인 31 의 DDS 포트를 연다.

   ```bash
   sudo ufw allow proto udp from <상대 IP> to any port 15150:15200
   ```

   Windows 머신이면 `NVIDIA Omniverse Kit` 인바운드를 사설 네트워크에 허용한다.
3. **시계** — 두 머신에 `chrony`/NTP. `use_sim_time:=true` 라 `/clock` 을 함께 쓰므로 대부분 면역이지만, 로그 시각과 rosbag 파일 타임스탬프가 어긋나면 추적이 어렵다. 펑션베이에서 2.22 초 오프셋으로 데이터셋이 오염될 뻔한 사례가 있다.

**멀티캐스트가 스위치에서 막히면** 상대 IP 를 initialPeers 에 넣는다(부록의 프로파일에서 주소만 `127.0.0.1` → 상대 IP 로 바꾼다). 증상은 한쪽만 상대를 보고, 발행은 되는데 수신이 0 인 것이다.

**카메라는 단편화를 먼저 본다 — 대역폭이 아니다.** 1280×720 rgb8 @ 10 Hz 는 221 Mbps 로
1 GbE 의 22% 라 링크는 감당한다. 문제는 **프레임 하나가 2.76 MB** 라는 것이다.

```
2.76 MB 프레임 → UDP 조각 약 1,878개 → 10 Hz 면 초당 약 18,800 조각
```

**조각 하나만 잃어도 프레임 전체가 날아간다.** 이 저장소는 9 KB 짜리 SRDF 응답이 같은
이유로 15초 타임아웃 나던 것을 이미 겪었다(§7 「MTU 를 넘는 메시지가 조용히 사라진다」).
`CompressedImage`(JPEG)로 바꾸면 프레임이 100~200 KB 로 줄어 조각이 두 자릿수가 된다 —
대역폭 절감보다 **이쪽이 실질 이득**이다. 해상도·주파수를 낮추는 것도 같은 효과다.

> **한 머신에서는 해당 없다.** SHM 을 쓰므로 단편화가 없다. 대신 **배속**(720p RTX 렌더는
> GPU 를 더 먹는다)과 **저장 용량**(720p @ 10 Hz = **1.66 GB/분**)을 본다. 용량은 압축이
> 아니라 `delete_splits_after_import`(mp4 를 만든 뒤 rosbag 삭제)로 푸는 편이다.

---

### 부록 — Windows 에서 Isaac 을 돌릴 때

Windows Isaac + WSL2 스택(구성 A)의 DDS 프로파일 · 환경변수는 [isaac_windows_setup.md](isaac_windows_setup.md)
부록에 있다 (2026-09-21 에 그쪽으로 옮겼다 — 같은 내용이 두 문서에 있었다).

### 불변식 — 배포가 달라도 이것은 같다

**launch 파일과 노드 그래프는 어느 배포인지 모른다.** 차이는 전부 launch 바깥 (환경변수·`.wslconfig`·방화벽)에서 흡수한다. 그래서 `host_profile` 같은 launch 인자를 두지 않는다.

**검증 방법**: 비교할 두 배포 — 이를테면 **기본(Ubuntu 한 머신) ↔ 부록(Windows Isaac)** — 에서 각각 `ros2 node list` 와 각 토픽의 엔드포인트를 떠서 **diff 가 비어야 한다.** `rdfp_collect` 조합 동등성을 확인할 때 쓴 것과 같은 기법이다.

**이것은 처방이지 기록이 아니다** — 이 기법을 실제로 쓴 것은 `rdfp_collect` 쪽이고, 배포 사이의 diff 는 아직 한 적이 없다. Windows 에서 plugin 백엔드를 돌릴 때 첫 대상이 된다.

> **유일한 예외는 카메라다.** raw `Image` 는 프레임이 2.76 MB(1280×720 rgb8)라 머신 경계를 넘으면 UDP 단편화에서 깨지기 쉬워 `CompressedImage` 가 필요하다(§1 「Isaac 을 다른 머신에 둘 때」). 한 머신에서는 그대로 둔다. 예외를 이 한 곳으로 묶고, 조건 분기가 다른 데로 번지지 않게 한다.

## 2. 단계 계획 — Phase 0~11 (완료 · 보관)

**열한 단계를 전부 마쳤다 (2026-08-28 ~ 2026-09-05).** 단계별 수용 기준 · 검사 스크립트 · 실측값 · 철회 경위는
[../../archive/isaac/isaac_phase_log.md](../../archive/isaac/isaac_phase_log.md) 에 그대로 있다. 절 번호 §2 는 다른
문서의 참조("§2 Phase 11")를 위해 남긴다. 지금 남아 있는 수치는 다음 한 표다.

| Phase | 무엇 | 수용 기준 · 실측 | 검사 |
|:-:|---|---|---|
| 0 | 골격 — 게이트 · 토픽 계약 | 기동 게이트 통과, `/clock` 진행 | `is_check_phase0.py` |
| 1 | 팔 관절 명령 | 정착 오차 **0.3 mrad** · τ 50 ms 급 (`tune_drive`) | `is_check_phase1.py` |
| 2 | 그리퍼 | `gripper_cmds` 심볼 → `at_goal` (effort 로 판정, `stall_effort` 1.0) | `is_check_phase2.py` |
| 3 | scene 객체 | `/scene/objects` 가 TF 경유로 xyzw · m 단위 | `is_check_phase3.py` |
| 4 | 카메라 | 640×480 · 설정 fps 로 `rgb8` 발행 (⚠️ 레코더 `pixel_format` 을 맞춘다) | `is_check_phase4.py` |
| 5 | 철회 (2026-09-01) | — | — |
| 6 | 물리 파지 | 블록을 들어 **+0.0999 m** 유지 (`tune_grasp` 마찰) | `is_check_phase6.py` |
| 7 | 수집 계층 연동 | 세션 → scene reset → 에피소드 → rosbag → mp4 → DB 전 과정 통과 | — |
| 8 | 테스트 · 토폴로지 계약 | 노드 · 토픽 그래프 고정 | `robot_control/isaac/tests/` |
| 9 | robot_twin 연동 | REST 로 팔 · 그리퍼 · 세션 · `reset_scene` | — |
| 10 | servo(twist) 경로 | `teleop_keyboard` 로 조작 — 래칫 수정은 [../../teleop/servo_vs_planned_motion.md](../../teleop/servo_vs_planned_motion.md) | `is_servo_push.py` |
| 11 | ros2_control (`TopicBasedSystem`) | 정착 0.3 mrad · 데카르트 0.2 mm · **배속 −5 % · load 3배 · dead time +100 ms** 를 대가로 JTC 를 얻었다 (D1″) | 전 검사 재통과 |

## 3. 골격 구성

### 파일

```
── ROS 쪽 스택 ────────────────────────────────────────────────────────────
src/robot_control/launch/panda_isaac.launch.py            백엔드 launch
src/rdfp/launch/rdfp_collect.launch.py backend:=isaac     + 수집 계층
src/robot_control/robot_control/isaac/
    scene_state_node.py                                   Isaac TF → /scene/objects, /scene/reset
    servo_command_bridge_node.py                          Float64MultiArray → JointState
                                                          (**Isaac 은 안 쓴다** — 펑션베이 전용, §2 #12)
    tests/                                                scene_state 24 · servo_bridge 19
src/robot_control/config/
    isaac_scene.json                                      **물체·카메라 단일 진실원본**
    fastdds_wsl_bridge.xml                                DDS 전송 프로파일 (Windows+WSL2 전용)
    panda_real_joint_limits.yaml                          실제 Franka 관절 한계 (Q6)
src/robot_twin/config/robot_twin_panda_isaac.yaml         트윈 설정
config/recording_topics_isaac.list                        녹화 토픽 (카메라만 mock 과 다르다)

── 검증 스크립트 (ROS 쪽) ──────────────────────────────────────────────────
scripts/isaac/is_backend.py                               검사들이 공유하는 백엔드 계약
scripts/isaac/is_check_phase0.py ~ is_check_phase6.py     단계별 수용 기준 (phase5 는 철회)
scripts/isaac/is_recover.py                               충돌 자세 탈출 (계획 없이 ready 복귀)
scripts/isaac/is_topics.py                                배관 진단 (자체시험 + 수신량)
scripts/isaac/is_probe_srdf.py                            파라미터 서비스 2×2 진단

── Isaac 쪽 (sim_side) ─────────────────────────────────────────────────────
scripts/isaac/sim_side/headless_bringup.py                아래 일곱을 순서대로 도는 진입점
    load_robot.py                                         Franka 를 스테이지에 올린다 (멱등)
    setup_scene.py                                        물체·카메라·조명 생성 / 리셋
    setup_graph.py                                        OmniGraph 구성
    place_robot.py                                        panda_link0 을 월드 원점으로 (멱등)
    set_home_pose.py                                      Play 시작 자세를 ready 로 (멱등)
    tune_drive.py                                         팔 drive 게인 (멱등)
    tune_grasp.py                                         손가락 drive·손끝 마찰 (멱등)
scripts/isaac/sim_side/check_colliders.py                 충돌·배치 진단 (읽기 전용)
scripts/isaac/sim_side/check_bridge.py                    Isaac 내부 토픽 가시성 진단
scripts/isaac/sim_side/verify_saved_scene.py              저장된 scene 검증
scripts/isaac/sim_side/run_isaac_humble.bat               Windows 런처 (humble 고정)
scripts/run_isaac_sim.sh                                  Linux 런처 (--gui / --headless)
```

`sim_side/` 아래만 **Isaac Sim 안에서** 돈다 — ROS 노드가 아니고 `colcon` 이 빌드하지도
않는다. **기본 경로는 `run_isaac_sim.sh --gui`** 이며 `headless_bringup.py` 가 일곱을
자동으로 돈다. Script Editor 는 인자 없이(전체 편집기로) 띄웠을 때만 쓴다.
`.bat` 은 Windows 전용이며 `C:\isaacsim\` 에 **복사본**이 있다(원본을 고치면 복사해야 한다).

### 토픽 계약

`setup_graph.py` 의 상수가 이 표를 정본으로 지목한다 — **여기를 고치면 코드도 함께 고친다.**

| 방향 | 채널 | 타입 | 정하는 곳 |
|---|---|---|---|
| Isaac → 스택 | `/clock` | `rosgraph_msgs/Clock` | `setup_graph.CLOCK_TOPIC` |
| Isaac → 스택 | **`/isaac_joint_states`** | `sensor_msgs/JointState` | `setup_graph.JOINT_STATE_TOPIC` ↔ xacro `joint_states_topic` |
| 스택 → Isaac | `/isaac/arm_command` | `sensor_msgs/JointState` | `setup_graph.ARM_COMMAND_TOPIC` ↔ xacro `isaac_arm_command_topic` |
| 스택 → Isaac | `/isaac/gripper_command` | `sensor_msgs/JointState` | `setup_graph.GRIPPER_COMMAND_TOPIC` ↔ xacro `isaac_gripper_command_topic` |
| Isaac → 스택 | **`/tf`** (물체 프레임) | `tf2_msgs/TFMessage` | `setup_graph` 의 `PublishSceneTF` |
| Isaac → 스택 | `/isaac/camera/image_raw` · `camera_info` | `sensor_msgs/Image` · `CameraInfo` | `isaac_scene.json` 의 `camera` |
| 스택 → Isaac | `/set_entity_state` (**서비스**) | `simulation_interfaces/srv` | `isaac_scene_state_node` 가 `/scene/reset` 을 받아 호출 |

**scene 은 전용 토픽이 아니라 `/tf` 로 온다.** `PoseArray` 대신 TF 를 고른 이유는 좌표
변환·쿼터니언 규약을 tf2 가 대신하기 때문이다(Q2). `isaac_scene_state_node` 가 그것을
`rdfp_msgs/SceneObjects` 로 바꿔 `/scene/objects` 에 싣는다 — 이름·종류·크기는 TF 에
없으므로 `isaac_scene.json` 을 함께 읽는다.

`/isaac/` 접두사는 **시뮬레이터 경계임을 이름으로 드러내기 위한 것**이다. 스택 내부
토픽(`/joint_states`, `/scene/objects`)과 섞이면 어느 쪽이 원본인지 로그만 보고는
알 수 없다. 펑션베이의 `/input`·`/output` 과 같은 의도다.

> ⚠️ **관절 상태만 `/isaac_joint_states` 로 밑줄이다** — 규약대로면 `/isaac/joint_states`
> 여야 한다. 상위 MoveIt 의 Panda xacro 가 `isaac` 분기 기본값으로 그 이름을 쓰고 있어
> 따라간 것이다. 바꾸려면 xacro 인자와 `setup_graph` 를 같이 고쳐야 하며, 아직 안 했다.

### 기동 순서

```
static_transform_publisher · robot_state_publisher · readiness_gate
    └ readiness_gate 가 첫 /isaac_joint_states 를 받고 **정상 종료**
         └ ros2_control_node
              └ joint_state_broadcaster
                   └ panda_arm_controller (JTC)
                        └ panda_hand_controller (GripperActionController)
                             └ move_group · servo · rviz · ee_pose · gripper · scene · viewer
```

**게이트가 `ros2_control_node` 보다 앞이다.** spawner 는 `controller_manager` 만 뜨면
성공하므로 "Isaac 이 Play 중"을 보증하지 못한다. `use_sim_time` 이 켜진 채 `/clock` 이
없으면 CM 의 update 루프가 시각 0 에 멈추고, **컨트롤러는 `active` 인데 아무것도 안
움직이는** 상태가 된다(§7).

게이트가 실패 종료하면 `_chain_or_shutdown` 이 launch 전체를 내린다 — 시뮬레이터가 꺼져
있는데 `move_group` 만 올라와 원인을 감추는 상황을 막는다. 그 뒤 spawner 사슬은
`panda_mock` 과 같은 헬퍼(`create_controller_startup_handlers`)를 그대로 쓴다.

**`joint_state_fusion` 이 없다** — Q1 이 Phase 0 실측으로 닫혔다(§8). `/joint_states` 는
`joint_state_broadcaster` 가 소유한다.

---

## 4. 재사용하는 것

펑션베이에서 만든 두 노드가 **파라미터화돼 있어 코드 수정 없이 그대로 쓰인다.**

| 실행 파일 | Isaac 에서의 역할 | 넘기는 파라미터 |
|---|---|---|
| `fb_readiness_gate` | 기동 게이트 | `topic` · `timeout_sec` |
| ~~`fb_joint_state_fusion`~~ | **쓰지 않는다** — Isaac 이 이름과 9관절을 모두 채운다 (§8 Q1) | — |

특히 `extra_joint_names` / `extra_joint_positions` 는 **"그리퍼 미연동 구간 동안
finger 를 고정값으로 채워 TF 만 성립시키는"** 용도로 이미 만들어져 있다 — Phase 1 과
Phase 2 사이를 잇는 비계가 이미 있는 셈이다.

> 실행 파일 이름의 `fb_` 접두사는 펑션베이 전용이라는 인상을 주지만 구현은 백엔드
> 중립이다. 세 번째 소비자가 생기면 `sim_readiness_gate` 등으로 공용화한다. **지금
> 이름을 바꾸지 않는 이유**는 펑션베이 작업이 중단 상태라 회귀를 확인할 사람이
> 없기 때문이다.

~~`fb_hold.py` / `fb_probe.py` / `fb_recover.py` 도 토픽 상수만 바꿔 이식한다~~
→ **`is_recover.py` 만 만들었다.** hold/probe 의 역할은 `is_backend.ArmCommander` 가
대신한다 — 계획 없이 관절 목표를 밀어 넣는 경로가 하나면 충분했다.

---

## 5. 하드웨어

| | **검증 머신** (기본 배포) | WSL2 머신 (§1 부록) |
|---|---|---|
| OS | Ubuntu 22.04.5 | Windows + WSL2 |
| GPU | **RTX 4000 SFF Ada · VRAM 20 GB** | RTX 4060 · VRAM 8 GB |
| RAM | 62 GB | 32 GB (WSL2 몫 15.4 GB 기본) |
| CPU | 20 코어 | 28 코어 |

**Phase 11 실측(§2)은 전부 왼쪽 머신에서 나왔다.** 아래 절은 오른쪽 머신 전용이며,
현행 사양이 아니다.

### WSL2 머신의 제약 (Isaac Sim 5.1 기준)

**WSL2 를 그대로 두면 Isaac 이 최소 사양의 절반으로 돈다.** WSL2 는 한 번 잡은
메모리를 자동 반환하지 않으므로(`autoMemoryReclaim` 미설정) 캡을 거는 것이 맞다.

```ini
# C:\Users\<user>\.wslconfig
[wsl2]
memory=10GB              # MoveIt+RViz+recorder+postgres 에 충분
processors=12            # 28개 전부 주면 Isaac 물리 스텝과 코어를 다툰다
swap=8GB
networkingMode=mirrored  # 유지 — Isaac↔WSL2 DDS 에 유리
dhcp=true
dnsTunneling=true

[experimental]
hostAddressLoopback=true
autoMemoryReclaim=gradual   # ★ 추가
```

**더 빡빡한 쪽은 RAM 이 아니라 VRAM 이다.** 8 GB 는 최소 미달이라 Panda 한 대짜리
단순 scene 은 돌아가도 **RTX 카메라 센서를 붙이는 순간이 위험 구간**이다. WSL2 에서
RViz 를 띄우면 WSLg 가 같은 8 GB 를 나눠 쓰므로, 이 PC 에서는 `enable_rviz:=false`
를 기본 운용으로 삼는다.

→ **실측으로 뒤집혔다.** 640×480 @ 5 Hz 카메라 하나는 VRAM 180 MiB · GPU 47% 로
이 PC 에서 문제없이 돌았다(Phase 4 완료). 최소 사양 미달은 사실이지만, 그 기준은
센서가 많은 복잡한 scene 을 전제한다. **고해상도·고주파가 필요해지면 그때 Ubuntu 머신으로
옮긴다.**

---

## 6. 운용의 근거


> **새 Windows 머신에 처음 구축하는 경우**는 [isaac_windows_setup.md](isaac_windows_setup.md)
> 를 따른다. WSL2 설치부터 검증까지의 절차만 추린 문서이며, 아래는 그중 Isaac 쪽
> 운용에 해당한다.

**클릭으로 만들지 않는다.** `scripts/isaac/sim_side/setup_graph.py` 가
OmniGraph 를 구성한다 — UI 절차는 버전마다 바뀌고 재현이 안 되지만 스크립트는
저장소에 남는다.

> **절차 자체는 [isaac_bringup_runbook.md](isaac_bringup_runbook.md) 에 있다.**
> `./scripts/run_isaac_sim.sh --gui` 가 `headless_bringup.py` 를 통해 sim_side 스크립트
> 일곱을 순서대로 돌린다 — 평소에는 Script Editor 를 쓰지 않는다. **이 절은 그 절차가
> 왜 그 모양인지만 남긴다.**

### 일곱의 순서에는 이유가 있다

`load_robot` → `setup_scene` → `setup_graph` → `place_robot` → `set_home_pose` →
`tune_drive` → `tune_grasp` ([`SIM_SIDE_SCRIPTS`](../../../scripts/isaac/sim_side/headless_bringup.py)).
**전부 멱등이라 여러 번 돌려도 같은 상태가 된다.**

| 순서 | 왜 그 자리인가 |
|---|---|
| `load_robot` 이 맨 앞 | 로봇이 없으면 `setup_graph` 가 articulation root 를 못 찾고 멈춘다 |
| `setup_scene` 이 `setup_graph` 앞 | TF 발행 대상 prim 이 먼저 있어야 `targetPrims` 가 잡힌다 |
| `place_robot` | ROS 쪽 static TF 가 `world → panda_link0` 를 **항등변환으로 못박는다.** 안 맞추면 에러 없이 모든 좌표가 그만큼 어긋난다(§7 로봇 배치) |
| `set_home_pose` | 에셋 기본 자세는 손이 탁자 안이라, 그대로 Play 하면 MoveIt 이 시작 자세 충돌로 계획을 거부한다(§7 시작 자세) |
| `tune_drive` · `tune_grasp` | 에셋 기본 게인은 τ 195.6 ms 로 느리고, 손끝 마찰이 없으면 블록이 미끄러진다. 값은 **절대값으로** 설정한다 |

**`tune_grasp` 는 `setup_scene` 을 다시 돌린 뒤에도 다시 돌릴 필요가 없다** — 로봇
에셋을 만지므로 scene 재생성과 무관하다. 그래서 손끝 재질을 scene root(`/World/Scene`)
가 아니라 `/World/GraspMaterials` 에 둔다. scene 안에 두면 `setup_scene.py` 가 root 를
통째로 지울 때 **바인딩이 끊어진 재질을 가리키게** 된다.

### Script Editor 에서는 스크립트를 **붙여넣지 않는다**

전체 편집기로 띄웠을 때만 쓰는 경로다. 런북의 snippet 은 4줄짜리 로더이고, 그것이
파일을 `exec` 한다 — **내용을 통째로 붙여넣지 않는 이유가 둘이다.** ① Windows 기본
인코딩이 cp949 라 한글 주석이 깨진다. ② 붙여넣은 사본이 저장소보다 낡는다.

Windows 런처(`run_isaac_humble.bat`)가 왜 필요한지는
[isaac_windows_setup.md](isaac_windows_setup.md) 에 있다 — 벤더 런처가 `jazzy` 를 기본으로
넣기 때문이고, `ROS_DISTRO=humble` 만 미리 넣으면 **오히려 더 나빠진다.**

### 설정을 되읽어 확인한다

`setup_graph.py` 는 `SET_VALUES`(`domain_id` · 두 `topicName`)를 설정한 뒤 **되읽어
로그에 남긴다.** `og.Controller.edit` 이 속성 이름을 틀려도 조용히 넘어가는 경우가 있어,
"설정한 줄 알았는데 기본값"인 상태를 눈으로 확인하기 위해서다. 로그는 파일로도 남는다
(`/tmp/isaac_setup_graph.log`) — Script Editor 출력창은 복사가 되지 않는다.


### 저장한 스테이지를 쓰지 않는다 — 매번 짓는다

**현행은 저장하지 않는 쪽이다.** `.usd` 로 저장하면 물체 이름·크기가 USD 와
`isaac_scene.json` **두 곳**에 살게 되는데, JSON 은 `isaac_scene_state_node` 도 읽는
정본이라 갈라지면 시뮬레이터와 ROS 가 다른 크기를 믿는다 — **에러 없이.** 매번
스크립트로 짓는 편이 느려도 어긋나지 않는다(런북 §6, `headless_bringup.py` 머리말).

> 저장을 시도한 이력이 §8 Q7 에 있고 `verify_saved_scene.py` 도 남아 있다. 그 스크립트는
> **저장된 파일이 있을 때의 진단 도구**로 자리를 옮겼다.

**설령 저장하더라도 `setup_graph.py` 는 매번 돌려야 한다.** 저장된 USD 에 그래프 prim 은
보존되지만 **ROS 2 bridge 확장의 활성화 상태는 저장되지 않는다.** scene 만 열면 그래프
prim 은 있는데 노드 타입이 해석되지 않아 발행이 하나도 없고, kit 로그에
`internal rclpy for ROS Distro` 줄조차 나오지 않는다. 확장을 켜는 것이 그 스크립트의 첫
동작이기 때문이다.

### 정적 물체도 kinematic rigid body 로 만든다

collider 만 있는 prim 은 PhysX 에서 제자리에 고정되지만 **TF 발행 노드가 물리 객체로
인식하지 못한다** — 매 틱마다 아래를 60 Hz 로 쏟아낸다.

```
[PoseTree] target getObjectType eInvalid for '/World/Scene/table'
```

값이 틀리지는 않지만 kit 로그가 부풀어 **진짜 경고가 묻힌다.** `RigidBodyAPI` 를
붙이고 `kinematicEnabled=True` 로 두면 중력·충돌에 밀리지 않으면서 물리 객체로는
온전해져 경고가 사라진다 (`setup_scene.py` 가 그렇게 만든다).

### 만들어지는 그래프

```
OnPlaybackTick ─┬─> ROS2PublishClock       → /clock
                └─> ROS2PublishJointState  → /isaac_joint_states
IsaacReadSimulationTime ─> 두 노드의 timeStamp
ROS2Context(domain_id=31) ─> 두 노드의 context
```

### 출력은 영어, 주석은 한국어

`sim_side/` 스크립트의 `print` 와 예외 메시지는 **영어**다. Windows 콘솔이 cp949 라
한글이 깨지기 때문이며, CLAUDE.md 의 "로깅·예외 메시지는 영어" 규약과도 같다.
주석과 docstring 은 규약대로 한국어로 둔다 — 위 실행 방식에서는 편집기에 뜨지 않는다.

WSL 에서 도는 `is_check_phase0.py` 는 리눅스 터미널이라 한글이 정상 출력되므로
`scripts/functionbay/fb_*.py` 와 같은 한국어 리포트를 유지한다.

### 스크립트가 못박는 두 가지

| 설정 | 이유 |
|---|---|
| `useDomainIDEnvVar=False` + `domain_id=31` | Windows 에서 Isaac 을 GUI 로 띄우면 `ROS_DOMAIN_ID` 가 프로세스 환경에 없을 수 있다. 그러면 도메인 0 으로 붙어 **아무 에러 없이** 스택과 서로 보이지 않는다 |
| `PublishJointState.topicName = /isaac_joint_states` | 기본값 `joint_states` 를 그대로 두면 `joint_state_broadcaster` 와 **같은 토픽에 publisher 가 둘** 붙고, `TopicBasedSystem` 이 자기 출력을 되읽는 고리가 생긴다. (fusion 노드가 이유이던 시절이 있으나 그 노드는 Q1 에서 제거됐다) |

**Windows 에서는 Isaac 내장 ROS 2 라이브러리를 쓴다** (별도 ROS 2 설치 불필요).
Ubuntu 머신에서는 워크스페이스를 source 한 상태로 실행해도 되지만, D2 에 따라 어느
쪽이든 표준 메시지만 오간다.

### 로봇 에셋 — 재고품인가 URDF 변환본인가

Phase 0 은 **Isaac 재고 Franka 에셋으로 시작한다.** 설계 원칙은 "URDF 를 단일
진실원본으로 두고 USD 를 거기서 생성"이지만, 재고 에셋의 관절 이름이 이미
`panda_joint1~7` 이면 변환 없이 진행할 수 있다.

**그 판정을 Phase 0 수용 기준 2번이 대신한다** — `is_check_phase0.py` 가
`/joint_states` 의 이름을 URDF 와 대조해 다르면 그 자리에서 실패시킨다. 실패하면
그때 URDF importer 로 변환하고, 통과하면 재고 에셋을 계속 쓴다. **먼저 재보고
정하는 순서다.**


## 7. 함정 사전

**증상으로 찾는다.** 여기 있는 것은 전부 실제로 겪은 것이고, 각 항목은
*증상 · 원인 · 판별법* 순으로 적혀 있다.

| 증상 | 원인 | 절 |
|---|---|---|
| 토픽이 하나도 안 보인다 (에러 없음) | Isaac 이 **jazzy** 를 로드했다 | Windows ↔ WSL2 DDS |
| 토픽이 안 보인다 (양쪽 다 정상) | SHM 전송 오판 · 단방향 멀티캐스트 | Windows ↔ WSL2 DDS |
| **일부 노드만** 안 보인다 (넷 넘게 띄운 뒤) | `initialPeersList` 포트 누락 | `initialPeersList` |
| 큰 파라미터·이미지만 안 온다 (작은 건 옴) | MTU 초과 UDP 단편 소실 | `maxMessageSize` |
| 기동 게이트가 **즉시** 또는 **영원히** 실패 | ROS 클럭으로 시간을 쟀다 | 기동 게이트 |
| `TF_OLD_DATA` 경고 후 조회 실패 | Stop 이 sim time 을 0 으로 되돌렸다 | Stop/Play |
| 팔이 목표에 못 미치는데 강성을 올려도 그대로 | **관절 한계 클램프** (처짐이 아니다) | 관절 한계 |
| 그리퍼가 명령을 받는데 안 움직인다 | 손가락이 탁자·물체에 박혔다 — **고치지 않는 비용** | 팔이 탁자를 뚫는다 |
| 계획은 되는데 시뮬레이터에서 막힌다 | MoveIt 이 scene 을 모른다 — **의도된 것** (Q8) | 팔이 탁자를 뚫는다 |
| `UnicodeDecodeError` 가 뜬다 | 진짜 원인을 cp949 가 가렸다 | 한국어 Windows |
| 저장한 scene 이 비어 보인다 | `strings` 로는 USDC 를 못 읽는다 | 저장된 USD |
| `PoseTree ... eInvalid` 가 60 Hz 로 쏟아진다 | 정적 물체에 RigidBody 가 없다 | §6 정적 물체 |
| 손가락이 **끝까지** 닫히고 물체는 1 mm 도 안 움직인다 | 로봇이 월드 원점에 없다 | 로봇 배치 |
| Play 직후 팔이 탁자 위로 뚝 떨어진다 | 에셋 기본 자세가 탁자 안이다 | 시작 자세 |
| 계획이 `-2`(INVALID_MOTION_PLAN)로 거부된다 | 시작 자세가 충돌이다 — `is_recover.py` | 시작 자세 |
| 트윈 오퍼레이션이 `RUNNING` 인 채 안 끝난다 | `externally_spun` 미전달 | 외부 spin |
| twist 를 보내는데 팔이 거의 안 움직인다 | 개루프 드리프트가 쌓였다 (scene 탓이 아니다) | servo 드리프트 |
| 명령은 29 Hz 로 나가는데 팔이 **전혀** 안 움직인다 | servo 가 **NaN** 을 내보낸다 | servo NaN |
| 파지 한 번 뒤 그리퍼가 명령을 **영영 무시**한다 | `gripper_action_node` 가 로거 예외로 죽었다 | 로거 severity |
| 그리퍼 액션 결과를 기다리다 타임아웃한다 | 파지에서는 그 액션이 완료되지 않는다 | 파지 액션 |
| 시뮬 **프로세스** 재시작 후 컨트롤러는 `active` 인데 TF 가 쏟아진다 | sim 시계 되감기 (plugin) | 시뮬레이터 재시작 |
| 타임라인 Stop 중 토픽이 멎는다 | 정상이다 — Play 하면 복구된다 | 시뮬레이터 재시작 |
| JSON 을 고쳤는데 ROS 쪽만 옛 값을 쓴다 | `src` 와 `install/share` 사본이 다르다 | 단일 출처 |
| 문 블록이 손가락 사이로 미끄러진다 | 마찰 재질 미바인딩 (기본 0.5) | 마찰은 조용히 |

**진단 도구 세 개면 대부분 갈린다.**

| 도구 | 무엇을 가르나 |
|---|---|
| `scripts/isaac/is_topics.py` | WSL 내부 DDS 가부 + 각 토픽 수신량. 반드시 끝난다 |
| `scripts/isaac/sim_side/check_bridge.py` | Isaac **내부**에서 보이는지. 보이면 브리지는 무죄 |
| `ROS_DOMAIN_ID` 를 바꿔 본다 | 부하 탓인지 참가자 수 탓인지 |
| `scripts/isaac/sim_side/check_colliders.py` | prim 의 **월드 좌표**·물리 설정. 배치 어긋남을 가른다 |

### `initialPeersList` 는 **포트를 명시해야 한다** — 노드 넷을 넘기면 사라진다

**증상** — 스택 노드를 하나 더 추가했더니 **rclpy 노드 다섯이 통째로 안 보이게 됐다.**
`ros2 node list` 에 C++ 노드(move_group·robot_state_publisher)와 Isaac 노드만 뜨고,
Python 노드는 프로세스가 멀쩡히 돌면서 로그도 남기는데 아무도 그 발행을 못 받는다.

**원인** — `initialPeersList` 에 주소만 적고 포트를 생략하면 Fast DDS 가 **참가자 ID
0~3 만** 훑는다(`maxInitialPeersRange` 기본 4). 참가자가 넷을 넘는 순간, 뒤에 뜬
참가자는 유니캐스트 탐색 범위 밖으로 밀려난다. 멀티캐스트가 한 방향만 통하는
mirrored networking 에서는 그것을 메울 수단도 없다.

**해결** — 포트를 열거한다. 도메인 31 기준 ID 0~15.

```
포트 = 7400 + 250*도메인 + 10 + 2*참가자ID
     = 15160, 15162, ... 15190   (도메인 31)
```

**도메인을 바꾸면 이 목록을 다시 만들어야 한다.**

> **오진의 기록** — 마침 같은 시점에 카메라(Phase 4)를 켠 터라 "이미지 트래픽이
> 디스커버리를 굶긴다"고 한참 헤맸다. 갈라낸 방법은 **도메인을 바꿔 보는 것**이었다.
> 트래픽 없는 도메인 77 에서 같은 Python 노드 둘이 서로를 즉시 보았고, 그 순간
> 트래픽 가설이 죽고 참가자 수 가설이 남았다.
>
> 부하를 의심할 때는 **부하만 없는 동일 조건**을 만들어 보는 것이 가장 빠르다.

### MTU 를 넘는 메시지가 조용히 사라진다 — `maxMessageSize`

**증상** — `/move_group/get_parameters` 로 `robot_description_semantic`(SRDF, 9 KB)을
요청하면 30초를 기다려도 응답이 없다. 같은 서비스로 `use_sim_time`(bool)은 즉시 온다.

**격리 재현** — Isaac 도 MoveIt 도 무관하다. 노드 두 개만으로 재현된다.

| 파라미터 크기 | 기본 프로파일 | `maxMessageSize=1400` |
|---|---|---|
| 100 B | 0.00s 수신 | — |
| 5 KB | **15s 타임아웃** | — |
| 9 KB | **15s 타임아웃** | **0.00s 수신** |
| 20 KB | **15s 타임아웃** | **0.08s 수신** |

**원인** — 기본값(65500)이면 9 KB 응답이 UDP 데이터그램 하나로 나가고 커널이 IP
단편화를 하는데, **WSL2 mirrored networking 에서 그 조각이 버려진다.** RELIABLE 이라도
같은 크기로 재전송하므로 영영 도착하지 않는다. 1400 으로 두면 RTPS 가 직접 조각내어
IP 단편화가 일어나지 않는다.

**보내는 쪽에도 적용해야 한다** — 받는 쪽만 바꾸면 그대로 실패한다(실측).

> **토픽은 멀쩡해 보였다** — 반복 발행이라 재시도가 손실을 가렸다. 단발성인 서비스
> 응답에서만 드러났다. 크기가 큰 채널을 시험할 때는 **한 번만 보내 보는** 것이
> 중요하다.
>
> 이 설정이 없었으면 **Phase 4 의 카메라 이미지도 같은 벽**에 부딪혔을 것이다.
> 문서에 "대역폭 문제"로 적어 뒀던 것이 실은 이 단편화 문제였다.

### Windows ↔ WSL2 DDS — 전송 프로파일이 **필수다** (해결됨)

**증상** — WSL 에서 Isaac 토픽이 하나도 안 보인다. `/clock` 조차 0건인데 어디에도
에러가 없다. Isaac 은 재생 중이고 그래프도 tick 하며, 방화벽도 전부 Allow 다.

**원인은 둘이 겹친 것이었다.**

1. **SHM 전송 오판** — mirrored networking 에서 WSL2 는 Windows 의 네트워크 정체를
   그대로 쓴다. 그래서 Fast DDS 가 양쪽 참가자를 **같은 호스트**로 판단해 공유메모리
   전송을 고른다. Windows 의 SHM 과 Linux 의 `/dev/shm` 은 다른 물건이라 아무것도
   도착하지 않는다.
2. **단방향 멀티캐스트** — SHM 을 끄자 `Windows → WSL2` 만 뚫렸다. WSL 은 Isaac 의
   구독자(`/isaac/arm_command`)를 보는데 Isaac 은 WSL 의 구독자를 못 봐서, **보낼 곳을
   모른 채 발행하지 않는** 상태가 됐다.

**해결** — `config/fastdds_wsl_bridge.xml` 을 **양쪽 모두**에 건다.

```xml
<useBuiltinTransports>false</useBuiltinTransports>   <!-- SHM 제거 -->
<transport_descriptor><type>UDPv4</type></...>       <!-- UDP 만 -->
<initialPeersList>127.0.0.1</initialPeersList>       <!-- 유니캐스트로 서로 찌름 -->
```

| 쪽 | 거는 법 |
|---|---|
| WSL2 | `export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/src/robot_control/config/fastdds_wsl_bridge.xml` |
| Isaac | `run_isaac_humble.bat` 이 `C:\isaacsim\fastdds_wsl_bridge.xml` 로 설정 |

**한쪽만 걸면 안 된다.** 전송 선택은 양쪽 합의다 — WSL 만 UDP 로 바꿔도 Isaac 이
SHM locator 만 광고하면 닿을 곳이 없다.

적용 후 실측: `/clock` **60.4 Hz** · `/joint_states` **59.9 Hz** · `/tf` **59.6 Hz**.

> **Ubuntu 22.04 단독 호스트에서는 이 프로파일을 쓰지 않는다.** 거기서는 SHM 이
> 실제로 같은 호스트를 뜻하고 UDP 보다 빠르다. §1 의 "호스트 차이는 DDS 설정뿐"이
> 바로 이 자리이며, 이번에 그 내용이 구체화됐다.

**진단 사다리 — 이 순서로 구간이 특정된다.**

| # | 도구 | 무엇을 가르나 |
|:-:|---|---|
| 1 | `scripts/isaac/is_topics.py` | 자체시험으로 **WSL 내부 DDS** 가부, 각 토픽 수신량. `ros2 topic hz` 와 달리 반드시 끝난다 |
| 2 | `scripts/isaac/sim_side/check_bridge.py` | **Isaac 내부**에서 토픽이 보이는지. 보이면 브리지는 무죄 |
| 3 | `Get-NetFirewallApplicationFilter` / `Get-NetFirewallHyperVVMSetting` | Windows·Hyper-V 방화벽 |
| 4 | 전송 프로파일 | 위 셋이 정상인데 안 통하면 여기다 |

> **스택을 띄우기 전에 `is_topics.py` 를 먼저 돌린다.** 이 상태에서 스택을 띄우면
> `readiness_gate` 가 60초를 기다린 뒤 실패하는데, 그 로그만 보면 스택 문제처럼
> 보인다. 실제로 그렇게 두 번 헛짚었다.
>
> 또 하나 — 진단 스크립트는 토픽 타입을 **미리 못박아** 구독해야 한다. 그래프에서
> 타입을 조회해 구독하면 "안 보이니 구독 안 함 → 구독이 없으니 발행 안 함" 교착에
> 빠진다. 실제로 그 교착 때문에 원인을 한 번 더 놓쳤다.

### 저장된 USD 는 `strings` 로 검증할 수 없다

저장한 scene 이 8.8 KB 라 `strings` 로 들여다봤더니 `ActionGraph` 도 `stiffness` 도
안 잡혀 "빈 파일"로 판정했다. **틀린 판정이었다.** USDC crate 는 토큰 테이블을
압축하므로 실제로 들어 있는 이름도 검색되지 않는다 (덤프가 `0dam`, `5arm`,
`P.ROS2X` 처럼 깨져 나온다).

**`scripts/isaac/sim_side/verify_saved_scene.py` 로 확인한다** — 저장된 파일을
별도 스테이지로 열어 ActionGraph prim · 타임라인 구간 · drive gain 을 찍는다.
작업 중인 스테이지를 건드리지 않으므로 아무 때나 돌려도 된다.

```
[verify] ActionGraph prims: 10
[verify] timeline: start=0.0 end=10000000.0
[verify] panda_joint4: stiffness=10000.0 damping=400.0
[verify] verdict: SAVED OK
```

> 참고로 `setup_graph.py` / `tune_drive.py` 는 실행 시 edit target 을 root layer 로
> 되돌린다. 이번 사례에서는 원래 root 였지만(오진이었다), session layer 에 쓰면
> 저장에 담기지 않는 것은 사실이므로 방어적으로 남겨 둔다.

### 기동 게이트의 시간 기준은 벽시계여야 한다

`readiness_gate` 가 **60초 한도를 113 ms 만에** 터뜨린 적이 있다. deadline 을
ROS 클럭으로 잡았기 때문이다.

`use_sim_time` 이 켜진 상태에서 첫 `/clock` 을 받기 전까지 `now()` 는 **0** 이다.
그 시점에 `deadline = 0 + 60s` 로 잡히는데, 시뮬레이터가 이미 오래 돌아 sim time 이
60 을 넘겼으면 **첫 `/clock` 이 도착하는 순간 곧바로 타임아웃**한다. `resetOnStop`
을 끈 뒤로는 sim time 이 리셋되지 않으므로 이 조건이 쉽게 성립한다.

반대 방향도 있다 — `/clock` 이 영영 오지 않으면 시간이 0 에 멈춰 **영원히 대기**한다.
실측으로 둘 다 재현했다.

`time.monotonic()` 으로 고쳤다. 기동 게이트가 재려는 것은 시뮬레이션 시간이 아니라
**사람이 기다리는 실제 시간**이다.

> 이 실패는 진단을 거꾸로 이끈다 — "타임아웃"이라 시뮬레이터가 죽은 것처럼 보이지만,
> **113 ms 만에 터졌다는 사실 자체가 `/clock` 이 오고 있었다는 증거**다. 로그의
> 시각 차이를 보는 것이 먼저다.

### Stop/Play 는 시간을 되돌린다 — TF_OLD_DATA

Isaac 을 Stop 하면 sim time 이 0 으로 리셋된다. 스택은 계속 떠 있으므로 TF 버퍼에
이미 **미래** 시각의 변환이 들어 있고, 다시 Play 하면 새 데이터가 과거로 보여 버려진다.

```
[ee_pose_node] Warning: TF_OLD_DATA ignoring data from the past for frame panda_link6
```

경고로만 끝나지 않는다 — TF 조회가 조용히 실패하고 `move_group` 이 현재 상태를 못
읽는다. **`ReadSimTime.inputs:resetOnStop = False`** 로 막는다(`setup_graph.py` 가
설정한다). 그러면 Stop 에서 시간이 **멈출 뿐 되돌아가지 않는다.**

이미 경고가 난 세션은 버퍼가 오염돼 있으므로 **스택을 재시작한다.** 순서는 언제나
**Isaac Play → 스택 기동**이다.

### 관절 한계 — URDF 가 실제 로봇보다 헐겁다 (Q6 결정)

`moveit_resources_panda` 의 URDF 는 **전 관절이 실제 Franka 스펙보다 4° 헐겁고**,
`panda_joint4` 상한은 **9°** 헐겁다. 규칙적인 패딩이라 한 관절만의 문제가 아니다.

| joint | URDF | 실제 스펙 | 차 |
|---|---|---|---|
| joint1·3·5·7 | ±2.9671 | ±2.8973 | 0.0698 (4°) |
| joint2 | ±1.8326 | ±1.7628 | 0.0698 |
| joint6 | -0.0873 ~ 3.8223 | -0.0175 ~ 3.7525 | 0.0698 |
| **joint4** | -3.1416 ~ **+0.0873** | -3.0718 ~ **-0.0698** | 상한 **0.1571 (9°)** |

**Isaac 은 실제 스펙을 강제한다.** 그래서 MoveIt 이 계획한 자세를 시뮬레이터가 한계에서
막고, 그것이 추종 오차처럼 보인다 — `extended`(j4=0)로 보냈더니 팔이 -0.0698 에서
멈췄고 그 값을 중력 처짐으로 오독했다. 강성을 **625배** 올려도 소수점 4자리까지
그대로인 것을 보고서야 갈렸다(처짐은 K 에 반비례하고 한계 클램프는 무관하다).

**결정: 실제 스펙 쪽으로 좁힌다.** 수집한 데이터가 실물로 가야 하므로, 실제 로봇이
갈 수 없는 자세가 데이터셋에 섞이면 안 된다. 계획 단계에서 막는 편이 시뮬레이터가
조용히 클램프하는 것보다 낫다.

`config/panda_real_joint_limits.yaml` 에 position 한계를 넣고 `panda_isaac.launch.py`
가 `build_moveit_config(joint_limits_file=...)` 로 넘긴다. 속도·가속도는 원본 그대로다.

**적용 범위는 Isaac 뿐이다.** mock/Gazebo/펑션베이까지 넓히는 것은 별도 결정으로 남긴다
— 그 백엔드들은 한계를 강제하지 않아 당장 문제가 없고, 공용 헬퍼 기본값을 바꾸면
검증되지 않은 영향이 다른 launch 로 번진다.

> **부작용 하나** — SRDF 의 `extended` 는 j4=0 을 요구하므로 이제 **계획이 실패한다.**
> 실물에 없는 자세이므로 실패가 옳다. `is_check_phase0.py` 의 계획 검사는 `ready` 로
> 바꾸고, SRDF 파싱은 named target **목록 조회**로 따로 확인한다.

### 팔이 탁자를 뚫는다 — **고치지 않기로 했다** (Q8 최종)

`/scene/objects` 를 발행하는 것만으로는 **MoveIt 이 물체를 모른다.** 빈 공간을 가정해
계획하므로 팔이 테이블을 뚫는 궤적이 나온다 — Phase 2 에서 손가락이 상판 안에 박혀
그리퍼가 물리적으로 막혔고, 이동량 0.0018 m 를 "그리퍼 고장"으로 오독했다.

**관측된 실패이지 가설이 아니다.** 그런데도 **감수하기로 했다** (2026-09-01, Q8).

#### 왜 감수하나

`/scene/objects` 에 실리는 것은 **전부 조작 대상**이고, 조작 대상을 planning scene 에
넣으면 **파지 자세가 시작 상태 충돌이 되어** 관절 공간 계획이 `INVALID_MOTION_PLAN`(-2)
으로 거부된다. 그래서 "고정물만 넣자"로 좁혔는데, 이어서 **환경 물체를 `/scene/objects`
에 아예 발행하지 않기로** 정하자 넣을 것이 남지 않았다. `planning_scene_sync` 노드와
`SceneObject.fixture` 필드를 함께 삭제했다.

**노출은 좁다** — 자기 충돌 검사는 ACM 이라 world 물체와 무관하게 계속 동작하고, 파지
동작은 접근·하강·들어올림이 전부 cartesian 이라 `GetCartesianPath` 의 `avoid_collisions`
기본값 `False` 로 planning scene 을 보지 않는다. **관절 공간 이동에서만 드러난다.**

#### 되살린다면

토픽 → planning scene 동기화 노드를 다시 만들지 않는다. **탁자 하나만 launch 시점에
상자로 정적 등록**하는 편이 맞다 — 조작 대상은 여전히 넣지 않으므로 위 -2 문제가 생기지
않고, 실제로 부딪히는 것은 탁자뿐이다.

<details>
<summary>이력 — 삭제된 `planning_scene_sync` (2026-08~09-01)</summary>

`/scene/objects` 를 planning scene 으로 옮기는 노드였다. mock 백엔드와 방향이 반대라
(mock 은 planning scene → `/scene/objects`) **mock 에서 띄우면 자기가 읽은 것을 자기가
되쓰는 고리**가 됐다. 시뮬레이터가 scene 의 원본인 백엔드에서만 썼다.

실측으로는 의도대로 동작했다 — planning scene 에 물체 4개가 올라가고 문제의 자세가
계획 단계에서 막혔다(`GOAL_IN_COLLISION`, -27). 전에는 계획이 성공하고 시뮬레이터에서
조용히 막혔으니 **실패가 눈에 보이는 편이 나았다.** 남은 과제로 "집을 때 손가락과 물체의
접촉이 충돌로 잡힌다 — `AttachedCollisionObject` 로 옮기거나 ACM 을 손봐야 한다"를
적어 두었는데, 그 `attach`/`detach` 자체가 Q9 에서 철회되면서 이 노드도 함께 정리됐다.

</details>

### Isaac 은 관절 한계를 **강제한다**

펑션베이는 한계를 넘겨도 그냥 갔고(그 문서 §2.2) 그것이 복구 불능 자세(§4)의
원인이었다. Isaac 은 막는다 — 안전하지만, **MoveIt 이 계획한 자세를 시뮬레이터가
거부할 수 있다**는 뜻이기도 하다 (Q6).

막힌 것인지 처진 것인지는 **강성을 바꿔 보면 갈린다.** 처짐은 `오차 = 토크/K` 라
K 에 반비례해 줄고, 한계 클램프는 **K 를 625배 해도 소수점 4자리까지 그대로**다.
실제로 그 방법으로 갈랐다.

### 한국어 Windows 의 함정 — 오류가 오류를 가린다

ROS 2 라이브러리 로딩이 실패하면 Isaac 은 그 하위 프로세스 출력을
`grepexc.output.decode("utf-8")` 로 읽는다. 한국어 Windows 는 오류 문구를 **cp949**
로 내보내므로 여기서 `UnicodeDecodeError: 'utf-8' codec can't decode byte 0xc1` 가
터지고, **진짜 원인 대신 이 예외가 보인다.**

이 예외를 보면 인코딩 문제로 오해하지 말고 그 아래
`Failed to load system ROS 2 libraries for ROS_DISTRO='...'` 줄을 찾는다. 그쪽이
원인이다.

### 로봇이 **월드 원점에 없으면** 모든 좌표가 조용히 어긋난다

ROS 쪽 static TF 가 `world -> panda_link0` 를 **항등변환으로 못박고** 있다
(`launch_helpers/common.py`). 에셋을 스테이지에 드래그해 올리면 원점이 아닌 자리에
떨어지는데, 그러면 양쪽이 어긋난 채로 **각자 일관되게** 동작한다.

* 로봇 내부는 맞는다 — `/joint_states` → `robot_state_publisher` → TF 가 전부
  `panda_link0` 기준이라, 로봇이 어디 놓여 있든 자기들끼리는 정확하다.
* scene 물체는 `setup_scene.py` 가 **월드 좌표로** 만든다.

그래서 그리퍼는 *"목표에 0.5 mm 오차로 도달"* 이라고 보고하면서 실제로는 물체에서
수십 cm 떨어진 허공을 쥔다. **에러도 경고도 없다.**

실측 — `panda_link0` 이 월드 `(-0.035, -0.349, 0)` 에 있었다. 파지 목표는
`panda_link0` 기준 `(0.500, -0.150, 0.528)` 이었고 `panda_hand` 의 월드 위치는
`(0.465, -0.499, 0.528)` 로 정확히 그만큼 밀려 있었다. 블록은 월드 `(0.500, -0.150)`
그대로였으니 손은 **y 축으로 35 cm 떨어진** 자리를 쥔 것이다.

**증상이 마찰·콜라이더를 가리킨다.** 손가락은 0 까지 닫히고, 블록은 소수점 넷째
자리까지 움직이지 않는다 — "물체를 통과했다"로 읽히고, 실제로는 통과한 게 아니라
애초에 거기 없었다. 마찰 재질과 콜라이더 설정을 한참 뒤진 뒤에야 드러났다.

판별 — `check_colliders.py` 가 각 prim 의 **월드 좌표**를 찍는다. `panda_link0` 이
원점이 아니면 이것이다. 고치는 것은 `place_robot.py` (Stop 상태에서, 멱등).

> `check_colliders.py` 의 `collision=NONE` 과 `gravity=-inf` 는 **오탐**이다.
> 전자는 Franka 에셋이 instanceable 이라 `GetChildren()` 이 프로토타입 내부를
> 못 보는 것이고, 후자는 USD 의 '기본 중력' 센티널이다. 둘 다 정상이다.

### 시작 자세 — 에셋 기본값은 **탁자 안**이다

Franka 에셋의 기본 관절값은 손목이 말려 있다(`panda_joint6 = 3.1475`, `ready` 는
1.571). 로봇을 원점에 놓고 나면 그 자세에서 손이 탁자 상판 안으로 들어가므로,
Play 를 누르는 순간 **팔이 탁자 위로 뚝 떨어지는 것처럼** 보인다.

**증상이 두 갈래로 나타난다.** 하나는 눈에 보이는 낙하이고, 다른 하나는
`INVALID_MOTION_PLAN`(-2)이다 — 시작 자세가 충돌이면 MoveIt 은 계획 자체를 거부한다.
후자는 원인이 전혀 안 보인다.

**로봇이 원점에서 벗어나 있으면 이 증상이 숨는다.** 탁자를 비껴가 허공에 떨어지기
때문이다. 배치를 고친 뒤에야 드러나므로, 두 함정은 순서대로 만나게 된다.

`탁자가 너무 높은 것이 아니다` — 실측으로 `ready` 는 상판(z=0.400)에서 손끝이
z≈0.487 로 **8.7 cm 여유**가 있고 `/check_state_validity` 가 `valid: True` 를 준다.
높이를 낮추면 증상은 가려지지만 원인은 그대로 남는다.

고치는 것은 `set_home_pose.py` 다. **두 속성을 함께 쓴다** — `state:angular:physics:position`
(PhysX 의 시작 상태)과 `drive:angular:physics:targetPosition`(드라이브 목표). 앞의
것만 쓰면 시작하자마자 드라이브가 옛 목표로 끌고 가고, 뒤의 것만 쓰면 시작 자세에서
목표까지 한 번 크게 휘두른다.

> **USD 의 각도는 도(degree)다.** 설정에는 라디안으로 적고 스크립트가 바꾼다. 안
> 바꾸면 1.571 rad 가 1.6도가 되어, 크래시 없이 '그럴듯하게 틀린' 자세가 된다.

### 단일 출처 — `src` 와 `install/share` 는 **다른 파일**이다

`isaac_scene.json` 을 두 쪽이 읽는데 **경로가 다르다.**

| 읽는 쪽 | 경로 |
|---|---|
| Isaac 스크립트 (`setup_scene.py`, `tune_grasp.py`, `set_home_pose.py`) | `src/robot_control/config/` |
| ROS `isaac_scene_state_node` | `install/robot_control/share/robot_control/config/` |

Isaac 쪽은 ament 를 못 쓰므로 저장소 경로를 직접 읽을 수밖에 없다. 그래서 **"단일
출처"는 `colcon build` 를 해야만 성립한다.** 물체를 추가하고 `setup_scene.py` 만
다시 돌리면 Isaac 에는 생기고 ROS 는 모르는 상태가 조용히 만들어진다 — `/scene/objects`
에 그 물체만 없고, 에러는 없다.

판별 — 아래가 서로 다르면 빌드를 빠뜨린 것이다.

```bash
diff <(python3 -c "import json;print(json.dumps(json.load(open('src/robot_control/config/isaac_scene.json')),indent=1))") \
     <(python3 -c "import json;print(json.dumps(json.load(open('install/robot_control/share/robot_control/config/isaac_scene.json')),indent=1))")
```

### 외부에서 spin 되는 노드에는 **`externally_spun` 을 넘겨야 한다**

`MoveGroupClient` 의 여러 대기 지점은 기본적으로 **노드를 직접 spin** 한다
(`await_future_spin`). 스크립트에서는 맞지만, executor 가 이미 다른 스레드에서 그
노드를 돌리는 곳(로봇 트윈)에서는 콜백이 executor 쪽으로 가므로 **여기서는 영영
오지 않는다.**

**예외도 타임아웃도 없다.** 오퍼레이션이 `RUNNING` 인 채로 남고 팔은 움직이지 않으며,
로그에는 아무 단서도 없다.

실측 — `plan_joints_async` 가 이 인자를 **받지도 넘기지도 않아** 트윈의
`move_to_joints` 가 그렇게 멈췄다. 인자를 추가하고
`move_to_joints_streamed_async` 에서 전달하게 고쳤다.

`plan_named_target_async` 는 관절값을 콜백 체인으로 얻으므로 애초에 spin 하지 않는다.
그래서 **`move_to_named_target` 만 먼저 동작해** 원인이 더 헷갈렸다 — "트윈은 되는데
관절 지정만 안 된다"로 보인다.

회귀 검사는 `moveit/tests/test_externally_spun_propagation.py` 에 있다.

### servo 개루프 주행은 **드리프트가 쌓여 결국 멈춘다**

twist 를 반복해 보내면 팔이 서서히 어긋나고, 충분히 쌓이면 servo 가 움직이기를
거부한다.

```
/servo_node/status
  HALT_FOR_COLLISION        80건
  DECELERATE_FOR_COLLISION  67건
  NO_WARNING                 0건
```

**원인이 현재 명령이 아니라 이전 이력에 있다.** 방금 보낸 twist 는 정상인데 팔이
안 움직이므로, 그 명령이나 그때 켜 둔 기능을 의심하게 된다.

**실제로 그렇게 오독했다.** 처음에는 `enable_scene:=true` 를 원인으로 지목했다 —
scene 을 끄고 다시 띄우니 움직였기 때문이다. 그러나 그 재기동 과정에 `is_recover.py` 가
끼어 있어 **자세가 함께 초기화된** 것이 진짜 차이였다.

가른 방법은 **매 시행 전에 `ready` 로 되돌리고** planning scene 내용만 바꾼 것이다.

| 조건 | 이동(명령 0.2 m/s x 2s) | status |
|---|---|---|
| diff 없음 | +0.1843 m | `NO_WARNING` x103 |
| 빈 diff (물체 0) | +0.1955 m | `NO_WARNING` x103 |
| table 만 | +0.1974 m | `NO_WARNING` x103 |
| table + 블록 3 | +0.1829 m | `NO_WARNING` x95 |

**scene 물체는 무관하다.** 넷이 같다. 결정적인 것은 대조군에서도 오염된 자세로 시작하면
planning scene 이 **완전히 비어 있는데도** `DECELERATE_FOR_COLLISION` 이 나온다는
점이다 — scene 이 원인이면 나올 수 없다. 남는 것은 자기충돌뿐이다.

**대응** — 텔레오퍼레이션 세션 사이에는 `scripts/isaac/is_recover.py` 로 자세를
되돌린다. 에피소드 경계마다 되돌리는 것이 데이터 품질에도 맞다. 추종 배율이
명령 대비 0.46~0.49 인 것도 같은 개루프 성질에서 온다.

### servo 는 **NaN 을 내보내면서 `NO_WARNING` 을 유지한다**

> ⚠️ **아래는 bridge 시절(2026-08-30)의 실측이며, plugin 에서 재검증하지 않았다.**
> servo 출력이 다리 노드를 거치지 않고 **JTC 로 직행**하도록 바뀌었기 때문이다(D1″).
> **닫혔다 (2026-09-05).** 사슬에 NaN 필터가 하나도 없다 — JTC 는 로그 없이 받아들이고
> `TopicBasedSystem` 은 오히려 반드시 발행해서 **NaN 이 `/isaac/arm_command` 까지 간다.**
> **그런데 Isaac 이 흡수한다** — 관절별로 무시하고 마지막 목표를 유지하며 다음 정상
> 궤적에 스스로 복구한다. 그래서 방어 노드를 되살리지 않기로 했다 (§8 Q20).
> 남는 위험은 **부분 NaN 의 '그럴듯하게 틀린 자세'** 하나다.

servo 출력 일곱 값이 전부 NaN 이 되는 상태가 있다. 그런데,

```
/isaac/servo_command   29.4 Hz   전부 [nan, nan, nan, nan, nan, nan, nan]
/servo_node/status     전부 NO_WARNING
관절 변화               0.0000 (7축 모두)
```

**모든 지표가 정상으로 보인다.** 발행 주기는 `publish_period` 와 정확히 맞고, 경고는
없고, 노드도 살아 있다. 팔만 안 움직인다. 값을 직접 찍기 전까지 드러나지 않는다.

#### 전제조건 — 재현했다

무작위가 아니다. **두 가지가 겹쳐야** 한다.

1. servo 가 **충돌 정지 상태**다 (`HALT_FOR_COLLISION` / `DECELERATE_FOR_COLLISION`)
2. 그 상태에서 **외부가 로봇을 점프시킨다** — 당시에는 `/isaac/arm_command` 에 직접
   쓰는 것이었다. **지금 `is_recover.py` 는 `ArmCommander` 로 JTC 에 `JointTrajectory` 를
   쓴다** — 같은 조건이 성립하는지는 미확인이다

```
한 방향 주행 -> status {3: 72, 4: 5}   (HALT 진입)
그 상태에서 직접 스트리밍
  -> 이후 twist  NaN 68/68,  status {0: 68}
```

**건강한 상태에서는 같은 직접 스트리밍을 해도 NaN 이 안 난다** — 그래서 처음에는
재현이 안 돼 원인을 못 잡았다. 전제조건을 갖추자 1회에 재현됐다.

`is_recover.py` 가 바로 그 "직접 쓰기"를 하므로, **드리프트 대응책이 NaN 을
유발하는 구조**였다. 지금은 스트리밍 전후로 servo 를 멈췄다 켠다.

#### `pause`/`unpause` 를 쓰면 안 된다

처음에는 `pause_servo` / `unpause_servo` 로 고쳤는데 **더 나빴다.**

```
현재        출력 0건
unpause -> (True, '')  ->  출력  0건     성공을 반환하면서 복구하지 않는다
start   -> (True, '')  ->  출력 68건
```

`unpause_servo` 는 **성공을 반환하면서도 발행을 되살리지 못한다.** 그 쌍을 쓰면 복구할
때마다 servo 가 조용히 죽고, 증상은 다시 "팔이 안 움직인다"가 된다. `stop_servo` /
`start_servo` 를 쓴다.

수정 후 같은 전제조건에서 확인 — 복구가 정상 완료되고(관절 오차 0.0003 rad) 이후
twist 3회 모두 **NaN 0건**이다.

#### ~~다리가 막는다~~ — **Isaac 에는 그 방어층이 없다**

당시에는 `isaac_servo_bridge` 가 유한하지 않은 값을 **버리고 경고**했다. 원인을 못
막더라도 NaN 이 시뮬레이터까지 가지는 않게 하려는 것이다 — 가면 조용히 무시되어 증상이
servo 바깥을 가리킨다.

**D1″ 이후 Isaac 경로에는 그 노드가 없다.** servo 출력이 JTC 로 직행하므로, NaN 을
막는 것이 있다면 **JTC 자신**이어야 한다. 확인하지 않았다.

> 방어 코드 자체는 살아 있다 —
> [`servo_command_bridge_node.py`](../../../src/robot_control/robot_control/isaac/servo_command_bridge_node.py)
> 의 유한성 검사(L122~)는 그대로이고, **펑션베이가 그 노드를 쓴다.** 즉 방어층이 사라진
> 것은 Isaac 경로뿐이다.

### 마찰은 **조용히** 기본값이 된다

USD 에서 마찰은 prim 의 속성이 아니라 **별도 Material prim** 이고, 콜라이더에
`physics` purpose 로 바인딩해야 적용된다. 안 붙여도 경고가 없다 — PhysX 기본값
(정적·동적 모두 0.5)이 쓰이고, 그 값으로는 매끈한 상자를 옆에서 눌러 들지 못한다.

**증상이 '파지 실패'가 아니라 '집었는데 안 들린다'로 나온다.** 손가락은 정상적으로
닫히고, 액션은 성공으로 돌아오고, MoveIt 의 attach 도 걸린다. 팔만 올라가고 블록은
테이블에 남는다.

메시가 자식 prim 에 있는 에셋이면 링크에만 바인딩해도 새어나간다. `tune_grasp.py` 는
링크와 **그 아래 `CollisionAPI` 가 붙은 prim 전부**에 건다.

판별 — `is_check_phase6.py` 의 4번(들어올림)만 실패하고 3번(물림)은 통과한다.
3번까지 실패하면서 폭이 0 에 가까우면 마찰이 아니라 **접근 높이**가 어긋난 것이다
(`TCP_OFFSET` 0.1034 m, 또는 손이 아래를 보지 않는 경우 — 검사가 기울기를 함께 찍는다).

### 파지 한 번 뒤 그리퍼가 명령을 영영 무시한다 — **로거 severity** (해결됨)

`gripper_action_node` 가 **죽는다.** 그런데 launch 전체가 내려가지 않으므로 로그를 보지
않으면 "그리퍼가 이상하다"로만 보인다. 명령은 발행되고 아무 일도 일어나지 않는다.

```
File ".../gripper_action_node.py", line 235, in _on_result
    level(f'[gripper] {goal}: action finished (status={status})')
ValueError: Logger severity cannot be changed between calls.
```

**rclpy 로거는 호출 지점(파일·행)마다 severity 를 기억한다.** 레벨을 변수에 담아 한 줄에서
부르면, 두 번째 호출이 다른 레벨일 때 예외가 난다. 그것이 `done_callback` 안에서 터지므로
노드가 통째로 죽는다.

발현 조건이 **plugin 에서 처음 갖춰졌다** — `GripperActionController` 는 선점을
`CANCELED`(status 5), 정상 종료를 `SUCCEEDED`(status 4)로 돌려주므로 같은 줄이 WARN 과
INFO 로 번갈아 불린다. `isaac_gripper_bridge` 는 늘 SUCCEEDED 였다. **백엔드에 특유한
버그가 아니다** — mock 에서도 비성공 결과가 한 번 섞이면 같은 일이 난다.

고침: 레벨을 변수에 담지 말고 `if`/`else` 로 갈라 각 줄에서 부른다.

### 그리퍼 액션 결과를 기다리면 타임아웃한다 — **파지는 완료되지 않는다**

plugin 에서 `panda_hand_controller`(`GripperActionController`)에 `grasp` 를 보내면
**액션이 끝나지 않는다.** 다음 명령이 선점할 때까지 살아 있다가 `CANCELED` 로 끝난다.

원인은 이 컨트롤러가 stall 을 **속도**로 판정하기 때문이다 (`stall_velocity_threshold`,
기본 0.001). Isaac 은 PhysX 접촉에서 손가락이 계속 떨려 그 조건이 서지 않는다.
`allow_stalling: true` 로 켜도 마찬가지다 — 속도 기반으로는 원리적으로 안 된다.

**그런데 이것은 고칠 필요가 없다.** 소비자는 액션 결과를 보지 않는다 — `GripperNode` 가
effort 로 `at_goal` 을 판정해 `/gripper_states` 로 내고, 트윈·텔레오퍼레이션은 그것만
본다. 실측으로 파지 22.4 N·m 에서 `stalled: true, at_goal: true` 가 정확히 선다.
미완료 목표가 쌓이지도 않는다.

**결과를 기다리는 코드만 고치면 된다.** `is_check_phase6.py` 가 그렇게 만들어져 있어
15 초 타임아웃을 "그리퍼 목표가 거부됐다"로 보고했는데, 컨트롤러 로그에는
`Received & accepted` 가 찍혀 있었다 — **거부가 아니라 미완료다.**

### 시뮬레이터 재시작 — **타임라인 Stop/Play 와 프로세스 재시작은 다르다** (plugin)

둘을 같은 것으로 묶으면 안 된다. 실측(2026-09-05)으로 갈렸다.

**① 타임라인 Stop → Play (`/set_simulation_state`) — 스택을 건드릴 필요가 없다.**

`setup_graph.py` 가 `resetOnStop` 을 꺼 두었으므로 **sim 시계가 되감기지 않는다.**
정지 중에는 `/clock` 과 `/joint_states` 가 멎지만(컨트롤러는 `active` 로 남는다),
Play 하면 **스스로 복구된다** — 실측 `TF_OLD_DATA` 0 건, 시계는 414.8 에서 이어졌다.

**② 프로세스 재시작 — 스택도 다시 띄워야 한다.**

sim 시계가 0 으로 돌아가고 TF 버퍼에 미래 타임스탬프가 남아 `TF_OLD_DATA` 가 쏟아진다
(실측 6,424 건). **컨트롤러는 계속 `active` 라고 보고하므로 상태를 건강 신호로 쓰면
안 된다** — bridge 에서는 `/ee_pose` 가 어는 것이 유일한 증상이었는데 plugin 은
"정상"이라고 답하는 창구가 하나 더 생긴 셈이다.

> `/joint_states` 가 **완전히 멎는지**는 깨끗하게 가르지 못했다. 한 번은 0 건이었고
> 다른 한 번은 흐르고 있었는데, 두 번째 관측이 스택 재기동과 겹쳐 오염됐다. **믿을 수
> 있는 지표는 `TF_OLD_DATA` 누적과 `/ee_pose` 의 stamp 정지다.**

**조치와 그 효과는 확인됐다** — `./scripts/kill_stack.sh --ros` 후 스택만 다시 띄우면
게이트가 통과하고 `TF_OLD_DATA` 0 건으로 세 채널(`/clock`·`/joint_states`·`/ee_pose`)의
stamp 가 일관되게 흐른다.

---

## 8. 열려 있는 결정

**열린 것을 먼저, 각 묶음 안에서는 번호순.** 닫힌 항목은 지우지 않는다 — 판단이 왜
뒤집혔는지가 그 자체로 근거다.

| # | 항목 | 언제 정하나 |
|:-:|---|---|
| Q5 | `PublishJointState.targetPrim` 이 deprecated — `IsaacReadJointState` 노드 출력을 연결하는 방식으로 교체할지 | Phase 0 통과 후 (동작에는 지장 없음) |
| Q17 | **수집 launch 의 mp4 레코더가 `/session` 을 따라 돌지 않는다.** 옛 묶음 launch 는 `image_recorder_node`(서비스 구동, `auto_start:=false`)를 띄웠으므로 `/image_recorder/start_session` 을 따로 불러야 한다. `/session` 을 보고 자동 분절하는 것은 `rdfp_image_recorder` 로 다른 노드다. **데이터셋에는 지장이 없다**(mp4 는 `import` 가 rosbag 에서 만든다) — 다만 "세션만 시작하면 mp4 가 나온다"는 기대와 어긋난다. 둘 중 무엇을 기본으로 둘지 | 수집 절차를 문서로 굳힐 때 |
| Q18 | **에피소드 라벨을 놓치기 쉽다.** `set_task_label` 의 필드는 `task_label` 인데 이름을 틀리면 서비스가 실패하고, **그대로 진행하면 빈 라벨로 기록된다.** 기록이 끝난 뒤에는 재import 로도 못 고친다. `start_episode` 가 라벨 없이 시작할 때 경고하거나, 라벨을 인자로 받게 하는 편이 나을지 | 수집을 반복하기 전에 |
| Q19 | **배속 0.872 가 부하 탓인지 기동 과도인지 안 갈렸다.** load average 와 **단조롭지 않다** — load 8.4 에서 0.937 이 나왔는데 load 7.7 에서 0.872 가 나왔다. 후보 둘: ① load average 는 1분 평균이라 순간 경합을 못 잡는다, ② 0.872 는 **스택 기동 직후 첫 검사** 값이라 MoveIt 이 계획 파이프라인 셋을 올리고 TF 버퍼가 차는 과도 구간일 수 있다 — 같은 실행에서 `phase0` 의 「계획」이 타임아웃했다가 재실행에서 통과한 것이 그 정황이다. 가르는 법: 배속을 **프로세스별 CPU 사용률과 함께** 재거나, plugin 스택이 안정된 뒤 `phase0` 를 반복해 첫 실행만 낮은지 본다 (bridge 에서는 재실행 시 0.996 이 나왔으나 조건이 달라 결정적이지 않다) | 배속이 실제로 문제가 될 때 |

**닫힌 결정(Q1~Q4 · Q6~Q16 · Q20 · Q21)** 은 판단이 왜 뒤집혔는지가 근거라 지우지 않고 [../../archive/isaac/isaac_phase_log.md](../../archive/isaac/isaac_phase_log.md) 「닫힌 결정」 표로 옮겼다.
